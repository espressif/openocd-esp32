#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "gen_ut_app.h"
#include "esp_log.h"
#if UT_IDF_VER < MAKE_UT_IDF_VER(5,0,0,0)
#include "esp_spi_flash.h"
#define SET_BP(id, addr)                cpu_hal_set_breakpoint(id, addr)
#define SET_WP(id, addr, size, trigger) cpu_hal_set_watchpoint(id, addr, size, trigger)
#else
#define SET_BP(id, addr)                esp_cpu_set_breakpoint(id, addr)
#define SET_WP(id, addr, size, trigger) esp_cpu_set_watchpoint(id, addr, size, trigger)
#include "spi_flash_mmap.h"
#include "esp_private/cache_utils.h"
#include "hal/cpu_hal.h"
#endif

const static char *TAG = "special_test";

static void crash_task(void *pvParameter)
{
    ESP_LOGI(TAG, "Start crash task on core %d", xPortGetCoreID());
    int *p = 0;
    *p = 0; TEST_BREAK_LOC(crash);
}

static void cache_check_task(void *pvParameter)
{
    int count = 0;
    gpio_reset_pin(BLINK_GPIO);

    bool cache_before = spi_flash_cache_enabled();
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);   TEST_BREAK_LOC(gpio_set_direction);
    bool cache_after = spi_flash_cache_enabled();
    ESP_LOGI(TAG, "Cache 0x%x -> 0x%x", cache_before, cache_after);
    assert(cache_before == cache_after);

    while(1) {
        ESP_LOGI(TAG, "Toggle LED %d", count);

        cache_before = spi_flash_cache_enabled();
        gpio_set_level(BLINK_GPIO, count++ % 2);        TEST_BREAK_LOC(gpio_set_level);
        cache_after = spi_flash_cache_enabled();
        ESP_LOGI(TAG, "Cache 0x%x -> 0x%x", cache_before, cache_after);
        assert(cache_before == cache_after);

        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

#if CONFIG_IDF_TARGET_ARCH_XTENSA
#define SPIRAM_TEST_ARRAY_SZ    100

static void psram_check_task(void *pvParameter)
{
#if CONFIG_IDF_TARGET_ESP32S3 && UT_IDF_VER < MAKE_UT_IDF_VER(5,0,0,0)
    /* In ESP32S3, PSRAM is mapped from high to low. Check s_mapped_vaddr_start at esp32s3/spiram.c
        There is a plan to change from low to high same as ESP32
        Follow up jira https://jira.espressif.com:8443/browse/IDF-4318
    */
    uint32_t *mem = (uint32_t *)SOC_EXTRAM_DATA_HIGH - (SPIRAM_TEST_ARRAY_SZ * (xPortGetCoreID() + 1));
#else
    uint32_t *mem = (uint32_t *)SOC_EXTRAM_DATA_LOW;
#endif
    mem += xPortGetCoreID() * SPIRAM_TEST_ARRAY_SZ;
    for (int i = 0, k = 0x20; i < SPIRAM_TEST_ARRAY_SZ; i++, k++) {
        mem[i] = k;
    }

    /* Configure the IOMUX register for pad BLINK_GPIO (some pads are
       muxed to GPIO on reset already, but some default to other
       functions and need to be switched to GPIO. Consult the
       Technical Reference for a list of pads and their default
       functions.)
    */
    gpio_reset_pin(BLINK_GPIO);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);   TEST_BREAK_LOC(gpio_set_direction);
    while(1) {
        ESP_LOGI(TAG, "Toggle LED, curr %d", gpio_get_level(BLINK_GPIO));
        /* Blink off (output low) */
        gpio_set_level(BLINK_GPIO, 0);                  TEST_BREAK_LOC(gpio_set_level0);
        vTaskDelay(100 / portTICK_PERIOD_MS);           TEST_BREAK_LOC(vTaskDelay0);
        /* check that SPIRAM data access is not broken by flasher stub */
        for (int i = 0, k = 0x20; i < SPIRAM_TEST_ARRAY_SZ; i++, k++) {
            assert(mem[i] == k);
        }
        /* Blink on (output high) */
        gpio_set_level(BLINK_GPIO, 1);                  TEST_BREAK_LOC(gpio_set_level1);
        vTaskDelay(100 / portTICK_PERIOD_MS);           TEST_BREAK_LOC(vTaskDelay1);
    }
}

static void illegal_instruction_exc(void *pvParameter)
{
    int core_id = xPortGetCoreID();
    ESP_LOGI(TAG, "CPU[%d]: Illegal instruction exception test started", core_id);
    __asm__ __volatile__ (
        ".global exception_bp\n" \
        ".type   exception_bp,@function\n" \
        "exception_bp_1:\n" \
        "ILL\n" \
    );
}

static void load_prohibited_exc(void *pvParameter)
{
    int core_id = xPortGetCoreID();
    ESP_LOGI(TAG, "CPU[%d]: Load prohibited exception test started", core_id);
    register long a2 asm ("a2") = 0;
    register long a3 asm ("a3") = 0;
    __asm__ __volatile__ (
        ".global exception_bp_2\n" \
        ".type   exception_bp_2,@function\n" \
        "exception_bp_2:\n" \
        "L8UI a3, a2, 0xFFFFFFFF\n" \
        : "+r"(a2) : "r"(a3)
    );
}

static void store_prohibited_exc(void *pvParameter)
{
    int core_id = xPortGetCoreID();
    ESP_LOGI(TAG, "CPU[%d]: Store prohibited exception test started", core_id);
    register long a2 asm ("a2") = 0;
    register long a3 asm ("a3") = 0;
    __asm__ __volatile__ (
        ".global exception_bp_3\n" \
        ".type   exception_bp_3,@function\n" \
        "exception_bp_3:\n" \
        "S8I a3, a2, 0\n" \
        : "+r"(a2) : "r"(a3)
    );
}

static void divide_by_zero_exc(void *pvParameter)
{
    int core_id = xPortGetCoreID();
    ESP_LOGI(TAG, "CPU[%d]: Divide by zero exception test started", core_id);
    register long a2 asm ("a2") = 0;
    register long a3 asm ("a3") = 0;
    __asm__ __volatile__ (
        ".global exception_bp_4\n" \
        ".type   exception_bp_4,@function\n" \
        "exception_bp_4:\n" \
        "QUOS a2, a2, a3\n" \
        : "+r"(a2) : "r"(a3)
    );
}
#endif

volatile static int s_var1;
volatile static int s_var2;

static void target_bp_func2()
{
    TEST_BREAK_LOC_EX(target_bp_func2, -1); /* -1 to keep previous line number (function entry)*/
    s_var2 = 0x56789; TEST_BREAK_LOC(target_wp_var2_1);
    ESP_LOGI(TAG, "Target BP func '%s' on core %d", __func__,  xPortGetCoreID());
    volatile int tmp = s_var2; (void)tmp; TEST_BREAK_LOC(target_wp_var2_2);
}

static void target_bp_func1()
{
    TEST_BREAK_LOC_EX(target_bp_func1, -1); /* -1 to keep previous line number (function entry)*/
    s_var1 = 0x12345; TEST_BREAK_LOC(target_wp_var1_1);
    ESP_LOGI(TAG, "Target BP func '%s' on core %d.", __func__,  xPortGetCoreID());
    volatile int tmp = s_var1; (void)tmp; TEST_BREAK_LOC(target_wp_var1_2);
    /* we've just resumed from WP on previous line, deugger could modify breakpoints config, so set next BP here */
    SET_BP(1, target_bp_func2);
    target_bp_func2();
}

static void target_bp_task(void *pvParameter)
{
    ESP_LOGI(TAG, "Start target BP task on core %d", xPortGetCoreID());

    SET_BP(0, target_bp_func1);
    SET_WP(0, (void *)&s_var1, sizeof(s_var1), WATCHPOINT_TRIGGER_ON_RW);
    SET_WP(1, (void *)&s_var2, sizeof(s_var2), WATCHPOINT_TRIGGER_ON_RW);

    target_bp_func1();
}

#if CONFIG_IDF_TARGET_ARCH_RISCV
static void target_wp_reconf_task(void *pvParameter)
{
    ESP_LOGI(TAG, "Start target WP reconfigure task on core %d", xPortGetCoreID());

    /* we need to set BP with the same ID the number of times that exceeds HW BP slots number for chip.
       But only one HW slot should be used by OpenOCD.
       So finally the behaviour should be equal to 'target_bp_task' above. */
    for (int i = 0; i < SOC_CPU_BREAKPOINTS_NUM+1; i++) {
        SET_BP(0, target_bp_func1);
    }

    /* we need to set WP with the same ID the number of times that exceeds HW WP slots number for chip.
       But only one HW slot should be used by OpenOCD.
       So finally the behaviour should be equal to 'target_bp_task' above. */
    for (int i = 0; i < SOC_CPU_WATCHPOINTS_NUM+1; i++) {
        SET_WP(0, (void *)&s_var1, sizeof(s_var1), WATCHPOINT_TRIGGER_ON_RW);
    }
    SET_WP(1, (void *)&s_var2, sizeof(s_var2), WATCHPOINT_TRIGGER_ON_RW);

    target_bp_func1();
}
#endif

ut_result_t special_test_do(int test_num)
{
    switch (test_num) {
        case 800:
        {
            xTaskCreatePinnedToCore(&crash_task, "crash_task", 2048, NULL, 5, NULL, portNUM_PROCESSORS-1);
            break;
        }
        case 801:
        {
            xTaskCreatePinnedToCore(&cache_check_task, "cache_check_task", 4096, NULL, 5, NULL, portNUM_PROCESSORS-1);
            break;
        }
#if CONFIG_IDF_TARGET_ARCH_XTENSA
        case 802:
        {
            xTaskCreatePinnedToCore(&psram_check_task, "psram_task", 4096, NULL, 5, NULL, portNUM_PROCESSORS-1);
            break;
        }
        case 804:
        {
            xTaskCreatePinnedToCore(&illegal_instruction_exc, "illegal_instruction_exc", 4096, NULL, 5, NULL, portNUM_PROCESSORS-1);
            break;
        }
        case 805:
        {
            xTaskCreatePinnedToCore(&load_prohibited_exc, "load_prohibited_exc", 4096, NULL, 5, NULL, portNUM_PROCESSORS-1);
            break;
        }
        case 806:
        {
            xTaskCreatePinnedToCore(&store_prohibited_exc, "store_prohibited_exc", 4096, NULL, 5, NULL, portNUM_PROCESSORS-1);
            break;
        }
        case 807:
        {
            xTaskCreatePinnedToCore(&divide_by_zero_exc, "divide_by_zero_exc", 4096, NULL, 5, NULL, portNUM_PROCESSORS-1);
            break;
        }
#endif
        case 803:
        {
            xTaskCreatePinnedToCore(&target_bp_task, "target_bp_task", 4096, NULL, 5, NULL, portNUM_PROCESSORS-1);
            break;
        }
#if CONFIG_IDF_TARGET_ARCH_RISCV
        case 804:
        {
            xTaskCreatePinnedToCore(&target_wp_reconf_task, "target_wp_reconf_task", 2048, NULL, 5, NULL, portNUM_PROCESSORS-1);
            break;
        }
#endif
        default:
            return UT_UNSUPPORTED;
    }
    return UT_OK;
}
