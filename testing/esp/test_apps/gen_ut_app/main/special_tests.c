#include "driver/gpio.h"
#include "esp_spi_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "gen_ut_app.h"

#include "esp_log.h"
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
#if CONFIG_IDF_TARGET_ESP32S3
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
#endif

#if UT_IDF_VER >= MAKE_UT_IDF_VER(4,3,0,0)
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
    cpu_hal_set_breakpoint(1, target_bp_func2);
    target_bp_func2();
}

static void target_bp_task(void *pvParameter)
{
    ESP_LOGI(TAG, "Start target BP task on core %d", xPortGetCoreID());

    cpu_hal_set_breakpoint(0, target_bp_func1);
    cpu_hal_set_watchpoint(0, (const void *)&s_var1, sizeof(s_var1), WATCHPOINT_TRIGGER_ON_RW);
    cpu_hal_set_watchpoint(1, (const void *)&s_var2, sizeof(s_var2), WATCHPOINT_TRIGGER_ON_RW);

    target_bp_func1();
}
#endif /* UT_IDF_VER >= MAKE_UT_IDF_VER(4,3,0,0) */

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
#endif
#if UT_IDF_VER >= MAKE_UT_IDF_VER(4,3,0,0)
        case 803:
        {
            xTaskCreatePinnedToCore(&target_bp_task, "target_bp_task", 2048, NULL, 5, NULL, portNUM_PROCESSORS-1);
            break;
        }
#endif /* UT_IDF_VER >= MAKE_UT_IDF_VER(4,3,0,0) */
        default:
            return UT_UNSUPPORTED;
    }
    return UT_OK;
}

