#include <string.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "gen_ut_app.h"
#include "esp_log.h"
#include "spi_flash_mmap.h"
#include "esp_private/cache_utils.h"
#include "hal/cpu_hal.h"

#define SET_BP(id, addr)                esp_cpu_set_breakpoint(id, addr)
#define SET_WP(id, addr, size, trigger) esp_cpu_set_watchpoint(id, addr, size, trigger)

const static char *TAG = "special_test";

TEST_DECL(restart_debug_from_crash, "test_special.DebuggerSpecialTests*.test_restart_debug_from_crash")
{
    ESP_LOGI(TAG, "Start crash task on core %d", xPortGetCoreID());
    int *p = 0;
    *p = 0; TEST_BREAK_LOC(crash);
}

TEST_DECL(cache_handling, "test_flasher.FlasherTests*.test_cache_handling")
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

#define SPIRAM_TEST_ARRAY_SZ    100

TEST_DECL(psram_with_flash_breakpoints, "test_special.PsramTests*.test_psram_with_flash_breakpoints")
{
    uint32_t *mem = (uint32_t *)heap_caps_malloc(sizeof(uint32_t)*SPIRAM_TEST_ARRAY_SZ, MALLOC_CAP_DEFAULT|MALLOC_CAP_SPIRAM);
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

static bool mem_check(uint8_t *mem, uint32_t mem_sz)
{
    const uint8_t buf[256] = {0};

    for (uint32_t i = 0; i < mem_sz;) {
        uint32_t cmp_sz = mem_sz > sizeof(buf) ? sizeof(buf) : mem_sz;
        if (memcmp(&mem[i], buf, cmp_sz)) {
            return false;
        }
        i += cmp_sz;
        mem_sz -= cmp_sz;
    }
    return true;
}

// GH issue reported for ESP32-S3. See https://github.com/espressif/openocd-esp32/issues/264
TEST_DECL(gh264_psram_check, "test_special.PsramTests*.test_psram_with_flash_breakpoints_gh264")
{
    uint32_t alloc_sz = 100000;

    while (1) {
        // CONFIG_SPIRAM_MALLOC_ALWAYSINTERNAL == 16384, so any block larger than this
        // will be forced into PSRAM.  The following shows an overwrite.
        // 24540 works; 24541 begins to show overwrite; 1000000 is massive overwrite
        uint8_t *obj = calloc(alloc_sz,sizeof(uint8_t));
        assert(obj);
        printf("Allocated %u bytes @ %p\n", (unsigned int)alloc_sz, obj);

        // Test breakpoints, to be set when you are stopped at app_main.
        // - if you only set "b 15" then "c", when it breaks do an "x/128xb obj" you'll see 0's as you should
        // - but if you set "b 15" AND "b 16" AND "b 17" then "c", when it breaks you'll see bad values
        assert(mem_check(obj, alloc_sz));
        TEST_BREAK_LBL(gh264_psram_check_bp_1);
        printf("breakpoint 1\n"); TEST_BREAK_LOC(gh264_psram_check_1);
        assert(mem_check(obj, alloc_sz));
        TEST_BREAK_LBL(gh264_psram_check_bp_2);
        printf("breakpoint 2\n"); TEST_BREAK_LOC(gh264_psram_check_2);
        assert(mem_check(obj, alloc_sz));
        TEST_BREAK_LBL(gh264_psram_check_bp_3);
        printf("breakpoint 3\n"); TEST_BREAK_LOC(gh264_psram_check_3);

        free(obj);
        alloc_sz /= 2;
    }
}

#if CONFIG_IDF_TARGET_ARCH_XTENSA
TEST_DECL(illegal_instruction_ex, "test_special.DebuggerSpecialTests*.test_exception_illegal_instruction")
{
    __asm__ __volatile__ (
        ".global exception_bp\n" \
        ".type   exception_bp,@function\n" \
        "exception_bp_1:\n" \
        "ILL\n" \
    );
}

TEST_DECL(load_prohibited_ex, "test_special.DebuggerSpecialTests*.test_exception_load_prohibited")
{
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

TEST_DECL(store_prohibited_ex, "test_special.DebuggerSpecialTests*.test_exception_store_prohibited")
{
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

TEST_DECL(divide_by_zero_ex, "test_special.DebuggerSpecialTests*.test_exception_divide_by_zero")
{
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

TEST_DECL(pseudo_debug_ex, "test_special.DebuggerSpecialTests*.test_exception_pseudo_debug")
{
    __asm__ __volatile__ (
        ".global exception_bp_5\n" \
        ".type   exception_bp_5,@function\n" \
        "exception_bp_5:\n" \
        "call0 _DebugExceptionVector\n" \
    );
}

TEST_DECL(pseudo_coprocessor_ex, "test_special.DebuggerSpecialTests*.test_exception_pseudo_coprocessor")
{
    __asm__ __volatile__ (
        ".global exception_bp_6\n" \
        ".type   exception_bp_6,@function\n" \
        "exception_bp_6:\n" \
        "movi    a0,4\n" \
        "wsr     a0,EXCCAUSE\n" \
        "call0   _xt_panic \n" \
    );
}

#else /* CONFIG_IDF_TARGET_ARCH_RISCV */

TEST_DECL(illegal_instruction_ex, "test_special.DebuggerSpecialTests*.test_exception_illegal_instruction")
{
    __asm__ __volatile__ (
        ".global exception_bp\n" \
        ".type   exception_bp,@function\n" \
        "exception_bp_1:\n" \
        "unimp\n" \
    );
}

TEST_DECL(load_access_fault_ex, "test_special.DebuggerSpecialTests*.test_exception_load_access_fault")
{
    int value;

    __asm__ __volatile__ (
		".global exception_bp_2\n" \
        ".type   exception_bp_2,@function\n" \
        "exception_bp_2:\n" \
        "lw %0, 0(%1)\n" \
        : "=r"(value) : "r"(0x1000)
    );
}

TEST_DECL(store_access_fault_ex, "test_special.DebuggerSpecialTests*.test_exception_store_access_fault")
{
    int value = 42;

    asm __volatile__ (
		".global exception_bp_3\n" \
		".type   exception_bp_3,@function\n" \
		"exception_bp_3:\n" \
        "sw %0, 0(%1)"
        :
        : "r"(0x1000),
          "r"(value)
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
    /* we've just resumed from WP on previous line, debugger could modify breakpoints config, so set next BP here */
    SET_BP(1, target_bp_func2);
    target_bp_func2();
}

TEST_DECL(target_bp_wp, "test_special.DebuggerSpecialTest*.test_bp_and_wp_set_by_program")
{
    ESP_LOGI(TAG, "Start target BP and WP task on core %d", xPortGetCoreID());

    SET_BP(0, target_bp_func1);
    SET_WP(0, (void *)&s_var1, sizeof(s_var1), WATCHPOINT_TRIGGER_ON_RW);
    SET_WP(1, (void *)&s_var2, sizeof(s_var2), WATCHPOINT_TRIGGER_ON_RW);

    target_bp_func1();
}

TEST_DECL(wp_reconfigure_by_program, "test_special.DebuggerSpecialTests*.test_wp_reconfigure_by_program")
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

TEST_DECL(assert_failure_ex, "test_special.DebuggerSpecialTests*.test_exception_assert_failure")
{
	TEST_BREAK_LBL(assert_failure_bp);
	assert(0);
}

TEST_DECL(abort_ex, "test_special.DebuggerSpecialTests*.test_exception_abort")
{
	TEST_BREAK_LBL(abort_bp);
	abort();
}

ut_result_t special_test_do(int test_num, int core_num)
{
    if (core_num < 0 || core_num >= portNUM_PROCESSORS)
        core_num = portNUM_PROCESSORS-1;
    if (TEST_ID_MATCH(TEST_ID_PATTERN(target_bp_wp), test_num)) {
        xTaskCreatePinnedToCore(TEST_ENTRY(target_bp_wp), "target_bp_wp_task", 4096, NULL, 5, NULL, core_num);
    } else if (TEST_ID_MATCH(TEST_ID_PATTERN(restart_debug_from_crash), test_num)) {
        xTaskCreatePinnedToCore(TEST_ENTRY(restart_debug_from_crash), "crash_task", 2048, NULL, 5, NULL, core_num);
    } else if (TEST_ID_MATCH(TEST_ID_PATTERN(cache_handling), test_num)) {
        xTaskCreatePinnedToCore(TEST_ENTRY(cache_handling), "cache_check_task", 4096, NULL, 5, NULL, core_num);
    } else if (TEST_ID_MATCH(TEST_ID_PATTERN(wp_reconfigure_by_program), test_num)) {
        xTaskCreatePinnedToCore(TEST_ENTRY(wp_reconfigure_by_program), "target_wp_reconf_task", 2048, NULL, 5, NULL, core_num);
    } else if (TEST_ID_MATCH(TEST_ID_PATTERN(illegal_instruction_ex), test_num)) {
        xTaskCreatePinnedToCore(TEST_ENTRY(illegal_instruction_ex), "illegal_instruction_ex", 2048, NULL, 5, NULL, core_num);
	} else if (TEST_ID_MATCH(TEST_ID_PATTERN(assert_failure_ex), test_num)) {
        xTaskCreatePinnedToCore(TEST_ENTRY(assert_failure_ex), "assert_failure_ex", 2048, NULL, 5, NULL, core_num);
    } else if (TEST_ID_MATCH(TEST_ID_PATTERN(abort_ex), test_num)) {
        xTaskCreatePinnedToCore(TEST_ENTRY(abort_ex), "abort_ex", 2048, NULL, 5, NULL, core_num);
#if CONFIG_IDF_TARGET_ARCH_RISCV
    } else if (TEST_ID_MATCH(TEST_ID_PATTERN(load_access_fault_ex), test_num)) {
        xTaskCreatePinnedToCore(TEST_ENTRY(load_access_fault_ex), "load_access_fault_ex", 2048, NULL, 5, NULL, core_num);
    } else if (TEST_ID_MATCH(TEST_ID_PATTERN(store_access_fault_ex), test_num)) {
        xTaskCreatePinnedToCore(TEST_ENTRY(store_access_fault_ex), "store_access_fault_ex", 2048, NULL, 5, NULL, core_num);
#else /* CONFIG_IDF_TARGET_ARCH_XTENSA */
    } else if (TEST_ID_MATCH(TEST_ID_PATTERN(load_prohibited_ex), test_num)) {
        xTaskCreatePinnedToCore(TEST_ENTRY(load_prohibited_ex), "load_prohibited_ex", 2048, NULL, 5, NULL, core_num);
    } else if (TEST_ID_MATCH(TEST_ID_PATTERN(store_prohibited_ex), test_num)) {
        xTaskCreatePinnedToCore(TEST_ENTRY(store_prohibited_ex), "store_prohibited_ex", 2048, NULL, 5, NULL, core_num);
    } else if (TEST_ID_MATCH(TEST_ID_PATTERN(divide_by_zero_ex), test_num)) {
        xTaskCreatePinnedToCore(TEST_ENTRY(divide_by_zero_ex), "divide_by_zero_ex", 2048, NULL, 5, NULL, core_num);
	} else if (TEST_ID_MATCH(TEST_ID_PATTERN(pseudo_debug_ex), test_num)) {
        xTaskCreatePinnedToCore(TEST_ENTRY(pseudo_debug_ex), "pseudo_debug_ex", 2048, NULL, 5, NULL, core_num);
    } else if (TEST_ID_MATCH(TEST_ID_PATTERN(pseudo_coprocessor_ex), test_num)) {
        xTaskCreatePinnedToCore(TEST_ENTRY(pseudo_coprocessor_ex), "pseudo_coprocessor_ex", 2048, NULL, 5, NULL, core_num);
#endif
    } else if (TEST_ID_MATCH(TEST_ID_PATTERN(gh264_psram_check), test_num)) {
        xTaskCreatePinnedToCore(TEST_ENTRY(gh264_psram_check), "gh264_psram_check_task", 4096, NULL, 5, NULL, core_num);
    } else if (TEST_ID_MATCH(TEST_ID_PATTERN(psram_with_flash_breakpoints), test_num)) {
        xTaskCreatePinnedToCore(TEST_ENTRY(psram_with_flash_breakpoints), "psram_task", 4096, NULL, 5, NULL, core_num);
    } else {
        return UT_UNSUPPORTED;
    }
    return UT_OK;
}
