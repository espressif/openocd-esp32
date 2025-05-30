/* Blink Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <string.h>
#include <fnmatch.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "gen_ut_app.h"
#if CONFIG_IDF_TARGET_ARCH_XTENSA
#if (UT_IDF_VER_MAJOR == 5) && (UT_IDF_VER_MINOR <= 1)
#include "freertos/xtensa_api.h"
#else
#include "xtensa_api.h"
#endif
#include "xtensa/core-macros.h"
#endif

#include "esp_memory_utils.h"
#include "driver/gpio.h"
#include "test_timer.h"

#define LOG_LOCAL_LEVEL CONFIG_LOG_DEFAULT_LEVEL
#include "esp_log.h"

const static char *TAG = "ut_app";

#define SPIRAM_TEST_ARRAY_SZ    5

// test app algorithm selector
volatile static int s_run_test = CONFIG_GEN_UT_APP_RUNTEST;
volatile static char s_run_test_str[256];
volatile static int s_run_core = -1;

// vars for WP tests
volatile static int s_count1 = 0;
volatile static int s_count2 = 100;
volatile static int s_count3 = 200;

extern ut_result_t gcov_test_do(int test_num, int core_num);
extern ut_result_t thread_test_do(int test_num, int core_num);
extern ut_result_t tracing_test_do(int test_num, int core_num);
extern ut_result_t semihost_test_do(int test_num, int core_num);
extern ut_result_t special_test_do(int test_num, int core_num);

static test_func_t s_test_funcs[] = {
    gcov_test_do,
    thread_test_do,
    tracing_test_do,
    semihost_test_do,
    special_test_do // test num start from 800
    //TODO: auto-manage test numbers and addition of new tests
};

static void blink_task(void *pvParameter)
{
    struct timer_task_arg *arg = (struct timer_task_arg *)pvParameter;
    if (arg) {
        int res = test_timer_init(arg);
        if (res != ESP_OK) {
            ESP_LOGE(TAG, "Failed to initilaze timer (%d)!", res);
            return;
        }
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
        ESP_LOGI(TAG, "Toggle LED %d, curr %d", s_count1, gpio_get_level(BLINK_GPIO)); TEST_BREAK_LOC(s_count10);
        /* Blink off (output low) */
        gpio_set_level(BLINK_GPIO, 0);                  TEST_BREAK_LOC(gpio_set_level0);
        vTaskDelay(100 / portTICK_PERIOD_MS);           TEST_BREAK_LOC(vTaskDelay0);
        /* Blink on (output high) */
        gpio_set_level(BLINK_GPIO, 1);                  TEST_BREAK_LOC(gpio_set_level1);
        vTaskDelay(100 / portTICK_PERIOD_MS);           TEST_BREAK_LOC(vTaskDelay1);
        s_count1++;                                     TEST_BREAK_LOC(s_count11);
        s_count2--;                                     TEST_BREAK_LOC(s_count2);
        s_count3++;                                     TEST_BREAK_LOC(s_count3);
    }
}

TEST_DECL(rom_bp_test, "test_bp.*.test_bp_in_rom")
{
    while(1) {
        esp_rom_printf("Hello, world!\n");
        esp_rom_delay_us(100000);
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}

TEST_DECL(gdb_detach, "test_connect.GDBConnectTests*.test_gdb_detach")
{
    ESP_LOGI(TAG, "Detach test started");
    while(1) {
        gpio_reset_pin(BLINK_GPIO);
        LABEL_SYMBOL(gdb_detach0);
        gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
        LABEL_SYMBOL(gdb_detach1);
        vTaskDelay(100 / portTICK_PERIOD_MS);
        gpio_set_level(BLINK_GPIO, 0);
        LABEL_SYMBOL(gdb_detach2);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        gpio_set_level(BLINK_GPIO, 1);
        LABEL_SYMBOL(gdb_detach3);
    }
}

void unused_func0(void)
{
    s_tmp_ln++;
}
void unused_func1(void)
{
    s_tmp_ln++;
}
void unused_func2(void)
{
    s_tmp_ln++;
}
void unused_func3(void)
{
    s_tmp_ln++;
}
void unused_func4(void)
{
    s_tmp_ln++;
}
void unused_func5(void)
{
    s_tmp_ln++;
}
void unused_func6(void)
{
    s_tmp_ln++;
}
void unused_func7(void)
{
    s_tmp_ln++;
}


/* This test calls functions recursively many times, exhausting the
 * register space and triggering window overflow exceptions.
 * Upon returning, it triggers window underflow exceptions.
 * If the test passes, then OpenOCD and GDB can both handle
 * window exceptions correctly.
 */
int sum;  // not static so the whole loop is not optimized away

static void recursive(int levels)
{
    LABEL_SYMBOL(_recursive_func);

    if (levels - 1 == 0) {
        return;
    }
    sum += levels;
    recursive(levels - 1);
}

TEST_DECL(window_exception, "test_step.DebuggerStepTests*.test_step*_window_exception*")
{
    recursive(20);
    printf("sum=%d\n",sum);
}


/* Three nested functions - from nested_top to nested_bottom
 */
static  __attribute__((noinline)) void nested_bottom()
{
    sum++;  // you have reached the bottom
}

static __attribute__((noinline)) void nested_middle()
{
    sum++;
    nested_bottom();
}

static  __attribute__((noinline)) void nested_top()
{
    sum++;
    nested_middle();
}

TEST_DECL(step_out_of_function, "test_step.DebuggerStepTests*.test_step_out_of_function")
{
    while(1)
    {
        sum = 0;
        nested_top();
    }
}

#if CONFIG_IDF_TARGET_ARCH_XTENSA
#define L5_TIMER_INUM   16

TEST_DECL(level5_int, "test_step.DebuggerStepTests*.test_step_level5_int")
{
    XTHAL_SET_CCOMPARE(2, XTHAL_GET_CCOUNT() + 1000000);
    xt_ints_on(BIT(L5_TIMER_INUM));
    while(true) {
        vTaskDelay(1);
    }
}

TEST_DECL(step_over_insn_using_scratch_reg, "test_step.DebuggerStepTests*.test_step_over_insn_using_scratch_reg")
{
    int val = 100;
    while(1) {
        __asm__ volatile (
            ".global _scratch_reg_using_task_break\n" \
            ".type   _scratch_reg_using_task_break,@function\n" \
            "_scratch_reg_using_task_break:\n" \
            "   mov a3,%0\n" \
            "   mov a4,a3\n" \
            ::"r"(val):"a3", "a4");
        if (++val == 2000)
            val = 100;
    }
}
#endif

static void IRAM_ATTR dummy_iram_func(void)
{
#if CONFIG_IDF_TARGET_ARCH_XTENSA
    __asm__ volatile (
        ".global _step_over_bp_break5\n" \
        ".type   _step_over_bp_break5,@function\n" \
        "_step_over_bp_break5:\n" \
        "   movi a3,30\n" \
        "   nop\n" \
        ".global _step_over_bp_break6\n" \
        ".type   _step_over_bp_break6,@function\n" \
        "_step_over_bp_break6:\n" \
        "   mov a4,a3\n" \
        :::"a3", "a4");
#elif CONFIG_IDF_TARGET_ARCH_RISCV
    __asm__ volatile (
        ".option push\n" \
        ".option norvc\n" \
        ".global _step_over_bp_break5\n" \
        ".type   _step_over_bp_break5,@function\n" \
        "_step_over_bp_break5:\n" \
        "   li a3,30\n" \
        "   nop\n" \
        ".global _step_over_bp_break6\n" \
        ".type   _step_over_bp_break6,@function\n" \
        "_step_over_bp_break6:\n" \
        "   mv a4,a3\n" \
    	".option pop\n" \
        :::"a3", "a4");
#endif
}

TEST_DECL(step_over_bp, "test_step.DebuggerStepTests*.test_step_over_bp")
{
    while(1) {
#if CONFIG_IDF_TARGET_ARCH_XTENSA
        __asm__ volatile (
            ".global _step_over_bp_break1\n" \
            ".type   _step_over_bp_break1,@function\n" \
            "_step_over_bp_break1:\n" \
            "   movi a3,20\n" \
            "   nop\n" \
            ".global _step_over_bp_break2\n" \
            ".type   _step_over_bp_break2,@function\n" \
            "_step_over_bp_break2:\n" \
            "   mov a4,a3\n" \
            ".global _step_over_bp_break3\n" \
            ".type   _step_over_bp_break3,@function\n" \
            "_step_over_bp_break3:\n" \
            "   movi a4,10\n" \
            "   nop\n" \
            ".global _step_over_bp_break4\n" \
            ".type   _step_over_bp_break4,@function\n" \
            "_step_over_bp_break4:\n" \
            "   mov a5,a4\n" \
            :::"a3","a4","a5");
#elif CONFIG_IDF_TARGET_ARCH_RISCV
        __asm__ volatile (
            ".option push\n" \
            ".option norvc\n" \
            ".global _step_over_bp_break1\n" \
            ".type   _step_over_bp_break1,@function\n" \
            "_step_over_bp_break1:\n" \
            "   li a3,20\n" \
            "   nop\n" \
            ".global _step_over_bp_break2\n" \
            ".type   _step_over_bp_break2,@function\n" \
            "_step_over_bp_break2:\n" \
            "   mv a4,a3\n" \
            ".global _step_over_bp_break3\n" \
            ".type   _step_over_bp_break3,@function\n" \
            "_step_over_bp_break3:\n" \
            "   li a4,10\n" \
            "   nop\n" \
            ".global _step_over_bp_break4\n" \
            ".type   _step_over_bp_break4,@function\n" \
            "_step_over_bp_break4:\n" \
            "   mv a5,a4\n" \
        	".option pop\n" \
            :::"a3","a4","a5");
#endif
        dummy_iram_func();
    }
}

TEST_DECL(fibonacci_calc, "test_step.DebuggerStepTests*.test_step_multimode")
/* calculation of 3 Fibonacci sequences: f0, f1 and f2
 * f(n) = f(n-1) + f(n-2) -> f(n) : 0, 1, 1, 2, 3, 5, 8, 13, 21, 34, ...*/
{
    volatile int f0_nm2, f1_nm2, f2_nm2; // n-2
    volatile int f0_nm1, f1_nm1, f2_nm1; // n-1
    volatile int f0_n, f1_n, f2_n; // n
    // setting three starting state for each f sequence: n-2 points:
    f0_nm2 = 0;
    f1_nm2 = 1;
    f2_nm2 = 3;
    // setting three starting state for each f sequence: n-1 points:
    f0_nm1 = 1;
    f1_nm1 = 2;
    f2_nm1 = 5;
    while (1)
    {
        LABEL_SYMBOL(fib_while);
        f0_n = f0_nm1 + f0_nm2; // calculating f0_n
        f0_nm2 = f0_nm1; // n shift
        f0_nm1 = f0_n;
        f1_n = f1_nm1 + f1_nm2; // calculating f1_n
        f1_nm2 = f1_nm1; // n shift
        f1_nm1 = f1_n;
        f2_n = f2_nm1 + f2_nm2;
        f2_nm2 = f2_nm1; // n shift// calculating f2_n
        f2_nm1 = f2_n;
    }
}

TEST_DECL(cores_concurrently_hit, "test_bp.Debugger*Tests*.test_2cores_concurrently_hit*")
{
    xTaskCreatePinnedToCore(&blink_task, "blink_task0", 4096, NULL, 5, NULL, 0);
    xTaskCreatePinnedToCore(&blink_task, "blink_task1", 4096, NULL, 5, NULL, 1);
}

#if CONFIG_IDF_TARGET_ARCH_XTENSA
TEST_DECL(step_over_inst_changing_intlevel, "test_step.DebuggerStepTests*.test_step_over_intlevel_disabled_isr")
{
    while(1)
    {
        __asm__ volatile (
            " rsil      a2, 4\n"             // a2 = ps, ps.intlevel = 4
            ".global _step_over_intlevel_ch\n"
            ".type   _step_over_intlevel_ch, @function\n"
            "_step_over_intlevel_ch:\n"
            " wsr       a2, ps\n"           // ps = a2
            " movi      a2, 0\n"            // a2 = 0
        );
    }
}
#endif

#if CONFIG_IDF_TARGET_ARCH_RISCV
TEST_DECL(step_isr_masking_check_mstatus, "test_step.DebuggerStepTests*.test_step_isr_masking_check_mstatus")
{
    while (1)
    {
        __asm__ volatile (
            "csrc mstatus, a0\n"
            "csrc mstatus, a0\n"
            "csrs mstatus, a0\n"
            "csrs mstatus, a0\n"
            "csrw mstatus, a0\n"
            "csrw mstatus, a0\n"
        );
    }
}
#endif

// match test string ID with pattern. See fnmatch for wildcard format description.
bool test_id_match(const char *pattern, const char *id)
{
    if (esp_ptr_internal(id) && esp_ptr_byte_accessible(id)) {
        return fnmatch(pattern, id, 0) == 0;
    }
    return false;
}

void app_main()
{
    if (s_run_test == -1) {
        ESP_LOGI(TAG, "Run test '%s'\n", s_run_test_str);
        s_run_test = (int)s_run_test_str;
    } else {
        ESP_LOGI(TAG, "Run test %d\n", s_run_test);
    }
    int core_num = s_run_core < 0 || s_run_core >= portNUM_PROCESSORS ? portNUM_PROCESSORS-1 : s_run_core;

    if (TEST_ID_MATCH("blink", s_run_test)) {
        static struct timer_task_arg task_arg = { .tim_grp = TEST_TIMER_GROUP_1, .tim_id = TEST_TIMER_0, .tim_period = 500000UL, .isr_func = test_timer_isr};
        xTaskCreatePinnedToCore(&blink_task, "blink_task", 4096, &task_arg, 5, NULL, core_num);
    } else if (TEST_ID_MATCH(TEST_ID_PATTERN(rom_bp_test), s_run_test)) {
        xTaskCreatePinnedToCore(TEST_ENTRY(rom_bp_test), "rom_bp_test_task", 2048, NULL, 5, NULL, core_num);
    } else if (TEST_ID_MATCH(TEST_ID_PATTERN(cores_concurrently_hit), s_run_test)) {
        TEST_ENTRY(cores_concurrently_hit)(NULL);
#if CONFIG_IDF_TARGET_ARCH_XTENSA
    } else if (TEST_ID_MATCH(TEST_ID_PATTERN(step_over_insn_using_scratch_reg), s_run_test)) {
        xTaskCreatePinnedToCore(TEST_ENTRY(step_over_insn_using_scratch_reg), "sreg_task", 2048, NULL, 5, NULL, core_num);
#endif
    } else if (TEST_ID_MATCH(TEST_ID_PATTERN(step_over_bp), s_run_test)) {
        xTaskCreatePinnedToCore(TEST_ENTRY(step_over_bp), "step_over_bp_task", 2048, NULL, 5, NULL, core_num);
    } else if (TEST_ID_MATCH(TEST_ID_PATTERN(fibonacci_calc), s_run_test)) {
        xTaskCreatePinnedToCore(TEST_ENTRY(fibonacci_calc), "fibonacci_calc", 2048, NULL, 5, NULL, core_num);
    } else if (TEST_ID_MATCH(TEST_ID_PATTERN(gdb_detach), s_run_test)) {
        xTaskCreatePinnedToCore(TEST_ENTRY(gdb_detach), "gdb_detach_task", 4096, NULL, 5, NULL, core_num);
#if CONFIG_IDF_TARGET_ARCH_XTENSA
    } else if (TEST_ID_MATCH(TEST_ID_PATTERN(step_over_inst_changing_intlevel), s_run_test)) {
        xTaskCreatePinnedToCore(TEST_ENTRY(step_over_inst_changing_intlevel), "step_over_inst_changing_intlevel", 2048, NULL, 5, NULL, core_num);
#endif
    } else if (TEST_ID_MATCH(TEST_ID_PATTERN(window_exception), s_run_test)) {
        xTaskCreatePinnedToCore(TEST_ENTRY(window_exception), "win_exc_task", 8192, NULL, 5, NULL, core_num);
    } else if (TEST_ID_MATCH(TEST_ID_PATTERN(step_out_of_function), s_run_test)) {
        xTaskCreatePinnedToCore(TEST_ENTRY(step_out_of_function), "step_out_func", 2048, NULL, 5, NULL, core_num);
#if CONFIG_IDF_TARGET_ARCH_XTENSA
    } else if (TEST_ID_MATCH(TEST_ID_PATTERN(level5_int), s_run_test)) {
        xTaskCreatePinnedToCore(TEST_ENTRY(level5_int), "level5_int_test", 2048, NULL, 5, NULL, core_num);
#endif
#if CONFIG_IDF_TARGET_ARCH_RISCV
    } else if (TEST_ID_MATCH(TEST_ID_PATTERN(step_isr_masking_check_mstatus), s_run_test)) {
        xTaskCreate(TEST_ENTRY(step_isr_masking_check_mstatus), "step_isr_masking_check_mstatus", 2048, NULL, 5, NULL);
#endif
    } else {
        ut_result_t res = UT_UNSUPPORTED;
        for (int i = 0; i < sizeof(s_test_funcs)/sizeof(s_test_funcs[0]); i++) {
            res = s_test_funcs[i](s_run_test, s_run_core);
            if (res != UT_UNSUPPORTED) {
                break;
            }
        }
        if (s_run_test != -1) {
            if (res == UT_UNSUPPORTED) {
                ESP_LOGE(TAG, "Invalid test id (%d)!", s_run_test);
            } else if (res != UT_OK) {
                ESP_LOGE(TAG, "Test %d failed (%d)!", s_run_test, res);
            } else {
                ESP_LOGI(TAG, "Test %d completed!", s_run_test);
            }
        } else {
            if (res == UT_UNSUPPORTED) {
                ESP_LOGE(TAG, "Invalid test id (%s)!", s_run_test_str);
            } else if (res != UT_OK) {
                ESP_LOGE(TAG, "Test '%s' failed (%d)!", s_run_test_str, res);
            } else {
                ESP_LOGI(TAG, "Test '%s' completed!", s_run_test_str);
            }
        }
        // wait forever
        ulTaskNotifyTake(pdFALSE, portMAX_DELAY);
    }
}
