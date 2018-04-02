/* Blink Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/timer.h"
#include "sdkconfig.h"

/* Can run 'make menuconfig' to choose the GPIO to blink,
   or you can edit the following line and set a number here.
*/
#define BLINK_GPIO CONFIG_BLINK_GPIO

#define LOG_LOCAL_LEVEL CONFIG_LOG_DEFAULT_LEVEL
#include "esp_log.h"
const static char *TAG = "ut_app";

#define TEST_BREAK_LOC(_nm_)  \
    volatile static const int _nm_ ## _break_ln = __LINE__; \
    s_tmp_ln = _nm_ ## _break_ln;

// used to prevent linker from optimizing out the variables holding BP line numbers
volatile static int s_tmp_ln = 0;
// test app algorithm selector
volatile static int s_run_test = 0;
// vars for WP tests
volatile static int s_count1 = 0;
volatile static int s_count2 = 100;
volatile static int s_count3 = 200;

extern void gcov_task(void *pvParameter);

struct blink_task_arg {
    int tim_grp;
    int tim_id;
    uint32_t tim_period;
};

static void test_timer_init(int timer_group, int timer_idx, uint32_t period)
{
    timer_config_t config;
    uint64_t alarm_val = (period * (TIMER_BASE_CLK / 1000000UL)) / 2;

    config.alarm_en = 1;
    config.auto_reload = 1;
    config.counter_dir = TIMER_COUNT_UP;
    config.divider = 2;     //Range is 2 to 65536
    config.intr_type = TIMER_INTR_LEVEL;
    config.counter_en = TIMER_PAUSE;
    /*Configure timer*/
    timer_init(timer_group, timer_idx, &config);
    /*Stop timer counter*/
    timer_pause(timer_group, timer_idx);
    /*Load counter value */
    timer_set_counter_value(timer_group, timer_idx, 0x00000000ULL);
    /*Set alarm value*/
    timer_set_alarm_value(timer_group, timer_idx, alarm_val);
    /*Enable timer interrupt*/
    timer_enable_intr(timer_group, timer_idx);
}

static void test_timer_isr_func(void)
{
    s_tmp_ln++;
}

static void IRAM_ATTR test_timer_isr_ram_func(void)
{
    s_tmp_ln++;
}

static void test_timer_isr(void *arg)
{
    struct blink_task_arg *tim_arg = (struct blink_task_arg *)arg;

    test_timer_isr_func();
    test_timer_isr_ram_func();

    if (tim_arg->tim_grp == 0) {
        if (tim_arg->tim_id == 0) {
            TIMERG0.int_clr_timers.t0 = 1;
            TIMERG0.hw_timer[0].update = 1;
            TIMERG0.hw_timer[0].config.alarm_en = 1;
        } else {
            TIMERG0.int_clr_timers.t1 = 1;
            TIMERG0.hw_timer[1].update = 1;
            TIMERG0.hw_timer[1].config.alarm_en = 1;
        }
    } else if (tim_arg->tim_grp == 1) {
        if (tim_arg->tim_id == 0) {
            TIMERG1.int_clr_timers.t0 = 1;
            TIMERG1.hw_timer[0].update = 1;
            TIMERG1.hw_timer[0].config.alarm_en = 1;
        } else {
            TIMERG1.int_clr_timers.t1 = 1;
            TIMERG1.hw_timer[1].update = 1;
            TIMERG1.hw_timer[1].config.alarm_en = 1;
        }
    }
}

static void blink_task(void *pvParameter)
{
    struct blink_task_arg *arg = (struct blink_task_arg *)pvParameter;

    if (arg) {
        test_timer_init(arg->tim_grp, arg->tim_id, arg->tim_period);
        int res = timer_isr_register(arg->tim_grp, arg->tim_id, test_timer_isr, arg, 0, NULL);
        if (res != ESP_OK) {
            ESP_LOGE(TAG, "Failed to register timer ISR (%d)!", res);
            return;
        }
        res = timer_start(arg->tim_grp, arg->tim_id);
        if (res != ESP_OK) {
            ESP_LOGE(TAG, "Failed to start timer (%d)!", res);
            return;
        }
    }
    /* Configure the IOMUX register for pad BLINK_GPIO (some pads are
       muxed to GPIO on reset already, but some default to other
       functions and need to be switched to GPIO. Consult the
       Technical Reference for a list of pads and their default
       functions.)
    */
    gpio_pad_select_gpio(BLINK_GPIO);
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

/* This test calls functions recursively many times, exhausing the
 * register space and triggering window overflow exceptions.
 * Upon returning, it triggers window underflow exceptions.
 * If the test passes, then OpenOCD and GDB can both handle
 * window exceptions correctly.
 */
int sum;  // not static so the whole loop is not optimized away

static void recursive(int levels)
{
    if (levels - 1 == 0) {
        return;
    }
    sum += levels;
    recursive(levels - 1);
}

void window_exception_test(void* arg)
{
    recursive(20);
    printf("sum=%d\n",sum);
}

static void scratch_reg_using_task(void *pvParameter)
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

static void IRAM_ATTR dummy_iram_func(void)
{
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
}

static void step_over_bp_task(void *pvParameter)
{
    while(1) {
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
        dummy_iram_func();
    }
}

/* Add new  */

void app_main()
{
    static struct blink_task_arg task_arg = { .tim_grp = TIMER_GROUP_1, .tim_id = TIMER_0, .tim_period = 500000UL};

    ESP_LOGI(TAG, "Run test %d\n", s_run_test);
    switch(s_run_test){
        case 100:
            xTaskCreate(&blink_task, "blink_task", 2048, &task_arg, 5, NULL);
            break;
        case 101:
            xTaskCreatePinnedToCore(&blink_task, "blink_task0", 2048, NULL, 5, NULL, 0);
            xTaskCreatePinnedToCore(&blink_task, "blink_task1", 2048, NULL, 5, NULL, 1);
            break;
        case 102:
#if CONFIG_FREERTOS_UNICORE
            xTaskCreatePinnedToCore(&scratch_reg_using_task, "sreg_task", 2048, NULL, 5, NULL, 0);
#else
            xTaskCreatePinnedToCore(&scratch_reg_using_task, "sreg_task", 2048, NULL, 5, NULL, 1);
#endif
            break;
        case 103:
            xTaskCreate(&step_over_bp_task, "step_over_bp_task", 2048, NULL, 5, NULL);
            break;
        case 200:
            xTaskCreate(&window_exception_test, "win_exc_task", 8192, NULL, 5, NULL);
            break;
#if CONFIG_ESP32_GCOV_ENABLE
        case 300:
            xTaskCreate(&gcov_task, "gcov_task", 2048, (void *)true, 5, NULL);
            break;
        case 301:
            xTaskCreate(&gcov_task, "gcov_task", 2048, (void *)false, 5, NULL);
            break;
#endif
        default:
            ESP_LOGE(TAG, "Invalid test id (%d)!", s_run_test);
            while(1){
              vTaskDelay(1);
            }
    }
}
