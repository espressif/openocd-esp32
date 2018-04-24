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
#include "sdkconfig.h"

/* Can run 'make menuconfig' to choose the GPIO to blink,
   or you can edit the following line and set a number here.
*/
#define BLINK_GPIO CONFIG_BLINK_GPIO

#define LOG_LOCAL_LEVEL CONFIG_LOG_DEFAULT_LEVEL
#include "esp_log.h"
const static char *TAG = "ut_app";

// used to prevent linker from optimizing out the variables holding BP line numbers
volatile static int s_tmp_ln = 0;
// test app algorithm selector
volatile static int s_run_test = 0;
// vars for WP tests
volatile static int s_count1 = 0;
volatile static int s_count2 = 100;
volatile static int s_count3 = 200;

#define TEST_BREAK_LOC(_nm_)  \
    volatile static const int _nm_ ## _break_ln = __LINE__; \
    s_tmp_ln = _nm_ ## _break_ln;

void blink_task(void *pvParameter)
{
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

void scratch_reg_using_task(void *pvParameter)
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

/* Add new  */

extern void gcov_task(void *pvParameter);

void app_main()
{
    ESP_LOGI(TAG, "Run test %d\n", s_run_test);
    switch(s_run_test){
        case 100:
            xTaskCreate(&blink_task, "blink_task", 2048, NULL, 5, NULL);
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
