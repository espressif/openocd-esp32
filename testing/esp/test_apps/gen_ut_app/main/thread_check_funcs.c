#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "gen_ut_app.h"

#define LOG_LOCAL_LEVEL CONFIG_LOG_DEFAULT_LEVEL
#include "esp_log.h"

const static char *TAG = "threads";

#define MAX_TASK_COUNT 6

static void test_check_nops(int param)
{
    asm("   nop;");
    asm("test_check_bp:   nop;");
    asm("   nop;");
    asm("   nop;");
    asm("   nop;");
    asm("   nop;");
    asm("   nop;");
    asm("   nop;");
    asm("   nop;");
}

void thread_check_task(void *pvParameter)
{
    int arg = (int)pvParameter;
    int sem_relese_id = arg + 1;
    if (sem_relese_id >= MAX_TASK_COUNT) sem_relese_id = 0;

    ESP_LOGI(TAG, "Start task %d",  arg);
    vTaskDelay(3000 / portTICK_PERIOD_MS);
    while(1) {
        test_check_nops(sem_relese_id);
    }
}

static void __attribute__((noinline)) go_to_level_task1(int frame_level)
{
    if (--frame_level > 0) {
        go_to_level_task1(frame_level);
    }
    while(1) {
        s_tmp_ln++; TEST_BREAK_LOC(go_to_level_task1);
    }
}

static void __attribute__((noinline)) go_to_level_task2(int frame_level)
{
    if (--frame_level > 0) {
        go_to_level_task2(frame_level);
    }
    while(1) {
        s_tmp_ln++; TEST_BREAK_LOC(go_to_level_task2);
    }
}

static void __attribute__((noinline)) go_to_level_task3(int frame_level)
{
    if (--frame_level > 0) {
        go_to_level_task3(frame_level);
    }
    while(1) {
        s_tmp_ln++; TEST_BREAK_LOC(go_to_level_task3);
    }
}

void check_backtrace_task3(void *pvParameter)
{
    vTaskDelay(100);
    go_to_level_task3((int)pvParameter);
}

void check_backtrace_task2(void *pvParameter)
{
    vTaskDelay(100);
    go_to_level_task2((int)pvParameter);
}

void check_backtrace_task1(void *pvParameter)
{
    vTaskDelay(100);
    go_to_level_task1((int)pvParameter);
}

ut_result_t thread_test_do(int test_num)
{
    switch (test_num) {
        case 400:
            xTaskCreatePinnedToCore(&check_backtrace_task1, "check_bt_task1", 2048, (void*)3, 5, NULL, 0);
            xTaskCreatePinnedToCore(&check_backtrace_task2, "check_bt_task2", 2048, (void*)7, 5, NULL, 0);
#if CONFIG_FREERTOS_UNICORE
            xTaskCreatePinnedToCore(&check_backtrace_task3, "check_bt_task3", 2048, (void*)5, 5, NULL, 0);
#else
            xTaskCreatePinnedToCore(&check_backtrace_task3, "check_bt_task3", 2048, (void*)5, 5, NULL, 1);
#endif
            break;
        case 401:
            ESP_LOGE(TAG, "Start Thread tests!");
            for (int i=0 ; i< 6 ; i++)
            {
                char task_name[64];
                sprintf(task_name, "thread_task%i", i);
                xTaskCreate(&thread_check_task, task_name, 2048, (void*)i, 5, NULL);
                vTaskDelay(1);
            }
            break;
        default:
            return UT_UNSUPPORTED;
    }
    return UT_OK;
}
