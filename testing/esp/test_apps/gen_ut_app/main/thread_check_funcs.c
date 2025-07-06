#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "gen_ut_app.h"

#define LOG_LOCAL_LEVEL CONFIG_LOG_DEFAULT_LEVEL
#include "esp_log.h"

const static char *TAG = "threads";

#define MAX_TASK_COUNT 6

volatile static int s_task_flags = 0;

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

SemaphoreHandle_t s_task_flags_mutex;

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

static void __attribute__((noinline, noreturn)) go_to_level_task1(int frame_level)
{
    if (--frame_level > 0) {
        go_to_level_task1(frame_level);
    }
    while (xSemaphoreTake(s_task_flags_mutex, portMAX_DELAY) != pdTRUE)
        ;
    s_task_flags |= (1 << 0);
    xSemaphoreGive(s_task_flags_mutex);
    TEST_BREAK_LOC(go_to_level_task1);
    while(1)
        ;
}

static void __attribute__((noinline, noreturn)) go_to_level_task2(int frame_level)
{
    if (--frame_level > 0) {
        go_to_level_task2(frame_level);
    }
    while (xSemaphoreTake(s_task_flags_mutex, portMAX_DELAY) != pdTRUE)
        ;
    s_task_flags |= (1 << 1);
    xSemaphoreGive(s_task_flags_mutex);
    TEST_BREAK_LOC(go_to_level_task2);
    while(1)
        ;
}

static void __attribute__((noinline, noreturn)) go_to_level_task3(int frame_level)
{
    if (--frame_level > 0) {
        go_to_level_task3(frame_level);
    }
    while (xSemaphoreTake(s_task_flags_mutex, portMAX_DELAY) != pdTRUE)
        ;
    s_task_flags |= (1 << 2);
    xSemaphoreGive(s_task_flags_mutex);
    TEST_BREAK_LOC(go_to_level_task3);
    while(1)
        ;
}

static void check_backtrace_task3(void *pvParameter)
{
    vTaskDelay(100);
    go_to_level_task3((int)pvParameter);
}

static void check_backtrace_task2(void *pvParameter)
{
    vTaskDelay(100);
    go_to_level_task2((int)pvParameter);
}

static void check_backtrace_task1(void *pvParameter)
{
    vTaskDelay(100);
    go_to_level_task1((int)pvParameter);
}

static void check_backtrace_test_done(void)
{
    ESP_LOGI(TAG, "Backtrace Thread test is done");
}

static void do_test_threads_backtraces(void)
{
    s_task_flags_mutex = xSemaphoreCreateMutex();
    TaskHandle_t handle;
    xTaskCreatePinnedToCore(&check_backtrace_task1, "check_bt_task1", 2048, (void*)3, 5, NULL, 0);
    xTaskCreatePinnedToCore(&check_backtrace_task2, "check_bt_task2", 2048, (void*)7, 5, NULL, 0);
    xTaskCreatePinnedToCore(&check_backtrace_task3, "check_bt_task3", 2048, (void*)5, 5, &handle, portNUM_PROCESSORS-1);
    /* wait untill all tasks are in dedicated loops */
    while (s_task_flags < 0x7) {
        vTaskDelay(10);
    }
    /* only one task was running on cpu1, making it more likely to run into an interrupt routine when the breakpoint is hit,
       suspend the task so that the backtrace is as expected */
    if (portNUM_PROCESSORS > 1) {
        vTaskSuspend(handle);
        while (eTaskGetState(handle) != eSuspended)
            ;
        vTaskDelay(1);
    }
    check_backtrace_test_done();
}

TEST_DECL(threads_backtraces, "test_threads.DebuggerThreadsTests*.test_threads_backtraces")
{
    do_test_threads_backtraces();
}

TEST_DECL(thread_switch, "test_threads.DebuggerThreadsTests*.test_thread_switch")
{
    ESP_LOGI(TAG, "Start Thread tests!");
    for (int i=0 ; i< 6 ; i++)
    {
        char task_name[64];
        sprintf(task_name, "thread_task%i", i);
        xTaskCreate(&thread_check_task, task_name, 2048, (void*)i, 5, NULL);
        vTaskDelay(1);
    }
}

TEST_DECL(thread_registers, "test_threads.DebuggerThreadsTests*.test_thread_registers")
{
    do_test_threads_backtraces();
}

ut_result_t thread_test_do(int test_num, int core_num)
{
    if (TEST_ID_MATCH(TEST_ID_PATTERN(threads_backtraces), test_num)) {
        xTaskCreatePinnedToCore(TEST_ENTRY(threads_backtraces), "check_bt_ctrl_task", 2048, NULL, 5, NULL, 0);
    } else if (TEST_ID_MATCH(TEST_ID_PATTERN(thread_switch), test_num)) {
        TEST_ENTRY(thread_switch)(NULL);
    } else if (TEST_ID_MATCH(TEST_ID_PATTERN(thread_registers), test_num)) {
        xTaskCreatePinnedToCore(TEST_ENTRY(thread_registers), "check_bt_ctrl_task", 2048, NULL, 5, NULL, 0);
    } else {
        return UT_UNSUPPORTED;
    }
    return UT_OK;
}
