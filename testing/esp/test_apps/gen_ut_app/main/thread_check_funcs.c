#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "driver/timer.h"
#include "sdkconfig.h"

#define LOG_LOCAL_LEVEL CONFIG_LOG_DEFAULT_LEVEL
#include "esp_log.h"

const static char *TAG = "threads";

#define MAX_TASK_COUNT 6

void test_check_nops(int param)
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