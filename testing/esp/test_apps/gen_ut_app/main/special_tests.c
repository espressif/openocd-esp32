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
    ESP_LOGI(TAG, "Start crash task on core %d", xPortGetCoreID());
    int *p = 0;
    *p = 0; TEST_BREAK_LOC(crash);
}

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
            xTaskCreatePinnedToCore(&cache_check_task, "cache_check_task", 2048, NULL, 5, NULL, portNUM_PROCESSORS-1);
            break;
        }
        default:
            return UT_UNSUPPORTED;
    }
    return UT_OK;
}
