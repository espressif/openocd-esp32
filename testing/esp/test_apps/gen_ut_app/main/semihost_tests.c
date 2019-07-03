//#include <stdio.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "gen_ut_app.h"
#if UT_IDF_VER == UT_IDF_VER_LATEST
#include "esp_vfs_semihost.h"

#include "esp_log.h"
const static char *TAG = "semihost_test";

static void semihost_task(void *pvParameter)
{
    uint8_t s_buf[512];
    int core_id = xPortGetCoreID();
    char fname[32];
    esp_err_t ret;

    if (core_id == 0) {
        ret = esp_vfs_semihost_register("/host", NULL);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to register semihost driver (%s)!", esp_err_to_name(ret));
            return;
        }
#if !CONFIG_FREERTOS_UNICORE
        xTaskCreatePinnedToCore(&semihost_task, "semihost_task1", 2048, xTaskGetCurrentTaskHandle(), 5, NULL, 1);
        vTaskDelay(1);
#endif
    }
    snprintf(fname, sizeof(fname)-1, "/host/test_write.%d", core_id);
    FILE *f_out = fopen(fname, "w");
    if(f_out == NULL) {
        ESP_LOGE(TAG, "Failed to open file for writing (%d)!", errno);
        return;
    }
    snprintf(fname, sizeof(fname)-1, "/host/test_read.%d", core_id);
    int fd_in = open(fname, O_RDONLY, 0);
    if(fd_in == -1) {
        ESP_LOGE(TAG, "Failed to open file for reading (%d)!", errno);
        fclose(f_out);
        return;
    }
    ssize_t read_bytes;
    int count = 0;
    do {
        read_bytes = read(fd_in, s_buf, sizeof(s_buf));
        if(read_bytes == -1) {
            ESP_LOGE(TAG, "Failed to read file (%d)!", errno);
        } else if(read_bytes > 0) {
            fwrite(s_buf, 1, read_bytes, f_out);
            count += read_bytes;
        }
    } while(read_bytes > 0);

    ESP_LOGI(TAG, "CPU[%d]: Read %d bytes", core_id, count);
    ESP_LOGI(TAG, "CPU[%d]: Wrote %ld bytes", core_id, ftell(f_out));

    if (close(fd_in) == -1) {
        ESP_LOGE(TAG, "Failed to close input file (%d)!", errno);
    }
    if (fclose(f_out) != 0) {
        ESP_LOGE(TAG, "Failed to close output file (%d)!", errno);
    }
    ESP_LOGI(TAG, "CPU[%d]: Closed files", core_id);

    if (core_id == 0) {                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             
#if !CONFIG_FREERTOS_UNICORE
        ulTaskNotifyTake(pdFALSE, portMAX_DELAY);
#endif
        ESP_LOGI(TAG, "Unregister host FS");
        ret = esp_vfs_semihost_unregister("/host");
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to unregister semihost driver (%s)!", esp_err_to_name(ret));
            return;
        }
    } else {
        xTaskNotifyGive((TaskHandle_t)pvParameter);
    }
    while(1) {
        vTaskDelay(1);
    }        
}
#endif

ut_result_t semihost_test_do(int test_num)
{
    switch (test_num) {
#if UT_IDF_VER == UT_IDF_VER_LATEST
        case 700:
        {
            xTaskCreatePinnedToCore(&semihost_task, "semihost_task0", 2048, NULL, 5, NULL, 0);
            break;
        }
#endif
        default:
            return UT_UNSUPPORTED;
    }
    return UT_OK;
}
