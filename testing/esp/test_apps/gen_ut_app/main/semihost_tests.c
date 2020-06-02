/*
 * *** About the test ***
 *
 * N - number of cores
 *
 * Its sequence:
 * - There are N (test_read.*) files containing random bytes
 * - Test creates N (test_write.*) files
 * - Test opens the read- and write-files
 * - Test writes (test_read.*) content to (test_write.*)
 * - Test closes the files
 */

#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "gen_ut_app.h"
#if UT_IDF_VER >= MAKE_UT_IDF_VER(4,0,0,0)
#include "esp_vfs_semihost.h"

#include "esp_log.h"
const static char *TAG = "semihost_test";

static void semihost_task(void *pvParameter)
{
    uint8_t s_buf[512];
    int core_id = xPortGetCoreID();
    char fname[32];
    esp_err_t ret;
    ESP_LOGI(TAG, "Started test thread for a core #%d", core_id);

    /**** Init ****/
    ESP_LOGI(TAG, "CPU[%d]: Initialization", core_id);
    if (core_id == 0) {
        ret = esp_vfs_semihost_register("/host", NULL);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "CPU[%d]: Failed to register semihost driver (%s)!", core_id, esp_err_to_name(ret));
            return;
        }
#if !CONFIG_FREERTOS_UNICORE
        xTaskCreatePinnedToCore(&semihost_task, "semihost_task1", 4096, xTaskGetCurrentTaskHandle(), 5, NULL, 1);
        vTaskDelay(1);
#endif
    }

    /**** Opening files ****/
    snprintf(fname, sizeof(fname)-1, "/host/test_write.%d", core_id);
    ESP_LOGI(TAG, "CPU[%d]: Opening %s", core_id, fname);
    FILE *f_out = fopen(fname, "w+");
    if(f_out == NULL) {
        ESP_LOGE(TAG, "CPU[%d]: Failed to open file for writing (%d)!", core_id, errno);
        return;
    }
    snprintf(fname, sizeof(fname)-1, "/host/test_read.%d", core_id);
    ESP_LOGI(TAG, "CPU[%d]: Opening %s", core_id, fname);
    int fd_in = open(fname, O_RDONLY, 0);
    if(fd_in == -1) {
        ESP_LOGE(TAG, "CPU[%d]: Failed to open file for reading (%d)!", core_id, errno);
        fclose(f_out);
        return;
    }

    /**** Copy: In->Buf->Out ****/
    ESP_LOGI(TAG, "CPU[%d]: Writing test_read.%d ->  test_write.%d", core_id, core_id, core_id);
    ssize_t read_bytes;
    int count = 0;
    do {
        read_bytes = read(fd_in, s_buf, sizeof(s_buf)); // in -> buffer
        if(read_bytes == -1) {
            ESP_LOGE(TAG, "CPU[%d]: Failed to read file (%d)!", core_id, errno);
        } else if(read_bytes > 0) {
            fwrite(s_buf, 1, read_bytes, f_out); // buffer -> out
            count += read_bytes;
        }
    } while(read_bytes > 0);

    /***** Checking *****/
    long int f_size = ftell(f_out);
    if (count !=  f_size) {
        ESP_LOGE(TAG, "CPU[%d]: Failed to determine file size! (size is %ld)", core_id, f_size);
        return;
    }

    ESP_LOGI(TAG, "CPU[%d]: Read %d bytes", core_id, count);
    ESP_LOGI(TAG, "CPU[%d]: Wrote %ld bytes", core_id, f_size);

    /***** De-init *****/
    ESP_LOGI(TAG, "Closing the files");
    if (close(fd_in) == -1) {
        ESP_LOGE(TAG, "CPU[%d] :Failed to close input file (%d)!", core_id, errno);
    }
    if (fclose(f_out) != 0) {
        ESP_LOGE(TAG, "CPU[%d]: Failed to close output file (%d)!", core_id, errno);
    }
    ESP_LOGI(TAG, "CPU[%d]: Closed files", core_id);

    if (core_id == 0) {
#if !CONFIG_FREERTOS_UNICORE
        ulTaskNotifyTake(pdFALSE, portMAX_DELAY);
#endif
        ESP_LOGI(TAG, "CPU[%d]: Unregister host FS", core_id);
        ret = esp_vfs_semihost_unregister("/host");
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "CPU[%d]: Failed to unregister semihost driver (%s)!", core_id, esp_err_to_name(ret));
            return;
        }
    } else {
        xTaskNotifyGive((TaskHandle_t)pvParameter);
    }
    ESP_LOGI(TAG, "CPU[%d]: [ DONE ]", core_id);
    while(1) {
        vTaskDelay(1);
    }
}
#endif /* if UT_IDF_VER == UT_IDF_VER_LATEST */


ut_result_t semihost_test_do(int test_num)
{
    switch (test_num) {
#if UT_IDF_VER >= MAKE_UT_IDF_VER(4,0,0,0)
        case 700:
        {
            xTaskCreatePinnedToCore(&semihost_task, "semihost_task0", 4096, NULL, 5, NULL, 0);
            break;
        }
#endif /* #if UT_IDF_VER >= MAKE_UT_IDF_VER(4,0,0,0) */
        default:
            return UT_UNSUPPORTED;
    }
    return UT_OK;
}
