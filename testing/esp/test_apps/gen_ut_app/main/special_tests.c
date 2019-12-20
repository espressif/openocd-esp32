#include "driver/gpio.h"
#include "esp_spi_flash.h"
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
    int count = 0;
    gpio_pad_select_gpio(BLINK_GPIO);

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

static void psram_check_task(void *pvParameter)
{
    uint32_t *mem = (uint32_t *)SOC_EXTRAM_DATA_LOW;
    mem += xPortGetCoreID() * SPIRAM_TEST_ARRAY_SZ;
    for (int i = 0, k = 0x20; i < SPIRAM_TEST_ARRAY_SZ; i++, k++) {
        mem[i] = k;
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
        case 802:
        {
            xTaskCreatePinnedToCore(&psram_check_task, "psram_task", 2048, NULL, 5, NULL, portNUM_PROCESSORS-1);
            break;
        }
        default:
            return UT_UNSUPPORTED;
    }
    return UT_OK;
}
