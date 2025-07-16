#include "gen_ut_app.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#if CONFIG_ULP_COPROC_ENABLED
#include "ulp_lp_core.h"

const static char *TAG = "lpcore_test";

extern const uint8_t lp_core_main_bin_start[] asm("_binary_lp_core_main_bin_start");
extern const uint8_t lp_core_main_bin_end[]   asm("_binary_lp_core_main_bin_end");

static void lp_core_init(void)
{
    /* Set LP core wakeup source as the HP CPU */
    ulp_lp_core_cfg_t cfg = {
        .wakeup_source = ULP_LP_CORE_WAKEUP_SOURCE_HP_CPU,
    };

    /* Load LP core firmware */
    ESP_ERROR_CHECK(ulp_lp_core_load_binary(lp_core_main_bin_start, (lp_core_main_bin_end - lp_core_main_bin_start)));

    /* Run LP core */
    ESP_ERROR_CHECK(ulp_lp_core_run(&cfg));

    ESP_LOGI(TAG, "LP core loaded with firmware and running successfully\n");
}

TEST_DECL(debug_crash, "test_lpcore.LpCoreTests*.test_debug_crash")
{
    ESP_LOGI(TAG, "Initializing LP core...\n");
    /* Load LP Core binary and start the coprocessor */
    lp_core_init();

    ESP_LOGI(TAG, "Do some work on HP core...\n");
    while(1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
#endif // CONFIG_ULP_COPROC_ENABLED

ut_result_t lpcore_test_do(int test_num)
{
#if CONFIG_ULP_COPROC_ENABLED
    if (TEST_ID_MATCH(TEST_ID_PATTERN(debug_crash), test_num)) {
        xTaskCreatePinnedToCore(TEST_ENTRY(debug_crash), "debug_crash", 4096, NULL, 5, NULL, portNUM_PROCESSORS-1);
        return UT_OK;
    }
#endif // CONFIG_ULP_COPROC_ENABLED
    return UT_UNSUPPORTED;
}
