#include "esp_log.h"
#include "test_timer.h"
#include "esp_system.h" 

static void test_timer_isr_func(void)
{
    s_tmp_ln++;
}

static void IRAM_ATTR test_timer_isr_ram_func(void)
{
    s_tmp_ln++;
}

bool test_timer_isr(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx)
{
    test_timer_isr_func();
    test_timer_isr_ram_func();
    return true;
}

bool os_trace_test_timer_isr(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx)
{
    return true;
}

int test_timer_init(struct timer_task_arg* arg)
{
    if (!arg)
        return -1;

    gptimer_handle_t gptimer = NULL;
    gptimer_config_t config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 2.5 * 1000 * 1000,
    };
    uint64_t alarm_val = ((float)arg->tim_period / 1000000UL) *  (config.resolution_hz);

    gptimer_alarm_config_t alarm_config = { 
        .alarm_count = alarm_val, 
        .flags.auto_reload_on_alarm = true,
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&config, &gptimer));

    gptimer_event_callbacks_t cbs = {
        .on_alarm = arg->isr_func,
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(gptimer, &cbs, NULL));
    ESP_ERROR_CHECK(gptimer_enable(gptimer));
    ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer, &alarm_config));
    ESP_ERROR_CHECK(gptimer_start(gptimer));
    
    return 0;
}
