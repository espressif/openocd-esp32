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

#if LEGACY_TIMER_GROUP == 1
#define TIMER_DIVIDER         16  //  Hardware timer clock divider
#define TIMER_SCALE           (TIMER_BASE_CLK / TIMER_DIVIDER)  // convert counter value to seconds
#define TIM_CLR(_tg_, _tn_) do{  timer_group_clr_intr_status_in_isr(_tg_, _tn_); }while(0)

const static char *TAG = "test_timer";

void test_timer_rearm(int timer_group, int timer_idx)
{
    if (timer_group == 0) {
        if (timer_idx == 0) {
            TIM_CLR(0, 0);
            timer_set_alarm(0, 0, TIMER_ALARM_EN);
        } else {
#if !CONFIG_IDF_TARGET_ARCH_RISCV
            TIM_CLR(0, 1);
            timer_set_alarm(0, 1, TIMER_ALARM_EN);
#endif
        }
    } else if (timer_group == 1) {
        if (timer_idx == 0) {
            TIM_CLR(1, 0);
            timer_set_alarm(1, 0, TIMER_ALARM_EN);
        } else {
#if !CONFIG_IDF_TARGET_ARCH_RISCV
            TIM_CLR(1, 1);
            timer_set_alarm(1, 1, TIMER_ALARM_EN);
#endif
        }
    }
}

void test_timer_isr(void *arg)
{
    test_timer_isr_func();
    test_timer_isr_ram_func();
    struct timer_task_arg *tim_arg = (struct timer_task_arg *)arg;
    test_timer_rearm(tim_arg->tim_grp, tim_arg->tim_id);
}

void os_trace_test_timer_isr(void *arg)
{
    struct os_trace_task_arg *tim_arg = (struct os_trace_task_arg *)arg;
    test_timer_rearm(tim_arg->tim_grp, tim_arg->tim_id);
}

int test_timer_init(struct timer_task_arg* arg)
{
    if (!arg)
        return -1;

    timer_config_t config;
    uint64_t alarm_val = ((float)arg->tim_period / 1000000UL) * TIMER_SCALE;

    config.alarm_en = 1;
    config.auto_reload = 1;
    config.counter_dir = TIMER_COUNT_UP;
    config.divider = 2;     //Range is 2 to 65536
    config.intr_type = TIMER_INTR_LEVEL;
    config.counter_en = TIMER_PAUSE;
#if SOC_TIMER_GROUP_SUPPORT_XTAL
    config.clk_src = TIMER_SRC_CLK_APB;
#endif
    /*Configure timer*/
    timer_init(arg->tim_grp, arg->tim_id, &config);
    /*Stop timer counter*/
    timer_pause(arg->tim_grp, arg->tim_id);
    /*Load counter value */
    timer_set_counter_value(arg->tim_grp, arg->tim_id, 0x00000000ULL);
    /*Set alarm value*/
    timer_set_alarm_value(arg->tim_grp, arg->tim_id, alarm_val);
    /*Enable timer interrupt*/
    timer_enable_intr(arg->tim_grp, arg->tim_id);

    if (arg->isr_func != NULL) {
        int res = timer_isr_register(arg->tim_grp, arg->tim_id, arg->isr_func, arg, 0, NULL);
        if (res != ESP_OK) {
            ESP_LOGE(TAG, "Failed to register timer ISR (%d)!", res);
            return -1;
        }
        res = timer_start(arg->tim_grp, arg->tim_id);
        if (res != ESP_OK) {
            ESP_LOGE(TAG, "Failed to start timer (%d)!", res);
            return -1;
        }
    }
    else {
        return -1;
    }
    
    return 0;
}
#else
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
#if CONFIG_SOC_TIMER_GROUP_SUPPORT_APB
        .clk_src = GPTIMER_CLK_SRC_APB,
#elif CONFIG_SOC_TIMER_GROUP_SUPPORT_PLL_F40M
        .clk_src = GPTIMER_CLK_SRC_PLL_F40M,
#else
        .clk_src = GPTIMER_CLK_SRC_XTAL,
#endif
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
#endif

