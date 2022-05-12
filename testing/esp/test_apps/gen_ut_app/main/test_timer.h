#ifndef TEST_TIMER_H
#define TEST_TIMER_H
#include "gen_ut_app.h"

#if UT_IDF_VER < MAKE_UT_IDF_VER(5,0,0,0)
#define LEGACY_TIMER_GROUP 1
#include "driver/timer.h"
typedef void (*isr_fptr_t)(void*);
#else
#include "driver/gptimer.h"
#define LEGACY_TIMER_GROUP 0
typedef bool (*isr_fptr_t)(gptimer_handle_t , const gptimer_alarm_event_data_t *, void *);
#endif

#define TEST_TIMER_GROUP_0      0
#define TEST_TIMER_GROUP_1      1
#define TEST_TIMER_0            0
#define TEST_TIMER_1            1

struct os_trace_task_arg {
    int tim_grp;
    int tim_id;
    uint32_t tim_period;
    uint32_t task_period;
};

struct timer_task_arg {
    int tim_grp;
    int tim_id;
    uint32_t tim_period;
    isr_fptr_t isr_func;
};

#if LEGACY_TIMER_GROUP == 1
void test_timer_isr(void *arg);
void os_trace_test_timer_isr(void *arg);
#else
bool test_timer_isr(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx);
bool os_trace_test_timer_isr(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx);
#endif
int test_timer_init(struct timer_task_arg* arg);

#endif