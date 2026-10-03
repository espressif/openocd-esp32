#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_attr.h"
#include "esp_bit_defs.h"
#include "sdkconfig.h"
#include "gen_ut_app.h"

#if CONFIG_ESP_RISCV_TRACE_ENABLE

#define RISCV_TRACE_WORK_ITERATIONS 10000
#define RISCV_TRACE_WORK_STACK_SIZE 2048
#define RISCV_TRACE_WORK_PRIORITY   5

static volatile uint32_t s_riscv_trace_sink;

/* Branchy loop that fills and wraps the trace buffer within milliseconds */
static void riscv_trace_branchy_work(void)
{
    uint32_t acc = s_riscv_trace_sink;
    for (volatile int i = 0; i < RISCV_TRACE_WORK_ITERATIONS; i++) {
        if ((i & 3) == 0) {
            acc += (uint32_t)i;
        } else {
            acc ^= ((uint32_t)i << 1);
        }
    }
    s_riscv_trace_sink = acc;
}

static void riscv_trace_work_task(void *arg)
{
    (void)arg;
    for (;;) {
        riscv_trace_branchy_work();
        /* Lets the idle task run so the task watchdog stays quiet */
        vTaskDelay(1);
    }
}

TEST_DECL(riscv_trace_loop, "test_riscv_trace.RiscvTraceTests*.test_riscv_trace_loop*")
{
    for (int core = 0; core < portNUM_PROCESSORS; core++) {
        if (CONFIG_ESP_RISCV_TRACE_CORE_MASK & BIT(core)) {
            xTaskCreatePinnedToCore(riscv_trace_work_task, "riscv_trace_work", RISCV_TRACE_WORK_STACK_SIZE, NULL,
                                    RISCV_TRACE_WORK_PRIORITY, NULL, core);
        }
    }
}

#endif /* CONFIG_ESP_RISCV_TRACE_ENABLE */

ut_result_t riscv_trace_test_do(int test_num, int core_num)
{
    (void)core_num;
#if CONFIG_ESP_RISCV_TRACE_ENABLE
    if (TEST_ID_MATCH(TEST_ID_PATTERN(riscv_trace_loop), test_num)) {
        TEST_ENTRY(riscv_trace_loop)(NULL);
        return UT_OK;
    }
#endif
    return UT_UNSUPPORTED;
}
