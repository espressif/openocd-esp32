#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "gen_ut_app.h"
#include "esp_app_trace.h"
#include "test_timer.h"
#include "esp_log.h"
const static char *TAG = "tracing_tests";

#if CONFIG_HEAP_TRACING
#include "esp_heap_trace.h"
#endif //CONFIG_HEAP_TRACING
#if CONFIG_SYSVIEW_ENABLE
#include "esp_sysview_trace.h"
#endif

struct trace_test_task_arg;
typedef void (*do_trace_test_t)(struct trace_test_task_arg* task);
typedef int (*trace_printf_t)(const char *fmt, ...);
typedef int (*trace_flush_t)(uint32_t tmo);
typedef bool (*trace_is_started_t)(void);

typedef struct trace_test_task_arg {
    do_trace_test_t test_func;
    TaskHandle_t    other_task;
    trace_printf_t  trace_printf;
    trace_flush_t   trace_flush;
    trace_is_started_t trace_is_started;
} trace_test_task_arg_t;

#if CONFIG_SYSVIEW_ENABLE
int do_trace_printf(const char *fmt, ...)
{
    va_list ap;

    va_start(ap, fmt);
    int ret = esp_sysview_vprintf(fmt, ap);
    va_end(ap);

    return ret;
}

static int do_trace_flush(uint32_t tmo)
{
    return esp_sysview_flush(tmo);
}

static bool do_trace_is_started(void)
{
    return SEGGER_SYSVIEW_Started();
}
#else
int do_trace_printf(const char *fmt, ...)
{
    return 0;
}

static int do_trace_flush(uint32_t tmo)
{
    return ESP_OK;
}

static bool do_trace_is_started(void)
{
    return true;
}
#endif

#if CONFIG_HEAP_TRACING
static void __attribute__((optimize("O0"))) do_trace_test_heap_log(uint32_t num, trace_printf_t trace_printf)
{
    volatile TaskHandle_t curr_task = xTaskGetCurrentTaskHandle();

    void *a = malloc(64); TEST_BREAK_LOC(malloc0);
    trace_printf("task[%p]: Sample print 0x%lx 0x%hx 0x%x %lu %hu %u %ld %hd %d %c\n",
        curr_task, num, num, num, num, num, num, num, num, num, 'A' + (num  % ('Z' - 'A')));
    num++;
    void *b = malloc(96); TEST_BREAK_LOC(malloc1);
    trace_printf("task[%p]: Sample print 0x%lx 0x%hx 0x%x %lu %hu %u %ld %hd %d %c\n",
        curr_task, num, num, num, num, num, num, num, num, num, 'A' + (num  % ('Z' - 'A')));
    num++;
    free(a); TEST_BREAK_LOC(free0);
    trace_printf("task[%p]: Sample print 0x%lx 0x%hx 0x%x %lu %hu %u %ld %hd %d %c\n",
        curr_task, num, num, num, num, num, num, num, num, num, 'A' + (num  % ('Z' - 'A')));
    num++;
    b = malloc(10); TEST_BREAK_LOC(malloc2);
    trace_printf("task[%p]: Sample print 0x%lx 0x%hx 0x%x %lu %hu %u %ld %hd %d %c\n",
        curr_task, num, num, num, num, num, num, num, num, num, 'A' + (num  % ('Z' - 'A')));
    num++;
    b = malloc(23); TEST_BREAK_LOC(malloc3);
    trace_printf("task[%p]: Sample print 0x%lx 0x%hx 0x%x %lu %hu %u %ld %hd %d %c\n",
        curr_task, num, num, num, num, num, num, num, num, num, 'A' + (num  % ('Z' - 'A')));
    num++;
    free(b); TEST_BREAK_LOC(free1);
    trace_printf("task[%p]: Sample print 0x%lx 0x%hx 0x%x %lu %hu %u %ld %hd %d %c\n",
        curr_task, num, num, num, num, num, num, num, num, num, 'A' + (num  % ('Z' - 'A')));
    __asm__ volatile (
        ".global _do_trace_test_heap_log_end\n" \
        ".type   _do_trace_test_heap_log_end,@function\n" \
        "_do_trace_test_heap_log_end:\n" \
        "   nop\n" \
        :::);
}

static void trace_test_heap_log_main(trace_test_task_arg_t *arg)
{
    static uint32_t num = 0;

    if(heap_trace_init_tohost() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init heap trace!");
        return;
    }
    heap_trace_start(HEAP_TRACE_LEAKS);
#if !CONFIG_FREERTOS_UNICORE
    xTaskNotify(arg->other_task, 0, eNoAction);
#endif

    do_trace_test_heap_log(num, arg->trace_printf); TEST_BREAK_LOC(trace_test_func_call);
    num += 10;

#if !CONFIG_FREERTOS_UNICORE
    xTaskNotifyWait(0, 0, NULL, portMAX_DELAY);
#endif
    // does trace data flush implicitly
    heap_trace_stop();
}

#if !CONFIG_FREERTOS_UNICORE
static void trace_test_heap_log_slave(trace_test_task_arg_t *arg)
{
    static uint32_t num = 0;
    xTaskNotifyWait(0, 0, NULL, portMAX_DELAY);
    do_trace_test_heap_log(num, arg->trace_printf); TEST_BREAK_LOC(trace_test_func_call);
    num += 10;
    xTaskNotify(arg->other_task, 0, eNoAction);
}
#endif
#endif //CONFIG_HEAP_TRACING

static __attribute__((noinline)) void _trace_test_log_continuous_start(void)
{
    __asm__ volatile (
        "   nop\n" \
        :::);
}

static __attribute__((noinline)) void _trace_test_log_continuous_stop(void)
{
    __asm__ volatile (
        "   nop\n" \
        :::);
}

static __attribute__((noinline)) void do_trace_test_log_continuous(uint32_t num, trace_printf_t trace_printf)
{
    volatile TaskHandle_t curr_task = xTaskGetCurrentTaskHandle();

    for (int i = 0; i < 400; i++) {
        trace_printf("task[%p]: Sample print 0x%lx 0x%hx 0x%x %lu %hu %u %ld %hd %d %c\n",
            curr_task, num, num, num, num, num, num, num, num, num, 'A' + (num  % ('Z' - 'A')));
        num++;
        vTaskDelay(1);
    }
    __asm__ volatile (
        ".global _trace_test_log_continuous_end\n" \
        ".type   _trace_test_log_continuous_end,@function\n" \
        "_trace_test_log_continuous_end:\n" \
        "   nop\n" \
        :::);
}

static __attribute__((noinline)) void trace_test_log_continuous_main(trace_test_task_arg_t *arg)
{
    static uint32_t num = 0;

    _trace_test_log_continuous_start();
#if !CONFIG_FREERTOS_UNICORE
    xTaskNotify(arg->other_task, 0, eNoAction);
#endif
    while(!arg->trace_is_started()) {
        vTaskDelay(1);
    }

    do_trace_test_log_continuous(num, arg->trace_printf);
    num += 10;

#if !CONFIG_FREERTOS_UNICORE
    xTaskNotifyWait(0, 0, NULL, portMAX_DELAY);
#endif
    arg->trace_flush(ESP_APPTRACE_TMO_INFINITE);
    _trace_test_log_continuous_stop();
}

#if !CONFIG_FREERTOS_UNICORE
static void trace_test_log_continuous_slave(trace_test_task_arg_t *arg)
{
    static uint32_t num = 0;
    xTaskNotifyWait(0, 0, NULL, portMAX_DELAY);
    do_trace_test_log_continuous(num, arg->trace_printf);
    num += 10;
    xTaskNotify(arg->other_task, 0, eNoAction);
}
#endif

static void trace_test_task(void *pvParameter)
{
    trace_test_task_arg_t *arg = (trace_test_task_arg_t *)pvParameter;
#if !CONFIG_FREERTOS_UNICORE
    // wait until other task handle is known
    while(arg->other_task == NULL) {
        vTaskDelay(1);
    }
#endif
    arg->test_func(arg);
    while(1) {
        vTaskDelay(1);
    }
}

static void os_trace_test_task(void *pvParameter)
{
    int i = 0;
    struct os_trace_task_arg *arg = (struct os_trace_task_arg *)pvParameter;
    struct timer_task_arg tim_arg = {
        .tim_grp = arg->tim_grp, 
        .tim_id = arg->tim_id, 
        .tim_period = arg->tim_period, 
        .isr_func = os_trace_test_timer_isr
    };

    int res = test_timer_init(&tim_arg);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initilaze timer (%d)!", res);
        return;
    }

    while(1) {
        printf("task[%p] work %d\n", xTaskGetCurrentTaskHandle(), i++);
        vTaskDelay(arg->task_period / portTICK_PERIOD_MS);
    }
}

TEST_DECL(os_tracing, "test_sysview.SysView*TracingTests*.test_os_tracing")
{
    static struct os_trace_task_arg task_args[2] = {
        { .tim_grp = TEST_TIMER_GROUP_0, .tim_id = TEST_TIMER_0, .tim_period = 300000UL /*us*/, .task_period = 500 /*ms*/},
        { .tim_grp = TEST_TIMER_GROUP_1, .tim_id = TEST_TIMER_0, .tim_period = 500000UL /*us*/, .task_period = 2000 /*ms*/}
    };
    xTaskCreatePinnedToCore(os_trace_test_task, "trace_task0", 4096, (void *)&task_args[0], 5, NULL, 0);
#if !CONFIG_FREERTOS_UNICORE
    xTaskCreatePinnedToCore(os_trace_test_task, "trace_task1", 4096, (void *)&task_args[1], 5, NULL, 1);
#endif
}

#if CONFIG_APPTRACE_ENABLE

// Wrapper functions to handle ESP-IDF version differences
// In ESP-IDF v6.0.0+ , destination parameter was removed from esp_apptrace functions
static inline int esp_trace_write(const char* data, int size, uint32_t timeout_ms)
{
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(6, 0, 0)
    return esp_apptrace_write(data, size, timeout_ms);
#else
    return esp_apptrace_write(ESP_APPTRACE_DEST_JTAG, data, size, timeout_ms);
#endif
}

static inline esp_err_t esp_trace_flush(uint32_t timeout_ms)
{
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(6, 0, 0)
    return esp_apptrace_flush(timeout_ms);
#else
    return esp_apptrace_flush(ESP_APPTRACE_DEST_JTAG, timeout_ms);
#endif
}

static inline bool esp_trace_host_is_connected(void)
{
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(6, 0, 0)
    return esp_apptrace_host_is_connected();
#else
    return esp_apptrace_host_is_connected(ESP_APPTRACE_DEST_JTAG);
#endif
}

static int apptrace_writefn(void* cookie, const char* data, int size)
{
    int res = esp_trace_write(data, size, 1000);
    if (res != ESP_OK) {
        return 0;
    }

    /* OCD-831. Why is the below delay avoids data corruption on ESP32-S3? */
    vTaskDelay(1);

    /* this function may fail if host is busy and is not able to read data (flushed previously) within 1 ms  */
    esp_trace_flush(1000);
    return size;
}

static void raw_trace_log_done(void)
{
    __asm__ __volatile__("nop");
}

static void raw_trace_log(void* arg)
{
    uint32_t iter_count = (uint32_t)arg;
    stdout = fwopen(NULL, &apptrace_writefn);
    static char stdout_buf[128];
    setvbuf(stdout, stdout_buf, _IOLBF, sizeof(stdout_buf));

    for (int i = 0; i < iter_count; ++i) {
        printf("[%d %*.s]\n", i, i * 20, "test");
    }
    /* ensure that all data are gone to the host in case the last call to esp_apptrace_flush() from apptrace_writefn() failed */
    esp_trace_flush(ESP_APPTRACE_TMO_INFINITE);
    raw_trace_log_done();
    vTaskDelete(NULL);
}

TEST_DECL(apptrace_reset, "test_apptrace.ApptraceTests*.test_apptrace_reset")
{
    uint32_t delay = (uint32_t)pvParameter;
    stdout = fwopen(NULL, &apptrace_writefn);
    static char stdout_buf[128];
    setvbuf(stdout, stdout_buf, _IOLBF, sizeof(stdout_buf));

    while (!esp_trace_host_is_connected())
        vTaskDelay(1);

    int cnt = 0;
    while (1) {
        printf("apptrace test line#%d\n", cnt++);
        vTaskDelay(delay / portTICK_PERIOD_MS);
    }

}

TEST_DECL(apptrace_dest_tcp, "test_apptrace.ApptraceTests*.test_apptrace_dest_tcp")
{
    xTaskCreate(raw_trace_log, "raw_trace_log", 4096, (void *)10, 5, NULL);
}

TEST_DECL(apptrace_autostop, "test_apptrace.ApptraceTests*.test_apptrace_autostop")
{
    xTaskCreate(raw_trace_log, "raw_trace_log", 4096, (void *)100, 5, NULL);
}
#endif // CONFIG_APPTRACE_ENABLE

#if CONFIG_HEAP_TRACING
TEST_DECL(log_heap_tracing, "test_sysview.SysView*TracingTests*.test_heap_log_from_file")
{
    static trace_test_task_arg_t task_args[2];
    memset(task_args, 0, sizeof(task_args));
    task_args[0].test_func = (do_trace_test_t)trace_test_heap_log_main;
    task_args[0].trace_printf = do_trace_printf;
    task_args[0].trace_flush = do_trace_flush;
    task_args[0].trace_is_started = do_trace_is_started;
    xTaskCreatePinnedToCore(trace_test_task, "trace_task0", 4096, (void *)&task_args[0], 5, &task_args[1].other_task, 0);
#if !CONFIG_FREERTOS_UNICORE
    task_args[1].test_func = (do_trace_test_t)trace_test_heap_log_slave;
    task_args[1].trace_printf = do_trace_printf;
    task_args[1].trace_flush = do_trace_flush;
    task_args[1].trace_is_started = do_trace_is_started;
    xTaskCreatePinnedToCore(trace_test_task, "trace_task1", 4096, (void *)&task_args[1], 5, &task_args[0].other_task, 1);
#endif
}
#endif // CONFIG_HEAP_TRACING

TEST_DECL(log_continuous_tracing, "test_sysview.SysView*TracingTests*.test_log_from_file")
{
    static trace_test_task_arg_t task_args[2];
    memset(task_args, 0, sizeof(task_args));
    task_args[0].test_func = (do_trace_test_t)trace_test_log_continuous_main;
    task_args[0].trace_printf = do_trace_printf;
    task_args[0].trace_flush = do_trace_flush;
    task_args[0].trace_is_started = do_trace_is_started;
    xTaskCreatePinnedToCore(trace_test_task, "trace_task0", 4096, (void *)&task_args[0], 5, &task_args[1].other_task, 0);
#if !CONFIG_FREERTOS_UNICORE
    task_args[1].test_func = (do_trace_test_t)trace_test_log_continuous_slave;
    task_args[1].trace_printf = do_trace_printf;
    task_args[1].trace_flush = do_trace_flush;
    task_args[1].trace_is_started = do_trace_is_started;
    xTaskCreatePinnedToCore(trace_test_task, "trace_task1", 4096, (void *)&task_args[1], 5, &task_args[0].other_task, 1);
#endif
}

ut_result_t tracing_test_do(int test_num, int core_num)
{
    if (TEST_ID_MATCH(TEST_ID_PATTERN(os_tracing), test_num)) {
        TEST_ENTRY(os_tracing)(NULL);
    } else if (TEST_ID_MATCH(TEST_ID_PATTERN(log_continuous_tracing), test_num)) {
        TEST_ENTRY(log_continuous_tracing)(NULL);
#if CONFIG_HEAP_TRACING
    } else if (TEST_ID_MATCH(TEST_ID_PATTERN(log_heap_tracing), test_num)) {
        TEST_ENTRY(log_heap_tracing)(NULL);
#endif //CONFIG_HEAP_TRACING
#if CONFIG_APPTRACE_ENABLE
    } else if (TEST_ID_MATCH(TEST_ID_PATTERN(apptrace_dest_tcp), test_num)) {
        TEST_ENTRY(apptrace_dest_tcp)(NULL);
    } else if (TEST_ID_MATCH(TEST_ID_PATTERN(apptrace_autostop), test_num)) {
        TEST_ENTRY(apptrace_autostop)(NULL);
    } else if (TEST_ID_MATCH(TEST_ID_PATTERN(apptrace_reset), test_num)) {
        xTaskCreate(TEST_ENTRY(apptrace_reset), "raw_trace_log_periodic", 4096, (void *)100, 5, NULL);
#endif // CONFIG_APPTRACE_ENABLE
    } else {
        return UT_UNSUPPORTED;
    }
    return UT_OK;
}
