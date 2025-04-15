/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 */

#include <stdint.h>
#include <stdarg.h>
#include <stddef.h>

#include <esp-stub-lib/flash.h>
#include <esp-stub-lib/log.h>

#include "esp_stub.h"

extern uint32_t _bss_start;
extern uint32_t _bss_end;

#define __maybe_unused __attribute__((unused))

const struct esp_stub_desc __attribute__((section(".stub_desc")))  s_esp_stub_desc = {
	.magic = ESP_STUB_FLASHER_MAGIC_NUM,
	.stub_version = ESP_STUB_FLASHER_VERSION,
	.idf_key = 0x00,
};

struct stub_cmd_handler {
	int cmd;
	const char *name;
	int (*handler)(va_list ap);
};

static __maybe_unused int handle_test1(va_list ap)
{
    (void)ap;

    uint32_t __maybe_unused addr = 0x4080393C;

    STUB_LOG("stub command test:%d\n", 1);
    STUB_LOG("stub command test:%d\n", -1);
    STUB_LOG("stub command test:0x%x\n", addr);
    STUB_LOG("stub command test:%s\n", "test");
    STUB_LOG("stub command test:%c\n", 'A');
    STUB_LOG("stub command test:%l\n", 'A');

    STUB_LOGE("stub command test\n");
    STUB_LOGW("stub command test\n");
    STUB_LOGI("stub command test\n");
    STUB_LOGD("stub command test\n");
    STUB_LOGV("stub command test\n");
    STUB_LOG_TRACE();
    STUB_LOG_TRACEF("foo:%u\n", 0x2A);

    return 0;
}

static __maybe_unused int handle_test2(va_list ap)
{
    (void)ap;

    STUB_LOG("stub command test2\n");

    return 0;
}

static const struct stub_cmd_handler cmd_handlers[] = {
    #ifdef STUB_CMD_TEST1_ENABLED
    {ESP_STUB_CMD_TEST1, "CMD_TEST1", handle_test1},
    #endif
    #ifdef STUB_CMD_TEST2_ENABLED
    {ESP_STUB_CMD_TEST2, "CMD_TEST2", handle_test2},
    #endif
    {0, NULL, NULL}
};

int stub_main(int cmd, ...) __attribute__((used));

#ifdef ESP8266
__asm__(
    ".global stub_main_esp8266\n"
    ".literal_position\n"
    ".align 4\n"
    "stub_main_esp8266:\n"
    "movi a0, 0x400010a8;"
    "j stub_main;");
#endif

int stub_main(int cmd, ...)
{
    va_list ap;
    void *flash_state = NULL;
    int ret = -1;

    /* zero bss */
    for (uint32_t *p = &_bss_start; p < &_bss_end; p++) {
        *p = 0;
    }

    va_start(ap, cmd);

    STUB_LOG_INIT();

    stub_lib_flash_init(&flash_state);

    STUB_LOG("Command: %x\n", cmd);

    const struct stub_cmd_handler *handler = cmd_handlers;
    while (handler->handler) {
        if (handler->cmd == cmd) {
            STUB_LOG("Executing command: %s\n", handler->name);
            ret = handler->handler(ap);
            break;
        }
        handler++;
    }

    if (!handler->handler) {
        STUB_LOG("Unknown command!\n");
    }

    va_end(ap);

    if (flash_state) {
        stub_lib_flash_deinit(flash_state);
    }

    return ret;
}
