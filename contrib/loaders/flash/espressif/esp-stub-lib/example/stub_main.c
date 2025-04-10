/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 */

#include <stdint.h>
#include <stdarg.h>
#include <stddef.h>

#include <esp-stub-lib/log.h>
#include <esp-stub-lib/flash.h>

#include "stub_main.h"

extern uint32_t _bss_start;
extern uint32_t _bss_end;

struct stub_cmd_handler {
    int cmd;
    const char *name;
    int (*handler)(va_list ap);
};

static  __attribute__((unused)) int handle_test1(va_list ap)
{
    (void)ap;

    STUB_LIB_LOG("test1\n");

    return 0;
}

static  __attribute__((unused)) int handle_test2(va_list ap)
{
    (void)ap;

    STUB_LIB_LOG("test2\n");

    return 0;
}

static const struct stub_cmd_handler cmd_handlers[] = {
    {ESP_STUB_CMD_TEST1, "CMD_TEST1", handle_test1},
    {ESP_STUB_CMD_TEST2, "CMD_TEST2", handle_test2},
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

    stub_lib_log_init(0, 115200);

    stub_lib_flash_init(&flash_state);

    const struct stub_cmd_handler *handler = cmd_handlers;
    while (handler->handler) {
        if (handler->cmd == cmd) {
            STUB_LIB_LOG("Executing command: %s\n", handler->name);
            ret = handler->handler(ap);
            break;
        }
        handler++;
    }

    if (!handler->handler) {
        STUB_LIB_LOG("Unknown command!\n");
    }

    va_end(ap);

    if (flash_state) {
        stub_lib_flash_deinit(flash_state);
    }

    return ret;
}
