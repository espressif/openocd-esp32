/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 */

#pragma once

#include <stdint.h>

extern void ets_printf(const char *fmt, ...);

void stub_lib_log_init(uint8_t uart_num, uint32_t baudrate);

#ifdef STUB_LOG_ENABLE

#define STUB_LIB_LOG(format, ...) ets_printf(format, ## __VA_ARGS__);

#else

#define STUB_LIB_LOG(format, ...) do {} while (0)

#endif
