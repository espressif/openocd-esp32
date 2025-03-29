/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 */

#pragma once

#include <esp-stub-lib/log.h>

#ifdef STUB_LOG_ENABLED
#define STUB_LOG_INIT(uart_num, baudrate) stub_lib_log_init(uart_num, baudrate)
#define STUB_LOG(fmt, ...) stub_lib_log_printf(fmt, ##__VA_ARGS__)
#else
#define STUB_LOG_INIT(uart_num, baudrate)
#define STUB_LOG(fmt, ...)
#endif
