/* SPDX-License-Identifier: Apache-2.0 OR MIT */
/* SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD */

#pragma once

#include <stdint.h>

typedef int (*stub_apptrace_recv_cb_t)(const uint8_t *buf, uint32_t size);
typedef int (*stub_apptrace_send_cb_t)(uint32_t addr, uint8_t *buf, uint32_t size);

/**
 * @brief Read trace data sent by host
 */
int stub_apptrace_recv_data(const void *arg1, stub_apptrace_recv_cb_t process_cb);

/**
 * @brief Write data to the trace buffer to be received by host
 */
int stub_apptrace_send_data(uint32_t addr, uint32_t size, stub_apptrace_send_cb_t process_cb);
