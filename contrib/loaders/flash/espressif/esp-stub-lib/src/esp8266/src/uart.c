/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 */

#include <stdint.h>

void stub_target_uart_init(uint8_t uart_num, uint32_t baudrate)
{
    (void)uart_num;
    (void)baudrate;
    // TODO: implement
}
