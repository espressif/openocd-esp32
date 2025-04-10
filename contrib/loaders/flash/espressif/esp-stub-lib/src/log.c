/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 */
#include <stdint.h>

#include <target/uart.h>

// These functions are defined in the ROM
extern void ets_install_uart_printf(void);

#ifdef STUB_LOG_ENABLE
void stub_lib_log_init(uint8_t uart_num, uint32_t baudrate)
{
    stub_target_uart_init(uart_num, baudrate);
    ets_install_uart_printf();
}
#else
void stub_lib_log_init(uint8_t uart_num, uint32_t baudrate)
{
    (void)uart_num;
    (void)baudrate;
}
#endif
