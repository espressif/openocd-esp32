/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 */

#include <stdint.h>

#include <rtc_clk.h>

// These functions are defined in the ROM
extern void uartAttach(void);
extern void uart_div_modify(uint8_t uart_no, uint32_t DivLatchValue);

void stub_target_uart_init(uint8_t uart_num, uint32_t baudrate)
{
    uartAttach();
    uart_div_modify(uart_num, (stub_lib_rtc_clk_apb_freq_get() << 4) / baudrate);
}
