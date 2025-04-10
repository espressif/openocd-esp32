/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 */

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

// These functions are defined in the ROM
extern void uartAttach(void *rxBuffer);
extern void Uart_Init(uint8_t uart_no, uint32_t clock);
extern uint32_t ets_get_apb_freq(void);
extern void ets_update_cpu_frequency(uint32_t ticks_per_us);

#define  UART_CLK_FREQ_ROM     (40 * 1000000)

void stub_target_uart_init(uint8_t uart_num, uint32_t baudrate)
{
    (void)baudrate;
    extern bool g_uart_print;
    uartAttach(NULL);
    uint32_t clock = ets_get_apb_freq();
    ets_update_cpu_frequency(clock / 1000000);
    Uart_Init(uart_num, UART_CLK_FREQ_ROM);
    g_uart_print = true;
}
