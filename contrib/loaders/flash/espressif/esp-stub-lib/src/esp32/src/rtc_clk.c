/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 */

#include <stdint.h>

#include "reg_base.h"

// TODO: Add related header files for each target
#define RTC_APB_FREQ_REG     (DR_REG_RTCCNTL_BASE + 0xB4)
#define READ_PERI_REG(addr)  (*((volatile uint32_t *)(addr)))

uint32_t stub_lib_rtc_clk_apb_freq_get(void)
{
    uint32_t apb_freq_hz = (READ_PERI_REG(RTC_APB_FREQ_REG) & UINT16_MAX) << 12;
    // Round to the nearest MHz
    apb_freq_hz += 1000000 / 2;
    uint32_t remainder = apb_freq_hz % 1000000;
    return apb_freq_hz - remainder;
}
