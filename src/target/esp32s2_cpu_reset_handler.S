/***************************************************************************
 *   Reset stub used by esp32s2 target                                       *
 *   Copyright (C) 2019 Espressif Systems Ltd.                             *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.           *
 ***************************************************************************/


#define RTC_CNTL_RESET_STATE_REG    0x3f408038
#define RTC_CNTL_RESET_STATE_DEF    0x3000
#define RTC_CNTL_CLK_CONF_REG       0x3f408074
#define RTC_CNTL_CLK_CONF_DEF       0x1583218
#define RTC_CNTL_STORE4_REG         0x3f4080c0
#define RTC_CNTL_STORE5_REG         0x3f4080c4
#define WDT_WKEY_VALUE              0x50D83AA1
#define TIMG0_WDTWPROTECT_REG       0x3F41F064
#define TIMG0_WDTCONFIG0_REG        0x3F41F048
#define TIMG1_WDTWPROTECT_REG       0x3F420064
#define TIMG1_WDTCONFIG0_REG        0x3F420048
#define RTC_CNTL_WDTCONFIG0_REG     0x3F408094
#define RTC_CNTL_WDTWPROTECT_REG    0x3F4080ac
#define RTC_CNTL_OPTIONS0_REG       0x3F408000
#define RTC_CNTL_OPTIONS0_DEF       0x1C492000
#define RTC_CNTL_SW_SYS_RST         0x80000000
#define DPORT_PMS_OCCUPY_3_REG      0x3F4C10E0


/* This stub is copied to RTC_SLOW_MEM by OpenOCD, and the CPU starts executing
 * it instead of the ROM code (0x40000400). This stub disables watchdogs and
 * goes into a loop.
 * OpenOCD will then halt the target and perform CPU reset using OCD.
 */


/* Has to be at offset 0. This is the entry point of the CPU, once
 * RTC_CNTL_PROCPU_STAT_VECTOR_SEL is cleared.
 * CPU will come here after the system reset, triggered by RTC_CNTL_SW_SYS_RST.
 */
    .global     cpu_at_start_handler
    .type       cpu_at_start_handler,@function
    .align      4
cpu_at_start_handler:
    j start


/* Has to be at offset 4. Once the stub code has been uploaded into RTC Slow
 * memory, OpenOCD will set the PC to this address, and resume execution.
 * The stub will then jump to 'reset' label and perform the reset.
 */
    .global     cpu_reset_handler
    .type       cpu_reset_handler,@function
    .align      4
cpu_reset_handler:
    j reset

    .align 4
    .literal_position

    .align 4
reset:
    /* Use a5 as a zero register */
    xor a5, a5, a5
    /* Select static reset vector 0 (XCHAL_RESET_VECTOR0_VADDR, 0x50000000) */
    movi a4, RTC_CNTL_RESET_STATE_REG
    s32i a5, a4, 0
    /* Set some clock-related RTC registers to the default values */
    movi a4, RTC_CNTL_STORE4_REG
    s32i a5, a4, 0
    movi a4, RTC_CNTL_STORE5_REG
    s32i a5, a4, 0
    movi a4, RTC_CNTL_CLK_CONF_REG
    movi a3, RTC_CNTL_CLK_CONF_DEF
    s32i a3, a4, 0
    /* Reset the digital part of the chip (RTC controller doesn't get reset) */
    movi a3, (RTC_CNTL_OPTIONS0_DEF | RTC_CNTL_SW_SYS_RST)
    movi a4, RTC_CNTL_OPTIONS0_REG
    s32i a3, a4, 0
    /* Doesn't reach beyond this instruction */

    .align 4
start:
    /* Use a5 as a zero register */
    xor a5, a5, a5
    /* Disable the watchdogs */
    movi a3, WDT_WKEY_VALUE
    movi a4, RTC_CNTL_WDTWPROTECT_REG
    s32i.n a3, a4, 0
    movi a4, TIMG0_WDTWPROTECT_REG
    s32i.n a3, a4, 0
    movi a4, TIMG1_WDTWPROTECT_REG
    s32i.n a3, a4, 0
    movi a4, RTC_CNTL_WDTCONFIG0_REG
    s32i.n a5, a4, 0
    movi a4, TIMG0_WDTCONFIG0_REG
    s32i.n a5, a4, 0
    movi a4, TIMG1_WDTCONFIG0_REG
    s32i.n a5, a4, 0
    /* Disable trace memory mapping */
    movi a4, DPORT_PMS_OCCUPY_3_REG
    s32i.n a5, a4, 0
    /* Restore the reset vector to ROM */
    movi a4, RTC_CNTL_RESET_STATE_REG
    movi a3, RTC_CNTL_RESET_STATE_DEF
    s32i.n a3, a4, 0


parking_loop:
    /* CPU will be in this loop, until OpenOCD
     * finds the JTAG taps and puts the CPUs into debug mode.
     */
    waiti 0
    j parking_loop
