/***************************************************************************
 *   Reset stub used by esp108/esp32 targets                               *
 *   Copyright (C) 2017 Espressif Systems Ltd.                             *
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

/* This stub is copied to RTC_SLOW_MEM by OpenOCD, and the CPU starts executing
 * it instead of the ROM code (0x40000400). This stub disables watchdogs and
 * goes into a loop.
 * OpenOCD will then halt the target and perform CPU reset using OCD.
 */

    .global     cpu_at_start_handler
    .type       cpu_at_start_handler,@function
    .align      4

cpu_at_start_handler:

	j st

	.align 4
.Lwdt_key:
	.word 0x50d83aa1
.Lwdt_conf_val:
	.word 0x0
.Lrtc_wdt_protect:
	.word 0x3ff480a4
.Lrtc_wdt_conf:
	.word 0x3ff4808c
.Ltg0_wdt_protect:
	.word 0x3ff5f064
.Ltg0_wdt_conf:
	.word 0x3ff5f048
.Ltg1_wdt_protect:
	.word 0x3FF60064
.Ltg1_wdt_conf:
	.word 0x3ff60048

st:
	l32r a3, .Lwdt_key
	l32r a4, .Lrtc_wdt_protect
	s32i.n a3, a4, 0
	l32r a4, .Ltg0_wdt_protect
	s32i.n a3, a4, 0
	l32r a4, .Ltg1_wdt_protect
	s32i.n a3, a4, 0
	l32r a3, .Lwdt_conf_val
	l32r a4, .Lrtc_wdt_conf
	s32i.n a3, a4, 0
	l32r a4, .Ltg0_wdt_conf
	s32i.n a3, a4, 0
	l32r a4, .Ltg1_wdt_conf
	s32i.n a3, a4, 0
wait:
	waiti 0
	j wait
