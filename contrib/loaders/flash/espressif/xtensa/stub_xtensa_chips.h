/***************************************************************************
 *   Xtensa chips common definitions for flasher stub                      *
 *   Copyright (C) 2021 Espressif Systems Ltd.                             *
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
#ifndef ESP_XTENSA_CHIPS_FLASHER_STUB_H
#define ESP_XTENSA_CHIPS_FLASHER_STUB_H

#include "eri.h"
#include "esp_private/trax.h"
#include "xtensa/hal.h"
#include "stub_flasher.h"

#define ESP_APPTRACE_TRAX_BLOCK_SIZE    (0x4000UL)
#define ESP_APPTRACE_USR_DATA_LEN_MAX   (ESP_APPTRACE_TRAX_BLOCK_SIZE - 2)

#define ESP_APPTRACE_TRAX_CTRL_REG      ERI_TRAX_DELAYCNT
#define ESP_APPTRACE_TRAX_HOST_CONNECT  (1 << 23)

#define XT_INS_BREAK    0x004000
#define XT_INS_BREAKN   0xF02D

#define CPUTICKS2US(_t_)      ((_t_) / (stub_esp_clk_cpu_freq() / 1000000))

inline int stub_apptrace_prepare(void)
{
	/* imply that host is auto-connected */
	uint32_t reg = eri_read(ESP_APPTRACE_TRAX_CTRL_REG);
	reg |= ESP_APPTRACE_TRAX_HOST_CONNECT;
	eri_write(ESP_APPTRACE_TRAX_CTRL_REG, reg);

	return ESP_STUB_ERR_OK;
}

inline uint64_t stub_get_time(void)
{
	uint32_t ticks = xthal_get_ccount();
	return CPUTICKS2US(ticks);
}

inline int64_t esp_timer_get_time(void)
{
	return (int64_t)stub_get_time();
}

static inline uint8_t stub_get_insn_size(uint8_t *insn)
{
	return insn[0] & 0x8 ? 2 : 3;
}

static inline uint32_t stub_get_break_insn(uint8_t insn_sz)
{
	return insn_sz == 2 ? XT_INS_BREAKN : XT_INS_BREAK;
}

static inline uint32_t stub_get_coreid()
{
	int id;
	__asm__ volatile (
		"rsr.prid %0\n"
		" extui %0,%0,13,1"
		: "=r" (id));
	return id;
}

#endif	/*ESP_XTENSA_CHIPS_FLASHER_STUB_H */
