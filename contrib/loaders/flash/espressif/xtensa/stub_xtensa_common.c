// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   Xtensa specific flasher stub functions                                 *
 *   Copyright (C) 2023 Espressif Systems Ltd.                             *
 ***************************************************************************/
#include <eri.h>
#include <esp_private/trax.h>

#include <stub_flasher_int.h>
#include <stub_flasher.h>
#include "stub_xtensa_common.h"

#define ESP_APPTRACE_TRAX_CTRL_REG      ERI_TRAX_DELAYCNT
#define ESP_APPTRACE_TRAX_HOST_CONNECT  BIT(23)

#define XT_INS_BREAK                    0x004000
#define XT_INS_BREAKN                   0xF02D

#define CPUTICKS2US(_t_)                ((_t_) / (stub_esp_clk_cpu_freq() / 1000000))

void vPortEnterCritical(void *mux)
{
}

void vPortExitCritical(void *mux)
{
}

int stub_apptrace_prepare(void)
{
	/* imply that host is auto-connected */
	uint32_t reg = eri_read(ESP_APPTRACE_TRAX_CTRL_REG);
	reg |= ESP_APPTRACE_TRAX_HOST_CONNECT;
	eri_write(ESP_APPTRACE_TRAX_CTRL_REG, reg);

	return ESP_STUB_ERR_OK;
}

uint64_t stub_get_time(void)
{
	extern unsigned int xthal_get_ccount(void);
	uint32_t ticks = xthal_get_ccount();
	return CPUTICKS2US(ticks);
}

int64_t esp_timer_get_time(void)
{
	return (int64_t)stub_get_time();
}

uint8_t stub_get_insn_size(uint8_t *insn)
{
	return insn[0] & 0x8 ? 2 : 3;
}

uint8_t stub_get_max_insn_size(void)
{
	return 3;
}

uint32_t stub_get_break_insn(uint8_t insn_sz)
{
	return insn_sz == 2 ? XT_INS_BREAKN : XT_INS_BREAK;
}

uint32_t stub_get_coreid(void)
{
	int id;
	__asm__ volatile ("rsr.prid %0\n"
		" extui %0,%0,13,1"
		: "=r" (id));
	return id;
}
