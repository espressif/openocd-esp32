/* SPDX-License-Identifier: Apache-2.0 OR MIT */
/* SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD */

#pragma once

#ifdef __XTENSA__

#include <stdint.h>

#define XT_INS_BREAK   0x004000
#define XT_INS_BREAKN  0xF02D

static inline __attribute__((always_inline)) uint8_t stub_get_insn_size(const uint8_t *insn)
{
	return insn[0] & 0x8 ? 2 : 3;
}

static inline __attribute__((always_inline)) uint32_t stub_get_break_insn(uint8_t insn_sz)
{
	return insn_sz == 2 ? XT_INS_BREAKN : XT_INS_BREAK;
}

static inline __attribute__((always_inline)) uint32_t stub_get_max_insn_size(void)
{
	return 3;
}

static inline __attribute__((always_inline, noreturn)) void stub_abort(void)
{
	asm("ill");
	__builtin_unreachable();
}

#endif /* __XTENSA__ */
