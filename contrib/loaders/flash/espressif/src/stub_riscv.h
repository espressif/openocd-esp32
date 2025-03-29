/* SPDX-License-Identifier: Apache-2.0 OR MIT */
/* SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD */

#pragma once

#ifdef __riscv

#include <stdint.h>

#define RISCV_EBREAK   0x9002

static inline __attribute__((always_inline)) uint8_t stub_get_insn_size(const uint8_t *insn)
{
	(void)insn;

	return 2;
}

static inline __attribute__((always_inline)) uint32_t stub_get_break_insn(uint8_t insn_sz)
{
	(void)insn_sz;

	return RISCV_EBREAK;
}

static inline __attribute__((always_inline)) uint32_t stub_get_max_insn_size(void)
{
	return 2;
}

static inline __attribute__((always_inline, noreturn)) void stub_abort(void)
{
	__asm__ volatile ("unimp");
	__builtin_unreachable();
}

#endif /* __riscv */
