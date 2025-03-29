/* SPDX-License-Identifier: Apache-2.0 OR MIT */
/* SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD */

/*
 * Minimal Xtensa vector table for stub exception handling.
 *
 * OpenOCD sets VECBASE to stub_trap_entry so that any user-mode
 * exception (illegal instruction, load/store error, etc.) is caught
 * and recorded without depending on ROM/XTOS _xtos_set_exception_handler.
 *
 * Window overflow/underflow vectors are required for windowed ABI to work.
 * All other exception/interrupt vectors halt the CPU with break.
 */

#include "esp_stub.h"

/*
 * Vector table — must be 1024-byte aligned (VECBASE requirement).
 * Only simple instructions (no literals) inside the table to avoid
 * literal pool contamination from -mtext-section-literals.
 */
	.section .text.stub_vectors, "ax"
	.balign 1024

	.global stub_trap_entry
	.type   stub_trap_entry, @function
stub_trap_entry:

/* ===== 0x000: WindowOverflow4 ===== */
	.org stub_trap_entry + 0x000
	s32e    a0, a5, -16
	s32e    a1, a5, -12
	s32e    a2, a5, -8
	s32e    a3, a5, -4
	rfwo

/* ===== 0x040: WindowUnderflow4 ===== */
	.org stub_trap_entry + 0x040
	l32e    a0, a5, -16
	l32e    a1, a5, -12
	l32e    a2, a5, -8
	l32e    a3, a5, -4
	rfwu

/* ===== 0x080: WindowOverflow8 ===== */
	.org stub_trap_entry + 0x080
	s32e    a0, a9, -16
	l32e    a0, a1, -12
	s32e    a1, a9, -12
	s32e    a2, a9, -8
	s32e    a3, a9, -4
	s32e    a4, a0, -32
	s32e    a5, a0, -28
	s32e    a6, a0, -24
	s32e    a7, a0, -20
	rfwo

/* ===== 0x0C0: WindowUnderflow8 ===== */
	.org stub_trap_entry + 0x0C0
	l32e    a0, a9, -16
	l32e    a1, a9, -12
	l32e    a2, a9, -8
	l32e    a7, a1, -12
	l32e    a3, a9, -4
	l32e    a4, a7, -32
	l32e    a5, a7, -28
	l32e    a6, a7, -24
	l32e    a7, a7, -20
	rfwu

/* ===== 0x100: WindowOverflow12 ===== */
	.org stub_trap_entry + 0x100
	s32e    a0,  a13, -16
	l32e    a0,  a1,  -12
	s32e    a1,  a13, -12
	s32e    a2,  a13, -8
	s32e    a3,  a13, -4
	s32e    a4,  a0,  -48
	s32e    a5,  a0,  -44
	s32e    a6,  a0,  -40
	s32e    a7,  a0,  -36
	s32e    a8,  a0,  -32
	s32e    a9,  a0,  -28
	s32e    a10, a0,  -24
	s32e    a11, a0,  -20
	rfwo

/* ===== 0x140: WindowUnderflow12 ===== */
	.org stub_trap_entry + 0x140
	l32e    a0,  a13, -16
	l32e    a1,  a13, -12
	l32e    a2,  a13, -8
	l32e    a11, a1,  -12
	l32e    a3,  a13, -4
	l32e    a4,  a11, -48
	l32e    a5,  a11, -44
	l32e    a6,  a11, -40
	l32e    a7,  a11, -36
	l32e    a8,  a11, -32
	l32e    a9,  a11, -28
	l32e    a10, a11, -24
	l32e    a11, a11, -20
	rfwu

/*
 * 0x180-0x2C0: Level 2-5 interrupt, debug exception, and NMI vectors.
 * Not expected: the stub runs with interrupts masked (PS.INTLEVEL=5).
 */
	.irp off, 0x180, 0x1C0, 0x200, 0x240, 0x280, 0x2C0
	.org stub_trap_entry + \off
	break   1, 0
	j       .
	.endr

/*
 * 0x300 (Kernel) and 0x3C0 (Double) are not expected:
 * the stub runs with PS.UM=1, so exceptions go to 0x340.
 */
	.org stub_trap_entry + 0x300
	break   1, 0
	j       .

/* ===== 0x340: UserExceptionVector ===== */
	.org stub_trap_entry + 0x340
	wsr     a0, excsave1
	call0   stub_trap_handler

	.org stub_trap_entry + 0x3C0
	break   1, 0
	j       .

/*
 * Exception handler
 * Fills the trap record from special registers, sets a2, and halts.
 * Does NOT return — all registers except a2 may be clobbered.
 */
	.section .text.stub_trap_handler, "ax"
	.balign 4
	.global stub_trap_handler
	.type   stub_trap_handler, @function
stub_trap_handler:
	movi    a0, g_stub_trap_record

	/* Save original SP (offset 28) */
	s32i    a1, a0, 28

	/* Save original a0 (offset 32) */
	rsr     a1, excsave1
	s32i    a1, a0, 32

	/* exccause → mcause (offset 12) */
	rsr     a1, exccause
	s32i    a1, a0, 12

	/* epc1 → mepc (offset 16) */
	rsr     a1, epc1
	s32i    a1, a0, 16

	/* excvaddr → mtval (offset 20) */
	rsr     a1, excvaddr
	s32i    a1, a0, 20

	/* ps → mstatus (offset 24) */
	rsr     a1, ps
	s32i    a1, a0, 24

	/* hartid = 0 (offset 36) */
	movi    a1, 0
	s32i    a1, a0, 36

	/* version (offset 4) */
	movi    a1, ESP_STUB_TRAP_RECORD_VERSION
	s32i    a1, a0, 4

	/* flags (offset 8) */
	movi    a1, ESP_STUB_TRAP_RECORD_XTENSA
	s32i    a1, a0, 8

	/* magic (offset 0) */
	movi    a1, ESP_STUB_TRAP_RECORD_MAGIC
	s32i    a1, a0, 0

	/* Fence stores, set return code, halt */
	movi    a2, ESP_STUB_ERR_EXCEPTION
	memw
	break   1, 0
1:	j       1b
	.size stub_trap_handler, .-stub_trap_handler
