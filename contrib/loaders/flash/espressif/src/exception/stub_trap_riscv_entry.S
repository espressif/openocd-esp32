/* SPDX-License-Identifier: Apache-2.0 OR MIT */
/* SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD */

	.section .text.stub_trap_entry, "ax", @progbits
	/* Only 256-byte aligned addresses are allowed for mtvec */
	.balign 256
	.global stub_trap_entry
	.type stub_trap_entry, @function
stub_trap_entry:
	mv a0, sp
	mv a1, ra
	call stub_trap_handler_c
	.size stub_trap_entry, .-stub_trap_entry
