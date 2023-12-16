# SPDX-License-Identifier: GPL-2.0-or-later

add_library(riscv STATIC)

target_sources(riscv PRIVATE
	asm.h
	batch.h
	debug_defines.h
	debug_reg_printer.h
	encoding.h
	gdb_regs.h
	opcodes.h
	program.h
	riscv.h
	batch.c
	program.c
	riscv-011.c
	riscv-013.c
	riscv.c
	riscv_semihosting.c
	debug_defines.c
	debug_reg_printer.c
)
