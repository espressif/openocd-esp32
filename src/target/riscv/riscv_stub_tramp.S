/***************************************************************************
 *   RISCV flasher stub wrapper                                            *
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

/* To assemble:
 * riscv32-esp-elf-gcc -c riscv_stub_tramp.S -o riscv_stub_tramp.o
 *
 * To disassemble:
 * riscv32-esp-elf-objdump -d riscv_stub_tramp.o
 *
 * To generate binary file:
 * riscv32-esp-elf-objcopy -O binary riscv_stub_tramp.o riscv_stub_tramp.bin
 *
 * To generate include file:
 * cat riscv_stub_tramp.bin | xxd -i > riscv_stub_tramp.inc
 */

/*
 * Expects :
 * ra = zero
 * sp = stack_base + stack_size - 16, 16 bytes aligned
 * a7 = address of the function to call
 * Params :
 * a0 = command arg0, result (out)
 * a1..a6 = other command args
 */
    .text

    /*.balign  4*/
_stub_enter:
    /* call stub */
    jalr a7
    ebreak
