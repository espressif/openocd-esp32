/***************************************************************************
 *   Xtensa flasher stub wrapper                                           *
 *   Copyright (C) 2017 Espressif Systems Ltd.                             *
 *   Author: Alexey Gerenkov, alexey@espressif.com                         *
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
 * xtensa-xtensa-elf-gcc -c xtensa_stub_tramp_win.S -o xtensa_stub_tramp_win.o
 *
 * To disassemble:
 * xtensa-xtensa-elf-objdump -d xtensa_stub_tramp_win.o
 *
 * To generate binary file:
 * xtensa-xtensa-elf-objcopy -O binary xtensa_stub_tramp_win.o xtensa_stub_tramp_win.bin
 *
 * To generate include file:
 * cat xtensa_stub_tramp_win.bin | xxd -i > xtensa_stub_tramp_win.inc
 */

/*
 * Expects :
 * a0 = zero
 * a1 = stack_base + stack_size - 16, 16 bytes aligned
 * a8 = address of the function to call
 * Params :
 * a2 = command arg0, result (out)
 * a3 = command arg1
 * a4 = command arg2
 * a5 = command arg3
 * a6 = command arg4
 * Maximum 5 user args
 */
    .text

    .align  4
_stub_enter:
    /* initialize initial stack frame for callx8 */
    addi    a9, sp,  32 /* point 16 past extra save area */
    s32e    a9, sp, -12 /* access to extra save area */
    /* prepare args */
    mov     a10, a2
    mov     a11, a3
    mov     a12, a4
    mov     a13, a5
    mov     a14, a6
    /* call stub */
    callx8  a8
    /* prepare return value */
    mov     a2, a10
    break 0,0

_idle_loop:
    j _idle_loop
