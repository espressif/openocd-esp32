/***************************************************************************
 *   ESP32 flasher stub wrapper                                           *
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

/*
 * Params :
 * a2 = command ID, result (out)
 * a3 = command arg1: flash addr, start sector
 * a4 = command arg2: size, nunmber of sectors
 * a5 = command arg3: down buffer start, sectors state buf address
 * a6 = command arg3: down buffer size
 */
    .text

    .align  4
_stub_enter:
    mov     a10, a2
    mov     a11, a3
    mov     a12, a4
    mov     a13, a5
    mov     a14, a6
    callx8  a0
    mov     a2, a10
    break 0,0
