/***************************************************************************
 *   Copyright (C) 2010 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
 *                                                                         *
 *   Copyright (C) 2011 Øyvind Harboe                                      *
 *   oyvind.harboe@zylin.com                                               *
 *                                                                         *
 *   Copyright (C) 2015 Uwe Bonnes                                         *
 *   bon@elektron.ikp.physik.tu-darmstadt.de                               *
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
 *   Free Software Foundation, Inc.                                        *
 ***************************************************************************/

/*
 * Params :
 * a2 = count, result (out)
 * a3 = command arg1: flash addr
 * a4 = command arg2: size
 * a5 = command arg3: down buffer start
 * a6 = command arg3: down buffer size
 */
    .text

/*
    .globl  _stub_func
    .type   _stub_func,@function
    .align  4
_stub_func:
    entry   a1, 32
    movi.n  a2, -10
    retw
*/

/*
    .globl  _stub_func2
    .type   _stub_func2,@function
    .align  4
_stub_func2:
    movi.n  a2, -12
    ret
*/

    .align  4
_stub_enter:
    mov     a10, a2
    mov     a11, a3
    mov     a12, a4
    mov     a13, a5
    mov     a14, a6
    callx8  a0
/*    call0 _stub_func2
    call8 _stub_func*/
    mov     a2, a10
    break 0,0
