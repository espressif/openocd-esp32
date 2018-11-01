/***************************************************************************
 *   ESP32 test app for OpenOCD                                            *
 *   Copyright (C) 2018 Espressif Systems Ltd.                             *
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

#ifndef GEN_UT_APP_H
#define GEN_UT_APP_H

#define TEST_BREAK_LOC(_nm_)  \
    volatile static const int _nm_ ## _break_ln = __LINE__; \
    s_tmp_ln = _nm_ ## _break_ln;

// used to prevent linker from optimizing out the variables holding BP line numbers
volatile static int s_tmp_ln = 0;

#endif //GEN_UT_APP_H