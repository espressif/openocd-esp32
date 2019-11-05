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

#include "ut_idf_ver.h"
#include "sdkconfig.h"

#define MAKE_UT_IDF_VER(t,x,y,z)    (((t & 0xFF) << 24) | ((x & 0xFF) << 16) | ((y & 0xFF) << 8) | (z & 0xFF))

#define UT_IDF_VER_LATEST           MAKE_UT_IDF_VER(0xFF,0xFF,0xFF,0xFF)
#define UT_IDF_VER 	                MAKE_UT_IDF_VER(UT_IDF_VER_MAJOR,UT_IDF_VER_MINOR,UT_IDF_VER_BUGFIX,UT_IDF_VER_UPDATE)

#define TEST_BREAK_LOC(_nm_)  \
    volatile static const int _nm_ ## _break_ln = __LINE__; \
    s_tmp_ln = _nm_ ## _break_ln;

// used to prevent linker from optimizing out the variables holding BP line numbers
volatile static int s_tmp_ln = 0;

#define LABEL_SYMBOL(name) __asm__ volatile(".global " name "\n.type " name ",@function\n" name":");

typedef enum {
	UT_OK,
	UT_FAIL,
	UT_UNSUPPORTED
} ut_result_t;

typedef ut_result_t (*test_func_t)(int test_num);

void test_timer_init(int timer_group, int timer_idx, uint32_t period);
void test_timer_rearm(int timer_group, int timer_idx);

/* Can run 'make menuconfig' to choose the GPIO to blink,
   or you can edit the following line and set a number here.
*/
#define BLINK_GPIO CONFIG_BLINK_GPIO

#endif //GEN_UT_APP_H