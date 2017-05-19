/***************************************************************************
*   Copyright (C) 2017 Espressif Systems Ltd.                             *
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

#ifndef XTENSA_H
#define XTENSA_H

#include <jtag/jtag.h>
#include "breakpoints.h"

enum xtensa_state {
	XT_NORMAL,
	XT_OCD_RUN,
	XT_OCD_HALT,
};


enum FlashBootstrap {
	FBS_DONTCARE = 0,
	FBS_TMSLOW,
	FBS_TMSHIGH,
};
// Local data for debug!!!! DYA
#define ESP32_CPU_COUNT		2	
#define ESP32_MAIN_ID		0
#define ESP32_APP_CPU_ID	0
struct target;

struct esp32_common {
//	struct jtag_tap *tap;
	enum xtensa_state state;
	struct reg_cache *core_cache;
	struct target *target;
	uint8_t prevpwrstat;
	int resetAsserted;
	int traceActive;

	uint32_t num_brps; /* Number of breakpoints available */
	struct breakpoint **hw_brps;
	uint32_t num_wps; /* Number of watchpoints available */
	struct watchpoint **hw_wps;

	enum FlashBootstrap flashBootstrap; /* 0 - don't care, 1 - TMS low, 2 - TMS high */

	struct target* esp32_targets[ESP32_CPU_COUNT];
	int active_cpu;
	struct reg_cache *core_caches[ESP32_CPU_COUNT];
	int64_t current_threadid;
};


#endif /* XTENSA_H */
