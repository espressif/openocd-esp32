/***************************************************************************
 *   ESP32 target for OpenOCD                                              *
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

#ifndef XTENSA_ESP32_H
#define XTENSA_ESP32_H

#include "esp108_common.h"

#define ESP32_CPU_COUNT		2
#define ESP32_PRO_CPU_ID	0

enum esp32_isrmasking_mode {
	ESP32_ISRMASK_OFF,
	ESP32_ISRMASK_ON,
};


struct esp32_common {

	// Common fields definition for all esp108 targets
	ESP108_COMMON_FIELDS;

	struct target* esp32_targets[ESP32_CPU_COUNT];
	int active_cpu;
	struct reg_cache *core_caches[ESP32_CPU_COUNT];
	int64_t current_threadid;
	enum esp32_isrmasking_mode isrmasking_mode;
};


#endif /* XTENSA_ESP32_H */
