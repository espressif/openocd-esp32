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

#include "xtensa_algorithm.h"
#include "esp_xtensa.h"

#define ESP32_DROM_LOW             0x3F400000
#define ESP32_DROM_HIGH            0x3F800000
#define ESP32_IROM_LOW             0x400D0000
#define ESP32_IROM_HIGH            0x40400000

/*Number of registers returned directly by the G command
 *Corresponds to the amount of regs listed in regformats/reg-xtensa.dat in the gdb source */
#define ESP32_NUM_REGS_G_COMMAND   105

/* 0 - don't care, 1 - TMS low, 2 - TMS high */
enum esp32_flash_bootstrap {
	FBS_DONTCARE = 0,
	FBS_TMSLOW,
	FBS_TMSHIGH,
};

struct esp32_common {
	struct esp_xtensa_common esp_xtensa;
	enum esp32_flash_bootstrap flash_bootstrap;
	bool other_core_does_resume;
};

static inline struct esp32_common *target_to_esp32(struct target *target)
{
	return container_of(target->arch_info, struct esp32_common, esp_xtensa);
}

int esp32_run_func_image(struct target *target,
	struct xtensa_algo_run_data *run,
	struct xtensa_algo_image *image,
	uint32_t num_args,
	...);

#endif	/* XTENSA_ESP32_H */
