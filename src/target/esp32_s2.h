/***************************************************************************
 *   ESP32-S2 target for OpenOCD                                           *
 *   Copyright (C) 2019 Espressif Systems Ltd.                             *
 *   Author: Alexey Gerenkov <alexey@espressif.com>                        *
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

#ifndef XTENSA_ESP32_S2_H
#define XTENSA_ESP32_S2_H

#include "esp_xtensa.h"

#define ESP32_S2_DROM_LOW    0x3F000000
#define ESP32_S2_DROM_HIGH   0x3F400000
#define ESP32_S2_IROM_LOW    0x40080000
#define ESP32_S2_IROM_HIGH   0x40c00000

/*Number of registers returned directly by the G command
 *Corresponds to the amount of regs listed in regformats/reg-xtensa.dat in the gdb source */
#define ESP32_S2_NUM_REGS_G_COMMAND   72

enum esp32_s2_rev {
	ESP32_S2_REV_UNKNOWN = -1,
	ESP32_S2_REV_BETA,
	ESP32_S2_REV_0,
	ESP32_S2_REV_LATEST = ESP32_S2_REV_0
};

struct esp32_s2_common {
	struct esp_xtensa_common esp_xtensa;
	enum esp32_s2_rev chip_rev;
};

static inline struct esp32_s2_common *target_to_esp32_s2(struct target *target)
{
	return container_of(target->arch_info, struct esp32_s2_common, esp_xtensa);
}

#endif	/* XTENSA_ESP32_S2_H */
