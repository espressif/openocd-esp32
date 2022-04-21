/***************************************************************************
 *   ESP32-C3 target for OpenOCD                                           *
 *   Copyright (C) 2020 Espressif Systems Ltd.                             *
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
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>. *
 ***************************************************************************/

#ifndef OPENOCD_TARGET_ESP32C3_H
#define OPENOCD_TARGET_ESP32C3_H

#include "esp_riscv.h"

#define ESP32C3_DROM_LOW    0x3C000000
#define ESP32C3_DROM_HIGH   0x3C800000
#define ESP32C3_IROM_LOW    0x42000000
#define ESP32C3_IROM_HIGH   0x42800000

struct esp32c3_common {
	struct esp_riscv_common esp_riscv;
	bool was_reset;
};

static inline struct  esp32c3_common *esp32c3_common(const struct target *target)
{
	return target->arch_info;
}

#endif	/* OPENOCD_TARGET_ESP32C3_H */
