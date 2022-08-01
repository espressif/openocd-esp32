/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   ESP32-C3 target for OpenOCD                                           *
 *   Copyright (C) 2020 Espressif Systems Ltd.                             *
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
