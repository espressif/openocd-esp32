/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   ESP32-C6 target for OpenOCD                                           *
 *   Copyright (C) 2022 Espressif Systems Ltd.                             *
 ***************************************************************************/

#ifndef OPENOCD_TARGET_ESP32C6_H
#define OPENOCD_TARGET_ESP32C6_H

#include "esp_riscv.h"

#define ESP32C6_DROM_LOW    0x42000000
#define ESP32C6_DROM_HIGH   0x43000000
#define ESP32C6_IROM_LOW    0x42000000
#define ESP32C6_IROM_HIGH   0x43000000

struct esp32c6_common {
	struct esp_riscv_common esp_riscv;
	bool was_reset;
};

static inline struct esp32c6_common *esp32c6_common(const struct target *target)
{
	return target->arch_info;
}

#endif	/* OPENOCD_TARGET_ESP32C6_H */
