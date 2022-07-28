/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   ESP32-H2 target for OpenOCD                                           *
 *   Copyright (C) 2022 Espressif Systems Ltd.                             *
 ***************************************************************************/

#ifndef OPENOCD_TARGET_ESP32H2_H
#define OPENOCD_TARGET_ESP32H2_H

#include "esp_riscv.h"

#define ESP32H2_DROM_LOW    0x42000000
#define ESP32H2_DROM_HIGH   0x43000000
#define ESP32H2_IROM_LOW    0x42000000
#define ESP32H2_IROM_HIGH   0x43000000

struct esp32h2_common {
	struct esp_riscv_common esp_riscv;
	bool was_reset;
};

static inline struct esp32h2_common *esp32h2_common(const struct target *target)
{
	return target->arch_info;
}

#endif	/* OPENOCD_TARGET_ESP32H2_H */
