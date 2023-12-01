/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   ESP32-S2 flasher stub definitions                                     *
 *   Copyright (C) 2023 Espressif Systems Ltd.                             *
 ***************************************************************************/
#ifndef OPENOCD_LOADERS_FLASH_ESPRESSIF_ESP32S2_STUB_FLASHER_CHIP_H
#define OPENOCD_LOADERS_FLASH_ESPRESSIF_ESP32S2_STUB_FLASHER_CHIP_H

#include <esp32s2/rom/spi_flash.h>
#include <esp32s2/rom/miniz.h>
#include <esp32s2/rom/sha.h>
#include <stub_xtensa_common.h>

struct stub_flash_state {
	uint32_t cache_flags[2];
	bool cache_enabled;
};

void stub_flash_state_prepare(struct stub_flash_state *state);
void stub_flash_state_restore(struct stub_flash_state *state);

#endif	/* OPENOCD_LOADERS_FLASH_ESPRESSIF_ESP32S2_STUB_FLASHER_CHIP_H */
