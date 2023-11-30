/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Generic flash driver for Espressif Xtensa chips                       *
 *   Copyright (C) 2019 Espressif Systems Ltd.                             *
 ***************************************************************************/

#ifndef OPENOCD_FLASH_NOR_ESP_XTENSA_H
#define OPENOCD_FLASH_NOR_ESP_XTENSA_H

#include "esp_flash.h"

/* ESP Xtensa flash data.
   It should be the first member of flash data structs for concrete chips.
   For example see ESP32 flash driver implementation. */
struct esp_xtensa_flash_bank {
	struct esp_flash_bank esp;
};

int esp_xtensa_flash_init(struct esp_xtensa_flash_bank *esp_info, uint32_t sec_sz,
	int (*run_func_image)(struct target *target, struct esp_algorithm_run_data *run,
		uint32_t num_args, ...),
	bool (*is_irom_address)(target_addr_t addr),
	bool (*is_drom_address)(target_addr_t addr),
	const struct esp_flasher_stub_config *(*get_stub)(struct flash_bank *bank));

#endif	/* OPENOCD_FLASH_NOR_ESP_XTENSA_H */
