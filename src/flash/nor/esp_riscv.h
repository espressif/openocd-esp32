/***************************************************************************
 *   Generic flash driver for Espressif RISCV chips                        *
 *   Copyright (C) 2021 Espressif Systems Ltd.                             *
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

#ifndef OPENOCD_FLASH_NOR_ESP_RISCV_H
#define OPENOCD_FLASH_NOR_ESP_RISCV_H

#include "esp_flash.h"

/* ESP RISCV flash data.
   It should be the first member of flash data structs for concrete chips.
   For example see ESP32-C3 flash driver implementation. */
struct esp_riscv_flash_bank {
	struct esp_flash_bank esp;
};

int esp_riscv_flash_init(struct esp_riscv_flash_bank *esp_info, uint32_t sec_sz,
	int (*run_func_image)(struct target *target, struct algorithm_run_data *run,
		uint32_t num_args, ...),
	bool (*is_irom_address)(target_addr_t addr),
	bool (*is_drom_address)(target_addr_t addr),
	const struct esp_flasher_stub_config *(*get_stub)(struct flash_bank *bank));


#endif	/* OPENOCD_FLASH_NOR_ESP_RISCV_H */
