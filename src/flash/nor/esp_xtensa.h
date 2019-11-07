/***************************************************************************
 *   Generic flash driver for Espressif Xtensa chips                       *
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

#ifndef FLASH_ESP_XTENSA_H
#define FLASH_ESP_XTENSA_H

#include <target/target.h>
#include <helper/command.h>
#include <target/xtensa_algorithm.h>
#include <target/esp_xtensa.h>
#include <flash/nor/core.h>

/* ESP xtensa flash data.
   It should be the first member of flash data structs for concrete chips.
   For example see ESP32 flash driver implementation. */
struct esp_xtensa_flash_bank {
	int probed;
	/* Sector size */
	uint32_t sec_sz;
	/* Base address of the bank in the flash, 0 - for HW flash bank, non-zero for special
	 * IROM/DROM fake banks */
	/* Those fake banks are necessary for generating proper memory map for GDB and using flash
	 * breakpoints */
	uint32_t hw_flash_base;
	/* Offset of the application image in the HW flash bank */
	uint32_t appimage_flash_base;
	const struct esp_xtensa_flasher_stub_config *(*get_stub)(struct flash_bank *bank);
	/* function to run algorithm on Xtensa target */
	int (*run_func_image)(struct target *target, struct xtensa_algo_run_data *run,
		struct xtensa_algo_image *image, uint32_t num_args, ...);
	bool (*is_irom_address)(target_addr_t addr);
	bool (*is_drom_address)(target_addr_t addr);
};

struct esp_xtensa_flasher_stub_config {
	const uint8_t *code;
	uint32_t code_sz;
	const uint8_t *data;
	uint32_t data_sz;
	target_addr_t entry_addr;
	uint32_t bss_sz;
};

extern const struct command_registration esp_xtensa_exec_command_handlers[];

int esp_xtensa_flash_init(struct esp_xtensa_flash_bank *esp_xtensa_info, uint32_t sec_sz,
	int (*run_func_image)(struct target *target, struct xtensa_algo_run_data *run,
		struct xtensa_algo_image *image, uint32_t num_args, ...),
	bool (*is_irom_address)(target_addr_t addr),
	bool (*is_drom_address)(target_addr_t addr),
	const struct esp_xtensa_flasher_stub_config *(*get_stub)(struct flash_bank *bank));
int esp_xtensa_protect(struct flash_bank *bank, int set, int first, int last);
int esp_xtensa_protect_check(struct flash_bank *bank);
int esp_xtensa_blank_check(struct flash_bank *bank);
int esp_xtensa_erase(struct flash_bank *bank, int first, int last);
int esp_xtensa_write(struct flash_bank *bank, const uint8_t *buffer,
	uint32_t offset, uint32_t count);
int esp_xtensa_read(struct flash_bank *bank, uint8_t *buffer,
	uint32_t offset, uint32_t count);
int esp_xtensa_probe(struct flash_bank *bank);
int esp_xtensa_auto_probe(struct flash_bank *bank);
int esp_xtensa_flash_breakpoint_add(struct target *target,
	struct breakpoint *breakpoint,
	struct esp_xtensa_special_breakpoint *sw_bp);
int esp_xtensa_flash_breakpoint_remove(struct target *target,
	struct esp_xtensa_special_breakpoint *sw_bp);

#endif	/*FLASH_ESP_XTENSA_H*/
