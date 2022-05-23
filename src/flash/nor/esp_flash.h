/***************************************************************************
 *   Generic flash driver for Espressif chips                              *
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

#ifndef OPENOCD_FLASH_NOR_ESP_FLASH_H
#define OPENOCD_FLASH_NOR_ESP_FLASH_H

#include <target/target.h>
#include <helper/command.h>
#include <target/espressif/esp_algorithm.h>
#include <target/breakpoints.h>
#include <flash/nor/core.h>

struct esp_flash_apptrace_hw {
	int (*info_init)(struct target *target,
		target_addr_t ctrl_addr,
		target_addr_t *old_ctrl_addr);
	int (*data_len_read)(struct target *target,
		uint32_t *block_id,
		uint32_t *len);
	int (*data_read)(struct target *target,
		uint32_t size,
		uint8_t *buffer,
		uint32_t block_id,
		bool ack);
	int (*ctrl_reg_read)(struct target *target,
		uint32_t *block_id,
		uint32_t *len,
		bool *conn);
	int (*ctrl_reg_write)(struct target *target,
		uint32_t block_id,
		uint32_t len,
		bool conn,
		bool data);
	int (*usr_block_write)(struct target *target,
		uint32_t block_id,
		const uint8_t *data,
		uint32_t size);
	uint8_t *(*usr_block_get)(uint8_t * buffer, uint32_t * size);
	uint32_t (*block_max_size_get)(struct target *target);
	uint32_t (*usr_block_max_size_get)(struct target *target);
};

struct esp_flasher_stub_config {
	const uint8_t *code;
	uint32_t code_sz;
	const uint8_t *data;
	uint32_t data_sz;
	target_addr_t entry_addr;
	uint32_t bss_sz;
	uint32_t first_user_reg_param;
	target_addr_t apptrace_ctrl_addr;
	uint32_t stack_data_pool_sz;
};

/* ESP flash data.
   It should be the first member of flash data structs for concrete chips.
   For example see ESP32 flash driver implementation. */
struct esp_flash_bank {
	int probed;
	/* Sector size */
	uint32_t sec_sz;
	/* Base address of the bank in the flash, 0 - for HW flash bank, non-zero for special
	 * IROM/DROM fake banks */
	/* Those fake banks are necessary for generating proper memory map for GDB and using flash
	 * breakpoints */
	uint32_t hw_flash_base;
	/* Minimal offset for erase/write on flash bank */
	uint32_t flash_min_offset;
	/* Offset of the application image in the HW flash bank */
	uint32_t appimage_flash_base;
	const struct esp_flasher_stub_config *(*get_stub)(struct flash_bank *bank);
	/* function to run algorithm on Xtensa target */
	int (*run_func_image)(struct target *target, struct algorithm_run_data *run,
		uint32_t num_args, ...);
	bool (*is_irom_address)(target_addr_t addr);
	bool (*is_drom_address)(target_addr_t addr);
	const struct esp_flash_apptrace_hw *apptrace_hw;
	const struct algorithm_hw *stub_hw;
	/* Upload compressed or uncompressed image */
	int compression;
	/* Stub cpu frequency before boost */
	int old_cpu_freq;
};

struct esp_flash_breakpoint {
	struct breakpoint *oocd_bp;
	/* original insn or part of it */
	uint8_t insn[4];
	/* original insn size. Actually this is size of break instruction. */
	uint8_t insn_sz;
	struct flash_bank *bank;
};

int esp_flash_init(struct esp_flash_bank *esp_info, uint32_t sec_sz,
	int (*run_func_image)(struct target *target, struct algorithm_run_data *run,
		uint32_t num_args, ...),
	bool (*is_irom_address)(target_addr_t addr),
	bool (*is_drom_address)(target_addr_t addr),
	const struct esp_flasher_stub_config *(*get_stub)(struct flash_bank *bank),
	const struct esp_flash_apptrace_hw *apptrace_hw,
	const struct algorithm_hw *stub_hw);
int esp_flash_protect(struct flash_bank *bank, int set, unsigned int first, unsigned int last);
int esp_flash_protect_check(struct flash_bank *bank);
int esp_flash_blank_check(struct flash_bank *bank);
int esp_flash_erase(struct flash_bank *bank, unsigned int first, unsigned int last);
int esp_flash_write(struct flash_bank *bank, const uint8_t *buffer,
	uint32_t offset, uint32_t count);
int esp_flash_read(struct flash_bank *bank, uint8_t *buffer,
	uint32_t offset, uint32_t count);
int esp_flash_probe(struct flash_bank *bank);
int esp_flash_auto_probe(struct flash_bank *bank);
int esp_flash_breakpoint_add(struct target *target,
	struct breakpoint *breakpoint,
	struct esp_flash_breakpoint *sw_bp);
int esp_flash_breakpoint_remove(struct target *target,
	struct esp_flash_breakpoint *sw_bp);

extern const struct command_registration esp_flash_exec_flash_command_handlers[];

COMMAND_HELPER(esp_flash_cmd_appimage_flashoff_do, struct target *target);
COMMAND_HELPER(esp_flash_cmd_set_compression, struct target *target);
COMMAND_HELPER(esp_flash_parse_cmd_verify_bank_hash, struct target *target);
COMMAND_HELPER(esp_flash_parse_cmd_clock_boost, struct target *target);

#endif	/* OPENOCD_FLASH_NOR_ESP_FLASH_H */
