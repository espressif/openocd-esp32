/***************************************************************************
 *   Application level tracing API for Espressif RISCV chips               *
 *   Copyright (C) 2019 Espressif Systems Ltd.                             *
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

#ifndef OPENOCD_TARGET_ESP_RISCV_APPTRACE_H
#define OPENOCD_TARGET_ESP_RISCV_APPTRACE_H

#include "esp32_apptrace.h"

struct esp_riscv_apptrace_mem_block {
	uint32_t start;	/* start address */
	uint32_t sz;	/* size */
};

struct esp_riscv_apptrace_info {
	const struct esp32_apptrace_hw *hw;
	target_addr_t ctrl_addr;
	struct esp_riscv_apptrace_mem_block mem_blocks[2];
};

extern struct esp32_apptrace_hw esp_riscv_apptrace_hw;

int esp_riscv_apptrace_info_init(struct target *target,
	target_addr_t ctrl_addr,
	target_addr_t *old_ctrl_addr);
int esp_riscv_apptrace_data_len_read(struct target *target,
	uint32_t *block_id,
	uint32_t *len);
int esp_riscv_apptrace_data_read(struct target *target,
	uint32_t size,
	uint8_t *buffer,
	uint32_t block_id,
	bool ack);
int esp_riscv_apptrace_ctrl_reg_read(struct target *target,
	uint32_t *block_id,
	uint32_t *len,
	bool *conn);
int esp_riscv_apptrace_ctrl_reg_write(struct target *target,
	uint32_t block_id,
	uint32_t len,
	bool conn,
	bool data);
uint32_t esp_riscv_apptrace_block_max_size_get(struct target *target);
uint32_t esp_riscv_apptrace_usr_block_max_size_get(struct target *target);
int esp_riscv_apptrace_usr_block_write(struct target *target,
	uint32_t block_id,
	const uint8_t *data,
	uint32_t size);

#endif	/* OPENOCD_TARGET_ESP_RISCV_APPTRACE_H */
