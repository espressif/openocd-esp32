/***************************************************************************
 *   Application level tracing API for Espressif Xtensa chips              *
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

#ifndef OPENOCD_TARGET_ESP_XTENSA_APPTRACE_H
#define OPENOCD_TARGET_ESP_XTENSA_APPTRACE_H

#include "esp32_apptrace.h"

struct esp_xtensa_apptrace_info {
	const struct esp32_apptrace_hw *hw;
};

extern struct esp32_apptrace_hw esp_xtensa_apptrace_hw;

int esp_xtensa_apptrace_data_len_read(struct target *target,
	uint32_t *block_id,
	uint32_t *len);
int esp_xtensa_apptrace_data_read(struct target *target,
	uint32_t size,
	uint8_t *buffer,
	uint32_t block_id,
	bool ack);
int esp_xtensa_apptrace_ctrl_reg_read(struct target *target,
	uint32_t *block_id,
	uint32_t *len,
	bool *conn);
int esp_xtensa_apptrace_ctrl_reg_write(struct target *target,
	uint32_t block_id,
	uint32_t len,
	bool conn,
	bool data);
int esp_xtensa_apptrace_status_reg_write(struct target *target, uint32_t stat);
int esp_xtensa_apptrace_status_reg_read(struct target *target, uint32_t *stat);
uint32_t esp_xtensa_apptrace_block_max_size_get(struct target *target);
uint32_t esp_xtensa_apptrace_usr_block_max_size_get(struct target *target);
int esp_xtensa_apptrace_usr_block_write(struct target *target,
	uint32_t block_id,
	const uint8_t *data,
	uint32_t size);

#endif	/* OPENOCD_TARGET_ESP_XTENSA_APPTRACE_H */
