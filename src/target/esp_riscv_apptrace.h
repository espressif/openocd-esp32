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
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.           *
 ***************************************************************************/
#ifndef ESP_RISCV_APPTRACE_H__
#define ESP_RISCV_APPTRACE_H__

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

int esp_riscv_apptrace_info_init(struct target *target, target_addr_t ctrl_addr);

extern struct esp32_apptrace_hw esp_riscv_apptrace_hw;

#endif	/*ESP_RISCV_APPTRACE_H__*/
