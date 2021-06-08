/***************************************************************************
 *   ESP32-S2 flasher stub definitions                                     *
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
#ifndef ESP32C3_FLASHER_STUB_H
#define ESP32C3_FLASHER_STUB_H

#include "sdkconfig.h"

#define STUB_FLASH_SECTOR_SIZE  0x1000
/* Flash geometry constants */
#define STUB_FLASH_BLOCK_SIZE   0x10000
#define STUB_FLASH_PAGE_SIZE    0x100
#define STUB_FLASH_STATUS_MASK  0xFFFF

struct stub_flash_state {
	uint32_t cache_flags[2];
	bool cache_enabled;
};

#define ESP_APPTRACE_USR_DATA_LEN_MAX   (CONFIG_APPTRACE_BUF_SIZE - 2)
#define RISCV_EBREAK    0x00100073

uint32_t stub_esp_clk_cpu_freq(void);


static inline uint8_t stub_get_insn_size(uint8_t *insn)
{
	/* we use 32bit `ebreak`, so there possible oprions:
	  - 16bit instruction will be overwritten plus 2 bytes followed that instruction
	  - 32bit instruction will be overwritten completely
	  - > 32bit instruction will be overwritten partially (the first 4 bytes)
	  */
	return 4;
}

static inline uint32_t stub_get_break_insn(uint8_t insn_sz)
{
	return RISCV_EBREAK;
}

void stub_stack_data_pool_init(uint8_t *data, size_t sz);

#endif	/*ESP32C3_FLASHER_STUB_H */
