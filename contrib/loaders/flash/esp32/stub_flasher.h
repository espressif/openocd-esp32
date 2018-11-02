/***************************************************************************
 *   ESP32 flassher stub definitions                                       *
 *   Copyright (C) 2017 Espressif Systems Ltd.                             *
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
#ifndef ESP32_FLASHER_STUB_H
#define ESP32_FLASHER_STUB_H

#define ESP32_STUB_ERR_OK                   0
#define ESP32_STUB_ERR_FAIL                 (-1)
#define ESP32_STUB_ERR_NOT_SUPPORTED        (-2)

#define ESP32_STUB_CMD_FLASH_READ           0
#define ESP32_STUB_CMD_FLASH_WRITE          1
#define ESP32_STUB_CMD_FLASH_ERASE          2
#define ESP32_STUB_CMD_FLASH_ERASE_CHECK    3
#define ESP32_STUB_CMD_FLASH_SIZE	        4
#define ESP32_STUB_CMD_FLASH_MAP_GET        5
#define ESP32_STUB_CMD_FLASH_BP_SET			6
#define ESP32_STUB_CMD_FLASH_BP_CLEAR		7
#define ESP32_STUB_CMD_FLASH_MAX_ID	        ESP32_STUB_CMD_FLASH_BP_CLEAR
#define ESP32_STUB_CMD_FLASH_TEST           (ESP32_STUB_CMD_FLASH_MAX_ID+1)
#define ESP32_STUB_CMD_TEST                 8

#define ESP32_STUB_FLASH_MAPPINGS_MAX_NUM		2 // IROM, DROM

#define ESP32_FLASH_SECTOR_SIZE       			4096
#define ESP32_STUB_BP_INSN_SECT_BUF_SIZE		(2*ESP32_FLASH_SECTOR_SIZE)

struct esp32_flash_region_mapping {
	uint32_t phy_addr;
	uint32_t load_addr;
	uint32_t size;
};

struct esp32_flash_mapping {
	uint32_t maps_num;
	struct esp32_flash_region_mapping maps[ESP32_STUB_FLASH_MAPPINGS_MAX_NUM];
};

static inline uint8_t xtensa_get_insn_size(uint8_t *insn)
{
  return insn[0] & 0x8 ? 2 : 3;
}

#endif //ESP32_FLASHER_STUB_H
