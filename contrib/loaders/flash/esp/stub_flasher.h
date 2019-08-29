/***************************************************************************
 *   ESP xtensa chips flasher stub definitions                             *
 *   Copyright (C) 2017-2019 Espressif Systems Ltd.                        *
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
#ifndef ESP_XTENSA_FLASHER_STUB_H
#define ESP_XTENSA_FLASHER_STUB_H

#define ESP_XTENSA_STUB_ERR_OK                  0
#define ESP_XTENSA_STUB_ERR_FAIL                (-1)
#define ESP_XTENSA_STUB_ERR_NOT_SUPPORTED       (-2)

#define ESP_XTENSA_STUB_CMD_FLASH_READ          0
#define ESP_XTENSA_STUB_CMD_FLASH_WRITE         1
#define ESP_XTENSA_STUB_CMD_FLASH_ERASE         2
#define ESP_XTENSA_STUB_CMD_FLASH_ERASE_CHECK   3
#define ESP_XTENSA_STUB_CMD_FLASH_SIZE          4
#define ESP_XTENSA_STUB_CMD_FLASH_MAP_GET       5
#define ESP_XTENSA_STUB_CMD_FLASH_BP_SET        6
#define ESP_XTENSA_STUB_CMD_FLASH_BP_CLEAR      7
#define ESP_XTENSA_STUB_CMD_FLASH_TEST          8
#define ESP_XTENSA_STUB_CMD_FLASH_MAX_ID        ESP_XTENSA_STUB_CMD_FLASH_TEST
#define ESP_XTENSA_STUB_CMD_TEST                (ESP_XTENSA_STUB_CMD_FLASH_MAX_ID+2)

#define ESP_XTENSA_STUB_FLASH_MAPPINGS_MAX_NUM  2	/* IROM, DROM */

struct esp_xtensa_flash_region_mapping {
	uint32_t phy_addr;
	uint32_t load_addr;
	uint32_t size;
};

struct esp_xtensa_flash_mapping {
	uint32_t maps_num;
	struct esp_xtensa_flash_region_mapping maps[ESP_XTENSA_STUB_FLASH_MAPPINGS_MAX_NUM];
};

#endif	/*ESP_XTENSA_FLASHER_STUB_H */
