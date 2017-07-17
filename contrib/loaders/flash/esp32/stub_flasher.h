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

#define ESP32_STUB_CMD_TEST                 0
#define ESP32_STUB_CMD_FLASH_READ           1
#define ESP32_STUB_CMD_FLASH_WRITE          2
#define ESP32_STUB_CMD_FLASH_ERASE          3
#define ESP32_STUB_CMD_FLASH_ERASE_CHECK    4
#define ESP32_STUB_CMD_FLASH_SIZE	        5
#define ESP32_STUB_CMD_FLASH_MAX_ID	        ESP32_STUB_CMD_FLASH_SIZE
#define ESP32_STUB_CMD_FLASH_TEST           6

#endif //ESP32_FLASHER_STUB_H