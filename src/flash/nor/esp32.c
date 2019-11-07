/**************************************************************************
 *   ESP32 flash driver for OpenOCD                                        *
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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "imp.h"
#include <target/esp_xtensa_apptrace.h>
#include <target/xtensa_mcore.h>
#include <target/esp32.h>
#include "esp_xtensa.h"
#include "contrib/loaders/flash/esp/esp32/stub_flasher_image.h"

#define ESP32_FLASH_SECTOR_SIZE 4096

struct esp32_flash_bank {
	struct esp_xtensa_flash_bank esp_xtensa;
};

static const uint8_t esp32_flasher_stub_code[] = {
#include "contrib/loaders/flash/esp/esp32/stub_flasher_code.inc"
};
static const uint8_t esp32_flasher_stub_data[] = {
#include "contrib/loaders/flash/esp/esp32/stub_flasher_data.inc"
};

static struct esp_xtensa_flasher_stub_config s_stub_cfg = {
	.code = esp32_flasher_stub_code,
	.code_sz = sizeof(esp32_flasher_stub_code),
	.data = esp32_flasher_stub_data,
	.data_sz = sizeof(esp32_flasher_stub_data),
	.entry_addr = ESP32_STUB_ENTRY_ADDR,
	.bss_sz = ESP32_STUB_BSS_SIZE
};


static bool esp32_is_irom_address(target_addr_t addr)
{
	return (addr >= ESP32_IROM_LOW && addr < ESP32_IROM_HIGH);
}

static bool esp32_is_drom_address(target_addr_t addr)
{
	return (addr >= ESP32_DROM_LOW && addr < ESP32_DROM_HIGH);
}

static const struct esp_xtensa_flasher_stub_config *esp32_get_stub(struct flash_bank *bank)
{
	return &s_stub_cfg;
}

/* flash bank <bank_name> esp32 <base> <size> 0 0 <target#>
   If <size> is zero flash size will be autodetected, otherwise user value will be used
 */
FLASH_BANK_COMMAND_HANDLER(esp32_flash_bank_command)
{
	struct esp32_flash_bank *esp32_info;

	if (CMD_ARGC < 6)
		return ERROR_COMMAND_SYNTAX_ERROR;

	esp32_info = malloc(sizeof(struct esp32_flash_bank));
	if (esp32_info == NULL)
		return ERROR_FAIL;
	int ret = esp_xtensa_flash_init(&esp32_info->esp_xtensa,
		ESP32_FLASH_SECTOR_SIZE,
		xtensa_mcore_run_func_image,
		esp32_is_irom_address,
		esp32_is_drom_address,
		esp32_get_stub);
	if (ret != ERROR_OK) {
		free(esp32_info);
		return ret;
	}
	bank->driver_priv = esp32_info;
	return ERROR_OK;
}

static int esp32_get_info(struct flash_bank *bank, char *buf, int buf_size)
{
	snprintf(buf, buf_size, "ESP32");
	return ERROR_OK;
}

static const struct command_registration esp32_command_handlers[] = {
	{
		.name = "esp32",
		.mode = COMMAND_ANY,
		.help = "esp32 flash command group",
		.usage = "",
		.chain = esp_xtensa_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

struct flash_driver esp32_flash = {
	.name = "esp32",
	.commands = esp32_command_handlers,
	.flash_bank_command = esp32_flash_bank_command,
	.erase = esp_xtensa_erase,
	.protect = esp_xtensa_protect,
	.write = esp_xtensa_write,
	.read = esp_xtensa_read,
	.probe = esp_xtensa_probe,
	.auto_probe = esp_xtensa_auto_probe,
	.erase_check = esp_xtensa_blank_check,
	.protect_check = esp_xtensa_protect_check,
	.info = esp32_get_info,
};
