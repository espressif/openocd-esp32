/***************************************************************************
 *   ESP32-S2 flash driver for OpenOCD                                     *
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
#include <target/esp_xtensa.h>
#include <target/esp32_s2.h>
#include "esp_xtensa.h"
#include "contrib/loaders/flash/esp/esp32_s2/stub_flasher_image.h"
#include "contrib/loaders/flash/esp/esp32_s2beta/stub_flasher_image.h"

#define ESP32_S2_FLASH_SECTOR_SIZE 4096

struct esp32_s2_flash_bank {
	struct esp_xtensa_flash_bank esp_xtensa;
};

static const uint8_t esp32_s2_flasher_stub_code[] = {
#include "contrib/loaders/flash/esp/esp32_s2/stub_flasher_code.inc"
};
static const uint8_t esp32_s2_flasher_stub_data[] = {
#include "contrib/loaders/flash/esp/esp32_s2/stub_flasher_data.inc"
};

static const uint8_t esp32_s2beta_flasher_stub_code[] = {
#include "contrib/loaders/flash/esp/esp32_s2beta/stub_flasher_code.inc"
};
static const uint8_t esp32_s2beta_flasher_stub_data[] = {
#include "contrib/loaders/flash/esp/esp32_s2beta/stub_flasher_data.inc"
};


static struct esp_xtensa_flasher_stub_config s_esp32_s2_stub_cfg = {
	.code = esp32_s2_flasher_stub_code,
	.code_sz = sizeof(esp32_s2_flasher_stub_code),
	.data = esp32_s2_flasher_stub_data,
	.data_sz = sizeof(esp32_s2_flasher_stub_data),
	.entry_addr = ESP32_S2_STUB_ENTRY_ADDR,
	.bss_sz = ESP32_S2_STUB_BSS_SIZE
};

static struct esp_xtensa_flasher_stub_config s_esp32_s2beta_stub_cfg = {
	.code = esp32_s2beta_flasher_stub_code,
	.code_sz = sizeof(esp32_s2beta_flasher_stub_code),
	.data = esp32_s2beta_flasher_stub_data,
	.data_sz = sizeof(esp32_s2beta_flasher_stub_data),
	.entry_addr = ESP32_S2BETA_STUB_ENTRY_ADDR,
	.bss_sz = ESP32_S2BETA_STUB_BSS_SIZE
};

static bool esp32_s2_is_irom_address(target_addr_t addr)
{
	return (addr >= ESP32_S2_IROM_LOW && addr < ESP32_S2_IROM_HIGH);
}

static bool esp32_s2_is_drom_address(target_addr_t addr)
{
	return (addr >= ESP32_S2_DROM_LOW && addr < ESP32_S2_DROM_HIGH);
}

static const struct esp_xtensa_flasher_stub_config *esp32_s2_get_stub(struct flash_bank *bank)
{
	struct esp32_s2_common *esp32 = target_to_esp32_s2(bank->target);

	if (esp32->chip_rev == ESP32_S2_REV_BETA)
		return &s_esp32_s2beta_stub_cfg;
	else if (esp32->chip_rev == ESP32_S2_REV_0)
		return &s_esp32_s2_stub_cfg;
	return NULL;
}

/* flash bank <bank_name> esp32 <base> <size> 0 0 <target#>
   If <size> is zero flash size will be autodetected, otherwise user value will be used
 */
FLASH_BANK_COMMAND_HANDLER(esp32_s2_flash_bank_command)
{
	struct esp32_s2_flash_bank *esp32_s2_info;

	if (CMD_ARGC < 6)
		return ERROR_COMMAND_SYNTAX_ERROR;

	esp32_s2_info = malloc(sizeof(struct esp32_s2_flash_bank));
	if (esp32_s2_info == NULL)
		return ERROR_FAIL;
	int ret = esp_xtensa_flash_init(&esp32_s2_info->esp_xtensa,
		ESP32_S2_FLASH_SECTOR_SIZE,
		xtensa_run_func_image,
		esp32_s2_is_irom_address,
		esp32_s2_is_drom_address,
		esp32_s2_get_stub);
	if (ret != ERROR_OK) {
		free(esp32_s2_info);
		return ret;
	}
	bank->driver_priv = esp32_s2_info;
	return ERROR_OK;
}

static int esp32_s2_get_info(struct flash_bank *bank, char *buf, int buf_size)
{
	snprintf(buf, buf_size, "ESP32_S2");
	return ERROR_OK;
}

static const struct command_registration esp32_s2_command_handlers[] = {
	{
		.name = "esp32_s2",
		.mode = COMMAND_ANY,
		.help = "esp32_s2 flash command group",
		.usage = "",
		.chain = esp_xtensa_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

struct flash_driver esp32_s2_flash = {
	.name = "esp32_s2",
	.commands = esp32_s2_command_handlers,
	.flash_bank_command = esp32_s2_flash_bank_command,
	.erase = esp_xtensa_erase,
	.protect = esp_xtensa_protect,
	.write = esp_xtensa_write,
	.read = esp_xtensa_read,
	.probe = esp_xtensa_probe,
	.auto_probe = esp_xtensa_auto_probe,
	.erase_check = esp_xtensa_blank_check,
	.protect_check = esp_xtensa_protect_check,
	.info = esp32_s2_get_info,
};
