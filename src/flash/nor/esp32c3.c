/***************************************************************************
 *   ESP32-C3 flash driver for OpenOCD                                     *
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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <target/espressif/esp32c3.h>
#include <target/espressif/esp_riscv_algorithm.h>
#include "imp.h"
#include "esp_riscv.h"
#include "contrib/loaders/flash/esp/esp32c3/stub_flasher_image.h"
#include "contrib/loaders/flash/esp/esp32c3/sdkconfig.h"

#define ESP32C3_FLASH_SECTOR_SIZE 4096

struct esp32c3_flash_bank {
	struct esp_riscv_flash_bank riscv;
};

static const uint8_t s_esp32c3_flasher_stub_code[] = {
#include "contrib/loaders/flash/esp/esp32c3/stub_flasher_code.inc"
};
static const uint8_t s_esp32c3_flasher_stub_data[] = {
#include "contrib/loaders/flash/esp/esp32c3/stub_flasher_data.inc"
};

static const struct esp_flasher_stub_config s_esp32c3_stub_cfg = {
	.code = s_esp32c3_flasher_stub_code,
	.code_sz = sizeof(s_esp32c3_flasher_stub_code),
	.data = s_esp32c3_flasher_stub_data,
	.data_sz = sizeof(s_esp32c3_flasher_stub_data),
	.entry_addr = ESP32C3_STUB_ENTRY_ADDR,
	.bss_sz = ESP32C3_STUB_BSS_SIZE,
	.first_user_reg_param = ESP_RISCV_STUB_ARGS_FUNC_START,
	.apptrace_ctrl_addr = ESP32C3_STUB_APPTRACE_CTRL_ADDR,
	.stack_data_pool_sz = CONFIG_STUB_STACK_DATA_POOL_SIZE
};

static bool esp32c3_is_irom_address(target_addr_t addr)
{
	return (addr >= ESP32C3_IROM_LOW && addr < ESP32C3_IROM_HIGH);
}

static bool esp32c3_is_drom_address(target_addr_t addr)
{
	return (addr >= ESP32C3_DROM_LOW && addr < ESP32C3_DROM_HIGH);
}

static const struct esp_flasher_stub_config *esp32c3_get_stub(struct flash_bank *bank)
{
	return &s_esp32c3_stub_cfg;
}

/* flash bank <bank_name> esp32 <base> <size> 0 0 <target#>
   If <size> is zero flash size will be autodetected, otherwise user value will be used
 */
FLASH_BANK_COMMAND_HANDLER(esp32c3_flash_bank_command)
{
	struct esp32c3_flash_bank *esp32c3_info;

	if (CMD_ARGC < 6)
		return ERROR_COMMAND_SYNTAX_ERROR;

	esp32c3_info = malloc(sizeof(struct esp32c3_flash_bank));
	if (esp32c3_info == NULL)
		return ERROR_FAIL;
	int ret = esp_riscv_flash_init(&esp32c3_info->riscv,
		ESP32C3_FLASH_SECTOR_SIZE,
		algorithm_run_func_image,
		esp32c3_is_irom_address,
		esp32c3_is_drom_address,
		esp32c3_get_stub);
	if (ret != ERROR_OK) {
		free(esp32c3_info);
		return ret;
	}
	bank->driver_priv = esp32c3_info;
	return ERROR_OK;
}

static int esp32c3_get_info(struct flash_bank *bank, struct command_invocation *cmd)
{
	/* TODO: print some flash information */
	command_print_sameline(cmd, "Flash driver: ESP32-C3\n");
	return ERROR_OK;
}

static const struct command_registration esp32c3_command_handlers[] = {
	{
		.name = "esp",
		.mode = COMMAND_ANY,
		.help = "ESP flash command group",
		.usage = "",
		.chain = esp_flash_exec_flash_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

struct flash_driver esp32c3_flash = {
	.name = "esp32c3",
	.commands = esp32c3_command_handlers,
	.flash_bank_command = esp32c3_flash_bank_command,
	.erase = esp_flash_erase,
	.protect = esp_flash_protect,
	.write = esp_flash_write,
	.read = esp_flash_read,
	.probe = esp_flash_probe,
	.auto_probe = esp_flash_auto_probe,
	.erase_check = esp_flash_blank_check,
	.protect_check = esp_flash_protect_check,
	.info = esp32c3_get_info,
	.free_driver_priv = default_flash_free_driver_priv,
};
