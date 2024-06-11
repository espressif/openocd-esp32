/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   ESP32-C3 flash driver for OpenOCD                                     *
 *   Copyright (C) 2021 Espressif Systems Ltd.                             *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <target/espressif/esp_riscv_algorithm.h>
#include "imp.h"
#include "esp_riscv.h"
#include <target/espressif/esp_riscv_apptrace.h>

#define ESP_TARGET_ESP32C3
#include "esp_stub_config.h"
#undef ESP_TARGET_ESP32C3

#define ESP32C3_FLASH_SECTOR_SIZE 4096

/* memory map */
#define ESP32C3_DROM_LOW    0x3C000000
#define ESP32C3_DROM_HIGH   0x3C800000
#define ESP32C3_IROM_LOW    0x42000000
#define ESP32C3_IROM_HIGH   0x42800000

struct esp32c3_flash_bank {
	struct esp_riscv_flash_bank riscv;
};

static bool esp32c3_is_irom_address(target_addr_t addr)
{
	return addr >= ESP32C3_IROM_LOW && addr < ESP32C3_IROM_HIGH;
}

static bool esp32c3_is_drom_address(target_addr_t addr)
{
	return addr >= ESP32C3_DROM_LOW && addr < ESP32C3_DROM_HIGH;
}

static const struct command_map s_cmd_map[ESP_STUB_CMD_FLASH_MAX_ID + 1] = {
	MAKE_CMD_MAP_ENTRIES
};

static const struct esp_flasher_stub_config *esp32c3_get_stub(struct flash_bank *bank, int cmd)
{
	struct esp_flash_bank *esp_info = bank->driver_priv;
	if (esp_info->stub_log_enabled)
		return s_cmd_map[ESP_STUB_CMD_FLASH_WITH_LOG].config;
	return s_cmd_map[cmd].config;
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
		esp_algorithm_run_func_image,
		esp32c3_is_irom_address,
		esp32c3_is_drom_address,
		esp32c3_get_stub,
		false);
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

const struct flash_driver esp32c3_flash = {
	.name = "esp32c3",
	.commands = esp32c3_command_handlers,
	.flash_bank_command = esp32c3_flash_bank_command,
	.erase = esp_algo_flash_erase,
	.protect = esp_algo_flash_protect,
	.write = esp_algo_flash_write,
	.read = esp_algo_flash_read,
	.probe = esp_algo_flash_probe,
	.auto_probe = esp_algo_flash_auto_probe,
	.erase_check = esp_algo_flash_blank_check,
	.protect_check = esp_algo_flash_protect_check,
	.info = esp32c3_get_info,
	.free_driver_priv = default_flash_free_driver_priv,
};
