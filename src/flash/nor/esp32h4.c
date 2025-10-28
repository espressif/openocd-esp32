// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   ESP32-H4 flash driver for OpenOCD                                    *
 *   Copyright (C) 2025 Espressif Systems Ltd.                             *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <target/espressif/esp_riscv_algorithm.h>
#include "imp.h"
#include "esp_riscv.h"
#include <target/espressif/esp_riscv_apptrace.h>
#include <target/espressif/esp_riscv.h>

#define ESP_TARGET_ESP32H4
#include "esp_stub_config.h"
#undef ESP_TARGET_ESP32H4

#define ESP32H4_FLASH_SECTOR_SIZE 4096

/* memory map */
#define ESP32H4_DROM_LOW    0x40810000
#define ESP32H4_DROM_HIGH   0x40860000
#define ESP32H4_IROM_LOW    0x42000000
#define ESP32H4_IROM_HIGH   0x44000000

struct esp32h4_flash_bank {
	struct esp_riscv_flash_bank riscv;
};

static bool esp32h4_is_irom_address(target_addr_t addr)
{
	return addr >= ESP32H4_IROM_LOW && addr < ESP32H4_IROM_HIGH;
}

static bool esp32h4_is_drom_address(target_addr_t addr)
{
	return addr >= ESP32H4_DROM_LOW && addr < ESP32H4_DROM_HIGH;
}

static const struct esp_flasher_stub_config *esp32h4_get_stub(struct flash_bank *bank, int cmd)
{
	struct esp_flash_bank *esp_info = bank->driver_priv;
	if (esp_info->stub_log_enabled)
		return s_cmd_map[ESP_STUB_CMD_TEST_ALL].config;
	return s_cmd_map[cmd].config;
}

/* flash bank <bank_name> esp32 <base> <size> 0 0 <target#>
   If <size> is zero flash size will be autodetected, otherwise user value will be used
 */
FLASH_BANK_COMMAND_HANDLER(esp32h4_flash_bank_command)
{
	struct esp32h4_flash_bank *esp32h4_info;

	if (CMD_ARGC < 6)
		return ERROR_COMMAND_SYNTAX_ERROR;

	esp32h4_info = malloc(sizeof(struct esp32h4_flash_bank));
	if (!esp32h4_info)
		return ERROR_FAIL;
	int ret = esp_riscv_flash_init(&esp32h4_info->riscv,
		ESP32H4_FLASH_SECTOR_SIZE,
		esp_riscv_smp_run_func_image,
		esp32h4_is_irom_address,
		esp32h4_is_drom_address,
		esp32h4_get_stub,
		false);
	if (ret != ERROR_OK) {
		free(esp32h4_info);
		return ret;
	}
	bank->driver_priv = esp32h4_info;
	return ERROR_OK;
}

static int esp32h4_get_info(struct flash_bank *bank, struct command_invocation *cmd)
{
	command_print_sameline(cmd, "Flash driver: ESP32-H4\n");
	return ERROR_OK;
}

static const struct command_registration esp32h4_command_handlers[] = {
	{
		.name = "esp",
		.mode = COMMAND_ANY,
		.help = "ESP flash command group",
		.usage = "",
		.chain = esp_flash_exec_flash_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

const struct flash_driver esp32h4_flash = {
	.name = "esp32h4",
	.commands = esp32h4_command_handlers,
	.flash_bank_command = esp32h4_flash_bank_command,
	.erase = esp_algo_flash_erase,
	.protect = esp_algo_flash_protect,
	.write = esp_algo_flash_write,
	.read = esp_algo_flash_read,
	.probe = esp_algo_flash_probe,
	.auto_probe = esp_algo_flash_auto_probe,
	.erase_check = esp_algo_flash_blank_check,
	.protect_check = esp_algo_flash_protect_check,
	.info = esp32h4_get_info,
	.free_driver_priv = default_flash_free_driver_priv,
};
