// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   ESP32-P4 flash driver for OpenOCD                                    *
 *   Copyright (C) 2024 Espressif Systems Ltd.                             *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <target/espressif/esp_riscv_algorithm.h>
#include "imp.h"
#include "esp_riscv.h"
#include <target/espressif/esp_riscv_apptrace.h>
#include <target/espressif/esp_riscv.h>

#define ESP_TARGET_ESP32P4
#include "esp_stub_config.h"
#undef ESP_TARGET_ESP32P4

#define ESP32P4_FLASH_SECTOR_SIZE 4096

/* memory map */
#define ESP32P4_DROM_LOW    0x40000000
#define ESP32P4_DROM_HIGH   0x44000000
#define ESP32P4_IROM_LOW    0x40000000
#define ESP32P4_IROM_HIGH   0x44000000

struct esp32p4_flash_bank {
	struct esp_riscv_flash_bank riscv;
};

static bool esp32p4_is_irom_address(target_addr_t addr)
{
	return addr >= ESP32P4_IROM_LOW && addr < ESP32P4_IROM_HIGH;
}

static bool esp32p4_is_drom_address(target_addr_t addr)
{
	return addr >= ESP32P4_DROM_LOW && addr < ESP32P4_DROM_HIGH;
}

static const struct esp_flasher_stub_config *esp32p4_get_stub(struct flash_bank *bank, int cmd)
{
	struct esp_flash_bank *esp_info = bank->driver_priv;
	if (esp_info->stub_log_enabled)
		return s_cmd_map[ESP_STUB_CMD_TEST_ALL].config;
	return s_cmd_map[cmd].config;
}

/* flash bank <bank_name> esp32 <base> <size> 0 0 <target#>
   If <size> is zero flash size will be autodetected, otherwise user value will be used
 */
FLASH_BANK_COMMAND_HANDLER(esp32p4_flash_bank_command)
{
	struct esp32p4_flash_bank *esp32p4_info;

	if (CMD_ARGC < 6)
		return ERROR_COMMAND_SYNTAX_ERROR;

	esp32p4_info = malloc(sizeof(struct esp32p4_flash_bank));
	if (!esp32p4_info)
		return ERROR_FAIL;
	int ret = esp_riscv_flash_init(&esp32p4_info->riscv,
		ESP32P4_FLASH_SECTOR_SIZE,
		esp_riscv_smp_run_func_image,
		esp32p4_is_irom_address,
		esp32p4_is_drom_address,
		esp32p4_get_stub,
		false);
	if (ret != ERROR_OK) {
		free(esp32p4_info);
		return ret;
	}
	bank->driver_priv = esp32p4_info;
	return ERROR_OK;
}

static int esp32p4_get_info(struct flash_bank *bank, struct command_invocation *cmd)
{
	command_print_sameline(cmd, "Flash driver: ESP32-P4\n");
	return ERROR_OK;
}

static const struct command_registration esp32p4_command_handlers[] = {
	{
		.name = "esp",
		.mode = COMMAND_ANY,
		.help = "ESP flash command group",
		.usage = "",
		.chain = esp_flash_exec_flash_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

const struct flash_driver esp32p4_flash = {
	.name = "esp32p4",
	.commands = esp32p4_command_handlers,
	.flash_bank_command = esp32p4_flash_bank_command,
	.erase = esp_algo_flash_erase,
	.protect = esp_algo_flash_protect,
	.write = esp_algo_flash_write,
	.read = esp_algo_flash_read,
	.probe = esp_algo_flash_probe,
	.auto_probe = esp_algo_flash_auto_probe,
	.erase_check = esp_algo_flash_blank_check,
	.protect_check = esp_algo_flash_protect_check,
	.info = esp32p4_get_info,
	.free_driver_priv = default_flash_free_driver_priv,
};
