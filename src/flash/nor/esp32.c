/* SPDX-License-Identifier: GPL-2.0-or-later */

/**************************************************************************
 *   ESP32 flash driver for OpenOCD                                        *
 *   Copyright (C) 2017 Espressif Systems Ltd.                             *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "imp.h"
#include <target/smp.h>
#include <target/espressif/esp_xtensa_apptrace.h>
#include <target/espressif/esp_xtensa_algorithm.h>
#include <target/espressif/esp_xtensa_smp.h>
#include "esp_xtensa.h"
#include "../../../contrib/loaders/flash/espressif/esp32/stub_flasher_image.h"
#include "../../../contrib/loaders/flash/espressif/esp32/stub_flasher_image_wlog.h"

#define ESP32_DROM_LOW            0x3F400000
#define ESP32_DROM_HIGH           0x3F800000
#define ESP32_IROM_LOW            0x400D0000
#define ESP32_IROM_HIGH           0x40400000

#define ESP32_FLASH_SECTOR_SIZE         4096

struct esp32_flash_bank {
	struct esp_xtensa_flash_bank esp_xtensa;
};

static const uint8_t esp32_flasher_stub_code[] = {
#include "../../../contrib/loaders/flash/espressif/esp32/stub_flasher_code.inc"
};
static const uint8_t esp32_flasher_stub_data[] = {
#include "../../../contrib/loaders/flash/espressif/esp32/stub_flasher_data.inc"
};
static const uint8_t esp32_flasher_stub_code_wlog[] = {
#include "../../../contrib/loaders/flash/espressif/esp32/stub_flasher_code_wlog.inc"
};
static const uint8_t esp32_flasher_stub_data_wlog[] = {
#include "../../../contrib/loaders/flash/espressif/esp32/stub_flasher_data_wlog.inc"
};

static const struct esp_flasher_stub_config s_esp32_stub_cfg = {
	.code = esp32_flasher_stub_code,
	.code_sz = sizeof(esp32_flasher_stub_code),
	.data = esp32_flasher_stub_data,
	.data_sz = sizeof(esp32_flasher_stub_data),
	.entry_addr = ESP32_STUB_ENTRY_ADDR,
	.bss_sz = ESP32_STUB_BSS_SIZE,
	.iram_org = ESP32_STUB_IRAM_ORG,
	.iram_len = ESP32_STUB_IRAM_LEN,
	.dram_org = ESP32_STUB_DRAM_ORG,
	.dram_len = ESP32_STUB_DRAM_LEN,
	.first_user_reg_param = ESP_XTENSA_STUB_ARGS_FUNC_START,
	.reverse = true
};

static const struct esp_flasher_stub_config s_esp32_stub_cfg_wlog = {
	.code = esp32_flasher_stub_code_wlog,
	.code_sz = sizeof(esp32_flasher_stub_code_wlog),
	.data = esp32_flasher_stub_data_wlog,
	.data_sz = sizeof(esp32_flasher_stub_data_wlog),
	.entry_addr = ESP32_STUB_WLOG_ENTRY_ADDR,
	.bss_sz = ESP32_STUB_WLOG_BSS_SIZE,
	.iram_org = ESP32_STUB_IRAM_ORG,
	.iram_len = ESP32_STUB_IRAM_LEN,
	.dram_org = ESP32_STUB_DRAM_ORG,
	.dram_len = ESP32_STUB_DRAM_LEN,
	.first_user_reg_param = ESP_XTENSA_STUB_ARGS_FUNC_START,
	.log_buff_addr = ESP32_STUB_WLOG_LOG_ADDR,
	.log_buff_size = ESP32_STUB_WLOG_LOG_SIZE,
	.reverse = true
};

static bool esp32_is_irom_address(target_addr_t addr)
{
	return addr >= ESP32_IROM_LOW && addr < ESP32_IROM_HIGH;
}

static bool esp32_is_drom_address(target_addr_t addr)
{
	return addr >= ESP32_DROM_LOW && addr < ESP32_DROM_HIGH;
}

static const struct esp_flasher_stub_config *esp32_get_stub(struct flash_bank *bank)
{
	struct esp_flash_bank *esp_info = bank->driver_priv;
	if (esp_info->stub_log_enabled)
		return &s_esp32_stub_cfg_wlog;
	return &s_esp32_stub_cfg;
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
		esp_xtensa_smp_run_func_image,
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

static int esp32_get_info(struct flash_bank *bank, struct command_invocation *cmd)
{
	/* TODO: print some flash information */
	command_print_sameline(cmd, "Flash driver: ESP32\n");
	return ERROR_OK;
}

static const struct command_registration esp32_command_handlers[] = {
	{
		.name = "esp",
		.mode = COMMAND_ANY,
		.help = "ESP flash command group",
		.usage = "",
		.chain = esp_flash_exec_flash_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration esp32_legacy_command_handlers[] = {
	{
		.name = "esp32",
		.mode = COMMAND_ANY,
		.help = "ESP32 flash command group",
		.usage = "",
		.chain = esp_flash_exec_flash_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration esp32_all_command_handlers[] = {
	{
		.usage = "",
		.chain = esp32_command_handlers,
	},
	{
		.usage = "",
		.chain = esp32_legacy_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

const struct flash_driver esp32_flash = {
	.name = "esp32",
	.commands = esp32_all_command_handlers,
	.flash_bank_command = esp32_flash_bank_command,
	.erase = esp_algo_flash_erase,
	.protect = esp_algo_flash_protect,
	.write = esp_algo_flash_write,
	.read = esp_algo_flash_read,
	.probe = esp_algo_flash_probe,
	.auto_probe = esp_algo_flash_auto_probe,
	.erase_check = esp_algo_flash_blank_check,
	.protect_check = esp_algo_flash_protect_check,
	.info = esp32_get_info,
	.free_driver_priv = default_flash_free_driver_priv,
};
