/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   ESP32-C2 flash driver for OpenOCD                                     *
 *   Copyright (C) 2022 Espressif Systems Ltd.                             *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <target/espressif/esp32c2.h>
#include <target/espressif/esp_riscv_algorithm.h>
#include "imp.h"
#include "esp_riscv.h"
#include "contrib/loaders/flash/esp/esp32c2/stub_flasher_image.h"
#include "contrib/loaders/flash/esp/esp32c2/stub_flasher_image_wlog.h"

#define ESP32C2_FLASH_SECTOR_SIZE 4096

struct esp32c2_flash_bank {
	struct esp_riscv_flash_bank riscv;
};

static const uint8_t s_esp32c2_flasher_stub_code[] = {
#include "contrib/loaders/flash/esp/esp32c2/stub_flasher_code.inc"
};
static const uint8_t s_esp32c2_flasher_stub_data[] = {
#include "contrib/loaders/flash/esp/esp32c2/stub_flasher_data.inc"
};
static const uint8_t s_esp32c2_flasher_stub_code_wlog[] = {
#include "contrib/loaders/flash/esp/esp32c2/stub_flasher_code_wlog.inc"
};
static const uint8_t s_esp32c2_flasher_stub_data_wlog[] = {
#include "contrib/loaders/flash/esp/esp32c2/stub_flasher_data_wlog.inc"
};

static const struct esp_flasher_stub_config s_esp32c2_stub_cfg = {
	.code = s_esp32c2_flasher_stub_code,
	.code_sz = sizeof(s_esp32c2_flasher_stub_code),
	.data = s_esp32c2_flasher_stub_data,
	.data_sz = sizeof(s_esp32c2_flasher_stub_data),
	.entry_addr = ESP32C2_STUB_ENTRY_ADDR,
	.bss_sz = ESP32C2_STUB_BSS_SIZE,
	.first_user_reg_param = ESP_RISCV_STUB_ARGS_FUNC_START,
	.apptrace_ctrl_addr = ESP32C2_STUB_APPTRACE_CTRL_ADDR,
	.stack_data_pool_sz = ESP_RISCV_STACK_DATA_POOL_SIZE
};

static const struct esp_flasher_stub_config s_esp32c2_stub_cfg_wlog = {
	.code = s_esp32c2_flasher_stub_code_wlog,
	.code_sz = sizeof(s_esp32c2_flasher_stub_code_wlog),
	.data = s_esp32c2_flasher_stub_data_wlog,
	.data_sz = sizeof(s_esp32c2_flasher_stub_data_wlog),
	.entry_addr = ESP32C2_STUB_WLOG_ENTRY_ADDR,
	.bss_sz = ESP32C2_STUB_WLOG_BSS_SIZE,
	.first_user_reg_param = ESP_RISCV_STUB_ARGS_FUNC_START,
	.apptrace_ctrl_addr = ESP32C2_STUB_WLOG_APPTRACE_CTRL_ADDR,
	.stack_data_pool_sz = ESP_RISCV_STACK_DATA_POOL_SIZE,
	.log_buff_addr = ESP32C2_STUB_WLOG_LOG_ADDR,
	.log_buff_size = ESP32C2_STUB_WLOG_LOG_SIZE
};

static bool esp32c2_is_irom_address(target_addr_t addr)
{
	return addr >= ESP32C2_IROM_LOW && addr < ESP32C2_IROM_HIGH;
}

static bool esp32c2_is_drom_address(target_addr_t addr)
{
	return addr >= ESP32C2_DROM_LOW && addr < ESP32C2_DROM_HIGH;
}

static const struct esp_flasher_stub_config *esp32c2_get_stub(struct flash_bank *bank)
{
	struct esp_flash_bank *esp_info = bank->driver_priv;
	if (esp_info->stub_log_enabled)
		return &s_esp32c2_stub_cfg_wlog;
	return &s_esp32c2_stub_cfg;
}

/* flash bank <bank_name> esp32 <base> <size> 0 0 <target#>
   If <size> is zero flash size will be autodetected, otherwise user value will be used
 */
FLASH_BANK_COMMAND_HANDLER(esp32c2_flash_bank_command)
{
	struct esp32c2_flash_bank *esp32c2_info;

	if (CMD_ARGC < 6)
		return ERROR_COMMAND_SYNTAX_ERROR;

	esp32c2_info = malloc(sizeof(struct esp32c2_flash_bank));
	if (esp32c2_info == NULL)
		return ERROR_FAIL;
	int ret = esp_riscv_flash_init(&esp32c2_info->riscv,
		ESP32C2_FLASH_SECTOR_SIZE,
		algorithm_run_func_image,
		esp32c2_is_irom_address,
		esp32c2_is_drom_address,
		esp32c2_get_stub);
	if (ret != ERROR_OK) {
		free(esp32c2_info);
		return ret;
	}
	bank->driver_priv = esp32c2_info;
	return ERROR_OK;
}

static int esp32c2_get_info(struct flash_bank *bank, struct command_invocation *cmd)
{
	/* TODO: print some flash information */
	command_print_sameline(cmd, "Flash driver: ESP32-C2\n");
	return ERROR_OK;
}

static const struct command_registration esp32c2_command_handlers[] = {
	{
		.name = "esp",
		.mode = COMMAND_ANY,
		.help = "ESP flash command group",
		.usage = "",
		.chain = esp_flash_exec_flash_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

struct flash_driver esp32c2_flash = {
	.name = "esp32c2",
	.commands = esp32c2_command_handlers,
	.flash_bank_command = esp32c2_flash_bank_command,
	.erase = esp_algo_flash_erase,
	.protect = esp_algo_flash_protect,
	.write = esp_algo_flash_write,
	.read = esp_algo_flash_read,
	.probe = esp_algo_flash_probe,
	.auto_probe = esp_algo_flash_auto_probe,
	.erase_check = esp_algo_flash_blank_check,
	.protect_check = esp_algo_flash_protect_check,
	.info = esp32c2_get_info,
	.free_driver_priv = default_flash_free_driver_priv,
};
