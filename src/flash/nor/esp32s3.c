/* SPDX-License-Identifier: GPL-2.0-or-later */

/**************************************************************************
 *   ESP32-S3 flash driver for OpenOCD                                     *
 *   Copyright (C) 2021 Espressif Systems Ltd.                             *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "imp.h"
#include <target/smp.h>
#include <target/xtensa/xtensa_algorithm.h>
#include <target/espressif/esp_xtensa_apptrace.h>
#include <target/espressif/esp_xtensa_smp.h>
#include "esp_xtensa.h"
#include "contrib/loaders/flash/esp/esp32s3/stub_flasher_image.h"
#include "contrib/loaders/flash/esp/esp32s3/stub_flasher_image_wlog.h"

#define ESP32_S3_DROM_LOW             0x3C000000
#define ESP32_S3_DROM_HIGH            0x3D000000
#define ESP32_S3_IROM_LOW             0x42000000
#define ESP32_S3_IROM_HIGH            0x44000000

#define ESP32_S3_FLASH_SECTOR_SIZE      4096

struct esp32s3_flash_bank {
	struct esp_xtensa_flash_bank esp_xtensa;
};

static const uint8_t esp32s3_flasher_stub_code[] = {
#include "contrib/loaders/flash/esp/esp32s3/stub_flasher_code.inc"
};
static const uint8_t esp32s3_flasher_stub_data[] = {
#include "contrib/loaders/flash/esp/esp32s3/stub_flasher_data.inc"
};
static const uint8_t esp32s3_flasher_stub_code_wlog[] = {
#include "contrib/loaders/flash/esp/esp32s3/stub_flasher_code_wlog.inc"
};
static const uint8_t esp32s3_flasher_stub_data_wlog[] = {
#include "contrib/loaders/flash/esp/esp32s3/stub_flasher_data_wlog.inc"
};

static const struct esp_flasher_stub_config s_esp32s3_stub_cfg = {
	.code = esp32s3_flasher_stub_code,
	.code_sz = sizeof(esp32s3_flasher_stub_code),
	.data = esp32s3_flasher_stub_data,
	.data_sz = sizeof(esp32s3_flasher_stub_data),
	.entry_addr = ESP32S3_STUB_ENTRY_ADDR,
	.bss_sz = ESP32S3_STUB_BSS_SIZE,
	.iram_org = ESP32S3_STUB_IRAM_ORG,
	.iram_len = ESP32S3_STUB_IRAM_LEN,
	.dram_org = ESP32S3_STUB_DRAM_ORG,
	.dram_len = ESP32S3_STUB_DRAM_LEN,
	.first_user_reg_param = XTENSA_STUB_ARGS_FUNC_START
};

static const struct esp_flasher_stub_config s_esp32s3_stub_cfg_wlog = {
	.code = esp32s3_flasher_stub_code_wlog,
	.code_sz = sizeof(esp32s3_flasher_stub_code_wlog),
	.data = esp32s3_flasher_stub_data_wlog,
	.data_sz = sizeof(esp32s3_flasher_stub_data_wlog),
	.entry_addr = ESP32S3_STUB_WLOG_ENTRY_ADDR,
	.bss_sz = ESP32S3_STUB_WLOG_BSS_SIZE,
	.iram_org = ESP32S3_STUB_IRAM_ORG,
	.iram_len = ESP32S3_STUB_IRAM_LEN,
	.dram_org = ESP32S3_STUB_DRAM_ORG,
	.dram_len = ESP32S3_STUB_DRAM_LEN,
	.first_user_reg_param = XTENSA_STUB_ARGS_FUNC_START,
	.log_buff_addr = ESP32S3_STUB_WLOG_LOG_ADDR,
	.log_buff_size = ESP32S3_STUB_WLOG_LOG_SIZE
};

static bool esp32s3_is_irom_address(target_addr_t addr)
{
	return addr >= ESP32_S3_IROM_LOW && addr < ESP32_S3_IROM_HIGH;
}

static bool esp32s3_is_drom_address(target_addr_t addr)
{
	return addr >= ESP32_S3_DROM_LOW && addr < ESP32_S3_DROM_HIGH;
}

static const struct esp_flasher_stub_config *esp32s3_get_stub(struct flash_bank *bank)
{
	struct esp_flash_bank *esp_info = bank->driver_priv;
	if (esp_info->stub_log_enabled)
		return &s_esp32s3_stub_cfg_wlog;
	return &s_esp32s3_stub_cfg;
}

/* flash bank <bank_name> esp32s3 <base> <size> 0 0 <target#>
   If <size> is zero flash size will be autodetected, otherwise user value will be used
 */
FLASH_BANK_COMMAND_HANDLER(esp32s3_flash_bank_command)
{
	struct esp32s3_flash_bank *esp32s3_info;

	if (CMD_ARGC < 6)
		return ERROR_COMMAND_SYNTAX_ERROR;

	esp32s3_info = malloc(sizeof(struct esp32s3_flash_bank));
	if (esp32s3_info == NULL)
		return ERROR_FAIL;
	int ret = esp_xtensa_flash_init(&esp32s3_info->esp_xtensa,
		ESP32_S3_FLASH_SECTOR_SIZE,
		esp_xtensa_smp_run_func_image,
		esp32s3_is_irom_address,
		esp32s3_is_drom_address,
		esp32s3_get_stub);
	if (ret != ERROR_OK) {
		free(esp32s3_info);
		return ret;
	}
	bank->driver_priv = esp32s3_info;
	return ERROR_OK;
}

static int esp32s3_get_info(struct flash_bank *bank, struct command_invocation *cmd)
{
	/* TODO: print some flash information */
	command_print_sameline(cmd, "Flash driver: ESP32-S3\n");
	return ERROR_OK;
}

COMMAND_HANDLER(esp32s3_cmd_appimage_flashoff)
{
	struct target *target = get_current_target(CMD_CTX);

	if (target->smp) {
		struct target_list *head;
		struct target *curr;
		foreach_smp_target(head, target->smp_targets) {
			curr = head->target;
			int ret = CALL_COMMAND_HANDLER(esp_algo_flash_cmd_appimage_flashoff_do, curr);
			if (ret != ERROR_OK)
				return ret;
		}
		return ERROR_OK;
	}
	return CALL_COMMAND_HANDLER(esp_algo_flash_cmd_appimage_flashoff_do, target);
}

COMMAND_HANDLER(esp32s3_cmd_compression)
{
	struct target *target = get_current_target(CMD_CTX);

	if (target->smp) {
		struct target_list *head;
		struct target *curr;
		foreach_smp_target(head, target->smp_targets) {
			curr = head->target;
			int ret = CALL_COMMAND_HANDLER(esp_algo_flash_cmd_set_compression, curr);
			if (ret != ERROR_OK)
				return ret;
		}
		return ERROR_OK;
	}
	return CALL_COMMAND_HANDLER(esp_algo_flash_cmd_set_compression, target);
}

COMMAND_HANDLER(esp32s3_cmd_encryption)
{
	struct target *target = get_current_target(CMD_CTX);

	if (target->smp) {
		struct target_list *head;
		struct target *curr;
		foreach_smp_target(head, target->smp_targets) {
			curr = head->target;
			int ret = CALL_COMMAND_HANDLER(esp_algo_flash_cmd_set_encryption, curr);
			if (ret != ERROR_OK)
				return ret;
		}
		return ERROR_OK;
	}
	return CALL_COMMAND_HANDLER(esp_algo_flash_cmd_set_encryption, target);
}

COMMAND_HANDLER(esp32s3_cmd_stub_log)
{
	struct target *target = get_current_target(CMD_CTX);

	if (target->smp) {
		struct target_list *head;
		struct target *curr;
		foreach_smp_target(head, target->smp_targets) {
			curr = head->target;
			int ret = CALL_COMMAND_HANDLER(esp_algo_flash_parse_cmd_stub_log, curr);
			if (ret != ERROR_OK)
				return ret;
		}
		return ERROR_OK;
	}
	return CALL_COMMAND_HANDLER(esp_algo_flash_parse_cmd_stub_log, target);
}

COMMAND_HANDLER(esp32s3_cmd_verify_bank_hash)
{
	return CALL_COMMAND_HANDLER(esp_algo_flash_parse_cmd_verify_bank_hash,
		get_current_target(CMD_CTX));
}

COMMAND_HANDLER(esp32s3_cmd_set_clock)
{
	return CALL_COMMAND_HANDLER(esp_algo_flash_parse_cmd_clock_boost,
		get_current_target(CMD_CTX));
}

static const struct command_registration esp32s3_flash_command_handlers[] = {
	{
		.name = "appimage_offset",
		.handler = esp32s3_cmd_appimage_flashoff,
		.mode = COMMAND_ANY,
		.help =
			"Set offset of application image in flash. Use -1 to debug the first application image from partition table.",
		.usage = "offset",
	},
	{
		.name = "compression",
		.handler = esp32s3_cmd_compression,
		.mode = COMMAND_ANY,
		.help =
			"Set compression flag",
		.usage = "['on'|'off']",
	},
	{
		.name = "verify_bank_hash",
		.handler = esp32s3_cmd_verify_bank_hash,
		.mode = COMMAND_ANY,
		.usage = "bank_id filename [offset]",
		.help = "Perform a comparison between the file and the contents of the "
			"flash bank using SHA256 hash values. Allow optional offset from beginning of the bank "
			"(defaults to zero).",
	},
	{
		.name = "flash_stub_clock_boost",
		.handler = esp32s3_cmd_set_clock,
		.mode = COMMAND_ANY,
		.help =
			"Set cpu clock freq to the max level. Use 'off' to restore the clock speed",
		.usage = "['on'|'off']",
	},
	{
		.name = "encrypt_binary",
		.handler = esp32s3_cmd_encryption,
		.mode = COMMAND_ANY,
		.help =
			"Set if binary encryption needs to be handled on chip before writing to flash",
		.usage = "['yes'|'no']",
	},
	{
		.name = "stub_log",
		.handler = esp32s3_cmd_stub_log,
		.mode = COMMAND_ANY,
		.help = "Enable stub flasher logs",
		.usage = "['on'|'off']",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration esp32s3_command_handlers[] = {
	{
		.name = "esp",
		.mode = COMMAND_ANY,
		.help = "ESP flash command group",
		.usage = "",
		.chain = esp32s3_flash_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration esp32s3_legacy_command_handlers[] = {
	{
		.name = "esp32s3",
		.mode = COMMAND_ANY,
		.help = "ESP32_S3 flash command group",
		.usage = "",
		.chain = esp32s3_flash_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration esp32s3_all_command_handlers[] = {
	{
		.usage = "",
		.chain = esp32s3_command_handlers,
	},
	{
		.usage = "",
		.chain = esp32s3_legacy_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

const struct flash_driver esp32s3_flash = {
	.name = "esp32s3",
	.commands = esp32s3_all_command_handlers,
	.flash_bank_command = esp32s3_flash_bank_command,
	.erase = esp_algo_flash_erase,
	.protect = esp_algo_flash_protect,
	.write = esp_algo_flash_write,
	.read = esp_algo_flash_read,
	.probe = esp_algo_flash_probe,
	.auto_probe = esp_algo_flash_auto_probe,
	.erase_check = esp_algo_flash_blank_check,
	.protect_check = esp_algo_flash_protect_check,
	.info = esp32s3_get_info,
	.free_driver_priv = default_flash_free_driver_priv,
};
