/* SPDX-License-Identifier: GPL-2.0-or-later */

#ifndef OPENOCD_FLASH_NOR_ESP_STUB_CONFIG_H
#define OPENOCD_FLASH_NOR_ESP_STUB_CONFIG_H

#include "../../../contrib/loaders/flash/espressif/stub_flasher.h"

#ifdef ESP_TARGET_ESP32C6
#include "../../../contrib/loaders/flash/espressif/esp32c6/stub_flasher_image.h"
#define STUB_ARGS_FUNC_START            ESP_RISCV_STUB_ARGS_FUNC_START
#define STUB_STACK_DATA_POOL_SIZE       ESP_RISCV_STACK_DATA_POOL_SIZE
#define STUB_REVERSE_BINARY             false
#elif defined(ESP_TARGET_ESP32H2)
#include "../../../contrib/loaders/flash/espressif/esp32h2/stub_flasher_image.h"
#define STUB_ARGS_FUNC_START            ESP_RISCV_STUB_ARGS_FUNC_START
#define STUB_STACK_DATA_POOL_SIZE       ESP_RISCV_STACK_DATA_POOL_SIZE
#define STUB_REVERSE_BINARY             false
#elif defined(ESP_TARGET_ESP32P4)
#include "../../../contrib/loaders/flash/espressif/esp32p4/stub_flasher_image.h"
#define STUB_ARGS_FUNC_START            ESP_RISCV_STUB_ARGS_FUNC_START
#define STUB_STACK_DATA_POOL_SIZE       ESP_RISCV_STACK_DATA_POOL_SIZE
#define STUB_REVERSE_BINARY             false
#elif defined(ESP_TARGET_ESP32C2)
#include "../../../contrib/loaders/flash/espressif/esp32c2/stub_flasher_image.h"
#define STUB_ARGS_FUNC_START            ESP_RISCV_STUB_ARGS_FUNC_START
#define STUB_STACK_DATA_POOL_SIZE       ESP_RISCV_STACK_DATA_POOL_SIZE
#define STUB_REVERSE_BINARY             false
#elif defined(ESP_TARGET_ESP32C3)
#include "../../../contrib/loaders/flash/espressif/esp32c3/stub_flasher_image.h"
#define STUB_ARGS_FUNC_START            ESP_RISCV_STUB_ARGS_FUNC_START
#define STUB_STACK_DATA_POOL_SIZE       ESP_RISCV_STACK_DATA_POOL_SIZE
#define STUB_REVERSE_BINARY             false
#elif defined(ESP_TARGET_ESP32)
#include "../../../contrib/loaders/flash/espressif/esp32/stub_flasher_image.h"
#define STUB_ARGS_FUNC_START            ESP_XTENSA_STUB_ARGS_FUNC_START
#define STUB_STACK_DATA_POOL_SIZE       0
#define STUB_REVERSE_BINARY             true
#elif defined(ESP_TARGET_ESP32S2)
#include "../../../contrib/loaders/flash/espressif/esp32s2/stub_flasher_image.h"
#define STUB_ARGS_FUNC_START            ESP_XTENSA_STUB_ARGS_FUNC_START
#define STUB_STACK_DATA_POOL_SIZE       0
#define STUB_REVERSE_BINARY             false
#elif defined(ESP_TARGET_ESP32S3)
#include "../../../contrib/loaders/flash/espressif/esp32s3/stub_flasher_image.h"
#define STUB_ARGS_FUNC_START            ESP_XTENSA_STUB_ARGS_FUNC_START
#define STUB_STACK_DATA_POOL_SIZE       0
#define STUB_REVERSE_BINARY             false
#endif

#define MAKE_ESP_STUB_CFG(name, code_array, data_array, entry, bss, apptrace, log_addr, log_size, \
	first_user_reg, stack_pool_sz, reverse_bin) \
	static const struct esp_flasher_stub_config s_esp_stub_cfg_##name = { \
		.code = code_array, \
		.code_sz = sizeof(code_array), \
		.data = data_array, \
		.data_sz = sizeof(data_array), \
		.entry_addr = entry, \
		.bss_sz = bss, \
		.first_user_reg_param = first_user_reg, \
		.apptrace_ctrl_addr = apptrace, \
		.stack_data_pool_sz = stack_pool_sz, \
		.log_buff_addr = log_addr, \
		.log_buff_size = log_size, \
		.iram_org = ESP_STUB_IRAM_ORG, \
		.iram_len = ESP_STUB_IRAM_LEN, \
		.dram_org = ESP_STUB_DRAM_ORG, \
		.dram_len = ESP_STUB_DRAM_LEN, \
		.reverse = reverse_bin \
	}

#define COMMANDS \
	X(flash_read, FLASH_READ) \
	X(flash_write, FLASH_WRITE) \
	X(flash_erase, FLASH_ERASE) \
	X(flash_erase_check, FLASH_ERASE_CHECK) \
	X(flash_map_get, FLASH_MAP_GET) \
	X(flash_bp_set, FLASH_BP_SET) \
	X(flash_bp_clear, FLASH_BP_CLEAR) \
	X(flash_test, FLASH_TEST) \
	X(flash_write_deflated, FLASH_WRITE_DEFLATED) \
	X(flash_calc_hash, FLASH_CALC_HASH) \
	X(flash_clock_configure, FLASH_CLOCK_CONFIGURE) \
	X(flash_with_log, FLASH_WITH_LOG)

struct command_map {
	int command;
	const struct esp_flasher_stub_config *config;
};

#define X(name, uppercase_name) \
	MAKE_ESP_STUB_CFG(name, s_esp_flasher_stub_##name##_code, s_esp_flasher_stub_##name##_data, \
		ESP_STUB_##uppercase_name##_ENTRY_ADDR, ESP_STUB_##uppercase_name##_BSS_SIZE, \
		ESP_STUB_##uppercase_name##_APPTRACE_CTRL_ADDR, \
		ESP_STUB_##uppercase_name##_LOG_ADDR, ESP_STUB_##uppercase_name##_LOG_SIZE, \
		STUB_ARGS_FUNC_START, STUB_STACK_DATA_POOL_SIZE, STUB_REVERSE_BINARY);

/* Expand the macro for each command */
COMMANDS

#undef X

#define X(name, uppercase_name) \
	{ESP_STUB_CMD_##uppercase_name, &s_esp_stub_cfg_##name},

#define MAKE_CMD_MAP_ENTRIES \
	COMMANDS

#endif /* OPENOCD_FLASH_NOR_ESP_STUB_CONFIG_H */
