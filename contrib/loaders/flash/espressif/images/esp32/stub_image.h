/* SPDX-License-Identifier: Apache-2.0 OR MIT */
/* SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD */

#pragma once

#include <stdint.h>

#define ESP_STUB_REVERSE_BINARY 1
#define ESP_STUB_STACK_SIZE 512

// cmd_test1 definitions
#define ESP_STUB_CMD_TEST1_BSS_SIZE 0x0UL
#define ESP_STUB_CMD_TEST1_IRAM_ORG 0x400bc000UL
#define ESP_STUB_CMD_TEST1_IRAM_LEN 0x00004000UL
#define ESP_STUB_CMD_TEST1_DRAM_ORG 0x3ffe4000UL
#define ESP_STUB_CMD_TEST1_DRAM_LEN 0x00014000UL
#define ESP_STUB_CMD_TEST1_ENTRY_ADDR 0x400bc08cUL
#define ESP_STUB_CMD_TEST1_APPTRACE_CTRL_ADDR 0x0UL
#define ESP_STUB_CMD_TEST1_LOG_ADDR 0x0UL
#define ESP_STUB_CMD_TEST1_LOG_SIZE 0x0UL

static const uint8_t s_esp_flasher_stub_cmd_test1_code[] = {
#include "contrib/loaders/flash/espressif/images/esp32/stub_cmd_test1_code.inc"
};

static const uint8_t s_esp_flasher_stub_cmd_test1_data[] = {
#include "contrib/loaders/flash/espressif/images/esp32/stub_cmd_test1_data.inc"
};

static const struct esp_flasher_stub_config s_esp_stub_cmd_test1_cfg = {
	.code = s_esp_flasher_stub_cmd_test1_code,
	.code_sz = sizeof(s_esp_flasher_stub_cmd_test1_code),
	.data = s_esp_flasher_stub_cmd_test1_data,
	.data_sz = sizeof(s_esp_flasher_stub_cmd_test1_data),
	.entry_addr = ESP_STUB_CMD_TEST1_ENTRY_ADDR,
	.bss_sz = ESP_STUB_CMD_TEST1_BSS_SIZE,
	.apptrace_ctrl_addr = ESP_STUB_CMD_TEST1_APPTRACE_CTRL_ADDR,
	.stack_default_sz = ESP_STUB_STACK_SIZE,
	.log_buff_addr = ESP_STUB_CMD_TEST1_LOG_ADDR,
	.log_buff_size = ESP_STUB_CMD_TEST1_LOG_SIZE,
	.iram_org = ESP_STUB_CMD_TEST1_IRAM_ORG,
	.iram_len = ESP_STUB_CMD_TEST1_IRAM_LEN,
	.dram_org = ESP_STUB_CMD_TEST1_DRAM_ORG,
	.dram_len = ESP_STUB_CMD_TEST1_DRAM_LEN,
	.reverse = ESP_STUB_REVERSE_BINARY,
};

// cmd_recv_from_host definitions
#define ESP_STUB_CMD_RECV_FROM_HOST_BSS_SIZE 0x00002cUL
#define ESP_STUB_CMD_RECV_FROM_HOST_IRAM_ORG 0x400bc000UL
#define ESP_STUB_CMD_RECV_FROM_HOST_IRAM_LEN 0x00004000UL
#define ESP_STUB_CMD_RECV_FROM_HOST_DRAM_ORG 0x3ffe4000UL
#define ESP_STUB_CMD_RECV_FROM_HOST_DRAM_LEN 0x00014000UL
#define ESP_STUB_CMD_RECV_FROM_HOST_ENTRY_ADDR 0x400bc30cUL
#define ESP_STUB_CMD_RECV_FROM_HOST_APPTRACE_CTRL_ADDR 0x0UL
#define ESP_STUB_CMD_RECV_FROM_HOST_LOG_ADDR 0x0UL
#define ESP_STUB_CMD_RECV_FROM_HOST_LOG_SIZE 0x0UL

static const uint8_t s_esp_flasher_stub_cmd_recv_from_host_code[] = {
#include "contrib/loaders/flash/espressif/images/esp32/stub_cmd_recv_from_host_code.inc"
};

static const uint8_t s_esp_flasher_stub_cmd_recv_from_host_data[] = {
#include "contrib/loaders/flash/espressif/images/esp32/stub_cmd_recv_from_host_data.inc"
};

static const struct esp_flasher_stub_config s_esp_stub_cmd_recv_from_host_cfg = {
	.code = s_esp_flasher_stub_cmd_recv_from_host_code,
	.code_sz = sizeof(s_esp_flasher_stub_cmd_recv_from_host_code),
	.data = s_esp_flasher_stub_cmd_recv_from_host_data,
	.data_sz = sizeof(s_esp_flasher_stub_cmd_recv_from_host_data),
	.entry_addr = ESP_STUB_CMD_RECV_FROM_HOST_ENTRY_ADDR,
	.bss_sz = ESP_STUB_CMD_RECV_FROM_HOST_BSS_SIZE,
	.apptrace_ctrl_addr = ESP_STUB_CMD_RECV_FROM_HOST_APPTRACE_CTRL_ADDR,
	.stack_default_sz = ESP_STUB_STACK_SIZE,
	.log_buff_addr = ESP_STUB_CMD_RECV_FROM_HOST_LOG_ADDR,
	.log_buff_size = ESP_STUB_CMD_RECV_FROM_HOST_LOG_SIZE,
	.iram_org = ESP_STUB_CMD_RECV_FROM_HOST_IRAM_ORG,
	.iram_len = ESP_STUB_CMD_RECV_FROM_HOST_IRAM_LEN,
	.dram_org = ESP_STUB_CMD_RECV_FROM_HOST_DRAM_ORG,
	.dram_len = ESP_STUB_CMD_RECV_FROM_HOST_DRAM_LEN,
	.reverse = ESP_STUB_REVERSE_BINARY,
};

// cmd_send_to_host definitions
#define ESP_STUB_CMD_SEND_TO_HOST_BSS_SIZE 0x00002dUL
#define ESP_STUB_CMD_SEND_TO_HOST_IRAM_ORG 0x400bc000UL
#define ESP_STUB_CMD_SEND_TO_HOST_IRAM_LEN 0x00004000UL
#define ESP_STUB_CMD_SEND_TO_HOST_DRAM_ORG 0x3ffe4000UL
#define ESP_STUB_CMD_SEND_TO_HOST_DRAM_LEN 0x00014000UL
#define ESP_STUB_CMD_SEND_TO_HOST_ENTRY_ADDR 0x400bc374UL
#define ESP_STUB_CMD_SEND_TO_HOST_APPTRACE_CTRL_ADDR 0x0UL
#define ESP_STUB_CMD_SEND_TO_HOST_LOG_ADDR 0x0UL
#define ESP_STUB_CMD_SEND_TO_HOST_LOG_SIZE 0x0UL

static const uint8_t s_esp_flasher_stub_cmd_send_to_host_code[] = {
#include "contrib/loaders/flash/espressif/images/esp32/stub_cmd_send_to_host_code.inc"
};

static const uint8_t s_esp_flasher_stub_cmd_send_to_host_data[] = {
#include "contrib/loaders/flash/espressif/images/esp32/stub_cmd_send_to_host_data.inc"
};

static const struct esp_flasher_stub_config s_esp_stub_cmd_send_to_host_cfg = {
	.code = s_esp_flasher_stub_cmd_send_to_host_code,
	.code_sz = sizeof(s_esp_flasher_stub_cmd_send_to_host_code),
	.data = s_esp_flasher_stub_cmd_send_to_host_data,
	.data_sz = sizeof(s_esp_flasher_stub_cmd_send_to_host_data),
	.entry_addr = ESP_STUB_CMD_SEND_TO_HOST_ENTRY_ADDR,
	.bss_sz = ESP_STUB_CMD_SEND_TO_HOST_BSS_SIZE,
	.apptrace_ctrl_addr = ESP_STUB_CMD_SEND_TO_HOST_APPTRACE_CTRL_ADDR,
	.stack_default_sz = ESP_STUB_STACK_SIZE,
	.log_buff_addr = ESP_STUB_CMD_SEND_TO_HOST_LOG_ADDR,
	.log_buff_size = ESP_STUB_CMD_SEND_TO_HOST_LOG_SIZE,
	.iram_org = ESP_STUB_CMD_SEND_TO_HOST_IRAM_ORG,
	.iram_len = ESP_STUB_CMD_SEND_TO_HOST_IRAM_LEN,
	.dram_org = ESP_STUB_CMD_SEND_TO_HOST_DRAM_ORG,
	.dram_len = ESP_STUB_CMD_SEND_TO_HOST_DRAM_LEN,
	.reverse = ESP_STUB_REVERSE_BINARY,
};

// cmd_flash_read definitions
#define ESP_STUB_CMD_FLASH_READ_BSS_SIZE 0x0UL
#define ESP_STUB_CMD_FLASH_READ_IRAM_ORG 0x400bc000UL
#define ESP_STUB_CMD_FLASH_READ_IRAM_LEN 0x00004000UL
#define ESP_STUB_CMD_FLASH_READ_DRAM_ORG 0x3ffe4000UL
#define ESP_STUB_CMD_FLASH_READ_DRAM_LEN 0x00014000UL
#define ESP_STUB_CMD_FLASH_READ_ENTRY_ADDR 0x400bc08cUL
#define ESP_STUB_CMD_FLASH_READ_APPTRACE_CTRL_ADDR 0x0UL
#define ESP_STUB_CMD_FLASH_READ_LOG_ADDR 0x0UL
#define ESP_STUB_CMD_FLASH_READ_LOG_SIZE 0x0UL

static const uint8_t s_esp_flasher_stub_cmd_flash_read_code[] = {
#include "contrib/loaders/flash/espressif/images/esp32/stub_cmd_flash_read_code.inc"
};

static const uint8_t s_esp_flasher_stub_cmd_flash_read_data[] = {
#include "contrib/loaders/flash/espressif/images/esp32/stub_cmd_flash_read_data.inc"
};

static const struct esp_flasher_stub_config s_esp_stub_cmd_flash_read_cfg = {
	.code = s_esp_flasher_stub_cmd_flash_read_code,
	.code_sz = sizeof(s_esp_flasher_stub_cmd_flash_read_code),
	.data = s_esp_flasher_stub_cmd_flash_read_data,
	.data_sz = sizeof(s_esp_flasher_stub_cmd_flash_read_data),
	.entry_addr = ESP_STUB_CMD_FLASH_READ_ENTRY_ADDR,
	.bss_sz = ESP_STUB_CMD_FLASH_READ_BSS_SIZE,
	.apptrace_ctrl_addr = ESP_STUB_CMD_FLASH_READ_APPTRACE_CTRL_ADDR,
	.stack_default_sz = ESP_STUB_STACK_SIZE,
	.log_buff_addr = ESP_STUB_CMD_FLASH_READ_LOG_ADDR,
	.log_buff_size = ESP_STUB_CMD_FLASH_READ_LOG_SIZE,
	.iram_org = ESP_STUB_CMD_FLASH_READ_IRAM_ORG,
	.iram_len = ESP_STUB_CMD_FLASH_READ_IRAM_LEN,
	.dram_org = ESP_STUB_CMD_FLASH_READ_DRAM_ORG,
	.dram_len = ESP_STUB_CMD_FLASH_READ_DRAM_LEN,
	.reverse = ESP_STUB_REVERSE_BINARY,
};

// cmd_flash_write definitions
#define ESP_STUB_CMD_FLASH_WRITE_BSS_SIZE 0x0UL
#define ESP_STUB_CMD_FLASH_WRITE_IRAM_ORG 0x400bc000UL
#define ESP_STUB_CMD_FLASH_WRITE_IRAM_LEN 0x00004000UL
#define ESP_STUB_CMD_FLASH_WRITE_DRAM_ORG 0x3ffe4000UL
#define ESP_STUB_CMD_FLASH_WRITE_DRAM_LEN 0x00014000UL
#define ESP_STUB_CMD_FLASH_WRITE_ENTRY_ADDR 0x400bc08cUL
#define ESP_STUB_CMD_FLASH_WRITE_APPTRACE_CTRL_ADDR 0x0UL
#define ESP_STUB_CMD_FLASH_WRITE_LOG_ADDR 0x0UL
#define ESP_STUB_CMD_FLASH_WRITE_LOG_SIZE 0x0UL

static const uint8_t s_esp_flasher_stub_cmd_flash_write_code[] = {
#include "contrib/loaders/flash/espressif/images/esp32/stub_cmd_flash_write_code.inc"
};

static const uint8_t s_esp_flasher_stub_cmd_flash_write_data[] = {
#include "contrib/loaders/flash/espressif/images/esp32/stub_cmd_flash_write_data.inc"
};

static const struct esp_flasher_stub_config s_esp_stub_cmd_flash_write_cfg = {
	.code = s_esp_flasher_stub_cmd_flash_write_code,
	.code_sz = sizeof(s_esp_flasher_stub_cmd_flash_write_code),
	.data = s_esp_flasher_stub_cmd_flash_write_data,
	.data_sz = sizeof(s_esp_flasher_stub_cmd_flash_write_data),
	.entry_addr = ESP_STUB_CMD_FLASH_WRITE_ENTRY_ADDR,
	.bss_sz = ESP_STUB_CMD_FLASH_WRITE_BSS_SIZE,
	.apptrace_ctrl_addr = ESP_STUB_CMD_FLASH_WRITE_APPTRACE_CTRL_ADDR,
	.stack_default_sz = ESP_STUB_STACK_SIZE,
	.log_buff_addr = ESP_STUB_CMD_FLASH_WRITE_LOG_ADDR,
	.log_buff_size = ESP_STUB_CMD_FLASH_WRITE_LOG_SIZE,
	.iram_org = ESP_STUB_CMD_FLASH_WRITE_IRAM_ORG,
	.iram_len = ESP_STUB_CMD_FLASH_WRITE_IRAM_LEN,
	.dram_org = ESP_STUB_CMD_FLASH_WRITE_DRAM_ORG,
	.dram_len = ESP_STUB_CMD_FLASH_WRITE_DRAM_LEN,
	.reverse = ESP_STUB_REVERSE_BINARY,
};

// cmd_flash_erase definitions
#define ESP_STUB_CMD_FLASH_ERASE_BSS_SIZE 0x0UL
#define ESP_STUB_CMD_FLASH_ERASE_IRAM_ORG 0x400bc000UL
#define ESP_STUB_CMD_FLASH_ERASE_IRAM_LEN 0x00004000UL
#define ESP_STUB_CMD_FLASH_ERASE_DRAM_ORG 0x3ffe4000UL
#define ESP_STUB_CMD_FLASH_ERASE_DRAM_LEN 0x00014000UL
#define ESP_STUB_CMD_FLASH_ERASE_ENTRY_ADDR 0x400bc08cUL
#define ESP_STUB_CMD_FLASH_ERASE_APPTRACE_CTRL_ADDR 0x0UL
#define ESP_STUB_CMD_FLASH_ERASE_LOG_ADDR 0x0UL
#define ESP_STUB_CMD_FLASH_ERASE_LOG_SIZE 0x0UL

static const uint8_t s_esp_flasher_stub_cmd_flash_erase_code[] = {
#include "contrib/loaders/flash/espressif/images/esp32/stub_cmd_flash_erase_code.inc"
};

static const uint8_t s_esp_flasher_stub_cmd_flash_erase_data[] = {
#include "contrib/loaders/flash/espressif/images/esp32/stub_cmd_flash_erase_data.inc"
};

static const struct esp_flasher_stub_config s_esp_stub_cmd_flash_erase_cfg = {
	.code = s_esp_flasher_stub_cmd_flash_erase_code,
	.code_sz = sizeof(s_esp_flasher_stub_cmd_flash_erase_code),
	.data = s_esp_flasher_stub_cmd_flash_erase_data,
	.data_sz = sizeof(s_esp_flasher_stub_cmd_flash_erase_data),
	.entry_addr = ESP_STUB_CMD_FLASH_ERASE_ENTRY_ADDR,
	.bss_sz = ESP_STUB_CMD_FLASH_ERASE_BSS_SIZE,
	.apptrace_ctrl_addr = ESP_STUB_CMD_FLASH_ERASE_APPTRACE_CTRL_ADDR,
	.stack_default_sz = ESP_STUB_STACK_SIZE,
	.log_buff_addr = ESP_STUB_CMD_FLASH_ERASE_LOG_ADDR,
	.log_buff_size = ESP_STUB_CMD_FLASH_ERASE_LOG_SIZE,
	.iram_org = ESP_STUB_CMD_FLASH_ERASE_IRAM_ORG,
	.iram_len = ESP_STUB_CMD_FLASH_ERASE_IRAM_LEN,
	.dram_org = ESP_STUB_CMD_FLASH_ERASE_DRAM_ORG,
	.dram_len = ESP_STUB_CMD_FLASH_ERASE_DRAM_LEN,
	.reverse = ESP_STUB_REVERSE_BINARY,
};

// cmd_flash_erase_check definitions
#define ESP_STUB_CMD_FLASH_ERASE_CHECK_BSS_SIZE 0x0UL
#define ESP_STUB_CMD_FLASH_ERASE_CHECK_IRAM_ORG 0x400bc000UL
#define ESP_STUB_CMD_FLASH_ERASE_CHECK_IRAM_LEN 0x00004000UL
#define ESP_STUB_CMD_FLASH_ERASE_CHECK_DRAM_ORG 0x3ffe4000UL
#define ESP_STUB_CMD_FLASH_ERASE_CHECK_DRAM_LEN 0x00014000UL
#define ESP_STUB_CMD_FLASH_ERASE_CHECK_ENTRY_ADDR 0x400bc08cUL
#define ESP_STUB_CMD_FLASH_ERASE_CHECK_APPTRACE_CTRL_ADDR 0x0UL
#define ESP_STUB_CMD_FLASH_ERASE_CHECK_LOG_ADDR 0x0UL
#define ESP_STUB_CMD_FLASH_ERASE_CHECK_LOG_SIZE 0x0UL

static const uint8_t s_esp_flasher_stub_cmd_flash_erase_check_code[] = {
#include "contrib/loaders/flash/espressif/images/esp32/stub_cmd_flash_erase_check_code.inc"
};

static const uint8_t s_esp_flasher_stub_cmd_flash_erase_check_data[] = {
#include "contrib/loaders/flash/espressif/images/esp32/stub_cmd_flash_erase_check_data.inc"
};

static const struct esp_flasher_stub_config s_esp_stub_cmd_flash_erase_check_cfg = {
	.code = s_esp_flasher_stub_cmd_flash_erase_check_code,
	.code_sz = sizeof(s_esp_flasher_stub_cmd_flash_erase_check_code),
	.data = s_esp_flasher_stub_cmd_flash_erase_check_data,
	.data_sz = sizeof(s_esp_flasher_stub_cmd_flash_erase_check_data),
	.entry_addr = ESP_STUB_CMD_FLASH_ERASE_CHECK_ENTRY_ADDR,
	.bss_sz = ESP_STUB_CMD_FLASH_ERASE_CHECK_BSS_SIZE,
	.apptrace_ctrl_addr = ESP_STUB_CMD_FLASH_ERASE_CHECK_APPTRACE_CTRL_ADDR,
	.stack_default_sz = ESP_STUB_STACK_SIZE,
	.log_buff_addr = ESP_STUB_CMD_FLASH_ERASE_CHECK_LOG_ADDR,
	.log_buff_size = ESP_STUB_CMD_FLASH_ERASE_CHECK_LOG_SIZE,
	.iram_org = ESP_STUB_CMD_FLASH_ERASE_CHECK_IRAM_ORG,
	.iram_len = ESP_STUB_CMD_FLASH_ERASE_CHECK_IRAM_LEN,
	.dram_org = ESP_STUB_CMD_FLASH_ERASE_CHECK_DRAM_ORG,
	.dram_len = ESP_STUB_CMD_FLASH_ERASE_CHECK_DRAM_LEN,
	.reverse = ESP_STUB_REVERSE_BINARY,
};

// cmd_flash_map_get definitions
#define ESP_STUB_CMD_FLASH_MAP_GET_BSS_SIZE 0x0UL
#define ESP_STUB_CMD_FLASH_MAP_GET_IRAM_ORG 0x400bc000UL
#define ESP_STUB_CMD_FLASH_MAP_GET_IRAM_LEN 0x00004000UL
#define ESP_STUB_CMD_FLASH_MAP_GET_DRAM_ORG 0x3ffe4000UL
#define ESP_STUB_CMD_FLASH_MAP_GET_DRAM_LEN 0x00014000UL
#define ESP_STUB_CMD_FLASH_MAP_GET_ENTRY_ADDR 0x400bc08cUL
#define ESP_STUB_CMD_FLASH_MAP_GET_APPTRACE_CTRL_ADDR 0x0UL
#define ESP_STUB_CMD_FLASH_MAP_GET_LOG_ADDR 0x0UL
#define ESP_STUB_CMD_FLASH_MAP_GET_LOG_SIZE 0x0UL

static const uint8_t s_esp_flasher_stub_cmd_flash_map_get_code[] = {
#include "contrib/loaders/flash/espressif/images/esp32/stub_cmd_flash_map_get_code.inc"
};

static const uint8_t s_esp_flasher_stub_cmd_flash_map_get_data[] = {
#include "contrib/loaders/flash/espressif/images/esp32/stub_cmd_flash_map_get_data.inc"
};

static const struct esp_flasher_stub_config s_esp_stub_cmd_flash_map_get_cfg = {
	.code = s_esp_flasher_stub_cmd_flash_map_get_code,
	.code_sz = sizeof(s_esp_flasher_stub_cmd_flash_map_get_code),
	.data = s_esp_flasher_stub_cmd_flash_map_get_data,
	.data_sz = sizeof(s_esp_flasher_stub_cmd_flash_map_get_data),
	.entry_addr = ESP_STUB_CMD_FLASH_MAP_GET_ENTRY_ADDR,
	.bss_sz = ESP_STUB_CMD_FLASH_MAP_GET_BSS_SIZE,
	.apptrace_ctrl_addr = ESP_STUB_CMD_FLASH_MAP_GET_APPTRACE_CTRL_ADDR,
	.stack_default_sz = ESP_STUB_STACK_SIZE,
	.log_buff_addr = ESP_STUB_CMD_FLASH_MAP_GET_LOG_ADDR,
	.log_buff_size = ESP_STUB_CMD_FLASH_MAP_GET_LOG_SIZE,
	.iram_org = ESP_STUB_CMD_FLASH_MAP_GET_IRAM_ORG,
	.iram_len = ESP_STUB_CMD_FLASH_MAP_GET_IRAM_LEN,
	.dram_org = ESP_STUB_CMD_FLASH_MAP_GET_DRAM_ORG,
	.dram_len = ESP_STUB_CMD_FLASH_MAP_GET_DRAM_LEN,
	.reverse = ESP_STUB_REVERSE_BINARY,
};

// cmd_flash_bp_set definitions
#define ESP_STUB_CMD_FLASH_BP_SET_BSS_SIZE 0x0UL
#define ESP_STUB_CMD_FLASH_BP_SET_IRAM_ORG 0x400bc000UL
#define ESP_STUB_CMD_FLASH_BP_SET_IRAM_LEN 0x00004000UL
#define ESP_STUB_CMD_FLASH_BP_SET_DRAM_ORG 0x3ffe4000UL
#define ESP_STUB_CMD_FLASH_BP_SET_DRAM_LEN 0x00014000UL
#define ESP_STUB_CMD_FLASH_BP_SET_ENTRY_ADDR 0x400bc08cUL
#define ESP_STUB_CMD_FLASH_BP_SET_APPTRACE_CTRL_ADDR 0x0UL
#define ESP_STUB_CMD_FLASH_BP_SET_LOG_ADDR 0x0UL
#define ESP_STUB_CMD_FLASH_BP_SET_LOG_SIZE 0x0UL

static const uint8_t s_esp_flasher_stub_cmd_flash_bp_set_code[] = {
#include "contrib/loaders/flash/espressif/images/esp32/stub_cmd_flash_bp_set_code.inc"
};

static const uint8_t s_esp_flasher_stub_cmd_flash_bp_set_data[] = {
#include "contrib/loaders/flash/espressif/images/esp32/stub_cmd_flash_bp_set_data.inc"
};

static const struct esp_flasher_stub_config s_esp_stub_cmd_flash_bp_set_cfg = {
	.code = s_esp_flasher_stub_cmd_flash_bp_set_code,
	.code_sz = sizeof(s_esp_flasher_stub_cmd_flash_bp_set_code),
	.data = s_esp_flasher_stub_cmd_flash_bp_set_data,
	.data_sz = sizeof(s_esp_flasher_stub_cmd_flash_bp_set_data),
	.entry_addr = ESP_STUB_CMD_FLASH_BP_SET_ENTRY_ADDR,
	.bss_sz = ESP_STUB_CMD_FLASH_BP_SET_BSS_SIZE,
	.apptrace_ctrl_addr = ESP_STUB_CMD_FLASH_BP_SET_APPTRACE_CTRL_ADDR,
	.stack_default_sz = ESP_STUB_STACK_SIZE,
	.log_buff_addr = ESP_STUB_CMD_FLASH_BP_SET_LOG_ADDR,
	.log_buff_size = ESP_STUB_CMD_FLASH_BP_SET_LOG_SIZE,
	.iram_org = ESP_STUB_CMD_FLASH_BP_SET_IRAM_ORG,
	.iram_len = ESP_STUB_CMD_FLASH_BP_SET_IRAM_LEN,
	.dram_org = ESP_STUB_CMD_FLASH_BP_SET_DRAM_ORG,
	.dram_len = ESP_STUB_CMD_FLASH_BP_SET_DRAM_LEN,
	.reverse = ESP_STUB_REVERSE_BINARY,
};

// cmd_flash_bp_clear definitions
#define ESP_STUB_CMD_FLASH_BP_CLEAR_BSS_SIZE 0x0UL
#define ESP_STUB_CMD_FLASH_BP_CLEAR_IRAM_ORG 0x400bc000UL
#define ESP_STUB_CMD_FLASH_BP_CLEAR_IRAM_LEN 0x00004000UL
#define ESP_STUB_CMD_FLASH_BP_CLEAR_DRAM_ORG 0x3ffe4000UL
#define ESP_STUB_CMD_FLASH_BP_CLEAR_DRAM_LEN 0x00014000UL
#define ESP_STUB_CMD_FLASH_BP_CLEAR_ENTRY_ADDR 0x400bc08cUL
#define ESP_STUB_CMD_FLASH_BP_CLEAR_APPTRACE_CTRL_ADDR 0x0UL
#define ESP_STUB_CMD_FLASH_BP_CLEAR_LOG_ADDR 0x0UL
#define ESP_STUB_CMD_FLASH_BP_CLEAR_LOG_SIZE 0x0UL

static const uint8_t s_esp_flasher_stub_cmd_flash_bp_clear_code[] = {
#include "contrib/loaders/flash/espressif/images/esp32/stub_cmd_flash_bp_clear_code.inc"
};

static const uint8_t s_esp_flasher_stub_cmd_flash_bp_clear_data[] = {
#include "contrib/loaders/flash/espressif/images/esp32/stub_cmd_flash_bp_clear_data.inc"
};

static const struct esp_flasher_stub_config s_esp_stub_cmd_flash_bp_clear_cfg = {
	.code = s_esp_flasher_stub_cmd_flash_bp_clear_code,
	.code_sz = sizeof(s_esp_flasher_stub_cmd_flash_bp_clear_code),
	.data = s_esp_flasher_stub_cmd_flash_bp_clear_data,
	.data_sz = sizeof(s_esp_flasher_stub_cmd_flash_bp_clear_data),
	.entry_addr = ESP_STUB_CMD_FLASH_BP_CLEAR_ENTRY_ADDR,
	.bss_sz = ESP_STUB_CMD_FLASH_BP_CLEAR_BSS_SIZE,
	.apptrace_ctrl_addr = ESP_STUB_CMD_FLASH_BP_CLEAR_APPTRACE_CTRL_ADDR,
	.stack_default_sz = ESP_STUB_STACK_SIZE,
	.log_buff_addr = ESP_STUB_CMD_FLASH_BP_CLEAR_LOG_ADDR,
	.log_buff_size = ESP_STUB_CMD_FLASH_BP_CLEAR_LOG_SIZE,
	.iram_org = ESP_STUB_CMD_FLASH_BP_CLEAR_IRAM_ORG,
	.iram_len = ESP_STUB_CMD_FLASH_BP_CLEAR_IRAM_LEN,
	.dram_org = ESP_STUB_CMD_FLASH_BP_CLEAR_DRAM_ORG,
	.dram_len = ESP_STUB_CMD_FLASH_BP_CLEAR_DRAM_LEN,
	.reverse = ESP_STUB_REVERSE_BINARY,
};

// cmd_flash_write_deflated definitions
#define ESP_STUB_CMD_FLASH_WRITE_DEFLATED_BSS_SIZE 0x0UL
#define ESP_STUB_CMD_FLASH_WRITE_DEFLATED_IRAM_ORG 0x400bc000UL
#define ESP_STUB_CMD_FLASH_WRITE_DEFLATED_IRAM_LEN 0x00004000UL
#define ESP_STUB_CMD_FLASH_WRITE_DEFLATED_DRAM_ORG 0x3ffe4000UL
#define ESP_STUB_CMD_FLASH_WRITE_DEFLATED_DRAM_LEN 0x00014000UL
#define ESP_STUB_CMD_FLASH_WRITE_DEFLATED_ENTRY_ADDR 0x400bc08cUL
#define ESP_STUB_CMD_FLASH_WRITE_DEFLATED_APPTRACE_CTRL_ADDR 0x0UL
#define ESP_STUB_CMD_FLASH_WRITE_DEFLATED_LOG_ADDR 0x0UL
#define ESP_STUB_CMD_FLASH_WRITE_DEFLATED_LOG_SIZE 0x0UL

static const uint8_t s_esp_flasher_stub_cmd_flash_write_deflated_code[] = {
#include "contrib/loaders/flash/espressif/images/esp32/stub_cmd_flash_write_deflated_code.inc"
};

static const uint8_t s_esp_flasher_stub_cmd_flash_write_deflated_data[] = {
#include "contrib/loaders/flash/espressif/images/esp32/stub_cmd_flash_write_deflated_data.inc"
};

static const struct esp_flasher_stub_config s_esp_stub_cmd_flash_write_deflated_cfg = {
	.code = s_esp_flasher_stub_cmd_flash_write_deflated_code,
	.code_sz = sizeof(s_esp_flasher_stub_cmd_flash_write_deflated_code),
	.data = s_esp_flasher_stub_cmd_flash_write_deflated_data,
	.data_sz = sizeof(s_esp_flasher_stub_cmd_flash_write_deflated_data),
	.entry_addr = ESP_STUB_CMD_FLASH_WRITE_DEFLATED_ENTRY_ADDR,
	.bss_sz = ESP_STUB_CMD_FLASH_WRITE_DEFLATED_BSS_SIZE,
	.apptrace_ctrl_addr = ESP_STUB_CMD_FLASH_WRITE_DEFLATED_APPTRACE_CTRL_ADDR,
	.stack_default_sz = ESP_STUB_STACK_SIZE,
	.log_buff_addr = ESP_STUB_CMD_FLASH_WRITE_DEFLATED_LOG_ADDR,
	.log_buff_size = ESP_STUB_CMD_FLASH_WRITE_DEFLATED_LOG_SIZE,
	.iram_org = ESP_STUB_CMD_FLASH_WRITE_DEFLATED_IRAM_ORG,
	.iram_len = ESP_STUB_CMD_FLASH_WRITE_DEFLATED_IRAM_LEN,
	.dram_org = ESP_STUB_CMD_FLASH_WRITE_DEFLATED_DRAM_ORG,
	.dram_len = ESP_STUB_CMD_FLASH_WRITE_DEFLATED_DRAM_LEN,
	.reverse = ESP_STUB_REVERSE_BINARY,
};

// cmd_flash_calc_hash definitions
#define ESP_STUB_CMD_FLASH_CALC_HASH_BSS_SIZE 0x0UL
#define ESP_STUB_CMD_FLASH_CALC_HASH_IRAM_ORG 0x400bc000UL
#define ESP_STUB_CMD_FLASH_CALC_HASH_IRAM_LEN 0x00004000UL
#define ESP_STUB_CMD_FLASH_CALC_HASH_DRAM_ORG 0x3ffe4000UL
#define ESP_STUB_CMD_FLASH_CALC_HASH_DRAM_LEN 0x00014000UL
#define ESP_STUB_CMD_FLASH_CALC_HASH_ENTRY_ADDR 0x400bc08cUL
#define ESP_STUB_CMD_FLASH_CALC_HASH_APPTRACE_CTRL_ADDR 0x0UL
#define ESP_STUB_CMD_FLASH_CALC_HASH_LOG_ADDR 0x0UL
#define ESP_STUB_CMD_FLASH_CALC_HASH_LOG_SIZE 0x0UL

static const uint8_t s_esp_flasher_stub_cmd_flash_calc_hash_code[] = {
#include "contrib/loaders/flash/espressif/images/esp32/stub_cmd_flash_calc_hash_code.inc"
};

static const uint8_t s_esp_flasher_stub_cmd_flash_calc_hash_data[] = {
#include "contrib/loaders/flash/espressif/images/esp32/stub_cmd_flash_calc_hash_data.inc"
};

static const struct esp_flasher_stub_config s_esp_stub_cmd_flash_calc_hash_cfg = {
	.code = s_esp_flasher_stub_cmd_flash_calc_hash_code,
	.code_sz = sizeof(s_esp_flasher_stub_cmd_flash_calc_hash_code),
	.data = s_esp_flasher_stub_cmd_flash_calc_hash_data,
	.data_sz = sizeof(s_esp_flasher_stub_cmd_flash_calc_hash_data),
	.entry_addr = ESP_STUB_CMD_FLASH_CALC_HASH_ENTRY_ADDR,
	.bss_sz = ESP_STUB_CMD_FLASH_CALC_HASH_BSS_SIZE,
	.apptrace_ctrl_addr = ESP_STUB_CMD_FLASH_CALC_HASH_APPTRACE_CTRL_ADDR,
	.stack_default_sz = ESP_STUB_STACK_SIZE,
	.log_buff_addr = ESP_STUB_CMD_FLASH_CALC_HASH_LOG_ADDR,
	.log_buff_size = ESP_STUB_CMD_FLASH_CALC_HASH_LOG_SIZE,
	.iram_org = ESP_STUB_CMD_FLASH_CALC_HASH_IRAM_ORG,
	.iram_len = ESP_STUB_CMD_FLASH_CALC_HASH_IRAM_LEN,
	.dram_org = ESP_STUB_CMD_FLASH_CALC_HASH_DRAM_ORG,
	.dram_len = ESP_STUB_CMD_FLASH_CALC_HASH_DRAM_LEN,
	.reverse = ESP_STUB_REVERSE_BINARY,
};

// cmd_flash_clock_configure definitions
#define ESP_STUB_CMD_FLASH_CLOCK_CONFIGURE_BSS_SIZE 0x0UL
#define ESP_STUB_CMD_FLASH_CLOCK_CONFIGURE_IRAM_ORG 0x400bc000UL
#define ESP_STUB_CMD_FLASH_CLOCK_CONFIGURE_IRAM_LEN 0x00004000UL
#define ESP_STUB_CMD_FLASH_CLOCK_CONFIGURE_DRAM_ORG 0x3ffe4000UL
#define ESP_STUB_CMD_FLASH_CLOCK_CONFIGURE_DRAM_LEN 0x00014000UL
#define ESP_STUB_CMD_FLASH_CLOCK_CONFIGURE_ENTRY_ADDR 0x400bc08cUL
#define ESP_STUB_CMD_FLASH_CLOCK_CONFIGURE_APPTRACE_CTRL_ADDR 0x0UL
#define ESP_STUB_CMD_FLASH_CLOCK_CONFIGURE_LOG_ADDR 0x0UL
#define ESP_STUB_CMD_FLASH_CLOCK_CONFIGURE_LOG_SIZE 0x0UL

static const uint8_t s_esp_flasher_stub_cmd_flash_clock_configure_code[] = {
#include "contrib/loaders/flash/espressif/images/esp32/stub_cmd_flash_clock_configure_code.inc"
};

static const uint8_t s_esp_flasher_stub_cmd_flash_clock_configure_data[] = {
#include "contrib/loaders/flash/espressif/images/esp32/stub_cmd_flash_clock_configure_data.inc"
};

static const struct esp_flasher_stub_config s_esp_stub_cmd_flash_clock_configure_cfg = {
	.code = s_esp_flasher_stub_cmd_flash_clock_configure_code,
	.code_sz = sizeof(s_esp_flasher_stub_cmd_flash_clock_configure_code),
	.data = s_esp_flasher_stub_cmd_flash_clock_configure_data,
	.data_sz = sizeof(s_esp_flasher_stub_cmd_flash_clock_configure_data),
	.entry_addr = ESP_STUB_CMD_FLASH_CLOCK_CONFIGURE_ENTRY_ADDR,
	.bss_sz = ESP_STUB_CMD_FLASH_CLOCK_CONFIGURE_BSS_SIZE,
	.apptrace_ctrl_addr = ESP_STUB_CMD_FLASH_CLOCK_CONFIGURE_APPTRACE_CTRL_ADDR,
	.stack_default_sz = ESP_STUB_STACK_SIZE,
	.log_buff_addr = ESP_STUB_CMD_FLASH_CLOCK_CONFIGURE_LOG_ADDR,
	.log_buff_size = ESP_STUB_CMD_FLASH_CLOCK_CONFIGURE_LOG_SIZE,
	.iram_org = ESP_STUB_CMD_FLASH_CLOCK_CONFIGURE_IRAM_ORG,
	.iram_len = ESP_STUB_CMD_FLASH_CLOCK_CONFIGURE_IRAM_LEN,
	.dram_org = ESP_STUB_CMD_FLASH_CLOCK_CONFIGURE_DRAM_ORG,
	.dram_len = ESP_STUB_CMD_FLASH_CLOCK_CONFIGURE_DRAM_LEN,
	.reverse = ESP_STUB_REVERSE_BINARY,
};

// cmd_flash_multi_command definitions
#define ESP_STUB_CMD_FLASH_MULTI_COMMAND_BSS_SIZE 0x0UL
#define ESP_STUB_CMD_FLASH_MULTI_COMMAND_IRAM_ORG 0x400bc000UL
#define ESP_STUB_CMD_FLASH_MULTI_COMMAND_IRAM_LEN 0x00004000UL
#define ESP_STUB_CMD_FLASH_MULTI_COMMAND_DRAM_ORG 0x3ffe4000UL
#define ESP_STUB_CMD_FLASH_MULTI_COMMAND_DRAM_LEN 0x00014000UL
#define ESP_STUB_CMD_FLASH_MULTI_COMMAND_ENTRY_ADDR 0x400bc09cUL
#define ESP_STUB_CMD_FLASH_MULTI_COMMAND_APPTRACE_CTRL_ADDR 0x0UL
#define ESP_STUB_CMD_FLASH_MULTI_COMMAND_LOG_ADDR 0x0UL
#define ESP_STUB_CMD_FLASH_MULTI_COMMAND_LOG_SIZE 0x0UL

static const uint8_t s_esp_flasher_stub_cmd_flash_multi_command_code[] = {
#include "contrib/loaders/flash/espressif/images/esp32/stub_cmd_flash_multi_command_code.inc"
};

static const uint8_t s_esp_flasher_stub_cmd_flash_multi_command_data[] = {
#include "contrib/loaders/flash/espressif/images/esp32/stub_cmd_flash_multi_command_data.inc"
};

static const struct esp_flasher_stub_config s_esp_stub_cmd_flash_multi_command_cfg = {
	.code = s_esp_flasher_stub_cmd_flash_multi_command_code,
	.code_sz = sizeof(s_esp_flasher_stub_cmd_flash_multi_command_code),
	.data = s_esp_flasher_stub_cmd_flash_multi_command_data,
	.data_sz = sizeof(s_esp_flasher_stub_cmd_flash_multi_command_data),
	.entry_addr = ESP_STUB_CMD_FLASH_MULTI_COMMAND_ENTRY_ADDR,
	.bss_sz = ESP_STUB_CMD_FLASH_MULTI_COMMAND_BSS_SIZE,
	.apptrace_ctrl_addr = ESP_STUB_CMD_FLASH_MULTI_COMMAND_APPTRACE_CTRL_ADDR,
	.stack_default_sz = ESP_STUB_STACK_SIZE,
	.log_buff_addr = ESP_STUB_CMD_FLASH_MULTI_COMMAND_LOG_ADDR,
	.log_buff_size = ESP_STUB_CMD_FLASH_MULTI_COMMAND_LOG_SIZE,
	.iram_org = ESP_STUB_CMD_FLASH_MULTI_COMMAND_IRAM_ORG,
	.iram_len = ESP_STUB_CMD_FLASH_MULTI_COMMAND_IRAM_LEN,
	.dram_org = ESP_STUB_CMD_FLASH_MULTI_COMMAND_DRAM_ORG,
	.dram_len = ESP_STUB_CMD_FLASH_MULTI_COMMAND_DRAM_LEN,
	.reverse = ESP_STUB_REVERSE_BINARY,
};

// cmd_flash_idf_binary definitions
#define ESP_STUB_CMD_FLASH_IDF_BINARY_BSS_SIZE 0x0UL
#define ESP_STUB_CMD_FLASH_IDF_BINARY_IRAM_ORG 0x400bc000UL
#define ESP_STUB_CMD_FLASH_IDF_BINARY_IRAM_LEN 0x00004000UL
#define ESP_STUB_CMD_FLASH_IDF_BINARY_DRAM_ORG 0x3ffe4000UL
#define ESP_STUB_CMD_FLASH_IDF_BINARY_DRAM_LEN 0x00014000UL
#define ESP_STUB_CMD_FLASH_IDF_BINARY_ENTRY_ADDR 0x400bc09cUL
#define ESP_STUB_CMD_FLASH_IDF_BINARY_APPTRACE_CTRL_ADDR 0x0UL
#define ESP_STUB_CMD_FLASH_IDF_BINARY_LOG_ADDR 0x0UL
#define ESP_STUB_CMD_FLASH_IDF_BINARY_LOG_SIZE 0x0UL

static const uint8_t s_esp_flasher_stub_cmd_flash_idf_binary_code[] = {
#include "contrib/loaders/flash/espressif/images/esp32/stub_cmd_flash_idf_binary_code.inc"
};

static const uint8_t s_esp_flasher_stub_cmd_flash_idf_binary_data[] = {
#include "contrib/loaders/flash/espressif/images/esp32/stub_cmd_flash_idf_binary_data.inc"
};

static const struct esp_flasher_stub_config s_esp_stub_cmd_flash_idf_binary_cfg = {
	.code = s_esp_flasher_stub_cmd_flash_idf_binary_code,
	.code_sz = sizeof(s_esp_flasher_stub_cmd_flash_idf_binary_code),
	.data = s_esp_flasher_stub_cmd_flash_idf_binary_data,
	.data_sz = sizeof(s_esp_flasher_stub_cmd_flash_idf_binary_data),
	.entry_addr = ESP_STUB_CMD_FLASH_IDF_BINARY_ENTRY_ADDR,
	.bss_sz = ESP_STUB_CMD_FLASH_IDF_BINARY_BSS_SIZE,
	.apptrace_ctrl_addr = ESP_STUB_CMD_FLASH_IDF_BINARY_APPTRACE_CTRL_ADDR,
	.stack_default_sz = ESP_STUB_STACK_SIZE,
	.log_buff_addr = ESP_STUB_CMD_FLASH_IDF_BINARY_LOG_ADDR,
	.log_buff_size = ESP_STUB_CMD_FLASH_IDF_BINARY_LOG_SIZE,
	.iram_org = ESP_STUB_CMD_FLASH_IDF_BINARY_IRAM_ORG,
	.iram_len = ESP_STUB_CMD_FLASH_IDF_BINARY_IRAM_LEN,
	.dram_org = ESP_STUB_CMD_FLASH_IDF_BINARY_DRAM_ORG,
	.dram_len = ESP_STUB_CMD_FLASH_IDF_BINARY_DRAM_LEN,
	.reverse = ESP_STUB_REVERSE_BINARY,
};

// cmd_test_all definitions
#define ESP_STUB_CMD_TEST_ALL_BSS_SIZE 0x002034UL
#define ESP_STUB_CMD_TEST_ALL_IRAM_ORG 0x400bc000UL
#define ESP_STUB_CMD_TEST_ALL_IRAM_LEN 0x00004000UL
#define ESP_STUB_CMD_TEST_ALL_DRAM_ORG 0x3ffe4000UL
#define ESP_STUB_CMD_TEST_ALL_DRAM_LEN 0x00014000UL
#define ESP_STUB_CMD_TEST_ALL_ENTRY_ADDR 0x400bcc58UL
#define ESP_STUB_CMD_TEST_ALL_APPTRACE_CTRL_ADDR 0x0UL
#define ESP_STUB_CMD_TEST_ALL_LOG_ADDR 0x3ffe4cf0UL
#define ESP_STUB_CMD_TEST_ALL_LOG_SIZE 0x8196UL

static const uint8_t s_esp_flasher_stub_cmd_test_all_code[] = {
#include "contrib/loaders/flash/espressif/images/esp32/stub_cmd_test_all_code.inc"
};

static const uint8_t s_esp_flasher_stub_cmd_test_all_data[] = {
#include "contrib/loaders/flash/espressif/images/esp32/stub_cmd_test_all_data.inc"
};

static const struct esp_flasher_stub_config s_esp_stub_cmd_test_all_cfg = {
	.code = s_esp_flasher_stub_cmd_test_all_code,
	.code_sz = sizeof(s_esp_flasher_stub_cmd_test_all_code),
	.data = s_esp_flasher_stub_cmd_test_all_data,
	.data_sz = sizeof(s_esp_flasher_stub_cmd_test_all_data),
	.entry_addr = ESP_STUB_CMD_TEST_ALL_ENTRY_ADDR,
	.bss_sz = ESP_STUB_CMD_TEST_ALL_BSS_SIZE,
	.apptrace_ctrl_addr = ESP_STUB_CMD_TEST_ALL_APPTRACE_CTRL_ADDR,
	.stack_default_sz = ESP_STUB_STACK_SIZE,
	.log_buff_addr = ESP_STUB_CMD_TEST_ALL_LOG_ADDR,
	.log_buff_size = ESP_STUB_CMD_TEST_ALL_LOG_SIZE,
	.iram_org = ESP_STUB_CMD_TEST_ALL_IRAM_ORG,
	.iram_len = ESP_STUB_CMD_TEST_ALL_IRAM_LEN,
	.dram_org = ESP_STUB_CMD_TEST_ALL_DRAM_ORG,
	.dram_len = ESP_STUB_CMD_TEST_ALL_DRAM_LEN,
	.reverse = ESP_STUB_REVERSE_BINARY,
};

static const struct command_map s_cmd_map[ESP_STUB_CMD_FLASH_MAX_ID + 1] = {
	{ESP_STUB_CMD_TEST1, &s_esp_stub_cmd_test1_cfg},
	{ESP_STUB_CMD_RECV_FROM_HOST, &s_esp_stub_cmd_recv_from_host_cfg},
	{ESP_STUB_CMD_SEND_TO_HOST, &s_esp_stub_cmd_send_to_host_cfg},
	{ESP_STUB_CMD_FLASH_READ, &s_esp_stub_cmd_flash_read_cfg},
	{ESP_STUB_CMD_FLASH_WRITE, &s_esp_stub_cmd_flash_write_cfg},
	{ESP_STUB_CMD_FLASH_ERASE, &s_esp_stub_cmd_flash_erase_cfg},
	{ESP_STUB_CMD_FLASH_ERASE_CHECK, &s_esp_stub_cmd_flash_erase_check_cfg},
	{ESP_STUB_CMD_FLASH_MAP_GET, &s_esp_stub_cmd_flash_map_get_cfg},
	{ESP_STUB_CMD_FLASH_BP_SET, &s_esp_stub_cmd_flash_bp_set_cfg},
	{ESP_STUB_CMD_FLASH_BP_CLEAR, &s_esp_stub_cmd_flash_bp_clear_cfg},
	{ESP_STUB_CMD_FLASH_WRITE_DEFLATED, &s_esp_stub_cmd_flash_write_deflated_cfg},
	{ESP_STUB_CMD_FLASH_CALC_HASH, &s_esp_stub_cmd_flash_calc_hash_cfg},
	{ESP_STUB_CMD_FLASH_CLOCK_CONFIGURE, &s_esp_stub_cmd_flash_clock_configure_cfg},
	{ESP_STUB_CMD_FLASH_MULTI_COMMAND, &s_esp_stub_cmd_flash_multi_command_cfg},
	{ESP_STUB_CMD_FLASH_IDF_BINARY, &s_esp_stub_cmd_flash_idf_binary_cfg},
	{ESP_STUB_CMD_TEST_ALL, &s_esp_stub_cmd_test_all_cfg},
};
