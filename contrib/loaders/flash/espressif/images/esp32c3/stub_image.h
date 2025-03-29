/* SPDX-License-Identifier: Apache-2.0 OR MIT */
/* SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD */

#pragma once

#include <stdint.h>

#define ESP_STUB_REVERSE_BINARY 0
#define ESP_STUB_STACK_SIZE 512

// cmd_test1 definitions
#define ESP_STUB_CMD_TEST1_BSS_SIZE 0x0UL
#define ESP_STUB_CMD_TEST1_IRAM_ORG 0x40380000UL
#define ESP_STUB_CMD_TEST1_IRAM_LEN 0x00004000UL
#define ESP_STUB_CMD_TEST1_DRAM_ORG 0x3fc84000UL
#define ESP_STUB_CMD_TEST1_DRAM_LEN 0x00020000UL
#define ESP_STUB_CMD_TEST1_ENTRY_ADDR 0x40380040UL
#define ESP_STUB_CMD_TEST1_APPTRACE_CTRL_ADDR 0x0UL
#define ESP_STUB_CMD_TEST1_LOG_ADDR 0x0UL
#define ESP_STUB_CMD_TEST1_LOG_SIZE 0x0UL

static const uint8_t s_esp_flasher_stub_cmd_test1_code[] = {
#include "contrib/loaders/flash/espressif/images/esp32c3/stub_cmd_test1_code.inc"
};

static const uint8_t s_esp_flasher_stub_cmd_test1_data[] = {
#include "contrib/loaders/flash/espressif/images/esp32c3/stub_cmd_test1_data.inc"
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
#define ESP_STUB_CMD_RECV_FROM_HOST_BSS_SIZE 0x000040UL
#define ESP_STUB_CMD_RECV_FROM_HOST_IRAM_ORG 0x40380000UL
#define ESP_STUB_CMD_RECV_FROM_HOST_IRAM_LEN 0x00004000UL
#define ESP_STUB_CMD_RECV_FROM_HOST_DRAM_ORG 0x3fc84000UL
#define ESP_STUB_CMD_RECV_FROM_HOST_DRAM_LEN 0x00020000UL
#define ESP_STUB_CMD_RECV_FROM_HOST_ENTRY_ADDR 0x40380282UL
#define ESP_STUB_CMD_RECV_FROM_HOST_APPTRACE_CTRL_ADDR 0x3fc84058UL
#define ESP_STUB_CMD_RECV_FROM_HOST_LOG_ADDR 0x0UL
#define ESP_STUB_CMD_RECV_FROM_HOST_LOG_SIZE 0x0UL

static const uint8_t s_esp_flasher_stub_cmd_recv_from_host_code[] = {
#include "contrib/loaders/flash/espressif/images/esp32c3/stub_cmd_recv_from_host_code.inc"
};

static const uint8_t s_esp_flasher_stub_cmd_recv_from_host_data[] = {
#include "contrib/loaders/flash/espressif/images/esp32c3/stub_cmd_recv_from_host_data.inc"
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
#define ESP_STUB_CMD_SEND_TO_HOST_BSS_SIZE 0x000041UL
#define ESP_STUB_CMD_SEND_TO_HOST_IRAM_ORG 0x40380000UL
#define ESP_STUB_CMD_SEND_TO_HOST_IRAM_LEN 0x00004000UL
#define ESP_STUB_CMD_SEND_TO_HOST_DRAM_ORG 0x3fc84000UL
#define ESP_STUB_CMD_SEND_TO_HOST_DRAM_LEN 0x00020000UL
#define ESP_STUB_CMD_SEND_TO_HOST_ENTRY_ADDR 0x403802ceUL
#define ESP_STUB_CMD_SEND_TO_HOST_APPTRACE_CTRL_ADDR 0x3fc84058UL
#define ESP_STUB_CMD_SEND_TO_HOST_LOG_ADDR 0x0UL
#define ESP_STUB_CMD_SEND_TO_HOST_LOG_SIZE 0x0UL

static const uint8_t s_esp_flasher_stub_cmd_send_to_host_code[] = {
#include "contrib/loaders/flash/espressif/images/esp32c3/stub_cmd_send_to_host_code.inc"
};

static const uint8_t s_esp_flasher_stub_cmd_send_to_host_data[] = {
#include "contrib/loaders/flash/espressif/images/esp32c3/stub_cmd_send_to_host_data.inc"
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

// cmd_flash_info definitions
#define ESP_STUB_CMD_FLASH_INFO_BSS_SIZE 0x0UL
#define ESP_STUB_CMD_FLASH_INFO_IRAM_ORG 0x40380000UL
#define ESP_STUB_CMD_FLASH_INFO_IRAM_LEN 0x00004000UL
#define ESP_STUB_CMD_FLASH_INFO_DRAM_ORG 0x3fc84000UL
#define ESP_STUB_CMD_FLASH_INFO_DRAM_LEN 0x00020000UL
#define ESP_STUB_CMD_FLASH_INFO_ENTRY_ADDR 0x40380076UL
#define ESP_STUB_CMD_FLASH_INFO_APPTRACE_CTRL_ADDR 0x0UL
#define ESP_STUB_CMD_FLASH_INFO_LOG_ADDR 0x0UL
#define ESP_STUB_CMD_FLASH_INFO_LOG_SIZE 0x0UL

static const uint8_t s_esp_flasher_stub_cmd_flash_info_code[] = {
#include "contrib/loaders/flash/espressif/images/esp32c3/stub_cmd_flash_info_code.inc"
};

static const uint8_t s_esp_flasher_stub_cmd_flash_info_data[] = {
#include "contrib/loaders/flash/espressif/images/esp32c3/stub_cmd_flash_info_data.inc"
};

static const struct esp_flasher_stub_config s_esp_stub_cmd_flash_info_cfg = {
	.code = s_esp_flasher_stub_cmd_flash_info_code,
	.code_sz = sizeof(s_esp_flasher_stub_cmd_flash_info_code),
	.data = s_esp_flasher_stub_cmd_flash_info_data,
	.data_sz = sizeof(s_esp_flasher_stub_cmd_flash_info_data),
	.entry_addr = ESP_STUB_CMD_FLASH_INFO_ENTRY_ADDR,
	.bss_sz = ESP_STUB_CMD_FLASH_INFO_BSS_SIZE,
	.apptrace_ctrl_addr = ESP_STUB_CMD_FLASH_INFO_APPTRACE_CTRL_ADDR,
	.stack_default_sz = ESP_STUB_STACK_SIZE,
	.log_buff_addr = ESP_STUB_CMD_FLASH_INFO_LOG_ADDR,
	.log_buff_size = ESP_STUB_CMD_FLASH_INFO_LOG_SIZE,
	.iram_org = ESP_STUB_CMD_FLASH_INFO_IRAM_ORG,
	.iram_len = ESP_STUB_CMD_FLASH_INFO_IRAM_LEN,
	.dram_org = ESP_STUB_CMD_FLASH_INFO_DRAM_ORG,
	.dram_len = ESP_STUB_CMD_FLASH_INFO_DRAM_LEN,
	.reverse = ESP_STUB_REVERSE_BINARY,
};

// cmd_flash_test definitions
#define ESP_STUB_CMD_FLASH_TEST_BSS_SIZE 0x0UL
#define ESP_STUB_CMD_FLASH_TEST_IRAM_ORG 0x40380000UL
#define ESP_STUB_CMD_FLASH_TEST_IRAM_LEN 0x00004000UL
#define ESP_STUB_CMD_FLASH_TEST_DRAM_ORG 0x3fc84000UL
#define ESP_STUB_CMD_FLASH_TEST_DRAM_LEN 0x00020000UL
#define ESP_STUB_CMD_FLASH_TEST_ENTRY_ADDR 0x40380042UL
#define ESP_STUB_CMD_FLASH_TEST_APPTRACE_CTRL_ADDR 0x0UL
#define ESP_STUB_CMD_FLASH_TEST_LOG_ADDR 0x0UL
#define ESP_STUB_CMD_FLASH_TEST_LOG_SIZE 0x0UL

static const uint8_t s_esp_flasher_stub_cmd_flash_test_code[] = {
#include "contrib/loaders/flash/espressif/images/esp32c3/stub_cmd_flash_test_code.inc"
};

static const uint8_t s_esp_flasher_stub_cmd_flash_test_data[] = {
#include "contrib/loaders/flash/espressif/images/esp32c3/stub_cmd_flash_test_data.inc"
};

static const struct esp_flasher_stub_config s_esp_stub_cmd_flash_test_cfg = {
	.code = s_esp_flasher_stub_cmd_flash_test_code,
	.code_sz = sizeof(s_esp_flasher_stub_cmd_flash_test_code),
	.data = s_esp_flasher_stub_cmd_flash_test_data,
	.data_sz = sizeof(s_esp_flasher_stub_cmd_flash_test_data),
	.entry_addr = ESP_STUB_CMD_FLASH_TEST_ENTRY_ADDR,
	.bss_sz = ESP_STUB_CMD_FLASH_TEST_BSS_SIZE,
	.apptrace_ctrl_addr = ESP_STUB_CMD_FLASH_TEST_APPTRACE_CTRL_ADDR,
	.stack_default_sz = ESP_STUB_STACK_SIZE,
	.log_buff_addr = ESP_STUB_CMD_FLASH_TEST_LOG_ADDR,
	.log_buff_size = ESP_STUB_CMD_FLASH_TEST_LOG_SIZE,
	.iram_org = ESP_STUB_CMD_FLASH_TEST_IRAM_ORG,
	.iram_len = ESP_STUB_CMD_FLASH_TEST_IRAM_LEN,
	.dram_org = ESP_STUB_CMD_FLASH_TEST_DRAM_ORG,
	.dram_len = ESP_STUB_CMD_FLASH_TEST_DRAM_LEN,
	.reverse = ESP_STUB_REVERSE_BINARY,
};

// cmd_test_all definitions
#define ESP_STUB_CMD_TEST_ALL_BSS_SIZE 0x002044UL
#define ESP_STUB_CMD_TEST_ALL_IRAM_ORG 0x40380000UL
#define ESP_STUB_CMD_TEST_ALL_IRAM_LEN 0x00004000UL
#define ESP_STUB_CMD_TEST_ALL_DRAM_ORG 0x3fc84000UL
#define ESP_STUB_CMD_TEST_ALL_DRAM_LEN 0x00020000UL
#define ESP_STUB_CMD_TEST_ALL_ENTRY_ADDR 0x40380a72UL
#define ESP_STUB_CMD_TEST_ALL_APPTRACE_CTRL_ADDR 0x3fc84abcUL
#define ESP_STUB_CMD_TEST_ALL_LOG_ADDR 0x3fc84ac8UL
#define ESP_STUB_CMD_TEST_ALL_LOG_SIZE 0x8196UL

static const uint8_t s_esp_flasher_stub_cmd_test_all_code[] = {
#include "contrib/loaders/flash/espressif/images/esp32c3/stub_cmd_test_all_code.inc"
};

static const uint8_t s_esp_flasher_stub_cmd_test_all_data[] = {
#include "contrib/loaders/flash/espressif/images/esp32c3/stub_cmd_test_all_data.inc"
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
	{ESP_STUB_CMD_FLASH_INFO, &s_esp_stub_cmd_flash_info_cfg},
	{ESP_STUB_CMD_FLASH_TEST, &s_esp_stub_cmd_flash_test_cfg},
	{ESP_STUB_CMD_TEST_ALL, &s_esp_stub_cmd_test_all_cfg},
};
