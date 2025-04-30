/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 */

#pragma once

#include <stdint.h>

#define ESP_STUB_REVERSE_BINARY 0
#define ESP_STUB_STACK_SIZE 512

// cmd_test1 definitions
#define ESP_STUB_CMD_TEST1_BSS_SIZE 0x0UL
#define ESP_STUB_CMD_TEST1_IRAM_ORG 0x40028000UL
#define ESP_STUB_CMD_TEST1_IRAM_LEN 0x00004000UL
#define ESP_STUB_CMD_TEST1_DRAM_ORG 0x3ffbc000UL
#define ESP_STUB_CMD_TEST1_DRAM_LEN 0x00020000UL
#define ESP_STUB_CMD_TEST1_ENTRY_ADDR 0x40028020UL
#define ESP_STUB_CMD_TEST1_APPTRACE_CTRL_ADDR 0x0UL
#define ESP_STUB_CMD_TEST1_LOG_ADDR 0x0UL
#define ESP_STUB_CMD_TEST1_LOG_SIZE 0x0UL

static const uint8_t s_esp_flasher_stub_cmd_test1_code[] = {
#include "contrib/loaders/flash/espressif/images/esp32s2/stub_cmd_test1_code.inc"
};

static const uint8_t s_esp_flasher_stub_cmd_test1_data[] = {
#include "contrib/loaders/flash/espressif/images/esp32s2/stub_cmd_test1_data.inc"
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
#define ESP_STUB_CMD_RECV_FROM_HOST_IRAM_ORG 0x40028000UL
#define ESP_STUB_CMD_RECV_FROM_HOST_IRAM_LEN 0x00004000UL
#define ESP_STUB_CMD_RECV_FROM_HOST_DRAM_ORG 0x3ffbc000UL
#define ESP_STUB_CMD_RECV_FROM_HOST_DRAM_LEN 0x00020000UL
#define ESP_STUB_CMD_RECV_FROM_HOST_ENTRY_ADDR 0x40028288UL
#define ESP_STUB_CMD_RECV_FROM_HOST_APPTRACE_CTRL_ADDR 0x0UL
#define ESP_STUB_CMD_RECV_FROM_HOST_LOG_ADDR 0x0UL
#define ESP_STUB_CMD_RECV_FROM_HOST_LOG_SIZE 0x0UL

static const uint8_t s_esp_flasher_stub_cmd_recv_from_host_code[] = {
#include "contrib/loaders/flash/espressif/images/esp32s2/stub_cmd_recv_from_host_code.inc"
};

static const uint8_t s_esp_flasher_stub_cmd_recv_from_host_data[] = {
#include "contrib/loaders/flash/espressif/images/esp32s2/stub_cmd_recv_from_host_data.inc"
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
#define ESP_STUB_CMD_SEND_TO_HOST_IRAM_ORG 0x40028000UL
#define ESP_STUB_CMD_SEND_TO_HOST_IRAM_LEN 0x00004000UL
#define ESP_STUB_CMD_SEND_TO_HOST_DRAM_ORG 0x3ffbc000UL
#define ESP_STUB_CMD_SEND_TO_HOST_DRAM_LEN 0x00020000UL
#define ESP_STUB_CMD_SEND_TO_HOST_ENTRY_ADDR 0x400282ecUL
#define ESP_STUB_CMD_SEND_TO_HOST_APPTRACE_CTRL_ADDR 0x0UL
#define ESP_STUB_CMD_SEND_TO_HOST_LOG_ADDR 0x0UL
#define ESP_STUB_CMD_SEND_TO_HOST_LOG_SIZE 0x0UL

static const uint8_t s_esp_flasher_stub_cmd_send_to_host_code[] = {
#include "contrib/loaders/flash/espressif/images/esp32s2/stub_cmd_send_to_host_code.inc"
};

static const uint8_t s_esp_flasher_stub_cmd_send_to_host_data[] = {
#include "contrib/loaders/flash/espressif/images/esp32s2/stub_cmd_send_to_host_data.inc"
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

// cmd_test_all definitions
#define ESP_STUB_CMD_TEST_ALL_BSS_SIZE 0x002034UL
#define ESP_STUB_CMD_TEST_ALL_IRAM_ORG 0x40028000UL
#define ESP_STUB_CMD_TEST_ALL_IRAM_LEN 0x00004000UL
#define ESP_STUB_CMD_TEST_ALL_DRAM_ORG 0x3ffbc000UL
#define ESP_STUB_CMD_TEST_ALL_DRAM_LEN 0x00020000UL
#define ESP_STUB_CMD_TEST_ALL_ENTRY_ADDR 0x400289c4UL
#define ESP_STUB_CMD_TEST_ALL_APPTRACE_CTRL_ADDR 0x0UL
#define ESP_STUB_CMD_TEST_ALL_LOG_ADDR 0x3ffbc9f4UL
#define ESP_STUB_CMD_TEST_ALL_LOG_SIZE 0x8196UL

static const uint8_t s_esp_flasher_stub_cmd_test_all_code[] = {
#include "contrib/loaders/flash/espressif/images/esp32s2/stub_cmd_test_all_code.inc"
};

static const uint8_t s_esp_flasher_stub_cmd_test_all_data[] = {
#include "contrib/loaders/flash/espressif/images/esp32s2/stub_cmd_test_all_data.inc"
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
	{ESP_STUB_CMD_TEST_ALL, &s_esp_stub_cmd_test_all_cfg},
};
