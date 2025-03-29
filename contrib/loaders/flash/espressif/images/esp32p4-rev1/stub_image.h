/* SPDX-License-Identifier: Apache-2.0 OR MIT */
/* SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD */

#pragma once

#include <stdint.h>

#define ESP_STUB_STACK_SIZE_REV1 1024

// cmd_test1 definitions
#define ESP_STUB_CMD_TEST1_BSS_SIZE_REV1 0x000082UL
#define ESP_STUB_CMD_TEST1_IRAM_ORG_REV1 0x8ff00000UL
#define ESP_STUB_CMD_TEST1_IRAM_LEN_REV1 0x00004000UL
#define ESP_STUB_CMD_TEST1_DRAM_ORG_REV1 0x8ff04000UL
#define ESP_STUB_CMD_TEST1_DRAM_LEN_REV1 0x00020000UL
#define ESP_STUB_CMD_TEST1_ENTRY_ADDR_REV1 0x8ff0009aUL
#define ESP_STUB_CMD_TEST1_APPTRACE_CTRL_ADDR_REV1 0x0UL
#define ESP_STUB_CMD_TEST1_LOG_ADDR_REV1 0x0UL
#define ESP_STUB_CMD_TEST1_LOG_SIZE_REV1 0x0UL
#define ESP_STUB_CMD_TEST1_TRAP_RECORD_ADDR_REV1 0x8ff0404cUL
#define ESP_STUB_CMD_TEST1_TRAP_ENTRY_ADDR_REV1 0x8ff00300UL

static const uint8_t s_esp_flasher_stub_cmd_test1_code_rev1[] = {
#include "contrib/loaders/flash/espressif/images/esp32p4-rev1/stub_cmd_test1_code_rev1.inc"
};

static const uint8_t s_esp_flasher_stub_cmd_test1_data_rev1[] = {
#include "contrib/loaders/flash/espressif/images/esp32p4-rev1/stub_cmd_test1_data_rev1.inc"
};

static const struct esp_flasher_stub_config s_esp_stub_cmd_test1_cfg_rev1 = {
	.name = "cmd_test1",
	.code = s_esp_flasher_stub_cmd_test1_code_rev1,
	.code_sz = sizeof(s_esp_flasher_stub_cmd_test1_code_rev1),
	.data = s_esp_flasher_stub_cmd_test1_data_rev1,
	.data_sz = sizeof(s_esp_flasher_stub_cmd_test1_data_rev1),
	.entry_addr = ESP_STUB_CMD_TEST1_ENTRY_ADDR_REV1,
	.bss_sz = ESP_STUB_CMD_TEST1_BSS_SIZE_REV1,
	.apptrace_ctrl_addr = ESP_STUB_CMD_TEST1_APPTRACE_CTRL_ADDR_REV1,
	.stack_default_sz = ESP_STUB_STACK_SIZE_REV1,
	.log_buff_addr = ESP_STUB_CMD_TEST1_LOG_ADDR_REV1,
	.log_buff_size = ESP_STUB_CMD_TEST1_LOG_SIZE_REV1,
	.trap_record_addr = ESP_STUB_CMD_TEST1_TRAP_RECORD_ADDR_REV1,
	.trap_entry_addr = ESP_STUB_CMD_TEST1_TRAP_ENTRY_ADDR_REV1,
	.iram_org = ESP_STUB_CMD_TEST1_IRAM_ORG_REV1,
	.iram_len = ESP_STUB_CMD_TEST1_IRAM_LEN_REV1,
	.dram_org = ESP_STUB_CMD_TEST1_DRAM_ORG_REV1,
	.dram_len = ESP_STUB_CMD_TEST1_DRAM_LEN_REV1,
};

// cmd_recv_from_host definitions
#define ESP_STUB_CMD_RECV_FROM_HOST_BSS_SIZE_REV1 0x0000c0UL
#define ESP_STUB_CMD_RECV_FROM_HOST_IRAM_ORG_REV1 0x8ff00000UL
#define ESP_STUB_CMD_RECV_FROM_HOST_IRAM_LEN_REV1 0x00004000UL
#define ESP_STUB_CMD_RECV_FROM_HOST_DRAM_ORG_REV1 0x8ff04000UL
#define ESP_STUB_CMD_RECV_FROM_HOST_DRAM_LEN_REV1 0x00020000UL
#define ESP_STUB_CMD_RECV_FROM_HOST_ENTRY_ADDR_REV1 0x8ff002deUL
#define ESP_STUB_CMD_RECV_FROM_HOST_APPTRACE_CTRL_ADDR_REV1 0x8ff04080UL
#define ESP_STUB_CMD_RECV_FROM_HOST_LOG_ADDR_REV1 0x0UL
#define ESP_STUB_CMD_RECV_FROM_HOST_LOG_SIZE_REV1 0x0UL
#define ESP_STUB_CMD_RECV_FROM_HOST_TRAP_RECORD_ADDR_REV1 0x8ff0408cUL
#define ESP_STUB_CMD_RECV_FROM_HOST_TRAP_ENTRY_ADDR_REV1 0x8ff00500UL

static const uint8_t s_esp_flasher_stub_cmd_recv_from_host_code_rev1[] = {
#include "contrib/loaders/flash/espressif/images/esp32p4-rev1/stub_cmd_recv_from_host_code_rev1.inc"
};

static const uint8_t s_esp_flasher_stub_cmd_recv_from_host_data_rev1[] = {
#include "contrib/loaders/flash/espressif/images/esp32p4-rev1/stub_cmd_recv_from_host_data_rev1.inc"
};

static const struct esp_flasher_stub_config s_esp_stub_cmd_recv_from_host_cfg_rev1 = {
	.name = "cmd_recv_from_host",
	.code = s_esp_flasher_stub_cmd_recv_from_host_code_rev1,
	.code_sz = sizeof(s_esp_flasher_stub_cmd_recv_from_host_code_rev1),
	.data = s_esp_flasher_stub_cmd_recv_from_host_data_rev1,
	.data_sz = sizeof(s_esp_flasher_stub_cmd_recv_from_host_data_rev1),
	.entry_addr = ESP_STUB_CMD_RECV_FROM_HOST_ENTRY_ADDR_REV1,
	.bss_sz = ESP_STUB_CMD_RECV_FROM_HOST_BSS_SIZE_REV1,
	.apptrace_ctrl_addr = ESP_STUB_CMD_RECV_FROM_HOST_APPTRACE_CTRL_ADDR_REV1,
	.stack_default_sz = ESP_STUB_STACK_SIZE_REV1,
	.log_buff_addr = ESP_STUB_CMD_RECV_FROM_HOST_LOG_ADDR_REV1,
	.log_buff_size = ESP_STUB_CMD_RECV_FROM_HOST_LOG_SIZE_REV1,
	.trap_record_addr = ESP_STUB_CMD_RECV_FROM_HOST_TRAP_RECORD_ADDR_REV1,
	.trap_entry_addr = ESP_STUB_CMD_RECV_FROM_HOST_TRAP_ENTRY_ADDR_REV1,
	.iram_org = ESP_STUB_CMD_RECV_FROM_HOST_IRAM_ORG_REV1,
	.iram_len = ESP_STUB_CMD_RECV_FROM_HOST_IRAM_LEN_REV1,
	.dram_org = ESP_STUB_CMD_RECV_FROM_HOST_DRAM_ORG_REV1,
	.dram_len = ESP_STUB_CMD_RECV_FROM_HOST_DRAM_LEN_REV1,
};

// cmd_send_to_host definitions
#define ESP_STUB_CMD_SEND_TO_HOST_BSS_SIZE_REV1 0x0000c1UL
#define ESP_STUB_CMD_SEND_TO_HOST_IRAM_ORG_REV1 0x8ff00000UL
#define ESP_STUB_CMD_SEND_TO_HOST_IRAM_LEN_REV1 0x00004000UL
#define ESP_STUB_CMD_SEND_TO_HOST_DRAM_ORG_REV1 0x8ff04000UL
#define ESP_STUB_CMD_SEND_TO_HOST_DRAM_LEN_REV1 0x00020000UL
#define ESP_STUB_CMD_SEND_TO_HOST_ENTRY_ADDR_REV1 0x8ff0032aUL
#define ESP_STUB_CMD_SEND_TO_HOST_APPTRACE_CTRL_ADDR_REV1 0x8ff04080UL
#define ESP_STUB_CMD_SEND_TO_HOST_LOG_ADDR_REV1 0x0UL
#define ESP_STUB_CMD_SEND_TO_HOST_LOG_SIZE_REV1 0x0UL
#define ESP_STUB_CMD_SEND_TO_HOST_TRAP_RECORD_ADDR_REV1 0x8ff0408cUL
#define ESP_STUB_CMD_SEND_TO_HOST_TRAP_ENTRY_ADDR_REV1 0x8ff00500UL

static const uint8_t s_esp_flasher_stub_cmd_send_to_host_code_rev1[] = {
#include "contrib/loaders/flash/espressif/images/esp32p4-rev1/stub_cmd_send_to_host_code_rev1.inc"
};

static const uint8_t s_esp_flasher_stub_cmd_send_to_host_data_rev1[] = {
#include "contrib/loaders/flash/espressif/images/esp32p4-rev1/stub_cmd_send_to_host_data_rev1.inc"
};

static const struct esp_flasher_stub_config s_esp_stub_cmd_send_to_host_cfg_rev1 = {
	.name = "cmd_send_to_host",
	.code = s_esp_flasher_stub_cmd_send_to_host_code_rev1,
	.code_sz = sizeof(s_esp_flasher_stub_cmd_send_to_host_code_rev1),
	.data = s_esp_flasher_stub_cmd_send_to_host_data_rev1,
	.data_sz = sizeof(s_esp_flasher_stub_cmd_send_to_host_data_rev1),
	.entry_addr = ESP_STUB_CMD_SEND_TO_HOST_ENTRY_ADDR_REV1,
	.bss_sz = ESP_STUB_CMD_SEND_TO_HOST_BSS_SIZE_REV1,
	.apptrace_ctrl_addr = ESP_STUB_CMD_SEND_TO_HOST_APPTRACE_CTRL_ADDR_REV1,
	.stack_default_sz = ESP_STUB_STACK_SIZE_REV1,
	.log_buff_addr = ESP_STUB_CMD_SEND_TO_HOST_LOG_ADDR_REV1,
	.log_buff_size = ESP_STUB_CMD_SEND_TO_HOST_LOG_SIZE_REV1,
	.trap_record_addr = ESP_STUB_CMD_SEND_TO_HOST_TRAP_RECORD_ADDR_REV1,
	.trap_entry_addr = ESP_STUB_CMD_SEND_TO_HOST_TRAP_ENTRY_ADDR_REV1,
	.iram_org = ESP_STUB_CMD_SEND_TO_HOST_IRAM_ORG_REV1,
	.iram_len = ESP_STUB_CMD_SEND_TO_HOST_IRAM_LEN_REV1,
	.dram_org = ESP_STUB_CMD_SEND_TO_HOST_DRAM_ORG_REV1,
	.dram_len = ESP_STUB_CMD_SEND_TO_HOST_DRAM_LEN_REV1,
};

// cmd_flash_read definitions
#define ESP_STUB_CMD_FLASH_READ_BSS_SIZE_REV1 0x0000f4UL
#define ESP_STUB_CMD_FLASH_READ_IRAM_ORG_REV1 0x8ff00000UL
#define ESP_STUB_CMD_FLASH_READ_IRAM_LEN_REV1 0x00004000UL
#define ESP_STUB_CMD_FLASH_READ_DRAM_ORG_REV1 0x8ff04000UL
#define ESP_STUB_CMD_FLASH_READ_DRAM_LEN_REV1 0x00020000UL
#define ESP_STUB_CMD_FLASH_READ_ENTRY_ADDR_REV1 0x8ff00566UL
#define ESP_STUB_CMD_FLASH_READ_APPTRACE_CTRL_ADDR_REV1 0x8ff0407cUL
#define ESP_STUB_CMD_FLASH_READ_LOG_ADDR_REV1 0x0UL
#define ESP_STUB_CMD_FLASH_READ_LOG_SIZE_REV1 0x0UL
#define ESP_STUB_CMD_FLASH_READ_TRAP_RECORD_ADDR_REV1 0x8ff040b4UL
#define ESP_STUB_CMD_FLASH_READ_TRAP_ENTRY_ADDR_REV1 0x8ff00700UL

static const uint8_t s_esp_flasher_stub_cmd_flash_read_code_rev1[] = {
#include "contrib/loaders/flash/espressif/images/esp32p4-rev1/stub_cmd_flash_read_code_rev1.inc"
};

static const uint8_t s_esp_flasher_stub_cmd_flash_read_data_rev1[] = {
#include "contrib/loaders/flash/espressif/images/esp32p4-rev1/stub_cmd_flash_read_data_rev1.inc"
};

static const struct esp_flasher_stub_config s_esp_stub_cmd_flash_read_cfg_rev1 = {
	.name = "cmd_flash_read",
	.code = s_esp_flasher_stub_cmd_flash_read_code_rev1,
	.code_sz = sizeof(s_esp_flasher_stub_cmd_flash_read_code_rev1),
	.data = s_esp_flasher_stub_cmd_flash_read_data_rev1,
	.data_sz = sizeof(s_esp_flasher_stub_cmd_flash_read_data_rev1),
	.entry_addr = ESP_STUB_CMD_FLASH_READ_ENTRY_ADDR_REV1,
	.bss_sz = ESP_STUB_CMD_FLASH_READ_BSS_SIZE_REV1,
	.apptrace_ctrl_addr = ESP_STUB_CMD_FLASH_READ_APPTRACE_CTRL_ADDR_REV1,
	.stack_default_sz = ESP_STUB_STACK_SIZE_REV1,
	.log_buff_addr = ESP_STUB_CMD_FLASH_READ_LOG_ADDR_REV1,
	.log_buff_size = ESP_STUB_CMD_FLASH_READ_LOG_SIZE_REV1,
	.trap_record_addr = ESP_STUB_CMD_FLASH_READ_TRAP_RECORD_ADDR_REV1,
	.trap_entry_addr = ESP_STUB_CMD_FLASH_READ_TRAP_ENTRY_ADDR_REV1,
	.iram_org = ESP_STUB_CMD_FLASH_READ_IRAM_ORG_REV1,
	.iram_len = ESP_STUB_CMD_FLASH_READ_IRAM_LEN_REV1,
	.dram_org = ESP_STUB_CMD_FLASH_READ_DRAM_ORG_REV1,
	.dram_len = ESP_STUB_CMD_FLASH_READ_DRAM_LEN_REV1,
};

// cmd_flash_write definitions
#define ESP_STUB_CMD_FLASH_WRITE_BSS_SIZE_REV1 0x0000e0UL
#define ESP_STUB_CMD_FLASH_WRITE_IRAM_ORG_REV1 0x8ff00000UL
#define ESP_STUB_CMD_FLASH_WRITE_IRAM_LEN_REV1 0x00004000UL
#define ESP_STUB_CMD_FLASH_WRITE_DRAM_ORG_REV1 0x8ff04000UL
#define ESP_STUB_CMD_FLASH_WRITE_DRAM_LEN_REV1 0x00020000UL
#define ESP_STUB_CMD_FLASH_WRITE_ENTRY_ADDR_REV1 0x8ff0078eUL
#define ESP_STUB_CMD_FLASH_WRITE_APPTRACE_CTRL_ADDR_REV1 0x8ff04094UL
#define ESP_STUB_CMD_FLASH_WRITE_LOG_ADDR_REV1 0x0UL
#define ESP_STUB_CMD_FLASH_WRITE_LOG_SIZE_REV1 0x0UL
#define ESP_STUB_CMD_FLASH_WRITE_TRAP_RECORD_ADDR_REV1 0x8ff040a0UL
#define ESP_STUB_CMD_FLASH_WRITE_TRAP_ENTRY_ADDR_REV1 0x8ff00a00UL

static const uint8_t s_esp_flasher_stub_cmd_flash_write_code_rev1[] = {
#include "contrib/loaders/flash/espressif/images/esp32p4-rev1/stub_cmd_flash_write_code_rev1.inc"
};

static const uint8_t s_esp_flasher_stub_cmd_flash_write_data_rev1[] = {
#include "contrib/loaders/flash/espressif/images/esp32p4-rev1/stub_cmd_flash_write_data_rev1.inc"
};

static const struct esp_flasher_stub_config s_esp_stub_cmd_flash_write_cfg_rev1 = {
	.name = "cmd_flash_write",
	.code = s_esp_flasher_stub_cmd_flash_write_code_rev1,
	.code_sz = sizeof(s_esp_flasher_stub_cmd_flash_write_code_rev1),
	.data = s_esp_flasher_stub_cmd_flash_write_data_rev1,
	.data_sz = sizeof(s_esp_flasher_stub_cmd_flash_write_data_rev1),
	.entry_addr = ESP_STUB_CMD_FLASH_WRITE_ENTRY_ADDR_REV1,
	.bss_sz = ESP_STUB_CMD_FLASH_WRITE_BSS_SIZE_REV1,
	.apptrace_ctrl_addr = ESP_STUB_CMD_FLASH_WRITE_APPTRACE_CTRL_ADDR_REV1,
	.stack_default_sz = ESP_STUB_STACK_SIZE_REV1,
	.log_buff_addr = ESP_STUB_CMD_FLASH_WRITE_LOG_ADDR_REV1,
	.log_buff_size = ESP_STUB_CMD_FLASH_WRITE_LOG_SIZE_REV1,
	.trap_record_addr = ESP_STUB_CMD_FLASH_WRITE_TRAP_RECORD_ADDR_REV1,
	.trap_entry_addr = ESP_STUB_CMD_FLASH_WRITE_TRAP_ENTRY_ADDR_REV1,
	.iram_org = ESP_STUB_CMD_FLASH_WRITE_IRAM_ORG_REV1,
	.iram_len = ESP_STUB_CMD_FLASH_WRITE_IRAM_LEN_REV1,
	.dram_org = ESP_STUB_CMD_FLASH_WRITE_DRAM_ORG_REV1,
	.dram_len = ESP_STUB_CMD_FLASH_WRITE_DRAM_LEN_REV1,
};

// cmd_flash_erase definitions
#define ESP_STUB_CMD_FLASH_ERASE_BSS_SIZE_REV1 0x000082UL
#define ESP_STUB_CMD_FLASH_ERASE_IRAM_ORG_REV1 0x8ff00000UL
#define ESP_STUB_CMD_FLASH_ERASE_IRAM_LEN_REV1 0x00004000UL
#define ESP_STUB_CMD_FLASH_ERASE_DRAM_ORG_REV1 0x8ff04000UL
#define ESP_STUB_CMD_FLASH_ERASE_DRAM_LEN_REV1 0x00020000UL
#define ESP_STUB_CMD_FLASH_ERASE_ENTRY_ADDR_REV1 0x8ff0036aUL
#define ESP_STUB_CMD_FLASH_ERASE_APPTRACE_CTRL_ADDR_REV1 0x0UL
#define ESP_STUB_CMD_FLASH_ERASE_LOG_ADDR_REV1 0x0UL
#define ESP_STUB_CMD_FLASH_ERASE_LOG_SIZE_REV1 0x0UL
#define ESP_STUB_CMD_FLASH_ERASE_TRAP_RECORD_ADDR_REV1 0x8ff04050UL
#define ESP_STUB_CMD_FLASH_ERASE_TRAP_ENTRY_ADDR_REV1 0x8ff00500UL

static const uint8_t s_esp_flasher_stub_cmd_flash_erase_code_rev1[] = {
#include "contrib/loaders/flash/espressif/images/esp32p4-rev1/stub_cmd_flash_erase_code_rev1.inc"
};

static const uint8_t s_esp_flasher_stub_cmd_flash_erase_data_rev1[] = {
#include "contrib/loaders/flash/espressif/images/esp32p4-rev1/stub_cmd_flash_erase_data_rev1.inc"
};

static const struct esp_flasher_stub_config s_esp_stub_cmd_flash_erase_cfg_rev1 = {
	.name = "cmd_flash_erase",
	.code = s_esp_flasher_stub_cmd_flash_erase_code_rev1,
	.code_sz = sizeof(s_esp_flasher_stub_cmd_flash_erase_code_rev1),
	.data = s_esp_flasher_stub_cmd_flash_erase_data_rev1,
	.data_sz = sizeof(s_esp_flasher_stub_cmd_flash_erase_data_rev1),
	.entry_addr = ESP_STUB_CMD_FLASH_ERASE_ENTRY_ADDR_REV1,
	.bss_sz = ESP_STUB_CMD_FLASH_ERASE_BSS_SIZE_REV1,
	.apptrace_ctrl_addr = ESP_STUB_CMD_FLASH_ERASE_APPTRACE_CTRL_ADDR_REV1,
	.stack_default_sz = ESP_STUB_STACK_SIZE_REV1,
	.log_buff_addr = ESP_STUB_CMD_FLASH_ERASE_LOG_ADDR_REV1,
	.log_buff_size = ESP_STUB_CMD_FLASH_ERASE_LOG_SIZE_REV1,
	.trap_record_addr = ESP_STUB_CMD_FLASH_ERASE_TRAP_RECORD_ADDR_REV1,
	.trap_entry_addr = ESP_STUB_CMD_FLASH_ERASE_TRAP_ENTRY_ADDR_REV1,
	.iram_org = ESP_STUB_CMD_FLASH_ERASE_IRAM_ORG_REV1,
	.iram_len = ESP_STUB_CMD_FLASH_ERASE_IRAM_LEN_REV1,
	.dram_org = ESP_STUB_CMD_FLASH_ERASE_DRAM_ORG_REV1,
	.dram_len = ESP_STUB_CMD_FLASH_ERASE_DRAM_LEN_REV1,
};

// cmd_flash_erase_check definitions
#define ESP_STUB_CMD_FLASH_ERASE_CHECK_BSS_SIZE_REV1 0x0000b5UL
#define ESP_STUB_CMD_FLASH_ERASE_CHECK_IRAM_ORG_REV1 0x8ff00000UL
#define ESP_STUB_CMD_FLASH_ERASE_CHECK_IRAM_LEN_REV1 0x00004000UL
#define ESP_STUB_CMD_FLASH_ERASE_CHECK_DRAM_ORG_REV1 0x8ff04000UL
#define ESP_STUB_CMD_FLASH_ERASE_CHECK_DRAM_LEN_REV1 0x00020000UL
#define ESP_STUB_CMD_FLASH_ERASE_CHECK_ENTRY_ADDR_REV1 0x8ff0037aUL
#define ESP_STUB_CMD_FLASH_ERASE_CHECK_APPTRACE_CTRL_ADDR_REV1 0x0UL
#define ESP_STUB_CMD_FLASH_ERASE_CHECK_LOG_ADDR_REV1 0x0UL
#define ESP_STUB_CMD_FLASH_ERASE_CHECK_LOG_SIZE_REV1 0x0UL
#define ESP_STUB_CMD_FLASH_ERASE_CHECK_TRAP_RECORD_ADDR_REV1 0x8ff04084UL
#define ESP_STUB_CMD_FLASH_ERASE_CHECK_TRAP_ENTRY_ADDR_REV1 0x8ff00600UL

static const uint8_t s_esp_flasher_stub_cmd_flash_erase_check_code_rev1[] = {
#include "contrib/loaders/flash/espressif/images/esp32p4-rev1/stub_cmd_flash_erase_check_code_rev1.inc"
};

static const uint8_t s_esp_flasher_stub_cmd_flash_erase_check_data_rev1[] = {
#include "contrib/loaders/flash/espressif/images/esp32p4-rev1/stub_cmd_flash_erase_check_data_rev1.inc"
};

static const struct esp_flasher_stub_config s_esp_stub_cmd_flash_erase_check_cfg_rev1 = {
	.name = "cmd_flash_erase_check",
	.code = s_esp_flasher_stub_cmd_flash_erase_check_code_rev1,
	.code_sz = sizeof(s_esp_flasher_stub_cmd_flash_erase_check_code_rev1),
	.data = s_esp_flasher_stub_cmd_flash_erase_check_data_rev1,
	.data_sz = sizeof(s_esp_flasher_stub_cmd_flash_erase_check_data_rev1),
	.entry_addr = ESP_STUB_CMD_FLASH_ERASE_CHECK_ENTRY_ADDR_REV1,
	.bss_sz = ESP_STUB_CMD_FLASH_ERASE_CHECK_BSS_SIZE_REV1,
	.apptrace_ctrl_addr = ESP_STUB_CMD_FLASH_ERASE_CHECK_APPTRACE_CTRL_ADDR_REV1,
	.stack_default_sz = ESP_STUB_STACK_SIZE_REV1,
	.log_buff_addr = ESP_STUB_CMD_FLASH_ERASE_CHECK_LOG_ADDR_REV1,
	.log_buff_size = ESP_STUB_CMD_FLASH_ERASE_CHECK_LOG_SIZE_REV1,
	.trap_record_addr = ESP_STUB_CMD_FLASH_ERASE_CHECK_TRAP_RECORD_ADDR_REV1,
	.trap_entry_addr = ESP_STUB_CMD_FLASH_ERASE_CHECK_TRAP_ENTRY_ADDR_REV1,
	.iram_org = ESP_STUB_CMD_FLASH_ERASE_CHECK_IRAM_ORG_REV1,
	.iram_len = ESP_STUB_CMD_FLASH_ERASE_CHECK_IRAM_LEN_REV1,
	.dram_org = ESP_STUB_CMD_FLASH_ERASE_CHECK_DRAM_ORG_REV1,
	.dram_len = ESP_STUB_CMD_FLASH_ERASE_CHECK_DRAM_LEN_REV1,
};

// cmd_flash_map_get definitions
#define ESP_STUB_CMD_FLASH_MAP_GET_BSS_SIZE_REV1 0x0000b5UL
#define ESP_STUB_CMD_FLASH_MAP_GET_IRAM_ORG_REV1 0x8ff00000UL
#define ESP_STUB_CMD_FLASH_MAP_GET_IRAM_LEN_REV1 0x00004000UL
#define ESP_STUB_CMD_FLASH_MAP_GET_DRAM_ORG_REV1 0x8ff04000UL
#define ESP_STUB_CMD_FLASH_MAP_GET_DRAM_LEN_REV1 0x00020000UL
#define ESP_STUB_CMD_FLASH_MAP_GET_ENTRY_ADDR_REV1 0x8ff00510UL
#define ESP_STUB_CMD_FLASH_MAP_GET_APPTRACE_CTRL_ADDR_REV1 0x0UL
#define ESP_STUB_CMD_FLASH_MAP_GET_LOG_ADDR_REV1 0x0UL
#define ESP_STUB_CMD_FLASH_MAP_GET_LOG_SIZE_REV1 0x0UL
#define ESP_STUB_CMD_FLASH_MAP_GET_TRAP_RECORD_ADDR_REV1 0x8ff04080UL
#define ESP_STUB_CMD_FLASH_MAP_GET_TRAP_ENTRY_ADDR_REV1 0x8ff00700UL

static const uint8_t s_esp_flasher_stub_cmd_flash_map_get_code_rev1[] = {
#include "contrib/loaders/flash/espressif/images/esp32p4-rev1/stub_cmd_flash_map_get_code_rev1.inc"
};

static const uint8_t s_esp_flasher_stub_cmd_flash_map_get_data_rev1[] = {
#include "contrib/loaders/flash/espressif/images/esp32p4-rev1/stub_cmd_flash_map_get_data_rev1.inc"
};

static const struct esp_flasher_stub_config s_esp_stub_cmd_flash_map_get_cfg_rev1 = {
	.name = "cmd_flash_map_get",
	.code = s_esp_flasher_stub_cmd_flash_map_get_code_rev1,
	.code_sz = sizeof(s_esp_flasher_stub_cmd_flash_map_get_code_rev1),
	.data = s_esp_flasher_stub_cmd_flash_map_get_data_rev1,
	.data_sz = sizeof(s_esp_flasher_stub_cmd_flash_map_get_data_rev1),
	.entry_addr = ESP_STUB_CMD_FLASH_MAP_GET_ENTRY_ADDR_REV1,
	.bss_sz = ESP_STUB_CMD_FLASH_MAP_GET_BSS_SIZE_REV1,
	.apptrace_ctrl_addr = ESP_STUB_CMD_FLASH_MAP_GET_APPTRACE_CTRL_ADDR_REV1,
	.stack_default_sz = ESP_STUB_STACK_SIZE_REV1,
	.log_buff_addr = ESP_STUB_CMD_FLASH_MAP_GET_LOG_ADDR_REV1,
	.log_buff_size = ESP_STUB_CMD_FLASH_MAP_GET_LOG_SIZE_REV1,
	.trap_record_addr = ESP_STUB_CMD_FLASH_MAP_GET_TRAP_RECORD_ADDR_REV1,
	.trap_entry_addr = ESP_STUB_CMD_FLASH_MAP_GET_TRAP_ENTRY_ADDR_REV1,
	.iram_org = ESP_STUB_CMD_FLASH_MAP_GET_IRAM_ORG_REV1,
	.iram_len = ESP_STUB_CMD_FLASH_MAP_GET_IRAM_LEN_REV1,
	.dram_org = ESP_STUB_CMD_FLASH_MAP_GET_DRAM_ORG_REV1,
	.dram_len = ESP_STUB_CMD_FLASH_MAP_GET_DRAM_LEN_REV1,
};

// cmd_flash_bp_set definitions
#define ESP_STUB_CMD_FLASH_BP_SET_BSS_SIZE_REV1 0x0000b7UL
#define ESP_STUB_CMD_FLASH_BP_SET_IRAM_ORG_REV1 0x8ff00000UL
#define ESP_STUB_CMD_FLASH_BP_SET_IRAM_LEN_REV1 0x00004000UL
#define ESP_STUB_CMD_FLASH_BP_SET_DRAM_ORG_REV1 0x8ff04000UL
#define ESP_STUB_CMD_FLASH_BP_SET_DRAM_LEN_REV1 0x00020000UL
#define ESP_STUB_CMD_FLASH_BP_SET_ENTRY_ADDR_REV1 0x8ff00946UL
#define ESP_STUB_CMD_FLASH_BP_SET_APPTRACE_CTRL_ADDR_REV1 0x0UL
#define ESP_STUB_CMD_FLASH_BP_SET_LOG_ADDR_REV1 0x0UL
#define ESP_STUB_CMD_FLASH_BP_SET_LOG_SIZE_REV1 0x0UL
#define ESP_STUB_CMD_FLASH_BP_SET_TRAP_RECORD_ADDR_REV1 0x8ff04080UL
#define ESP_STUB_CMD_FLASH_BP_SET_TRAP_ENTRY_ADDR_REV1 0x8ff00b00UL

static const uint8_t s_esp_flasher_stub_cmd_flash_bp_set_code_rev1[] = {
#include "contrib/loaders/flash/espressif/images/esp32p4-rev1/stub_cmd_flash_bp_set_code_rev1.inc"
};

static const uint8_t s_esp_flasher_stub_cmd_flash_bp_set_data_rev1[] = {
#include "contrib/loaders/flash/espressif/images/esp32p4-rev1/stub_cmd_flash_bp_set_data_rev1.inc"
};

static const struct esp_flasher_stub_config s_esp_stub_cmd_flash_bp_set_cfg_rev1 = {
	.name = "cmd_flash_bp_set",
	.code = s_esp_flasher_stub_cmd_flash_bp_set_code_rev1,
	.code_sz = sizeof(s_esp_flasher_stub_cmd_flash_bp_set_code_rev1),
	.data = s_esp_flasher_stub_cmd_flash_bp_set_data_rev1,
	.data_sz = sizeof(s_esp_flasher_stub_cmd_flash_bp_set_data_rev1),
	.entry_addr = ESP_STUB_CMD_FLASH_BP_SET_ENTRY_ADDR_REV1,
	.bss_sz = ESP_STUB_CMD_FLASH_BP_SET_BSS_SIZE_REV1,
	.apptrace_ctrl_addr = ESP_STUB_CMD_FLASH_BP_SET_APPTRACE_CTRL_ADDR_REV1,
	.stack_default_sz = ESP_STUB_STACK_SIZE_REV1,
	.log_buff_addr = ESP_STUB_CMD_FLASH_BP_SET_LOG_ADDR_REV1,
	.log_buff_size = ESP_STUB_CMD_FLASH_BP_SET_LOG_SIZE_REV1,
	.trap_record_addr = ESP_STUB_CMD_FLASH_BP_SET_TRAP_RECORD_ADDR_REV1,
	.trap_entry_addr = ESP_STUB_CMD_FLASH_BP_SET_TRAP_ENTRY_ADDR_REV1,
	.iram_org = ESP_STUB_CMD_FLASH_BP_SET_IRAM_ORG_REV1,
	.iram_len = ESP_STUB_CMD_FLASH_BP_SET_IRAM_LEN_REV1,
	.dram_org = ESP_STUB_CMD_FLASH_BP_SET_DRAM_ORG_REV1,
	.dram_len = ESP_STUB_CMD_FLASH_BP_SET_DRAM_LEN_REV1,
};

// cmd_flash_bp_clear definitions
#define ESP_STUB_CMD_FLASH_BP_CLEAR_BSS_SIZE_REV1 0x0000b7UL
#define ESP_STUB_CMD_FLASH_BP_CLEAR_IRAM_ORG_REV1 0x8ff00000UL
#define ESP_STUB_CMD_FLASH_BP_CLEAR_IRAM_LEN_REV1 0x00004000UL
#define ESP_STUB_CMD_FLASH_BP_CLEAR_DRAM_ORG_REV1 0x8ff04000UL
#define ESP_STUB_CMD_FLASH_BP_CLEAR_DRAM_LEN_REV1 0x00020000UL
#define ESP_STUB_CMD_FLASH_BP_CLEAR_ENTRY_ADDR_REV1 0x8ff00930UL
#define ESP_STUB_CMD_FLASH_BP_CLEAR_APPTRACE_CTRL_ADDR_REV1 0x0UL
#define ESP_STUB_CMD_FLASH_BP_CLEAR_LOG_ADDR_REV1 0x0UL
#define ESP_STUB_CMD_FLASH_BP_CLEAR_LOG_SIZE_REV1 0x0UL
#define ESP_STUB_CMD_FLASH_BP_CLEAR_TRAP_RECORD_ADDR_REV1 0x8ff04080UL
#define ESP_STUB_CMD_FLASH_BP_CLEAR_TRAP_ENTRY_ADDR_REV1 0x8ff00b00UL

static const uint8_t s_esp_flasher_stub_cmd_flash_bp_clear_code_rev1[] = {
#include "contrib/loaders/flash/espressif/images/esp32p4-rev1/stub_cmd_flash_bp_clear_code_rev1.inc"
};

static const uint8_t s_esp_flasher_stub_cmd_flash_bp_clear_data_rev1[] = {
#include "contrib/loaders/flash/espressif/images/esp32p4-rev1/stub_cmd_flash_bp_clear_data_rev1.inc"
};

static const struct esp_flasher_stub_config s_esp_stub_cmd_flash_bp_clear_cfg_rev1 = {
	.name = "cmd_flash_bp_clear",
	.code = s_esp_flasher_stub_cmd_flash_bp_clear_code_rev1,
	.code_sz = sizeof(s_esp_flasher_stub_cmd_flash_bp_clear_code_rev1),
	.data = s_esp_flasher_stub_cmd_flash_bp_clear_data_rev1,
	.data_sz = sizeof(s_esp_flasher_stub_cmd_flash_bp_clear_data_rev1),
	.entry_addr = ESP_STUB_CMD_FLASH_BP_CLEAR_ENTRY_ADDR_REV1,
	.bss_sz = ESP_STUB_CMD_FLASH_BP_CLEAR_BSS_SIZE_REV1,
	.apptrace_ctrl_addr = ESP_STUB_CMD_FLASH_BP_CLEAR_APPTRACE_CTRL_ADDR_REV1,
	.stack_default_sz = ESP_STUB_STACK_SIZE_REV1,
	.log_buff_addr = ESP_STUB_CMD_FLASH_BP_CLEAR_LOG_ADDR_REV1,
	.log_buff_size = ESP_STUB_CMD_FLASH_BP_CLEAR_LOG_SIZE_REV1,
	.trap_record_addr = ESP_STUB_CMD_FLASH_BP_CLEAR_TRAP_RECORD_ADDR_REV1,
	.trap_entry_addr = ESP_STUB_CMD_FLASH_BP_CLEAR_TRAP_ENTRY_ADDR_REV1,
	.iram_org = ESP_STUB_CMD_FLASH_BP_CLEAR_IRAM_ORG_REV1,
	.iram_len = ESP_STUB_CMD_FLASH_BP_CLEAR_IRAM_LEN_REV1,
	.dram_org = ESP_STUB_CMD_FLASH_BP_CLEAR_DRAM_ORG_REV1,
	.dram_len = ESP_STUB_CMD_FLASH_BP_CLEAR_DRAM_LEN_REV1,
};

// cmd_flash_write_deflated definitions
#define ESP_STUB_CMD_FLASH_WRITE_DEFLATED_BSS_SIZE_REV1 0x0000e0UL
#define ESP_STUB_CMD_FLASH_WRITE_DEFLATED_IRAM_ORG_REV1 0x8ff00000UL
#define ESP_STUB_CMD_FLASH_WRITE_DEFLATED_IRAM_LEN_REV1 0x00004000UL
#define ESP_STUB_CMD_FLASH_WRITE_DEFLATED_DRAM_ORG_REV1 0x8ff04000UL
#define ESP_STUB_CMD_FLASH_WRITE_DEFLATED_DRAM_LEN_REV1 0x00020000UL
#define ESP_STUB_CMD_FLASH_WRITE_DEFLATED_ENTRY_ADDR_REV1 0x8ff007acUL
#define ESP_STUB_CMD_FLASH_WRITE_DEFLATED_APPTRACE_CTRL_ADDR_REV1 0x8ff040a0UL
#define ESP_STUB_CMD_FLASH_WRITE_DEFLATED_LOG_ADDR_REV1 0x0UL
#define ESP_STUB_CMD_FLASH_WRITE_DEFLATED_LOG_SIZE_REV1 0x0UL
#define ESP_STUB_CMD_FLASH_WRITE_DEFLATED_TRAP_RECORD_ADDR_REV1 0x8ff040acUL
#define ESP_STUB_CMD_FLASH_WRITE_DEFLATED_TRAP_ENTRY_ADDR_REV1 0x8ff00a00UL

static const uint8_t s_esp_flasher_stub_cmd_flash_write_deflated_code_rev1[] = {
#include "contrib/loaders/flash/espressif/images/esp32p4-rev1/stub_cmd_flash_write_deflated_code_rev1.inc"
};

static const uint8_t s_esp_flasher_stub_cmd_flash_write_deflated_data_rev1[] = {
#include "contrib/loaders/flash/espressif/images/esp32p4-rev1/stub_cmd_flash_write_deflated_data_rev1.inc"
};

static const struct esp_flasher_stub_config s_esp_stub_cmd_flash_write_deflated_cfg_rev1 = {
	.name = "cmd_flash_write_deflated",
	.code = s_esp_flasher_stub_cmd_flash_write_deflated_code_rev1,
	.code_sz = sizeof(s_esp_flasher_stub_cmd_flash_write_deflated_code_rev1),
	.data = s_esp_flasher_stub_cmd_flash_write_deflated_data_rev1,
	.data_sz = sizeof(s_esp_flasher_stub_cmd_flash_write_deflated_data_rev1),
	.entry_addr = ESP_STUB_CMD_FLASH_WRITE_DEFLATED_ENTRY_ADDR_REV1,
	.bss_sz = ESP_STUB_CMD_FLASH_WRITE_DEFLATED_BSS_SIZE_REV1,
	.apptrace_ctrl_addr = ESP_STUB_CMD_FLASH_WRITE_DEFLATED_APPTRACE_CTRL_ADDR_REV1,
	.stack_default_sz = ESP_STUB_STACK_SIZE_REV1,
	.log_buff_addr = ESP_STUB_CMD_FLASH_WRITE_DEFLATED_LOG_ADDR_REV1,
	.log_buff_size = ESP_STUB_CMD_FLASH_WRITE_DEFLATED_LOG_SIZE_REV1,
	.trap_record_addr = ESP_STUB_CMD_FLASH_WRITE_DEFLATED_TRAP_RECORD_ADDR_REV1,
	.trap_entry_addr = ESP_STUB_CMD_FLASH_WRITE_DEFLATED_TRAP_ENTRY_ADDR_REV1,
	.iram_org = ESP_STUB_CMD_FLASH_WRITE_DEFLATED_IRAM_ORG_REV1,
	.iram_len = ESP_STUB_CMD_FLASH_WRITE_DEFLATED_IRAM_LEN_REV1,
	.dram_org = ESP_STUB_CMD_FLASH_WRITE_DEFLATED_DRAM_ORG_REV1,
	.dram_len = ESP_STUB_CMD_FLASH_WRITE_DEFLATED_DRAM_LEN_REV1,
};

// cmd_flash_calc_hash definitions
#define ESP_STUB_CMD_FLASH_CALC_HASH_BSS_SIZE_REV1 0x00018dUL
#define ESP_STUB_CMD_FLASH_CALC_HASH_IRAM_ORG_REV1 0x8ff00000UL
#define ESP_STUB_CMD_FLASH_CALC_HASH_IRAM_LEN_REV1 0x00004000UL
#define ESP_STUB_CMD_FLASH_CALC_HASH_DRAM_ORG_REV1 0x8ff04000UL
#define ESP_STUB_CMD_FLASH_CALC_HASH_DRAM_LEN_REV1 0x00020000UL
#define ESP_STUB_CMD_FLASH_CALC_HASH_ENTRY_ADDR_REV1 0x8ff00384UL
#define ESP_STUB_CMD_FLASH_CALC_HASH_APPTRACE_CTRL_ADDR_REV1 0x0UL
#define ESP_STUB_CMD_FLASH_CALC_HASH_LOG_ADDR_REV1 0x0UL
#define ESP_STUB_CMD_FLASH_CALC_HASH_LOG_SIZE_REV1 0x0UL
#define ESP_STUB_CMD_FLASH_CALC_HASH_TRAP_RECORD_ADDR_REV1 0x8ff04158UL
#define ESP_STUB_CMD_FLASH_CALC_HASH_TRAP_ENTRY_ADDR_REV1 0x8ff00600UL

static const uint8_t s_esp_flasher_stub_cmd_flash_calc_hash_code_rev1[] = {
#include "contrib/loaders/flash/espressif/images/esp32p4-rev1/stub_cmd_flash_calc_hash_code_rev1.inc"
};

static const uint8_t s_esp_flasher_stub_cmd_flash_calc_hash_data_rev1[] = {
#include "contrib/loaders/flash/espressif/images/esp32p4-rev1/stub_cmd_flash_calc_hash_data_rev1.inc"
};

static const struct esp_flasher_stub_config s_esp_stub_cmd_flash_calc_hash_cfg_rev1 = {
	.name = "cmd_flash_calc_hash",
	.code = s_esp_flasher_stub_cmd_flash_calc_hash_code_rev1,
	.code_sz = sizeof(s_esp_flasher_stub_cmd_flash_calc_hash_code_rev1),
	.data = s_esp_flasher_stub_cmd_flash_calc_hash_data_rev1,
	.data_sz = sizeof(s_esp_flasher_stub_cmd_flash_calc_hash_data_rev1),
	.entry_addr = ESP_STUB_CMD_FLASH_CALC_HASH_ENTRY_ADDR_REV1,
	.bss_sz = ESP_STUB_CMD_FLASH_CALC_HASH_BSS_SIZE_REV1,
	.apptrace_ctrl_addr = ESP_STUB_CMD_FLASH_CALC_HASH_APPTRACE_CTRL_ADDR_REV1,
	.stack_default_sz = ESP_STUB_STACK_SIZE_REV1,
	.log_buff_addr = ESP_STUB_CMD_FLASH_CALC_HASH_LOG_ADDR_REV1,
	.log_buff_size = ESP_STUB_CMD_FLASH_CALC_HASH_LOG_SIZE_REV1,
	.trap_record_addr = ESP_STUB_CMD_FLASH_CALC_HASH_TRAP_RECORD_ADDR_REV1,
	.trap_entry_addr = ESP_STUB_CMD_FLASH_CALC_HASH_TRAP_ENTRY_ADDR_REV1,
	.iram_org = ESP_STUB_CMD_FLASH_CALC_HASH_IRAM_ORG_REV1,
	.iram_len = ESP_STUB_CMD_FLASH_CALC_HASH_IRAM_LEN_REV1,
	.dram_org = ESP_STUB_CMD_FLASH_CALC_HASH_DRAM_ORG_REV1,
	.dram_len = ESP_STUB_CMD_FLASH_CALC_HASH_DRAM_LEN_REV1,
};

// cmd_flash_clock_configure definitions
#define ESP_STUB_CMD_FLASH_CLOCK_CONFIGURE_BSS_SIZE_REV1 0x000082UL
#define ESP_STUB_CMD_FLASH_CLOCK_CONFIGURE_IRAM_ORG_REV1 0x8ff00000UL
#define ESP_STUB_CMD_FLASH_CLOCK_CONFIGURE_IRAM_LEN_REV1 0x00004000UL
#define ESP_STUB_CMD_FLASH_CLOCK_CONFIGURE_DRAM_ORG_REV1 0x8ff04000UL
#define ESP_STUB_CMD_FLASH_CLOCK_CONFIGURE_DRAM_LEN_REV1 0x00020000UL
#define ESP_STUB_CMD_FLASH_CLOCK_CONFIGURE_ENTRY_ADDR_REV1 0x8ff0009cUL
#define ESP_STUB_CMD_FLASH_CLOCK_CONFIGURE_APPTRACE_CTRL_ADDR_REV1 0x0UL
#define ESP_STUB_CMD_FLASH_CLOCK_CONFIGURE_LOG_ADDR_REV1 0x0UL
#define ESP_STUB_CMD_FLASH_CLOCK_CONFIGURE_LOG_SIZE_REV1 0x0UL
#define ESP_STUB_CMD_FLASH_CLOCK_CONFIGURE_TRAP_RECORD_ADDR_REV1 0x8ff0405cUL
#define ESP_STUB_CMD_FLASH_CLOCK_CONFIGURE_TRAP_ENTRY_ADDR_REV1 0x8ff00300UL

static const uint8_t s_esp_flasher_stub_cmd_flash_clock_configure_code_rev1[] = {
#include "contrib/loaders/flash/espressif/images/esp32p4-rev1/stub_cmd_flash_clock_configure_code_rev1.inc"
};

static const uint8_t s_esp_flasher_stub_cmd_flash_clock_configure_data_rev1[] = {
#include "contrib/loaders/flash/espressif/images/esp32p4-rev1/stub_cmd_flash_clock_configure_data_rev1.inc"
};

static const struct esp_flasher_stub_config s_esp_stub_cmd_flash_clock_configure_cfg_rev1 = {
	.name = "cmd_flash_clock_configure",
	.code = s_esp_flasher_stub_cmd_flash_clock_configure_code_rev1,
	.code_sz = sizeof(s_esp_flasher_stub_cmd_flash_clock_configure_code_rev1),
	.data = s_esp_flasher_stub_cmd_flash_clock_configure_data_rev1,
	.data_sz = sizeof(s_esp_flasher_stub_cmd_flash_clock_configure_data_rev1),
	.entry_addr = ESP_STUB_CMD_FLASH_CLOCK_CONFIGURE_ENTRY_ADDR_REV1,
	.bss_sz = ESP_STUB_CMD_FLASH_CLOCK_CONFIGURE_BSS_SIZE_REV1,
	.apptrace_ctrl_addr = ESP_STUB_CMD_FLASH_CLOCK_CONFIGURE_APPTRACE_CTRL_ADDR_REV1,
	.stack_default_sz = ESP_STUB_STACK_SIZE_REV1,
	.log_buff_addr = ESP_STUB_CMD_FLASH_CLOCK_CONFIGURE_LOG_ADDR_REV1,
	.log_buff_size = ESP_STUB_CMD_FLASH_CLOCK_CONFIGURE_LOG_SIZE_REV1,
	.trap_record_addr = ESP_STUB_CMD_FLASH_CLOCK_CONFIGURE_TRAP_RECORD_ADDR_REV1,
	.trap_entry_addr = ESP_STUB_CMD_FLASH_CLOCK_CONFIGURE_TRAP_ENTRY_ADDR_REV1,
	.iram_org = ESP_STUB_CMD_FLASH_CLOCK_CONFIGURE_IRAM_ORG_REV1,
	.iram_len = ESP_STUB_CMD_FLASH_CLOCK_CONFIGURE_IRAM_LEN_REV1,
	.dram_org = ESP_STUB_CMD_FLASH_CLOCK_CONFIGURE_DRAM_ORG_REV1,
	.dram_len = ESP_STUB_CMD_FLASH_CLOCK_CONFIGURE_DRAM_LEN_REV1,
};

// cmd_flash_multi_command definitions
#define ESP_STUB_CMD_FLASH_MULTI_COMMAND_BSS_SIZE_REV1 0x0000b7UL
#define ESP_STUB_CMD_FLASH_MULTI_COMMAND_IRAM_ORG_REV1 0x8ff00000UL
#define ESP_STUB_CMD_FLASH_MULTI_COMMAND_IRAM_LEN_REV1 0x00004000UL
#define ESP_STUB_CMD_FLASH_MULTI_COMMAND_DRAM_ORG_REV1 0x8ff04000UL
#define ESP_STUB_CMD_FLASH_MULTI_COMMAND_DRAM_LEN_REV1 0x00020000UL
#define ESP_STUB_CMD_FLASH_MULTI_COMMAND_ENTRY_ADDR_REV1 0x8ff00d48UL
#define ESP_STUB_CMD_FLASH_MULTI_COMMAND_APPTRACE_CTRL_ADDR_REV1 0x0UL
#define ESP_STUB_CMD_FLASH_MULTI_COMMAND_LOG_ADDR_REV1 0x0UL
#define ESP_STUB_CMD_FLASH_MULTI_COMMAND_LOG_SIZE_REV1 0x0UL
#define ESP_STUB_CMD_FLASH_MULTI_COMMAND_TRAP_RECORD_ADDR_REV1 0x8ff040c0UL
#define ESP_STUB_CMD_FLASH_MULTI_COMMAND_TRAP_ENTRY_ADDR_REV1 0x8ff00f00UL

static const uint8_t s_esp_flasher_stub_cmd_flash_multi_command_code_rev1[] = {
#include "contrib/loaders/flash/espressif/images/esp32p4-rev1/stub_cmd_flash_multi_command_code_rev1.inc"
};

static const uint8_t s_esp_flasher_stub_cmd_flash_multi_command_data_rev1[] = {
#include "contrib/loaders/flash/espressif/images/esp32p4-rev1/stub_cmd_flash_multi_command_data_rev1.inc"
};

static const struct esp_flasher_stub_config s_esp_stub_cmd_flash_multi_command_cfg_rev1 = {
	.name = "cmd_flash_multi_command",
	.code = s_esp_flasher_stub_cmd_flash_multi_command_code_rev1,
	.code_sz = sizeof(s_esp_flasher_stub_cmd_flash_multi_command_code_rev1),
	.data = s_esp_flasher_stub_cmd_flash_multi_command_data_rev1,
	.data_sz = sizeof(s_esp_flasher_stub_cmd_flash_multi_command_data_rev1),
	.entry_addr = ESP_STUB_CMD_FLASH_MULTI_COMMAND_ENTRY_ADDR_REV1,
	.bss_sz = ESP_STUB_CMD_FLASH_MULTI_COMMAND_BSS_SIZE_REV1,
	.apptrace_ctrl_addr = ESP_STUB_CMD_FLASH_MULTI_COMMAND_APPTRACE_CTRL_ADDR_REV1,
	.stack_default_sz = ESP_STUB_STACK_SIZE_REV1,
	.log_buff_addr = ESP_STUB_CMD_FLASH_MULTI_COMMAND_LOG_ADDR_REV1,
	.log_buff_size = ESP_STUB_CMD_FLASH_MULTI_COMMAND_LOG_SIZE_REV1,
	.trap_record_addr = ESP_STUB_CMD_FLASH_MULTI_COMMAND_TRAP_RECORD_ADDR_REV1,
	.trap_entry_addr = ESP_STUB_CMD_FLASH_MULTI_COMMAND_TRAP_ENTRY_ADDR_REV1,
	.iram_org = ESP_STUB_CMD_FLASH_MULTI_COMMAND_IRAM_ORG_REV1,
	.iram_len = ESP_STUB_CMD_FLASH_MULTI_COMMAND_IRAM_LEN_REV1,
	.dram_org = ESP_STUB_CMD_FLASH_MULTI_COMMAND_DRAM_ORG_REV1,
	.dram_len = ESP_STUB_CMD_FLASH_MULTI_COMMAND_DRAM_LEN_REV1,
};

// cmd_flash_idf_binary definitions
#define ESP_STUB_CMD_FLASH_IDF_BINARY_BSS_SIZE_REV1 0x0000b7UL
#define ESP_STUB_CMD_FLASH_IDF_BINARY_IRAM_ORG_REV1 0x8ff00000UL
#define ESP_STUB_CMD_FLASH_IDF_BINARY_IRAM_LEN_REV1 0x00004000UL
#define ESP_STUB_CMD_FLASH_IDF_BINARY_DRAM_ORG_REV1 0x8ff04000UL
#define ESP_STUB_CMD_FLASH_IDF_BINARY_DRAM_LEN_REV1 0x00020000UL
#define ESP_STUB_CMD_FLASH_IDF_BINARY_ENTRY_ADDR_REV1 0x8ff00d48UL
#define ESP_STUB_CMD_FLASH_IDF_BINARY_APPTRACE_CTRL_ADDR_REV1 0x0UL
#define ESP_STUB_CMD_FLASH_IDF_BINARY_LOG_ADDR_REV1 0x0UL
#define ESP_STUB_CMD_FLASH_IDF_BINARY_LOG_SIZE_REV1 0x0UL
#define ESP_STUB_CMD_FLASH_IDF_BINARY_TRAP_RECORD_ADDR_REV1 0x8ff040c0UL
#define ESP_STUB_CMD_FLASH_IDF_BINARY_TRAP_ENTRY_ADDR_REV1 0x8ff00f00UL

static const uint8_t s_esp_flasher_stub_cmd_flash_idf_binary_code_rev1[] = {
#include "contrib/loaders/flash/espressif/images/esp32p4-rev1/stub_cmd_flash_idf_binary_code_rev1.inc"
};

static const uint8_t s_esp_flasher_stub_cmd_flash_idf_binary_data_rev1[] = {
#include "contrib/loaders/flash/espressif/images/esp32p4-rev1/stub_cmd_flash_idf_binary_data_rev1.inc"
};

static const struct esp_flasher_stub_config s_esp_stub_cmd_flash_idf_binary_cfg_rev1 = {
	.name = "cmd_flash_idf_binary",
	.code = s_esp_flasher_stub_cmd_flash_idf_binary_code_rev1,
	.code_sz = sizeof(s_esp_flasher_stub_cmd_flash_idf_binary_code_rev1),
	.data = s_esp_flasher_stub_cmd_flash_idf_binary_data_rev1,
	.data_sz = sizeof(s_esp_flasher_stub_cmd_flash_idf_binary_data_rev1),
	.entry_addr = ESP_STUB_CMD_FLASH_IDF_BINARY_ENTRY_ADDR_REV1,
	.bss_sz = ESP_STUB_CMD_FLASH_IDF_BINARY_BSS_SIZE_REV1,
	.apptrace_ctrl_addr = ESP_STUB_CMD_FLASH_IDF_BINARY_APPTRACE_CTRL_ADDR_REV1,
	.stack_default_sz = ESP_STUB_STACK_SIZE_REV1,
	.log_buff_addr = ESP_STUB_CMD_FLASH_IDF_BINARY_LOG_ADDR_REV1,
	.log_buff_size = ESP_STUB_CMD_FLASH_IDF_BINARY_LOG_SIZE_REV1,
	.trap_record_addr = ESP_STUB_CMD_FLASH_IDF_BINARY_TRAP_RECORD_ADDR_REV1,
	.trap_entry_addr = ESP_STUB_CMD_FLASH_IDF_BINARY_TRAP_ENTRY_ADDR_REV1,
	.iram_org = ESP_STUB_CMD_FLASH_IDF_BINARY_IRAM_ORG_REV1,
	.iram_len = ESP_STUB_CMD_FLASH_IDF_BINARY_IRAM_LEN_REV1,
	.dram_org = ESP_STUB_CMD_FLASH_IDF_BINARY_DRAM_ORG_REV1,
	.dram_len = ESP_STUB_CMD_FLASH_IDF_BINARY_DRAM_LEN_REV1,
};

// cmd_test_all definitions
#define ESP_STUB_CMD_TEST_ALL_BSS_SIZE_REV1 0x0021f0UL
#define ESP_STUB_CMD_TEST_ALL_IRAM_ORG_REV1 0x8ff00000UL
#define ESP_STUB_CMD_TEST_ALL_IRAM_LEN_REV1 0x00004000UL
#define ESP_STUB_CMD_TEST_ALL_DRAM_ORG_REV1 0x8ff04000UL
#define ESP_STUB_CMD_TEST_ALL_DRAM_LEN_REV1 0x00020000UL
#define ESP_STUB_CMD_TEST_ALL_ENTRY_ADDR_REV1 0x8ff0262eUL
#define ESP_STUB_CMD_TEST_ALL_APPTRACE_CTRL_ADDR_REV1 0x8ff05ac8UL
#define ESP_STUB_CMD_TEST_ALL_LOG_ADDR_REV1 0x8ff05c70UL
#define ESP_STUB_CMD_TEST_ALL_LOG_SIZE_REV1 0x8196UL
#define ESP_STUB_CMD_TEST_ALL_TRAP_RECORD_ADDR_REV1 0x8ff05bf0UL
#define ESP_STUB_CMD_TEST_ALL_TRAP_ENTRY_ADDR_REV1 0x8ff02a00UL

static const uint8_t s_esp_flasher_stub_cmd_test_all_code_rev1[] = {
#include "contrib/loaders/flash/espressif/images/esp32p4-rev1/stub_cmd_test_all_code_rev1.inc"
};

static const uint8_t s_esp_flasher_stub_cmd_test_all_data_rev1[] = {
#include "contrib/loaders/flash/espressif/images/esp32p4-rev1/stub_cmd_test_all_data_rev1.inc"
};

static const struct esp_flasher_stub_config s_esp_stub_cmd_test_all_cfg_rev1 = {
	.name = "cmd_test_all",
	.code = s_esp_flasher_stub_cmd_test_all_code_rev1,
	.code_sz = sizeof(s_esp_flasher_stub_cmd_test_all_code_rev1),
	.data = s_esp_flasher_stub_cmd_test_all_data_rev1,
	.data_sz = sizeof(s_esp_flasher_stub_cmd_test_all_data_rev1),
	.entry_addr = ESP_STUB_CMD_TEST_ALL_ENTRY_ADDR_REV1,
	.bss_sz = ESP_STUB_CMD_TEST_ALL_BSS_SIZE_REV1,
	.apptrace_ctrl_addr = ESP_STUB_CMD_TEST_ALL_APPTRACE_CTRL_ADDR_REV1,
	.stack_default_sz = ESP_STUB_STACK_SIZE_REV1,
	.log_buff_addr = ESP_STUB_CMD_TEST_ALL_LOG_ADDR_REV1,
	.log_buff_size = ESP_STUB_CMD_TEST_ALL_LOG_SIZE_REV1,
	.trap_record_addr = ESP_STUB_CMD_TEST_ALL_TRAP_RECORD_ADDR_REV1,
	.trap_entry_addr = ESP_STUB_CMD_TEST_ALL_TRAP_ENTRY_ADDR_REV1,
	.iram_org = ESP_STUB_CMD_TEST_ALL_IRAM_ORG_REV1,
	.iram_len = ESP_STUB_CMD_TEST_ALL_IRAM_LEN_REV1,
	.dram_org = ESP_STUB_CMD_TEST_ALL_DRAM_ORG_REV1,
	.dram_len = ESP_STUB_CMD_TEST_ALL_DRAM_LEN_REV1,
};

static const struct command_map s_cmd_map_rev1[ESP_STUB_CMD_FLASH_MAX_ID + 1] = {
	{ESP_STUB_CMD_TEST1, &s_esp_stub_cmd_test1_cfg_rev1},
	{ESP_STUB_CMD_RECV_FROM_HOST, &s_esp_stub_cmd_recv_from_host_cfg_rev1},
	{ESP_STUB_CMD_SEND_TO_HOST, &s_esp_stub_cmd_send_to_host_cfg_rev1},
	{ESP_STUB_CMD_FLASH_READ, &s_esp_stub_cmd_flash_read_cfg_rev1},
	{ESP_STUB_CMD_FLASH_WRITE, &s_esp_stub_cmd_flash_write_cfg_rev1},
	{ESP_STUB_CMD_FLASH_ERASE, &s_esp_stub_cmd_flash_erase_cfg_rev1},
	{ESP_STUB_CMD_FLASH_ERASE_CHECK, &s_esp_stub_cmd_flash_erase_check_cfg_rev1},
	{ESP_STUB_CMD_FLASH_MAP_GET, &s_esp_stub_cmd_flash_map_get_cfg_rev1},
	{ESP_STUB_CMD_FLASH_BP_SET, &s_esp_stub_cmd_flash_bp_set_cfg_rev1},
	{ESP_STUB_CMD_FLASH_BP_CLEAR, &s_esp_stub_cmd_flash_bp_clear_cfg_rev1},
	{ESP_STUB_CMD_FLASH_WRITE_DEFLATED, &s_esp_stub_cmd_flash_write_deflated_cfg_rev1},
	{ESP_STUB_CMD_FLASH_CALC_HASH, &s_esp_stub_cmd_flash_calc_hash_cfg_rev1},
	{ESP_STUB_CMD_FLASH_CLOCK_CONFIGURE, &s_esp_stub_cmd_flash_clock_configure_cfg_rev1},
	{ESP_STUB_CMD_FLASH_MULTI_COMMAND, &s_esp_stub_cmd_flash_multi_command_cfg_rev1},
	{ESP_STUB_CMD_FLASH_IDF_BINARY, &s_esp_stub_cmd_flash_idf_binary_cfg_rev1},
	{ESP_STUB_CMD_TEST_ALL, &s_esp_stub_cmd_test_all_cfg_rev1},
};
