/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 */

#pragma once

#include <stdint.h>

#define ESP_STUB_VERSION     2
#define ESP_STUB_MAGIC_NUM   0xFEEDFACE

struct esp_stub_desc {
	uint32_t magic;
	uint32_t stub_version;
	uint32_t idf_key; /* idf will set the key with known value. (5C3A9F5A) */
};

#define ESP_STUB_FLASHER_DESC_MAGIC_NUM       (offsetof(struct esp_stub_desc, magic))
#define ESP_STUB_FLASHER_DESC_MAGIC_VERSION   (offsetof(struct esp_stub_desc, stub_version))
#define ESP_STUB_FLASHER_DESC_IDF_KEY         (offsetof(struct esp_stub_desc, idf_key))
#define ESP_STUB_FLASHER_DESC_SIZE            (sizeof(struct esp_stub_desc))

// Define command numbers. Should be the same order as the commands in the root CMakeLists.txt
enum esp_stub_cmd {
	ESP_STUB_CMD_TEST1 = 0x00,
	ESP_STUB_CMD_TEST2,
	ESP_STUB_CMD_ALL,
	ESP_STUB_CMD_FLASH_MAX_ID = ESP_STUB_CMD_ALL,
};
