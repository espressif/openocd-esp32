/* SPDX-License-Identifier: Apache-2.0 OR MIT */
/* SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD */

#pragma once

/* Espressif Partition Table Format */

#include <stdint.h>
#include <assert.h>

#define ESP_PARTITION_TABLE_OFFSET  (0x8000)
#define ESP_PARTITION_MAGIC         (0x50AA)
#define ESP_PARTITION_TYPE_APP      (0x00)
#define ESP_PARTITION_INFO_SIZE     (0x20)

// Partition Table entry
struct esp_partition_info {
	uint16_t magic; // ESP_PARTITION_MAGIC
	uint8_t  type;
	uint8_t  subtype;
	struct {
		uint32_t offset;
		uint32_t size;
	} pos;
	uint8_t  label[16];
	uint32_t flags;
};

static_assert(sizeof(struct esp_partition_info) == ESP_PARTITION_INFO_SIZE, "struct has incorrest size");
