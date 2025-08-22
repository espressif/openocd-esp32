/* SPDX-License-Identifier: Apache-2.0 OR MIT */
/* SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD */

#pragma once

/* Espressif App Image Format */

#include <stdint.h>
#include <assert.h>

#define ESP_IMAGE_HEADER_MAGIC 0xE9
#define ESP_IMAGE_HEADER_SIZE 24
#define ESP_IMAGE_SEGMENT_HEADER_SIZE 8
#define ESP_IMAGE_MAX_SEGMENTS 16

struct esp_image_header {
	uint8_t magic;              /*!< Magic word ESP_IMAGE_HEADER_MAGIC */
	uint8_t segment_count;      /*!< Count of memory segments */
	uint8_t spi_mode;
	uint8_t spi_speed: 4;
	uint8_t spi_size: 4;
	uint32_t entry_addr;
	uint8_t wp_pin;
	uint8_t spi_pin_drv[3];
	uint16_t chip_id;
	uint8_t min_chip_rev;
	uint16_t min_chip_rev_full;
	uint16_t max_chip_rev_full;
	uint8_t reserved[4];
	uint8_t hash_appended;
} __attribute__((packed));

static_assert(sizeof(struct esp_image_header) == ESP_IMAGE_HEADER_SIZE, "struct has incorrest size");

struct esp_image_segment_header {
	uint32_t load_addr;         /*!< Virtual address of segment */
	uint32_t data_len;          /*!< Length of data */
};

static_assert(sizeof(struct esp_image_segment_header) == ESP_IMAGE_SEGMENT_HEADER_SIZE, "struct has incorrest size");
