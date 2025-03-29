/* SPDX-License-Identifier: Apache-2.0 OR MIT */
/* SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD */

#pragma once

#include <stdint.h>
#include "esp_stub_err.h"

struct esp_stub_desc {
	uint32_t magic;
	uint32_t stub_version;
	uint32_t idf_key; /* idf will set the key with known value. (5C3A9F5A) */
};

#define ESP_STUB_FLASHER_DESC_MAGIC_NUM       (offsetof(struct esp_stub_desc, magic))
#define ESP_STUB_FLASHER_DESC_MAGIC_VERSION   (offsetof(struct esp_stub_desc, stub_version))
#define ESP_STUB_FLASHER_DESC_IDF_KEY         (offsetof(struct esp_stub_desc, idf_key))
#define ESP_STUB_FLASHER_DESC_SIZE            (sizeof(struct esp_stub_desc))

#define ESP_STUB_FLASHER_MAGIC_NUM            0xFEEDFACE

/*
The IDF key is important when the stub code and data work area backup feature are disabled.
For example, the program_esp_bins command disables the work area backup to increase programming speed.
After programming, if the chip is not reset, we can see the stub descriptor header in the target memory and might
assume that there is a stub code loaded. However, this assumption is not correct for every chip.
To differentiate whether the target memory is loaded from JTAG or the IDF application, we will use the idf_key field.
This field will be zero if the stub code is loaded from JTAG.
OpenOCD will reload the stub code when it reads this field as all zeros.
Otherwise, for specific commands, the stub will not be loaded and will run directly from memory.
*/
#define ESP_STUB_FLASHER_IDF_KEY               0x5C3A9F5A

/*
Bump the version if there are any changes related to mapping, bp setting, or removal.
In other words, if the binary generated for the `ESP_STUB_CMD_FLASH_IDF_BINARY` command changes,
we need to increase the version and update the related .inc files in the
esp-idf/components/esp_system/openocd_stub_bins/ directory.
*/
#define ESP_STUB_FLASHER_VERSION               2

// Define command numbers. Should be the same order as the commands in the root CMakeLists.txt
enum {
	/* Test commands */
	ESP_STUB_CMD_TEST1 = 0x00,
	ESP_STUB_CMD_RECV_FROM_HOST,
	ESP_STUB_CMD_SEND_TO_HOST,

	/* Flash commands */
	ESP_STUB_CMD_FLASH_READ,
	ESP_STUB_CMD_FLASH_WRITE,
	ESP_STUB_CMD_FLASH_ERASE,
	ESP_STUB_CMD_FLASH_ERASE_CHECK,
	ESP_STUB_CMD_FLASH_MAP_GET,
	ESP_STUB_CMD_FLASH_BP_SET,
	ESP_STUB_CMD_FLASH_BP_CLEAR,
	ESP_STUB_CMD_FLASH_WRITE_DEFLATED,
	ESP_STUB_CMD_FLASH_CALC_HASH,
	ESP_STUB_CMD_FLASH_CLOCK_CONFIGURE,
	ESP_STUB_CMD_FLASH_MULTI_COMMAND,
	ESP_STUB_CMD_FLASH_IDF_BINARY,
	ESP_STUB_CMD_TEST_ALL,
	ESP_STUB_CMD_FLASH_MAX_ID = ESP_STUB_CMD_TEST_ALL,
};

/* exported to let openocd know for stack allocation */
#define ESP_STUB_UNZIP_BUFF_SIZE                32768
#define ESP_STUB_IFLATOR_SIZE                   11520
#define ESP_STUB_RDWR_BUFF_SIZE                 32768

/* stub runtime options */
#define ESP_STUB_FLASH_WR_RAW                   0x0
#define ESP_STUB_FLASH_ENCRYPT_BINARY           0x1

#define ESP_STUB_FLASH_MAPPINGS_MAX_NUM         2  /* IROM, DROM */

struct esp_flash_region_mapping {
	uint32_t phy_addr;
	uint32_t load_addr;
	uint32_t size;
};

struct esp_flash_mapping {
	uint32_t maps_num;
	struct esp_flash_region_mapping maps[ESP_STUB_FLASH_MAPPINGS_MAX_NUM];
};

#define ESP_STUB_FLASHMAP_MAPSNUM_OFF \
	(offsetof(struct esp_flash_mapping, maps_num))
#define ESP_STUB_FLASHMAP_MAPS_OFF(_i_) \
	(offsetof(struct esp_flash_mapping, maps) + (_i_) * sizeof(struct esp_flash_region_mapping))
#define ESP_STUB_FLASHMAP_PHYADDR_OFF(_i_) \
	(ESP_STUB_FLASHMAP_MAPS_OFF(_i_) + offsetof(struct esp_flash_region_mapping, phy_addr))
#define ESP_STUB_FLASHMAP_LOADADDR_OFF(_i_) \
	(ESP_STUB_FLASHMAP_MAPS_OFF(_i_) + offsetof(struct esp_flash_region_mapping, load_addr))
#define ESP_STUB_FLASHMAP_SIZE_OFF(_i_) \
	(ESP_STUB_FLASHMAP_MAPS_OFF(_i_) + offsetof(struct esp_flash_region_mapping, size))

struct esp_stub_flash_map {
	struct esp_flash_mapping map; /* must be at the beginning */
	uint32_t flash_size;
	int32_t retcode;
};

#define ESP_STUB_FLASHMAP_FLASH_SIZE \
	(offsetof(struct esp_stub_flash_map, flash_size))
#define ESP_STUB_FLASHMAP_RETCODE \
	(offsetof(struct esp_stub_flash_map, retcode))

struct esp_flash_stub_flash_write_args {
	uint32_t start_addr;
	uint32_t size;
	uint32_t ring_buf_addr;		/* used for downlink ring buffer address */
	uint32_t ring_buf_size;		/* used for downlink ring buffer size */
	uint32_t total_size;        /* uncompressed file size */
	uint32_t extra_stack_addr;  /* extra stack for compression */
	uint32_t options;           /* Write options. e.g. encrypted */
};

struct esp_flash_stub_bp_instructions {
	uint8_t size;
	uint8_t buff[3];
};
struct stub_flash_info {
	uint32_t id;
	uint32_t size;
	uint32_t block_size;
	uint32_t sector_size;
	uint32_t page_size;
	uint32_t mode; // SPI, Octal, Dual, Quad
	uint32_t encrypted;
};
