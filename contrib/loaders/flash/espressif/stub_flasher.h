/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   ESP xtensa chips flasher stub definitions                             *
 *   Copyright (C) 2023 Espressif Systems Ltd.                             *
 ***************************************************************************/
#ifndef OPENOCD_LOADERS_FLASH_ESPRESSIF_STUB_FLASHER_H
#define OPENOCD_LOADERS_FLASH_ESPRESSIF_STUB_FLASHER_H

#include <stdint.h>

#define ESP_STUB_ERR_OK                         0
#define ESP_STUB_ERR_FAIL                       (-1)
#define ESP_STUB_ERR_NOT_SUPPORTED              (-2)
#define ESP_STUB_ERR_INFLATE                    (-3)
#define ESP_STUB_ERR_NOT_ENOUGH_DATA            (-4)
#define ESP_STUB_ERR_TOO_MUCH_DATA              (-5)
#define ESP_STUB_ERR_INVALID_IMAGE              (-6)
#define ESP_STUB_ERR_INVALID_PARTITION          (-7)
#define ESP_STUB_ERR_INVALID_APP_MAGIC          (-8)
#define ESP_STUB_ERR_FLASH_SIZE                 (-9)
#define ESP_STUB_ERR_READ_PARTITION             (-10)
#define ESP_STUB_ERR_READ_APP_SEGMENT           (-11)
#define ESP_STUB_ERR_READ_APP_IMAGE_HEADER      (-12)

#define ESP_STUB_CMD_FLASH_READ                 0
#define ESP_STUB_CMD_FLASH_WRITE                1
#define ESP_STUB_CMD_FLASH_ERASE                2
#define ESP_STUB_CMD_FLASH_ERASE_CHECK          3
#define ESP_STUB_CMD_FLASH_MAP_GET              4
#define ESP_STUB_CMD_FLASH_BP_SET               5
#define ESP_STUB_CMD_FLASH_BP_CLEAR             6
#define ESP_STUB_CMD_FLASH_TEST                 7
#define ESP_STUB_CMD_FLASH_WRITE_DEFLATED       8
#define ESP_STUB_CMD_FLASH_CALC_HASH            9
#define ESP_STUB_CMD_FLASH_CLOCK_CONFIGURE      10
#define ESP_STUB_CMD_FLASH_MULTI_COMMAND        11 /* not an actual command. Added for the automation */
#define ESP_STUB_CMD_FLASH_IDF_BINARY           12 /* not an actual command. Added for the automation */
#define ESP_STUB_CMD_FLASH_WITH_LOG             13 /* not an actual command. Added for the automation */
#define ESP_STUB_CMD_FLASH_MAX_ID               ESP_STUB_CMD_FLASH_WITH_LOG
#define ESP_STUB_CMD_TEST                       (ESP_STUB_CMD_FLASH_MAX_ID + 2)

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
	uint32_t down_buf_addr;
	uint32_t down_buf_size;
	uint32_t total_size;        /* uncompressed file size */
	uint32_t extra_stack_addr;  /* extra stack for compression */
	uint32_t options;           /* Write options. e.g. encrypted */
};

struct esp_flash_stub_bp_instructions {
	uint8_t size;
	uint8_t buff[3];
};

struct esp_flash_stub_desc {
	uint32_t magic;
	uint32_t stub_version;
	uint32_t idf_key; /* idf will set the key with known value. (5C3A9F5A) */
};

#define ESP_STUB_FLASHER_DESC_MAGIC_NUM       (offsetof(struct esp_flash_stub_desc, magic))
#define ESP_STUB_FLASHER_DESC_MAGIC_VERSION   (offsetof(struct esp_flash_stub_desc, stub_version))
#define ESP_STUB_FLASHER_DESC_IDF_KEY         (offsetof(struct esp_flash_stub_desc, idf_key))
#define ESP_STUB_FLASHER_DESC_SIZE            (sizeof(struct esp_flash_stub_desc))

#define ESP_STUB_FLASHER_MAGIC_NUM             0xFEEDFACE

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
#define ESP_STUB_FLASHER_VERSION               1

#endif	/* OPENOCD_LOADERS_FLASH_ESPRESSIF_STUB_FLASHER_H */
