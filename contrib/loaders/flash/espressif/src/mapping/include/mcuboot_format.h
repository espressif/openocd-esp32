/* SPDX-License-Identifier: Apache-2.0 OR MIT */
/* SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD */

#pragma once

/* MCUboot format */

#include <stdint.h>
#include <assert.h>

#define MCUBOOT_HEADER_MAGIC            (0x96f3b83d)
#define MCUBOOT_HEADER_SIZE             (0x20)

#define MCUBOOT_PROGRAM_HEADER_MAGIC    (0xace637d3)
#define MCUBOOT_PROGRAM_HEADER_SIZE     (0x60)

// https://github.com/mcu-tools/mcuboot/blob/v2.2.0/boot/bootutil/include/bootutil/image.h#L164
struct mcuboot_header {
	uint32_t ih_magic;              /* MCUBOOT_HEADER_MAGIC */
	uint32_t ih_load_addr;
	uint16_t ih_hdr_size;           /* Size of image header (bytes). */
	uint16_t ih_protect_tlv_size;   /* Size of protected TLV area (bytes). */
	uint32_t ih_img_size;           /* Does not include header. */
	uint32_t ih_flags;              /* IMAGE_F_[...]. */
	struct image_version {
		uint8_t iv_major;
		uint8_t iv_minor;
		uint16_t iv_revision;
		uint32_t iv_build_num;
	} ih_ver;
	uint32_t _pad1;
} __attribute__((packed));

static_assert(sizeof(struct mcuboot_header) == MCUBOOT_HEADER_SIZE, "struct has incorrest size");

// https://github.com/mcu-tools/mcuboot/blob/v2.2.0/boot/espressif/hal/include/esp_mcuboot_image.h
struct mcuboot_esp_program_header {
	uint32_t header_magic;             /* Magic for load header, MCUBOOT_PROGRAM_HEADER_MAGIC */
	uint32_t entry_addr;               /* Application entry address */
	uint32_t iram_dest_addr;           /* Destination address(VMA) for IRAM region */
	uint32_t iram_flash_offset;        /* Flash offset(LMA) for start of IRAM region */
	uint32_t iram_size;                /* Size of IRAM region */
	uint32_t dram_dest_addr;           /* Destination address(VMA) for DRAM region */
	uint32_t dram_flash_offset;        /* Flash offset(LMA) for start of DRAM region */
	uint32_t dram_size;                /* Size of DRAM region */
	uint32_t lp_rtc_iram_dest_addr;    /* Destination address (VMA) for LP_IRAM region */
	uint32_t lp_rtc_iram_flash_offset; /* Flash offset (LMA) for LP_IRAM region */
	uint32_t lp_rtc_iram_size;         /* Size of LP_IRAM region */
	uint32_t lp_rtc_dram_dest_addr;    /* Destination address (VMA) for LP_DRAM region */
	uint32_t lp_rtc_dram_flash_offset; /* Flash offset (LMA) for LP_DRAM region */
	uint32_t lp_rtc_dram_size;         /* Size of LP_DRAM region */
	uint32_t irom_map_addr;            /* Mapped address (VMA) for IROM region */
	uint32_t irom_flash_offset;        /* Flash offset (LMA) for IROM region */
	uint32_t irom_size;                /* Size of IROM region */
	uint32_t drom_map_addr;            /* Mapped address (VMA) for DROM region */
	uint32_t drom_flash_offset;        /* Flash offset (LMA) for DROM region */
	uint32_t drom_size;                /* Size of DROM region */
	uint32_t _reserved[4];             /* Up to 24 words reserved for the header */
} __attribute__((packed));

static_assert(sizeof(struct mcuboot_esp_program_header) == MCUBOOT_PROGRAM_HEADER_SIZE, "struct has incorrest size");
