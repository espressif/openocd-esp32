/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   ESP xtensa chips flasher stub definitions                             *
 *   Copyright (C) 2025 Espressif Systems Ltd.                             *
 ***************************************************************************/
#ifndef OPENOCD_LOADERS_FLASH_ESPRESSIF_MCU_BOOT_H
#define OPENOCD_LOADERS_FLASH_ESPRESSIF_MCU_BOOT_H

#include <stdint.h>

#define MCU_BOOT_HEADER_MAGIC 0x96f3b83d
#define MCU_BOOT_HEADER_SIZE  0x20

#define MCU_BOOT_PROGRAM_HEADER_MAGIC 0xace637d3
#define MCU_BOOT_PROGRAM_HEADER_SIZE  0x60 /* sizeof(esp_program_header_t) */

typedef struct esp_program_header {
	uint32_t header_magic;             /* Magic for load header */
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
	/*uint32_t _reserved[4];*/         /* Up to 4 words reserved for the header. Not used in OpenOCD */
} esp_program_header_t;

#endif /* OPENOCD_LOADERS_FLASH_ESPRESSIF_MCU_BOOT_H */
