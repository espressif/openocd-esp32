// SPDX-License-Identifier: Apache-2.0 OR MIT
// SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD

#include <stdbool.h>
#include <stdint.h>
#include <assert.h>
#include <string.h>

#include <esp-stub-lib/err.h>
#include <esp-stub-lib/log.h>
#include <esp-stub-lib/flash.h>
#include <esp-stub-lib/mmu.h>
#include <esp-stub-lib/mem_utils.h>
#include <esp-stub-lib/bit_utils.h>

#include "esp_stub.h"

#include "flash_mapping.h"
#include "mcuboot_format.h"
#include "app_image_format.h"
#include "partition_table_format.h"

/*

Supported on-board flash formats

Note: The Espressif App Image format is used both in bootloader
	  and applications.

Note: The offsets below may have different values.


The Original Espressif Format:

. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
. 2nd stage IDF Bootloader   . IDF Partition Table   . IDF App Partition N        .     .
.                            .                       .                            .     .
. offset: 0x0                . offset: 0x8000        . offset: 0x10000            .     .
+--------+------+------+-----+-----+-----------+-----+--------+------+------+-----+-----+
| esp    | esp  | esp  | ... | ... | Partition | ... | esp    | esp  | esp  | ... | ... |
| image  | seg0 | seg1 |     |     | Entity N  |     | image  | seg0 | seg1 |     |     |
| header |      |      |     |     |           |     | header |      |      |     |     |
|        |      |      |     |     | Type: app |     |        |      |      |     |     |
+--------+------+------+-----+-----+-----------+-----+--------+------+------+-----+-----+


MCUboot with IDF Bootloader:

. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
. 2nd stage IDF Bootloader   . IDF Partition Table   . MCUboot Image Slot      .     .
.                            .                       .                         .     .
. offset: 0x0                . offset: 0x8000        . offset: 0x10000         .     .
+--------+------+------+-----+-----+-----------+-----+---------+---------+-----+-----+
| esp    | esp  | esp  | ... | ... | Partition | ... | mcuboot | mcuboot | ... | ... |
| image  | seg0 | seg1 |     |     | Entity N  |     | header  | program |     |     |
| header |      |      |     |     |           |     |         | header  |     |     |
|        |      |      |     |     | Type: app |     |         |         |     |     |
+--------+------+------+-----+-----+-----------+-----+---------+---------+-----+-----+


MCUboot with its bootloader:

. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
. MCUboot 2nd stage Bootldr  .     . MCUboot Image Slot            .     .
. (loadable by chip ROM)     .     .                               .     .
.                            .     . offset: 0x20000               .     .
. offset: 0x0                .     .                               .     .
+--------+------+------+-----+-----+---------+---------+-----+-----+-----+
| esp    | esp  | esp  | ... | ... | mcuboot | mcuboot | ... | ... | ... |
| image  | seg0 | seg1 |     |     | header  | program |     |     |     |
| header |      |      |     |     |         | header  |     |     |     |
+--------+------+------+-----+-----+---------+---------+-----+-----+-----+


Espressif Simple Boot:

. . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
. Simple Boot                                             .
. (loadable by chip ROM)                                  .
.                                                         .
. offset: 0x0                                             .
+--------+------+------+-----+----------++-------+--------+
| esp    | esp  | esp  | ... | 8-bit    || esp   | esp    |
| image  | seg0 | seg1 |     | checksum || segN  | segN+1 |
| header |      |      |     | (16 B    ||       |        |
|        | RAM  | RAM  |     | aligned) || Flash | Flash  |
+--------+------+------+-----+----------++-------+--------+

*/

#define ESP_CHECKSUM_ALIGN 16

#define ESP_MAP_DROM 0
#define ESP_MAP_IROM 1
#define ESP_MAP_MAX_NUM ESP_STUB_FLASH_MAPPINGS_MAX_NUM

// Combined: mcuboot_header and esp_image_header
union image_probe_header {
	struct {
		uint8_t esp_magic; // ESP_IMAGE_HEADER_MAGIC
		uint8_t segment_count;
	};
	uint32_t mcuboot_magic; // MCUBOOT_HEADER_MAGIC
};

static bool mem_is_unknown(uint32_t addr)
{
	return !(addr == 0 // a special padding segment for some ROM segments,
		// https://github.com/espressif/esptool/blob/70afbaabee112e252136fef677ce09a0c141f29d/esptool/bin_image.py#L879
		|| stub_lib_mem_is_irom(addr) || stub_lib_mem_is_drom(addr)
		|| stub_lib_mem_is_iram(addr) || stub_lib_mem_is_dram(addr)
		|| stub_lib_mem_is_rtc_iram_fast(addr) || stub_lib_mem_is_rtc_dram_fast(addr)
		|| stub_lib_mem_is_rtc_slow(addr) || stub_lib_mem_is_tcm(addr)
		);
}

static int get_mcuboot_mappings(uint32_t mcuboot_hdr_off, struct esp_stub_flash_map *flash_map)
{
	struct mcuboot_esp_program_header prg_hdr;

	uint32_t prg_hdr_off = mcuboot_hdr_off + MCUBOOT_HEADER_SIZE;
	if (prg_hdr_off + sizeof(prg_hdr) > flash_map->flash_size) {
		flash_map->retcode = ESP_STUB_MAP_RES_INVALID_MCUBOOT;
		return ESP_STUB_OK;
	}
	int rc = stub_lib_mmu_read_flash(prg_hdr_off, &prg_hdr, sizeof(prg_hdr));
	if (rc != STUB_LIB_OK) {
		STUB_LOGE("MCUboot program - failed read 0x%x\n", prg_hdr_off);
		flash_map->retcode = ESP_STUB_MAP_RES_READ_MCUBOOT;
		return rc;
	}

	if (prg_hdr.header_magic != MCUBOOT_PROGRAM_HEADER_MAGIC) {
		STUB_LOGE("MCUboot program - unknown magic 0x%x\n", prg_hdr.header_magic);
		flash_map->retcode = ESP_STUB_MAP_RES_UNKNOWN_MCUBOOT_MAGIC;
		return ESP_STUB_OK;
	}

	if (!stub_lib_mem_is_drom(prg_hdr.drom_map_addr) || !stub_lib_mem_is_irom(prg_hdr.irom_map_addr)) {
		STUB_LOGE("MCUboot program - invalid DROM/IROM addr (0x%x)/(0x%x)\n",
			prg_hdr.drom_map_addr, prg_hdr.irom_map_addr);
		flash_map->retcode = ESP_STUB_MAP_RES_MCUBOOT_VADDR;
		return ESP_STUB_OK;
	}

	flash_map->map.maps[ESP_MAP_DROM].phy_addr = prg_hdr.drom_flash_offset + mcuboot_hdr_off;
	flash_map->map.maps[ESP_MAP_DROM].load_addr = prg_hdr.drom_map_addr;
	flash_map->map.maps[ESP_MAP_DROM].size = prg_hdr.drom_size;
	STUB_LOGI("Mapped MCUboot DROM segment: %d bytes @ 0x%x -> 0x%x\n",
		prg_hdr.drom_size,
		prg_hdr.drom_flash_offset + mcuboot_hdr_off,
		prg_hdr.drom_map_addr);

	flash_map->map.maps[ESP_MAP_IROM].phy_addr = prg_hdr.irom_flash_offset + mcuboot_hdr_off;
	flash_map->map.maps[ESP_MAP_IROM].load_addr = prg_hdr.irom_map_addr;
	flash_map->map.maps[ESP_MAP_IROM].size = prg_hdr.irom_size;
	STUB_LOGI("Mapped MCUboot IROM segment: %d bytes @ 0x%x -> 0x%x\n",
		prg_hdr.irom_size,
		prg_hdr.irom_flash_offset + mcuboot_hdr_off,
		prg_hdr.irom_map_addr);

	flash_map->map.maps_num = ESP_MAP_MAX_NUM;

	return ESP_STUB_OK;
}

static int get_esp_image_mappings(uint32_t esp_image_hdr_off, uint8_t seg_count, struct esp_stub_flash_map *flash_map)
{
	uint32_t seg_off = esp_image_hdr_off + ESP_IMAGE_HEADER_SIZE;
	memset(&flash_map->map, 0, sizeof(flash_map->map));

	// Walk through the possible segments:
	//
	//     [0, ..., seg_count, ..., ESP_IMAGE_MAX_SEGMENTS)
	//
	// [0; seg_count) - used by both Espressif App Image Boot and Simple Boot
	// [seg_count; ESP_IMAGE_MAX_SEGMENTS) - used by Simple Boot only (non-RAM segments)

	for (int i = 0; i < ESP_IMAGE_MAX_SEGMENTS; i++) {
		struct esp_image_segment_header seg_hdr;

		// Espressif Simple Boot way,
		// https://github.com/espressif/esptool/blob/70afbaabee112e252136fef677ce09a0c141f29d/esptool/bin_image.py#L864
		// Add an aligned checksum size after the latest RAM segment
		if (i == seg_count)
			seg_off += ESP_CHECKSUM_ALIGN - (seg_off % ESP_CHECKSUM_ALIGN);

		if (seg_off + sizeof(seg_hdr) > flash_map->flash_size) {
			flash_map->retcode = ESP_STUB_MAP_RES_INVALID_SEGMENT;
			return ESP_STUB_OK;
		}
		int rc = stub_lib_mmu_read_flash(seg_off, &seg_hdr, sizeof(seg_hdr));
		if (rc != ESP_STUB_OK) {
			STUB_LOGE("App segment - failed read 0x%x\n", seg_off);
			flash_map->retcode = ESP_STUB_MAP_RES_READ_SEGMENT;
			return rc;
		}

		STUB_LOGI("App segment %d: %d bytes @ 0x%x\n",
			i,
			seg_hdr.data_len,
			seg_hdr.load_addr);

		// Stop searching on invalid segment address
		if (mem_is_unknown(seg_hdr.load_addr)) {
			STUB_LOGI("App segment %d - unknown vaddr 0x%x. Stopping search\n", i, seg_hdr.load_addr);
			if (flash_map->map.maps_num == 1 && flash_map->map.maps[0].size == 0)
				// DROM is always in maps[0], but has zero size if no segment found.
				// So it means only IROM was found. For IDF, we will almost never hit this path.
				// For Other RTOS builds, better to handle this corner case.
				// In this case, report to caller that both DROM(zeroed) and IROM are defined
				flash_map->map.maps_num = 2;
			break;
		}

		// Check for mapped flash segments:
		if (stub_lib_mem_is_irom(seg_hdr.load_addr) || stub_lib_mem_is_drom(seg_hdr.load_addr)) {
			/*
			 * Make sure DROM mapping will be at the first index. (OpenOCD expects DROM mapping at index 0)
			 * Note: New targets have identical IROM/DROM address. Thats why additional check is needed.
			 */
			int index = stub_lib_mem_is_drom(seg_hdr.load_addr) && !flash_map->map.maps[ESP_MAP_DROM].load_addr
				? ESP_MAP_DROM : ESP_MAP_IROM;
			flash_map->map.maps[index].phy_addr = seg_off + ESP_IMAGE_SEGMENT_HEADER_SIZE;
			flash_map->map.maps[index].load_addr = seg_hdr.load_addr;
			flash_map->map.maps[index].size = seg_hdr.data_len;

			STUB_LOGI("Mapped %s segment: %d bytes @ 0x%x -> 0x%x\n",
				index == ESP_MAP_DROM ? "DROM" : "IROM",
				seg_hdr.data_len,
				seg_off + ESP_IMAGE_SEGMENT_HEADER_SIZE,
				seg_hdr.load_addr);

			flash_map->map.maps_num++;
		}

		if (flash_map->map.maps_num >= ESP_MAP_MAX_NUM)
			// No need to read more. Both DROM and IROM mapping were found
			break;

		seg_off += ESP_IMAGE_SEGMENT_HEADER_SIZE + seg_hdr.data_len;
	}

	return ESP_STUB_OK;
}

/**
 * @brief Search for an application partition in on-board flash.
 *
 * Scans the entire flash starting from ESP_PARTITION_TABLE_OFFSET
 * to find the partition table, then looks up an application partition.
 *
 * @return Result:
 * - ESP_STUB_OK if partition was found
 * - ESP_STUB_ERR_PARTITION_NOT_FOUND if no partition was found in flash
 * - STUB_LIB_ERR_FLASH_READ_UNALIGNED
 * - STUB_LIB_ERR_FLASH_READ
 */
static int find_esp_partition(uint32_t flash_size, uint32_t *app_offset, int32_t *retcode)
{
	uint32_t pt_entry_off = ESP_PARTITION_TABLE_OFFSET;
	while (true) {
		struct esp_partition_info pt_entry;
		uint32_t __attribute__((unused)) pt_cnt = 0;

		if (pt_entry_off + sizeof(pt_entry) > flash_size) {
			*retcode = ESP_STUB_MAP_RES_INVALID_PARTITION;
			return ESP_STUB_ERR_PARTITION_NOT_FOUND;
		}
		int rc = stub_lib_mmu_read_flash(pt_entry_off, &pt_entry, sizeof(pt_entry));
		if (rc != STUB_LIB_OK) {
			STUB_LOGE("Partition %d - failed read 0x%x\n", pt_cnt, pt_entry_off);
			*retcode = ESP_STUB_MAP_RES_READ_PARTITION;
			return rc;
		}

		if (pt_entry.magic != ESP_PARTITION_MAGIC) {
			STUB_LOGE("Partition %d - unknown magic 0x%x\n", pt_cnt, pt_entry.magic);
			*retcode = ESP_STUB_MAP_RES_UNKNOWN_PARTITION_MAGIC;
			return ESP_STUB_ERR_PARTITION_NOT_FOUND;
		}

		if (pt_entry.pos.offset > flash_size
			|| pt_entry.pos.offset + pt_entry.pos.size > flash_size) {
			STUB_LOGE("Partition %d - offset 0x%x size 0x%x exceeds flash size 0x%x\n",
				pt_cnt,
				pt_entry.pos.offset,
				pt_entry.pos.size,
				flash_size);
			*retcode = ESP_STUB_MAP_RES_INVALID_PARTITION;
			return ESP_STUB_ERR_PARTITION_NOT_FOUND;
		}
		STUB_LOGI("Partition %d, m 0x%x, t 0x%x, st 0x%x, l '%s'\n",
			pt_cnt,
			pt_entry.magic,
			pt_entry.type,
			pt_entry.subtype,
			pt_entry.label);

		if (pt_entry.type == ESP_PARTITION_TYPE_APP) {
			STUB_LOGI("App partition: '%s' %d KB @ 0x%x\n",
				pt_entry.label,
				BYTES_TO_KIB(pt_entry.pos.size),
				pt_entry.pos.offset);

			*app_offset = pt_entry.pos.offset;
			return ESP_STUB_OK;
		}

		pt_entry_off += sizeof(pt_entry);
		pt_cnt++;
	}
}

int stub_flash_mapping(uint32_t app_offset_hint, struct esp_stub_flash_map *flash_map)
{
	stub_lib_flash_config_t flash_info;

	stub_lib_flash_get_config(&flash_info);
	if (flash_info.flash_size == 0) {
		flash_map->retcode = ESP_STUB_MAP_RES_INVALID_FLASH_SIZE;
		return ESP_STUB_OK;
	}

	flash_map->flash_size = flash_info.flash_size;
	flash_map->retcode = ESP_STUB_MAP_RES_OK;
	flash_map->map.maps_num = 0;

	uint32_t app_off = app_offset_hint;
	if (app_offset_hint == UINT32_MAX) {
		int ret = find_esp_partition(flash_info.flash_size, &app_off, &flash_map->retcode);
		if (ret == ESP_STUB_ERR_PARTITION_NOT_FOUND)
			return ESP_STUB_OK;
		if (ret != ESP_STUB_OK)
			return ret;
	}

	union image_probe_header probe_hdr;

	if (app_off + sizeof(probe_hdr) > flash_map->flash_size) {
		flash_map->retcode = ESP_STUB_MAP_RES_INVALID_PROBE;
		return ESP_STUB_OK;
	}
	int rc = stub_lib_mmu_read_flash(app_off, &probe_hdr, sizeof(probe_hdr));
	if (rc != ESP_STUB_OK) {
		STUB_LOGE("Probe image - failed read 0x%x\n", app_off);
		flash_map->retcode = ESP_STUB_MAP_RES_READ_PROBE;
		return rc;
	}

	if (probe_hdr.mcuboot_magic == MCUBOOT_HEADER_MAGIC) {
		STUB_LOGI("MCUboot header: addr 0x%x, magic 0x%x\n",
			app_off,
			probe_hdr.mcuboot_magic);
		return get_mcuboot_mappings(app_off, flash_map);
	}

	if (probe_hdr.esp_magic == ESP_IMAGE_HEADER_MAGIC) {
		STUB_LOGI("App image header: addr 0x%x, magic 0x%x, %d segments\n",
			app_off,
			probe_hdr.esp_magic,
			probe_hdr.segment_count);
		return get_esp_image_mappings(app_off, probe_hdr.segment_count, flash_map);
	}

	STUB_LOGE("Unknown probe magic number 0x%x\n", probe_hdr.mcuboot_magic);
	flash_map->retcode = ESP_STUB_MAP_RES_UNKNOWN_PROBE_MAGIC;
	return ESP_STUB_OK;
}
