/***************************************************************************
 *   Espressif chips common target API for OpenOCD                         *
 *   Copyright (C) 2021 Espressif Systems Ltd.                             *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>. *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <helper/log.h>
#include <helper/binarybuffer.h>
#include "target/target.h"
#include "esp.h"

#define ESP_FLASH_BREAKPOINTS_MAX_NUM  32

int esp_common_init(struct esp_common *esp,
	const struct esp_flash_breakpoint_ops *flash_brps_ops,
	const struct algorithm_hw *algo_hw)
{
	esp->algo_hw = algo_hw;
	esp->flash_brps.ops = flash_brps_ops;
	esp->flash_brps.brps = calloc(ESP_FLASH_BREAKPOINTS_MAX_NUM, sizeof(struct esp_flash_breakpoint));
	if (!esp->flash_brps.brps)
		return ERROR_FAIL;

	return ERROR_OK;
}

int esp_dbgstubs_table_read(struct target *target, struct esp_dbg_stubs *dbg_stubs)
{
	int table_size, table_start_id, desc_entry_id, gcov_entry_id;
	uint32_t entries[ESP_DBG_STUB_ENTRY_MAX];

	LOG_DEBUG("%s: Read debug stubs info %" PRIu32 " / %d", target_name(target),
		dbg_stubs->base, dbg_stubs->entries_count);

	/* first read 2 entries to get magic num and table size */
	int res = target_read_memory(target, dbg_stubs->base, sizeof(uint32_t),
		2,
		(uint8_t *)&entries[0]);
	if (res != ERROR_OK) {
		LOG_ERROR("%s: Failed to read first debug stub entry!", target_name(target));
		return res;
	}
	if (entries[0] != ESP_DBG_STUB_MAGIC_NUM_VAL) {
		/* idf with the old table entry structure */
		table_size = 2;
		table_start_id = desc_entry_id = 0;
		gcov_entry_id = 1;
	} else {
		table_size = entries[1];
		table_start_id = desc_entry_id = ESP_DBG_STUB_TABLE_START;
		gcov_entry_id = ESP_DBG_STUB_ENTRY_FIRST;

		if (table_size < 2) {
			LOG_ERROR("Invalid stub table entry size (%x)", table_size);
			return ERROR_FAIL;
		}
		/* discard unsupported entries */
		if (table_size > ESP_DBG_STUB_ENTRY_MAX)
			table_size = ESP_DBG_STUB_ENTRY_MAX;

		/* now read the remaining entries */
		res = target_read_memory(target,
			dbg_stubs->base + 2 * sizeof(uint32_t),
			sizeof(uint32_t),
			table_size - 2,
			(uint8_t *)&entries[2]);
		if (res != ERROR_OK) {
			LOG_ERROR("%s: Failed to read debug stubs info!", target_name(target));
			return res;
		}
		dbg_stubs->entries[ESP_DBG_STUB_CAPABILITIES] =
			entries[ESP_DBG_STUB_CAPABILITIES];
	}

	dbg_stubs->entries[ESP_DBG_STUB_DESC] = entries[desc_entry_id];
	dbg_stubs->entries[ESP_DBG_STUB_ENTRY_GCOV] = entries[gcov_entry_id];

	for (enum esp_dbg_stub_id i = ESP_DBG_STUB_DESC; i < ESP_DBG_STUB_ENTRY_MAX; i++) {
		LOG_DEBUG("Check dbg stub %d - %x", i, dbg_stubs->entries[i]);
		if (dbg_stubs->entries[i]) {
			dbg_stubs->entries[i] = buf_get_u32(
				(uint8_t *)&dbg_stubs->entries[i],
				0,
				32);
			LOG_DEBUG("New dbg stub %d at %x",
				dbg_stubs->entries_count,
				dbg_stubs->entries[i]);
			dbg_stubs->entries_count++;
		}
	}
	if (dbg_stubs->entries_count <
		(uint32_t)(table_size - table_start_id)) {
		LOG_WARNING("Not full dbg stub table %d of %d", dbg_stubs->entries_count,
			(table_size - table_start_id));
	}

	return ERROR_OK;
}

static int esp_common_flash_breakpoints_clear(struct target *target, struct esp_common *esp)
{
	for (size_t slot = 0; slot < ESP_FLASH_BREAKPOINTS_MAX_NUM;
		slot++) {
		struct esp_flash_breakpoint *flash_bp =
			&esp->flash_brps.brps[slot];
		if (flash_bp->oocd_bp != NULL) {
			int ret = esp->flash_brps.ops->breakpoint_remove(
				target,
				flash_bp);
			if (ret != ERROR_OK) {
				LOG_TARGET_ERROR(
					target,
					"Failed to remove SW flash BP @ "
					TARGET_ADDR_FMT " (%d)!",
					flash_bp->oocd_bp->address,
					ret);
				return ret;
			}
		}
	}
	memset(esp->flash_brps.brps,
		0,
		ESP_FLASH_BREAKPOINTS_MAX_NUM *
		sizeof(struct esp_flash_breakpoint));
	return ERROR_OK;
}

bool esp_common_flash_breakpoint_exists(struct esp_common *esp, struct breakpoint *breakpoint)
{
	for (uint32_t slot = 0; slot < ESP_FLASH_BREAKPOINTS_MAX_NUM; slot++) {
		struct breakpoint *curr = esp->flash_brps.brps[slot].oocd_bp;
		if (curr != NULL && curr->address == breakpoint->address)
			return true;
	}
	return false;
}

int esp_common_flash_breakpoint_add(struct target *target,
	struct esp_common *esp,
	struct breakpoint *breakpoint)
{
	uint32_t slot;

	for (slot = 0; slot < ESP_FLASH_BREAKPOINTS_MAX_NUM; slot++) {
		if (esp->flash_brps.brps[slot].oocd_bp == NULL ||
			esp->flash_brps.brps[slot].oocd_bp == breakpoint)
			break;
	}
	if (slot == ESP_FLASH_BREAKPOINTS_MAX_NUM) {
		LOG_WARNING("%s: max SW flash slot reached, slot=%u", target_name(target), slot);
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}
	return esp->flash_brps.ops->breakpoint_add(target, breakpoint,
		&esp->flash_brps.brps[slot]);
}

int esp_common_flash_breakpoint_remove(struct target *target,
	struct esp_common *esp,
	struct breakpoint *breakpoint)
{
	uint32_t slot;

	for (slot = 0; slot < ESP_FLASH_BREAKPOINTS_MAX_NUM; slot++) {
		if (esp->flash_brps.brps[slot].oocd_bp != NULL &&
			esp->flash_brps.brps[slot].oocd_bp == breakpoint)
			break;
	}
	if (slot == ESP_FLASH_BREAKPOINTS_MAX_NUM) {
		LOG_DEBUG("%s: max SW flash slot reached, slot=%u", target_name(target), slot);
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	return esp->flash_brps.ops->breakpoint_remove(target, &esp->flash_brps.brps[slot]);
}

int esp_common_handle_gdb_detach(struct target *target, struct esp_common *esp_common)
{
	int ret;

	enum target_state old_state = target->state;
	if (target->state != TARGET_HALTED) {
		ret = target_halt(target);
		if (ret != ERROR_OK) {
			LOG_TARGET_ERROR(
				target,
				"Failed to halt target to remove flash BPs (%d)!",
				ret);
			return ret;
		}
		ret = target_wait_state(target, TARGET_HALTED, 3000);
		if (ret != ERROR_OK) {
			LOG_TARGET_ERROR(
				target,
				"Failed to wait halted target to remove flash BPs (%d)!",
				ret);
			return ret;
		}
	}
	ret = esp_common_flash_breakpoints_clear(target, esp_common);
	if (ret != ERROR_OK)
		return ret;
	if (old_state == TARGET_RUNNING) {
		ret = target_resume(target, 1, 0, 1, 0);
		if (ret != ERROR_OK) {
			LOG_TARGET_ERROR(
				target,
				"Failed to resume target after flash BPs removal (%d)!",
				ret);
			return ret;
		}
	}
	return ERROR_OK;
}
