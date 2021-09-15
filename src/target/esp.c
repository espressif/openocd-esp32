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
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.           *
 ***************************************************************************/

#include "log.h"
#include "target.h"
#include "binarybuffer.h"
#include "esp.h"


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
