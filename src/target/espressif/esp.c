/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Espressif chips common target API for OpenOCD                         *
 *   Copyright (C) 2021 Espressif Systems Ltd.                             *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <helper/log.h>
#include <helper/binarybuffer.h>
#include <target/smp.h>
#include <target/target.h>
#include "esp_riscv.h"
#include "esp_xtensa.h"
#include "esp.h"

#define ESP_FLASH_BREAKPOINTS_MAX_NUM  32
#define ESP_ASSIST_DEBUG_INVALID_VALUE 0xFFFFFFFF

struct esp_common *target_to_esp_common(struct target *target)
{
	struct xtensa *xtensa = target->arch_info;
	if (xtensa->common_magic == RISCV_COMMON_MAGIC)
		return &(target_to_esp_riscv(target)->esp);
	else if (xtensa->common_magic == XTENSA_COMMON_MAGIC)
		return &(target_to_esp_xtensa(target)->esp);
	LOG_ERROR("Unknown target arch!");
	return NULL;
}

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
	uint32_t table_size, table_start_id, desc_entry_id, gcov_entry_id;
	uint32_t entries[ESP_DBG_STUB_ENTRY_MAX] = {0};
	uint8_t entry_buff[sizeof(entries)] = {0}; /* to avoid endiannes issues */

	LOG_TARGET_DEBUG(target, "Read debug stubs info %" PRIx32 " / %d", dbg_stubs->base, dbg_stubs->entries_count);

	/* First of, read 2 entries to get magic num and table size */
	int res = target_read_buffer(target, dbg_stubs->base, sizeof(uint32_t) * 2, entry_buff);
	if (res != ERROR_OK) {
		LOG_ERROR("%s: Failed to read first debug stub entry!", target_name(target));
		return res;
	}
	entries[0] = target_buffer_get_u32(target, entry_buff);
	entries[1] = target_buffer_get_u32(target, entry_buff + sizeof(uint32_t));

	if (entries[0] != ESP_DBG_STUB_MAGIC_NUM_VAL) {
		/* idf with the old table entry structure */
		table_size = 2;
		table_start_id = 0;
		desc_entry_id = 0;
		gcov_entry_id = 1;
	} else {
		table_size = entries[1];
		table_start_id = ESP_DBG_STUB_TABLE_START;
		desc_entry_id = ESP_DBG_STUB_TABLE_START;
		gcov_entry_id = ESP_DBG_STUB_ENTRY_FIRST;

		/* discard unsupported entries */
		if (table_size < 2) {
			LOG_ERROR("Invalid stub table entry size (%x)", table_size);
			return ERROR_FAIL;
		}
		if (table_size > ESP_DBG_STUB_ENTRY_MAX)
			table_size = ESP_DBG_STUB_ENTRY_MAX;

		/* now read the remaining entries */
		res = target_read_buffer(target, dbg_stubs->base + 2 * sizeof(uint32_t), sizeof(uint32_t) * table_size - 2,
			entry_buff + sizeof(uint32_t) * 2);
		if (res != ERROR_OK) {
			LOG_TARGET_ERROR(target, "Failed to read debug stubs info!");
			return res;
		}
		for (unsigned int i = 2; i < table_size; ++i)
			entries[i] = target_buffer_get_u32(target, entry_buff + sizeof(uint32_t) * i);

		dbg_stubs->entries[ESP_DBG_STUB_CAPABILITIES] = entries[ESP_DBG_STUB_CAPABILITIES];
	}

	dbg_stubs->entries[ESP_DBG_STUB_DESC] = entries[desc_entry_id];
	dbg_stubs->entries[ESP_DBG_STUB_ENTRY_GCOV] = entries[gcov_entry_id];

	for (enum esp_dbg_stub_id i = ESP_DBG_STUB_DESC; i < ESP_DBG_STUB_ENTRY_MAX; i++) {
		LOG_DEBUG("Check dbg stub %d - %x", i, dbg_stubs->entries[i]);
		if (dbg_stubs->entries[i]) {
			LOG_DEBUG("New dbg stub %d at %x", dbg_stubs->entries_count, dbg_stubs->entries[i]);
			dbg_stubs->entries_count++;
		}
	}
	if (dbg_stubs->entries_count < table_size - table_start_id)
		LOG_WARNING("Not full dbg stub table %d of %d", dbg_stubs->entries_count, table_size - table_start_id);

	return ERROR_OK;
}

static int esp_common_flash_breakpoints_clear(struct target *target)
{
	struct esp_common *esp = target_to_esp_common(target);

	for (size_t slot = 0; slot < ESP_FLASH_BREAKPOINTS_MAX_NUM; slot++) {
		struct esp_flash_breakpoint *flash_bp = &esp->flash_brps.brps[slot];
		if (flash_bp->oocd_bp != NULL) {
			int ret = esp->flash_brps.ops->breakpoint_remove(target, flash_bp);
			if (ret != ERROR_OK) {
				LOG_TARGET_ERROR(target,
					"Failed to remove SW flash BP @ "
					TARGET_ADDR_FMT " (%d)!",
					flash_bp->oocd_bp->address,
					ret);
				return ret;
			}
		}
	}
	memset(esp->flash_brps.brps, 0, ESP_FLASH_BREAKPOINTS_MAX_NUM * sizeof(struct esp_flash_breakpoint));
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

/* TODO: Prefer keeping OpenOCD default and not change it in the C code.
 * If for any reason a user wants a different behavior, he/she can use a TCL event handler
 * Currently just in use from RISC-V
 */
int esp_common_handle_gdb_detach(struct target *target)
{
	int ret;

	enum target_state old_state = target->state;
	if (target->state != TARGET_HALTED) {
		ret = target_halt(target);
		if (ret != ERROR_OK) {
			LOG_TARGET_ERROR(target, "Failed to halt target to remove flash BPs (%d)!", ret);
			return ret;
		}
		ret = target_wait_state(target, TARGET_HALTED, 3000);
		if (ret != ERROR_OK) {
			LOG_TARGET_ERROR(target, "Failed to wait halted target to remove flash BPs (%d)!", ret);
			return ret;
		}
	}
	ret = esp_common_flash_breakpoints_clear(target);
	if (ret != ERROR_OK)
		return ret;
	if (old_state == TARGET_RUNNING) {
		ret = target_resume(target, 1, 0, 1, 0);
		if (ret != ERROR_OK) {
			LOG_TARGET_ERROR(target, "Failed to resume target after flash BPs removal (%d)!", ret);
			return ret;
		}
	}
	return ERROR_OK;
}

void esp_common_assist_debug_monitor_disable(struct target *target, uint32_t address, uint32_t *value)
{
	LOG_TARGET_DEBUG(target, "addr 0x%08" PRIx32, address);

	int res = target_read_u32(target, address, value);
	if (res != ERROR_OK) {
		LOG_ERROR("Can not read assist_debug register (%d)!", res);
		*value = ESP_ASSIST_DEBUG_INVALID_VALUE;
		return;
	}
	LOG_DEBUG("Saved register value 0x%08" PRIx32, *value);

	res = target_write_u32(target, address, 0);
	if (res != ERROR_OK) {
		LOG_ERROR("Can not write assist_debug register (%d)!", res);
		*value = ESP_ASSIST_DEBUG_INVALID_VALUE;
	}
}

void esp_common_assist_debug_monitor_restore(struct target *target, uint32_t address, uint32_t value)
{
	LOG_TARGET_DEBUG(target, "value 0x%08" PRIx32 " addr 0x%08" PRIx32, value, address);

	/* value was not set by disable function */
	if (value == ESP_ASSIST_DEBUG_INVALID_VALUE)
		return;

	int res = target_write_u32(target, address, value);
	if (res != ERROR_OK)
		LOG_ERROR("Can not restore assist_debug register (%d)!", res);
}

int esp_common_read_pseudo_ex_reason(struct target *target)
{
	struct esp_common *esp = target_to_esp_common(target);
	if (esp && esp->panic_reason.addr) {
		uint8_t str[esp->panic_reason.len + 1];
		memset(str, 0x00, sizeof(str));
		int retval = target_read_memory(target, esp->panic_reason.addr, 1, esp->panic_reason.len, str);
		if (retval == ERROR_OK)
			LOG_TARGET_INFO(target, "Halt cause (%s)", str);
		else
			LOG_TARGET_ERROR(target, "Pseudo exception reason read failed (%d)", retval);
		esp->panic_reason.addr = 0;
		return ERROR_OK;
	}

	return ERROR_FAIL;
}

/* Generic commands for xtensa and riscv */
int esp_common_gdb_detach_command(struct command_invocation *cmd)
{
	if (CMD_ARGC != 0)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct target *target = get_current_target(CMD_CTX);
	if (target->smp) {
		struct target_list *head;
		foreach_smp_target(head, target->smp_targets) {
			int ret = esp_common_handle_gdb_detach(head->target);
			if (ret != ERROR_OK)
				return ret;
		}
		return ERROR_OK;
	}
	return esp_common_handle_gdb_detach(target);
}
