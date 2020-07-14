/***************************************************************************
 *   Espressif Xtensa target API for OpenOCD                               *
 *   Copyright (C) 2019 Espressif Systems Ltd.                             *
 *   Author: Alexey Gerenkov <alexey@espressif.com>                        *
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

#include <stdbool.h>
#include <stdint.h>
#include "register.h"
#include "esp_xtensa.h"
#include "esp_xtensa_apptrace.h"
#include "esp_xtensa_semihosting.h"
#include "smp.h"


#define ESP_XTENSA_DBGSTUBS_UPDATE_DATA_ENTRY(_e_) \
	do { \
		(_e_) = buf_get_u32((uint8_t *)&(_e_), 0, 32); \
		if (!xtensa_data_addr_valid(target, (_e_))) { \
			LOG_ERROR("No valid stub data entry found (0x%x)!", (uint32_t)(_e_)); \
			return;	\
		} \
	} while (0)

#define ESP_XTENSA_DBGSTUBS_UPDATE_CODE_ENTRY(_e_) \
	do { \
		(_e_) = buf_get_u32((uint8_t *)&(_e_), 0, 32); \
		if ((_e_) == 0) { \
			LOG_ERROR("No valid stub code entry found (0x%x)!", (uint32_t)(_e_)); \
			return;	\
		} \
	} while (0)

static void esp_xtensa_dbgstubs_info_update(struct target *target);
static void esp_xtensa_dbgstubs_addr_check(struct target *target);


static int esp_xtensa_dbgstubs_restore(struct target *target)
{
	struct esp_xtensa_common *esp_xtensa = target_to_esp_xtensa(target);

	if (esp_xtensa->dbg_stubs.base == 0)
		return ERROR_OK;

	LOG_INFO("%s: Restore debug stubs address %x",
		target_name(target),
		esp_xtensa->dbg_stubs.base);
	int res = esp_xtensa_apptrace_status_reg_write(target, esp_xtensa->dbg_stubs.base);
	if (res != ERROR_OK) {
		LOG_ERROR("Failed to write trace status (%d)!", res);
		return res;
	}
	return ERROR_OK;
}

static int esp_xtensa_flash_breakpoints_clear(struct target *target)
{
	struct esp_xtensa_common *esp_xtensa = target_to_esp_xtensa(target);

	for (size_t slot = 0; slot < ESP_XTENSA_FLASH_BREAKPOINTS_MAX_NUM;
		slot++) {
		struct esp_xtensa_flash_breakpoint *flash_bp =
			&esp_xtensa->flash_brps[slot];
		if (flash_bp->data.oocd_bp != NULL) {
			int ret = esp_xtensa->flash_brps_ops.breakpoint_remove(
				target,
				flash_bp);
			if (ret != ERROR_OK) {
				LOG_ERROR(
					"%s: Failed to remove SW flash BP @ "
					TARGET_ADDR_FMT " (%d)!",
					target_name(target),
					flash_bp->data.oocd_bp->address,
					ret);
				return ret;
			}
		}
	}
	memset(esp_xtensa->flash_brps,
		0,
		ESP_XTENSA_FLASH_BREAKPOINTS_MAX_NUM*
		sizeof(struct esp_xtensa_flash_breakpoint));
	return ERROR_OK;
}

int esp_xtensa_handle_target_event(struct target *target, enum target_event event,
	void *priv)
{
	int ret;

	if (target != priv)
		return ERROR_OK;

	LOG_DEBUG("%d", event);

	ret = xtensa_handle_target_event(target, event, priv);
	if (ret != ERROR_OK)
		return ret;

	switch (event) {
		case TARGET_EVENT_HALTED:
			/* debug stubs can be used in HALTED state only, so it is OK to get info
			 * about them here */
			esp_xtensa_dbgstubs_info_update(target);
			break;
		case TARGET_EVENT_GDB_DETACH:
		{
			enum target_state old_state = target->state;
			if (target->state != TARGET_HALTED) {
				ret = target_halt(target);
				if (ret != ERROR_OK) {
					LOG_ERROR(
						"%s: Failed to halt target to remove flash BPs (%d)!",
						target_name(target),
						ret);
					return ret;
				}
				ret = target_wait_state(target, TARGET_HALTED, 3000);
				if (ret != ERROR_OK) {
					LOG_ERROR(
						"%s: Failed to wait halted target to remove flash BPs (%d)!",
						target_name(target),
						ret);
					return ret;
				}
			}
			ret = esp_xtensa_flash_breakpoints_clear(target);
			if (ret != ERROR_OK)
				return ret;
			if (old_state == TARGET_RUNNING) {
				ret = target_resume(target, 1, 0, 1, 0);
				if (ret != ERROR_OK) {
					LOG_ERROR(
						"%s: Failed to resume target after flash BPs removal (%d)!",
						target_name(target),
						ret);
					return ret;
				}
			}
			break;
		}
		default:
			break;
	}
	return ERROR_OK;
}

int esp_xtensa_init_arch_info(struct target *target,
	struct esp_xtensa_common *esp_xtensa,
	const struct xtensa_config *xtensa_cfg,
	struct xtensa_debug_module_config *dm_cfg,
	const struct esp_xtensa_flash_breakpoint_ops *flash_brps_ops)
{
	int ret = xtensa_init_arch_info(target, &esp_xtensa->xtensa, xtensa_cfg, dm_cfg);
	if (ret != ERROR_OK)
		return ret;
	memcpy(&esp_xtensa->flash_brps_ops, flash_brps_ops, sizeof(esp_xtensa->flash_brps_ops));
	esp_xtensa->flash_brps =
		calloc(ESP_XTENSA_FLASH_BREAKPOINTS_MAX_NUM,
		sizeof(struct esp_xtensa_flash_breakpoint));
	if (esp_xtensa->flash_brps == NULL)
		return ERROR_FAIL;
	return ERROR_OK;
}


int esp_xtensa_target_init(struct command_context *cmd_ctx, struct target *target)
{
	return xtensa_target_init(cmd_ctx, target);
}

void esp_xtensa_target_deinit(struct target *target)
{
	LOG_DEBUG("start");

	int ret = esp_xtensa_dbgstubs_restore(target);
	if (ret != ERROR_OK)
		return;
	xtensa_target_deinit(target);
}

int esp_xtensa_arch_state(struct target *target)
{
	return ERROR_OK;
}

int esp_xtensa_poll(struct target *target)
{
	struct xtensa *xtensa = target_to_xtensa(target);
	struct esp_xtensa_common *esp_xtensa = target_to_esp_xtensa(target);

	int ret = xtensa_poll(target);

	if (xtensa_dm_power_status_get(&xtensa->dbg_mod) & PWRSTAT_COREWASRESET) {
		LOG_DEBUG("%s: Clear debug stubs info", target_name(target));
		memset(&esp_xtensa->dbg_stubs, 0, sizeof(esp_xtensa->dbg_stubs));
	}
	if (target->state != TARGET_DEBUG_RUNNING)
		esp_xtensa_dbgstubs_addr_check(target);

	return ret;
}

static void esp_xtensa_dbgstubs_addr_check(struct target *target)
{
	struct esp_xtensa_common *esp_xtensa = target_to_esp_xtensa(target);
	uint32_t vec_addr = 0;

	if (esp_xtensa->dbg_stubs.base != 0)
		return;

	int res = esp_xtensa_apptrace_status_reg_read(target, &vec_addr);
	if (res != ERROR_OK) {
		LOG_ERROR("Failed to read debug stubs address location (%d)!", res);
		return;
	}
	if (xtensa_data_addr_valid(target, vec_addr)) {
		LOG_INFO("%s: Detected debug stubs @ %x", target_name(target), vec_addr);
		res = esp_xtensa_apptrace_status_reg_write(target, 0);
		if (res != ERROR_OK)
			LOG_ERROR("Failed to clear debug stubs address location (%d)!", res);
		esp_xtensa->dbg_stubs.base = vec_addr;
	}
}

static void esp_xtensa_dbgstubs_info_update(struct target *target)
{
	struct esp_xtensa_common *esp_xtensa = target_to_esp_xtensa(target);

	LOG_DEBUG("%s: Read debug stubs info %d / %d", target_name(target),
		esp_xtensa->dbg_stubs.base, esp_xtensa->dbg_stubs.entries_count);

	if (esp_xtensa->dbg_stubs.base == 0 || esp_xtensa->dbg_stubs.entries_count != 0)
		return;

	int res = target_read_memory(target, esp_xtensa->dbg_stubs.base, sizeof(uint32_t),
		ESP_DBG_STUB_ENTRY_MAX-ESP_DBG_STUB_TABLE_START,
		(uint8_t *)&esp_xtensa->dbg_stubs.entries);
	if (res != ERROR_OK) {
		LOG_ERROR("%s: Failed to read debug stubs info!", target_name(target));
		return;
	}
	for (enum esp_dbg_stub_id i = ESP_DBG_STUB_TABLE_START; i < ESP_DBG_STUB_ENTRY_MAX; i++) {
		LOG_DEBUG("Check dbg stub %d", i);
		if (esp_xtensa->dbg_stubs.entries[i]) {
			esp_xtensa->dbg_stubs.entries[i] = buf_get_u32(
				(uint8_t *)&esp_xtensa->dbg_stubs.entries[i],
				0,
				32);
			LOG_DEBUG("New dbg stub %d at %x",
				esp_xtensa->dbg_stubs.entries_count,
				esp_xtensa->dbg_stubs.entries[i]);
			esp_xtensa->dbg_stubs.entries_count++;
		}
	}
	if (esp_xtensa->dbg_stubs.entries_count <
		(ESP_DBG_STUB_ENTRY_MAX-ESP_DBG_STUB_TABLE_START)) {
		LOG_DEBUG("Not full dbg stub table %d of %d", esp_xtensa->dbg_stubs.entries_count,
			(ESP_DBG_STUB_ENTRY_MAX-ESP_DBG_STUB_TABLE_START));
		esp_xtensa->dbg_stubs.entries_count = 0;
		return;
	}
	/* read debug stubs descriptor */
	ESP_XTENSA_DBGSTUBS_UPDATE_DATA_ENTRY(esp_xtensa->dbg_stubs.entries[ESP_DBG_STUB_DESC]);
	res =
		target_read_memory(target, esp_xtensa->dbg_stubs.entries[ESP_DBG_STUB_DESC],
		sizeof(struct esp_dbg_stubs_desc), 1,
		(uint8_t *)&esp_xtensa->dbg_stubs.desc);
	if (res != ERROR_OK) {
		LOG_ERROR("Failed to read target memory (%d)!", res);
		return;
	}
	ESP_XTENSA_DBGSTUBS_UPDATE_CODE_ENTRY(esp_xtensa->dbg_stubs.desc.tramp_addr);
	ESP_XTENSA_DBGSTUBS_UPDATE_DATA_ENTRY(esp_xtensa->dbg_stubs.desc.min_stack_addr);
	ESP_XTENSA_DBGSTUBS_UPDATE_CODE_ENTRY(esp_xtensa->dbg_stubs.desc.data_alloc);
	ESP_XTENSA_DBGSTUBS_UPDATE_CODE_ENTRY(esp_xtensa->dbg_stubs.desc.data_free);
}

static bool esp_xtensa_flash_breakpoint_exists(struct target *target, struct breakpoint *breakpoint)
{
	struct esp_xtensa_common *esp_xtensa = target_to_esp_xtensa(target);
	for (uint32_t slot = 0; slot < ESP_XTENSA_FLASH_BREAKPOINTS_MAX_NUM; slot++) {
		struct breakpoint *curr = esp_xtensa->flash_brps[slot].data.oocd_bp;
		if (curr != NULL && curr->address == breakpoint->address)
			return true;
	}
	return false;
}

static int esp_xtensa_flash_breakpoint_add(struct target *target, struct breakpoint *breakpoint)
{
	struct esp_xtensa_common *esp_xtensa = target_to_esp_xtensa(target);
	uint32_t slot;

	/* For SMP target return OK if SW flash breakpoint is already set using another core;
	        GDB causes call to esp_xtensa_flash_breakpoint_add() for every core, since it treats flash breakpoints as HW ones */
	if (target->smp) {
		struct target_list *head;
		foreach_smp_target(head, target->head) {
			if (esp_xtensa_flash_breakpoint_exists(head->target, breakpoint))
				return ERROR_OK;
		}
	}

	for (slot = 0; slot < ESP_XTENSA_FLASH_BREAKPOINTS_MAX_NUM; slot++) {
		if (esp_xtensa->flash_brps[slot].data.oocd_bp == NULL ||
			esp_xtensa->flash_brps[slot].data.oocd_bp == breakpoint)
			break;
	}
	if (slot == ESP_XTENSA_FLASH_BREAKPOINTS_MAX_NUM) {
		LOG_WARNING("%s: max SW flash slot reached, slot=%u", target_name(target), slot);
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}
	return esp_xtensa->flash_brps_ops.breakpoint_add(target, breakpoint,
		&esp_xtensa->flash_brps[slot]);
}

int esp_xtensa_breakpoint_add(struct target *target, struct breakpoint *breakpoint)
{
	int res = xtensa_breakpoint_add(target, breakpoint);
	if (res == ERROR_TARGET_RESOURCE_NOT_AVAILABLE && breakpoint->type == BKPT_HARD)
		return esp_xtensa_flash_breakpoint_add(target, breakpoint);
	return res;
}

static int esp_xtensa_flash_breakpoint_remove(struct target *target,
	struct breakpoint *breakpoint)
{
	struct esp_xtensa_common *esp_xtensa = target_to_esp_xtensa(target);
	uint32_t slot;

	for (slot = 0; slot < ESP_XTENSA_FLASH_BREAKPOINTS_MAX_NUM; slot++) {
		if (esp_xtensa->flash_brps[slot].data.oocd_bp != NULL &&
			esp_xtensa->flash_brps[slot].data.oocd_bp == breakpoint)
			break;
	}
	if (slot == ESP_XTENSA_FLASH_BREAKPOINTS_MAX_NUM) {
		LOG_DEBUG("%s: max SW flash slot reached, slot=%u", target_name(target), slot);
		/* For SMP target return OK always, because SW flash breakpoint are set only using one core,
		   but GDB causes call to esp_xtensa_flash_breakpoint_remove() for every core, since it treats flash breakpoints as HW ones */
		return target->smp ? ERROR_OK : ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}
	return esp_xtensa->flash_brps_ops.breakpoint_remove(target, &esp_xtensa->flash_brps[slot]);
}

int esp_xtensa_breakpoint_remove(struct target *target, struct breakpoint *breakpoint)
{
	int res = xtensa_breakpoint_remove(target, breakpoint);
	if (res == ERROR_TARGET_RESOURCE_NOT_AVAILABLE && breakpoint->type == BKPT_HARD)
		return esp_xtensa_flash_breakpoint_remove(target, breakpoint);
	return res;
}

COMMAND_HELPER(esp_xtensa_cmd_semihost_basedir_do, struct esp_xtensa_common *esp_xtensa)
{
	if (CMD_ARGC != 1) {
		command_print(CMD,
			"Current semihosting base dir: %s",
			esp_xtensa->semihost.basedir ? esp_xtensa->semihost.basedir : "");
		return ERROR_OK;
	}

	char *s = strdup(CMD_ARGV[0]);
	if (!s) {
		command_print(CMD, "Failed to allocate memory!");
		return ERROR_FAIL;
	}
	if (esp_xtensa->semihost.basedir)
		free(esp_xtensa->semihost.basedir);
	esp_xtensa->semihost.basedir = s;

	return ERROR_OK;
}

COMMAND_HANDLER(esp_xtensa_cmd_semihost_basedir)
{
	return CALL_COMMAND_HANDLER(esp_xtensa_cmd_semihost_basedir_do,
		target_to_esp_xtensa(get_current_target(CMD_CTX)));
}

const struct command_registration esp_command_handlers[] = {
	{
		.name = "semihost_basedir",
		.handler = esp_xtensa_cmd_semihost_basedir,
		.mode = COMMAND_ANY,
		.help = "Set the base directory for semohosting I/O.",
		.usage = "dir",
	},
	COMMAND_REGISTRATION_DONE
};
