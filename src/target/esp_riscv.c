/***************************************************************************
 *   Espressif RISCV target API for OpenOCD                                *
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

#include <stdbool.h>
#include <stdint.h>
#include "smp.h"
#include "semihosting_common.h"
#include "esp_riscv.h"


#define ESP_RISCV_APPTRACE_SYSNR    0x64
#define ESP_RISCV_DEBUG_STUBS_SYSNR 0x65

#define ESP_RISCV_DBGSTUBS_UPDATE_DATA_ENTRY(_e_) \
	do { \
		(_e_) = buf_get_u32((uint8_t *)&(_e_), 0, 32); \
		if ((_e_) == 0) { \
			LOG_WARNING("No valid stub data entry found (0x%x)!", (uint32_t)(_e_)); \
		} \
	} while (0)

#define ESP_RISCV_DBGSTUBS_UPDATE_CODE_ENTRY(_e_) \
	do { \
		(_e_) = buf_get_u32((uint8_t *)&(_e_), 0, 32); \
		if ((_e_) == 0) { \
			LOG_WARNING("No valid stub code entry found (0x%x)!", (uint32_t)(_e_)); \
		} \
	} while (0)

static int esp_riscv_semihosting_post_result(struct target *target);
static int esp_riscv_debug_stubs_info_init(struct target *target,
	target_addr_t ctrl_addr);


int esp_riscv_semihosting(struct target *target)
{
	int res = ERROR_OK;
	struct esp_riscv_common *esp_riscv = target_to_esp_riscv(target);
	struct semihosting *semihosting = target->semihosting;

	LOG_DEBUG("enter");
	if (esp_riscv->semi_ops && esp_riscv->semi_ops->prepare)
		esp_riscv->semi_ops->prepare(target);

	if (semihosting->op == ESP_RISCV_APPTRACE_SYSNR) {
		res = esp_riscv_apptrace_info_init(target, semihosting->param, NULL);
		if (res != ERROR_OK)
			return res;
	}
	else if (semihosting->op == ESP_RISCV_DEBUG_STUBS_SYSNR) {
		res = esp_riscv_debug_stubs_info_init(target, semihosting->param);
		if (res != ERROR_OK)
			return res;
	} else
		return ERROR_FAIL;
	semihosting->result = res == ERROR_OK ? 0 : -1;
	semihosting->is_resumable = true;
	res = esp_riscv_semihosting_post_result(target);
	if (res != ERROR_OK) {
		LOG_ERROR("Failed to post semihosting result (%d)!", res);
		return res;
	}

	return res;
}

static int esp_riscv_semihosting_post_result(struct target *target)
{
	struct semihosting *semihosting = target->semihosting;
	if (!semihosting) {
		/* If not enabled, silently ignored. */
		return ERROR_OK;
	}

	LOG_DEBUG("0x%" PRIx64, semihosting->result);
	riscv_reg_t new_pc;
	riscv_get_register(target, &new_pc, GDB_REGNO_DPC);
	new_pc += 4;
	riscv_set_register(target, GDB_REGNO_DPC, new_pc);
	riscv_set_register(target, GDB_REGNO_PC, new_pc);

	riscv_set_register(target, GDB_REGNO_A0, semihosting->result);
	return ERROR_OK;
}

static int esp_riscv_debug_stubs_info_init(struct target *target,
	target_addr_t vec_addr)
{
	struct esp_riscv_common *esp_riscv = target_to_esp_riscv(target);

    LOG_INFO("%s: Detected debug stubs @ " TARGET_ADDR_FMT, target_name(target), vec_addr);

    memset(&esp_riscv->esp.dbg_stubs, 0, sizeof(esp_riscv->esp.dbg_stubs));

	esp_riscv->esp.dbg_stubs.base = vec_addr;
	int res = esp_dbgstubs_table_read(target, &esp_riscv->esp.dbg_stubs);
	if (res != ERROR_OK) {
		return res;
	}
	if (esp_riscv->esp.dbg_stubs.entries_count == 0)
		return ERROR_OK;

	/* read debug stubs descriptor */
	ESP_RISCV_DBGSTUBS_UPDATE_DATA_ENTRY(esp_riscv->esp.dbg_stubs.entries[ESP_DBG_STUB_DESC]);
	res =
		target_read_buffer(target, esp_riscv->esp.dbg_stubs.entries[ESP_DBG_STUB_DESC],
		sizeof(struct esp_dbg_stubs_desc),
		(uint8_t *)&esp_riscv->esp.dbg_stubs.desc);
	if (res != ERROR_OK) {
		LOG_ERROR("Failed to read debug stubs descriptor (%d)!", res);
		return res;
	}
	ESP_RISCV_DBGSTUBS_UPDATE_CODE_ENTRY(esp_riscv->esp.dbg_stubs.desc.tramp_addr);
	ESP_RISCV_DBGSTUBS_UPDATE_DATA_ENTRY(esp_riscv->esp.dbg_stubs.desc.min_stack_addr);
	ESP_RISCV_DBGSTUBS_UPDATE_CODE_ENTRY(esp_riscv->esp.dbg_stubs.desc.data_alloc);
	ESP_RISCV_DBGSTUBS_UPDATE_CODE_ENTRY(esp_riscv->esp.dbg_stubs.desc.data_free);

	return ERROR_OK;
}

int esp_riscv_breakpoint_add(struct target *target, struct breakpoint *breakpoint)
{
	struct esp_riscv_common *esp_riscv;

	int res = riscv_add_breakpoint(target, breakpoint);
	if (res == ERROR_TARGET_RESOURCE_NOT_AVAILABLE && breakpoint->type == BKPT_HARD) {
		/* For SMP target return OK if SW flash breakpoint is already set using another core;
			GDB causes call to esp_flash_breakpoint_add() for every core, since it treats flash breakpoints as HW ones */
		if (target->smp) {
			struct target_list *curr;
			foreach_smp_target(curr, target->head) {
				esp_riscv = target_to_esp_riscv(curr->target);
				if (esp_common_flash_breakpoint_exists(&esp_riscv->esp, breakpoint))
					return ERROR_OK;
			}
		}
		esp_riscv = target_to_esp_riscv(target);
		return esp_common_flash_breakpoint_add(target, &esp_riscv->esp, breakpoint);
	}
	return res;
}

int esp_riscv_breakpoint_remove(struct target *target, struct breakpoint *breakpoint)
{
	struct esp_riscv_common *esp_riscv = target_to_esp_riscv(target);

	int res = riscv_remove_breakpoint(target, breakpoint);
	if (res == ERROR_TARGET_RESOURCE_NOT_AVAILABLE && breakpoint->type == BKPT_HARD) {
		res = esp_common_flash_breakpoint_remove(target, &esp_riscv->esp, breakpoint);
		if (res == ERROR_TARGET_RESOURCE_NOT_AVAILABLE && target->smp) {
			/* For SMP target return OK always, because SW flash breakpoint are set only using one core,
			but GDB causes call to esp_flash_breakpoint_remove() for every core, since it treats flash breakpoints as HW ones */
			return ERROR_OK;
		}
	}

	return res;
}

int esp_riscv_handle_target_event(struct target *target, enum target_event event,
	void *priv)
{
	int ret;

	if (target != priv)
		return ERROR_OK;

	LOG_DEBUG("%d", event);

	switch (event) {
		case TARGET_EVENT_GDB_DETACH:
		{
			struct esp_riscv_common *esp_riscv = target_to_esp_riscv(target);
			ret = esp_common_handle_gdb_detach(target, &esp_riscv->esp);
			if (ret != ERROR_OK)
				return ret;
			break;
		}
		default:
			break;
	}
	return ERROR_OK;
}
