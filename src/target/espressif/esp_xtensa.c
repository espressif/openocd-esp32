/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Espressif Xtensa target API for OpenOCD                               *
 *   Copyright (C) 2019 Espressif Systems Ltd.                             *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include <stdbool.h>
#include <stdint.h>
#include <target/smp.h>
#include <target/xtensa/xtensa_algorithm.h>
#include <target/espressif/esp_semihosting.h>
#include "esp_xtensa.h"
#include "esp_xtensa_apptrace.h"
#include "esp_xtensa_semihosting.h"
#include <target/register.h>

#define XTENSA_EXCCAUSE(reg_val)         ((reg_val) & 0x3F)

static const char *xtensa_get_exception_reason(struct target *target, enum xtensa_exception_cause exccause_code)
{
	struct xtensa_config *chip_config = (struct xtensa_config *)target_to_xtensa(target)->core_config;

	switch (exccause_code) {
	case ILLEGAL_INSTRUCTION:
		return "Illegal instruction";
	case SYSCALL:
		return "System call";
	case INSTRUCTION_FETCH_ERROR:
		return "Instruction fetch error";
	case LOAD_STORE_ERROR:
		return "Load or store error";
	case LEVEL1_INTERRUPT:
		return chip_config->irq.enabled ? "Level-1 interrupt" : "Unknown exception";
	case ALLOCA:
		return chip_config->windowed ? "Alloca exception" : "Unknown exception";
	case INTEGER_DIVIDE_BY_ZERO:
		return chip_config->int_div_32 ? "Integer divide by zero" : "Unknown exception";
	case PRIVILEGED:
		return chip_config->mmu.enabled ? "Privileged instruction" : "Unknown exception";
	case LOAD_STORE_ALIGNMENT:
		return chip_config->exc.unaligned ? "Load or store alignment" : "Unknown exception";
	case INSTR_PIF_DATA_ERROR:
		return chip_config->proc_intf ? "Instruction PIF data error" : "Unknown exception";
	case LOAD_STORE_PIF_DATA_ERROR:
		return chip_config->proc_intf ? "Load or store PIF data error" : "Unknown exception";
	case INSTR_PIF_ADDR_ERROR:
		return chip_config->proc_intf ? "Instruction PIF address error" : "Unknown exception";
	case LOAD_STORE_PIF_ADDR_ERROR:
		return chip_config->proc_intf ? "Load or store PIF address error" : "Unknown exception";
	case INST_TLB_MISS:
		return chip_config->mmu.enabled ? "Instruction TLB miss" : "Unknown exception";
	case INST_TLB_MULTIHIT:
		return chip_config->mmu.enabled ? "Instruction TLB multi hit" : "Unknown exception";
	case INST_FETCH_PRIVILEGE:
		return chip_config->mmu.enabled ? "Instruction fetch privilege" : "Unknown exception";
	case INST_FETCH_PROHIBITED:
		return (chip_config->mmu.enabled ||
			chip_config->region_protect.enabled) ? "Instruction fetch prohibited" :
		       "Unknown exception";
	case LOAD_STORE_TLB_MISS:
		return chip_config->mmu.enabled ? "Load or store TLB miss" : "Unknown exception";
	case LOAD_STORE_TLB_MULTIHIT:
		return chip_config->mmu.enabled ? "Load or store TLB multi hit" : "Unknown exception";
	case LOAD_STORE_PRIVILEGE:
		return chip_config->mmu.enabled ? "Load or store privilege" : "Unknown exception";
	case LOAD_PROHIBITED:
		return (chip_config->mmu.enabled || chip_config->region_protect.enabled) ? "Load prohibited" :
		       "Unknown exception";
	case STORE_PROHIBITED:
		return (chip_config->mmu.enabled || chip_config->region_protect.enabled) ? "Store prohibited" :
		       "Unknown exception";
	case COPROCESSOR_N_DISABLED_0:
	case COPROCESSOR_N_DISABLED_1:
	case COPROCESSOR_N_DISABLED_2:
	case COPROCESSOR_N_DISABLED_3:
	case COPROCESSOR_N_DISABLED_4:
	case COPROCESSOR_N_DISABLED_5:
	case COPROCESSOR_N_DISABLED_6:
	case COPROCESSOR_N_DISABLED_7:
		return chip_config->coproc ? "Coprocessor disabled" : "Unknown exception";
	}
	return "Unknown exception";
}

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

	if (esp_xtensa->esp.dbg_stubs.base == 0)
		return ERROR_OK;

	LOG_INFO("%s: Restore debug stubs address %x",
		target_name(target),
		esp_xtensa->esp.dbg_stubs.base);
	int res = esp_xtensa_apptrace_status_reg_write(target, esp_xtensa->esp.dbg_stubs.base);
	if (res != ERROR_OK) {
		LOG_ERROR("Failed to write trace status (%d)!", res);
		return res;
	}
	return ERROR_OK;
}

void esp_xtensa_print_exception_reason(struct target *target)
{
	int exccause_val;

	if (target_to_xtensa(target)->core_config->exc.enabled) {
		exccause_val = XTENSA_EXCCAUSE(xtensa_reg_get(target, XT_REG_IDX_EXCCAUSE));
		LOG_TARGET_INFO(target, "Halt cause (%d) - (%s)", exccause_val,
			xtensa_get_exception_reason(target, exccause_val));
	} else {
		LOG_TARGET_ERROR(target, "Exception option is not enabled!");
	}
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
		esp_xtensa_print_exception_reason(target);
		/* debug stubs can be used in HALTED state only, so it is OK to get info
		 * about them here */
		esp_xtensa_dbgstubs_info_update(target);
		break;
	case TARGET_EVENT_GDB_DETACH:
	{
		struct esp_xtensa_common *esp_xtensa = target_to_esp_xtensa(target);
		ret = esp_common_handle_gdb_detach(target, &esp_xtensa->esp);
		if (ret != ERROR_OK)
			return ret;
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
	const struct esp_flash_breakpoint_ops *flash_brps_ops,
	const struct esp_semihost_ops *semihost_ops)
{
	int ret = xtensa_init_arch_info(target, &esp_xtensa->xtensa, xtensa_cfg, dm_cfg);
	if (ret != ERROR_OK)
		return ret;
	ret = esp_common_init(&esp_xtensa->esp, flash_brps_ops, &xtensa_algo_hw);
	if (ret != ERROR_OK)
		return ret;

	INIT_LIST_HEAD(&esp_xtensa->semihost.dir_map_list);
	esp_xtensa->semihost.ops = (struct esp_semihost_ops *)semihost_ops;
	esp_xtensa->apptrace.hw = &esp_xtensa_apptrace_hw;
	return ERROR_OK;
}

int esp_xtensa_target_init(struct command_context *cmd_ctx, struct target *target)
{
	return xtensa_target_init(cmd_ctx, target);
}

void esp_xtensa_target_deinit(struct target *target)
{
	LOG_DEBUG("start");

	if (target_was_examined(target)) {
		int ret = esp_xtensa_dbgstubs_restore(target);
		if (ret != ERROR_OK)
			return;
	}
	xtensa_target_deinit(target);
	struct esp_xtensa_common *esp_xtensa_common = target_to_esp_xtensa(target);
	if (esp_xtensa_common->semihost.ops->post_reset)
		esp_xtensa_common->semihost.ops->post_reset(target);
	free(esp_xtensa_common->esp.flash_brps.brps);
	free(esp_xtensa_common);	/* same as free(xtensa) */
}

int esp_xtensa_arch_state(struct target *target)
{
	return ERROR_OK;
}

int esp_xtensa_poll(struct target *target)
{
	struct xtensa *xtensa = target_to_xtensa(target);
	struct esp_xtensa_common *esp_xtensa_common = target_to_esp_xtensa(target);

	int ret = xtensa_poll(target);

	if (xtensa_dm_power_status_get(&xtensa->dbg_mod) & PWRSTAT_COREWASRESET) {
		LOG_DEBUG("%s: Clear debug stubs info", target_name(target));
		memset(&esp_xtensa_common->esp.dbg_stubs, 0, sizeof(esp_xtensa_common->esp.dbg_stubs));
		if (esp_xtensa_common->semihost.ops->post_reset)
			esp_xtensa_common->semihost.ops->post_reset(target);
	}
	if (target->state != TARGET_DEBUG_RUNNING)
		esp_xtensa_dbgstubs_addr_check(target);

	return ret;
}

static void esp_xtensa_dbgstubs_addr_check(struct target *target)
{
	struct esp_xtensa_common *esp_xtensa = target_to_esp_xtensa(target);
	uint32_t vec_addr = 0;

	if (esp_xtensa->esp.dbg_stubs.base != 0)
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
		esp_xtensa->esp.dbg_stubs.base = vec_addr;
	}
}

static void esp_xtensa_dbgstubs_info_update(struct target *target)
{
	struct esp_xtensa_common *esp_xtensa = target_to_esp_xtensa(target);

	if (esp_xtensa->esp.dbg_stubs.base == 0 || esp_xtensa->esp.dbg_stubs.entries_count != 0)
		return;

	int res = esp_dbgstubs_table_read(target, &esp_xtensa->esp.dbg_stubs);
	if (res != ERROR_OK)
		return;
	if (esp_xtensa->esp.dbg_stubs.entries_count == 0)
		return;

	/* read debug stubs descriptor */
	ESP_XTENSA_DBGSTUBS_UPDATE_DATA_ENTRY(esp_xtensa->esp.dbg_stubs.entries[ESP_DBG_STUB_DESC]);
	res =
		target_read_buffer(target, esp_xtensa->esp.dbg_stubs.entries[ESP_DBG_STUB_DESC],
		sizeof(struct esp_dbg_stubs_desc),
		(uint8_t *)&esp_xtensa->esp.dbg_stubs.desc);
	if (res != ERROR_OK) {
		LOG_ERROR("Failed to read debug stubs descriptor (%d)!", res);
		return;
	}
	ESP_XTENSA_DBGSTUBS_UPDATE_CODE_ENTRY(esp_xtensa->esp.dbg_stubs.desc.tramp_addr);
	ESP_XTENSA_DBGSTUBS_UPDATE_DATA_ENTRY(esp_xtensa->esp.dbg_stubs.desc.min_stack_addr);
	ESP_XTENSA_DBGSTUBS_UPDATE_CODE_ENTRY(esp_xtensa->esp.dbg_stubs.desc.data_alloc);
	ESP_XTENSA_DBGSTUBS_UPDATE_CODE_ENTRY(esp_xtensa->esp.dbg_stubs.desc.data_free);
}

int esp_xtensa_breakpoint_add(struct target *target, struct breakpoint *breakpoint)
{
	struct esp_xtensa_common *esp_xtensa;

	int res = xtensa_breakpoint_add(target, breakpoint);
	if (res == ERROR_TARGET_RESOURCE_NOT_AVAILABLE && breakpoint->type == BKPT_HARD) {
		/* For SMP target return OK if SW flash breakpoint is already set using another core;
		        GDB causes call to esp_flash_breakpoint_add() for every core, since it treats flash breakpoints as HW ones */
		if (target->smp) {
			struct target_list *curr;
			foreach_smp_target(curr, target->smp_targets) {
				esp_xtensa = target_to_esp_xtensa(curr->target);
				if (esp_common_flash_breakpoint_exists(&esp_xtensa->esp, breakpoint))
					return ERROR_OK;
			}
		}
		esp_xtensa = target_to_esp_xtensa(target);
		return esp_common_flash_breakpoint_add(target, &esp_xtensa->esp, breakpoint);
	}
	return res;
}

int esp_xtensa_breakpoint_remove(struct target *target, struct breakpoint *breakpoint)
{
	struct esp_xtensa_common *esp_xtensa = target_to_esp_xtensa(target);

	int res = xtensa_breakpoint_remove(target, breakpoint);
	if (res == ERROR_TARGET_RESOURCE_NOT_AVAILABLE && breakpoint->type == BKPT_HARD) {
		res = esp_common_flash_breakpoint_remove(target, &esp_xtensa->esp, breakpoint);
		if (res == ERROR_TARGET_RESOURCE_NOT_AVAILABLE && target->smp) {
			/* For SMP target return OK always, because SW flash breakpoint are set only using one core,
			but GDB causes call to esp_flash_breakpoint_remove() for every core, since it treats flash breakpoints as HW ones */
			return ERROR_OK;
		}
	}
	return res;
}

const struct command_registration esp_command_handlers[] = {
	{
		.name = "semihost_basedir",
		.handler = esp_semihosting_basedir_command,
		.mode = COMMAND_ANY,
		.help = "Set the base directory for semihosting I/O."
			"DEPRECATED! use arm semihosting_basedir",
		.usage = "dir",
	},
	COMMAND_REGISTRATION_DONE
};
