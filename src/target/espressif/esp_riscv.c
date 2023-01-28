/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   ESP RISCV common definitions for OpenOCD                              *
 *   Copyright (C) 2021 Espressif Systems Ltd.                             *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <helper/bits.h>
#include <stdbool.h>
#include <stdint.h>
#include "esp_riscv.h"
#include <target/target_type.h>
#include <target/smp.h>
#include <target/semihosting_common.h>

#include "esp_semihosting.h"

/* Argument indexes for ESP_SEMIHOSTING_SYS_BREAKPOINT_SET */
enum {
	ESP_RISCV_SET_BREAKPOINT_ARG_SET,
	ESP_RISCV_SET_BREAKPOINT_ARG_ID,
	ESP_RISCV_SET_BREAKPOINT_ARG_ADDR,	/* missed if `set` is false */
	ESP_RISCV_SET_BREAKPOINT_ARG_MAX
};

/* Argument indexes for ESP_SEMIHOSTING_SYS_WATCHPOINT_SET */
enum {
	ESP_RISCV_SET_WATCHPOINT_ARG_SET,
	ESP_RISCV_SET_WATCHPOINT_ARG_ID,
	ESP_RISCV_SET_WATCHPOINT_ARG_ADDR,	/* missed if `set` is false */
	ESP_RISCV_SET_WATCHPOINT_ARG_SIZE,	/* missed if `set` is false */
	ESP_RISCV_SET_WATCHPOINT_ARG_FLAGS,	/* missed if `set` is false */
	ESP_RISCV_SET_WATCHPOINT_ARG_MAX
};

enum esp_riscv_exception_cause {
	INSTR_ADDR_MISALIGNED = 0x0,
	PMP_INSTRUCTION_ACCESS_FAULT = 0x1,
	ILLEGAL_INSTRUCTION = 0x2,
	HARDWARE_BREAKPOINT = 0x3,
	LOAD_ADDR_MISALIGNED = 0x4,
	PMP_LOAD_ACCESS_FAULT = 0x5,
	STORE_ADDR_MISALIGNED = 0x6,
	PMP_STORE_ACCESS_FAULT = 0x7,
	ECALL_FROM_U_MODE = 0x8,
	ECALL_FROM_S_MODE = 0x9,
	ECALL_FROM_M_MODE = 0xb,
	INSTR_PAGE_FAULT = 0xc,
	LOAD_PAGE_FAULT = 0xd,
	STORE_PAGE_FAULT = 0xf,
};

#define ESP_RISCV_EXCEPTION_CAUSE(reg_val)  ((reg_val) & 0x1F)

#define ESP_SEMIHOSTING_WP_FLG_RD   (1UL << 0)
#define ESP_SEMIHOSTING_WP_FLG_WR   (1UL << 1)

#define ESP_RISCV_DBGSTUBS_UPDATE_DATA_ENTRY(_e_) \
	do { \
		(_e_) = buf_get_u32((uint8_t *)&(_e_), 0, 32); \
		if ((_e_) == 0) { \
			LOG_WARNING("No valid stub data entry found (0x%x)!", (uint32_t)(_e_));	\
		} \
	} while (0)

#define ESP_RISCV_DBGSTUBS_UPDATE_CODE_ENTRY(_e_) \
	do { \
		(_e_) = buf_get_u32((uint8_t *)&(_e_), 0, 32); \
		if ((_e_) == 0) { \
			LOG_WARNING("No valid stub code entry found (0x%x)!", (uint32_t)(_e_));	\
		} \
	} while (0)

extern struct target_type riscv_target;
static int esp_riscv_debug_stubs_info_init(struct target *target,
	target_addr_t ctrl_addr);

static const char *esp_riscv_get_exception_reason(enum esp_riscv_exception_cause exception_code)
{
	switch (ESP_RISCV_EXCEPTION_CAUSE(exception_code)) {
	case INSTR_ADDR_MISALIGNED:
		return "Instruction address misaligned";
	case PMP_INSTRUCTION_ACCESS_FAULT:
		return "PMP Instruction access fault";
	case ILLEGAL_INSTRUCTION:
		return "Illegal Instruction";
	case HARDWARE_BREAKPOINT:
		return "Hardware Breakpoint/Watchpoint or EBREAK";
	case LOAD_ADDR_MISALIGNED:
		return "Load address misaligned";
	case PMP_LOAD_ACCESS_FAULT:
		return "PMP Load access fault";
	case PMP_STORE_ACCESS_FAULT:
		return "PMP Store access fault";
	case ECALL_FROM_U_MODE:
		return "ECALL from U mode";
	case ECALL_FROM_S_MODE:
		return "ECALL from S-mode";
	case ECALL_FROM_M_MODE:
		return "ECALL from M mode";
	case INSTR_PAGE_FAULT:
		return "Instruction page fault";
	case LOAD_PAGE_FAULT:
		return "Load page fault";
	case STORE_PAGE_FAULT:
		return "Store page fault";
	}
	return "Unknown exception cause";
}

static void esp_riscv_print_exception_reason(struct target *target)
{
	if (target->state != TARGET_HALTED)
		return;

	if (target->halt_issued)
		/* halted upon `halt` request. This is not an exception */
		return;

	riscv_reg_t mcause;
	int result = riscv_get_register(target, &mcause, GDB_REGNO_MCAUSE);
	if (result != ERROR_OK) {
		LOG_ERROR("Failed to read mcause register. Unknown exception reason!");
	} else {
		/* Exception ID 0x0 (instruction access misaligned) is not present because CPU always masks the lowest
		 * bit of the address during instruction fetch.
		 * And (mcause(31) is 1 for interrupts and 0 for exceptions). We will print only exception reasons */
		LOG_TARGET_DEBUG(target, "mcause=%" PRIx64, mcause);
		if (mcause & BIT(31) || mcause == 0)
			return;
		LOG_TARGET_INFO(target, "Halt cause (%d) - (%s)", (int)ESP_RISCV_EXCEPTION_CAUSE(mcause),
			esp_riscv_get_exception_reason(mcause));
	}
}

int esp_riscv_alloc_trigger_addr(struct target *target)
{
	struct esp_riscv_common *esp_riscv = target_to_esp_riscv(target);

	esp_riscv->target_bp_addr = calloc(esp_riscv->max_bp_num, sizeof(*esp_riscv->target_bp_addr));
	if (!esp_riscv->target_bp_addr)
		return ERROR_FAIL;

	esp_riscv->target_wp_addr = calloc(esp_riscv->max_wp_num, sizeof(*esp_riscv->target_wp_addr));
	if (!esp_riscv->target_wp_addr)
		return ERROR_FAIL;

	return ERROR_OK;
}

int esp_riscv_semihosting(struct target *target)
{
	int res = ERROR_OK;
	struct esp_riscv_common *esp_riscv = target_to_esp_riscv(target);
	struct semihosting *semihosting = target->semihosting;

	LOG_DEBUG("op:(%x) param: (%" PRIx64 ")", semihosting->op, semihosting->param);

	if (esp_riscv->semi_ops && esp_riscv->semi_ops->prepare)
		esp_riscv->semi_ops->prepare(target);

	switch (semihosting->op) {
	case ESP_SEMIHOSTING_SYS_APPTRACE_INIT:
		res = esp_riscv_apptrace_info_init(target, semihosting->param, NULL);
		if (res != ERROR_OK)
			return res;
		break;
	case ESP_SEMIHOSTING_SYS_DEBUG_STUBS_INIT:
		res = esp_riscv_debug_stubs_info_init(target, semihosting->param);
		if (res != ERROR_OK)
			return res;
		break;
	case ESP_SEMIHOSTING_SYS_BREAKPOINT_SET:
	{
		/* Enough space to hold 3 long words for both riscv32 and riscv64 archs. */
		uint8_t fields[ESP_RISCV_SET_BREAKPOINT_ARG_MAX * sizeof(uint64_t)];
		res = semihosting_read_fields(target,
				ESP_RISCV_SET_BREAKPOINT_ARG_MAX,
				fields);
		if (res != ERROR_OK)
			return res;
		int id = semihosting_get_field(target,
				ESP_RISCV_SET_BREAKPOINT_ARG_ID,
				fields);
		if (id >= esp_riscv->max_bp_num) {
			LOG_ERROR("Unsupported breakpoint ID (%d)!", id);
			return ERROR_FAIL;
		}
		int set = semihosting_get_field(target,
				ESP_RISCV_SET_WATCHPOINT_ARG_SET,
				fields);
		if (set) {
			if (esp_riscv->target_bp_addr[id]) {
				breakpoint_remove(target, esp_riscv->target_bp_addr[id]);
			}
			esp_riscv->target_bp_addr[id] = semihosting_get_field(target,
					ESP_RISCV_SET_BREAKPOINT_ARG_ADDR,
					fields);
			res = breakpoint_add(target,
					esp_riscv->target_bp_addr[id],
					2,
					BKPT_HARD);
			if (res != ERROR_OK)
				return res;
		} else {
			breakpoint_remove(target, esp_riscv->target_bp_addr[id]);
			esp_riscv->target_bp_addr[id] = 0;
		}
		break;
	}
	case ESP_SEMIHOSTING_SYS_WATCHPOINT_SET:
	{
		/* Enough space to hold 5 long words for both riscv32 and riscv64 archs. */
		uint8_t fields[ESP_RISCV_SET_WATCHPOINT_ARG_MAX * sizeof(uint64_t)];
		res = semihosting_read_fields(target,
				ESP_RISCV_SET_WATCHPOINT_ARG_MAX,
				fields);
		if (res != ERROR_OK)
			return res;
		int id = semihosting_get_field(target,
				ESP_RISCV_SET_WATCHPOINT_ARG_ID,
				fields);
		if (id >= esp_riscv->max_wp_num) {
			LOG_ERROR("Unsupported watchpoint ID (%d)!", id);
			return ERROR_FAIL;
		}
		int set = semihosting_get_field(target,
				ESP_RISCV_SET_WATCHPOINT_ARG_SET,
				fields);
		if (set) {
			if (esp_riscv->target_wp_addr[id]) {
				watchpoint_remove(target, esp_riscv->target_wp_addr[id]);
			}
			esp_riscv->target_wp_addr[id] = semihosting_get_field(target,
					ESP_RISCV_SET_WATCHPOINT_ARG_ADDR,
					fields);
			int size = semihosting_get_field(target,
					ESP_RISCV_SET_WATCHPOINT_ARG_SIZE,
					fields);
			int flags = semihosting_get_field(target,
					ESP_RISCV_SET_WATCHPOINT_ARG_FLAGS,
					fields);
			enum watchpoint_rw wp_type;
			switch (flags &
					(ESP_SEMIHOSTING_WP_FLG_RD | ESP_SEMIHOSTING_WP_FLG_WR)) {
			case ESP_SEMIHOSTING_WP_FLG_RD:
				wp_type = WPT_READ;
				break;
			case ESP_SEMIHOSTING_WP_FLG_WR:
				wp_type = WPT_WRITE;
				break;
			case ESP_SEMIHOSTING_WP_FLG_RD | ESP_SEMIHOSTING_WP_FLG_WR:
				wp_type = WPT_ACCESS;
				break;
			default:
				LOG_ERROR("Unsupported watchpoint type (0x%x)!",
						flags);
				return ERROR_FAIL;
			}
			res = watchpoint_add(target,
					esp_riscv->target_wp_addr[id],
					size,
					wp_type,
					0,
					0);
			if (res != ERROR_OK)
				return res;
		} else {
			watchpoint_remove(target, esp_riscv->target_wp_addr[id]);
			esp_riscv->target_wp_addr[id] = 0;
		}
		break;
	}
	default:
		return ERROR_FAIL;
	}

	semihosting->result = res == ERROR_OK ? 0 : -1;
	semihosting->is_resumable = true;

	return res;
}

static int esp_riscv_debug_stubs_info_init(struct target *target,
	target_addr_t vec_addr)
{
	struct esp_riscv_common *esp_riscv = target_to_esp_riscv(target);

	LOG_INFO("%s: Detected debug stubs @ " TARGET_ADDR_FMT, target_name(target), vec_addr);

	memset(&esp_riscv->esp.dbg_stubs, 0, sizeof(esp_riscv->esp.dbg_stubs));

	esp_riscv->esp.dbg_stubs.base = vec_addr;
	int res = esp_dbgstubs_table_read(target, &esp_riscv->esp.dbg_stubs);
	if (res != ERROR_OK)
		return res;
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
		/* For SMP target return OK if SW flash breakpoint is already set using another
		 *core; GDB causes call to esp_algo_flash_breakpoint_add() for every core, since it
		 *treats flash breakpoints as HW ones */
		if (target->smp) {
			struct target_list *curr;
			foreach_smp_target(curr, target->smp_targets) {
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
			/* For SMP target return OK always, because SW flash breakpoint are set only
			 *using one core, but GDB causes call to esp_algo_flash_breakpoint_remove() for
			 *every core, since it treats flash breakpoints as HW ones */
			return ERROR_OK;
		}
	}

	return res;
}

int esp_riscv_on_halt(struct target *target)
{
	esp_riscv_print_exception_reason(target);
	return ERROR_OK;
}

int esp_riscv_start_algorithm(struct target *target,
	int num_mem_params, struct mem_param *mem_params,
	int num_reg_params, struct reg_param *reg_params,
	target_addr_t entry_point, target_addr_t exit_point,
	void *arch_info)
{
	struct esp_riscv_algorithm *algorithm_info = arch_info;
	size_t max_saved_reg = algorithm_info->max_saved_reg;

	if (target->state != TARGET_HALTED) {
		LOG_WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* Save registers */
	for (uint32_t number = GDB_REGNO_ZERO + 1;
		number <= max_saved_reg && number < target->reg_cache->num_regs; number++) {
		struct reg *r = &target->reg_cache->reg_list[number];

		algorithm_info->valid_saved_registers[r->number] = r->exist;
		if (!r->exist)
			continue;

		LOG_DEBUG("save %s", r->name);

		if (r->size > 64) {
			LOG_ERROR("Register %s is %d bits! Max 64-bits are supported.",
				r->name,
				r->size);
			return ERROR_FAIL;
		}

		if (r->type->get(r) != ERROR_OK) {
			LOG_ERROR("get(%s) failed", r->name);
			r->exist = false;
			return ERROR_FAIL;
		}
		algorithm_info->saved_registers[r->number] = buf_get_u64(r->value, 0, r->size);
	}

	/* write mem params */
	for (int i = 0; i < num_mem_params; i++) {
		if (mem_params[i].direction != PARAM_IN) {
			int retval = target_write_buffer(target, mem_params[i].address,
				mem_params[i].size,
				mem_params[i].value);
			if (retval != ERROR_OK)
				return retval;
		}
	}

	for (int i = 0; i < num_reg_params; i++) {
		LOG_DEBUG("set %s", reg_params[i].reg_name);
		struct reg *r = register_get_by_name(target->reg_cache,
			reg_params[i].reg_name,
			false);
		if (!r) {
			LOG_ERROR("Couldn't find register named '%s'", reg_params[i].reg_name);
			return ERROR_FAIL;
		}

		if (r->size != reg_params[i].size) {
			LOG_ERROR("Register %s is %d bits instead of %d bits.",
				reg_params[i].reg_name, r->size, reg_params[i].size);
			return ERROR_FAIL;
		}

		if (r->number > GDB_REGNO_XPR31) {
			LOG_ERROR("Only GPRs can be use as argument registers.");
			return ERROR_FAIL;
		}

		if (reg_params[i].direction == PARAM_OUT || reg_params[i].direction ==
			PARAM_IN_OUT) {
			if (r->type->set(r, reg_params[i].value) != ERROR_OK) {
				LOG_ERROR("set(%s) failed", reg_params[i].reg_name);
				return ERROR_FAIL;
			}
		}
	}

	/* Disable Interrupts before attempting to run the algorithm. */
	int retval = riscv_interrupts_disable(target,
		MSTATUS_MIE | MSTATUS_HIE | MSTATUS_SIE | MSTATUS_UIE,
		NULL);
	if (retval != ERROR_OK)
		return retval;

	/* Run algorithm */
	LOG_DEBUG("resume at 0x%" TARGET_PRIxADDR, entry_point);
	return riscv_resume(target, 0, entry_point, 0, 1, true);
}

/* Algorithm must end with a software breakpoint instruction. */
int esp_riscv_wait_algorithm(struct target *target,
	int num_mem_params, struct mem_param *mem_params,
	int num_reg_params, struct reg_param *reg_params,
	target_addr_t exit_point, int timeout_ms,
	void *arch_info)
{
	RISCV_INFO(info);
	struct esp_riscv_algorithm *algorithm_info = arch_info;
	size_t max_saved_reg = algorithm_info->max_saved_reg;

	int64_t start = timeval_ms();
	while (target->state != TARGET_HALTED) {
		LOG_DEBUG_IO("poll()");
		int64_t now = timeval_ms();
		if (now - start > timeout_ms) {
			LOG_ERROR("Algorithm timed out after %" PRId64 " ms.", now - start);
			riscv_halt(target);
			riscv_openocd_poll(target);
			enum gdb_regno regnums[] = {
				GDB_REGNO_RA, GDB_REGNO_SP, GDB_REGNO_GP, GDB_REGNO_TP,
				GDB_REGNO_T0, GDB_REGNO_T1, GDB_REGNO_T2, GDB_REGNO_FP,
				GDB_REGNO_S1, GDB_REGNO_A0, GDB_REGNO_A1, GDB_REGNO_A2,
				GDB_REGNO_A3, GDB_REGNO_A4, GDB_REGNO_A5, GDB_REGNO_A6,
				GDB_REGNO_A7, GDB_REGNO_S2, GDB_REGNO_S3, GDB_REGNO_S4,
				GDB_REGNO_S5, GDB_REGNO_S6, GDB_REGNO_S7, GDB_REGNO_S8,
				GDB_REGNO_S9, GDB_REGNO_S10, GDB_REGNO_S11, GDB_REGNO_T3,
				GDB_REGNO_T4, GDB_REGNO_T5, GDB_REGNO_T6,
				GDB_REGNO_PC,
				GDB_REGNO_MSTATUS, GDB_REGNO_MEPC, GDB_REGNO_MCAUSE,
			};
			for (unsigned int i = 0; i < ARRAY_SIZE(regnums); i++) {
				enum gdb_regno regno = regnums[i];
				riscv_reg_t reg_value;
				if (riscv_get_register(target, &reg_value, regno) != ERROR_OK)
					break;
				LOG_ERROR("%s = 0x%" PRIx64, gdb_regno_name(regno), reg_value);
			}
			return ERROR_TARGET_TIMEOUT;
		}

		int result = riscv_openocd_poll(target);
		if (result != ERROR_OK)
			return result;
	}

	/* The current hart id might have been changed in poll(). */
	if (riscv_select_current_hart(target) != ERROR_OK)
		return ERROR_FAIL;

	struct reg *reg_pc = register_get_by_name(target->reg_cache, "pc", true);
	if (reg_pc->type->get(reg_pc) != ERROR_OK)
		return ERROR_FAIL;
	uint64_t final_pc = buf_get_u64(reg_pc->value, 0, reg_pc->size);
	if (exit_point && final_pc != exit_point) {
		LOG_ERROR("PC ended up at 0x%" PRIx64 " instead of 0x%"
			TARGET_PRIxADDR, final_pc, exit_point);
		return ERROR_FAIL;
	}

	for (int i = 0; i < num_reg_params; i++) {
		if (reg_params[i].direction == PARAM_IN ||
			reg_params[i].direction == PARAM_IN_OUT) {
			struct reg *r = register_get_by_name(target->reg_cache,
				reg_params[i].reg_name,
				false);
			if (r->type->get(r) != ERROR_OK) {
				LOG_ERROR("get(%s) failed", r->name);
				return ERROR_FAIL;
			}
			buf_cpy(r->value, reg_params[i].value, reg_params[i].size);
		}
	}
	/* Read memory values to mem_params */
	LOG_DEBUG("Read mem params");
	for (int i = 0; i < num_mem_params; i++) {
		LOG_DEBUG("Check mem param @ " TARGET_ADDR_FMT, mem_params[i].address);
		if (mem_params[i].direction != PARAM_OUT) {
			LOG_DEBUG("Read mem param @ " TARGET_ADDR_FMT, mem_params[i].address);
			int retval = target_read_buffer(target, mem_params[i].address,
				mem_params[i].size,
				mem_params[i].value);
			if (retval != ERROR_OK)
				return retval;
		}
	}

	/* Restore registers */
	for (uint32_t number = GDB_REGNO_ZERO + 1;
		number <= max_saved_reg && number < target->reg_cache->num_regs; number++) {
		struct reg *r = &target->reg_cache->reg_list[number];

		if (!algorithm_info->valid_saved_registers[r->number])
			continue;

		LOG_DEBUG("restore %s", r->name);
		uint8_t buf[8];
		buf_set_u64(buf, 0, info->xlen, algorithm_info->saved_registers[r->number]);
		if (r->type->set(r, buf) != ERROR_OK) {
			LOG_ERROR("set(%s) failed", r->name);
			return ERROR_FAIL;
		}
	}
	return ERROR_OK;
}

int esp_riscv_run_algorithm(struct target *target, int num_mem_params,
	struct mem_param *mem_params, int num_reg_params,
	struct reg_param *reg_params, target_addr_t entry_point,
	target_addr_t exit_point, int timeout_ms, void *arch_info)
{
	int retval;

	retval = esp_riscv_start_algorithm(target,
		num_mem_params, mem_params,
		num_reg_params, reg_params,
		entry_point, exit_point,
		arch_info);

	if (retval == ERROR_OK) {
		retval = esp_riscv_wait_algorithm(target,
			num_mem_params, mem_params,
			num_reg_params, reg_params,
			exit_point, timeout_ms,
			arch_info);
	}

	return retval;
}

int esp_riscv_read_memory(struct target *target, target_addr_t address,
	uint32_t size, uint32_t count, uint8_t *buffer)
{
	/* TODO: find out the widest system bus access size. For now we are assuming it is equal to xlen */
	uint32_t sba_access_size = target_data_bits(target) / 8;

	if (size < sba_access_size) {
		LOG_DEBUG("Use %d-bit access: size: %d\tcount:%d\tstart address: 0x%08"
			TARGET_PRIxADDR, sba_access_size * 8, size, count, address);
		target_addr_t al_addr = address & ~(sba_access_size - 1);
		uint32_t al_len = (size * count) + address - al_addr;
		uint32_t al_cnt = (al_len + sba_access_size - 1) & ~(sba_access_size - 1);
		uint8_t al_buf[al_cnt];
		int ret = riscv_target.read_memory(target,
			al_addr,
			sba_access_size,
			al_cnt / sba_access_size,
			al_buf);
		if (ret == ERROR_OK)
			memcpy(buffer, &al_buf[address & (sba_access_size - 1)], size * count);
		return ret;
	}

	return riscv_target.read_memory(target, address, size, count, buffer);
}

int esp_riscv_write_memory(struct target *target, target_addr_t address,
	uint32_t size, uint32_t count, const uint8_t *buffer)
{
	/* TODO: find out the widest system bus access size. For now we are assuming it is equal to xlen */
	uint32_t sba_access_size = target_data_bits(target) / 8;

	/* Emulate using 32-bit SBA access */
	if (size < sba_access_size) {
		LOG_DEBUG("Use %d-bit access: size: %d\tcount:%d\tstart address: 0x%08"
			TARGET_PRIxADDR, sba_access_size * 8, size, count, address);
		target_addr_t al_addr = address & ~(sba_access_size - 1);
		uint32_t al_len = (size * count) + address - al_addr;
		uint32_t al_cnt = (al_len + sba_access_size - 1) & ~(sba_access_size - 1);
		uint8_t al_buf[al_cnt];
		int ret = riscv_target.read_memory(target,
			al_addr,
			sba_access_size,
			al_cnt / sba_access_size,
			al_buf);
		if (ret == ERROR_OK) {
			memcpy(&al_buf[address & (sba_access_size - 1)],
				buffer,
				size * count);
			ret = riscv_target.write_memory(target,
				address,
				sba_access_size,
				al_cnt / sba_access_size,
				al_buf);
		}
		return ret;
	}
	return riscv_target.write_memory(target, address, size, count, buffer);
}

int esp_riscv_poll(struct target *target)
{
	return riscv_target.poll(target);
}

int esp_riscv_halt(struct target *target)
{
	return riscv_target.halt(target);
}

int esp_riscv_resume(struct target *target, int current, target_addr_t address,
	int handle_breakpoints, int debug_execution)
{
	return riscv_target.resume(target, current, address, handle_breakpoints, debug_execution);
}

int esp_riscv_step(
	struct target *target,
	int current,
	target_addr_t address,
	int handle_breakpoints)
{
	return riscv_target.step(target, current, address, handle_breakpoints);
}

int esp_riscv_assert_reset(struct target *target)
{
	return riscv_target.assert_reset(target);
}

int esp_riscv_deassert_reset(struct target *target)
{
	return riscv_target.deassert_reset(target);
}

int esp_riscv_checksum_memory(struct target *target,
	target_addr_t address, uint32_t count,
	uint32_t *checksum)
{
	return riscv_target.checksum_memory(target, address, count, checksum);
}

int esp_riscv_get_gdb_reg_list_noread(struct target *target,
	struct reg **reg_list[], int *reg_list_size,
	enum target_register_class reg_class)
{
	return riscv_target.get_gdb_reg_list_noread(target, reg_list, reg_list_size, reg_class);
}

int esp_riscv_get_gdb_reg_list(struct target *target,
	struct reg **reg_list[], int *reg_list_size,
	enum target_register_class reg_class)
{
	return riscv_target.get_gdb_reg_list(target, reg_list, reg_list_size, reg_class);
}

const char *esp_riscv_get_gdb_arch(struct target *target)
{
	return riscv_target.get_gdb_arch(target);
}

int esp_riscv_arch_state(struct target *target)
{
	return riscv_target.arch_state(target);
}

int esp_riscv_add_watchpoint(struct target *target, struct watchpoint *watchpoint)
{
	return riscv_target.add_watchpoint(target, watchpoint);
}

int esp_riscv_remove_watchpoint(struct target *target,
	struct watchpoint *watchpoint)
{
	return riscv_target.remove_watchpoint(target, watchpoint);
}

int esp_riscv_hit_watchpoint(struct target *target, struct watchpoint **hit_watchpoint)
{
	return riscv_target.hit_watchpoint(target, hit_watchpoint);
}

unsigned int esp_riscv_address_bits(struct target *target)
{
	return riscv_target.address_bits(target);
}

bool esp_riscv_core_is_halted(struct target *target)
{
	uint32_t dmstatus;
	RISCV_INFO(r);
	if (r->dmi_read(target, &dmstatus, DM_DMSTATUS) != ERROR_OK)
		return false;
	return get_field(dmstatus, DM_DMSTATUS_ALLHALTED);
}

int esp_riscv_core_halt(struct target *target)
{
	RISCV_INFO(r);

	/* Issue the halt command, and then wait for the current hart to halt. */
	uint32_t dmcontrol = DM_DMCONTROL_DMACTIVE | DM_DMCONTROL_HALTREQ;
	r->dmi_write(target, DM_DMCONTROL, dmcontrol);
	for (size_t i = 0; i < 256; ++i)
		if (esp_riscv_core_is_halted(target))
			break;

	if (!esp_riscv_core_is_halted(target)) {
		uint32_t dmstatus;
		if (r->dmi_read(target, &dmstatus, DM_DMSTATUS) != ERROR_OK)
			return ERROR_FAIL;
		if (r->dmi_read(target, &dmcontrol, DM_DMCONTROL) != ERROR_OK)
			return ERROR_FAIL;

		LOG_ERROR("unable to halt core");
		LOG_ERROR("  dmcontrol=0x%08x", dmcontrol);
		LOG_ERROR("  dmstatus =0x%08x", dmstatus);
		return ERROR_FAIL;
	}

	dmcontrol = set_field(dmcontrol, DM_DMCONTROL_HALTREQ, 0);
	r->dmi_write(target, DM_DMCONTROL, dmcontrol);
	return ERROR_OK;
}

int esp_riscv_core_resume(struct target *target)
{
	RISCV_INFO(r);

	/* Issue the resume command, and then wait for the current hart to resume. */
	uint32_t dmcontrol = DM_DMCONTROL_DMACTIVE | DM_DMCONTROL_RESUMEREQ;
	r->dmi_write(target, DM_DMCONTROL, dmcontrol);

	dmcontrol = set_field(dmcontrol, DM_DMCONTROL_HASEL, 0);
	dmcontrol = set_field(dmcontrol, DM_DMCONTROL_RESUMEREQ, 0);

	uint32_t dmstatus;
	for (size_t i = 0; i < 256; ++i) {
		usleep(10);
		int res = r->dmi_read(target, &dmstatus, DM_DMSTATUS);
		if (res != ERROR_OK) {
			LOG_ERROR("Failed to read dmstatus!");
			return res;
		}
		if (get_field(dmstatus, DM_DMSTATUS_ALLRESUMEACK) == 0)
			continue;
		res = r->dmi_write(target, DM_DMCONTROL, dmcontrol);
		if (res != ERROR_OK) {
			LOG_ERROR("Failed to write dmcontrol!");
			return res;
		}
		return ERROR_OK;
	}

	r->dmi_write(target, DM_DMCONTROL, dmcontrol);

	LOG_ERROR("unable to resume core");
	if (r->dmi_read(target, &dmstatus, DM_DMSTATUS) != ERROR_OK)
		return ERROR_FAIL;
	LOG_ERROR("  dmstatus =0x%08x", dmstatus);

	return ERROR_FAIL;
}

int esp_riscv_core_ebreaks_enable(struct target *target)
{
	riscv_reg_t dcsr;
	RISCV_INFO(r);
	int result = r->get_register(target, &dcsr, GDB_REGNO_DCSR);
	if (result != ERROR_OK)
		return result;
	LOG_DEBUG("DCSR: %" PRIx64, dcsr);
	dcsr = set_field(dcsr, CSR_DCSR_EBREAKM, 1);
	dcsr = set_field(dcsr, CSR_DCSR_EBREAKS, 1);
	dcsr = set_field(dcsr, CSR_DCSR_EBREAKU, 1);
	return r->set_register(target, GDB_REGNO_DCSR, dcsr);
}

void esp_riscv_deinit_target(struct target *target)
{
	struct esp_riscv_common *esp_riscv = target_to_esp_riscv(target);
	if (esp_riscv->semi_ops->post_reset)
		esp_riscv->semi_ops->post_reset(target);

	free(esp_riscv->target_bp_addr);
	free(esp_riscv->target_wp_addr);

	riscv_target.deinit_target(target);
}

COMMAND_HANDLER(esp_riscv_gdb_detach_command)
{
	if (CMD_ARGC != 0)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct target *target = get_current_target(CMD_CTX);
	if (!target) {
		LOG_ERROR("No target selected");
		return ERROR_FAIL;
	}

	struct esp_riscv_common *esp_riscv = target_to_esp_riscv(target);
	return esp_common_handle_gdb_detach(target, &esp_riscv->esp);
}

COMMAND_HANDLER(esp_riscv_halted_command)
{
	if (CMD_ARGC != 0)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct target *target = get_current_target(CMD_CTX);
	if (!target) {
		LOG_ERROR("No target selected");
		return ERROR_FAIL;
	}

	return esp_riscv_on_halt(target);
}

const struct command_registration esp_riscv_command_handlers[] = {
	{
		.name = "semihost_basedir",
		.handler = esp_semihosting_basedir_command,
		.mode = COMMAND_ANY,
		.help = "Set the base directory for semihosting I/O."
			"DEPRECATED! use arm semihosting_basedir",
		.usage = "dir",
	},
	{
		.name = "gdb_detach_handler",
		.handler = esp_riscv_gdb_detach_command,
		.mode = COMMAND_ANY,
		.help = "Handles gdb-detach events and makes necessary cleanups such as removing flash breakpoints",
		.usage = "",
	},
	{
		.name = "halted_event_handler",
		.handler = esp_riscv_halted_command,
		.mode = COMMAND_ANY,
		.help = "Handles halted event and prints exception reason",
		.usage = "",
	},
	COMMAND_REGISTRATION_DONE
};
