/***************************************************************************
 *   Multi-core Xtensa target for OpenOCD                                  *
 *   Copyright (C) 2017-2019 Espressif Systems Ltd.                        *
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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "target.h"
#include "target_type.h"
#include "xtensa_mcore.h"


int xtensa_mcore_assert_reset(struct target *target)
{
	struct xtensa_mcore_common *xtensa_mcore = target_to_xtensa_mcore(target);

	LOG_DEBUG("%s: begin", target_name(target));

	target->state = TARGET_RESET;
	for (int i = 0; i < xtensa_mcore->configured_cores_num; i++) {
		struct target *sub_target = &xtensa_mcore->cores_targets[i];
		sub_target->reset_halt = target->reset_halt;
		int res = sub_target->type->assert_reset(sub_target);
		if (res != ERROR_OK)
			return res;
	}

	return ERROR_OK;
}

int xtensa_mcore_deassert_reset(struct target *target)
{
	struct xtensa_mcore_common *xtensa_mcore = target_to_xtensa_mcore(target);

	int res = xtensa_mcore_smpbreak_set(target);
	if (res != ERROR_OK)
		return res;

	for (int i = 0; i < xtensa_mcore->configured_cores_num; i++) {
		struct target *sub_target = &xtensa_mcore->cores_targets[i];
		res = sub_target->type->deassert_reset(sub_target);
		if (res != ERROR_OK)
			return res;
	}
	target->state = TARGET_RUNNING;
	return ERROR_OK;
}

int xtensa_mcore_halt(struct target *target)
{
	struct xtensa_mcore_common *xtensa_mcore = target_to_xtensa_mcore(target);
	int res;

	LOG_DEBUG("%s: halt request", target_name(target));
	if (target->state == TARGET_HALTED) {
		LOG_DEBUG("%s: target was already halted", target_name(target));
		return ERROR_OK;
	}
	/* when SMP breakINOUT is enabled we can halt just one core, the second will be halted via
	 * BreakInOut circuit */
	if ((xtensa_mcore->smp_break & (OCDDCR_BREAKINEN|OCDDCR_BREAKOUTEN)) ==
		(OCDDCR_BREAKINEN|OCDDCR_BREAKOUTEN)) {
		struct target *sub_target = &xtensa_mcore->cores_targets[xtensa_mcore->active_core];
		res = sub_target->type->halt(sub_target);
		if (res != ERROR_OK) {
			LOG_ERROR("%s.%s: Failed to halt!", target_name(target),
				target_name(sub_target));
			return res;
		}
	} else
		for (size_t i = 0; i < xtensa_mcore->configured_cores_num; i++) {
			struct target *sub_target = &xtensa_mcore->cores_targets[i];
			res = sub_target->type->halt(sub_target);
			if (res != ERROR_OK) {
				LOG_ERROR("%s.%s: Failed to halt!",
					target_name(target),
					target_name(sub_target));
				return res;
			}
		}
	return ERROR_OK;
}

int xtensa_mcore_resume(struct target *target,
	int current,
	target_addr_t address,
	int handle_breakpoints,
	int debug_execution)
{
	struct xtensa_mcore_common *xtensa_mcore = target_to_xtensa_mcore(target);
	struct target *sub_target;
	int res;

	LOG_DEBUG("%s: resume request", target_name(target));
	if (target->state != TARGET_HALTED) {
		LOG_WARNING("%s: target not halted", target_name(target));
		return ERROR_TARGET_NOT_HALTED;
	}
	/* we break resume process into two phases in order to resume both cores as simultaneously
	 * as we can */
	/* at the first stage we write all the necessary data to the cores (HW
	 * breakpoints/watchpoints, dirty registers etc.) */
	/* at the second stage we just execute RFDO instruction on each core to resume them
	 * TODO: think about using DebugStall feature for fast and synchronous stop/resume,
	 * it can also be used for future heterogeneous systems: Xtensa+RiscV */
	sub_target = &xtensa_mcore->cores_targets[xtensa_mcore->active_core];
	res = xtensa_mcore->cores_ops[xtensa_mcore->active_core]->core_prepare_resume(sub_target,
		current,
		address,
		handle_breakpoints,
		debug_execution);
	if (res != ERROR_OK) {
		LOG_ERROR("%s.%s: Failed to prepare for resume on active core!",
			target_name(target),
			target_name(sub_target));
		return res;
	}
	for (size_t i = 0; i < xtensa_mcore->configured_cores_num; i++) {
		sub_target = &xtensa_mcore->cores_targets[i];
		if (i == xtensa_mcore->active_core)
			continue;	/*  everything is already done for the active core */
		/* This operation required to clear state of non-active cores, because
		 * core_prepare_resume() on active can step over watchpoint and */
		/* activate SMP break-in signals on other cores
		 * TODO: check that SMP break is really configured */
		xtensa_core_status_clear(sub_target, OCDDSR_DEBUGPENDBREAK|OCDDSR_DEBUGINTBREAK);
		res = xtensa_mcore->cores_ops[i]->core_prepare_resume(sub_target,
			i == xtensa_mcore->active_core ? current : 1,
			address,
			handle_breakpoints,
			debug_execution);
		if (res != ERROR_OK) {
			LOG_ERROR("%s.%s: Failed to prepare for resume!",
				target_name(target),
				target_name(sub_target));
			return res;
		}
	}
	for (size_t i = 0; i < xtensa_mcore->configured_cores_num; i++) {
		sub_target = &xtensa_mcore->cores_targets[i];
		res = xtensa_mcore->cores_ops[i]->core_do_resume(sub_target);
		if (res != ERROR_OK) {
			LOG_ERROR("%s.%s: Failed to resume!", target_name(target),
				target_name(sub_target));
			return res;
		}
		sub_target->debug_reason = DBG_REASON_NOTHALTED;
		if (!debug_execution)
			sub_target->state = TARGET_RUNNING;
		else
			sub_target->state = TARGET_DEBUG_RUNNING;
	}

	target->debug_reason = DBG_REASON_NOTHALTED;
	if (!debug_execution)
		target->state = TARGET_RUNNING;
	else
		target->state = TARGET_DEBUG_RUNNING;
	int res1 = target_call_event_callbacks(target, TARGET_EVENT_RESUMED);
	if (res1 != ERROR_OK)
		res = res1;
	return res;
}

int xtensa_mcore_step(struct target *target,
	int current,
	target_addr_t address,
	int handle_breakpoints)
{
	struct xtensa_mcore_common *xtensa_mcore = target_to_xtensa_mcore(target);
	struct target *sub_target = &xtensa_mcore->cores_targets[xtensa_mcore->active_core];

	int retval = xtensa_mcore->cores_ops[xtensa_mcore->active_core]->core_do_step(sub_target,
		current,
		address,
		handle_breakpoints);
	if (retval != ERROR_OK) {
		LOG_ERROR("%s.%s: Failed to step!", target_name(target), target_name(sub_target));
		return retval;
	}
	/* This operation required to clear state of non-active cores */
	for (size_t i = 0; i < xtensa_mcore->configured_cores_num; i++) {
		if (i != xtensa_mcore->active_core)
			xtensa_core_status_clear(&xtensa_mcore->cores_targets[i],
				OCDDSR_DEBUGPENDBREAK|OCDDSR_DEBUGINTBREAK);
	}
	target->debug_reason = DBG_REASON_SINGLESTEP;
	target->state = TARGET_HALTED;
	target_call_event_callbacks(target, TARGET_EVENT_HALTED);
	return retval;
}

int xtensa_mcore_mmu(struct target *target, int *enabled)
{
	struct xtensa_mcore_common *xtensa_mcore = target_to_xtensa_mcore(target);
	struct target *sub_target = &xtensa_mcore->cores_targets[xtensa_mcore->active_core];
	return sub_target->type->mmu(sub_target, enabled);
}

int xtensa_mcore_read_memory(struct target *target,
	target_addr_t address,
	uint32_t size,
	uint32_t count,
	uint8_t *buffer)
{
	struct xtensa_mcore_common *xtensa_mcore = target_to_xtensa_mcore(target);
	struct target *sub_target = &xtensa_mcore->cores_targets[xtensa_mcore->active_core];
	return sub_target->type->read_memory(sub_target, address, size, count, buffer);
}

int xtensa_mcore_read_buffer(struct target *target,
	target_addr_t address,
	uint32_t count,
	uint8_t *buffer)
{
	/*xtensa_mcore_read_memory can also read unaligned stuff. Just pass through to that routine.
	 * */
	return xtensa_mcore_read_memory(target, address, 1, count, buffer);
}

int xtensa_mcore_write_memory(struct target *target,
	target_addr_t address,
	uint32_t size,
	uint32_t count,
	const uint8_t *buffer)
{
	struct xtensa_mcore_common *xtensa_mcore = target_to_xtensa_mcore(target);
	struct target *sub_target = &xtensa_mcore->cores_targets[xtensa_mcore->active_core];
	return sub_target->type->write_memory(sub_target, address, size, count, buffer);
}

int xtensa_mcore_write_buffer(struct target *target,
	target_addr_t address,
	uint32_t count,
	const uint8_t *buffer)
{
	/*xtensa_mcore_write_memory can handle everything. Just pass on to that. */
	return xtensa_mcore_write_memory(target, address, 1, count, buffer);
}

int xtensa_mcore_checksum_memory(struct target *target,
	target_addr_t address,
	uint32_t count,
	uint32_t *checksum)
{
	struct xtensa_mcore_common *xtensa_mcore = target_to_xtensa_mcore(target);
	struct target *sub_target = &xtensa_mcore->cores_targets[xtensa_mcore->active_core];
	return sub_target->type->checksum_memory(sub_target, address, count, checksum);
}

int xtensa_mcore_get_gdb_reg_list(struct target *target,
	struct reg **reg_list[],
	int *reg_list_size,
	enum target_register_class reg_class)
{
	struct xtensa_mcore_common *xtensa_mcore = target_to_xtensa_mcore(target);
	struct target *sub_target = &xtensa_mcore->cores_targets[xtensa_mcore->active_core];
	return sub_target->type->get_gdb_reg_list(sub_target, reg_list, reg_list_size, reg_class);
}

static int xtensa_mcore_smpbreak_set_core(struct xtensa_mcore_common *xtensa_mcore, int core)
{
	uint32_t set = 0;
	if (xtensa_mcore->configured_cores_num > 1)
		set = xtensa_mcore->smp_break;
	return xtensa_smpbreak_set(&xtensa_mcore->cores_targets[core], set);
}

int xtensa_mcore_smpbreak_set(struct target *target)
{
	int res = ERROR_OK;
	struct xtensa_mcore_common *xtensa_mcore = target_to_xtensa_mcore(target);

	for (uint8_t core = 0; core < xtensa_mcore->configured_cores_num && res == ERROR_OK; core++)
		res = xtensa_mcore_smpbreak_set_core(xtensa_mcore, core);
	return res;
}

int xtensa_mcore_target_init(struct command_context *cmd_ctx, struct target *target)
{
	struct xtensa_mcore_common *xtensa_mcore = target_to_xtensa_mcore(target);

	LOG_DEBUG("%s", __func__);
	for (int i = 0; i < xtensa_mcore->configured_cores_num; i++) {
		struct target *sub_target = &xtensa_mcore->cores_targets[i];
		/* copy working area into xtensa core targets. */
		/* it is safe to have the same woking areas on both CPUs, because algorithm can be
		 * run only on one CPU at the same time */
		memcpy(&sub_target->working_area_cfg, &target->working_area_cfg,
			sizeof(sub_target->working_area_cfg));
		memcpy(&sub_target->alt_working_area_cfg, &target->alt_working_area_cfg,
			sizeof(sub_target->alt_working_area_cfg));
		int res = sub_target->type->init_target(cmd_ctx, sub_target);
		if (res != ERROR_OK)
			return res;
	}
	return ERROR_OK;
}

int xtensa_mcore_init_arch_info(struct target *target, struct xtensa_mcore_common *xtensa_mcore,
	uint32_t configured_cores_num,
	const struct xtensa_chip_ops *chip_ops,
	struct target_type *cores_target_types[],
	struct xtensa_core_ops *mcores_ops[],
	struct xtensa_config *xtensa_cfgs[],
	struct xtensa_debug_module_config dm_cfgs[],
	void *cores_privs[])
{
	target->arch_info = xtensa_mcore;
	xtensa_mcore->chip_ops = chip_ops;
	xtensa_mcore->cores_num = 0;	/* unknown */
	xtensa_mcore->configured_cores_num = configured_cores_num;
	LOG_INFO("Configured %d cores", xtensa_mcore->configured_cores_num);
	xtensa_mcore->cores_targets =
		calloc(xtensa_mcore->configured_cores_num, sizeof(struct target));
	if (xtensa_mcore->cores_targets == NULL) {
		LOG_ERROR("Failed to alloc memory for sub-targets!");
		return ERROR_FAIL;
	}
	xtensa_mcore->cores_ops =
		calloc(xtensa_mcore->configured_cores_num, sizeof(struct xtensa_core_ops));
	if (xtensa_mcore->cores_ops == NULL) {
		LOG_ERROR("Failed to alloc memory for cores ops!");
		return ERROR_FAIL;
	}
	memcpy(xtensa_mcore->cores_ops, mcores_ops,
		xtensa_mcore->configured_cores_num*sizeof(struct xtensa_core_ops *));
	for (uint32_t i = 0; i < xtensa_mcore->configured_cores_num; i++) {
		struct target *subtarget = &xtensa_mcore->cores_targets[i];
		memcpy(subtarget, target, sizeof(struct target));
		subtarget->type = cores_target_types[i];
		subtarget->coreid = i;
		subtarget->tap = dm_cfgs[i].tap;
		subtarget->cmd_name = malloc(snprintf(NULL, 0, "cpu%u", i) + 1);
		if (subtarget->cmd_name == NULL) {
			LOG_ERROR("Failed to alloc memory for su-target name!");
			/*TODO: free cmd_name for already initialized subtargets */
			free(xtensa_mcore->cores_ops);
			free(xtensa_mcore->cores_targets);
			return ERROR_FAIL;
		}
		sprintf(subtarget->cmd_name, "cpu%u", i);
		int ret = mcores_ops[i]->core_init_arch_info(subtarget,
			target, NULL,
			xtensa_cfgs[i],
			&dm_cfgs[i],
			NULL,
			cores_privs[i]);
		if (ret != ERROR_OK) {
			LOG_ERROR("Failed to alloc memory for arch info!");
			free(subtarget->cmd_name);	/*TODO: free cmd_name for already
							 * initialized subtargets */
			free(xtensa_mcore->cores_ops);
			free(xtensa_mcore->cores_targets);
			return ret;
		}
		subtarget->state = TARGET_RUNNING;
		subtarget->debug_reason = DBG_REASON_NOTHALTED;
	}
	return ERROR_OK;
}

/* TODO: refactor this function into smaller ones */
int xtensa_mcore_poll(struct target *target)
{
	struct xtensa_mcore_common *xtensa_mcore = target_to_xtensa_mcore(target);
	bool need_resume = false;
	int res;

	uint32_t core_poweron_mask = 0;
	for (uint8_t i = 0; i < xtensa_mcore->configured_cores_num; i++) {
		struct xtensa *xtensa = target_to_xtensa(&xtensa_mcore->cores_targets[i]);
		if (xtensa_dm_is_online(&xtensa->dbg_mod))
			core_poweron_mask |= (1 << i);
	}
	/* Target might be held in reset by external signal.
	 * Sanity check target responses using idcode (checking CPU0 is sufficient). */
	if (core_poweron_mask == 0) {
		if (target->state != TARGET_UNKNOWN) {
			LOG_INFO("%s: Target offline", __func__);
			target->state = TARGET_UNKNOWN;
		}
		for (uint8_t i = 0; i < xtensa_mcore->configured_cores_num; i++) {
			struct xtensa *xtensa = target_to_xtensa(&xtensa_mcore->cores_targets[i]);
			xtensa_dm_power_status_cache_reset(&xtensa->dbg_mod);
		}
		xtensa_mcore->core_poweron_mask = 0;
		LOG_ERROR("%s: Target failure", __func__);
		return ERROR_TARGET_FAILURE;
	}
	uint32_t cores_came_online = core_poweron_mask & (~xtensa_mcore->core_poweron_mask);
	xtensa_mcore->core_poweron_mask = core_poweron_mask;
	if (cores_came_online != 0)
		LOG_DEBUG("%s: core_poweron_mask=%x", __func__, core_poweron_mask);

	for (size_t i = 0; i < xtensa_mcore->configured_cores_num; i++) {
		struct xtensa *xtensa = target_to_xtensa(&xtensa_mcore->cores_targets[i]);
		res = xtensa_dm_power_status_read(&xtensa->dbg_mod,
			PWRSTAT_DEBUGWASRESET|PWRSTAT_COREWASRESET);
		if (res != ERROR_OK)
			return res;
		if (xtensa_dm_tap_was_reset(&xtensa->dbg_mod)) {
			LOG_INFO("%s: Debug controller %d was reset.", target_name(target), (int)i);
			xtensa_mcore->core_poweron_mask &= ~(1 << i);
		}
		if (xtensa_dm_core_was_reset(&xtensa->dbg_mod)) {
			LOG_INFO("%s: Core %d was reset.", target_name(target), (int)i);
			if (xtensa_mcore->cores_num > 0)
				xtensa_mcore->cores_num = 0;	/* unknown */
			xtensa_mcore->cores_ops[i]->core_on_reset(&xtensa_mcore->cores_targets[i]);
		}
		xtensa_dm_power_status_cache(&xtensa->dbg_mod);
		/*Enable JTAG, set reset if needed */
		res = xtensa_wakeup(&xtensa_mcore->cores_targets[i]);
		if (res != ERROR_OK)
			return res;
	}

	for (size_t i = 0; i < xtensa_mcore->configured_cores_num; i++) {
		struct xtensa *xtensa = target_to_xtensa(&xtensa_mcore->cores_targets[i]);
		if (cores_came_online & (1 << i)) {
			LOG_DEBUG("%s: Core %d came online, setting up DCR", __func__, (int) i);
			xtensa_mcore_smpbreak_set_core(xtensa_mcore, i);
		}
		res = xtensa_dm_core_status_read(&xtensa->dbg_mod);
		if (res != ERROR_OK)
			return res;
	}

	xtensa_dsr_t common_core_stat = 0;
	xtensa_pwrstat_t common_power_stat = 0;
	for (size_t i = 0; i < xtensa_mcore->configured_cores_num; i++) {
		struct xtensa *xtensa = target_to_xtensa(&xtensa_mcore->cores_targets[i]);
		common_core_stat |= xtensa_dm_core_status_get(&xtensa->dbg_mod);
		common_power_stat |= xtensa_dm_power_status_get(&xtensa->dbg_mod);
	}
	if (common_core_stat & OCDDSR_STOPPED) {
		int oldstate = target->state;
		bool algo_stopped = false;
		if (target->state == TARGET_DEBUG_RUNNING) {
			/* algo can be run on any CPU while other one can be stalled by SW run on target, in this case OCDDSR_STOPPED will not be set for other CPU;
			 situation when target is in TARGET_DEBUG_RUNNING and both CPUs are stalled is impossible (one must run algo). */
			algo_stopped = true;
			size_t enab_cores_num = xtensa_mcore_get_enabled_cores_count(target);
			if (enab_cores_num > 1) {
				for (size_t i = 0; i < enab_cores_num; i++) {
					struct xtensa *xtensa = target_to_xtensa(
						&xtensa_mcore->cores_targets[i]);
					if (!(xtensa_dm_core_status_get(&xtensa->dbg_mod) &
							(OCDDSR_STOPPED|OCDDSR_RUNSTALLSAMPLE))) {
						algo_stopped = false;	/* algo is not stopped if
									 * any of the core is
									 * running */
						break;
					}
				}
			} else {
				struct xtensa *xtensa = target_to_xtensa(
					&xtensa_mcore->cores_targets[0]);
				algo_stopped = xtensa_dm_core_status_get(&xtensa->dbg_mod) &
					OCDDSR_STOPPED;
			}
		}
		if ((target->state != TARGET_HALTED && target->state != TARGET_DEBUG_RUNNING) ||
			algo_stopped) {
			/* TODO: When BreakIn/BreakOut is enabled other CPU is stopped
			 * automatically. */
			/* Should we stop the second CPU if BreakIn/BreakOut is not configured? */
			target->state = TARGET_HALTED;
			/*Examine why the target has been halted */
			target->debug_reason = DBG_REASON_UNDEFINED;
			for (size_t i = 0; i < xtensa_mcore->configured_cores_num; i++) {
				xtensa_fetch_all_regs(&xtensa_mcore->cores_targets[i]);
				xtensa_mcore->cores_targets[i].state = TARGET_HALTED;
			}
			/* When setting debug reason DEBUGCAUSE events have the followuing
			 * priorites: watchpoint == breakpoint > single step > debug interrupt. */
			/* Watchpoint and breakpoint events at the same time results in special
			 * debug reason: DBG_REASON_WPTANDBKPT. */
			/* When similar debug events are present on both CPUs events on CPU0 take
			 * priority over CPU1 ones and CPU0 is considred to be active. */
			/* TODO: review this scheme for the case when BreakIn/BreakOut is not
			 * enabled */
			for (size_t k = xtensa_mcore->configured_cores_num; k > 0; k--) {
				struct target *sub_target = &xtensa_mcore->cores_targets[k-1];
				xtensa_reg_val_t halt_cause = xtensa_reg_get(sub_target,
					XT_REG_IDX_DEBUGCAUSE);
				struct xtensa *xtensa = target_to_xtensa(sub_target);
				if ((halt_cause & DEBUGCAUSE_DI) &&
					((xtensa_dm_core_status_get(&xtensa->dbg_mod)&
							(OCDDSR_DEBUGPENDHOST|
								OCDDSR_DEBUGINTHOST)) != 0)) {
					target->debug_reason = DBG_REASON_DBGRQ;
					xtensa_mcore->active_core = k-1;
				}
			}
			for (size_t k = xtensa_mcore->configured_cores_num; k > 0; k--) {
				xtensa_reg_val_t halt_cause = xtensa_reg_get(
					&xtensa_mcore->cores_targets[k-1],
					XT_REG_IDX_DEBUGCAUSE);
				if (halt_cause & DEBUGCAUSE_IC) {
					target->debug_reason = DBG_REASON_SINGLESTEP;
					xtensa_mcore->active_core = k-1;
				}
			}
			for (size_t k = xtensa_mcore->configured_cores_num; k > 0; k--) {
				xtensa_reg_val_t halt_cause = xtensa_reg_get(
					&xtensa_mcore->cores_targets[k-1],
					XT_REG_IDX_DEBUGCAUSE);
				if (halt_cause & (DEBUGCAUSE_IB | DEBUGCAUSE_BN | DEBUGCAUSE_BI)) {
					if (halt_cause & DEBUGCAUSE_DB)
						target->debug_reason = DBG_REASON_WPTANDBKPT;
					else
						target->debug_reason = DBG_REASON_BREAKPOINT;
					xtensa_mcore->active_core = k-1;
				} else if (halt_cause & DEBUGCAUSE_DB) {
					target->debug_reason = DBG_REASON_WATCHPOINT;
					xtensa_mcore->active_core = k-1;
				}
			}
			/* Handle special case when halt is send to the stalled active core. */
			/* We need to switch focus to another core in order to be able to run
			 * algorithms. */
			if (target->debug_reason == DBG_REASON_DBGRQ) {
				struct xtensa *xtensa = target_to_xtensa(
					&xtensa_mcore->cores_targets[xtensa_mcore->
						active_core]);
				if (xtensa_dm_core_status_get(&xtensa->dbg_mod) &
					OCDDSR_RUNSTALLSAMPLE) {
					LOG_DEBUG(
						"Received debug request on stalled active core %d. Switch active core.",
						(int)xtensa_mcore->active_core);
					/* find not stalled core */
					size_t k;
					for (k = 0; k < xtensa_mcore->configured_cores_num; k++) {
						if (k == xtensa_mcore->active_core)
							continue;
						xtensa = target_to_xtensa(
							&xtensa_mcore->cores_targets[k]);
						if ((xtensa_dm_core_status_get(&xtensa->dbg_mod) &
								OCDDSR_RUNSTALLSAMPLE) == 0) {
							xtensa_mcore->active_core = k;
							break;
						}
					}
					if (k == xtensa_mcore->configured_cores_num)
						LOG_WARNING(
							"Failed to switch stalled active core %d. All cores are stalled!",
							(int)xtensa_mcore->active_core);
				}
			}
			for (size_t i = 0; i < xtensa_mcore->configured_cores_num; i++) {
				struct target *sub_target = &xtensa_mcore->cores_targets[i];
				struct xtensa *xtensa = target_to_xtensa(sub_target);
				LOG_DEBUG(
					"%s.%s: Target halted, pc=0x%08X, debug_reason=%08x, oldstate=%08x, active=%s",
					target_name(target),
					target_name(sub_target),
					xtensa_reg_get(sub_target, XT_REG_IDX_PC),
					target->debug_reason,
					oldstate,
					(i == xtensa_mcore->active_core) ? "true" : "false");
				LOG_DEBUG("%s.%s: Halt reason=0x%08X, exc_cause=%d, dsr=0x%08x",
					target_name(target),
					target_name(sub_target),
					xtensa_reg_get(sub_target, XT_REG_IDX_DEBUGCAUSE),
					xtensa_reg_get(sub_target, XT_REG_IDX_EXCCAUSE),
					xtensa_dm_core_status_get(&xtensa->dbg_mod));
				LOG_INFO("Target halted. CPU%u: PC=0x%08X %s",
					(uint32_t)i,
					xtensa_reg_get(sub_target, XT_REG_IDX_PC),
					xtensa_mcore->active_core == i ? "(active)" : "");
				xtensa_dm_core_status_clear(
					&xtensa->dbg_mod,
					OCDDSR_DEBUGPENDBREAK|OCDDSR_DEBUGINTBREAK|
					OCDDSR_DEBUGPENDHOST|
					OCDDSR_DEBUGINTHOST);
			}

			target->reg_cache =
				xtensa_mcore->cores_targets[xtensa_mcore->active_core].reg_cache;
			target->coreid = xtensa_mcore->active_core;
			/*Call any event callbacks that are applicable
			 * TODO: disable WDTs */
			if (oldstate == TARGET_DEBUG_RUNNING)
				target_call_event_callbacks(target, TARGET_EVENT_DEBUG_HALTED);
			else {
				need_resume = xtensa_mcore->chip_ops->on_halt !=
					NULL ? xtensa_mcore->chip_ops->on_halt(target) : false;
				/* in case of semihosting call we will resume automatically a bit
				 * later, so do not confuse GDB */
				if (!need_resume)
					target_call_event_callbacks(target, TARGET_EVENT_HALTED);
			}
		}
	} else if (common_power_stat & PWRSTAT_COREWASRESET) {
		target->state = TARGET_RESET;
		for (size_t i = 0; i < xtensa_mcore->configured_cores_num; i++)
			xtensa_mcore->cores_targets[i].state = TARGET_RESET;
	} else {
		target->debug_reason = DBG_REASON_NOTHALTED;
		if (target->state != TARGET_RUNNING && target->state != TARGET_DEBUG_RUNNING) {
			LOG_DEBUG("%s: Target %s -> RUNNING",
				target_name(target),
				target_state_name(target));
			target->state = TARGET_RUNNING;
			for (size_t i = 0; i < xtensa_mcore->configured_cores_num; i++)
				xtensa_mcore->cores_targets[i].state = TARGET_RUNNING;
			target->debug_reason = DBG_REASON_NOTHALTED;
		}
	}
	if (xtensa_mcore->chip_ops->on_poll != NULL)
		xtensa_mcore->chip_ops->on_poll(target);
	if (need_resume) {
		res = target_resume(target, 1, 0, 1, 0);
		if (res != ERROR_OK)
			LOG_ERROR("Failed to resume target upon core request!");
	}
	return ERROR_OK;
}

int xtensa_mcore_examine(struct target *target)
{
	struct xtensa_mcore_common *xtensa_mcore = target_to_xtensa_mcore(target);
	bool core_online = false;

	for (size_t i = 0; i < xtensa_mcore->configured_cores_num; i++) {
		struct target *sub_target = &xtensa_mcore->cores_targets[i];
		int res = sub_target->type->examine(sub_target);
		if (res != ERROR_OK) {
			if (res != ERROR_TARGET_FAILURE)
				return res;
			/* ERROR_TARGET_FAILURE means that xtensa is offline */
		} else
			core_online = true;
	}
	if (core_online) {
		/* we can work if at least one core is online */
		if (!target_was_examined(target))
			target_set_examined(target);
		return ERROR_OK;
	}
	return ERROR_TARGET_FAILURE;
}

int xtensa_mcore_breakpoint_add(struct target *target, struct breakpoint *breakpoint)
{
	struct xtensa_mcore_common *xtensa_mcore = target_to_xtensa_mcore(target);
	int res;

	for (size_t i = 0; i < xtensa_mcore->configured_cores_num; i++) {
		struct target *sub_target = &xtensa_mcore->cores_targets[i];
		res = sub_target->type->add_breakpoint(sub_target, breakpoint);
		if (breakpoint->type == BKPT_SOFT ||
			xtensa_mcore->cores_ops[i]->core_is_special_breakpoint(sub_target,
				breakpoint)) {
			/* SW breakpoints are global for all cores,
			 * so addition on one core adds it for all other too */
			break;
		} else if (res != ERROR_OK)
			break;
	}
	return res;
}

int xtensa_mcore_breakpoint_remove(struct target *target, struct breakpoint *breakpoint)
{
	struct xtensa_mcore_common *xtensa_mcore = target_to_xtensa_mcore(target);
	int res;

	for (size_t i = 0; i < xtensa_mcore->configured_cores_num; i++) {
		struct target *sub_target = &xtensa_mcore->cores_targets[i];
		bool is_special_breakpoint = xtensa_mcore->cores_ops[i]->core_is_special_breakpoint(
			sub_target,
			breakpoint);
		res = sub_target->type->remove_breakpoint(sub_target, breakpoint);
		if (breakpoint->type == BKPT_SOFT || is_special_breakpoint) {
			/* SW breakpoints are global for all cores,
			 * so removal on one core removes it for all other too */
			break;
		} else if (res != ERROR_OK)
			break;
	}
	return res;
}

int xtensa_mcore_watchpoint_add(struct target *target, struct watchpoint *watchpoint)
{
	struct xtensa_mcore_common *xtensa_mcore = target_to_xtensa_mcore(target);
	int res = ERROR_OK;

	for (size_t i = 0; i < xtensa_mcore->configured_cores_num; i++) {
		struct target *sub_target = &xtensa_mcore->cores_targets[i];
		res = sub_target->type->add_watchpoint(sub_target, watchpoint);
		if (res != ERROR_OK)
			break;
	}
	return res;
}

int xtensa_mcore_watchpoint_remove(struct target *target, struct watchpoint *watchpoint)
{
	struct xtensa_mcore_common *xtensa_mcore = target_to_xtensa_mcore(target);
	int res = ERROR_OK;

	for (size_t i = 0; i < xtensa_mcore->configured_cores_num; i++) {
		struct target *sub_target = &xtensa_mcore->cores_targets[i];
		res = sub_target->type->remove_watchpoint(sub_target, watchpoint);
		if (res != ERROR_OK)
			break;
	}
	return res;
}

int xtensa_mcore_start_algorithm(struct target *target,
	int num_mem_params, struct mem_param *mem_params,
	int num_reg_params, struct reg_param *reg_params,
	target_addr_t entry_point, target_addr_t exit_point,
	void *arch_info)
{
	struct xtensa_mcore_common *xtensa_mcore = target_to_xtensa_mcore(target);
	struct target *sub_target = &xtensa_mcore->cores_targets[xtensa_mcore->active_core];

	return sub_target->type->start_algorithm(sub_target,
		num_mem_params, mem_params,
		num_reg_params, reg_params,
		entry_point, exit_point,
		arch_info);
}

/** Waits for an algorithm in the target. */
int xtensa_mcore_wait_algorithm(struct target *target,
	int num_mem_params, struct mem_param *mem_params,
	int num_reg_params, struct reg_param *reg_params,
	target_addr_t exit_point, int timeout_ms,
	void *arch_info)
{
	struct xtensa_mcore_common *xtensa_mcore = target_to_xtensa_mcore(target);
	struct target *sub_target = &xtensa_mcore->cores_targets[xtensa_mcore->active_core];

	return sub_target->type->wait_algorithm(sub_target,
		num_mem_params, mem_params,
		num_reg_params, reg_params,
		exit_point, timeout_ms, arch_info);
}

int xtensa_mcore_run_algorithm(struct target *target,
	int num_mem_params, struct mem_param *mem_params,
	int num_reg_params, struct reg_param *reg_params,
	target_addr_t entry_point, target_addr_t exit_point,
	int timeout_ms, void *arch_info)
{
	struct xtensa_mcore_common *xtensa_mcore = target_to_xtensa_mcore(target);
	struct target *sub_target = &xtensa_mcore->cores_targets[xtensa_mcore->active_core];

	return sub_target->type->run_algorithm(sub_target,
		num_mem_params, mem_params,
		num_reg_params, reg_params,
		entry_point, exit_point,
		timeout_ms, arch_info);
}

int xtensa_mcore_run_func_image(struct target *target,
	struct xtensa_algo_run_data *run,
	struct xtensa_algo_image *image,
	uint32_t num_args,
	...)
{
	struct xtensa_mcore_common *xtensa_mcore = target_to_xtensa_mcore(target);
	struct target *sub_target = &xtensa_mcore->cores_targets[xtensa_mcore->active_core];
	va_list ap;

	va_start(ap, num_args);
	int retval = xtensa_run_func_image_va(sub_target, run, image, num_args, ap);
	va_end(ap);
	if ((xtensa_mcore->smp_break & (OCDDCR_BREAKINEN|OCDDCR_BREAKOUTEN)) ==
		(OCDDCR_BREAKINEN|OCDDCR_BREAKOUTEN)) {
		/* need to clear core status bits on other cores */
		for (size_t i = 0; i < xtensa_mcore->configured_cores_num; i++) {
			if (i != xtensa_mcore->active_core) {
				sub_target = &xtensa_mcore->cores_targets[i];
				LOG_DEBUG("%s: check/clear DSR", target_name(target));
				xtensa_core_status_check(sub_target);
				xtensa_core_status_clear(sub_target,
					OCDDSR_DEBUGPENDBREAK|OCDDSR_DEBUGINTBREAK);
			}
		}
	}
	return retval;
}

size_t xtensa_mcore_get_enabled_cores_count(struct target *target)
{
	struct xtensa_mcore_common *xtensa_mcore = target_to_xtensa_mcore(target);
	if (xtensa_mcore == NULL)
		return 0;
	return xtensa_mcore->cores_num ? xtensa_mcore->cores_num : xtensa_mcore->
	       configured_cores_num;
}

size_t xtensa_mcore_get_active_core(struct target *target)
{
	struct xtensa_mcore_common *xtensa_mcore = target_to_xtensa_mcore(target);
	return xtensa_mcore->active_core;
}

void xtensa_mcore_set_active_core(struct target *target, size_t core)
{
	struct xtensa_mcore_common *xtensa_mcore = target_to_xtensa_mcore(target);
	xtensa_mcore->active_core = core;
}

COMMAND_HANDLER(xtensa_mcore_cmd_smpbreak)
{
	struct target *target = get_current_target(CMD_CTX);
	struct xtensa_mcore_common *xtensa_mcore = target_to_xtensa_mcore(target);
	int res = ERROR_OK;
	uint32_t set = 0;

	if (CMD_ARGC >= 1) {
		for (uint32_t i = 0; i < CMD_ARGC; i++) {
			if (!strcasecmp(CMD_ARGV[0], "none"))
				set = 0;
			else if (!strcasecmp(CMD_ARGV[i], "BreakIn"))
				set |= OCDDCR_BREAKINEN;
			else if (!strcasecmp(CMD_ARGV[i], "BreakOut"))
				set |= OCDDCR_BREAKOUTEN;
			else if (!strcasecmp(CMD_ARGV[i], "RunStallIn"))
				set |= OCDDCR_RUNSTALLINEN;
			else if (!strcasecmp(CMD_ARGV[i], "DebugModeOut"))
				set |= OCDDCR_DEBUGMODEOUTEN;
			else if (!strcasecmp(CMD_ARGV[i], "BreakInOut"))
				set |= OCDDCR_BREAKINEN|OCDDCR_BREAKOUTEN;
			else if (!strcasecmp(CMD_ARGV[i], "RunStall"))
				set |= OCDDCR_RUNSTALLINEN|OCDDCR_DEBUGMODEOUTEN;
			else {
				command_print(CMD, "Unknown arg %s", CMD_ARGV[i]);
				command_print(
					CMD,
					"use either BreakInOut, None or RunStall as arguments, or any combination of BreakIn, BreakOut, RunStallIn and DebugModeOut.");
				return ERROR_OK;
			}
		}
		xtensa_mcore->smp_break = set;
		res = xtensa_mcore_smpbreak_set(target);
	} else {
		command_print(CMD, "%s: Current bits set:%s%s%s%s",
			target_name(target),
			(xtensa_mcore->smp_break & OCDDCR_BREAKINEN) ? " BreakIn" : "",
			(xtensa_mcore->smp_break & OCDDCR_BREAKOUTEN) ? " BreakOut" : "",
			(xtensa_mcore->smp_break & OCDDCR_RUNSTALLINEN) ? " RunStallIn" : "",
			(xtensa_mcore->smp_break & OCDDCR_DEBUGMODEOUTEN) ? " DebugModeOut" : ""
			);
	}
	return res;
}

COMMAND_HANDLER(xtensa_mcore_cmd_permissive_mode)
{
	struct xtensa_mcore_common *xtensa_mcore = target_to_xtensa_mcore(get_current_target(
			CMD_CTX));

	for (int i = 0; i < xtensa_mcore->configured_cores_num; i++) {
		int ret =
			CALL_COMMAND_HANDLER(xtensa_cmd_permissive_mode_do,
			target_to_xtensa(&xtensa_mcore->cores_targets[i]));
		if (ret != ERROR_OK)
			return ret;
	}
	return ERROR_OK;
}

COMMAND_HANDLER(xtensa_mcore_cmd_mask_interrupts)
{
	struct target *target = get_current_target(CMD_CTX);
	struct xtensa_mcore_common *xtensa_mcore = target_to_xtensa_mcore(target);

	if (CMD_ARGC < 1)
		return CALL_COMMAND_HANDLER(xtensa_cmd_mask_interrupts_do,
			target_to_xtensa(&xtensa_mcore->cores_targets[0]));
	for (int i = 0; i < xtensa_mcore->configured_cores_num; i++) {
		int res =
			CALL_COMMAND_HANDLER(xtensa_cmd_mask_interrupts_do,
			target_to_xtensa(&xtensa_mcore->cores_targets[i]));
		if (res != ERROR_OK)
			return res;
	}
	return ERROR_OK;
}

/* perfmon_enable <counter_id> <select> [mask] [kernelcnt] [tracelevel] */
COMMAND_HANDLER(xtensa_mcore_cmd_perfmon_enable)
{
	struct target *target = get_current_target(CMD_CTX);
	struct xtensa_mcore_common *xtensa_mcore = target_to_xtensa_mcore(target);

	for (int i = 0; i < xtensa_mcore->configured_cores_num; i++) {
		int res =
			CALL_COMMAND_HANDLER(xtensa_cmd_perfmon_enable_do,
			target_to_xtensa(&xtensa_mcore->cores_targets[i]));
		if (res != ERROR_OK)
			return res;
	}
	return ERROR_OK;
}

/* perfmon_dump [counter_id] */
COMMAND_HANDLER(xtensa_mcore_cmd_perfmon_dump)
{
	struct target *target = get_current_target(CMD_CTX);
	struct xtensa_mcore_common *xtensa_mcore = target_to_xtensa_mcore(target);

	for (int i = 0; i < xtensa_mcore->configured_cores_num; i++) {
		LOG_INFO("CPU%d:", i);
		int res =
			CALL_COMMAND_HANDLER(xtensa_cmd_perfmon_dump_do,
			target_to_xtensa(&xtensa_mcore->cores_targets[i]));
		if (res != ERROR_OK)
			return res;
	}
	return ERROR_OK;
}

COMMAND_HANDLER(xtensa_mcore_cmd_tracestart)
{
	struct target *target = get_current_target(CMD_CTX);
	struct xtensa_mcore_common *xtensa_mcore = target_to_xtensa_mcore(target);

	for (int i = 0; i < xtensa_mcore->configured_cores_num; i++) {
		int res =
			CALL_COMMAND_HANDLER(xtensa_cmd_tracestart_do,
			target_to_xtensa(&xtensa_mcore->cores_targets[i]));
		if (res != ERROR_OK)
			return res;
	}
	return ERROR_OK;
}

COMMAND_HANDLER(xtensa_mcore_cmd_tracestop)
{
	struct target *target = get_current_target(CMD_CTX);
	struct xtensa_mcore_common *xtensa_mcore = target_to_xtensa_mcore(target);

	for (int i = 0; i < xtensa_mcore->configured_cores_num; i++) {
		int res =
			CALL_COMMAND_HANDLER(xtensa_cmd_tracestop_do,
			target_to_xtensa(&xtensa_mcore->cores_targets[i]));
		if (res != ERROR_OK)
			return res;
	}
	return ERROR_OK;
}

COMMAND_HANDLER(xtensa_mcore_cmd_tracedump)
{
	struct target *target = get_current_target(CMD_CTX);
	struct xtensa_mcore_common *xtensa_mcore = target_to_xtensa_mcore(target);

	if (CMD_ARGC < xtensa_mcore->configured_cores_num) {
		command_print(CMD,
			"Need %d filenames to dump to as output!",
			xtensa_mcore->configured_cores_num);
		return ERROR_FAIL;
	}

	for (int i = 0; i < xtensa_mcore->configured_cores_num; i++) {
		int res =
			CALL_COMMAND_HANDLER(xtensa_cmd_tracedump_do,
			target_to_xtensa(&xtensa_mcore->cores_targets[i]), CMD_ARGV[i]);
		if (res != ERROR_OK)
			return res;
	}

	return ERROR_OK;
}

const struct command_registration xtensa_mcore_command_handlers[] = {
	{
		.name = "set_permissive",
		.handler = xtensa_mcore_cmd_permissive_mode,
		.mode = COMMAND_ANY,
		.help = "When set to 1, enable ESP108 permissive mode (less client-side checks)",
		.usage = "[0|1]",
	},
	{
		.name = "maskisr",
		.handler = xtensa_mcore_cmd_mask_interrupts,
		.mode = COMMAND_ANY,
		.help = "mask Xtensa interrupts at step",
		.usage = "['on'|'off']",
	},
	{
		.name = "smpbreak",
		.handler = xtensa_mcore_cmd_smpbreak,
		.mode = COMMAND_EXEC,
		.help = "Set the way the CPU chains OCD breaks",
		.usage =
			"[none|breakinout|runstall] | [BreakIn] [BreakOut] [RunStallIn] [DebugModeOut]",
	},
	{
		.name = "perfmon_enable",
		.handler = xtensa_mcore_cmd_perfmon_enable,
		.mode = COMMAND_EXEC,
		.help = "Enable and start performance counter",
		.usage = "<counter_id> <select> [mask] [kernelcnt] [tracelevel]",
	},
	{
		.name = "perfmon_dump",
		.handler = xtensa_mcore_cmd_perfmon_dump,
		.mode = COMMAND_EXEC,
		.help =
			"Dump performance counter value. If no argument specified, dumps all counters.",
		.usage = "[counter_id]",
	},
	{
		.name = "tracestart",
		.handler = xtensa_mcore_cmd_tracestart,
		.mode = COMMAND_EXEC,
		.help =
			"Tracing: Set up and start a trace. Optionally set stop trigger address and amount of data captured after.",
		.usage = "[pc <pcval>/[maskbitcount]] [after <n> [ins|words]]",
	},
	{
		.name = "tracestop",
		.handler = xtensa_mcore_cmd_tracestop,
		.mode = COMMAND_EXEC,
		.help = "Tracing: Stop current trace as started by the tracestart command",
		.usage = "",
	},
	{
		.name = "tracedump",
		.handler = xtensa_mcore_cmd_tracedump,
		.mode = COMMAND_EXEC,
		.help = "Tracing: Dump trace memory to a files. One file per core.",
		.usage = "<outfile1> [outfile2 ... outfileN]",
	},
	COMMAND_REGISTRATION_DONE
};
