/***************************************************************************
 *   Multi-core Xtensa target for OpenOCD                                  *
 *   Copyright (C) 2017 Espressif Systems Ltd.                             *
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

#ifndef XTENSA_MCORE_H
#define XTENSA_MCORE_H

#include <target/image.h>
#include "target.h"
#include "command.h"
#include "xtensa.h"
#include "xtensa_algorithm.h"

struct xtensa_core_ops {
	int (*core_init_arch_info)(struct target *target, struct target *chip_target,
		void *arch_info, const struct xtensa_config *xtensa_cfg,
		struct xtensa_debug_module_config *dm_cfg, const struct xtensa_chip_ops *chip_ops,
		void *priv);
	void (*core_on_reset)(struct target *target);
	int (*core_do_step)(struct target *target,
		int current,
		target_addr_t address,
		int handle_breakpoints);
	int (*core_prepare_resume)(struct target *target,
		int current,
		target_addr_t address,
		int handle_breakpoints,
		int debug_execution);
	int (*core_do_resume)(struct target *target);
	/* 'special' means that breakpoint is neither HW nor SW and requires some unusual
	 * (chip-specific) handling (running flasher or smth like this). */
	bool (*core_is_special_breakpoint)(struct target *target, struct breakpoint *breakpoint);
};

struct xtensa_mcore_common {
	/* Pointer to array of sub-targets for cores. Array length is `configured_cores_num`. */
	struct target *cores_targets;
	/* Pointer to array of multi-core support operations for cores. Array length is
	 * `configured_cores_num`. */
	struct xtensa_core_ops **cores_ops;
	const struct xtensa_chip_ops *chip_ops;
	/* Number of configured cores */
	uint8_t configured_cores_num;
	/* Number of working cores */
	uint8_t cores_num;
	size_t active_core;
	uint32_t core_poweron_mask;
	uint32_t smp_break;
};

extern const struct command_registration xtensa_mcore_command_handlers[];

static inline struct xtensa_mcore_common *target_to_xtensa_mcore(struct target *target)
{
	return (struct xtensa_mcore_common *)target->arch_info;
}

static inline void xtensa_mcore_stepping_isr_mode_set(struct target *target,
	enum xtensa_stepping_isr_mode mode)
{
	struct xtensa_mcore_common *xtensa_mcore = target_to_xtensa_mcore(target);
	for (size_t i = 0; i < xtensa_mcore->configured_cores_num; i++) {
		struct xtensa *xtensa = target_to_xtensa(&xtensa_mcore->cores_targets[i]);
		xtensa->stepping_isr_mode = mode;
	}
}

int xtensa_mcore_init_arch_info(struct target *target, struct xtensa_mcore_common *xtensa_mcore,
	uint32_t configured_cores_num,
	const struct xtensa_chip_ops *chip_ops,
	struct target_type *cores_target_types[],
	struct xtensa_core_ops *mcores_ops[],
	struct xtensa_config *xtensa_cfgs[],
	struct xtensa_debug_module_config dm_cfgs[],
	void *cores_privs[]);
int xtensa_mcore_target_init(struct command_context *cmd_ctx, struct target *target);
int xtensa_mcore_poll(struct target *target);
int xtensa_mcore_assert_reset(struct target *target);
int xtensa_mcore_deassert_reset(struct target *target);
int xtensa_mcore_smpbreak_set(struct target *target);
int xtensa_mcore_examine(struct target *target);
int xtensa_mcore_read_memory(struct target *target,
	target_addr_t address,
	uint32_t size,
	uint32_t count,
	uint8_t *buffer);
int xtensa_mcore_read_buffer(struct target *target,
	target_addr_t address,
	uint32_t count,
	uint8_t *buffer);
int xtensa_mcore_write_memory(struct target *target,
	target_addr_t address,
	uint32_t size,
	uint32_t count,
	const uint8_t *buffer);
int xtensa_mcore_write_buffer(struct target *target,
	target_addr_t address,
	uint32_t count,
	const uint8_t *buffer);
int xtensa_mcore_checksum_memory(struct target *target,
	target_addr_t address,
	uint32_t count,
	uint32_t *checksum);
size_t xtensa_mcore_get_enabled_cores_count(struct target *target);
size_t xtensa_mcore_get_active_core(struct target *target);
void xtensa_mcore_set_active_core(struct target *target, size_t core);
int xtensa_mcore_halt(struct target *target);
int xtensa_mcore_resume(struct target *target,
	int current,
	target_addr_t address,
	int handle_breakpoints,
	int debug_execution);
int xtensa_mcore_step(struct target *target,
	int current,
	target_addr_t address,
	int handle_breakpoints);
int xtensa_mcore_mmu(struct target *target, int *enabled);
int xtensa_mcore_get_gdb_reg_list(struct target *target,
	struct reg **reg_list[],
	int *reg_list_size,
	enum target_register_class reg_class);
int xtensa_mcore_breakpoint_add(struct target *target, struct breakpoint *breakpoint);
int xtensa_mcore_breakpoint_remove(struct target *target, struct breakpoint *breakpoint);
int xtensa_mcore_watchpoint_add(struct target *target, struct watchpoint *watchpoint);
int xtensa_mcore_watchpoint_remove(struct target *target, struct watchpoint *watchpoint);
int xtensa_mcore_start_algorithm(struct target *target,
	int num_mem_params, struct mem_param *mem_params,
	int num_reg_params, struct reg_param *reg_params,
	target_addr_t entry_point, target_addr_t exit_point,
	void *arch_info);
int xtensa_mcore_wait_algorithm(struct target *target,
	int num_mem_params, struct mem_param *mem_params,
	int num_reg_params, struct reg_param *reg_params,
	target_addr_t exit_point, int timeout_ms,
	void *arch_info);
int xtensa_mcore_run_algorithm(struct target *target,
	int num_mem_params, struct mem_param *mem_params,
	int num_reg_params, struct reg_param *reg_params,
	target_addr_t entry_point, target_addr_t exit_point,
	int timeout_ms, void *arch_info);

int xtensa_mcore_run_func_image(struct target *target,
	struct xtensa_algo_run_data *run,
	struct xtensa_algo_image *image,
	uint32_t num_args,
	...);

#endif	/* XTENSA_MCORE_H */
