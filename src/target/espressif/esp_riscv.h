/***************************************************************************
 *   ESP RISCV common definitions for OpenOCD                              *
 *   Copyright (C) 2020 Espressif Systems Ltd.                             *
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

#ifndef OPENOCD_TARGET_ESP_RISCV_H
#define OPENOCD_TARGET_ESP_RISCV_H

#include <target/target.h>
#include <target/riscv/riscv.h>
#include <target/riscv/debug_defines.h>
#include "esp_riscv_apptrace.h"
#include "esp_riscv_algorithm.h"
#include "esp.h"

#define get_field(reg, mask) (((reg) & (mask)) / ((mask) & ~((mask) << 1)))
#define set_field(reg, mask, val) (((reg) & ~(mask)) | (((val) * ((mask) & ~((mask) << 1))) & (mask)))
#define ESP_RISCV_TARGET_BP_NUM         8
#define ESP_RISCV_TARGET_WP_NUM         8

struct esp_riscv_common {
	/* should be first, will be accessed by riscv generic code */
	riscv_info_t riscv;
	struct esp_common esp;
	struct esp_riscv_apptrace_info apptrace;
	struct esp_semihost_data semihost;
	struct esp_semihost_ops *semi_ops;
	target_addr_t target_bp_addr[ESP_RISCV_TARGET_BP_NUM];
	target_addr_t target_wp_addr[ESP_RISCV_TARGET_WP_NUM];
};

static inline struct esp_riscv_common *target_to_esp_riscv(const struct target *target)
{
	return target->arch_info;
}

static inline int esp_riscv_init_arch_info(struct command_context *cmd_ctx, struct target *target,
	struct esp_riscv_common *esp_riscv, int (*on_reset)(struct target *),
	const struct esp_flash_breakpoint_ops *flash_brps_ops,
	const struct esp_semihost_ops *semi_ops)
{
	esp_riscv->riscv.on_reset = on_reset;

	int ret = esp_common_init(&esp_riscv->esp, flash_brps_ops, &riscv_algo_hw);
	if (ret != ERROR_OK)
		return ret;

	esp_riscv->apptrace.hw = &esp_riscv_apptrace_hw;
	esp_riscv->semi_ops = (struct esp_semihost_ops *)semi_ops;

	return ERROR_OK;
}

int esp_riscv_semihosting(struct target *target);
int esp_riscv_breakpoint_add(struct target *target, struct breakpoint *breakpoint);
int esp_riscv_breakpoint_remove(struct target *target, struct breakpoint *breakpoint);
int esp_riscv_handle_target_event(struct target *target, enum target_event event,
	void *priv);
int esp_riscv_start_algorithm(struct target *target,
	int num_mem_params, struct mem_param *mem_params,
	int num_reg_params, struct reg_param *reg_params,
	target_addr_t entry_point, target_addr_t exit_point,
	void *arch_info);
int esp_riscv_wait_algorithm(struct target *target,
	int num_mem_params, struct mem_param *mem_params,
	int num_reg_params, struct reg_param *reg_params,
	target_addr_t exit_point, int timeout_ms,
	void *arch_info);
int esp_riscv_run_algorithm(struct target *target, int num_mem_params,
	struct mem_param *mem_params, int num_reg_params,
	struct reg_param *reg_params, target_addr_t entry_point,
	target_addr_t exit_point, int timeout_ms, void *arch_info);
int esp_riscv_read_memory(struct target *target, target_addr_t address,
	uint32_t size, uint32_t count, uint8_t *buffer);
int esp_riscv_write_memory(struct target *target, target_addr_t address,
	uint32_t size, uint32_t count, const uint8_t *buffer);
int esp_riscv_poll(struct target *target);
int esp_riscv_halt(struct target *target);
int esp_riscv_resume(struct target *target, int current, target_addr_t address,
	int handle_breakpoints, int debug_execution);
int esp_riscv_step(
	struct target *target,
	int current,
	target_addr_t address,
	int handle_breakpoints);
int esp_riscv_assert_reset(struct target *target);
int esp_riscv_deassert_reset(struct target *target);
int esp_riscv_checksum_memory(struct target *target,
	target_addr_t address, uint32_t count,
	uint32_t *checksum);
int esp_riscv_get_gdb_reg_list_noread(struct target *target,
	struct reg **reg_list[], int *reg_list_size,
	enum target_register_class reg_class);
int esp_riscv_get_gdb_reg_list(struct target *target,
	struct reg **reg_list[], int *reg_list_size,
	enum target_register_class reg_class);
const char *esp_riscv_get_gdb_arch(struct target *target);
int esp_riscv_arch_state(struct target *target);
int esp_riscv_add_watchpoint(struct target *target, struct watchpoint *watchpoint);
int esp_riscv_remove_watchpoint(struct target *target,
	struct watchpoint *watchpoint);
int esp_riscv_hit_watchpoint(struct target *target, struct watchpoint **hit_watchpoint);
unsigned int esp_riscv_address_bits(struct target *target);
bool esp_riscv_core_is_halted(struct target *target);
int esp_riscv_core_halt(struct target *target);
int esp_riscv_core_resume(struct target *target);
int esp_riscv_core_ebreaks_enable(struct target *target);

extern const struct command_registration esp_riscv_command_handlers[];

#endif	/* OPENOCD_TARGET_ESP_RISCV_H */
