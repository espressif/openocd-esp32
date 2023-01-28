/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   ESP RISCV common definitions for OpenOCD                              *
 *   Copyright (C) 2021 Espressif Systems Ltd.                             *
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

struct esp_riscv_common {
	/* should be first, will be accessed by riscv generic code */
	struct riscv_info riscv;
	struct esp_common esp;
	struct esp_riscv_apptrace_info apptrace;
	struct esp_semihost_data semihost;
	struct esp_semihost_ops *semi_ops;
	target_addr_t *target_bp_addr;
	target_addr_t *target_wp_addr;
	uint8_t max_bp_num;
	uint8_t max_wp_num;
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

	INIT_LIST_HEAD(&esp_riscv->semihost.dir_map_list);

	int ret = esp_common_init(&esp_riscv->esp, flash_brps_ops, &riscv_algo_hw);
	if (ret != ERROR_OK)
		return ret;

	esp_riscv->apptrace.hw = &esp_riscv_apptrace_hw;
	esp_riscv->semi_ops = (struct esp_semihost_ops *)semi_ops;

	return ERROR_OK;
}

int esp_riscv_alloc_trigger_addr(struct target *target);
int esp_riscv_semihosting(struct target *target);
int esp_riscv_breakpoint_add(struct target *target, struct breakpoint *breakpoint);
int esp_riscv_breakpoint_remove(struct target *target, struct breakpoint *breakpoint);
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
void esp_riscv_deinit_target(struct target *target);

extern const struct command_registration esp_riscv_command_handlers[];

#endif	/* OPENOCD_TARGET_ESP_RISCV_H */
