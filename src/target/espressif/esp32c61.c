// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   ESP32-C61 target for OpenOCD                                           *
 *   Copyright (C) 2024 Espressif Systems Ltd.                             *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <helper/command.h>
#include <helper/bits.h>
#include <target/target.h>
#include <target/target_type.h>
#include <target/register.h>
#include <target/semihosting_common.h>
#include <target/riscv/debug_defines.h>

#include "esp_semihosting.h"
#include "esp_riscv_apptrace.h"
#include "esp_riscv.h"

/* max supported hw breakpoint and watchpoint count */
#define ESP32C61_BP_NUM                          3
#define ESP32C61_WP_NUM                          3

/* ASSIST_DEBUG registers */
#define ESP32C61_ASSIST_DEBUG_CPU0_MON_REG       0xFFFFFFFF//0x600C2000

#define ESP32C61_DRAM_LOW    0x40800000
#define ESP32C61_DRAM_HIGH   0x40860000

static bool esp32c61_is_idram_address(target_addr_t addr)
{
	return addr >= ESP32C61_DRAM_LOW && addr < ESP32C61_DRAM_HIGH;
}

static const struct esp_semihost_ops esp32c61_semihost_ops = {
	.prepare = NULL,
	.post_reset = esp_semihosting_post_reset
};

static const struct esp_flash_breakpoint_ops esp32c61_flash_brp_ops = {
	.breakpoint_prepare = esp_algo_flash_breakpoint_prepare,
	.breakpoint_add = esp_algo_flash_breakpoint_add,
	.breakpoint_remove = esp_algo_flash_breakpoint_remove,
	.breakpoint_lazy_process = true,
};

static const char *esp32c61_csrs[] = {
	"mideleg", "medeleg", "mie", "mip",
};

static int esp32c61_target_create(struct target *target, Jim_Interp *interp)
{
	struct esp_riscv_common *esp_riscv = calloc(1, sizeof(*esp_riscv));
	if (!esp_riscv)
		return ERROR_FAIL;

	target->arch_info = esp_riscv;

	esp_riscv->assist_debug_cpu0_mon_reg = ESP32C61_ASSIST_DEBUG_CPU0_MON_REG;
	esp_riscv->assist_debug_cpu_offset = 0;

	esp_riscv->max_bp_num = ESP32C61_BP_NUM;
	esp_riscv->max_wp_num = ESP32C61_WP_NUM;

	esp_riscv->rtccntl_reset_state_reg = 0;//ESP32C61_RTCCNTL_RESET_STATE_REG;
	esp_riscv->print_reset_reason = NULL;//&esp32c61_print_reset_reason;
	esp_riscv->existent_csrs = esp32c61_csrs;
	esp_riscv->existent_csr_size = ARRAY_SIZE(esp32c61_csrs);
	esp_riscv->is_dram_address = esp32c61_is_idram_address;
	esp_riscv->is_iram_address = esp32c61_is_idram_address;

	if (esp_riscv_alloc_trigger_addr(target) != ERROR_OK)
		return ERROR_FAIL;

	riscv_info_init(target, &esp_riscv->riscv);

	return ERROR_OK;
}

static int esp32c61_init_target(struct command_context *cmd_ctx,
	struct target *target)
{
	int ret = riscv_target.init_target(cmd_ctx, target);
	if (ret != ERROR_OK)
		return ret;

	target->semihosting->user_command_extension = esp_semihosting_common;

	struct esp_riscv_common *esp_riscv = target_to_esp_riscv(target);

	ret = esp_riscv_init_arch_info(target,
		esp_riscv,
		&esp32c61_flash_brp_ops,
		&esp32c61_semihost_ops);
	if (ret != ERROR_OK)
		return ret;

	return ERROR_OK;
}

static const struct command_registration esp32c61_command_handlers[] = {
	{
		.usage = "",
		.chain = riscv_command_handlers,
	},
	{
		.name = "esp",
		.usage = "",
		.chain = esp_riscv_command_handlers,
	},
	{
		.name = "esp",
		.usage = "",
		.chain = esp32_apptrace_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

struct target_type esp32c61_target = {
	.name = "esp32c61",

	.target_create = esp32c61_target_create,
	.init_target = esp32c61_init_target,
	.deinit_target = esp_riscv_deinit_target,
	.examine = esp_riscv_examine,

	/* poll current target status */
	.poll = esp_riscv_poll,

	.halt = riscv_halt,
	.resume = esp_riscv_resume,
	.step = riscv_openocd_step,

	.assert_reset = riscv_assert_reset,
	.deassert_reset = riscv_deassert_reset,

	.read_memory = esp_riscv_read_memory,
	.write_memory = esp_riscv_write_memory,

	.checksum_memory = riscv_checksum_memory,

	.get_gdb_arch = riscv_get_gdb_arch,
	.get_gdb_reg_list = riscv_get_gdb_reg_list,
	.get_gdb_reg_list_noread = riscv_get_gdb_reg_list_noread,

	.add_breakpoint = esp_riscv_breakpoint_add,
	.remove_breakpoint = esp_riscv_breakpoint_remove,

	.add_watchpoint = riscv_add_watchpoint,
	.remove_watchpoint = riscv_remove_watchpoint,
	.hit_watchpoint = esp_riscv_hit_watchpoint,

	.arch_state = riscv_arch_state,

	.run_algorithm = esp_riscv_run_algorithm,
	.start_algorithm = esp_riscv_start_algorithm,
	.wait_algorithm = esp_riscv_wait_algorithm,

	.commands = esp32c61_command_handlers,

	.address_bits = riscv_xlen_nonconst,
};
