// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   ESP32-P4 target for OpenOCD                                           *
 *   Copyright (C) 2023 Espressif Systems Ltd.                             *
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

/* boot mode */
#define ESP32P4_GPIO_BASE						(0x500C0000 + 0x20000)
#define ESP32P4_GPIO_STRAP_REG_OFF              0x0038
#define ESP32P4_GPIO_STRAP_REG                  (ESP32P4_GPIO_BASE + ESP32P4_GPIO_STRAP_REG_OFF)

/* reset cause */
#define ESP32P4_LP_AON_BASE						0x50110000
#define ESP32P4_LP_CLKRST_RESET_CAUSE_REG       (ESP32P4_LP_AON_BASE + 0x1000 + 0x10)
#define ESP32P4_LP_CLKRST_RESET_CAUSE_MASK      (BIT(6) - 1) /* 0x3F */
#define ESP32P4_RESET_CAUSE(reg_val)            ((reg_val) & ESP32P4_LP_CLKRST_RESET_CAUSE_MASK)

/* max supported hw breakpoint and watchpoint count */
#define ESP32P4_BP_NUM                          4
#define ESP32P4_WP_NUM                          4

/* components/soc/esp32p4/include/soc/reset_reasons.h */
enum esp32p4_reset_reason {
	ESP32P4_CHIP_POWER_ON_RESET   = 0x01,	/* Power on reset */
	ESP32P4_CORE_SW_RESET         = 0x03,	/* Software resets the digital core */
	ESP32P4_SYS_PMU_PWR_DOWN_RESET = 0x05,	/* PMU HP power down system reset */
	ESP32P4_SYS_HP_WDT_RESET      = 0x07,	/* HP WDT resets system */
	ESP32P4_SYS_LP_WDT_RESET      = 0x09,	/* LP WDT resets system */
	ESP32P4_CORE_HP_WDT_RESET     = 0x0B,	/* HP WDT resets digital core */
	ESP32P4_CPU0_SW_RESET         = 0x0C,	/* Software resets CPU 0 */
	ESP32P4_CORE_LP_WDT_RESET     = 0x0D,	/* LP WDT resets digital core */
	ESP32P4_SYS_BROWN_OUT_RESET   = 0x0F,	/* VDD voltage is not stable and resets the digital core */
	ESP32P4_CHIP_LP_WDT_RESET     = 0x10,	/* LP WDT resets chip */
	ESP32P4_SYS_SUPER_WDT_RESET   = 0x12,	/* Super watch dog resets the digital core and rtc module */
	ESP32P4_SYS_CLK_GLITCH_RESET  = 0x13,	/* Glitch on clock resets the digital core and rtc module */
	ESP32P4_CORE_EFUSE_CRC_RESET  = 0x14,	/* eFuse CRC error resets the digital core */
	ESP32P4_CORE_USB_JTAG_RESET   = 0x16,	/* USB JTAG resets the digital core */
	ESP32P4_CORE_USB_UART_RESET   = 0x17,	/* UART resets the digital core */
	ESP32P4_CPU_JTAG_RESET        = 0x18,	/* JTAG resets the digital core */
	ESP32P4_CPU_LOCKUP_RESET      = 0x1A,	/* Cpu lockup resets the chip */
};

static const char *esp32p4_get_reset_reason(int reset_number)
{
	switch (ESP32P4_RESET_CAUSE(reset_number)) {
	case ESP32P4_CHIP_POWER_ON_RESET:
		return "Power on reset";
	case ESP32P4_CORE_SW_RESET:
		return "Software core reset";
	case ESP32P4_SYS_PMU_PWR_DOWN_RESET:
		return "PMU HP power down system reset";
	case ESP32P4_SYS_HP_WDT_RESET:
		return "HP WDT resets system";
	case ESP32P4_SYS_LP_WDT_RESET:
		return "LP WDT resets system";
	case ESP32P4_CORE_HP_WDT_RESET:
		return "HP WDT resets digital core";
	case ESP32P4_CPU0_SW_RESET:
		return "Software CPU reset";
	case ESP32P4_CORE_LP_WDT_RESET:
		return "LP WDT resets digital core";
	case ESP32P4_SYS_BROWN_OUT_RESET:
		return "Brown-out core reset";
	case ESP32P4_CHIP_LP_WDT_RESET:
		return "LP WDT resets chip";
	case ESP32P4_SYS_SUPER_WDT_RESET:
		return "Super Watchdog core and rtc";
	case ESP32P4_SYS_CLK_GLITCH_RESET:
		return "Glitch on clock reset";
	case ESP32P4_CORE_EFUSE_CRC_RESET:
		return "eFuse CRC error core reset";
	case ESP32P4_CORE_USB_JTAG_RESET:
		return "USB (JTAG) core reset";
	case ESP32P4_CORE_USB_UART_RESET:
		return "UART resets the digital core";
	case ESP32P4_CPU_JTAG_RESET:
		return "JTAG CPU reset";
	case ESP32P4_CPU_LOCKUP_RESET:
		return "CPU Lockup reset";
	}
	return "Unknown reset cause";
}

static const struct esp_semihost_ops esp32p4_semihost_ops = {
	.prepare = NULL,
	.post_reset = esp_semihosting_post_reset
};

static const struct esp_flash_breakpoint_ops esp32p4_flash_brp_ops = {
	.breakpoint_add = esp_algo_flash_breakpoint_add,
	.breakpoint_remove = esp_algo_flash_breakpoint_remove
};

/* TODO: update this table for the P4 */
static const char *esp32p4_existent_regs[] = {
	"zero", "ra", "sp", "gp", "tp", "t0", "t1", "t2", "t3", "t4", "t5", "t6",
	"fp", "pc", "mstatus", "misa", "mtvec", "mscratch", "mepc", "mcause", "mtval", "priv",
	"s1", "s2", "s3", "s4", "s5", "s6", "s7", "s8", "s9", "s10", "s11",
	"a0", "a1", "a2", "a3", "a4", "a5", "a6", "a7",
	"pmpcfg0", "pmpcfg1", "pmpcfg2", "pmpcfg3",
	"pmpaddr0", "pmpaddr1", "pmpaddr2", "pmpaddr3", "pmpaddr4", "pmpaddr5", "pmpaddr6", "pmpaddr7",
	"pmpaddr8", "pmpaddr9", "pmpaddr10", "pmpaddr11", "pmpaddr12", "pmpaddr13", "pmpaddr14", "pmpaddr15",
	"tselect", "tdata1", "tdata2", "tcontrol", "dcsr", "dpc", "dscratch0", "dscratch1", "hpmcounter16",
	/* custom exposed CSRs will start with 'csr_' prefix*/
	/* TODO */
};

static int esp32p4_target_create(struct target *target, Jim_Interp *interp)
{
	struct esp_riscv_common *esp_riscv = calloc(1, sizeof(*esp_riscv));
	if (!esp_riscv)
		return ERROR_FAIL;

	target->arch_info = esp_riscv;

	// esp32p4->esp_riscv.assist_debug_cpu0_mon_reg = ESP32P4_ASSIST_DEBUG_CPU0_MON_REG;
	// esp32p4->esp_riscv.assist_debug_cpu_offset = 0;

	esp_riscv->max_bp_num = ESP32P4_BP_NUM;
	esp_riscv->max_wp_num = ESP32P4_WP_NUM;

	esp_riscv->gpio_strap_reg = ESP32P4_GPIO_STRAP_REG;
	esp_riscv->rtccntl_reset_state_reg = ESP32P4_LP_CLKRST_RESET_CAUSE_REG;
	esp_riscv->reset_cause_mask = ESP32P4_LP_CLKRST_RESET_CAUSE_MASK;
	esp_riscv->get_reset_reason = &esp32p4_get_reset_reason;
	esp_riscv->is_flash_boot = &esp_is_flash_boot;
	esp_riscv->existent_regs = esp32p4_existent_regs;
	esp_riscv->existent_regs_size = ARRAY_SIZE(esp32p4_existent_regs);

	if (esp_riscv_alloc_trigger_addr(target) != ERROR_OK)
		return ERROR_FAIL;

	riscv_info_init(target, &esp_riscv->riscv);

	return ERROR_OK;
}

static int esp32p4_init_target(struct command_context *cmd_ctx,
	struct target *target)
{
	int ret = riscv_target.init_target(cmd_ctx, target);
	if (ret != ERROR_OK)
		return ret;

	target->semihosting->user_command_extension = esp_semihosting_common;

	struct esp_riscv_common *esp_riscv = target_to_esp_riscv(target);

	ret = esp_riscv_init_arch_info(cmd_ctx,
		target,
		esp_riscv,
		&esp32p4_flash_brp_ops,
		&esp32p4_semihost_ops);
	if (ret != ERROR_OK)
		return ret;

	return ERROR_OK;
}

static const struct command_registration esp32p4_command_handlers[] = {
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

struct target_type esp32p4_target = {
	.name = "esp32p4",

	.target_create = esp32p4_target_create,
	.init_target = esp32p4_init_target,
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

	.commands = esp32p4_command_handlers,

	.address_bits = riscv_xlen_nonconst,
};
