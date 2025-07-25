// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   ESP32-C5 target for OpenOCD                                           *
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

#define ESP32C5_LP_CLKRST_BASE                  0x600B0400
#define ESP32C5_LP_CLKRST_RESET_CAUSE_REG       (ESP32C5_LP_CLKRST_BASE + 0x10)
#define ESP32C5_RTCCNTL_RESET_STATE_REG         (ESP32C5_LP_CLKRST_RESET_CAUSE_REG)

#define ESP32C5_RTCCNTL_RESET_CAUSE_MASK        (BIT(5) - 1)
#define ESP32C5_RESET_CAUSE(reg_val)            ((reg_val) & ESP32C5_RTCCNTL_RESET_CAUSE_MASK)

/* max supported hw breakpoint and watchpoint count */
#define ESP32C5_BP_NUM                          3
#define ESP32C5_WP_NUM                          3

/* ASSIST_DEBUG registers */
#define ESP32C5_ASSIST_DEBUG_CPU0_MON_REG       0x600C2000

#define ESP32C5_IROM_MASK_LOW                   0x40000000
#define ESP32C5_IROM_MASK_HIGH                  0x40050000
#define ESP32C5_DRAM_LOW                        0x40800000
#define ESP32C5_DRAM_HIGH                       0x40860000

enum esp32c5_reset_reason {
	ESP32C5_CHIP_POWER_ON_RESET   = 0x01, /* Power on reset */
	ESP32C5_CHIP_BROWN_OUT_RESET  = 0x01, /* VDD voltage is not stable and resets the chip */
	ESP32C5_CORE_SW_RESET         = 0x03, /* Software resets the digital core (hp system) by LP_AON_HPSYS_SW_RESET */
	ESP32C5_CORE_DEEP_SLEEP_RESET = 0x05, /* Deep sleep reset the digital core (hp system) */
	ESP32C5_CORE_MWDT0_RESET      = 0x07, /* Main watch dog 0 resets digital core (hp system) */
	ESP32C5_CORE_MWDT1_RESET      = 0x08, /* Main watch dog 1 resets digital core (hp system) */
	ESP32C5_CORE_RTC_WDT_RESET    = 0x09, /* RTC watch dog resets digital core (hp system) */
	ESP32C5_CPU0_MWDT0_RESET      = 0x0B, /* Main watch dog 0 resets CPU 0 */
	ESP32C5_CPU0_SW_RESET         = 0x0C, /* Software resets CPU 0 by LP_AON_CPU_CORE0_SW_RESET */
	ESP32C5_CPU0_RTC_WDT_RESET    = 0x0D, /* RTC watch dog resets CPU 0 */
	ESP32C5_SYS_BROWN_OUT_RESET   = 0x0F, /* VDD voltage is not stable and resets the digital core */
	ESP32C5_SYS_RTC_WDT_RESET     = 0x10, /* RTC watch dog resets digital core and rtc module */
	ESP32C5_CPU0_MWDT1_RESET      = 0x11, /* Main watch dog 1 resets CPU 0 */
	ESP32C5_SYS_SUPER_WDT_RESET   = 0x12, /* Super watch dog resets the digital core and rtc module */
	ESP32C5_CORE_EFUSE_CRC_RESET  = 0x14, /* eFuse CRC error resets the digital core (hp system) */
	ESP32C5_CORE_USB_UART_RESET   = 0x15, /* USB UART resets the digital core (hp system) */
	ESP32C5_CORE_USB_JTAG_RESET   = 0x16, /* USB JTAG resets the digital core (hp system) */
	ESP32C5_CPU0_JTAG_RESET       = 0x18, /* JTAG resets the CPU 0 */
	ESP32C5_CORE_PWR_GLITCH_RESET = 0x19, /* Glitch on power resets the digital core and rtc module */
	ESP32C5_CPU0_LOCKUP_RESET     = 0x1A, /* Triggered when the CPU enters lockup
											 (exception inside the exception handler would cause this) */
};

static const char *esp32c5_get_reset_reason(uint32_t reset_reason_reg_val)
{
	switch (ESP32C5_RESET_CAUSE(reset_reason_reg_val)) {
	case ESP32C5_CHIP_POWER_ON_RESET:
		/* case ESP32C5_CHIP_BROWN_OUT_RESET: */
		return "Chip reset";
	case ESP32C5_CORE_SW_RESET:
		return "Software core reset";
	case ESP32C5_CORE_DEEP_SLEEP_RESET:
		return "Deep-sleep core reset";
	case ESP32C5_CORE_MWDT0_RESET:
		return "Main WDT0 core reset";
	case ESP32C5_CORE_MWDT1_RESET:
		return "Main WDT1 core reset";
	case ESP32C5_CORE_RTC_WDT_RESET:
		return "RTC WDT core reset";
	case ESP32C5_CPU0_MWDT0_RESET:
		return "Main WDT0 CPU reset";
	case ESP32C5_CPU0_SW_RESET:
		return "Software CPU reset";
	case ESP32C5_CPU0_RTC_WDT_RESET:
		return "RTC WDT CPU reset";
	case ESP32C5_SYS_BROWN_OUT_RESET:
		return "Brown-out core reset";
	case ESP32C5_SYS_RTC_WDT_RESET:
		return "RTC WDT core and rtc reset";
	case ESP32C5_CPU0_MWDT1_RESET:
		return "Main WDT1 CPU reset";
	case ESP32C5_SYS_SUPER_WDT_RESET:
		return "Super Watchdog core and rtc";
	case ESP32C5_CORE_EFUSE_CRC_RESET:
		return "eFuse CRC error core reset";
	case ESP32C5_CORE_USB_UART_RESET:
		return "USB (UART) core reset";
	case ESP32C5_CORE_USB_JTAG_RESET:
		return "USB (JTAG) core reset";
	case ESP32C5_CPU0_JTAG_RESET:
		return "JTAG CPU reset";
	case ESP32C5_CORE_PWR_GLITCH_RESET:
		return "Power glitch core reset";
	case ESP32C5_CPU0_LOCKUP_RESET:
		return "CPU lockup reset";
	}
	return "Unknown reset cause";
}

static void esp32c5_print_reset_reason(struct target *target, uint32_t reset_reason_reg_val)
{
	LOG_TARGET_INFO(target, "Reset cause (%ld) - (%s)",
		ESP32C5_RESET_CAUSE(reset_reason_reg_val),
		esp32c5_get_reset_reason(reset_reason_reg_val));
}

static bool esp32c5_is_idram_address(target_addr_t addr)
{
	return addr >= ESP32C5_DRAM_LOW && addr < ESP32C5_DRAM_HIGH;
}

static const struct esp_semihost_ops esp32c5_semihost_ops = {
	.prepare = NULL,
	.post_reset = esp_semihosting_post_reset
};

static const struct esp_flash_breakpoint_ops esp32c5_flash_brp_ops = {
	.breakpoint_prepare = esp_algo_flash_breakpoint_prepare,
	.breakpoint_add = esp_algo_flash_breakpoint_add,
	.breakpoint_remove = esp_algo_flash_breakpoint_remove,
};

static const char *esp32c5_csrs[] = {
	"mie", "mip", "jvt", "mtvt", "mcontext", "tdata3",
	"mnxti", "mscratchcsw", "mscratchcswl", "utvt", "unxti",
	"mcycle", "mcycleh", "minstret", "minstreth",
	"mhpmevent8", "mhpmevent9", "mhpmevent13",
	"mhpmcounter8", "mhpmcounter9", "mhpmcounter13", "mhpmcounter8h", "mhpmcounter9h", "mhpmcounter13h",
	"mcounteren", "mcountinhibit",
	/* custom exposed CSRs will start with 'csr_' prefix*/
	"csr_ustatus", "csr_utvec", "csr_uepc", "csr_ucause",
	"csr_gpio_oen_user", "csr_gpio_in_user", "csr_gpio_out_user",
	"csr_pma_cfg0", "csr_pma_cfg1", "csr_pma_cfg2", "csr_pma_cfg3", "csr_pma_cfg4", "csr_pma_cfg5",
	"csr_pma_cfg6", "csr_pma_cfg7", "csr_pma_cfg8", "csr_pma_cfg9", "csr_pma_cfg10", "csr_pma_cfg11",
	"csr_pma_cfg12", "csr_pma_cfg13", "csr_pma_cfg14", "csr_pma_cfg15", "csr_pma_addr0", "csr_pma_addr1",
	"csr_pma_addr2", "csr_pma_addr3", "csr_pma_addr4", "csr_pma_addr5", "csr_pma_addr6", "csr_pma_addr7",
	"csr_pma_addr8", "csr_pma_addr9", "csr_pma_addr10", "csr_pma_addr11", "csr_pma_addr12", "csr_pma_addr13",
	"csr_pma_addr14", "csr_pma_addr15", "csr_mxstatus", "csr_mhcr", "csr_mhint", "csr_mexstatus",
	"csr_mclicbase", "csr_mraddr", "csr_mintthresh", "csr_uscratch",  "csr_uintthresh", "csr_uclicbase",
};

static const char *esp32c5_ro_csrs[] = {
	/* read-only CSRs, cannot be save/restored as the write would fail */
	/* custom exposed CSRs will start with 'csr_' prefix*/
	"csr_mintstatus", "csr_mcpuid", "csr_uintstatus",
};

static const char *esp32c5_user_counter_csrs[] = {
	/* user counters cannot be accessed in user mode unless corresponding mcounteren bit is set */
	"cycle", "time", "instreth", "cycleh", "instret", "timeh",
	"hpmcounter8", "hpmcounter9", "hpmcounter13", "hpmcounter8h", "hpmcounter9h", "hpmcounter13h",
};

static struct esp_riscv_reg_class esp32c5_registers[] = {
	{ esp32c5_csrs, ARRAY_SIZE(esp32c5_csrs), true, NULL },
	{ esp32c5_ro_csrs, ARRAY_SIZE(esp32c5_ro_csrs), false, NULL },
	{ esp32c5_user_counter_csrs, ARRAY_SIZE(esp32c5_user_counter_csrs), false, &esp_riscv_user_counter_type },
};

static int esp32c5_target_create(struct target *target)
{
	struct esp_riscv_common *esp_riscv = calloc(1, sizeof(*esp_riscv));
	if (!esp_riscv)
		return ERROR_FAIL;

	target->arch_info = esp_riscv;

	esp_riscv->assist_debug_cpu0_mon_reg = ESP32C5_ASSIST_DEBUG_CPU0_MON_REG;
	esp_riscv->assist_debug_cpu_offset = 0;

	esp_riscv->max_bp_num = ESP32C5_BP_NUM;
	esp_riscv->max_wp_num = ESP32C5_WP_NUM;

	esp_riscv->rtccntl_reset_state_reg = ESP32C5_RTCCNTL_RESET_STATE_REG;
	esp_riscv->print_reset_reason = &esp32c5_print_reset_reason;
	esp_riscv->chip_specific_registers = esp32c5_registers;
	esp_riscv->chip_specific_registers_size = ARRAY_SIZE(esp32c5_registers);
	esp_riscv->is_dram_address = esp32c5_is_idram_address;
	esp_riscv->is_iram_address = esp32c5_is_idram_address;

	if (esp_riscv_alloc_trigger_addr(target) != ERROR_OK)
		return ERROR_FAIL;

	riscv_info_init(target, &esp_riscv->riscv);

	return ERROR_OK;
}

static int esp32c5_init_target(struct command_context *cmd_ctx,
	struct target *target)
{
	int ret = riscv_target.init_target(cmd_ctx, target);
	if (ret != ERROR_OK)
		return ret;

	target->semihosting->user_command_extension = esp_semihosting_common;

	struct esp_riscv_common *esp_riscv = target_to_esp_riscv(target);

	esp_riscv->riscv.halt_delay_us = 10000;

	ret = esp_riscv_init_arch_info(target,
		esp_riscv,
		&esp32c5_flash_brp_ops,
		&esp32c5_semihost_ops);
	if (ret != ERROR_OK)
		return ret;

	return ERROR_OK;
}

static int esp32c5_get_gdb_memory_map(struct target *target, struct target_memory_map *memory_map)
{
	struct target_memory_region region = { 0 };

	region.type = MEMORY_TYPE_ROM;
	region.start = ESP32C5_IROM_MASK_LOW;
	region.length = ESP32C5_IROM_MASK_HIGH - ESP32C5_IROM_MASK_LOW;
	return target_add_memory_region(memory_map, &region);
}

static const struct command_registration esp32c5_command_handlers[] = {
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

struct target_type esp32c5_target = {
	.name = "esp32c5",

	.target_create = esp32c5_target_create,
	.init_target = esp32c5_init_target,
	.deinit_target = esp_riscv_deinit_target,
	.examine = esp_riscv_examine,

	/* poll current target status */
	.poll = esp_riscv_poll,

	.halt = riscv_halt,
	.resume = esp_riscv_resume,
	.step = riscv_openocd_step,

	.assert_reset = esp_riscv_assert_reset,
	.deassert_reset = riscv_deassert_reset,

	.read_memory = esp_riscv_read_memory,
	.write_memory = esp_riscv_write_memory,

	.checksum_memory = riscv_checksum_memory,

	.get_gdb_arch = riscv_get_gdb_arch,
	.get_gdb_reg_list = esp_riscv_get_gdb_reg_list,
	.get_gdb_reg_list_noread = riscv_get_gdb_reg_list_noread,
	.get_gdb_memory_map = esp32c5_get_gdb_memory_map,

	.add_breakpoint = esp_riscv_breakpoint_add,
	.remove_breakpoint = esp_riscv_breakpoint_remove,

	.add_watchpoint = riscv_add_watchpoint,
	.remove_watchpoint = riscv_remove_watchpoint,
	.hit_watchpoint = esp_riscv_hit_watchpoint,

	.arch_state = riscv_arch_state,

	.run_algorithm = esp_riscv_run_algorithm,
	.start_algorithm = esp_riscv_start_algorithm,
	.wait_algorithm = esp_riscv_wait_algorithm,

	.commands = esp32c5_command_handlers,

	.address_bits = riscv_xlen_nonconst,
};
