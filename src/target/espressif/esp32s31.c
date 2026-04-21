// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   ESP32-S31 target for OpenOCD                                    *
 *   Copyright (C) 2026 Espressif Systems Ltd.                       *
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
#include <target/riscv/riscv.h>

#include "esp_semihosting.h"
#include "esp_riscv_apptrace.h"
#include "esp_riscv.h"

#define ESP32S31_LP_AONCLKRST_BASE                    0x20701000
#define ESP32S31_LP_AONCLKRST_RESET_STATE_REG         (ESP32S31_LP_AONCLKRST_BASE + 0x30)

/* HPCORE0_RESET_CAUSE field is at bits [6:1] */
#define ESP32S31_LP_AONCLKRST_RESET_CAUSE_MASK        (0x3FU)
#define ESP32S31_RESET_CAUSE(reg_val)                 (((reg_val) >> 1) & ESP32S31_LP_AONCLKRST_RESET_CAUSE_MASK)

/* max supported hw breakpoint and watchpoint count */
#define ESP32S31_BP_NUM                               4
#define ESP32S31_WP_NUM                               4

/* ASSIST_DEBUG registers */
#define ESP32S31_ASSIST_DEBUG_CPU0_MON_REG            0x2D002000
#define ESP32S31_ASSIST_DEBUG_CPU_OFFSET              0x88

#define ESP32S31_IROM_MASK_LOW                        0x2F800000
#define ESP32S31_IROM_MASK_HIGH                       0x2F850000
#define ESP32S31_DRAM_LOW                             0x2F000000
#define ESP32S31_DRAM_HIGH                            0x2F080000
#define ESP32S31_IRAM_LOW                             0x2F000000
#define ESP32S31_IRAM_HIGH                            0x2F080000

#define ESP32S31_EFUSE_HW_REV_ADDR                    0x2071504c

/* PMA entry 14 covers the HP RAM region (0x2F000000..0x2F080000, 512 KB).
 * ROM's riscv_pma_init_cfg() programs it as NAPOT RW (no X); the X bit is only
 * added later by riscv_pma_flash_boot_cfg(). If the debugger halts the CPU
 * before that second step runs, the RAM entry still has X cleared and the
 * flasher stub (loaded into the work area at 0x2F000000) cannot be fetched,
 * producing mcause=1 (PMP Instruction access fault).
 *
 * NAPOT encoding of 0x2F000000..0x2F080000 (size 0x80000, power of 2):
 *   pmaaddr14 = (base | (size/2 - 1)) >> 2
 *             = (0x2F000000 | 0x3FFFF) >> 2
 *             = 0x0BC0FFFF
 *   pmacfg14  = PMA_NAPOT | PMA_EN | PMA_R | PMA_W | PMA_X
 *             = 0xC0000000 | BIT(0) | BIT(4) | BIT(3) | BIT(2)
 *             = 0xC000001D
 */
#define ESP32S31_PMA_ENTRY_NUM                        16
#define ESP32S31_PMA_ENTRY_RAM                        (ESP32S31_PMA_ENTRY_NUM - 2)
#define ESP32S31_CSR_PMACFG0                          0xBC0
#define ESP32S31_CSR_PMAADDR0                         0xBD0
#define ESP32S31_PMA_CFG_X                            BIT(2)
#define ESP32S31_PMA_RAM_NAPOT_ADDR                   0x0BC0FFFFUL
#define ESP32S31_PMA_RAM_NAPOT_CFG_RWX                0xC000001DUL

/* components/soc/esp32s31/include/soc/reset_reasons.h */
enum esp32s31_reset_reason {
	ESP32S31_CHIP_POWER_ON_RESET   = 0x01,  /* Power on reset */
	ESP32S31_CORE_SW_RESET         = 0x03,  /* Software reset */
	ESP32S31_CORE_DEEP_SLEEP_RESET = 0x05,  /* Deep sleep reset */
	ESP32S31_CPU_PMU_PWR_DOWN_RESET = 0x06,  /* PMU HP power down CPU reset */
	ESP32S31_CORE_MWDT0_RESET      = 0x07,  /* MWDT0 resets digital core */
	ESP32S31_CORE_MWDT1_RESET      = 0x08,  /* MWDT1 resets digital core */
	ESP32S31_CORE_RWDT_RESET       = 0x09,  /* RWDT resets digital core */
	ESP32S31_CPU_MWDT_RESET        = 0x0B,  /* MWDT CPU reset */
	ESP32S31_CPU_SW_RESET          = 0x0C,  /* Software CPU reset */
	ESP32S31_CPU_RWDT_RESET        = 0x0D,  /* RWDT CPU reset */
	ESP32S31_SYS_BROWN_OUT_RESET   = 0x0F,  /* Brown-out reset */
	ESP32S31_SYS_RWDT_RESET        = 0x10,  /* RWDT resets chip */
	ESP32S31_SYS_SUPER_WDT_RESET   = 0x12,  /* Super Watchdog reset */
	ESP32S31_CORE_PWR_GLITCH_RESET = 0x13,  /* Power glitch reset */
	ESP32S31_CORE_EFUSE_CRC_RESET  = 0x14,  /* eFuse CRC error reset */
	ESP32S31_CORE_USB_JTAG_RESET   = 0x16,  /* USB (JTAG) reset */
	ESP32S31_CORE_USB_UART_RESET   = 0x17,  /* USB (UART) reset */
	ESP32S31_CPU_JTAG_RESET        = 0x18,  /* JTAG CPU reset */
	ESP32S31_CPU_LOCKUP_RESET      = 0x1A,  /* CPU lockup reset */
};

static const char *esp32s31_get_reset_reason(uint32_t reset_reason_reg_val)
{
	switch (ESP32S31_RESET_CAUSE(reset_reason_reg_val)) {
	case ESP32S31_CHIP_POWER_ON_RESET:
		return "Power on reset";
	case ESP32S31_CORE_SW_RESET:
		return "Software reset";
	case ESP32S31_CORE_DEEP_SLEEP_RESET:
		/* case ESP32S31_CORE_PMU_PWR_DOWN_RESET: */
		return "Deep sleep reset";
	case ESP32S31_CPU_PMU_PWR_DOWN_RESET:
		return "PMU HP power down CPU reset";
	case ESP32S31_CORE_MWDT0_RESET:
		return "MWDT0 resets digital core";
	case ESP32S31_CORE_MWDT1_RESET:
		return "MWDT1 resets digital core";
	case ESP32S31_CORE_RWDT_RESET:
		return "RWDT resets digital core";
	case ESP32S31_CPU_MWDT_RESET:
		return "MWDT CPU reset";
	case ESP32S31_CPU_SW_RESET:
		/* case ESP32S31_CPU0_SW_RESET: */
		return "Software CPU reset";
	case ESP32S31_CPU_RWDT_RESET:
		return "RWDT CPU reset";
	case ESP32S31_SYS_BROWN_OUT_RESET:
		return "Brown-out reset";
	case ESP32S31_SYS_RWDT_RESET:
		return "RWDT resets chip";
	case ESP32S31_SYS_SUPER_WDT_RESET:
		return "Super Watchdog reset";
	case ESP32S31_CORE_PWR_GLITCH_RESET:
		return "Power glitch reset";
	case ESP32S31_CORE_EFUSE_CRC_RESET:
		return "eFuse CRC error reset";
	case ESP32S31_CORE_USB_JTAG_RESET:
		return "USB (JTAG) reset";
	case ESP32S31_CORE_USB_UART_RESET:
		return "USB (UART) reset";
	case ESP32S31_CPU_JTAG_RESET:
		return "JTAG CPU reset";
	case ESP32S31_CPU_LOCKUP_RESET:
		return "CPU lockup reset";
	}
	return "Unknown reset cause";
}

static void esp32s31_print_reset_reason(struct target *target, uint32_t reset_reason_reg_val)
{
	LOG_TARGET_INFO(target, "Reset cause (%u) - (%s)",
		ESP32S31_RESET_CAUSE(reset_reason_reg_val),
		esp32s31_get_reset_reason(reset_reason_reg_val));
}

static int esp32s31_read_hw_rev(struct target *target)
{
	static uint32_t hw_rev;

	if (hw_rev != 0) {
		target->hw_rev = hw_rev;
		return ERROR_OK;
	}

	int ret = target_read_u32(target, ESP32S31_EFUSE_HW_REV_ADDR, &hw_rev);
	if (ret != ERROR_OK) {
		LOG_TARGET_ERROR(target, "Failed to read HW rev (%d)", ret);
		return ret;
	}

	unsigned int major = (hw_rev >> 4) & 0x03;
	unsigned int minor = hw_rev & 0x0F;

	hw_rev = 100 * major + minor;
	target->hw_rev = hw_rev;
	LOG_TARGET_INFO(target, "Chip revision v%u.%u", major, minor);

	return ERROR_OK;
}

static int esp32s31_examine_end(struct target *target)
{
	esp32s31_read_hw_rev(target);
	return ERROR_OK;
}

static int esp32s31_pre_algorithm(struct target *target)
{
	enum gdb_regno cfg_regno = GDB_REGNO_CSR0 + ESP32S31_CSR_PMACFG0 + ESP32S31_PMA_ENTRY_RAM;
	enum gdb_regno addr_regno = GDB_REGNO_CSR0 + ESP32S31_CSR_PMAADDR0 + ESP32S31_PMA_ENTRY_RAM;

	riscv_reg_t cfg_val;
	int ret = riscv_reg_get(target, &cfg_val, cfg_regno);
	if (ret != ERROR_OK) {
		LOG_TARGET_ERROR(target, "Failed to read pma_cfg%u (%d)", ESP32S31_PMA_ENTRY_RAM, ret);
		return ret;
	}
	if (cfg_val & ESP32S31_PMA_CFG_X) {
		LOG_TARGET_DEBUG(target, "pma_cfg%u already grants X, skipping override", ESP32S31_PMA_ENTRY_RAM);
		return ERROR_OK;
	}

	ret = riscv_reg_set(target, addr_regno, ESP32S31_PMA_RAM_NAPOT_ADDR);
	if (ret != ERROR_OK) {
		LOG_TARGET_ERROR(target, "Failed to set pma_addr%u (%d)", ESP32S31_PMA_ENTRY_RAM, ret);
		return ret;
	}
	ret = riscv_reg_set(target, cfg_regno, ESP32S31_PMA_RAM_NAPOT_CFG_RWX);
	if (ret != ERROR_OK) {
		LOG_TARGET_ERROR(target, "Failed to set pma_cfg%u (%d)", ESP32S31_PMA_ENTRY_RAM, ret);
		return ret;
	}
	LOG_TARGET_DEBUG(target, "PMA entry %u forced to NAPOT RWX for RAM region", ESP32S31_PMA_ENTRY_RAM);
	return ERROR_OK;
}

static bool esp32s31_is_dram_address(target_addr_t addr)
{
	return addr >= ESP32S31_DRAM_LOW && addr < ESP32S31_DRAM_HIGH;
}

static bool esp32s31_is_iram_address(target_addr_t addr)
{
	return addr >= ESP32S31_IRAM_LOW && addr < ESP32S31_IRAM_HIGH;
}

static const struct esp_semihost_ops esp32s31_semihost_ops = {
	.prepare = NULL,
	.post_reset = esp_semihosting_post_reset
};

static const struct esp_flash_breakpoint_ops esp32s31_flash_brp_ops = {
	.breakpoint_prepare = esp_algo_flash_breakpoint_prepare,
	.breakpoint_add = esp_algo_flash_breakpoint_add,
	.breakpoint_remove = esp_algo_flash_breakpoint_remove,
};

/* TODO: add missing CSRs */
static const char *esp32s31_csrs[] = {
	"mie", "mcause", "mip", "mtvt", "mnxti",
	"mscratchcsw", "mscratchcswl",
	"mcycle", "minstret", "mcounteren", "mcountinhibit",
	"tdata3", "tinfo", "mcontext",
	"mclicbase", "mxstatus", "mhcr", "mhint", "mraddr", "mexstatus",
	"mnmicause", "mnmipc", "mcpuid", "cpu_testbus_ctrl", "pm_user",
	"gpio_oen_user", "gpio_in_user", "gpio_out_user",
	"pma_cfg0", "pma_cfg1", "pma_cfg2", "pma_cfg3", "pma_cfg4", "pma_cfg5",
	"pma_cfg6", "pma_cfg7", "pma_cfg8", "pma_cfg9", "pma_cfg10", "pma_cfg11",
	"pma_cfg12", "pma_cfg13", "pma_cfg14", "pma_cfg15", "pma_addr0", "pma_addr1",
	"pma_addr2", "pma_addr3", "pma_addr4", "pma_addr5", "pma_addr6", "pma_addr7",
	"pma_addr8", "pma_addr9", "pma_addr10", "pma_addr11", "pma_addr12", "pma_addr13",
	"pma_addr14", "pma_addr15",
};

static struct esp_riscv_reg_class esp32s31_registers[] = {
	{
		.reg_array = esp32s31_csrs,
		.reg_array_size = ARRAY_SIZE(esp32s31_csrs),
		.save_restore = true
	},
};

static int esp32s31_target_create(struct target *target)
{
	struct esp_riscv_common *esp_riscv = calloc(1, sizeof(*esp_riscv));
	if (!esp_riscv)
		return ERROR_FAIL;

	target->arch_info = esp_riscv;

	esp_riscv->assist_debug_cpu0_mon_reg = ESP32S31_ASSIST_DEBUG_CPU0_MON_REG;
	esp_riscv->assist_debug_cpu_offset = ESP32S31_ASSIST_DEBUG_CPU_OFFSET;

	esp_riscv->max_bp_num = ESP32S31_BP_NUM;
	esp_riscv->max_wp_num = ESP32S31_WP_NUM;

	esp_riscv->rtccntl_reset_state_reg = ESP32S31_LP_AONCLKRST_RESET_STATE_REG;
	esp_riscv->print_reset_reason = &esp32s31_print_reset_reason;
	esp_riscv->chip_specific_registers = esp32s31_registers;
	esp_riscv->chip_specific_registers_size = ARRAY_SIZE(esp32s31_registers);
	esp_riscv->is_dram_address = esp32s31_is_dram_address;
	esp_riscv->is_iram_address = esp32s31_is_iram_address;
	esp_riscv->examine_end = esp32s31_examine_end;
	esp_riscv->pre_algorithm = esp32s31_pre_algorithm;

	if (esp_riscv_alloc_trigger_addr(target) != ERROR_OK)
		return ERROR_FAIL;

	riscv_info_init(target, &esp_riscv->riscv);
	struct riscv_private_config *config = target->private_config;
	if (!config) {
		config = alloc_default_riscv_private_config();
		if (!config)
			return ERROR_FAIL;
		target->private_config = config;
	}

	return ERROR_OK;
}

static int esp32s31_init_target(struct command_context *cmd_ctx,
	struct target *target)
{
	int ret = riscv_target.init_target(cmd_ctx, target);
	if (ret != ERROR_OK)
		return ret;

	target->semihosting->user_command_extension = esp_semihosting_common;

	struct esp_riscv_common *esp_riscv = target_to_esp_riscv(target);

	ret = esp_riscv_init_arch_info(target,
		esp_riscv,
		&esp32s31_flash_brp_ops,
		&esp32s31_semihost_ops);
	if (ret != ERROR_OK)
		return ret;

	return ERROR_OK;
}

static int esp32s31_get_gdb_memory_map(struct target *target, struct target_memory_map *memory_map)
{
	struct target_memory_region region = { 0 };

	region.type = MEMORY_TYPE_ROM;
	region.start = ESP32S31_IROM_MASK_LOW;
	region.length = ESP32S31_IROM_MASK_HIGH - ESP32S31_IROM_MASK_LOW;
	return target_add_memory_region(memory_map, &region);
}

static const struct command_registration esp32s31_command_handlers[] = {
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

struct target_type esp32s31_target = {
	.name = "esp32s31",

	.target_create = esp32s31_target_create,
	.target_jim_configure = riscv_jim_configure,
	.init_target = esp32s31_init_target,
	.deinit_target = esp_riscv_deinit_target,
	.examine = esp_riscv_examine,

	/* poll current target status */
	.poll = esp_riscv_poll,

	.halt = riscv_halt,
	.resume = esp_riscv_resume,
	.step = esp_riscv_step,

	.assert_reset = esp_riscv_assert_reset,
	.deassert_reset = riscv_deassert_reset,

	.read_memory = esp_riscv_read_memory,
	.write_memory = esp_riscv_write_memory,

	.checksum_memory = riscv_checksum_memory,

	.get_gdb_arch = riscv_get_gdb_arch,
	.get_gdb_reg_list = riscv_get_gdb_reg_list,
	.get_gdb_reg_list_noread = riscv_get_gdb_reg_list_noread,
	.get_gdb_memory_map = esp32s31_get_gdb_memory_map,

	.add_breakpoint = esp_riscv_breakpoint_add,
	.remove_breakpoint = esp_riscv_breakpoint_remove,

	.add_watchpoint = riscv_add_watchpoint,
	.remove_watchpoint = riscv_remove_watchpoint,
	.hit_watchpoint = esp_riscv_hit_watchpoint,

	.arch_state = riscv_arch_state,

	.run_algorithm = esp_riscv_run_algorithm,
	.start_algorithm = esp_riscv_start_algorithm,
	.wait_algorithm = esp_riscv_wait_algorithm,

	.commands = esp32s31_command_handlers,

	.address_bits = riscv_xlen_nonconst,
};
