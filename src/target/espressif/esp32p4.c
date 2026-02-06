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
#include <helper/align.h>
#include <target/target.h>
#include <target/target_type.h>
#include <target/register.h>
#include <target/semihosting_common.h>
#include <target/riscv/debug_defines.h>
#include <target/riscv/program.h>

#include "esp_semihosting.h"
#include "esp_riscv_apptrace.h"
#include "esp_riscv.h"

/* reset cause */
#define ESP32P4_LP_AON_BASE                     0x50110000
#define ESP32P4_LP_CLKRST_RESET_CAUSE_REG       (ESP32P4_LP_AON_BASE + 0x1000 + 0x10)
#define ESP32P4_RESET_CAUSE_MASK                (BIT(6) - 1) /* 0x3F */
#define ESP32P4_LP_CORE_RESET_CAUSE_SHIFT       0
#define ESP32P4_HP_CORE0_RESET_CAUSE_SHIFT      7
#define ESP32P4_HP_CORE1_RESET_CAUSE_SHIFT      14
#define ESP32P4_RESET_CAUSE(reg_val, shift)     ((reg_val >> shift) & ESP32P4_RESET_CAUSE_MASK)

/* cache */
#define ESP32P4_CACHE_BASE                      (0x3FF00000 + 0x10000)
#define ESP32P4_CACHE_SYNC_CTRL_REG             (ESP32P4_CACHE_BASE + 0x98)
#define ESP32P4_CACHE_SYNC_MAP_REG              (ESP32P4_CACHE_BASE + 0x9C)
#define ESP32P4_CACHE_SYNC_ADDR_REG             (ESP32P4_CACHE_BASE + 0xA0)
#define ESP32P4_CACHE_SYNC_SIZE_REG             (ESP32P4_CACHE_BASE + 0xA4)

#define ESP32P4_CACHE_MAP_L1_ICACHE0            BIT(0)
#define ESP32P4_CACHE_MAP_L1_ICACHE1            BIT(1)
#define ESP32P4_CACHE_MAP_L1_DCACHE             BIT(4)
#define ESP32P4_CACHE_MAP_L2_CACHE              BIT(5)
#define ESP32P4_CACHE_SYNC_INVALIDATE           BIT(0)
#define ESP32P4_CACHE_SYNC_WRITEBACK            BIT(2)
#define ESP32P4_CACHE_SYNC_FLUSH                BIT(3) /* Writeback + invalidate */
#define ESP32P4_CACHE_SYNC_DONE                 BIT(4)

#define ESP32P4_CACHE_L1_LINE_SIZE              64

#define ESP32P4_CACHE_MAP_L1_ICACHE (ESP32P4_CACHE_MAP_L1_ICACHE0 | ESP32P4_CACHE_MAP_L1_ICACHE1)
#define ESP32P4_CACHE_MAP_L1_CACHE (ESP32P4_CACHE_MAP_L1_ICACHE | ESP32P4_CACHE_MAP_L1_DCACHE)
#define ESP32P4_CACHE_MAP_ALL (ESP32P4_CACHE_MAP_L1_CACHE | ESP32P4_CACHE_MAP_L2_CACHE)

#define ESP32P4_EXRAM_CACHEABLE_ADDR_LOW        0x48000000U
#define ESP32P4_EXRAM_CACHEABLE_ADDR_HIGH       0x4BFFFFFFU
#define ESP32P4_IRAM0_CACHEABLE_ADDR_LOW        0x4ff00000U
#define ESP32P4_IRAM0_CACHEABLE_ADDR_HIGH       0x4ffc0000U
#define ESP32P4_HPROM_CACHEABLE_ADDR_LOW        0x4fc00000U
#define ESP32P4_HPROM_CACHEABLE_ADDR_HIGH       0x4fc20000U
#define ESP32P4_NON_CACHEABLE_OFFSET            0x40000000U
#define ESP32P4_NON_CACHEABLE_ADDR(addr)        ((addr) + ESP32P4_NON_CACHEABLE_OFFSET)
#define ESP32P4_EXRAM_NON_CACHEABLE_ADDR_LOW     ESP32P4_NON_CACHEABLE_ADDR(ESP32P4_EXRAM_CACHEABLE_ADDR_LOW)
#define ESP32P4_EXRAM_NON_CACHEABLE_ADDR_HIGH    ESP32P4_NON_CACHEABLE_ADDR(ESP32P4_EXRAM_CACHEABLE_ADDR_HIGH)
#define ESP32P4_IRAM0_NON_CACHEABLE_ADDR_LOW     ESP32P4_NON_CACHEABLE_ADDR(ESP32P4_IRAM0_CACHEABLE_ADDR_LOW)
#define ESP32P4_IRAM0_NON_CACHEABLE_ADDR_HIGH    ESP32P4_NON_CACHEABLE_ADDR(ESP32P4_IRAM0_CACHEABLE_ADDR_HIGH)
#define ESP32P4_HPROM_NON_CACHEABLE_ADDR_LOW     ESP32P4_NON_CACHEABLE_ADDR(ESP32P4_HPROM_CACHEABLE_ADDR_LOW)
#define ESP32P4_HPROM_NON_CACHEABLE_ADDR_HIGH    ESP32P4_NON_CACHEABLE_ADDR(ESP32P4_HPROM_CACHEABLE_ADDR_HIGH)

#define ESP32P4_ADDR_IS_EXRAM_CACHEABLE(addr)    ((addr) >= ESP32P4_EXRAM_CACHEABLE_ADDR_LOW && \
	(addr) < ESP32P4_EXRAM_CACHEABLE_ADDR_HIGH)
#define ESP32P4_ADDR_IS_EXRAM_NONCACHEABLE(addr) ((addr) >= (ESP32P4_EXRAM_NON_CACHEABLE_ADDR_LOW) && \
	(addr) < (ESP32P4_EXRAM_NON_CACHEABLE_ADDR_HIGH))
#define ESP32P4_ADDR_IS_EXMEM(addr) (ESP32P4_ADDR_IS_EXRAM_NONCACHEABLE(addr) || ESP32P4_ADDR_IS_EXRAM_CACHEABLE(addr))

#define ESP32P4_ADDR_IS_IRAM_CACHEABLE(addr)      ((addr) >= ESP32P4_IRAM0_CACHEABLE_ADDR_LOW && \
	(addr) < ESP32P4_IRAM0_CACHEABLE_ADDR_HIGH)
#define ESP32P4_ADDR_IS_IRAM_NONCACHEABLE(addr)   ((addr) >= (ESP32P4_IRAM0_NON_CACHEABLE_ADDR_LOW) && \
	(addr) < (ESP32P4_IRAM0_NON_CACHEABLE_ADDR_HIGH))
#define ESP32P4_ADDR_IS_L2MEM(addr) (ESP32P4_ADDR_IS_IRAM_NONCACHEABLE(addr) || ESP32P4_ADDR_IS_IRAM_CACHEABLE(addr))

#define ESP32P4_ADDR_IS_HPROM_CACHEABLE(addr)      ((addr) >= ESP32P4_HPROM_CACHEABLE_ADDR_LOW && \
	(addr) < ESP32P4_HPROM_CACHEABLE_ADDR_HIGH)
#define ESP32P4_ADDR_IS_HPROM_NONCACHEABLE(addr)   ((addr) >= (ESP32P4_HPROM_NON_CACHEABLE_ADDR_LOW) && \
	(addr) < (ESP32P4_HPROM_NON_CACHEABLE_ADDR_HIGH))
#define ESP32P4_ADDR_IS_HPROM(addr) (ESP32P4_ADDR_IS_HPROM_NONCACHEABLE(addr) || ESP32P4_ADDR_IS_HPROM_CACHEABLE(addr))

#define ESP32P4_ADDR_IS_CACHEABLE(addr) (ESP32P4_ADDR_IS_L2MEM(addr) || ESP32P4_ADDR_IS_EXMEM(addr) || \
	ESP32P4_ADDR_IS_HPROM(addr))

#define ESP32P4_TCM_ADDR_LOW                    0x30100000U
#define ESP32P4_TCM_ADDR_HIGH                   0x30102000U
#define ESP32P4_ADDR_IS_TCMEM(addr) ((addr) >= ESP32P4_TCM_ADDR_LOW && (addr) < ESP32P4_TCM_ADDR_HIGH)

#define ESP32P4_RESERVED_ADDR_LOW               0x00000000U
#define ESP32P4_RESERVED_ADDR_HIGH              0x300FFFFFU

#define ESP32P4_IROM_MASK_LOW                   0x4fc00000
#define ESP32P4_IROM_MASK_HIGH                  0x4fc20000
#define ESP32P4_LP_ROM_LOW                      0x50100000
#define ESP32P4_LP_ROM_HIGH                     0x50104000

/* max supported hw breakpoint and watchpoint count */
#define ESP32P4_BP_NUM                          3
#define ESP32P4_WP_NUM                          3

#define ESP32P4_ASSIST_DEBUG_CPU0_MON_REG       0x3FF06000
#define ESP32P4_ASSIST_DEBUG_CPU_OFFSET         0x80

#define ESP32P4_ROM_ECO_VERSION_REG             0x4fc00014

/* components/soc/esp32p4/include/soc/reset_reasons.h */
enum esp32p4_reset_reason {
	ESP32P4_CHIP_POWER_ON_RESET   = 0x01,	/* Power on reset */
	ESP32P4_CORE_SW_RESET         = 0x03,	/* Software resets the digital core */
	ESP32P4_SYS_PMU_PWR_DOWN_RESET = 0x05,	/* PMU HP power down system reset */
	ESP32P4_CPU_PMU_PWR_DOWN_RESET = 0x06,	/* PMU HP power down CPU reset */
	ESP32P4_SYS_HP_WDT_RESET      = 0x07,	/* HP WDT resets system */
	ESP32P4_SYS_LP_WDT_RESET      = 0x09,	/* LP WDT resets system */
	ESP32P4_SYS_LP_CPU_RESET      = 0x0A,	/* LP CPU reset */
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

static const char *esp32p4_get_reset_reason(uint32_t reset_reason_reg_val, int shift_val)
{
	switch (ESP32P4_RESET_CAUSE(reset_reason_reg_val, shift_val)) {
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
	case ESP32P4_SYS_LP_CPU_RESET:
		return "PMU LP CPU reset";
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
	if (shift_val == ESP32P4_LP_CORE_RESET_CAUSE_SHIFT)
		return "PMU LP CPU reset";
	else
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

static void esp32p4_print_reset_reason(struct target *target, uint32_t reset_reason_reg_val)
{
	if (target->coreid == 0)
		LOG_TARGET_INFO(target, "Reset cause (%ld) - (%s)",
			ESP32P4_RESET_CAUSE(reset_reason_reg_val, ESP32P4_HP_CORE0_RESET_CAUSE_SHIFT),
			esp32p4_get_reset_reason(reset_reason_reg_val, ESP32P4_HP_CORE0_RESET_CAUSE_SHIFT));
	else
		LOG_TARGET_INFO(target, "Reset cause (%ld) - (%s)",
			ESP32P4_RESET_CAUSE(reset_reason_reg_val, ESP32P4_HP_CORE1_RESET_CAUSE_SHIFT),
			esp32p4_get_reset_reason(reset_reason_reg_val, ESP32P4_HP_CORE1_RESET_CAUSE_SHIFT));

}

static bool esp32p4_is_idram_address(target_addr_t addr)
{
	return ESP32P4_ADDR_IS_L2MEM(addr) || ESP32P4_ADDR_IS_TCMEM(addr);
}

static bool esp32p4_is_reserved_address(target_addr_t addr)
{
	return addr < ESP32P4_RESERVED_ADDR_HIGH;
}

static const struct esp_semihost_ops esp32p4_semihost_ops = {
	.prepare = NULL,
	.post_reset = esp_semihosting_post_reset
};

static const struct esp_flash_breakpoint_ops esp32p4_flash_brp_ops = {
	.breakpoint_prepare = esp_algo_flash_breakpoint_prepare,
	.breakpoint_add = esp_algo_flash_breakpoint_add,
	.breakpoint_remove = esp_algo_flash_breakpoint_remove,
};

static const char *esp32p4_csrs[] = {
	"mie", "mcause", "mip", "mtvt", "mnxti", "jvt",
	"mscratchcsw", "mscratchcswl",
	"mcycle", "minstret", "mcounteren", "mcountinhibit",
	"mhpmcounter8", "mhpmcounter9", "mhpmcounter13", "mhpmevent8", "mhpmevent9", "mhpmevent13",
	"mcycleh", "minstreth", "mhpmcounter8h", "mhpmcounter9h", "mhpmcounter13h",
	"tdata3", "tinfo", "mcontext", "mintstatus",
	"fflags", "frm", "fcsr",
	"csr_mintstatus", "mclicbase", "mxstatus", "mhcr", "mhint", "mraddr", "mexstatus",
	"mnmicause", "mnmipc", "mcpuid", "cpu_testbus_ctrl", "pm_user",
	"gpio_oen_user", "gpio_in_user", "gpio_out_user",
	"pma_cfg0", "pma_cfg1", "pma_cfg2", "pma_cfg3", "pma_cfg4", "pma_cfg5",
	"pma_cfg6", "pma_cfg7", "pma_cfg8", "pma_cfg9", "pma_cfg10", "pma_cfg11",
	"pma_cfg12", "pma_cfg13", "pma_cfg14", "pma_cfg15", "pma_addr0", "pma_addr1",
	"pma_addr2", "pma_addr3", "pma_addr4", "pma_addr5", "pma_addr6", "pma_addr7",
	"pma_addr8", "pma_addr9", "pma_addr10", "pma_addr11", "pma_addr12", "pma_addr13",
	"pma_addr14", "pma_addr15",
	"mext_ill_reg", "mhwloop_state_reg", "mext_pie_status",
	"ldpc0", "ldpc1", "stpc0", "stpc1", "stpc2",
	"ldtval0", "ldtval1", "sttval0", "sttval1", "sttval2",
};

static const char *esp32p4_user_counter_csrs[] = {
	/* user counters cannot be accessed in user mode unless corresponding mcounteren bit is set */
	"cycle", "time", "instreth", "cycleh", "instret", "timeh",
	"hpmcounter8", "hpmcounter9", "hpmcounter13", "hpmcounter8h", "hpmcounter9h", "hpmcounter13h",
};

static const char *esp32p4_hwloop_csrs[] = {
	"uhwloop0_start_addr", "uhwloop0_end_addr", "uhwloop0_count",
	"uhwloop1_start_addr", "uhwloop1_end_addr", "uhwloop1_count",
	"mhwloop0_start_addr", "mhwloop0_end_addr", "mhwloop0_count",
	"mhwloop1_start_addr", "mhwloop1_end_addr", "mhwloop1_count",
};

static const char *esp32p4_fpu_csrs[] = {
	"fxcr",
};

static const char *esp32p4_pie_movx_regs[] = {
	"sar", "sar_byte", "fft_bit_width", "cfg",
};

static const char *esp32p4_pie_vec_regs[] = {
	"qacc_l_l", "qacc_l_h", "qacc_h_l", "qacc_h_h", "ua_state",
	"q0", "q1", "q2", "q3", "q4", "q5", "q6", "q7",
};

static const char *esp32p4_pie_xacc_regs[] = {
	"xacc",
};

#define HWLOOP_STATE_OFF      0
#define HWLOOP_STATE_INIT     1
#define HWLOOP_STATE_MASK     3
#define CSR_MHWLOOP_STATE_REG 2033

static int esp32p4_hwloop_csr_get(struct reg *reg)
{
	return esp_riscv_csr_access_enable(reg, NULL, GDB_REGNO_CSR0 + CSR_MHWLOOP_STATE_REG,
		HWLOOP_STATE_MASK, HWLOOP_STATE_OFF, HWLOOP_STATE_INIT);
}

static int esp32p4_hwloop_csr_set(struct reg *reg, uint8_t *buf)
{
	return esp_riscv_csr_access_enable(reg, buf, GDB_REGNO_CSR0 + CSR_MHWLOOP_STATE_REG,
		HWLOOP_STATE_MASK, HWLOOP_STATE_OFF, HWLOOP_STATE_INIT);
}

static int esp32p4_read_hw_rev(struct target *target)
{
	static uint32_t hw_rev;

	if (hw_rev != 0) {
		target->hw_rev = hw_rev;
		return ERROR_OK;
	}

	int ret = target_read_u32(target, ESP32P4_ROM_ECO_VERSION_REG, &hw_rev);
	if (ret != ERROR_OK) {
		LOG_TARGET_ERROR(target, "Failed to read HW rev (%d)", ret);
		return ret;
	}

	target->hw_rev = hw_rev;
	LOG_TARGET_INFO(target, "ROM ECO version %d", hw_rev);

	return ERROR_OK;
}

static int esp32p4_examine_end(struct target *target)
{
	esp32p4_read_hw_rev(target);

	if (target->hw_rev >= 5) {
		target_free_all_working_areas(target); // Free the default working area
		target->working_area_phys = ESP32P4_IRAM0_NON_CACHEABLE_ADDR_LOW + 0x80000;
		target->working_area_virt = ESP32P4_IRAM0_NON_CACHEABLE_ADDR_LOW + 0x80000;
		target->working_area_size = 0x24000;
		target->backup_working_area = 1;
		target->working_area_phys_spec = true;
		target->working_area_virt_spec = true;
		target_free_all_working_areas(target); // Free the new working area
	}

	for (unsigned int i = 0; i < target->reg_cache->num_regs; i++) {
		const char *reg_name = target->reg_cache->reg_list[i].name;
		if ((target->hw_rev < 5
				&& !strcmp(reg_name, "csr_mintstatus")) ||
			(target->hw_rev >= 5
				&& (!strcmp(reg_name, "mnmicause")
					|| !strcmp(reg_name, "mnmipc")
					|| !strcmp(reg_name, "mintstatus"))))
			target->reg_cache->reg_list[i].exist = false;
	}

	return ERROR_OK;
}

static struct reg_arch_type esp32p4_hwloop_reg_type = {
	.get = esp32p4_hwloop_csr_get,
	.set = esp32p4_hwloop_csr_set
};

#define PIE_STATE_OFF		0
#define PIE_STATE_INIT		1
#define PIE_STATE_MASK		3
#define CSR_MEXT_PIE_STATUS 2034

struct pie_inst_table {
	const char *name;
	riscv_insn_t write_inst;
	riscv_insn_t read_inst;
	bool is_ldst_inst;
};

/*
Bellow tables contain instruction values for PIE register access.
Instructions using register s0 were selected:

ESP.MOVX.[W|R].[CFG|SAR|SAR.BYTES|FFT.BIT.WIDTH] s0
ESP.[LD|ST].UA.STATE.IP s0, 0
ESP.[LD|ST].QACC.[H|L].[H|L].128.IP s0, 0
ESP.[ST.U.XACC|LD.XACC].IP s0, 0
ESP.[VLD|VST].128.IP q[0-7], s0, 0

Can regenerate using tooolchain with xespv support e.g.:
riscv32-esp-elf-as -march=rv32imac_xespv -mespv-spec=[2p1|2p2] tmp.S -o tmp.elf
*/

static const struct pie_inst_table pie_v2p1_regs[] = {
	{ "sar", 0x90b0005f, 0x80b0005f, false },
	{ "sar_byte", 0x98b0005f, 0x88b0005f, false },
	{ "fft_bit_width", 0x94d0005f, 0x84d0005f, false },
	{ "cfg", 0x90d0005f, 0x80d0005f, false },
	{ "ua_state", 0x2000413b, 0xa000413b, true },
	{ "xacc", 0x2000433b, 0x200041bb, true },
	{ "qacc_h_l", 0x6000403b, 0xe000403b, true },
	{ "qacc_h_h", 0x4000403b, 0xc000403b, true },
	{ "qacc_l_l", 0x2000403b, 0xa000403b, true },
	{ "qacc_l_h", 0x0000403b, 0x8000403b, true },
	{ "q0", 0x0200203b, 0x8200203b, true },
	{ "q1", 0x0200243b, 0x8200243b, true },
	{ "q2", 0x0200283b, 0x8200283b, true },
	{ "q3", 0x02002c3b, 0x82002c3b, true },
	{ "q4", 0x0200303b, 0x8200303b, true },
	{ "q5", 0x0200343b, 0x8200343b, true },
	{ "q6", 0x0200383b, 0x8200383b, true },
	{ "q7", 0x02003c3b, 0x82003c3b, true },
	{ 0 },
};

static const struct pie_inst_table pie_v2p2_regs[] = {
	{ "sar", 0x90c0201b, 0x80c0201b, false },
	{ "sar_byte", 0x98c0201b, 0x88c0201b, false },
	{ "fft_bit_width", 0x9480201b, 0x8480201b, false },
	{ "cfg", 0x9080201b, 0x8080201b, false },
	{ "ua_state", 0x0010011f, 0x1010011f, true },
	{ "xacc", 0x0010031f, 0x0010019f, true },
	{ "qacc_h_l", 0x0810001f, 0x1810001f, true },
	{ "qacc_h_h", 0x0800001f, 0x1800001f, true },
	{ "qacc_l_l", 0x0010001f, 0x1010001f, true },
	{ "qacc_l_h", 0x0000001f, 0x1000001f, true },
	{ "q0", 0x0310001f, 0x8310001f, true },
	{ "q1", 0x0310041f, 0x8310041f, true },
	{ "q2", 0x0310081f, 0x8310081f, true },
	{ "q3", 0x03100c1f, 0x83100c1f, true },
	{ "q4", 0x0310101f, 0x8310101f, true },
	{ "q5", 0x0310141f, 0x8310141f, true },
	{ "q6", 0x0310181f, 0x8310181f, true },
	{ "q7", 0x03101c1f, 0x83101c1f, true },
	{ 0 },
};

static int esp32p4_read_memory(struct target *target, target_addr_t address,
	uint32_t size, uint32_t count, uint8_t *buffer);
static int esp32p4_write_memory(struct target *target, target_addr_t address,
	uint32_t size, uint32_t count, const uint8_t *buffer);

static int execute_pie_inst(struct target *target, riscv_insn_t inst)
{
	struct riscv_program program;
	riscv_program_init(&program, target);
	if (riscv_program_insert(&program, inst) != ERROR_OK)
		return ERROR_FAIL;
	return riscv_program_exec(&program, target);
}

static int pie_movx_access(struct target *target, struct reg *reg, uint8_t *buf, riscv_insn_t inst)
{
	riscv_reg_t reg_val;
	if (buf) {
		reg_val = buf_get_u64(buf, 0, riscv_xlen(target));
		riscv_set_register(target, GDB_REGNO_S0, reg_val);
		riscv_flush_registers(target);
	}
	int ret = execute_pie_inst(target, inst);
	if (ret != ERROR_OK)
		return ret;
	if (!buf) {
		target->reg_cache->reg_list[GDB_REGNO_S0].valid = false;
		target->reg_cache->reg_list[GDB_REGNO_S0].dirty = false;
		riscv_get_register(target, &reg_val, GDB_REGNO_S0);
	}
	buf_set_u64(reg->value, 0, reg->size, reg_val);
	return ERROR_OK;
}

static int pie_ldst_access(struct target *target, struct reg *reg, uint8_t *buf, riscv_insn_t inst)
{
	uint8_t saved_mem[16];
	target_addr_t temp_mem = target->working_area_phys - ESP32P4_NON_CACHEABLE_OFFSET;
	riscv_set_register(target, GDB_REGNO_S0, temp_mem);
	riscv_flush_registers(target);
	esp32p4_read_memory(target, temp_mem, 4, 4, saved_mem);
	if (buf)
		esp32p4_write_memory(target, temp_mem, 1, DIV_ROUND_UP(reg->size, 8), buf);
	int ret = execute_pie_inst(target, inst);
	if (ret == ERROR_OK) {
		esp32p4_read_memory(target, temp_mem, 1, DIV_ROUND_UP(reg->size, 8), reg->value);
		reg->valid = true; // these can be set cacheable
	}
	esp32p4_write_memory(target, temp_mem, 4, 4, saved_mem);
	return ret;
}

static int pie_access(struct reg *reg, uint8_t *buf)
{
	struct target *target = ((riscv_reg_info_t *)(reg->arch_info))->target;

	const struct pie_inst_table *pie_inst_table = target->hw_rev < 5 ? pie_v2p1_regs : pie_v2p2_regs;
	riscv_insn_t inst = 0;
	bool is_ldst_inst = false;
	for (size_t i = 0; pie_inst_table[i].name; i++) {
		if (!strcmp(reg->name, pie_inst_table[i].name)) {
			inst = buf ? pie_inst_table[i].write_inst : pie_inst_table[i].read_inst;
			is_ldst_inst = pie_inst_table[i].is_ldst_inst;
			break;
		}
	}
	if (inst == 0)
		return ERROR_FAIL;

	riscv_reg_t reg_val_s0, reg_val_pie;
	riscv_get_register(target, &reg_val_s0, GDB_REGNO_S0);
	riscv_get_register(target, &reg_val_pie, GDB_REGNO_CSR0 + CSR_MEXT_PIE_STATUS);
	int state = get_field(reg_val_pie, PIE_STATE_MASK);
	if (state == PIE_STATE_OFF)
		riscv_set_register(target, GDB_REGNO_CSR0 + CSR_MEXT_PIE_STATUS, reg_val_pie | PIE_STATE_INIT);

	int ret = (is_ldst_inst ? pie_ldst_access : pie_movx_access) (target, reg, buf, inst);

	riscv_set_register(target, GDB_REGNO_S0, reg_val_s0);
	if (state == PIE_STATE_OFF)
		riscv_set_register(target, GDB_REGNO_CSR0 + CSR_MEXT_PIE_STATUS, reg_val_pie);
	return ret;
}

static int pie_get(struct reg *reg)
{
	return pie_access(reg, NULL);
}

static int pie_set(struct reg *reg, uint8_t *buf)
{
	return pie_access(reg, buf);
}

static struct reg_arch_type esp32p4_pie_reg_type = {
	.get = pie_get,
	.set = pie_set
};

static struct reg_data_type type_int8 = { .type = REG_TYPE_INT8, .id = "int8" };
static struct reg_data_type type_int32 = { .type = REG_TYPE_INT32, .id = "int32" };
static struct reg_data_type type_int64 = { .type = REG_TYPE_INT64, .id = "int64" };
static struct reg_data_type type_uint128 = { .type = REG_TYPE_UINT128, .id = "uint128" };

static struct reg_data_type_vector type_v5i8 = {
	.type = &type_int8,
	.count = 5
};

static struct reg_data_type_vector type_v4i32 = {
	.type = &type_int32,
	.count = 4
};

static struct reg_data_type_vector type_v2i64 = {
	.type = &type_int64,
	.count = 2
};

static struct reg_data_type type_esp32p4_v5i8 = {
	.type = REG_TYPE_ARCH_DEFINED,
	.id = "v5_int8",
	.type_class = REG_TYPE_CLASS_VECTOR,
	.reg_type_vector = &type_v5i8,
};

static struct reg_data_type type_esp32p4_v4i32 = {
	.type = REG_TYPE_ARCH_DEFINED,
	.id = "v4_int32",
	.type_class = REG_TYPE_CLASS_VECTOR,
	.reg_type_vector = &type_v4i32,
};

static struct reg_data_type type_esp32p4_v2i64 = {
	.type = REG_TYPE_ARCH_DEFINED,
	.id = "v2_int64",
	.type_class = REG_TYPE_CLASS_VECTOR,
	.reg_type_vector = &type_v2i64,
};

static struct reg_data_type_union_field type_vec40[] = {
	{"v5_int8", &type_esp32p4_v5i8, type_vec40 + 1},
	{"int64", &type_int64, NULL},
};

static struct reg_data_type_union_field type_vec128[] = {
	{"v4_int32", &type_esp32p4_v4i32, type_vec128 + 1},
	{"v2_int64", &type_esp32p4_v2i64, type_vec128 + 2},
	{"uint128", &type_uint128, NULL},
};

static struct reg_data_type_union type_vec40_union = {
	.fields = type_vec40
};

static struct reg_data_type_union type_vec128_union = {
	.fields = type_vec128
};

static struct reg_data_type type_esp32p4_vector40 = {
	.type = REG_TYPE_ARCH_DEFINED,
	.id = "vector40",
	.type_class = REG_TYPE_CLASS_UNION,
	.reg_type_union = &type_vec40_union,
};

static struct reg_data_type type_esp32p4_vector128 = {
	.type = REG_TYPE_ARCH_DEFINED,
	.id = "vector128",
	.type_class = REG_TYPE_CLASS_UNION,
	.reg_type_union = &type_vec128_union,
};

static struct esp_riscv_reg_class esp32p4_registers[] = {
	{
		.reg_array = esp32p4_csrs,
		.reg_array_size = ARRAY_SIZE(esp32p4_csrs),
		.save_restore = true
	},
	{
		.reg_array = esp32p4_user_counter_csrs,
		.reg_array_size = ARRAY_SIZE(esp32p4_user_counter_csrs),
		.reg_arch_type = &esp_riscv_user_counter_type
	},
	{
		.reg_array = esp32p4_fpu_csrs,
		.reg_array_size = ARRAY_SIZE(esp32p4_fpu_csrs),
		.reg_arch_type = &esp_riscv_fpu_csr_type
	},
	{
		.reg_array = esp32p4_hwloop_csrs,
		.reg_array_size = ARRAY_SIZE(esp32p4_hwloop_csrs),
		.reg_arch_type = &esp32p4_hwloop_reg_type
	},
	{
		.reg_array = esp32p4_pie_movx_regs,
		.reg_array_size = ARRAY_SIZE(esp32p4_pie_movx_regs),
		.reg_arch_type = &esp32p4_pie_reg_type
	},
	{
		.reg_array = esp32p4_pie_xacc_regs,
		.reg_array_size = ARRAY_SIZE(esp32p4_pie_xacc_regs),
		.reg_width = 64,
		.reg_arch_type = &esp32p4_pie_reg_type,
		.reg_data_type = &type_esp32p4_vector40
	},
	{
		.reg_array = esp32p4_pie_vec_regs,
		.reg_array_size = ARRAY_SIZE(esp32p4_pie_vec_regs),
		.reg_width = 128,
		.reg_arch_type = &esp32p4_pie_reg_type,
		.reg_data_type = &type_esp32p4_vector128
	},
};

static int esp32p4_sync_cache(struct target *target, target_addr_t address, uint32_t size, uint32_t map,
	uint32_t op)
{
	uint8_t value_buf[4];
	target_addr_t start_aligned_addr = ALIGN_DOWN(address, ESP32P4_CACHE_L1_LINE_SIZE);
	target_addr_t end_aligned_addr = ALIGN_DOWN(address + size + ESP32P4_CACHE_L1_LINE_SIZE - 1,
		ESP32P4_CACHE_L1_LINE_SIZE);
	uint32_t aligned_size = end_aligned_addr - start_aligned_addr;

	// TODO: what if cache is disabled! No way to understand from the OpenOCD point of view.

	target_buffer_set_u32(target, value_buf, map);
	int res = esp_riscv_write_memory(target, ESP32P4_CACHE_SYNC_MAP_REG, 4, 1, value_buf);
	if (res != ERROR_OK)
		return res;

	target_buffer_set_u32(target, value_buf, start_aligned_addr);
	res = esp_riscv_write_memory(target, ESP32P4_CACHE_SYNC_ADDR_REG, 4, 1, value_buf);
	if (res != ERROR_OK)
		return res;

	target_buffer_set_u32(target, value_buf, aligned_size);
	res = esp_riscv_write_memory(target, ESP32P4_CACHE_SYNC_SIZE_REG, 4, 1, value_buf);
	if (res != ERROR_OK)
		return res;

	target_buffer_set_u32(target, value_buf, op);
	return esp_riscv_write_memory(target, ESP32P4_CACHE_SYNC_CTRL_REG, 4, 1, value_buf);

	/* Looks like no need to wait for sync done. Everytime ESP32P4_CACHE_SYNC_CTRL_REG read as 0x10 at first try */
}

static void esp32p4_cache_writeback(struct target *target, target_addr_t address, uint32_t size)
{
	if (ESP32P4_ADDR_IS_CACHEABLE(address)) {
		/* Write-back is for dcache and l2 cache only */
		if (esp32p4_sync_cache(target, address, size,
				ESP32P4_CACHE_MAP_L1_DCACHE | ESP32P4_CACHE_MAP_L2_CACHE, ESP32P4_CACHE_SYNC_WRITEBACK) != ERROR_OK)
			LOG_TARGET_WARNING(target, "Cache writeback failed! Read main memory anyway.");
	}
}

static void esp32p4_cache_invalidate(struct target *target, target_addr_t address, uint32_t size)
{
	if (ESP32P4_ADDR_IS_CACHEABLE(address)) {
		/* Don't invalidate the L2CACHE here. We don't know if it has been written back to the PSRAM yet. */
		if (esp32p4_sync_cache(target, address, size,
				ESP32P4_CACHE_MAP_L1_CACHE, ESP32P4_CACHE_SYNC_INVALIDATE) != ERROR_OK)
			LOG_TARGET_WARNING(target, "Cache invalidate failed!");
	}
}

static int esp32p4_target_create(struct target *target)
{
	struct esp_riscv_common *esp_riscv = calloc(1, sizeof(*esp_riscv));
	if (!esp_riscv)
		return ERROR_FAIL;

	target->arch_info = esp_riscv;

	esp_riscv->assist_debug_cpu0_mon_reg = ESP32P4_ASSIST_DEBUG_CPU0_MON_REG;
	esp_riscv->assist_debug_cpu_offset = ESP32P4_ASSIST_DEBUG_CPU_OFFSET;

	esp_riscv->max_bp_num = ESP32P4_BP_NUM;
	esp_riscv->max_wp_num = ESP32P4_WP_NUM;

	esp_riscv->rtccntl_reset_state_reg = ESP32P4_LP_CLKRST_RESET_CAUSE_REG;
	esp_riscv->print_reset_reason = &esp32p4_print_reset_reason;
	esp_riscv->chip_specific_registers = esp32p4_registers;
	esp_riscv->chip_specific_registers_size = ARRAY_SIZE(esp32p4_registers);
	esp_riscv->is_dram_address = esp32p4_is_idram_address;
	esp_riscv->is_iram_address = esp32p4_is_idram_address;
	esp_riscv->examine_end = esp32p4_examine_end;

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

	RISCV_INFO(info);
	info->mem_access_sysbus_cache_sync = true;
	info->cache_writeback = esp32p4_cache_writeback;
	info->cache_invalidate = esp32p4_cache_invalidate;

	target->semihosting->user_command_extension = esp_semihosting_common;

	struct esp_riscv_common *esp_riscv = target_to_esp_riscv(target);

	ret = esp_riscv_init_arch_info(target,
		esp_riscv,
		&esp32p4_flash_brp_ops,
		&esp32p4_semihost_ops);
	if (ret != ERROR_OK)
		return ret;

	return ERROR_OK;
}

static int esp32p4_read_memory(struct target *target, target_addr_t address,
	uint32_t size, uint32_t count, uint8_t *buffer)
{
	if (esp32p4_is_reserved_address(address)) {
		/* TODO: OCD-976 */
		memset(buffer, 0, size * count);
		return ERROR_OK;
	}

	/* OpenOCD can not read from cacheable address through sysbus on ECO5. */
	if (ESP32P4_ADDR_IS_CACHEABLE(address) && target->state == TARGET_RUNNING)
		address = ESP32P4_NON_CACHEABLE_ADDR(address);

	return esp_riscv_read_memory(target, address, size, count, buffer);
}

static int esp32p4_write_memory(struct target *target, target_addr_t address,
	uint32_t size, uint32_t count, const uint8_t *buffer)
{
	if (ESP32P4_ADDR_IS_CACHEABLE(address) && target->state == TARGET_RUNNING)
		address = ESP32P4_NON_CACHEABLE_ADDR(address);

	return esp_riscv_write_memory(target, address, size, count, buffer);
}

static int esp32p4_get_gdb_memory_map(struct target *target, struct target_memory_map *memory_map)
{
	struct target_memory_region region = { 0 };

	region.type = MEMORY_TYPE_ROM;
	region.start = ESP32P4_IROM_MASK_LOW;
	region.length = ESP32P4_IROM_MASK_HIGH - ESP32P4_IROM_MASK_LOW;
	int ret = target_add_memory_region(memory_map, &region);
	if (ret != ERROR_OK)
		return ret;

	region.type = MEMORY_TYPE_ROM;
	region.start = ESP32P4_LP_ROM_LOW;
	region.length = ESP32P4_LP_ROM_HIGH - ESP32P4_LP_ROM_LOW;
	return target_add_memory_region(memory_map, &region);
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
	.step = esp_riscv_step,

	.assert_reset = esp_riscv_assert_reset,
	.deassert_reset = riscv_deassert_reset,

	.read_memory = esp32p4_read_memory,
	.write_memory = esp32p4_write_memory,

	.checksum_memory = riscv_checksum_memory,

	.get_gdb_arch = riscv_get_gdb_arch,
	.get_gdb_reg_list = esp_riscv_get_gdb_reg_list,
	.get_gdb_reg_list_noread = riscv_get_gdb_reg_list_noread,
	.get_gdb_memory_map = esp32p4_get_gdb_memory_map,

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
