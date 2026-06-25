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
#include <target/riscv/riscv.h>
#include <target/riscv/riscv_reg.h>

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

#define ESP32P4_ADDR_IN_CACHE_REGION(addr) (ESP32P4_ADDR_IS_L2MEM(addr) || \
	ESP32P4_ADDR_IS_EXMEM(addr) || ESP32P4_ADDR_IS_HPROM(addr))

#define ESP32P4_ADDR_IS_CACHEABLE(addr) (ESP32P4_ADDR_IS_IRAM_CACHEABLE(addr) || \
	ESP32P4_ADDR_IS_EXRAM_CACHEABLE(addr) || ESP32P4_ADDR_IS_HPROM_CACHEABLE(addr))


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

#define ESP32P4_EFUSE_HW_REV_ADDR               0x5012D04c

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
	"utvt", "unxti",
	"mscratchcsw", "mscratchcswl",
	"mcycle", "minstret", "mcounteren", "mcountinhibit",
	"mhpmcounter8", "mhpmcounter9", "mhpmcounter13", "mhpmevent8", "mhpmevent9", "mhpmevent13",
	"mcycleh", "minstreth", "mhpmcounter8h", "mhpmcounter9h", "mhpmcounter13h",
	"tdata3", "tinfo", "mcontext", "mintstatus",
	"fflags", "frm", "fcsr",
};

static const char *esp32p4_user_counter_csrs[] = {
	/* user counters cannot be accessed in user mode unless corresponding mcounteren bit is set */
	"cycle", "time", "instreth", "cycleh", "instret", "timeh",
	"hpmcounter8", "hpmcounter9", "hpmcounter13", "hpmcounter8h", "hpmcounter9h", "hpmcounter13h",
};

static int esp32p4_read_hw_rev(struct target *target)
{
	static uint32_t hw_rev;

	if (hw_rev != 0) {
		target->hw_rev = hw_rev;
		return ERROR_OK;
	}

	int ret = target_read_u32(target, ESP32P4_EFUSE_HW_REV_ADDR, &hw_rev);
	if (ret != ERROR_OK) {
		LOG_TARGET_ERROR(target, "Failed to read HW rev (%d)", ret);
		return ret;
	}

	unsigned int major = ((hw_rev >> 23) & 1) << 2 | ((hw_rev >> 4) & 0x03);
	unsigned int minor = hw_rev & 0x0F;

	hw_rev = 100 * major + minor;
	target->hw_rev = hw_rev;
	LOG_TARGET_INFO(target, "Chip revision v%u.%u", major, minor);

	return ERROR_OK;
}

static int esp32p4_examine_end(struct target *target)
{
	esp32p4_read_hw_rev(target);

	struct esp_riscv_common *esp_riscv = target_to_esp_riscv(target);
	if (target->hw_rev >= 300) {
		target_free_all_working_areas(target); // Free the default working area
		target->working_area_phys = ESP32P4_IRAM0_NON_CACHEABLE_ADDR_LOW + 0x80000;
		target->working_area_virt = ESP32P4_IRAM0_NON_CACHEABLE_ADDR_LOW + 0x80000;
		target->working_area_size = 0x24000;
		target->backup_working_area = 1;
		target->working_area_phys_spec = true;
		target->working_area_virt_spec = true;
		target_free_all_working_areas(target); // Free the new working area
	} else {
		esp_riscv->pie_version = PIE_V2P1;
	}
	esp_riscv->pie_temp_mem = target->working_area_phys - ESP32P4_NON_CACHEABLE_OFFSET;

	static const char *esp32p4_rev1_csrs[] = {"mnmicause", "mnmipc", "mintstatus", ""};
	static const char *esp32p4_rev3_csrs[] = {
		"ustatus", "utvec", "uscratch", "uepc", "ucause",
		"mintthresh", "uintthresh", "uclicbase", "utvt", "unxti",
		"csr_mintstatus", "csr_uintstatus", "uhwloop_state_reg",
		""
	};
	const char **esp32p4_missing_regs = target->hw_rev < 300 ? esp32p4_rev3_csrs : esp32p4_rev1_csrs;
	for (unsigned int i = 0; i < target->reg_cache->num_regs; i++) {
		const char *reg_name = target->reg_cache->reg_list[i].name;
		for (int j = 0; strlen(esp32p4_missing_regs[j]); ++j) {
			if (!strcmp(reg_name, esp32p4_missing_regs[j])) {
				target->reg_cache->reg_list[i].exist = false;
				free(target->reg_cache->reg_list[i].value);
				target->reg_cache->reg_list[i].value = NULL;
			}
		}
	}

	return ERROR_OK;
}

static int esp32p4_user_counter_get(struct reg *reg)
{
	struct target *target = ((riscv_reg_info_t *)(reg->arch_info))->target;
	if (target->hw_rev < 300)
		return esp_riscv_user_counter_type.get(reg);
	return target->reg_cache->reg_list[GDB_REGNO_A0].type->get(reg);
}

static int esp32p4_user_counter_set(struct reg *reg, uint8_t *buf)
{
	struct target *target = ((riscv_reg_info_t *)(reg->arch_info))->target;
	if (target->hw_rev < 300)
		return esp_riscv_user_counter_type.set(reg, buf);
	return target->reg_cache->reg_list[GDB_REGNO_A0].type->set(reg, buf);
}

struct reg_arch_type esp32p4_user_counter_type = {
	.get = esp32p4_user_counter_get,
	.set = esp32p4_user_counter_set
};

static struct esp_riscv_reg_class esp32p4_registers[] = {
	{
		.reg_array = esp32p4_csrs,
		.reg_array_size = ARRAY_SIZE(esp32p4_csrs),
	},
	{
		.reg_array = esp32p4_user_counter_csrs,
		.reg_array_size = ARRAY_SIZE(esp32p4_user_counter_csrs),
		.reg_arch_type = &esp32p4_user_counter_type
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
	if (ESP32P4_ADDR_IN_CACHE_REGION(address)) {
		/* Write-back is for dcache and l2 cache only */
		if (esp32p4_sync_cache(target, address, size,
				ESP32P4_CACHE_MAP_L1_DCACHE | ESP32P4_CACHE_MAP_L2_CACHE, ESP32P4_CACHE_SYNC_WRITEBACK) != ERROR_OK)
			LOG_TARGET_WARNING(target, "Cache writeback failed! Read main memory anyway.");
	}
}

static void esp32p4_cache_invalidate(struct target *target, target_addr_t address, uint32_t size)
{
	if (ESP32P4_ADDR_IN_CACHE_REGION(address)) {
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
	struct riscv_private_config *config = target->private_config;
	if (!config) {
		config = alloc_default_riscv_private_config();
		if (!config)
			return ERROR_FAIL;
		target->private_config = config;
	}

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
	.target_jim_configure = riscv_jim_configure,
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
	.memory_ready = esp_riscv_memory_ready,

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
