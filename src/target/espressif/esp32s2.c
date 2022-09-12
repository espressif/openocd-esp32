/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   ESP32-S2 target for OpenOCD                                           *
 *   Copyright (C) 2019 Espressif Systems Ltd.                             *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <helper/time_support.h>
#include "assert.h"
#include <target/target.h>
#include <target/target_type.h>
#include <target/semihosting_common.h>
#include <rtos/rtos.h>
#include <flash/nor/esp_xtensa.h>
#include "esp32s2.h"
#include "esp32_apptrace.h"
#include "esp_xtensa.h"
#include "esp_semihosting.h"

/* Overall memory map
 * TODO: read memory configuration from target registers */
#define ESP32_S2_IROM_MASK_LOW          0x40000000
#define ESP32_S2_IROM_MASK_HIGH         0x40020000
#define ESP32_S2_INTERNAL_DROM_LOW      0x3FFA0000
#define ESP32_S2_INTERNAL_DROM_HIGH     0x3FFB0000
#define ESP32_S2_IRAM_LOW               0x40020000
#define ESP32_S2_IRAM_HIGH              0x40070000
#define ESP32_S2_DRAM_LOW               0x3ffb0000
#define ESP32_S2_DRAM_HIGH              0x40000000
#define ESP32_S2_RTC_IRAM_LOW           0x40070000
#define ESP32_S2_RTC_IRAM_HIGH          0x40072000
#define ESP32_S2_RTC_DRAM_LOW           0x3ff9e000
#define ESP32_S2_RTC_DRAM_HIGH          0x3ffa0000
#define ESP32_S2_RTC_DATA_LOW           0x50000000
#define ESP32_S2_RTC_DATA_HIGH          0x50002000
#define ESP32_S2_EXTRAM_DATA_LOW        0x3f500000
#define ESP32_S2_EXTRAM_DATA_HIGH       0x3ff80000
#define ESP32_S2_DR_REG_LOW             0x3f400000
#define ESP32_S2_DR_REG_HIGH            0x3f4d3FFC
#define ESP32_S2_SYS_RAM_LOW            0x60000000UL
#define ESP32_S2_SYS_RAM_HIGH           (ESP32_S2_SYS_RAM_LOW + 0x20000000UL)
/* ESP32-S2 DROM mapping is not contiguous. */
/* IDF declares this as 0x3F000000..0x3FF80000, but there are peripheral registers mapped to
 * 0x3f400000..0x3f4d3FFC. */
#define ESP32_S2_DROM0_LOW              ESP32_S2_DROM_LOW
#define ESP32_S2_DROM0_HIGH             ESP32_S2_DR_REG_LOW
#define ESP32_S2_DROM1_LOW              ESP32_S2_DR_REG_HIGH
#define ESP32_S2_DROM1_HIGH             ESP32_S2_DROM_HIGH

/* ESP32 WDT */
#define ESP32_S2_WDT_WKEY_VALUE         0x50d83aa1
#define ESP32_S2_TIMG0_BASE             0x3f41F000
#define ESP32_S2_TIMG1_BASE             0x3f420000
#define ESP32_S2_TIMGWDT_CFG0_OFF       0x48
#define ESP32_S2_TIMGWDT_PROTECT_OFF    0x64
#define ESP32_S2_TIMG0WDT_CFG0          (ESP32_S2_TIMG0_BASE + ESP32_S2_TIMGWDT_CFG0_OFF)
#define ESP32_S2_TIMG1WDT_CFG0          (ESP32_S2_TIMG1_BASE + ESP32_S2_TIMGWDT_CFG0_OFF)
#define ESP32_S2_TIMG0WDT_PROTECT       (ESP32_S2_TIMG0_BASE + ESP32_S2_TIMGWDT_PROTECT_OFF)
#define ESP32_S2_TIMG1WDT_PROTECT       (ESP32_S2_TIMG1_BASE + ESP32_S2_TIMGWDT_PROTECT_OFF)
#define ESP32_S2_RTCCNTL_BASE           0x3f408000
#define ESP32_S2_RTCWDT_CFG_OFF         0x94
#define ESP32_S2_RTCWDT_PROTECT_OFF     0xAC
#define ESP32_S2_SWD_CONF_OFF           0xB0
#define ESP32_S2_SWD_WPROTECT_OFF       0xB4
#define ESP32_S2_RTC_CNTL_DIG_PWC_REG_OFF      0x8C
#define ESP32_S2_RTC_CNTL_DIG_PWC_REG   (ESP32_S2_RTCCNTL_BASE + ESP32_S2_RTC_CNTL_DIG_PWC_REG_OFF)
#define ESP32_S2_RTCWDT_CFG             (ESP32_S2_RTCCNTL_BASE + ESP32_S2_RTCWDT_CFG_OFF)
#define ESP32_S2_RTCWDT_PROTECT         (ESP32_S2_RTCCNTL_BASE + ESP32_S2_RTCWDT_PROTECT_OFF)
#define ESP32_S2_SWD_CONF_REG           (ESP32_S2_RTCCNTL_BASE + ESP32_S2_SWD_CONF_OFF)
#define ESP32_S2_SWD_WPROTECT_REG       (ESP32_S2_RTCCNTL_BASE + ESP32_S2_SWD_WPROTECT_OFF)
#define ESP32_S2_SWD_AUTO_FEED_EN_M     BIT(31)
#define ESP32_S2_SWD_WKEY_VALUE         0x8F1D312AU
#define ESP32_S2_OPTIONS0               (ESP32_S2_RTCCNTL_BASE + 0x0000)
#define ESP32_S2_SW_SYS_RST_M           0x80000000
#define ESP32_S2_SW_SYS_RST_V           0x1
#define ESP32_S2_SW_SYS_RST_S           31
#define ESP32_S2_SW_STALL_PROCPU_C0_M   ((ESP32_S2_SW_STALL_PROCPU_C0_V) << (ESP32_S2_SW_STALL_PROCPU_C0_S))
#define ESP32_S2_SW_STALL_PROCPU_C0_V   0x3
#define ESP32_S2_SW_STALL_PROCPU_C0_S   2
#define ESP32_S2_SW_CPU_STALL           (ESP32_S2_RTCCNTL_BASE + 0x00B8)
#define ESP32_S2_SW_STALL_PROCPU_C1_M   ((ESP32_S2_SW_STALL_PROCPU_C1_V) << (ESP32_S2_SW_STALL_PROCPU_C1_S))
#define ESP32_S2_SW_STALL_PROCPU_C1_V   0x3FU
#define ESP32_S2_SW_STALL_PROCPU_C1_S   26
#define ESP32_S2_CLK_CONF               (ESP32_S2_RTCCNTL_BASE + 0x0074)
#define ESP32_S2_CLK_CONF_DEF           0x1583218
#define ESP32_S2_STORE4                 (ESP32_S2_RTCCNTL_BASE + 0x00BC)
#define ESP32_S2_STORE5                 (ESP32_S2_RTCCNTL_BASE + 0x00C0)
#define ESP32_S2_DPORT_PMS_OCCUPY_3     0x3F4C10E0

#define ESP32_S2_TRACEMEM_BLOCK_SZ      0x4000

#define ESP32_S2_DR_REG_UART_BASE       0x3f400000
#define ESP32_S2_REG_UART_BASE(i)       (ESP32_S2_DR_REG_UART_BASE + (i) * 0x10000)
#define ESP32_S2_UART_DATE_REG(i)       (ESP32_S2_REG_UART_BASE(i) + 0x74)
#define ESP32S2_RTC_CNTL_RESET_STATE_REG          (ESP32_S2_RTCCNTL_BASE + 0x38)

/* RTC_CNTL_RESET_CAUSE_PROCPU : RO ;bitpos:[5:0] ;default: 0 ;
 *description: reset cause of PRO CPU.*/
#define ESP32S2_RTC_CNTL_RESET_CAUSE_PROCPU     0x0000003F
#define ESP32S2_RTC_CNTL_RESET_CAUSE_PROCPU_M   ((ESP32S2_RTC_CNTL_RESET_CAUSE_PROCPU_V) << \
		(ESP32S2_RTC_CNTL_RESET_CAUSE_PROCPU_S))
#define ESP32S2_RTC_CNTL_RESET_CAUSE_PROCPU_V   0x3F
#define ESP32S2_RTC_CNTL_RESET_CAUSE_PROCPU_S   0

enum esp32s2_reset_reason {
	RESET_REASON_CHIP_POWER_ON   = 0x01,	/* Power on reset */
	RESET_REASON_CHIP_BROWN_OUT  = 0x01,	/* VDD voltage is not stable and resets the chip */
	RESET_REASON_CHIP_SUPER_WDT  = 0x01,	/* Super watch dog resets the chip */
	RESET_REASON_CORE_SW         = 0x03,	/* Software resets the digital core by RTC_CNTL_SW_SYS_RST */
	RESET_REASON_CORE_DEEP_SLEEP = 0x05,	/* Deep sleep reset the digital core */
	RESET_REASON_CORE_MWDT0      = 0x07,	/* Main watch dog 0 resets digital core */
	RESET_REASON_CORE_MWDT1      = 0x08,	/* Main watch dog 1 resets digital core */
	RESET_REASON_CORE_RTC_WDT    = 0x09,	/* RTC watch dog resets digital core */
	RESET_REASON_CPU0_MWDT0      = 0x0B,	/* Main watch dog 0 resets CPU 0 */
	RESET_REASON_CPU0_SW         = 0x0C,	/* Software resets CPU 0 by RTC_CNTL_SW_PROCPU_RST */
	RESET_REASON_CPU0_RTC_WDT    = 0x0D,	/* RTC watch dog resets CPU 0 */
	RESET_REASON_SYS_BROWN_OUT   = 0x0F,	/* VDD voltage is not stable and resets the digital core */
	RESET_REASON_SYS_RTC_WDT     = 0x10,	/* RTC watch dog resets digital core and rtc module */
	RESET_REASON_CPU0_MWDT1      = 0x11,	/* Main watch dog 1 resets CPU 0 */
	RESET_REASON_SYS_SUPER_WDT   = 0x12,	/* Super watch dog resets the digital core and rtc module */
	RESET_REASON_SYS_CLK_GLITCH  = 0x13,	/* Glitch on clock resets the digital core and rtc module */
	RESET_REASON_CORE_EFUSE_CRC  = 0x14,	/* eFuse CRC error resets the digital core */
};

/* this should map local reg IDs to GDB reg mapping as defined in xtensa-config.c 'rmap' in
 * xtensa-overlay */
static const unsigned int esp32s2_gdb_regs_mapping[ESP32_S2_NUM_REGS] = {
	XT_REG_IDX_PC,
	XT_REG_IDX_AR0, XT_REG_IDX_AR1, XT_REG_IDX_AR2, XT_REG_IDX_AR3,
	XT_REG_IDX_AR4, XT_REG_IDX_AR5, XT_REG_IDX_AR6, XT_REG_IDX_AR7,
	XT_REG_IDX_AR8, XT_REG_IDX_AR9, XT_REG_IDX_AR10, XT_REG_IDX_AR11,
	XT_REG_IDX_AR12, XT_REG_IDX_AR13, XT_REG_IDX_AR14, XT_REG_IDX_AR15,
	XT_REG_IDX_AR16, XT_REG_IDX_AR17, XT_REG_IDX_AR18, XT_REG_IDX_AR19,
	XT_REG_IDX_AR20, XT_REG_IDX_AR21, XT_REG_IDX_AR22, XT_REG_IDX_AR23,
	XT_REG_IDX_AR24, XT_REG_IDX_AR25, XT_REG_IDX_AR26, XT_REG_IDX_AR27,
	XT_REG_IDX_AR28, XT_REG_IDX_AR29, XT_REG_IDX_AR30, XT_REG_IDX_AR31,
	XT_REG_IDX_AR32, XT_REG_IDX_AR33, XT_REG_IDX_AR34, XT_REG_IDX_AR35,
	XT_REG_IDX_AR36, XT_REG_IDX_AR37, XT_REG_IDX_AR38, XT_REG_IDX_AR39,
	XT_REG_IDX_AR40, XT_REG_IDX_AR41, XT_REG_IDX_AR42, XT_REG_IDX_AR43,
	XT_REG_IDX_AR44, XT_REG_IDX_AR45, XT_REG_IDX_AR46, XT_REG_IDX_AR47,
	XT_REG_IDX_AR48, XT_REG_IDX_AR49, XT_REG_IDX_AR50, XT_REG_IDX_AR51,
	XT_REG_IDX_AR52, XT_REG_IDX_AR53, XT_REG_IDX_AR54, XT_REG_IDX_AR55,
	XT_REG_IDX_AR56, XT_REG_IDX_AR57, XT_REG_IDX_AR58, XT_REG_IDX_AR59,
	XT_REG_IDX_AR60, XT_REG_IDX_AR61, XT_REG_IDX_AR62, XT_REG_IDX_AR63,
	XT_REG_IDX_SAR,
	XT_REG_IDX_WINDOWBASE, XT_REG_IDX_WINDOWSTART, XT_REG_IDX_CONFIGID0, XT_REG_IDX_CONFIGID1,
	XT_REG_IDX_PS, XT_REG_IDX_THREADPTR,
	ESP32_S2_REG_IDX_GPIOOUT,
	XT_REG_IDX_MMID, XT_REG_IDX_IBREAKENABLE, XT_REG_IDX_OCD_DDR,
	XT_REG_IDX_IBREAKA0, XT_REG_IDX_IBREAKA1, XT_REG_IDX_DBREAKA0, XT_REG_IDX_DBREAKA1,
	XT_REG_IDX_DBREAKC0, XT_REG_IDX_DBREAKC1,
	XT_REG_IDX_EPC1, XT_REG_IDX_EPC2, XT_REG_IDX_EPC3, XT_REG_IDX_EPC4,
	XT_REG_IDX_EPC5, XT_REG_IDX_EPC6, XT_REG_IDX_EPC7, XT_REG_IDX_DEPC,
	XT_REG_IDX_EPS2, XT_REG_IDX_EPS3, XT_REG_IDX_EPS4, XT_REG_IDX_EPS5,
	XT_REG_IDX_EPS6, XT_REG_IDX_EPS7,
	XT_REG_IDX_EXCSAVE1, XT_REG_IDX_EXCSAVE2, XT_REG_IDX_EXCSAVE3, XT_REG_IDX_EXCSAVE4,
	XT_REG_IDX_EXCSAVE5, XT_REG_IDX_EXCSAVE6, XT_REG_IDX_EXCSAVE7, XT_REG_IDX_CPENABLE,
	XT_REG_IDX_INTERRUPT, XT_REG_IDX_INTSET, XT_REG_IDX_INTCLEAR, XT_REG_IDX_INTENABLE,
	XT_REG_IDX_VECBASE, XT_REG_IDX_EXCCAUSE, XT_REG_IDX_DEBUGCAUSE, XT_REG_IDX_CCOUNT,
	XT_REG_IDX_PRID, XT_REG_IDX_ICOUNT, XT_REG_IDX_ICOUNTLEVEL, XT_REG_IDX_EXCVADDR,
	XT_REG_IDX_CCOMPARE0, XT_REG_IDX_CCOMPARE1, XT_REG_IDX_CCOMPARE2,
	XT_REG_IDX_MISC0, XT_REG_IDX_MISC1, XT_REG_IDX_MISC2, XT_REG_IDX_MISC3,
	XT_REG_IDX_A0, XT_REG_IDX_A1, XT_REG_IDX_A2, XT_REG_IDX_A3,
	XT_REG_IDX_A4, XT_REG_IDX_A5, XT_REG_IDX_A6, XT_REG_IDX_A7,
	XT_REG_IDX_A8, XT_REG_IDX_A9, XT_REG_IDX_A10, XT_REG_IDX_A11,
	XT_REG_IDX_A12, XT_REG_IDX_A13, XT_REG_IDX_A14, XT_REG_IDX_A15,
	XT_REG_IDX_PWRCTL, XT_REG_IDX_PWRSTAT, XT_REG_IDX_ERISTAT,
	XT_REG_IDX_CS_ITCTRL, XT_REG_IDX_CS_CLAIMSET, XT_REG_IDX_CS_CLAIMCLR,
	XT_REG_IDX_CS_LOCKACCESS, XT_REG_IDX_CS_LOCKSTATUS, XT_REG_IDX_CS_AUTHSTATUS,
	XT_REG_IDX_FAULT_INFO,
	XT_REG_IDX_TRAX_ID, XT_REG_IDX_TRAX_CTRL, XT_REG_IDX_TRAX_STAT,
	XT_REG_IDX_TRAX_DATA, XT_REG_IDX_TRAX_ADDR, XT_REG_IDX_TRAX_PCTRIGGER,
	XT_REG_IDX_TRAX_PCMATCH, XT_REG_IDX_TRAX_DELAY, XT_REG_IDX_TRAX_MEMSTART,
	XT_REG_IDX_TRAX_MEMEND,
	XT_REG_IDX_PMG, XT_REG_IDX_PMPC, XT_REG_IDX_PM0, XT_REG_IDX_PM1,
	XT_REG_IDX_PMCTRL0, XT_REG_IDX_PMCTRL1, XT_REG_IDX_PMSTAT0, XT_REG_IDX_PMSTAT1,
	XT_REG_IDX_OCD_ID, XT_REG_IDX_OCD_DCRCLR, XT_REG_IDX_OCD_DCRSET, XT_REG_IDX_OCD_DSR,
};

static const struct xtensa_user_reg_desc esp32s2_user_regs[ESP32_S2_NUM_REGS - XT_NUM_REGS] = {
	{ "gpio_out", 0x00, 0, 32, &xtensa_user_reg_u32_type },
};

static struct xtensa_config esp32s2_xtensa_cfg = {
	.core_type = XT_LX,
	.density = true,
	.aregs_num = XT_AREGS_NUM_MAX,
	.windowed = true,
	.coproc = true,
	.miscregs_num = 4,
	.reloc_vec = true,
	.proc_id = true,
	.threadptr = true,
	.int_div_32 = true,
	.user_regs_num = ARRAY_SIZE(esp32s2_user_regs),
	.user_regs = esp32s2_user_regs,
	.fetch_user_regs = xtensa_fetch_user_regs_u32,
	.queue_write_dirty_user_regs = xtensa_queue_write_dirty_user_regs_u32,
	.gdb_general_regs_num = ESP32_S2_NUM_REGS_G_COMMAND,
	.gdb_regs_mapping = esp32s2_gdb_regs_mapping,
	.region_protect = {
		.enabled = true,
	},
	.irom = {
		.count = 2,
		.regions = {
			{
				.base = ESP32_S2_IROM_LOW,
				.size = ESP32_S2_IROM_HIGH - ESP32_S2_IROM_LOW,
				.access = XT_MEM_ACCESS_READ,
			},
			{
				.base = ESP32_S2_IROM_MASK_LOW,
				.size = ESP32_S2_IROM_MASK_HIGH - ESP32_S2_IROM_MASK_LOW,
				.access = XT_MEM_ACCESS_READ,
			},
		}
	},
	.iram = {
		.count = 2,
		.regions = {
			{
				.base = ESP32_S2_IRAM_LOW,
				.size = ESP32_S2_IRAM_HIGH - ESP32_S2_IRAM_LOW,
				.access = XT_MEM_ACCESS_READ | XT_MEM_ACCESS_WRITE,
			},
			{
				.base = ESP32_S2_RTC_IRAM_LOW,
				.size = ESP32_S2_RTC_IRAM_HIGH - ESP32_S2_RTC_IRAM_LOW,
				.access = XT_MEM_ACCESS_READ | XT_MEM_ACCESS_WRITE,
			},
		}
	},
	.drom = {
		.count = 3,
		.regions = {
			{
				.base = ESP32_S2_DROM0_LOW,
				.size = ESP32_S2_DROM0_HIGH - ESP32_S2_DROM0_LOW,
				.access = XT_MEM_ACCESS_READ,
			},
			{
				.base = ESP32_S2_DROM1_LOW,
				.size = ESP32_S2_DROM1_HIGH - ESP32_S2_DROM1_LOW,
				.access = XT_MEM_ACCESS_READ,
			},
			{
				.base = ESP32_S2_INTERNAL_DROM_LOW,
				.size = ESP32_S2_INTERNAL_DROM_HIGH - ESP32_S2_INTERNAL_DROM_LOW,
				.access = XT_MEM_ACCESS_READ,
			},
		}
	},
	.dram = {
		.count = 6,
		.regions = {
			{
				.base = ESP32_S2_DRAM_LOW,
				.size = ESP32_S2_DRAM_HIGH - ESP32_S2_DRAM_LOW,
				.access = XT_MEM_ACCESS_READ | XT_MEM_ACCESS_WRITE,
			},
			{
				.base = ESP32_S2_RTC_DRAM_LOW,
				.size = ESP32_S2_RTC_DRAM_HIGH - ESP32_S2_RTC_DRAM_LOW,
				.access = XT_MEM_ACCESS_READ | XT_MEM_ACCESS_WRITE,
			},
			{
				.base = ESP32_S2_RTC_DATA_LOW,
				.size = ESP32_S2_RTC_DATA_HIGH - ESP32_S2_RTC_DATA_LOW,
				.access = XT_MEM_ACCESS_READ | XT_MEM_ACCESS_WRITE,
			},
			{
				.base = ESP32_S2_EXTRAM_DATA_LOW,
				.size = ESP32_S2_EXTRAM_DATA_HIGH - ESP32_S2_EXTRAM_DATA_LOW,
				.access = XT_MEM_ACCESS_READ | XT_MEM_ACCESS_WRITE,
			},
			{
				.base = ESP32_S2_DR_REG_LOW,
				.size = ESP32_S2_DR_REG_HIGH - ESP32_S2_DR_REG_LOW,
				.access = XT_MEM_ACCESS_READ | XT_MEM_ACCESS_WRITE,
			},
			{
				.base = ESP32_S2_SYS_RAM_LOW,
				.size = ESP32_S2_SYS_RAM_HIGH - ESP32_S2_SYS_RAM_LOW,
				.access = XT_MEM_ACCESS_READ | XT_MEM_ACCESS_WRITE,
			},
		}
	},
	.exc = {
		.enabled = true,
	},
	.irq = {
		.enabled = true,
		.irq_num = 32,
	},
	.high_irq = {
		.enabled = true,
		.excm_level = 3,
		.nmi_num = 1,
	},
	.tim_irq = {
		.enabled = true,
		.comp_num = 3,
	},
	.debug = {
		.enabled = true,
		.irq_level = 6,
		.ibreaks_num = 2,
		.dbreaks_num = 2,
		.icount_sz = 32,
		.eps_dbglevel_reg_idx = XT_REG_IDX_EPS6,
	},
	.trace = {
		.enabled = true,
		.mem_sz = ESP32_S2_TRACEMEM_BLOCK_SZ,
	},
};

struct esp32s2_common {
	struct esp_xtensa_common esp_xtensa;
};

static int esp32s2_soc_reset(struct target *target);
static int esp32s2_disable_wdts(struct target *target);

static int esp32s2_assert_reset(struct target *target)
{
	return ERROR_OK;
}

static int esp32s2_deassert_reset(struct target *target)
{
	struct xtensa *xtensa = target_to_xtensa(target);

	LOG_TARGET_DEBUG(target, "begin");

	int res = xtensa_deassert_reset(target);
	if (res != ERROR_OK)
		return res;

	/* restore configured value
	   esp32s2_soc_reset() modified it, but can not restore just after SW reset for some reason (???) */
	res = xtensa_smpbreak_write(xtensa, xtensa->smp_break);
	if (res != ERROR_OK) {
		LOG_ERROR("Failed to restore smpbreak (%d)!", res);
		return res;
	}
	return ERROR_OK;
}

static int esp32s2_soft_reset_halt(struct target *target)
{
	LOG_TARGET_DEBUG(target, "begin");

	/* Reset the SoC first */
	int res = esp32s2_soc_reset(target);
	if (res != ERROR_OK)
		return res;
	return xtensa_soft_reset_halt(target);
}

static int esp32s2_set_peri_reg_mask(struct target *target,
	target_addr_t addr,
	uint32_t mask,
	uint32_t val)
{
	uint32_t reg_val;
	int res = target_read_u32(target, addr, &reg_val);
	if (res != ERROR_OK)
		return res;
	reg_val = (reg_val & (~mask)) | val;
	res = target_write_u32(target, addr, reg_val);
	if (res != ERROR_OK)
		return res;

	return ERROR_OK;
}

static int esp32s2_stall_set(struct target *target, bool stall)
{
	LOG_TARGET_DEBUG(target, "begin");

	int res = esp32s2_set_peri_reg_mask(target,
		ESP32_S2_SW_CPU_STALL,
		ESP32_S2_SW_STALL_PROCPU_C1_M,
		stall ? 0x21U << ESP32_S2_SW_STALL_PROCPU_C1_S : 0);
	if (res != ERROR_OK) {
		LOG_ERROR("Failed to write ESP32_S2_SW_CPU_STALL (%d)!", res);
		return res;
	}
	res = esp32s2_set_peri_reg_mask(target,
		ESP32_S2_OPTIONS0,
		ESP32_S2_SW_STALL_PROCPU_C0_M,
		stall ? 0x2 << ESP32_S2_SW_STALL_PROCPU_C0_S : 0);
	if (res != ERROR_OK) {
		LOG_ERROR("Failed to write ESP32_S2_OPTIONS0 (%d)!", res);
		return res;
	}
	return ERROR_OK;
}

static inline int esp32s2_stall(struct target *target)
{
	return esp32s2_stall_set(target, true);
}

static inline int esp32s2_unstall(struct target *target)
{
	return esp32s2_stall_set(target, false);
}

/* Reset ESP32-S2's peripherals.
Postconditions: all peripherals except RTC_CNTL are reset, CPU's PC is undefined, PRO CPU is halted, APP CPU is in reset
How this works:
0. make sure target is halted; if not, try to halt it; if that fails, try to reset it (via OCD) and then halt
1. Resets clock related registers
2. Stalls CPU
3. trigger SoC reset using RTC_CNTL_SW_SYS_RST bit
4. CPU is reset and stalled at the first reset vector instruction
5. wait for the OCD to be reset
6. halt the target
7. Unstalls CPU
8. Disables WDTs and trace memory mapping
*/
static int esp32s2_soc_reset(struct target *target)
{
	int res;
	struct xtensa *xtensa = target_to_xtensa(target);

	LOG_DEBUG("start");

	/* In order to write to peripheral registers, target must be halted first */
	if (target->state != TARGET_HALTED) {
		LOG_TARGET_DEBUG(target, "Target not halted before SoC reset, trying to halt it first");
		xtensa_halt(target);
		res = target_wait_state(target, TARGET_HALTED, 1000);
		if (res != ERROR_OK) {
			LOG_TARGET_DEBUG(target, "Couldn't halt target before SoC reset, trying to do reset-halt");
			res = xtensa_assert_reset(target);
			if (res != ERROR_OK) {
				LOG_TARGET_ERROR(
					target,
					"Couldn't halt target before SoC reset! (xtensa_assert_reset returned %d)",
					res);
				return res;
			}
			alive_sleep(10);
			xtensa_poll(target);
			int reset_halt_save = target->reset_halt;
			target->reset_halt = 1;
			res = xtensa_deassert_reset(target);
			target->reset_halt = reset_halt_save;
			if (res != ERROR_OK) {
				LOG_TARGET_ERROR(
					target,
					"Couldn't halt target before SoC reset! (xtensa_deassert_reset returned %d)",
					res);
				return res;
			}
			alive_sleep(10);
			xtensa_poll(target);
			xtensa_halt(target);
			res = target_wait_state(target, TARGET_HALTED, 1000);
			if (res != ERROR_OK) {
				LOG_TARGET_ERROR(target, "Couldn't halt target before SoC reset");
				return res;
			}
		}
	}

	assert(target->state == TARGET_HALTED);

	/* Set some clock-related RTC registers to the default values */
	res = target_write_u32(target, ESP32_S2_STORE4, 0);
	if (res != ERROR_OK) {
		LOG_ERROR("Failed to write ESP32_S2_STORE4 (%d)!", res);
		return res;
	}
	res = target_write_u32(target, ESP32_S2_STORE5, 0);
	if (res != ERROR_OK) {
		LOG_ERROR("Failed to write ESP32_S2_STORE5 (%d)!", res);
		return res;
	}
	res = target_write_u32(target, ESP32_S2_RTC_CNTL_DIG_PWC_REG, 0);
	if (res != ERROR_OK) {
		LOG_ERROR("Failed to write ESP32_S2_RTC_CNTL_DIG_PWC_REG (%d)!", res);
		return res;
	}
	res = target_write_u32(target, ESP32_S2_CLK_CONF, ESP32_S2_CLK_CONF_DEF);
	if (res != ERROR_OK) {
		LOG_ERROR("Failed to write ESP32_S2_CLK_CONF (%d)!", res);
		return res;
	}
	/* Stall CPU */
	res = esp32s2_stall(target);
	if (res != ERROR_OK)
		return res;
	/* enable stall */
	res = xtensa_smpbreak_write(xtensa, OCDDCR_RUNSTALLINEN);
	if (res != ERROR_OK) {
		LOG_ERROR("Failed to set smpbreak (%d)!", res);
		return res;
	}
	/* Reset CPU */
	xtensa->suppress_dsr_errors = true;
	res = esp32s2_set_peri_reg_mask(target,
		ESP32_S2_OPTIONS0,
		ESP32_S2_SW_SYS_RST_M,
		BIT(ESP32_S2_SW_SYS_RST_S));
	xtensa->suppress_dsr_errors = false;
	if (res != ERROR_OK) {
		LOG_ERROR("Failed to write ESP32_S2_OPTIONS0 (%d)!", res);
		return res;
	}
	/* Wait for SoC to reset */
	alive_sleep(100);
	int64_t timeout = timeval_ms() + 100;
	while (target->state != TARGET_RESET && target->state != TARGET_RUNNING) {
		alive_sleep(10);
		xtensa_poll(target);
		if (timeval_ms() >= timeout) {
			LOG_TARGET_ERROR(target, "Timed out waiting for CPU to be reset, target state=%d",
				target->state);
			return ERROR_TARGET_TIMEOUT;
		}
	}

	xtensa_halt(target);
	res = target_wait_state(target, TARGET_HALTED, 1000);
	if (res != ERROR_OK) {
		LOG_TARGET_ERROR(target, "Couldn't halt target before SoC reset");
		return res;
	}

	esp_xtensa_reset_reason_read(target);

	/* Unstall CPU */
	res = esp32s2_unstall(target);
	if (res != ERROR_OK)
		return res;
	/* Disable WDTs */
	res = esp32s2_disable_wdts(target);
	if (res != ERROR_OK)
		return res;
	/* Disable trace memory mapping */
	res = target_write_u32(target, ESP32_S2_DPORT_PMS_OCCUPY_3, 0);
	if (res != ERROR_OK) {
		LOG_ERROR("Failed to write ESP32_S2_DPORT_PMS_OCCUPY_3 (%d)!", res);
		return res;
	}
	/* Clear memory which is used by RTOS layer to get the task count */
	if (target->rtos && target->rtos->type->post_reset_cleanup) {
		res = (*target->rtos->type->post_reset_cleanup)(target);
		if (res != ERROR_OK)
			LOG_WARNING("Failed to do rtos-specific cleanup (%d)", res);
	}
	return ERROR_OK;
}

static int esp32s2_disable_wdts(struct target *target)
{
	/* TIMG1 WDT */
	int res = target_write_u32(target, ESP32_S2_TIMG0WDT_PROTECT, ESP32_S2_WDT_WKEY_VALUE);
	if (res != ERROR_OK) {
		LOG_ERROR("Failed to write ESP32_S2_TIMG0WDT_PROTECT (%d)!", res);
		return res;
	}
	res = target_write_u32(target, ESP32_S2_TIMG0WDT_CFG0, 0);
	if (res != ERROR_OK) {
		LOG_ERROR("Failed to write ESP32_S2_TIMG0WDT_CFG0 (%d)!", res);
		return res;
	}
	/* TIMG2 WDT */
	res = target_write_u32(target, ESP32_S2_TIMG1WDT_PROTECT, ESP32_S2_WDT_WKEY_VALUE);
	if (res != ERROR_OK) {
		LOG_ERROR("Failed to write ESP32_S2_TIMG1WDT_PROTECT (%d)!", res);
		return res;
	}
	res = target_write_u32(target, ESP32_S2_TIMG1WDT_CFG0, 0);
	if (res != ERROR_OK) {
		LOG_ERROR("Failed to write ESP32_S2_TIMG1WDT_CFG0 (%d)!", res);
		return res;
	}
	/* RTC WDT */
	res = target_write_u32(target, ESP32_S2_RTCWDT_PROTECT, ESP32_S2_WDT_WKEY_VALUE);
	if (res != ERROR_OK) {
		LOG_ERROR("Failed to write ESP32_S2_RTCWDT_PROTECT (%d)!", res);
		return res;
	}
	res = target_write_u32(target, ESP32_S2_RTCWDT_CFG, 0);
	if (res != ERROR_OK) {
		LOG_ERROR("Failed to write ESP32_S2_RTCWDT_CFG (%d)!", res);
		return res;
	}
	/* Enable SWD auto-feed */
	res = target_write_u32(target, ESP32_S2_SWD_WPROTECT_REG, ESP32_S2_SWD_WKEY_VALUE);
	if (res != ERROR_OK) {
		LOG_ERROR("Failed to write ESP32_S2_SWD_WPROTECT_REG (%d)!", res);
		return res;
	}
	uint32_t swd_conf_reg = 0;
	res = target_read_u32(target, ESP32_S2_SWD_CONF_REG, &swd_conf_reg);
	if (res != ERROR_OK) {
		LOG_ERROR("Failed to read ESP32_S2_SWD_CONF_REG (%d)!", res);
		return res;
	}
	swd_conf_reg |= ESP32_S2_SWD_AUTO_FEED_EN_M;
	res = target_write_u32(target, ESP32_S2_SWD_CONF_REG, swd_conf_reg);
	if (res != ERROR_OK) {
		LOG_ERROR("Failed to write ESP32_S2_SWD_CONF_REG (%d)!", res);
		return res;
	}
	return ERROR_OK;
}

static int esp32s2_arch_state(struct target *target)
{
	return ERROR_OK;
}

static int esp32s2_on_halt(struct target *target)
{
	int ret = esp32s2_disable_wdts(target);
	if (ret == ERROR_OK)
		ret = esp_xtensa_on_halt(target);
	return ret;
}

static int esp32s2_step(struct target *target, int current, target_addr_t address, int handle_breakpoints)
{
	int ret = xtensa_step(target, current, address, handle_breakpoints);
	if (ret == ERROR_OK) {
		esp32s2_on_halt(target);
		target_call_event_callbacks(target, TARGET_EVENT_HALTED);
	}
	return ret;
}

static int esp32s2_poll(struct target *target)
{
	enum target_state old_state = target->state;
	int ret = esp_xtensa_poll(target);

	if (old_state != TARGET_HALTED && target->state == TARGET_HALTED) {
		/* Call any event callbacks that are applicable */
		if (old_state == TARGET_DEBUG_RUNNING) {
			target_call_event_callbacks(target, TARGET_EVENT_DEBUG_HALTED);
		} else {
			if (esp_xtensa_semihosting(target, &ret) == SEMIHOSTING_HANDLED) {
				struct esp_xtensa_common *esp_xtensa = target_to_esp_xtensa(target);
				if (ret == ERROR_OK && esp_xtensa->semihost.need_resume) {
					esp_xtensa->semihost.need_resume = false;
					/* Resume xtensa_resume will handle BREAK instruction. */
					ret = target_resume(target, 1, 0, 1, 0);
					if (ret != ERROR_OK) {
						LOG_ERROR("Failed to resume target");
						return ret;
					}
				}
				return ret;
			}
			esp32s2_on_halt(target);
			target_call_event_callbacks(target, TARGET_EVENT_HALTED);
		}
	}

	return ret;
}

static int esp32s2_virt2phys(struct target *target,
	target_addr_t virtual, target_addr_t *physical)
{
	*physical = virtual;
	return ERROR_OK;
}

static const char *esp32s2_reset_reason_str(int coreid, enum esp32s2_reset_reason reset_number)
{
	switch (reset_number) {
	case RESET_REASON_CHIP_POWER_ON:
		return "Power on reset";
	case RESET_REASON_CORE_SW:
		return "Software core reset";
	case RESET_REASON_CORE_DEEP_SLEEP:
		return "Deep-sleep core reset";
	case RESET_REASON_CORE_MWDT0:
		return "Main WDT0 core reset";
	case RESET_REASON_CORE_MWDT1:
		return "Main WDT1 core reset";
	case RESET_REASON_CORE_RTC_WDT:
		return "RTC WDT core reset";
	case RESET_REASON_CPU0_MWDT0:
		return "Main WDT0 CPU0 reset";
	case RESET_REASON_CPU0_SW:
		return "Software CPU0 reset";
	case RESET_REASON_CPU0_RTC_WDT:
		return "RTC WDT CPU0 reset";
	case RESET_REASON_SYS_BROWN_OUT:
		return "Brown-out core reset";
	case RESET_REASON_SYS_RTC_WDT:
		return "RTC WDT core and rtc reset";
	case RESET_REASON_CPU0_MWDT1:
		return "Main WDT1 resets CPU0";
	case RESET_REASON_SYS_SUPER_WDT:
		return "Super WDT reset";
	case RESET_REASON_SYS_CLK_GLITCH:
		return "Glitch on clock reset";
	case RESET_REASON_CORE_EFUSE_CRC:
		return "eFuse CRC error reset";
	}
	return "Unknown reset cause";
}

int esp32s2_reset_reason_fetch(struct target *target, int *rsn_id, const char **rsn_str)
{
	uint32_t rsn_val;

	int ret = target_read_u32(target, ESP32S2_RTC_CNTL_RESET_STATE_REG, &rsn_val);
	if (ret != ERROR_OK) {
		LOG_TARGET_ERROR(target, "Error on read reset reason register (%d)!", ret);
		return ret;
	}
	rsn_val &= ESP32S2_RTC_CNTL_RESET_CAUSE_PROCPU_M;
	rsn_val >>= ESP32S2_RTC_CNTL_RESET_CAUSE_PROCPU_S;

	/* sanity check for valid value */
	if (rsn_val == 0)
		return ERROR_FAIL;

	*rsn_id = rsn_val;
	*rsn_str = esp32s2_reset_reason_str(target->coreid, rsn_val);
	return ERROR_OK;
}

static int esp32s2_target_init(struct command_context *cmd_ctx, struct target *target)
{
	int ret = esp_xtensa_target_init(cmd_ctx, target);
	if (ret != ERROR_OK)
		return ret;

	return esp_xtensa_semihosting_init(target);
}

static const struct xtensa_debug_ops esp32s2_dbg_ops = {
	.queue_enable = xtensa_dm_queue_enable,
	.queue_reg_read = xtensa_dm_queue_reg_read,
	.queue_reg_write = xtensa_dm_queue_reg_write
};

static const struct xtensa_power_ops esp32s2_pwr_ops = {
	.queue_reg_read = xtensa_dm_queue_pwr_reg_read,
	.queue_reg_write = xtensa_dm_queue_pwr_reg_write
};

static const struct esp_flash_breakpoint_ops esp32s2_spec_brp_ops = {
	.breakpoint_add = esp_flash_breakpoint_add,
	.breakpoint_remove = esp_flash_breakpoint_remove
};

static const struct esp_semihost_ops esp32s2_semihost_ops = {
	.prepare = esp32s2_disable_wdts,
	.post_reset = esp_semihosting_post_reset
};

static int esp32s2_target_create(struct target *target, Jim_Interp *interp)
{
	struct xtensa_debug_module_config esp32s2_dm_cfg = {
		.dbg_ops = &esp32s2_dbg_ops,
		.pwr_ops = &esp32s2_pwr_ops,
		.tap = target->tap,
		.queue_tdi_idle = NULL,
		.queue_tdi_idle_arg = NULL
	};

	/* creates xtensa object */
	struct esp32s2_common *esp32 = calloc(1, sizeof(*esp32));
	if (!esp32) {
		LOG_ERROR("Failed to alloc memory for arch info!");
		return ERROR_FAIL;
	}

	struct esp_ops esp32s2_ops = {
		.flash_brps_ops = &esp32s2_spec_brp_ops,
		.chip_ops = NULL,
		.semihost_ops = &esp32s2_semihost_ops,
		.reset_reason_fetch = esp32s2_reset_reason_fetch
	};

	int ret = esp_xtensa_init_arch_info(target, &esp32->esp_xtensa, &esp32s2_xtensa_cfg,
		&esp32s2_dm_cfg, &esp32s2_ops);
	if (ret != ERROR_OK) {
		LOG_ERROR("Failed to init arch info!");
		free(esp32);
		return ret;
	}

	/* Assume running target. If different, the first poll will fix this */
	target->state = TARGET_RUNNING;
	target->debug_reason = DBG_REASON_NOTHALTED;
	return ERROR_OK;
}

static const struct command_registration esp_any_command_handlers[] = {
	{
		.mode = COMMAND_ANY,
		.usage = "",
		.chain = esp_command_handlers,
	},
	{
		.mode = COMMAND_ANY,
		.usage = "",
		.chain = esp32_apptrace_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration esp32s2_command_handlers[] = {
	{
		.name = "xtensa",
		.mode = COMMAND_ANY,
		.help = "Xtensa commands group",
		.usage = "",
		.chain = xtensa_command_handlers,
	},
	{
		.name = "esp",
		.mode = COMMAND_ANY,
		.help = "ESP command group",
		.usage = "",
		.chain = esp_any_command_handlers,
	},
	{
		.name = "arm",
		.mode = COMMAND_ANY,
		.help = "ARM Command Group",
		.usage = "",
		.chain = semihosting_common_handlers
	},
	COMMAND_REGISTRATION_DONE
};

/* Holds methods for Xtensa targets. */
struct target_type esp32s2_target = {
	.name = "esp32s2",

	.poll = esp32s2_poll,
	.arch_state = esp32s2_arch_state,

	.halt = xtensa_halt,
	.resume = xtensa_resume,
	.step = esp32s2_step,

	.assert_reset = esp32s2_assert_reset,
	.deassert_reset = esp32s2_deassert_reset,
	.soft_reset_halt = esp32s2_soft_reset_halt,

	.virt2phys = esp32s2_virt2phys,
	.mmu = xtensa_mmu_is_enabled,
	.read_memory = xtensa_read_memory,
	.write_memory = xtensa_write_memory,

	.read_buffer = xtensa_read_buffer,
	.write_buffer = xtensa_write_buffer,

	.checksum_memory = xtensa_checksum_memory,

	.get_gdb_arch = xtensa_get_gdb_arch,
	.get_gdb_reg_list = xtensa_get_gdb_reg_list,

	.run_algorithm = xtensa_run_algorithm,
	.start_algorithm = xtensa_start_algorithm,
	.wait_algorithm = xtensa_wait_algorithm,

	.add_breakpoint = esp_xtensa_breakpoint_add,
	.remove_breakpoint = esp_xtensa_breakpoint_remove,

	.add_watchpoint = xtensa_watchpoint_add,
	.remove_watchpoint = xtensa_watchpoint_remove,

	.target_create = esp32s2_target_create,
	.init_target = esp32s2_target_init,
	.examine = xtensa_examine,
	.deinit_target = esp_xtensa_target_deinit,

	.commands = esp32s2_command_handlers,
};
