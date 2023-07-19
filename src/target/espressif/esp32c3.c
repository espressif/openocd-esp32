/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   ESP32-C3 target for OpenOCD                                           *
 *   Copyright (C) 2020 Espressif Systems Ltd.                             *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "esp32c3.h"
#include <helper/command.h>
#include <helper/bits.h>
#include <target/target_type.h>
#include <target/register.h>
#include <target/semihosting_common.h>
#include "esp_semihosting.h"
#include <target/riscv/debug_defines.h>
#include "esp32_apptrace.h"
#include "rtos/rtos.h"

/* ESP32-C3 WDT */
#define ESP32C3_WDT_WKEY_VALUE                  0x50d83aa1
#define ESP32C3_TIMG0_BASE                      0x6001F000
#define ESP32C3_TIMG1_BASE                      0x60020000
#define ESP32C3_TIMGWDT_CFG0_OFF                0x48
#define ESP32C3_TIMGWDT_PROTECT_OFF             0x64
#define ESP32C3_TIMG0WDT_CFG0                   (ESP32C3_TIMG0_BASE + ESP32C3_TIMGWDT_CFG0_OFF)
#define ESP32C3_TIMG1WDT_CFG0                   (ESP32C3_TIMG1_BASE + ESP32C3_TIMGWDT_CFG0_OFF)
#define ESP32C3_TIMG0WDT_PROTECT                (ESP32C3_TIMG0_BASE + ESP32C3_TIMGWDT_PROTECT_OFF)
#define ESP32C3_TIMG1WDT_PROTECT                (ESP32C3_TIMG1_BASE + ESP32C3_TIMGWDT_PROTECT_OFF)
#define ESP32C3_RTCCNTL_BASE                    0x60008000
#define ESP32C3_RTCWDT_CFG_OFF                  0x90
#define ESP32C3_RTCWDT_PROTECT_OFF              0xa8
#define ESP32C3_RTCWDT_CFG                      (ESP32C3_RTCCNTL_BASE + ESP32C3_RTCWDT_CFG_OFF)
#define ESP32C3_RTCWDT_PROTECT                  (ESP32C3_RTCCNTL_BASE + ESP32C3_RTCWDT_PROTECT_OFF)
#define ESP32C3_RTCCNTL_RESET_STATE_OFF         0x0038
#define ESP32C3_RTCCNTL_RESET_STATE_REG         (ESP32C3_RTCCNTL_BASE + ESP32C3_RTCCNTL_RESET_STATE_OFF)

#define ESP32C3_GPIO_BASE                       0x60004000
#define ESP32C3_GPIO_STRAP_REG_OFF              0x0038
#define ESP32C3_GPIO_STRAP_REG                  (ESP32C3_GPIO_BASE + ESP32C3_GPIO_STRAP_REG_OFF)
#define IS_1XXX(v)                              (((v) & 0x08) == 0x08)
#define ESP32C3_IS_FLASH_BOOT(_r_)              IS_1XXX(_r_)
#define ESP32C3_FLASH_BOOT_MODE                 0x08

#define ESP32C3_RTCCNTL_RESET_CAUSE_MASK        (BIT(6) - 1)
#define ESP32C3_RESET_CAUSE(reg_val)            ((reg_val) & ESP32C3_RTCCNTL_RESET_CAUSE_MASK)

/* max supported hw breakpoint and watchpoint count */
#define ESP32C3_BP_NUM                          8
#define ESP32C3_WP_NUM                          8

/* ASSIST_DEBUG registers */
#define ESP32C3_ASSIST_DEBUG_CPU0_MON_REG       0x600CE000

enum esp32c3_reset_reason {
	ESP32C3_CHIP_POWER_ON_RESET      = 0x01,	/* Power on reset */
	ESP32C3_CHIP_BROWN_OUT_RESET     = 0x01,	/* VDD voltage is not stable and resets the chip */
	ESP32C3_CHIP_SUPER_WDT_RESET     = 0x01,	/* Super watch dog resets the chip */
	ESP32C3_CORE_SW_RESET            = 0x03,	/* Software resets the digital core by RTC_CNTL_SW_SYS_RST */
	ESP32C3_CORE_DEEP_SLEEP_RESET    = 0x05,	/* Deep sleep reset the digital core */
	ESP32C3_CORE_MWDT0_RESET         = 0x07,	/* Main watch dog 0 resets digital core */
	ESP32C3_CORE_MWDT1_RESET         = 0x08,	/* Main watch dog 1 resets digital core */
	ESP32C3_CORE_RTC_WDT_RESET       = 0x09,	/* RTC watch dog resets digital core */
	ESP32C3_CPU0_MWDT0_RESET         = 0x0B,	/* Main watch dog 0 resets CPU 0 */
	ESP32C3_CPU0_SW_RESET            = 0x0C,	/* Software resets CPU 0 by RTC_CNTL_SW_PROCPU_RST */
	ESP32C3_CPU0_RTC_WDT_RESET       = 0x0D,	/* RTC watch dog resets CPU 0 */
	ESP32C3_SYS_BROWN_OUT_RESET      = 0x0F,	/* VDD voltage is not stable and resets the digital core */
	ESP32C3_SYS_RTC_WDT_RESET        = 0x10,	/* RTC watch dog resets digital core and rtc module */
	ESP32C3_CPU0_MWDT1_RESET         = 0x11,	/* Main watch dog 1 resets CPU 0 */
	ESP32C3_SYS_SUPER_WDT_RESET      = 0x12,	/* Super watch dog resets the digital core and rtc module */
	ESP32C3_SYS_CLK_GLITCH_RESET     = 0x13,	/* Glitch on clock resets the digital core and rtc module */
	ESP32C3_CORE_EFUSE_CRC_RESET     = 0x14,	/* eFuse CRC error resets the digital core */
	ESP32C3_CORE_USB_UART_RESET      = 0x15,	/* USB UART resets the digital core */
	ESP32C3_CORE_USB_JTAG_RESET      = 0x16,	/* USB JTAG resets the digital core */
	ESP32C3_CORE_PWR_GLITCH_RESET    = 0x17,	/* Glitch on power resets the digital core */
};

static const char *esp32c3_get_reset_reason(enum esp32c3_reset_reason reset_number)
{
	switch (ESP32C3_RESET_CAUSE(reset_number)) {
	case ESP32C3_CHIP_POWER_ON_RESET:
		/* case ESP32C3_CHIP_BROWN_OUT_RESET:
		 * case ESP32C3_CHIP_SUPER_WDT_RESET: */
		return "Chip reset";
	case ESP32C3_CORE_SW_RESET:
		return "Software core reset";
	case ESP32C3_CORE_DEEP_SLEEP_RESET:
		return "Deep-sleep core reset";
	case ESP32C3_CORE_MWDT0_RESET:
		return "Main WDT0 core reset";
	case ESP32C3_CORE_MWDT1_RESET:
		return "Main WDT1 core reset";
	case ESP32C3_CORE_RTC_WDT_RESET:
		return "RTC WDT core reset";
	case ESP32C3_CPU0_MWDT0_RESET:
		return "Main WDT0 CPU reset";
	case ESP32C3_CPU0_SW_RESET:
		return "Software CPU reset";
	case ESP32C3_CPU0_RTC_WDT_RESET:
		return "RTC WDT CPU reset";
	case ESP32C3_SYS_BROWN_OUT_RESET:
		return "Brown-out core reset";
	case ESP32C3_SYS_RTC_WDT_RESET:
		return "RTC WDT core and rtc reset";
	case ESP32C3_CPU0_MWDT1_RESET:
		return "Main WDT1 CPU reset";
	case ESP32C3_SYS_SUPER_WDT_RESET:
		return "Super Watchdog core and rtc";
	case ESP32C3_SYS_CLK_GLITCH_RESET:
		return "CLK GLITCH core and rtc reset";
	case ESP32C3_CORE_EFUSE_CRC_RESET:
		return "eFuse CRC error core reset";
	case ESP32C3_CORE_USB_UART_RESET:
		return "USB (UART) core reset";
	case ESP32C3_CORE_USB_JTAG_RESET:
		return "USB (JTAG) core reset";
	case ESP32C3_CORE_PWR_GLITCH_RESET:
		return "Power glitch core reset";
	}
	return "Unknown reset cause";
}

static int esp32c3_on_reset(struct target *target);

static int esp32c3_wdt_disable(struct target *target)
{
	/* TIMG1 WDT */
	int res = target_write_u32(target, ESP32C3_TIMG0WDT_PROTECT, ESP32C3_WDT_WKEY_VALUE);
	if (res != ERROR_OK) {
		LOG_ERROR("Failed to write ESP32C3_TIMG0WDT_PROTECT (%d)!", res);
		return res;
	}
	res = target_write_u32(target, ESP32C3_TIMG0WDT_CFG0, 0);
	if (res != ERROR_OK) {
		LOG_ERROR("Failed to write ESP32C3_TIMG0WDT_CFG0 (%d)!", res);
		return res;
	}
	/* TIMG2 WDT */
	res = target_write_u32(target, ESP32C3_TIMG1WDT_PROTECT, ESP32C3_WDT_WKEY_VALUE);
	if (res != ERROR_OK) {
		LOG_ERROR("Failed to write ESP32C3_TIMG1WDT_PROTECT (%d)!", res);
		return res;
	}
	res = target_write_u32(target, ESP32C3_TIMG1WDT_CFG0, 0);
	if (res != ERROR_OK) {
		LOG_ERROR("Failed to write ESP32C3_TIMG1WDT_CFG0 (%d)!", res);
		return res;
	}
	/* RTC WDT */
	res = target_write_u32(target, ESP32C3_RTCWDT_PROTECT, ESP32C3_WDT_WKEY_VALUE);
	if (res != ERROR_OK) {
		LOG_ERROR("Failed to write ESP32C3_RTCWDT_PROTECT (%d)!", res);
		return res;
	}
	res = target_write_u32(target, ESP32C3_RTCWDT_CFG, 0);
	if (res != ERROR_OK) {
		LOG_ERROR("Failed to write ESP32C3_RTCWDT_CFG (%d)!", res);
		return res;
	}
	return ERROR_OK;
}

static const struct esp_semihost_ops esp32c3_semihost_ops = {
	.prepare = esp32c3_wdt_disable,
	.post_reset = esp_semihosting_post_reset
};

static const struct esp_flash_breakpoint_ops esp32c3_flash_brp_ops = {
	.breakpoint_add = esp_algo_flash_breakpoint_add,
	.breakpoint_remove = esp_algo_flash_breakpoint_remove
};

static int esp32c3_target_create(struct target *target, Jim_Interp *interp)
{
	struct esp32c3_common *esp32c3 = calloc(1, sizeof(*esp32c3));
	if (!esp32c3)
		return ERROR_FAIL;

	target->arch_info = esp32c3;

	esp32c3->esp_riscv.assist_debug_cpu0_mon_reg = ESP32C3_ASSIST_DEBUG_CPU0_MON_REG;
	esp32c3->esp_riscv.assist_debug_cpu_offset = 0;

	esp32c3->esp_riscv.max_bp_num = ESP32C3_BP_NUM;
	esp32c3->esp_riscv.max_wp_num = ESP32C3_WP_NUM;

	if (esp_riscv_alloc_trigger_addr(target) != ERROR_OK)
		return ERROR_FAIL;

	riscv_info_init(target, &esp32c3->esp_riscv.riscv);

	return ERROR_OK;
}

static int esp32c3_init_target(struct command_context *cmd_ctx,
	struct target *target)
{
	int ret = riscv_target.init_target(cmd_ctx, target);
	if (ret != ERROR_OK)
		return ret;

	target->semihosting->user_command_extension = esp_semihosting_common;

	struct esp32c3_common *esp32c3 = esp32c3_common(target);

	ret = esp_riscv_init_arch_info(cmd_ctx,
		target,
		&esp32c3->esp_riscv,
		esp32c3_on_reset,
		&esp32c3_flash_brp_ops,
		&esp32c3_semihost_ops);
	if (ret != ERROR_OK)
		return ret;

	return ERROR_OK;
}

static const char *const s_existent_regs[] = {
	"zero", "ra", "sp", "gp", "tp", "t0", "t1", "t2", "t3", "t4", "t5", "t6",
	"fp", "pc", "mstatus", "misa", "mtvec", "mscratch", "mepc", "mcause", "mtval", "priv",
	"s1", "s2", "s3", "s4", "s5", "s6", "s7", "s8", "s9", "s10", "s11",
	"a0", "a1", "a2", "a3", "a4", "a5", "a6", "a7",
	"pmpcfg0", "pmpcfg1", "pmpcfg2", "pmpcfg3",
	"pmpaddr0", "pmpaddr1", "pmpaddr2", "pmpaddr3", "pmpaddr4", "pmpaddr5", "pmpaddr6", "pmpaddr7",
	"pmpaddr8", "pmpaddr9", "pmpaddr10", "pmpaddr11", "pmpaddr12", "pmpaddr13", "pmpaddr14", "pmpaddr15",
	"tselect", "tdata1", "tdata2", "tcontrol", "dcsr", "dpc", "dscratch0", "dscratch1", "hpmcounter16",
	/* custom exposed CSRs will start with 'csr_' prefix*/
	"csr_mpcer", "csr_mpcmr", "csr_mpccr", "csr_cpu_gpio_oen", "csr_cpu_gpio_in", "csr_cpu_gpio_out",
};

static int esp32c3_examine(struct target *target)
{
	int ret = riscv_target.examine(target);
	if (ret != ERROR_OK)
		return ret;
	/* RISCV code initializes registers upon target examination.
	   disable some registers because their reading or writing causes exception. Not supported in ESP32-C3??? */
	for (unsigned int i = 0; i < target->reg_cache->num_regs; i++) {
		if (target->reg_cache->reg_list[i].exist) {
			target->reg_cache->reg_list[i].exist = false;
			for (unsigned int j = 0; j < ARRAY_SIZE(s_existent_regs); j++)
				if (!strcmp(target->reg_cache->reg_list[i].name, s_existent_regs[j])) {
					target->reg_cache->reg_list[i].exist = true;
					break;
				}
		}
	}
	return ERROR_OK;
}

static int esp32c3_on_reset(struct target *target)
{
	LOG_DEBUG("esp32c3_on_reset!");
	struct esp32c3_common *esp32c3 = esp32c3_common(target);
	esp32c3->was_reset = true;
	return ERROR_OK;
}

static int esp32c3_poll(struct target *target)
{
	struct esp32c3_common *esp32c3 = esp32c3_common(target);
	int res = ERROR_OK;

	RISCV_INFO(r);
	if (esp32c3->was_reset && r->dmi_read && r->dmi_write) {
		uint32_t dmstatus;
		res = r->dmi_read(target, &dmstatus, DM_DMSTATUS);
		if (res != ERROR_OK) {
			LOG_ERROR("Failed to read DMSTATUS (%d)!", res);
		} else {
			uint32_t strap_reg;
			LOG_DEBUG("Core is out of reset: dmstatus 0x%x", dmstatus);
			esp32c3->was_reset = false;
			res = target_read_u32(target, ESP32C3_GPIO_STRAP_REG, &strap_reg);
			if (res != ERROR_OK) {
				LOG_WARNING("Failed to read ESP32C3_GPIO_STRAP_REG (%d)!", res);
				strap_reg = ESP32C3_FLASH_BOOT_MODE;
			}
			uint32_t reset_buffer = 0;
			res = target_read_u32(target, ESP32C3_RTCCNTL_RESET_STATE_REG, &reset_buffer);
			if (res != ERROR_OK) {
				LOG_WARNING("Failed to read read reset cause register (%d)!", res);
			} else {
				LOG_INFO("Reset cause (%ld) - (%s)", (ESP32C3_RESET_CAUSE(reset_buffer)),
					esp32c3_get_reset_reason((reset_buffer)));
			}

			if (ESP32C3_IS_FLASH_BOOT(strap_reg) && get_field(dmstatus, DM_DMSTATUS_ALLHALTED) == 0) {
				LOG_DEBUG("Halt core");
				res = esp_riscv_core_halt(target);
				if (res == ERROR_OK) {
					res = esp32c3_wdt_disable(target);
					if (res != ERROR_OK)
						LOG_ERROR("Failed to disable WDTs (%d)!", res);
				} else {
					LOG_ERROR("Failed to halt core (%d)!", res);
				}
			}

			if (esp32c3->esp_riscv.semi_ops->post_reset)
				esp32c3->esp_riscv.semi_ops->post_reset(target);
			/* Clear memory which is used by RTOS layer to get the task count */
			if (target->rtos && target->rtos->type->post_reset_cleanup) {
				res = (*target->rtos->type->post_reset_cleanup)(target);
				if (res != ERROR_OK)
					LOG_WARNING("Failed to do rtos-specific cleanup (%d)", res);
			}
			/* clear previous apptrace ctrl_addr to avoid invalid tracing control block usage in the long
			 *run. */
			esp32c3->esp_riscv.apptrace.ctrl_addr = 0;

			if (ESP32C3_IS_FLASH_BOOT(strap_reg)) {
				/* enable ebreaks */
				res = esp_riscv_core_ebreaks_enable(target);
				if (res != ERROR_OK)
					LOG_ERROR("Failed to enable EBREAKS handling (%d)!", res);
				if (get_field(dmstatus, DM_DMSTATUS_ALLHALTED) == 0) {
					LOG_DEBUG("Resume core");
					res = esp_riscv_core_resume(target);
					if (res != ERROR_OK)
						LOG_ERROR("Failed to resume core (%d)!", res);
					LOG_DEBUG("resumed core");
				}
			}
		}
	}
	return riscv_openocd_poll(target);
}

static const struct command_registration esp32c3_command_handlers[] = {
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

struct target_type esp32c3_target = {
	.name = "esp32c3",

	.target_create = esp32c3_target_create,
	.init_target = esp32c3_init_target,
	.deinit_target = esp_riscv_deinit_target,
	.examine = esp32c3_examine,

	/* poll current target status */
	.poll = esp32c3_poll,

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

	.commands = esp32c3_command_handlers,

	.address_bits = riscv_xlen_nonconst,
};
