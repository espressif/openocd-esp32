/***************************************************************************
 *   ESP32-C2 target for OpenOCD                                           *
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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "esp32c2.h"
#include <helper/command.h>
#include <helper/bits.h>
#include <target/target_type.h>
#include <target/register.h>
#include <target/semihosting_common.h>
#include "esp_semihosting.h"
#include <target/riscv/debug_defines.h>
#include "esp32_apptrace.h"
#include <rtos/rtos.h>

/* ESP32-C2 memory map */
#define ESP32C2_IRAM_LOW      0x40380000
#define ESP32C2_IRAM_HIGH     0x403C0000
#define ESP32C2_DRAM_LOW      0x3FCA0000
#define ESP32C2_DRAM_HIGH     0x3FCE0000

/* ESP32-C2 WDT */
#define ESP32C2_WDT_WKEY_VALUE                 0x50d83aa1
#define ESP32C2_TIMG0_BASE                     0x6001F000
#define ESP32C2_TIMGWDT_CFG0_OFF               0x48
#define ESP32C2_TIMGWDT_PROTECT_OFF            0x64
#define ESP32C2_TIMG0WDT_CFG0                  (ESP32C2_TIMG0_BASE + ESP32C2_TIMGWDT_CFG0_OFF)
#define ESP32C2_TIMG0WDT_PROTECT               (ESP32C2_TIMG0_BASE + ESP32C2_TIMGWDT_PROTECT_OFF)
#define ESP32C2_RTCCNTL_BASE                   0x60008000
#define ESP32C2_RTCWDT_CFG_OFF                 0x84
#define ESP32C2_RTCWDT_PROTECT_OFF             0x9C
#define ESP32C2_RTCWDT_CFG                     (ESP32C2_RTCCNTL_BASE + ESP32C2_RTCWDT_CFG_OFF)
#define ESP32C2_RTCWDT_PROTECT                 (ESP32C2_RTCCNTL_BASE + ESP32C2_RTCWDT_PROTECT_OFF)
#define ESP32C2_RTCCNTL_RESET_STATE_OFF        0x0030
#define ESP32C2_RTCCNTL_RESET_STATE_REG        (ESP32C2_RTCCNTL_BASE + ESP32C2_RTCCNTL_RESET_STATE_OFF)

#define ESP32C2_GPIO_BASE                      0x60004000
#define ESP32C2_GPIO_STRAP_REG_OFF             0x0038
#define ESP32C2_GPIO_STRAP_REG                 (ESP32C2_GPIO_BASE + ESP32C2_GPIO_STRAP_REG_OFF)
#define IS_1XXX(v)                             (((v) & 0x08) == 0x08)
#define ESP32C2_IS_FLASH_BOOT(_r_)             IS_1XXX(_r_)
#define ESP32C2_FLASH_BOOT_MODE                0x08

#define ESP32C2_RTCCNTL_RESET_CAUSE_MASK       (BIT(6) - 1)
#define ESP32C2_RESET_CAUSE(reg_val)           ((reg_val) & ESP32C2_RTCCNTL_RESET_CAUSE_MASK)

/* ESP32-C2 PMP Config and address */
#define ESP32C2_PMP_CFG_IRAM_START             0
#define ESP32C2_PMP_CFG_IRAM_END               1
#define ESP32C2_PMP_CFG_DRAM_START             2
#define ESP32C2_PMP_CFG_DRAM_END               3
#define ESP32C2_PMP_ADDR_IRAM_START            (CSR_PMPADDR0 + ESP32C2_PMP_CFG_IRAM_START)
#define ESP32C2_PMP_ADDR_IRAM_END              (CSR_PMPADDR0 + ESP32C2_PMP_CFG_IRAM_END)
#define ESP32C2_PMP_ADDR_DRAM_START            (CSR_PMPADDR0 + ESP32C2_PMP_CFG_DRAM_START)
#define ESP32C2_PMP_ADDR_DRAM_END              (CSR_PMPADDR0 + ESP32C2_PMP_CFG_DRAM_END)
#define ESP32C2_PMP_LOCKED_BIT                 BIT(7)

typedef enum {
	RESET_REASON_CHIP_POWER_ON = 0x01,	/* Power on reset */
	RESET_REASON_CORE_SW = 0x03,	/* Software resets the digital core by RTC_CNTL_SW_SYS_RST */
	RESET_REASON_CORE_DEEP_SLEEP = 0x05,	/* Deep sleep reset the digital core */
	RESET_REASON_CORE_MWDT0 = 0x07,	/* Main watch dog 0 resets digital core */
	RESET_REASON_CORE_RTC_WDT = 0x09,	/* RTC watch dog resets digital core */
	RESET_REASON_CPU0_MWDT0 = 0x0B,	/* Main watch dog 0 resets CPU 0 */
	RESET_REASON_CPU0_SW = 0x0C,	/* Software resets CPU 0 by RTC_CNTL_SW_PROCPU_RST */
	RESET_REASON_CPU0_RTC_WDT = 0x0D,	/* RTC watch dog resets CPU 0 */
	RESET_REASON_SYS_BROWN_OUT = 0x0F,	/* VDD voltage is not stable and resets the digital core */
	RESET_REASON_SYS_RTC_WDT = 0x10,/* RTC watch dog resets digital core and rtc module */
	RESET_REASON_SYS_SUPER_WDT = 0x12,	/* Super watch dog resets the digital core and rtc module */
	RESET_REASON_SYS_CLK_GLITCH = 0x13,	/* Glitch on clock resets the digital core and rtc module */
	RESET_REASON_CORE_EFUSE_CRC = 0x14,	/* eFuse CRC error resets the digital core */
	RESET_REASON_CPU0_JTAG = 0x18,	/* JTAG resets the CPU 0 */
} soc_reset_reason_t;

static const char *esp32c2_get_reset_reason(soc_reset_reason_t reset_number)
{
	switch (ESP32C2_RESET_CAUSE(reset_number)) {
	case RESET_REASON_CHIP_POWER_ON:
		return "Power on reset";
	case RESET_REASON_CORE_SW:
		return "Software core reset";
	case RESET_REASON_CORE_DEEP_SLEEP:
		return "Deep-sleep core reset";
	case RESET_REASON_CORE_MWDT0:
		return "Main WDT0 core reset";
	case RESET_REASON_CORE_RTC_WDT:
		return "RTC WDT core reset";
	case RESET_REASON_CPU0_MWDT0:
		return "Main WDT0 CPU Reset";
	case RESET_REASON_CPU0_SW:
		return "Software CPU Reset";
	case RESET_REASON_CPU0_RTC_WDT:
		return "RTC WDT CPU Reset";
	case RESET_REASON_SYS_BROWN_OUT:
		return "Brown-out core reset";
	case RESET_REASON_SYS_RTC_WDT:
		return "RTC WDT core and rtc reset";
	case RESET_REASON_SYS_SUPER_WDT:
		return "Super WDT core and rtc reset";
	case RESET_REASON_SYS_CLK_GLITCH:
		return "CLK GLITCH core and rtc reset";
	case RESET_REASON_CORE_EFUSE_CRC:
		return "eFuse CRC error core reset";
	case RESET_REASON_CPU0_JTAG:
		return "JTAG CPU reset";
	}
	return "Unknown reset cause";
}

extern struct target_type riscv_target;
extern const struct command_registration riscv_command_handlers[];

static int esp32c2_on_reset(struct target *target);

static int esp32c2_wdt_disable(struct target *target)
{
	/* TIMG1 WDT */
	int res = target_write_u32(target, ESP32C2_TIMG0WDT_PROTECT, ESP32C2_WDT_WKEY_VALUE);
	if (res != ERROR_OK) {
		LOG_ERROR("Failed to write ESP32C2_TIMG0WDT_PROTECT (%d)!", res);
		return res;
	}
	res = target_write_u32(target, ESP32C2_TIMG0WDT_CFG0, 0);
	if (res != ERROR_OK) {
		LOG_ERROR("Failed to write ESP32C2_TIMG0WDT_CFG0 (%d)!", res);
		return res;
	}
	/* RTC WDT */
	res = target_write_u32(target, ESP32C2_RTCWDT_PROTECT, ESP32C2_WDT_WKEY_VALUE);
	if (res != ERROR_OK) {
		LOG_ERROR("Failed to write ESP32C2_RTCWDT_PROTECT (%d)!", res);
		return res;
	}
	res = target_write_u32(target, ESP32C2_RTCWDT_CFG, 0);
	if (res != ERROR_OK) {
		LOG_ERROR("Failed to write ESP32C2_RTCWDT_CFG (%d)!", res);
		return res;
	}
	return ERROR_OK;
}

static const struct esp_semihost_ops esp32c2_semihost_ops = {
	.prepare = esp32c2_wdt_disable
};

static const struct esp_flash_breakpoint_ops esp32c2_flash_brp_ops = {
	.breakpoint_add = esp_flash_breakpoint_add,
	.breakpoint_remove = esp_flash_breakpoint_remove
};

static int esp32c2_handle_target_event(struct target *target, enum target_event event, void *priv)
{
	if (target != priv)
		return ERROR_OK;

	LOG_DEBUG("%d", event);

	int ret = esp_riscv_handle_target_event(target, event, priv);
	if (ret != ERROR_OK)
		return ret;

	return ERROR_OK;
}

static int esp32c2_target_create(struct target *target, Jim_Interp *interp)
{
	struct esp32c2_common *esp32c2 = calloc(1, sizeof(*esp32c2));
	if (!esp32c2)
		return ERROR_FAIL;

	target->arch_info = esp32c2;

	riscv_info_init(target, &esp32c2->esp_riscv.riscv);

	return ERROR_OK;
}

static int esp32c2_init_target(struct command_context *cmd_ctx,
	struct target *target)
{
	int ret = riscv_target.init_target(cmd_ctx, target);
	if (ret != ERROR_OK)
		return ret;

	target->semihosting->user_command_extension = esp_semihosting_common;

	struct esp32c2_common *esp32c2 = esp32c2_common(target);

	ret = esp_riscv_init_arch_info(cmd_ctx,
		target,
		&esp32c2->esp_riscv,
		esp32c2_on_reset,
		&esp32c2_flash_brp_ops,
		&esp32c2_semihost_ops);
	if (ret != ERROR_OK)
		return ret;

	ret = target_register_event_callback(esp32c2_handle_target_event, target);
	if (ret != ERROR_OK)
		return ret;

	return ERROR_OK;
}

static void esp32c2_deinit_target(struct target *target)
{
	riscv_target.deinit_target(target);
}

static const char *const s_nonexistent_regs[] = {
	"mie", "mip", "tdata3", "uie", "utvt", "utvec", "vcsr", "uscratch", "utval",
	"uip", "unxti", "uintstatus", "uscratchcsw", "uscratchcswl", "sedeleg",
	"sideleg", "stvt", "snxti", "sintstatus", "sscratchcsw", "sscratchcswl",
	"vsstatus", "vsie", "vstvec", "vsscratch", "vsepc", "vscause", "vstval",
	"vsip", "vsatp", "mtvt", "mstatush", "mcountinhibit", "mnxti", "mintstatus",
	"mscratchcsw", "mscratchcswl", "mtinst", "mtval2", "hstatus", "hedeleg",
	"hideleg", "hie", "htimedelta", "hcounteren", "hgeie", "htimedeltah",
	"htval", "hip", "hvip", "htinst", "hgatp", "hgeip", "mvendorid", "marchid",
	"mimpid", "mhartid", "seed", "mcounteren", "mhpmevent3", "mhpmevent4", "mhpmevent5",
	"mhpmevent6", "mhpmevent7", "mhpmevent8", "mhpmevent9", "mhpmevent10", "mhpmevent11",
	"mhpmevent12", "mhpmevent13", "mhpmevent14", "mhpmevent15", "mhpmevent16", "mhpmevent17",
	"mhpmevent18", "mhpmevent19", "mhpmevent20", "mhpmevent21", "mhpmevent22", "mhpmevent23",
	"mhpmevent24", "mhpmevent25", "mhpmevent26", "mhpmevent27", "mhpmevent28", "mhpmevent29",
	"mhpmevent30", "mhpmevent31",
	"scontext", "hcontext", "tinfo", "mcontext", "mscontext", "mcycle", "minstret",
	"mhpmcounter3", "mhpmcounter4", "mhpmcounter5", "mhpmcounter6", "mhpmcounter7",
	"mhpmcounter8", "mhpmcounter9", "mhpmcounter10", "mhpmcounter11", "mhpmcounter12",
	"mhpmcounter13", "mhpmcounter14", "mhpmcounter15", "mhpmcounter16", "mhpmcounter17",
	"mhpmcounter18", "mhpmcounter19", "mhpmcounter20", "mhpmcounter21", "mhpmcounter22",
	"mhpmcounter23", "mhpmcounter24", "mhpmcounter25", "mhpmcounter26", "mhpmcounter27",
	"mhpmcounter28", "mhpmcounter29", "mhpmcounter30", "mhpmcounter31", "mcycleh",
	"minstreth", "mhpmcounter3h", "mhpmcounter4h", "mhpmcounter5h", "mhpmcounter6h",
	"mhpmcounter7h", "mhpmcounter8h", "mhpmcounter9h", "mhpmcounter10h", "mhpmcounter11h",
	"mhpmcounter12h", "mhpmcounter13h", "mhpmcounter14h", "mhpmcounter15h", "mhpmcounter16h",
	"mhpmcounter17h", "mhpmcounter18h", "mhpmcounter19h", "mhpmcounter20h", "mhpmcounter21h",
	"mhpmcounter22h", "mhpmcounter23h", "mhpmcounter24h", "mhpmcounter25h", "mhpmcounter26h",
	"mhpmcounter27h", "mhpmcounter28h", "mhpmcounter29h", "mhpmcounter30h", "mhpmcounter31h",
	"cycle", "time", "instret", "hpmcounter3", "hpmcounter4", "hpmcounter5", "hpmcounter6",
	"hpmcounter7", "hpmcounter8", "hpmcounter9", "hpmcounter10", "hpmcounter11", "hpmcounter12",
	"hpmcounter13", "hpmcounter14", "hpmcounter15", "hpmcounter17", "hpmcounter18",
	"hpmcounter19",
	"hpmcounter20", "hpmcounter21", "hpmcounter22", "hpmcounter23", "hpmcounter24",
	"hpmcounter25",
	"hpmcounter26", "hpmcounter27", "hpmcounter28", "hpmcounter29", "hpmcounter30",
	"hpmcounter31",
	"cycleh", "timeh", "instreth", "hpmcounter3h", "hpmcounter4h", "hpmcounter5h",
	"hpmcounter6h",
	"hpmcounter7h", "hpmcounter8h", "hpmcounter9h", "hpmcounter10h", "hpmcounter11h",
	"hpmcounter12h",
	"hpmcounter13h", "hpmcounter14h", "hpmcounter15h", "hpmcounter17h", "hpmcounter18h",
	"hpmcounter19h",
	"hpmcounter20h", "hpmcounter21h", "hpmcounter22h", "hpmcounter23h", "hpmcounter24h",
	"hpmcounter25h",
	"hpmcounter26h", "hpmcounter27h", "hpmcounter28h", "hpmcounter29h", "hpmcounter30h",
	"hpmcounter31h",
	"hpmcounter16h", "mhpmevent4"
};

static int esp32c2_examine(struct target *target)
{
	int ret = riscv_target.examine(target);
	if (ret != ERROR_OK)
		return ret;

	/* RISCV code initializes registers upon target examination.
	   disable some registers because their reading or writing causes exception. Not supported in ESP32-C3??? */
	for (size_t i = 0; i < ARRAY_SIZE(s_nonexistent_regs); i++) {
		struct reg *r = register_get_by_name(target->reg_cache, s_nonexistent_regs[i], 1);
		if (r)
			r->exist = false;
	}
	return ERROR_OK;
}

static int esp32c2_on_reset(struct target *target)
{
	LOG_DEBUG("esp32c3_on_reset!");
	struct esp32c2_common *esp32c2 = esp32c2_common(target);
	esp32c2->was_reset = true;
	return ERROR_OK;
}

static int esp32c2_poll(struct target *target)
{
	struct esp32c2_common *esp32c2 = esp32c2_common(target);
	int res = ERROR_OK;

	RISCV_INFO(r);
	if (esp32c2->was_reset && r->dmi_read && r->dmi_write) {
		uint32_t dmstatus;
		res = r->dmi_read(target, &dmstatus, DM_DMSTATUS);
		if (res != ERROR_OK) {
			LOG_ERROR("Failed to read DMSTATUS (%d)!", res);
		} else {
			uint32_t strap_reg;
			LOG_DEBUG("Core is out of reset: dmstatus 0x%x", dmstatus);
			esp32c2->was_reset = false;
			res = target_read_u32(target, ESP32C2_GPIO_STRAP_REG, &strap_reg);
			if (res != ERROR_OK) {
				LOG_WARNING("Failed to read ESP32C3_GPIO_STRAP_REG (%d)!", res);
				strap_reg = ESP32C2_FLASH_BOOT_MODE;
			}

			uint32_t reset_buffer = 0;
			res = target_read_u32(target,
				ESP32C2_RTCCNTL_RESET_STATE_REG,
				&reset_buffer);
			if (res != ERROR_OK) {
				LOG_WARNING("Failed to read read reset cause register (%d)!", res);
			} else {
				LOG_INFO("Reset cause (%ld) - (%s)",
					(ESP32C2_RESET_CAUSE(reset_buffer)),
					esp32c2_get_reset_reason((reset_buffer)));
			}

			if (ESP32C2_IS_FLASH_BOOT(strap_reg) &&
				get_field(dmstatus, DM_DMSTATUS_ALLHALTED) == 0) {
				LOG_TARGET_DEBUG(target, "Halt core");
				res = esp_riscv_core_halt(target);
				if (res == ERROR_OK) {
					res = esp32c2_wdt_disable(target);
					if (res != ERROR_OK)
						LOG_ERROR("Failed to disable WDTs (%d)!", res);
				} else {
					LOG_ERROR("Failed to halt core (%d)!", res);
				}
			}
			/* Clear memory which is used by RTOS layer to get the task count */
			if (target->rtos && target->rtos->type->post_reset_cleanup) {
				res = (*target->rtos->type->post_reset_cleanup)(target);
				if (res != ERROR_OK)
					LOG_WARNING("Failed to do rtos-specific cleanup (%d)", res);
			}
			if (ESP32C2_IS_FLASH_BOOT(strap_reg)) {
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
	return esp_riscv_poll(target);
}

int esp32c2_memprot_is_enabled(struct command_invocation *cmd)
{
	struct target *target = get_current_target(CMD_CTX);

	if (!target) {
		LOG_ERROR("No target selected");
		return ERROR_FAIL;
	}
	riscv_reg_t pmpcfg0, pmpaddr;
	bool enabled = false;
	int res;
	RISCV_INFO(r);
	while (1) {
		res = r->get_register(target, &pmpcfg0, CSR_PMPCFG0 + GDB_REGNO_CSR0);
		if (res == ERROR_OK) {
			uint8_t buf[4];
			buf_set_u32(buf, 0, 32, pmpcfg0);
			if (buf[ESP32C2_PMP_CFG_IRAM_START] & ESP32C2_PMP_LOCKED_BIT) {
				res = r->get_register(target, &pmpaddr, ESP32C2_PMP_ADDR_IRAM_START + GDB_REGNO_CSR0);
				if (res == ERROR_OK) {
					if ((pmpaddr << 2) > ESP32C2_IRAM_LOW) {
						enabled = true;
						break;
					}
				}
			}
			if (buf[ESP32C2_PMP_CFG_IRAM_END] & ESP32C2_PMP_LOCKED_BIT) {
				res = r->get_register(target, &pmpaddr, ESP32C2_PMP_ADDR_IRAM_END + GDB_REGNO_CSR0);
				if (res == ERROR_OK) {
					if ((pmpaddr << 2) < ESP32C2_IRAM_HIGH) {
						enabled = true;
						break;
					}
				}
			}
			if (buf[ESP32C2_PMP_CFG_DRAM_START] & ESP32C2_PMP_LOCKED_BIT) {
				res = r->get_register(target, &pmpaddr, ESP32C2_PMP_ADDR_DRAM_START + GDB_REGNO_CSR0);
				if (res == ERROR_OK) {
					if ((pmpaddr << 2) > ESP32C2_DRAM_LOW) {
						enabled = true;
						break;
					}
				}
			}
			if (buf[ESP32C2_PMP_CFG_DRAM_END] & ESP32C2_PMP_LOCKED_BIT) {
				res = r->get_register(target, &pmpaddr, ESP32C2_PMP_ADDR_DRAM_END + GDB_REGNO_CSR0);
				if (res == ERROR_OK) {
					if ((pmpaddr << 2) < ESP32C2_DRAM_HIGH) {
						enabled = true;
						break;
					}
				}
			}
		}
		break;
	}

	command_print(CMD, "%d", enabled);

	return res;
}

static const struct command_registration esp32c2_target_command_handlers[] = {
	{
		.name = "memprot_stat",
		.handler = esp32c2_memprot_is_enabled,
		.mode = COMMAND_ANY,
		.help = "returns memory protection is enabled or not",
		.usage = "memprot_stat"
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration esp32c2_command_handlers[] = {
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
	{
		.name = "esp32c2",
		.usage = "",
		.chain = esp32c2_target_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

struct target_type esp32c2_target = {
	.name = "esp32c2",

	.target_create = esp32c2_target_create,
	.init_target = esp32c2_init_target,
	.deinit_target = esp32c2_deinit_target,
	.examine = esp32c2_examine,

	/* poll current target status */
	.poll = esp32c2_poll,

	.halt = esp_riscv_halt,
	.resume = esp_riscv_resume,
	.step = esp_riscv_step,

	.assert_reset = esp_riscv_assert_reset,
	.deassert_reset = esp_riscv_deassert_reset,

	.read_memory = esp_riscv_read_memory,
	.write_memory = esp_riscv_write_memory,

	.checksum_memory = esp_riscv_checksum_memory,

	.get_gdb_arch = esp_riscv_get_gdb_arch,
	.get_gdb_reg_list = esp_riscv_get_gdb_reg_list,
	.get_gdb_reg_list_noread = esp_riscv_get_gdb_reg_list_noread,

	.add_breakpoint = esp_riscv_breakpoint_add,
	.remove_breakpoint = esp_riscv_breakpoint_remove,

	.add_watchpoint = esp_riscv_add_watchpoint,
	.remove_watchpoint = esp_riscv_remove_watchpoint,
	.hit_watchpoint = esp_riscv_hit_watchpoint,

	.arch_state = esp_riscv_arch_state,

	.run_algorithm = esp_riscv_run_algorithm,
	.start_algorithm = esp_riscv_start_algorithm,
	.wait_algorithm = esp_riscv_wait_algorithm,

	.commands = esp32c2_command_handlers,

	.address_bits = esp_riscv_address_bits,
};
