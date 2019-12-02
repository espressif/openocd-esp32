/***************************************************************************
 *   ESP32-S2 target for OpenOCD                                           *
 *   Copyright (C) 2019 Espressif Systems Ltd.                             *
 *   Author: Alexey Gerenkov <alexey@espressif.com>                        *
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
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.           *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "target.h"
#include "target_type.h"
#include "assert.h"
#include "rtos/rtos.h"
#include "flash/nor/esp_xtensa.h"
#include "esp32_s2.h"
#include "esp32_apptrace.h"
#include "esp_xtensa.h"

/* Overall memory map
 * TODO: read memory configuration from target registers */
#define ESP32_S2_IRAM_LOW       0x40020000
#define ESP32_S2_IRAM_HIGH      0x40070000
#define ESP32_S2_RTC_IRAM_LOW   0x40070000
#define ESP32_S2_RTC_IRAM_HIGH  0x40072000
#define ESP32_S2_RTC_DRAM_LOW   0x3ff9e000
#define ESP32_S2_RTC_DRAM_HIGH  0x3ffa0000
#define ESP32_S2_DRAM_LOW       0x3FFB0000
#define ESP32_S2_DRAM_HIGH      0x3FFFFFFF
#define ESP32_S2_RTC_DATA_LOW   0x50000000
#define ESP32_S2_RTC_DATA_HIGH  0x50002000

/* ESP32 WDT */
#define ESP32_S2_WDT_WKEY_VALUE       0x50D83AA1
#define ESP32_S2_TIMG0_BASE           0x3f41F000
#define ESP32_S2_TIMG1_BASE           0x3f420000
#define ESP32_S2_TIMGWDT_CFG0_OFF     0x48
#define ESP32_S2_TIMGWDT_PROTECT_OFF  0x64
#define ESP32_S2_TIMG0WDT_CFG0        (ESP32_S2_TIMG0_BASE + ESP32_S2_TIMGWDT_CFG0_OFF)
#define ESP32_S2_TIMG1WDT_CFG0        (ESP32_S2_TIMG1_BASE + ESP32_S2_TIMGWDT_CFG0_OFF)
#define ESP32_S2_TIMG0WDT_PROTECT     (ESP32_S2_TIMG0_BASE + ESP32_S2_TIMGWDT_PROTECT_OFF)
#define ESP32_S2_TIMG1WDT_PROTECT     (ESP32_S2_TIMG1_BASE + ESP32_S2_TIMGWDT_PROTECT_OFF)
#define ESP32_S2_RTCCNTL_BASE         0x3f408000
#define ESP32_S2_RTCWDT_CFG_OFF       0x94
#define ESP32_S2_RTCWDT_PROTECT_OFF   0xAC
#define ESP32_S2_RTCWDT_CFG           (ESP32_S2_RTCCNTL_BASE + ESP32_S2_RTCWDT_CFG_OFF)
#define ESP32_S2_RTCWDT_PROTECT       (ESP32_S2_RTCCNTL_BASE + ESP32_S2_RTCWDT_PROTECT_OFF)

#define ESP32_S2_TRACEMEM_BLOCK_SZ      0x4000

#define ESP32_S2_DR_REG_UART_BASE       0x3f400000
#define ESP32_S2_REG_UART_BASE(i)             (ESP32_S2_DR_REG_UART_BASE + (i) * 0x10000 )
#define ESP32_S2_UART_DATE_REG(i)       (ESP32_S2_REG_UART_BASE(i) + 0x74)
#define ESP32_S2_CHIP_REV_REG                   ESP32_S2_UART_DATE_REG(0)
#define ESP32_S2_CHIP_REV_VAL                   0x19031400
#define ESP32_S2BETA_CHIP_REV_VAL               0x18082800


static const struct xtensa_config esp32_s2_xtensa_cfg = {
	.density        = true,
	.aregs_num      = XT_AREGS_NUM_MAX,
	.windowed       = true,
	.coproc         = true,
	.miscregs_num   = 4,
	.threadptr      = true,
	.gdb_regs_num   = ESP32_S2_NUM_REGS_G_COMMAND,
	.irom           = {
		.count = 1,
		.regions = {
			{
				.base = ESP32_S2_IROM_LOW,
				.size = ESP32_S2_IROM_HIGH-ESP32_S2_IROM_LOW,
				.access = XT_MEM_ACCESS_READ,
			},
		}
	},
	.iram           = {
		.count = 2,
		.regions = {
			{
				.base = ESP32_S2_IRAM_LOW,
				.size = ESP32_S2_IRAM_HIGH-ESP32_S2_IRAM_LOW,
				.access = XT_MEM_ACCESS_READ|XT_MEM_ACCESS_WRITE,
			},
			{
				.base = ESP32_S2_RTC_IRAM_LOW,
				.size = ESP32_S2_RTC_IRAM_HIGH-ESP32_S2_RTC_IRAM_LOW,
				.access = XT_MEM_ACCESS_READ|XT_MEM_ACCESS_WRITE,
			},
		}
	},
	.drom           = {
		.count = 1,
		.regions = {
			{
				.base = ESP32_S2_DROM_LOW,
				.size = ESP32_S2_DROM_HIGH-ESP32_S2_DROM_LOW,
				.access = XT_MEM_ACCESS_READ,
			},
		}
	},
	.dram           = {
		.count = 3,
		.regions = {
			{
				.base = ESP32_S2_DRAM_LOW,
				.size = ESP32_S2_DRAM_HIGH-ESP32_S2_DRAM_LOW,
				.access = XT_MEM_ACCESS_READ|XT_MEM_ACCESS_WRITE,
			},
			{
				.base = ESP32_S2_RTC_DRAM_LOW,
				.size = ESP32_S2_RTC_DRAM_HIGH-ESP32_S2_RTC_DRAM_LOW,
				.access = XT_MEM_ACCESS_READ|XT_MEM_ACCESS_WRITE,
			},
			{
				.base = ESP32_S2_RTC_DATA_LOW,
				.size = ESP32_S2_RTC_DATA_HIGH-ESP32_S2_RTC_DATA_LOW,
				.access = XT_MEM_ACCESS_READ|XT_MEM_ACCESS_WRITE,
			},
		}
	},
	.exc           = {
		.enabled = true,
	},
	.irq           = {
		.enabled = true,
		.irq_num = 32,
	},
	.high_irq      = {
		.enabled = true,
		.excm_level = 3,
		.nmi_num = 1,
	},
	.tim_irq      = {
		.enabled = true,
		.comp_num = 3,
	},
	.debug         = {
		.enabled = true,
		.irq_level = 6,
		.ibreaks_num = 2,
		.dbreaks_num = 2,
		.icount_sz = 32,
	},
	.trace         = {
		.enabled = true,
		.mem_sz = ESP32_S2_TRACEMEM_BLOCK_SZ,
	},
};

static int esp32_s2_soc_reset(struct target *target);

static int esp32_s2_assert_reset(struct target *target)
{
	LOG_DEBUG("%s: begin", target_name(target));

	/* Reset the SoC first */
	int res = esp32_s2_soc_reset(target);
	if (res != ERROR_OK)
		return res;
	return xtensa_assert_reset(target);
}

/* Reset ESP32-S2's peripherals.
Postconditions: all peripherals except RTC_CNTL are reset, CPU's PC is undefined, PRO CPU is halted, APP CPU is in reset
How this works:
0. make sure target is halted; if not, try to halt it; if that fails, try to reset it (via OCD) and then halt
1. set CPU initial PC to 0x50000000 (ESP32_S2_RTC_DATA_LOW) by clearing RTC_CNTL_{PRO,APP}CPU_STAT_VECTOR_SEL
2. load stub code into ESP32_S2_RTC_DATA_LOW; once executed, stub code will disable watchdogs and make CPU spin in an idle loop.
3. trigger SoC reset using RTC_CNTL_SW_SYS_RST bit
4. wait for the OCD to be reset
5. halt the target and wait for it to be halted (at this point CPU is in the idle loop)
6. restore initial PC and the contents of ESP32_S2_RTC_DATA_LOW
TODO: some state of RTC_CNTL is not reset during SW_SYS_RST. Need to reset that manually.
*/
static int esp32_s2_soc_reset(struct target *target)
{
	int res;

	LOG_DEBUG("start");
	/* In order to write to peripheral registers, target must be halted first */
	if (target->state != TARGET_HALTED) {
		LOG_DEBUG("%s: Target not halted before SoC reset, trying to halt it first",
			__func__);
		xtensa_halt(target);
		res = target_wait_state(target, TARGET_HALTED, 1000);
		if (res != ERROR_OK) {
			LOG_DEBUG(
				"%s: Couldn't halt target before SoC reset, trying to do reset-halt",
				__func__);
			res = xtensa_assert_reset(target);
			if (res != ERROR_OK) {
				LOG_ERROR(
					"%s: Couldn't halt target before SoC reset! (xtensa_assert_reset returned %d)",
					__func__,
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
				LOG_ERROR(
					"%s: Couldn't halt target before SoC reset! (xtensa_deassert_reset returned %d)",
					__func__,
					res);
				return res;
			}
			alive_sleep(10);
			xtensa_poll(target);
			xtensa_halt(target);
			res = target_wait_state(target, TARGET_HALTED, 1000);
			if (res != ERROR_OK) {
				LOG_ERROR("%s: Couldn't halt target before SoC reset", __func__);
				return res;
			}
		}
	}

	assert(target->state == TARGET_HALTED);

	/* This this the stub code compiled from esp32_cpu_reset_handler.S.
	   To compile it, run:
	       xtensa-esp32s2-elf-gcc -c -mtext-section-literals -o stub.o esp32s2_cpu_reset_handler.S
	       xtensa-esp32s2-elf-objcopy -j .text -O binary stub.o stub.bin
	   These steps are not included into OpenOCD build process so that a
	   dependency on xtensa-esp32s2-elf toolchain is not introduced.
	*/
	const uint32_t esp32_reset_stub_code[] = {
		0x00001B06,
		0x00001106,0x3F408038, 0x3F4080C0,0x3F4080C4,
		0x3F408074,0x01583218, 0x9C492000,0x3F408000,
		0x50D83AA1,0x3F4080AC, 0x3F41F064,0x3F420064,
		0x3F408094,0x3F41F048, 0x3F420048,0x3F4C10E0,
		0x3F408038,0x00003000, 0x41305550,0x0459FFEE,
		0x59FFEE41,0xFFED4104, 0xED410459,0xFFED31FF,
		0xED310439,0xFFED41FF, 0x00000439,0x31305550,
		0xEC41FFEC,0x410439FF, 0x0439FFEC,0x39FFEC41,
		0xFFEB4104,0xEB410459, 0x410459FF,0x0459FFEB,
		0x59FFEB41,0xFFEA4104, 0x39FFEB31,0x00700004,
		0x00FFFE46
	};

	LOG_DEBUG("loading stub code into RTC RAM");
	uint32_t slow_mem_save[sizeof(esp32_reset_stub_code) / sizeof(uint32_t)];

	/* Save contents of ESP32_S2_RTC_DATA_LOW which we are about to overwrite */
	res =
		target_read_buffer(target,
		ESP32_S2_RTC_DATA_LOW,
		sizeof(slow_mem_save),
		(uint8_t *)slow_mem_save);
	if (res != ERROR_OK) {
		LOG_ERROR("%s %d err=%d", __func__, __LINE__, res);
		return res;
	}

	/* Write stub code into ESP32_S2_RTC_DATA_LOW */
	res =
		target_write_buffer(target, ESP32_S2_RTC_DATA_LOW,
		sizeof(esp32_reset_stub_code),
		(const uint8_t *)esp32_reset_stub_code);
	if (res != ERROR_OK) {
		LOG_ERROR("%s %d err=%d", __func__, __LINE__, res);
		return res;
	}

	LOG_DEBUG("resuming the target");
	struct xtensa *xtensa = target_to_xtensa(target);
	xtensa->suppress_dsr_errors = true;
	res = xtensa_resume(target, 0, ESP32_S2_RTC_DATA_LOW + 4, 0, 0);
	if (res != ERROR_OK) {
		LOG_WARNING("%s xtensa_resume err=%d", __func__, res);
		return res;
	}
	xtensa->suppress_dsr_errors = false;
	LOG_DEBUG("resume done, waiting for the target to come alive");

	/* Wait for SoC to reset */
	alive_sleep(100);
	int timeout = 100;
	while (target->state != TARGET_RESET && target->state !=
		TARGET_RUNNING && --timeout > 0) {
		alive_sleep(10);
		xtensa_poll(target);
	}
	if (timeout == 0) {
		LOG_ERROR("%s: Timed out waiting for CPU to be reset, target->state=%d",
			__func__,
			target->state);
		return ERROR_TARGET_TIMEOUT;
	}

	/* Halt the CPU again */
	LOG_DEBUG("halting the target");
	xtensa_halt(target);
	res = target_wait_state(target, TARGET_HALTED, 1000);
	if (res != ERROR_OK) {
		LOG_ERROR("%s: Timed out waiting for CPU to be halted after SoC reset", __func__);
		return res;
	}

	/* Restore the original contents of ESP32_S2_RTC_DATA_LOW */
	LOG_DEBUG("restoring ESP32_S2_RTC_DATA_LOW");
	res =
		target_write_buffer(target, ESP32_S2_RTC_DATA_LOW, sizeof(slow_mem_save),
		(const uint8_t *)slow_mem_save);
	if (res != ERROR_OK) {
		LOG_ERROR("%s %d err=%d", __func__, __LINE__, res);
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

static int esp32_s2_disable_wdts(struct target *target)
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
	return ERROR_OK;
}

static int esp32_s2_arch_state(struct target *target)
{
	return ERROR_OK;
}

static bool esp32_s2_on_halt(struct target *target)
{
	struct esp32_s2_common *esp32 = target_to_esp32_s2(target);
	uint32_t val = (uint32_t)-1;

	int ret = esp32_s2_disable_wdts(target);
	if (ret != ERROR_OK)
		return false;

	if (esp32->chip_rev == ESP32_S2_REV_UNKNOWN) {
		ret = xtensa_read_buffer(target,
			ESP32_S2_CHIP_REV_REG,
			sizeof(val),
			(uint8_t *)&val);
		if (ret != ERROR_OK)
			LOG_ERROR("Failed to read chip revision register (%d)!", ret);
		LOG_DEBUG("Chip ver 0x%x", val);
		if (val == ESP32_S2_CHIP_REV_VAL) {
			LOG_INFO("Detected ESP32-S2 chip");
			esp32->chip_rev = ESP32_S2_REV_0;
		} else if (val == ESP32_S2BETA_CHIP_REV_VAL) {
			LOG_INFO("Detected ESP32-S2-Beta chip");
			esp32->chip_rev = ESP32_S2_REV_BETA;
		} else
			LOG_WARNING("Unknown ESP32-S2 chip revision (0x%x)!", val);
	}

	return esp_xtensa_on_halt(target);
}

static int esp32_s2_virt2phys(struct target *target,
	target_addr_t virtual, target_addr_t *physical)
{
	*physical = virtual;
	return ERROR_OK;
}

static int esp32_s2_target_init(struct command_context *cmd_ctx, struct target *target)
{
	int ret = esp_xtensa_target_init(cmd_ctx, target);
	if (ret != ERROR_OK)
		return ret;
	return ERROR_OK;
}

static const struct xtensa_debug_ops esp32_s2_dbg_ops = {
	.queue_enable = xtensa_dm_queue_enable,
	.queue_reg_read = xtensa_dm_queue_reg_read,
	.queue_reg_write = xtensa_dm_queue_reg_write
};

static const struct xtensa_power_ops esp32_s2_pwr_ops = {
	.queue_reg_read = xtensa_dm_queue_pwr_reg_read,
	.queue_reg_write = xtensa_dm_queue_pwr_reg_write
};

static const struct esp_xtensa_special_breakpoint_ops esp32_s2_spec_brp_ops = {
	.breakpoint_add = esp_xtensa_flash_breakpoint_add,
	.breakpoint_remove = esp_xtensa_flash_breakpoint_remove
};

static const struct xtensa_chip_ops esp32_s2_chip_ops = {
	.on_reset = esp_xtensa_on_reset,
	.on_poll = esp_xtensa_on_poll,
	.on_halt = esp32_s2_on_halt,
};

static int esp32_s2_target_create(struct target *target, Jim_Interp *interp)
{
	struct xtensa_debug_module_config esp32_s2_dm_cfg = {
		.dbg_ops = &esp32_s2_dbg_ops,
		.pwr_ops = &esp32_s2_pwr_ops,
		.tap = target->tap,
		.queue_tdi_idle = NULL,
		.queue_tdi_idle_arg = NULL
	};

	struct esp32_s2_common *esp32 = calloc(1, sizeof(struct esp32_s2_common));
	if (esp32 == NULL) {
		LOG_ERROR("Failed to alloc memory for arch info!");
		return ERROR_FAIL;
	}

	int ret = esp_xtensa_init_arch_info(target, target, esp32, &esp32_s2_xtensa_cfg,
		&esp32_s2_dm_cfg, &esp32_s2_chip_ops, &esp32_s2_spec_brp_ops);
	if (ret != ERROR_OK) {
		LOG_ERROR("Failed to init arch info!");
		free(esp32);
		return ret;
	}
	esp32->chip_rev = ESP32_S2_REV_UNKNOWN;

	/*Assume running target. If different, the first poll will fix this. */
	target->state = TARGET_RUNNING;
	target->debug_reason = DBG_REASON_NOTHALTED;
	return ERROR_OK;
}

static const struct command_registration esp32_s2_command_handlers[] = {
	{
		.name = "esp32_s2",
		.mode = COMMAND_ANY,
		.help = "ESP32-S2 commands group",
		.usage = "",
		.chain = xtensa_command_handlers,
	},
	{
		.name = "esp32_s2",
		.mode = COMMAND_ANY,
		.help = "ESP32-S2 command group",
		.usage = "",
		.chain = esp_xtensa_command_handlers,
	},
	{
		.name = "esp32_s2",
		.mode = COMMAND_ANY,
		.help = "ESP32-S2 apptrace commands group",
		.usage = "",
		.chain = esp32_apptrace_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

/** Holds methods for Xtensa targets. */
struct target_type esp32_s2_target = {
	.name = "esp32_s2",

	.poll = xtensa_poll,
	.arch_state = esp32_s2_arch_state,

	.halt = xtensa_halt,
	.resume = xtensa_resume,
	.step = xtensa_step,

	.assert_reset = esp32_s2_assert_reset,
	.deassert_reset = xtensa_deassert_reset,

	.virt2phys = esp32_s2_virt2phys,
	.mmu = xtensa_mmu_is_enabled,
	.read_memory = xtensa_read_memory,
	.write_memory = xtensa_write_memory,

	.read_buffer = xtensa_read_buffer,
	.write_buffer = xtensa_write_buffer,

	.checksum_memory = xtensa_checksum_memory,

	.get_gdb_reg_list = xtensa_get_gdb_reg_list,

	.run_algorithm = xtensa_run_algorithm,
	.start_algorithm = xtensa_start_algorithm,
	.wait_algorithm = xtensa_wait_algorithm,

	.add_breakpoint = esp_xtensa_breakpoint_add,
	.remove_breakpoint = esp_xtensa_breakpoint_remove,

	.add_watchpoint = xtensa_watchpoint_add,
	.remove_watchpoint = xtensa_watchpoint_remove,

	.target_create = esp32_s2_target_create,
	.init_target = esp32_s2_target_init,
	.examine = xtensa_examine,

	.commands = esp32_s2_command_handlers,
};
