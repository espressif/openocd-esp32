/***************************************************************************
 *   ESP32 target API for OpenOCD                                          *
 *   Copyright (C) 2016-2019 Espressif Systems Ltd.                        *
 *   Author: Dmitry Yakovlev <dmitry@espressif.com>                        *
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
#include "esp32.h"
#include "esp32_apptrace.h"
#include "esp_xtensa.h"

/*
This is a JTAG driver for the ESP32, the are two Tensilica cores inside
the ESP32 chip. For more information please have a look into ESP32 target
implementation.
*/

/*
Multiprocessor stuff common:

The ESP32 has two ESP32 processors in it, which can run in SMP-mode if an
SMP-capable OS is running. The hardware has a few features which make
debugging this much easier.

First of all, there's something called a 'break network', consisting of a
BreakIn input  and a BreakOut output on each CPU. The idea is that as soon
as a CPU goes into debug mode for whatever reason, it'll signal that using
its DebugOut pin. This signal is connected to the other CPU's DebugIn
input, causing this CPU also to go into debugging mode. To resume execution
when using only this break network, we will need to manually resume both
CPUs.

An alternative to this is the XOCDMode output and the RunStall (or DebugStall)
input. When these are cross-connected, a CPU that goes into debug mode will
halt execution entirely on the other CPU. Execution on the other CPU can be
resumed by either the first CPU going out of debug mode, or the second CPU
going into debug mode: the stall is temporarily lifted as long as the stalled
CPU is in debug mode.

A third, separate, signal is CrossTrigger. This is connected in the same way
as the breakIn/breakOut network, but is for the TRAX (trace memory) feature;
it does not affect OCD in any way.
*/

/*
ESP32 Multiprocessor stuff:

The ESP32 chip has two Xtensa cores inside, but represent themself to the OCD
as one chip that works in multithreading mode under FreeRTOS OS.
The core that initiate the stop condition will be defined as an active cpu.
When one core stops, then other core will be stoped automativally by smpbreak.
The core that initiate stop condition will be defined as an active core, and
registers of this core will be transfered.
*/


/* ESP32 memory map */
#define ESP32_EXT_RAM_LOW          0x3F800000
#define ESP32_EXT_RAM_HIGH         0x3FC00000
#define ESP32_DPORT_LOW            0x3ff00000
#define ESP32_DPORT_HIGH           0x3ff80000
#define ESP32_DRAM_LOW             0x3ffA0000
#define ESP32_DRAM_HIGH            0x40000000
#define ESP32_IRAM00_LOW           0x40000000
#define ESP32_IRAM00_HIGH          0x40070000
#define ESP32_IRAM02_LOW           0x40070000
#define ESP32_IRAM02_HIGH          0x400C0000
#define ESP32_RTC_IRAM_LOW         0x400C0000
#define ESP32_RTC_IRAM_HIGH        0x400C2000
#define ESP32_RTC_DATA_LOW         0x50000000
#define ESP32_RTC_DATA_HIGH        0x50002000

/* ESP32 WDT */
#define ESP32_WDT_WKEY_VALUE       0x50D83AA1
#define ESP32_TIMG0_BASE           0x3ff5F000
#define ESP32_TIMG1_BASE           0x3ff60000
#define ESP32_TIMGWDT_CFG0_OFF     0x48
#define ESP32_TIMGWDT_PROTECT_OFF  0x64
#define ESP32_TIMG0WDT_CFG0        (ESP32_TIMG0_BASE + ESP32_TIMGWDT_CFG0_OFF)
#define ESP32_TIMG1WDT_CFG0        (ESP32_TIMG1_BASE + ESP32_TIMGWDT_CFG0_OFF)
#define ESP32_TIMG0WDT_PROTECT     (ESP32_TIMG0_BASE + ESP32_TIMGWDT_PROTECT_OFF)
#define ESP32_TIMG1WDT_PROTECT     (ESP32_TIMG1_BASE + ESP32_TIMGWDT_PROTECT_OFF)
#define ESP32_RTCCNTL_BASE         0x3ff48000
#define ESP32_RTCWDT_CFG_OFF       0x8C
#define ESP32_RTCWDT_PROTECT_OFF   0xA4
#define ESP32_RTCWDT_CFG           (ESP32_RTCCNTL_BASE + ESP32_RTCWDT_CFG_OFF)
#define ESP32_RTCWDT_PROTECT       (ESP32_RTCCNTL_BASE + ESP32_RTCWDT_PROTECT_OFF)

#define ESP32_TRACEMEM_BLOCK_SZ    0x4000

/* ESP32 dport regs */
#define ESP32_DR_REG_DPORT_BASE         0x3ff00000
#define ESP32_DPORT_APPCPU_CTRL_B_REG   (ESP32_DR_REG_DPORT_BASE + 0x030)
#define ESP32_DPORT_APPCPU_CLKGATE_EN   (1 << 0)
/* ESP32 RTC regs */
#define ESP32_DR_REG_RTCCNTL_BASE       0x3ff48000
#define ESP32_RTC_CNTL_SW_CPU_STALL_REG (ESP32_DR_REG_RTCCNTL_BASE + 0xac)
#define ESP32_RTC_CNTL_SW_CPU_STALL_DEF 0x0


static const struct xtensa_config esp32_xtensa_cfg = {
	.density        = true,
	.aregs_num      = XT_AREGS_NUM_MAX,
	.windowed       = true,
	.coproc         = true,
	.fp_coproc      = true,
	.loop           = true,
	.miscregs_num   = 4,
	.threadptr      = true,
	.boolean        = true,
	.cond_store     = true,
	.mac16          = true,
	.user_regs_num  = 4,
	.user_regs      =
	{XT_REG_IDX_EXPSTATE, XT_REG_IDX_F64R_LO, XT_REG_IDX_F64R_HI, XT_REG_IDX_F64S},
	.gdb_regs_num   = ESP32_NUM_REGS_G_COMMAND,
	.irom           = {
		.count = 2,
		.regions = {
			{
				.base = ESP32_IROM_LOW,
				.size = ESP32_IROM_HIGH-ESP32_IROM_LOW,
				.access = XT_MEM_ACCESS_READ,
			},
			{
				.base = ESP32_IRAM00_LOW,
				.size = ESP32_IRAM00_HIGH-ESP32_IRAM00_LOW,
				.access = XT_MEM_ACCESS_READ,
			},
		}
	},
	.iram           = {
		.count = 2,
		.regions = {
			{
				.base = ESP32_IRAM02_LOW,
				.size = ESP32_IRAM02_HIGH-ESP32_IRAM02_LOW,
				.access = XT_MEM_ACCESS_READ|XT_MEM_ACCESS_WRITE,
			},
			{
				.base = ESP32_RTC_IRAM_LOW,
				.size = ESP32_RTC_IRAM_HIGH-ESP32_RTC_IRAM_LOW,
				.access = XT_MEM_ACCESS_READ|XT_MEM_ACCESS_WRITE,
			},
		}
	},
	.drom           = {
		.count = 1,
		.regions = {
			{
				.base = ESP32_DROM_LOW,
				.size = ESP32_DROM_HIGH-ESP32_DROM_LOW,
				.access = XT_MEM_ACCESS_READ,
			},
		}
	},
	.dram           = {
		.count = 2,
		.regions = {
			{
				.base = ESP32_DRAM_LOW,
				.size = ESP32_DRAM_HIGH-ESP32_DRAM_LOW,
				.access = XT_MEM_ACCESS_READ|XT_MEM_ACCESS_WRITE,
			},
			{
				.base = ESP32_RTC_DATA_LOW,
				.size = ESP32_RTC_DATA_HIGH-ESP32_RTC_DATA_LOW,
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
		.mem_sz = ESP32_TRACEMEM_BLOCK_SZ,
		.reversed_mem_access = true,
	},
};

static int esp32_soc_reset(struct xtensa_mcore_common *xtensa_mcore);

static int esp32_assert_reset(struct target *target)
{
	struct xtensa_mcore_common *xtensa_mcore = target_to_xtensa_mcore(target);

	LOG_DEBUG("%s: begin", target_name(target));

	/* Reset the SoC first. It is enough to run the reset func on PRO CPU */
	xtensa_mcore->active_core = 0;
	int res = esp32_soc_reset(xtensa_mcore);
	if (res != ERROR_OK)
		return res;
	return xtensa_mcore_assert_reset(target);
}

/* Reset ESP32's peripherals.
 * 1. OpenOCD makes sure the target is halted; if not, tries to halt it.
 *    If that fails, tries to reset it (via OCD) and then halt.
 * 2. OpenOCD loads the stub code into RTC_SLOW_MEM.
 * 3. Executes the stub code from address 0x50000004.
 * 4. The stub code changes the reset vector to 0x50000000, and triggers
 *    a system reset using RTC_CNTL_SW_SYS_RST bit.
 * 5. Once the PRO CPU is out of reset, it executes the stub code from address 0x50000000.
 *    The stub code disables the watchdog, re-enables JTAG and the APP CPU,
 *    restores the reset vector, and enters an infinite loop.
 * 6. OpenOCD waits until it can talk to the OCD module again, then halts the target.
 * 7. OpenOCD restores the contents of RTC_SLOW_MEM.
 *
 * End result: all the peripherals except RTC_CNTL are reset, CPU's PC is undefined,
 * PRO CPU is halted, APP CPU is in reset.
 */
static int esp32_soc_reset(struct xtensa_mcore_common *xtensa_mcore)
{
	int res;
	struct target *active_core_target = &xtensa_mcore->cores_targets[xtensa_mcore->active_core];

	LOG_DEBUG("start");
	/* In order to write to peripheral registers, target must be halted first */
	if (active_core_target->state != TARGET_HALTED) {
		LOG_DEBUG("%s: Target not halted before SoC reset, trying to halt it first",
			__func__);
		xtensa_halt(active_core_target);
		res = target_wait_state(active_core_target, TARGET_HALTED, 1000);
		if (res != ERROR_OK) {
			LOG_DEBUG(
				"%s: Couldn't halt target before SoC reset, trying to do reset-halt",
				__func__);
			res = xtensa_assert_reset(active_core_target);
			if (res != ERROR_OK) {
				LOG_ERROR(
					"%s: Couldn't halt target before SoC reset! (xtensa_assert_reset returned %d)",
					__func__,
					res);
				return res;
			}
			alive_sleep(10);
			xtensa_poll(active_core_target);
			int reset_halt_save = active_core_target->reset_halt;
			active_core_target->reset_halt = 1;
			res = xtensa_deassert_reset(active_core_target);
			active_core_target->reset_halt = reset_halt_save;
			if (res != ERROR_OK) {
				LOG_ERROR(
					"%s: Couldn't halt target before SoC reset! (xtensa_deassert_reset returned %d)",
					__func__,
					res);
				return res;
			}
			alive_sleep(10);
			xtensa_poll(active_core_target);
			xtensa_halt(active_core_target);
			res = target_wait_state(active_core_target, TARGET_HALTED, 1000);
			if (res != ERROR_OK) {
				LOG_ERROR("%s: Couldn't halt target before SoC reset", __func__);
				return res;
			}
		}
	}
	assert(active_core_target->state == TARGET_HALTED);

	for (uint8_t i = 0; i < xtensa_mcore->configured_cores_num; i++) {
		struct xtensa *xtensa = target_to_xtensa(&xtensa_mcore->cores_targets[i]);
		/* if any of the cores is stalled unstall them */
		if (xtensa_dm_core_is_stalled(&xtensa->dbg_mod)) {
			uint32_t word = ESP32_RTC_CNTL_SW_CPU_STALL_DEF;
			LOG_DEBUG("%s: Unstall CPUs before SW reset!",
				target_name(&xtensa_mcore->cores_targets[i]));
			res = xtensa_write_buffer(active_core_target,
				ESP32_RTC_CNTL_SW_CPU_STALL_REG,
				sizeof(word),
				(uint8_t *)&word);
			if (res != ERROR_OK) {
				LOG_ERROR("%s: Failed to unstall CPUs before SW reset!",
					target_name(&xtensa_mcore->cores_targets[i]));
				return res;
			}
			break;	/* both cores are unstalled now, so exit the loop */
		}
	}

	/* This this the stub code compiled from esp32_cpu_reset_handler.S.
	   To compile it, run:
	       xtensa-esp32-elf-gcc -c -mtext-section-literals -o stub.o esp32_cpu_reset_handler.S
	       xtensa-esp32-elf-objcopy -j .text -O binary stub.o stub.bin
	   These steps are not included into OpenOCD build process so that a
	   dependency on xtensa-esp32-elf toolchain is not introduced.
	*/
	const uint32_t esp32_reset_stub_code[] = {
		0x00001e06, 0x00001406, 0x3ff48034, 0x3ff480b0,
		0x3ff480b4, 0x3ff48070, 0x00002210, 0x9c492000,
		0x3ff48000, 0x50d83aa1, 0x3ff480a4, 0x3ff5f064,
		0x3ff60064, 0x3ff4808c, 0x3ff5f048, 0x3ff60048,
		0x3ff5a1fc, 0x3ff00038, 0x3ff00030, 0x3ff0002c,
		0x3ff48034, 0x00003000, 0x41305550, 0x0459ffeb,
		0x59ffeb41, 0xffea4104, 0xea410459, 0xffea31ff,
		0xea310439, 0xffea41ff, 0x00000439, 0x6003eb60,
		0x66560461, 0x30555004, 0x41ffe731, 0x0439ffe7,
		0x39ffe741, 0xffe64104, 0xe6410439, 0x410459ff,
		0x0459ffe6, 0x59ffe641, 0xffe54104, 0xe5410459,
		0x410459ff, 0x130cffe5, 0xe4410439, 0x39130cff,
		0x41045904, 0xe331ffe3, 0x006432ff, 0x46007000,
		0x0000fffe,
	};

	LOG_DEBUG("loading stub code into RTC RAM");
	uint32_t slow_mem_save[sizeof(esp32_reset_stub_code) / sizeof(uint32_t)];

	const int RTC_SLOW_MEM_BASE = 0x50000000;
	/* Save contents of RTC_SLOW_MEM which we are about to overwrite */
	res =
		target_read_buffer(active_core_target,
		RTC_SLOW_MEM_BASE,
		sizeof(slow_mem_save),
		(uint8_t *)slow_mem_save);
	if (res != ERROR_OK) {
		LOG_ERROR("%s %d err=%d", __func__, __LINE__, res);
		return res;
	}

	/* Write stub code into RTC_SLOW_MEM */
	res =
		target_write_buffer(active_core_target, RTC_SLOW_MEM_BASE,
		sizeof(esp32_reset_stub_code),
		(const uint8_t *)esp32_reset_stub_code);
	if (res != ERROR_OK) {
		LOG_ERROR("%s %d err=%d", __func__, __LINE__, res);
		return res;
	}

	LOG_DEBUG("resuming the target");
	struct xtensa *xtensa = target_to_xtensa(active_core_target);
	xtensa->suppress_dsr_errors = true;
	res = xtensa_resume(active_core_target, 0, RTC_SLOW_MEM_BASE + 4, 0, 0);
	if (res != ERROR_OK) {
		LOG_WARNING("%s xtensa_resume err=%d", __func__, res);
		return res;
	}
	xtensa->suppress_dsr_errors = false;
	LOG_DEBUG("resume done, waiting for the target to come alive");

	/* Wait for SoC to reset */
	alive_sleep(100);
	int timeout = 100;
	while (active_core_target->state != TARGET_RESET && active_core_target->state !=
		TARGET_RUNNING && --timeout > 0) {
		alive_sleep(10);
		xtensa_poll(active_core_target);
	}
	if (timeout == 0) {
		LOG_ERROR("%s: Timed out waiting for CPU to be reset, target->state=%d",
			__func__,
			active_core_target->state);
		return ERROR_TARGET_TIMEOUT;
	}

	/* Halt the CPU again */
	LOG_DEBUG("halting the target");
	xtensa_halt(active_core_target);
	res = target_wait_state(active_core_target, TARGET_HALTED, 1000);
	if (res != ERROR_OK) {
		LOG_ERROR("%s: Timed out waiting for CPU to be halted after SoC reset", __func__);
		return res;
	}

	/* Restore the original contents of RTC_SLOW_MEM */
	LOG_DEBUG("restoring RTC_SLOW_MEM");
	res =
		target_write_buffer(active_core_target, RTC_SLOW_MEM_BASE, sizeof(slow_mem_save),
		(const uint8_t *)slow_mem_save);
	if (res != ERROR_OK) {
		LOG_ERROR("%s %d err=%d", __func__, __LINE__, res);
		return res;
	}

	/* Clear memory which is used by RTOS layer to get the task count */
	if (active_core_target->rtos && active_core_target->rtos->type->post_reset_cleanup) {
		res = (*active_core_target->rtos->type->post_reset_cleanup)(active_core_target);
		if (res != ERROR_OK)
			LOG_WARNING("Failed to do rtos-specific cleanup (%d)", res);
	}

	return ERROR_OK;
}

static int esp32_disable_wdts(struct target *target)
{
	/* TIMG1 WDT */
	int res = target_write_u32(target, ESP32_TIMG0WDT_PROTECT, ESP32_WDT_WKEY_VALUE);
	if (res != ERROR_OK) {
		LOG_ERROR("Failed to write ESP32_TIMG0WDT_PROTECT (%d)!", res);
		return res;
	}
	res = target_write_u32(target, ESP32_TIMG0WDT_CFG0, 0);
	if (res != ERROR_OK) {
		LOG_ERROR("Failed to write ESP32_TIMG0WDT_CFG0 (%d)!", res);
		return res;
	}
	/* TIMG2 WDT */
	res = target_write_u32(target, ESP32_TIMG1WDT_PROTECT, ESP32_WDT_WKEY_VALUE);
	if (res != ERROR_OK) {
		LOG_ERROR("Failed to write ESP32_TIMG1WDT_PROTECT (%d)!", res);
		return res;
	}
	res = target_write_u32(target, ESP32_TIMG1WDT_CFG0, 0);
	if (res != ERROR_OK) {
		LOG_ERROR("Failed to write ESP32_TIMG1WDT_CFG0 (%d)!", res);
		return res;
	}
	/* RTC WDT */
	res = target_write_u32(target, ESP32_RTCWDT_PROTECT, ESP32_WDT_WKEY_VALUE);
	if (res != ERROR_OK) {
		LOG_ERROR("Failed to write ESP32_RTCWDT_PROTECT (%d)!", res);
		return res;
	}
	res = target_write_u32(target, ESP32_RTCWDT_CFG, 0);
	if (res != ERROR_OK) {
		LOG_ERROR("Failed to write ESP32_RTCWDT_CFG (%d)!", res);
		return res;
	}
	return ERROR_OK;
}

static bool esp32_is_app_cpu_enabled(struct target *target)
{
	uint32_t appcpu_ctrl = 0;
	int res;

	LOG_DEBUG("Read cores number");
	res =
		target_read_memory(target, ESP32_DPORT_APPCPU_CTRL_B_REG, sizeof(uint32_t), 1,
		(uint8_t *)&appcpu_ctrl);
	if (res != ERROR_OK) {
		LOG_ERROR("Failed to read ESP32_DPORT_APPCPU_CTRL_B_REG (%d)!", res);
		return false;
	}
	LOG_DEBUG("Read APP CPU ctrl reg 0x%x", appcpu_ctrl);
	if (appcpu_ctrl & ESP32_DPORT_APPCPU_CLKGATE_EN) {
		LOG_DEBUG("APP CPU enabled");
		return true;
	}
	return false;
}

static size_t esp32_get_enabled_cores_count(struct target *target)
{
	struct esp32_common *esp32 = target_to_esp32(target);
	if (esp32 == NULL)
		return ESP32_MAX_CORES_NUM;
	return xtensa_mcore_get_enabled_cores_count(target);
}

static int esp32_arch_state(struct target *target)
{
	return ERROR_OK;
}

static void esp32_on_poll(struct target *target)
{
	struct xtensa_mcore_common *xtensa_mcore = target_to_xtensa_mcore(target);

	for (size_t i = 0; i < xtensa_mcore->configured_cores_num; i++)
		esp_xtensa_on_poll(&xtensa_mcore->cores_targets[i]);
}

static bool esp32_on_halt(struct target *target)
{
	struct xtensa_mcore_common *xtensa_mcore = target_to_xtensa_mcore(target);

	int ret = esp32_disable_wdts(target);
	if (ret != ERROR_OK)
		return false;
	/* do some stuff generic for ESP Xtensa (semihosting) */
	return esp_xtensa_on_halt(&xtensa_mcore->cores_targets[xtensa_mcore->active_core]);
}

static int esp32_virt2phys(struct target *target,
	target_addr_t virtual, target_addr_t *physical)
{
	*physical = virtual;
	return ERROR_OK;
}

static int esp32_handle_target_event(struct target *target, enum target_event event, void *priv)
{
	struct xtensa_mcore_common *xtensa_mcore = target_to_xtensa_mcore(target);
	enum target_state old_state = target->state;

	if (target != priv)
		return ERROR_OK;

	LOG_DEBUG("%d", event);
	switch (event) {
		case TARGET_EVENT_HALTED:
			if (xtensa_mcore->configured_cores_num > 1 &&
			esp32_is_app_cpu_enabled(target))
				xtensa_mcore->cores_num = ESP32_MAX_CORES_NUM;
			else
				xtensa_mcore->cores_num = 1;
			LOG_DEBUG("Detected %d cores", xtensa_mcore->cores_num);
			break;
		default:
			break;
	}

	for (size_t i = 0; i < xtensa_mcore->configured_cores_num; i++)
		target_call_event_callbacks(&xtensa_mcore->cores_targets[i], event);

	if (event == TARGET_EVENT_GDB_DETACH && old_state == TARGET_RUNNING) {
		/* the target was stopped by esp_xtensa_handle_target_event(), but not resumed because
		 esp32_xtensa_core_resume() does nothing
		 TODO: remove this hack when normal OpenOCD SMP mechanism is supported */
		int ret = target_resume(target, 1, 0, 1, 0);
		if (ret != ERROR_OK) {
			LOG_ERROR(
				"%s: Failed to resume target after flash BPs removal (%d)!",
				target_name(target),
				ret);
			return ret;
		}
	}

	return ERROR_OK;
}

static int esp32_target_init(struct command_context *cmd_ctx, struct target *target)
{
	struct xtensa_mcore_common *xtensa_mcore = target_to_xtensa_mcore(target);

	int ret = target_register_event_callback(esp32_handle_target_event, target);
	if (ret != ERROR_OK)
		return ret;
	ret = xtensa_mcore_target_init(cmd_ctx, target);
	if (ret != ERROR_OK)
		return ret;
	xtensa_mcore->smp_break = OCDDCR_BREAKINEN|OCDDCR_BREAKOUTEN;
	return ERROR_OK;
}

static int esp32_xtensa_core_resume(struct target *target,
	int current,
	target_addr_t address,
	int handle_breakpoints,
	int debug_execution)
{
	/* done via core_prepare_resume() and core_do_resume() from
	 * esp32_xtensa_mcores_ops */
	return ERROR_OK;
}

static int esp32_xtensa_core_step(struct target *target,
	int current,
	target_addr_t address,
	int handle_breakpoints)
{
	/* done via core_do_step() from esp32_xtensa_mcores_ops */
	return ERROR_OK;
}

/** Special target type for ESP32 cores with minimal funcs support */
static struct target_type esp32_xtensa_core_target_type = {
	.name = "esp32_core",

	.poll = xtensa_poll,
	.arch_state = esp_xtensa_arch_state,

	.assert_reset = xtensa_assert_reset,
	.deassert_reset = xtensa_deassert_reset,

	.halt = xtensa_halt,
	.resume = esp32_xtensa_core_resume,
	.step = esp32_xtensa_core_step,

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

	.init_target = esp_xtensa_target_init,
	.examine = xtensa_examine,
};

static struct xtensa_debug_ops esp32_dbg_ops = {
	.queue_enable = xtensa_dm_queue_enable,
	.queue_reg_read = xtensa_dm_queue_reg_read,
	.queue_reg_write = xtensa_dm_queue_reg_write
};

static struct xtensa_power_ops esp32_pwr_ops = {
	.queue_reg_read = xtensa_dm_queue_pwr_reg_read,
	.queue_reg_write = xtensa_dm_queue_pwr_reg_write
};

static struct xtensa_core_ops esp32_xtensa_mcores_ops = {
	.core_init_arch_info = (void *)esp_xtensa_init_arch_info,
	.core_on_reset = esp_xtensa_on_reset,
	.core_do_step = xtensa_do_step,
	.core_prepare_resume = xtensa_prepare_resume,
	.core_do_resume = xtensa_do_resume,
	.core_is_special_breakpoint = esp_xtensa_is_special_breakpoint,
};

static struct esp_xtensa_special_breakpoint_ops esp32_xtensa_spec_brp_ops = {
	.breakpoint_add = esp_xtensa_flash_breakpoint_add,
	.breakpoint_remove = esp_xtensa_flash_breakpoint_remove
};

static const struct xtensa_chip_ops esp32_chip_ops = {
	.on_poll = esp32_on_poll,
	.on_halt = esp32_on_halt,
};

static int esp32_target_create(struct target *target, Jim_Interp *interp)
{
	struct target_type *core_target_types[ESP32_MAX_CORES_NUM];
	struct xtensa_core_ops *mcores_ops[ESP32_MAX_CORES_NUM];
	void *cores_privs[ESP32_MAX_CORES_NUM];
	struct xtensa_config *xtensa_cfgs[ESP32_MAX_CORES_NUM];
	struct xtensa_debug_module_config dm_cfgs[ESP32_MAX_CORES_NUM];
	struct jtag_tap *tap;

	struct esp32_common *esp32 = calloc(1, sizeof(struct esp32_common));
	if (esp32 == NULL) {
		LOG_ERROR("Failed to alloc memory for arch info!");
		return ERROR_FAIL;
	}

	/* determine number of configured esp32 cores basing on the number of configured TAPs for
	 * this chip */
	/* TODO: implement esp32 SMP target, in this case number of cores will be equal to the
	 *       number of sub-targets in SMP chain (linked list) */
	uint32_t configured_cores_num = 0;
	tap = NULL;
	do {
		tap = jtag_tap_next_enabled(tap);
		if (tap) {
			if (strcmp(tap->chip,
					target->type->name) == 0 &&
				configured_cores_num < ESP32_MAX_CORES_NUM) {
				/* all ESP32 cores are instances of ESP Xtensa */
				mcores_ops[configured_cores_num] = &esp32_xtensa_mcores_ops;
				/* any ESP32 core can work with flash breakpoints */
				cores_privs[configured_cores_num] = &esp32_xtensa_spec_brp_ops;
				/* all ESP32 cores have the same Xtensa core config */
				xtensa_cfgs[configured_cores_num] = (void *)&esp32_xtensa_cfg;
				core_target_types[configured_cores_num] =
					&esp32_xtensa_core_target_type;
				/* all ESP32 cores have the same Xtensa debug interface, but use
				 * different TAPs */
				dm_cfgs[configured_cores_num].dbg_ops = &esp32_dbg_ops;
				dm_cfgs[configured_cores_num].pwr_ops = &esp32_pwr_ops;
				dm_cfgs[configured_cores_num].tap = tap;
				dm_cfgs[configured_cores_num].queue_tdi_idle = NULL;
				dm_cfgs[configured_cores_num].queue_tdi_idle_arg = NULL;
				configured_cores_num++;
			}
		}
	} while (tap != NULL);
	if (configured_cores_num < 1 || configured_cores_num > ESP32_MAX_CORES_NUM) {
		LOG_ERROR("Invalid number of configured cores: %d!", configured_cores_num);
		free(esp32);
		return ERROR_FAIL;
	}

	int ret = xtensa_mcore_init_arch_info(target,
		&esp32->xtensa_mcore,
		configured_cores_num,
		&esp32_chip_ops,
		core_target_types,
		mcores_ops,
		xtensa_cfgs,
		dm_cfgs,
		cores_privs);
	if (ret != ERROR_OK) {
		LOG_ERROR("Failed to init arch info!");
		free(esp32);
		return ret;
	}
	/*Assume running target. If different, the first poll will fix this. */
	target->state = TARGET_RUNNING;
	target->debug_reason = DBG_REASON_NOTHALTED;
	return ERROR_OK;
}

COMMAND_HANDLER(esp32_cmd_flashbootstrap)
{
	struct xtensa_mcore_common *xtensa_mcore = target_to_xtensa_mcore(get_current_target(
			CMD_CTX));

	if (CMD_ARGC < 1)
		return CALL_COMMAND_HANDLER(esp_xtensa_cmd_flashbootstrap_do,
			target_to_esp_xtensa(&xtensa_mcore->cores_targets[0]));
	for (int i = 0; i < xtensa_mcore->configured_cores_num; i++) {
		int ret = CALL_COMMAND_HANDLER(esp_xtensa_cmd_flashbootstrap_do,
			target_to_esp_xtensa(&xtensa_mcore->cores_targets[i]));
		if (ret != ERROR_OK)
			return ret;
	}

	return ERROR_OK;
}

COMMAND_HANDLER(esp32_cmd_semihost_basedir)
{
	struct xtensa_mcore_common *xtensa_mcore = target_to_xtensa_mcore(get_current_target(
			CMD_CTX));

	if (CMD_ARGC != 1) {
		/* print only once, both cores have the same settings */
		return CALL_COMMAND_HANDLER(esp_xtensa_cmd_semihost_basedir_do,
			target_to_esp_xtensa(&xtensa_mcore->cores_targets[0]));
	}

	for (int i = 0; i < xtensa_mcore->configured_cores_num; i++) {
		int ret = CALL_COMMAND_HANDLER(esp_xtensa_cmd_semihost_basedir_do,
			target_to_esp_xtensa(&xtensa_mcore->cores_targets[i]));
		if (ret != ERROR_OK)
			return ret;
	}

	return ERROR_OK;
}

static const struct command_registration esp32_any_command_handlers[] = {
	{
		.name = "flashbootstrap",
		.handler = esp32_cmd_flashbootstrap,
		.mode = COMMAND_ANY,
		.help =
			"Set the idle state of the TMS pin, which at reset also is the voltage selector for the flash chip.",
		.usage = "none|1.8|3.3|high|low",
	},
	{
		.name = "semihost_basedir",
		.handler = esp32_cmd_semihost_basedir,
		.mode = COMMAND_ANY,
		.help = "Set the base directory for semohosting I/O.",
		.usage = "dir",
	},
	COMMAND_REGISTRATION_DONE
};

extern const struct command_registration esp108_common_command_handlers[];

static const struct command_registration esp32_command_handlers[] = {
	{
		.name = "esp32",
		.mode = COMMAND_ANY,
		.help = "ESP32 command group",
		.usage = "",
		.chain = esp32_any_command_handlers,
	},
	{
		.name = "esp32",
		.mode = COMMAND_ANY,
		.help = "ESP32 multi-core commands group",
		.usage = "",
		.chain = xtensa_mcore_command_handlers,
	},
	{
		.name = "esp32",
		.mode = COMMAND_ANY,
		.help = "ESP32 apptrace commands group",
		.usage = "",
		.chain = esp32_apptrace_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

/** Holds methods for Xtensa targets. */
struct target_type esp32_target = {
	.name = "esp32",

	.poll = xtensa_mcore_poll,
	.arch_state = esp32_arch_state,

	.halt = xtensa_mcore_halt,
	.resume = xtensa_mcore_resume,
	.step = xtensa_mcore_step,

	.assert_reset = esp32_assert_reset,
	.deassert_reset = xtensa_mcore_deassert_reset,

	.virt2phys = esp32_virt2phys,
	.mmu = xtensa_mcore_mmu,
	.read_memory = xtensa_mcore_read_memory,
	.write_memory = xtensa_mcore_write_memory,

	.read_buffer = xtensa_mcore_read_buffer,
	.write_buffer = xtensa_mcore_write_buffer,

	.checksum_memory = xtensa_mcore_checksum_memory,

	.get_gdb_reg_list = xtensa_mcore_get_gdb_reg_list,

	.run_algorithm = xtensa_mcore_run_algorithm,
	.start_algorithm = xtensa_mcore_start_algorithm,
	.wait_algorithm = xtensa_mcore_wait_algorithm,

	.add_breakpoint = xtensa_mcore_breakpoint_add,
	.remove_breakpoint = xtensa_mcore_breakpoint_remove,

	.add_watchpoint = xtensa_mcore_watchpoint_add,
	.remove_watchpoint = xtensa_mcore_watchpoint_remove,

	.target_create = esp32_target_create,
	.init_target = esp32_target_init,
	.examine = xtensa_mcore_examine,

	.commands = esp32_command_handlers,

	.get_cores_count = esp32_get_enabled_cores_count,
	.get_active_core = xtensa_mcore_get_active_core,
	.set_active_core = xtensa_mcore_set_active_core,
};
