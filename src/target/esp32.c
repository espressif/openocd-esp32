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
#include "smp.h"

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
#define ESP32_DRAM_LOW            0x3ffae000
#define ESP32_DRAM_HIGH           0x40000000
#define ESP32_IROM_MASK_LOW       0x40000000
#define ESP32_IROM_MASK_HIGH      0x40064f00
#define ESP32_IRAM_LOW            0x40070000
#define ESP32_IRAM_HIGH           0x400a0000
#define ESP32_RTC_IRAM_LOW        0x400c0000
#define ESP32_RTC_IRAM_HIGH       0x400c2000
#define ESP32_RTC_DRAM_LOW        0x3ff80000
#define ESP32_RTC_DRAM_HIGH       0x3ff82000
#define ESP32_RTC_DATA_LOW        0x50000000
#define ESP32_RTC_DATA_HIGH       0x50002000
#define ESP32_EXTRAM_DATA_LOW     0x3f800000
#define ESP32_EXTRAM_DATA_HIGH    0x3fc00000
#define ESP32_DR_REG_LOW          0x3ff00000
#define ESP32_DR_REG_HIGH         0x3ff71000
#define ESP32_SYS_RAM_LOW         0x60000000UL
#define ESP32_SYS_RAM_HIGH        (ESP32_SYS_RAM_LOW+0x20000000UL)

/* ESP32 WDT */
#define ESP32_WDT_WKEY_VALUE       0x50d83aa1
#define ESP32_TIMG0_BASE           0x3ff5f000
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
#define ESP32_DR_REG_DPORT_BASE         ESP32_DR_REG_LOW
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
				.base = ESP32_IROM_MASK_LOW,
				.size = ESP32_IROM_MASK_HIGH-ESP32_IROM_MASK_LOW,
				.access = XT_MEM_ACCESS_READ,
			},
		}
	},
	.iram           = {
		.count = 2,
		.regions = {
			{
				.base = ESP32_IRAM_LOW,
				.size = ESP32_IRAM_HIGH-ESP32_IRAM_LOW,
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
		.count = 6,
		.regions = {
			{
				.base = ESP32_DRAM_LOW,
				.size = ESP32_DRAM_HIGH-ESP32_DRAM_LOW,
				.access = XT_MEM_ACCESS_READ|XT_MEM_ACCESS_WRITE,
			},
			{
				.base = ESP32_RTC_DRAM_LOW,
				.size = ESP32_RTC_DRAM_HIGH-ESP32_RTC_DRAM_LOW,
				.access = XT_MEM_ACCESS_READ|XT_MEM_ACCESS_WRITE,
			},
			{
				.base = ESP32_RTC_DATA_LOW,
				.size = ESP32_RTC_DATA_HIGH-ESP32_RTC_DATA_LOW,
				.access = XT_MEM_ACCESS_READ|XT_MEM_ACCESS_WRITE,
			},
			{
				.base = ESP32_EXTRAM_DATA_LOW,
				.size = ESP32_EXTRAM_DATA_HIGH-ESP32_EXTRAM_DATA_LOW,
				.access = XT_MEM_ACCESS_READ|XT_MEM_ACCESS_WRITE,
			},
			{
				.base = ESP32_DR_REG_LOW,
				.size = ESP32_DR_REG_HIGH-ESP32_DR_REG_LOW,
				.access = XT_MEM_ACCESS_READ|XT_MEM_ACCESS_WRITE,
			},
			{
				.base = ESP32_SYS_RAM_LOW,
				.size = ESP32_SYS_RAM_HIGH-ESP32_SYS_RAM_LOW,
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

static int esp32_soc_reset(struct target *target);
static int esp32_smp_update_halt_gdb(struct target *target, bool *need_resume);
static int esp32_resume(struct target *target,
	int current,
	target_addr_t address,
	int handle_breakpoints,
	int debug_execution);

static int esp32_assert_reset(struct target *target)
{
	struct target_list *head;

	LOG_DEBUG("%s: begin", target_name(target));
	/* in SMP mode we need to ensure that at first we reset SOC on PRO-CPU
	   and then call xtensa_assert_reset() for all cores */
	if (target->coreid != 0)
		return ERROR_OK;
	/* Reset the SoC first */
	int res = esp32_soc_reset(target);
	if (res != ERROR_OK)
		return res;
	if (!target->smp)
		return xtensa_assert_reset(target);

	foreach_smp_target(head, target->head) {
		res = xtensa_assert_reset(head->target);
		if (res != ERROR_OK)
			return res;
	}
	return res;
}

static int esp32_deassert_reset(struct target *target)
{
	LOG_DEBUG("%s: begin", target_name(target));

	int ret = xtensa_deassert_reset(target);
	if (ret != ERROR_OK)
		return ret;
	/* in SMP mode when chip was running single-core app the other core can be left un-examined,
	   becasue examination is done before SOC reset. But after SOC reset it is functional and should be handled.
	   So try to examine un-examined core just after SOC reset */
	if (target->smp && !target_was_examined(target))
		ret = xtensa_examine(target);
	return ret;
}

/* Reset ESP32 peripherals.
Postconditions: all peripherals except RTC_CNTL are reset, CPU's PC is undefined, PRO CPU is halted, APP CPU is in reset
How this works:
0. make sure target is halted; if not, try to halt it; if that fails, try to reset it (via OCD) and then halt
1. set CPU initial PC to 0x50000000 (ESP32_SMP_RTC_DATA_LOW) by clearing RTC_CNTL_{PRO,APP}CPU_STAT_VECTOR_SEL
2. load stub code into ESP32_SMP_RTC_DATA_LOW; once executed, stub code will disable watchdogs and make CPU spin in an idle loop.
3. trigger SoC reset using RTC_CNTL_SW_SYS_RST bit
4. wait for the OCD to be reset
5. halt the target and wait for it to be halted (at this point CPU is in the idle loop)
6. restore initial PC and the contents of ESP32_SMP_RTC_DATA_LOW
TODO: some state of RTC_CNTL is not reset during SW_SYS_RST. Need to reset that manually.
*/
static int esp32_soc_reset(struct target *target)
{
	int res;
	struct target_list *head;
	struct xtensa *xtensa;

	LOG_DEBUG("start");
	/* In order to write to peripheral registers, target must be halted first */
	if (target->state != TARGET_HALTED) {
		LOG_DEBUG("Target not halted before SoC reset, trying to halt it first");
		xtensa_halt(target);
		res = target_wait_state(target, TARGET_HALTED, 1000);
		if (res != ERROR_OK) {
			LOG_DEBUG("Couldn't halt target before SoC reset, trying to do reset-halt");
			res = xtensa_assert_reset(target);
			if (res != ERROR_OK) {
				LOG_ERROR(
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
				LOG_ERROR(
					"Couldn't halt target before SoC reset! (xtensa_deassert_reset returned %d)",
					res);
				return res;
			}
			alive_sleep(10);
			xtensa_poll(target);
			xtensa_halt(target);
			res = target_wait_state(target, TARGET_HALTED, 1000);
			if (res != ERROR_OK) {
				LOG_ERROR("Couldn't halt target before SoC reset");
				return res;
			}
		}
	}
	assert(target->state == TARGET_HALTED);

	if (target->smp) {
		foreach_smp_target(head, target->head) {
			xtensa = target_to_xtensa(head->target);
			/* if any of the cores is stalled unstall them */
			if (xtensa_dm_core_is_stalled(&xtensa->dbg_mod)) {
				uint32_t word = ESP32_RTC_CNTL_SW_CPU_STALL_DEF;
				LOG_DEBUG("%s: Unstall CPUs before SW reset!",
					target_name(head->target));
				res = xtensa_write_buffer(target,
					ESP32_RTC_CNTL_SW_CPU_STALL_REG,
					sizeof(word),
					(uint8_t *)&word);
				if (res != ERROR_OK) {
					LOG_ERROR("%s: Failed to unstall CPUs before SW reset!",
						target_name(head->target));
					return res;
				}
				break;	/* both cores are unstalled now, so exit the loop */
			}
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

	LOG_DEBUG("Loading stub code into RTC RAM");
	uint32_t slow_mem_save[sizeof(esp32_reset_stub_code) / sizeof(uint32_t)];

	const int RTC_SLOW_MEM_BASE = 0x50000000;
	/* Save contents of RTC_SLOW_MEM which we are about to overwrite */
	res =
		target_read_buffer(target,
		RTC_SLOW_MEM_BASE,
		sizeof(slow_mem_save),
		(uint8_t *)slow_mem_save);
	if (res != ERROR_OK) {
		LOG_ERROR("Failed to save contents of RTC_SLOW_MEM (%d)!", res);
		return res;
	}

	/* Write stub code into RTC_SLOW_MEM */
	res =
		target_write_buffer(target, RTC_SLOW_MEM_BASE,
		sizeof(esp32_reset_stub_code),
		(const uint8_t *)esp32_reset_stub_code);
	if (res != ERROR_OK) {
		LOG_ERROR("Failed to write stub (%d)!", res);
		return res;
	}

	LOG_DEBUG("Resuming the target");
	xtensa = target_to_xtensa(target);
	xtensa->suppress_dsr_errors = true;
	res = xtensa_resume(target, 0, RTC_SLOW_MEM_BASE + 4, 0, 0);
	if (res != ERROR_OK) {
		LOG_ERROR("Failed to run stub (%d)!", res);
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
		LOG_ERROR("Timed out waiting for CPU to be reset, target state=%d", target->state);
		return ERROR_TARGET_TIMEOUT;
	}

	/* Halt the CPU again */
	LOG_DEBUG("halting the target");
	xtensa_halt(target);
	res = target_wait_state(target, TARGET_HALTED, 1000);
	if (res != ERROR_OK) {
		LOG_ERROR("Timed out waiting for CPU to be halted after SoC reset");
		return res;
	}

	/* Restore the original contents of RTC_SLOW_MEM */
	LOG_DEBUG("restoring RTC_SLOW_MEM");
	res =
		target_write_buffer(target, RTC_SLOW_MEM_BASE, sizeof(slow_mem_save),
		(const uint8_t *)slow_mem_save);
	if (res != ERROR_OK) {
		LOG_ERROR("Failed to restore contents of RTC_SLOW_MEM (%d)!", res);
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

static int esp32_arch_state(struct target *target)
{
	return ERROR_OK;
}

static struct target *get_halted_esp32(struct target *target, int32_t coreid)
{
	struct target_list *head;
	struct target *curr;

	foreach_smp_target(head, target->head) {
		curr = head->target;
		if ((curr->coreid == coreid) && (curr->state == TARGET_HALTED))
			return curr;
	}

	return target;
}

static int esp32_poll(struct target *target)
{
	enum target_state old_state = target->state;
	struct esp32_common *esp32 = target_to_esp32(target);
	struct esp_xtensa_common *esp_xtensa = target_to_esp_xtensa(target);
	uint32_t old_dbg_stubs_base = esp_xtensa->dbg_stubs.base;
	struct target_list *head;
	struct target *curr;
	bool other_core_resume_req = false;
	int ret;

	/*  toggle to another core is done by gdb as follow
	 *  maint packet J core_id
	 *  continue
	 *  the next polling trigger an halt event sent to gdb */
	if ((target->state == TARGET_HALTED) && (target->smp) &&
		(target->gdb_service) &&
		(target->gdb_service->target == NULL)) {
		target->gdb_service->target =
			get_halted_esp32(target, target->gdb_service->core[1]);
		LOG_INFO("Switch GDB target to '%s'", target_name(target->gdb_service->target));
		target_call_event_callbacks(target, TARGET_EVENT_HALTED);
		return ERROR_OK;
	}

	ret = esp_xtensa_poll(target);
	if (esp_xtensa->dbg_stubs.base && old_dbg_stubs_base != esp_xtensa->dbg_stubs.base) {
		/* debug stubs base is set only in PRO-CPU TRAX register, so sync this info */
		foreach_smp_target(head, target->head) {
			curr = head->target;
			if (curr == target)
				continue;
			target_to_esp_xtensa(curr)->dbg_stubs.base = esp_xtensa->dbg_stubs.base;
		}
	}

	if (old_state != TARGET_HALTED && target->state == TARGET_HALTED) {
		if (target->smp) {
			ret = esp32_smp_update_halt_gdb(target, &other_core_resume_req);
			if (ret != ERROR_OK)
				return ret;
		}
		/*Call any event callbacks that are applicable */
		if (old_state == TARGET_DEBUG_RUNNING)
			target_call_event_callbacks(target, TARGET_EVENT_DEBUG_HALTED);
		else {
			if (esp_xtensa_semihosting(target, &ret) != 0) {
				if (target->smp && target->semihosting->op == ESP_SYS_DRV_INFO) {
					/* semihosting's version syncing with other cores */
					foreach_smp_target(head, target->head) {
						curr = head->target;
						if (curr == target)
							continue;
						target_to_esp_xtensa(curr)->semihost.version =
							esp_xtensa->semihost.version;
					}
				}
				if (ret == ERROR_OK && esp_xtensa->semihost.need_resume &&
					!esp32->other_core_does_resume) {
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
			/* check whether any core polled by esp32_smp_update_halt_gdb() requested
			 *resume */
			if (target->smp && other_core_resume_req) {
				/* Resume xtensa_resume will handle BREAK instruction. */
				ret = target_resume(target, 1, 0, 1, 0);
				if (ret != ERROR_OK) {
					LOG_ERROR("Failed to resume target");
					return ret;
				}
				return ret;
			}
			target_call_event_callbacks(target, TARGET_EVENT_HALTED);
		}
	}

	return ret;
}

static int esp32_smp_update_halt_gdb(struct target *target, bool *need_resume)
{
	struct target *gdb_target = NULL;
	struct target_list *head;
	struct target *curr;
	int retval = 0;

	*need_resume = false;

	if (target->gdb_service && target->gdb_service->target)
		LOG_DEBUG("GDB target '%s'", target_name(target->gdb_service->target));

	if (target->gdb_service && target->gdb_service->core[0] == -1) {
		target->gdb_service->target = target;
		target->gdb_service->core[0] = target->coreid;
		LOG_INFO("Set GDB target to '%s'", target_name(target));
	}

	if (target->gdb_service)
		gdb_target = target->gdb_service->target;

	/* due to smpbreak config other cores can also go to HALTED state */
	foreach_smp_target(head, target->head) {
		curr = head->target;
		LOG_DEBUG("Check target '%s'", target_name(curr));
		/* skip calling context */
		if (curr == target)
			continue;
		if (!target_was_examined(curr)) {
			curr->state = TARGET_HALTED;
			continue;
		}
		/* skip targets that were already halted */
		if (curr->state == TARGET_HALTED)
			continue;
		/* Skip gdb_target; it alerts GDB so has to be polled as last one */
		if (curr == gdb_target)
			continue;
		LOG_DEBUG("Poll target '%s'", target_name(curr));

		struct esp32_common *esp32 = target_to_esp32(curr);
		/* avoid auto-resume after syscall, it will be done later */
		esp32->other_core_does_resume = true;
		/* avoid recursion in esp32_poll() */
		curr->smp = 0;
		esp32_poll(curr);
		curr->smp = 1;
		esp32->other_core_does_resume = false;
		struct esp_xtensa_common *curr_esp_xtensa = target_to_esp_xtensa(curr);
		if (curr_esp_xtensa->semihost.need_resume) {
			curr_esp_xtensa->semihost.need_resume = false;
			*need_resume = true;
		}
	}

	/* after all targets were updated, poll the gdb serving target */
	if (gdb_target != NULL && gdb_target != target)
		esp32_poll(gdb_target);

	LOG_DEBUG("exit");

	return retval;
}

static inline int esp32_smpbreak_disable(struct target *target, uint32_t *smp_break)
{
	int res = xtensa_smpbreak_get(target, smp_break);
	if (res != ERROR_OK)
		return res;
	res = xtensa_smpbreak_set(target, 0);
	if (res != ERROR_OK)
		return res;
	return ERROR_OK;
}

static inline int esp32_smpbreak_restore(struct target *target, uint32_t smp_break)
{
	return xtensa_smpbreak_set(target, smp_break);
}

static int esp32_smp_resume(struct target *target, int handle_breakpoints, int debug_execution)
{
	int res = ERROR_OK;
	struct target_list *head;
	struct target *curr;

	LOG_DEBUG("%s", target_name(target));

	foreach_smp_target(head, target->head) {
		curr = head->target;
		if ((curr != target) && (curr->state != TARGET_RUNNING)
			/* in single-core mode disabled core cannot be examined, but need to be
			 *resumed too
			 *&& target_was_examined(curr)*/) {
			/*  resume current address, not in SMP mode */
			curr->smp = 0;
			res = esp32_resume(curr, 1, 0, handle_breakpoints, debug_execution);
			curr->smp = 1;
			if (res != ERROR_OK)
				return res;
		}
	}
	return res;
}

static int esp32_resume(struct target *target,
	int current,
	target_addr_t address,
	int handle_breakpoints,
	int debug_execution)
{
	int res;
	uint32_t smp_break;

	xtensa_smpbreak_get(target, &smp_break);
	LOG_DEBUG("%s: smp_break=0x%x", target_name(target), smp_break);

	/* dummy resume for smp toggle in order to reduce gdb impact  */
	if ((target->smp) && (target->gdb_service->core[1] != -1)) {
		/*   simulate a start and halt of target */
		target->gdb_service->target = NULL;
		target->gdb_service->core[0] = target->gdb_service->core[1];
		/*  fake resume at next poll we play the  target core[1], see poll*/
		LOG_DEBUG("%s: Fake resume", target_name(target));
		target_call_event_callbacks(target, TARGET_EVENT_RESUMED);
		return ERROR_OK;
	}

	/* xtensa_prepare_resume() can step over breakpoint/watchpoint and
	        generate signals on BreakInOut circuit for other cores.
	        So disconnect this core from BreakInOut circuit and do xtensa_prepare_resume().
	*/
	res = esp32_smpbreak_disable(target, &smp_break);
	if (res != ERROR_OK)
		return res;
	res = xtensa_prepare_resume(target,
		current,
		address,
		handle_breakpoints,
		debug_execution);
	/* restore configured BreakInOut signals config */
	int ret = esp32_smpbreak_restore(target, smp_break);
	if (ret != ERROR_OK)
		return ret;
	if (res != ERROR_OK) {
		LOG_ERROR("%s: Failed to prepare for resume!", target_name(target));
		return res;
	}

	if (target->smp) {
		target->gdb_service->core[0] = -1;
		res = esp32_smp_resume(target, handle_breakpoints, debug_execution);
		if (res != ERROR_OK)
			return res;
	}

	res = xtensa_do_resume(target);
	if (res != ERROR_OK) {
		LOG_ERROR("%s: Failed to resume!", target_name(target));
		return res;
	}

	target->debug_reason = DBG_REASON_NOTHALTED;
	if (!debug_execution)
		target->state = TARGET_RUNNING;
	else
		target->state = TARGET_DEBUG_RUNNING;

	target_call_event_callbacks(target, TARGET_EVENT_RESUMED);
	return res;
}

static int esp32_step(struct target *target,
	int current,
	target_addr_t address,
	int handle_breakpoints)
{
	int res = ERROR_OK;
	uint32_t smp_break;

	if (target->smp) {
		res = esp32_smpbreak_disable(target, &smp_break);
		if (res != ERROR_OK)
			return res;
	}
	res = xtensa_step(target,
		current,
		address,
		handle_breakpoints);
	if (target->smp) {
		int ret = esp32_smpbreak_restore(target, smp_break);
		if (ret != ERROR_OK)
			return ret;
	}
	return res;
}

static int esp32_watchpoint_add(struct target *target, struct watchpoint *watchpoint)
{
	int res = ERROR_OK;

	res = xtensa_watchpoint_add(target, watchpoint);
	if (target->smp && res == ERROR_OK) {
		struct target_list *head;
		foreach_smp_target(head, target->head) {
			struct target *curr = head->target;
			if (curr == target || !target_was_examined(curr))
				continue;
			/* Need to use high level API here because every target for core contains list of watchpoints.
			   GDB works with active core only, so we need to duplicate every watchpoint on other cores,
			   otherwise watchpoint_free() on active core can fail if WP has been initially added on another core. */
			curr->smp = 0;
			res = watchpoint_add(curr, watchpoint->address, watchpoint->length,
				watchpoint->rw, watchpoint->value, watchpoint->mask);
			curr->smp = 1;
			if (res != ERROR_OK)
				break;
		}
	}
	return res;
}

static int esp32_watchpoint_remove(struct target *target, struct watchpoint *watchpoint)
{
	int res = ERROR_OK;

	res = xtensa_watchpoint_remove(target, watchpoint);
	if (target->smp && res == ERROR_OK) {
		struct target_list *head;
		foreach_smp_target(head, target->head) {
			struct target *curr = head->target;
			if (curr == target)
				continue;
			/* see big comment in esp32_watchpoint_add() */
			curr->smp = 0;
			watchpoint_remove(curr, watchpoint->address);
			curr->smp = 1;
		}
	}
	return res;
}

int esp32_run_func_image(struct target *target,
	struct xtensa_algo_run_data *run,
	struct xtensa_algo_image *image,
	uint32_t num_args,
	...)
{
	struct target *run_target;
	struct target_list *head;
	va_list ap;
	uint32_t smp_break;
	int res;

	if (target->smp) {
		/* find first HALTED and examined core */
		foreach_smp_target(head, target->head) {
			run_target = head->target;
			if (target_was_examined(run_target) && run_target->state == TARGET_HALTED)
				break;
		}
		if (head == NULL) {
			LOG_ERROR("Failed to find HALTED core!");
			return ERROR_FAIL;
		}
		res = esp32_smpbreak_disable(run_target, &smp_break);
		if (res != ERROR_OK)
			return res;
	} else
		run_target = target;

	va_start(ap, num_args);
	res = xtensa_run_func_image_va(run_target, run, image, num_args, ap);
	va_end(ap);

	if (target->smp) {
		res = esp32_smpbreak_restore(run_target, smp_break);
		if (res != ERROR_OK)
			return res;
	}
	return ERROR_OK;
}

static int esp32_virt2phys(struct target *target,
	target_addr_t virtual, target_addr_t *physical)
{
	*physical = virtual;
	return ERROR_OK;
}

static int esp32_handle_target_event(struct target *target, enum target_event event, void *priv)
{
	if (target != priv)
		return ERROR_OK;

	LOG_DEBUG("%d", event);

	int ret = esp_xtensa_handle_target_event(target, event, priv);
	if (ret != ERROR_OK)
		return ret;

	switch (event) {
		case TARGET_EVENT_HALTED:
			ret = esp32_disable_wdts(target);
			if (ret != ERROR_OK)
				return ret;
			break;
		default:
			break;
	}
	return ERROR_OK;
}

/*
        The TDI pin is also used as a flash Vcc bootstrap pin. If we reset the CPU externally, the last state of the TDI pin can
        allow the power to an 1.8V flash chip to be raised to 3.3V, or the other way around. Users can use the
        esp32 flashbootstrap command to set a level, and this routine will make sure the tdi line will return to
        that when the jtag port is idle.
*/
static void esp32_queue_tdi_idle(struct target *target)
{
	struct esp32_common *esp32 = target_to_esp32(target);
	static uint8_t value;
	uint8_t t[4] = { 0, 0, 0, 0 };

	if (esp32->flash_bootstrap == FBS_TMSLOW) {
		/*Make sure tdi is 0 at the exit of queue execution */
		value = 0;
	} else if (esp32->flash_bootstrap == FBS_TMSHIGH) {
		/*Make sure tdi is 1 at the exit of queue execution */
		value = 1;
	} else
		return;

	/*Scan out 1 bit, do not move from IRPAUSE after we're done. */
	buf_set_u32(t, 0, 1, value);
	jtag_add_plain_ir_scan(1, t, NULL, TAP_IRPAUSE);
}

static int esp32_target_init(struct command_context *cmd_ctx, struct target *target)
{
	int ret = esp_xtensa_target_init(cmd_ctx, target);
	if (ret != ERROR_OK)
		return ret;

	ret = target_register_event_callback(esp32_handle_target_event, target);
	if (ret != ERROR_OK)
		return ret;

	if (target->smp) {
		struct target_list *head;
		if (!target->working_area_cfg.phys_spec) {
			/* Working areas are configured for one core only. Use the same config data for other cores.
			It is safe to share config data because algorithms can not be ran on different cores concurrently. */
			foreach_smp_target(head, target->head) {
				struct target *curr = head->target;
				if (curr == target)
					continue;
				if (curr->working_area_cfg.phys_spec) {
					memcpy(&target->working_area_cfg,
						&curr->working_area_cfg,
						sizeof(curr->working_area_cfg));
					break;
				}
			}
		}
		if (!target->alt_working_area_cfg.phys_spec) {
			foreach_smp_target(head, target->head) {
				struct target *curr = head->target;
				if (curr == target)
					continue;
				if (curr->alt_working_area_cfg.phys_spec) {
					memcpy(&target->alt_working_area_cfg,
						&curr->alt_working_area_cfg,
						sizeof(curr->alt_working_area_cfg));
					break;
				}
			}
		}
		/* TODO: make one cycle instead of three */
		foreach_smp_target(head, target->head) {
			struct target *curr = head->target;
			ret = esp_xtensa_semihosting_init(curr);
			if (ret != ERROR_OK)
				return ret;
		}
	} else {
		ret = esp_xtensa_semihosting_init(target);
		if (ret != ERROR_OK)
			return ret;
	}
	return ERROR_OK;
}

static const struct xtensa_debug_ops esp32_dbg_ops = {
	.queue_enable = xtensa_dm_queue_enable,
	.queue_reg_read = xtensa_dm_queue_reg_read,
	.queue_reg_write = xtensa_dm_queue_reg_write
};

static const struct xtensa_power_ops esp32_pwr_ops = {
	.queue_reg_read = xtensa_dm_queue_pwr_reg_read,
	.queue_reg_write = xtensa_dm_queue_pwr_reg_write
};

static const struct esp_xtensa_flash_breakpoint_ops esp32_flash_brp_ops = {
	.breakpoint_add = esp_xtensa_flash_breakpoint_add,
	.breakpoint_remove = esp_xtensa_flash_breakpoint_remove
};

static int esp32_target_create(struct target *target, Jim_Interp *interp)
{
	struct xtensa_debug_module_config esp32_dm_cfg = {
		.dbg_ops = &esp32_dbg_ops,
		.pwr_ops = &esp32_pwr_ops,
		.tap = target->tap,
		.queue_tdi_idle = esp32_queue_tdi_idle,
		.queue_tdi_idle_arg = target
	};

	struct esp32_common *esp32 = calloc(1, sizeof(struct esp32_common));
	if (esp32 == NULL) {
		LOG_ERROR("Failed to alloc memory for arch info!");
		return ERROR_FAIL;
	}

	int ret = esp_xtensa_init_arch_info(target, &esp32->esp_xtensa, &esp32_xtensa_cfg,
		&esp32_dm_cfg, &esp32_flash_brp_ops);
	if (ret != ERROR_OK) {
		LOG_ERROR("Failed to init arch info!");
		free(esp32);
		return ret;
	}
	esp32->flash_bootstrap = FBS_DONTCARE;

	/*Assume running target. If different, the first poll will fix this. */
	target->state = TARGET_RUNNING;
	target->debug_reason = DBG_REASON_NOTHALTED;
	return ERROR_OK;
}

COMMAND_HANDLER(esp32_cmd_permissive_mode)
{
	struct target *target = get_current_target(CMD_CTX);
	if (target->smp && CMD_ARGC > 0) {
		struct target_list *head;
		struct target *curr;
		foreach_smp_target(head, target->head) {
			curr = head->target;
			int ret = CALL_COMMAND_HANDLER(xtensa_cmd_permissive_mode_do,
				target_to_xtensa(curr));
			if (ret != ERROR_OK)
				return ret;
		}
		return ERROR_OK;
	}
	return CALL_COMMAND_HANDLER(xtensa_cmd_permissive_mode_do,
		target_to_xtensa(target));
}

COMMAND_HANDLER(esp32_cmd_smpbreak)
{
	struct target *target = get_current_target(CMD_CTX);
	if (target->smp && CMD_ARGC > 0) {
		struct target_list *head;
		struct target *curr;
		foreach_smp_target(head, target->head) {
			curr = head->target;
			int ret = CALL_COMMAND_HANDLER(xtensa_cmd_smpbreak_do, curr);
			if (ret != ERROR_OK)
				return ret;
		}
		return ERROR_OK;
	}
	return CALL_COMMAND_HANDLER(xtensa_cmd_smpbreak_do, target);
}

COMMAND_HANDLER(esp32_cmd_mask_interrupts)
{
	struct target *target = get_current_target(CMD_CTX);
	if (target->smp && CMD_ARGC > 0) {
		struct target_list *head;
		struct target *curr;
		foreach_smp_target(head, target->head) {
			curr = head->target;
			int ret = CALL_COMMAND_HANDLER(xtensa_cmd_mask_interrupts_do,
				target_to_xtensa(curr));
			if (ret != ERROR_OK)
				return ret;
		}
		return ERROR_OK;
	}
	return CALL_COMMAND_HANDLER(xtensa_cmd_mask_interrupts_do,
		target_to_xtensa(target));
}

COMMAND_HANDLER(esp32_cmd_perfmon_enable)
{
	struct target *target = get_current_target(CMD_CTX);
	if (target->smp && CMD_ARGC > 0) {
		struct target_list *head;
		struct target *curr;
		foreach_smp_target(head, target->head) {
			curr = head->target;
			int ret = CALL_COMMAND_HANDLER(xtensa_cmd_perfmon_enable_do,
				target_to_xtensa(curr));
			if (ret != ERROR_OK)
				return ret;
		}
		return ERROR_OK;
	}
	return CALL_COMMAND_HANDLER(xtensa_cmd_perfmon_enable_do,
		target_to_xtensa(target));
}

COMMAND_HANDLER(esp32_cmd_perfmon_dump)
{
	struct target *target = get_current_target(CMD_CTX);
	if (target->smp) {
		struct target_list *head;
		struct target *curr;
		foreach_smp_target(head, target->head) {
			curr = head->target;
			LOG_INFO("CPU%d:", curr->coreid);
			int ret = CALL_COMMAND_HANDLER(xtensa_cmd_perfmon_dump_do,
				target_to_xtensa(curr));
			if (ret != ERROR_OK)
				return ret;
		}
		return ERROR_OK;
	}
	return CALL_COMMAND_HANDLER(xtensa_cmd_perfmon_dump_do,
		target_to_xtensa(target));
}

COMMAND_HANDLER(esp32_cmd_tracestart)
{
	struct target *target = get_current_target(CMD_CTX);
	if (target->smp) {
		struct target_list *head;
		struct target *curr;
		foreach_smp_target(head, target->head) {
			curr = head->target;
			int ret = CALL_COMMAND_HANDLER(xtensa_cmd_tracestart_do,
				target_to_xtensa(curr));
			if (ret != ERROR_OK)
				return ret;
		}
		return ERROR_OK;
	}
	return CALL_COMMAND_HANDLER(xtensa_cmd_tracestart_do,
		target_to_xtensa(target));
}

COMMAND_HANDLER(esp32_cmd_tracestop)
{
	struct target *target = get_current_target(CMD_CTX);
	if (target->smp && CMD_ARGC > 0) {
		struct target_list *head;
		struct target *curr;
		foreach_smp_target(head, target->head) {
			curr = head->target;
			int ret = CALL_COMMAND_HANDLER(xtensa_cmd_tracestop_do,
				target_to_xtensa(curr));
			if (ret != ERROR_OK)
				return ret;
		}
		return ERROR_OK;
	}
	return CALL_COMMAND_HANDLER(xtensa_cmd_tracestop_do,
		target_to_xtensa(target));
}

COMMAND_HANDLER(esp32_cmd_tracedump)
{
	struct target *target = get_current_target(CMD_CTX);
	if (target->smp) {
		struct target_list *head;
		struct target *curr;
		uint32_t cores_max_id = 0;
		/* assume that core IDs are assigned to SMP targets sequentially: 0,1,2... */
		foreach_smp_target(head, target->head) {
			curr = head->target;
			if (cores_max_id < (uint32_t)curr->coreid)
				cores_max_id = curr->coreid;
		}
		if (CMD_ARGC < (cores_max_id+1)) {
			command_print(CMD,
				"Need %d filenames to dump to as output!",
				cores_max_id+1);
			return ERROR_FAIL;
		}
		foreach_smp_target(head, target->head) {
			curr = head->target;
			int ret = CALL_COMMAND_HANDLER(xtensa_cmd_tracedump_do,
				target_to_xtensa(curr), CMD_ARGV[curr->coreid]);
			if (ret != ERROR_OK)
				return ret;
		}
		return ERROR_OK;
	}
	return CALL_COMMAND_HANDLER(xtensa_cmd_tracedump_do,
		target_to_xtensa(target), CMD_ARGV[0]);
}

COMMAND_HANDLER(esp32_cmd_semihost_basedir)
{
	struct target *target = get_current_target(CMD_CTX);
	if (target->smp && CMD_ARGC > 0) {
		struct target_list *head;
		struct target *curr;
		foreach_smp_target(head, target->head) {
			curr = head->target;
			int ret = CALL_COMMAND_HANDLER(esp_xtensa_cmd_semihost_basedir_do,
				target_to_esp_xtensa(curr));
			if (ret != ERROR_OK)
				return ret;
		}
		return ERROR_OK;
	}
	return CALL_COMMAND_HANDLER(esp_xtensa_cmd_semihost_basedir_do,
		target_to_esp_xtensa(target));
}

COMMAND_HELPER(esp32_cmd_flashbootstrap_do, struct esp32_common *esp32)
{
	int state = -1;

	if (CMD_ARGC < 1) {
		const char *st;
		state = esp32->flash_bootstrap;
		if (state == FBS_DONTCARE)
			st = "Don't care";
		else if (state == FBS_TMSLOW)
			st = "Low (3.3V)";
		else if (state == FBS_TMSHIGH)
			st = "High (1.8V)";
		else
			st = "None";
		command_print(CMD, "Current idle tms state: %s", st);
		return ERROR_OK;
	}

	if (!strcasecmp(CMD_ARGV[0], "none"))
		state = FBS_DONTCARE;
	else if (!strcasecmp(CMD_ARGV[0], "1.8"))
		state = FBS_TMSHIGH;
	else if (!strcasecmp(CMD_ARGV[0], "3.3"))
		state = FBS_TMSLOW;
	else if (!strcasecmp(CMD_ARGV[0], "high"))
		state = FBS_TMSHIGH;
	else if (!strcasecmp(CMD_ARGV[0], "low"))
		state = FBS_TMSLOW;

	if (state == -1) {
		command_print(CMD,
			"Argument unknown. Please pick one of none, high, low, 1.8 or 3.3");
		return ERROR_FAIL;
	}
	esp32->flash_bootstrap = state;
	return ERROR_OK;
}

COMMAND_HANDLER(esp32_cmd_flashbootstrap)
{
	struct target *target = get_current_target(CMD_CTX);

	if (target->smp) {
		struct target_list *head;
		struct target *curr;
		foreach_smp_target(head, target->head) {
			curr = head->target;
			int ret = CALL_COMMAND_HANDLER(esp32_cmd_flashbootstrap_do,
				target_to_esp32(curr));
			if (ret != ERROR_OK)
				return ret;
		}
		return ERROR_OK;
	}
	return CALL_COMMAND_HANDLER(esp32_cmd_flashbootstrap_do,
		target_to_esp32(target));
}

static const struct command_registration esp32_xtensa_command_handlers[] = {
	{
		.name = "set_permissive",
		.handler = esp32_cmd_permissive_mode,
		.mode = COMMAND_ANY,
		.help = "When set to 1, enable Xtensa permissive mode (less client-side checks)",
		.usage = "[0|1]",
	},
	{
		.name = "maskisr",
		.handler = esp32_cmd_mask_interrupts,
		.mode = COMMAND_ANY,
		.help = "mask Xtensa interrupts at step",
		.usage = "['on'|'off']",
	},
	{
		.name = "smpbreak",
		.handler = esp32_cmd_smpbreak,
		.mode = COMMAND_ANY,
		.help = "Set the way the CPU chains OCD breaks",
		.usage =
			"[none|breakinout|runstall] | [BreakIn] [BreakOut] [RunStallIn] [DebugModeOut]",
	},
	{
		.name = "perfmon_enable",
		.handler = esp32_cmd_perfmon_enable,
		.mode = COMMAND_EXEC,
		.help = "Enable and start performance counter",
		.usage = "<counter_id> <select> [mask] [kernelcnt] [tracelevel]",
	},
	{
		.name = "perfmon_dump",
		.handler = esp32_cmd_perfmon_dump,
		.mode = COMMAND_EXEC,
		.help =
			"Dump performance counter value. If no argument specified, dumps all counters.",
		.usage = "[counter_id]",
	},
	{
		.name = "tracestart",
		.handler = esp32_cmd_tracestart,
		.mode = COMMAND_EXEC,
		.help =
			"Tracing: Set up and start a trace. Optionally set stop trigger address and amount of data captured after.",
		.usage = "[pc <pcval>/[maskbitcount]] [after <n> [ins|words]]",
	},
	{
		.name = "tracestop",
		.handler = esp32_cmd_tracestop,
		.mode = COMMAND_EXEC,
		.help = "Tracing: Stop current trace as started by the tracestart command",
		.usage = "",
	},
	{
		.name = "tracedump",
		.handler = esp32_cmd_tracedump,
		.mode = COMMAND_EXEC,
		.help = "Tracing: Dump trace memory to a files. One file per core.",
		.usage = "<outfile1> <outfile2>",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration esp32_esp_command_handlers[] = {
	{
		.name = "semihost_basedir",
		.handler = esp32_cmd_semihost_basedir,
		.mode = COMMAND_ANY,
		.help = "Set the base directory for semohosting I/O.",
		.usage = "dir",
	},
	{
		.mode = COMMAND_ANY,
		.usage = "",
		.chain = esp32_apptrace_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration esp32_any_command_handlers[] = {
	{
		.name = "flashbootstrap",
		.handler = esp32_cmd_flashbootstrap,
		.mode = COMMAND_ANY,
		.help =
			"Set the idle state of the TMS pin, which at reset also is the voltage selector for the flash chip.",
		.usage = "none|1.8|3.3|high|low",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration esp32_command_handlers[] = {
	{
		.name = "xtensa",
		.usage = "",
		.chain = esp32_xtensa_command_handlers,
	},
	{
		.name = "esp",
		.usage = "",
		.chain = esp32_esp_command_handlers,
	},
	{
		.name = "esp32",
		.usage = "",
		.chain = smp_command_handlers,
	},
	{
		.name = "esp32",
		.usage = "",
		.chain = esp32_any_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration esp32_legacy_command_handlers[] = {
	{
		.name = "esp32",
		.usage = "",
		.chain = esp32_xtensa_command_handlers,
	},
	{
		.name = "esp32",
		.usage = "",
		.chain = esp32_esp_command_handlers,
	},
	{
		.name = "esp32",
		.usage = "",
		.chain = esp32_any_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration esp32_all_command_handlers[] = {
	{
		.usage = "",
		.chain = esp32_command_handlers,
	},
	{
		.usage = "",
		.chain = esp32_legacy_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

/** Holds methods for Xtensa targets. */
struct target_type esp32_target = {
	.name = "esp32",

	.poll = esp32_poll,
	.arch_state = esp32_arch_state,

	.halt = xtensa_halt,
	.resume = esp32_resume,
	.step = esp32_step,

	.assert_reset = esp32_assert_reset,
	.deassert_reset = esp32_deassert_reset,

	.virt2phys = esp32_virt2phys,
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

	.add_watchpoint = esp32_watchpoint_add,
	.remove_watchpoint = esp32_watchpoint_remove,

	.target_create = esp32_target_create,
	.init_target = esp32_target_init,
	.examine = xtensa_examine,
	.deinit_target = esp_xtensa_target_deinit,

	.commands = esp32_all_command_handlers,
};
