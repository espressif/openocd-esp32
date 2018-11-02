/***************************************************************************
 *   ESP32 target for OpenOCD                                              *
 *   Copyright (C) 2017 Espressif Systems Ltd.                             *
 *                                                                         *
 *   Derived from original ESP8266 target.                                 *
 *   Copyright (C) 2015 by Angus Gratton                                   *
 *   gus@projectgus.com                                                    *
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
#include "register.h"
#include "assert.h"
#include "time_support.h"
#include "rtos/rtos.h"
#include "flash/nor/core.h"
#include "flash/nor/imp.h"
#include "contrib/loaders/flash/esp32/stub_flasher.h"
#include "contrib/loaders/flash/esp32/stub_flasher_image.h"

#include "esp32.h"
#include "esp108_dbg_regs.h"
#include "esp108_apptrace.h"

/*
This is a JTAG driver for the ESP32, the are two Tensilica cores inside
the ESP32 chip. For more information please have a look into ESP108 target
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

The ESP32 chip has two ESP108 cores inside, but represent themself to the OCD
as one chip that works in multithreading mode under FreeRTOS OS.
The core that initiate the stop condition will be defined as an active cpu.
When one core stops, then other core will be stoped automativally by smpbreak.
The core that initiate stop condition will be defined as an active core, and
registers of this core will be transfered.
*/

#define esp32_regs esp108_regs

#define ESP32_STUB_DEBUG			0
#define ESP32_ALGORITHM_EXIT_TMO	40000 // ms
#define ESP32_TARGET_STATE_TMO		3000 // ms

#if ESP32_STUB_DEBUG
#define ESP32_STUB_STACK_STAMP		0xCE
#define ESP32_STUB_STACK_DEBUG		128
#else
#define ESP32_STUB_STACK_DEBUG		0
#endif

#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
#define ESP32_HOSTTOL(_v_)		((((_v_) & 0xFFUL) << 24) | (((_v_) & 0xFF00UL) << 8) | (((_v_) & 0xFF0000UL) >> 8) | (((_v_) & 0xFF000000UL) >> 24))
#else
#define ESP32_HOSTTOL(_v_)		(_v_)
#endif

/* ESP32 dport regs */
#define ESP32_DR_REG_DPORT_BASE         0x3ff00000
#define ESP32_DPORT_APPCPU_CTRL_B_REG   (ESP32_DR_REG_DPORT_BASE + 0x030)
#define ESP32_DPORT_APPCPU_CLKGATE_EN	(1 << 0)

#define ESP32_DBGSTUBS_UPDATE_DATA_ENTRY(_e_) \
do { \
	(_e_) = intfromchars((uint8_t *)&(_e_)); \
	if (!esp32_data_addr_valid((_e_))) { \
		LOG_ERROR("No stub entry found!"); \
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE; \
	} \
} while(0)

#define ESP32_DBGSTUBS_UPDATE_CODE_ENTRY(_e_) \
do { \
	(_e_) = intfromchars((uint8_t *)&(_e_)); \
	if ((_e_) == 0) { \
		LOG_ERROR("No stub entry found!"); \
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE; \
	} \
} while(0)

#define ESP32_IDLE_LOOP_CODE_SZ			3 // size of jump insn

//forward declarations
static int xtensa_do_step(struct target *target,
	int current,
	uint32_t address,
	int handle_breakpoints);
static int xtensa_poll(struct target *target);
static int xtensa_assert_reset(struct target *target);
static int xtensa_deassert_reset(struct target *target);
static int xtensa_smpbreak_set(struct target *target);
static int xtensa_smpbreak_set_core(struct target *target, int core);
static int xtensa_read_memory(struct target *target,
	uint32_t address,
	uint32_t size,
	uint32_t count,
	uint8_t *buffer);
static size_t esp32_get_cores_count(struct target *target);
static size_t esp32_get_active_core(struct target *target);
static void esp32_set_active_core(struct target *target, size_t core);
static size_t esp32_read_cores_num(struct target *target);
static int esp32_dbgstubs_restore(struct target *target);
static int esp32_dbgstubs_update_info(struct target *target);
static uint32_t esp32_dbgstubs_get(struct target *target);
static int esp32_handle_target_event(struct target *target, enum target_event event, void *priv);

//Utility function: check DSR for any weirdness and report.
//Also does tms_reset to bootstrap level indicated.
#define esp32_checkdsr(target) esp108_do_checkdsr(target, __FUNCTION__, __LINE__)

static const uint8_t esp32_stub_tramp[] = {
	#include "src/target/esp32_stub_tramp.inc"
};

static void esp32_mark_register_dirty(struct reg *reg_list, int regidx)
{
	reg_list[regidx].dirty=1;
}

static int esp32_fetch_all_regs(struct target *target, uint8_t cpu_mask)
{
	int i, j;
	int cpenable;
	int res;
	uint32_t regval;
	uint32_t windowbase;
	struct esp32_common *esp32=(struct esp32_common*)target->arch_info;
	struct reg *reg_list[ESP32_CPU_COUNT];
	uint8_t regvals[ESP32_CPU_COUNT][XT_NUM_REGS][4];
	uint8_t dsrs[ESP32_CPU_COUNT][XT_NUM_REGS][4];

	LOG_DEBUG("%s: %s", target->cmd_name, __FUNCTION__);

	for (int c = 0; c < ESP32_CPU_COUNT; c++)
	{
		reg_list[c] = esp32->core_caches[c]->reg_list;
	}

	//Assume the CPU has just halted. We now want to fill the register cache with all the
	//register contents GDB needs. For speed, we pipeline all the read operations, execute them
	//in one go, then sort everything out from the regvals variable.

	// Read registers from both cores
	for (size_t c = 0; c < ESP32_CPU_COUNT; c++)
	{
		if ((cpu_mask & (1 << c)) == 0) {
			continue;
		}
		//Start out with A0-A63; we can reach those immediately. Grab them per 16 registers.
		for (j = 0; j < 64; j += 16) {
			//Grab the 16 registers we can see
			for (i = 0; i < 16; i++) {
				esp108_queue_exec_ins(esp32->esp32_targets[c], XT_INS_WSR(XT_SR_DDR, esp32_regs[XT_REG_IDX_AR0 + i].reg_num));
				esp108_queue_nexus_reg_read(esp32->esp32_targets[c], NARADR_DDR, regvals[c][XT_REG_IDX_AR0 + i + j]);
				esp108_queue_nexus_reg_read(esp32->esp32_targets[c], NARADR_DSR, dsrs[c][XT_REG_IDX_AR0 + i + j]);
			}
			//Now rotate the window so we'll see the next 16 registers. The final rotate will wraparound,
			//leaving us in the state we were.
			esp108_queue_exec_ins(esp32->esp32_targets[c], XT_INS_ROTW(4));
		}

		//As the very first thing after A0-A63, go grab the CPENABLE registers. It indicates if we can also grab the FP
		//(and theoretically other coprocessor) registers, or if this is a bad thing to do.
		esp108_queue_exec_ins(esp32->esp32_targets[c], XT_INS_RSR(esp32_regs[XT_REG_IDX_CPENABLE].reg_num, XT_REG_A3));
		esp108_queue_exec_ins(esp32->esp32_targets[c], XT_INS_WSR(XT_SR_DDR, XT_REG_A3));
		esp108_queue_nexus_reg_read(esp32->esp32_targets[c], NARADR_DDR, regvals[c][XT_REG_IDX_CPENABLE]);


		res=jtag_execute_queue();
		if (res!=ERROR_OK) {
			LOG_ERROR("Failed to read ARs (%d)!\n", res);
			return res;
		}

		esp32_checkdsr(esp32->esp32_targets[c]);
		cpenable = intfromchars(regvals[c][XT_REG_IDX_CPENABLE]);


		//We're now free to use any of A0-A15 as scratch registers
		//Grab the SFRs and user registers first. We use A3 as a scratch register.
		for (i = 0; i < XT_NUM_REGS; i++) {
			if (regReadable(esp32_regs[i].flags, cpenable) && (esp32_regs[i].type == XT_REG_SPECIAL || esp32_regs[i].type == XT_REG_USER || esp32_regs[i].type == XT_REG_FR)) {
				if (esp32_regs[i].type == XT_REG_USER) {
					esp108_queue_exec_ins(esp32->esp32_targets[c], XT_INS_RUR(esp32_regs[i].reg_num, XT_REG_A3));
				}
				else if (esp32_regs[i].type == XT_REG_FR) {
					esp108_queue_exec_ins(esp32->esp32_targets[c], XT_INS_RFR(esp32_regs[i].reg_num, XT_REG_A3));
				}
				else { //SFR
					esp108_queue_exec_ins(esp32->esp32_targets[c], XT_INS_RSR(esp32_regs[i].reg_num, XT_REG_A3));
				}
				esp108_queue_exec_ins(esp32->esp32_targets[c], XT_INS_WSR(XT_SR_DDR, XT_REG_A3));
				esp108_queue_nexus_reg_read(esp32->esp32_targets[c], NARADR_DDR, regvals[c][i]);
				esp108_queue_nexus_reg_read(esp32->esp32_targets[c], NARADR_DSR, dsrs[c][i]);
			}
		}

		//Ok, send the whole mess to the CPU.
		res=jtag_execute_queue();
		if (res!=ERROR_OK) {
			LOG_ERROR("Failed to fetch AR regs!\n");
			return res;
		}

		esp32_checkdsr(esp32->esp32_targets[c]);
		//DSR checking: follows order in which registers are requested.
		for (i = 0; i < XT_NUM_REGS; i++) {
			if (regReadable(esp32_regs[i].flags, cpenable) && (esp32_regs[i].type == XT_REG_SPECIAL || esp32_regs[i].type == XT_REG_USER || esp32_regs[i].type == XT_REG_FR)) {
				if (intfromchars(dsrs[c][i])&OCDDSR_EXECEXCEPTION) {
					LOG_ERROR("Exception reading %s!\n", esp32_regs[i].name);
					return ERROR_FAIL;
				}
			}
		}

		//We need the windowbase to decode the general addresses.
		windowbase = intfromchars(regvals[c][XT_REG_IDX_WINDOWBASE]);
		//Decode the result and update the cache.
		for (i = 0; i < XT_NUM_REGS; i++) {
			if (regReadable(esp32_regs[i].flags, cpenable)) {
				if (esp32_regs[i].type == XT_REG_GENERAL) {
					//The 64-value general register set is read from (windowbase) on down. We need
					//to get the real register address by subtracting windowbase and wrapping around.
					int realadr = canonical_to_windowbase_offset(i, windowbase);
					regval = intfromchars(regvals[c][realadr]);
					//				LOG_INFO("mapping: %s -> %s (windowbase off %d)\n",esp32_regs[i].name, esp32_regs[realadr].name, windowbase*4);
				}
				else if (esp32_regs[i].type == XT_REG_RELGEN) {
					regval = intfromchars(regvals[c][esp32_regs[i].reg_num]);
				}
				else {
					regval = intfromchars(regvals[c][i]);
					//				LOG_INFO("Register %s: 0x%X", esp32_regs[i].name, regval);
				}
				esp108_reg_set(&reg_list[c][i], regval);
				reg_list[c][i].valid = 1;
				reg_list[c][i].dirty = 0; //always do this _after_ esp108_reg_set!
				//LOG_DEBUG("%s Register %s: 0x%X", esp32->esp32_targets[c]->cmd_name, reg_list[c][i].name, regval);
			}
			else {
				reg_list[c][i].valid = 0;
				//LOG_DEBUG("Register NOT READBLE %s: 0x%X", reg_list[c][i].name, 0);
			}
		}
		//We have used A3 as a scratch register and we will need to write that back.
		struct reg *cpu_reg_list = esp32->core_caches[c]->reg_list;
		esp32_mark_register_dirty(cpu_reg_list, XT_REG_IDX_A3);
	}
	return ERROR_OK;
}

static int esp32_write_dirty_registers(struct target *target, struct reg *reg_list)
{
	int i, j;
	int res;
	uint32_t regval, windowbase;
	bool scratch_reg_dirty = false;

	LOG_DEBUG("%s: %s", target->cmd_name, __FUNCTION__);

	//We need to write the dirty registers in the cache list back to the processor.
	//Start by writing the SFR/user registers.
	for (i=0; i<XT_NUM_REGS; i++) {
		if (reg_list[i].dirty) {
			if (esp32_regs[i].type==XT_REG_SPECIAL || esp32_regs[i].type==XT_REG_USER || esp32_regs[i].type==XT_REG_FR) {
				scratch_reg_dirty = true;
				regval=esp108_reg_get(&reg_list[i]);
				LOG_DEBUG("%s: Writing back reg %s val %08X", target->cmd_name, esp32_regs[i].name, regval);
				esp108_queue_nexus_reg_write(target, NARADR_DDR, regval);
				esp108_queue_exec_ins(target, XT_INS_RSR(XT_SR_DDR, XT_REG_A3));
				if (esp32_regs[i].type==XT_REG_USER) {
					esp108_queue_exec_ins(target, XT_INS_WUR(esp32_regs[i].reg_num, XT_REG_A3));
				} else if (esp32_regs[i].type==XT_REG_FR) {
					esp108_queue_exec_ins(target, XT_INS_WFR(esp32_regs[i].reg_num, XT_REG_A3));
				} else { //SFR
					esp108_queue_exec_ins(target, XT_INS_WSR(esp32_regs[i].reg_num, XT_REG_A3));
				}
				reg_list[i].dirty=0;
			}
		}
	}
	if (scratch_reg_dirty) {
		esp32_mark_register_dirty(reg_list, XT_REG_IDX_A3);
	}

	//Grab the windowbase, we need it.
	windowbase=esp108_reg_get(&reg_list[XT_REG_IDX_WINDOWBASE]);

	//Check if there are problems with both the ARx as well as the corresponding Rx registers set and dirty.
	//Warn the user if this happens, not much else we can do...
	for (i=XT_REG_IDX_A0; i<=XT_REG_IDX_A15; i++) {
		j=windowbase_offset_to_canonical(i, windowbase);
		if (reg_list[i].dirty && reg_list[j].dirty) {
			if (memcmp(reg_list[i].value, reg_list[j].value, 4)!=0) {
				LOG_WARNING("Warning: Both A%d as well as the physical register it points to (AR%d) are dirty and differs in value. Results are undefined!", i-XT_REG_IDX_A0, j-XT_REG_IDX_AR0);
			}
		}
	}

	//Write A0-A16
	for (i=0; i<16; i++) {
		if (reg_list[XT_REG_IDX_A0+i].dirty) {
			regval=esp108_reg_get(&reg_list[XT_REG_IDX_A0+i]);
			LOG_DEBUG("%s: Writing back reg %s value %08X, num =%i", target->cmd_name, esp32_regs[XT_REG_IDX_A0 + i].name, regval, esp32_regs[XT_REG_IDX_A0 + i].reg_num);
			esp108_queue_nexus_reg_write(target, NARADR_DDR, regval);
			esp108_queue_exec_ins(target, XT_INS_RSR(XT_SR_DDR, i));
			reg_list[XT_REG_IDX_A0+i].dirty=0;
		}
	}

	//Now write AR0-AR63.
	for (j=0; j<64; j+=16) {
		//Write the 16 registers we can see
		for (i=0; i<16; i++) {
			int realadr=windowbase_offset_to_canonical(XT_REG_IDX_AR0+i+j, windowbase);
			//Write back any dirty un-windowed registers
			if (reg_list[realadr].dirty) {
				regval=esp108_reg_get(&reg_list[realadr]);
				LOG_DEBUG("%s: Writing back reg %s value %08X, num =%i", target->cmd_name, esp32_regs[realadr].name, regval, esp32_regs[realadr].reg_num);
				esp108_queue_nexus_reg_write(target, NARADR_DDR, regval);
				esp108_queue_exec_ins(target, XT_INS_RSR(XT_SR_DDR, esp32_regs[XT_REG_IDX_AR0+i].reg_num));
				reg_list[realadr].dirty=0;
			}
		}
		//Now rotate the window so we'll see the next 16 registers. The final rotate will wraparound,
		//leaving us in the state we were.
		esp108_queue_exec_ins(target, XT_INS_ROTW(4));
	}
	res=jtag_execute_queue();
	esp32_checkdsr(target);
	return res;
}

static int xtensa_halt(struct target *target)
{
	int res;
	uint8_t dsr[4];
	struct esp32_common* esp32 = (struct esp32_common*)target->arch_info;

	LOG_DEBUG("%s, target: %s", __func__, target->cmd_name);
	if (target->state == TARGET_HALTED) {
		LOG_DEBUG("%s: target was already halted", target->cmd_name);
		return ERROR_OK;
	}

	// First we have to read dsr and check if the target stopped
	int i = esp32->active_cpu;
	esp108_queue_nexus_reg_write(esp32->esp32_targets[i], NARADR_DCRSET, OCDDCR_ENABLEOCD);
	esp108_queue_nexus_reg_read(esp32->esp32_targets[i], NARADR_DSR, dsr);
	esp108_queue_tdi_idle(esp32->esp32_targets[i]);
	res = jtag_execute_queue();
	if (res != ERROR_OK)
	{
		LOG_ERROR("%s: Failed to read NARADR_DSR. Can't halt.", target->cmd_name);
		return res;
	}

	if (0 == (intfromchars(dsr) & OCDDSR_STOPPED))
	{
		LOG_DEBUG("Need to halt CPU %d", (int) i);
		esp108_queue_nexus_reg_write(esp32->esp32_targets[i], NARADR_DCRSET, OCDDCR_DEBUGINTERRUPT);
		esp108_queue_tdi_idle(esp32->esp32_targets[i]);
		res = jtag_execute_queue();

		if (res != ERROR_OK) {
			LOG_ERROR("%s: Failed to set OCDDCR_DEBUGINTERRUPT. Can't halt.", target->cmd_name);
			return ERROR_FAIL;
		}
	} else {
		LOG_DEBUG("CPU %d already halted", (int) i);
	}
	return ERROR_OK;
}

static int xtensa_resume(struct target *target,
			 int current,
			 uint32_t address,
			 int handle_breakpoints,
			 int debug_execution)
{
	struct esp32_common *esp32=(struct esp32_common*)target->arch_info;
	struct reg *reg_list = esp32->core_caches[esp32->active_cpu]->reg_list;
	int res=ERROR_OK;
	size_t slot;
	uint32_t bpena;

	LOG_DEBUG("%s: %s current=%d address=%04x, handle_breakpoints=%i, debug_execution=%i)", target->cmd_name, __func__, current, address, handle_breakpoints, debug_execution);

	if (target->state != TARGET_HALTED) {
		LOG_WARNING("%s: %s: target not halted", target->cmd_name, __func__);
		return ERROR_TARGET_NOT_HALTED;
	}

	if(address && !current) {
		esp108_reg_set(&reg_list[XT_REG_IDX_PC], address);
	} else {
		int cause=esp108_reg_get(&reg_list[XT_REG_IDX_DEBUGCAUSE]);
		if (cause&DEBUGCAUSE_DB) {
			//We stopped due to a watchpoint. We can't just resume executing the instruction again because
			//that would trigger the watchpoint again. To fix this, we single-step, which ignores watchpoints.
			xtensa_do_step(target, current, address, handle_breakpoints);
		}
		if (cause&(DEBUGCAUSE_BI|DEBUGCAUSE_BN)) {
			//We stopped due to a break instruction. We can't just resume executing the instruction again because
			//that would trigger the breake again. To fix this, we single-step, which ignores break.
			xtensa_do_step(target, current, address, handle_breakpoints);
		}
	}

	//Write back hw breakpoints. Current FreeRTOS SMP code can set a hw breakpoint on an
	//exception; we need to clear that and return to the breakpoints gdb has set on resume.
	bpena=0;
	for(slot = 0; slot < esp32->num_brps; slot++) {
		if (esp32->hw_brps[slot]!=NULL) {
			/* Write IBREAKA[slot] and set bit #slot in IBREAKENABLE */
			for (size_t cp = 0; cp < ESP32_CPU_COUNT; cp++)
			{
				// We have equival amount of BP for each cpu
				struct reg *cpu_reg_list = esp32->core_caches[cp]->reg_list;
				esp108_reg_set(&cpu_reg_list[XT_REG_IDX_IBREAKA0 + slot], esp32->hw_brps[slot]->address);
			}
			bpena|=(1<<slot);
		}
	}
	for (size_t cp = 0; cp < ESP32_CPU_COUNT; cp++)
	{
		// We have equival amount of BP for each cpu
		struct reg *cpu_reg_list = esp32->core_caches[cp]->reg_list;
		esp108_reg_set(&cpu_reg_list[XT_REG_IDX_IBREAKENABLE], bpena);
	}

	// Here we write all registers to the targets
	for (size_t i = 0; i < ESP32_CPU_COUNT; i++)
	{
		struct reg *cpu_reg_list = esp32->core_caches[i]->reg_list;
		res = esp32_write_dirty_registers(esp32->esp32_targets[i], cpu_reg_list);
		if (res != ERROR_OK) {
			LOG_ERROR("%s: Failed to write back register cache.", esp32->esp32_targets[i]->cmd_name);
			return ERROR_FAIL;
		}

		//Execute return from debug exception instruction
		esp108_queue_exec_ins(esp32->esp32_targets[i], XT_INS_RFDO);
		res = jtag_execute_queue();
		if (res != ERROR_OK) {
			LOG_ERROR("%s: Failed to clear OCDDCR_DEBUGINTERRUPT and resume execution. res=%i", __func__, res);
			return ERROR_FAIL;
		}
	}

	for (size_t i = 0; i < ESP32_CPU_COUNT; i++)
	{
		esp32_checkdsr(esp32->esp32_targets[i]);
	}
	target->debug_reason = DBG_REASON_NOTHALTED;
	if (!debug_execution){
		target->state = TARGET_RUNNING;
	}
	else {
		target->state = TARGET_DEBUG_RUNNING;
	}
	int res1 = target_call_event_callbacks(target, TARGET_EVENT_RESUMED);
	if (res1 != ERROR_OK) res = res1;
	return res;
}

static int xtensa_resume_active_cpu(struct target *target,
	int current,
	uint32_t address,
	int handle_breakpoints,
	int debug_execution)
{
	struct esp32_common *esp32 = (struct esp32_common*)target->arch_info;
	struct reg *reg_list = esp32->core_caches[esp32->active_cpu]->reg_list;
	int res = ERROR_OK;
	size_t slot;
	uint32_t bpena;

	LOG_DEBUG("%s: %s current=%d address=%04" PRIx32, target->cmd_name, __func__, current, address);

	if (target->state != TARGET_HALTED) {
		LOG_WARNING("%s: %s: target not halted", target->cmd_name, __func__);
		return ERROR_TARGET_NOT_HALTED;
	}

	if (address && !current) {
		esp108_reg_set(&reg_list[XT_REG_IDX_PC], address);
	}
	else {
		int cause = esp108_reg_get(&reg_list[XT_REG_IDX_DEBUGCAUSE]);
		if (cause&DEBUGCAUSE_DB) {
			//We stopped due to a watchpoint. We can't just resume executing the instruction again because
			//that would trigger the watchpoint again. To fix this, we single-step, which ignores watchpoints.
			xtensa_do_step(target, current, address, handle_breakpoints);
		}
	}
	//Write back hw breakpoints. Current FreeRTOS SMP code can set a hw breakpoint on an
	//exception; we need to clear that and return to the breakpoints gdb has set on resume.
	bpena = 0;
	for (slot = 0; slot < esp32->num_brps; slot++) {
		if (esp32->hw_brps[slot] != NULL) {
			/* Write IBREAKA[slot] and set bit #slot in IBREAKENABLE */
			esp108_reg_set(&reg_list[XT_REG_IDX_IBREAKA0 + slot], esp32->hw_brps[slot]->address);
			bpena |= (1 << slot);
		}
	}
	esp108_reg_set(&reg_list[XT_REG_IDX_IBREAKENABLE], bpena);

	// Here we write all registers to the targets
	// We need to write dirty registers on both CPUs because GDB can read/write memory using A3 on CPU0 at any time (when halted)
	// even when it is not active.
	// TODO: Maybe it is better to use active CPU for memory access.
	for (size_t i = 0; i < ESP32_CPU_COUNT; i++)
	{
		struct reg *cpu_reg_list = esp32->core_caches[i]->reg_list;
		res = esp32_write_dirty_registers(esp32->esp32_targets[i], cpu_reg_list);
		if (res != ERROR_OK) {
			LOG_ERROR("%s: Failed to write back register cache.", esp32->esp32_targets[i]->cmd_name);
			return ERROR_FAIL;
		}
		if (i == esp32->active_cpu){
			//Execute return from debug exception instruction for active CPU only
			esp108_queue_exec_ins(esp32->esp32_targets[i], XT_INS_RFDO);
			res = jtag_execute_queue();
			if (res != ERROR_OK) {
				LOG_ERROR("%s: Failed to clear OCDDCR_DEBUGINTERRUPT and resume execution. res=%i", __func__, res);
				return ERROR_FAIL;
			}
		}
	}

	for (size_t i = 0; i < ESP32_CPU_COUNT; i++)
	{
		esp32_checkdsr(esp32->esp32_targets[i]);
	}

	target->debug_reason = DBG_REASON_NOTHALTED;
	if (!debug_execution) {
		target->state = TARGET_RUNNING;
	}
	else {
		target->state = TARGET_DEBUG_RUNNING;
	}

	return res;
}

static int xtensa_read_memory(struct target *target,
							uint32_t address,
							uint32_t size,
							uint32_t count,
							uint8_t *buffer)
{
	//We are going to read memory in 32-bit increments. This may not be what the calling function expects, so we may need to allocate a temp buffer and read into that first.
	uint32_t addrstart_al=(address)&~3;
	uint32_t addrend_al=(address+(size*count)+3)&~3;
	uint32_t adr=addrstart_al;
	int i=0;
	int res;
	uint8_t *albuff;

	if (esp108_get_addr_type(address) == INVALID && !esp108_permissive_mode) {
		LOG_DEBUG("%s: address 0x%08x not readable", __func__, address);
		return ERROR_FAIL;
	}

	struct esp32_common *esp32 = (struct esp32_common*)target->arch_info;

	LOG_DEBUG("%s: %s: reading %d bytes from addr %08X", target->cmd_name, __FUNCTION__, size*count, address);
//	LOG_DEBUG("Converted to aligned addresses: read from %08X to %08X", addrstart_al, addrend_al);
	if (target->state != TARGET_HALTED) {
		LOG_WARNING("%s: %s: target not halted", __func__, target->cmd_name);
		return ERROR_TARGET_NOT_HALTED;
	}

	if (addrstart_al==address && addrend_al==address+(size*count)) {
		albuff=buffer;
	} else {
		albuff=malloc(addrend_al-addrstart_al);
		if (!albuff) {
			LOG_ERROR("%s: Out of memory allocating %d bytes!", __FUNCTION__, addrend_al-addrstart_al);
			return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		}
	}

	//We're going to use A3 here
	esp32_mark_register_dirty(esp32->core_caches[ESP32_PRO_CPU_ID]->reg_list, XT_REG_IDX_A3);
	//Write start address to A3
	esp108_queue_nexus_reg_write(esp32->esp32_targets[ESP32_PRO_CPU_ID], NARADR_DDR, addrstart_al);
	esp108_queue_exec_ins(esp32->esp32_targets[ESP32_PRO_CPU_ID], XT_INS_RSR(XT_SR_DDR, XT_REG_A3));
	//Now we can safely read data from addrstart_al up to addrend_al into albuff
	while (adr!=addrend_al) {
		esp108_queue_exec_ins(esp32->esp32_targets[ESP32_PRO_CPU_ID], XT_INS_LDDR32P(XT_REG_A3));
		albuff[i] = 0;
		esp108_queue_nexus_reg_read(esp32->esp32_targets[ESP32_PRO_CPU_ID], NARADR_DDR, &albuff[i]);
		adr+=4;
		i+=4;
	}
	res=jtag_execute_queue();
	if (res == ERROR_OK)
	{
		res = esp32_checkdsr(esp32->esp32_targets[ESP32_PRO_CPU_ID]);
	}

	if (res != ERROR_OK)
	{
		LOG_WARNING("%s: Failed reading %d bytes at address 0x%08X", esp32->esp32_targets[ESP32_PRO_CPU_ID]->cmd_name, count*size, address);
	}
	if (albuff!=buffer) {
		memcpy(buffer, albuff+(address&3), (size*count));
		free(albuff);
	}
	return res;
}

static int xtensa_read_buffer(struct target *target,
							uint32_t address,
							uint32_t count,
							uint8_t *buffer)
{
	//xtensa_read_memory can also read unaligned stuff. Just pass through to that routine.
	return xtensa_read_memory(target, address, 1, count, buffer);
}

static int xtensa_write_memory(struct target *target,
							uint32_t address,
							uint32_t size,
							uint32_t count,
							const uint8_t *buffer)
{
	//This memory write function can get thrown nigh everything into it, from
	//aligned uint32 writes to unaligned uint8ths. The Xtensa memory doesn't always
	//accept anything but aligned uint32 writes, though. That is why we convert
	//everything into that.

	uint32_t addrstart_al=(address)&~3;
	uint32_t addrend_al=(address+(size*count)+3)&~3;
	uint32_t adr=addrstart_al;

	int i=0;
	int res;
	uint8_t *albuff;

	if (esp108_get_addr_type(address) != READWRITE && !esp108_permissive_mode) {
		LOG_DEBUG("%s: address 0x%08x not writable", __func__, address);
		return ERROR_FAIL;
	}

	if (target->state != TARGET_HALTED) {
		LOG_WARNING("%s: %s: target not halted", __func__, target->cmd_name);
		return ERROR_TARGET_NOT_HALTED;
	}

	LOG_DEBUG("%s: %s: writing %d bytes to addr %08X", target->cmd_name, __FUNCTION__, size*count, address);
//	LOG_DEBUG("al start %x al end %x", addrstart_al, addrend_al);

	if ((size==0) || (count == 0) || !(buffer)) return ERROR_COMMAND_SYNTAX_ERROR;

	//Allocate a temporary buffer to put the aligned bytes in, if needed.
	if (addrstart_al==address && addrend_al==address+(size*count)) {
		//We discard the const here because albuff can also be non-const
		albuff=(uint8_t*)buffer;
	} else {
		albuff=malloc(addrend_al-addrstart_al);
		if (!albuff) {
			LOG_ERROR("%s: Out of memory allocating %d bytes!", __FUNCTION__, addrend_al-addrstart_al);
			return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		}
	}
	struct esp32_common *esp32 = (struct esp32_common*)target->arch_info;

	//We're going to use A3 here
	esp32_mark_register_dirty(esp32->core_caches[ESP32_PRO_CPU_ID]->reg_list, XT_REG_IDX_A3);

	//If we're using a temp aligned buffer, we need to fill the head and/or tail bit of it.
	if (albuff!=buffer) {
		//See if we need to read the first and/or last word.
		if (address&3) {
			esp108_queue_nexus_reg_write(esp32->esp32_targets[ESP32_PRO_CPU_ID], NARADR_DDR, addrstart_al);
			esp108_queue_exec_ins(esp32->esp32_targets[ESP32_PRO_CPU_ID], XT_INS_RSR(XT_SR_DDR, XT_REG_A3));
			esp108_queue_exec_ins(esp32->esp32_targets[ESP32_PRO_CPU_ID], XT_INS_LDDR32P(XT_REG_A3));
			esp108_queue_nexus_reg_read(esp32->esp32_targets[ESP32_PRO_CPU_ID], NARADR_DDR, &albuff[0]);
		}
		if ((address+(size*count))&3) {
			esp108_queue_nexus_reg_write(esp32->esp32_targets[ESP32_PRO_CPU_ID], NARADR_DDR, addrend_al - 4);
			esp108_queue_exec_ins(esp32->esp32_targets[ESP32_PRO_CPU_ID], XT_INS_RSR(XT_SR_DDR, XT_REG_A3));
			esp108_queue_exec_ins(esp32->esp32_targets[ESP32_PRO_CPU_ID], XT_INS_LDDR32P(XT_REG_A3));
			esp108_queue_nexus_reg_read(esp32->esp32_targets[ESP32_PRO_CPU_ID], NARADR_DDR, &albuff[addrend_al - addrstart_al - 4]);
		}
		//Grab bytes
		res=jtag_execute_queue();
		esp32_checkdsr(esp32->esp32_targets[ESP32_PRO_CPU_ID]);
		//Copy data to be written into the aligned buffer
		memcpy(&albuff[address&3], buffer, size*count);
		//Now we can write albuff in aligned uint32s.
	}

	//Write start address to A3
	esp108_queue_nexus_reg_write(esp32->esp32_targets[ESP32_PRO_CPU_ID], NARADR_DDR, addrstart_al);
	esp108_queue_exec_ins(esp32->esp32_targets[ESP32_PRO_CPU_ID], XT_INS_RSR(XT_SR_DDR, XT_REG_A3));
	//Write the aligned buffer
	while (adr!=addrend_al) {
		esp108_queue_nexus_reg_write(esp32->esp32_targets[ESP32_PRO_CPU_ID], NARADR_DDR, intfromchars(&albuff[i]));
		esp108_queue_exec_ins(esp32->esp32_targets[ESP32_PRO_CPU_ID], XT_INS_SDDR32P(XT_REG_A3));
		adr+=4;
		i+=4;
	}
	res=jtag_execute_queue();
	if (res == ERROR_OK) res = esp32_checkdsr(esp32->esp32_targets[ESP32_PRO_CPU_ID]);
	if (res != ERROR_OK)
	{
		LOG_WARNING("%s: Failed writing %d bytes at address 0x%08X, data - %02x, %02x, %02x, %02x, %02x, %02x, %02x, %02x",
			target->cmd_name, count*size, address,
			buffer[0], buffer[1], buffer[2], buffer[3], buffer[4], buffer[5], buffer[6], buffer[7]
			);
	}

	/* NB: if we were supporting the ICACHE option, we would need
	 * to invalidate it here */

	return res;
}

static int xtensa_write_buffer(struct target *target,
							uint32_t address,
							uint32_t count,
							const uint8_t *buffer)
{
	//xtensa_write_memory can handle everything. Just pass on to that.
	return xtensa_write_memory(target, address, 1, count, buffer);
}

static int xtensa_get_gdb_reg_list(struct target *target,
	struct reg **reg_list[],
	int *reg_list_size,
	enum target_register_class reg_class)
{
	int i;
	struct esp32_common *esp32 = target->arch_info;

	if (reg_class == REG_CLASS_ALL)
	{
		*reg_list_size = esp32->core_caches[esp32->active_cpu]->num_regs;
	}
	else
	{
		*reg_list_size = XT_NUM_REGS_G_COMMAND;
	}

	LOG_DEBUG("%s, reg_class=%i, num_regs=%d", __func__, (int)reg_class, *reg_list_size);

	*reg_list = malloc(sizeof(struct reg *) * (*reg_list_size));

	if (!*reg_list) {
		return ERROR_FAIL;
	}

	for (i = 0; i < *reg_list_size; i++) {
		(*reg_list)[i] = &esp32->core_caches[esp32->active_cpu]->reg_list[i];
	}

	return ERROR_OK;
}

static int xtensa_assert_reset(struct target *target)
{
	int res;
	struct esp32_common *esp32=(struct esp32_common*)target->arch_info;

	/* Reset the SoC first */
	esp32->active_cpu = ESP32_PRO_CPU_ID;
	res = esp32_soc_reset(target);
	if (res != ERROR_OK) {
		return res;
	}

	LOG_DEBUG("%s[%s] coreid=%i, target_number=%i, begin", __func__, target->cmd_name, target->coreid, target->target_number);
	target->state = TARGET_RESET;

	for (int i = 0; i < ESP32_CPU_COUNT; i++)
	{
		esp108_queue_pwrctl_set(esp32->esp32_targets[i], PWRCTL_JTAGDEBUGUSE | PWRCTL_DEBUGWAKEUP | PWRCTL_MEMWAKEUP | PWRCTL_COREWAKEUP | PWRCTL_CORERESET);
		esp108_queue_tdi_idle(esp32->esp32_targets[i]);
	}
	res=jtag_execute_queue();
	if (res != ERROR_OK) {
		return res;
	}
	esp32->resetAsserted=1;

	if (target->reset_halt) {
		res = target_halt(target);
		if (res != ERROR_OK) {
			return res;
		}
	}

	return ERROR_OK;
}

static int xtensa_smpbreak_set(struct target *target)
{
	int res = ERROR_OK;
	for (int core = 0; core < ESP32_CPU_COUNT && res == ERROR_OK; core++)
	{
		res = xtensa_smpbreak_set_core(target, core);
	}
	return res;
}

static int xtensa_smpbreak_set_core(struct target *target, int core)
{
	struct esp32_common *esp32 = (struct esp32_common*)target->arch_info;
	int res;
	uint32_t dsr_data = 0x00110000;
	uint32_t set = 0, clear = 0;
	set |= OCDDCR_BREAKINEN | OCDDCR_BREAKOUTEN | OCDDCR_RUNSTALLINEN | OCDDCR_ENABLEOCD;
	clear = set ^ (OCDDCR_BREAKINEN | OCDDCR_BREAKOUTEN | OCDDCR_RUNSTALLINEN | OCDDCR_DEBUGMODEOUTEN);

	esp108_queue_nexus_reg_write(esp32->esp32_targets[core], NARADR_DCRSET, set);
	esp108_queue_nexus_reg_write(esp32->esp32_targets[core], NARADR_DCRCLR, clear);
	esp108_queue_nexus_reg_write(esp32->esp32_targets[core], NARADR_DSR, dsr_data);
	esp108_queue_tdi_idle(esp32->esp32_targets[core]);
	res = jtag_execute_queue();

	LOG_DEBUG("%s[%s] core %d, set smpbreak=%x, state=%i", __func__, target->cmd_name, core, set, target->state);
	return res;
}


static int xtensa_deassert_reset(struct target *target)
{
	struct esp32_common *esp32=(struct esp32_common*)target->arch_info;
	int res;

	res = xtensa_smpbreak_set(target);
	if (res != ERROR_OK) return res;

	LOG_DEBUG("%s begin reset_halt=%i", __func__, target->reset_halt);
	for (int i = 0; i < ESP32_CPU_COUNT; i++)
	{
		esp108_queue_nexus_reg_write(esp32->esp32_targets[i], NARADR_DCRSET, OCDDCR_ENABLEOCD);
		uint8_t dsr[4];
		esp108_queue_nexus_reg_read(esp32->esp32_targets[i], NARADR_DSR, dsr);
		if (target->reset_halt) {
			esp108_queue_nexus_reg_write(esp32->esp32_targets[i], NARADR_DCRSET, OCDDCR_DEBUGINTERRUPT);
		}
		esp108_queue_pwrctl_set(esp32->esp32_targets[i], PWRCTL_JTAGDEBUGUSE | PWRCTL_DEBUGWAKEUP | PWRCTL_MEMWAKEUP | PWRCTL_COREWAKEUP);
		esp108_queue_tdi_idle(esp32->esp32_targets[i]);
	}
	res=jtag_execute_queue();
	LOG_DEBUG("%s[%s] begin reset_halt=%i, state=%i \n", __func__, target->cmd_name, target->reset_halt, target->state);
	target->state = TARGET_RUNNING;
	esp32->resetAsserted=0;
	return res;
}

static int esp32_handle_target_event(struct target *target, enum target_event event, void *priv)
{
	struct esp32_common *esp32=(struct esp32_common*)target->arch_info;

	LOG_DEBUG("%s %d", __func__, event);
	switch (event) {
		case TARGET_EVENT_HALTED:
			esp32->cores_num = esp32_read_cores_num(target);
			if (esp32->dbg_stubs.entries_count == 0 && esp32->dbg_stubs.base) {
				esp32_dbgstubs_update_info(target);
			}
			break;
		case TARGET_EVENT_GDB_DETACH:
		{
			enum target_state old_state = target->state;
			if (target->state != TARGET_HALTED) {
				int ret = target_halt(target);
				if (ret != ERROR_OK) {
					LOG_ERROR("%s: Failed to halt target to remove flash BPs (%d)!",
							target->cmd_name, ret);
					return ret;
				}
				ret = target_wait_state(target, TARGET_HALTED, ESP32_TARGET_STATE_TMO);
				if (ret != ERROR_OK) {
					LOG_ERROR("%s: Failed to wait halted target to remove flash BPs (%d)!",
							target->cmd_name, ret);
					return ret;
				}
			}
			for(size_t slot = 0; slot < ESP32_FLASH_SW_BREAKPOINTS_MAX_NUM; slot++) {
				struct esp32_flash_sw_breakpoint *flash_bp = esp32->flash_sw_brps[slot];
				if(flash_bp != NULL) {
					int ret = esp32_remove_flash_breakpoint(target, flash_bp);
					if (ret != ERROR_OK) {
						LOG_ERROR("%s: Failed to remove SW flash BP @ 0x%x (%d)!",
								target->cmd_name, flash_bp->data.oocd_bp->address, ret);
						return ret;
					}
				}
			}
			memset(esp32->flash_sw_brps, 0, ESP32_FLASH_SW_BREAKPOINTS_MAX_NUM*sizeof(struct esp32_flash_sw_breakpoint *));
			if (old_state == TARGET_RUNNING) {
				int ret = target_resume(target, 1, 0, 1, 0);
				if (ret != ERROR_OK) {
					LOG_ERROR("%s: Failed to resume target after flash BPs removal (%d)!",
							target->cmd_name, ret);
					return ret;
				}
			}
			break;
		}
		default:
			break;
	}
	return ERROR_OK;
}

static struct esp32_sw_breakpoint * esp32_add_sw_breakpoint(struct target *target, struct breakpoint *breakpoint)
{
	union {
		uint32_t d32;
		uint8_t d8[4];
	} break_insn;

	int ret = target_read_buffer(target, breakpoint->address, 3, break_insn.d8);
	if (ret != ERROR_OK) {
		LOG_ERROR("%s: Failed to read insn (%d)!", target->cmd_name, ret);
		return NULL;
	}
	struct esp32_sw_breakpoint *sw_bp = malloc(sizeof(struct esp32_flash_sw_breakpoint));
	if (sw_bp == NULL) {
		LOG_ERROR("Failed to alloc memory for sw breakpoint data!");
		return NULL;
	}
	sw_bp->insn_sz = xtensa_get_insn_size(break_insn.d8);
	memcpy(sw_bp->insn, break_insn.d8, sw_bp->insn_sz);
	sw_bp->oocd_bp = breakpoint;

	break_insn.d32 = sw_bp->insn_sz == 2 ? XT_INS_BREAKN(0) : XT_INS_BREAK(0, 0);

	ret = target_write_buffer(target, breakpoint->address, sw_bp->insn_sz, break_insn.d8);
	if (ret != ERROR_OK) {
		LOG_ERROR("%s: Failed to read insn (%d)!", target->cmd_name, ret);
		free(sw_bp);
		return NULL;
	}

	return sw_bp;
}

static int esp32_remove_sw_breakpoint(struct target *target, struct esp32_sw_breakpoint *breakpoint)
{
	int ret = target_write_buffer(target, breakpoint->oocd_bp->address, breakpoint->insn_sz, breakpoint->insn);
	if (ret != ERROR_OK) {
		LOG_ERROR("%s: Failed to read insn (%d)!", target->cmd_name, ret);
		return ret;
	}
	free(breakpoint);
	return ERROR_OK;
}

static int xtensa_add_breakpoint(struct target *target, struct breakpoint *breakpoint)
{
	struct esp32_common *esp32=(struct esp32_common*)target->arch_info;
	size_t slot;

	if (breakpoint->type == BKPT_SOFT) {
		for(slot = 0; slot < ESP32_SW_BREAKPOINTS_MAX_NUM; slot++) {
			if (esp32->sw_brps[slot] == NULL || esp32->sw_brps[slot]->oocd_bp == breakpoint) break;
		}
		if (slot == ESP32_SW_BREAKPOINTS_MAX_NUM)
		{
			LOG_WARNING("%s: max SW slot reached, slot=%i", __func__, (unsigned int)slot);
			return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		}
		esp32->sw_brps[slot] = esp32_add_sw_breakpoint(target, breakpoint);
		if (esp32->sw_brps[slot] == NULL) {
			LOG_ERROR("%s: Failed to add SW BP!", target->cmd_name);
			return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		}
		LOG_DEBUG("%s: placed SW breakpoint %d at 0x%X", target->cmd_name, (int)slot, breakpoint->address);
		return ERROR_OK;
	}

	for(slot = 0; slot < esp32->num_brps; slot++) {
		if (esp32->hw_brps[slot] == NULL || esp32->hw_brps[slot] == breakpoint) break;
	}
	if (slot == esp32->num_brps)
	{
		LOG_WARNING("%s: max HW slot reached, slot=%i", __func__, (unsigned int)slot);
		for(slot = 0; slot < ESP32_FLASH_SW_BREAKPOINTS_MAX_NUM; slot++) {
			if (esp32->flash_sw_brps[slot] == NULL || esp32->flash_sw_brps[slot]->data.oocd_bp == breakpoint) break;
		}
		if (slot == ESP32_FLASH_SW_BREAKPOINTS_MAX_NUM)
		{
			LOG_WARNING("%s: max SW flash slot reached, slot=%i", __func__, (unsigned int)slot);
			return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		}
		esp32->flash_sw_brps[slot] = esp32_add_flash_breakpoint(target, breakpoint);
		if (esp32->flash_sw_brps[slot] == NULL) {
			LOG_ERROR("%s: Failed to add SW flash BP!", target->cmd_name);
			return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		}
		LOG_DEBUG("%s: placed SW_FLASH breakpoint @ 0x%X", target->cmd_name, breakpoint->address);
		return ERROR_OK;
	}

	esp32->hw_brps[slot] = breakpoint;
	//We will actually write the breakpoints when we resume the target.
	LOG_DEBUG("%s: placed HW breakpoint @ 0x%X", target->cmd_name, breakpoint->address);

	return ERROR_OK;
}

static int xtensa_remove_breakpoint(struct target *target, struct breakpoint *breakpoint)
{
	struct esp32_common *esp32=(struct esp32_common*)target->arch_info;
	size_t slot;

	LOG_DEBUG("%s: %p", __func__, breakpoint);

	if (breakpoint->type == BKPT_SOFT) {
		for(slot = 0; slot < ESP32_SW_BREAKPOINTS_MAX_NUM; slot++) {
			if(esp32->sw_brps[slot] != NULL && esp32->sw_brps[slot]->oocd_bp == breakpoint)
				break;
		}
		if (slot == ESP32_SW_BREAKPOINTS_MAX_NUM)
		{
			LOG_WARNING("%s: max SW slot reached, slot=%i", __func__, (unsigned int)slot);
			return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		}
		int ret = esp32_remove_sw_breakpoint(target, esp32->sw_brps[slot]);
		if (ret != ERROR_OK) {
			LOG_ERROR("%s: Failed to remove SW BP (%d)!", target->cmd_name, ret);
			return ret;
		}
		esp32->sw_brps[slot] = NULL;
		LOG_DEBUG("%s: cleared SW breakpoint %d at 0x%X", target->cmd_name, (int)slot, breakpoint->address);
		return ERROR_OK;
	}

	for(slot = 0; slot < esp32->num_brps; slot++) {
		if(esp32->hw_brps[slot] == breakpoint)
			break;
	}
	if (slot==esp32->num_brps) {
		LOG_DEBUG("%s: HW BP not found!", target->cmd_name);
		for(slot = 0; slot < ESP32_FLASH_SW_BREAKPOINTS_MAX_NUM; slot++) {
			if(esp32->flash_sw_brps[slot] != NULL && esp32->flash_sw_brps[slot]->data.oocd_bp == breakpoint)
				break;
		}
		if (slot == ESP32_FLASH_SW_BREAKPOINTS_MAX_NUM) {
			LOG_WARNING("%s: SW flash BP not found!", __func__);
			return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		}
		int ret = esp32_remove_flash_breakpoint(target, esp32->flash_sw_brps[slot]);
		if (ret != ERROR_OK) {
			LOG_ERROR("%s: Failed to remove SW flash BP (%d)!", target->cmd_name, ret);
			return ret;
		}
		esp32->flash_sw_brps[slot] = NULL;
		LOG_DEBUG("%s: cleared SW_FLASH breakpoint @ 0x%X", target->cmd_name, breakpoint->address);
		return ERROR_OK;
	}
	esp32->hw_brps[slot] = NULL;
	LOG_DEBUG("%s: cleared HW breakpoint %d at 0x%X", target->cmd_name, (int)slot, breakpoint->address);
	return ERROR_OK;
}

static int xtensa_add_watchpoint(struct target *target, struct watchpoint *watchpoint)
{
	struct esp32_common *esp32=(struct esp32_common*)target->arch_info;
	size_t slot;
	int dbreakcval;

	if (target->state != TARGET_HALTED) {
		LOG_WARNING("%s: %s: target not halted", __func__, target->cmd_name);
		return ERROR_TARGET_NOT_HALTED;
	}

	if (watchpoint->mask != ~(uint32_t)0) {
		LOG_ERROR("%s: watchpoint value masks not supported", target->cmd_name);
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	for(slot = 0; slot < esp32->num_wps; slot++) {
		if (esp32->hw_wps[slot] == NULL || esp32->hw_wps[slot] == watchpoint) break;
	}
	if (slot==esp32->num_wps) return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;

	//Figure out value for dbreakc5..0
	//It's basically 0x3F with an incremental bit removed from the LSB for each extra length power of 2.
	//Yes, I could have calculated this, but for 7 options...
	dbreakcval=0xaa; //invalid, so we can check if this is changed later
	if (watchpoint->length==1 && (watchpoint->address&0x00)==0) dbreakcval=0x3F;
	if (watchpoint->length==2 && (watchpoint->address&0x01)==0) dbreakcval=0x3E;
	if (watchpoint->length==4 && (watchpoint->address&0x03)==0) dbreakcval=0x3C;
	if (watchpoint->length==8 && (watchpoint->address&0x07)==0) dbreakcval=0x38;
	if (watchpoint->length==16 && (watchpoint->address&0x0F)==0) dbreakcval=0x30;
	if (watchpoint->length==32 && (watchpoint->address&0x1F)==0) dbreakcval=0x20;
	if (watchpoint->length==64 && (watchpoint->address&0x3F)==0) dbreakcval=0x00;
	if (dbreakcval==0xaa) {
		LOG_WARNING("%s: Watchpoint with length %d on address 0x%X not supported by hardware.", target->cmd_name, watchpoint->length, watchpoint->address);
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	if (watchpoint->rw==WPT_READ) dbreakcval|=(1<<30);
	if (watchpoint->rw==WPT_WRITE) dbreakcval|=(1<<31);
	if (watchpoint->rw==WPT_ACCESS) dbreakcval|=(1<<30)+(1<<31);

	/* Write DBREAKA[slot] and DBCREAKC[slot]*/
	for (size_t core = 0; core < ESP32_CPU_COUNT; core++)
	{
		struct reg *cpu_reg_list = esp32->core_caches[core]->reg_list;
		esp108_reg_set(&cpu_reg_list[XT_REG_IDX_DBREAKA0 + slot], watchpoint->address);
		esp108_reg_set(&cpu_reg_list[XT_REG_IDX_DBREAKC0 + slot], dbreakcval);
	}
	esp32->hw_wps[slot] = watchpoint;

	return ERROR_OK;
}

static int xtensa_remove_watchpoint(struct target *target, struct watchpoint *watchpoint)
{
	struct esp32_common *esp32=(struct esp32_common*)target->arch_info;
	size_t slot;

	for(slot = 0; slot < esp32->num_wps; slot++) {
		if(esp32->hw_wps[slot] == watchpoint)
			break;
	}
	if (slot==esp32->num_wps) return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;

	/* Clear DBREAKC[slot] to disable watchpoint */
	for (size_t core = 0; core < ESP32_CPU_COUNT; core++)
	{
		struct reg *cpu_reg_list = esp32->core_caches[core]->reg_list;
		esp108_reg_set(&cpu_reg_list[XT_REG_IDX_DBREAKC0 + slot], 0);
	}
	esp32->hw_wps[slot] = NULL;

	return ERROR_OK;

}
#define XCHAL_EXCM_LEVEL		3	/* level masked by PS.EXCM */

static bool pc_in_window_exception(struct target *target, uint32_t pc)
{
	uint32_t insn;
	int err = xtensa_read_memory(target, pc, 4, 1, (uint8_t*) &insn);
	if (err != ERROR_OK) {
		return false;
	}

	uint32_t masked = insn & XT_INS_L32E_S32E_MASK;
	if (masked == XT_INS_L32E(0, 0, 0) || masked == XT_INS_S32E(0, 0, 0)) {
		return true;
	}

	masked = insn & XT_INS_RFWO_RFWU_MASK;
	if (masked == XT_INS_RFWO || masked == XT_INS_RFWU) {
		return true;
	}

	return false;
}

static int xtensa_do_step(struct target *target,
	int current,
	uint32_t address,
	int handle_breakpoints)
{
	struct esp32_common *esp32=(struct esp32_common*)target->arch_info;
	struct reg *reg_list = esp32->core_caches[esp32->active_cpu]->reg_list;
	int res, cause;
	uint32_t dbreakc[esp32->num_wps*4];
	size_t slot;
	uint8_t dsr[4];
	static const uint32_t icount_val = -2; /* ICOUNT value to load for 1 step */
	uint32_t icountlvl;
	uint32_t oldps, newps, oldpc, cur_pc;

	LOG_DEBUG("%s: %s(current=%d, address=0x%04x, handle_breakpoints=%i)", target->cmd_name, __func__, current, address, handle_breakpoints);

	if (target->state != TARGET_HALTED) {
		LOG_WARNING("%s: %s: target not halted", __func__, target->cmd_name);
		return ERROR_TARGET_NOT_HALTED;
	}

	//Save old ps/pc
	oldps=esp108_reg_get(&reg_list[XT_REG_IDX_PS]);
	oldpc=esp108_reg_get(&reg_list[XT_REG_IDX_PC]);

	// Increasing the interrupt level to avoid context switching from C++ interrupts.
	if (esp32->isrmasking_mode == ESP32_ISRMASK_ON)
	{
		uint32_t temp_ps = (oldps&~0xf) | XCHAL_EXCM_LEVEL;
		esp108_reg_set(&reg_list[XT_REG_IDX_PS], temp_ps);
		// Now we have to set up max posssible interrupt level (ilevel + 1)
		icountlvl = XCHAL_EXCM_LEVEL + 1;
	}
	else
	{
		icountlvl = (oldps & 0xf) + 1;
	}

	cause=esp108_reg_get(&reg_list[XT_REG_IDX_DEBUGCAUSE]);
	if (cause&DEBUGCAUSE_DB) {
		//We stopped due to a watchpoint. We can't just resume executing the instruction again because
		//that would trigger the watchpoint again. To fix this, we remove watchpoints, single-step and
		//re-enable the watchpoint.
		LOG_DEBUG("%s: Single-stepping to get past instruction that triggered the watchpoint...", esp32->esp32_targets[esp32->active_cpu]->cmd_name);
		esp108_reg_set(&reg_list[XT_REG_IDX_DEBUGCAUSE], 0); //so we don't recurse into the same routine
		reg_list[XT_REG_IDX_DEBUGCAUSE].dirty=0;
		//Save all DBREAKCx registers and set to 0 to disable watchpoints
		for(slot = 0; slot < esp32->num_wps; slot++) {
			dbreakc[slot]=esp108_reg_get(&reg_list[XT_REG_IDX_DBREAKC0+slot]);
			esp108_reg_set(&reg_list[XT_REG_IDX_DBREAKC0+slot], 0);
		}
	}

	do {
		esp108_reg_set(&reg_list[XT_REG_IDX_ICOUNTLEVEL], icountlvl);
		esp108_reg_set(&reg_list[XT_REG_IDX_ICOUNT], icount_val);

		/* Now ICOUNT is set, we can resume as if we were going to run */
		res = xtensa_resume_active_cpu(target, current, address, 0, 0);
		if (res != ERROR_OK) {
			LOG_ERROR("%s: %s: Failed to resume after setting up single step", target->cmd_name, __func__);
			return res;
		}

		/* Wait for stepping to complete */
		int64_t start = timeval_ms();
		while(timeval_ms() < start+500) {
			//Do not use target_poll here, it also triggers other things... just manually read the DSR until stepping
			//is complete.
			esp108_queue_nexus_reg_read(esp32->esp32_targets[esp32->active_cpu], NARADR_DSR, dsr);
			esp108_queue_tdi_idle(esp32->esp32_targets[esp32->active_cpu]);
			res=jtag_execute_queue();
			if(res != ERROR_OK) return res;
			if (intfromchars(dsr)&OCDDSR_STOPPED) {
				break;
			} else {
				usleep(50000);
			}
		}
		if(!(intfromchars(dsr)&OCDDSR_STOPPED)) {
			LOG_WARNING("%s: %s: Timed out waiting for target to finish stepping. dsr=0x%08x", target->cmd_name, __func__, intfromchars(dsr));
			target->debug_reason = DBG_REASON_NOTHALTED;
			target->state = TARGET_RUNNING;
			return ERROR_OK;
		}
		target->debug_reason = DBG_REASON_SINGLESTEP;
		target->state = TARGET_HALTED;

		esp32_fetch_all_regs(target, 1 << esp32->active_cpu);

		cur_pc = esp108_reg_get(&reg_list[XT_REG_IDX_PC]);

		if (esp32->isrmasking_mode == ESP32_ISRMASK_ON &&
			pc_in_window_exception(target, cur_pc))
		{
			/* isrmask = on, need to step out of the window exception handler */
			LOG_DEBUG("Stepping out of window exception, PC=%X", cur_pc);
			oldpc = cur_pc;
			address = oldpc + 3;
			continue;
		}

		if (oldpc == cur_pc) {
			LOG_WARNING("%s: %s: Stepping doesn't seem to change PC! dsr=0x%08x", target->cmd_name, __func__, intfromchars(dsr));
		}
		else {
			LOG_DEBUG("Stepped from %X to %X", oldpc, cur_pc);
		}
		break;
	}
	while(true);
	LOG_DEBUG("Done stepping, PC=%X", cur_pc);

	// This operation required to clear state
	esp108_clear_dsr(esp32->esp32_targets[esp32->active_cpu == ESP32_PRO_CPU_ID ? ESP32_APP_CPU_ID : ESP32_PRO_CPU_ID],
		OCDDSR_DEBUGPENDBREAK|OCDDSR_DEBUGINTBREAK);

	if (cause&DEBUGCAUSE_DB) {
		LOG_DEBUG("%s: ...Done, re-instating watchpoints.", target->cmd_name);
		//Restore the DBREAKCx registers
		for(slot = 0; slot < esp32->num_wps; slot++) {
			esp108_reg_set(&reg_list[XT_REG_IDX_DBREAKC0+slot], dbreakc[slot]);
		}
	}

	//Restore int level
	//ToDo: Theoretically, this can mess up stepping over an instruction that modifies ps.intlevel
	//by itself. Hmmm. ToDo: Look into this.
	if (esp32->isrmasking_mode == ESP32_ISRMASK_ON){
		newps = esp108_reg_get(&reg_list[XT_REG_IDX_PS]);
		newps = (newps&~0xf) | (oldps & 0xf);
		esp108_reg_set(&reg_list[XT_REG_IDX_PS], newps);
	}

	/* write ICOUNTLEVEL back to zero */
	// We have equival amount of BP for each cpu
	esp108_reg_set(&reg_list[XT_REG_IDX_ICOUNTLEVEL], 0);
	res = esp32_write_dirty_registers(esp32->esp32_targets[esp32->active_cpu], reg_list);

	esp32_fetch_all_regs(target, 1 << esp32->active_cpu);
	return res;
}

static int xtensa_step(struct target *target,
	int current,
	uint32_t address,
	int handle_breakpoints)
{
	int retval = xtensa_do_step(target, current, address, handle_breakpoints);
	if (retval != ERROR_OK) {
		return retval;
	}
	target_call_event_callbacks(target, TARGET_EVENT_HALTED);
	return retval;
}

static int xtensa_start_algorithm(struct target *target,
	int num_mem_params, struct mem_param *mem_params,
	int num_reg_params, struct reg_param *reg_params,
	uint32_t entry_point, uint32_t exit_point,
	void *arch_info)
{
	struct esp32_common *esp32 = (struct esp32_common*)target->arch_info;
	struct reg_cache *core_cache = esp32->core_caches[esp32->active_cpu];

	int retval = xtensa_start_algorithm_generic(target, num_mem_params, mem_params, num_reg_params,
		reg_params, entry_point, exit_point, arch_info, core_cache);
	if (retval != ERROR_OK) {
		return retval;
	}

	return xtensa_resume_active_cpu(target, 0, entry_point, 1, 1);
}

/** Waits for an algorithm in the target. */
static int xtensa_wait_algorithm(struct target *target,
	int num_mem_params, struct mem_param *mem_params,
	int num_reg_params, struct reg_param *reg_params,
	uint32_t exit_point, int timeout_ms,
	void *arch_info)
{
	int retval = ERROR_OK;
	struct esp32_common *esp32 = (struct esp32_common*)target->arch_info;
	struct reg_cache *core_cache = esp32->core_caches[esp32->active_cpu];

	retval = xtensa_wait_algorithm_generic(target, num_mem_params, mem_params, num_reg_params,
		reg_params, exit_point, timeout_ms, arch_info, core_cache);
	if (retval != ERROR_OK) {
		return retval;
	}

	retval = esp32_write_dirty_registers(esp32->esp32_targets[esp32->active_cpu], core_cache->reg_list);
	if (retval != ERROR_OK) {
		LOG_ERROR("Failed to write dirty regs (%d)!", retval);
	}

	return retval;
}


static const struct reg_arch_type esp32_reg_type = {
	.get = xtensa_get_core_reg,
	.set = xtensa_set_core_reg,
};

static int xtensa_target_create(struct target *target, Jim_Interp *interp)
{
	struct esp32_common *esp32 = calloc(1, sizeof(struct esp32_common));
	struct reg_cache **cache_p = register_get_last_cache_p(&target->reg_cache);
	struct reg_cache *cache = malloc(sizeof(struct reg_cache));
	struct reg *reg_list = calloc(XT_NUM_REGS, sizeof(struct reg));

	if (!esp32)
		return ERROR_COMMAND_SYNTAX_ERROR;

	int ret = target_register_event_callback(esp32_handle_target_event, NULL);
	if (ret != ERROR_OK) {
		LOG_ERROR("Failed to register target event callback (%d)!", ret);
		return ret;
	}

	target->arch_info = esp32;
	esp32->target = target; //TODO: not actually necessary

	esp32->cores_num = 0; // unknown
	esp32->num_brps = XT_NUM_BREAKPOINTS;
	esp32->hw_brps = calloc(XT_NUM_BREAKPOINTS, sizeof(struct breakpoint *));
	esp32->num_wps = XT_NUM_WATCHPOINTS;
	esp32->hw_wps = calloc(XT_NUM_WATCHPOINTS, sizeof(struct watchpoint *));
	esp32->flash_sw_brps = calloc(ESP32_FLASH_SW_BREAKPOINTS_MAX_NUM, sizeof(struct esp32_flash_sw_breakpoint *));
	esp32->sw_brps = calloc(ESP32_SW_BREAKPOINTS_MAX_NUM, sizeof(struct esp32_sw_breakpoint	*));
	esp32->appimage_flash_base = (uint32_t)-1;

	//Create the register cache
	cache->name = "Xtensa registers";
	cache->next = NULL;
	cache->reg_list = reg_list;
	cache->num_regs = XT_NUM_REGS;
	*cache_p = cache;
	esp32->core_cache = NULL;// not used

	for (int i = 0; i < ESP32_CPU_COUNT; i++) {
		esp32->esp32_targets[i] = malloc(sizeof(struct target));
		if (esp32->esp32_targets[i] == NULL) return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;

		memcpy(esp32->esp32_targets[i], target, sizeof(struct target));
		if (i != ESP32_PRO_CPU_ID) {
			esp32->esp32_targets[i]->tap = malloc(sizeof(struct jtag_tap));
			memcpy(esp32->esp32_targets[i]->tap, target->tap, sizeof(struct jtag_tap));
			esp32->esp32_targets[i]->tap->tapname = "cpu1";
			esp32->esp32_targets[i]->tap->dotted_name = "esp32.cpu1";
			jtag_tap_init(esp32->esp32_targets[i]->tap);
		}
		esp32->esp32_targets[i]->coreid = i;
	}
	esp32->esp32_targets[0]->cmd_name = "cpu0";
	esp32->esp32_targets[1]->cmd_name = "cpu1";

	for (int cp = 0; cp < ESP32_CPU_COUNT; cp++)
	{
		struct reg_cache *cpu_cache = malloc(sizeof(struct reg_cache));

		// Init reglist
		esp32->core_caches[cp] = cpu_cache;
		struct reg *cpu_reg_list = calloc(XT_NUM_REGS, sizeof(struct reg));

		cpu_cache->name = "Xtensa registers";
		cpu_cache->next = NULL;
		cpu_cache->reg_list = cpu_reg_list;
		cpu_cache->num_regs = XT_NUM_REGS;
		esp32->core_caches[cp] = cpu_cache;
		for (int i = 0; i < XT_NUM_REGS; i++)
		{
			cpu_reg_list[i].name = esp32_regs[i].name;
			cpu_reg_list[i].size = 32;
			cpu_reg_list[i].value = calloc(1, 4);
			cpu_reg_list[i].dirty = 0;
			cpu_reg_list[i].valid = 0;
			cpu_reg_list[i].type = &esp32_reg_type;
			cpu_reg_list[i].arch_info = esp32;
		}
	}
	esp32->core_caches[0]->name = "Xtensa registers CPU0";
	esp32->core_caches[1]->name = "Xtensa registers CPU1";

	for (int i = 0; i < ESP32_CPU_COUNT; i++)
	{
		esp32->esp32_targets[i]->reg_cache = esp32->core_caches[i];
	}
	esp32->flashBootstrap = FBS_DONTCARE;
	esp32->isrmasking_mode = ESP32_ISRMASK_ON;

	for(int i = 0; i < XT_NUM_REGS; i++) {
		reg_list[i].name = esp32_regs[i].name;
		reg_list[i].size = 32;
		reg_list[i].value = calloc(1,4);
		reg_list[i].dirty = 0;
		reg_list[i].valid = 0;
		reg_list[i].type = &esp32_reg_type;
		reg_list[i].arch_info = esp32;
	}

	//Assume running target. If different, the first poll will fix this.
	target->state = TARGET_RUNNING;
	target->debug_reason = DBG_REASON_NOTHALTED;
	esp32->active_cpu = ESP32_PRO_CPU_ID;

	return ERROR_OK;
}

int xtensa_on_exit(struct target *target, void *priv)
{
	int ret = esp32_dbgstubs_restore(target);
	if (ret != ERROR_OK) {
		return ret;
	}
	return ERROR_OK;
}

static int xtensa_init_target(struct command_context *cmd_ctx, struct target *target)
{
	struct esp32_common *esp32=(struct esp32_common*)target->arch_info;

	LOG_DEBUG("%s", __func__);
	int ret = target_register_exit_callback(xtensa_on_exit, NULL);
	if (ret != ERROR_OK) {
		return ret;
	}

	esp32->state = XT_NORMAL; // Assume normal state until we examine

	return ERROR_OK;
}

static size_t esp32_read_cores_num(struct target *target)
{
	size_t cores_num = 1;
	uint32_t appcpu_ctrl = 0;
	int res;

	LOG_DEBUG("Read cores number");
	res = xtensa_read_memory(target, ESP32_DPORT_APPCPU_CTRL_B_REG, sizeof(uint32_t), 1, (uint8_t *)&appcpu_ctrl);
	if (res != ERROR_OK) {
		LOG_ERROR("Failed to read target memory (%d)!", res);
		return 0;
	}
	LOG_DEBUG("Read APP CPU ctrl reg 0x%x", appcpu_ctrl);
	if (appcpu_ctrl & ESP32_DPORT_APPCPU_CLKGATE_EN) {
		cores_num++;
		LOG_DEBUG("APP CPU enabled");
	}
	LOG_DEBUG("Detected %u cores", (uint32_t)cores_num);
	return cores_num;
}

static int xtensa_poll(struct target *target)
{
	struct esp32_common *esp32=(struct esp32_common*)target->arch_info;
	uint8_t pwrstat[ESP32_CPU_COUNT], pwrstath[ESP32_CPU_COUNT];
	int res, cmd;
	uint8_t traxstat[ESP32_CPU_COUNT][4] = {{0}}, traxctl[ESP32_CPU_COUNT][4] = {{0}};
	unsigned int dsr[ESP32_CPU_COUNT] = {0};
	uint32_t idcode[ESP32_CPU_COUNT] = {0};

	//Read reset state
	uint32_t core_poweron_mask = 0;
	for (size_t i = 0; i < ESP32_CPU_COUNT; i++)
	{
		esp108_queue_idcode_read(esp32->esp32_targets[i], (uint8_t*) &idcode[i]);
		esp108_queue_pwrstat_readclear(esp32->esp32_targets[i], &pwrstat[i]);
		//Read again, to see if the state holds...
		esp108_queue_pwrstat_readclear(esp32->esp32_targets[i], &pwrstath[i]);
		esp108_queue_tdi_idle(esp32->esp32_targets[i]);
		res = jtag_execute_queue();
		if (res != ERROR_OK) return res;
		if (idcode[i] != 0xffffffff && idcode[i] != 0) {
			core_poweron_mask |= (1 << i);
		}
	}

	// Target might be held in reset by external signal.
	// Sanity check target responses using idcode (checking CPU0 is sufficient).
	if (core_poweron_mask == 0) {
		if (target->state != TARGET_UNKNOWN) {
			LOG_INFO("%s: Target offline", __func__);
			target->state = TARGET_UNKNOWN;
		}
		memset(esp32->prevpwrstat, 0, sizeof(esp32->prevpwrstat));
		esp32->core_poweron_mask = 0;
		return ERROR_TARGET_FAILURE;
	}
	uint32_t cores_came_online = core_poweron_mask & (~esp32->core_poweron_mask);
	esp32->core_poweron_mask = core_poweron_mask;
	if (cores_came_online != 0) {
		LOG_DEBUG("%s: core_poweron_mask=%x", __func__, core_poweron_mask);
	}

	for (size_t i = 0; i < ESP32_CPU_COUNT; i++)
	{
		if (!(esp32->prevpwrstat[i]&PWRSTAT_DEBUGWASRESET) && pwrstat[i] & PWRSTAT_DEBUGWASRESET) {
			LOG_INFO("%s: Debug controller %d was reset (pwrstat=0x%02X, after clear 0x%02X).", target->cmd_name, (int)i, pwrstat[i], pwrstath[i]);
			esp32->core_poweron_mask &= ~(1 << i);
			//esp32->core_poweron_mask = 0;
		}
		if (!(esp32->prevpwrstat[i]&PWRSTAT_COREWASRESET) && pwrstat[i] & PWRSTAT_COREWASRESET) {
			LOG_INFO("%s: Core %d was reset (pwrstat=0x%02X, after clear 0x%02X).", target->cmd_name, (int)i, pwrstat[i], pwrstath[i]);
			if (esp32->cores_num > 0) {
				esp32->cores_num = 0; // unknown
				memset(&esp32->dbg_stubs, 0, sizeof(struct esp32_dbg_stubs));
			}
		}
		esp32->prevpwrstat[i] = pwrstath[i];
	}

	//Enable JTAG, set reset if needed
	cmd=PWRCTL_DEBUGWAKEUP|PWRCTL_MEMWAKEUP|PWRCTL_COREWAKEUP;
	if (esp32->resetAsserted)
	{
		cmd |= PWRCTL_CORERESET;
	}
	for (size_t i = 0; i < ESP32_CPU_COUNT; i++)
	{
		esp108_queue_pwrctl_set(esp32->esp32_targets[i], cmd);
		esp108_queue_pwrctl_set(esp32->esp32_targets[i], cmd | PWRCTL_JTAGDEBUGUSE);
		esp108_queue_tdi_idle(esp32->esp32_targets[i]);
		res = jtag_execute_queue();
		if (res != ERROR_OK) return res;
	}

	for (size_t i = 0; i < ESP32_CPU_COUNT; i++)
	{
		if (cores_came_online & (1 << i)) {
			LOG_DEBUG("%s: Core %d came online, setting up DCR", __func__, (int) i);
			xtensa_smpbreak_set_core(target, i);
		}

		uint8_t dsr_buf[4] = {0};
		esp108_queue_nexus_reg_write(esp32->esp32_targets[i], NARADR_DCRSET, OCDDCR_ENABLEOCD);
		esp108_queue_nexus_reg_read(esp32->esp32_targets[i], NARADR_DSR, dsr_buf);
		esp108_queue_nexus_reg_read(esp32->esp32_targets[i], NARADR_TRAXSTAT, traxstat[i]);
		esp108_queue_nexus_reg_read(esp32->esp32_targets[i], NARADR_TRAXCTRL, traxctl[i]);
		esp108_queue_tdi_idle(esp32->esp32_targets[i]);
		res = jtag_execute_queue();
		if (res != ERROR_OK) return res;
		dsr[i] = intfromchars(dsr_buf);

	}

	unsigned int common_reason = dsr[0] | dsr[1]; // We should know if even one of CPU was stopped
	unsigned int common_pwrstath = pwrstath[0] | pwrstath[1];
	if (common_reason & OCDDSR_STOPPED) {
		int oldstate=target->state;
		size_t cores_num = esp32_get_cores_count(target);
		bool algo_stopped = false;
		if (target->state == TARGET_DEBUG_RUNNING) {
			if (cores_num == ESP32_CPU_COUNT) {
				/* algo can be run on any CPU while other one can be stalled by SW run on target, in this case OCDDSR_STOPPED will not be set for other CPU;
				 situation when target is in TARGET_DEBUG_RUNNING and both CPUs are stalled is impossible (one must run algo). */
				algo_stopped = (dsr[0] & (OCDDSR_STOPPED|OCDDSR_RUNSTALLSAMPLE)) && (dsr[1] & (OCDDSR_STOPPED|OCDDSR_RUNSTALLSAMPLE));
			} else {
				algo_stopped = (dsr[0] & OCDDSR_STOPPED);
			}
		}
		if((target->state != TARGET_HALTED && target->state != TARGET_DEBUG_RUNNING) || algo_stopped) {
			LOG_DEBUG("Stopped: CPU0: %d CPU1: %d", (dsr[0] & OCDDSR_STOPPED) ? 1 : 0, (dsr[1] & OCDDSR_STOPPED) ? 1 : 0);
			// TODO: When BreakIn/BreakOut is enabled other CPU is stopped automatically.
			// Should we stop the second CPU if BreakIn/BreakOut is not configured?
			target->state = TARGET_HALTED;
			esp32_fetch_all_regs(target, 0x3);
			//Examine why the target was halted
			target->debug_reason = DBG_REASON_DBGRQ;
			uint32_t halt_cause[ESP32_CPU_COUNT] = {0};
			for (size_t i = 0; i < ESP32_CPU_COUNT; i++)
			{
				struct reg *cpu_reg_list = esp32->core_caches[i]->reg_list;
				int exc_cause = esp108_reg_get(&cpu_reg_list[XT_REG_IDX_EXCCAUSE]);
				int cause = esp108_reg_get(&cpu_reg_list[XT_REG_IDX_DEBUGCAUSE]);

				halt_cause[i] = (uint32_t)cause;
				esp108_clear_dsr(esp32->esp32_targets[i], OCDDSR_DEBUGPENDBREAK|OCDDSR_DEBUGINTBREAK|OCDDSR_DEBUGPENDHOST|OCDDSR_DEBUGINTHOST);

				LOG_DEBUG("%s: Target halted, pc=0x%08X, debug_reason=%08x, oldstate=%08x, active=%s", esp32->esp32_targets[i]->cmd_name,
					esp108_reg_get(&cpu_reg_list[XT_REG_IDX_PC]), target->debug_reason, oldstate, (i == esp32->active_cpu) ? "true" : "false");
				LOG_DEBUG("%s: Halt reason=0x%08X, exc_cause=%d, dsr=0x%08x",
					esp32->esp32_targets[i]->cmd_name, cause, exc_cause, dsr[i]);
			}
			// When setting debug reason DEBUGCAUSE events have the followuing priorites: watchpoint == breakpoint > single step > debug interrupt.
			// Watchpoint and breakpoint events at the same time results in special debug reason: DBG_REASON_WPTANDBKPT.
			// When similar debug events are present on both CPUs events on CPU0 take priority over CPU1 ones and CPU0 is considred to be active.
			// TODO: review this scheme for the case when BreakIn/BreakOut is not enabled
			for (size_t k = ESP32_CPU_COUNT; k > 0; k--) {
				if ((halt_cause[k-1] & DEBUGCAUSE_DI) && ((dsr[k-1]&(OCDDSR_DEBUGPENDHOST|OCDDSR_DEBUGINTHOST)) != 0))
				{
					target->debug_reason = DBG_REASON_DBGRQ;
					esp32->active_cpu = k-1;
				}
			}
			for (size_t k = ESP32_CPU_COUNT; k > 0; k--) {
				if (halt_cause[k-1] & DEBUGCAUSE_IC)
				{
					target->debug_reason = DBG_REASON_SINGLESTEP;
					esp32->active_cpu = k-1;
				}
			}
			for (size_t k = ESP32_CPU_COUNT; k > 0; k--) {
				if (halt_cause[k-1] & (DEBUGCAUSE_IB | DEBUGCAUSE_BN | DEBUGCAUSE_BI))
				{
					if (halt_cause[k-1] & DEBUGCAUSE_DB) {
						target->debug_reason = DBG_REASON_WPTANDBKPT;
					} else {
						target->debug_reason = DBG_REASON_BREAKPOINT;
					}
					esp32->active_cpu = k-1;
				} else if (halt_cause[k-1] & DEBUGCAUSE_DB)
				{
					target->debug_reason = DBG_REASON_WATCHPOINT;
					esp32->active_cpu = k-1;
				}
			}

			LOG_INFO("Target halted. PRO_CPU: PC=0x%08X %s    APP_CPU: PC=0x%08X %s",
				esp108_reg_get(&esp32->core_caches[0]->reg_list[XT_REG_IDX_PC]),
				esp32->active_cpu == 0 ? "(active)" : "        ",
				esp108_reg_get(&esp32->core_caches[1]->reg_list[XT_REG_IDX_PC]),
				esp32->active_cpu == 1 ? "(active)" : "" );

			target->reg_cache = esp32->core_caches[esp32->active_cpu & 1];
			target->coreid = esp32->active_cpu;

			//Call any event callbacks that are applicable
			if (oldstate == TARGET_DEBUG_RUNNING) {
				target_call_event_callbacks(target, TARGET_EVENT_DEBUG_HALTED);
			}
			else {
				target_call_event_callbacks(target, TARGET_EVENT_HALTED);
			}
		}
	} else if (common_pwrstath & PWRSTAT_COREWASRESET) {
		target->state = TARGET_RESET;
	} else {
		target->debug_reason = DBG_REASON_NOTHALTED;
		if (target->state!=TARGET_RUNNING && target->state!=TARGET_DEBUG_RUNNING) {
			target->state = TARGET_RUNNING;
			target->debug_reason = DBG_REASON_NOTHALTED;
		}
	}

	if (esp32->traceActive) {
		//Detect if tracing was active but has stopped.
		for (size_t core = 0; core < ESP32_CPU_COUNT; core++)
		{
			if ((intfromchars(traxctl[core])&TRAXCTRL_TREN) && (!(intfromchars(traxstat[core])&TRAXSTAT_TRACT))) {
				LOG_INFO("Detected end of trace.");
				LOG_INFO("TRAXSTAT[%s]= 0x%08x", esp32->esp32_targets[core]->cmd_name, intfromchars(traxstat[core]));
				if (intfromchars(traxstat[core])&TRAXSTAT_PCMTG) LOG_INFO("%s: Trace stop triggered by PC match", esp32->esp32_targets[core]->cmd_name);
				if (intfromchars(traxstat[core])&TRAXSTAT_PTITG) LOG_INFO("%s: Trace stop triggered by Processor Trigger Input", esp32->esp32_targets[core]->cmd_name);
				if (intfromchars(traxstat[core])&TRAXSTAT_CTITG) LOG_INFO("%s: Trace stop triggered by Cross-trigger Input", esp32->esp32_targets[core]->cmd_name);
				esp32->traceActive=0;
			}
		}
	}
	if (target->state != TARGET_DEBUG_RUNNING && esp32->dbg_stubs.base == 0) {
		esp32->dbg_stubs.base = esp32_dbgstubs_get(target);
	}
	return ERROR_OK;
}

int xtensa_esp32_examine(struct target *target)
{
	struct esp32_common *esp32=(struct esp32_common*)target->arch_info;
	int res, cmd;
	uint32_t idcode[ESP32_CPU_COUNT] = {0}, dsr[ESP32_CPU_COUNT] = {0};

	cmd = PWRCTL_DEBUGWAKEUP | PWRCTL_MEMWAKEUP | PWRCTL_COREWAKEUP;
	for (size_t i = 0; i < ESP32_CPU_COUNT; i++)
	{
		esp108_queue_pwrctl_set(esp32->esp32_targets[i], cmd);
		esp108_queue_pwrctl_set(esp32->esp32_targets[i], cmd | PWRCTL_JTAGDEBUGUSE);
		esp108_queue_nexus_reg_write(esp32->esp32_targets[i], NARADR_DCRSET, OCDDCR_ENABLEOCD);
		esp108_queue_nexus_reg_read(esp32->esp32_targets[i], NARADR_DSR, (uint8_t*) &dsr[i]);
		esp108_queue_idcode_read(esp32->esp32_targets[i], (uint8_t*) &idcode[i]);
		esp108_queue_tdi_idle(esp32->esp32_targets[i]);
		res = jtag_execute_queue();
		if (res != ERROR_OK) {
			return res;
		}
	}
	if (idcode[0] == 0xffffffff || idcode[0] == 0) {
		LOG_DEBUG("%s: Target idcode=%08x", __func__, idcode[0]);
		return ERROR_TARGET_FAILURE;
	}


	if (!target_was_examined(target)) {
		target_set_examined(target);
	}
	return ERROR_OK;
}

static int xtensa_arch_state(struct target *target)
{
	LOG_DEBUG("%s", __func__);
	return ERROR_OK;
}

static inline bool esp32_data_addr_valid(uint32_t addr)
{
	return ((addr >= ESP32_DROM_LOW) && (addr < ESP32_DRAM_HIGH)) ? true : false;
}

static uint32_t esp32_dbgstubs_get(struct target *target)
{
	struct esp32_common *esp32 = (struct esp32_common *)target->arch_info;
	uint32_t vec_addr = 0, addr = 0;

	int res = esp108_apptrace_read_status(esp32->esp32_targets[esp32->active_cpu], &vec_addr);
	if (res != ERROR_OK) {
		LOG_ERROR("Failed to read trace status (%d)!", res);
		return 0;
	}
	if (esp32_data_addr_valid(vec_addr)) {
		LOG_INFO("Detected debug stubs @ %x on core%u of target '%s'", vec_addr, (uint32_t)esp32->active_cpu, target_type_name(target));
		res = esp108_apptrace_write_status(esp32->esp32_targets[esp32->active_cpu], 0);
		if (res != ERROR_OK) {
			LOG_ERROR("Failed to clear trace status (%d)!", res);
		}
	}
	res = esp108_apptrace_read_status(esp32->esp32_targets[esp32->active_cpu ? 0 : 1], &addr);
	if (res != ERROR_OK) {
		LOG_ERROR("Failed to read trace status (%d)!", res);
		return 0;
	}
	if (esp32_data_addr_valid(addr)) {
		LOG_INFO("Detected debug stubs @ %x on core%d of target '%s'", addr, esp32->active_cpu ? 0 : 1, target_type_name(target));
		res = esp108_apptrace_write_status(esp32->esp32_targets[esp32->active_cpu ? 0 : 1], 0);
		if (res != ERROR_OK) {
			LOG_ERROR("Failed to clear trace status (%d)!", res);
		}
		if (!esp32_data_addr_valid(vec_addr)) {
			vec_addr = addr;
		}
	}
	// not set yet, or there is no program with enabled dbg stubs in flash
	return esp32_data_addr_valid(vec_addr) ? vec_addr : 0;
}

static int esp32_dbgstubs_update_info(struct target *target)
{
	struct esp32_common *esp32 = (struct esp32_common *)target->arch_info;

	int res = target_read_memory(target, esp32->dbg_stubs.base, sizeof(uint32_t),
							ESP32_DBG_STUB_ENTRY_MAX-ESP32_DBG_STUB_TABLE_START,
							(uint8_t *)&esp32->dbg_stubs.entries);
	if (res != ERROR_OK) {
		LOG_ERROR("Failed to halt target '%s' (%d)!", target_state_name(target), res);
		return res;
	}
	for (esp32_dbg_stub_id_t i = ESP32_DBG_STUB_TABLE_START; i < ESP32_DBG_STUB_ENTRY_MAX; i++) {
		LOG_DEBUG("Check dbg stub %d", i);
		if (esp32->dbg_stubs.entries[i]) {
			esp32->dbg_stubs.entries[i] = intfromchars((uint8_t *)&esp32->dbg_stubs.entries[i]);
			LOG_DEBUG("New dbg stub %d at %x", esp32->dbg_stubs.entries_count, esp32->dbg_stubs.entries[i]);
			esp32->dbg_stubs.entries_count++;
		}
	}
	if (esp32->dbg_stubs.entries_count < (ESP32_DBG_STUB_ENTRY_MAX-ESP32_DBG_STUB_TABLE_START)) {
		LOG_DEBUG("Not full dbg stub table %d of %d", esp32->dbg_stubs.entries_count, (ESP32_DBG_STUB_ENTRY_MAX-ESP32_DBG_STUB_TABLE_START));
		esp32->dbg_stubs.entries_count = 0;
		return ERROR_OK;
	}
	// read debug stubs descriptor
	ESP32_DBGSTUBS_UPDATE_DATA_ENTRY(esp32->dbg_stubs.entries[ESP32_DBG_STUB_DESC]);
	res = target_read_memory(target, esp32->dbg_stubs.entries[ESP32_DBG_STUB_DESC], sizeof(struct esp32_dbg_stubs_desc), 1, (uint8_t *)&esp32->dbg_stubs.desc);
	if (res != ERROR_OK) {
		LOG_ERROR("Failed to read target memory (%d)!", res);
		return res;
	}
	ESP32_DBGSTUBS_UPDATE_CODE_ENTRY(esp32->dbg_stubs.desc.tramp_addr);
	ESP32_DBGSTUBS_UPDATE_DATA_ENTRY(esp32->dbg_stubs.desc.min_stack_addr);
	ESP32_DBGSTUBS_UPDATE_CODE_ENTRY(esp32->dbg_stubs.desc.data_alloc);
	ESP32_DBGSTUBS_UPDATE_CODE_ENTRY(esp32->dbg_stubs.desc.data_free);

	return res;
}

static int esp32_dbgstubs_restore(struct target *target)
{
	struct esp32_common *esp32 = (struct esp32_common *)target->arch_info;

	LOG_INFO("Restore debug stubs @ %x on core%u of target '%s'", esp32->dbg_stubs.base, (uint32_t)esp32->active_cpu, target_type_name(target));
	int res = esp108_apptrace_write_status(esp32->esp32_targets[esp32->active_cpu], esp32->dbg_stubs.base);
	if (res != ERROR_OK) {
		LOG_ERROR("Failed to write trace status (%d)!", res);
		return res;
	}
	return ERROR_OK;
}

static void esp32_algo_args_init(struct esp32_stub *stub, uint32_t stack_addr)
{
	LOG_DEBUG("Check stack addr 0x%x", stack_addr);
	if (stack_addr & 0xFUL) {
		LOG_DEBUG("Adjust stack addr 0x%x", stack_addr);
		stack_addr &= ~0xFUL;
	}
	stack_addr -= 16;
	init_reg_param(&stub->reg_params[0], "a0", 			32, PARAM_OUT); //TODO: move to tramp
	init_reg_param(&stub->reg_params[1], "a1", 			32, PARAM_OUT);
	init_reg_param(&stub->reg_params[2], "a8", 			32, PARAM_OUT);
	init_reg_param(&stub->reg_params[3], "windowbase", 	32, PARAM_OUT); //TODO: move to tramp
	init_reg_param(&stub->reg_params[4], "windowstart", 32, PARAM_OUT); //TODO: move to tramp
	init_reg_param(&stub->reg_params[5], "ps", 			32, PARAM_OUT);
	buf_set_u32(stub->reg_params[0].value, 0, 32, 0); // a0 TODO: move to tramp
	buf_set_u32(stub->reg_params[1].value, 0, 32, stack_addr); // a1
	buf_set_u32(stub->reg_params[2].value, 0, 32, stub->entry); // a8
	buf_set_u32(stub->reg_params[3].value, 0, 32, 0x0); // initial window base TODO: move to tramp
	buf_set_u32(stub->reg_params[4].value, 0, 32, 0x1); // initial window start TODO: move to tramp
	buf_set_u32(stub->reg_params[5].value, 0, 32, 0x60025); // enable WOE, UM and debug interrupts level (6)
}

static int esp32_stub_load(struct target *target, struct esp32_algo_image *algo_image, struct esp32_stub *stub, uint32_t stack_size)
{
	int retval;

	if (algo_image) {
		//TODO: add description of how to build proper ELF image to to be loaded to workspace
		LOG_DEBUG("stub: base 0x%x, start 0x%x, %d sections", algo_image->image.base_address_set ? (unsigned) algo_image->image.base_address : 0, algo_image->image.start_address, algo_image->image.num_sections);
		stub->entry = algo_image->image.start_address;
		for (int i = 0; i < algo_image->image.num_sections; i++) {
			struct imagesection *section = &algo_image->image.sections[i];
			LOG_DEBUG("addr %x, sz %d, flags %x", section->base_address, section->size, section->flags);
			if (section->flags & IMAGE_ELF_PHF_EXEC) {
				if (target_alloc_working_area(target, section->size, &stub->code) != ERROR_OK) {
					LOG_ERROR("no working area available, can't alloc space for stub code!");
					retval = ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
					goto _on_error;
				}
				if (section->base_address == 0) {
					section->base_address = stub->code->address;
				// sanity check, stub is compiled to be run from working area
				} else if (stub->code->address != section->base_address) {
					LOG_ERROR("working area 0x%x and stub code section 0x%x address mismatch!", section->base_address, stub->code->address);
					retval = ERROR_FAIL;
					goto _on_error;
				}
			} else {
				if (target_alloc_alt_working_area(target, section->size + algo_image->bss_size, &stub->data) != ERROR_OK) {
					LOG_ERROR("no working area available, can't alloc space for stub data!");
					retval = ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
					goto _on_error;
				}
				if (section->base_address == 0) {
					section->base_address = stub->data->address;
				// sanity check, stub is compiled to be run from working area
				} else  if (stub->data->address != section->base_address) {
					LOG_ERROR("working area 0x%x and stub data section 0x%x address mismatch!", section->base_address, stub->data->address);
					retval = ERROR_FAIL;
					goto _on_error;
				}
			}
			uint32_t sec_wr = 0;
			uint8_t buf[512];
			while (sec_wr < section->size) {
				uint32_t nb = section->size - sec_wr > sizeof(buf) ? sizeof(buf) : section->size - sec_wr;
				size_t size_read = 0;
				retval = image_read_section(&(algo_image->image), i, sec_wr, nb, buf, &size_read);
				if (retval != ERROR_OK) {
					LOG_ERROR("Failed to read stub section (%d)!", retval);
					goto _on_error;
				}
				retval = target_write_buffer(target, section->base_address + sec_wr, size_read, buf);
				if (retval != ERROR_OK) {
					LOG_ERROR("Failed to write stub section!");
					goto _on_error;
				}
				sec_wr += size_read;
			}
		}
	}
	if (stub->tramp_addr == 0) {
		// alloc trampoline in code working area
		if (target_alloc_working_area(target, sizeof(esp32_stub_tramp), &stub->tramp) != ERROR_OK) {
			LOG_ERROR("no working area available, can't alloc space for stub jumper!");
			return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		}
		stub->tramp_addr = stub->tramp->address;
	}
	if (stub->stack_addr == 0) {
		// alloc stack in data working area
		if (target_alloc_alt_working_area(target, stack_size, &stub->stack) != ERROR_OK) {
			LOG_ERROR("no working area available, can't alloc stub stack!");
			target_free_working_area(target, stub->tramp);
			return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		}
		stub->stack_addr = stub->stack->address + stack_size;
	}
	retval = target_write_buffer(target, stub->tramp_addr, sizeof(esp32_stub_tramp), esp32_stub_tramp);
	if (retval != ERROR_OK) {
		LOG_ERROR("Failed to write stub jumper!");
		goto _on_error;
	}

	return ERROR_OK;

_on_error:
	if (stub->tramp) {
		target_free_working_area(target, stub->tramp);
	}
	if (stub->stack) {
		target_free_alt_working_area(target, stub->stack);
	}
	if (stub->code) {
		target_free_working_area(target, stub->code);
	}
	if (stub->data) {
		target_free_alt_working_area(target, stub->data);
	}
	return retval;
}

static void esp32_stub_cleanup(struct target *target, struct esp32_stub *stub)
{
	destroy_reg_param(&stub->reg_params[5]);
	destroy_reg_param(&stub->reg_params[4]);
	destroy_reg_param(&stub->reg_params[3]);
	destroy_reg_param(&stub->reg_params[2]);
	destroy_reg_param(&stub->reg_params[1]);
	destroy_reg_param(&stub->reg_params[0]);
	if (stub->tramp) {
		target_free_working_area(target, stub->tramp);
	}
	if (stub->stack) {
		target_free_alt_working_area(target, stub->stack);
	}
	if (stub->code) {
		target_free_working_area(target, stub->code);
	}
	if (stub->data) {
		target_free_alt_working_area(target, stub->data);
	}
}

#if ESP32_STUB_STACK_DEBUG
static int esp32_stub_fill_stack(struct target *target, uint32_t stack_addr, uint32_t stack_size, uint32_t sz)
{
	uint8_t buf[256];

	// fill stub stack with canary bytes
	memset(buf, ESP32_STUB_STACK_STAMP, sizeof(buf));
	for (uint32_t i = 0; i < sz;) {
		uint32_t wr_sz = stack_size - i >= sizeof(buf) ? sizeof(buf) : stack_size - i;
		// int retval = target_write_buffer(target, stack_addr + i, wr_sz, buf);
		int retval = target_write_memory(target, stack_addr + i, 1, wr_sz, buf);
		if (retval != ERROR_OK) {
			LOG_ERROR("Failed to init stub stack (%d)!", retval);
			return retval;
		}
		i += wr_sz;
	}
	return ERROR_OK;
}

static int esp32_stub_check_stack(struct target *target, uint32_t stack_addr, uint32_t stack_size, uint32_t sz)
{
	int retval = ERROR_OK;
	uint8_t buf[256];

	// check stub stack for overflow
	for (uint32_t i = 0; i < sz;) {
		uint32_t rd_sz = sz - i >= sizeof(buf) ? sizeof(buf) : sz - i;
		retval = target_read_memory(target, stack_addr + i, 1, rd_sz, buf);
		if (retval != ERROR_OK) {
			LOG_ERROR("Failed to read stub stack (%d)!", retval);
			return retval;
		}
		int checked = 0;
		LOG_DEBUG("STK[%x]: check", stack_addr+i);
		uint32_t j;
		for (j = 0; j < rd_sz; j++) {
			if (buf[j] != ESP32_STUB_STACK_STAMP) {
				if (i+j > 0) {
					LOG_WARNING("Stub stack bytes unused %d / %d", i+j, stack_size);
				} else {
					LOG_ERROR("Stub stack OVF!!!");
					retval = ERROR_FAIL;
				}
				checked = 1;
				break;
			}
		}
		if (checked) {
			break;
		}
		i += rd_sz;
	}
	return retval;
}
#endif

static int esp32_algo_run(struct target *target, struct esp32_algo_image *image,
						struct esp32_algo_run_data *run)
{
	int retval;
	struct duration algo_time;
	void **mem_handles = NULL;
	struct esp32_common *esp32 = (struct esp32_common *)target->arch_info;

	if (duration_start(&algo_time) != 0) {
		LOG_ERROR("Failed to start algo time measurement!");
		return ERROR_FAIL;
	}

	retval = esp32_stub_load(target, image, &run->priv.stub, run->stack_size);
	if (retval != ERROR_OK) {
		LOG_ERROR("Failed to load stub (%d)!", retval);
		return retval;
	}
	if (duration_measure(&algo_time) != 0) {
		LOG_ERROR("Failed to stop algo run measurement!");
		return ERROR_FAIL;
	}
	LOG_DEBUG("Stub loaded in %g ms", duration_elapsed(&algo_time)*1000);

	esp32_algo_args_init(&run->priv.stub, run->priv.stub.stack_addr);
	// allocate memory arguments and fill respective reg params
	if (run->mem_args.count > 0) {
		mem_handles = malloc(sizeof(void *)*run->mem_args.count);
		if (mem_handles == NULL) {
			LOG_ERROR("Failed to alloc target mem handles!");
			retval = ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
			goto _cleanup;
		}
		memset(mem_handles, 0, sizeof(void *)*run->mem_args.count);
		// alloc memory args target buffers
		for (uint32_t i = 0; i < run->mem_args.count; i++) {
			// small hack: if we need to update some reg param this field holds appropriate user argument number,
			// otherwise should hold UINT_MAX
			uint32_t usr_param_num = run->mem_args.params[i].address;
			if (image) {
				struct working_area *area;
				retval = target_alloc_alt_working_area(target, run->mem_args.params[i].size, &area);
				if (retval != ERROR_OK) {
					LOG_ERROR("Failed to alloc target buffer!");
					retval = ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
					goto _cleanup;
				}
				mem_handles[i] = area;
				run->mem_args.params[i].address = area->address;
			} else {
				struct esp32_algo_run_data alloc_run;

				if (esp32->dbg_stubs.entries_count < 1 || esp32->dbg_stubs.desc.data_alloc == 0) {
					LOG_ERROR("No dbg stubs found!");
					retval = ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
					goto _cleanup;
				}
				memset(&alloc_run, 0, sizeof(alloc_run));
				alloc_run.stack_size = ESP32_DBG_STUBS_STACK_MIN_SIZE;
				retval = esp32_run_onboard_func(target, &alloc_run, esp32->dbg_stubs.desc.data_alloc, 1, run->mem_args.params[i].size);
				if (retval != ERROR_OK) {
					LOG_ERROR("Failed to run mem arg alloc onboard algo (%d)!", retval);
					goto _cleanup;
				}
				if (alloc_run.ret_code == 0) {
					LOG_ERROR("Failed to alloc onboard memory (%d)!", retval);
					retval = ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
					goto _cleanup;
				}
				mem_handles[i] = (void *)((long)alloc_run.ret_code);
				run->mem_args.params[i].address = alloc_run.ret_code;
			}
			if (usr_param_num != UINT_MAX) { // if we need update some register param with to mem param addr
				buf_set_u32(run->priv.stub.reg_params[ESP32_STUB_ARGS_FUNC_START+usr_param_num].value, 0, 32, run->mem_args.params[i].address);
			}
		}
	}

	if (run->usr_func_init) {
		retval = run->usr_func_init(target, run, run->usr_func_arg);
		if (retval != ERROR_OK) {
			LOG_ERROR("Failed to prepare algorithm host side args stub (%d)!", retval);
			goto _cleanup;
		}
	}

#if ESP32_STUB_STACK_DEBUG
	LOG_DEBUG("Fill stack 0x%x", run->priv.stub.stack_addr);
	retval = esp32_stub_fill_stack(target, run->priv.stub.stack_addr - run->stack_size, run->stack_size, ESP32_STUB_STACK_DEBUG);
	if (retval != ERROR_OK) {
		LOG_ERROR("Failed to init stub stack (%d)!", retval);
		goto _cleanup;
	}
#endif
	LOG_DEBUG("Algorithm start @ 0x%x, stack %d bytes @ 0x%x ", run->priv.stub.tramp_addr, run->stack_size, run->priv.stub.stack_addr);
	retval = target_start_algorithm(target,
			run->mem_args.count, run->mem_args.params,
			run->priv.stub.reg_params_count, run->priv.stub.reg_params,
			run->priv.stub.tramp_addr, 0,
			&run->priv.stub.ainfo);
	if (retval != ERROR_OK) {
		LOG_ERROR("Failed to start algorithm (%d)!", retval);
		goto _cleanup;
	}

	if (run->usr_func) {
		// give target algorithm stub time to init itself, then user func can communicate to it safely
		alive_sleep(100);
		retval = run->usr_func(target, run->usr_func_arg);
		if (retval != ERROR_OK) {
			LOG_ERROR("Failed to exec algorithm user func (%d)!", retval);
		}
	}

	LOG_DEBUG("Wait algorithm completion");
	retval = target_wait_algorithm(target,
			run->mem_args.count, run->mem_args.params,
			run->priv.stub.reg_params_count, run->priv.stub.reg_params,
			0, run->tmo ? run->tmo : ESP32_ALGORITHM_EXIT_TMO,
			&run->priv.stub.ainfo);
	if (retval != ERROR_OK) {
		LOG_ERROR("Failed to wait algorithm (%d)!", retval);
		// target has been forced to stop in target_wait_algorithm()
	}
#if ESP32_STUB_STACK_DEBUG
	retval = esp32_stub_check_stack(target, run->priv.stub.stack_addr - run->stack_size, run->stack_size, ESP32_STUB_STACK_DEBUG);
	if (retval != ERROR_OK) {
		LOG_ERROR("Failed to check stub stack (%d)!", retval);
	}
#endif
	if (run->usr_func_done) {
		run->usr_func_done(target, run, run->usr_func_arg);
	}
_cleanup:
	// free memory arguments
	if (mem_handles) {
		for (uint32_t i = 0; i < run->mem_args.count; i++) {
			if (mem_handles[i]) {
				if (image) {
					target_free_alt_working_area(target, mem_handles[i]);
				} else {
					struct esp32_algo_run_data free_run;
					memset(&free_run, 0, sizeof(free_run));
					free_run.stack_size = ESP32_DBG_STUBS_STACK_MIN_SIZE;
					retval = esp32_run_onboard_func(target, &free_run, esp32->dbg_stubs.desc.data_free, 1, mem_handles[i]);
					if (retval != ERROR_OK) {
						LOG_ERROR("Failed to run mem arg free onboard algo (%d)!", retval);
					}
				}
			}
		}
		free(mem_handles);
	}
	esp32_stub_cleanup(target, &run->priv.stub);

	return retval;
}

int esp32_run_algorithm_image(struct target *target, struct esp32_algo_run_data *run, struct esp32_algo_image *algo_image)
{
	if (!algo_image->image.start_address_set || algo_image->image.start_address == 0) {
		return ERROR_FAIL;
	}
	// to be allocated automatically in corresponding working areas
	run->priv.stub.stack_addr = 0;
	run->priv.stub.tramp_addr = 0;
	// entry will be set from algo_image->image
	return esp32_algo_run(target, algo_image, run);
}

int esp32_run_algorithm_onboard(struct target *target, struct esp32_algo_run_data *run, uint32_t entry)
{
	int res;
	struct esp32_common *esp32 = (struct esp32_common *)target->arch_info;

	if (sizeof(esp32_stub_tramp) > ESP32_DBG_STUBS_CODE_BUF_SIZE) {
		LOG_ERROR("Stub tramp size %u bytes exceeds target buf size %d bytes!",
				(uint32_t)sizeof(esp32_stub_tramp), ESP32_DBG_STUBS_CODE_BUF_SIZE);
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	if (run->stack_size > ESP32_DBG_STUBS_STACK_MIN_SIZE) {
		if (esp32->dbg_stubs.desc.data_alloc == 0 || esp32->dbg_stubs.desc.data_free == 0) {
			LOG_ERROR("No stubs stack funcs found!");
			return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		}
		// alloc stack
		struct esp32_algo_run_data alloc_run;
		memset(&alloc_run, 0, sizeof(alloc_run));
		alloc_run.stack_size = ESP32_DBG_STUBS_STACK_MIN_SIZE;
		res = esp32_run_onboard_func(target, &alloc_run, esp32->dbg_stubs.desc.data_alloc, 1, run->stack_size);
		if (res != ERROR_OK) {
			LOG_ERROR("Failed to run stack alloc onboard algo (%d)!", res);
			return res;
		}
		LOG_DEBUG("RETCODE: %x!", alloc_run.ret_code);
		if (alloc_run.ret_code == 0) {
			LOG_ERROR("Failed to alloc onboard stack (%d)!", res);
			return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		}
		run->priv.stub.stack_addr = alloc_run.ret_code + run->stack_size;
	} else {
		run->priv.stub.stack_addr = esp32->dbg_stubs.desc.min_stack_addr + run->stack_size;
	}
	run->priv.stub.tramp_addr = esp32->dbg_stubs.desc.tramp_addr;
	run->priv.stub.entry = entry;

	res = esp32_algo_run(target, NULL, run);

	if (run->stack_size > ESP32_DBG_STUBS_STACK_MIN_SIZE) {
		// free stack
		struct esp32_algo_run_data free_run;
		memset(&free_run, 0, sizeof(free_run));
		free_run.stack_size = ESP32_DBG_STUBS_STACK_MIN_SIZE;
		res = esp32_run_onboard_func(target, &free_run, esp32->dbg_stubs.desc.data_free, 1, run->priv.stub.stack_addr - run->stack_size);
		if (res != ERROR_OK) {
			LOG_ERROR("Failed to run stack free onboard algo (%d)!", res);
			return res;
		}
	}
	return res;
}

static int esp32_run_do(struct target *target, struct esp32_algo_run_data *run, void *algo_arg, uint32_t num_args, va_list ap)
{
	char *arg_regs[] = {"a3", "a4", "a5", "a6"};

	init_reg_param(&run->priv.stub.reg_params[ESP32_STUB_ARGS_FUNC_START+0], "a2", 32, PARAM_IN_OUT);
	if (num_args > 0) {
		uint32_t arg = va_arg(ap, uint32_t);
		buf_set_u32(run->priv.stub.reg_params[ESP32_STUB_ARGS_FUNC_START+0].value, 0, 32, arg);
		LOG_DEBUG("Set arg[0] = %d", arg);
	} else {
		buf_set_u32(run->priv.stub.reg_params[ESP32_STUB_ARGS_FUNC_START+0].value, 0, 32, 0);
	}
	for (uint32_t i = 1; i < num_args; i++) {
		uint32_t arg = va_arg(ap, uint32_t);
		init_reg_param(&run->priv.stub.reg_params[ESP32_STUB_ARGS_FUNC_START+i], arg_regs[i-1], 32, PARAM_OUT);
		buf_set_u32(run->priv.stub.reg_params[ESP32_STUB_ARGS_FUNC_START+i].value, 0, 32, arg);
		LOG_DEBUG("Set arg[%d] = %d", i, arg);
	}

	run->priv.stub.ainfo.core_mode = XT_MODE_ANY;
	run->priv.stub.reg_params_count = ESP32_STUB_ARGS_FUNC_START + num_args;

	int retval = run->algo_func(target, run, algo_arg);
	if (retval != ERROR_OK) {
		LOG_ERROR("Algorithm run failed (%d)!", retval);
	} else {
		run->ret_code = buf_get_u32(run->priv.stub.reg_params[ESP32_STUB_ARGS_FUNC_START+0].value, 0, 32);
		LOG_DEBUG("Got algorithm RC %x", run->ret_code);
	}

	for (uint32_t i = 0; i < num_args; i++) {
		destroy_reg_param(&run->priv.stub.reg_params[ESP32_STUB_ARGS_FUNC_START+i]);
	}

	return  retval;
}

int esp32_run_func_image(struct target *target, struct esp32_algo_run_data *run, struct esp32_algo_image *image, uint32_t num_args, ...)
{
	va_list ap;

	va_start(ap, num_args);
	run->algo_func = (esp32_algo_func_t)esp32_run_algorithm_image;
	int retval = esp32_run_do(target, run, image, num_args, ap);
	va_end(ap);

	return retval;
}

int esp32_run_onboard_func(struct target *target, struct esp32_algo_run_data *run, uint32_t func_addr, uint32_t num_args, ...)
{
	va_list ap;

	va_start(ap, num_args);
	run->algo_func = (esp32_algo_func_t)esp32_run_algorithm_onboard;
	int retval = esp32_run_do(target, run, (void *)(unsigned long)func_addr, num_args, ap);
	va_end(ap);

	return retval;
}

COMMAND_HANDLER(esp32_cmd_smpbreak)
{
	struct target *target = get_current_target(CMD_CTX);
	struct esp32_common *esp32 = (struct esp32_common*)target->arch_info;
	unsigned int i;
	int res;
	uint32_t set=0, clear=0;
	uint8_t dsr[4];

	if (CMD_ARGC >= 1) {
		for (i=0; i<CMD_ARGC; i++) {
			if (!strcasecmp(CMD_ARGV[0], "none")) {
				set=0;
			} else if (!strcasecmp(CMD_ARGV[i], "BreakIn")) {
				set|=OCDDCR_BREAKINEN;
			} else if (!strcasecmp(CMD_ARGV[i], "BreakOut")) {
				set|=OCDDCR_BREAKOUTEN;
			} else if (!strcasecmp(CMD_ARGV[i], "RunStallIn")) {
				set|=OCDDCR_RUNSTALLINEN;
			} else if (!strcasecmp(CMD_ARGV[i], "DebugModeOut")) {
				set|=OCDDCR_DEBUGMODEOUTEN;
			} else if (!strcasecmp(CMD_ARGV[i], "BreakInOut")) {
				set|=OCDDCR_BREAKINEN|OCDDCR_BREAKOUTEN;
			} else if (!strcasecmp(CMD_ARGV[i], "RunStall")) {
				set|=OCDDCR_RUNSTALLINEN|OCDDCR_DEBUGMODEOUTEN;
			} else {
				command_print(CMD_CTX, "Unknown arg %s", CMD_ARGV[i]);
				command_print(CMD_CTX, "use either BreakInOut, None or RunStall as arguments, or any combination of BreakIn, BreakOut, RunStallIn and DebugModeOut.");
				return ERROR_OK;
			}
		}
		clear=set^(OCDDCR_BREAKINEN|OCDDCR_BREAKOUTEN|OCDDCR_RUNSTALLINEN|OCDDCR_DEBUGMODEOUTEN);
		for (int core = 0; core < ESP32_CPU_COUNT; core++)
		{
			esp108_queue_nexus_reg_write(esp32->esp32_targets[core], NARADR_DCRSET, set);
			esp108_queue_nexus_reg_write(esp32->esp32_targets[core], NARADR_DCRCLR, clear);
		}
	}
	for (int core = 0; core < ESP32_CPU_COUNT; core++)
	{
		esp108_queue_nexus_reg_read(esp32->esp32_targets[core], NARADR_DCRSET, dsr);
		esp108_queue_tdi_idle(esp32->esp32_targets[core]);
		res = jtag_execute_queue();
		if (res==ERROR_OK) {
			i=intfromchars(dsr)&(OCDDCR_BREAKINEN|OCDDCR_BREAKOUTEN|OCDDCR_RUNSTALLINEN|OCDDCR_DEBUGMODEOUTEN);
			command_print(CMD_CTX, "%s: Current bits set:%s%s%s%s%s",
				esp32->esp32_targets[core]->cmd_name,
						(i==0)?" none":"",
						(i&OCDDCR_BREAKINEN)?" BreakIn":"",
						(i&OCDDCR_BREAKOUTEN)?" BreakOut":"",
						(i&OCDDCR_RUNSTALLINEN)?" RunStallIn":"",
						(i&OCDDCR_DEBUGMODEOUTEN)?" DebugModeOut":""
					);
		}
		else
		{
			return res;
		}
	}
	return res;
}

COMMAND_HANDLER(esp32_cmd_tracestart)
{
	struct target *target = get_current_target(CMD_CTX);
	struct esp32_common *esp32=(struct esp32_common*)target->arch_info;
	uint32_t stoppc=0;
	int stopmask=-1;
	int after=0;
	int afterIsWords=0;
	int res;
	unsigned int i;
	uint8_t traxstat[8], traxctl[4];

	for (size_t core = 0; core < ESP32_CPU_COUNT; core++)
	{
		esp108_queue_nexus_reg_read(esp32->esp32_targets[core], NARADR_TRAXSTAT, traxstat);
		esp108_queue_nexus_reg_read(esp32->esp32_targets[core], NARADR_TRAXCTRL, traxctl);
		res = jtag_execute_queue();
		if (res != ERROR_OK) return res;

		esp108_queue_nexus_reg_write(esp32->esp32_targets[core], NARADR_TRAXCTRL, intfromchars(traxctl) | TRAXCTRL_TRSTP);
		esp108_queue_tdi_idle(esp32->esp32_targets[core]);
		res = jtag_execute_queue();
		if (res != ERROR_OK) return res;

		//Check current status of trace hardware
		esp108_queue_nexus_reg_read(esp32->esp32_targets[core], NARADR_TRAXSTAT, traxstat);
		esp108_queue_tdi_idle(esp32->esp32_targets[core]);
		res = jtag_execute_queue();
		if (res != ERROR_OK) return res;
		if (intfromchars(traxstat)&TRAXSTAT_TRACT) {
			command_print(CMD_CTX, "Tracing is already active. Please stop it first.");
			return ERROR_FAIL;
		}
	}

	//Parse arguments
	for (i=0; i<CMD_ARGC; i++) {
		if ((!strcasecmp(CMD_ARGV[i], "pc")) && CMD_ARGC>i) {
			char *e;
			i++;
			stoppc=strtol(CMD_ARGV[i], &e, 0);
			stopmask=0;
			if (*e=='/') stopmask=strtol(e, NULL, 0);
		} else if ((!strcasecmp(CMD_ARGV[i], "after")) && CMD_ARGC>i) {
			i++;
			after=strtol(CMD_ARGV[i], NULL, 0);
		} else if (!strcasecmp(CMD_ARGV[i], "ins")) {
			afterIsWords=0;
		} else if (!strcasecmp(CMD_ARGV[i], "words")) {
			afterIsWords=1;
		} else {
			command_print(CMD_CTX, "Did not understand %s", CMD_ARGV[i]);
			return ERROR_FAIL;
		}
	}
	stopmask = 1;
	for (size_t core = 0; core < ESP32_CPU_COUNT; core++)
	{
		//Turn off trace unit so we can start a new trace.
		esp108_queue_nexus_reg_write(esp32->esp32_targets[core], NARADR_TRAXCTRL, 0);
		esp108_queue_tdi_idle(esp32->esp32_targets[core]);
		res = jtag_execute_queue();
		if (res != ERROR_OK) return res;

		//Set up parameters
		esp108_queue_nexus_reg_write(esp32->esp32_targets[core], NARADR_TRAXADDR, 0);
		if (stopmask != -1) {
			esp108_queue_nexus_reg_write(esp32->esp32_targets[core], NARADR_PCMATCHCTRL, (stopmask << PCMATCHCTRL_PCML_SHIFT));
			esp108_queue_nexus_reg_write(esp32->esp32_targets[core], NARADR_TRIGGERPC, stoppc);
		}
		esp108_queue_nexus_reg_write(esp32->esp32_targets[core], NARADR_DELAYCNT, after);
		//stopmask = TRAXCTRL_PCMEN;
		//Options are mostly hardcoded for now. ToDo: make this more configurable.
		esp108_queue_nexus_reg_write(esp32->esp32_targets[core], NARADR_TRAXCTRL,
			TRAXCTRL_TREN | ((stopmask != -1) ? TRAXCTRL_PCMEN : 0) | TRAXCTRL_TMEN |
			(afterIsWords ? 0 : TRAXCTRL_CNTU) | (0 << TRAXCTRL_SMPER_SHIFT) | TRAXCTRL_PTOWS);
		esp108_queue_tdi_idle(esp32->esp32_targets[core]);
		res = jtag_execute_queue();
		if (res != ERROR_OK) return res;
	}

	LOG_INFO("Trace started.\n");
	esp32->traceActive=1;
	return ERROR_OK;
}

COMMAND_HANDLER(esp32_cmd_tracestop)
{
	struct target *target = get_current_target(CMD_CTX);
	uint8_t traxctl[4], traxstat[4];
	int res;

	esp108_queue_nexus_reg_read(target, NARADR_TRAXSTAT, traxstat);
	esp108_queue_nexus_reg_read(target, NARADR_TRAXCTRL, traxctl);
	res=jtag_execute_queue();
	if (res!=ERROR_OK) return res;
	if (!(intfromchars(traxstat)&TRAXSTAT_TRACT)) {
		command_print(CMD_CTX, "No trace is currently active.");
		return ERROR_FAIL;
	}

	esp108_queue_nexus_reg_write(target, NARADR_TRAXCTRL, intfromchars(traxctl)|TRAXCTRL_TRSTP);
	esp108_queue_tdi_idle(target);
	res=jtag_execute_queue();
	if (res!=ERROR_OK) return res;

	command_print(CMD_CTX, "Trace stop triggered.");
	return ERROR_OK;
}

COMMAND_HANDLER(esp32_cmd_tracedump)
{
	struct target *target = get_current_target(CMD_CTX);
	uint8_t traxstat[4], memadrstart[4], memadrend[4], adr[4], traxctl[4];
	int memsz, wmem;
	uint8_t *tracemem;
	int i, f, res;
	int isAllZeroes;

	if (CMD_ARGC != 1) {
		command_print(CMD_CTX, "Need filename to dump to as output!");
		return ERROR_FAIL;
	}

	esp108_queue_nexus_reg_read(target, NARADR_TRAXSTAT, traxstat);
	esp108_queue_nexus_reg_read(target, NARADR_TRAXCTRL, traxctl);
	esp108_queue_nexus_reg_read(target, NARADR_MEMADDRSTART, memadrstart);
	esp108_queue_nexus_reg_read(target, NARADR_MEMADDREND, memadrend);
	esp108_queue_nexus_reg_read(target, NARADR_TRAXADDR, adr);
	esp108_queue_tdi_idle(target);
	res=jtag_execute_queue();
	if (res!=ERROR_OK) return res;
	if (intfromchars(traxstat)&TRAXSTAT_TRACT) {
		command_print(CMD_CTX, "Tracing is still active. Please stop it first.");
		return ERROR_FAIL;
	}

	if (!(intfromchars(traxctl)&TRAXCTRL_TREN)) {
		command_print(CMD_CTX, "No active trace found; nothing to dump.");
		return ERROR_FAIL;
	}

	f=open(CMD_ARGV[0], O_WRONLY|O_CREAT|O_TRUNC, 0666);
	if (f<=0) {
		command_print(CMD_CTX, "Unable to open file %s", CMD_ARGV[0]);
		return ERROR_FAIL;
	}

	memsz=(intfromchars(memadrend)-intfromchars(memadrstart)+1);
	LOG_INFO("Total trace memory: %d words", memsz);
	if ((intfromchars(adr)&((TRAXADDR_TWRAP_MASK<<TRAXADDR_TWRAP_SHIFT)|TRAXADDR_TWSAT))==0) {
		//Memory hasn't overwritten itself yet.
		wmem=intfromchars(adr)&TRAXADDR_TADDR_MASK;
		LOG_INFO("...but trace is only %d words", wmem);
		if (wmem<memsz) memsz=wmem;
	} else {
		if (intfromchars(adr)&TRAXADDR_TWSAT) {
			LOG_INFO("Real trace is many times longer than that (overflow)");
		} else {
			i=(intfromchars(adr)>>TRAXADDR_TWRAP_SHIFT)&TRAXADDR_TWRAP_MASK;
			i=(i*memsz)+(intfromchars(adr)&TRAXADDR_TADDR_MASK);
			LOG_INFO("Real trace is %d words, but the start has been truncated.", i);
		}
	}
	tracemem=malloc(memsz*4);
	for (i=0; i<memsz; i++) {
		esp108_queue_nexus_reg_read(target, NARADR_TRAXDATA, &tracemem[i*4]);
	}
	esp108_queue_tdi_idle(target);
	res=jtag_execute_queue();
	if (res!=ERROR_OK) return res;

	if (write(f, tracemem, memsz*4)!=memsz*4) {
		command_print(CMD_CTX, "Unable to write to file %s", CMD_ARGV[0]);
	}
	close(f);
	command_print(CMD_CTX, "Written %d bytes of trace data to %s", memsz*4, CMD_ARGV[0]);

	isAllZeroes=1;
	for (i=0; i<memsz*4; i++) {
		if (tracemem[i]!=0) isAllZeroes=0;
	}
	if (isAllZeroes) {
		command_print(CMD_CTX, "WARNING: File written is all zeroes. Are you sure you enabled trace memory?");
	}

	return ERROR_OK;
}

COMMAND_HANDLER(esp32_cmd_flashbootstrap)
{
	struct target *target = get_current_target(CMD_CTX);
	struct esp32_common *esp32=(struct esp32_common*)target->arch_info;
	int state=-1;
	if (CMD_ARGC != 1) {
		const char *st;
		if (esp32->flashBootstrap==FBS_DONTCARE) st="Don't care";
		if (esp32->flashBootstrap==FBS_TMSLOW) st="Low (3.3V)";
		if (esp32->flashBootstrap==FBS_TMSHIGH) st="High (1.8V)";
		command_print(CMD_CTX, "Current idle tms state: %s", st);
		return ERROR_OK;
	}

	if (!strcasecmp(CMD_ARGV[0], "none")) state=FBS_DONTCARE;
	if (!strcasecmp(CMD_ARGV[0], "1.8")) state=FBS_TMSHIGH;
	if (!strcasecmp(CMD_ARGV[0], "3.3")) state=FBS_TMSLOW;
	if (!strcasecmp(CMD_ARGV[0], "high")) state=FBS_TMSHIGH;
	if (!strcasecmp(CMD_ARGV[0], "low")) state=FBS_TMSLOW;

	if (state==-1) {
		command_print(CMD_CTX, "Argument unknown. Please pick one of none, high, low, 1.8 or 3.3");
		return ERROR_FAIL;
	}

	esp32->flashBootstrap=state;

	return ERROR_OK;
}

COMMAND_HANDLER(esp32_cmd_apptrace)
{
	return esp_cmd_apptrace_generic(get_current_target(CMD_CTX), 0, CMD_ARGV, CMD_ARGC);
}

COMMAND_HANDLER(esp32_cmd_sysview)
{
	return esp_cmd_apptrace_generic(get_current_target(CMD_CTX), 1, CMD_ARGV, CMD_ARGC);
}

COMMAND_HANDLER(esp32_cmd_gcov)
{
	return esp_cmd_gcov(get_current_target(CMD_CTX), CMD_ARGV, CMD_ARGC);
}

/* perfmon_enable <counter_id> <select> [mask] [kernelcnt] [tracelevel] */
COMMAND_HANDLER(handle_perfmon_enable)
{
	struct target* target = get_current_target(CMD_CTX);
	struct esp32_common *esp32=(struct esp32_common*)target->arch_info;
	int res;

	struct esp108_perfmon_config config = {
		.mask = 0xffff,
		.kernelcnt = 0,
		.tracelevel = XT_DEBUGLEVEL
	};

	if (CMD_ARGC < 2) {
		return ERROR_COMMAND_SYNTAX_ERROR;
	}
	unsigned counter_id = strtoul(CMD_ARGV[0], NULL, 0);
	if (counter_id >= ESP108_MAX_PERF_COUNTERS) {
		LOG_ERROR("counter_id should be < %d", ESP108_MAX_PERF_COUNTERS);
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	config.select = strtoul(CMD_ARGV[1], NULL, 0);
	if (config.select > ESP108_MAX_PERF_SELECT) {
		LOG_ERROR("select should be < %d", ESP108_MAX_PERF_SELECT);
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	if (CMD_ARGC >= 3) {
		config.mask = strtoul(CMD_ARGV[2], NULL, 0);
		if (config.mask > ESP108_MAX_PERF_MASK) {
			LOG_ERROR("mask should be < %d", ESP108_MAX_PERF_MASK);
			return ERROR_COMMAND_SYNTAX_ERROR;
		}
	}

	if (CMD_ARGC >= 4) {
		config.kernelcnt = strtoul(CMD_ARGV[3], NULL, 0);
		if (config.kernelcnt > 1) {
			LOG_ERROR("kernelcnt should be 0 or 1");
			return ERROR_COMMAND_SYNTAX_ERROR;
		}
	}

	if (CMD_ARGC >= 5) {
		config.tracelevel = strtoul(CMD_ARGV[4], NULL, 0);
		if (config.tracelevel > 7) {
			LOG_ERROR("tracelevel should be <=7");
			return ERROR_COMMAND_SYNTAX_ERROR;
		}
	}

	int cores_count = esp32_get_cores_count(target);
	for (int i = 0; i < cores_count; ++i) {
		struct target* cur_target = esp32->esp32_targets[i];
		res = esp108_perfmon_enable(cur_target, counter_id, &config);
		if (res != ERROR_OK) {
			return res;
		}
	}

	return ERROR_OK;
}

/* perfmon_dump [counter_id] */
COMMAND_HANDLER(handle_perfmon_dump)
{
	struct target* target = get_current_target(CMD_CTX);
	struct esp32_common *esp32=(struct esp32_common*)target->arch_info;
	int res;

	if (CMD_ARGC > 1) {
		return ERROR_COMMAND_SYNTAX_ERROR;
	}
	int counter_id = -1;
	if (CMD_ARGC == 1) {
		counter_id = strtol(CMD_ARGV[0], NULL, 0);
		if (counter_id > ESP108_MAX_PERF_COUNTERS) {
			LOG_ERROR("counter_id should be < %d", ESP108_MAX_PERF_COUNTERS);
			return ERROR_COMMAND_SYNTAX_ERROR;
		}
	}

	int cores_count = esp32_get_cores_count(target);
	int counter_start = (counter_id < 0) ? 0 : counter_id;
	int counter_end = (counter_id < 0) ? ESP108_MAX_PERF_COUNTERS : counter_id + 1;
	for (int counter = counter_start; counter < counter_end; ++counter) {
		char result_buf[128] = { 0 };
		size_t result_pos = 0;
		result_pos += snprintf(result_buf, sizeof(result_buf), "Counter %d: ", counter);
		uint64_t total_count = 0;
		bool total_overflow = false;
		for (int core = 0; core < cores_count; ++core) {
			struct target* cur_target = esp32->esp32_targets[core];
			struct esp108_perfmon_result result;
			res = esp108_perfmon_dump(cur_target, counter, &result);
			if (res != ERROR_OK) {
				return res;
			}
			result_pos += snprintf(result_buf + result_pos, sizeof(result_buf) - result_pos,
				"CPU%d: %-12llu%s%s",
				core, (unsigned long long) result.value,
				(result.overflow) ? " (overflow)" : "",
				(core < cores_count - 1) ? " ":"");
			total_count += result.value;
			total_overflow = total_overflow || result.overflow;
		}
		if (cores_count > 1 && ! total_overflow) {
			snprintf(result_buf + result_pos, sizeof(result_buf) - result_pos, " Total: %-12llu",
				(unsigned long long) total_count);
		}
		LOG_INFO("%s", result_buf);
	}

	return ERROR_OK;
}

COMMAND_HANDLER(handle_esp32_a_mask_interrupts_command)
{
	struct target *target = get_current_target(CMD_CTX);
	struct esp32_common *esp32 = (struct esp32_common*)target->arch_info;

	static const Jim_Nvp nvp_maskisr_modes[] = {
		{ .name = "off", .value = ESP32_ISRMASK_OFF },
		{ .name = "on", .value = ESP32_ISRMASK_ON },
		{ .name = NULL, .value = -1 },
	};
	const Jim_Nvp *n;

	if (CMD_ARGC > 0) {
		n = Jim_Nvp_name2value_simple(nvp_maskisr_modes, CMD_ARGV[0]);
		if (n->name == NULL) {
			LOG_ERROR("Unknown parameter: %s - should be off or on", CMD_ARGV[0]);
			return ERROR_COMMAND_SYNTAX_ERROR;
		}

		esp32->isrmasking_mode = n->value;
	}

	n = Jim_Nvp_value2name_simple(nvp_maskisr_modes, esp32->isrmasking_mode);
	command_print(CMD_CTX, "esp32 interrupt mask %s", n->name);

	return ERROR_OK;
}

static const struct command_registration esp32_any_command_handlers[] = {
	{
		.name = "tracestart",
		.handler = esp32_cmd_tracestart,
		.mode = COMMAND_ANY,
		.help = "Tracing: Set up and start a trace. Optionally set stop trigger address and amount of data captured after.",
		.usage = "[pc <pcval>/[maskbitcount]] [after <n> [ins|words]]",
	},
	{
		.name = "tracestop",
		.handler = esp32_cmd_tracestop,
		.mode = COMMAND_ANY,
		.help = "Tracing: Stop current trace as started by the tracestart command",
		.usage = "",
	},
	{
		.name = "tracedump",
		.handler = esp32_cmd_tracedump,
		.mode = COMMAND_ANY,
		.help = "Tracing: Dump trace memory to a file",
		.usage = "outfile",
	},
	{
		.name = "apptrace",
		.handler = esp32_cmd_apptrace,
		.mode = COMMAND_ANY,
		.help = "App Tracing: application level trace control. Starts, stops or queries tracing process status.",
		.usage = "[start file://<outfile> [poll_period [trace_size [stop_tmo [wait4halt [skip_size]]]]] | [stop] | [status] | [dump file://<outfile>]",
	},
	{
		.name = "sysview",
		.handler = esp32_cmd_sysview,
		.mode = COMMAND_ANY,
		.help = "App Tracing: SEGGER SystemView compatible trace control. Starts, stops or queries tracing process status.",
		.usage = "[start file://<outfile1> [file://<outfile2>] [poll_period [trace_size [stop_tmo [wait4halt [skip_size]]]]] | [stop] | [status]",
	},
	{
		.name = "gcov",
		.handler = esp32_cmd_gcov,
		.mode = COMMAND_ANY,
		.help = "GCOV: Dumps gcov info collected on target.",
		.usage = "",
	},
	{
		.name = "smpbreak",
		.handler = esp32_cmd_smpbreak,
		.mode = COMMAND_ANY,
		.help = "Set the way the CPU chains OCD breaks",
		.usage = "[none|breakinout|runstall] | [BreakIn] [BreakOut] [RunStallIn] [DebugModeOut]",
	},
	{
		.name = "flashbootstrap",
		.handler = esp32_cmd_flashbootstrap,
		.mode = COMMAND_ANY,
		.help = "Set the idle state of the TMS pin, which at reset also is the voltage selector for the flash chip.",
		.usage = "none|1.8|3.3|high|low",
	},
	{
		.name = "maskisr",
		.handler = handle_esp32_a_mask_interrupts_command,
		.mode = COMMAND_ANY,
		.help = "mask esp32 interrupts at step",
		.usage = "['on'|'off']",
	},
	{
		.name = "perfmon_enable",
		.handler = handle_perfmon_enable,
		.mode = COMMAND_ANY,
		.help = "Enable and start performance counter",
		.usage = "<counter_id> <select> [mask] [kernelcnt] [tracelevel]",
	},
	{
		.name = "perfmon_dump",
		.handler = handle_perfmon_dump,
		.mode = COMMAND_ANY,
		.help = "Dump performance counter value. If no argument specified, dumps all counters.",
		.usage = "[counter_id]",
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
		.chain = esp108_common_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

static size_t esp32_get_cores_count(struct target *target)
{
	struct esp32_common *esp32 = (struct esp32_common*)target->arch_info;
	if (esp32 == NULL) {
		return ESP32_CPU_COUNT;
	}
	return esp32->cores_num ? esp32->cores_num : ESP32_CPU_COUNT;
}

static size_t esp32_get_active_core(struct target *target)
{
	struct esp32_common *esp32 = (struct esp32_common*)target->arch_info;
	return esp32->active_cpu;
}

static void esp32_set_active_core(struct target *target, size_t core)
{
	struct esp32_common *esp32 = (struct esp32_common*)target->arch_info;
	esp32->active_cpu = core;
}

/** Holds methods for Xtensa targets. */
struct target_type esp32_target = {
	.name = "esp32",

	.poll = xtensa_poll,
	.arch_state = xtensa_arch_state,

	.halt = xtensa_halt,
	.resume = xtensa_resume,
	.step = xtensa_step,

	.assert_reset = xtensa_assert_reset,
	.deassert_reset = xtensa_deassert_reset,

	.read_memory = xtensa_read_memory,
	.write_memory = xtensa_write_memory,

	.read_buffer = xtensa_read_buffer,
	.write_buffer = xtensa_write_buffer,

	.get_gdb_reg_list = xtensa_get_gdb_reg_list,

	.run_algorithm = xtensa_run_algorithm,
	.start_algorithm = xtensa_start_algorithm,
	.wait_algorithm = xtensa_wait_algorithm,

	.add_breakpoint = xtensa_add_breakpoint,
	.remove_breakpoint = xtensa_remove_breakpoint,

	.add_watchpoint = xtensa_add_watchpoint,
	.remove_watchpoint = xtensa_remove_watchpoint,

	.target_create = xtensa_target_create,
	.init_target = xtensa_init_target,
	.examine = xtensa_esp32_examine,

	.commands = esp32_command_handlers,

	.get_cores_count = esp32_get_cores_count,
	.get_active_core = esp32_get_active_core,
	.set_active_core = esp32_set_active_core,
};
