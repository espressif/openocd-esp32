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

//forward declarations
static int xtensa_step(struct target *target,
	int current,
	uint32_t address,
	int handle_breakpoints);
static int xtensa_poll(struct target *target);
static int xtensa_assert_reset(struct target *target);
static int xtensa_deassert_reset(struct target *target);
//static int xtensa_write_uint32_list(struct target *target, const uint32_t* addr_value_pairs_list, size_t count);
//static int xtensa_write_uint32(struct target *target, uint32_t addr, uint32_t val);
static int xtensa_read_memory(struct target *target,
	uint32_t address,
	uint32_t size,
	uint32_t count,
	uint8_t *buffer);


//Utility function: check DSR for any weirdness and report.
//Also does tms_reset to bootstrap level indicated.
#define esp32_checkdsr(target) esp108_do_checkdsr(target, __FUNCTION__, __LINE__)

static void esp32_mark_register_dirty(struct reg *reg_list, int regidx)
{
	reg_list[regidx].dirty=1;
}

static int esp32_fetch_all_regs(struct target *target)
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


	for (int c = 0; c < ESP32_CPU_COUNT; c++)
	{
		reg_list[c] = esp32->core_caches[c]->reg_list;
	}

	//Assume the CPU has just halted. We now want to fill the register cache with all the
	//register contents GDB needs. For speed, we pipeline all the read operations, execute them
	//in one go, then sort everything out from the regvals variable.

	// Read registers from both cores
	for (int c = 0; c < ESP32_CPU_COUNT; c++)
	{
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
		if (res!=ERROR_OK) return res;

		esp32_checkdsr(esp32->esp32_targets[c]);
		cpenable = intfromchars(regvals[c][XT_REG_IDX_CPENABLE]);


		//We're now free to use any of A0-A15 as scratch registers
		//Grab the SFRs and user registers first. We use A3 as a scratch register.
		for (i = 0; i < XT_NUM_REGS; i++) {
			if (regReadable(esp32_regs[i].flags, cpenable) && (esp32_regs[i].type == XT_REG_SPECIAL || esp32_regs[i].type == XT_REG_USER)) {
				if (esp32_regs[i].type == XT_REG_USER) {
					esp108_queue_exec_ins(target, XT_INS_RUR(esp32_regs[i].reg_num, XT_REG_A3));
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
		if (res!=ERROR_OK) return res;

		esp32_checkdsr(esp32->esp32_targets[c]);
		//DSR checking: follows order in which registers are requested.
		for (i = 0; i < XT_NUM_REGS; i++) {
			if (regReadable(esp32_regs[i].flags, cpenable) && (esp32_regs[i].type == XT_REG_SPECIAL || esp32_regs[i].type == XT_REG_USER)) {
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
	esp32_checkdsr(esp32->esp32_targets[0]);
	return ERROR_OK;
}

static int esp32_write_dirty_registers(struct target *target, struct reg *reg_list)
{
	int i, j;
	int res;
	uint32_t regval, windowbase;

	LOG_DEBUG("%s: %s", target->cmd_name, __FUNCTION__);

	//We need to write the dirty registers in the cache list back to the processor.
	//Start by writing the SFR/user registers.
	for (i=0; i<XT_NUM_REGS; i++) {
		if (reg_list[i].dirty) {
			if (esp32_regs[i].type==XT_REG_SPECIAL || esp32_regs[i].type==XT_REG_USER) {
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
				esp108_queue_exec_ins(target, XT_INS_RSR(XT_SR_DDR, esp32_regs[XT_REG_IDX_AR0+i+j].reg_num));
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

	LOG_DEBUG("%s, target: %s", __func__, target->cmd_name);

	struct esp32_common* esp32 = (struct esp32_common*)target->arch_info;

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
			xtensa_step(target, current, address, handle_breakpoints);
		}
		if (cause&DEBUGCAUSE_BI) {
			//We stopped due to a break instruction. We can't just resume executing the instruction again because
			//that would trigger the breake again. To fix this, we single-step, which ignores break.
			xtensa_step(target, current, address, handle_breakpoints);
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
	for (int i = 0; i < ESP32_CPU_COUNT; i++)
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

	for (int i = 0; i < ESP32_CPU_COUNT; i++)
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


static int xtensa_resume_cpu(struct target *target,
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
			xtensa_step(target, current, address, handle_breakpoints);
		}
	}

	//Write back hw breakpoints. Current FreeRTOS SMP code can set a hw breakpoint on an
	//exception; we need to clear that and return to the breakpoints gdb has set on resume.
	bpena = 0;
	for (slot = 0; slot < esp32->num_brps; slot++) {
		if (esp32->hw_brps[slot] != NULL) {
			/* Write IBREAKA[slot] and set bit #slot in IBREAKENABLE */
			for (size_t cp = 0; cp < ESP32_CPU_COUNT; cp++)
			{
				// We have equival amount of BP for each cpu
				struct reg *cpu_reg_list = esp32->core_caches[cp]->reg_list;
				esp108_reg_set(&cpu_reg_list[XT_REG_IDX_IBREAKA0 + slot], esp32->hw_brps[slot]->address);
			}
			bpena |= (1 << slot);
		}
	}
	for (size_t cp = 0; cp < ESP32_CPU_COUNT; cp++)
	{
		// We have equival amount of BP for each cpu
		struct reg *cpu_reg_list = esp32->core_caches[cp]->reg_list;
		esp108_reg_set(&cpu_reg_list[XT_REG_IDX_IBREAKENABLE], bpena);
	}

	// Here we write all registers to the targets
	for (int i = 0; i < ESP32_CPU_COUNT; i++)
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

	for (int i = 0; i < ESP32_CPU_COUNT; i++)
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

	if (esp108_get_addr_type(address) == INVALID) {
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
	//printf("%s: %s: reading size=%d , count = %d, bytes from addr %08X \n", target->cmd_name, __FUNCTION__, 1, count, address);
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

	if (esp108_get_addr_type(address) != READWRITE) {
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
	LOG_DEBUG("%s, reg_class=%i", __func__, (int)reg_class);

	*reg_list_size = XT_NUM_REGS_G_COMMAND;
	*reg_list = malloc(sizeof(struct reg *) * (*reg_list_size));

	if (!*reg_list) {
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	for (i = 0; i < XT_NUM_REGS_G_COMMAND; i++) {
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
	esp32->resetAsserted=1;
	return res;
}

static int xtensa_smpbreak_set(struct target *target)
{
	struct esp32_common *esp32 = (struct esp32_common*)target->arch_info;
	int res;
	uint32_t dsr_data = 0x00110000;
	uint32_t set = 0, clear = 0;
	set |= OCDDCR_BREAKINEN | OCDDCR_BREAKOUTEN;
	clear = set ^ (OCDDCR_BREAKINEN | OCDDCR_BREAKOUTEN | OCDDCR_RUNSTALLINEN | OCDDCR_DEBUGMODEOUTEN);

	for (int core = 0; core < ESP32_CPU_COUNT; core++)
	{
		esp108_queue_nexus_reg_write(esp32->esp32_targets[core], NARADR_DCRSET, set);
		esp108_queue_nexus_reg_write(esp32->esp32_targets[core], NARADR_DCRCLR, clear);
		esp108_queue_nexus_reg_write(esp32->esp32_targets[core], NARADR_DSR, dsr_data);
		esp108_queue_tdi_idle(esp32->esp32_targets[core]);
	}
	res = jtag_execute_queue();
	LOG_DEBUG("%s[%s] set smpbreak=%i, state=%i", __func__, target->cmd_name, target->reset_halt, target->state);
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


static int xtensa_add_breakpoint(struct target *target, struct breakpoint *breakpoint)
{
	struct esp32_common *esp32=(struct esp32_common*)target->arch_info;
	size_t slot;

	if (breakpoint->type == BKPT_SOFT) {
		LOG_ERROR("%s: sw breakpoint requested, but software breakpoints not enabled", target->cmd_name);
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	for(slot = 0; slot < esp32->num_brps; slot++) {
		if (esp32->hw_brps[slot] == NULL || esp32->hw_brps[slot] == breakpoint) break;
	}
	if (slot == esp32->num_brps)
	{
		LOG_WARNING("%s: max slot reached, slot=%i", __func__, (unsigned int)slot);
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	esp32->hw_brps[slot] = breakpoint;
	//We will actually write the breakpoints when we resume the target.

	LOG_DEBUG("%s: placed hw breakpoint %d at 0x%X, num_brps=%i", target->cmd_name, (int)slot, breakpoint->address, esp32->num_brps);
	return ERROR_OK;
}


static int xtensa_remove_breakpoint(struct target *target, struct breakpoint *breakpoint)
{
	struct esp32_common *esp32=(struct esp32_common*)target->arch_info;
	size_t slot;

	if (target->state != TARGET_HALTED) {
		LOG_WARNING("%s: %s: target not halted", __func__, target->cmd_name);
		return ERROR_TARGET_NOT_HALTED;
	}

	for(slot = 0; slot < esp32->num_brps; slot++) {
		if(esp32->hw_brps[slot] == breakpoint)
			break;
	}
	if (slot==esp32->num_brps) return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	esp32->hw_brps[slot] = NULL;
	LOG_DEBUG("%s: cleared hw breakpoint %d at 0x%X", target->cmd_name, (int)slot, breakpoint->address);
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
	for (int core = 0; core < ESP32_CPU_COUNT; core++)
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

	if (target->state != TARGET_HALTED) {
		LOG_WARNING("%s: %s: target not halted", __func__, target->cmd_name);
		return ERROR_TARGET_NOT_HALTED;
	}

	for(slot = 0; slot < esp32->num_wps; slot++) {
		if(esp32->hw_wps[slot] == watchpoint)
			break;
	}
	if (slot==esp32->num_wps) return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;

	/* Clear DBREAKC[slot] to disable watchpoint */
	for (int core = 0; core < ESP32_CPU_COUNT; core++)
	{
		struct reg *cpu_reg_list = esp32->core_caches[core]->reg_list;
		esp108_reg_set(&cpu_reg_list[XT_REG_IDX_DBREAKC0 + slot], 0);
	}
	esp32->hw_wps[slot] = NULL;

	return ERROR_OK;

}
#define XCHAL_EXCM_LEVEL		3	/* level masked by PS.EXCM */

static int xtensa_step(struct target *target,
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
	uint32_t oldps, newps, oldpc;
	int tries=10;

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
	if (cause&DEBUGCAUSE_BI) {
		LOG_DEBUG("%s: Increment PC to pass break instruction...", esp32->esp32_targets[esp32->active_cpu]->cmd_name);
		address = oldpc + 3; // PC = PC+3
		current = 0;		 // The PC was modified.
	}



	do {
		// We have equival amount of BP for each cpu
		{
			struct reg *cpu_reg_list = esp32->core_caches[esp32->active_cpu]->reg_list;
			esp108_reg_set(&cpu_reg_list[XT_REG_IDX_ICOUNTLEVEL], icountlvl);
			esp108_reg_set(&cpu_reg_list[XT_REG_IDX_ICOUNT], icount_val);
		}

		/* Now ICOUNT is set, we can resume as if we were going to run */
		int cause_local = esp108_reg_get(&reg_list[XT_REG_IDX_DEBUGCAUSE]);
		if ((cause_local&(DEBUGCAUSE_BI | DEBUGCAUSE_DB)) == 0)
		{
			res = xtensa_resume_cpu(target, current, address, 0, 0);
			if (res != ERROR_OK) {
				LOG_ERROR("%s: %s: Failed to resume after setting up single step", target->cmd_name, __func__);
				return res;
			}
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
			LOG_ERROR("%s: %s: Timed out waiting for target to finish stepping. dsr=0x%08x", target->cmd_name, __func__, intfromchars(dsr));
			return ERROR_TARGET_TIMEOUT;
		} else
		{
			target->state = TARGET_HALTED;
			esp32_fetch_all_regs(target);
		}
	} while (esp108_reg_get(&reg_list[XT_REG_IDX_PC])==oldpc && --tries);
	LOG_DEBUG("Stepped from %X to %X", oldpc, esp108_reg_get(&reg_list[XT_REG_IDX_PC]));

	if (!tries) {
		LOG_WARNING("%s: %s: Stepping doesn't seem to change PC! dsr=0x%08x", target->cmd_name, __func__, intfromchars(dsr));
	}

	// This operation required to clear state
	for (int cp = 0; cp < ESP32_CPU_COUNT; cp++)
	{
		if (cp != esp32->active_cpu) xtensa_read_dsr(esp32->esp32_targets[cp]);
	}

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

	//Make sure the poll routine will pick up that something has changed by artificially
	//triggering a running->halted state change
	target->state = TARGET_RUNNING;
	return res;
}

static int xtensa_start_algorithm(struct target *target,
	int num_mem_params, struct mem_param *mem_params,
	int num_reg_params, struct reg_param *reg_params,
	uint32_t entry_point, uint32_t exit_point,
	void *arch_info)
{
	struct esp32_common *esp32 = (struct esp32_common*)target->arch_info;
	struct reg_cache *core_cache = esp32->core_caches[esp32->active_cpu];

	return xtensa_start_algorithm_generic(target, num_mem_params, mem_params, num_reg_params,
		reg_params, entry_point, exit_point, arch_info, core_cache);
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

	retval = esp32_write_dirty_registers(target, core_cache->reg_list);
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

	target->arch_info = esp32;
	esp32->target=target;

	esp32->num_brps = XT_NUM_BREAKPOINTS;
	esp32->hw_brps = calloc(XT_NUM_BREAKPOINTS, sizeof(struct breakpoint *));
	esp32->num_wps = XT_NUM_WATCHPOINTS;
	esp32->hw_wps = calloc(XT_NUM_WATCHPOINTS, sizeof(struct watchpoint *));

	//Create the register cache
	cache->name = "Xtensa registers";
	cache->next = NULL;
	cache->reg_list = reg_list;
	cache->num_regs = XT_NUM_REGS;
	*cache_p = cache;
	esp32->core_cache = cache;

	for (int i = 0; i < ESP32_CPU_COUNT; i++) {
		esp32->esp32_targets[i] = malloc(sizeof(struct target));
		if (esp32->esp32_targets[i] == NULL) return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;

		memcpy(esp32->esp32_targets[i], target, sizeof(struct target));
		if (i == ESP32_PRO_CPU_ID) {
			esp32->esp32_targets[i]->tap = target->tap;
		} else {
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
	esp32->flashBootstrap=FBS_DONTCARE;
	esp32->isrmasking_mode = ESP32_ISRMASK_ON;

	for(int i = 0; i < XT_NUM_REGS; i++) {
		reg_list[i].name = esp32_regs[i].name;
		reg_list[i].size = 32;
		reg_list[i].value = calloc(1,4);
		reg_list[i].dirty = 0;
		reg_list[i].valid = 0;
		reg_list[i].type = &esp32_reg_type;
		reg_list[i].arch_info=esp32;
	}

	//Assume running target. If different, the first poll will fix this.
	target->state=TARGET_RUNNING;
	target->debug_reason = DBG_REASON_NOTHALTED;
	esp32->core_cache = esp32->core_caches[0];
	esp32->active_cpu = 0;
	return ERROR_OK;
}

static int xtensa_init_target(struct command_context *cmd_ctx, struct target *target)
{	LOG_DEBUG("%s", __func__);
	struct esp32_common *esp32=(struct esp32_common*)target->arch_info;


	esp32->state = XT_NORMAL; // Assume normal state until we examine

	return ERROR_OK;
}


static int xtensa_poll(struct target *target)
{
	struct esp32_common *esp32=(struct esp32_common*)target->arch_info;
	uint8_t pwrstat[ESP32_CPU_COUNT], pwrstath[ESP32_CPU_COUNT];
	int res;
	int cmd;
	uint8_t dsr[ESP32_CPU_COUNT][4], ocdid[ESP32_CPU_COUNT][4], traxstat[ESP32_CPU_COUNT][4], traxctl[ESP32_CPU_COUNT][4];

	//Read reset state
	for (size_t i = 0; i < ESP32_CPU_COUNT; i++)
	{
		esp108_queue_pwrstat_readclear(esp32->esp32_targets[i], &pwrstat[i]);
		//Read again, to see if the state holds...
		esp108_queue_pwrstat_readclear(esp32->esp32_targets[i], &pwrstath[i]);
		esp108_queue_tdi_idle(esp32->esp32_targets[i]);
		res = jtag_execute_queue();
		if (res != ERROR_OK) return res;
	}
	if (!(esp32->prevpwrstat&PWRSTAT_DEBUGWASRESET) && pwrstat[ESP32_PRO_CPU_ID] & PWRSTAT_DEBUGWASRESET) {
		LOG_INFO("%s: Debug controller was reset (pwrstat=0x%02X, after clear 0x%02X).", target->cmd_name, pwrstat[ESP32_PRO_CPU_ID], pwrstath[ESP32_PRO_CPU_ID]);
	}
	if (!(esp32->prevpwrstat&PWRSTAT_COREWASRESET) && pwrstat[ESP32_PRO_CPU_ID] & PWRSTAT_COREWASRESET) {
		LOG_INFO("%s: Core was reset (pwrstat=0x%02X, after clear 0x%02X).", target->cmd_name, pwrstat[ESP32_PRO_CPU_ID], pwrstath[ESP32_PRO_CPU_ID]);
	}
	esp32->prevpwrstat = pwrstath[ESP32_PRO_CPU_ID];

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
		esp108_queue_nexus_reg_write(esp32->esp32_targets[i], NARADR_DCRSET, OCDDCR_ENABLEOCD);
		esp108_queue_nexus_reg_read(esp32->esp32_targets[i], NARADR_OCDID, ocdid[i]);
		esp108_queue_nexus_reg_read(esp32->esp32_targets[i], NARADR_DSR, dsr[i]);
		esp108_queue_nexus_reg_read(esp32->esp32_targets[i], NARADR_TRAXSTAT, traxstat[i]);
		esp108_queue_nexus_reg_read(esp32->esp32_targets[i], NARADR_TRAXCTRL, traxctl[i]);
		esp108_queue_tdi_idle(esp32->esp32_targets[i]);
		res = jtag_execute_queue();
		if (res != ERROR_OK) return res;
	}

	unsigned int dsr0 = intfromchars(dsr[0]);
	unsigned int dsr1 = intfromchars(dsr[1]);
	unsigned int common_reason = dsr0 | dsr1; // We should know if even one of CPU was stopped

	unsigned int common_pwrstath = pwrstath[0] | pwrstath[1];

	if ((dsr0 & OCDDSR_STOPPED) != (dsr1 & OCDDSR_STOPPED))
	{
		LOG_DEBUG("%s: dsr0=0x%08x, dsr1=0x%08x", __func__, dsr0, dsr1);
		res = xtensa_smpbreak_set(target);
		if (res != ERROR_OK) return res;
	}

	if (common_reason & OCDDSR_STOPPED) {
		if(target->state != TARGET_HALTED) {
			LOG_DEBUG("Stopped: CPU0: %d CPU1: %d", (dsr0 & OCDDSR_STOPPED) ? 1 : 0, (dsr1 & OCDDSR_STOPPED) ? 1 : 0);
			int oldstate=target->state;
			xtensa_halt(target);
			target->state = TARGET_HALTED;
			esp32_fetch_all_regs(target);
			//Examine why the target was halted
			target->debug_reason = DBG_REASON_DBGRQ;
			for (int i = 0; i < ESP32_CPU_COUNT; i++)
			{
				struct reg *cpu_reg_list = esp32->core_caches[i]->reg_list;
				volatile int temp_cause = xtensa_read_reg_direct(esp32->esp32_targets[i], XT_REG_IDX_DEBUGCAUSE);
				int cause = esp108_reg_get(&cpu_reg_list[XT_REG_IDX_DEBUGCAUSE]);

				volatile unsigned int dsr_core = xtensa_read_dsr(esp32->esp32_targets[i]);
				if ((dsr_core&OCDDSR_DEBUGPENDBREAK) != 0)
				{
					if (esp32->active_cpu != i)
					{
						LOG_INFO("active_cpu: %i, changed to %i, reson = 0x%08x", esp32->active_cpu, i, dsr_core);
					}
					esp32->active_cpu = i;
				}

				int dcrset = read_reg_direct(esp32->esp32_targets[i], NARADR_DCRSET);

				LOG_DEBUG("%s: Halt reason =0x%08X, temp_cause =%08x, dsr=0x%08x, dcrset=0x%08x", esp32->esp32_targets[i]->cmd_name, cause, temp_cause, dsr_core, dcrset);
				if (cause&DEBUGCAUSE_IC)
				{
					target->debug_reason = DBG_REASON_SINGLESTEP;
				}
				if (cause&(DEBUGCAUSE_IB | DEBUGCAUSE_BN | DEBUGCAUSE_BI))
				{
					target->debug_reason = DBG_REASON_BREAKPOINT;
				}
				if (cause&DEBUGCAUSE_DB)
				{
					target->debug_reason = DBG_REASON_WATCHPOINT;
				}
			}
			for (int i = 0; i < ESP32_CPU_COUNT; i++)
			{
				struct reg *cpu_reg_list = esp32->core_caches[i]->reg_list;
				LOG_DEBUG("%s: Target halted, pc=0x%08X, debug_reason=%08x, oldstate=%08x, active=%s", esp32->esp32_targets[i]->cmd_name, esp108_reg_get(&cpu_reg_list[XT_REG_IDX_PC]), target->debug_reason, oldstate, (i == esp32->active_cpu) ? "true" : "false");
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
			//LOG_INFO("%s: Core running again.", target->cmd_name);
			target->state = TARGET_RUNNING;
			target->debug_reason = DBG_REASON_NOTHALTED;
		}
	}


	if (esp32->traceActive) {
		//Detect if tracing was active but has stopped.
		for (int core = 0; core < ESP32_CPU_COUNT; core++)
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

	return ERROR_OK;
}

static int xtensa_arch_state(struct target *target)
{
	LOG_DEBUG("%s", __func__);
	return ERROR_OK;
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

	for (int core = 0; core < ESP32_CPU_COUNT; core++)
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
	for (int core = 0; core < ESP32_CPU_COUNT; core++)
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
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration esp32_command_handlers[] = {
	{
		.name = "esp32",
		.mode = COMMAND_ANY,
		.help = "ESP32 command group",
		.usage = "",
		.chain = esp32_any_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

static int esp32_get_cores_count(struct target *target);
static int esp32_get_active_core(struct target *target);
static void esp32_set_active_core(struct target *target, int core);

static int esp32_get_cores_count(struct target *target)
{
	return ESP32_CPU_COUNT;
}

static int esp32_get_active_core(struct target *target)
{
	struct esp32_common *esp32 = (struct esp32_common*)target->arch_info;
	return esp32->active_cpu;
}

static void esp32_set_active_core(struct target *target, int core)
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
	.examine = xtensa_examine,

	.commands = esp32_command_handlers,

	.get_cores_count = esp32_get_cores_count,
	.get_active_core = esp32_get_active_core,
	.set_active_core = esp32_set_active_core,
};

