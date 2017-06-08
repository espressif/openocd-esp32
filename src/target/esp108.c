/***************************************************************************
 *   ESP108 target for OpenOCD                                             *
 *   Copyright (C) 2016 Espressif Systems Ltd.                             *
 *   <jeroen@espressif.com>                                                *
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

#include "esp108.h"
#include "esp108_dbg_regs.h"

/*
This is a JTAG driver for the ESP108, the Tensilica core inside the ESP32
chips. The ESP108 actually is specific configuration of the configurable
Tensilica Diamons 108Mini Xtensa core. Although this driver could also be
used to control other Diamond 108Mini implementations, we have none to
test this code on, so for now, this code is ESP108 specific.

The code is fairly different from the LX106 JTAG code as written by
projectgus etc for the ESP8266, because the debug controller in the LX106
is different from that in the 108Mini.

Quick reminder how everything works:
The JTAG-pins communicate with a TAP. Using serial shifting, you can set
two registers: the Instruction Register (IR) and a Data Register (DR) for
every instruction. The idea is that you select the IR first, then clock
data in and out of the DR belonging to that IR. (By the way, setting IR/DR
both sets it to the value you clock in, as well as gives you the value it
used to contain. You essentially read and write it at the same time.)

The ESP108 has a 5-bit IR, with (for debug) one important instruction:
11100/0x1C aka NARSEL. Selecting this instruction alternatingly presents
the NAR and NDR (Nexus Address/Data Register) as the DR.

The 8-bit NAR that's written to the chip should contains an address in bit
7-1 and a read/write bit as bit 0 that should be one if you want to write
data to one of the 128 Nexus registers and zero if you want to read from it. The
data that's read from the NAR register indicates the status: Busy (bit 1) and
Error (bit 0). The 32-bit NDR then can be used to read or write the actual
register (and execute whatever function is tied to a write).

For OCD, the OCD registers are important. Debugging is mostly done by using
these to feed the Xtensa core instructions to execute, combined with a
data register that's directly readable/writable from the JTAG port.

To execute an instruction, either write it into DIR0EXEC and it will
immediately execute. Alternatively, write it into DIR0 and write
the data for the DDR register into DDREXEC, and that also will execute
the instruction. DIR1-DIRn are for longer instructions, of which there don't
appear to be any the ESP108.
*/

/*
Multiprocessor stuff:

The ESP32 has two ESP108 processors in it, which can run in SMP-mode if an
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
Tracing:

The ESP108 core has some trace memory that can be used to trace program
flow up to a trigger point. Barebone tracing support is included in this
driver in the form of trace* commands. OpenOCD does have some existing
infrastructure for tracing hardware, but it's all very undocumented and
seems to have some ARM-specific things, so we do not use that.

The tracing infrastructure does have the option to stop at a certain PC
and trigger a debugging interrupt. Theoretically, if we do not use
the trace functionality, we might be able to use that as a 3rd hardware
breakpoint. ToDo: look into that. I asked, seems with the trace memory
disabled any traces done will disappear in the bitbucket, so this may be
very much a viable option.
*/

/*
ToDo:
This code very much assumes the host machines endianness is the same as that
of the target CPU, which isn't necessarily always the case. Specifically the
esp108_reg_set etc functions are suspect.
*/


#define XT_INS_NUM_BITS 24
#define XT_DEBUGLEVEL    6 /* XCHAL_DEBUGLEVEL in xtensa-config.h */
#define XT_NUM_BREAKPOINTS 2
#define XT_NUM_WATCHPOINTS 2


enum esp108_reg_t {
	XT_REG_GENERAL = 0,		//General-purpose register; part of the windowed register set
	XT_REG_USER = 1,		//User register, needs RUR to read
	XT_REG_SPECIAL = 2,		//Special register, needs RSR to read
	XT_REG_DEBUG = 3,		//Register used for the debug interface. Don't mess with this.
	XT_REG_RELGEN = 4,		//Relative general address. Points to the absolute addresses plus the window index
	XT_REG_FR = 5,			//Floating-point register
};

enum esp108_regflags_t {
	XT_REGF_NOREAD = 0x01,	//Register is write-only
	XT_REGF_COPROC0 = 0x02	//Can't be read if coproc0 isn't enabled
};

struct esp108_reg_desc {
	const char *name;
	int reg_num; /* ISA register num (meaning depends on register type) */
	enum esp108_reg_t type;
	enum esp108_regflags_t flags;
};

//Register file can be auto-generated
#include "esp108_regs.h"


#define _XT_INS_FORMAT_RSR(OPCODE,SR,T) (OPCODE			\
					 | ((SR & 0xFF) << 8)	\
					 | ((T & 0x0F) << 4))

#define _XT_INS_FORMAT_RRR(OPCODE,ST,R) (OPCODE			\
					 | ((ST & 0xFF) << 4)	\
					 | ((R & 0x0F) << 12))

#define _XT_INS_FORMAT_RRRN(OPCODE,S, T,IMM4) (OPCODE         \
                     | ((T & 0x0F) << 4)   \
                     | ((S & 0x0F) << 8)   \
                     | ((IMM4 & 0x0F) << 12))

#define _XT_INS_FORMAT_RRI8(OPCODE,R,S,T,IMM8) (OPCODE			\
						| ((IMM8 & 0xFF) << 16) \
						| ((R & 0x0F) << 12 )	\
						| ((S & 0x0F) << 8 )	\
						| ((T & 0x0F) << 4 ))

/* Special register number macro for DDR register.
 * this gets used a lot so making a shortcut to it is
 * useful.
 */
#define XT_SR_DDR         (esp108_regs[XT_REG_IDX_DDR].reg_num)

//Same thing for A3/A4
#define XT_REG_A3         (esp108_regs[XT_REG_IDX_AR3].reg_num)
#define XT_REG_A4         (esp108_regs[XT_REG_IDX_AR4].reg_num)


/* Xtensa processor instruction opcodes
*/
/* "Return From Debug Operation" to Normal */
#define XT_INS_RFDO      0xf1e000
/* "Return From Debug and Dispatch" - allow sw debugging stuff to take over */
#define XT_INS_RFDD      0xf1e010

/* Load to DDR register, increase addr register */
#define XT_INS_LDDR32P(S) (0x0070E0|(S<<8))
/* Store from DDR register, increase addr register */
#define XT_INS_SDDR32P(S) (0x0070F0|(S<<8))

/* Load 32-bit Indirect from A(S)+4*IMM8 to A(T) */
#define XT_INS_L32I(S,T,IMM8)  _XT_INS_FORMAT_RRI8(0x002002,0,S,T,IMM8)
/* Load 16-bit Unsigned from A(S)+2*IMM8 to A(T) */
#define XT_INS_L16UI(S,T,IMM8) _XT_INS_FORMAT_RRI8(0x001002,0,S,T,IMM8)
/* Load 8-bit Unsigned from A(S)+IMM8 to A(T) */
#define XT_INS_L8UI(S,T,IMM8)  _XT_INS_FORMAT_RRI8(0x000002,0,S,T,IMM8)

/* Store 32-bit Indirect to A(S)+4*IMM8 from A(T) */
#define XT_INS_S32I(S,T,IMM8) _XT_INS_FORMAT_RRI8(0x006002,0,S,T,IMM8)
/* Store 16-bit to A(S)+2*IMM8 from A(T) */
#define XT_INS_S16I(S,T,IMM8) _XT_INS_FORMAT_RRI8(0x005002,0,S,T,IMM8)
/* Store 8-bit to A(S)+IMM8 from A(T) */
#define XT_INS_S8I(S,T,IMM8)  _XT_INS_FORMAT_RRI8(0x004002,0,S,T,IMM8)
/* S32I.N is similar to S32I, but has a 16-bit encoding and supports a smaller range of
offset values encoded in the instruction word. */
#define XT_INS_S32I_N(S,T,IMM4) _XT_INS_FORMAT_RRRN(0x0009,S,T,IMM4)
#define XT_INS_S32I_N_GET_T(INSN)   ((INSN >> 4) & 0x0F)

/* Read Special Register */
#define XT_INS_RSR(SR,T) _XT_INS_FORMAT_RSR(0x030000,SR,T)
/* Write Special Register */
#define XT_INS_WSR(SR,T) _XT_INS_FORMAT_RSR(0x130000,SR,T)
/* Swap Special Register */
#define XT_INS_XSR(SR,T) _XT_INS_FORMAT_RSR(0x610000,SR,T)

/* Rotate Window by (-8..7) */
#define XT_INS_ROTW(N) ((0x408000)|((N&15)<<4))

/* Read User Register */
#define XT_INS_RUR(UR,T) _XT_INS_FORMAT_RRR(0xE30000,UR,T)
/* Write User Register */
#define XT_INS_WUR(UR,T) _XT_INS_FORMAT_RRR(0xF30000,UR,T)

/* Read Floating-Point Register */
#define XT_INS_RFR(FR,T) _XT_INS_FORMAT_RRR(0xFA0000,((FR<<4)|0x4),T)
/* Write Floating-Point Register */
#define XT_INS_WFR(FR,T) _XT_INS_FORMAT_RRR(0xFA0000,((FR<<4)|0x5),T)


extern int esp108_cmd_apptrace(struct command_invocation *cmd);

//forward declarations
static int xtensa_step(struct target *target,
	int current,
	uint32_t address,
	int handle_breakpoints);
static int xtensa_poll(struct target *target);


void esp108_add_set_ir(struct target *target, uint8_t value)
{
	uint8_t t[4];
	struct scan_field field;
	memset(&field, 0, sizeof field);
	field.num_bits = target->tap->ir_length;
	field.out_value = t;
	buf_set_u32(t, 0, field.num_bits, value);
	jtag_add_ir_scan(target->tap, &field, TAP_IDLE);
}


void esp108_add_dr_scan(struct target *target, int len, const uint8_t *src, uint8_t *dest, tap_state_t endstate)
{
	struct scan_field field;
	memset(&field, 0, sizeof field);
	field.num_bits=len;
	field.out_value=src;
	field.in_value=dest;
	jtag_add_dr_scan(target->tap, 1, &field, endstate);
}

//Set the PWRCTL TAP register to a value
static void esp108_queue_pwrctl_set(struct target *target, uint8_t value)
{
	esp108_add_set_ir(target, TAPINS_PWRCTL);
	esp108_add_dr_scan(target, TAPINS_PWRCTL_LEN, &value, NULL, TAP_IDLE);
}

//Read the PWRSTAT TAP register and clear the XWASRESET bits.
static void esp108_queue_pwrstat_readclear(struct target *target, uint8_t *value)
{
	const uint8_t pwrstatClr=PWRSTAT_DEBUGWASRESET|PWRSTAT_COREWASRESET;
	esp108_add_set_ir(target, TAPINS_PWRSTAT);
	esp108_add_dr_scan(target, TAPINS_PWRCTL_LEN, &pwrstatClr, value, TAP_IDLE);
}


void esp108_queue_nexus_reg_write(struct target *target, const uint8_t reg, const uint32_t value)
{
	uint8_t regdata=(reg<<1)|1;
	uint8_t valdata[]={value, value>>8, value>>16, value>>24};
	esp108_add_set_ir(target, TAPINS_NARSEL);
	esp108_add_dr_scan(target, TAPINS_NARSEL_ADRLEN, &regdata, NULL, TAP_IDLE);
	esp108_add_dr_scan(target, TAPINS_NARSEL_DATALEN, valdata, NULL, TAP_IDLE);
}

void esp108_queue_nexus_reg_read(struct target *target, const uint8_t reg, uint8_t *value)
{
	uint8_t regdata=(reg<<1)|0;
	uint8_t dummy[4]={0,0,0,0};
	esp108_add_set_ir(target, TAPINS_NARSEL);
	esp108_add_dr_scan(target, TAPINS_NARSEL_ADRLEN, &regdata, NULL, TAP_IDLE);
	esp108_add_dr_scan(target, TAPINS_NARSEL_DATALEN, dummy, value, TAP_IDLE);
}

static void esp108_queue_exec_ins(struct target *target, int32_t ins)
{
	esp108_queue_nexus_reg_write(target, NARADR_DIR0EXEC, ins);
}

static uint32_t esp108_reg_get(struct reg *reg)
{
	return *((uint32_t*)reg->value);
}

static void esp108_reg_set(struct reg *reg, uint32_t value)
{
	uint32_t oldval;
	oldval=*((uint32_t*)reg->value);
	if (oldval==value) return;
	*((uint32_t*)reg->value)=value;
	reg->dirty=1;
}


/*
The TDI pin is also used as a flash Vcc bootstrap pin. If we reset the CPU externally, the last state of the TDI pin can
allow the power to an 1.8V flash chip to be raised to 3.3V, or the other way around. Users can use the
esp108 flashbootstrap command to set a level, and this routine will make sure the tdi line will return to
that when the jtag port is idle.
*/
void esp108_queue_tdi_idle(struct target *target) {
	struct esp108_common *esp108=(struct esp108_common*)target->arch_info;
	static uint8_t value;
	uint8_t t[4]={0,0,0,0};

	if (esp108->flashBootstrap==FBS_TMSLOW) {
		//Make sure tdi is 0 at the exit of queue execution
		value=0;
	} else if (esp108->flashBootstrap==FBS_TMSHIGH) {
		//Make sure tdi is 1 at the exit of queue execution
		value=1;
	} else {
		return;
	}

	//Scan out 1 bit, do not move from IRPAUSE after we're done.
	buf_set_u32(t, 0, 1, value);
	jtag_add_plain_ir_scan(1, t, NULL, TAP_IRPAUSE);
}


//Utility function: check DSR for any weirdness and report.
//Also does tms_reset to bootstrap level indicated.
#define esp108_checkdsr(target) esp108_do_checkdsr(target, __FUNCTION__, __LINE__)
static int esp108_do_checkdsr(struct target *target, const char *function, const int line)
{
	uint8_t dsr[4];
	int res;
	int needclear=0;
	esp108_queue_nexus_reg_read(target, NARADR_DSR, dsr);
	esp108_queue_tdi_idle(target);
	res=jtag_execute_queue();
	if (res!=ERROR_OK) {
		LOG_ERROR("%s: %s (line %d): reading DSR failed!", target->cmd_name, function, line);
		return ERROR_FAIL;
	}
	if (intfromchars(dsr)&OCDDSR_EXECBUSY) {
		LOG_ERROR("%s: %s (line %d): DSR (%08X) indicates target still busy!", target->cmd_name, function, line, intfromchars(dsr));
		needclear=1;
	}
	if (intfromchars(dsr)&OCDDSR_EXECEXCEPTION) {
		LOG_ERROR("%s: %s (line %d): DSR (%08X) indicates DIR instruction generated an exception!", target->cmd_name, function, line, intfromchars(dsr));
		needclear=1;
	}
	if (intfromchars(dsr)&OCDDSR_EXECOVERRUN) {
		LOG_ERROR("%s: %s (line %d): DSR (%08X) indicates DIR instruction generated an overrun!", target->cmd_name, function, line, intfromchars(dsr));
		needclear=1;
	}
	if (needclear) {
		esp108_queue_nexus_reg_write(target, NARADR_DSR, OCDDSR_EXECEXCEPTION|OCDDSR_EXECOVERRUN);
		res=jtag_execute_queue();
		if (res!=ERROR_OK) {
			LOG_ERROR("%s: %s (line %d): clearing DSR failed!", target->cmd_name, function, line);
		}
		return ERROR_FAIL;
	}
	return ERROR_OK;
}

static void esp108_mark_register_dirty(struct target *target, int regidx)
{
	struct esp108_common *esp108=(struct esp108_common*)target->arch_info;
	struct reg *reg_list=esp108->core_cache->reg_list;
	reg_list[regidx].dirty=1;
}

//Convert a register index that's indexed relative to windowbase, to the real address.
static enum xtensa_reg_idx windowbase_offset_to_canonical(const enum xtensa_reg_idx reg, const int windowbase)
{
	int idx;
	if (reg>=XT_REG_IDX_AR0 && reg<=XT_REG_IDX_AR63) {
		idx=reg-XT_REG_IDX_AR0;
	} else if (reg>=XT_REG_IDX_A0 && reg<=XT_REG_IDX_A15) {
		idx=reg-XT_REG_IDX_A0;
	} else {
		LOG_ERROR("Error: can't convert register %d to non-windowbased register!\n", reg);
		return -1;
	}
	return ((idx+(windowbase*4))&63)+XT_REG_IDX_AR0;
}

static enum xtensa_reg_idx canonical_to_windowbase_offset(const enum xtensa_reg_idx reg, const int windowbase)
{
	return windowbase_offset_to_canonical(reg, -windowbase);
}

static int regReadable(int flags, int cpenable) {
	if (flags&XT_REGF_NOREAD) return 0;
	if ((flags&XT_REGF_COPROC0) && (cpenable&(1<<0))==0) return 0;
	return 1;

}

static int esp108_fetch_all_regs(struct target *target)
{
	int i, j;
	int cpenable;
	int res;
	uint32_t regval;
	uint32_t windowbase;
	struct esp108_common *esp108=(struct esp108_common*)target->arch_info;
	struct reg *reg_list=esp108->core_cache->reg_list;
	uint8_t regvals[XT_NUM_REGS][4];
	uint8_t dsrs[XT_NUM_REGS][4];

	//Assume the CPU has just halted. We now want to fill the register cache with all the
	//register contents GDB needs. For speed, we pipeline all the read operations, execute them
	//in one go, then sort everything out from the regvals variable.


	//Start out with A0-A63; we can reach those immediately. Grab them per 16 registers.
	for (j=0; j<64; j+=16) {
		//Grab the 16 registers we can see
		for (i=0; i<16; i++) {
			esp108_queue_exec_ins(target, XT_INS_WSR(XT_SR_DDR, esp108_regs[XT_REG_IDX_AR0+i].reg_num));
			esp108_queue_nexus_reg_read(target, NARADR_DDR, regvals[XT_REG_IDX_AR0+i+j]);
			esp108_queue_nexus_reg_read(target, NARADR_DSR, dsrs[XT_REG_IDX_AR0+i+j]);
		}
		//Now rotate the window so we'll see the next 16 registers. The final rotate will wraparound,
		//leaving us in the state we were.
		esp108_queue_exec_ins(target, XT_INS_ROTW(4));
	}

	//As the very first thing after A0-A63, go grab the CPENABLE registers. It indicates if we can also grab the FP
	//(and theoretically other coprocessor) registers, or if this is a bad thing to do.
	esp108_queue_exec_ins(target, XT_INS_RSR(esp108_regs[XT_REG_IDX_CPENABLE].reg_num, XT_REG_A3));
	esp108_queue_exec_ins(target, XT_INS_WSR(XT_SR_DDR, XT_REG_A3));
	esp108_queue_nexus_reg_read(target, NARADR_DDR, regvals[XT_REG_IDX_CPENABLE]);
	res=jtag_execute_queue();
	if (res!=ERROR_OK) return res;
	esp108_checkdsr(target);
	cpenable=intfromchars(regvals[XT_REG_IDX_CPENABLE]);


	//We're now free to use any of A0-A15 as scratch registers
	//Grab the SFRs and user registers first. We use A3 as a scratch register.
	for (i=0; i<XT_NUM_REGS; i++) {
		if (regReadable(esp108_regs[i].flags, cpenable) && (esp108_regs[i].type==XT_REG_SPECIAL || esp108_regs[i].type==XT_REG_USER)) {
			if (esp108_regs[i].type==XT_REG_USER) {
				esp108_queue_exec_ins(target, XT_INS_RUR(esp108_regs[i].reg_num, XT_REG_A3));
			} else if (esp108_regs[i].type==XT_REG_FR) {
				esp108_queue_exec_ins(target, XT_INS_RFR(esp108_regs[i].reg_num, XT_REG_A3));
			} else { //SFR
				esp108_queue_exec_ins(target, XT_INS_RSR(esp108_regs[i].reg_num, XT_REG_A3));
			}
			esp108_queue_exec_ins(target, XT_INS_WSR(XT_SR_DDR, XT_REG_A3));
			esp108_queue_nexus_reg_read(target, NARADR_DDR, regvals[i]);
			esp108_queue_nexus_reg_read(target, NARADR_DSR, dsrs[i]);
		}
	}

	//Ok, send the whole mess to the CPU.
	res=jtag_execute_queue();
	if (res!=ERROR_OK) return res;
	esp108_checkdsr(target);


	//DSR checking: follows order in which registers are requested.
	for (i=0; i<XT_NUM_REGS; i++) {
		if (regReadable(esp108_regs[i].flags, cpenable) && (esp108_regs[i].type==XT_REG_SPECIAL || esp108_regs[i].type==XT_REG_USER)) {
			if (intfromchars(dsrs[i])&OCDDSR_EXECEXCEPTION) {
				LOG_ERROR("Exception reading %s!\n", esp108_regs[i].name);
				return ERROR_FAIL;
			}
		}
	}

	//We need the windowbase to decode the general addresses.
	windowbase=intfromchars(regvals[XT_REG_IDX_WINDOWBASE]);
	//Decode the result and update the cache.
	for (i=0; i<XT_NUM_REGS; i++) {
		if (regReadable(esp108_regs[i].flags, cpenable)) {
			if (esp108_regs[i].type==XT_REG_GENERAL) {
				//The 64-value general register set is read from (windowbase) on down. We need
				//to get the real register address by subtracting windowbase and wrapping around.
				int realadr=canonical_to_windowbase_offset(i, windowbase);
				regval=intfromchars(regvals[realadr]);
//				LOG_INFO("mapping: %s -> %s (windowbase off %d)\n",esp108_regs[i].name, esp108_regs[realadr].name, windowbase*4);
			} else if (esp108_regs[i].type==XT_REG_RELGEN) {
				regval=intfromchars(regvals[esp108_regs[i].reg_num]);
			} else {
				regval=intfromchars(regvals[i]);
//				LOG_INFO("Register %s: 0x%X", esp108_regs[i].name, regval);
			}
			esp108_reg_set(&reg_list[i], regval);
			reg_list[i].valid=1;
			reg_list[i].dirty=0; //always do this _after_ esp108_reg_set!
		} else {
			reg_list[i].valid=0;
		}
	}
	//We have used A3 as a scratch register and we will need to write that back.
	esp108_mark_register_dirty(target, XT_REG_IDX_A3);


	return ERROR_OK;
}


static int esp108_write_dirty_registers(struct target *target)
{
	int i, j;
	int res;
	uint32_t regval, windowbase;
	struct esp108_common *esp108=(struct esp108_common*)target->arch_info;
	struct reg *reg_list=esp108->core_cache->reg_list;

	LOG_DEBUG("%s: %s", target->cmd_name, __FUNCTION__);

	//We need to write the dirty registers in the cache list back to the processor.
	//Start by writing the SFR/user registers.
	for (i=0; i<XT_NUM_REGS; i++) {
		if (reg_list[i].dirty) {
			if (esp108_regs[i].type==XT_REG_SPECIAL || esp108_regs[i].type==XT_REG_USER) {
				regval=esp108_reg_get(&reg_list[i]);
				LOG_DEBUG("%s: Writing back reg %s val %08X", target->cmd_name, esp108_regs[i].name, regval);
				esp108_queue_nexus_reg_write(target, NARADR_DDR, regval);
				esp108_queue_exec_ins(target, XT_INS_RSR(XT_SR_DDR, XT_REG_A3));
				if (esp108_regs[i].type==XT_REG_USER) {
					esp108_queue_exec_ins(target, XT_INS_WUR(esp108_regs[i].reg_num, XT_REG_A3));
				} else if (esp108_regs[i].type==XT_REG_FR) {
					esp108_queue_exec_ins(target, XT_INS_WFR(esp108_regs[i].reg_num, XT_REG_A3));
				} else { //SFR
					esp108_queue_exec_ins(target, XT_INS_WSR(esp108_regs[i].reg_num, XT_REG_A3));
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
			LOG_DEBUG("%s: Writing back reg %s value %08X", target->cmd_name, esp108_regs[XT_REG_IDX_A0+i].name, regval);
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
				LOG_DEBUG("%s: Writing back reg %s value %08X", target->cmd_name, esp108_regs[realadr].name, regval);
				esp108_queue_nexus_reg_write(target, NARADR_DDR, regval);
				esp108_queue_exec_ins(target, XT_INS_RSR(XT_SR_DDR, esp108_regs[XT_REG_IDX_AR0+i+j].reg_num));
				reg_list[realadr].dirty=0;
			}
		}
		//Now rotate the window so we'll see the next 16 registers. The final rotate will wraparound,
		//leaving us in the state we were.
		esp108_queue_exec_ins(target, XT_INS_ROTW(4));
	}
	res=jtag_execute_queue();
	esp108_checkdsr(target);
	return res;
}

static int xtensa_halt(struct target *target)
{
	int res;

	LOG_DEBUG("%s, target: %s", __func__, target->cmd_name);
	if (target->state == TARGET_HALTED) {
		LOG_DEBUG("%s: target was already halted", target->cmd_name);
		return ERROR_OK;
	}

	esp108_queue_nexus_reg_write(target, NARADR_DCRSET, OCDDCR_DEBUGINTERRUPT);
	esp108_queue_tdi_idle(target);
	res=jtag_execute_queue();

	if(res != ERROR_OK) {
		LOG_ERROR("%s: Failed to set OCDDCR_DEBUGINTERRUPT. Can't halt.", target->cmd_name);
		return ERROR_FAIL;
	}
	return ERROR_OK;
}

static int xtensa_resume(struct target *target,
			 int current,
			 uint32_t address,
			 int handle_breakpoints,
			 int debug_execution)
{
	struct esp108_common *esp108=(struct esp108_common*)target->arch_info;
	struct reg *reg_list=esp108->core_cache->reg_list;
	int res=ERROR_OK;
	size_t slot;
	uint32_t bpena;

	LOG_DEBUG("%s: %s current=%d address=%04" PRIx32, target->cmd_name, __func__, current, address);

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
	}

	//Write back hw breakpoints. Current FreeRTOS SMP code can set a hw breakpoint on an
	//exception; we need to clear that and return to the breakpoints gdb has set on resume.
	bpena=0;
	for(slot = 0; slot < esp108->num_brps; slot++) {
		if (esp108->hw_brps[slot]!=NULL) {
			/* Write IBREAKA[slot] and set bit #slot in IBREAKENABLE */
			esp108_reg_set(&reg_list[XT_REG_IDX_IBREAKA0+slot], esp108->hw_brps[slot]->address);
			bpena|=(1<<slot);
		}
	}
	esp108_reg_set(&reg_list[XT_REG_IDX_IBREAKENABLE], bpena);

	res=esp108_write_dirty_registers(target);
	if(res != ERROR_OK) {
		LOG_ERROR("%s: Failed to write back register cache.", target->cmd_name);
		return ERROR_FAIL;
	}

	//Execute return from debug exception instruction
	esp108_queue_exec_ins(target, XT_INS_RFDO);
	res=jtag_execute_queue();
	if(res != ERROR_OK) {
		LOG_ERROR("%s: Failed to clear OCDDCR_DEBUGINTERRUPT and resume execution.", target->cmd_name);
		return ERROR_FAIL;
	}
	esp108_checkdsr(target);

	target->debug_reason = DBG_REASON_NOTHALTED;
	if (!debug_execution)
		target->state = TARGET_RUNNING;
	else
		target->state = TARGET_DEBUG_RUNNING;
	res = target_call_event_callbacks(target, TARGET_EVENT_RESUMED);

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

//	LOG_INFO("%s: %s: reading %d bytes from addr %08X", target->cmd_name, __FUNCTION__, size*count, address);
//	LOG_INFO("Converted to aligned addresses: read from %08X to %08X", addrstart_al, addrend_al);
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
	esp108_mark_register_dirty(target, XT_REG_IDX_A3);
	//Write start address to A3
	esp108_queue_nexus_reg_write(target, NARADR_DDR, addrstart_al);
	esp108_queue_exec_ins(target, XT_INS_RSR(XT_SR_DDR, XT_REG_A3));
	//Now we can safely read data from addrstart_al up to addrend_al into albuff
	while (adr!=addrend_al) {
		esp108_queue_exec_ins(target, XT_INS_LDDR32P(XT_REG_A3));
		esp108_queue_nexus_reg_read(target, NARADR_DDR, &albuff[i]);
		adr+=4;
		i+=4;
	}
	res=jtag_execute_queue();
	if (res==ERROR_OK) res=esp108_checkdsr(target);
	if (res!=ERROR_OK) LOG_WARNING("%s: Failed reading %d bytes at address 0x%08X",target->cmd_name, count*size, address);

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

	if (target->state != TARGET_HALTED) {
		LOG_WARNING("%s: %s: target not halted", __func__, target->cmd_name);
		return ERROR_TARGET_NOT_HALTED;
	}

//	LOG_INFO("%s: %s: writing %d bytes to addr %08X", target->cmd_name, __FUNCTION__, size*count, address);
//	LOG_INFO("al start %x al end %x", addrstart_al, addrend_al);

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

	//We're going to use A3 here
	esp108_mark_register_dirty(target, XT_REG_IDX_A3);

	//If we're using a temp aligned buffer, we need to fill the head and/or tail bit of it.
	if (albuff!=buffer) {
		//See if we need to read the first and/or last word.
		if (address&3) {
			esp108_queue_nexus_reg_write(target, NARADR_DDR, addrstart_al);
			esp108_queue_exec_ins(target, XT_INS_RSR(XT_SR_DDR, XT_REG_A3));
			esp108_queue_exec_ins(target, XT_INS_LDDR32P(XT_REG_A3));
			esp108_queue_nexus_reg_read(target, NARADR_DDR, &albuff[0]);
		}
		if ((address+(size*count))&3) {
			esp108_queue_nexus_reg_write(target, NARADR_DDR, addrend_al-4);
			esp108_queue_exec_ins(target, XT_INS_RSR(XT_SR_DDR, XT_REG_A3));
			esp108_queue_exec_ins(target, XT_INS_LDDR32P(XT_REG_A3));
			esp108_queue_nexus_reg_read(target, NARADR_DDR, &albuff[addrend_al-addrstart_al-4]);
		}
		//Grab bytes
		res=jtag_execute_queue();
		esp108_checkdsr(target);
		//Copy data to be written into the aligned buffer
		memcpy(&albuff[address&3], buffer, size*count);
		//Now we can write albuff in aligned uint32s.
	}

	//Write start address to A3
	esp108_queue_nexus_reg_write(target, NARADR_DDR, addrstart_al);
	esp108_queue_exec_ins(target, XT_INS_RSR(XT_SR_DDR, XT_REG_A3));
	//Write the aligned buffer
	while (adr!=addrend_al) {
		esp108_queue_nexus_reg_write(target, NARADR_DDR, intfromchars(&albuff[i]));
		esp108_queue_exec_ins(target, XT_INS_SDDR32P(XT_REG_A3));
		adr+=4;
		i+=4;
	}
	res=jtag_execute_queue();
	if (res==ERROR_OK) res=esp108_checkdsr(target);
	if (res!=ERROR_OK) LOG_WARNING("%s: Failed writing %d bytes at address 0x%08X",target->cmd_name, count*size, address);

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
	struct esp108_common *esp108 = target->arch_info;
	LOG_DEBUG("%s", __func__);

	*reg_list_size = XT_NUM_REGS_G_COMMAND;
	*reg_list = malloc(sizeof(struct reg *) * (*reg_list_size));

	if (!*reg_list) {
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	for (i = 0; i < XT_NUM_REGS_G_COMMAND; i++) {
		(*reg_list)[i] = &esp108->core_cache->reg_list[i];
	}

	return ERROR_OK;
}

static int xtensa_get_core_reg(struct reg *reg)
{
//We don't need this because we read all registers on halt anyway.
	struct esp108_common *esp108 = reg->arch_info;
	struct target *target = esp108->target;
	if (target->state != TARGET_HALTED) return ERROR_TARGET_NOT_HALTED;
	return ERROR_OK;
}

static int xtensa_set_core_reg(struct reg *reg, uint8_t *buf)
{
	struct esp108_common *esp108 = reg->arch_info;
	struct target *target = esp108->target;

	uint32_t value = buf_get_u32(buf, 0, 32);

	if (target->state != TARGET_HALTED) return ERROR_TARGET_NOT_HALTED;

	buf_set_u32(reg->value, 0, reg->size, value);
	reg->dirty = 1;
	reg->valid = 1;
	return ERROR_OK;
}

static int xtensa_write_uint32(struct target *target, uint32_t addr, uint32_t val)
{
	return xtensa_write_memory(target, addr, 4, 1, (uint8_t*) &val);
}

static int xtensa_write_uint32_list(struct target *target, const uint32_t* addr_value_pairs_list, size_t count)
{
	int res;
	for (size_t i = 0; i < count; ++i) 
	{
		res = xtensa_write_uint32(target, addr_value_pairs_list[2 * i], addr_value_pairs_list[2 * i + 1]);
		if (res != ERROR_OK) {
			LOG_ERROR("%s: error writing to %08x", __func__, addr_value_pairs_list[2 * i]);
			return res;
		}
	}
	return ERROR_OK;
}

static int xtensa_assert_reset(struct target *target);
static int xtensa_deassert_reset(struct target *target);

/* Reset ESP32's peripherals.
   Postconditions: all peripherals except RTC_CNTL are reset, CPU's PC is undefined, PRO CPU is halted, APP CPU is in reset 
   How this works:
    0. make sure target is halted; if not, try to halt it; if that fails, try to reset it (via OCD) and then halt
    1. set CPU initial PC to 0x50000000 (RTC_SLOW_MEM) by clearing RTC_CNTL_{PRO,APP}CPU_STAT_VECTOR_SEL
    2. load stub code into RTC_SLOW_MEM; once executed, stub code will disable watchdogs and make CPU spin in an idle loop.
    3. trigger SoC reset using RTC_CNTL_SW_SYS_RST bit
    4. wait for the OCD to be reset
    5. halt the target and wait for it to be halted (at this point CPU is in the idle loop)
    6. restore initial PC and the contents of RTC_SLOW_MEM
   TODO: some state of RTC_CNTL is not reset during SW_SYS_RST. Need to reset that manually.
 */
static int esp32_soc_reset(struct target *target)
{
	LOG_DEBUG("%s %d", __func__, __LINE__);
	int res;

	/* In order to write to peripheral registers, target must be halted first */
	if (target->state != TARGET_HALTED) {
		LOG_DEBUG("%s: Target not halted before SoC reset, trying to halt it first", __func__);
		xtensa_halt(target);
		res = target_wait_state(target, TARGET_HALTED, 1000);
		if (res != ERROR_OK) {
			LOG_DEBUG("%s: Couldn't halt target before SoC reset, trying to do reset-halt", __func__);
			res = xtensa_assert_reset(target);
			if (res != ERROR_OK) {
				LOG_ERROR("%s: Couldn't halt target before SoC reset! (xtensa_assert_reset returned %d)", __func__, res);
				return res;
			}
			alive_sleep(10);
			xtensa_poll(target);
			int reset_halt_save = target->reset_halt;
			target->reset_halt = 1;
			res = xtensa_deassert_reset(target);
			target->reset_halt = reset_halt_save;
			if (res != ERROR_OK) {
				LOG_ERROR("%s: Couldn't halt target before SoC reset! (xtensa_deassert_reset returned %d)", __func__, res);
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


	const uint32_t esp32_post_reset_code[] = {
		0x00000806, 0x50d83aa1, 0x00000000, 0x3ff480a4, 0x3ff4808c, 0x3ff5f064, 0x3ff5f048, 0x3ff60064, 
		0x3ff60048, 0x41fff831, 0x0439fff9, 0x39fffa41, 0xfffa4104, 0xf4310439, 0xfff541ff, 0xf6410439, 
		0x410439ff, 0x0439fff7, 0x46007000,
	};

	uint32_t slow_mem_save[sizeof(esp32_post_reset_code) / sizeof(uint32_t)];

	const int RTC_SLOW_MEM_BASE = 0x50000000;
	/* Save contents of RTC_SLOW_MEM which we are about to overwrite */
	res = xtensa_read_buffer(target, RTC_SLOW_MEM_BASE, sizeof(slow_mem_save), (uint8_t*) slow_mem_save);
	if (res != ERROR_OK)  {
		LOG_ERROR("%s %d err=%d", __func__, __LINE__, res);
		return res;
	}

	/* Write stub code into RTC_SLOW_MEM */
	res = xtensa_write_buffer(target, RTC_SLOW_MEM_BASE, sizeof(esp32_post_reset_code), (const uint8_t*) esp32_post_reset_code);
	if (res != ERROR_OK)  {
		LOG_ERROR("%s %d err=%d", __func__, __LINE__, res);
		return res;
	}

	const int RTC_CNTL_RESET_STATE_REG = 0x3ff48034;
	const int RTC_CNTL_RESET_STATE_DEF = 0x3000;
	const int RTC_CNTL_CLK_CONF_REG = 0x3ff48070;
	const int RTC_CNTL_CLK_CONF_DEF = 0x2210;
	const int RTC_CNTL_STORE4_REG = 0x3ff480b0;
	const int RTC_CNTL_STORE5_REG = 0x3ff480b4;
	const int RTC_CNTL_OPTIONS0_REG = 0x3ff48000;
	const int RTC_CNTL_OPTIONS0_DEF = 0x1c492000;
	const int RTC_CNTL_SW_SYS_RST = 0x80000000;
	const int DPORT_APPCPU_CTRL_A_REG = 0x3ff0002c;
	const int DPORT_APPCPU_CTRL_B_REG = 0x3ff00030;
	const int DPORT_APPCPU_CLKGATE_EN = 0x1;
	const int DPORT_APPCPU_CTRL_D_REG = 0x3ff00038;

	/* Set a list of registers to these values */
	const uint32_t reg_value_pairs_pre[] = {
		/* Set entry point to RTC_SLOW_MEM */
		RTC_CNTL_RESET_STATE_REG, 0,
		/* Reset SoC clock to XTAL, in case it was running from PLL */
		RTC_CNTL_CLK_CONF_REG, RTC_CNTL_CLK_CONF_DEF,
		/* Reset RTC_CNTL_STORE{4,5}_REG, which are related to clock state */
		RTC_CNTL_STORE4_REG, 0,
		RTC_CNTL_STORE5_REG, 0,
		/* Perform reset */
		RTC_CNTL_OPTIONS0_REG, RTC_CNTL_OPTIONS0_DEF | RTC_CNTL_SW_SYS_RST
	};
	res = xtensa_write_uint32_list(target, reg_value_pairs_pre, sizeof(reg_value_pairs_pre) / 8);
	if (res != ERROR_OK)  {
		LOG_WARNING("%s xtensa_write_uint32_list (reg_value_pairs_pre) err=%d", __func__, res);
		return res;
	}

	/* Wait for SoC to reset */
	res = target_wait_state(target, TARGET_RUNNING, 1000);
	if (res != ERROR_OK) {
		LOG_ERROR("%s: Timed out waiting for CPU to be reset", __func__);
		return ERROR_TARGET_TIMEOUT;
	}

	/* Halt the CPU again */
	xtensa_halt(target);
	res = target_wait_state(target, TARGET_HALTED, 1000);
	if (res != ERROR_OK) {
		LOG_ERROR("%s: Timed out waiting for CPU to be halted after SoC reset", __func__);
		return res;
	}

	const uint32_t reg_value_pairs_post[] = {
		/* Reset entry point back to the reset vector */
		RTC_CNTL_RESET_STATE_REG, RTC_CNTL_RESET_STATE_DEF,
		/* Clear APP CPU boot address */
		DPORT_APPCPU_CTRL_D_REG, 0,
		/* Enable clock to APP CPU */
		DPORT_APPCPU_CTRL_B_REG, DPORT_APPCPU_CLKGATE_EN,
		/* Take APP CPU out of reset */
		DPORT_APPCPU_CTRL_A_REG, 0,
	};
	res = xtensa_write_uint32_list(target, reg_value_pairs_post, sizeof(reg_value_pairs_post) / 8);
	if (res != ERROR_OK)  {
		LOG_WARNING("%s xtensa_write_uint32_list (reg_value_pairs_post) err=%d", __func__, res);
		return res;
	}

	/* Restore the original contents of RTC_SLOW_MEM */
	res = xtensa_write_buffer(target, RTC_SLOW_MEM_BASE, sizeof(slow_mem_save), (const uint8_t*) slow_mem_save);
	if (res != ERROR_OK)  {
		LOG_ERROR("%s %d err=%d", __func__, __LINE__, res);
		return res;
	}

	LOG_DEBUG("%s %d", __func__, __LINE__);

	return ERROR_OK;
}

static int xtensa_assert_reset_full(struct target *target)
{
	int res;
	LOG_DEBUG("%s coreid=%d", __func__, target->coreid);
	/* Reset the SoC first */

	if (target->coreid != 1) {
		res = esp32_soc_reset(target);
		if (res != ERROR_OK) {
			LOG_WARNING("%s: failed to reset SoC; continuing anyway", __func__);
		}
	} else {
		res = target_wait_state(target, TARGET_RUNNING, 1000);
		if (res != ERROR_OK) {
			LOG_WARNING("%s: timed out waiting for APP CPU to start running", __func__);
		}
	}
	return xtensa_assert_reset(target);
}


static int xtensa_assert_reset(struct target *target)
{
	struct esp108_common *esp108=(struct esp108_common*)target->arch_info;
	int res;
	LOG_DEBUG("%s", __func__);
	target->state = TARGET_RESET;
	esp108_queue_pwrctl_set(target, PWRCTL_JTAGDEBUGUSE|PWRCTL_DEBUGWAKEUP|PWRCTL_MEMWAKEUP|PWRCTL_COREWAKEUP|PWRCTL_CORERESET);
	res=jtag_execute_queue();
	esp108_queue_tdi_idle(target);
	esp108->resetAsserted=1;
	return res;
}

static int xtensa_deassert_reset(struct target *target)
{
	struct esp108_common *esp108=(struct esp108_common*)target->arch_info;
	int res;
	LOG_DEBUG("%s coreid=%d halt=%d", __func__, target->coreid, target->reset_halt);
	if (target->reset_halt) {
		esp108_queue_nexus_reg_write(target, NARADR_DCRSET, OCDDCR_DEBUGINTERRUPT);
	}
	esp108_queue_pwrctl_set(target, PWRCTL_JTAGDEBUGUSE|PWRCTL_DEBUGWAKEUP|PWRCTL_MEMWAKEUP|PWRCTL_COREWAKEUP);
	esp108_queue_tdi_idle(target);
	res=jtag_execute_queue();
	target->state = TARGET_RUNNING;
	esp108->resetAsserted=0;
	return res;
}


static int xtensa_add_breakpoint(struct target *target, struct breakpoint *breakpoint)
{
	struct esp108_common *esp108=(struct esp108_common*)target->arch_info;
	size_t slot;

	if (target->state != TARGET_HALTED) {
		LOG_WARNING("%s: target not halted", target->cmd_name);
		return ERROR_TARGET_NOT_HALTED;
	}

	if (breakpoint->type == BKPT_SOFT) {
		LOG_ERROR("%s: sw breakpoint requested, but software breakpoints not enabled", target->cmd_name);
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	for(slot = 0; slot < esp108->num_brps; slot++) {
		if (esp108->hw_brps[slot] == NULL || esp108->hw_brps[slot] == breakpoint) break;
	}
	if (slot==esp108->num_brps) return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;

	esp108->hw_brps[slot] = breakpoint;
	//We will actually write the breakpoints when we resume the target.

	LOG_INFO("%s: placed hw breakpoint %d at 0x%X", target->cmd_name, (int)slot, breakpoint->address);
	return ERROR_OK;
}


static int xtensa_remove_breakpoint(struct target *target, struct breakpoint *breakpoint)
{
	struct esp108_common *esp108=(struct esp108_common*)target->arch_info;
	size_t slot;

	if (target->state != TARGET_HALTED) {
		LOG_WARNING("%s: target not halted", target->cmd_name);
		return ERROR_TARGET_NOT_HALTED;
	}

	for(slot = 0; slot < esp108->num_brps; slot++) {
		if(esp108->hw_brps[slot] == breakpoint)
			break;
	}
	if (slot==esp108->num_brps) return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	esp108->hw_brps[slot] = NULL;
	LOG_INFO("%s: cleared hw breakpoint %d at 0x%X", target->cmd_name, (int)slot, breakpoint->address);
	return ERROR_OK;
}

static int xtensa_add_watchpoint(struct target *target, struct watchpoint *watchpoint)
{
	struct esp108_common *esp108=(struct esp108_common*)target->arch_info;
	struct reg *reg_list=esp108->core_cache->reg_list;
	size_t slot;
	int dbreakcval;

	if (target->state != TARGET_HALTED) {
		LOG_WARNING("%s:target not halted", target->cmd_name);
		return ERROR_TARGET_NOT_HALTED;
	}

	if (watchpoint->mask != ~(uint32_t)0) {
		LOG_ERROR("%s: watchpoint value masks not supported", target->cmd_name);
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	for(slot = 0; slot < esp108->num_wps; slot++) {
		if (esp108->hw_wps[slot] == NULL || esp108->hw_wps[slot] == watchpoint) break;
	}
	if (slot==esp108->num_wps) return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;

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
	esp108_reg_set(&reg_list[XT_REG_IDX_DBREAKA0+slot], watchpoint->address);
	esp108_reg_set(&reg_list[XT_REG_IDX_DBREAKC0+slot], dbreakcval);
	esp108->hw_wps[slot] = watchpoint;

	return ERROR_OK;
}

static int xtensa_remove_watchpoint(struct target *target, struct watchpoint *watchpoint)
{
	struct esp108_common *esp108=(struct esp108_common*)target->arch_info;
	struct reg *reg_list = esp108->core_cache->reg_list;
	size_t slot;

	if (target->state != TARGET_HALTED) {
		LOG_WARNING("%s: target not halted", target->cmd_name);
		return ERROR_TARGET_NOT_HALTED;
	}

	for(slot = 0; slot < esp108->num_wps; slot++) {
		if(esp108->hw_wps[slot] == watchpoint)
			break;
	}
	if (slot==esp108->num_wps) return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;

	/* Clear DBREAKC[slot] to disable watchpoint */
	esp108_reg_set(&reg_list[XT_REG_IDX_DBREAKC0+slot], 0);
	esp108->hw_wps[slot] = NULL;

	return ERROR_OK;

}

static int xtensa_step(struct target *target,
	int current,
	uint32_t address,
	int handle_breakpoints)
{
	struct esp108_common *esp108=(struct esp108_common*)target->arch_info;
	struct reg *reg_list = esp108->core_cache->reg_list;
	int res, cause;
	uint32_t dbreakc[esp108->num_wps*4];
	size_t slot;
	uint8_t dsr[4];
	static const uint32_t icount_val = -2; /* ICOUNT value to load for 1 step */
	uint32_t icountlvl;
	uint32_t oldps, newps, oldpc;
	int tries=100;

	if (target->state != TARGET_HALTED) {
		LOG_WARNING("%s: %s: target not halted", target->cmd_name, __func__);
		return ERROR_TARGET_NOT_HALTED;
	}

	//Save old ps/pc
	oldps=esp108_reg_get(&reg_list[XT_REG_IDX_PS]);
	oldpc=esp108_reg_get(&reg_list[XT_REG_IDX_PC]);

	//If intlevel precludes single-stepping, downgrade it
	if ((oldps&0xF)==0xF) {
		LOG_INFO("PS is %d, which is too high for single-stepping. Resetting to 6.", oldps&0xF);
		esp108_reg_set(&reg_list[XT_REG_IDX_PS], (oldps&~0xf)|6);
	}

	cause=esp108_reg_get(&reg_list[XT_REG_IDX_DEBUGCAUSE]);
	if (cause&DEBUGCAUSE_DB) {
		//We stopped due to a watchpoint. We can't just resume executing the instruction again because
		//that would trigger the watchpoint again. To fix this, we remove watchpoints, single-step and
		//re-enable the watchpoint.
		LOG_DEBUG("%s: Single-stepping to get past instruction that triggered the watchpoint...", target->cmd_name);
		esp108_reg_set(&reg_list[XT_REG_IDX_DEBUGCAUSE], 0); //so we don't recurse into the same routine
		reg_list[XT_REG_IDX_DEBUGCAUSE].dirty=0;
		//Save all DBREAKCx registers and set to 0 to disable watchpoints
		for(slot = 0; slot < esp108->num_wps; slot++) {
			dbreakc[slot]=esp108_reg_get(&reg_list[XT_REG_IDX_DBREAKC0+slot]);
			esp108_reg_set(&reg_list[XT_REG_IDX_DBREAKC0+slot], 0);
		}
	}

	//Sometimes (because of eg an interrupt) the pc won't actually increment. In that case, we repeat the
	//step.
	//(Later edit: Not sure about that actually... may have been a glitch in my logic. I'm keeping in this
	//loop anyway, it probably doesn't hurt anyway.)
	do {
		icountlvl=(esp108_reg_get(&reg_list[XT_REG_IDX_PS])&15)+1;
		if (icountlvl>15) icountlvl=15;

		/* Load debug level into ICOUNTLEVEL. We'll make this one more than the current intlevel. */
		esp108_reg_set(&reg_list[XT_REG_IDX_ICOUNTLEVEL], icountlvl);
		esp108_reg_set(&reg_list[XT_REG_IDX_ICOUNT], icount_val);

		/* Now ICOUNT is set, we can resume as if we were going to run */
		res = xtensa_resume(target, current, address, 0, 0);
		if(res != ERROR_OK) {
			LOG_ERROR("%s: %s: Failed to resume after setting up single step", target->cmd_name, __func__);
			return res;
		}

		/* Wait for stepping to complete */
		int64_t start = timeval_ms();
		while(timeval_ms() < start+500) {
			//Do not use target_poll here, it also triggers other things... just manually read the DSR until stepping
			//is complete.
			esp108_queue_nexus_reg_read(target, NARADR_DSR, dsr);
			esp108_queue_tdi_idle(target);
			res=jtag_execute_queue();
			if(res != ERROR_OK) return res;
			if (intfromchars(dsr)&OCDDSR_STOPPED) {
				break;
			} else {
				usleep(50000);
			}
		}
		if(!(intfromchars(dsr)&OCDDSR_STOPPED)) {
			LOG_ERROR("%s: %s: Timed out waiting for target to finish stepping.", target->cmd_name, __func__);
			return ERROR_TARGET_TIMEOUT;
		} else {
			target->state = TARGET_HALTED;
			esp108_fetch_all_regs(target);
		}
	} while (esp108_reg_get(&reg_list[XT_REG_IDX_PC])==oldpc && --tries);
	LOG_DEBUG("Stepped from %X to %X", oldpc, esp108_reg_get(&reg_list[XT_REG_IDX_PC]));

	if (!tries) {
		LOG_ERROR("%s: %s: Stepping doesn't seem to change PC!", target->cmd_name, __func__);
	}

	if (cause&DEBUGCAUSE_DB) {
		LOG_DEBUG("%s: ...Done, re-instating watchpoints.", target->cmd_name);
		//Restore the DBREAKCx registers
		for(slot = 0; slot < esp108->num_wps; slot++) {
			esp108_reg_set(&reg_list[XT_REG_IDX_DBREAKC0+slot], dbreakc[slot]);
		}
	}

	//Restore int level
	//ToDo: Theoretically, this can mess up stepping over an instruction that modifies ps.intlevel
	//by itself. Hmmm. ToDo: Look into this.
	newps=esp108_reg_get(&reg_list[XT_REG_IDX_PS]);
	if (((oldps&0xF)>6) && (newps&0xf)!=(oldps&0xf)) {
		newps=(newps&~0xf)|(oldps&0xf);
		esp108_reg_set(&reg_list[XT_REG_IDX_PS], newps);
	}

	/* write ICOUNTLEVEL back to zero */
	esp108_reg_set(&reg_list[XT_REG_IDX_ICOUNTLEVEL], 0);
	res=esp108_write_dirty_registers(target);

	//Make sure the poll routine will pick up that something has changed by artificially
	//triggering a running->halted state change
	target->state = TARGET_RUNNING;


	return res;
}



static const struct reg_arch_type esp108_reg_type = {
	.get = xtensa_get_core_reg,
	.set = xtensa_set_core_reg,
};


static int xtensa_target_create(struct target *target, Jim_Interp *interp)
{
	struct esp108_common *esp108 = calloc(1, sizeof(struct esp108_common));
	struct reg_cache **cache_p = register_get_last_cache_p(&target->reg_cache);
	struct reg_cache *cache = malloc(sizeof(struct reg_cache));
	struct reg *reg_list = calloc(XT_NUM_REGS, sizeof(struct reg));
	uint8_t i;

	if (!esp108)
		return ERROR_COMMAND_SYNTAX_ERROR;

	target->arch_info = esp108;
	esp108->target=target;
//	xtensa->tap = target->tap;

	esp108->num_brps = XT_NUM_BREAKPOINTS;
	esp108->hw_brps = calloc(XT_NUM_BREAKPOINTS, sizeof(struct breakpoint *));
	esp108->num_wps = XT_NUM_WATCHPOINTS;
	esp108->hw_wps = calloc(XT_NUM_WATCHPOINTS, sizeof(struct watchpoint *));

	//Create the register cache
	cache->name = "Xtensa registers";
	cache->next = NULL;
	cache->reg_list = reg_list;
	cache->num_regs = XT_NUM_REGS;
	*cache_p = cache;
	esp108->core_cache = cache;

	esp108->flashBootstrap=FBS_DONTCARE;

	for(i = 0; i < XT_NUM_REGS; i++) {
		reg_list[i].name = esp108_regs[i].name;
		reg_list[i].size = 32;
		reg_list[i].value = calloc(1,4);
		reg_list[i].dirty = 0;
		reg_list[i].valid = 0;
		reg_list[i].type = &esp108_reg_type;
		reg_list[i].arch_info=esp108;
	}

	//Assume running target. If different, the first poll will fix this.
	target->state=TARGET_RUNNING;
	target->debug_reason = DBG_REASON_NOTHALTED;


	return ERROR_OK;
}

static int xtensa_init_target(struct command_context *cmd_ctx, struct target *target)
{
	LOG_DEBUG("%s", __func__);
	struct esp108_common *esp108=(struct esp108_common*)target->arch_info;


	esp108->state = XT_NORMAL; // Assume normal state until we examine

	return ERROR_OK;
}

//Stub
static int xtensa_examine(struct target *target)
{
	target_set_examined(target);
	return ERROR_OK;
}


static int xtensa_poll(struct target *target)
{
	struct esp108_common *esp108=(struct esp108_common*)target->arch_info;
	struct reg *reg_list=esp108->core_cache->reg_list;
	uint8_t pwrstat, pwrstath;
	int res;
	int cmd;
	uint8_t dsr[4], ocdid[4], traxstat[4], traxctl[4];

	//Read reset state
	esp108_queue_pwrstat_readclear(target, &pwrstat);
	//Read again, to see if the state holds...
	esp108_queue_pwrstat_readclear(target, &pwrstath);
	esp108_queue_tdi_idle(target);
	res=jtag_execute_queue();
	if (res!=ERROR_OK) return res;
	if (!(esp108->prevpwrstat&PWRSTAT_DEBUGWASRESET) && pwrstat&PWRSTAT_DEBUGWASRESET) LOG_INFO("%s: Debug controller was reset (pwrstat=0x%02X, after clear 0x%02X).", target->cmd_name, pwrstat, pwrstath);
	if (!(esp108->prevpwrstat&PWRSTAT_COREWASRESET) && pwrstat&PWRSTAT_COREWASRESET) LOG_INFO("%s: Core was reset (pwrstat=0x%02X, after clear 0x%02X).", target->cmd_name, pwrstat, pwrstath);
	esp108->prevpwrstat=pwrstath;

	//Enable JTAG, set reset if needed
	cmd=PWRCTL_DEBUGWAKEUP|PWRCTL_MEMWAKEUP|PWRCTL_COREWAKEUP;
	if (esp108->resetAsserted) cmd|=PWRCTL_CORERESET;
	esp108_queue_pwrctl_set(target, cmd);
	esp108_queue_pwrctl_set(target, cmd|PWRCTL_JTAGDEBUGUSE);
	esp108_queue_tdi_idle(target);
	res=jtag_execute_queue();
	if (res!=ERROR_OK) return res;

	esp108_queue_nexus_reg_write(target, NARADR_DCRSET, OCDDCR_ENABLEOCD);
	esp108_queue_nexus_reg_read(target, NARADR_OCDID, ocdid);
	esp108_queue_nexus_reg_read(target, NARADR_DSR, dsr);
	esp108_queue_nexus_reg_read(target, NARADR_TRAXSTAT, traxstat);
	esp108_queue_nexus_reg_read(target, NARADR_TRAXCTRL, traxctl);
	esp108_queue_tdi_idle(target);
	res=jtag_execute_queue();

	if (res!=ERROR_OK) return res;
//	LOG_INFO("esp8266: ocdid 0x%X dsr 0x%X", intfromchars(ocdid), intfromchars(dsr));

	if (pwrstath&PWRSTAT_COREWASRESET) {
		target->state = TARGET_RESET;
	} else if (intfromchars(dsr)&OCDDSR_STOPPED) {
		if(target->state != TARGET_HALTED) {
			int oldstate=target->state;
			target->state = TARGET_HALTED;

			//LOG_INFO("%s: %s: Target halted (dsr=%08X). Fetching register contents.", target->cmd_name, __FUNCTION__, intfromchars(dsr));
			esp108_fetch_all_regs(target);
			LOG_INFO("%s: Target halted, pc=0x%08X", target->cmd_name, esp108_reg_get(&reg_list[XT_REG_IDX_PC]));

			//Examine why the target was halted
			int cause=esp108_reg_get(&reg_list[XT_REG_IDX_DEBUGCAUSE]);
			target->debug_reason = DBG_REASON_DBGRQ;
			if (cause&DEBUGCAUSE_IC) target->debug_reason = DBG_REASON_SINGLESTEP;
			if (cause&(DEBUGCAUSE_IB|DEBUGCAUSE_BN|DEBUGCAUSE_BI)) target->debug_reason = DBG_REASON_BREAKPOINT;
			if (cause&DEBUGCAUSE_DB) target->debug_reason = DBG_REASON_WATCHPOINT;

			//Call any event callbacks that are applicable
			if(oldstate == TARGET_DEBUG_RUNNING) {
				target_call_event_callbacks(target, TARGET_EVENT_DEBUG_HALTED);
			} else {
				target_call_event_callbacks(target, TARGET_EVENT_HALTED);
			}
		}
	} else {
		target->debug_reason = DBG_REASON_NOTHALTED;
		if (target->state!=TARGET_RUNNING && target->state!=TARGET_DEBUG_RUNNING) {
			//LOG_INFO("%s: Core running again.", target->cmd_name);
			target->state = TARGET_RUNNING;
			target->debug_reason = DBG_REASON_NOTHALTED;
		}
	}


	if (esp108->traceActive) {
//		LOG_INFO("tracestat %x tracectl %x", intfromchars(traxstat), intfromchars(traxctl));
		//Detect if tracing was active but has stopped.
		if ((intfromchars(traxctl)&TRAXCTRL_TREN) && (!(intfromchars(traxstat)&TRAXSTAT_TRACT))) {
			LOG_INFO("Detected end of trace.");
			if (intfromchars(traxstat)&TRAXSTAT_PCMTG) LOG_INFO("%s: Trace stop triggered by PC match", target->cmd_name);
			if (intfromchars(traxstat)&TRAXSTAT_PTITG) LOG_INFO("%s: Trace stop triggered by Processor Trigger Input", target->cmd_name);
			if (intfromchars(traxstat)&TRAXSTAT_CTITG) LOG_INFO("%s: Trace stop triggered by Cross-trigger Input", target->cmd_name);
			esp108->traceActive=0;
		}
	}

	return ERROR_OK;
}


static int xtensa_arch_state(struct target *target)
{
	LOG_DEBUG("%s", __func__);
	return ERROR_OK;
}

COMMAND_HANDLER(esp108_cmd_smpbreak)
{
	struct target *target = get_current_target(CMD_CTX);
	unsigned int i;
	int res;
	uint32_t set=0, clear=0;
	uint8_t dsr[4];

	if (target->state != TARGET_HALTED) {
		command_print(CMD_CTX, "target must be stopped for \"%s\" command", CMD_NAME);
		return ERROR_OK;
	}

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
		esp108_queue_nexus_reg_write(target, NARADR_DCRSET, set);
		esp108_queue_nexus_reg_write(target, NARADR_DCRCLR, clear);
	}
	esp108_queue_nexus_reg_read(target, NARADR_DCRSET, dsr);
	esp108_queue_tdi_idle(target);
	res=jtag_execute_queue();
	if (res==ERROR_OK) {
		i=intfromchars(dsr)&(OCDDCR_BREAKINEN|OCDDCR_BREAKOUTEN|OCDDCR_RUNSTALLINEN|OCDDCR_DEBUGMODEOUTEN);
		command_print(CMD_CTX, "%s: Current bits set:%s%s%s%s%s",
					target->cmd_name,
					(i==0)?" none":"",
					(i&OCDDCR_BREAKINEN)?" BreakIn":"",
					(i&OCDDCR_BREAKOUTEN)?" BreakOut":"",
					(i&OCDDCR_RUNSTALLINEN)?" RunStallIn":"",
					(i&OCDDCR_DEBUGMODEOUTEN)?" DebugModeOut":""
				);
	}
	return res;
}


COMMAND_HANDLER(esp108_cmd_tracestart)
{
	struct target *target = get_current_target(CMD_CTX);
	struct esp108_common *esp108=(struct esp108_common*)target->arch_info;
	uint32_t stoppc=0;
	int stopmask=-1;
	int after=0;
	int afterIsWords=0;
	int res;
	unsigned int i;
	uint8_t traxstat[8];

	//Check current status of trace hardware
	esp108_queue_nexus_reg_read(target, NARADR_TRAXSTAT, traxstat);
	esp108_queue_tdi_idle(target);
	res=jtag_execute_queue();
	if (res!=ERROR_OK) return res;
	if (intfromchars(traxstat)&TRAXSTAT_TRACT) {
		command_print(CMD_CTX, "Tracing is already active. Please stop it first.");
		return ERROR_FAIL;
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

	//Turn off trace unit so we can start a new trace.
	esp108_queue_nexus_reg_write(target, NARADR_TRAXCTRL, 0);
	esp108_queue_tdi_idle(target);
	res=jtag_execute_queue();
	if (res!=ERROR_OK) return res;

	//Set up parameters
	esp108_queue_nexus_reg_write(target, NARADR_TRAXADDR, 0);
	if (stopmask!=-1) {
		esp108_queue_nexus_reg_write(target, NARADR_PCMATCHCTRL, (stopmask<<PCMATCHCTRL_PCML_SHIFT));
		esp108_queue_nexus_reg_write(target, NARADR_TRIGGERPC, stoppc);
	}
	esp108_queue_nexus_reg_write(target, NARADR_DELAYCNT, after);

	//Options are mostly hardcoded for now. ToDo: make this more configurable.
	esp108_queue_nexus_reg_write(target, NARADR_TRAXCTRL,
			TRAXCTRL_TREN | ((stopmask!=-1)?TRAXCTRL_PCMEN:0) | TRAXCTRL_TMEN |
			(afterIsWords?0:TRAXCTRL_CNTU) | (0<<TRAXCTRL_SMPER_SHIFT) | TRAXCTRL_PTOWS );
	esp108_queue_tdi_idle(target);
	res=jtag_execute_queue();
	if (res!=ERROR_OK) return res;

	LOG_INFO("Trace started.\n");
	esp108->traceActive=1;
	return ERROR_OK;
}


COMMAND_HANDLER(esp108_cmd_tracestop)
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

COMMAND_HANDLER(esp108_cmd_tracedump)
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

	if (!intfromchars(traxctl)&TRAXCTRL_TREN) {
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

COMMAND_HANDLER(esp108_cmd_flashbootstrap)
{
	struct target *target = get_current_target(CMD_CTX);
	struct esp108_common *esp108=(struct esp108_common*)target->arch_info;
	int state=-1;
	if (CMD_ARGC != 1) {
		const char *st;
		if (esp108->flashBootstrap==FBS_DONTCARE) st="Don't care";
		if (esp108->flashBootstrap==FBS_TMSLOW) st="Low (3.3V)";
		if (esp108->flashBootstrap==FBS_TMSHIGH) st="High (1.8V)";
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

	esp108->flashBootstrap=state;

	return ERROR_OK;
}


static const struct command_registration esp108_any_command_handlers[] = {
	{
		.name = "tracestart",
		.handler = esp108_cmd_tracestart,
		.mode = COMMAND_ANY,
		.help = "Tracing: Set up and start a trace. Optionally set stop trigger address and amount of data captured after.",
		.usage = "[pc <pcval>/[maskbitcount]] [after <n> [ins|words]]",
	},
	{
		.name = "tracestop",
		.handler = esp108_cmd_tracestop,
		.mode = COMMAND_ANY,
		.help = "Tracing: Stop current trace as started by the tracestart command",
		.usage = "",
	},
	{
		.name = "tracedump",
		.handler = esp108_cmd_tracedump,
		.mode = COMMAND_ANY,
		.help = "Tracing: Dump trace memory to a file",
		.usage = "outfile",
	},
    {
        .name = "apptrace",
        .handler = esp108_cmd_apptrace,
        .mode = COMMAND_ANY,
        .help = "Tracing: app tracing control and status. Starts, stops or queries tracing process status.",
        .usage = "[start outfile [trace_size [stop_tmo [skip_size [poll_period [wait4halt]]]]]] | [stop] | [status] | [dump outfile]",
    },
	{
		.name = "smpbreak",
		.handler = esp108_cmd_smpbreak,
		.mode = COMMAND_ANY,
		.help = "Set the way the CPU chains OCD breaks",
		.usage = "[none|breakinout|runstall] | [BreakIn] [BreakOut] [RunStallIn] [DebugModeOut]",
	},
	{
		.name = "flashbootstrap",
		.handler = esp108_cmd_flashbootstrap,
		.mode = COMMAND_ANY,
		.help = "Set the idle state of the TMS pin, which at reset also is the voltage selector for the flash chip.",
		.usage = "none|1.8|3.3|high|low",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration esp108_command_handlers[] = {
	{
		.name = "esp108",
		.mode = COMMAND_ANY,
		.help = "ESP108 command group",
		.usage = "",
		.chain = esp108_any_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};


/** Holds methods for Xtensa targets. */
struct target_type esp108_target = {
	.name = "esp108",

	.poll = xtensa_poll,
	.arch_state = xtensa_arch_state,

	.halt = xtensa_halt,
	.resume = xtensa_resume,
	.step = xtensa_step,

	.assert_reset = xtensa_assert_reset_full,
	.deassert_reset = xtensa_deassert_reset,

	.read_memory = xtensa_read_memory,
	.write_memory = xtensa_write_memory,

	.read_buffer = xtensa_read_buffer,
	.write_buffer = xtensa_write_buffer,

	.get_gdb_reg_list = xtensa_get_gdb_reg_list,

	.add_breakpoint = xtensa_add_breakpoint,
	.remove_breakpoint = xtensa_remove_breakpoint,

	.add_watchpoint = xtensa_add_watchpoint,
	.remove_watchpoint = xtensa_remove_watchpoint,

	.target_create = xtensa_target_create,
	.init_target = xtensa_init_target,
	.examine = xtensa_examine,

	.commands = esp108_command_handlers,
};

