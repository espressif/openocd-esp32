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
the instruction. DIR1-DIRn are for longer instructions, oif which there don't
appear to be any the ESP108.
*/

#define TAPINS_PWRCTL	0x08
#define TAPINS_PWRSTAT	0x09
#define TAPINS_NARSEL	0x1C
#define TAPINS_IDCODE	0x1E
#define TAPINS_BYPASS	0x1F

#define TAPINS_PWRCTL_LEN		8
#define TAPINS_PWRSTAT_LEN		8
#define TAPINS_NARSEL_ADRLEN	8
#define TAPINS_NARSEL_DATALEN	32
#define TAPINS_IDCODE_LEN		32
#define TAPINS_BYPASS_LEN		1


/* 
 From the manual:
 To properly use Debug registers through JTAG, software must ensure that:
 - Tap is out of reset
 - Xtensa Debug Module is out of reset
 - Other bits of PWRCTL are set to their desired values, and finally
 - JtagDebugUse transitions from 0 to 1
 The bit must continue to be 1 in order for JTAG accesses to the Debug 
 Module to happen correctly. When it is set, any write to this bit clears it.
 Either don't access it, or re-write it to 1 so JTAG accesses continue. 
*/
#define PWRCTL_JTAGDEBUGUSE	(1<<7)
#define PWRCTL_DEBUGRESET	(1<<6)
#define PWRCTL_CORERESET	(1<<4)
#define PWRCTL_DEBUGWAKEUP	(1<<2)
#define PWRCTL_MEMWAKEUP	(1<<1)
#define PWRCTL_COREWAKEUP	(1<<0)

#define PWRSTAT_DEBUGWASRESET	(1<<6)
#define PWRSTAT_COREWASRESET	(1<<4)
#define PWRSTAT_CORESTILLNEEDED	(1<<3)
#define PWRSTAT_DEBUGDOMAINON	(1<<2)
#define PWRSTAT_MEMDOMAINON		(1<<1)
#define PWRSTAT_COREDOMAINON	(1<<0)


// *** NAR addresses ***
//TRAX registers
#define NARADR_TRAXID		0x00
#define NARADR_TRAXCTRL		0x01
#define NARADR_TRAXSTAT		0x02
#define NARADR_TRAXDATA		0x03
#define NARADR_TRAXADDR		0x04
#define NARADR_TRIGGERPC	0x05
#define NARADR_PCMATCHCTRL	0x06
#define NARADR_DELAYCNT		0x07
#define NARADR_MEMADDRSTART	0x08
#define NARADR_MEMADDREND	0x09
//Performance monitor registers
#define NARADR_PMG			0x20
#define NARADR_INTPC		0x24
#define NARADR_PM0			0x28
//...
#define NARADR_PM7			0x2F
#define NARADR_PMCTRL0		0x30
//...
#define NARADR_PMCTRL7		0x37
#define NARADR_PMSTAT0		0x38
//...
#define NARADR_PMSTAT7		0x3F
//OCD registers
#define NARADR_OCDID		0x40
#define NARADR_DCRCLR		0x42
#define NARADR_DCRSET		0x43
#define NARADR_DSR			0x44
#define NARADR_DDR			0x45
#define NARADR_DDREXEC		0x46
#define NARADR_DIR0EXEC		0x47
#define NARADR_DIR0			0x48
#define NARADR_DIR1			0x49
//...
#define NARADR_DIR7			0x4F
//Misc registers
#define NARADR_PWRCTL		0x58
#define NARADR_PWRSTAT		0x69
#define NARADR_ERISTAT		0x5A
//CoreSight registers
#define NARADR_ITCTRL		0x60
#define NARADR_CLAIMSET		0x68
#define NARADR_CLAIMCLR		0x69
#define NARADR_LOCKACCESS	0x6c
#define NARADR_LOCKSTATUS	0x6d
#define NARADR_AUTHSTATUS	0x6e
#define NARADR_DEVID		0x72
#define NARADR_DEVTYPE		0x73
#define NARADR_PERID4		0x74
//...
#define NARADR_PERID7		0x77
#define NARADR_PERID0		0x78
//...
#define NARADR_PERID3		0x7b
#define NARADR_COMPID0		0x7c
//...
#define NARADR_COMPID3		0x7f

//OCD registers, bit definitions
#define OCDDCR_ENABLEOCD		(1<<0)
#define OCDDCR_DEBUGINTERRUPT	(1<<1)
#define OCDDCR_INTERRUPTALLCONDS	(1<<2)
#define OCDDCR_BREAKINEN		(1<<16)
#define OCDDCR_BREAKOUTEN		(1<<17)
#define OCDDCR_DEBUGSWACTIVE	(1<<20)
#define OCDDCR_RUNSTALLINEN		(1<<21)
#define OCDDCR_DEBUGMODEOUTEN	(1<<22)
#define OCDDCR_BREAKOUTITO		(1<<24)
#define OCDDCR_BREAKACKITO		(1<<25)

#define OCDDSR_EXECDONE			(1<<0)
#define OCDDSR_EXECEXCEPTION	(1<<1)
#define OCDDSR_EXECBUSY			(1<<2)
#define OCDDSR_EXECOVERRUN		(1<<3)
#define OCDDSR_STOPPED			(1<<4)
#define OCDDSR_COREWROTEDDR		(1<<10)
#define OCDDSR_COREREADDDR		(1<<11)
#define OCDDSR_HOSTWROTEDDR		(1<<14)
#define OCDDSR_HOSTREADDDR		(1<<15)
#define OCDDSR_DEBUGPENDBREAK	(1<<16)
#define OCDDSR_DEBUGPENDHOST	(1<<17)
#define OCDDSR_DEBUGPENDTRAX	(1<<18)
#define OCDDSR_DEBUGINTBREAK	(1<<20)
#define OCDDSR_DEBUGINTHOST		(1<<21)
#define OCDDSR_DEBUGINTTRAX		(1<<22)
#define OCDDSR_RUNSTALLTOGGLE	(1<<23)
#define OCDDSR_RUNSTALLSAMPLE	(1<<24)
#define OCDDSR_BREACKOUTACKITI	(1<<25)
#define OCDDSR_BREAKINITI		(1<<26)


#define DEBUGCAUSE_IC			(1<<0)
#define DEBUGCAUSE_IB			(1<<1)
#define DEBUGCAUSE_DB			(1<<2)
#define DEBUGCAUSE_BI			(1<<3)
#define DEBUGCAUSE_BN			(1<<4)
#define DEBUGCAUSE_DI			(1<<5)

#define XT_INS_NUM_BITS 24
#define XT_DEBUGLEVEL    6 /* XCHAL_DEBUGLEVEL in xtensa-config.h */
#define XT_NUM_BREAKPOINTS 2
#define XT_NUM_WATCHPOINTS 2

//Xtensa register list taken from gdb/gdb/xtensa-config.c
//gdb wants the registers in the order gdb/regformats/reg-xtensa.dat describes
//them. The enum and esp108_regs structs should be in the same order.

#define XT_NUM_REGS (151)

enum xtensa_reg_idx {
	XT_REG_IDX_PC=0,
	XT_REG_IDX_AR0,
	XT_REG_IDX_AR1,
	XT_REG_IDX_AR2,
	XT_REG_IDX_AR3,
	XT_REG_IDX_AR4,
	XT_REG_IDX_AR5,
	XT_REG_IDX_AR6,
	XT_REG_IDX_AR7,
	XT_REG_IDX_AR8,
	XT_REG_IDX_AR9,
	XT_REG_IDX_AR10,
	XT_REG_IDX_AR11,
	XT_REG_IDX_AR12,
	XT_REG_IDX_AR13,
	XT_REG_IDX_AR14,
	XT_REG_IDX_AR15,
	XT_REG_IDX_AR16,
	XT_REG_IDX_AR17,
	XT_REG_IDX_AR18,
	XT_REG_IDX_AR19,
	XT_REG_IDX_AR20,
	XT_REG_IDX_AR21,
	XT_REG_IDX_AR22,
	XT_REG_IDX_AR23,
	XT_REG_IDX_AR24,
	XT_REG_IDX_AR25,
	XT_REG_IDX_AR26,
	XT_REG_IDX_AR27,
	XT_REG_IDX_AR28,
	XT_REG_IDX_AR29,
	XT_REG_IDX_AR30,
	XT_REG_IDX_AR31,
	XT_REG_IDX_AR32,
	XT_REG_IDX_AR33,
	XT_REG_IDX_AR34,
	XT_REG_IDX_AR35,
	XT_REG_IDX_AR36,
	XT_REG_IDX_AR37,
	XT_REG_IDX_AR38,
	XT_REG_IDX_AR39,
	XT_REG_IDX_AR40,
	XT_REG_IDX_AR41,
	XT_REG_IDX_AR42,
	XT_REG_IDX_AR43,
	XT_REG_IDX_AR44,
	XT_REG_IDX_AR45,
	XT_REG_IDX_AR46,
	XT_REG_IDX_AR47,
	XT_REG_IDX_AR48,
	XT_REG_IDX_AR49,
	XT_REG_IDX_AR50,
	XT_REG_IDX_AR51,
	XT_REG_IDX_AR52,
	XT_REG_IDX_AR53,
	XT_REG_IDX_AR54,
	XT_REG_IDX_AR55,
	XT_REG_IDX_AR56,
	XT_REG_IDX_AR57,
	XT_REG_IDX_AR58,
	XT_REG_IDX_AR59,
	XT_REG_IDX_AR60,
	XT_REG_IDX_AR61,
	XT_REG_IDX_AR62,
	XT_REG_IDX_AR63,
	XT_REG_IDX_LBEG,
	XT_REG_IDX_LEND,
	XT_REG_IDX_LCOUNT,
	XT_REG_IDX_SAR,
	XT_REG_IDX_WINDOWBASE,
	XT_REG_IDX_WINDOWSTART,
	XT_REG_IDX_CONFIGID0,
	XT_REG_IDX_CONFIGID1,
	XT_REG_IDX_PS,
	XT_REG_IDX_THREADPTR,
	XT_REG_IDX_BR,
	XT_REG_IDX_SCOMPARE1,
	XT_REG_IDX_ACCLO,
	XT_REG_IDX_ACCHI,
	XT_REG_IDX_M0,
	XT_REG_IDX_M1,
	XT_REG_IDX_M2,
	XT_REG_IDX_M3,
	XT_REG_IDX_EXPSTATE,
	XT_REG_IDX_MMID,
	XT_REG_IDX_IBREAKENABLE,
	XT_REG_IDX_MEMCTL,
	XT_REG_IDX_ATOMCTL,
	XT_REG_IDX_DDR,
	XT_REG_IDX_IBREAKA0,
	XT_REG_IDX_IBREAKA1,
	XT_REG_IDX_DBREAKA0,
	XT_REG_IDX_DBREAKA1,
	XT_REG_IDX_DBREAKC0,
	XT_REG_IDX_DBREAKC1,
	XT_REG_IDX_EPC1,
	XT_REG_IDX_EPC2,
	XT_REG_IDX_EPC3,
	XT_REG_IDX_EPC4,
	XT_REG_IDX_EPC5,
	XT_REG_IDX_EPC6,
	XT_REG_IDX_EPC7,
	XT_REG_IDX_DEPC,
	XT_REG_IDX_EPS2,
	XT_REG_IDX_EPS3,
	XT_REG_IDX_EPS4,
	XT_REG_IDX_EPS5,
	XT_REG_IDX_EPS6,
	XT_REG_IDX_EPS7,
	XT_REG_IDX_EXCSAVE1,
	XT_REG_IDX_EXCSAVE2,
	XT_REG_IDX_EXCSAVE3,
	XT_REG_IDX_EXCSAVE4,
	XT_REG_IDX_EXCSAVE5,
	XT_REG_IDX_EXCSAVE6,
	XT_REG_IDX_EXCSAVE7,
	XT_REG_IDX_INTERRUPT,
	XT_REG_IDX_INTSET,
	XT_REG_IDX_INTCLEAR,
	XT_REG_IDX_INTENABLE,
	XT_REG_IDX_VECBASE,
	XT_REG_IDX_EXCCAUSE,
	XT_REG_IDX_DEBUGCAUSE,
	XT_REG_IDX_CCOUNT,
	XT_REG_IDX_PRID,
	XT_REG_IDX_ICOUNT,
	XT_REG_IDX_ICOUNTLEVEL,
	XT_REG_IDX_EXCVADDR,
	XT_REG_IDX_CCOMPARE0,
	XT_REG_IDX_CCOMPARE1,
	XT_REG_IDX_CCOMPARE2,
	XT_REG_IDX_MISC0,
	XT_REG_IDX_MISC1,
	XT_REG_IDX_MISC2,
	XT_REG_IDX_MISC3,
	XT_REG_IDX_A0,
	XT_REG_IDX_A1,
	XT_REG_IDX_A2,
	XT_REG_IDX_A3,
	XT_REG_IDX_A4,
	XT_REG_IDX_A5,
	XT_REG_IDX_A6,
	XT_REG_IDX_A7,
	XT_REG_IDX_A8,
	XT_REG_IDX_A9,
	XT_REG_IDX_A10,
	XT_REG_IDX_A11,
	XT_REG_IDX_A12,
	XT_REG_IDX_A13,
	XT_REG_IDX_A14,
	XT_REG_IDX_A15,
};

enum esp108_reg_t {
	XT_REG_GENERAL = 0,		//General-purpose register; part of the windowed register set
	XT_REG_USER = 1,		//User register, needs RUR to read
	XT_REG_SPECIAL = 2,		//Special register, needs RSR to read
	XT_REG_DEBUG = 3,		//Register used for the debug interface. Don't mess with this.
	XT_REG_RELGEN = 4,		//Relative general address. Points to the absolute addresses plus the window index
};

enum esp108_regflags_t {
	XT_REGF_NOREAD = 0x01,	//Register is write-only
};

struct esp108_reg_desc {
	const char *name;
	int reg_num; /* ISA register num (meaning depends on register type) */
	enum esp108_reg_t type;
	enum esp108_regflags_t flags;
};

static const struct esp108_reg_desc esp108_regs[XT_NUM_REGS] = {
	{ "pc",					176+XT_DEBUGLEVEL, XT_REG_SPECIAL, 0 }, //actually epc[debuglevel]
	{ "ar0",				0x00, XT_REG_GENERAL, 0 }, 
	{ "ar1",				0x01, XT_REG_GENERAL, 0 }, 
	{ "ar2",				0x02, XT_REG_GENERAL, 0 }, 
	{ "ar3",				0x03, XT_REG_GENERAL, 0 }, 
	{ "ar4",				0x04, XT_REG_GENERAL, 0 }, 
	{ "ar5",				0x05, XT_REG_GENERAL, 0 }, 
	{ "ar6",				0x06, XT_REG_GENERAL, 0 }, 
	{ "ar7",				0x07, XT_REG_GENERAL, 0 }, 
	{ "ar8",				0x08, XT_REG_GENERAL, 0 }, 
	{ "ar9",				0x09, XT_REG_GENERAL, 0 }, 
	{ "ar10",				0x0A, XT_REG_GENERAL, 0 }, 
	{ "ar11",				0x0B, XT_REG_GENERAL, 0 }, 
	{ "ar12",				0x0C, XT_REG_GENERAL, 0 }, 
	{ "ar13",				0x0D, XT_REG_GENERAL, 0 }, 
	{ "ar14",				0x0E, XT_REG_GENERAL, 0 }, 
	{ "ar15",				0x0F, XT_REG_GENERAL, 0 }, 
	{ "ar16",				0x10, XT_REG_GENERAL, 0 }, 
	{ "ar17",				0x11, XT_REG_GENERAL, 0 }, 
	{ "ar18",				0x12, XT_REG_GENERAL, 0 }, 
	{ "ar19",				0x13, XT_REG_GENERAL, 0 }, 
	{ "ar20",				0x14, XT_REG_GENERAL, 0 }, 
	{ "ar21",				0x15, XT_REG_GENERAL, 0 }, 
	{ "ar22",				0x16, XT_REG_GENERAL, 0 }, 
	{ "ar23",				0x17, XT_REG_GENERAL, 0 }, 
	{ "ar24",				0x18, XT_REG_GENERAL, 0 }, 
	{ "ar25",				0x19, XT_REG_GENERAL, 0 }, 
	{ "ar26",				0x1A, XT_REG_GENERAL, 0 }, 
	{ "ar27",				0x1B, XT_REG_GENERAL, 0 }, 
	{ "ar28",				0x1C, XT_REG_GENERAL, 0 }, 
	{ "ar29",				0x1D, XT_REG_GENERAL, 0 }, 
	{ "ar30",				0x1E, XT_REG_GENERAL, 0 }, 
	{ "ar31",				0x1F, XT_REG_GENERAL, 0 }, 
	{ "ar32",				0x20, XT_REG_GENERAL, 0 }, 
	{ "ar33",				0x21, XT_REG_GENERAL, 0 }, 
	{ "ar34",				0x22, XT_REG_GENERAL, 0 }, 
	{ "ar35",				0x23, XT_REG_GENERAL, 0 }, 
	{ "ar36",				0x24, XT_REG_GENERAL, 0 }, 
	{ "ar37",				0x25, XT_REG_GENERAL, 0 }, 
	{ "ar38",				0x26, XT_REG_GENERAL, 0 }, 
	{ "ar39",				0x27, XT_REG_GENERAL, 0 }, 
	{ "ar40",				0x28, XT_REG_GENERAL, 0 }, 
	{ "ar41",				0x29, XT_REG_GENERAL, 0 }, 
	{ "ar42",				0x2A, XT_REG_GENERAL, 0 }, 
	{ "ar43",				0x2B, XT_REG_GENERAL, 0 }, 
	{ "ar44",				0x2C, XT_REG_GENERAL, 0 }, 
	{ "ar45",				0x2D, XT_REG_GENERAL, 0 }, 
	{ "ar46",				0x2E, XT_REG_GENERAL, 0 }, 
	{ "ar47",				0x2F, XT_REG_GENERAL, 0 }, 
	{ "ar48",				0x30, XT_REG_GENERAL, 0 }, 
	{ "ar49",				0x31, XT_REG_GENERAL, 0 }, 
	{ "ar50",				0x32, XT_REG_GENERAL, 0 }, 
	{ "ar51",				0x33, XT_REG_GENERAL, 0 }, 
	{ "ar52",				0x34, XT_REG_GENERAL, 0 }, 
	{ "ar53",				0x35, XT_REG_GENERAL, 0 }, 
	{ "ar54",				0x36, XT_REG_GENERAL, 0 }, 
	{ "ar55",				0x37, XT_REG_GENERAL, 0 }, 
	{ "ar56",				0x38, XT_REG_GENERAL, 0 }, 
	{ "ar57",				0x39, XT_REG_GENERAL, 0 }, 
	{ "ar58",				0x3A, XT_REG_GENERAL, 0 }, 
	{ "ar59",				0x3B, XT_REG_GENERAL, 0 }, 
	{ "ar60",				0x3C, XT_REG_GENERAL, 0 }, 
	{ "ar61",				0x3D, XT_REG_GENERAL, 0 }, 
	{ "ar62",				0x3E, XT_REG_GENERAL, 0 }, 
	{ "ar63",				0x3F, XT_REG_GENERAL, 0 }, 
	{ "lbeg",				0x00, XT_REG_SPECIAL, 0 }, 
	{ "lend",				0x01, XT_REG_SPECIAL, 0 }, 
	{ "lcount",				0x02, XT_REG_SPECIAL, 0 }, 
	{ "sar",				0x03, XT_REG_SPECIAL, 0 }, 
	{ "windowbase",			0x48, XT_REG_SPECIAL, 0 }, 
	{ "windowstart",		0x49, XT_REG_SPECIAL, 0 }, 
	{ "configid0",			0xB0, XT_REG_SPECIAL, 0 }, 
	{ "configid1",			0xD0, XT_REG_SPECIAL, 0 }, 
	{ "ps",					0xC6, XT_REG_SPECIAL, 0 }, //actually EPS[debuglevel]
	{ "threadptr",			0xE7, XT_REG_USER, 0 }, 
	{ "br",					0x04, XT_REG_SPECIAL, 0 }, 
	{ "scompare1",			0x0C, XT_REG_SPECIAL, 0 }, 
	{ "acclo",				0x10, XT_REG_SPECIAL, 0 }, 
	{ "acchi",				0x11, XT_REG_SPECIAL, 0 }, 
	{ "m0",					0x20, XT_REG_SPECIAL, 0 }, 
	{ "m1",					0x21, XT_REG_SPECIAL, 0 }, 
	{ "m2",					0x22, XT_REG_SPECIAL, 0 }, 
	{ "m3",					0x23, XT_REG_SPECIAL, 0 }, 
	{ "expstate",			0xE6, XT_REG_USER, 0 },
	{ "mmid",				0x59, XT_REG_SPECIAL, XT_REGF_NOREAD }, 
	{ "ibreakenable",				0x60, XT_REG_SPECIAL, 0 }, 
	{ "memctl",				0x61, XT_REG_SPECIAL, 0 }, 
	{ "atomctl",				0x63, XT_REG_SPECIAL, 0 }, 
	{ "ddr",				0x68, XT_REG_DEBUG, XT_REGF_NOREAD }, 
	{ "ibreaka0",				0x80, XT_REG_SPECIAL, 0 }, 
	{ "ibreaka1",				0x81, XT_REG_SPECIAL, 0 }, 
	{ "dbreaka0",				0x90, XT_REG_SPECIAL, 0 }, 
	{ "dbreaka1",				0x91, XT_REG_SPECIAL, 0 }, 
	{ "dbreakc0",				0xA0, XT_REG_SPECIAL, 0 }, 
	{ "dbreakc1",				0xA1, XT_REG_SPECIAL, 0 }, 
	{ "epc1",				0xB1, XT_REG_SPECIAL, 0 }, 
	{ "epc2",				0xB2, XT_REG_SPECIAL, 0 }, 
	{ "epc3",				0xB3, XT_REG_SPECIAL, 0 }, 
	{ "epc4",				0xB4, XT_REG_SPECIAL, 0 }, 
	{ "epc5",				0xB5, XT_REG_SPECIAL, 0 }, 
	{ "epc6",				0xB6, XT_REG_SPECIAL, 0 }, 
	{ "epc7",				0xB7, XT_REG_SPECIAL, 0 }, 
	{ "depc",				0xC0, XT_REG_SPECIAL, 0 }, 
	{ "eps2",				0xC2, XT_REG_SPECIAL, 0 }, 
	{ "eps3",				0xC3, XT_REG_SPECIAL, 0 }, 
	{ "eps4",				0xC4, XT_REG_SPECIAL, 0 }, 
	{ "eps5",				0xC5, XT_REG_SPECIAL, 0 }, 
	{ "eps6",				0xC6, XT_REG_SPECIAL, 0 }, 
	{ "eps7",				0xC7, XT_REG_SPECIAL, 0 }, 
	{ "excsave1",				0xD1, XT_REG_SPECIAL, 0 }, 
	{ "excsave2",				0xD2, XT_REG_SPECIAL, 0 }, 
	{ "excsave3",				0xD3, XT_REG_SPECIAL, 0 }, 
	{ "excsave4",				0xD4, XT_REG_SPECIAL, 0 }, 
	{ "excsave5",				0xD5, XT_REG_SPECIAL, 0 }, 
	{ "excsave6",				0xD6, XT_REG_SPECIAL, 0 }, 
	{ "excsave7",				0xD7, XT_REG_SPECIAL, 0 }, 
	{ "interrupt",				0xE2, XT_REG_SPECIAL, 0 }, 
	{ "intset",				0xE2, XT_REG_SPECIAL, XT_REGF_NOREAD }, 
	{ "intclear",				0xE3, XT_REG_SPECIAL, XT_REGF_NOREAD }, 
	{ "intenable",				0xE4, XT_REG_SPECIAL, 0 }, 
	{ "vecbase",				0xE7, XT_REG_SPECIAL, 0 }, 
	{ "exccause",				0xE8, XT_REG_SPECIAL, 0 }, 
	{ "debugcause",				0xE9, XT_REG_SPECIAL, 0 }, 
	{ "ccount",				0xEA, XT_REG_SPECIAL, 0 }, 
	{ "prid",				0xEB, XT_REG_SPECIAL, 0 }, 
	{ "icount",				0xEC, XT_REG_SPECIAL, 0 }, 
	{ "icountlevel",				0xED, XT_REG_SPECIAL, 0 }, 
	{ "excvaddr",				0xEE, XT_REG_SPECIAL, 0 }, 
	{ "ccompare0",				0xF0, XT_REG_SPECIAL, 0 }, 
	{ "ccompare1",				0xF1, XT_REG_SPECIAL, 0 }, 
	{ "ccompare2",				0xF2, XT_REG_SPECIAL, 0 }, 
	{ "misc0",				0xF4, XT_REG_SPECIAL, 0 }, 
	{ "misc1",				0xF5, XT_REG_SPECIAL, 0 }, 
	{ "misc2",				0xF6, XT_REG_SPECIAL, 0 }, 
	{ "misc3",				0xF7, XT_REG_SPECIAL, 0 }, 
	{ "a0",					XT_REG_IDX_AR0, XT_REG_RELGEN, 0 },
	{ "a1",					XT_REG_IDX_AR1, XT_REG_RELGEN, 0 },
	{ "a2",					XT_REG_IDX_AR2, XT_REG_RELGEN, 0 },
	{ "a3",					XT_REG_IDX_AR3, XT_REG_RELGEN, 0 },
	{ "a4",					XT_REG_IDX_AR4, XT_REG_RELGEN, 0 },
	{ "a5",					XT_REG_IDX_AR5, XT_REG_RELGEN, 0 },
	{ "a6",					XT_REG_IDX_AR6, XT_REG_RELGEN, 0 },
	{ "a7",					XT_REG_IDX_AR7, XT_REG_RELGEN, 0 },
	{ "a8",					XT_REG_IDX_AR8, XT_REG_RELGEN, 0 },
	{ "a9",					XT_REG_IDX_AR9, XT_REG_RELGEN, 0 },
	{ "a10",				XT_REG_IDX_AR0, XT_REG_RELGEN, 0 },
	{ "a11",				XT_REG_IDX_AR11, XT_REG_RELGEN, 0 },
	{ "a12",				XT_REG_IDX_AR12, XT_REG_RELGEN, 0 },
	{ "a13",				XT_REG_IDX_AR13, XT_REG_RELGEN, 0 },
	{ "a14",				XT_REG_IDX_AR14, XT_REG_RELGEN, 0 },
	{ "a15",				XT_REG_IDX_AR15, XT_REG_RELGEN, 0 },
};

#define _XT_INS_FORMAT_RSR(OPCODE,SR,T) (OPCODE			\
					 | ((SR & 0xFF) << 8)	\
					 | ((T & 0x0F) << 4))

#define _XT_INS_FORMAT_RRR(OPCODE,ST,R) (OPCODE			\
					 | ((ST & 0xFF) << 4)	\
					 | ((R & 0x0F) << 12))

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

//Same thing for A3
#define XT_REG_A3         (esp108_regs[XT_REG_IDX_A3].reg_num)


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


static int xtensa_step(struct target *target,
	int current,
	uint32_t address,
	int handle_breakpoints);


//Set the PWRCTL TAP register to a value
static void esp108_queue_pwrctl_set(struct target *target, uint8_t value) 
{
	const uint8_t pwrctlIns=TAPINS_PWRCTL;
	jtag_add_plain_ir_scan(target->tap->ir_length, &pwrctlIns, NULL, TAP_IDLE);
	jtag_add_plain_dr_scan(TAPINS_PWRCTL_LEN, &value, NULL, TAP_IDLE);
}

//Read the PWRSTAT TAP register and clear the XWASRESET bits.
static void esp108_queue_pwrstat_readclear(struct target *target, uint8_t *value) 
{
	const uint8_t pwrctlIns=TAPINS_PWRSTAT;
	const uint8_t pwrstatClr=PWRSTAT_DEBUGWASRESET|PWRSTAT_COREWASRESET;
	jtag_add_plain_ir_scan(target->tap->ir_length, &pwrctlIns, NULL, TAP_IDLE);
	jtag_add_plain_dr_scan(TAPINS_PWRCTL_LEN, &pwrstatClr, value, TAP_IDLE);
}


static void esp108_queue_nexus_reg_write(struct target *target, const uint8_t reg, const uint32_t value) 
{
	const uint8_t narselIns=TAPINS_NARSEL;
	uint8_t regdata=(reg<<1)|1;
	uint8_t valdata[]={value, value>>8, value>>16, value>>24};
	jtag_add_plain_ir_scan(target->tap->ir_length, &narselIns, NULL, TAP_IDLE);
	jtag_add_plain_dr_scan(TAPINS_NARSEL_ADRLEN, &regdata, NULL, TAP_IDLE);
	jtag_add_plain_dr_scan(TAPINS_NARSEL_DATALEN, valdata, NULL, TAP_IDLE);
}

static void esp108_queue_nexus_reg_read(struct target *target, const uint8_t reg, uint8_t *value) 
{
	const uint8_t narselIns=TAPINS_NARSEL;
	uint8_t regdata=(reg<<1)|0;
	uint8_t dummy[4]={0,0,0,0};
	jtag_add_plain_ir_scan(target->tap->ir_length, &narselIns, NULL, TAP_IDLE);
	jtag_add_plain_dr_scan(TAPINS_NARSEL_ADRLEN, &regdata, NULL, TAP_IDLE);
	jtag_add_plain_dr_scan(TAPINS_NARSEL_DATALEN, dummy, value, TAP_IDLE);
}

static void esp108_queue_exec_ins(struct target *target, int32_t ins)
{
	esp108_queue_nexus_reg_write(target, NARADR_DIR0EXEC, ins);
}

//Small helper function to convert the char arrays that result from a jtag
//call to a well-formatted uint32_t.
static uint32_t intfromchars(uint8_t *c) 
{
	return c[0]+(c[1]<<8)+(c[2]<<16)+(c[3]<<24);
}

//Utility function: check DSR for any weirdness and report.
#define esp108_checkdsr(target) esp108_do_checkdsr(target, __FUNCTION__, __LINE__)
static int esp108_do_checkdsr(struct target *target, const char *function, const int line)
{
	uint8_t dsr[4];
	int res;
	int needclear=0;
	esp108_queue_nexus_reg_read(target, NARADR_DSR, dsr);
	res=jtag_execute_queue();
	if (res!=ERROR_OK) {
		LOG_ERROR("%s (line %d): reading DSR failed!", function, line);
		return ERROR_FAIL;
	}
	if (intfromchars(dsr)&OCDDSR_EXECBUSY) {
		LOG_ERROR("%s (line %d): DSR (%08X) indicates target still busy!", function, line, intfromchars(dsr));
		needclear=1;
	}
	if (intfromchars(dsr)&OCDDSR_EXECEXCEPTION) {
		LOG_ERROR("%s (line %d): DSR (%08X) indicates DIR instruction generated an exception!", function, line, intfromchars(dsr));
		needclear=1;
	}
	if (intfromchars(dsr)&OCDDSR_EXECOVERRUN) {
		LOG_ERROR("%s (line %d): DSR (%08X) indicates DIR instruction generated an overrun!", function, line, intfromchars(dsr));
		needclear=1;
	}
	if (needclear) {
		esp108_queue_nexus_reg_write(target, NARADR_DSR, OCDDSR_EXECEXCEPTION|OCDDSR_EXECOVERRUN);
		res=jtag_execute_queue();
		if (res!=ERROR_OK) {
			LOG_ERROR("%s (line %d): clearing DSR failed!", function, line);
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
static enum xtensa_reg_idx windowbase_offset_to_canonical(const enum xtensa_reg_idx reg, const int windowbase) {
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

static enum xtensa_reg_idx canonical_to_windowbase_offset(const enum xtensa_reg_idx reg, const int windowbase) {
	return windowbase_offset_to_canonical(reg, -windowbase);
}


static int esp108_fetch_all_regs(struct target *target)
{
	int i, j;
	int res;
	uint32_t regval;
	uint32_t windowbase;
	struct esp108_common *esp108=(struct esp108_common*)target->arch_info;
	struct reg *reg_list=esp108->core_cache->reg_list;
	uint8_t regvals[XT_NUM_REGS][4];
	
	//Assume the CPU has just halted. We now want to fill the register cache with all the 
	//register contents GDB needs. For speed, we pipeline all the read operations, execute them
	//in one go, then sort everything out from the regvals variable.

	//Start out with A0-A63; we can reach those immediately. Grab them per 16 registers.
	for (j=0; j<64; j+=16) {
		//Grab the 16 registers we can see
		for (i=0; i<16; i++) {
			esp108_queue_exec_ins(target, XT_INS_WSR(XT_SR_DDR, esp108_regs[XT_REG_IDX_AR0+i].reg_num));
			esp108_queue_nexus_reg_read(target, NARADR_DDR, regvals[XT_REG_IDX_AR0+i+j]);
		}
		//Now rotate the window so we'll see the next 16 registers. The final rotate will wraparound, 
		//leaving us in the state we were.
		esp108_queue_exec_ins(target, XT_INS_ROTW(4));
	}

	//We're now free to use any of A0-A15 as scratch registers
	//Grab the SFRs and user registers first. We use A3 as a scratch register.
	for (i=0; i<XT_NUM_REGS; i++) {
		if ((!(esp108_regs[i].flags&XT_REGF_NOREAD)) && (esp108_regs[i].type==XT_REG_SPECIAL || esp108_regs[i].type==XT_REG_USER)) {
			if (esp108_regs[i].type==XT_REG_USER) {
				esp108_queue_exec_ins(target, XT_INS_RUR(esp108_regs[i].reg_num, XT_REG_A3));
			} else { //SFR
				esp108_queue_exec_ins(target, XT_INS_RSR(esp108_regs[i].reg_num, XT_REG_A3));
			}
			esp108_queue_exec_ins(target, XT_INS_WSR(XT_SR_DDR, XT_REG_A3));
			esp108_queue_nexus_reg_read(target, NARADR_DDR, regvals[i]);
		}
	}



	//Ok, send the whole mess to the CPU.
	res=jtag_execute_queue();
	if (res!=ERROR_OK) return res;
	esp108_checkdsr(target);

	//We need the windowbase to decode the general addresses.
	windowbase=intfromchars(regvals[XT_REG_IDX_WINDOWBASE]);
	//Decode the result and update the cache.
	for (i=0; i<XT_NUM_REGS; i++) {
		if (!(esp108_regs[i].flags&XT_REGF_NOREAD)) {
			reg_list[i].valid=1;
			reg_list[i].dirty=0;
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
			*((uint32_t*)reg_list[i].value)=regval;
		} else {
			reg_list[i].valid=0;
		}
	}
	//We have used A3 as a scratch register and we will need to write that back.
	reg_list[XT_REG_IDX_A3].dirty=1;

	return ERROR_OK;
}


static int esp108_write_dirty_registers(struct target *target)
{
	int i, j;
	int res;
	uint32_t regval, windowbase;
	struct esp108_common *esp108=(struct esp108_common*)target->arch_info;
	struct reg *reg_list=esp108->core_cache->reg_list;

	LOG_INFO("%s", __FUNCTION__);

	//We need to write the dirty registers in the cache list back to the processor.
	//Start by writing the SFR/user registers.
	for (i=0; i<XT_NUM_REGS; i++) {
		if (reg_list[i].dirty) {
			if (esp108_regs[i].type==XT_REG_SPECIAL || esp108_regs[i].type==XT_REG_USER) {
				regval=*((uint32_t *)reg_list[i].value);
				LOG_INFO("Writing back reg %s val %08X", esp108_regs[i].name, regval);
				esp108_queue_nexus_reg_write(target, NARADR_DDR, regval);
				esp108_queue_exec_ins(target, XT_INS_RSR(XT_SR_DDR, XT_REG_A3));
				if (esp108_regs[i].type==XT_REG_USER) {
					esp108_queue_exec_ins(target, XT_INS_WUR(esp108_regs[i].reg_num, XT_REG_A3));
				} else { //SFR
					esp108_queue_exec_ins(target, XT_INS_WSR(esp108_regs[i].reg_num, XT_REG_A3));
				}
				reg_list[i].dirty=0;
			}
		}
	}

	//Grab the windowbase, we need it.
	windowbase=*((uint32_t *)reg_list[XT_REG_IDX_WINDOWBASE].value);

	//Check if there are problems with both the ARx as well as the corresponding Rx registers set and dirty.
	//Warn the user if this happens, not much else we can do...
	for (i=XT_REG_IDX_A0; i<=XT_REG_IDX_A15; i++) {
		j=windowbase_offset_to_canonical(i, windowbase);
		if (reg_list[i].dirty && reg_list[j].dirty) {
			if (memcmp(reg_list[i].value, reg_list[j].value, 4)!=0) {
				LOG_INFO("Warning: Both A%d as well as the physical register it points to (AR%d) are dirty and differs in value. Results are undefined!", i-XT_REG_IDX_A0, j-XT_REG_IDX_AR0);
			}
		}
	}

	//Write A0-A16
	for (i=0; i<16; i++) {
		if (reg_list[XT_REG_IDX_A0+i].dirty) {
			regval=*((uint32_t *)reg_list[XT_REG_IDX_A0+i].value);
			LOG_INFO("Writing back reg %s value %08X", esp108_regs[XT_REG_IDX_A0+i].name, regval);
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
				regval=*((uint32_t *)reg_list[realadr].value);
				LOG_INFO("Writing back reg %s value %08X", esp108_regs[realadr].name, regval);
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

	LOG_INFO("%s", __func__);
	if (target->state == TARGET_HALTED) {
		LOG_DEBUG("target was already halted");
		return ERROR_OK;
	}

	esp108_queue_nexus_reg_write(target, NARADR_DCRSET, OCDDCR_DEBUGINTERRUPT);
	res=jtag_execute_queue();

	if(res != ERROR_OK) {
		LOG_ERROR("Failed to set OCDDCR_DEBUGINTERRUPT. Can't halt.");
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

	LOG_INFO("%s current=%d address=%04" PRIx32, __func__, current, address);

	if (target->state != TARGET_HALTED) {
		LOG_WARNING("%s: target not halted", __func__);
		return ERROR_TARGET_NOT_HALTED;
	}

	if(address && !current) {
//		buf_set_u32(buf, 0, 32, address);
//		xtensa_set_core_reg(&xtensa->core_cache->reg_list[XT_REG_IDX_PC], buf);
	} else {
		int cause=*((int*)reg_list[XT_REG_IDX_DEBUGCAUSE].value);
		if (cause&DEBUGCAUSE_DB) {
			//We stopped due to a watchpoint. We can't just resume executing the instruction again because
			//that would trigger the watchpoint again. To fix this, we single-step, which ignores watchpoints.
			xtensa_step(target, current, address, handle_breakpoints);
		}
	}


	res=esp108_write_dirty_registers(target);
	if(res != ERROR_OK) {
		LOG_ERROR("Failed to write back register cache.");
		return ERROR_FAIL;
	}

	//Execute return from debug exception instruction
	esp108_queue_exec_ins(target, XT_INS_RFDO);
	res=jtag_execute_queue();
	if(res != ERROR_OK) {
		LOG_ERROR("Failed to clear OCDDCR_DEBUGINTERRUPT and resume execution.");
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

	LOG_INFO("%s: reading %d bytes from addr %08X", __FUNCTION__, size*count, address);
	LOG_INFO("Converted to aligned addresses: read from %08X to %08X", addrstart_al, addrend_al);

	if (addrstart_al==address && addrend_al==address+(size*count)) {
		albuff=buffer;
	} else {
		albuff=malloc(size*count);
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
		LOG_WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	LOG_INFO("%s: writing %d bytes to addr %08X", __FUNCTION__, size*count, address);
	LOG_INFO("al start %x al end %x", addrstart_al, addrend_al);

	if ((size==0) || (count == 0) || !(buffer)) return ERROR_COMMAND_SYNTAX_ERROR;

	//Allocate a temporary buffer to put the aligned bytes in, if needed.
	if (addrstart_al==address && addrend_al==address+(size*count)) {
		//We discard the const here because albuff can also be non-const
		albuff=(uint8_t*)buffer;
	} else {
		albuff=malloc(size*count);
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
	LOG_INFO("%s", __func__);

	if (target->state != TARGET_HALTED)
		return ERROR_TARGET_NOT_HALTED;


	*reg_list_size = XT_NUM_REGS;
	*reg_list = malloc(sizeof(struct reg *) * (*reg_list_size));

	if (!*reg_list)
		return ERROR_COMMAND_SYNTAX_ERROR;

	for (i = 0; i < XT_NUM_REGS; i++)
		(*reg_list)[i] = &esp108->core_cache->reg_list[i];

	return ERROR_OK;
}

static int xtensa_get_core_reg(struct reg *reg)
{
//We don't need this because we read all registers on halt anyway.
	struct esp108_common *esp108 = reg->arch_info;
	struct target *target = esp108->target;
	if (target->state != TARGET_HALTED)
		return ERROR_TARGET_NOT_HALTED;
	return ERROR_OK;
}

static int xtensa_set_core_reg(struct reg *reg, uint8_t *buf)
{
	struct esp108_common *esp108 = reg->arch_info;
	struct target *target = esp108->target;

	uint32_t value = buf_get_u32(buf, 0, 32);

	if (target->state != TARGET_HALTED)
		return ERROR_TARGET_NOT_HALTED;

	buf_set_u32(reg->value, 0, reg->size, value);
	reg->dirty = 1;
	reg->valid = 1;
	return ERROR_OK;
}


static int xtensa_assert_reset(struct target *target)
{
	int res;
	LOG_INFO("%s", __func__);
	target->state = TARGET_RESET;
	esp108_queue_pwrctl_set(target, PWRCTL_JTAGDEBUGUSE|PWRCTL_DEBUGWAKEUP|PWRCTL_MEMWAKEUP|PWRCTL_COREWAKEUP|PWRCTL_CORERESET);
	res=jtag_execute_queue();

	return res;
}

static int xtensa_deassert_reset(struct target *target)
{
	int res;
	LOG_INFO("%s", __func__);
	if (target->reset_halt) {
		esp108_queue_nexus_reg_write(target, NARADR_DCRSET, OCDDCR_DEBUGINTERRUPT);
	}
	esp108_queue_pwrctl_set(target, PWRCTL_JTAGDEBUGUSE|PWRCTL_DEBUGWAKEUP|PWRCTL_MEMWAKEUP|PWRCTL_COREWAKEUP);
	res=jtag_execute_queue();
	target->state = TARGET_RUNNING;
	return res;
}


static int xtensa_add_breakpoint(struct target *target, struct breakpoint *breakpoint)
{
	struct esp108_common *esp108=(struct esp108_common*)target->arch_info;
	struct reg *reg_list=esp108->core_cache->reg_list;
	size_t slot;

	if (target->state != TARGET_HALTED) {
		LOG_WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (breakpoint->type == BKPT_SOFT) {
		LOG_ERROR("sw breakpoint requested, but software breakpoints not enabled");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	for(slot = 0; slot < esp108->num_brps; slot++) {
		if (esp108->hw_brps[slot] == NULL || esp108->hw_brps[slot] == breakpoint) break;
	}
	if (slot==esp108->num_brps) return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;

	/* Write IBREAKA[slot] and set bit #slot in IBREAKENABLE */
	*((int*)reg_list[XT_REG_IDX_IBREAKA0+slot].value)=breakpoint->address;
	reg_list[XT_REG_IDX_IBREAKA0+slot].dirty=1;
	*((int*)reg_list[XT_REG_IDX_IBREAKENABLE].value)|=(1<<slot);
	reg_list[XT_REG_IDX_IBREAKENABLE].dirty=1;
	esp108->hw_brps[slot] = breakpoint;

	return ERROR_OK;
}

static int xtensa_remove_breakpoint(struct target *target, struct breakpoint *breakpoint)
{
	struct esp108_common *esp108=(struct esp108_common*)target->arch_info;
	struct reg *reg_list = esp108->core_cache->reg_list;
	size_t slot;

	if (target->state != TARGET_HALTED) {
		LOG_WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	for(slot = 0; slot < esp108->num_brps; slot++) {
		if(esp108->hw_brps[slot] == breakpoint)
			break;
	}
	if (slot==esp108->num_brps) return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	/* Clear bit #slot in IBREAKENABLE */
	*((int*)reg_list[XT_REG_IDX_IBREAKENABLE].value)&=~(1<<slot);
	reg_list[XT_REG_IDX_IBREAKENABLE].dirty=1;
	esp108->hw_brps[slot] = NULL;
	return ERROR_OK;
}

static int xtensa_add_watchpoint(struct target *target, struct watchpoint *watchpoint)
{
	struct esp108_common *esp108=(struct esp108_common*)target->arch_info;
	struct reg *reg_list=esp108->core_cache->reg_list;
	size_t slot;
	int dbreakcval;

	if (target->state != TARGET_HALTED) {
		LOG_WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (watchpoint->mask != ~(uint32_t)0) {
		LOG_DEBUG("watchpoint value masks not supported");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	for(slot = 0; slot < esp108->num_wps; slot++) {
		if (esp108->hw_wps[slot] == NULL || esp108->hw_wps[slot] == watchpoint) break;
	}
	if (slot==esp108->num_wps) return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;

	/* Figure out value for dbreakc5..0 */
	dbreakcval=0xaa; //invalid, so we can check if this is changed later
	if (watchpoint->length==1 && (watchpoint->address&0x00)==0) dbreakcval=0x3F;
	if (watchpoint->length==2 && (watchpoint->address&0x01)==0) dbreakcval=0x3E;
	if (watchpoint->length==4 && (watchpoint->address&0x03)==0) dbreakcval=0x3C;
	if (watchpoint->length==8 && (watchpoint->address&0x07)==0) dbreakcval=0x38;
	if (watchpoint->length==16 && (watchpoint->address&0x0F)==0) dbreakcval=0x30;
	if (watchpoint->length==32 && (watchpoint->address&0x1F)==0) dbreakcval=0x20;
	if (watchpoint->length==64 && (watchpoint->address&0x3F)==0) dbreakcval=0x00;
	if (dbreakcval==0xaa) {
		LOG_WARNING("Watchpoint with length %d on address 0x%X not supported by hardware.", watchpoint->length, watchpoint->address);
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	if (watchpoint->rw==WPT_READ) dbreakcval|=(1<<30);
	if (watchpoint->rw==WPT_WRITE) dbreakcval|=(1<<31);
	if (watchpoint->rw==WPT_ACCESS) dbreakcval|=(1<<30)+(1<<31);

	/* Write DBREAKA[slot] and DBCREAKC[slot]*/
	*((int*)reg_list[XT_REG_IDX_DBREAKA0+slot].value)=watchpoint->address;
	reg_list[XT_REG_IDX_DBREAKA0+slot].dirty=1;
	*((int*)reg_list[XT_REG_IDX_DBREAKC0+slot].value)|=dbreakcval;
	reg_list[XT_REG_IDX_DBREAKC0+slot].dirty=1;
	esp108->hw_wps[slot] = watchpoint;

	return ERROR_OK;
}

static int xtensa_remove_watchpoint(struct target *target, struct watchpoint *watchpoint)
{
	struct esp108_common *esp108=(struct esp108_common*)target->arch_info;
	struct reg *reg_list = esp108->core_cache->reg_list;
	size_t slot;

	if (target->state != TARGET_HALTED) {
		LOG_WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	for(slot = 0; slot < esp108->num_wps; slot++) {
		if(esp108->hw_wps[slot] == watchpoint)
			break;
	}
	if (slot==esp108->num_wps) return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;

	/* Clear DBREAKC[slot] to disable watchpoint */
	*((int*)reg_list[XT_REG_IDX_DBREAKC0+slot].value)=0;
	reg_list[XT_REG_IDX_DBREAKC0+slot].dirty=1;
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

	if (target->state != TARGET_HALTED) {
		LOG_WARNING("%s: target not halted", __func__);
		return ERROR_TARGET_NOT_HALTED;
	}

	/* Load debug level into ICOUNTLEVEL

	   (This bit of text copied from the ESP8266 JTAG code.
	   No idea if it's still relevant. ToDo: check. -JD)

	   Originally had DEBUGLEVEL (ie 2) set here, not 1, but
	   seemed to result in occasionally stepping out into
	   inaccessible bits of ROM (low level interrupt handlers?)
	   and never quite recovering... One loop started at
	   0x40000050. Re-attaching with ICOUNTLEVEL 1 caused this to
	   immediately step into an interrupt handler.

	   ICOUNTLEVEL 1 still steps into interrupt handlers, but also
	   seems to recover.

	   TODO: Experiment more, look into CPU exception nuances,
	   consider making this step level a configuration command.
	 */
	*((int*)reg_list[XT_REG_IDX_ICOUNTLEVEL].value)=1;
	reg_list[XT_REG_IDX_ICOUNTLEVEL].dirty=1;
	*((int*)reg_list[XT_REG_IDX_ICOUNT].value)=icount_val;
	reg_list[XT_REG_IDX_ICOUNT].dirty=1;

	cause=*((int*)reg_list[XT_REG_IDX_DEBUGCAUSE].value);
	if (cause&DEBUGCAUSE_DB) {
		//We stopped due to a watchpoint. We can't just resume executing the instruction again because
		//that would trigger the watchpoint again. To fix this, we remove watchpoints, single-step and
		//re-enable the watchpoint.
		LOG_INFO("Single-stepping to get past instruction that triggered the watchpoint...");
		*((int*)reg_list[XT_REG_IDX_DEBUGCAUSE].value)=0; //so we don't recurse into the same routine
		//Save all DBREAKCx registers and set to 0 to disable watchpoints
		for(slot = 0; slot < esp108->num_wps; slot++) {
			dbreakc[slot]=*((uint32_t*)reg_list[XT_REG_IDX_DBREAKC0+slot].value);
			*((uint32_t*)reg_list[XT_REG_IDX_DBREAKC0+slot].value)=0;
			reg_list[XT_REG_IDX_DBREAKC0+slot].dirty=1;
		}
	}

	/* Now ICOUNT is set, we can resume as if we were going to run */
	res = xtensa_resume(target, current, address, 0, 0);
	if(res != ERROR_OK) {
		LOG_ERROR("%s: Failed to resume after setting up single step", __func__);
		return res;
	}

	/* Wait for stepping to complete */
	int64_t start = timeval_ms();
	while(timeval_ms() < start+500) {
		//Do not use target_poll here, it also triggers other things... just manually read the DSR until stepping
		//is complete.
		esp108_queue_nexus_reg_read(target, NARADR_DSR, dsr);
		res=jtag_execute_queue();
		if(res != ERROR_OK) return res;
		if (intfromchars(dsr)&OCDDSR_STOPPED) {
			break;
		} else {
			usleep(50000);
		}
	}
	if(!(intfromchars(dsr)&OCDDSR_STOPPED)) {
		LOG_ERROR("%s: Timed out waiting for target to finish stepping.", __func__);
		return ERROR_TARGET_TIMEOUT;
	}

	if (cause&DEBUGCAUSE_DB) {
		LOG_INFO("...Done, re-instating watchpoints.");
		//Restore the DBREAKCx registers
		for(slot = 0; slot < esp108->num_wps; slot++) {
			*((uint32_t*)reg_list[XT_REG_IDX_DBREAKC0+slot].value)=dbreakc[slot];
			reg_list[XT_REG_IDX_DBREAKC0+slot].dirty=1;
		}
	}

	/* write ICOUNTLEVEL back to zero */
	*((int*)reg_list[XT_REG_IDX_ICOUNTLEVEL].value)=0;
	reg_list[XT_REG_IDX_ICOUNTLEVEL].dirty=1;

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
	LOG_INFO("%s", __func__);
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
	uint8_t pwrstat;
	int res;
	int cmd;
	uint8_t dsr[4], ocdid[4];

	//Read reset state
	esp108_queue_pwrstat_readclear(target, &pwrstat);
	res=jtag_execute_queue();
	if (res!=ERROR_OK) return res;
	if (pwrstat&PWRSTAT_DEBUGWASRESET) LOG_INFO("esp108: Debug controller was reset.");
	if (pwrstat&PWRSTAT_COREWASRESET) LOG_INFO("esp108: Core was reset.");

	//Enable JTAG
	cmd=PWRCTL_DEBUGWAKEUP|PWRCTL_MEMWAKEUP|PWRCTL_COREWAKEUP;
	if (target->state==TARGET_RESET) cmd|=PWRCTL_CORERESET;
	esp108_queue_pwrctl_set(target, cmd);
	esp108_queue_pwrctl_set(target, cmd|PWRCTL_JTAGDEBUGUSE);
	res=jtag_execute_queue();
	if (res!=ERROR_OK) return res;
	
	esp108_queue_nexus_reg_write(target, NARADR_DCRSET, OCDDCR_ENABLEOCD);
	esp108_queue_nexus_reg_read(target, NARADR_OCDID, ocdid);
	esp108_queue_nexus_reg_read(target, NARADR_DSR, dsr);
	res=jtag_execute_queue();
	if (res!=ERROR_OK) return res;
//	LOG_INFO("esp8266: ocdid 0x%X dsr 0x%X", intfromchars(ocdid), intfromchars(dsr));
	
	if (intfromchars(dsr)&OCDDSR_STOPPED) {
		if(target->state != TARGET_HALTED) {
			int oldstate=target->state;
			target->state = TARGET_HALTED;
			
			LOG_INFO("%s: Target halted (dsr=%08X). Fetching register contents.", __FUNCTION__, intfromchars(dsr));
			esp108_fetch_all_regs(target);

			//Examine why the target was halted
			int cause=*((int*)reg_list[XT_REG_IDX_DEBUGCAUSE].value);
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
			LOG_INFO("esp108: Core running again.");
			target->state = TARGET_RUNNING;
			target->debug_reason = DBG_REASON_NOTHALTED;
		}
	}
	return ERROR_OK;
}


static int xtensa_arch_state(struct target *target)
{
	LOG_DEBUG("%s", __func__);
	return ERROR_OK;
}

/** Holds methods for Xtensa targets. */
struct target_type esp108_target = {
	.name = "esp108",

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

	.add_breakpoint = xtensa_add_breakpoint,
	.remove_breakpoint = xtensa_remove_breakpoint,

	.add_watchpoint = xtensa_add_watchpoint,
	.remove_watchpoint = xtensa_remove_watchpoint,

	.target_create = xtensa_target_create,
	.init_target = xtensa_init_target,
	.examine = xtensa_examine,
};

