/***************************************************************************
 *   ESP108 target for OpenOCD                                             *
 *   Copyright (c) 2016 Espressif Systems Ltd.                             *
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
the instruction. DIR1-DIRn are for longer instructions, which afaik are
not available on the ESP108.
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


#define XT_INS_NUM_BITS 24
#define XT_DEBUGLEVEL    2 /* XCHAL_DEBUGLEVEL in xtensa-config.h */
#define XT_NUM_BREAKPOINTS 1
#define XT_NUM_WATCHPOINTS 1

//Xtensa register list taken from gdb/gdb/xtensa-config.c
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
	XT_REG_IDX_B0,
	XT_REG_IDX_B1,
	XT_REG_IDX_B2,
	XT_REG_IDX_B3,
	XT_REG_IDX_B4,
	XT_REG_IDX_B5,
	XT_REG_IDX_B6,
	XT_REG_IDX_B7,
	XT_REG_IDX_B8,
	XT_REG_IDX_B9,
	XT_REG_IDX_B10,
	XT_REG_IDX_B11,
	XT_REG_IDX_B12,
	XT_REG_IDX_B13,
	XT_REG_IDX_B14,
	XT_REG_IDX_B15,
	XT_REG_IDX_PSINTLEVEL,
	XT_REG_IDX_PSUM,
	XT_REG_IDX_PSWOE,
	XT_REG_IDX_PSEXCM,
	XT_REG_IDX_PSCALLINC,
	XT_REG_IDX_PSOWB,
	XT_REG_IDX_ACC,
};

#define XT_NUM_REGS 174

static const struct xtensa_core_reg xt_regs[] = {
	{ XT_REG_IDX_PC, "pc",					0x00, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_AR0, "ar0",				0x01, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_AR1, "ar1",				0x02, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_AR2, "ar2",				0x03, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_AR3, "ar3",				0x04, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_AR4, "ar4",				0x05, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_AR5, "ar5",				0x06, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_AR6, "ar6",				0x07, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_AR7, "ar7",				0x08, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_AR8, "ar8",				0x09, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_AR9, "ar9",				0x0A, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_AR10, "ar10",				0x0B, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_AR11, "ar11",				0x0C, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_AR12, "ar12",				0x0D, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_AR13, "ar13",				0x0E, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_AR14, "ar14",				0x0F, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_AR15, "ar15",				0x10, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_AR16, "ar16",				0x11, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_AR17, "ar17",				0x12, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_AR18, "ar18",				0x13, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_AR19, "ar19",				0x14, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_AR20, "ar20",				0x15, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_AR21, "ar21",				0x16, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_AR22, "ar22",				0x17, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_AR23, "ar23",				0x18, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_AR24, "ar24",				0x19, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_AR25, "ar25",				0x1A, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_AR26, "ar26",				0x1B, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_AR27, "ar27",				0x1C, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_AR28, "ar28",				0x1D, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_AR29, "ar29",				0x1E, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_AR30, "ar30",				0x1F, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_AR31, "ar31",				0x20, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_AR32, "ar32",				0x21, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_AR33, "ar33",				0x22, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_AR34, "ar34",				0x23, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_AR35, "ar35",				0x24, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_AR36, "ar36",				0x25, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_AR37, "ar37",				0x26, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_AR38, "ar38",				0x27, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_AR39, "ar39",				0x28, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_AR40, "ar40",				0x29, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_AR41, "ar41",				0x2A, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_AR42, "ar42",				0x2B, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_AR43, "ar43",				0x2C, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_AR44, "ar44",				0x2D, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_AR45, "ar45",				0x2E, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_AR46, "ar46",				0x2F, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_AR47, "ar47",				0x30, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_AR48, "ar48",				0x31, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_AR49, "ar49",				0x32, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_AR50, "ar50",				0x33, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_AR51, "ar51",				0x34, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_AR52, "ar52",				0x35, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_AR53, "ar53",				0x36, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_AR54, "ar54",				0x37, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_AR55, "ar55",				0x38, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_AR56, "ar56",				0x39, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_AR57, "ar57",				0x3A, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_AR58, "ar58",				0x3B, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_AR59, "ar59",				0x3C, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_AR60, "ar60",				0x3D, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_AR61, "ar61",				0x3E, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_AR62, "ar62",				0x3F, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_AR63, "ar63",				0x40, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_LBEG, "lbeg",				0x41, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_LEND, "lend",				0x42, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_LCOUNT, "lcount",			0x43, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_SAR, "sar",				0x44, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_WINDOWBASE, "windowbase",	0x45, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_WINDOWSTART, "windowstart",0x46, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_CONFIGID0, "configid0",	0x47, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_CONFIGID1, "configid1",	0x48, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_PS, "ps",					0x49, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_THREADPTR, "threadptr",	0x4A, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_BR, "br",					0x4B, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_SCOMPARE1, "scompare1",	0x4C, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_ACCLO, "acclo",			0x4D, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_ACCHI, "acchi",			0x4E, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_M0, "m0",					0x4F, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_M1, "m1",					0x50, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_M2, "m2",					0x51, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_M3, "m3",					0x52, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_EXPSTATE, "expstate",		0x53, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_MMID, "mmid",				0x54, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_IBREAKENABLE, "ibreakenable",0x55, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_MEMCTL, "memctl",			0x56, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_ATOMCTL, "atomctl",		0x57, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_DDR, "ddr",				0x58, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_IBREAKA0, "ibreaka0",		0x59, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_IBREAKA1, "ibreaka1",		0x5A, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_DBREAKA0, "dbreaka0",		0x5B, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_DBREAKA1, "dbreaka1",		0x5C, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_DBREAKC0, "dbreakc0",		0x5D, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_DBREAKC1, "dbreakc1",		0x5E, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_EPC1, "epc1",				0x5F, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_EPC2, "epc2",				0x60, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_EPC3, "epc3",				0x61, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_EPC4, "epc4",				0x62, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_EPC5, "epc5",				0x63, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_EPC6, "epc6",				0x64, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_EPC7, "epc7",				0x65, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_DEPC, "depc",				0x66, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_EPS2, "eps2",				0x67, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_EPS3, "eps3",				0x68, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_EPS4, "eps4",				0x69, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_EPS5, "eps5",				0x6A, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_EPS6, "eps6",				0x6B, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_EPS7, "eps7",				0x6C, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_EXCSAVE1, "excsave1",		0x6D, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_EXCSAVE2, "excsave2",		0x6E, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_EXCSAVE3, "excsave3",		0x6F, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_EXCSAVE4, "excsave4",		0x70, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_EXCSAVE5, "excsave5",		0x71, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_EXCSAVE6, "excsave6",		0x72, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_EXCSAVE7, "excsave7",		0x73, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_INTERRUPT, "interrupt",	0x74, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_INTSET, "intset",			0x75, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_INTCLEAR, "intclear",		0x76, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_INTENABLE, "intenable",	0x77, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_VECBASE, "vecbase",		0x78, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_EXCCAUSE, "exccause",		0x79, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_DEBUGCAUSE, "debugcause",	0x7A, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_CCOUNT, "ccount",			0x7B, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_PRID, "prid",				0x7C, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_ICOUNT, "icount",			0x7D, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_ICOUNTLEVEL, "icountlevel",0x7E, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_EXCVADDR, "excvaddr",		0x7F, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_CCOMPARE0, "ccompare0",	0x80, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_CCOMPARE1, "ccompare1",	0x81, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_CCOMPARE2, "ccompare2",	0x82, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_MISC0, "misc0",			0x83, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_MISC1, "misc1",			0x84, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_MISC2, "misc2",			0x85, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_MISC3, "misc3",			0x86, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_A0, "a0",					0x87, XT_REG_GENERAL, 0 }, 
	{ XT_REG_IDX_A1, "a1",					0x88, XT_REG_GENERAL, 0 }, 
	{ XT_REG_IDX_A2, "a2",					0x89, XT_REG_GENERAL, 0 }, 
	{ XT_REG_IDX_A3, "a3",					0x8A, XT_REG_GENERAL, 0 }, 
	{ XT_REG_IDX_A4, "a4",					0x8B, XT_REG_GENERAL, 0 }, 
	{ XT_REG_IDX_A5, "a5",					0x8C, XT_REG_GENERAL, 0 }, 
	{ XT_REG_IDX_A6, "a6",					0x8D, XT_REG_GENERAL, 0 }, 
	{ XT_REG_IDX_A7, "a7",					0x8E, XT_REG_GENERAL, 0 }, 
	{ XT_REG_IDX_A8, "a8",					0x8F, XT_REG_GENERAL, 0 }, 
	{ XT_REG_IDX_A9, "a9",					0x90, XT_REG_GENERAL, 0 }, 
	{ XT_REG_IDX_A10, "a10",				0x91, XT_REG_GENERAL, 0 }, 
	{ XT_REG_IDX_A11, "a11",				0x92, XT_REG_GENERAL, 0 }, 
	{ XT_REG_IDX_A12, "a12",				0x93, XT_REG_GENERAL, 0 }, 
	{ XT_REG_IDX_A13, "a13",				0x94, XT_REG_GENERAL, 0 }, 
	{ XT_REG_IDX_A14, "a14",				0x95, XT_REG_GENERAL, 0 }, 
	{ XT_REG_IDX_A15, "a15",				0x96, XT_REG_GENERAL, 0 }, 
	{ XT_REG_IDX_B0, "b0",					0x97, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_B1, "b1",					0x98, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_B2, "b2",					0x99, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_B3, "b3",					0x9A, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_B4, "b4",					0x9B, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_B5, "b5",					0x9C, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_B6, "b6",					0x9D, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_B7, "b7",					0x9E, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_B8, "b8",					0x9F, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_B9, "b9",					0xA0, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_B10, "b10",				0xA1, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_B11, "b11",				0xA2, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_B12, "b12",				0xA3, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_B13, "b13",				0xA4, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_B14, "b14",				0xA5, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_B15, "b15",				0xA6, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_PSINTLEVEL, "psintlevel",	0xA7, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_PSUM, "psum",				0xA8, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_PSWOE, "pswoe",			0xA9, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_PSEXCM, "psexcm",			0xAA, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_PSCALLINC, "pscallinc",	0xAB, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_PSOWB, "psowb",			0xAC, XT_REG_SPECIAL, 0 }, 
	{ XT_REG_IDX_ACC, "acc",				0xAD, XT_REG_SPECIAL, 0 }, 
};

#define _XT_INS_FORMAT_RSR(OPCODE,SR,T) (OPCODE			\
					 | ((SR & 0xFF) << 8)	\
					 | ((T & 0x0F) << 4))

#define _XT_INS_FORMAT_RRI8(OPCODE,R,S,T,IMM8) (OPCODE			\
						| ((IMM8 & 0xFF) << 16) \
						| ((R & 0x0F) << 12 )	\
						| ((S & 0x0F) << 8 )	\
						| ((T & 0x0F) << 4 ))

/* Special register number macro for DDR register.
 * this gets used a lot so making a shortcut to it is
 * useful.
 */
#define XT_SR_DDR         (xt_regs[XT_REG_IDX_DDR].reg_num)

/* Xtensa processor instruction opcodes
*/
/* "Return From Debug Operation" to Normal */
#define XT_INS_RFDO_0      0xf1e000
/* "Return From Debug Operation" to OCD Run */
#define XT_INS_RFDO_1      0xf1e100

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


static int xtensa_get_gdb_reg_list(struct target *target,
	struct reg **reg_list[],
	int *reg_list_size,
	enum target_register_class reg_class)
{
	int i;
	struct xtensa_common *xtensa = target_to_xtensa(target);

	*reg_list_size = XT_NUM_REGS;
	*reg_list = malloc(sizeof(struct reg *) * (*reg_list_size));

	if (!*reg_list)
		return ERROR_COMMAND_SYNTAX_ERROR;

	for (i = 0; i < XT_NUM_REGS; i++)
		(*reg_list)[i] = &xtensa->core_cache->reg_list[i];

	return ERROR_OK;
}


static int xtensa_target_create(struct target *target, Jim_Interp *interp)
{
	struct xtensa_common *xtensa = calloc(1, sizeof(struct xtensa_common));

	if (!xtensa)
		return ERROR_COMMAND_SYNTAX_ERROR;

	target->arch_info = xtensa;
	xtensa->tap = target->tap;

	xtensa->num_brps = XT_NUM_BREAKPOINTS;
	xtensa->free_brps = XT_NUM_BREAKPOINTS;
	xtensa->hw_brps = calloc(XT_NUM_BREAKPOINTS, sizeof(struct breakpoint *));

	return ERROR_OK;
}

static int xtensa_init_target(struct command_context *cmd_ctx, struct target *target)
{
	LOG_INFO("%s", __func__);
	struct xtensa_common *xtensa = target_to_xtensa(target);


	xtensa->state = XT_NORMAL; // Assume normal state until we examine

	return ERROR_OK;
}

//Stub
static int xtensa_examine(struct target *target)
{
	target_set_examined(target);
	return ERROR_OK;
}

static uint32_t intfromchars(uint8_t *c) 
{
	return c[0]+(c[1]<<8)+(c[2]<<16)+(c[3]<<24);
}


static int xtensa_poll(struct target *target)
{
	uint8_t pwrstat;
	int res;
	uint8_t dsr[4], ocdid[4];

	//Read reset state
	esp108_queue_pwrstat_readclear(target, &pwrstat);
	res=jtag_execute_queue();
	if (res!=ERROR_OK) return res;
	if (pwrstat&PWRSTAT_DEBUGWASRESET) LOG_INFO("esp108: Debug controller was reset.");
	if (pwrstat&PWRSTAT_COREWASRESET) LOG_INFO("esp108: Core was reset.");

	//Enable JTAG
	esp108_queue_pwrctl_set(target, PWRCTL_DEBUGWAKEUP|PWRCTL_MEMWAKEUP|PWRCTL_COREWAKEUP);
	esp108_queue_pwrctl_set(target, PWRCTL_JTAGDEBUGUSE|PWRCTL_DEBUGWAKEUP|PWRCTL_MEMWAKEUP|PWRCTL_COREWAKEUP);
	res=jtag_execute_queue();
	if (res!=ERROR_OK) return res;
	
	esp108_queue_nexus_reg_write(target, NARADR_DCRSET, OCDDCR_ENABLEOCD);
	esp108_queue_nexus_reg_read(target, NARADR_OCDID, ocdid);
	esp108_queue_nexus_reg_read(target, NARADR_DSR, dsr);
	res=jtag_execute_queue();
	if (res!=ERROR_OK) return res;
	LOG_INFO("esp8266: ocdid 0x%X dsr 0x%X", intfromchars(ocdid), intfromchars(dsr));
	
	
//	struct xtensa_common *xtensa = target_to_xtensa(target);
//	struct reg *reg;

	
	
	
	/*
	res = xtensa_tap_exec(target, TAP_INS_READ_DOSR, 0, &dosr);
	if(res != ERROR_OK) {
		LOG_ERROR("Failed to read DOSR. Not Xtensa OCD?");
		return ERROR_FAIL;
	}

	if(dosr & (DOSR_IN_OCD_MODE)) {
		if(target->state != TARGET_HALTED) {
			if(target->state != TARGET_UNKNOWN && (dosr & DOSR_EXCEPTION) == 0) {
				LOG_WARNING("%s: DOSR has set InOCDMode without the Exception flag. Unexpected. DOSR=0x%02x",
					    __func__, dosr);
			}
			int state = target->state;

			xtensa->state = XT_OCD_HALT;
			target->state = TARGET_HALTED;
			register_cache_invalidate(xtensa->core_cache);
			xtensa_save_context(target);

			if(state == TARGET_DEBUG_RUNNING) {
				target_call_event_callbacks(target, TARGET_EVENT_DEBUG_HALTED);
			} else {
				target_call_event_callbacks(target, TARGET_EVENT_HALTED);
			}

			LOG_DEBUG("target->state: %s", target_state_name(target));
			reg = &xtensa->core_cache->reg_list[XT_REG_IDX_PC];
			LOG_INFO("halted: PC: 0x%" PRIx32, buf_get_u32(reg->value, 0, 32));
			reg = &xtensa->core_cache->reg_list[XT_REG_IDX_DEBUGCAUSE];
			LOG_INFO("debug cause: 0x%" PRIx32, buf_get_u32(reg->value, 0, 32));
		}
	} else if(target->state != TARGET_RUNNING && target->state != TARGET_DEBUG_RUNNING){
		xtensa->state = XT_OCD_RUN;
		target->state = TARGET_RUNNING;
	}
	*/
	return ERROR_OK;
}



/** Holds methods for Xtensa targets. */
struct target_type esp108_target = {
	.name = "esp108",

	.poll = xtensa_poll,
//	.arch_state = xtensa_arch_state,

//	.halt = xtensa_halt,
//	.resume = xtensa_resume,
//	.step = xtensa_step,

//	.assert_reset = xtensa_assert_reset,
//	.deassert_reset = xtensa_deassert_reset,

//	.read_memory = xtensa_read_memory,
//	.write_memory = xtensa_write_memory,

//	.read_buffer = xtensa_read_buffer,
//	.write_buffer = xtensa_write_buffer,

	.get_gdb_reg_list = xtensa_get_gdb_reg_list,

//	.add_breakpoint = xtensa_add_breakpoint,
//	.remove_breakpoint = xtensa_remove_breakpoint,
	/*
	.add_watchpoint = xtensa_add_watchpoint,
	.remove_watchpoint = xtensa_remove_watchpoint,
	*/

	.target_create = xtensa_target_create,
	.init_target = xtensa_init_target,
	.examine = xtensa_examine,
};


//--------------------------------------------------------------------------------------------------------------

#if 0

/* Set up a register we intend to use for scratch purposes */
static int xtensa_setup_scratch_reg(struct target *target, int reg_idx)
{
	struct xtensa_common *xtensa = target_to_xtensa(target);
	struct reg *reg_list = xtensa->core_cache->reg_list;
	int res;
	if (reg_idx < 0 || reg_idx >= XT_NUM_REGS)
		return ERROR_COMMAND_SYNTAX_ERROR;

	if (!reg_list[reg_idx].valid) {
		res = xtensa_get_core_reg(&reg_list[reg_idx]);
		if (res != ERROR_OK)
			return res;
	}
	reg_list[reg_idx].dirty = 1;
	return ERROR_OK;
}


/* Queue an Xtensa CPU instruction via the TAP's LoadDI function */
static inline int xtensa_tap_queue_cpu_inst(struct target *target, uint32_t inst)
{
	uint8_t inst_buf[4];
	buf_set_u32(inst_buf, 0, 32, inst);
	return xtensa_tap_queue(target, TAP_INS_LOAD_DI, inst_buf, NULL);
}


/* Queue a load to a general register aX, via DDR  */
static inline int xtensa_tap_queue_load_general_reg(struct target *target, uint8_t general_reg_num, uint32_t value)
{
	uint8_t value_buf[4];
	int res;
	buf_set_u32(value_buf, 0, 32, value);
	res = xtensa_tap_queue(target, TAP_INS_SCAN_DDR, value_buf, NULL);
	if(res != ERROR_OK)
		return res;
	return xtensa_tap_queue_cpu_inst(target, XT_INS_RSR(XT_SR_DDR, general_reg_num));
}


/* Queue a write to an Xtensa special register, via the WSR instruction.

   This function does not go through the gdb-facing register cache.
*/
static int xtensa_tap_queue_write_sr(struct target *target, enum xtensa_reg_idx idx, uint32_t value)
{
	struct xtensa_common *xtensa = target_to_xtensa(target);
	struct reg *reg_list = xtensa->core_cache->reg_list;
	struct reg *reg;
	struct xtensa_core_reg *xt_reg;
	int res;

	if(idx < 0 || idx >= XT_NUM_REGS)
		return ERROR_COMMAND_SYNTAX_ERROR;

	reg = &reg_list[idx];
	xt_reg = reg->arch_info;

	if(xt_reg->type != XT_REG_SPECIAL)
		return ERROR_COMMAND_SYNTAX_ERROR;

	/* Use a0 as working register */
	xtensa_setup_scratch_reg(target, XT_REG_IDX_A0);

	/* Push new value into a0 via DDR */
	res = xtensa_tap_queue_load_general_reg(target, 0, value);
	if(res != ERROR_OK)
		return res;
	/* load Special Register from a0 */
	res = xtensa_tap_queue_cpu_inst(target, XT_INS_WSR(xt_reg->reg_num, 0));

	return res;
}


static void xtensa_build_reg_cache(struct target *target);




static int xtensa_halt(struct target *target)
{
	int res;

	if (target->state == TARGET_HALTED) {
		LOG_DEBUG("target was already halted");
		return ERROR_OK;
	}

	res = xtensa_tap_exec(target, TAP_INS_DEBUG_INT, 0, 0);
	if(res != ERROR_OK) {
		LOG_ERROR("Failed to issue DebugInt instruction. Can't halt.");
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
	struct xtensa_common *xtensa = target_to_xtensa(target);
	uint8_t buf[4];
	int res;

	LOG_DEBUG("%s current=%d address=%04" PRIx32, __func__, current, address);

	if (target->state != TARGET_HALTED) {
		LOG_WARNING("%s: target not halted", __func__);
		return ERROR_TARGET_NOT_HALTED;
	}

	if(address && !current) {
		buf_set_u32(buf, 0, 32, address);
		xtensa_set_core_reg(&xtensa->core_cache->reg_list[XT_REG_IDX_PC], buf);
	}
	xtensa_restore_context(target);
	register_cache_invalidate(xtensa->core_cache);

	res = xtensa_tap_exec(target, TAP_INS_LOAD_DI, XT_INS_RFDO_1, 0);
	if(res != ERROR_OK) {
		LOG_ERROR("Failed to issue LoadDI instruction. Can't resume.");
		return ERROR_FAIL;
	}

	target->debug_reason = DBG_REASON_NOTHALTED;
	if (!debug_execution)
		target->state = TARGET_RUNNING;
	else
		target->state = TARGET_DEBUG_RUNNING;
	res = target_call_event_callbacks(target, TARGET_EVENT_RESUMED);

	return res;
}

static int xtensa_step(struct target *target,
	int current,
	uint32_t address,
	int handle_breakpoints)
{
	int res;
	uint32_t dosr;
	static const uint32_t icount_val = -2; /* ICOUNT value to load for 1 step */
	if (target->state != TARGET_HALTED) {
		LOG_WARNING("%s: target not halted", __func__);
		return ERROR_TARGET_NOT_HALTED;
	}

	LOG_DEBUG("%s current=%d address=%"PRIx32, __func__, current, address);

	/* Load debug level into ICOUNTLEVEL

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
	res = xtensa_tap_queue_write_sr(target, XT_REG_IDX_ICOUNTLEVEL, 1);
	if(res != ERROR_OK)
		return res;

	/* load ICOUNT step count value */
	res = xtensa_tap_queue_write_sr(target, XT_REG_IDX_ICOUNT, icount_val);
	if(res != ERROR_OK)
		return res;

	res = jtag_execute_queue();
	if(res != ERROR_OK)
		return res;

	/* Wait for everything to settle, seems necessary to avoid bad resumes */
	do {
		res = xtensa_tap_exec(target, TAP_INS_READ_DOSR, 0, &dosr);
		if(res != ERROR_OK) {
			LOG_ERROR("Failed to read DOSR. Not Xtensa OCD?");
			return ERROR_FAIL;
		}
	} while(!(dosr & DOSR_IN_OCD_MODE) || (dosr & DOSR_EXCEPTION));

	/* Now ICOUNT is set, we can resume as if we were going to run */
	res = xtensa_resume(target, current, address, 0, 0);
	if(res != ERROR_OK) {
		LOG_ERROR("%s: Failed to resume after setting up single step", __func__);
		return res;
	}

	/* Wait for stepping to complete */
	int64_t start = timeval_ms();
	while(target->state != TARGET_HALTED && timeval_ms() < start+500) {
		res = target_poll(target);
		if(res != ERROR_OK)
			return res;
		if(target->state != TARGET_HALTED)
			usleep(50000);
	}
	if(target->state != TARGET_HALTED) {
		LOG_ERROR("%s: Timed out waiting for target to finish stepping.", __func__);
		return ERROR_TARGET_TIMEOUT;
	}

	/* write ICOUNTLEVEL back to zero */
	res = xtensa_tap_queue_write_sr(target, XT_REG_IDX_ICOUNTLEVEL, 0);
	if(res != ERROR_OK)
		return res;
	res = jtag_execute_queue();

	return res;
}


static int xtensa_arch_state(struct target *target)
{
	LOG_DEBUG("%s", __func__);
	return ERROR_OK;
}

static int xtensa_assert_reset(struct target *target)
{
	struct xtensa_common *xtensa = target_to_xtensa(target);
	enum reset_types jtag_reset_config = jtag_get_reset_config();

	if (jtag_reset_config & RESET_HAS_SRST) {
		/* default to asserting srst */
		if (jtag_reset_config & RESET_SRST_PULLS_TRST)
			jtag_add_reset(1, 1);
		else
			jtag_add_reset(0, 1);
	}

	target->state = TARGET_RESET;
	jtag_add_sleep(5000);

	register_cache_invalidate(xtensa->core_cache);

	LOG_DEBUG("%s", __func__);
	return ERROR_OK;
}

static int xtensa_deassert_reset(struct target *target)
{
	int res;

	/* deassert reset lines */
	jtag_add_reset(0, 0);

	usleep(100000);
	res = xtensa_poll(target);
	if (res != ERROR_OK)
		return res;

	if (target->reset_halt) {
		/* TODO: work out if possible to halt on reset (I think "no" */
		res = target_halt(target);
		if (res != ERROR_OK) {
			LOG_ERROR("%s: failed to halt afte reset", __func__);
			return res;
		}
		LOG_WARNING("%s: 'reset halt' is not supported for Xtensa. "
			    "Have halted some time after resetting (not the same thing!)", __func__);
	}

	LOG_DEBUG("%s", __func__);
	return ERROR_OK;
}

static int xtensa_read_memory_inner(struct target *target,
				    uint32_t address,
				    uint32_t size,
				    uint32_t count,
				    uint8_t *buffer)
{
	int res;
	uint32_t inst;
	uint8_t imm8;
	static const uint8_t zeroes[4] = {0};

	/* Load DDR with base address, save to register a0 */
	/* Push base base address to a0 via DDR */
	res = xtensa_tap_queue_load_general_reg(target, 0, address);
	if(res != ERROR_OK)
		return res;

	for(imm8 = 0; imm8 < count; imm8++) {
		/* determine the load instruction (based on size) */
		switch(size) {
		case 4:
			inst = XT_INS_L32I(0, 1, imm8); break;
		case 2:
			inst = XT_INS_L16UI(0, 1, imm8); break;
		case 1:
			inst = XT_INS_L8UI(0, 1, imm8); break;
		default:
			return ERROR_COMMAND_SYNTAX_ERROR;
		}
		/* queue the load instruction to the address register */
		res = xtensa_tap_queue_cpu_inst(target, inst);
		if(res != ERROR_OK)
			return res;

		/* queue the load instruction from the address register to DDR */
		res = xtensa_tap_queue_cpu_inst(target, XT_INS_WSR(XT_SR_DDR, 1));
		if(res != ERROR_OK)
			return res;

		/* queue the read of DDR - note specific length to avoid buffer overrun */
		jtag_add_plain_ir_scan(target->tap->ir_length,
				       tap_instr_buf+TAP_INS_SCAN_DDR*4,
				       NULL, TAP_IDLE);
		jtag_add_plain_dr_scan(8*size, zeroes, buffer+imm8*size, TAP_IDLE);
	}
	res = jtag_execute_queue();
	if(res != ERROR_OK) {
		LOG_ERROR("%s: JTAG scan failed", __func__);
		return res;
	}
	return ERROR_OK;
}

static int xtensa_read_memory(struct target *target,
			      uint32_t address,
			      uint32_t size,
			      uint32_t count,
			      uint8_t *buffer)
{
	int res;

	if (target->state != TARGET_HALTED) {
		LOG_WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* sanitize arguments */
	if (((size != 4) && (size != 2) && (size != 1)) || (count == 0) || !(buffer))
		return ERROR_COMMAND_SYNTAX_ERROR;

	if (((size == 4) && (address & 0x3u)) || ((size == 2) && (address & 0x1u)))
		return ERROR_TARGET_UNALIGNED_ACCESS;

	/* Read & dirty a0 & a1 as we're going to use them as working regs */
	xtensa_setup_scratch_reg(target, XT_REG_IDX_A0);
	xtensa_setup_scratch_reg(target, XT_REG_IDX_A1);

	/* NB: if we were supporting the ICACHE option, we would need
	 * to invalidate it here */

	/* All the LxxI instructions support up to 255 offsets per
	   instruction, so we break each read up into blocks of at
	   most this size.
	*/
	while(count > 255) {
		LOG_DEBUG("%s: splitting read from 0x%" PRIx32 " size %d count 255",__func__,
			  address,size);
		res = xtensa_read_memory_inner(target, address, size, 255, buffer);
		if(res != ERROR_OK) {
			LOG_ERROR("%s: inner read failed at address 0x%" PRIx32, __func__, address);
			return res;
		}
		count -= 255;
		address += (255 * size);
		buffer += (255 * size);
	}
	res = xtensa_read_memory_inner(target, address, size, count, buffer);
	if(res != ERROR_OK) {
		LOG_ERROR("%s: final read failed at address 0x%" PRIx32, __func__, address);
	}

	return res;
}

static int xtensa_write_memory_inner(struct target *target,
				     uint32_t address,
				     uint32_t size,
				     uint32_t count,
				     const uint8_t *buffer)
{
	int res;
	uint32_t inst;
	uint8_t imm8;

	/* Push base address to a0 via DDR */
	res = xtensa_tap_queue_load_general_reg(target, 0, address);
	if(res != ERROR_OK)
		return res;

	for(imm8 = 0; imm8 < count; imm8++) {
		/* load next word from buffer into a1, via DDR */
		res = xtensa_tap_queue_load_general_reg(target, 1,
							buf_get_u32(buffer+imm8*size, 0, 8*size));
		if(res != ERROR_OK)
			return res;

		/* determine the store instruction (based on size) */
		switch(size) {
		case 4:
			inst = XT_INS_S32I(0, 1, imm8); break;
		case 2:
			inst = XT_INS_S16I(0, 1, imm8); break;
		case 1:
			inst = XT_INS_S8I(0, 1, imm8); break;
		default:
			return ERROR_COMMAND_SYNTAX_ERROR;
		}
		/* queue the store instruction to the address register */
		res = xtensa_tap_queue_cpu_inst(target, inst);
		if(res != ERROR_OK)
			return res;
	}
	res = jtag_execute_queue();
	if(res != ERROR_OK) {
		LOG_ERROR("%s: JTAG scan failed", __func__);
		return res;
	}

	return ERROR_OK;
}

static int xtensa_write_memory(struct target *target,
			       uint32_t address,
			       uint32_t size,
			       uint32_t count,
			       const uint8_t *buffer)
{
	/* NOTE FOR LATER: This function is almost identical to
	   xtensa_read_memory, and can possibly be converted into a common
	   wrapper function. The only problem is the 'const uint8_t
	   *buffer' rather than the non-const read function version... :(.
	   */
	int res;

	if (target->state != TARGET_HALTED) {
		LOG_WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* sanitize arguments */
	if (((size != 4) && (size != 2) && (size != 1)) || (count == 0) || !(buffer))
		return ERROR_COMMAND_SYNTAX_ERROR;

	if (((size == 4) && (address & 0x3u)) || ((size == 2) && (address & 0x1u)))
		return ERROR_TARGET_UNALIGNED_ACCESS;

	/* Read & dirty a0 & a1 as we're going to use them as working regs */
	xtensa_setup_scratch_reg(target, XT_REG_IDX_A0);
	xtensa_setup_scratch_reg(target, XT_REG_IDX_A1);

	/* All the LxxI instructions support up to 255 offsets per
	   instruction, so we break each read up into blocks of at
	   most this size.
	*/
	while(count > 255) {
		LOG_DEBUG("%s: splitting read from 0x%" PRIx32 " size %d count 255",__func__,
			  address,size);
		res = xtensa_write_memory_inner(target, address, size, 255, buffer);
		if(res != ERROR_OK) {
			LOG_ERROR("%s: inner write failed at address 0x%" PRIx32, __func__, address);

			return res;
		}
		count -= 255;
		address += (255 * size);
		buffer += (255 * size);
	}
	res = xtensa_write_memory_inner(target, address, size, count, buffer);
	if(res != ERROR_OK) {
		LOG_ERROR("%s: final write failed at address 0x%" PRIx32, __func__, address);
	}

	/* NB: if we were supporting the ICACHE option, we would need
	 * to invalidate it here */

	return res;
}

static int xtensa_read_buffer(struct target *target,
			      uint32_t address,
			      uint32_t count,
			      uint8_t *buffer)
{
	uint8_t *aligned_buffer;
	uint32_t aligned_address;
	uint32_t aligned_count;
	int res;

	/* In case we are reading IRAM/IROM, extend our read to be
	 * 32-bit aligned 32-bit reads */
	aligned_address = address & ~3;
	aligned_count = ((address + count + 3) & ~3) - aligned_address;

	if (aligned_count != count)
		aligned_buffer = malloc(aligned_count);
	else
		aligned_buffer = buffer;

	LOG_DEBUG("%s: aligned_address=0x%" PRIx32 " aligned_count=0x%"
		  PRIx32, __func__, aligned_address, aligned_count);

	res = xtensa_read_memory(target, aligned_address,
				 4, aligned_count/4,
				 aligned_buffer);

	if(aligned_count != count) {
		if(res == ERROR_OK) {
			memcpy(buffer, aligned_buffer + (address & 3), count);
		}
		free(aligned_buffer);
	}

	return res;
}

static int xtensa_write_buffer(struct target *target,
			       uint32_t address,
			       uint32_t count,
			       const uint8_t *buffer)
{
	uint8_t *aligned_buffer = 0;
	uint32_t aligned_address;
	uint32_t aligned_count;
	int res;

	/* In case we are writing IRAM/IROM, extend our write to cover
	 * 32-bit aligned 32-bit writes */
	aligned_address = address & ~3;
	aligned_count = ((address + count + 3) & ~3) - aligned_address;

	if (aligned_count != count) {
		aligned_buffer = malloc(aligned_count);
		// Fill in head word with what's currently in memory
		res = xtensa_read_buffer(target, aligned_address,
					 4, aligned_buffer);
		if(res != ERROR_OK)
			goto cleanup;
		if(aligned_count > 4) {
			// Fill in tail word with what's currently in memory
			res = xtensa_read_buffer(target,
						 aligned_address+aligned_count-4,
						 4, aligned_buffer+aligned_count-4);
			if(res != ERROR_OK)
				goto cleanup;
		}
		memcpy(aligned_buffer + (address & 3), buffer, count);
		buffer = aligned_buffer;
	}

	LOG_DEBUG("%s: aligned_address=0x%" PRIx32 " aligned_count=0x%"
		  PRIx32, __func__, aligned_address, aligned_count);

	res = xtensa_write_memory(target, aligned_address,
				  4, aligned_count/4,
				  buffer);

 cleanup:
	if(aligned_buffer) {
		free(aligned_buffer);
	}

	return res;
}

static int xtensa_set_breakpoint(struct target *target, struct breakpoint *breakpoint)
{
	struct xtensa_common *xtensa = target_to_xtensa(target);
	struct reg *reg_list = xtensa->core_cache->reg_list;
	size_t slot;
	int res;

	for(slot = 0; slot < xtensa->num_brps; slot++) {
		if(xtensa->hw_brps[slot] == NULL || xtensa->hw_brps[slot] == breakpoint)
			break;
	}
	assert(slot < xtensa->num_brps && "Breakpoint slot should always be available to set breakpoint");

	/* Write IBREAKA[slot] and set bit #slot in IBREAKENABLE */
	enum xtensa_reg_idx bp_reg_idx = XT_REG_IDX_IBREAKA0+slot;
	xtensa_tap_queue_write_sr(target, bp_reg_idx, breakpoint->address);
	uint32_t ibe_val = buf_get_u32(reg_list[XT_REG_IDX_IBREAKENABLE].value, 0, 32);
	ibe_val |= (1<<slot);
	xtensa_tap_queue_write_sr(target, XT_REG_IDX_IBREAKENABLE, ibe_val);

	res = jtag_execute_queue();
	if(res != ERROR_OK)
		return res;

	xtensa->hw_brps[slot] = breakpoint;

	/* invalidate register cache */
	reg_list[XT_REG_IDX_IBREAKENABLE].valid = 0;
	reg_list[bp_reg_idx].valid = 0;

	return ERROR_OK;
}

static int xtensa_add_breakpoint(struct target *target, struct breakpoint *breakpoint)
{
	struct xtensa_common *xtensa = target_to_xtensa(target);

	if (target->state != TARGET_HALTED) {
		LOG_WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (breakpoint->type == BKPT_SOFT) {
		LOG_ERROR("sw breakpoint requested, but software breakpoints not enabled");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	if (!xtensa->free_brps) {
		LOG_ERROR("no free breakpoint available for hardware breakpoint");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}
	xtensa->free_brps--;

	return xtensa_set_breakpoint(target, breakpoint);
}

static int xtensa_unset_breakpoint(struct target *target, struct breakpoint *breakpoint)
{
	struct xtensa_common *xtensa = target_to_xtensa(target);
	struct reg *reg_list = xtensa->core_cache->reg_list;
	size_t slot;
	int res;

	for(slot = 0; slot < xtensa->num_brps; slot++) {
		if(xtensa->hw_brps[slot] == breakpoint)
			break;
	}
	assert(slot < xtensa->num_brps && "Breakpoint slot not found");

	uint32_t ibe_val = buf_get_u32(reg_list[XT_REG_IDX_IBREAKENABLE].value, 0, 32);
	ibe_val &= ~(1<<slot);
	xtensa_tap_queue_write_sr(target, XT_REG_IDX_IBREAKENABLE, ibe_val);

	res = jtag_execute_queue();
	if(res != ERROR_OK)
		return res;

	xtensa->hw_brps[slot] = NULL;

	/* invalidate register cache */
	reg_list[XT_REG_IDX_IBREAKENABLE].valid = 0;
	return ERROR_OK;
}

static int xtensa_remove_breakpoint(struct target *target, struct breakpoint *breakpoint)
{
	struct xtensa_common *xtensa = target_to_xtensa(target);
	int res;

	if (target->state != TARGET_HALTED) {
		LOG_WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	res = xtensa_unset_breakpoint(target, breakpoint);
	if(res == ERROR_OK) {
		xtensa->free_brps++;
		assert(xtensa->free_brps <= xtensa->num_brps && "Free breakpoint count should always be less than max breakpoints");
	}
	return res;
}


/* Read register value from target. This function goes via the gdb-facing register cache. */
static int xtensa_read_register(struct target *target, int idx, int force)
{
	struct xtensa_common *xtensa = target_to_xtensa(target);
	struct reg *reg_list = xtensa->core_cache->reg_list;
	struct reg *reg;
	struct xtensa_core_reg *xt_reg;
	uint32_t reg_value;
	uint8_t read_reg_buf[4];
	int res;

	if(idx < 0 || idx >= XT_NUM_REGS)
		return ERROR_COMMAND_SYNTAX_ERROR;

	reg = &reg_list[idx];
	xt_reg = reg->arch_info;

	LOG_DEBUG("%s %s valid=%d dirty=%d force=%d", __func__, reg->name,
		  reg->valid, reg->dirty, force);

	if((reg->valid && !force) || reg->dirty)
		return ERROR_OK; /* Still OK */

	if (target->state != TARGET_HALTED)
		return ERROR_TARGET_NOT_HALTED;

	if(xt_reg->type == XT_REG_GENERAL) {
		/* Access via WSR from general reg Ax to DDR */
		res = xtensa_tap_queue_cpu_inst(target, XT_INS_WSR(XT_SR_DDR,xt_reg->reg_num));
		if(res != ERROR_OK)
			return res;
		xtensa_tap_queue(target, TAP_INS_SCAN_DDR, NULL, read_reg_buf);
		if(res != ERROR_OK)
			return res;
	}
	else {
		/* Otherwise, access is via a special register read via a0 */

		/* Store a0 as being used as scratch reg */
		xtensa_setup_scratch_reg(target, XT_REG_IDX_A0);

		/* RSR to read special reg to a0, then WSR to DDR, then scan via OCD */
		res = xtensa_tap_queue_cpu_inst(target, XT_INS_RSR(xt_reg->reg_num, 0));
		if(res != ERROR_OK)
			return res;
		res = xtensa_tap_queue_cpu_inst(target, XT_INS_WSR(XT_SR_DDR, 0));
		res = xtensa_tap_queue(target, TAP_INS_SCAN_DDR, NULL, read_reg_buf);
		if(res != ERROR_OK)
			return res;
	}

	res = jtag_execute_queue();
	if(res != ERROR_OK)
		return res;

	reg_value = buf_get_u32(read_reg_buf, 0, 32);
	buf_set_u32(reg->value, 0, 32, reg_value);

	LOG_DEBUG("%s: read %s type %d num %d value 0x%" PRIx32, __func__, xt_reg->name,
		 xt_reg->type, xt_reg->reg_num, reg_value);

	reg->valid = 1;
	reg->dirty = 0;
	return ERROR_OK;
}

static int xtensa_write_register(struct target *target, enum xtensa_reg_idx idx)
{
	struct xtensa_common *xtensa = target_to_xtensa(target);
	struct reg *reg_list = xtensa->core_cache->reg_list;
	struct reg *reg;
	struct xtensa_core_reg *xt_reg, *xt_alias;
	uint32_t value;
	int res, i;

	if (target->state != TARGET_HALTED)
		return ERROR_TARGET_NOT_HALTED;

	if(idx < 0 || idx >= XT_NUM_REGS)
		return ERROR_COMMAND_SYNTAX_ERROR;

	reg = &reg_list[idx];
	xt_reg = reg->arch_info;

	LOG_DEBUG("%s %s dirty=%d value=%04"PRIx32, __func__, reg->name,
		  reg->dirty, buf_get_u32(reg->value, 0, 32));

	if(!reg->dirty)
		return ERROR_OK;

	if(xt_reg->type == XT_REG_GENERAL) {
		/* Load new register value to general register Ax via DDR */
		res = xtensa_tap_queue_load_general_reg(target, xt_reg->reg_num,
							buf_get_u32(reg->value, 0, 32));
		if(res != ERROR_OK)
			return res;
	}
	else {
		/* Special register load */
		value = buf_get_u32(reg->value, 0, 32);
		res = xtensa_tap_queue_write_sr(target, xt_reg->idx, value);
		if(res != ERROR_OK)
			return res;
	}

	res = jtag_execute_queue();
	if(res != ERROR_OK)
		return res;

	/* In case we just wrote to an aliased register, make sure to
	   invalidate any other register sharing the same special
	   register number */
	if(xt_reg->type == XT_REG_ALIASED || xt_reg->type==XT_REG_SPECIAL) {
		for(i = 0; i < XT_NUM_REGS; i++) {
			xt_alias = reg_list[i].arch_info;
			if((xt_alias->type == XT_REG_ALIASED || xt_alias->type == XT_REG_SPECIAL)
			   && xt_alias->reg_num == xt_reg->reg_num) {
				reg_list[i].valid = 0;
				reg_list[i].dirty = 0;
			}
		}
	}

	reg->valid = 1;
	reg->dirty = 0;

	return ERROR_OK;
}

static int xtensa_get_core_reg(struct reg *reg)
{
	struct xtensa_core_reg *xt_reg = reg->arch_info;
	return xtensa_read_register(xt_reg->target, xt_reg->idx, 1);
}

static int xtensa_set_core_reg(struct reg *reg, uint8_t *buf)
{
	struct xtensa_core_reg *xt_reg = reg->arch_info;
	struct target *target = xt_reg->target;
	uint32_t value = buf_get_u32(buf, 0, 32);

	if (target->state != TARGET_HALTED)
		return ERROR_TARGET_NOT_HALTED;

	buf_set_u32(reg->value, 0, reg->size, value);
	reg->dirty = 1;
	reg->valid = 1;
	return ERROR_OK;
}

/* Save context from target */
static int xtensa_save_context(struct target *target)
{
	int i;

	if (target->state != TARGET_HALTED)
		return ERROR_TARGET_NOT_HALTED;

	for(i = 0; i < XT_NUM_REGS; i++)
		xtensa_read_register(target, i, 1);
	return ERROR_OK;
}

/* Restore context to target */
static int xtensa_restore_context(struct target *target)
{
	int i;

	if (target->state != TARGET_HALTED)
		return ERROR_TARGET_NOT_HALTED;

	/* Write back registers in reverse order, because SRs (higher
	   indices) can use general registers (lower indices) as
	   part of writeback, thereby invalidating them.
	*/
	for(i = XT_NUM_REGS-1; i >= 0; i--) {
		xtensa_write_register(target, i);
	}
	return ERROR_OK;
}


static const struct reg_arch_type xtensa_reg_type = {
	.get = xtensa_get_core_reg,
	.set = xtensa_set_core_reg,
};

static void xtensa_build_reg_cache(struct target *target)
{
	struct xtensa_common *xtensa = target_to_xtensa(target);
	struct reg_cache **cache_p = register_get_last_cache_p(&target->reg_cache);
	struct reg_cache *cache = malloc(sizeof(struct reg_cache));
	struct reg *reg_list = calloc(XT_NUM_REGS, sizeof(struct reg));
	struct xtensa_core_reg *arch_info = malloc(
			sizeof(struct xtensa_core_reg) * XT_NUM_REGS);
	uint8_t i;

	cache->name = "Xtensa registers";
	cache->next = NULL;
	cache->reg_list = reg_list;
	cache->num_regs = XT_NUM_REGS;
	(*cache_p)= cache;
	xtensa->core_cache = cache;

	for(i = 0; i < XT_NUM_REGS; i++) {
		assert(xt_regs[i].idx == i && "xt_regs[] entry idx field should match index in array");
		arch_info[i] = xt_regs[i];
		arch_info[i].target = target;
		if(arch_info[i].type == XT_REG_ALIASED) {
			/* NB: This is a constant at the moment, but will eventually be target-specific */
			arch_info[i].reg_num += XT_DEBUGLEVEL;
		}
		reg_list[i].name = arch_info[i].name;
		reg_list[i].size = 32;
		reg_list[i].value = calloc(1,4);
		reg_list[i].dirty = 0;
		reg_list[i].valid = 0;
		reg_list[i].type = &xtensa_reg_type;
		reg_list[i].arch_info = &arch_info[i];
	}
}

#endif
