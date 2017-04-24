/***************************************************************************
 *	 ESP108 debug module's registers definitions						   *
 *	 Copyright (C) 2017 Espressif Systems Ltd.							   *
 *	 <alexey@espressif.com>												   *
 *																		   *
 *	 Derived from original ESP8266 target.								   *
 *	 Copyright (C) 2015 by Angus Gratton								   *
 *	 gus@projectgus.com													   *
 *																		   *
 *	 This program is free software; you can redistribute it and/or modify  *
 *	 it under the terms of the GNU General Public License as published by  *
 *	 the Free Software Foundation; either version 2 of the License, or	   *
 *	 (at your option) any later version.								   *
 *																		   *
 *	 This program is distributed in the hope that it will be useful,	   *
 *	 but WITHOUT ANY WARRANTY; without even the implied warranty of		   *
 *	 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the		   *
 *	 GNU General Public License for more details.						   *
 *																		   *
 *	 You should have received a copy of the GNU General Public License	   *
 *	 along with this program; if not, write to the						   *
 *	 Free Software Foundation, Inc.,									   *
 *	 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.		   *
 ***************************************************************************/
#ifndef __ESP108_DBG_REGS_H__
#define __ESP108_DBG_REGS_H__	1

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

#define DEBUGCAUSE_IC			(1<<0)	//ICOUNT exception
#define DEBUGCAUSE_IB			(1<<1)	//IBREAK exception
#define DEBUGCAUSE_DB			(1<<2)	//DBREAK exception
#define DEBUGCAUSE_BI			(1<<3)	//BREAK instruction encountered
#define DEBUGCAUSE_BN			(1<<4)	//BREAK.N instruction encountered
#define DEBUGCAUSE_DI			(1<<5)	//Debug Interrupt

#define TRAXCTRL_TREN			(1<<0)	//Trace enable. Tracing starts on 0->1
#define TRAXCTRL_TRSTP			(1<<1)	//Trace Stop. Make 1 to stop trace.
#define TRAXCTRL_PCMEN			(1<<2)	//PC match enable
#define TRAXCTRL_PTIEN			(1<<4)	//Processor-trigger enable
#define TRAXCTRL_CTIEN			(1<<5)	//Cross-trigger enable
#define TRAXCTRL_TMEN			(1<<7)	//Tracemem Enable. Always set.
#define TRAXCTRL_CNTU			(1<<9)	//Post-stop-trigger countdown units; selects when DelayCount-- happens.
										//0 - every 32-bit word written to tracemem, 1 - every cpu instruction
#define TRAXCTRL_TSEN			(1<<11)	//Undocumented/deprecated?
#define TRAXCTRL_SMPER_SHIFT	12		//Send sync every 2^(9-smper) messages. 7=reserved, 0=no sync msg
#define TRAXCTRL_SMPER_MASK		0x7		//Synchronization message period
#define TRAXCTRL_PTOWT			(1<<16) //Processor Trigger Out (OCD halt) enabled when stop triggered
#define TRAXCTRL_PTOWS			(1<<17) //Processor Trigger Out (OCD halt) enabled when trace stop completes
#define TRAXCTRL_CTOWT			(1<<20) //Cross-trigger Out enabled when stop triggered
#define TRAXCTRL_CTOWS			(1<<21) //Cross-trigger Out enabled when trace stop completes
#define TRAXCTRL_ITCTO			(1<<22) //Integration mode: cross-trigger output
#define TRAXCTRL_ITCTIA			(1<<23) //Integration mode: cross-trigger ack
#define TRAXCTRL_ITATV			(1<<24) //replaces ATID when in integration mode: ATVALID output
#define TRAXCTRL_ATID_MASK		0x7F	//ARB source ID
#define TRAXCTRL_ATID_SHIFT		24
#define TRAXCTRL_ATEN			(1<<31) //ATB interface enable

#define TRAXSTAT_TRACT			(1<<0)	//Trace active flag.
#define TRAXSTAT_TRIG			(1<<1)	//Trace stop trigger. Clears on TREN 1->0
#define TRAXSTAT_PCMTG			(1<<2)	//Stop trigger caused by PC match. Clears on TREN 1->0
#define TRAXSTAT_PJTR			(1<<3)	//JTAG transaction result. 1=err in preceding jtag transaction.
#define TRAXSTAT_PTITG			(1<<4)	//Stop trigger caused by Processor Trigger Input. Clears on TREN 1->0
#define TRAXSTAT_CTITG			(1<<5)	//Stop trigger caused by Cross-Trigger Input. Clears on TREN 1->0
#define TRAXSTAT_MEMSZ_SHIFT	8		//Traceram size inducator. Usable trace ram is 2^MEMSZ bytes.
#define TRAXSTAT_MEMSZ_MASK		0x1F
#define TRAXSTAT_PTO			(1<<16) //Processor Trigger Output: current value
#define TRAXSTAT_CTO			(1<<17) //Cross-Trigger Output: current value
#define TRAXSTAT_ITCTOA			(1<<22) //Cross-Trigger Out Ack: current value
#define TRAXSTAT_ITCTI			(1<<23) //Cross-Trigger Input: current value
#define TRAXSTAT_ITATR			(1<<24) //ATREADY Input: current value

#define TRAXADDR_TADDR_SHIFT	0		//Trax memory address, in 32-bit words.
#define TRAXADDR_TADDR_MASK		0x1FFFFF //Actually is only as big as the trace buffer size max addr.
#define TRAXADDR_TWRAP_SHIFT	21		//Amount of times TADDR has overflown
#define TRAXADDR_TWRAP_MASK		0x3FF
#define TRAXADDR_TWSAT			(1<<31) //1 if TWRAP has overflown, clear by disabling tren.

#define PCMATCHCTRL_PCML_SHIFT	0		//Amount of lower bits to ignore in pc trigger register
#define PCMATCHCTRL_PCML_MASK	0x1F 
#define PCMATCHCTRL_PCMS		(1<<31) //PC Match Sense, 0 - match when procs PC is in-range, 1 - match when
										//out-of-range

#endif /*__ESP108_DBG_REGS_H__*/
