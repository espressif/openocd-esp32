/***************************************************************************
 *   ESP108 and ESP32 targets for OpenOCD                                  *
 *   Copyright (C) 2016-2017 Espressif Systems Ltd.                        *
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


#ifndef XTENSA_ESP108_COMMON_H
#define XTENSA_ESP108_COMMON_H


#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "target.h"
#include "target_type.h"
#include "register.h"
#include "assert.h"
#include "time_support.h"

#include "time_support.h"
#include "rtos/rtos.h"

#include <jtag/jtag.h>
#include "breakpoints.h"
#include "algorithm.h"


#define XT_INS_NUM_BITS 24
#define XT_DEBUGLEVEL	 6 /* XCHAL_DEBUGLEVEL in xtensa-config.h */
#define XT_NUM_BREAKPOINTS 2
#define XT_NUM_WATCHPOINTS 2

enum xtensa_state {
	XT_NORMAL,
	XT_OCD_RUN,
	XT_OCD_HALT,
};


enum FlashBootstrap {
	FBS_DONTCARE = 0,
	FBS_TMSLOW,
	FBS_TMSHIGH,
};

// This is common fields definitions for all targets.
//
#define ESP108_COMMON_FIELDS	enum xtensa_state state;\
	struct reg_cache *core_cache;\
	struct target *target;\
	uint8_t prevpwrstat;\
	int resetAsserted;\
	int traceActive;\
	/* Number of breakpoints available */\
	uint32_t num_brps; \
	struct breakpoint **hw_brps;\
	/* Number of watchpoints available */\
	uint32_t num_wps; \
	struct watchpoint **hw_wps;\
	enum FlashBootstrap flashBootstrap; //0 - don't care, 1 - TMS low, 2 - TMS high

struct esp108_common {
	//	struct jtag_tap *tap;
	ESP108_COMMON_FIELDS;
};

/* Only supported in cores with in-CPU MMU. None of Espressif chips as of now. */
enum xtensa_mode {
	XT_MODE_RING0,
	XT_MODE_RING1,
	XT_MODE_RING2,
	XT_MODE_RING3,
	XT_MODE_ANY // special value to run algorithm in current core mode
};

enum xtensa_reg_idx windowbase_offset_to_canonical(const enum xtensa_reg_idx reg, const int windowbase);
enum xtensa_reg_idx canonical_to_windowbase_offset(const enum xtensa_reg_idx reg, const int windowbase);
int regReadable(int flags, int cpenable);

void esp108_queue_tdi_idle(struct target *target);
void esp108_add_set_ir(struct target *target, uint8_t value);
void esp108_add_dr_scan(struct target *target, int len, const uint8_t *src, uint8_t *dest, tap_state_t endstate);
void esp108_queue_nexus_reg_write(struct target *target, const uint8_t reg, const uint32_t value);
void esp108_queue_nexus_reg_read(struct target *target, const uint8_t reg, uint8_t *value);
void esp108_queue_pwrctl_set(struct target *target, uint8_t value);
void esp108_queue_pwrstat_readclear(struct target *target, uint8_t *value);
void esp108_queue_exec_ins(struct target *target, int32_t ins);
uint32_t esp108_reg_get(struct reg *reg);
void esp108_reg_set(struct reg *reg, uint32_t value);
int esp108_do_checkdsr(struct target *target, const char *function, const int line);

// Read and clear the DSR register
unsigned int xtensa_read_dsr(struct target *target);
int xtensa_get_core_reg(struct reg *reg);
int xtensa_set_core_reg(struct reg *reg, uint8_t *buf);

int xtensa_examine(struct target *target);

uint32_t xtensa_read_reg_direct(struct target *target, uint8_t reg);
int read_reg_direct(struct target *target, uint8_t addr);
int xtensa_write_uint32(struct target *target, uint32_t addr, uint32_t val);
int xtensa_write_uint32_list(struct target *target, const uint32_t* addr_value_pairs_list, size_t count);
int xtensa_run_algorithm(struct target *target,
	int num_mem_params, struct mem_param *mem_params,
	int num_reg_params, struct reg_param *reg_params,
	uint32_t entry_point, uint32_t exit_point,
	int timeout_ms, void *arch_info);
int xtensa_start_algorithm_generic(struct target *target,
	int num_mem_params, struct mem_param *mem_params,
	int num_reg_params, struct reg_param *reg_params,
	uint32_t entry_point, uint32_t exit_point,
	void *arch_info, struct reg_cache *core_cache);
int xtensa_wait_algorithm_generic(struct target *target,
	int num_mem_params, struct mem_param *mem_params,
	int num_reg_params, struct reg_param *reg_params,
	uint32_t exit_point, int timeout_ms,
	void *arch_info, struct reg_cache *core_cache);

int esp32_soc_reset(struct target *target);

uint32_t intfromchars(uint8_t *c);

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

struct xtensa_algorithm {
	enum xtensa_mode core_mode;
	uint32_t context[XT_NUM_REGS];
};

/* Special register number macro for DDR register.
* this gets used a lot so making a shortcut to it is
* useful.
*/
#define XT_SR_DDR		  (esp108_regs[XT_REG_IDX_DDR].reg_num)

//Same thing for A3/A4
#define XT_REG_A3		  (esp108_regs[XT_REG_IDX_AR3].reg_num)
#define XT_REG_A4		  (esp108_regs[XT_REG_IDX_AR4].reg_num)

#define _XT_INS_FORMAT_RSR(OPCODE,SR,T) (OPCODE			\
					 | ((SR & 0xFF) << 8)	\
					 | ((T & 0x0F) << 4))

#define _XT_INS_FORMAT_RRR(OPCODE,ST,R) (OPCODE			\
					 | ((ST & 0xFF) << 4)	\
					 | ((R & 0x0F) << 12))

#define _XT_INS_FORMAT_RRRN(OPCODE,S, T,IMM4) (OPCODE		  \
					 | ((T & 0x0F) << 4)   \
					 | ((S & 0x0F) << 8)   \
					 | ((IMM4 & 0x0F) << 12))

#define _XT_INS_FORMAT_RRI8(OPCODE,R,S,T,IMM8) (OPCODE			\
						| ((IMM8 & 0xFF) << 16) \
						| ((R & 0x0F) << 12 )	\
						| ((S & 0x0F) << 8 )	\
						| ((T & 0x0F) << 4 ))



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

/* Read Floating-Point Register */
#define XT_INS_RFR(FR,T) _XT_INS_FORMAT_RRR(0xFA0000,((FR<<4)|0x4),T)
/* Write Floating-Point Register */
#define XT_INS_WFR(FR,T) _XT_INS_FORMAT_RRR(0xFA0000,((FR<<4)|0x5),T)

#define XT_PS_RING(_v_)			((uint32_t)((_v_) & 0x3) << 6)
#define XT_PS_RING_MSK			(0x3 << 6)
#define XT_PS_RING_GET(_v_)		(((_v_) >> 6) & 0x3)
#define XT_PS_CALLINC_MSK		(0x3 << 16)
#define XT_PS_OWB_MSK			(0xF << 8)


/* ESP32 memory map */
#define ESP32_DROM_LOW      0x3F400000
#define ESP32_DROM_HIGH     0x3F800000
#define ESP32_EXT_RAM_LOW   0x3F800000
#define ESP32_EXT_RAM_HIGH  0x3FC00000
#define ESP32_DPORT_LOW     0x3ff00000
#define ESP32_DPORT_HIGH    0x3ff80000
#define ESP32_DRAM_LOW      0x3ffA0000
#define ESP32_DRAM_HIGH     0x40000000
#define ESP32_IRAM00_LOW    0x40000000
#define ESP32_IRAM00_HIGH   0x40070000
#define ESP32_IRAM02_LOW    0x40070000
#define ESP32_IRAM02_HIGH   0x400C0000
#define ESP32_RTC_IRAM_LOW  0x400C0000
#define ESP32_RTC_IRAM_HIGH 0x400C2000
#define ESP32_IROM_LOW      0x400D0000
#define ESP32_IROM_HIGH     0x40400000
#define ESP32_RTC_DATA_LOW  0x50000000
#define ESP32_RTC_DATA_HIGH 0x50002000

/* ESP32 dport regs */
#define ESP32_DR_REG_DPORT_BASE         0x3ff00000
#define ESP32_DPORT_APPCPU_CTRL_B_REG   (ESP32_DR_REG_DPORT_BASE + 0x030)
#define ESP32_DPORT_APPCPU_CLKGATE_EN	(1 << 0)

typedef enum
{
	INVALID,
	READONLY,
	READWRITE
} addr_type_t;

addr_type_t esp108_get_addr_type(uint32_t address);

#endif // XTENSA_ESP108_COMMON_H