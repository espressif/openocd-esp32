/***************************************************************************
 *   Generic Xtensa target                                                 *
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

#ifndef OPENOCD_TARGET_XTENSA_H
#define OPENOCD_TARGET_XTENSA_H

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif
#include "target.h"
#include "assert.h"
#include "breakpoints.h"
#include "xtensa_regs.h"
#include "xtensa_debug_module.h"

/**
 * @file
 * Holds the interface to Xtensa cores.
 */

#define _XT_INS_FORMAT_RSR(OPCODE,SR,T) ((OPCODE)	    \
		| (((SR) & 0xFF) << 8) \
		| (((T) & 0x0F) << 4))

#define _XT_INS_FORMAT_RRR(OPCODE,ST,R) ((OPCODE)	    \
		| (((ST) & 0xFF) << 4) \
		| (((R) & 0x0F) << 12))

#define _XT_INS_FORMAT_RRRN(OPCODE,S, T,IMM4) ((OPCODE)	      \
		| (((T) & 0x0F) << 4)	\
		| (((S) & 0x0F) << 8)	\
		| (((IMM4) & 0x0F) << 12))

#define _XT_INS_FORMAT_RRI8(OPCODE,R,S,T,IMM8) ((OPCODE)	    \
		| (((IMM8) & 0xFF) << 16) \
		| (((R) & 0x0F) << 12 )	\
		| (((S) & 0x0F) << 8 )	\
		| (((T) & 0x0F) << 4 ))

#define _XT_INS_FORMAT_RRI4(OPCODE,IMM4,R,S,T) ((OPCODE) \
		| (((IMM4) & 0x0F) << 20) \
		| (((R) & 0x0F) << 12) \
		| (((S) & 0x0F) << 8)	\
		| (((T) & 0x0F) << 4))


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

/* 32-bit break */
#define XT_INS_BREAK(IMM1,IMM2)  _XT_INS_FORMAT_RRR(0x000000,((IMM1&0x0F)<<4)|(IMM2&0x0F),0x4)
/* 16-bit break */
#define XT_INS_BREAKN(IMM4)  _XT_INS_FORMAT_RRRN(0x00000D,IMM4,0x2,0xF)

#define XT_PS_RING(_v_)         ((uint32_t)((_v_) & 0x3) << 6)
#define XT_PS_RING_MSK          (0x3 << 6)
#define XT_PS_RING_GET(_v_)     (((_v_) >> 6) & 0x3)
#define XT_PS_CALLINC_MSK       (0x3 << 16)
#define XT_PS_OWB_MSK           (0xF << 8)

#define XT_INS_L32E(R,S,T) _XT_INS_FORMAT_RRI4(0x90000,0,R,S,T)
#define XT_INS_S32E(R,S,T) _XT_INS_FORMAT_RRI4(0x490000,0,R,S,T)
#define XT_INS_L32E_S32E_MASK   0xFF000F

#define XT_INS_RFWO 0x3400
#define XT_INS_RFWU 0x3500
#define XT_INS_RFWO_RFWU_MASK   0xFFFFFF

#define XT_ISNS_SZ_MAX                  3
#define XT_LOCAL_MEM_REGIONS_NUM_MAX    8

#define XT_AREGS_NUM_MAX                64
#define XT_USER_REGS_NUM_MAX            256

#define XT_MEM_ACCESS_NONE              0x0
#define XT_MEM_ACCESS_READ              0x1
#define XT_MEM_ACCESS_WRITE             0x2

enum xtensa_mem_err_detect {
	XT_MEM_ERR_DETECT_NONE,
	XT_MEM_ERR_DETECT_PARITY,
	XT_MEM_ERR_DETECT_ECC,
};

struct xtensa_cache_config {
	uint8_t way_count;
	uint8_t line_size;
	uint16_t size;
	bool writeback;
	enum xtensa_mem_err_detect mem_err_check;
};

struct xtensa_local_mem_region_config {
	uint32_t base;
	uint32_t size;
	enum xtensa_mem_err_detect mem_err_check;
	int access;
};

struct xtensa_local_mem_config {
	uint16_t count;
	struct xtensa_local_mem_region_config regions[XT_LOCAL_MEM_REGIONS_NUM_MAX];
};

struct xtensa_mmu_config {
	bool enabled;
	uint8_t itlb_entries_count;
	uint8_t dtlb_entries_count;
	bool ivarway56;
	bool dvarway56;
};

struct xtensa_exception_config {
	bool enabled;
	uint8_t depc_num;
};

struct xtensa_irq_config {
	bool enabled;
	uint8_t irq_num;
};

struct xtensa_high_prio_irq_config {
	bool enabled;
	uint8_t excm_level;
	uint8_t nmi_num;
};

struct xtensa_debug_config {
	bool enabled;
	uint8_t irq_level;
	uint8_t ibreaks_num;
	uint8_t dbreaks_num;
	uint8_t icount_sz;
};

struct xtensa_tracing_config {
	bool enabled;
	uint32_t mem_sz;
	bool reversed_mem_access;
};

struct xtensa_timer_irq_config {
	bool enabled;
	uint8_t comp_num;
};

struct xtensa_config {
	bool density;
	uint8_t aregs_num;
	bool windowed;
	bool coproc;
	bool fp_coproc;
	bool loop;
	uint8_t miscregs_num;
	bool threadptr;
	bool boolean;
	bool cond_store;
	bool ext_l32r;
	bool mac16;
	bool reloc_vec;
	bool proc_id;
	bool mem_err_check;
	uint8_t user_regs_num;
	int user_regs[XT_USER_REGS_NUM_MAX];
	struct xtensa_cache_config icache;
	struct xtensa_cache_config dcache;
	struct xtensa_local_mem_config irom;
	struct xtensa_local_mem_config iram;
	struct xtensa_local_mem_config drom;
	struct xtensa_local_mem_config dram;
	struct xtensa_local_mem_config uram;
	struct xtensa_local_mem_config xlmi;
	struct xtensa_mmu_config mmu;
	struct xtensa_exception_config exc;
	struct xtensa_irq_config irq;
	struct xtensa_high_prio_irq_config high_irq;
	struct xtensa_timer_irq_config tim_irq;
	struct xtensa_debug_config debug;
	struct xtensa_tracing_config trace;
	uint8_t gdb_regs_num;
};

typedef uint32_t xtensa_insn_t;

enum xtensa_stepping_isr_mode {
	XT_STEPPING_ISR_OFF,	/* interrupts are disabled during stepping */
	XT_STEPPING_ISR_ON,	/* interrupts are enabled during stepping */
};

/* Only supported in cores with in-CPU MMU. None of Espressif chips as of now. */
enum xtensa_mode {
	XT_MODE_RING0,
	XT_MODE_RING1,
	XT_MODE_RING2,
	XT_MODE_RING3,
	XT_MODE_ANY	/* special value to run algorithm in current core mode */
};

struct xtensa_sw_breakpoint {
	struct breakpoint *oocd_bp;
	/* original insn */
	uint8_t insn[XT_ISNS_SZ_MAX];
	/* original insn size */
	uint8_t insn_sz;	/* 2 or 3 bytes */
};

struct xtensa_chip_ops {
	void (*on_reset)(struct target *target);
	void (*on_poll)(struct target *target);
	bool (*on_halt)(struct target *target);
};

/**
 * Represents a generic Xtensa core.
 */
struct xtensa {
	const struct xtensa_config *core_config;
	struct xtensa_debug_module dbg_mod;
	struct reg_cache *core_cache;
	uint32_t regs_num;
	struct target *target;
	const struct xtensa_chip_ops *chip_ops;
	bool reset_asserted;
	enum xtensa_stepping_isr_mode stepping_isr_mode;
	struct breakpoint **hw_brps;
	struct watchpoint **hw_wps;
	struct xtensa_sw_breakpoint *sw_brps;
	bool trace_active;
	bool permissive_mode;
	bool suppress_dsr_errors;
};

static inline struct xtensa *target_to_xtensa(struct target *target)
{
	assert(target != NULL);
	return target->arch_info;
}

static inline int xtensa_queue_dbg_reg_read(struct xtensa *xtensa, unsigned reg, uint8_t *data)
{
	struct xtensa_debug_module *dm = &xtensa->dbg_mod;

	if (!xtensa->core_config->trace.enabled &&
		(reg <= NARADR_MEMADDREND || (reg >= NARADR_PMG && reg <= NARADR_PMSTAT7))) {
		LOG_ERROR("Can not access %u reg when Trace Port option disabled!", reg);
		return ERROR_FAIL;
	}
	return dm->dbg_ops->queue_reg_read(dm, reg, data);
}

static inline int xtensa_queue_dbg_reg_write(struct xtensa *xtensa, unsigned reg, uint32_t data)
{
	struct xtensa_debug_module *dm = &xtensa->dbg_mod;

	if (!xtensa->core_config->trace.enabled &&
		(reg <= NARADR_MEMADDREND || (reg >= NARADR_PMG && reg <= NARADR_PMSTAT7))) {
		LOG_ERROR("Can not access %u reg when Trace Port option disabled!", reg);
		return ERROR_FAIL;
	}
	return dm->dbg_ops->queue_reg_write(dm, reg, data);
}

static inline int xtensa_queue_pwr_reg_read(struct xtensa *xtensa,
	unsigned reg,
	uint8_t *data,
	uint32_t clear)
{
	struct xtensa_debug_module *dm = &xtensa->dbg_mod;
	return dm->pwr_ops->queue_reg_read(dm, reg, data, clear);
}

static inline int xtensa_queue_pwr_reg_write(struct xtensa *xtensa, unsigned reg, uint32_t data)
{
	struct xtensa_debug_module *dm = &xtensa->dbg_mod;
	return dm->pwr_ops->queue_reg_write(dm, reg, data);
}

static inline void xtensa_queue_exec_ins(struct xtensa *xtensa, int32_t ins)
{
	xtensa_queue_dbg_reg_write(xtensa, NARADR_DIR0EXEC, ins);
}

int xtensa_init_arch_info(struct target *target,
	struct xtensa *xtensa,
	const struct xtensa_config *cfg,
	const struct xtensa_debug_module_config *dm_cfg,
	const struct xtensa_chip_ops *chip_ops);
void xtensa_build_reg_cache(struct target *target);

static inline void xtensa_stepping_isr_mode_set(struct target *target,
	enum xtensa_stepping_isr_mode mode)
{
	struct xtensa *xtensa = target_to_xtensa(target);
	xtensa->stepping_isr_mode = mode;
}

static inline enum xtensa_stepping_isr_mode xtensa_stepping_isr_mode_get(struct target *target)
{
	struct xtensa *xtensa = target_to_xtensa(target);
	return xtensa->stepping_isr_mode;
}

static inline bool xtensa_addr_in_mem(const struct xtensa_local_mem_config *mem, uint32_t addr)
{
	for (uint16_t i = 0; i < mem->count; i++) {
		if (addr >= mem->regions[i].base &&
			addr < mem->regions[i].base + mem->regions[i].size)
			return true;
	}
	return false;
}

static inline bool xtensa_data_addr_valid(struct target *target, uint32_t addr)
{
	struct xtensa *xtensa = target_to_xtensa(target);

	if (xtensa_addr_in_mem(&xtensa->core_config->drom, addr))
		return true;
	if (xtensa_addr_in_mem(&xtensa->core_config->dram, addr))
		return true;
	if (xtensa_addr_in_mem(&xtensa->core_config->uram, addr))
		return true;
	return false;
}

static inline int xtensa_core_status_clear(struct target *target, xtensa_dsr_t bits)
{
	struct xtensa *xtensa = target_to_xtensa(target);
	return xtensa_dm_core_status_clear(&xtensa->dbg_mod, bits);
}

int xtensa_core_status_check(struct target *target);

int xtensa_examine(struct target *target);
int xtensa_wakeup(struct target *target);
int xtensa_smpbreak_set(struct target *target, uint32_t set);
xtensa_reg_val_t xtensa_reg_get(struct target *target, enum xtensa_reg_id reg_id);
void xtensa_reg_set(struct target *target, enum xtensa_reg_id reg_id, xtensa_reg_val_t value);
int xtensa_fetch_all_regs(struct target *target);
int xtensa_get_gdb_reg_list(struct target *target,
	struct reg **reg_list[],
	int *reg_list_size,
	enum target_register_class reg_class);
int xtensa_poll(struct target *target);
void xtensa_on_poll(struct target *target);
int xtensa_halt(struct target *target);
int xtensa_resume(struct target *target,
	int current,
	target_addr_t address,
	int handle_breakpoints,
	int debug_execution);
int xtensa_prepare_resume(struct target *target,
	int current,
	target_addr_t address,
	int handle_breakpoints,
	int debug_execution);
int xtensa_do_resume(struct target *target);
int xtensa_step(struct target *target,
	int current,
	target_addr_t address,
	int handle_breakpoints);
int xtensa_do_step(struct target *target,
	int current,
	target_addr_t address,
	int handle_breakpoints);
int xtensa_mmu_is_enabled(struct target *target, int *enabled);
int xtensa_read_memory(struct target *target,
	target_addr_t address,
	uint32_t size,
	uint32_t count,
	uint8_t *buffer);
int xtensa_read_buffer(struct target *target,
	target_addr_t address,
	uint32_t count,
	uint8_t *buffer);
int xtensa_write_memory(struct target *target,
	target_addr_t address,
	uint32_t size,
	uint32_t count,
	const uint8_t *buffer);
int xtensa_write_buffer(struct target *target,
	target_addr_t address,
	uint32_t count,
	const uint8_t *buffer);
int xtensa_checksum_memory(struct target *target, target_addr_t address,
	uint32_t count, uint32_t *checksum);
int xtensa_assert_reset(struct target *target);
int xtensa_deassert_reset(struct target *target);
int xtensa_breakpoint_add(struct target *target, struct breakpoint *breakpoint);
int xtensa_breakpoint_remove(struct target *target, struct breakpoint *breakpoint);
int xtensa_watchpoint_add(struct target *target, struct watchpoint *watchpoint);
int xtensa_watchpoint_remove(struct target *target, struct watchpoint *watchpoint);
int xtensa_start_algorithm(struct target *target,
	int num_mem_params, struct mem_param *mem_params,
	int num_reg_params, struct reg_param *reg_params,
	target_addr_t entry_point, target_addr_t exit_point,
	void *arch_info);
int xtensa_wait_algorithm(struct target *target,
	int num_mem_params, struct mem_param *mem_params,
	int num_reg_params, struct reg_param *reg_params,
	target_addr_t exit_point, int timeout_ms,
	void *arch_info);
int xtensa_run_algorithm(struct target *target,
	int num_mem_params, struct mem_param *mem_params,
	int num_reg_params, struct reg_param *reg_params,
	target_addr_t entry_point, target_addr_t exit_point,
	int timeout_ms, void *arch_info);

COMMAND_HELPER(xtensa_cmd_permissive_mode_do, struct xtensa *xtensa);
COMMAND_HELPER(xtensa_cmd_mask_interrupts_do, struct xtensa *xtensa);
COMMAND_HELPER(xtensa_cmd_perfmon_dump_do, struct xtensa *xtensa);
COMMAND_HELPER(xtensa_cmd_perfmon_enable_do, struct xtensa *xtensa);
COMMAND_HELPER(xtensa_cmd_tracestart_do, struct xtensa *xtensa);
COMMAND_HELPER(xtensa_cmd_tracestop_do, struct xtensa *xtensa);
COMMAND_HELPER(xtensa_cmd_tracedump_do, struct xtensa *xtensa, const char *fname);

extern const struct command_registration xtensa_command_handlers[];

#endif	/* OPENOCD_TARGET_XTENSA_H */
