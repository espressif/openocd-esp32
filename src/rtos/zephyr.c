// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   Copyright (C) 2017 by Intel Corporation
 *   Leandro Pereira <leandro.pereira@intel.com>
 *   Daniel Glöckner <dg@emlix.com>*
 *   Copyright (C) 2021 by Synopsys, Inc.
 *   Evgeniy Didin <didin@synopsys.com>
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <helper/time_support.h>
#include <jtag/jtag.h>

#include "helper/log.h"
#include "helper/types.h"
#include "rtos.h"
#include "rtos_standard_stackings.h"
#include "target/target.h"
#include "target/armv7m.h"
#include "target/arc.h"
#include "target/riscv/riscv.h"
#include "target/register.h"
#include "target/xtensa/xtensa.h"

#define UNIMPLEMENTED 0xFFFFFFFFU

/* ARC specific defines */
#define ARC_AUX_SEC_BUILD_REG 0xdb
#define ARC_REG_NUM 38

/* ARM specific defines */
#define ARM_XPSR_OFFSET 28

struct zephyr_thread {
	uint32_t ptr, next_ptr;
	uint32_t entry;
	uint32_t stack_pointer;
	uint8_t state;
	uint8_t user_options;
	int8_t prio;
	char name[64];
};

enum zephyr_offsets {
	OFFSET_VERSION,
	OFFSET_K_CURR_THREAD,
	OFFSET_K_THREADS,
	OFFSET_T_ENTRY,
	OFFSET_T_NEXT_THREAD,
	OFFSET_T_STATE,
	OFFSET_T_USER_OPTIONS,
	OFFSET_T_PRIO,
	OFFSET_T_STACK_POINTER,
	OFFSET_T_NAME,
	OFFSET_T_ARCH,
	OFFSET_T_PREEMPT_FLOAT,
	OFFSET_T_COOP_FLOAT,
	OFFSET_T_ARM_EXC_RETURN,
	OFFSET_MAX
};

struct zephyr_params {
	const char *target_name;
	uint8_t size_width;
	uint8_t pointer_width;
	uint32_t num_offsets;
	uint32_t offsets[OFFSET_MAX];
	const struct rtos_register_stacking *callee_saved_stacking;
	const struct rtos_register_stacking *cpu_saved_nofp_stacking;
	const struct rtos_register_stacking *cpu_saved_fp_stacking;
	int (*get_cpu_state)(struct rtos *rtos, target_addr_t *addr,
			struct zephyr_params *params,
			struct rtos_reg *callee_saved_reg_list,
			struct rtos_reg **reg_list, int *num_regs);
};

static const struct stack_register_offset arm_callee_saved[] = {
	{ ARMV7M_R13, 32, 32 },
	{ ARMV7M_R4,  0,  32 },
	{ ARMV7M_R5,  4,  32 },
	{ ARMV7M_R6,  8,  32 },
	{ ARMV7M_R7,  12, 32 },
	{ ARMV7M_R8,  16, 32 },
	{ ARMV7M_R9,  20, 32 },
	{ ARMV7M_R10, 24, 32 },
	{ ARMV7M_R11, 28, 32 },
};

static const struct stack_register_offset arc_callee_saved[] = {
	{ ARC_R13,  0,  32 },
	{ ARC_R14,  4,  32 },
	{ ARC_R15,  8,  32 },
	{ ARC_R16,  12,  32 },
	{ ARC_R17,  16,  32 },
	{ ARC_R18,  20,  32 },
	{ ARC_R19,  24,  32 },
	{ ARC_R20,  28,  32 },
	{ ARC_R21,  32,  32 },
	{ ARC_R22,  36,  32 },
	{ ARC_R23,  40,  32 },
	{ ARC_R24,  44,  32 },
	{ ARC_R25,  48,  32 },
	{ ARC_GP,  52,  32 },
	{ ARC_FP,  56,  32 },
	{ ARC_R30,  60,  32 }
};

/* RISC-V callee-saved register structure:
 * struct _callee_saved {
 *     unsigned long sp;    // Stack pointer, (x2 register) - offset 0
 *     unsigned long ra;    // Return address (x1) - offset 4
 *     unsigned long s0;    // Saved register/frame pointer (x8) - offset 8
 *     unsigned long s1;    // Saved register (x9) - offset 12
 *     unsigned long s2;    // Saved register (x18) - offset 16
 *     unsigned long s3;    // Saved register (x19) - offset 20
 *     unsigned long s4;    // Saved register (x20) - offset 24
 *     unsigned long s5;    // Saved register (x21) - offset 28
 *     unsigned long s6;    // Saved register (x22) - offset 32
 *     unsigned long s7;    // Saved register (x23) - offset 36
 *     unsigned long s8;    // Saved register (x24) - offset 40
 *     unsigned long s9;    // Saved register (x25) - offset 44
 *     unsigned long s10;   // Saved register (x26) - offset 48
 *     unsigned long s11;   // Saved register (x27) - offset 52
 * };
 * Total size: 56 bytes (14 registers × 4 bytes)
 */
static struct stack_register_offset riscv_callee_saved[] = {
	{ GDB_REGNO_ZERO, -1, 32 },  /* x0: Hard-wired zero */
	{ GDB_REGNO_RA,    4, 32 },  /* x1: Return address */
	{ GDB_REGNO_SP,    0, 32 },  /* x2: Stack pointer at offset 0 */
	{ GDB_REGNO_GP,   -1, 32 },  /* x3: Global pointer (not saved) */
	{ GDB_REGNO_TP,   -1, 32 },  /* x4: Thread pointer (not saved) */
	{ GDB_REGNO_T0,   -1, 32 },  /* x5: Temporary (caller-saved) */
	{ GDB_REGNO_T1,   -1, 32 },  /* x6: Temporary (caller-saved) */
	{ GDB_REGNO_T2,   -1, 32 },  /* x7: Temporary (caller-saved) */
	{ GDB_REGNO_FP,    8, 32 },  /* x8: s0/fp - Saved/Frame pointer */
	{ GDB_REGNO_S1,   12, 32 },  /* x9: Saved register */
	{ GDB_REGNO_A0,   -1, 32 },  /* x10: Argument/return (caller-saved) */
	{ GDB_REGNO_A1,   -1, 32 },  /* x11: Argument/return (caller-saved) */
	{ GDB_REGNO_A2,   -1, 32 },  /* x12: Argument (caller-saved) */
	{ GDB_REGNO_A3,   -1, 32 },  /* x13: Argument (caller-saved) */
	{ GDB_REGNO_A4,   -1, 32 },  /* x14: Argument (caller-saved) */
	{ GDB_REGNO_A5,   -1, 32 },  /* x15: Argument (caller-saved) */
	{ GDB_REGNO_A6,   -1, 32 },  /* x16: Argument (caller-saved) */
	{ GDB_REGNO_A7,   -1, 32 },  /* x17: Argument (caller-saved) */
	{ GDB_REGNO_S2,   16, 32 },  /* x18: Saved register */
	{ GDB_REGNO_S3,   20, 32 },  /* x19: Saved register */
	{ GDB_REGNO_S4,   24, 32 },  /* x20: Saved register */
	{ GDB_REGNO_S5,   28, 32 },  /* x21: Saved register */
	{ GDB_REGNO_S6,   32, 32 },  /* x22: Saved register */
	{ GDB_REGNO_S7,   36, 32 },  /* x23: Saved register */
	{ GDB_REGNO_S8,   40, 32 },  /* x24: Saved register */
	{ GDB_REGNO_S9,   44, 32 },  /* x25: Saved register */
	{ GDB_REGNO_S10,  48, 32 },  /* x26: Saved register */
	{ GDB_REGNO_S11,  52, 32 },  /* x27: Saved register */
	{ GDB_REGNO_T3,   -1, 32 },  /* x28: Temporary (caller-saved) */
	{ GDB_REGNO_T4,   -1, 32 },  /* x29: Temporary (caller-saved) */
	{ GDB_REGNO_T5,   -1, 32 },  /* x30: Temporary (caller-saved) */
	{ GDB_REGNO_T6,   -1, 32 },  /* x31: Temporary (caller-saved) */
	{ GDB_REGNO_PC,   -1, 32 },  /* PC: Set from RA */
};

static const struct rtos_register_stacking arm_callee_saved_stacking = {
	.stack_registers_size = 36,
	.stack_growth_direction = -1,
	.num_output_registers = ARRAY_SIZE(arm_callee_saved),
	.register_offsets = arm_callee_saved,
};

static const struct rtos_register_stacking arc_callee_saved_stacking = {
	.stack_registers_size = 64,
	.stack_growth_direction = -1,
	.num_output_registers = ARRAY_SIZE(arc_callee_saved),
	.register_offsets = arc_callee_saved,
};

static const struct rtos_register_stacking riscv_callee_saved_stacking = {
	.stack_registers_size = 56,
	.stack_growth_direction = -1,
	.num_output_registers = ARRAY_SIZE(riscv_callee_saved),
	.calculate_process_stack = rtos_generic_stack_align8,
	.register_offsets = riscv_callee_saved,
};

static const struct stack_register_offset arm_cpu_saved[] = {
	{ ARMV7M_R0,   0,  32 },
	{ ARMV7M_R1,   4,  32 },
	{ ARMV7M_R2,   8,  32 },
	{ ARMV7M_R3,   12, 32 },
	{ ARMV7M_R4,   -1, 32 },
	{ ARMV7M_R5,   -1, 32 },
	{ ARMV7M_R6,   -1, 32 },
	{ ARMV7M_R7,   -1, 32 },
	{ ARMV7M_R8,   -1, 32 },
	{ ARMV7M_R9,   -1, 32 },
	{ ARMV7M_R10,  -1, 32 },
	{ ARMV7M_R11,  -1, 32 },
	{ ARMV7M_R12,  16, 32 },
	{ ARMV7M_R13,  -2, 32 },
	{ ARMV7M_R14,  20, 32 },
	{ ARMV7M_PC,   24, 32 },
	{ ARMV7M_XPSR, 28, 32 },
};

static struct stack_register_offset arc_cpu_saved[] = {
	{ ARC_R0,		-1,  32 },
	{ ARC_R1,		-1,  32 },
	{ ARC_R2,		-1,  32 },
	{ ARC_R3,		-1,  32 },
	{ ARC_R4,		-1,  32 },
	{ ARC_R5,		-1,  32 },
	{ ARC_R6,		-1,  32 },
	{ ARC_R7,		-1,  32 },
	{ ARC_R8,		-1,  32 },
	{ ARC_R9,		-1,  32	},
	{ ARC_R10,		-1,  32	},
	{ ARC_R11,		-1,  32 },
	{ ARC_R12,		-1,  32 },
	{ ARC_R13,		-1,  32 },
	{ ARC_R14,		-1,  32 },
	{ ARC_R15,		-1,  32 },
	{ ARC_R16,		-1,  32 },
	{ ARC_R17,		-1,  32 },
	{ ARC_R18,		-1,  32 },
	{ ARC_R19,		-1,  32 },
	{ ARC_R20,		-1,  32 },
	{ ARC_R21,		-1,  32 },
	{ ARC_R22,		-1,  32 },
	{ ARC_R23,		-1,  32 },
	{ ARC_R24,		-1,  32 },
	{ ARC_R25,		-1,  32 },
	{ ARC_GP,		-1,  32 },
	{ ARC_FP,		-1,  32 },
	{ ARC_SP,		-1,  32 },
	{ ARC_ILINK,		-1,  32 },
	{ ARC_R30,		-1,  32 },
	{ ARC_BLINK,		 0,  32 },
	{ ARC_LP_COUNT,		-1,  32 },
	{ ARC_PCL,		-1,  32 },
	{ ARC_PC,		-1,  32 },
	{ ARC_LP_START,		-1,  32 },
	{ ARC_LP_END,		-1,  32 },
	{ ARC_STATUS32,		 4,  32 }
};


enum zephyr_symbol_values {
	ZEPHYR_VAL__KERNEL,
	ZEPHYR_VAL__KERNEL_OPENOCD_OFFSETS,
	ZEPHYR_VAL__KERNEL_OPENOCD_SIZE_T_SIZE,
	ZEPHYR_VAL__KERNEL_OPENOCD_NUM_OFFSETS,
	ZEPHYR_VAL__KERNEL_XT_PS_OFFSET, /* optional */
	ZEPHYR_VAL__KERNEL_XT_BSA_SIZEOF, /* optional */
	ZEPHYR_VAL_COUNT
};

static target_addr_t zephyr_cortex_m_stack_align(struct target *target,
		const uint8_t *stack_data,
		const struct rtos_register_stacking *stacking, target_addr_t stack_ptr)
{
	return rtos_cortex_m_stack_align(target, stack_data, stacking,
			stack_ptr, ARM_XPSR_OFFSET);
}

static const struct rtos_register_stacking arm_cpu_saved_nofp_stacking = {
	.stack_registers_size = 32,
	.stack_growth_direction = -1,
	.num_output_registers = ARRAY_SIZE(arm_cpu_saved),
	.calculate_process_stack = zephyr_cortex_m_stack_align,
	.register_offsets = arm_cpu_saved,
};

static const struct rtos_register_stacking arm_cpu_saved_fp_stacking = {
	.stack_registers_size = 32 + 18 * 4,
	.stack_growth_direction = -1,
	.num_output_registers = ARRAY_SIZE(arm_cpu_saved),
	.calculate_process_stack = zephyr_cortex_m_stack_align,
	.register_offsets = arm_cpu_saved,
};

/* stack_registers_size is 8 because besides caller registers
 * there are only blink and Status32 registers on stack left */
static struct rtos_register_stacking arc_cpu_saved_stacking = {
	.stack_registers_size = 8,
	.stack_growth_direction = -1,
	.num_output_registers = ARRAY_SIZE(arc_cpu_saved),
	.register_offsets = arc_cpu_saved,
};

/* ARCv2 specific implementation */
static int zephyr_get_arc_state(struct rtos *rtos, target_addr_t *addr,
			 struct zephyr_params *params,
			 struct rtos_reg *callee_saved_reg_list,
			 struct rtos_reg **reg_list, int *num_regs)
{

	uint32_t real_stack_addr;
	int retval = 0;
	int num_callee_saved_regs;
	const struct rtos_register_stacking *stacking;

	*addr -= params->callee_saved_stacking->register_offsets[0].offset;

	/* Getting real stack address from Kernel thread struct */
	retval = target_read_u32(rtos->target, *addr, &real_stack_addr);
	if (retval != ERROR_OK)
		return retval;

	/* Getting callee registers */
	retval = rtos_generic_stack_read(rtos->target,
			params->callee_saved_stacking,
			real_stack_addr, &callee_saved_reg_list,
			&num_callee_saved_regs);
	if (retval != ERROR_OK)
		return retval;

	stacking = params->cpu_saved_nofp_stacking;

	/* Getting blink and status32 registers */
	retval = rtos_generic_stack_read(rtos->target, stacking,
			real_stack_addr + num_callee_saved_regs * 4,
			reg_list, num_regs);
	if (retval != ERROR_OK)
		return retval;

	for (int i = 0; i < num_callee_saved_regs; i++)
		buf_cpy(callee_saved_reg_list[i].value,
			(*reg_list)[callee_saved_reg_list[i].number].value,
			callee_saved_reg_list[i].size);

	/* The blink, sp, pc offsets in arc_cpu_saved structure may be changed,
	 * but the registers number shall not. So the next code searches the
	 * offsetst of these registers in arc_cpu_saved structure. */
	unsigned short blink_offset = 0, pc_offset = 0, sp_offset = 0;
	for (size_t i = 0; i < ARRAY_SIZE(arc_cpu_saved); i++) {
		if (arc_cpu_saved[i].number == ARC_BLINK)
			blink_offset = i;
		if (arc_cpu_saved[i].number == ARC_SP)
			sp_offset = i;
		if (arc_cpu_saved[i].number == ARC_PC)
			pc_offset = i;
	}

	if (blink_offset == 0 || sp_offset == 0 || pc_offset == 0) {
		LOG_ERROR("Basic registers offsets are missing, check <arc_cpu_saved> struct");
		return ERROR_FAIL;
	}

	/* Put blink value into PC */
	buf_cpy((*reg_list)[blink_offset].value,
		(*reg_list)[pc_offset].value, sizeof((*reg_list)[blink_offset].value));

	/* Put address after callee/caller in SP. */
	int64_t stack_top;

	stack_top = real_stack_addr + num_callee_saved_regs * 4
			+ arc_cpu_saved_stacking.stack_registers_size;
	buf_cpy(&stack_top, (*reg_list)[sp_offset].value, sizeof(stack_top));

	return retval;
}

/* ARM Cortex-M-specific implementation */
static int zephyr_get_arm_state(struct rtos *rtos, target_addr_t *addr,
			 struct zephyr_params *params,
			 struct rtos_reg *callee_saved_reg_list,
			 struct rtos_reg **reg_list, int *num_regs)
{

	int retval = 0;
	int num_callee_saved_regs;
	const struct rtos_register_stacking *stacking;

	*addr -= params->callee_saved_stacking->register_offsets[0].offset;

	retval = rtos_generic_stack_read(rtos->target,
			params->callee_saved_stacking,
			*addr, &callee_saved_reg_list,
			&num_callee_saved_regs);
	if (retval != ERROR_OK)
		return retval;

	*addr = target_buffer_get_u32(rtos->target,
			callee_saved_reg_list[0].value);

	if (params->offsets[OFFSET_T_PREEMPT_FLOAT] != UNIMPLEMENTED)
		stacking = params->cpu_saved_fp_stacking;
	else
		stacking = params->cpu_saved_nofp_stacking;

	retval = rtos_generic_stack_read(rtos->target, stacking, *addr, reg_list,
			num_regs);
	if (retval != ERROR_OK)
		return retval;

	for (int i = 1; i < num_callee_saved_regs; i++)
		buf_cpy(callee_saved_reg_list[i].value,
			(*reg_list)[callee_saved_reg_list[i].number].value,
			callee_saved_reg_list[i].size);
	return 0;
}

/* RISC-V specific implementation
 *
 * The callee-saved registers are stored in the thread's callee_saved structure:
 *   - SP, RA, S0-S11
 *
 * The 'addr' parameter points to thread->callee_saved.sp:
 *   - Calculated as: thread_base + OFFSET_T_STACK_POINTER
 *   - OFFSET_T_STACK_POINTER = offsetof(struct k_thread, callee_saved.sp)
 *     This offset is provided by Zephyr's thread_info.c
 *
 * For suspended threads:
 *   - PC is set to RA (where the thread will resume)
 *   - SP, RA, S0-S11 are read from callee_saved structure
 *   - Caller-saved registers (T0-T6, A0-A7) are unavailable
 */
static int zephyr_get_riscv_state(struct rtos *rtos, target_addr_t *addr,
			 struct zephyr_params *params,
			 struct rtos_reg *callee_saved_reg_list,
			 struct rtos_reg **reg_list, int *num_regs)
{
	int retval;
	uint32_t ra_value;

	LOG_DEBUG("Zephyr RISC-V: Reading callee_saved from 0x%" PRIx64, *addr);

	/* Read all callee-saved registers from the thread's callee_saved structure.
	 * The riscv_callee_saved array maps GDB register numbers to offsets in
	 * the callee_saved structure. This extracts SP, RA, S0-S11. */
	retval = rtos_generic_stack_read(rtos->target,
			params->callee_saved_stacking,
			*addr, reg_list,
			num_regs);
	if (retval != ERROR_OK) {
		LOG_ERROR("Zephyr RISC-V: Failed to read callee_saved registers");
		return retval;
	}

	/* For a suspended thread, the PC should be set to RA (return address).
	 * This is where the thread will resume execution when it's switched back in.
	 * Find the RA register value and copy it to PC. */
	ra_value = 0;
	for (int i = 0; i < *num_regs; i++) {
		if ((*reg_list)[i].number == GDB_REGNO_RA) {
			ra_value = target_buffer_get_u32(rtos->target, (*reg_list)[i].value);
			LOG_DEBUG("Zephyr RISC-V: RA = 0x%08" PRIx32, ra_value);
			break;
		}
	}

	/* Set PC = RA for proper backtrace in GDB */
	for (int i = 0; i < *num_regs; i++) {
		if ((*reg_list)[i].number == GDB_REGNO_PC) {
			target_buffer_set_u32(rtos->target, (*reg_list)[i].value, ra_value);
			LOG_DEBUG("Zephyr RISC-V: Set PC to 0x%08" PRIx32, ra_value);
			break;
		}
	}

	return ERROR_OK;
}

/* Xtensa specific implementation
 *
 * The saved registers are stored in the Base Save Area (BSA) structure on the stack:
 *   - PC, PS, A0-A3, SAR, LBEG, LEND, LCOUNT, and other special registers
 *   - Caller's A0-A3 spill slots are at the end of BSA (BSA_SIZE - 16, -12, -8, -4)
 *
 * The 'addr' parameter points to thread->switch_handle:
 *   - Calculated as: thread_base + OFFSET_T_SWITCH_HANDLE
 *   - OFFSET_T_SWITCH_HANDLE = offsetof(struct k_thread, switch_handle)
 *
 * Double indirection to access BSA:
 *   - thread->switch_handle contains address of ptr_to_bsa (points to BSA)
 *   - Read *ptr_to_bsa to get bsa_addr
 *   - BSA structure is located at bsa_addr
 *
 * For suspended threads:
 *   - PC, PS are read from BSA at offsets provided by Zephyr symbols
 *   - Caller's A0-A3 are read from end of BSA (spill slots)
 *   - BSA size and register offsets are read dynamically from Zephyr ELF symbols
 *   - High registers (A4-A15) are saved above BSA
 */
static int zephyr_get_xtensa_state(struct rtos *rtos, target_addr_t *addr,
			 struct zephyr_params *params,
			 struct rtos_reg *callee_saved_reg_list,
			 struct rtos_reg **reg_list, int *num_regs)
{
	uint32_t ptr_to_bsa, ptr_to_bsa_value;

	/* Read address of ptr_to_bsa from thread->switch_handle */
	int retval = target_read_u32(rtos->target, *addr, &ptr_to_bsa);
	if (retval != ERROR_OK)
		return retval;

	/* Read *ptr_to_bsa: contains BSA address (if thread not switched) or final SP (if switched) */
	retval = target_read_u32(rtos->target, ptr_to_bsa, &ptr_to_bsa_value);
	if (retval != ERROR_OK)
		return retval;

	LOG_DEBUG("Zephyr Xtensa: ptr_to_bsa = 0x%08" PRIx32, ptr_to_bsa);
	LOG_DEBUG("Zephyr Xtensa: ptr_to_bsa_value = 0x%08" PRIx32, ptr_to_bsa_value);

	uint32_t bsa_addr = ptr_to_bsa_value;

	/* Read configured BSA offsets from Zephyr symbols */
	uint32_t bsa_size = 0;
	uint32_t ps_offset = 0;

	if (rtos->symbols && rtos->symbols[ZEPHYR_VAL__KERNEL_XT_BSA_SIZEOF].address != 0) {
		bsa_size = rtos->symbols[ZEPHYR_VAL__KERNEL_XT_BSA_SIZEOF].address;
		LOG_DEBUG("Zephyr Xtensa: Symbol ___xtensa_irq_bsa_t_SIZEOF value = %d", bsa_size);
	} else {
		LOG_ERROR("Zephyr Xtensa: Symbol ___xtensa_irq_bsa_t_SIZEOF not found or address is 0");
		return ERROR_FAIL;
	}

	if (rtos->symbols && rtos->symbols[ZEPHYR_VAL__KERNEL_XT_PS_OFFSET].address != 0) {
		ps_offset = rtos->symbols[ZEPHYR_VAL__KERNEL_XT_PS_OFFSET].address;
		LOG_DEBUG("Zephyr Xtensa: Symbol ___xtensa_irq_bsa_t_ps_OFFSET value = %d", ps_offset);
	} else {
		LOG_ERROR("Zephyr Xtensa: Symbol ___xtensa_irq_bsa_t_ps_OFFSET not found or address is 0");
		return ERROR_FAIL;
	}

	/* Sanity check */
	if (bsa_size <= 0 || ps_offset >= bsa_size) {
		LOG_ERROR("Zephyr Xtensa: Invalid BSA size or PS offset");
		return ERROR_FAIL;
	}

	/* Read all registers from BSA */
	uint8_t bsa_data[bsa_size];
	retval = target_read_buffer(rtos->target, bsa_addr, bsa_size, bsa_data);
	if (retval != ERROR_OK) {
		LOG_ERROR("Zephyr Xtensa: Failed to read BSA");
		return retval;
	}

	/* Read PS, PC, and caller's A0-A3 from BSA directly into array */
	uint32_t reg_values[6];
	reg_values[0] = target_buffer_get_u32(rtos->target, &bsa_data[ps_offset + 0]); /* PS */
	reg_values[1] = target_buffer_get_u32(rtos->target, &bsa_data[ps_offset + 4]); /* PC */
	reg_values[2] = target_buffer_get_u32(rtos->target, &bsa_data[bsa_size - 16]); /* caller_a0 */
	reg_values[3] = target_buffer_get_u32(rtos->target, &bsa_data[bsa_size - 12]); /* caller_a1 */
	reg_values[4] = target_buffer_get_u32(rtos->target, &bsa_data[bsa_size - 8]);  /* caller_a2 */
	reg_values[5] = target_buffer_get_u32(rtos->target, &bsa_data[bsa_size - 4]);  /* caller_a3 */

	struct xtensa *xtensa = target_to_xtensa(rtos->target);
	LOG_DEBUG("Zephyr Xtensa: num_output_registers = %u", xtensa->genpkt_regs_num);

	/* Allocate register list directly - no need for rtos_generic_stack_read since
	 * all offsets are -1 and we already have bsa_data */
	*num_regs = xtensa->genpkt_regs_num;
	*reg_list = calloc(*num_regs, sizeof(struct rtos_reg));
	if (!*reg_list) {
		LOG_ERROR("Zephyr Xtensa: Failed to allocate register list");
		return ERROR_FAIL;
	}

	/* Initialize register numbers and sizes */
	for (int i = 0; i < *num_regs; i++) {
		(*reg_list)[i].number = i;
		(*reg_list)[i].size = 32;  /* All Xtensa registers are 32-bit */
	}

	/* Update reg_list with PC, PS, and caller's A0-A3 from BSA
	 * Registers A0-A3 are sequential, so we can calculate their numbers from A0
	 */
	struct reg *reg_ps = register_get_by_name(rtos->target->reg_cache, "ps", 1);
	struct reg *reg_pc = register_get_by_name(rtos->target->reg_cache, "pc", 1);
	struct reg *reg_a0 = register_get_by_name(rtos->target->reg_cache, "ar0", 1);
	uint32_t reg_numbers[6];

	reg_numbers[0] = reg_ps->number;
	reg_numbers[1] = reg_pc->number;
	reg_numbers[2] = reg_a0->number;
	reg_numbers[3] = reg_a0->number + 1;
	reg_numbers[4] = reg_a0->number + 2;
	reg_numbers[5] = reg_a0->number + 3;

	/* Update reg_list with values from BSA */
	for (int i = 0; i < *num_regs; i++) {
		for (int j = 0; j < 6; j++) {
			if ((*reg_list)[i].number == reg_numbers[j]) {
				struct reg *reg = register_get_by_number(rtos->target->reg_cache, reg_numbers[j], true);
				target_buffer_set_u32(rtos->target, (*reg_list)[i].value, reg_values[j]);
				LOG_DEBUG("Zephyr Xtensa: %s = 0x%08" PRIx32, reg->name, reg_values[j]);
				break;
			}
		}
	}

	/* Read high registers (A4-A15) from stack if spilled
	 * Check which quads are saved by reading marker values at known offsets
	 */
	uint8_t high_regs_data[48] = {0};  /* A4-A15: 12 registers */
	uint8_t quads_saved = 0;  /* Bitmask */

	/* Check A4-A7 quad: read marker at bsa_addr - 4 (A7 location) */
	uint32_t marker;
	retval = target_read_u32(rtos->target, bsa_addr - 4, &marker);
	if (retval == ERROR_OK && marker != bsa_addr) {
		retval = target_read_buffer(rtos->target, bsa_addr - 16, 16, &high_regs_data[0]);
		if (retval == ERROR_OK)
			quads_saved |= 0x01;  /* A4-A7 saved */
	}

	/* Check A8-A11 quad: read marker at bsa_addr - 20 (A11 location) */
	if (quads_saved & 0x01) {
		retval = target_read_u32(rtos->target, bsa_addr - 20, &marker);
		if (retval == ERROR_OK && marker != bsa_addr) {
			retval = target_read_buffer(rtos->target, bsa_addr - 32, 16, &high_regs_data[16]);
			if (retval == ERROR_OK)
				quads_saved |= 0x02;  /* A8-A11 saved */
		}
	}

	/* Check A12-A15 quad: read marker at bsa_addr - 36 (A15 location) */
	if (quads_saved & 0x02) {
		retval = target_read_u32(rtos->target, bsa_addr - 36, &marker);
		if (retval == ERROR_OK && marker != bsa_addr) {
			retval = target_read_buffer(rtos->target, bsa_addr - 48, 16, &high_regs_data[32]);
			if (retval == ERROR_OK)
				quads_saved |= 0x04;  /* A12-A15 saved */

			/* Sanity check: verify end marker at bsa_addr - 52 */
			if (retval == ERROR_OK) {
				retval = target_read_u32(rtos->target, bsa_addr - 52, &marker);
				if (retval == ERROR_OK && marker != bsa_addr) {
					LOG_WARNING("Zephyr Xtensa: High registers end marker mismatch!");
					/* Reset quads_saved to 0 to indicate no quads were saved */
					quads_saved = 0;
				}
			}
		}
	}

	/* Update reg_list with high register values
	 * Registers A4-A15 are sequential, so we can calculate their numbers from A4
	 */
	if (quads_saved != 0) {
		if (reg_a0) {
			/* A0-A15 are sequential, calculate register numbers from A0 */
			for (int i = 0; i < 12; i++) {
				/* Check if this register quad was saved */
				int quad_bit = 1 << (i / 4);
				if (!(quads_saved & quad_bit))
					break;

				uint32_t reg_num = reg_a0->number + 4 + i;
				/* Find and update register in reg_list */
				for (int r = 0; r < *num_regs; r++) {
					if ((*reg_list)[r].number == reg_num) {
						struct reg *reg = register_get_by_number(rtos->target->reg_cache, reg_num, true);
						uint32_t value = target_buffer_get_u32(rtos->target, &high_regs_data[i * 4]);
						target_buffer_set_u32(rtos->target, (*reg_list)[r].value, value);
						LOG_DEBUG("Zephyr Xtensa: %s = 0x%08" PRIx32, reg->name, value);
						break;
					}
				}
			}
		}
	}


	return ERROR_OK;
}

static struct zephyr_params zephyr_params_list[] = {
	{
		.target_name = "cortex_m",
		.pointer_width = 4,
		.callee_saved_stacking = &arm_callee_saved_stacking,
		.cpu_saved_nofp_stacking = &arm_cpu_saved_nofp_stacking,
		.cpu_saved_fp_stacking = &arm_cpu_saved_fp_stacking,
		.get_cpu_state = &zephyr_get_arm_state,
	},
	{
		.target_name = "cortex_r4",
		.pointer_width = 4,
		.callee_saved_stacking = &arm_callee_saved_stacking,
		.cpu_saved_nofp_stacking = &arm_cpu_saved_nofp_stacking,
		.cpu_saved_fp_stacking = &arm_cpu_saved_fp_stacking,
		.get_cpu_state = &zephyr_get_arm_state,
	},
	{
		.target_name = "hla_target",
		.pointer_width = 4,
		.callee_saved_stacking = &arm_callee_saved_stacking,
		.cpu_saved_nofp_stacking = &arm_cpu_saved_nofp_stacking,
		.cpu_saved_fp_stacking = &arm_cpu_saved_fp_stacking,
		.get_cpu_state = &zephyr_get_arm_state,

	},
	{
		.target_name = "arcv2",
		.pointer_width = 4,
		.callee_saved_stacking = &arc_callee_saved_stacking,
		.cpu_saved_nofp_stacking = &arc_cpu_saved_stacking,
		.get_cpu_state = &zephyr_get_arc_state,
	},
	{
		.target_name = "esp32",
		.pointer_width = 4,
		.callee_saved_stacking = NULL,
		.get_cpu_state = &zephyr_get_xtensa_state,
	},
	{
		.target_name = "esp32s2",
		.pointer_width = 4,
		.callee_saved_stacking = NULL,
		.get_cpu_state = &zephyr_get_xtensa_state,
	},
	{
		.target_name = "esp32s3",
		.pointer_width = 4,
		.callee_saved_stacking = NULL,
		.get_cpu_state = &zephyr_get_xtensa_state,
	},
	{
		.target_name = "esp32c2",
		.pointer_width = 4,
		.callee_saved_stacking = &riscv_callee_saved_stacking,
		.get_cpu_state = &zephyr_get_riscv_state,
	},
	{
		.target_name = "esp32c3",
		.pointer_width = 4,
		.callee_saved_stacking = &riscv_callee_saved_stacking,
		.get_cpu_state = &zephyr_get_riscv_state,
	},
	{
		.target_name = "esp32c6",
		.pointer_width = 4,
		.callee_saved_stacking = &riscv_callee_saved_stacking,
		.get_cpu_state = &zephyr_get_riscv_state,
	},
	{
		.target_name = "esp32h2",
		.pointer_width = 4,
		.callee_saved_stacking = &riscv_callee_saved_stacking,
		.get_cpu_state = &zephyr_get_riscv_state,
	},
	{
		.target_name = "esp32c5",
		.pointer_width = 4,
		.callee_saved_stacking = &riscv_callee_saved_stacking,
		.get_cpu_state = &zephyr_get_riscv_state,
	},
	{
		.target_name = "esp32c61",
		.pointer_width = 4,
		.callee_saved_stacking = &riscv_callee_saved_stacking,
		.get_cpu_state = &zephyr_get_riscv_state,
	},
	{
		.target_name = "esp32p4",
		.pointer_width = 4,
		.callee_saved_stacking = &riscv_callee_saved_stacking,
		.get_cpu_state = &zephyr_get_riscv_state,
	},
	{
		.target_name = "esp32h4",
		.pointer_width = 4,
		.callee_saved_stacking = &riscv_callee_saved_stacking,
		.get_cpu_state = &zephyr_get_riscv_state,
	},
	{
		.target_name = NULL
	}
};

static const struct symbol_table_elem zephyr_symbol_list[] = {
	{
		.symbol_name = "_kernel",
		.optional = false
	},
	{
		.symbol_name = "_kernel_thread_info_offsets",
		.optional = false
	},
	{
		.symbol_name = "_kernel_thread_info_size_t_size",
		.optional = false
	},
	{
		.symbol_name = "_kernel_thread_info_num_offsets",
		.optional = true
	},
	{
		.symbol_name = "___xtensa_irq_bsa_t_ps_OFFSET",
		.optional = true
	},
	{
		.symbol_name = "___xtensa_irq_bsa_t_SIZEOF",
		.optional = true
	},
	{
		.symbol_name = NULL
	}
};

static bool zephyr_detect_rtos(struct target *target)
{
	if (!target->rtos->symbols) {
		LOG_DEBUG("Zephyr: no symbols while detecting RTOS");
		return false;
	}

	for (enum zephyr_symbol_values symbol = ZEPHYR_VAL__KERNEL;
					symbol != ZEPHYR_VAL_COUNT; symbol++) {
		LOG_INFO("Zephyr: does it have symbol %d (%s)?", symbol,
			target->rtos->symbols[symbol].optional ? "optional" : "mandatory");

		if (target->rtos->symbols[symbol].optional)
			continue;
		if (target->rtos->symbols[symbol].address == 0)
			return false;
	}

	LOG_INFO("Zephyr: all mandatory symbols found");

	return true;
}

static int zephyr_create(struct target *target)
{
	const char *name;

	name = target_type_name(target);

	LOG_INFO("Zephyr: looking for target: %s", name);

	/* ARC specific, check if EM target has security subsystem
	 * In case of ARC_HAS_SECURE zephyr option enabled
	 * the thread stack contains blink,sec_stat,status32 register
	 * values. If ARC_HAS_SECURE is disabled, only blink and status32
	 * register values are saved on stack. */
	if (!strcmp(name, "arcv2")) {
		uint32_t value;
		struct arc_common *arc = target_to_arc(target);
		/* Reading SEC_BUILD bcr */
		CHECK_RETVAL(arc_jtag_read_aux_reg_one(&arc->jtag_info, ARC_AUX_SEC_BUILD_REG, &value));
		if (value != 0) {
			LOG_DEBUG("ARC EM board has security subsystem, changing offsets");
			arc_cpu_saved[ARC_REG_NUM - 1].offset = 8;
			/* After reading callee registers in stack
			 * now blink,sec_stat,status32 registers
			 * are located. */
			arc_cpu_saved_stacking.stack_registers_size = 12;
		}
	}

	for (struct zephyr_params *p = zephyr_params_list; p->target_name; p++) {
		if (!strcmp(p->target_name, name)) {
			LOG_INFO("Zephyr: target known, params at %p", p);
			target->rtos->rtos_specific_params = p;
			return ERROR_OK;
		}
	}

	target->rtos->current_threadid = -1;

	LOG_ERROR("Could not find target in Zephyr compatibility list");
	return ERROR_FAIL;
}

struct zephyr_array {
	void *ptr;
	size_t elements;
};

static void zephyr_array_init(struct zephyr_array *array)
{
	array->ptr = NULL;
	array->elements = 0;
}

static void zephyr_array_free(struct zephyr_array *array)
{
	free(array->ptr);
	zephyr_array_init(array);
}

static void *zephyr_array_append(struct zephyr_array *array, size_t size)
{
	if (!(array->elements % 16)) {
		void *ptr = realloc(array->ptr, (array->elements + 16) * size);

		if (!ptr) {
			LOG_ERROR("Out of memory");
			return NULL;
		}

		array->ptr = ptr;
	}

	return (unsigned char *)array->ptr + (array->elements++) * size;
}

static void *zephyr_array_detach_ptr(struct zephyr_array *array)
{
	void *ptr = array->ptr;

	zephyr_array_init(array);

	return ptr;
}

static uint32_t zephyr_kptr(const struct rtos *rtos, enum zephyr_offsets off)
{
	const struct zephyr_params *params = rtos->rtos_specific_params;

	return rtos->symbols[ZEPHYR_VAL__KERNEL].address + params->offsets[off];
}

static int zephyr_fetch_thread(const struct rtos *rtos,
				struct zephyr_thread *thread, uint32_t ptr)
{
	const struct zephyr_params *param = rtos->rtos_specific_params;
	int retval;

	thread->ptr = ptr;

	retval = target_read_u32(rtos->target, ptr + param->offsets[OFFSET_T_ENTRY],
				 &thread->entry);
	if (retval != ERROR_OK)
		return retval;

	retval = target_read_u32(rtos->target,
				 ptr + param->offsets[OFFSET_T_NEXT_THREAD],
				 &thread->next_ptr);
	if (retval != ERROR_OK)
		return retval;

	if (param->offsets[OFFSET_T_STACK_POINTER] != UNIMPLEMENTED) {
		retval = target_read_u32(rtos->target,
					ptr + param->offsets[OFFSET_T_STACK_POINTER],
					&thread->stack_pointer);
		if (retval != ERROR_OK)
			return retval;
	}

	retval = target_read_u8(rtos->target, ptr + param->offsets[OFFSET_T_STATE],
				&thread->state);
	if (retval != ERROR_OK)
		return retval;

	retval = target_read_u8(rtos->target,
				ptr + param->offsets[OFFSET_T_USER_OPTIONS],
				&thread->user_options);
	if (retval != ERROR_OK)
		return retval;

	uint8_t prio;
	retval = target_read_u8(rtos->target,
				ptr + param->offsets[OFFSET_T_PRIO], &prio);
	if (retval != ERROR_OK)
		return retval;
	thread->prio = prio;

	thread->name[0] = '\0';
	if (param->offsets[OFFSET_T_NAME] != UNIMPLEMENTED) {
		retval = target_read_buffer(rtos->target,
					ptr + param->offsets[OFFSET_T_NAME],
					sizeof(thread->name) - 1, (uint8_t *)thread->name);
		if (retval != ERROR_OK)
			return retval;

		thread->name[sizeof(thread->name) - 1] = '\0';
	}

	LOG_DEBUG("Fetched thread%" PRIx32 ": {entry@0x%" PRIx32
		", state=%" PRIu8 ", useropts=%" PRIu8 ", prio=%" PRId8 ", name=%s}",
		ptr, thread->entry, thread->state, thread->user_options, thread->prio, thread->name);

	return ERROR_OK;
}

static int zephyr_fetch_thread_list(struct rtos *rtos, uint32_t current_thread)
{
	struct zephyr_array thread_array;
	struct zephyr_thread thread;
	struct thread_detail *td;
	uint32_t curr;
	int retval;

	retval = target_read_u32(rtos->target, zephyr_kptr(rtos, OFFSET_K_THREADS),
		&curr);
	if (retval != ERROR_OK) {
		LOG_ERROR("Could not fetch current thread pointer");
		return retval;
	}

	zephyr_array_init(&thread_array);

	for (; curr; curr = thread.next_ptr) {
		retval = zephyr_fetch_thread(rtos, &thread, curr);
		if (retval != ERROR_OK)
			goto error;

		td = zephyr_array_append(&thread_array, sizeof(*td));
		if (!td)
			goto error;

		td->threadid = thread.ptr;
		td->exists = true;

		if (thread.name[0])
			td->thread_name_str = strdup(thread.name);
		else
			td->thread_name_str = alloc_printf("thr_%" PRIx32 "_%" PRIx32,
							   thread.entry, thread.ptr);
		td->extra_info_str = alloc_printf("prio:%" PRId8 ",useropts:%" PRIu8,
						  thread.prio, thread.user_options);
		if (!td->thread_name_str || !td->extra_info_str)
			goto error;

	}

	LOG_DEBUG("Got information for %zu threads", thread_array.elements);

	rtos_free_threadlist(rtos);

	rtos->thread_count = (int)thread_array.elements;
	rtos->thread_details = zephyr_array_detach_ptr(&thread_array);

	rtos->current_thread = current_thread;

	return ERROR_OK;

error:
	td = thread_array.ptr;
	for (size_t i = 0; i < thread_array.elements; i++) {
		free(td[i].thread_name_str);
		free(td[i].extra_info_str);
	}

	zephyr_array_free(&thread_array);

	return ERROR_FAIL;
}

static int zephyr_update_threads(struct rtos *rtos)
{
	struct zephyr_params *param;
	int retval;

	if (!rtos->rtos_specific_params)
		return ERROR_FAIL;

	param = (struct zephyr_params *)rtos->rtos_specific_params;

	if (!rtos->symbols) {
		LOG_WARNING("No symbols for Zephyr");
		return ERROR_FAIL;
	}

	if (rtos->symbols[ZEPHYR_VAL__KERNEL].address == 0) {
		LOG_ERROR("Can't obtain kernel struct from Zephyr");
		return ERROR_FAIL;
	}

	if (rtos->symbols[ZEPHYR_VAL__KERNEL_OPENOCD_OFFSETS].address == 0) {
		LOG_ERROR("Please build Zephyr with CONFIG_DEBUG_THREAD_INFO option set");
		return ERROR_FAIL;
	}

	retval = target_read_u8(rtos->target,
		rtos->symbols[ZEPHYR_VAL__KERNEL_OPENOCD_SIZE_T_SIZE].address,
		&param->size_width);
	if (retval != ERROR_OK) {
		LOG_ERROR("Couldn't determine size of size_t from host");
		return retval;
	}

	if (param->size_width != 4) {
		LOG_ERROR("Only size_t of 4 bytes are supported. Read with (%d)", param->size_width);
		return ERROR_FAIL;
	}

	if (rtos->symbols[ZEPHYR_VAL__KERNEL_OPENOCD_NUM_OFFSETS].address) {
		retval = target_read_u32(rtos->target,
				rtos->symbols[ZEPHYR_VAL__KERNEL_OPENOCD_NUM_OFFSETS].address,
				&param->num_offsets);
		if (retval != ERROR_OK) {
			LOG_ERROR("Couldn't not fetch number of offsets from Zephyr");
			return retval;
		}

		if (param->num_offsets <= OFFSET_T_STACK_POINTER) {
			LOG_ERROR("Number of offsets too small");
			return ERROR_FAIL;
		}
	} else {
		retval = target_read_u32(rtos->target,
				rtos->symbols[ZEPHYR_VAL__KERNEL_OPENOCD_OFFSETS].address,
				&param->offsets[OFFSET_VERSION]);
		if (retval != ERROR_OK) {
			LOG_ERROR("Couldn't not fetch offsets from Zephyr");
			return retval;
		}

		LOG_DEBUG("Zephyr OpenOCD support version %" PRId32,
			param->offsets[OFFSET_VERSION]);

		if (param->offsets[OFFSET_VERSION] > 1) {
			LOG_ERROR("Unexpected OpenOCD support version %" PRIu32,
					param->offsets[OFFSET_VERSION]);
			return ERROR_FAIL;
		}
		switch (param->offsets[OFFSET_VERSION]) {
		case 0:
			param->num_offsets = OFFSET_T_STACK_POINTER + 1;
			break;
		case 1:
			param->num_offsets = OFFSET_T_COOP_FLOAT + 1;
			break;
		}
	}
	/* We can fetch the whole array for version 0, as they're supposed
	 * to grow only */
	uint32_t address;
	address  = rtos->symbols[ZEPHYR_VAL__KERNEL_OPENOCD_OFFSETS].address;
	for (size_t i = 0; i < OFFSET_MAX; i++, address += param->size_width) {
		if (i >= param->num_offsets) {
			param->offsets[i] = UNIMPLEMENTED;
			continue;
		}

		retval = target_read_u32(rtos->target, address, &param->offsets[i]);
		if (retval != ERROR_OK) {
			LOG_ERROR("Could not fetch offsets from Zephyr");
			return ERROR_FAIL;
		}
	}

	LOG_DEBUG("Zephyr OpenOCD support version %" PRId32,
			  param->offsets[OFFSET_VERSION]);

	LOG_DEBUG("Zephyr OpenOCD stack offset %" PRId32,
			  param->offsets[OFFSET_T_STACK_POINTER]);

	if (param->offsets[OFFSET_T_STACK_POINTER] == UNIMPLEMENTED) {
		LOG_ERROR("Stack pointer offset is not implemented");
		return ERROR_FAIL;
	}

	uint32_t current_thread;
	retval = target_read_u32(rtos->target,
		zephyr_kptr(rtos, OFFSET_K_CURR_THREAD), &current_thread);
	if (retval != ERROR_OK) {
		LOG_ERROR("Could not obtain current thread ID");
		return retval;
	}

	LOG_DEBUG("Zephyr: Current thread ID 0x%" PRIx32, current_thread);

	retval = zephyr_fetch_thread_list(rtos, current_thread);
	if (retval != ERROR_OK) {
		LOG_ERROR("Could not obtain thread list");
		return retval;
	}
	return ERROR_OK;
}

static int zephyr_get_current_thread_reg_list(struct rtos *rtos,
	struct rtos_reg **reg_list, int *num_regs)
{
	struct reg **gdb_reg_list;

	/* Registers for currently running thread are not on task's stack and
	 * should be retrieved from reg caches via target_get_gdb_reg_list */
	int ret = target_get_gdb_reg_list(rtos->target, &gdb_reg_list, num_regs,
		REG_CLASS_GENERAL);
	if (ret != ERROR_OK) {
		LOG_ERROR("target_get_gdb_reg_list failed %d", ret);
		return ret;
	}

	*reg_list = calloc(*num_regs, sizeof(struct rtos_reg));
	if (!(*reg_list)) {
		LOG_ERROR("Failed to alloc memory for %d", *num_regs);
		free(gdb_reg_list);
		return ERROR_FAIL;
	}

	for (int i = 0; i < *num_regs; i++) {
		(*reg_list)[i].number = gdb_reg_list[i]->number;
		(*reg_list)[i].size = gdb_reg_list[i]->size;
		memcpy((*reg_list)[i].value, gdb_reg_list[i]->value, ((*reg_list)[i].size + 7) / 8);
	}

	free(gdb_reg_list);

	return ERROR_OK;
}

static int zephyr_get_thread_reg_list(struct rtos *rtos, int64_t thread_id,
		struct rtos_reg **reg_list, int *num_regs)
{
	struct zephyr_params *params;
	struct rtos_reg *callee_saved_reg_list = NULL;
	target_addr_t addr;
	int retval;

	LOG_DEBUG("Getting thread 0x%" PRIx64 " (%" PRId64 ") reg list", thread_id, thread_id);

	if (!rtos)
		return ERROR_FAIL;

	if (thread_id == 0)
		return ERROR_FAIL;

	params = rtos->rtos_specific_params;
	if (!params)
		return ERROR_FAIL;

	addr = thread_id;

	LOG_DEBUG("Zephyr OpenOCD stack offset %" PRId32, params->offsets[OFFSET_T_STACK_POINTER]);

	if (params->offsets[OFFSET_T_STACK_POINTER] != UNIMPLEMENTED)
		addr += params->offsets[OFFSET_T_STACK_POINTER];

	if (thread_id == rtos->current_thread) {
		retval = zephyr_get_current_thread_reg_list(rtos, reg_list, num_regs);
	} else {
		retval = params->get_cpu_state(rtos, &addr, params, callee_saved_reg_list, reg_list, num_regs);
		free(callee_saved_reg_list);
	}

	return retval;
}

static int zephyr_get_symbol_list_to_lookup(struct symbol_table_elem **symbol_list)
{
	*symbol_list = malloc(sizeof(zephyr_symbol_list));
	if (!*symbol_list) {
		LOG_ERROR("Out of memory");
		return ERROR_FAIL;
	}

	memcpy(*symbol_list, zephyr_symbol_list, sizeof(zephyr_symbol_list));
	return ERROR_OK;
}

const struct rtos_type zephyr_rtos = {
	.name = "Zephyr",

	.detect_rtos = zephyr_detect_rtos,
	.create = zephyr_create,
	.update_threads = zephyr_update_threads,
	.get_thread_reg_list = zephyr_get_thread_reg_list,
	.get_symbol_list_to_lookup = zephyr_get_symbol_list_to_lookup,
};
