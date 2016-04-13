/***************************************************************************
 *   Copyright (C) 2011 by Broadcom Corporation                            *
 *   Evan Hunter - ehunter@broadcom.com                                    *
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

#include "rtos.h"
#include "target/armv7m.h"

static const struct stack_register_offset rtos_standard_Cortex_M3_stack_offsets[ARMV7M_NUM_CORE_REGS] = {
	{ 0x20, 32 },		/* r0   */
	{ 0x24, 32 },		/* r1   */
	{ 0x28, 32 },		/* r2   */
	{ 0x2c, 32 },		/* r3   */
	{ 0x00, 32 },		/* r4   */
	{ 0x04, 32 },		/* r5   */
	{ 0x08, 32 },		/* r6   */
	{ 0x0c, 32 },		/* r7   */
	{ 0x10, 32 },		/* r8   */
	{ 0x14, 32 },		/* r9   */
	{ 0x18, 32 },		/* r10  */
	{ 0x1c, 32 },		/* r11  */
	{ 0x30, 32 },		/* r12  */
	{ -2,   32 },		/* sp   */
	{ 0x34, 32 },		/* lr   */
	{ 0x38, 32 },		/* pc   */
	{ 0x3c, 32 },		/* xPSR */
};

static const struct stack_register_offset rtos_standard_Cortex_R4_stack_offsets[] = {
	{ 0x08, 32 },		/* r0  (a1)   */
	{ 0x0c, 32 },		/* r1  (a2)  */
	{ 0x10, 32 },		/* r2  (a3)  */
	{ 0x14, 32 },		/* r3  (a4)  */
	{ 0x18, 32 },		/* r4  (v1)  */
	{ 0x1c, 32 },		/* r5  (v2)  */
	{ 0x20, 32 },		/* r6  (v3)  */
	{ 0x24, 32 },		/* r7  (v4)  */
	{ 0x28, 32 },		/* r8  (a1)  */
	{ 0x2c, 32 },		/* r9  (sb)  */
	{ 0x30, 32 },		/* r10 (sl) */
	{ 0x34, 32 },		/* r11 (fp) */
	{ 0x38, 32 },		/* r12 (ip) */
	{ -2,   32 },		/* sp   */
	{ 0x3c, 32 },		/* lr   */
	{ 0x40, 32 },		/* pc   */
	{ -1,   96 },		/* FPA1 */
	{ -1,   96 },		/* FPA2 */
	{ -1,   96 },		/* FPA3 */
	{ -1,   96 },		/* FPA4 */
	{ -1,   96 },		/* FPA5 */
	{ -1,   96 },		/* FPA6 */
	{ -1,   96 },		/* FPA7 */
	{ -1,   96 },		/* FPA8 */
	{ -1,   32 },		/* FPS  */
	{ 0x04, 32 },		/* CSPR */
};

static const struct stack_register_offset rtos_standard_NDS32_N1068_stack_offsets[] = {
	{ 0x88, 32 },		/* R0  */
	{ 0x8C, 32 },		/* R1 */
	{ 0x14, 32 },		/* R2 */
	{ 0x18, 32 },		/* R3 */
	{ 0x1C, 32 },		/* R4 */
	{ 0x20, 32 },		/* R5 */
	{ 0x24, 32 },		/* R6 */
	{ 0x28, 32 },		/* R7 */
	{ 0x2C, 32 },		/* R8 */
	{ 0x30, 32 },		/* R9 */
	{ 0x34, 32 },		/* R10 */
	{ 0x38, 32 },		/* R11 */
	{ 0x3C, 32 },		/* R12 */
	{ 0x40, 32 },		/* R13 */
	{ 0x44, 32 },		/* R14 */
	{ 0x48, 32 },		/* R15 */
	{ 0x4C, 32 },		/* R16 */
	{ 0x50, 32 },		/* R17 */
	{ 0x54, 32 },		/* R18 */
	{ 0x58, 32 },		/* R19 */
	{ 0x5C, 32 },		/* R20 */
	{ 0x60, 32 },		/* R21 */
	{ 0x64, 32 },		/* R22 */
	{ 0x68, 32 },		/* R23 */
	{ 0x6C, 32 },		/* R24 */
	{ 0x70, 32 },		/* R25 */
	{ 0x74, 32 },		/* R26 */
	{ 0x78, 32 },		/* R27 */
	{ 0x7C, 32 },		/* R28 */
	{ 0x80, 32 },		/* R29 */
	{ 0x84, 32 },		/* R30 (LP) */
	{ 0x00, 32 },		/* R31 (SP) */
	{ 0x04, 32 },		/* PSW */
	{ 0x08, 32 },		/* IPC */
	{ 0x0C, 32 },		/* IPSW */
	{ 0x10, 32 },		/* IFC_LP */
};

/*
The XTensa FreeRTOS implementation has *two* types of stack frames; one for 
involuntatrily swapped out tasks and another one for tasks which voluntarily yielded.
*/
static const struct stack_register_offset rtos_standard_esp108_stack_offsets[] = {
	{ 0x04, 32 },		/* PC */
	{ 0x0c, 32 },		/* A0 */
	{   -2, 32 },		/* A1 */ //0x10
	{ 0x14, 32 },		/* A2 */
	{ 0x18, 32 },		/* A3 */
	{ 0x1c, 32 },		/* A4 */
	{ 0x20, 32 },		/* A5 */
	{ 0x24, 32 },		/* A6 */
	{ 0x28, 32 },		/* A7 */
	{ 0x2c, 32 },		/* A8 */
	{ 0x30, 32 },		/* A9 */
	{ 0x34, 32 },		/* A10 */
	{ 0x38, 32 },		/* A11 */
	{ 0x3c, 32 },		/* A12 */
	{ 0x40, 32 },		/* A13 */
	{ 0x44, 32 },		/* A14 */
	{ 0x48, 32 },		/* A15 */
	{   -1, 32 },		/* A16 */ /* A16-A63 aren't in the stack frame because they've been flushed to the stack earlier */
	{   -1, 32 },		/* A17 */
	{   -1, 32 },		/* A18 */
	{   -1, 32 },		/* A19 */
	{   -1, 32 },		/* A20 */
	{   -1, 32 },		/* A21 */
	{   -1, 32 },		/* A22 */
	{   -1, 32 },		/* A23 */
	{   -1, 32 },		/* A24 */
	{   -1, 32 },		/* A25 */
	{   -1, 32 },		/* A26 */
	{   -1, 32 },		/* A27 */
	{   -1, 32 },		/* A28 */
	{   -1, 32 },		/* A29 */
	{   -1, 32 },		/* A30 */
	{   -1, 32 },		/* A31 */
	{   -1, 32 },		/* A32 */
	{   -1, 32 },		/* A33 */
	{   -1, 32 },		/* A34 */
	{   -1, 32 },		/* A35 */
	{   -1, 32 },		/* A36 */
	{   -1, 32 },		/* A37 */
	{   -1, 32 },		/* A38 */
	{   -1, 32 },		/* A39 */
	{   -1, 32 },		/* A40 */
	{   -1, 32 },		/* A41 */
	{   -1, 32 },		/* A42 */
	{   -1, 32 },		/* A43 */
	{   -1, 32 },		/* A44 */
	{   -1, 32 },		/* A45 */
	{   -1, 32 },		/* A46 */
	{   -1, 32 },		/* A47 */
	{   -1, 32 },		/* A48 */
	{   -1, 32 },		/* A49 */
	{   -1, 32 },		/* A50 */
	{   -1, 32 },		/* A51 */
	{   -1, 32 },		/* A52 */
	{   -1, 32 },		/* A53 */
	{   -1, 32 },		/* A54 */
	{   -1, 32 },		/* A55 */
	{   -1, 32 },		/* A56 */
	{   -1, 32 },		/* A57 */
	{   -1, 32 },		/* A58 */
	{   -1, 32 },		/* A59 */
	{   -1, 32 },		/* A60 */
	{   -1, 32 },		/* A61 */
	{   -1, 32 },		/* A62 */
	{   -1, 32 },		/* A63 */
	{ 0x58, 32 },		/* lbeg */
	{ 0x5c, 32 },		/* lend */
	{ 0x60, 32 },		/* lcount */
	{ 0x4c, 32 },		/* sar */
	{   -1, 32 },		/* windowbase */
	{   -1, 32 },		/* windowstart */
	{   -1, 32 },		/* configid0 */
	{   -1, 32 },		/* configid1 */
	{ 0x08, 32 },		/* ps */
	{   -1, 32 },		/* threadptr */
	{   -1, 32 },		/* br */
	{   -1, 32 },		/* scompare1 */
	{   -1, 32 },		/* acclo */
	{   -1, 32 },		/* acchi */
	{   -1, 32 },		/* m0 */
	{   -1, 32 },		/* m1 */
	{   -1, 32 },		/* m2 */
	{   -1, 32 },		/* m3 */
	{   -1, 32 },		/* expstate */
};


static const struct stack_register_offset rtos_standard_esp108_voluntary_stack_offsets[] = {
	{ 0x10, 32 },		/* PC */ //0x4
	{ 0x04, 32 },		/* A0 */ //0x10
	{   -2, 32 },		/* A1 */ //0x14
	{ 0x18, 32 },		/* A2 */
	{ 0x1C, 32 },		/* A3 */
	{   -1, 32 },		/* A4 */
	{   -1, 32 },		/* A5 */
	{   -1, 32 },		/* A6 */
	{   -1, 32 },		/* A7 */
	{   -1, 32 },		/* A8 */
	{   -1, 32 },		/* A9 */
	{   -1, 32 },		/* A10 */
	{   -1, 32 },		/* A11 */
	{   -1, 32 },		/* A12 */
	{   -1, 32 },		/* A13 */
	{   -1, 32 },		/* A14 */
	{   -1, 32 },		/* A15 */
	{   -1, 32 },		/* A16 */ /* A16-A63 aren't in the stack frame because they've been flushed to the stack earlier */
	{   -1, 32 },		/* A17 */
	{   -1, 32 },		/* A18 */
	{   -1, 32 },		/* A19 */
	{   -1, 32 },		/* A20 */
	{   -1, 32 },		/* A21 */
	{   -1, 32 },		/* A22 */
	{   -1, 32 },		/* A23 */
	{   -1, 32 },		/* A24 */
	{   -1, 32 },		/* A25 */
	{   -1, 32 },		/* A26 */
	{   -1, 32 },		/* A27 */
	{   -1, 32 },		/* A28 */
	{   -1, 32 },		/* A29 */
	{   -1, 32 },		/* A30 */
	{   -1, 32 },		/* A31 */
	{   -1, 32 },		/* A32 */
	{   -1, 32 },		/* A33 */
	{   -1, 32 },		/* A34 */
	{   -1, 32 },		/* A35 */
	{   -1, 32 },		/* A36 */
	{   -1, 32 },		/* A37 */
	{   -1, 32 },		/* A38 */
	{   -1, 32 },		/* A39 */
	{   -1, 32 },		/* A40 */
	{   -1, 32 },		/* A41 */
	{   -1, 32 },		/* A42 */
	{   -1, 32 },		/* A43 */
	{   -1, 32 },		/* A44 */
	{   -1, 32 },		/* A45 */
	{   -1, 32 },		/* A46 */
	{   -1, 32 },		/* A47 */
	{   -1, 32 },		/* A48 */
	{   -1, 32 },		/* A49 */
	{   -1, 32 },		/* A50 */
	{   -1, 32 },		/* A51 */
	{   -1, 32 },		/* A52 */
	{   -1, 32 },		/* A53 */
	{   -1, 32 },		/* A54 */
	{   -1, 32 },		/* A55 */
	{   -1, 32 },		/* A56 */
	{   -1, 32 },		/* A57 */
	{   -1, 32 },		/* A58 */
	{   -1, 32 },		/* A59 */
	{   -1, 32 },		/* A60 */
	{   -1, 32 },		/* A61 */
	{   -1, 32 },		/* A62 */
	{   -1, 32 },		/* A63 */
	{   -1, 32 },		/* lbeg */
	{   -1, 32 },		/* lend */
	{   -1, 32 },		/* lcount */
	{   -1, 32 },		/* sar */
	{   -1, 32 },		/* windowbase */
	{   -1, 32 },		/* windowstart */
	{   -1, 32 },		/* configid0 */
	{   -1, 32 },		/* configid1 */
	{ 0x08, 32 },		/* ps */
	{   -1, 32 },		/* threadptr */
	{   -1, 32 },		/* br */
	{   -1, 32 },		/* scompare1 */
	{   -1, 32 },		/* acclo */
	{   -1, 32 },		/* acchi */
	{   -1, 32 },		/* m0 */
	{   -1, 32 },		/* m1 */
	{   -1, 32 },		/* m2 */
	{   -1, 32 },		/* m3 */
	{   -1, 32 },		/* expstate */
};



static int64_t rtos_generic_stack_align(struct target *target,
	const uint8_t *stack_data, const struct rtos_register_stacking *stacking,
	int64_t stack_ptr, int align)
{
	int64_t new_stack_ptr;
	int64_t aligned_stack_ptr;
	new_stack_ptr = stack_ptr - stacking->stack_growth_direction *
		stacking->stack_registers_size;
	aligned_stack_ptr = new_stack_ptr & ~((int64_t)align - 1);
	if (aligned_stack_ptr != new_stack_ptr &&
		stacking->stack_growth_direction == -1) {
		/* If we have a downward growing stack, the simple alignment code
		 * above results in a wrong result (since it rounds down to nearest
		 * alignment).  We want to round up so add an extra align.
		 */
		aligned_stack_ptr += (int64_t)align;
	}
	return aligned_stack_ptr;
}

int64_t rtos_generic_stack_align8(struct target *target,
	const uint8_t *stack_data, const struct rtos_register_stacking *stacking,
	int64_t stack_ptr)
{
	return rtos_generic_stack_align(target, stack_data,
			stacking, stack_ptr, 8);
}

/* The Cortex M3 will indicate that an alignment adjustment
 * has been done on the stack by setting bit 9 of the stacked xPSR
 * register.  In this case, we can just add an extra 4 bytes to get
 * to the program stack.  Note that some places in the ARM documentation
 * make this a little unclear but the padding takes place before the
 * normal exception stacking - so xPSR is always available at a fixed
 * location.
 *
 * Relevant documentation:
 *    Cortex-M series processors -> Cortex-M3 -> Revision: xxx ->
 *        Cortex-M3 Devices Generic User Guide -> The Cortex-M3 Processor ->
 *        Exception Model -> Exception entry and return -> Exception entry
 *    Cortex-M series processors -> Cortex-M3 -> Revision: xxx ->
 *        Cortex-M3 Devices Generic User Guide -> Cortex-M3 Peripherals ->
 *        System control block -> Configuration and Control Register (STKALIGN)
 *
 * This is just a helper function for use in the calculate_process_stack
 * function for a given architecture/rtos.
 */
int64_t rtos_Cortex_M_stack_align(struct target *target,
	const uint8_t *stack_data, const struct rtos_register_stacking *stacking,
	int64_t stack_ptr, size_t xpsr_offset)
{
	const uint32_t ALIGN_NEEDED = (1 << 9);
	uint32_t xpsr;
	int64_t new_stack_ptr;

	new_stack_ptr = stack_ptr - stacking->stack_growth_direction *
		stacking->stack_registers_size;
	xpsr = (target->endianness == TARGET_LITTLE_ENDIAN) ?
			le_to_h_u32(&stack_data[xpsr_offset]) :
			be_to_h_u32(&stack_data[xpsr_offset]);
	if ((xpsr & ALIGN_NEEDED) != 0) {
		LOG_DEBUG("XPSR(0x%08" PRIx32 ") indicated stack alignment was necessary\r\n",
			xpsr);
		new_stack_ptr -= (stacking->stack_growth_direction * 4);
	}
	return new_stack_ptr;
}

static int64_t rtos_standard_Cortex_M3_stack_align(struct target *target,
	const uint8_t *stack_data, const struct rtos_register_stacking *stacking,
	int64_t stack_ptr)
{
	const int XPSR_OFFSET = 0x3c;
	return rtos_Cortex_M_stack_align(target, stack_data, stacking,
		stack_ptr, XPSR_OFFSET);
}

const struct rtos_register_stacking rtos_standard_Cortex_M3_stacking = {
	0x40,					/* stack_registers_size */
	-1,						/* stack_growth_direction */
	ARMV7M_NUM_CORE_REGS,	/* num_output_registers */
	rtos_standard_Cortex_M3_stack_align,	/* stack_alignment */
	rtos_standard_Cortex_M3_stack_offsets	/* register_offsets */
};

const struct rtos_register_stacking rtos_standard_Cortex_R4_stacking = {
	0x48,				/* stack_registers_size */
	-1,					/* stack_growth_direction */
	26,					/* num_output_registers */
	rtos_generic_stack_align8,	/* stack_alignment */
	rtos_standard_Cortex_R4_stack_offsets	/* register_offsets */
};

const struct rtos_register_stacking rtos_standard_NDS32_N1068_stacking = {
	0x90,				/* stack_registers_size */
	-1,					/* stack_growth_direction */
	32,					/* num_output_registers */
	rtos_generic_stack_align8,	/* stack_alignment */
	rtos_standard_NDS32_N1068_stack_offsets	/* register_offsets */
};

//ToDo: move to rtos_freertos_stackings.c

const struct rtos_register_stacking rtos_standard_esp108_stacking = {
	30*4,				/* stack_registers_size */
	-1,					/* stack_growth_direction */
	84,					/* num_output_registers */
	rtos_generic_stack_align8,	/* stack_alignment */
	rtos_standard_esp108_stack_offsets	/* register_offsets */
};

const struct rtos_register_stacking rtos_standard_voluntary_esp108_stacking = {
	8*4,				/* stack_registers_size */
	-1,					/* stack_growth_direction */
	84,					/* num_output_registers */
	rtos_generic_stack_align8,	/* stack_alignment */
	rtos_standard_esp108_voluntary_stack_offsets	/* register_offsets */
};



