/***************************************************************************
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
#include "rtos_standard_stackings.h"
#include "target/target.h"
#include "helper/log.h"
#include "helper/binarybuffer.h"

static int rtos_freertos_esp108_stack_read_involuntary(struct target *target, int64_t stack_ptr, const struct rtos_register_stacking *stacking, uint8_t *stack_data);
static int rtos_freertos_esp108_stack_read_voluntary(struct target *target, int64_t stack_ptr, const struct rtos_register_stacking *stacking, uint8_t *stack_data);

/*
 The XTensa FreeRTOS implementation has *two* types of stack frames; one for 
 involuntatrily swapped out tasks and another one for tasks which voluntarily yielded.
*/
static const struct stack_register_offset rtos_freertos_esp108_stack_offsets[] = {
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
	{   -1, 32 },		/* f64r_lo */
	{   -1, 32 },		/* f64r_hi */
	{   -1, 32 },		/* f64s */
	{   -1, 32 },		/* f0 */
	{   -1, 32 },		/* f1 */
	{   -1, 32 },		/* f2 */
	{   -1, 32 },		/* f3 */
	{   -1, 32 },		/* f4 */
	{   -1, 32 },		/* f5 */
	{   -1, 32 },		/* f6 */
	{   -1, 32 },		/* f7 */
	{   -1, 32 },		/* f8 */
	{   -1, 32 },		/* f9 */
	{   -1, 32 },		/* f10 */
	{   -1, 32 },		/* f11 */
	{   -1, 32 },		/* f12 */
	{   -1, 32 },		/* f13 */
	{   -1, 32 },		/* f14 */
	{   -1, 32 },		/* f15 */
	{   -1, 32 },		/* fcr */
	{   -1, 32 },		/* fsr */
};

//WARNING: There's some deeper magic going on when reading this. Please
//refer to rtos_freertos_esp108_stack_read_voluntary for more info.

static const struct stack_register_offset rtos_freertos_esp108_voluntary_stack_offsets[] = {
	{ 0x14, 32 },		/* PC */
	{ 0x00, 32 },		/* A0 */
	{ 0x04, 32 },		/* A1 */
	{ 0x08, 32 },		/* A2 */
	{ 0x0c, 32 },		/* A3 */
	{ 0x30, 32 },		/* A4 */
	{ 0x34, 32 },		/* A5 */
	{ 0x38, 32 },		/* A6 */
	{ 0x3c, 32 },		/* A7 */
	{ 0x40, 32 },		/* A8 */
	{ 0x44, 32 },		/* A9 */
	{ 0x48, 32 },		/* A10 */
	{ 0x4c, 32 },		/* A11 */
	{   -1, 32 },		/* A12 */ /* A12-A63 aren't in the stack frame because they've been flushed to the stack earlier or not relevant*/
	{   -1, 32 },		/* A13 */
	{   -1, 32 },		/* A14 */
	{   -1, 32 },		/* A15 */
	{   -1, 32 },		/* A16 */
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
	{ 0x18, 32 },		/* ps */
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
	{   -1, 32 },		/* f64r_lo */
	{   -1, 32 },		/* f64r_hi */
	{   -1, 32 },		/* f64s */
	{   -1, 32 },		/* f0 */
	{   -1, 32 },		/* f1 */
	{   -1, 32 },		/* f2 */
	{   -1, 32 },		/* f3 */
	{   -1, 32 },		/* f4 */
	{   -1, 32 },		/* f5 */
	{   -1, 32 },		/* f6 */
	{   -1, 32 },		/* f7 */
	{   -1, 32 },		/* f8 */
	{   -1, 32 },		/* f9 */
	{   -1, 32 },		/* f10 */
	{   -1, 32 },		/* f11 */
	{   -1, 32 },		/* f12 */
	{   -1, 32 },		/* f13 */
	{   -1, 32 },		/* f14 */
	{   -1, 32 },		/* f15 */
	{   -1, 32 },		/* fcr */
	{   -1, 32 },		/* fsr */
};


const struct rtos_register_stacking rtos_freertos_esp108_stacking = {
	30*4,				/* stack_registers_size */
	-1,					/* stack_growth_direction */
	105,					/* num_output_registers */
	rtos_generic_stack_align8,	/* stack_alignment */
	rtos_freertos_esp108_stack_offsets,		/* register_offsets */
	rtos_freertos_esp108_stack_read_involuntary		/* Custom stack frame read function */
};

const struct rtos_register_stacking rtos_freertos_voluntary_esp108_stacking = {
	0x50,				/* stack_registers_size, including 'faked' stack values */
	-1,					/* stack_growth_direction */
	105,					/* num_output_registers */
	rtos_generic_stack_align8,	/* stack_alignment */
	rtos_freertos_esp108_voluntary_stack_offsets,	/* register_offsets */
	rtos_freertos_esp108_stack_read_voluntary		/* Custom stack frame read function */
};


/*
 This function uses the first word of the stack memory to see if the stack frame is from a
 voluntary or unvoluntary yield, and returns the correct stack frame info.
*/
const struct rtos_register_stacking *rtos_freertos_esp108_pick_stacking_info(struct rtos *rtos, int64_t thread_id, int64_t stack_addr)
{
	int retval;
	uint32_t stack_ptr = 0;
	uint32_t stk_exit;

	if (rtos == NULL)
		return &rtos_freertos_esp108_stacking;

	if (thread_id == 0)
		return &rtos_freertos_esp108_stacking;

	/* Read the stack pointer */
	retval = target_read_buffer(rtos->target,
			stack_addr,
			4,
			(uint8_t *)&stack_ptr);

	/* Read the XT_STK_EXIT variable */
	if (retval!=ERROR_OK) return &rtos_freertos_esp108_stacking;
	retval = target_read_buffer(rtos->target,
			stack_ptr,
			4,
			(uint8_t *)&stk_exit);
	
	if (stk_exit) {
		return &rtos_freertos_esp108_stacking;
	} else {
		return &rtos_freertos_voluntary_esp108_stacking;
	}
}


/*
 Xtensa stacks are saved with the exception bit set, while the call structure is more like a normal
 windowed call. We get rid of the exception bit so gdb can parse it into a working backtrace.
*/
static int rtos_freertos_esp108_stack_read_involuntary(struct target *target, int64_t stack_ptr, const struct rtos_register_stacking *stacking, uint8_t *stack_data)
{
	int retval;
	retval = target_read_buffer(target, stack_ptr, stacking->stack_registers_size, stack_data);
	if (retval!=ERROR_OK) return retval;
	
	stack_data[8]&=~0x10; //clear exception bit in PS
	
	return retval;
}

/*
 Voluntary stacks also have the exception bit set, but they spill the registers on entry and only save the
 PC and PS, as they are inside the vPortYield function. 

 Also, because the vPortYield function is called using a window call, the top 2 bits of PC are killed (because
 they indicate the call size). We hard-set those to 0x4 now. ToDo: glance a correct address from a symbol 
 somewhere.

 To get to the original function, we have to restore the callers stack frame. On Xtensa, the amount of
 registers to restore are stored in the top of A0 on the callees (which is XT_SOL_PC now). The
 registers A0-A4 are saved on the callees stack, in the 4 bytes above the SP. The 

 Because of this madness, we read not only the stack frame, but also the 4 words *above* it (that is, the
 previous stack frame. That's the reason the stack frame definition above looks wonky. It's like this:
 (XT_SOL_xxx entries are part of the FreeRTOS solicited yield stack frame)

0x00 - A0 of orig fn
0x04 - A1 of orig fn
0x08 - A2 of orig fn
0x0c - A3 of orig fn
0x10 - XT_SOL_EXIT - 0 for a voluntary yield <- SP actually is here
0x14 - XT_SOL_PC   - A0 on call to vPortYield
0x18 - XT_SOL_PS   - PS on call to vPortYield
0x1c - XT_SOL_NEXT - Unused
0x20 - XT_SOL_A0   - Unused
0x24 - XT_SOL_A1   - Unused
0x28 - XT_SOL_A2   - Unused
0x2C - XT_SOL_A3   - Unused
0x30 - A4 of orig fn
...
0x4C - A11 of orig fn

 If we're called using CALL8/CALL12, register A4-A7 and in the case of CALL12 A8-A11 are stored in the 
 *callers* stack frame. We need some extra logic to dump them in the stack_data.
*/
static int rtos_freertos_esp108_stack_read_voluntary(struct target *target, int64_t stack_ptr, const struct rtos_register_stacking *stacking, uint8_t *stack_data)
{
	int retval;
	int callno;
	int i;
	uint32_t prevsp;
	uint32_t xt_sol_pc_reg;

	retval = target_read_buffer(target, stack_ptr-0x10, 4*8, stack_data);
	if (retval!=ERROR_OK) return retval;
	
	stack_data[0x18]&=~0x10; //clear exception bit in PS
	xt_sol_pc_reg = le_to_h_u32(stack_data + 5 * sizeof(uint32_t));
	callno = xt_sol_pc_reg >> 30;
	xt_sol_pc_reg = (xt_sol_pc_reg & 0x3FFFFFFF) | 0x40000000; //Hardcoded for now.
	h_u32_to_le(stack_data + 5 * sizeof(uint32_t), xt_sol_pc_reg);
	prevsp = le_to_h_u32(stack_data + 1 * sizeof(uint32_t));

	//Fill unknown regs with dummy value
	for (i=12; i<20; i++) {
		h_u32_to_le(stack_data + i * sizeof(uint32_t), 0xdeadbeef);
	}

	if (callno==2) { //call8
		retval = target_read_buffer(target, prevsp-32, 4*4, stack_data+0x30);
	} else if (callno==3) { //call12
		retval = target_read_buffer(target, prevsp-48, 8*4, stack_data+0x30);
	}
	return retval;
}


