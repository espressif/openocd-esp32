/***************************************************************************
 *   Copyright (C) 2020 Espressif Systems Ltd.                             *
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
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>. *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "rtos.h"
#include "rtos_standard_stackings.h"
#include "target/target.h"
#include "helper/log.h"
#include "helper/binarybuffer.h"
#include "target/armv7m.h"
#include <target/espressif/esp32.h>

static int nuttx_esp_xtensa_stack_read(struct target *target,
	int64_t stack_ptr, const struct rtos_register_stacking *stacking,
	uint8_t *stack_data);

/* see arch/arm/include/armv7-m/irq_cmnvector.h */
static const struct stack_register_offset nuttx_stack_offsets_cortex_m[] = {
	{ ARMV7M_R0, 0x28, 32 },		/* r0   */
	{ ARMV7M_R1, 0x2c, 32 },		/* r1   */
	{ ARMV7M_R2, 0x30, 32 },		/* r2   */
	{ ARMV7M_R3, 0x34, 32 },		/* r3   */
	{ ARMV7M_R4, 0x08, 32 },		/* r4   */
	{ ARMV7M_R5, 0x0c, 32 },		/* r5   */
	{ ARMV7M_R6, 0x10, 32 },		/* r6   */
	{ ARMV7M_R7, 0x14, 32 },		/* r7   */
	{ ARMV7M_R8, 0x18, 32 },		/* r8   */
	{ ARMV7M_R9, 0x1c, 32 },		/* r9   */
	{ ARMV7M_R10, 0x20, 32 },		/* r10  */
	{ ARMV7M_R11, 0x24, 32 },		/* r11  */
	{ ARMV7M_R12, 0x38, 32 },		/* r12  */
	{ ARMV7M_R13, 0, 32 },			/* sp   */
	{ ARMV7M_R14, 0x3c, 32 },		/* lr   */
	{ ARMV7M_PC, 0x40, 32 },		/* pc   */
	{ ARMV7M_xPSR, 0x44, 32 },		/* xPSR */
};

static const struct stack_register_offset nuttx_stack_offsets_cortex_m_fpu[] = {
	{ ARMV7M_R0, 0x6c, 32 },		/* r0   */
	{ ARMV7M_R1, 0x70, 32 },		/* r1   */
	{ ARMV7M_R2, 0x74, 32 },		/* r2   */
	{ ARMV7M_R3, 0x78, 32 },		/* r3   */
	{ ARMV7M_R4, 0x08, 32 },		/* r4   */
	{ ARMV7M_R5, 0x0c, 32 },		/* r5   */
	{ ARMV7M_R6, 0x10, 32 },		/* r6   */
	{ ARMV7M_R7, 0x14, 32 },		/* r7   */
	{ ARMV7M_R8, 0x18, 32 },		/* r8   */
	{ ARMV7M_R9, 0x1c, 32 },		/* r9   */
	{ ARMV7M_R10, 0x20, 32 },		/* r10  */
	{ ARMV7M_R11, 0x24, 32 },		/* r11  */
	{ ARMV7M_R12, 0x7c, 32 },		/* r12  */
	{ ARMV7M_R13, 0, 32 },			/* sp   */
	{ ARMV7M_R14, 0x80, 32 },		/* lr   */
	{ ARMV7M_PC, 0x84, 32 },		/* pc   */
	{ ARMV7M_xPSR, 0x88, 32 },		/* xPSR */
};

static const struct stack_register_offset nuttx_stack_offsets_esp32[] = {
	{ XT_REG_IDX_PC, 0x00, 32 },	/* PC */
	{ XT_REG_IDX_AR0, 0x08, 32 },	/* A0 */
	{ XT_REG_IDX_AR1, 0x0c, 32 },	/* A1 */
	{ XT_REG_IDX_AR2, 0x10, 32 },	/* A2 */
	{ XT_REG_IDX_AR3, 0x14, 32 },	/* A3 */
	{ XT_REG_IDX_AR4, 0x18, 32 },	/* A4 */
	{ XT_REG_IDX_AR5, 0x1c, 32 },	/* A5 */
	{ XT_REG_IDX_AR6, 0x20, 32 },	/* A6 */
	{ XT_REG_IDX_AR7, 0x24, 32 },	/* A7 */
	{ XT_REG_IDX_AR8, 0x28, 32 },	/* A8 */
	{ XT_REG_IDX_AR9, 0x2c, 32 },	/* A9 */
	{ XT_REG_IDX_AR10, 0x30, 32 },	/* A10 */
	{ XT_REG_IDX_AR11, 0x34, 32 },	/* A11 */
	{ XT_REG_IDX_AR12, 0x38, 32 },	/* A12 */
	{ XT_REG_IDX_AR13, 0x3c, 32 },	/* A13 */
	{ XT_REG_IDX_AR14, 0x40, 32 },	/* A14 */
	{ XT_REG_IDX_AR15, 0x44, 32 },	/* A15 */

	/* A16-A63 aren't in the stack frame because they've been flushed to the stack earlier */

	{ XT_REG_IDX_AR16, -1, 32 },		/* A16 */
	{ XT_REG_IDX_AR17, -1, 32 },		/* A17 */
	{ XT_REG_IDX_AR18, -1, 32 },		/* A18 */
	{ XT_REG_IDX_AR19, -1, 32 },		/* A19 */
	{ XT_REG_IDX_AR20, -1, 32 },		/* A20 */
	{ XT_REG_IDX_AR21, -1, 32 },		/* A21 */
	{ XT_REG_IDX_AR22, -1, 32 },		/* A22 */
	{ XT_REG_IDX_AR23, -1, 32 },		/* A23 */
	{ XT_REG_IDX_AR24, -1, 32 },		/* A24 */
	{ XT_REG_IDX_AR25, -1, 32 },		/* A25 */
	{ XT_REG_IDX_AR26, -1, 32 },		/* A26 */
	{ XT_REG_IDX_AR27, -1, 32 },		/* A27 */
	{ XT_REG_IDX_AR28, -1, 32 },		/* A28 */
	{ XT_REG_IDX_AR29, -1, 32 },		/* A29 */
	{ XT_REG_IDX_AR30, -1, 32 },		/* A30 */
	{ XT_REG_IDX_AR31, -1, 32 },		/* A31 */
	{ XT_REG_IDX_AR32, -1, 32 },		/* A32 */
	{ XT_REG_IDX_AR33, -1, 32 },		/* A33 */
	{ XT_REG_IDX_AR34, -1, 32 },		/* A34 */
	{ XT_REG_IDX_AR35, -1, 32 },		/* A35 */
	{ XT_REG_IDX_AR36, -1, 32 },		/* A36 */
	{ XT_REG_IDX_AR37, -1, 32 },		/* A37 */
	{ XT_REG_IDX_AR38, -1, 32 },		/* A38 */
	{ XT_REG_IDX_AR39, -1, 32 },		/* A39 */
	{ XT_REG_IDX_AR40, -1, 32 },		/* A40 */
	{ XT_REG_IDX_AR41, -1, 32 },		/* A41 */
	{ XT_REG_IDX_AR42, -1, 32 },		/* A42 */
	{ XT_REG_IDX_AR43, -1, 32 },		/* A43 */
	{ XT_REG_IDX_AR44, -1, 32 },		/* A44 */
	{ XT_REG_IDX_AR45, -1, 32 },		/* A45 */
	{ XT_REG_IDX_AR46, -1, 32 },		/* A46 */
	{ XT_REG_IDX_AR47, -1, 32 },		/* A47 */
	{ XT_REG_IDX_AR48, -1, 32 },		/* A48 */
	{ XT_REG_IDX_AR49, -1, 32 },		/* A49 */
	{ XT_REG_IDX_AR50, -1, 32 },		/* A50 */
	{ XT_REG_IDX_AR51, -1, 32 },		/* A51 */
	{ XT_REG_IDX_AR52, -1, 32 },		/* A52 */
	{ XT_REG_IDX_AR53, -1, 32 },		/* A53 */
	{ XT_REG_IDX_AR54, -1, 32 },		/* A54 */
	{ XT_REG_IDX_AR55, -1, 32 },		/* A55 */
	{ XT_REG_IDX_AR56, -1, 32 },		/* A56 */
	{ XT_REG_IDX_AR57, -1, 32 },		/* A57 */
	{ XT_REG_IDX_AR58, -1, 32 },		/* A58 */
	{ XT_REG_IDX_AR59, -1, 32 },		/* A59 */
	{ XT_REG_IDX_AR60, -1, 32 },		/* A60 */
	{ XT_REG_IDX_AR61, -1, 32 },		/* A61 */
	{ XT_REG_IDX_AR62, -1, 32 },		/* A62 */
	{ XT_REG_IDX_AR63, -1, 32 },		/* A63 */

	{ XT_REG_IDX_LBEG, 0x54, 32 },		/* lbeg */
	{ XT_REG_IDX_LEND, 0x58, 32 },		/* lend */
	{ XT_REG_IDX_LCOUNT, 0x5c, 32 },	/* lcount */
	{ XT_REG_IDX_SAR, 0x48, 32 },		/* SAR */

	{ XT_REG_IDX_WINDOWBASE, -1, 32 },		/* windowbase */
	{ XT_REG_IDX_WINDOWSTART, -1, 32 },		/* windowstart */
	{ XT_REG_IDX_CONFIGID0, -1, 32 },		/* configid0 */
	{ XT_REG_IDX_CONFIGID1, -1, 32 },		/* configid1 */

	{ XT_REG_IDX_PS, 0x04, 32 },		/* PS */

	{ XT_REG_IDX_THREADPTR, -1, 32 },	/* threadptr */
	{ XT_REG_IDX_BR, -1, 32 },			/* br */
	{ XT_REG_IDX_SCOMPARE1, -1, 32 },	/* scompare1 */
	{ XT_REG_IDX_ACCLO, -1, 32 },		/* acclo */
	{ XT_REG_IDX_ACCHI, -1, 32 },		/* acchi */
	{ XT_REG_IDX_M0, -1, 32 },			/* m0 */
	{ XT_REG_IDX_M1, -1, 32 },			/* m1 */
	{ XT_REG_IDX_M2, -1, 32 },			/* m2 */
	{ XT_REG_IDX_M3, -1, 32 },			/* m3 */
	{ ESP32_REG_IDX_EXPSTATE, -1, 32 },		/* expstate */
	{ ESP32_REG_IDX_F64R_LO, -1, 32 },		/* f64r_lo */
	{ ESP32_REG_IDX_F64R_HI, -1, 32 },		/* f64r_hi */
	{ ESP32_REG_IDX_F64S, -1, 32 },			/* f64s */
	{ XT_REG_IDX_F0, -1, 32 },		/* f0 */
	{ XT_REG_IDX_F1, -1, 32 },		/* f1 */
	{ XT_REG_IDX_F2, -1, 32 },		/* f2 */
	{ XT_REG_IDX_F3, -1, 32 },		/* f3 */
	{ XT_REG_IDX_F4, -1, 32 },		/* f4 */
	{ XT_REG_IDX_F5, -1, 32 },		/* f5 */
	{ XT_REG_IDX_F6, -1, 32 },		/* f6 */
	{ XT_REG_IDX_F7, -1, 32 },		/* f7 */
	{ XT_REG_IDX_F8, -1, 32 },		/* f8 */
	{ XT_REG_IDX_F9, -1, 32 },		/* f9 */
	{ XT_REG_IDX_F10, -1, 32 },		/* f10 */
	{ XT_REG_IDX_F11, -1, 32 },		/* f11 */
	{ XT_REG_IDX_F12, -1, 32 },		/* f12 */
	{ XT_REG_IDX_F13, -1, 32 },		/* f13 */
	{ XT_REG_IDX_F14, -1, 32 },		/* f14 */
	{ XT_REG_IDX_F15, -1, 32 },		/* f15 */
	{ XT_REG_IDX_FCR, -1, 32 },		/* fcr */
	{ XT_REG_IDX_FSR, -1, 32 },		/* fsr */
};

const struct rtos_register_stacking nuttx_stacking_cortex_m = {
	0x48,				/* stack_registers_size */
	-1,					/* stack_growth_direction */
	17,					/* num_output_registers */
	0,					/* stack_alignment */
	nuttx_stack_offsets_cortex_m,		/* register_offsets */
	NULL
};

const struct rtos_register_stacking nuttx_stacking_cortex_m_fpu = {
	0x8c,				/* stack_registers_size */
	-1,					/* stack_growth_direction */
	17,					/* num_output_registers */
	0,					/* stack_alignment */
	nuttx_stack_offsets_cortex_m_fpu,	/* register_offsets */
	NULL
};

const struct rtos_register_stacking nuttx_esp32_stacking = {
	26 * 4,							/* stack_registers_size */
	-1,								/* stack_growth_direction */
	ESP32_NUM_REGS_G_COMMAND,		/* num_output_registers */
	rtos_generic_stack_align8,		/* stack_alignment */
	nuttx_stack_offsets_esp32,		/* register_offsets */
	nuttx_esp_xtensa_stack_read		/* Custom stack frame read function */
};

static int nuttx_esp_xtensa_stack_read(struct target *target,
	int64_t stack_ptr, const struct rtos_register_stacking *stacking,
	uint8_t *stack_data)
{
	int retval;

	retval = target_read_buffer(target, stack_ptr, stacking->stack_registers_size,
		stack_data);
	if (retval != ERROR_OK)
		return retval;

	stack_data[4] &= ~0x10;	/* Clear exception bit in PS */

	return retval;
}
