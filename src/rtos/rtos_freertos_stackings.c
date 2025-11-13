// SPDX-License-Identifier: GPL-2.0-or-later

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "rtos_freertos_stackings.h"
#include "rtos_standard_stackings.h"
#include "target/target.h"
#include "helper/log.h"
#include "helper/binarybuffer.h"
#include <target/riscv/riscv.h>

static int rtos_freertos_esp_xtensa_stack_read_involuntary(struct target *target,
	int64_t stack_ptr,
	const struct rtos_register_stacking *stacking,
	uint8_t *stack_data);
static int rtos_freertos_esp_xtensa_stack_read_voluntary(struct target *target,
	int64_t stack_ptr,
	const struct rtos_register_stacking *stacking,
	uint8_t *stack_data);

#define ESP_PC_REG_OFFSET        4
#define ESP_PS_REG_OFFSET        8
#define ESP_PS_EXCM_BIT          4
#define ESP_BASE_SAVE_AREA_LEN   16

/*
 * The XTensa FreeRTOS implementation has *two* types of stack frames; one for
 * involuntarily swapped out tasks and another one for tasks which voluntarily yielded.
 */

/*
 * Important Note: If you modify one of the stack_register_offset array, also check corresponding
 * stack_registers_size in rtos_register_stacking
 */

static const struct stack_register_offset rtos_freertos_esp32_stack_offsets[] = {
	{ 0,   0x04, 32 },		/* PC */
	{ 1,   0x0c, 32 },		/* A0 */
	{ 2,   0x10, 32 },		/* A1 */
	{ 3,   0x14, 32 },		/* A2 */
	{ 4,   0x18, 32 },		/* A3 */
	{ 5,   0x1c, 32 },		/* A4 */
	{ 6,   0x20, 32 },		/* A5 */
	{ 7,   0x24, 32 },		/* A6 */
	{ 8,   0x28, 32 },		/* A7 */
	{ 9,   0x2c, 32 },		/* A8 */
	{ 10,  0x30, 32 },		/* A9 */
	{ 11,  0x34, 32 },		/* A10 */
	{ 12,  0x38, 32 },		/* A11 */
	{ 13,  0x3c, 32 },		/* A12 */
	{ 14,  0x40, 32 },		/* A13 */
	{ 15,  0x44, 32 },		/* A14 */
	{ 16,  0x48, 32 },		/* A15 */
	/* A16-A63 aren't in the stack frame because they've been flushed to the stack earlier */
	{ 17,  -1, 32 },		/* A16 */
	{ 18,  -1, 32 },		/* A17 */
	{ 19,  -1, 32 },		/* A18 */
	{ 20,  -1, 32 },		/* A19 */
	{ 21,  -1, 32 },		/* A20 */
	{ 22,  -1, 32 },		/* A21 */
	{ 23,  -1, 32 },		/* A22 */
	{ 24,  -1, 32 },		/* A23 */
	{ 25,  -1, 32 },		/* A24 */
	{ 26,  -1, 32 },		/* A25 */
	{ 27,  -1, 32 },		/* A26 */
	{ 28,  -1, 32 },		/* A27 */
	{ 29,  -1, 32 },		/* A28 */
	{ 30,  -1, 32 },		/* A29 */
	{ 31,  -1, 32 },		/* A30 */
	{ 32,  -1, 32 },		/* A31 */
	{ 33,  -1, 32 },		/* A32 */
	{ 34,  -1, 32 },		/* A33 */
	{ 35,  -1, 32 },		/* A34 */
	{ 36,  -1, 32 },		/* A35 */
	{ 37,  -1, 32 },		/* A36 */
	{ 38,  -1, 32 },		/* A37 */
	{ 39,  -1, 32 },		/* A38 */
	{ 40,  -1, 32 },		/* A39 */
	{ 41,  -1, 32 },		/* A40 */
	{ 42,  -1, 32 },		/* A41 */
	{ 43,  -1, 32 },		/* A42 */
	{ 44,  -1, 32 },		/* A43 */
	{ 45,  -1, 32 },		/* A44 */
	{ 46,  -1, 32 },		/* A45 */
	{ 47,  -1, 32 },		/* A46 */
	{ 48,  -1, 32 },		/* A47 */
	{ 49,  -1, 32 },		/* A48 */
	{ 50,  -1, 32 },		/* A49 */
	{ 51,  -1, 32 },		/* A50 */
	{ 52,  -1, 32 },		/* A51 */
	{ 53,  -1, 32 },		/* A52 */
	{ 54,  -1, 32 },		/* A53 */
	{ 55,  -1, 32 },		/* A54 */
	{ 56,  -1, 32 },		/* A55 */
	{ 57,  -1, 32 },		/* A56 */
	{ 58,  -1, 32 },		/* A57 */
	{ 59,  -1, 32 },		/* A58 */
	{ 60,  -1, 32 },		/* A59 */
	{ 61,  -1, 32 },		/* A60 */
	{ 62,  -1, 32 },		/* A61 */
	{ 63,  -1, 32 },		/* A62 */
	{ 64,  -1, 32 },		/* A63 */
	{ 65,  0x58, 32 },		/* lbeg */
	{ 66,  0x5c, 32 },		/* lend */
	{ 67,  0x60, 32 },		/* lcount */
	{ 68,  0x4c, 32 },		/* sar */
	{ 69,  -1, 32 },		/* windowbase */
	{ 70,  -1, 32 },		/* windowstart */
	{ 71,  -1, 32 },		/* configid0 */
	{ 72,  -1, 32 },		/* configid1 */
	{ 73,  0x08, 32 },		/* ps */
	{ 74,  0x70, 32 },		/* threadptr */
	{ 75,  0x7c, 32 },		/* br */
	{ 76,  0x80, 32 },		/* scompare1 */
	{ 77,  0x74, 32 },		/* acclo */
	{ 78,  0x78, 32 },		/* acchi */
	{ 79,  0x84, 32 },		/* m0 */
	{ 80,  0x88, 32 },		/* m1 */
	{ 81,  0x8c, 32 },		/* m2 */
	{ 82,  0x90, 32 },		/* m3 */
	{ 83,  -1, 32 },		/* expstate */
	{ 84,  0x94, 32 },		/* f64r_lo */
	{ 85,  0x98, 32 },		/* f64r_hi */
	{ 86,  0x9c, 32 },		/* f64s */
	{ 87,  -1, 32 },		/* f0 */
	{ 88,  -1, 32 },		/* f1 */
	{ 89,  -1, 32 },		/* f2 */
	{ 90,  -1, 32 },		/* f3 */
	{ 91,  -1, 32 },		/* f4 */
	{ 92,  -1, 32 },		/* f5 */
	{ 93,  -1, 32 },		/* f6 */
	{ 94,  -1, 32 },		/* f7 */
	{ 95,  -1, 32 },		/* f8 */
	{ 96,  -1, 32 },		/* f9 */
	{ 97,  -1, 32 },		/* f10 */
	{ 98,  -1, 32 },		/* f11 */
	{ 99,  -1, 32 },		/* f12 */
	{ 100, -1, 32 },		/* f13 */
	{ 101, -1, 32 },		/* f14 */
	{ 102, -1, 32 },		/* f15 */
	{ 103, -1, 32 },		/* fcr */
	{ 104, -1, 32 },		/* fsr */
};

static const struct stack_register_offset rtos_freertos_esp32s2_stack_offsets[] = {
	{ 0,   0x04, 32 },		/* PC */
	{ 1,   0x0c, 32 },		/* A0 */
	{ 2,   0x10, 32 },		/* A1 */
	{ 3,   0x14, 32 },		/* A2 */
	{ 4,   0x18, 32 },		/* A3 */
	{ 5,   0x1c, 32 },		/* A4 */
	{ 6,   0x20, 32 },		/* A5 */
	{ 7,   0x24, 32 },		/* A6 */
	{ 8,   0x28, 32 },		/* A7 */
	{ 9,   0x2c, 32 },		/* A8 */
	{ 10,  0x30, 32 },		/* A9 */
	{ 11,  0x34, 32 },		/* A10 */
	{ 12,  0x38, 32 },		/* A11 */
	{ 13,  0x3c, 32 },		/* A12 */
	{ 14,  0x40, 32 },		/* A13 */
	{ 15,  0x44, 32 },		/* A14 */
	{ 16,  0x48, 32 },		/* A15 */
	/* A16-A63 aren't in the stack frame because they've been flushed to the stack earlier */
	{ 17,  -1, 32 },		/* A16 */
	{ 18,  -1, 32 },		/* A17 */
	{ 19,  -1, 32 },		/* A18 */
	{ 20,  -1, 32 },		/* A19 */
	{ 21,  -1, 32 },		/* A20 */
	{ 22,  -1, 32 },		/* A21 */
	{ 23,  -1, 32 },		/* A22 */
	{ 24,  -1, 32 },		/* A23 */
	{ 25,  -1, 32 },		/* A24 */
	{ 26,  -1, 32 },		/* A25 */
	{ 27,  -1, 32 },		/* A26 */
	{ 28,  -1, 32 },		/* A27 */
	{ 29,  -1, 32 },		/* A28 */
	{ 30,  -1, 32 },		/* A29 */
	{ 31,  -1, 32 },		/* A30 */
	{ 32,  -1, 32 },		/* A31 */
	{ 33,  -1, 32 },		/* A32 */
	{ 34,  -1, 32 },		/* A33 */
	{ 35,  -1, 32 },		/* A34 */
	{ 36,  -1, 32 },		/* A35 */
	{ 37,  -1, 32 },		/* A36 */
	{ 38,  -1, 32 },		/* A37 */
	{ 39,  -1, 32 },		/* A38 */
	{ 40,  -1, 32 },		/* A39 */
	{ 41,  -1, 32 },		/* A40 */
	{ 42,  -1, 32 },		/* A41 */
	{ 43,  -1, 32 },		/* A42 */
	{ 44,  -1, 32 },		/* A43 */
	{ 45,  -1, 32 },		/* A44 */
	{ 46,  -1, 32 },		/* A45 */
	{ 47,  -1, 32 },		/* A46 */
	{ 48,  -1, 32 },		/* A47 */
	{ 49,  -1, 32 },		/* A48 */
	{ 50,  -1, 32 },		/* A49 */
	{ 51,  -1, 32 },		/* A50 */
	{ 52,  -1, 32 },		/* A51 */
	{ 53,  -1, 32 },		/* A52 */
	{ 54,  -1, 32 },		/* A53 */
	{ 55,  -1, 32 },		/* A54 */
	{ 56,  -1, 32 },		/* A55 */
	{ 57,  -1, 32 },		/* A56 */
	{ 58,  -1, 32 },		/* A57 */
	{ 59,  -1, 32 },		/* A58 */
	{ 60,  -1, 32 },		/* A59 */
	{ 61,  -1, 32 },		/* A60 */
	{ 62,  -1, 32 },		/* A61 */
	{ 63,  -1, 32 },		/* A62 */
	{ 64,  -1, 32 },		/* A63 */
	{ 65,  0x4c, 32 },		/* sar */
	{ 66,  -1, 32 },		/* windowbase */
	{ 67,  -1, 32 },		/* windowstart */
	{ 68,  -1, 32 },		/* configid0 */
	{ 69,  -1, 32 },		/* configid1 */
	{ 70,  0x08, 32 },		/* ps */
	{ 71,  0x64, 32 },		/* threadptr */
	{ 72,  -1, 32 },		/* gpio_out */
};

static const struct stack_register_offset rtos_freertos_esp32_s3_stack_offsets[] = {
	{ 0,   0x04, 32 },		/* PC */
	{ 1,   0x0c, 32 },		/* A0 */
	{ 2,   0x10, 32 },		/* A1 */
	{ 3,   0x14, 32 },		/* A2 */
	{ 4,   0x18, 32 },		/* A3 */
	{ 5,   0x1c, 32 },		/* A4 */
	{ 6,   0x20, 32 },		/* A5 */
	{ 7,   0x24, 32 },		/* A6 */
	{ 8,   0x28, 32 },		/* A7 */
	{ 9,   0x2c, 32 },		/* A8 */
	{ 10,  0x30, 32 },		/* A9 */
	{ 11,  0x34, 32 },		/* A10 */
	{ 12,  0x38, 32 },		/* A11 */
	{ 13,  0x3c, 32 },		/* A12 */
	{ 14,  0x40, 32 },		/* A13 */
	{ 15,  0x44, 32 },		/* A14 */
	{ 16,  0x48, 32 },		/* A15 */
	/* A16-A63 aren't in the stack frame because they've been flushed to the stack earlier */
	{ 17,  -1, 32 },		/* A16 */
	{ 18,  -1, 32 },		/* A17 */
	{ 19,  -1, 32 },		/* A18 */
	{ 20,  -1, 32 },		/* A19 */
	{ 21,  -1, 32 },		/* A20 */
	{ 22,  -1, 32 },		/* A21 */
	{ 23,  -1, 32 },		/* A22 */
	{ 24,  -1, 32 },		/* A23 */
	{ 25,  -1, 32 },		/* A24 */
	{ 26,  -1, 32 },		/* A25 */
	{ 27,  -1, 32 },		/* A26 */
	{ 28,  -1, 32 },		/* A27 */
	{ 29,  -1, 32 },		/* A28 */
	{ 30,  -1, 32 },		/* A29 */
	{ 31,  -1, 32 },		/* A30 */
	{ 32,  -1, 32 },		/* A31 */
	{ 33,  -1, 32 },		/* A32 */
	{ 34,  -1, 32 },		/* A33 */
	{ 35,  -1, 32 },		/* A34 */
	{ 36,  -1, 32 },		/* A35 */
	{ 37,  -1, 32 },		/* A36 */
	{ 38,  -1, 32 },		/* A37 */
	{ 39,  -1, 32 },		/* A38 */
	{ 40,  -1, 32 },		/* A39 */
	{ 41,  -1, 32 },		/* A40 */
	{ 42,  -1, 32 },		/* A41 */
	{ 43,  -1, 32 },		/* A42 */
	{ 44,  -1, 32 },		/* A43 */
	{ 45,  -1, 32 },		/* A44 */
	{ 46,  -1, 32 },		/* A45 */
	{ 47,  -1, 32 },		/* A46 */
	{ 48,  -1, 32 },		/* A47 */
	{ 49,  -1, 32 },		/* A48 */
	{ 50,  -1, 32 },		/* A49 */
	{ 51,  -1, 32 },		/* A50 */
	{ 52,  -1, 32 },		/* A51 */
	{ 53,  -1, 32 },		/* A52 */
	{ 54,  -1, 32 },		/* A53 */
	{ 55,  -1, 32 },		/* A54 */
	{ 56,  -1, 32 },		/* A55 */
	{ 57,  -1, 32 },		/* A56 */
	{ 58,  -1, 32 },		/* A57 */
	{ 59,  -1, 32 },		/* A58 */
	{ 60,  -1, 32 },		/* A59 */
	{ 61,  -1, 32 },		/* A60 */
	{ 62,  -1, 32 },		/* A61 */
	{ 63,  -1, 32 },		/* A62 */
	{ 64,  -1, 32 },		/* A63 */
	{ 65,  0x58, 32 },		/* lbeg */
	{ 66,  0x5c, 32 },		/* lend */
	{ 67,  0x60, 32 },		/* lcount */
	{ 68,  0x4c, 32 },		/* sar */
	{ 69,  -1, 32 },		/* windowbase */
	{ 70,  -1, 32 },		/* windowstart */
	{ 71,  -1, 32 },		/* configid0 */
	{ 72,  -1, 32 },		/* configid1 */
	{ 73,  0x08, 32 },		/* ps */
	{ 74,  0x70, 32 },		/* threadptr */
	{ 75,  0x7c, 32 },		/* br */
	{ 76,  0x80, 32 },		/* scompare1 */
	{ 77,  0x74, 32 },		/* acclo */
	{ 78,  0x78, 32 },		/* acchi */
	{ 79,  0x84, 32 },		/* m0 */
	{ 80,  0x88, 32 },		/* m1 */
	{ 81,  0x8c, 32 },		/* m2 */
	{ 82,  0x90, 32 },		/* m3 */
	{ 83,  -1, 32 },		/* gpio_out */
	{ 84,  -1, 32 },		/* f0 */
	{ 85,  -1, 32 },		/* f1 */
	{ 86,  -1, 32 },		/* f2 */
	{ 87,  -1, 32 },		/* f3 */
	{ 88,  -1, 32 },		/* f4 */
	{ 89,  -1, 32 },		/* f5 */
	{ 90,  -1, 32 },		/* f6 */
	{ 91,  -1, 32 },		/* f7 */
	{ 92,  -1, 32 },		/* f8 */
	{ 93,  -1, 32 },		/* f9 */
	{ 94,  -1, 32 },		/* f10 */
	{ 95,  -1, 32 },		/* f11 */
	{ 96,  -1, 32 },		/* f12 */
	{ 97,  -1, 32 },		/* f13 */
	{ 98,  -1, 32 },		/* f14 */
	{ 99,  -1, 32 },		/* f15 */
	{ 100, -1, 32 },		/* fcr */
	{ 101, -1, 32 },		/* fsr */
	{ 102, -1, 32 },		/* accx_0 */
	{ 103, -1, 32 },		/* accx_1 */
	{ 104, -1, 32 },		/* qacc_h_0 */
	{ 105, -1, 32 },		/* qacc_h_1 */
	{ 106, -1, 32 },		/* qacc_h_2 */
	{ 107, -1, 32 },		/* qacc_h_3 */
	{ 108, -1, 32 },		/* qacc_h_4 */
	{ 109, -1, 32 },		/* qacc_l_0 */
	{ 110, -1, 32 },		/* qacc_l_1 */
	{ 111, -1, 32 },		/* qacc_l_2 */
	{ 112, -1, 32 },		/* qacc_l_3 */
	{ 113, -1, 32 },		/* qacc_l_4 */
	{ 114, -1, 32 },		/* sar_byte */
	{ 115, -1, 32 },		/* fft_bit_width */
	{ 116, -1, 32 },		/* ua_state_0 */
	{ 117, -1, 32 },		/* ua_state_1 */
	{ 118, -1, 32 },		/* ua_state_2 */
	{ 119, -1, 32 },		/* ua_state_3 */
	/* TODO fails if these are 128. Need to define user reg size during config? */
	{ 120, -1, 32 },		/* q0 */
	{ 121, -1, 32 },		/* q1 */
	{ 122, -1, 32 },		/* q2 */
	{ 123, -1, 32 },		/* q3 */
	{ 124, -1, 32 },		/* q4 */
	{ 125, -1, 32 },		/* q5 */
	{ 126, -1, 32 },		/* q6 */
	{ 127, -1, 32 },		/* q7 */
};

/* WARNING: There's some deeper magic going on when reading this. Please
 * refer to rtos_freertos_esp_xtensa_stack_read_voluntary for more info.
 */

static const struct stack_register_offset rtos_freertos_esp32_voluntary_stack_offsets[] = {
	{ 0,   0x14, 32 },		/* PC */
	{ 1,   0x00, 32 },		/* A0 */
	{ 2,   0x04, 32 },		/* A1 */
	{ 3,   0x08, 32 },		/* A2 */
	{ 4,   0x0c, 32 },		/* A3 */
	{ 5,   0x30, 32 },		/* A4 */
	{ 6,   0x34, 32 },		/* A5 */
	{ 7,   0x38, 32 },		/* A6 */
	{ 8,   0x3c, 32 },		/* A7 */
	{ 9,   0x40, 32 },		/* A8 */
	{ 10,  0x44, 32 },		/* A9 */
	{ 11,  0x48, 32 },		/* A10 */
	{ 12,  0x4c, 32 },		/* A11 */
	/* A12-A63 aren't in the stack frame because they've been flushed to the stack earlier or not relevant*/
	{ 13,  -1, 32 },		/* A12 */
	{ 14,  -1, 32 },		/* A13 */
	{ 15,  -1, 32 },		/* A14 */
	{ 16,  -1, 32 },		/* A15 */
	{ 17,  -1, 32 },		/* A16 */
	{ 18,  -1, 32 },		/* A17 */
	{ 19,  -1, 32 },		/* A18 */
	{ 20,  -1, 32 },		/* A19 */
	{ 21,  -1, 32 },		/* A20 */
	{ 22,  -1, 32 },		/* A21 */
	{ 23,  -1, 32 },		/* A22 */
	{ 24,  -1, 32 },		/* A23 */
	{ 25,  -1, 32 },		/* A24 */
	{ 26,  -1, 32 },		/* A25 */
	{ 27,  -1, 32 },		/* A26 */
	{ 28,  -1, 32 },		/* A27 */
	{ 29,  -1, 32 },		/* A28 */
	{ 30,  -1, 32 },		/* A29 */
	{ 31,  -1, 32 },		/* A30 */
	{ 32,  -1, 32 },		/* A31 */
	{ 33,  -1, 32 },		/* A32 */
	{ 34,  -1, 32 },		/* A33 */
	{ 35,  -1, 32 },		/* A34 */
	{ 36,  -1, 32 },		/* A35 */
	{ 37,  -1, 32 },		/* A36 */
	{ 38,  -1, 32 },		/* A37 */
	{ 39,  -1, 32 },		/* A38 */
	{ 40,  -1, 32 },		/* A39 */
	{ 41,  -1, 32 },		/* A40 */
	{ 42,  -1, 32 },		/* A41 */
	{ 43,  -1, 32 },		/* A42 */
	{ 44,  -1, 32 },		/* A43 */
	{ 45,  -1, 32 },		/* A44 */
	{ 46,  -1, 32 },		/* A45 */
	{ 47,  -1, 32 },		/* A46 */
	{ 48,  -1, 32 },		/* A47 */
	{ 49,  -1, 32 },		/* A48 */
	{ 50,  -1, 32 },		/* A49 */
	{ 51,  -1, 32 },		/* A50 */
	{ 52,  -1, 32 },		/* A51 */
	{ 53,  -1, 32 },		/* A52 */
	{ 54,  -1, 32 },		/* A53 */
	{ 55,  -1, 32 },		/* A54 */
	{ 56,  -1, 32 },		/* A55 */
	{ 57,  -1, 32 },		/* A56 */
	{ 58,  -1, 32 },		/* A57 */
	{ 59,  -1, 32 },		/* A58 */
	{ 60,  -1, 32 },		/* A59 */
	{ 61,  -1, 32 },		/* A60 */
	{ 62,  -1, 32 },		/* A61 */
	{ 63,  -1, 32 },		/* A62 */
	{ 64,  -1, 32 },		/* A63 */
	{ 65,  -1, 32 },		/* lbeg */
	{ 66,  -1, 32 },		/* lend */
	{ 67,  -1, 32 },		/* lcount */
	{ 68,  -1, 32 },		/* sar */
	{ 69,  -1, 32 },		/* windowbase */
	{ 70,  -1, 32 },		/* windowstart */
	{ 71,  -1, 32 },		/* configid0 */
	{ 72,  -1, 32 },		/* configid1 */
	{ 73,  0x18, 32 },		/* ps */
	{ 74,  -1, 32 },		/* threadptr */
	{ 75,  -1, 32 },		/* br */
	{ 76,  -1, 32 },		/* scompare1 */
	{ 77,  -1, 32 },		/* acclo */
	{ 78,  -1, 32 },		/* acchi */
	{ 79,  -1, 32 },		/* m0 */
	{ 80,  -1, 32 },		/* m1 */
	{ 81,  -1, 32 },		/* m2 */
	{ 82,  -1, 32 },		/* m3 */
	{ 83,  -1, 32 },		/* expstate */
	{ 84,  -1, 32 },		/* f64r_lo */
	{ 85,  -1, 32 },		/* f64r_hi */
	{ 86,  -1, 32 },		/* f64s */
	{ 87,  -1, 32 },		/* f0 */
	{ 88,  -1, 32 },		/* f1 */
	{ 89,  -1, 32 },		/* f2 */
	{ 90,  -1, 32 },		/* f3 */
	{ 91,  -1, 32 },		/* f4 */
	{ 92,  -1, 32 },		/* f5 */
	{ 93,  -1, 32 },		/* f6 */
	{ 94,  -1, 32 },		/* f7 */
	{ 95,  -1, 32 },		/* f8 */
	{ 96,  -1, 32 },		/* f9 */
	{ 97,  -1, 32 },		/* f10 */
	{ 98,  -1, 32 },		/* f11 */
	{ 99,  -1, 32 },		/* f12 */
	{ 100, -1, 32 },		/* f13 */
	{ 101, -1, 32 },		/* f14 */
	{ 102, -1, 32 },		/* f15 */
	{ 103, -1, 32 },		/* fcr */
	{ 104, -1, 32 },		/* fsr */
};

static const struct stack_register_offset rtos_freertos_esp32_s2_voluntary_stack_offsets[] = {
	{ 0,   0x14, 32 },		/* PC */
	{ 1,   0x00, 32 },		/* A0 */
	{ 2,   0x04, 32 },		/* A1 */
	{ 3,   0x08, 32 },		/* A2 */
	{ 4,   0x0c, 32 },		/* A3 */
	{ 5,   0x30, 32 },		/* A4 */
	{ 6,   0x34, 32 },		/* A5 */
	{ 7,   0x38, 32 },		/* A6 */
	{ 8,   0x3c, 32 },		/* A7 */
	{ 9,   0x40, 32 },		/* A8 */
	{ 10,  0x44, 32 },		/* A9 */
	{ 11,  0x48, 32 },		/* A10 */
	{ 12,  0x4c, 32 },		/* A11 */
	/* A12-A63 aren't in the stack frame because they've been flushed to the stack earlier or not relevant*/
	{ 13,  -1, 32 },		/* A12 */
	{ 14,  -1, 32 },		/* A13 */
	{ 15,  -1, 32 },		/* A14 */
	{ 16,  -1, 32 },		/* A15 */
	{ 17,  -1, 32 },		/* A16 */
	{ 18,  -1, 32 },		/* A17 */
	{ 19,  -1, 32 },		/* A18 */
	{ 20,  -1, 32 },		/* A19 */
	{ 21,  -1, 32 },		/* A20 */
	{ 22,  -1, 32 },		/* A21 */
	{ 23,  -1, 32 },		/* A22 */
	{ 24,  -1, 32 },		/* A23 */
	{ 25,  -1, 32 },		/* A24 */
	{ 26,  -1, 32 },		/* A25 */
	{ 27,  -1, 32 },		/* A26 */
	{ 28,  -1, 32 },		/* A27 */
	{ 29,  -1, 32 },		/* A28 */
	{ 30,  -1, 32 },		/* A29 */
	{ 31,  -1, 32 },		/* A30 */
	{ 32,  -1, 32 },		/* A31 */
	{ 33,  -1, 32 },		/* A32 */
	{ 34,  -1, 32 },		/* A33 */
	{ 35,  -1, 32 },		/* A34 */
	{ 36,  -1, 32 },		/* A35 */
	{ 37,  -1, 32 },		/* A36 */
	{ 38,  -1, 32 },		/* A37 */
	{ 39,  -1, 32 },		/* A38 */
	{ 40,  -1, 32 },		/* A39 */
	{ 41,  -1, 32 },		/* A40 */
	{ 42,  -1, 32 },		/* A41 */
	{ 43,  -1, 32 },		/* A42 */
	{ 44,  -1, 32 },		/* A43 */
	{ 45,  -1, 32 },		/* A44 */
	{ 46,  -1, 32 },		/* A45 */
	{ 47,  -1, 32 },		/* A46 */
	{ 48,  -1, 32 },		/* A47 */
	{ 49,  -1, 32 },		/* A48 */
	{ 50,  -1, 32 },		/* A49 */
	{ 51,  -1, 32 },		/* A50 */
	{ 52,  -1, 32 },		/* A51 */
	{ 53,  -1, 32 },		/* A52 */
	{ 54,  -1, 32 },		/* A53 */
	{ 55,  -1, 32 },		/* A54 */
	{ 56,  -1, 32 },		/* A55 */
	{ 57,  -1, 32 },		/* A56 */
	{ 58,  -1, 32 },		/* A57 */
	{ 59,  -1, 32 },		/* A58 */
	{ 60,  -1, 32 },		/* A59 */
	{ 61,  -1, 32 },		/* A60 */
	{ 62,  -1, 32 },		/* A61 */
	{ 63,  -1, 32 },		/* A62 */
	{ 64,  -1, 32 },		/* A63 */
	{ 65,  -1, 32 },		/* sar */
	{ 66,  -1, 32 },		/* windowbase */
	{ 67,  -1, 32 },		/* windowstart */
	{ 68,   1, 32 },		/* configid0 */
	{ 69,  -1, 32 },		/* configid1 */
	{ 70,  0x18, 32 },		/* ps */
	{ 71,  -1, 32 },		/* threadptr */
	{ 72,  -1, 32 },		/* gpio_out */
};

static const struct stack_register_offset rtos_freertos_esp32_s3_voluntary_stack_offsets[] = {
	{ 0,   0x14, 32 },		/* PC */
	{ 1,   0x00, 32 },		/* A0 */
	{ 2,   0x04, 32 },		/* A1 */
	{ 3,   0x08, 32 },		/* A2 */
	{ 4,   0x0c, 32 },		/* A3 */
	{ 5,   0x30, 32 },		/* A4 */
	{ 6,   0x34, 32 },		/* A5 */
	{ 7,   0x38, 32 },		/* A6 */
	{ 8,   0x3c, 32 },		/* A7 */
	{ 9,   0x40, 32 },		/* A8 */
	{ 10,  0x44, 32 },		/* A9 */
	{ 11,  0x48, 32 },		/* A10 */
	{ 12,  0x4c, 32 },		/* A11 */
	/* A12-A63 aren't in the stack frame because they've been flushed to the stack earlier or not relevant*/
	{ 13,  -1, 32 },		/* A12 */
	{ 14,  -1, 32 },		/* A13 */
	{ 15,  -1, 32 },		/* A14 */
	{ 16,  -1, 32 },		/* A15 */
	{ 17,  -1, 32 },		/* A16 */
	{ 18,  -1, 32 },		/* A17 */
	{ 19,  -1, 32 },		/* A18 */
	{ 20,  -1, 32 },		/* A19 */
	{ 21,  -1, 32 },		/* A20 */
	{ 22,  -1, 32 },		/* A21 */
	{ 23,  -1, 32 },		/* A22 */
	{ 24,  -1, 32 },		/* A23 */
	{ 25,  -1, 32 },		/* A24 */
	{ 26,  -1, 32 },		/* A25 */
	{ 27,  -1, 32 },		/* A26 */
	{ 28,  -1, 32 },		/* A27 */
	{ 29,  -1, 32 },		/* A28 */
	{ 30,  -1, 32 },		/* A29 */
	{ 31,  -1, 32 },		/* A30 */
	{ 32,  -1, 32 },		/* A31 */
	{ 33,  -1, 32 },		/* A32 */
	{ 34,  -1, 32 },		/* A33 */
	{ 35,  -1, 32 },		/* A34 */
	{ 36,  -1, 32 },		/* A35 */
	{ 37,  -1, 32 },		/* A36 */
	{ 38,  -1, 32 },		/* A37 */
	{ 39,  -1, 32 },		/* A38 */
	{ 40,  -1, 32 },		/* A39 */
	{ 41,  -1, 32 },		/* A40 */
	{ 42,  -1, 32 },		/* A41 */
	{ 43,  -1, 32 },		/* A42 */
	{ 44,  -1, 32 },		/* A43 */
	{ 45,  -1, 32 },		/* A44 */
	{ 46,  -1, 32 },		/* A45 */
	{ 47,  -1, 32 },		/* A46 */
	{ 48,  -1, 32 },		/* A47 */
	{ 49,  -1, 32 },		/* A48 */
	{ 50,  -1, 32 },		/* A49 */
	{ 51,  -1, 32 },		/* A50 */
	{ 52,  -1, 32 },		/* A51 */
	{ 53,  -1, 32 },		/* A52 */
	{ 54,  -1, 32 },		/* A53 */
	{ 55,  -1, 32 },		/* A54 */
	{ 56,  -1, 32 },		/* A55 */
	{ 57,  -1, 32 },		/* A56 */
	{ 58,  -1, 32 },		/* A57 */
	{ 59,  -1, 32 },		/* A58 */
	{ 60,  -1, 32 },		/* A59 */
	{ 61,  -1, 32 },		/* A60 */
	{ 62,  -1, 32 },		/* A61 */
	{ 63,  -1, 32 },		/* A62 */
	{ 64,  -1, 32 },		/* A63 */
	{ 65,  -1, 32 },		/* lbeg */
	{ 66,  -1, 32 },		/* lend */
	{ 67,  -1, 32 },		/* lcount */
	{ 68,  -1, 32 },		/* sar */
	{ 69,  -1, 32 },		/* windowbase */
	{ 70,  -1, 32 },		/* windowstart */
	{ 71,  -1, 32 },		/* configid0 */
	{ 72,  -1, 32 },		/* configid1 */
	{ 73,  0x18, 32 },		/* ps */
	{ 74,  -1, 32 },		/* threadptr */
	{ 75,  -1, 32 },		/* br */
	{ 76,  -1, 32 },		/* scompare1 */
	{ 77,  -1, 32 },		/* acclo */
	{ 78,  -1, 32 },		/* acchi */
	{ 79,  -1, 32 },		/* m0 */
	{ 80,  -1, 32 },		/* m1 */
	{ 81,  -1, 32 },		/* m2 */
	{ 82,  -1, 32 },		/* m3 */
	{ 83,  -1, 32 },		/* gpio_out */
	{ 84,  -1, 32 },		/* f0 */
	{ 85,  -1, 32 },		/* f1 */
	{ 86,  -1, 32 },		/* f2 */
	{ 87,  -1, 32 },		/* f3 */
	{ 88,  -1, 32 },		/* f4 */
	{ 89,  -1, 32 },		/* f5 */
	{ 90,  -1, 32 },		/* f6 */
	{ 91,  -1, 32 },		/* f7 */
	{ 92,  -1, 32 },		/* f8 */
	{ 93,  -1, 32 },		/* f9 */
	{ 94,  -1, 32 },		/* f10 */
	{ 95,  -1, 32 },		/* f11 */
	{ 96,  -1, 32 },		/* f12 */
	{ 97,  -1, 32 },		/* f13 */
	{ 98,  -1, 32 },		/* f14 */
	{ 99,  -1, 32 },		/* f15 */
	{ 100, -1, 32 },		/* fcr */
	{ 101, -1, 32 },		/* fsr */
	{ 102, -1, 32 },		/* accx_0 */
	{ 103, -1, 32 },		/* accx_1 */
	{ 104, -1, 32 },		/* qacc_h_0 */
	{ 105, -1, 32 },		/* qacc_h_1 */
	{ 106, -1, 32 },		/* qacc_h_2 */
	{ 107, -1, 32 },		/* qacc_h_3 */
	{ 108, -1, 32 },		/* qacc_h_4 */
	{ 109, -1, 32 },		/* qacc_l_0 */
	{ 110, -1, 32 },		/* qacc_l_1 */
	{ 111, -1, 32 },		/* qacc_l_2 */
	{ 112, -1, 32 },		/* qacc_l_3 */
	{ 113, -1, 32 },		/* qacc_l_4 */
	{ 114, -1, 32 },		/* sar_byte */
	{ 115, -1, 32 },		/* fft_bit_width */
	{ 116, -1, 32 },		/* ua_state_0 */
	{ 117, -1, 32 },		/* ua_state_1 */
	{ 118, -1, 32 },		/* ua_state_2 */
	{ 119, -1, 32 },		/* ua_state_3 */
	/* TODO fails if these are 128. Need to define user reg size during config? */
	{ 120, -1, 32 },		/* q0 */
	{ 121, -1, 32 },		/* q1 */
	{ 122, -1, 32 },		/* q2 */
	{ 123, -1, 32 },		/* q3 */
	{ 124, -1, 32 },		/* q4 */
	{ 125, -1, 32 },		/* q5 */
	{ 126, -1, 32 },		/* q6 */
	{ 127, -1, 32 },		/* q7 */
};

static const struct rtos_register_stacking rtos_freertos_esp32_stacking = {
	0xa0,				/* stack_registers_size: 0x9c (f64s) + 4 bytes = 160 bytes */
	-1,					/* stack_growth_direction */
	ARRAY_SIZE(rtos_freertos_esp32_stack_offsets),	/* num_output_registers */
	rtos_generic_stack_align8,	/* stack_alignment */
	rtos_freertos_esp32_stack_offsets,		/* register_offsets */
	rtos_freertos_esp_xtensa_stack_read_involuntary		/* Custom stack frame read function */
};

static const struct rtos_register_stacking rtos_freertos_esp32s2_stacking = {
	0x68,				/* stack_registers_size: 0x64 (threadptr) + 4 bytes = 104 bytes */
	-1,					/* stack_growth_direction */
	ARRAY_SIZE(rtos_freertos_esp32s2_stack_offsets),					/* num_output_registers */
	rtos_generic_stack_align8,	/* stack_alignment */
	rtos_freertos_esp32s2_stack_offsets,		/* register_offsets */
	rtos_freertos_esp_xtensa_stack_read_involuntary		/* Custom stack frame read function */
};

static const struct rtos_register_stacking rtos_freertos_esp32s3_stacking = {
	0x94,				/* stack_registers_size: 0x90 (m3) + 4 bytes = 148 bytes */
	-1,					/* stack_growth_direction */
	ARRAY_SIZE(rtos_freertos_esp32_s3_stack_offsets),	/* num_output_registers */
	rtos_generic_stack_align8,	/* stack_alignment */
	rtos_freertos_esp32_s3_stack_offsets,		/* register_offsets */
	rtos_freertos_esp_xtensa_stack_read_involuntary		/* Custom stack frame read function */
};

static const struct rtos_register_stacking rtos_freertos_voluntary_esp32_stacking = {
	0x50,				/* stack_registers_size, including 'faked' stack values */
	-1,					/* stack_growth_direction */
	ARRAY_SIZE(rtos_freertos_esp32_voluntary_stack_offsets),	/* num_output_registers */
	rtos_generic_stack_align8,	/* stack_alignment */
	rtos_freertos_esp32_voluntary_stack_offsets,	/* register_offsets */
	rtos_freertos_esp_xtensa_stack_read_voluntary		/* Custom stack frame read function */
};

static const struct rtos_register_stacking rtos_freertos_voluntary_esp32s2_stacking = {
	0x50,				/* stack_registers_size, including 'faked' stack values */
	-1,					/* stack_growth_direction */
	ARRAY_SIZE(rtos_freertos_esp32_s2_voluntary_stack_offsets),	/* num_output_registers */
	rtos_generic_stack_align8,	/* stack_alignment */
	rtos_freertos_esp32_s2_voluntary_stack_offsets,	/* register_offsets */
	rtos_freertos_esp_xtensa_stack_read_voluntary		/* Custom stack frame read function */
};

static const struct rtos_register_stacking rtos_freertos_voluntary_esp32s3_stacking = {
	0x50,				/* stack_registers_size, including 'faked' stack values */
	-1,					/* stack_growth_direction */
	ARRAY_SIZE(rtos_freertos_esp32_s3_voluntary_stack_offsets),	/* num_output_registers */
	rtos_generic_stack_align8,	/* stack_alignment */
	rtos_freertos_esp32_s3_voluntary_stack_offsets,	/* register_offsets */
	rtos_freertos_esp_xtensa_stack_read_voluntary		/* Custom stack frame read function */
};

/*
 This function uses the first word of the stack memory to see if the stack frame is from a
 voluntary or unvoluntary yield, and returns the correct stack frame info.
*/
static const struct rtos_register_stacking *rtos_freertos_esp_xtensa_pick_stacking_info(struct rtos *rtos,
	int64_t thread_id,
	int64_t stack_addr,
	const struct rtos_register_stacking *stacking,
	const struct rtos_register_stacking *voluntary_stacking)
{
	int retval;
	uint32_t stack_ptr = 0;
	uint32_t stk_exit;

	if (!rtos)
		return stacking;

	if (thread_id == 0)
		return stacking;

	/* Read the stack pointer */
	retval = target_read_u32(rtos->target, stack_addr, &stack_ptr);

	/* Read the XT_STK_EXIT variable */
	if (retval != ERROR_OK)
		return stacking;

	retval = target_read_u32(rtos->target, stack_ptr, &stk_exit);
	if (retval != ERROR_OK)
		return stacking;

	if (stk_exit)
		return stacking;

	return voluntary_stacking;

}

const struct rtos_register_stacking *rtos_freertos_esp32_pick_stacking_info(struct rtos *rtos,
	int64_t thread_id, int64_t stack_addr)
{
	return rtos_freertos_esp_xtensa_pick_stacking_info(rtos,
		thread_id, stack_addr, &rtos_freertos_esp32_stacking, &rtos_freertos_voluntary_esp32_stacking);
}

const struct rtos_register_stacking *rtos_freertos_esp32_s2_pick_stacking_info(struct rtos *rtos,
	int64_t thread_id, int64_t stack_addr)
{
	return rtos_freertos_esp_xtensa_pick_stacking_info(rtos,
		thread_id, stack_addr, &rtos_freertos_esp32s2_stacking, &rtos_freertos_voluntary_esp32s2_stacking);
}

const struct rtos_register_stacking *rtos_freertos_esp32_s3_pick_stacking_info(struct rtos *rtos,
	int64_t thread_id, int64_t stack_addr)
{
	return rtos_freertos_esp_xtensa_pick_stacking_info(rtos,
		thread_id, stack_addr, &rtos_freertos_esp32s3_stacking, &rtos_freertos_voluntary_esp32s3_stacking);
}

/*
 * Xtensa stacks are saved with the exception bit set, while the call structure is more like a normal
 * windowed call. We get rid of the exception bit so gdb can parse it into a working backtrace.
 */
static int rtos_freertos_esp_xtensa_stack_read_involuntary(struct target *target,
	int64_t stack_ptr, const struct rtos_register_stacking *stacking, uint8_t *stack_data)
{
	int retval = target_read_buffer(target, stack_ptr, stacking->stack_registers_size, stack_data);
	if (retval != ERROR_OK)
		return retval;

	stack_data[ESP_PS_REG_OFFSET] &= ~BIT(ESP_PS_EXCM_BIT); // clear exception bit in PS

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
static int rtos_freertos_esp_xtensa_stack_read_voluntary(struct target *target,
	int64_t stack_ptr, const struct rtos_register_stacking *stacking, uint8_t *stack_data)
{
	int callno;
	uint32_t prevsp;
	uint32_t xt_sol_pc_reg;

	int retval = target_read_buffer(target, stack_ptr - ESP_BASE_SAVE_AREA_LEN, 4 * 8, stack_data);
	if (retval != ERROR_OK)
		return retval;

	stack_data[ESP_BASE_SAVE_AREA_LEN + ESP_PS_REG_OFFSET] &= ~BIT(ESP_PS_EXCM_BIT);  /* clear exception bit in PS */
	xt_sol_pc_reg = le_to_h_u32(stack_data + ESP_BASE_SAVE_AREA_LEN + ESP_PC_REG_OFFSET);
	callno = (xt_sol_pc_reg >> 30) & 0x3;
	xt_sol_pc_reg = (xt_sol_pc_reg & 0x3FFFFFFF) | 0x40000000; /* Hardcoded for now. */
	h_u32_to_le(stack_data + ESP_BASE_SAVE_AREA_LEN + ESP_PC_REG_OFFSET, xt_sol_pc_reg);
	prevsp = le_to_h_u32(stack_data + 1 * sizeof(uint32_t)); /* A1 in the BS area */

	/* Fill unknown regs with dummy value */
	for (unsigned int i = 12; i < 20; i++)
		h_u32_to_le(stack_data + i * sizeof(uint32_t), 0xdeadbeef);

	if (callno == 2) /* call8 */
		retval = target_read_buffer(target, prevsp - 32, 4 * 4, stack_data + 0x30);
	else if (callno == 3) /* call12 */
		retval = target_read_buffer(target, prevsp - 48, 8 * 4, stack_data + 0x30);

	return retval;
}

const struct rtos_register_stacking *rtos_freertos_riscv_pick_stacking_info(struct rtos *rtos, int64_t thread_id, int64_t stack_addr)
{
	return &rtos_standard_riscv32_stacking;
}

// Chip-specific data for calculating Thread Local Storage (tls) address
static struct freertos_tls_info s_xtensa_tls = {
	UINT32_MAX,		/* tls_reg not populated yet */
	16,				/* tls_align */
};

static const struct freertos_tls_info s_riscv_tls = {
	GDB_REGNO_TP,	/* tls_reg */
	0,				/* tls_align */
};

const struct freertos_tls_info *rtos_freertos_get_tls_info(struct target *target)
{
	if (strncmp(target_get_gdb_arch(target), "riscv", 5) == 0) {
		return &s_riscv_tls;
	} else if (strncmp(target_get_gdb_arch(target), "xtensa", 6) == 0) {
		if (s_xtensa_tls.tls_reg != UINT32_MAX)
			return &s_xtensa_tls;
		struct reg **reg_list;
		int num_regs;
		int retval = target_get_gdb_reg_list(target, &reg_list, &num_regs, REG_CLASS_GENERAL);
		if (retval != ERROR_OK)
			return NULL;

		for (int j = 0; j < num_regs; ++j) {
			if (!strcasecmp(reg_list[j]->name, "threadptr")) {
				s_xtensa_tls.tls_reg = j;
				free(reg_list);
				return &s_xtensa_tls;
			}
		}
		free(reg_list);
	}
	return NULL;
}
