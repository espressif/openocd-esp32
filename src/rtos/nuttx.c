/***************************************************************************
 *   Copyright 2016,2017 Sony Video & Sound Products Inc.                  *
 *   Masatoshi Tateishi - Masatoshi.Tateishi@jp.sony.com                   *
 *   Masayuki Ishikawa - Masayuki.Ishikawa@jp.sony.com                     *
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

#include <jtag/jtag.h>
#include "target/target.h"
#include "target/target_type.h"
#include "target/armv7m.h"
#include "target/cortex_m.h"
#include "rtos.h"
#include "helper/log.h"
#include "helper/types.h"
#include "server/gdb_server.h"
#include "rtos_standard_stackings.h"
#include "target/esp32.h"

#include "nuttx_header.h"


int rtos_thread_packet(struct connection *connection, const char *packet, int packet_size);
static int nuttx_esp_xtensa_stack_read(struct target *target,
           int64_t stack_ptr, const struct rtos_register_stacking *stacking,
           uint8_t *stack_data);

#ifdef CONFIG_DISABLE_SIGNALS
#define SIG_QUEUE_NUM 0
#else
#define SIG_QUEUE_NUM 1
#endif /* CONFIG_DISABLE_SIGNALS */

#ifdef CONFIG_DISABLE_MQUEUE
#define M_QUEUE_NUM 0
#else
#define M_QUEUE_NUM 2
#endif /* CONFIG_DISABLE_MQUEUE */

#ifdef CONFIG_PAGING
#define PAGING_QUEUE_NUM 1
#else
#define PAGING_QUEUE_NUM 0
#endif /* CONFIG_PAGING */


#define TASK_QUEUE_NUM (6 + SIG_QUEUE_NUM + M_QUEUE_NUM + PAGING_QUEUE_NUM)

/* see nuttx/sched/nx_start.c */
static char *nuttx_symbol_list[] = {
	"g_readytorun",            /* 0: must be top of this array */
	"g_tasklisttable",
	NULL
};

/* see nuttx/include/nuttx/sched.h */
struct tcb {
	uint32_t flink;
	uint32_t blink;
	uint8_t  dat[512];
};

struct {
	uint32_t addr;
	uint32_t prio;
} g_tasklist[TASK_QUEUE_NUM];

static char *task_state_str[] = {
	"INVALID",
	"PENDING",
	"READYTORUN",
	"RUNNING",
	"INACTIVE",
	"WAIT_SEM",
#ifndef CONFIG_DISABLE_SIGNALS
	"WAIT_SIG",
#endif /* CONFIG_DISABLE_SIGNALS */
#ifndef CONFIG_DISABLE_MQUEUE
	"WAIT_MQNOTEMPTY",
	"WAIT_MQNOTFULL",
#endif /* CONFIG_DISABLE_MQUEUE */
#ifdef CONFIG_PAGING
	"WAIT_PAGEFILL",
#endif /* CONFIG_PAGING */
};

/* see arch/arm/include/armv7-m/irq_cmnvector.h */
static const struct stack_register_offset nuttx_stack_offsets_cortex_m[] = {
	{ ARMV7M_R0,	0x28, 32 },		/* r0   */
	{ ARMV7M_R1,	0x2c, 32 },		/* r1   */
	{ ARMV7M_R2,	0x30, 32 },		/* r2   */
	{ ARMV7M_R3,	0x34, 32 },		/* r3   */
	{ ARMV7M_R4,	0x08, 32 },		/* r4   */
	{ ARMV7M_R5,	0x0c, 32 },		/* r5   */
	{ ARMV7M_R6,	0x10, 32 },		/* r6   */
	{ ARMV7M_R7,	0x14, 32 },		/* r7   */
	{ ARMV7M_R8,	0x18, 32 },		/* r8   */
	{ ARMV7M_R9,	0x1c, 32 },		/* r9   */
	{ ARMV7M_R10,	0x20, 32 },		/* r10  */
	{ ARMV7M_R11,	0x24, 32 },		/* r11  */
	{ ARMV7M_R12,	0x38, 32 },		/* r12  */
	{ ARMV7M_R13,	  0,  32 },		/* sp   */
	{ ARMV7M_R14,	0x3c, 32 },		/* lr   */
	{ ARMV7M_PC,	0x40, 32 },		/* pc   */
	{ ARMV7M_xPSR,	0x44, 32 },		/* xPSR */
};

static const struct stack_register_offset nuttx_stack_offsets_cortex_m_fpu[] = {
  { ARMV7M_R0,	0x6c, 32 },		/* r0   */
	{ ARMV7M_R1,	0x70, 32 },		/* r1   */
	{ ARMV7M_R2,	0x74, 32 },		/* r2   */
	{ ARMV7M_R3,	0x78, 32 },		/* r3   */
	{ ARMV7M_R4,	0x08, 32 },		/* r4   */
	{ ARMV7M_R5,	0x0c, 32 },		/* r5   */
	{ ARMV7M_R6,	0x10, 32 },		/* r6   */
	{ ARMV7M_R7,	0x14, 32 },		/* r7   */
	{ ARMV7M_R8,	0x18, 32 },		/* r8   */
	{ ARMV7M_R9,	0x1c, 32 },		/* r9   */
	{ ARMV7M_R10,	0x20, 32 },		/* r10  */
	{ ARMV7M_R11,	0x24, 32 },		/* r11  */
	{ ARMV7M_R12,	0x7c, 32 },		/* r12  */
	{ ARMV7M_R13,	  0,  32 },		/* sp   */
	{ ARMV7M_R14,	0x80, 32 },		/* lr   */
	{ ARMV7M_PC,	0x84, 32 },		/* pc   */
	{ ARMV7M_xPSR,	0x88, 32 },	/* xPSR */
};

static const struct stack_register_offset nuttx_esp32_stack_offsets[] = {
	{ XT_REG_IDX_PC, 0x00, 32 },		/* PC */
	{ XT_REG_IDX_AR0, 0x08, 32 },		/* A0 */
	{ XT_REG_IDX_AR1, 0x0c, 32 },		/* A1 */
	{ XT_REG_IDX_AR2, 0x10, 32 },		/* A2 */
	{ XT_REG_IDX_AR3, 0x14, 32 },		/* A3 */
	{ XT_REG_IDX_AR4, 0x18, 32 },		/* A4 */
	{ XT_REG_IDX_AR5, 0x1c, 32 },		/* A5 */
	{ XT_REG_IDX_AR6, 0x20, 32 },		/* A6 */
	{ XT_REG_IDX_AR7, 0x24, 32 },		/* A7 */
	{ XT_REG_IDX_AR8, 0x28, 32 },		/* A8 */
	{ XT_REG_IDX_AR9, 0x2c, 32 },		/* A9 */
	{ XT_REG_IDX_AR10, 0x30, 32 },  /* A10 */
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
	{ XT_REG_IDX_SAR, 0x48, 32 },		  /* SAR */

	{ XT_REG_IDX_WINDOWBASE, -1, 32 },		/* windowbase */
	{ XT_REG_IDX_WINDOWSTART, -1, 32 },		/* windowstart */
	{ XT_REG_IDX_CONFIGID0, -1, 32 },		/* configid0 */
	{ XT_REG_IDX_CONFIGID1, -1, 32 },		/* configid1 */

	{ XT_REG_IDX_PS, 0x04, 32 },		/* PS */

	{ XT_REG_IDX_THREADPTR, -1, 32 },		/* threadptr */
	{ XT_REG_IDX_BR, -1, 32 },		      /* br */
	{ XT_REG_IDX_SCOMPARE1, -1, 32 },		/* scompare1 */
	{ XT_REG_IDX_ACCLO, -1, 32 },		/* acclo */
	{ XT_REG_IDX_ACCHI, -1, 32 },		/* acchi */
	{ XT_REG_IDX_M0, -1, 32 },		/* m0 */
	{ XT_REG_IDX_M1, -1, 32 },		/* m1 */
	{ XT_REG_IDX_M2, -1, 32 },		/* m2 */
	{ XT_REG_IDX_M3, -1, 32 },		/* m3 */
	{ ESP32_REG_IDX_EXPSTATE, -1, 32 },		/* expstate */
	{ ESP32_REG_IDX_F64R_LO, -1, 32 },		/* f64r_lo */
	{ ESP32_REG_IDX_F64R_HI, -1, 32 },		/* f64r_hi */
	{ ESP32_REG_IDX_F64S, -1, 32 },		    /* f64s */
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

static const struct rtos_register_stacking nuttx_stacking_cortex_m = {
	0x48,                               /* stack_registers_size */
	-1,                                 /* stack_growth_direction */
	17,                                 /* num_output_registers */
	0,                                  /* stack_alignment */
	nuttx_stack_offsets_cortex_m,       /* register_offsets */
  NULL
};

static const struct rtos_register_stacking nuttx_stacking_cortex_m_fpu = {
	0x8c,                               /* stack_registers_size */
	-1,                                 /* stack_growth_direction */
	17,                                 /* num_output_registers */
	0,                                  /* stack_alignment */
	nuttx_stack_offsets_cortex_m_fpu,   /* register_offsets */
  NULL
};

const struct rtos_register_stacking nuttx_esp32_stacking = {
	26*4,			                          /* stack_registers_size */
	-1,					                        /* stack_growth_direction */
	ESP32_NUM_REGS_G_COMMAND,				    /* num_output_registers */
	rtos_generic_stack_align8,	        /* stack_alignment */
	nuttx_esp32_stack_offsets,		      /* register_offsets */
	nuttx_esp_xtensa_stack_read         /* Custom stack frame read function */
};

static int pid_offset = PID;
static int state_offset = STATE;
static int name_offset =  NAME;
static int xcpreg_offset = XCPREG;
static int name_size = NAME_SIZE;

static int nuttx_esp_xtensa_stack_read(struct target *target,
           int64_t stack_ptr, const struct rtos_register_stacking *stacking,
           uint8_t *stack_data)
{
	int retval;

	retval = target_read_buffer(target, stack_ptr, stacking->stack_registers_size,
                              stack_data);
	if (retval != ERROR_OK)
    return retval;

	stack_data[4] &= ~0x10; /* Clear exception bit in PS */

	return retval;
}

static int rcmd_offset(const char *cmd, const char *name)
{
	if (strncmp(cmd, name, strlen(name)))
		return -1;

	if (strlen(cmd) <= strlen(name) + 1)
		return -1;

	return atoi(cmd + strlen(name));
}

static int nuttx_thread_packet(struct connection *connection,
	char const *packet, int packet_size)
{
	char cmd[GDB_BUFFER_SIZE / 2 + 1] = ""; /* Extra byte for nul-termination */

	if (!strncmp(packet, "qRcmd", 5)) {
		size_t len = unhexify((uint8_t *)cmd, packet + 6, sizeof(cmd));
		int offset;

		if (len <= 0)
			goto pass;

		offset = rcmd_offset(cmd, "nuttx.pid_offset");

		if (offset >= 0) {
			LOG_INFO("pid_offset: %d", offset);
			pid_offset = offset;
			goto retok;
		}

		offset = rcmd_offset(cmd, "nuttx.state_offset");

		if (offset >= 0) {
			LOG_INFO("state_offset: %d", offset);
			state_offset = offset;
			goto retok;
		}

		offset = rcmd_offset(cmd, "nuttx.name_offset");

		if (offset >= 0) {
			LOG_INFO("name_offset: %d", offset);
			name_offset = offset;
			goto retok;
		}

		offset = rcmd_offset(cmd, "nuttx.xcpreg_offset");

		if (offset >= 0) {
			LOG_INFO("xcpreg_offset: %d", offset);
			xcpreg_offset = offset;
			goto retok;
		}

		offset = rcmd_offset(cmd, "nuttx.name_size");

		if (offset >= 0) {
			LOG_INFO("name_size: %d", offset);
			name_size = offset;
			goto retok;
		}
	}
pass:

	return rtos_thread_packet(connection, packet, packet_size);
retok:

	gdb_put_packet(connection, "OK", 2);
	return ERROR_OK;
}

static bool nuttx_detect_rtos(struct target *target)
{
	if ((target->rtos->symbols != NULL) &&
			(target->rtos->symbols[0].address != 0) &&
			(target->rtos->symbols[1].address != 0)) {
		return true;
	}
	return false;
}

static int nuttx_create(struct target *target)
{
	target->rtos->gdb_thread_packet = nuttx_thread_packet;
	LOG_INFO("target type name = %s", target->type->name);
	return 0;
}

static int nuttx_update_threads(struct rtos *rtos)
{
	uint32_t thread_count;
	struct tcb tcb;
	int ret;
	uint32_t head;
	uint32_t tcb_addr;
	uint32_t i;
	uint8_t state;

	if (rtos->symbols == NULL) {
		LOG_ERROR("No symbols for NuttX");
		return -3;
	}

	/* free previous thread details */
	rtos_free_threadlist(rtos);

	ret = target_read_buffer(rtos->target, rtos->symbols[1].address,
		sizeof(g_tasklist), (uint8_t *)&g_tasklist);
	if (ret) {
		LOG_ERROR("target_read_buffer : ret = %d\n", ret);
		return ERROR_FAIL;
	}

	thread_count = 0;

	for (i = 0; i < TASK_QUEUE_NUM; i++) {

		if (g_tasklist[i].addr == 0)
			continue;

		ret = target_read_u32(rtos->target, g_tasklist[i].addr, &head);
		if (ret) {
			LOG_ERROR("target_read_u32 : ret = %d\n", ret);
			return ERROR_FAIL;
		}

		/* readytorun head is current thread */
		if (g_tasklist[i].addr == rtos->symbols[0].address)
			rtos->current_thread = head;


		tcb_addr = head;
		while (tcb_addr) {
			struct thread_detail *thread;
			ret = target_read_buffer(rtos->target, tcb_addr,
				sizeof(tcb), (uint8_t *)&tcb);
			if (ret) {
				LOG_ERROR("target_read_buffer : ret = %d\n",
					ret);
				return ERROR_FAIL;
			}
			thread_count++;

			rtos->thread_details = realloc(rtos->thread_details,
				sizeof(struct thread_detail) * thread_count);
			thread = &rtos->thread_details[thread_count - 1];
			thread->threadid = tcb_addr;
			thread->exists = true;

			state = tcb.dat[state_offset - 8];
			thread->extra_info_str = NULL;
			if (state < sizeof(task_state_str)/sizeof(char *)) {
				thread->extra_info_str = malloc(256);
				snprintf(thread->extra_info_str, 256, "pid:%d, %s",
				    tcb.dat[pid_offset - 8] |
				    tcb.dat[pid_offset - 8 + 1] << 8,
				    task_state_str[state]);
			}

			if (name_offset) {
				thread->thread_name_str = malloc(name_size + 1);
				snprintf(thread->thread_name_str, name_size,
				    "%s", (char *)&tcb.dat[name_offset - 8]);
			} else {
				thread->thread_name_str = malloc(sizeof("None"));
				strcpy(thread->thread_name_str, "None");
			}

			tcb_addr = tcb.flink;
		}
	}
	rtos->thread_count = thread_count;

	return 0;
}

/*
 * thread_id = tcb address;
 */
static int nuttx_get_thread_reg_list(struct rtos *rtos, int64_t thread_id,
	struct rtos_reg **reg_list, int *num_regs)
{
  if (rtos == NULL){
    return -1;
  }

  if (strcmp(target_type_name(rtos->target), "esp32") == 0){
	  return rtos_generic_stack_read(rtos->target, &nuttx_esp32_stacking,
	      (uint32_t)thread_id + xcpreg_offset, reg_list, num_regs);
  }

	int retval;

	/* Check for armv7m with *enabled* FPU, i.e. a Cortex-M4F */
	int cm4_fpu_enabled = 0;
	struct armv7m_common *armv7m_target = target_to_armv7m(rtos->target);
	if (is_armv7m(armv7m_target)) {
		if (armv7m_target->fp_feature == FPv4_SP) {
			/* Found ARM v7m target which includes a FPU */
			uint32_t cpacr;

			retval = target_read_u32(rtos->target, FPU_CPACR, &cpacr);
			if (retval != ERROR_OK) {
				LOG_ERROR("Could not read CPACR register to check FPU state");
				return -1;
			}

			/* Check if CP10 and CP11 are set to full access. */
			if (cpacr & 0x00F00000) {
				/* Found target with enabled FPU */
				cm4_fpu_enabled = 1;
			}
		}
	}

	const struct rtos_register_stacking *stacking;
	if (cm4_fpu_enabled)
		stacking = &nuttx_stacking_cortex_m_fpu;
	else
		stacking = &nuttx_stacking_cortex_m;

	return rtos_generic_stack_read(rtos->target, stacking,
	    (uint32_t)thread_id + xcpreg_offset, reg_list, num_regs);
}

static int nuttx_get_symbol_list_to_lookup(symbol_table_elem_t *symbol_list[])
{
	unsigned int i;

	*symbol_list = (symbol_table_elem_t *) calloc(1,
		sizeof(symbol_table_elem_t) * ARRAY_SIZE(nuttx_symbol_list));

	for (i = 0; i < ARRAY_SIZE(nuttx_symbol_list); i++)
		(*symbol_list)[i].symbol_name = nuttx_symbol_list[i];

	return 0;
}

struct rtos_type nuttx_rtos = {
	.name = "NuttX",
	.detect_rtos = nuttx_detect_rtos,
	.create = nuttx_create,
	.update_threads = nuttx_update_threads,
	.get_thread_reg_list = nuttx_get_thread_reg_list,
	.get_symbol_list_to_lookup = nuttx_get_symbol_list_to_lookup,
};

