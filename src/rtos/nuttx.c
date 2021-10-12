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

#include "nuttx_header.h"
#include "rtos_nuttx_stackings.h"

#ifdef CONFIG_DISABLE_SIGNALS
#define SIG_QUEUE_NUM 0
#else
#define SIG_QUEUE_NUM 1
#endif	/* CONFIG_DISABLE_SIGNALS */

#ifdef CONFIG_DISABLE_MQUEUE
#define M_QUEUE_NUM 0
#else
#define M_QUEUE_NUM 2
#endif	/* CONFIG_DISABLE_MQUEUE */

#ifdef CONFIG_PAGING
#define PAGING_QUEUE_NUM 1
#else
#define PAGING_QUEUE_NUM 0
#endif	/* CONFIG_PAGING */

#define TASK_QUEUE_NUM (6 + SIG_QUEUE_NUM + M_QUEUE_NUM + PAGING_QUEUE_NUM)

#define NUTTX_NUM_PARAMS ((int)(sizeof(nuttx_params_list) / sizeof(struct nuttx_params)))

int rtos_thread_packet(struct connection *connection, const char *packet, int packet_size);

static bool cortexm_hasfpu(struct target *target);
static const struct rtos_register_stacking *
	cortexm_select_stackinfo(struct target *target);

static const struct rtos_register_stacking *
	esp32_select_stackinfo(struct target *target);

static int rcmd_offset(const char *cmd, const char *name);
static int nuttx_thread_packet(struct connection *connection,
	char const *packet, int packet_size);

static bool nuttx_detect_rtos(struct target *target);
static int nuttx_create(struct target *target);
static int nuttx_update_threads(struct rtos *rtos);
static int nuttx_get_thread_reg_list(struct rtos *rtos, int64_t thread_id,
	struct rtos_reg **reg_list, int *num_regs);
static int nuttx_get_symbol_list_to_lookup(symbol_table_elem_t *symbol_list[]);

struct nuttx_params {
	const char *target_name;
	const struct rtos_register_stacking *(*select_stackinfo)(struct target *target);
};

/* see nuttx/sched/nx_start.c */
static char *nuttx_symbol_list[] = {
	"g_readytorun",			/* 0: must be top of this array */
	"g_tasklisttable",
	NULL
};

/* see nuttx/include/nuttx/sched.h */
struct tcb {
	uint32_t flink;
	uint32_t blink;
	uint8_t dat[512];
};

static char *task_state_str[] = {
	"INVALID",
	"PENDING",
	"READYTORUN",
	"RUNNING",
	"INACTIVE",
	"WAIT_SEM",
#ifndef CONFIG_DISABLE_SIGNALS
	"WAIT_SIG",
#endif	/* CONFIG_DISABLE_SIGNALS */
#ifndef CONFIG_DISABLE_MQUEUE
	"WAIT_MQNOTEMPTY",
	"WAIT_MQNOTFULL",
#endif	/* CONFIG_DISABLE_MQUEUE */
#ifdef CONFIG_PAGING
	"WAIT_PAGEFILL",
#endif	/* CONFIG_PAGING */
};

struct {
	uint32_t addr;
	uint32_t prio;
} g_tasklist[TASK_QUEUE_NUM];

static int pid_offset = PID;
static int state_offset = STATE;
static int name_offset = NAME;
static int xcpreg_offset = XCPREG;
static int name_size = NAME_SIZE;

static const struct nuttx_params nuttx_params_list[] = {
	{
		.target_name      = "cortex_m",
		.select_stackinfo = cortexm_select_stackinfo,
	},
	{
		.target_name      = "hla_target",
		.select_stackinfo = cortexm_select_stackinfo,
	},
	{
		.target_name      = "esp32",
		.select_stackinfo = esp32_select_stackinfo,
	},
};

struct rtos_type nuttx_rtos = {
	.name = "NuttX",
	.detect_rtos = nuttx_detect_rtos,
	.create = nuttx_create,
	.update_threads = nuttx_update_threads,
	.get_thread_reg_list = nuttx_get_thread_reg_list,
	.get_symbol_list_to_lookup = nuttx_get_symbol_list_to_lookup,
};

static bool cortexm_hasfpu(struct target *target)
{
	uint32_t cpacr;
	int retval;
	struct armv7m_common *armv7m_target = target_to_armv7m(target);

	if (!is_armv7m(armv7m_target) || armv7m_target->fp_feature != FPv4_SP)
		return false;

	retval = target_read_u32(target, FPU_CPACR, &cpacr);
	if (retval != ERROR_OK) {
		LOG_ERROR("Could not read CPACR register to check FPU state");
		return false;
	}

	return (cpacr & 0x00F00000);
}

static const struct rtos_register_stacking *cortexm_select_stackinfo(struct target *target)
{
	return cortexm_hasfpu(target) ?
	       &nuttx_stacking_cortex_m_fpu : &nuttx_stacking_cortex_m;
}

static const struct rtos_register_stacking *esp32_select_stackinfo(struct target *target)
{
  return &nuttx_esp32_stacking;
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
	char cmd[GDB_BUFFER_SIZE / 2 + 1] = ""; /* Extra byte for null-termination */

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
		(target->rtos->symbols[1].address != 0))
		return true;
	return false;
}

static int nuttx_create(struct target *target)
{
	const struct nuttx_params *param;
	int i;

	for (i = 0; i < NUTTX_NUM_PARAMS; i++) {
		param = &nuttx_params_list[i];
		if (strcmp(target_type_name(target), param->target_name) == 0) {
			LOG_INFO("Detected target \"%s\"\n", param->target_name);
			break;
		}
	}

	if (i >= NUTTX_NUM_PARAMS) {
		LOG_ERROR("Could not find \"%s\" target in NuttX compatibility list",
			target_type_name(target));
		return -1;
	}

	/* We found a target in our list, copy its reference. */

	target->rtos->rtos_specific_params = (void *)param;
	target->rtos->gdb_thread_packet = nuttx_thread_packet;

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
	const struct nuttx_params *priv;
	const struct rtos_register_stacking *stacking;

	if (rtos == NULL)
		return -1;

	priv = (const struct nuttx_params *)rtos->rtos_specific_params;

	if (priv->select_stackinfo){
		stacking = priv->select_stackinfo(rtos->target);
	} else {
		LOG_ERROR("Can't find a way to select stacking info");
		return -2;
	}

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
