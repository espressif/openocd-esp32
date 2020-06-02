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
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>. *
 ***************************************************************************/

//Espressif ToDo: Merge this with the upstream code. As is, it probably breaks the arm stuff.


#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <helper/time_support.h>
#include <jtag/jtag.h>
#include "target/target.h"
#include "target/target_type.h"
#include "rtos.h"
#include "helper/log.h"
#include "helper/types.h"
#include "rtos_standard_stackings.h"
#include "rtos_freertos_stackings.h"
#include "target/armv7m.h"
#include "target/cortex_m.h"
#include "target/smp.h"
#include "target/register.h"
#include "server/gdb_server.h"


#define FREERTOS_MAX_PRIORITIES	63
#define FREERTOS_MAX_TASKS_NUM	512

#define FreeRTOS_STRUCT(int_type, ptr_type, list_prev_offset)


struct FreeRTOS_params {
	const char *target_name;
	const unsigned char thread_count_width;
	const unsigned char pointer_width;
	const unsigned char list_next_offset;
	const unsigned char list_width;
	const unsigned char list_elem_next_offset;
	const unsigned char list_elem_content_offset;
	const unsigned char thread_stack_offset;
	const unsigned char thread_name_offset;
	const struct rtos_register_stacking *stacking_info_cm3;
	const struct rtos_register_stacking *stacking_info_cm4f;
	const struct rtos_register_stacking *stacking_info_cm4f_fpu;
	const struct rtos_register_stacking* (*stacking_info_pick_fn)(struct rtos *rtos, int64_t thread_id, int64_t stack_addr);
};

static const struct FreeRTOS_params FreeRTOS_params_list[] = {
	{
	"cortex_m",			/* target_name */
	4,						/* thread_count_width; */
	4,						/* pointer_width; */
	16,						/* list_next_offset; */
	20,						/* list_width; */
	8,						/* list_elem_next_offset; */
	12,						/* list_elem_content_offset */
	0,						/* thread_stack_offset; */
	52,						/* thread_name_offset; */
	&rtos_standard_Cortex_M3_stacking,	/* stacking_info */
	&rtos_standard_Cortex_M4F_stacking,
	&rtos_standard_Cortex_M4F_FPU_stacking,
	NULL,					/* fn to pick stacking_info */
	},
	{
	"hla_target",			/* target_name */
	4,						/* thread_count_width; */
	4,						/* pointer_width; */
	16,						/* list_next_offset; */
	20,						/* list_width; */
	8,						/* list_elem_next_offset; */
	12,						/* list_elem_content_offset */
	0,						/* thread_stack_offset; */
	52,						/* thread_name_offset; */
	&rtos_standard_Cortex_M3_stacking,	/* stacking_info */
	&rtos_standard_Cortex_M4F_stacking,
	&rtos_standard_Cortex_M4F_FPU_stacking,
	NULL,					/* fn to pick stacking_info */
	},
	{
	"nds32_v3",			/* target_name */
	4,						/* thread_count_width; */
	4,						/* pointer_width; */
	16,						/* list_next_offset; */
	20,						/* list_width; */
	8,						/* list_elem_next_offset; */
	12,						/* list_elem_content_offset */
	0,						/* thread_stack_offset; */
	52,						/* thread_name_offset; */
	&rtos_standard_NDS32_N1068_stacking,	/* stacking_info */
	&rtos_standard_Cortex_M4F_stacking,
	&rtos_standard_Cortex_M4F_FPU_stacking,
	NULL,					/* fn to pick stacking_info */
	},
	{
	"esp32",				/* target_name */
	4,						/* thread_count_width; */
	4,						/* pointer_width; */
	16,						/* list_next_offset; */
	20,						/* list_width; */
	8,						/* list_elem_next_offset; */
	12,						/* list_elem_content_offset */
	0,						/* thread_stack_offset; */
	56,						/* thread_name_offset; */
	NULL,					/* stacking_info */
	NULL,
	NULL,
	rtos_freertos_esp32_pick_stacking_info, /* fn to pick stacking_info */
	},
	{
	"esp32s2",				/* target_name */
	4,						/* thread_count_width; */
	4,						/* pointer_width; */
	16,						/* list_next_offset; */
	20,						/* list_width; */
	8,						/* list_elem_next_offset; */
	12,						/* list_elem_content_offset */
	0,						/* thread_stack_offset; */
	52,						/* thread_name_offset; */
	NULL,					/* stacking_info */
	NULL,
	NULL,
	rtos_freertos_esp32_s2_pick_stacking_info, /* fn to pick stacking_info */
	},
};

#define FREERTOS_NUM_PARAMS ((int)(sizeof(FreeRTOS_params_list)/sizeof(struct FreeRTOS_params)))

struct current_thread {
	int64_t threadid;
	int32_t core_id;
	struct current_thread *next;
};

struct FreeRTOS_data
{
	const struct FreeRTOS_params *params;
	uint32_t nr_cpus;
	uint8_t *curr_threads_handles_buff;
};

static bool FreeRTOS_detect_rtos(struct target *target);
static int FreeRTOS_create(struct target *target);
static int FreeRTOS_update_threads(struct rtos *rtos);
static int FreeRTOS_get_thread_reg_list(struct rtos *rtos, int64_t thread_id,
		struct rtos_reg **reg_list, int *num_regs);
static int FreeRTOS_get_symbol_list_to_lookup(symbol_table_elem_t *symbol_list[]);
static int FreeRTOS_post_reset_cleanup(struct target *target);
static int FreeRTOS_clean(struct target *target);
static int FreeRTOS_smp_init(struct target *target);

struct rtos_type FreeRTOS_rtos = {
	.name = "FreeRTOS",

	.detect_rtos = FreeRTOS_detect_rtos,
	.create = FreeRTOS_create,
	.smp_init = FreeRTOS_smp_init,
	.update_threads = FreeRTOS_update_threads,
	.get_thread_reg_list = FreeRTOS_get_thread_reg_list,
	.get_symbol_list_to_lookup = FreeRTOS_get_symbol_list_to_lookup,
	.clean = FreeRTOS_clean,
	.post_reset_cleanup = FreeRTOS_post_reset_cleanup,
};

enum FreeRTOS_symbol_values {
	FreeRTOS_VAL_pxCurrentTCB = 0,
	FreeRTOS_VAL_pxReadyTasksLists = 1,
	FreeRTOS_VAL_xDelayedTaskList1 = 2,
	FreeRTOS_VAL_xDelayedTaskList2 = 3,
	FreeRTOS_VAL_pxDelayedTaskList = 4,
	FreeRTOS_VAL_pxOverflowDelayedTaskList = 5,
	FreeRTOS_VAL_xPendingReadyList = 6,
	FreeRTOS_VAL_xTasksWaitingTermination = 7,
	FreeRTOS_VAL_xSuspendedTaskList = 8,
	FreeRTOS_VAL_uxCurrentNumberOfTasks = 9,
	FreeRTOS_VAL_uxTopUsedPriority = 10,
	FreeRTOS_VAL_port_interruptNesting = 11,
};

struct symbols {
	const char *name;
	bool optional;
};

static const struct symbols FreeRTOS_symbol_list[] = {
	{ "pxCurrentTCB", false },
	{ "pxReadyTasksLists", false },
	{ "xDelayedTaskList1", false },
	{ "xDelayedTaskList2", false },
	{ "pxDelayedTaskList", false },
	{ "pxOverflowDelayedTaskList", false },
	{ "xPendingReadyList", false },
	{ "xTasksWaitingTermination", true }, /* Only if INCLUDE_vTaskDelete */
	{ "xSuspendedTaskList", true }, /* Only if INCLUDE_vTaskSuspend */
	{ "uxCurrentNumberOfTasks", false },
	{ "uxTopUsedPriority", true }, /* Unavailable since v7.5.3 */
	{ "port_interruptNesting", true },
	{ NULL, false }
};

static int FreeRTOS_smp_init(struct target *target)
{
	struct target_list *head;
	/* keep only target->rtos */
	struct rtos *rtos = target->rtos;
	struct FreeRTOS_data *rtos_data =
		(struct FreeRTOS_data *)rtos->rtos_specific_params;
	head = target->head;

	LOG_DEBUG("FreeRTOS_smp_init");
	while (head != (struct target_list *)NULL) {
		if (head->target->rtos != rtos) {
			struct FreeRTOS_data *smp_rtos_data =
				(struct FreeRTOS_data *)head->target->rtos->
				rtos_specific_params;
			/*  remap smp target on rtos  */
			free(head->target->rtos);
			head->target->rtos = rtos;
			free(smp_rtos_data);
			rtos_data->nr_cpus++;
		}
		head = head->next;
	}
	// allocate buffer for thread handles only once in order to avoid its allocation omn every xxx_update_threads() call
	if (rtos_data->params->pointer_width == 8) {
		rtos_data->curr_threads_handles_buff = calloc(rtos_data->nr_cpus, sizeof(uint64_t));
	}
	else if (rtos_data->params->pointer_width == 4) {
		rtos_data->curr_threads_handles_buff = calloc(rtos_data->nr_cpus, sizeof(uint32_t));
	} else {
		LOG_ERROR("Unsupported OS pointer width %u!", rtos_data->params->pointer_width);
		return JIM_ERR;
	}
	if (rtos_data->curr_threads_handles_buff == NULL) {
		LOG_ERROR("Failed to allocate current threads handles buffer!");
		return JIM_ERR;
	}
	return ERROR_OK;
}

static int target_buffer_get_uint(struct target *target, uint8_t width, uint8_t *buff, uint64_t *val)
{
	switch (width)
	{
	case 8:
		*val = target_buffer_get_u64(target, buff);
		break;
	case 4:
		*val = target_buffer_get_u32(target, buff);
		break;
	case 2:
		*val = target_buffer_get_u16(target, buff);
		break;
	case 1:
		*val = *buff;
		break;
	default:
		LOG_ERROR("Unsupported target integer of width %u!", width);
		return ERROR_FAIL;
	}
	return ERROR_OK;
}

static int target_buffer_read_uint(struct target *target, target_addr_t address,
		uint8_t width, uint64_t *val)
{
	int retval;

	*val = 0;
	switch (width)
	{
	case 8:
		retval = target_read_u64(target, address, val);
		break;
	case 4:
	{
		uint32_t v32;
		retval = target_read_u32(target, address, &v32);
		*val = v32;
		break;
	}
	case 2:
	{
		uint16_t v16;
		retval = target_read_u16(target, address, &v16);
		*val = v16;
		break;
	}
	case 1:
	{
		uint8_t v8;
		retval = target_read_u8(target, address, &v8);
		*val = v8;
		break;
	}
	default:
		LOG_ERROR("Unsupported target integer of width %u!", width);
		return ERROR_FAIL;
	}

	return retval;
}

static int target_buffer_write_uint(struct target *target, target_addr_t address,
		uint8_t width, uint64_t val)
{
	int retval;

	switch (width)
	{
	case 8:
	{
		retval = target_write_u64(target, address, val);
		break;
	}
	case 4:
	{
		retval = target_write_u32(target, address, (uint32_t)val);
		break;
	}
	case 2:
	{
		retval = target_write_u16(target, address, (uint16_t)val);
		break;
	}
	case 1:
	{
		retval = target_write_u8(target, address, (uint8_t)val);
		break;
	}
	default:
		LOG_ERROR("Unsupported target integer of width %u!", width);
		return ERROR_FAIL;
	}

	return retval;
}

static inline threadid_t FreeRTOS_current_threadid_from_target(struct target *target)
{
	struct FreeRTOS_data *rtos_data = (struct FreeRTOS_data *)target->rtos->rtos_specific_params;
	uint64_t tid = 0;

	int retval = target_buffer_get_uint(target, rtos_data->params->pointer_width,
				&rtos_data->curr_threads_handles_buff[target->coreid*rtos_data->params->pointer_width], &tid);
	if (retval != ERROR_OK) {
		return 0;
	}
	LOG_DEBUG("Curr thread 0x%x on target [%s]", (uint32_t)tid, target_name(target));
	return tid;
}

static int FreeRTOS_target_for_threadid(struct connection *connection, int64_t thread_id, struct target **p_target)
{
	struct target *target = get_target_from_connection(connection);
	struct target_list *head;

	LOG_DEBUG("Find target for thr 0x%x", (uint32_t)thread_id);
	if (target->smp) {
		/* Find the thread with that thread_id */
		foreach_smp_target(head, target->head) {
			struct target *current_target = head->target;
			if (thread_id == FreeRTOS_current_threadid_from_target(current_target)) {
				*p_target = current_target;
				return ERROR_OK;
			}
		}
	}
	*p_target = target;
	return ERROR_OK;
}

static bool FreeRTOS_halt_reason_priority_higher(enum target_debug_reason old, enum target_debug_reason new)
{
	const uint8_t debug_reason_prios[DBG_REASON_NUM] = {
	1, // DBG_REASON_DBGRQ,
	3, // DBG_REASON_BREAKPOINT,
	2, // DBG_REASON_WATCHPOINT,
	3, // DBG_REASON_WPTANDBKPT,
	4, // DBG_REASON_SINGLESTEP,
	0, // DBG_REASON_NOTHALTED,
	0, // DBG_REASON_EXIT,
	5, // DBG_REASON_EXC_CATCH,
	0, // DBG_REASON_UNDEFINED,
	};
	return debug_reason_prios[new] > debug_reason_prios[old];
}

static threadid_t FreeRTOS_smp_get_current_thread(struct rtos *rtos)
{
	enum target_debug_reason current_reason = DBG_REASON_UNDEFINED;
	struct target_list *head;
	threadid_t current_thread = 0;

	/* loop over all cores */
	foreach_smp_target(head, rtos->target->head) {
		struct target *current_target = head->target;
		if (!target_was_examined(current_target))
			continue;

		if (current_target->state != TARGET_HALTED)
			continue;

		threadid_t tid = FreeRTOS_current_threadid_from_target(current_target);

		LOG_DEBUG("FreeRTOS_current_threadid_from_target 0x%" PRIx64" @ core %d", tid, current_target->coreid);

		/* find an interesting core to set as current */
		if (FreeRTOS_halt_reason_priority_higher(current_reason, current_target->debug_reason)) {
			current_reason = current_target->debug_reason;
			current_thread = tid;
		} else if (current_reason == current_target->debug_reason && tid == rtos->current_threadid) {
			/* prefer the thread selected by GDB */
			current_thread = tid;
		}
	}

	if (current_thread != 0)
		return current_thread;

	return FreeRTOS_current_threadid_from_target(rtos->target);
}

/* TODO: */
/* may be problems reading if sizes are not 32 bit long integers. */
/* test mallocs for failure */

static int FreeRTOS_update_threads(struct rtos *rtos)
{
	int i = 0;
	int retval;
	uint32_t tasks_found = 0;
	struct FreeRTOS_data *rtos_data;
	uint64_t thread_list_size = 0;
	uint8_t uint_buff[sizeof(uint64_t)];
	struct target_list *head;
	struct target *target;

	LOG_DEBUG("FreeRTOS_update_threads");

	if (rtos->rtos_specific_params == NULL)
		return ERROR_FAIL;

	if (rtos->symbols == NULL) {
		LOG_WARNING("No symbols for FreeRTOS!");
		return ERROR_FAIL;
	}

	if (rtos->symbols[FreeRTOS_VAL_uxCurrentNumberOfTasks].address == 0) {
		LOG_ERROR("Don't have the number of threads in FreeRTOS!");
		return ERROR_FAIL;
	}
	if (rtos->symbols[FreeRTOS_VAL_uxTopUsedPriority].address == 0) {
		LOG_ERROR("FreeRTOS: uxTopUsedPriority is not defined, consult the OpenOCD manual for a work-around!");
		return ERROR_FAIL;
	}

	if (rtos->target->smp) {
		/* find target in HALTED state */
		foreach_smp_target(head, rtos->target->head) {
			target = head->target;
			if (target_was_examined(target) && target->state == TARGET_HALTED)
				break;
		}
		if (head == NULL) {
			LOG_ERROR("Failed to find HALTED core!");
			return ERROR_FAIL;
		}
	} else {
		target = rtos->target;
	}

	if (target->state != TARGET_HALTED) {
		LOG_WARNING("Target not HALTED!");
	}

	rtos_data = (struct FreeRTOS_data *)rtos->rtos_specific_params;
	retval = target_buffer_read_uint(target,
			rtos->symbols[FreeRTOS_VAL_uxCurrentNumberOfTasks].address,
			rtos_data->params->thread_count_width,
			&thread_list_size);
	if (retval != ERROR_OK) {
		LOG_ERROR("Could not read FreeRTOS thread count from target!");
		return retval;
	}
	if (thread_list_size > FREERTOS_MAX_TASKS_NUM) {
		LOG_ERROR("Too large number of threads %"PRIu64"!", thread_list_size);
		return ERROR_FAIL;
	}
	LOG_DEBUG("Read uxCurrentNumberOfTasks at 0x%" PRIx64 ", value %"PRIu64,
				rtos->symbols[FreeRTOS_VAL_uxCurrentNumberOfTasks].address,
				thread_list_size);

	/* wipe out previous thread details if any */
	rtos_free_threadlist(rtos);

	/* read the current threads */
	retval = target_read_buffer(target,
		rtos->symbols[FreeRTOS_VAL_pxCurrentTCB].address,
		rtos_data->params->pointer_width * rtos_data->nr_cpus,
		target->smp ? rtos_data->curr_threads_handles_buff : uint_buff);
	if (retval != ERROR_OK) {
		LOG_ERROR("Could not read FreeRTOS current threads!");
		return retval;
	}
	if (target->smp) {
		rtos->current_thread = FreeRTOS_smp_get_current_thread(rtos);
		LOG_DEBUG("FreeRTOS_current_thread 0x%" PRIx64, rtos->current_thread);
	} else {
		uint64_t thr_id;
		retval = target_buffer_get_uint(target, rtos_data->params->pointer_width, uint_buff, &thr_id);
		if (retval != ERROR_OK) {
			return retval;
		}
		rtos->current_thread = thr_id;
	}
	LOG_DEBUG("FreeRTOS: Read pxCurrentTCB at 0x%" PRIx64 ", value 0x%" PRIx64,
										rtos->symbols[FreeRTOS_VAL_pxCurrentTCB].address,
										rtos->current_thread);

	if ((thread_list_size  == 0) || (rtos->current_thread == 0)) {
		/* Either : No RTOS threads - there is always at least the current execution though */
		/* OR     : No current thread - all threads suspended - show the current execution
		 * of idling */
		char tmp_str[] = "Current Execution";
		thread_list_size++;
		tasks_found++;
		rtos->thread_details = calloc(thread_list_size, sizeof(struct thread_detail));
		if (!rtos->thread_details) {
			LOG_ERROR("Error allocating memory for %"PRIu64" threads", thread_list_size);
			return ERROR_FAIL;
		}
		rtos->thread_details->threadid = 0;
		rtos->thread_details->exists = true;
		rtos->thread_details->extra_info_str = NULL;
		rtos->thread_details->thread_name_str = strdup(tmp_str);
		rtos->current_thread = 0;

		if (thread_list_size == 1) {
			rtos->thread_count = 1;
			return ERROR_OK;
		}
	} else {
		/* create space for new thread details */
		rtos->thread_details = calloc(thread_list_size, sizeof(struct thread_detail));
		if (!rtos->thread_details) {
			LOG_ERROR("Error allocating memory for %"PRIu64" threads", thread_list_size);
			return ERROR_FAIL;
		}
	}

	/* Find out how many lists are needed to be read from pxReadyTasksLists, */
	int64_t max_used_priority = 0;
	retval = target_buffer_read_uint(target,
			rtos->symbols[FreeRTOS_VAL_uxTopUsedPriority].address,
			rtos_data->params->pointer_width,
			(uint64_t *)&max_used_priority);
	if (retval != ERROR_OK)
		return retval;
	LOG_DEBUG("Read uxTopUsedPriority at 0x%" PRIx64 ", value %" PRId64,
										rtos->symbols[FreeRTOS_VAL_uxTopUsedPriority].address,
										max_used_priority);
	if (max_used_priority > FREERTOS_MAX_PRIORITIES) {
		LOG_ERROR("FreeRTOS maximum used priority is unreasonably big, not proceeding: %" PRId64 "",
			max_used_priority);
		return ERROR_FAIL;
	}

	symbol_address_t *list_of_lists = malloc(sizeof(symbol_address_t) *
											(max_used_priority+1+5));
	if (!list_of_lists) {
		LOG_ERROR("Error allocating memory for %" PRId64 " priorities", max_used_priority);
		return ERROR_FAIL;
	}

	int num_lists;
	for (num_lists = 0; num_lists <= max_used_priority; num_lists++)
		list_of_lists[num_lists] = rtos->symbols[FreeRTOS_VAL_pxReadyTasksLists].address +
			num_lists * rtos_data->params->list_width;

	list_of_lists[num_lists++] = rtos->symbols[FreeRTOS_VAL_xDelayedTaskList1].address;
	list_of_lists[num_lists++] = rtos->symbols[FreeRTOS_VAL_xDelayedTaskList2].address;
	list_of_lists[num_lists++] = rtos->symbols[FreeRTOS_VAL_xPendingReadyList].address;
	list_of_lists[num_lists++] = rtos->symbols[FreeRTOS_VAL_xSuspendedTaskList].address;
	list_of_lists[num_lists++] = rtos->symbols[FreeRTOS_VAL_xTasksWaitingTermination].address;

	for (i = 0; i < num_lists; i++) {
		if (list_of_lists[i] == 0)
			continue;
		/* Read the number of threads in this list */
		int64_t list_thread_count = 0;
		retval = target_buffer_read_uint(target,
				list_of_lists[i],
				rtos_data->params->thread_count_width,
				(uint64_t *)&list_thread_count);
		if (retval != ERROR_OK) {
			LOG_ERROR("Error reading number of threads in FreeRTOS thread list");
			free(list_of_lists);
			return retval;
		}
		LOG_DEBUG("FreeRTOS: Read thread count for list %d at 0x%" PRIx64 ", value %" PRId64,
										i, list_of_lists[i], list_thread_count);
		if (list_thread_count == 0)
			continue;
		/* Read the location of first list item */
		uint64_t prev_list_elem_ptr = -1;
		uint64_t list_elem_ptr = 0;
		retval = target_buffer_read_uint(target,
				list_of_lists[i] + rtos_data->params->list_next_offset,
				rtos_data->params->pointer_width,
				&list_elem_ptr);
		if (retval != ERROR_OK) {
			LOG_ERROR("Error reading first thread item location in FreeRTOS thread list");
			free(list_of_lists);
			return retval;
		}
		LOG_DEBUG("FreeRTOS: Read first item for list %d at 0x%" PRIx64 ", value 0x%" PRIx64,
										i, list_of_lists[i] + rtos_data->params->list_next_offset, list_elem_ptr);

		while ((list_thread_count > 0) && (list_elem_ptr != 0) &&
				(list_elem_ptr != prev_list_elem_ptr) &&
				(tasks_found < thread_list_size)) {
			/* Get the location of the thread structure. */
			retval = target_buffer_read_uint(target,
					list_elem_ptr + rtos_data->params->list_elem_content_offset,
					rtos_data->params->pointer_width,
					(uint64_t *)&rtos->thread_details[tasks_found].threadid);
			if (retval != ERROR_OK) {
				LOG_ERROR("Error reading thread list item object in FreeRTOS thread list");
				free(list_of_lists);
				return retval;
			}
			LOG_DEBUG("FreeRTOS: Read Thread ID at 0x%" PRIx64 ", value 0x%" PRIx64 " %i",
										list_elem_ptr + rtos_data->params->list_elem_content_offset,
										rtos->thread_details[tasks_found].threadid, (unsigned int) rtos->thread_details[tasks_found].threadid);

			/* get thread name */
			#define FREERTOS_THREAD_NAME_STR_SIZE (200)
			char tmp_str[FREERTOS_THREAD_NAME_STR_SIZE];
			/* Read the thread name */
			retval = target_read_buffer(target,
					rtos->thread_details[tasks_found].threadid + rtos_data->params->thread_name_offset,
					FREERTOS_THREAD_NAME_STR_SIZE,
					(uint8_t *)&tmp_str);
			if (retval != ERROR_OK) {
				LOG_ERROR("Error reading FreeRTOS thread name");
				free(list_of_lists);
				return retval;
			}
			tmp_str[FREERTOS_THREAD_NAME_STR_SIZE-1] = '\x00';
			LOG_DEBUG("FreeRTOS: Read Thread Name at 0x%" PRIx64 ", value \"%s\"",
										rtos->thread_details[tasks_found].threadid + rtos_data->params->thread_name_offset,
										tmp_str);

			if (tmp_str[0] == '\x00')
				strcpy(tmp_str, "No Name");

			rtos->thread_details[tasks_found].thread_name_str = strdup(tmp_str);
			if (rtos->thread_details[tasks_found].thread_name_str == NULL) {
				LOG_ERROR("Failed to alloc mem for thread name!");
				free(list_of_lists);
				return ERROR_FAIL;
			}
			rtos->thread_details[tasks_found].exists = true;

			if (target->smp) {
				foreach_smp_target(head, target->head) {
					struct target *current_target = head->target;
					if (!target_was_examined(current_target))
						continue;
					LOG_DEBUG("Check thread 0x%" PRIx64 " %s @ core %d", rtos->thread_details[tasks_found].threadid, rtos->thread_details[tasks_found].thread_name_str, current_target->coreid);
					if (rtos->thread_details[tasks_found].threadid == FreeRTOS_current_threadid_from_target(current_target)) {
						retval = asprintf(&rtos->thread_details[tasks_found].extra_info_str, "State: Running @CPU%d", current_target->coreid);
						if (retval == -1) {
							LOG_ERROR("Failed to alloc mem for thread extra info!");
							free(rtos->thread_details[tasks_found].thread_name_str);
							free(list_of_lists);
							return ERROR_FAIL;
						}
					}
				}
			}
			tasks_found++;
			list_thread_count--;

			prev_list_elem_ptr = list_elem_ptr;
			list_elem_ptr = 0;
			retval = target_buffer_read_uint(target,
					prev_list_elem_ptr + rtos_data->params->list_elem_next_offset,
					rtos_data->params->pointer_width,
					&list_elem_ptr);
			if (retval != ERROR_OK) {
				LOG_ERROR("Error reading next thread item location in FreeRTOS thread list");
				free(list_of_lists);
				return retval;
			}
			LOG_DEBUG("FreeRTOS: Read next thread location at 0x%" PRIx64 ", value 0x%" PRIx64,
										prev_list_elem_ptr + rtos_data->params->list_elem_next_offset,
										list_elem_ptr);
		}
	}
	free(list_of_lists);
	rtos->thread_count = tasks_found;
	return 0;
}

static int FreeRTOS_get_thread_reg_list(struct rtos *rtos, int64_t thread_id,
		struct rtos_reg **reg_list, int *num_regs)
{
	int retval;
	int64_t stack_ptr = 0;
	struct target *current_target = NULL;

	if (rtos == NULL)
		return -1;

	if (thread_id == 0)
		return -2;

	if (rtos->rtos_specific_params == NULL)
		return -1;

	struct FreeRTOS_data* rtos_data = (struct FreeRTOS_data *)rtos->rtos_specific_params;

	LOG_DEBUG("FreeRTOS_get_thread_reg_list thread_id=0x%x", (uint32_t)thread_id);

	// registers for threads currently running on CPUs are not on task's stack and
	// should retrieved from reg caches via target_get_gdb_reg_list
	if (rtos->target->smp) {
		struct target_list *head;
		foreach_smp_target(head, rtos->target->head) {
			if (thread_id == FreeRTOS_current_threadid_from_target(head->target)) {
				current_target = head->target;
				break;
			}
		}
	} else if (thread_id == rtos->current_thread) {
		current_target = rtos->target;
	}
	if (current_target) {
		if (!target_was_examined(current_target))
			return ERROR_FAIL;

		struct reg **gdb_reg_list;
		retval = target_get_gdb_reg_list(current_target, &gdb_reg_list, num_regs,
				REG_CLASS_GENERAL);
		if (retval != ERROR_OK)
			return retval;

		*reg_list = calloc(*num_regs, sizeof(struct rtos_reg));
		if (*reg_list == NULL) {
			free(gdb_reg_list);
			return ERROR_FAIL;
		}

		for (int i = 0; i < *num_regs; i++) {
			(*reg_list)[i].number = (*gdb_reg_list)[i].number;
			(*reg_list)[i].size = (*gdb_reg_list)[i].size;
			memcpy((*reg_list)[i].value, (*gdb_reg_list)[i].value,
				((*reg_list)[i].size + 7) / 8);
		}
		free(gdb_reg_list);
		return ERROR_OK;
	}

	/* Read the stack pointer */
	retval = target_read_buffer(rtos->target,
			thread_id + rtos_data->params->thread_stack_offset,
			rtos_data->params->pointer_width,
			(uint8_t *)&stack_ptr);
	if (retval != ERROR_OK) {
		LOG_ERROR("Error reading stack frame from FreeRTOS thread");
		return retval;
	}
	LOG_DEBUG("FreeRTOS: Read stack pointer at 0x%" PRIx64 ", value 0x%" PRIx64,
										thread_id + rtos_data->params->thread_stack_offset,
										stack_ptr);
	if (rtos_data->params->stacking_info_pick_fn) {
		retval = rtos_generic_stack_read(rtos->target,
				rtos_data->params->stacking_info_pick_fn(rtos, thread_id, thread_id + rtos_data->params->thread_stack_offset),
				stack_ptr, reg_list, num_regs);
		return retval;
	}

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

	if (cm4_fpu_enabled == 1) {
		/* Read the LR to decide between stacking with or without FPU */
		uint32_t LR_svc = 0;
		retval = target_read_buffer(rtos->target,
				stack_ptr + 0x20,
				rtos_data->params->pointer_width,
				(uint8_t *)&LR_svc);
		if (retval != ERROR_OK) {
			LOG_OUTPUT("Error reading stack frame from FreeRTOS thread");
			return retval;
		}
		if ((LR_svc & 0x10) == 0)
			return rtos_generic_stack_read(rtos->target, rtos_data->params->stacking_info_cm4f_fpu, stack_ptr, reg_list, num_regs);
		else
			return rtos_generic_stack_read(rtos->target, rtos_data->params->stacking_info_cm4f, stack_ptr, reg_list, num_regs);
	} else
		return rtos_generic_stack_read(rtos->target, rtos_data->params->stacking_info_cm3, stack_ptr, reg_list, num_regs);

	return -1;
}

static int FreeRTOS_get_symbol_list_to_lookup(symbol_table_elem_t *symbol_list[])
{
	unsigned int i;

	*symbol_list = calloc(
			ARRAY_SIZE(FreeRTOS_symbol_list), sizeof(symbol_table_elem_t));

	for (i = 0; i < ARRAY_SIZE(FreeRTOS_symbol_list); i++) {
		(*symbol_list)[i].symbol_name = FreeRTOS_symbol_list[i].name;
		(*symbol_list)[i].optional = FreeRTOS_symbol_list[i].optional;
	}

	return 0;
}

static int FreeRTOS_post_reset_cleanup(struct target *target)
{
	LOG_DEBUG("FreeRTOS_post_reset_cleanup");

	if ((target->rtos->symbols != NULL) &&
			(target->rtos->symbols[FreeRTOS_VAL_uxCurrentNumberOfTasks].address != 0)) {
		struct FreeRTOS_data *rtos_data = (struct FreeRTOS_data *)target->rtos->rtos_specific_params;
		int ret = target_buffer_write_uint(target,
				target->rtos->symbols[FreeRTOS_VAL_uxCurrentNumberOfTasks].address,
				rtos_data->params->thread_count_width,
				0);
		if (ret != ERROR_OK) {
			LOG_ERROR("Failed to clear uxCurrentNumberOfTasks!");
			return ret;
		}
		if (target->smp) {
			// clear pxCurrentTCB for all cores
			struct target_list *head;
			foreach_smp_target(head, target->head) {
				struct target *current_target = head->target;
				if (!target_was_examined(current_target))
					continue;
				ret = target_buffer_write_uint(target,
						target->rtos->symbols[FreeRTOS_VAL_pxCurrentTCB].address + current_target->coreid*rtos_data->params->pointer_width,
						rtos_data->params->pointer_width,
						0);
				if (ret != ERROR_OK) {
					LOG_ERROR("Failed to clear pxCurrentTCB for core %d!", current_target->coreid);
					return ret;
				}
			}
		} else {
			ret = target_buffer_write_uint(target,
					target->rtos->symbols[FreeRTOS_VAL_pxCurrentTCB].address,
					rtos_data->params->pointer_width,
					0);
			if (ret != ERROR_OK) {
				LOG_ERROR("Failed to clear pxCurrentTCB!");
				return ret;
			}
		}
		/* wipe out previous thread details if any */
		rtos_free_threadlist(target->rtos);
	}
	target->rtos->current_threadid = -1;
	return ERROR_OK;
}

static int FreeRTOS_clean(struct target *target)
{
	struct FreeRTOS_data *rtos_data = (struct FreeRTOS_data *)target->rtos->rtos_specific_params;

	LOG_DEBUG("FreeRTOS_clean");
	// if rtos_auto_detect is true FreeRTOS_create() will be called upon receiption of the first 'qSymbol',
	// so we can free resources
	// if rtos_auto_detect is false FreeRTOS_create() is called only once upon target creation
	// FreeRTOS_clean() is called every time GDB initiates new connection
	// TODO: fix this in GDB server code
	if (!target->rtos_auto_detect)
		return ERROR_OK;

	if (target->smp) {
		free(rtos_data->curr_threads_handles_buff);
	}
	free(rtos_data);
	target->rtos->rtos_specific_params = NULL;
	return ERROR_OK;
}

static bool FreeRTOS_detect_rtos(struct target *target)
{
	if ((target->rtos->symbols != NULL) &&
			(target->rtos->symbols[FreeRTOS_VAL_pxReadyTasksLists].address != 0)) {
		/* looks like FreeRTOS */
		return true;
	}
	return false;
}

static int FreeRTOS_create(struct target *target)
{
	int i = 0;
	while ((i < FREERTOS_NUM_PARAMS) &&
			(0 != strcmp(FreeRTOS_params_list[i].target_name, target->type->name))) {
		i++;
	}
	if (i >= FREERTOS_NUM_PARAMS) {
		LOG_ERROR("Could not find target in FreeRTOS compatibility list");
		return JIM_ERR;
	}
	LOG_INFO("FreeRTOS creation");
	struct FreeRTOS_data *rtos_data = calloc(1, sizeof(struct FreeRTOS_data));
	if (rtos_data == NULL) {
		LOG_ERROR("Failed to allocate OS data!");
		return JIM_ERR;
	}
	target->rtos->current_threadid = -1;
	rtos_data->nr_cpus = 1;
	rtos_data->params = &FreeRTOS_params_list[i];
	target->rtos->rtos_specific_params = rtos_data;
	target->rtos->current_thread = 0;
	target->rtos->thread_details = NULL;
	target->rtos->gdb_target_for_threadid = FreeRTOS_target_for_threadid;
	return JIM_OK;
}
