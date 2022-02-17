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

/*Espressif ToDo: Merge this with the upstream code. As is, it probably breaks the arm stuff. */

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

/* FIXME: none of the _width parameters are actually observed properly!
 * you WILL need to edit more if you actually attempt to target a 8/16/64
 * bit target!
 */

struct freertos_params {
	const char *target_name;
	const unsigned char thread_count_width;
	const unsigned char pointer_width;
	const unsigned char list_next_offset;	/* offset of `xListEnd.pxPrevious` in List_t */
	const unsigned char list_end_offset;	/* offset of `xListEnd` in List_t */
	const unsigned char list_width;
	const unsigned char list_elem_next_offset;	/* offset of `pxPrevious` in ListItem_t */
	const unsigned char list_elem_content_offset;
	const unsigned char thread_stack_offset;
	const unsigned char thread_name_offset;
	const unsigned char thread_counter_width;
	const struct rtos_register_stacking *stacking_info_cm3;
	const struct rtos_register_stacking *stacking_info_cm4f;
	const struct rtos_register_stacking *stacking_info_cm4f_fpu;
	const struct rtos_register_stacking * (*stacking_info_pick_fn)(struct rtos *rtos,
		int64_t thread_id, int64_t stack_addr);
};

static const struct freertos_params freertos_params_list[] = {
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
	&rtos_standard_cortex_m3_stacking,	/* stacking_info */
	&rtos_standard_cortex_m4f_stacking,
	&rtos_standard_cortex_m4f_fpu_stacking,
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
	&rtos_standard_cortex_m3_stacking,	/* stacking_info */
	&rtos_standard_cortex_m4f_stacking,
	&rtos_standard_cortex_m4f_fpu_stacking,
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
	&rtos_standard_nds32_n1068_stacking,	/* stacking_info */
	&rtos_standard_cortex_m4f_stacking,
	&rtos_standard_cortex_m4f_fpu_stacking,
	},
};

static bool freertos_detect_rtos(struct target *target);
static int freertos_create(struct target *target);
static int freertos_update_threads(struct rtos *rtos);
static int freertos_get_thread_reg_list(struct rtos *rtos, int64_t thread_id,
		struct rtos_reg **reg_list, int *num_regs);
static int freertos_get_symbol_list_to_lookup(struct symbol_table_elem *symbol_list[]);

struct rtos_type freertos_rtos = {
	.name = "FreeRTOS",

	.detect_rtos = freertos_detect_rtos,
	.create = freertos_create,
	.update_threads = freertos_update_threads,
	.get_thread_reg_list = freertos_get_thread_reg_list,
	.get_symbol_list_to_lookup = freertos_get_symbol_list_to_lookup,
};

enum freertos_symbol_values {
	FREERTOS_VAL_PX_CURRENT_TCB = 0,
	FREERTOS_VAL_PX_READY_TASKS_LISTS = 1,
	FREERTOS_VAL_X_DELAYED_TASK_LIST1 = 2,
	FREERTOS_VAL_X_DELAYED_TASK_LIST2 = 3,
	FREERTOS_VAL_PX_DELAYED_TASK_LIST = 4,
	FREERTOS_VAL_PX_OVERFLOW_DELAYED_TASK_LIST = 5,
	FREERTOS_VAL_X_PENDING_READY_LIST = 6,
	FREERTOS_VAL_X_TASKS_WAITING_TERMINATION = 7,
	FREERTOS_VAL_X_SUSPENDED_TASK_LIST = 8,
	FREERTOS_VAL_UX_CURRENT_NUMBER_OF_TASKS = 9,
	FREERTOS_VAL_UX_TOP_USED_PRIORITY = 10,
};

struct symbols {
	const char *name;
	bool optional;
};

static const struct symbols freertos_symbol_list[] = {
	{ "pxCurrentTCB", false },
	{ "pxReadyTasksLists", false },
	{ "xDelayedTaskList1", false },
	{ "xDelayedTaskList2", false },
	{ "pxDelayedTaskList", false },
	{ "pxOverflowDelayedTaskList", false },
	{ "xPendingReadyList", false },
	{ "xTasksWaitingTermination", true },	/* Only if INCLUDE_vTaskDelete */
	{ "xSuspendedTaskList", true },	/* Only if INCLUDE_vTaskSuspend */
	{ "uxCurrentNumberOfTasks", false },
	{ "uxTopUsedPriority", true },	/* Unavailable since v7.5.3 */
	{ "port_interruptNesting", true },
	{ "uxTaskNumber", false },
	{ NULL, false }
};

static const char *const STATE_RUNNING_STR = "State: Running @CPU%d";

static int FreeRTOS_smp_init(struct target *target)
{
	/* keep only target->rtos */
	struct rtos *rtos = target->rtos;
	struct FreeRTOS_data *rtos_data = (struct FreeRTOS_data *)rtos->rtos_specific_params;
	struct target_list *head = target->head;

	/* use one of rtos instance for both target */
	while (head != (struct target_list *)NULL) {
		if (head->target->rtos != rtos) {
			struct FreeRTOS_data *smp_rtos_data =
				(struct FreeRTOS_data *)head->target->rtos->rtos_specific_params;
			/*  remap smp target on rtos  */
			free(head->target->rtos);
			head->target->rtos = rtos;
			free(smp_rtos_data);
			rtos_data->nr_cpus++;
		}
		head = head->next;
	}

	if (rtos_data->curr_threads_handles_buff)
		free(rtos_data->curr_threads_handles_buff);

	rtos_data->curr_threads_handles_buff = calloc(rtos_data->nr_cpus,
		rtos_data->params->pointer_width);

	if (rtos_data->curr_threads_handles_buff == NULL) {
		LOG_ERROR("Failed to allocate current threads handles buffer!");
		return JIM_ERR;
	}

	return ERROR_OK;
}

static int target_buffer_get_uint(struct target *target, uint8_t width, uint8_t *buff,
	uint64_t *val)
{
	switch (width) {
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
	int retval = ERROR_FAIL;

	switch (width) {
		case 8:
			retval = target_read_u64(target, address, val);
			break;
		case 4:
		{
			uint32_t v32 = 0;
			retval = target_read_u32(target, address, &v32);
			*val = v32;
			break;
		}
		case 2:
		{
			uint16_t v16 = 0;
			retval = target_read_u16(target, address, &v16);
			*val = v16;
			break;
		}
		case 1:
		{
			uint8_t v8 = 0;
			retval = target_read_u8(target, address, &v8);
			*val = v8;
			break;
		}
		default:
			LOG_ERROR("Unsupported target integer of width %u!", width);
			break;
	}

	return retval;
}

static int target_buffer_write_uint(struct target *target, target_addr_t address,
	uint8_t width, uint64_t val)
{
	int retval = ERROR_FAIL;

	switch (width) {
		case 8:
			retval = target_write_u64(target, address, val);
			break;
		case 4:
			retval = target_write_u32(target, address, (uint32_t)val);
			break;
		case 2:
			retval = target_write_u16(target, address, (uint16_t)val);
			break;
		case 1:
			retval = target_write_u8(target, address, (uint8_t)val);
			break;
		default:
			LOG_ERROR("Unsupported target integer of width %u!", width);
			break;
	}

	return retval;
}

static inline threadid_t FreeRTOS_current_threadid_from_target(struct target *target)
{
	struct FreeRTOS_data *rtos_data =
		(struct FreeRTOS_data *)target->rtos->rtos_specific_params;
	uint64_t tid = 0;

	int retval = target_buffer_get_uint(target, rtos_data->params->pointer_width,
		&rtos_data->curr_threads_handles_buff[target->coreid *
			rtos_data->params->pointer_width], &tid);
	if (retval != ERROR_OK)
		return 0;
	LOG_DEBUG("Curr thread 0x%x on target [%s]", (uint32_t)tid, target_name(target));
	return tid;
}

static int FreeRTOS_find_target_from_threadid(struct target *target,
	int64_t thread_id,
	struct target **p_target)
{
	struct target_list *head;

	LOG_DEBUG("Find target for thr 0x%x", (uint32_t)thread_id);

	if (target->smp) {
		/* Find the thread with that thread_id */
		foreach_smp_target(head, target->head) {
			struct target *current_target = head->target;
			if (thread_id == FreeRTOS_current_threadid_from_target(current_target)) {
				*p_target = current_target;
				LOG_DEBUG("target found : %s", current_target->cmd_name);
				return ERROR_OK;
			}
		}
		return ERROR_FAIL;
	}

static int freertos_update_threads(struct rtos *rtos)
{
	int retval;
	unsigned int tasks_found = 0;
	const struct freertos_params *param;

	if (!rtos->rtos_specific_params)
		return -1;

	param = (const struct freertos_params *) rtos->rtos_specific_params;

	if (!rtos->symbols) {
		LOG_ERROR("No symbols for FreeRTOS");
		return -3;
	}

	if (rtos->symbols[FREERTOS_VAL_UX_CURRENT_NUMBER_OF_TASKS].address == 0) {
		LOG_ERROR("Don't have the number of threads in FreeRTOS");
		return -2;
	}

	uint32_t thread_list_size = 0;
	retval = target_read_u32(rtos->target,
			rtos->symbols[FREERTOS_VAL_UX_CURRENT_NUMBER_OF_TASKS].address,
			&thread_list_size);
	LOG_DEBUG("FreeRTOS: Read uxCurrentNumberOfTasks at 0x%" PRIx64 ", value %" PRIu32,
										rtos->symbols[FREERTOS_VAL_UX_CURRENT_NUMBER_OF_TASKS].address,
										thread_list_size);

	if (retval != ERROR_OK) {
		LOG_ERROR("Could not read FreeRTOS thread count from target!");
		return retval;
	}

	LOG_DEBUG("Read uxCurrentNumberOfTasks at 0x%" PRIx64 ", value %" PRIu64,
		rtos->symbols[FreeRTOS_VAL_uxCurrentNumberOfTasks].address,
		thread_list_size);

	if (thread_list_size > FREERTOS_MAX_TASKS_NUM) {
		LOG_ERROR("Too large number of threads %" PRIu64 "!", thread_list_size);
		return ERROR_FAIL;
	}

	/* Read uxTaskNumber to detect the task lists changes */
	int64_t uxTaskNumber = 0;
	retval = target_buffer_read_uint(target,
		rtos->symbols[FreeRTOS_VAL_uxTaskNumber].address,
		rtos_data->params->thread_counter_width,
		(uint64_t *)&uxTaskNumber);

	/* read the current thread */
	uint32_t pointer_casts_are_bad;
	retval = target_read_u32(rtos->target,
			rtos->symbols[FREERTOS_VAL_PX_CURRENT_TCB].address,
			&pointer_casts_are_bad);
	if (retval != ERROR_OK) {
		LOG_ERROR("Could not read FreeRTOS uxTaskNumber from target!");
		return retval;
	}
	rtos->current_thread = pointer_casts_are_bad;
	LOG_DEBUG("FreeRTOS: Read pxCurrentTCB at 0x%" PRIx64 ", value 0x%" PRIx64,
										rtos->symbols[FREERTOS_VAL_PX_CURRENT_TCB].address,
										rtos->current_thread);

	LOG_DEBUG("Read uxTaskNumber at 0x%" PRIx64 ", value %" PRIu64,
		rtos->symbols[FreeRTOS_VAL_uxTaskNumber].address,
		uxTaskNumber);

	if (uxTaskNumber < rtos_data->thread_counter) {
		LOG_ERROR("FreeRTOS uxTaskNumber seems to be corrupted!");
		return ERROR_FAIL;
	}

	/* read the current threads */
	retval = target_read_buffer(target,
		rtos->symbols[FreeRTOS_VAL_pxCurrentTCB].address,
		rtos_data->params->pointer_width * rtos_data->nr_cpus,
		rtos_data->curr_threads_handles_buff);

	if (retval != ERROR_OK) {
		LOG_ERROR("Could not read FreeRTOS current threads!");
		return retval;
	}

	rtos->current_thread = FreeRTOS_smp_get_current_thread(rtos);

	LOG_DEBUG("FreeRTOS: Read pxCurrentTCB at 0x%" PRIx64 ", value 0x%" PRIx64,
		rtos->symbols[FreeRTOS_VAL_pxCurrentTCB].address,
		rtos->current_thread);

	if (rtos->thread_count != 0 && uxTaskNumber == rtos_data->thread_counter) {
		/* There is no new added or deleted RTOS task. Just update the state of the current
		 * task */
		return FreeRTOS_update_extra_details(target);
	}

	threadid_t temp = rtos->current_thread;
	rtos_free_threadlist(rtos);
	rtos->current_thread = temp;

	if ((thread_list_size == 0) || (rtos->current_thread == 0)) {
		/* Either : No RTOS threads - there is always at least the current execution though
		 * */
		/* OR     : No current thread - all threads suspended - show the current execution
		 * of idling */
		char tmp_str[] = "Current Execution";
		thread_list_size++;
		tasks_found++;
		rtos->thread_details = calloc(thread_list_size, sizeof(struct thread_detail));
		if (!rtos->thread_details) {
			LOG_ERROR("Error allocating memory for %" PRIu64 " threads",
				thread_list_size);
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
			LOG_ERROR("Error allocating memory for %" PRIu64 " threads",
				thread_list_size);
			return ERROR_FAIL;
		}
	}

	/* Find out how many lists are needed to be read from pxReadyTasksLists, */
	if (rtos->symbols[FREERTOS_VAL_UX_TOP_USED_PRIORITY].address == 0) {
		LOG_ERROR("FreeRTOS: uxTopUsedPriority is not defined, consult the OpenOCD manual for a work-around");
		return ERROR_FAIL;
	}
	uint32_t top_used_priority = 0;
	retval = target_read_u32(rtos->target,
			rtos->symbols[FREERTOS_VAL_UX_TOP_USED_PRIORITY].address,
			&top_used_priority);
	if (retval != ERROR_OK)
		return retval;
	LOG_DEBUG("FreeRTOS: Read uxTopUsedPriority at 0x%" PRIx64 ", value %" PRIu32,
										rtos->symbols[FREERTOS_VAL_UX_TOP_USED_PRIORITY].address,
										top_used_priority);
	if (top_used_priority > FREERTOS_MAX_PRIORITIES) {
		LOG_ERROR("FreeRTOS top used priority is unreasonably big, not proceeding: %" PRIu32,
			top_used_priority);
		return ERROR_FAIL;
	}

	/* uxTopUsedPriority was defined as configMAX_PRIORITIES - 1
	 * in old FreeRTOS versions (before V7.5.3)
	 * Use contrib/rtos-helpers/FreeRTOS-openocd.c to get compatible symbol
	 * in newer FreeRTOS versions.
	 * Here we restore the original configMAX_PRIORITIES value */
	unsigned int config_max_priorities = top_used_priority + 1;

	symbol_address_t *list_of_lists =
		malloc(sizeof(symbol_address_t) * (config_max_priorities + 5));
	if (!list_of_lists) {
		LOG_ERROR("Error allocating memory for %u priorities", config_max_priorities);
		return ERROR_FAIL;
	}

	unsigned int num_lists;
	for (num_lists = 0; num_lists < config_max_priorities; num_lists++)
		list_of_lists[num_lists] = rtos->symbols[FREERTOS_VAL_PX_READY_TASKS_LISTS].address +
			num_lists * param->list_width;

	list_of_lists[num_lists++] = rtos->symbols[FREERTOS_VAL_X_DELAYED_TASK_LIST1].address;
	list_of_lists[num_lists++] = rtos->symbols[FREERTOS_VAL_X_DELAYED_TASK_LIST2].address;
	list_of_lists[num_lists++] = rtos->symbols[FREERTOS_VAL_X_PENDING_READY_LIST].address;
	list_of_lists[num_lists++] = rtos->symbols[FREERTOS_VAL_X_SUSPENDED_TASK_LIST].address;
	list_of_lists[num_lists++] = rtos->symbols[FREERTOS_VAL_X_TASKS_WAITING_TERMINATION].address;

	for (unsigned int i = 0; i < num_lists; i++) {
		if (list_of_lists[i] == 0)
			continue;

		/* Read the number of threads in this list */
		uint32_t list_thread_count = 0;
		retval = target_read_u32(rtos->target,
				list_of_lists[i],
				&list_thread_count);
		if (retval != ERROR_OK) {
			LOG_ERROR("Error reading number of threads in FreeRTOS thread list");
			free(list_of_lists);
			return retval;
		}
		LOG_DEBUG("FreeRTOS: Read thread count for list %u at 0x%" PRIx64 ", value %" PRIu32,
										i, list_of_lists[i], list_thread_count);

		if (list_thread_count == 0)
			continue;

		/* Read the location of first list item */
		uint32_t prev_list_elem_ptr = -1;
		uint32_t list_elem_ptr = 0;
		retval = target_read_u32(rtos->target,
				list_of_lists[i] + param->list_next_offset,
				&list_elem_ptr);
		if (retval != ERROR_OK) {
			LOG_ERROR("Error reading first thread item location in FreeRTOS thread list");
			free(list_of_lists);
			return retval;
		}
		LOG_DEBUG("FreeRTOS: Read first item for list %u at 0x%" PRIx64 ", value 0x%" PRIx32,
										i, list_of_lists[i] + param->list_next_offset, list_elem_ptr);

		while ((list_thread_count > 0) && (list_elem_ptr != 0) &&
				(list_elem_ptr != prev_list_elem_ptr) &&
				(tasks_found < thread_list_size)) {
			/* Get the location of the thread structure. */
			rtos->thread_details[tasks_found].threadid = 0;
			retval = target_read_u32(rtos->target,
					list_elem_ptr + param->list_elem_content_offset,
					&pointer_casts_are_bad);
			if (retval != ERROR_OK) {
				LOG_ERROR("Error reading thread list item object in FreeRTOS thread list");
				free(list_of_lists);
				return retval;
			}
			rtos->thread_details[tasks_found].threadid = pointer_casts_are_bad;
			LOG_DEBUG("FreeRTOS: Read Thread ID at 0x%" PRIx32 ", value 0x%" PRIx64,
										list_elem_ptr + param->list_elem_content_offset,
										rtos->thread_details[tasks_found].threadid);

static int FreeRTOS_get_current_thread_registers(struct rtos *rtos, int64_t thread_id,
	enum target_register_class reg_class, bool *is_curr_thread,
	struct rtos_reg **reg_list, int *num_regs)
{
	int retval;
	struct target *current_target = NULL;

	LOG_DEBUG("FreeRTOS_get_current_thread_registers thread_id=0x%x", (uint32_t)thread_id);

	*is_curr_thread = false;
	if (rtos->target->smp)
		FreeRTOS_find_target_from_threadid(rtos->target, thread_id, &current_target);
	else if (thread_id == rtos->current_thread)
		current_target = rtos->target;
	if (current_target == NULL)
		return ERROR_OK;
	*is_curr_thread = true;
	if (!target_was_examined(current_target))
		return ERROR_FAIL;

	/* registers for threads currently running on CPUs are not on task's stack and
	 * should retrieved from reg caches via target_get_gdb_reg_list */
	struct reg **gdb_reg_list;
	retval = target_get_gdb_reg_list(current_target, &gdb_reg_list, num_regs,
		reg_class);
	if (retval != ERROR_OK) {
		LOG_ERROR("target_get_gdb_reg_list failed %d", retval);
		return retval;
	}

			/* Read the thread name */
			retval = target_read_buffer(rtos->target,
					rtos->thread_details[tasks_found].threadid + param->thread_name_offset,
					FREERTOS_THREAD_NAME_STR_SIZE,
					(uint8_t *)&tmp_str);
			if (retval != ERROR_OK) {
				LOG_ERROR("Error reading first thread item location in FreeRTOS thread list");
				free(list_of_lists);
				return retval;
			}
			tmp_str[FREERTOS_THREAD_NAME_STR_SIZE-1] = '\x00';
			LOG_DEBUG("FreeRTOS: Read Thread Name at 0x%" PRIx64 ", value '%s'",
										rtos->thread_details[tasks_found].threadid + param->thread_name_offset,
										tmp_str);

			if (tmp_str[0] == '\x00')
				strcpy(tmp_str, "No Name");

			rtos->thread_details[tasks_found].thread_name_str =
				malloc(strlen(tmp_str)+1);
			strcpy(rtos->thread_details[tasks_found].thread_name_str, tmp_str);
			rtos->thread_details[tasks_found].exists = true;

			if (rtos->thread_details[tasks_found].threadid == rtos->current_thread) {
				char running_str[] = "State: Running";
				rtos->thread_details[tasks_found].extra_info_str = malloc(
						sizeof(running_str));
				strcpy(rtos->thread_details[tasks_found].extra_info_str,
					running_str);
			} else
				rtos->thread_details[tasks_found].extra_info_str = NULL;

			tasks_found++;
			list_thread_count--;

			prev_list_elem_ptr = list_elem_ptr;
			list_elem_ptr = 0;
			retval = target_read_u32(rtos->target,
					prev_list_elem_ptr + param->list_elem_next_offset,
					&list_elem_ptr);
			if (retval != ERROR_OK) {
				LOG_ERROR("Error reading next thread item location in FreeRTOS thread list");
				free(list_of_lists);
				return retval;
			}
			LOG_DEBUG("FreeRTOS: Read next thread location at 0x%" PRIx32 ", value 0x%" PRIx32,
										prev_list_elem_ptr + param->list_elem_next_offset,
										list_elem_ptr);
		}
	}

	for (int i = 0; i < *num_regs; i++) {
		(*reg_list)[i].number = gdb_reg_list[i]->number;
		(*reg_list)[i].size = gdb_reg_list[i]->size;
		memcpy((*reg_list)[i].value, gdb_reg_list[i]->value,
			((*reg_list)[i].size + 7) / 8);
	}
	free(gdb_reg_list);
	return ERROR_OK;
}

static int freertos_get_thread_reg_list(struct rtos *rtos, int64_t thread_id,
		struct rtos_reg **reg_list, int *num_regs)
{
	int retval;
	const struct freertos_params *param;
	int64_t stack_ptr = 0;
	struct FreeRTOS_data *rtos_data = (struct FreeRTOS_data *)rtos->rtos_specific_params;

	if (!rtos)
		return -1;

	if (thread_id == 0)
		return -2;

	if (!rtos->rtos_specific_params)
		return -1;

	param = (const struct freertos_params *) rtos->rtos_specific_params;

	/* Read the stack pointer */
	uint32_t pointer_casts_are_bad;
	retval = target_read_u32(rtos->target,
			thread_id + param->thread_stack_offset,
			&pointer_casts_are_bad);
	if (retval != ERROR_OK) {
		LOG_ERROR("Error reading stack frame from FreeRTOS thread");
		return retval;
	}
	stack_ptr = pointer_casts_are_bad;
	LOG_DEBUG("FreeRTOS: Read stack pointer at 0x%" PRIx64 ", value 0x%" PRIx64,
										thread_id + param->thread_stack_offset,
										stack_ptr);

	/* Check for armv7m with *enabled* FPU, i.e. a Cortex-M4F */
	int cm4_fpu_enabled = 0;
	struct armv7m_common *armv7m_target = target_to_armv7m(rtos->target);
	if (is_armv7m(armv7m_target)) {
		if (armv7m_target->fp_feature == FPV4_SP) {
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
		uint32_t lr_svc = 0;
		retval = target_read_u32(rtos->target,
				stack_ptr + 0x20,
				&lr_svc);
		if (retval != ERROR_OK) {
			LOG_OUTPUT("Error reading stack frame from FreeRTOS thread");
			return retval;
		}
		if ((lr_svc & 0x10) == 0)
			return rtos_generic_stack_read(rtos->target, param->stacking_info_cm4f_fpu, stack_ptr, reg_list, num_regs);
		else
			return rtos_generic_stack_read(rtos->target,
				rtos_data->params->stacking_info_cm4f,
				stack_ptr,
				reg_list,
				num_regs);
	} else
		return rtos_generic_stack_read(rtos->target,
			rtos_data->params->stacking_info_cm3,
			stack_ptr,
			reg_list,
			num_regs);

	return -1;
}

static int freertos_get_symbol_list_to_lookup(struct symbol_table_elem *symbol_list[])
{
	unsigned int i;
	*symbol_list = calloc(
			ARRAY_SIZE(freertos_symbol_list), sizeof(struct symbol_table_elem));

	for (i = 0; i < ARRAY_SIZE(freertos_symbol_list); i++) {
		(*symbol_list)[i].symbol_name = freertos_symbol_list[i].name;
		(*symbol_list)[i].optional = freertos_symbol_list[i].optional;
	}

	if (thread_id == 0)
		return -2;

	LOG_DEBUG("FreeRTOS_get_thread_reg_list thread_id=0x%x", (uint32_t)thread_id);

static int freertos_set_current_thread(struct rtos *rtos, threadid_t thread_id)
{
	return 0;
}

static int freertos_get_thread_ascii_info(struct rtos *rtos, threadid_t thread_id, char **info)
{
	int retval;
	const struct freertos_params *param;

	if (!rtos)
		return -1;

	if (thread_id == 0)
		return -2;

	if (!rtos->rtos_specific_params)
		return -3;

	param = (const struct freertos_params *) rtos->rtos_specific_params;

#define FREERTOS_THREAD_NAME_STR_SIZE (200)
	char tmp_str[FREERTOS_THREAD_NAME_STR_SIZE];

	retval = FreeRTOS_get_current_thread_registers(rtos,
		thread_id,
		REG_CLASS_ALL,
		&is_curr_thread,
		&reg_list,
		&num_regs);
	if (retval != ERROR_OK)
		return retval;

	if (!is_curr_thread) {
		/* All registers (general + privileged ones) can be accessed for the threads currently running on cores.
		        It is enough for now. For non-current threads this function can return general registers only. */
		retval = FreeRTOS_get_thread_reg_list(rtos, thread_id, &reg_list, &num_regs);
		if (retval != ERROR_OK)
			return retval;
	}
	for (int i = 0; i < num_regs; ++i) {
		if (reg_list[i].number == (uint32_t)reg_num) {
			memcpy(reg, &reg_list[i], sizeof(*reg));
			free(reg_list);
			return ERROR_OK;
		}
	}
	free(reg_list);

	LOG_WARNING("Can not get register %d for thread 0x%" PRIx64,
		reg_num,
		thread_id);
	return ERROR_FAIL;
}

static int FreeRTOS_get_symbol_list_to_lookup(symbol_table_elem_t *symbol_list[])
{
	unsigned int i;

	*symbol_list = calloc(ARRAY_SIZE(FreeRTOS_symbol_list), sizeof(symbol_table_elem_t));

	for (i = 0; i < ARRAY_SIZE(FreeRTOS_symbol_list); i++) {
		(*symbol_list)[i].symbol_name = FreeRTOS_symbol_list[i].name;
		(*symbol_list)[i].optional = FreeRTOS_symbol_list[i].optional;
	}

	return 0;
}

static int FreeRTOS_post_reset_cleanup(struct target *target)
{
	LOG_DEBUG("FreeRTOS_post_reset_cleanup");

	struct FreeRTOS_data *rtos_data =
		(struct FreeRTOS_data *)target->rtos->rtos_specific_params;

	if ((target->rtos->symbols != NULL) &&
		(target->rtos->symbols[FreeRTOS_VAL_uxCurrentNumberOfTasks].address != 0)) {
		int ret = target_buffer_write_uint(target,
			target->rtos->symbols[FreeRTOS_VAL_uxCurrentNumberOfTasks].address,
			rtos_data->params->thread_count_width,
			0);
		if (ret != ERROR_OK) {
			LOG_ERROR("Failed to clear uxCurrentNumberOfTasks!");
			return ret;
		}
		if (target->smp) {
			/* clear pxCurrentTCB for all cores */
			struct target_list *head;
			foreach_smp_target(head, target->head) {
				struct target *current_target = head->target;
				if (!target_was_examined(current_target))
					continue;
				ret = target_buffer_write_uint(
					target,
					target->rtos->symbols[FreeRTOS_VAL_pxCurrentTCB].address +
					current_target->coreid*rtos_data->params->pointer_width,
					rtos_data->params->pointer_width,
					0);
				if (ret != ERROR_OK) {
					LOG_ERROR("Failed to clear pxCurrentTCB for core %d!",
						current_target->coreid);
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
	rtos_data->thread_counter = 0;
	return ERROR_OK;
}

static int FreeRTOS_clean(struct target *target)
{
	struct FreeRTOS_data *rtos_data =
		(struct FreeRTOS_data *)target->rtos->rtos_specific_params;

	LOG_DEBUG("FreeRTOS_clean");
	/* if rtos_auto_detect is true FreeRTOS_create() will be called upon receiption of the first
	 * 'qSymbol', */
	/* so we can free resources
	 * if rtos_auto_detect is false FreeRTOS_create() is called only once upon target creation
	 * FreeRTOS_clean() is called every time GDB initiates new connection
	 * TODO: fix this in GDB server code */
	if (!target->rtos_auto_detect)
		return ERROR_OK;

	free(rtos_data->curr_threads_handles_buff);
	free(rtos_data);
	target->rtos->rtos_specific_params = NULL;
	return ERROR_OK;
}

static bool freertos_detect_rtos(struct target *target)
{
	if ((target->rtos->symbols) &&
			(target->rtos->symbols[FREERTOS_VAL_PX_READY_TASKS_LISTS].address != 0)) {
		/* looks like FreeRTOS */
		return true;
	}
	return false;
}

static int freertos_create(struct target *target)
{
	for (unsigned int i = 0; i < ARRAY_SIZE(freertos_params_list); i++)
		if (strcmp(freertos_params_list[i].target_name, target->type->name) == 0) {
			target->rtos->rtos_specific_params = (void *)&freertos_params_list[i];
			return 0;
		}

	LOG_ERROR("Could not find target in FreeRTOS compatibility list");
	return -1;
}
