/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Copyright (C) 2011 by Broadcom Corporation                            *
 *   Evan Hunter - ehunter@broadcom.com                                    *
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

#define FREERTOS_MAX_PRIORITIES 63
#define FREERTOS_MAX_TASKS_NUM  512

#define FreeRTOS_STRUCT(int_type, ptr_type, list_prev_offset)

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
		"cortex_m",				/* target_name */
		4,						/* thread_count_width; */
		4,						/* pointer_width; */
		16,						/* list_next_offset; */
		8,						/* list_end_offset; */
		20,						/* list_width; */
		8,						/* list_elem_next_offset; */
		12,						/* list_elem_content_offset */
		0,						/* thread_stack_offset; */
		52,						/* thread_name_offset; */
		4,						/* thread_counter_width */
		&rtos_standard_cortex_m3_stacking,	/* stacking_info */
		&rtos_standard_cortex_m4f_stacking,
		&rtos_standard_cortex_m4f_fpu_stacking,
		NULL,					/* fn to pick stacking_info */
	},
	{
		"hla_target",			/* target_name */
		4,						/* thread_count_width; */
		4,						/* pointer_width; */
		16,						/* list_next_offset; */
		8,						/* list_end_offset; */
		20,						/* list_width; */
		8,						/* list_elem_next_offset; */
		12,						/* list_elem_content_offset */
		0,						/* thread_stack_offset; */
		52,						/* thread_name_offset; */
		4,						/* thread_counter_width */
		&rtos_standard_cortex_m3_stacking,	/* stacking_info */
		&rtos_standard_cortex_m4f_stacking,
		&rtos_standard_cortex_m4f_fpu_stacking,
		NULL,					/* fn to pick stacking_info */
	},
	{
		"esp32",				/* target_name */
		4,						/* thread_count_width; */
		4,						/* pointer_width; */
		16,						/* list_next_offset; */
		8,						/* list_end_offset; */
		20,						/* list_width; */
		8,						/* list_elem_next_offset; */
		12,						/* list_elem_content_offset */
		0,						/* thread_stack_offset; */
		56,						/* thread_name_offset; */
		4,						/* thread_counter_width */
		NULL,					/* stacking_info */
		NULL,
		NULL,
		rtos_freertos_esp32_pick_stacking_info,	/* fn to pick stacking_info */
	},
	{
		"esp32s2",				/* target_name */
		4,						/* thread_count_width; */
		4,						/* pointer_width; */
		16,						/* list_next_offset; */
		8,						/* list_end_offset; */
		20,						/* list_width; */
		8,						/* list_elem_next_offset; */
		12,						/* list_elem_content_offset */
		0,						/* thread_stack_offset; */
		52,						/* thread_name_offset; */
		4,						/* thread_counter_width */
		NULL,					/* stacking_info */
		NULL,
		NULL,
		rtos_freertos_esp32_s2_pick_stacking_info,	/* fn to pick stacking_info */
	},
	{
		"esp32s3",				/* target_name */
		4,						/* thread_count_width; */
		4,						/* pointer_width; */
		16,						/* list_next_offset; */
		8,						/* list_end_offset; */
		20,						/* list_width; */
		8,						/* list_elem_next_offset; */
		12,						/* list_elem_content_offset */
		0,						/* thread_stack_offset; */
		56,						/* thread_name_offset; */
		4,						/* thread_counter_width */
		NULL,					/* stacking_info */
		NULL,
		NULL,
		rtos_freertos_esp32_s3_pick_stacking_info,	/* fn to pick stacking_info */
	},
	{
		"esp32c2",				/* target_name */
		4,						/* thread_count_width; */
		4,						/* pointer_width; */
		16,						/* list_next_offset; */
		8,						/* list_end_offset; */
		20,						/* list_width; */
		8,						/* list_elem_next_offset; */
		12,						/* list_elem_content_offset */
		0,						/* thread_stack_offset; */
		52,						/* thread_name_offset; */
		4,						/* thread_counter_width */
		NULL,					/* stacking_info */
		NULL,
		NULL,
		rtos_freertos_riscv_pick_stacking_info,	/* fn to pick stacking_info */
	},
	{
		"esp32h2",				/* target_name */
		4,						/* thread_count_width; */
		4,						/* pointer_width; */
		16,						/* list_next_offset; */
		8,						/* list_end_offset; */
		20,						/* list_width; */
		8,						/* list_elem_next_offset; */
		12,						/* list_elem_content_offset */
		0,						/* thread_stack_offset; */
		52,						/* thread_name_offset; */
		4,						/* thread_counter_width */
		NULL,					/* stacking_info */
		NULL,
		NULL,
		rtos_freertos_riscv_pick_stacking_info,	/* fn to pick stacking_info */
	},
	{
		"esp32c3",				/* target_name */
		4,						/* thread_count_width; */
		4,						/* pointer_width; */
		16,						/* list_next_offset; */
		8,						/* list_end_offset; */
		20,						/* list_width; */
		8,						/* list_elem_next_offset; */
		12,						/* list_elem_content_offset */
		0,						/* thread_stack_offset; */
		52,						/* thread_name_offset; */
		4,						/* thread_counter_width */
		NULL,					/* stacking_info */
		NULL,
		NULL,
		rtos_freertos_riscv_pick_stacking_info,	/* fn to pick stacking_info */
	},
	{
		"esp32c6",				/* target_name */
		4,						/* thread_count_width; */
		4,						/* pointer_width; */
		16,						/* list_next_offset; */
		8,						/* list_end_offset; */
		20,						/* list_width; */
		8,						/* list_elem_next_offset; */
		12,						/* list_elem_content_offset */
		0,						/* thread_stack_offset; */
		52,						/* thread_name_offset; */
		4,						/* thread_counter_width */
		NULL,					/* stacking_info */
		NULL,
		NULL,
		rtos_freertos_riscv_pick_stacking_info,	/* fn to pick stacking_info */
	},
};

#define FREERTOS_NUM_PARAMS ARRAY_SIZE(freertos_params_list)

struct current_thread {
	int64_t threadid;
	int32_t core_id;
	struct current_thread *next;
};

struct freertos_data {
	const struct freertos_params *params;
	uint32_t nr_cpus;
	uint8_t *curr_threads_handles_buff;
	uint32_t thread_counter;/* equivalent to uxTaskNumber */
	uint8_t *esp_symbols;
};

static bool freertos_detect_rtos(struct target *target);
static int freertos_create(struct target *target);
static int freertos_update_threads(struct rtos *rtos);
static int freertos_get_thread_reg_list(struct rtos *rtos, int64_t thread_id,
	struct rtos_reg **reg_list, int *num_regs);
static int freertos_get_thread_reg(struct rtos *rtos, int64_t thread_id,
	uint32_t reg_num, struct rtos_reg *reg);
static int freertos_get_symbol_list_to_lookup(struct symbol_table_elem *symbol_list[]);
static int freertos_post_reset_cleanup(struct target *target);
static int freertos_clean(struct target *target);
static int freertos_smp_init(struct target *target);

const struct rtos_type freertos_rtos = {
	.name = "FreeRTOS",
	.detect_rtos = freertos_detect_rtos,
	.create = freertos_create,
	.smp_init = freertos_smp_init,
	.update_threads = freertos_update_threads,
	.get_thread_reg_list = freertos_get_thread_reg_list,	/* get general thread registers */
	/* We need this API to handle 'p' packets to retrieve non-general (privileged) registers properly in SMP mode.
	        The problem is that without this API rtos code will call `target_get_gdb_reg_list_noread()` on the target/core
	        currently exposed to GDB (gdb_service->target) which can be inconsistent with current thread (actually running on another core).
	        So privileged regiser value for wrong core will be returned. */
	.get_thread_reg = freertos_get_thread_reg,	/* get any thread register */
	.get_symbol_list_to_lookup = freertos_get_symbol_list_to_lookup,
	.clean = freertos_clean,
	.post_reset_cleanup = freertos_post_reset_cleanup,
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
	FREERTOS_VAL_X_SCHEDULER_RUNNING = 11,
	FREERTOS_VAL_PORT_INTERRUPT_NESTING = 12,
	FREERTOS_VAL_UX_TASK_NUMBER = 13,
	FREERTOS_VAL_ESP_OPENOCD_PARAMS = 14,
	FREERTOS_VAL_PX_CURRENT_TCBs = 15,
};

struct symbols {
	const char *name;
	bool optional;
};

static const struct symbols freertos_symbol_list[] = {
	{ "pxCurrentTCB", true },	/* Only before ESP-IDF v5.0 */
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
	{ "xSchedulerRunning", false },
	{ "port_interruptNesting", true },
	{ "uxTaskNumber", false },
	{ "FreeRTOS_openocd_params", true },	/* Only if ESP_PLATFORM defined */
	{ "pxCurrentTCBs", true },	/* Available since ESP-IDF v5.0 */
	{ NULL, false }
};

enum {
	ESP_FREERTOS_DEBUG_TABLE_SIZE = 0,
	ESP_FREERTOS_DEBUG_TABLE_VERSION,
	ESP_FREERTOS_DEBUG_KERNEL_VER_MAJOR,
	ESP_FREERTOS_DEBUG_KERNEL_VER_MINOR,
	ESP_FREERTOS_DEBUG_KERNEL_VER_BUILD,
	ESP_FREERTOS_DEBUG_UX_TOP_USED_PIORITY,
	ESP_FREERTOS_DEBUG_PX_TOP_OF_STACK,
	ESP_FREERTOS_DEBUG_PC_TASK_NAME,
	/* New entries must be inserted here */
	ESP_FREERTOS_DEBUG_TABLE_END,
};

static const char *const STATE_RUNNING_STR = "State: Running @CPU%d";

static int freertos_read_esp_symbol_table(struct rtos *rtos, int index, uint8_t *val)
{
	struct freertos_data *rtos_data = (struct freertos_data *)rtos->rtos_specific_params;
	static uint8_t table_size = 0;

	assert(val);

	if (!rtos_data->esp_symbols) {
		if (!rtos->symbols[FREERTOS_VAL_ESP_OPENOCD_PARAMS].address)
			return ERROR_FAIL;

		LOG_DEBUG(
			"Read FREERTOS_VAL_ESP_OPENOCD_PARAMS from address 0x%"
			PRIx64,
			rtos->symbols[FREERTOS_VAL_ESP_OPENOCD_PARAMS].address);

		int retval = target_read_buffer(
			rtos->target,
			rtos->symbols[FREERTOS_VAL_ESP_OPENOCD_PARAMS].address +
			ESP_FREERTOS_DEBUG_TABLE_SIZE,
			sizeof(table_size),
			&table_size);
		if (retval != ERROR_OK)
			return retval;

		if (table_size == 0) {
			LOG_WARNING("esp_symbols table size (%d) is not valid!", table_size);
			return ERROR_FAIL;
		}

		rtos_data->esp_symbols = (uint8_t *)calloc(table_size, sizeof(table_size));
		if (!rtos_data->esp_symbols) {
			LOG_ERROR("Error allocating memory for esp_symbols");
			return ERROR_FAIL;
		}
		retval = target_read_buffer(rtos->target,
			rtos->symbols[FREERTOS_VAL_ESP_OPENOCD_PARAMS].address,
			table_size,
			rtos_data->esp_symbols);
		if (retval != ERROR_OK ||
			rtos_data->esp_symbols[ESP_FREERTOS_DEBUG_TABLE_VERSION] != 1) {
			free(rtos_data->esp_symbols);
			rtos_data->esp_symbols = NULL;
			return retval;
		}

		LOG_INFO("Detected FreeRTOS version: (%d.%d.%d)",
			rtos_data->esp_symbols[ESP_FREERTOS_DEBUG_KERNEL_VER_MAJOR],
			rtos_data->esp_symbols[ESP_FREERTOS_DEBUG_KERNEL_VER_MINOR],
			rtos_data->esp_symbols[ESP_FREERTOS_DEBUG_KERNEL_VER_BUILD]);
	}
	if (index >= table_size)
		return ERROR_FAIL;
	*val = rtos_data->esp_symbols[index];
	LOG_DEBUG("requested inx (%d) val (%d)", index, *val);
	return ERROR_OK;
}

static uint8_t freertos_get_thread_name_offset(struct rtos *rtos)
{
	struct freertos_data *rtos_data = (struct freertos_data *)rtos->rtos_specific_params;

	uint8_t thread_name_offset = rtos_data->params->thread_name_offset;

	freertos_read_esp_symbol_table(rtos, ESP_FREERTOS_DEBUG_PC_TASK_NAME, &thread_name_offset);

	return thread_name_offset;
}

static uint8_t freertos_get_thread_stack_offset(struct rtos *rtos)
{
	struct freertos_data *rtos_data = (struct freertos_data *)rtos->rtos_specific_params;

	uint8_t thread_stack_offset = rtos_data->params->thread_stack_offset;

	freertos_read_esp_symbol_table(rtos,
		ESP_FREERTOS_DEBUG_PX_TOP_OF_STACK,
		&thread_stack_offset);

	return thread_stack_offset;
}

static uint8_t freertos_get_ux_top_used_priority(struct rtos *rtos)
{
	uint32_t ux_top_used_priority = 0;

	if (freertos_read_esp_symbol_table(rtos, ESP_FREERTOS_DEBUG_UX_TOP_USED_PIORITY,
			(uint8_t *)&ux_top_used_priority) == ERROR_OK)
		return ux_top_used_priority;

	if (rtos->symbols[FREERTOS_VAL_UX_TOP_USED_PRIORITY].address == 0)
		return 0;

	int retval = target_read_u32(rtos->target,
		rtos->symbols[FREERTOS_VAL_UX_TOP_USED_PRIORITY].address,
		&ux_top_used_priority);

	return retval == ERROR_OK ? ux_top_used_priority : 0;
}

static symbol_address_t freertos_current_tcb_address(struct rtos *rtos)
{
	if (rtos->symbols[FREERTOS_VAL_PX_CURRENT_TCB].address)
		return rtos->symbols[FREERTOS_VAL_PX_CURRENT_TCB].address;
	return rtos->symbols[FREERTOS_VAL_PX_CURRENT_TCBs].address;
}

static int freertos_smp_init(struct target *target)
{
	/* keep only target->rtos */
	struct rtos *rtos = target->rtos;
	struct freertos_data *rtos_data = (struct freertos_data *)rtos->rtos_specific_params;
	struct target_list *head;

	/* use one of rtos instance for both target */
	foreach_smp_target(head, target->smp_targets) {
		if (head->target->rtos != rtos) {
			struct freertos_data *smp_rtos_data =
				(struct freertos_data *)head->target->rtos->rtos_specific_params;
			/*  remap smp target on rtos  */
			free(head->target->rtos);
			head->target->rtos = rtos;
			free(smp_rtos_data);
			rtos_data->nr_cpus++;
		}
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

static inline threadid_t freertos_current_threadid_from_target(struct target *target)
{
	struct freertos_data *rtos_data =
		(struct freertos_data *)target->rtos->rtos_specific_params;
	uint64_t tid = 0;

	int retval = target_buffer_get_uint(target, rtos_data->params->pointer_width,
		&rtos_data->curr_threads_handles_buff[target->coreid *
			rtos_data->params->pointer_width], &tid);
	if (retval != ERROR_OK)
		return 0;
	LOG_DEBUG("Curr thread 0x%x on target [%s]", (uint32_t)tid, target_name(target));
	return tid;
}

static int freertos_find_target_from_threadid(struct target *target,
	int64_t thread_id,
	struct target **p_target)
{
	struct target_list *head;

	LOG_DEBUG("Find target for thr 0x%x", (uint32_t)thread_id);

	if (target->smp) {
		/* Find the thread with that thread_id */
		foreach_smp_target(head, target->smp_targets) {
			struct target *current_target = head->target;
			if (thread_id == freertos_current_threadid_from_target(current_target)) {
				*p_target = current_target;
				LOG_DEBUG("target found : %s", current_target->cmd_name);
				return ERROR_OK;
			}
		}
		return ERROR_FAIL;
	}

	*p_target = target;
	return ERROR_OK;
}

static int freertos_target_for_threadid(struct connection *connection,
	int64_t thread_id,
	struct target **p_target)
{
	struct target *target = get_target_from_connection(connection);

	if (freertos_find_target_from_threadid(target, thread_id, p_target) != ERROR_OK)
		*p_target = target;

	LOG_DEBUG("target found : %s", (*p_target)->cmd_name);

	return ERROR_OK;
}

static bool freertos_halt_reason_priority_higher(enum target_debug_reason old,
	enum target_debug_reason new)
{
	const uint8_t debug_reason_prios[DBG_REASON_NUM] = {
		1,	/* DBG_REASON_DBGRQ, */
		3,	/* DBG_REASON_BREAKPOINT, */
		2,	/* DBG_REASON_WATCHPOINT, */
		3,	/* DBG_REASON_WPTANDBKPT, */
		4,	/* DBG_REASON_SINGLESTEP, */
		0,	/* DBG_REASON_NOTHALTED, */
		0,	/* DBG_REASON_EXIT, */
		5,	/* DBG_REASON_EXC_CATCH, */
		0,	/* DBG_REASON_UNDEFINED, */
	};
	return debug_reason_prios[new] > debug_reason_prios[old];
}

static threadid_t freertos_smp_get_current_thread(struct rtos *rtos)
{
	enum target_debug_reason current_reason = DBG_REASON_UNDEFINED;
	struct target_list *head;
	threadid_t current_thread = 0;

	/* loop over all cores */
	foreach_smp_target(head, rtos->target->smp_targets) {
		struct target *current_target = head->target;
		if (!target_was_examined(current_target))
			continue;

		if (current_target->state != TARGET_HALTED)
			continue;

		threadid_t tid = freertos_current_threadid_from_target(current_target);

		LOG_DEBUG("freertos_current_threadid_from_target 0x%" PRIx64 " @ core %d",
			tid,
			current_target->coreid);

		/* find an interesting core to set as current */
		if (freertos_halt_reason_priority_higher(current_reason,
				current_target->debug_reason)) {
			current_reason = current_target->debug_reason;
			current_thread = tid;
		} else if (current_reason == current_target->debug_reason &&
			tid == rtos->current_threadid) {
			/* prefer the thread selected by GDB */
			current_thread = tid;
		}
	}

	if (current_thread != 0)
		return current_thread;

	return freertos_current_threadid_from_target(rtos->target);
}

static int freertos_update_extra_details(struct target *target)
{
	if (target->smp) {
		struct rtos *rtos = target->rtos;
		for (int i = 0; i < (int)rtos->thread_count; i++) {
			if (rtos->thread_details[i].exists) {
				if (rtos->thread_details[i].extra_info_str) {
					free(rtos->thread_details[i].extra_info_str);
					rtos->thread_details[i].extra_info_str = NULL;
				}

				struct target *current_target = NULL;
				int retval = freertos_find_target_from_threadid(target,
					rtos->thread_details[i].threadid,
					&current_target);
				if (retval != ERROR_OK)
					continue;

				retval = asprintf(&rtos->thread_details[i].extra_info_str,
					STATE_RUNNING_STR,
					current_target->coreid);
				if (retval == -1) {
					LOG_ERROR("Failed to alloc mem for thread extra info!");
					return ERROR_FAIL;
				}
			}
		}
	}

	return ERROR_OK;
}

static int freertos_get_tasks_details(struct target *target,
	const symbol_address_t *task_lists, int num_lists,
	uint64_t current_num_of_tasks, uint32_t *tasks_found)
{
	int retval = ERROR_FAIL;
	struct rtos *rtos = target->rtos;
	struct freertos_data *rtos_data = (struct freertos_data *)rtos->rtos_specific_params;
	uint32_t index = *tasks_found;

	for (int i = 0; i < num_lists; i++) {
		if (task_lists[i] == 0)
			continue;

		/* Read the number of tasks in this list */
		int64_t list_task_count = 0;
		retval = target_buffer_read_uint(target,
			task_lists[i],
			rtos_data->params->thread_count_width,
			(uint64_t *)&list_task_count);

		if (retval != ERROR_OK) {
			LOG_ERROR("Error reading number of threads in FreeRTOS thread list!");
			return retval;
		}

		LOG_DEBUG(
			"FreeRTOS: Read thread count for list %d at 0x%" PRIx64 ", value %" PRId64,
			i,
			task_lists[i],
			list_task_count);
		if (list_task_count == 0)
			continue;

		/* Read the location of first list item */
		uint64_t list_elem_ptr = 0;
		retval = target_buffer_read_uint(target,
			task_lists[i] + rtos_data->params->list_next_offset,
			rtos_data->params->pointer_width,
			&list_elem_ptr);

		if (retval != ERROR_OK) {
			LOG_ERROR(
				"Error reading first thread item location in FreeRTOS thread list!");
			continue;
		}

		LOG_DEBUG(
			"FreeRTOS: Read first item for list %d at 0x%" PRIx64 ", value 0x%" PRIx64,
			i,
			task_lists[i] + rtos_data->params->list_next_offset,
			list_elem_ptr);

		uint64_t list_end_ptr = task_lists[i] + rtos_data->params->list_end_offset;
		LOG_DEBUG("FreeRTOS: End list element at 0x%" PRIx64, list_end_ptr);

		while ((list_task_count > 0) && (list_elem_ptr != 0) &&
			(list_elem_ptr != list_end_ptr) &&
			(index < current_num_of_tasks)) {
			/* Get the location of the thread structure. */
			retval = target_buffer_read_uint(target,
				list_elem_ptr + rtos_data->params->list_elem_content_offset,
				rtos_data->params->pointer_width,
				(uint64_t *)&rtos->thread_details[index].threadid);

			if (retval != ERROR_OK) {
				LOG_WARNING(
					"Error reading thread list item object in FreeRTOS thread list!");
				break;	/* stop list processing */
			}

			LOG_DEBUG(
				"FreeRTOS: Read Thread ID at 0x%" PRIx64 ", value 0x%" PRIx64 " %i",
				list_elem_ptr + rtos_data->params->list_elem_content_offset,
				rtos->thread_details[index].threadid,
				(unsigned int)rtos->thread_details[index].threadid);

			/* get thread name */
			#define FREERTOS_THREAD_NAME_STR_SIZE (64)
			char tmp_str[FREERTOS_THREAD_NAME_STR_SIZE] = { 0 };

			int thread_name_offset = freertos_get_thread_name_offset(rtos);

			retval = target_read_buffer(
				target,
				rtos->thread_details[index].threadid +
				thread_name_offset,
				FREERTOS_THREAD_NAME_STR_SIZE,
				(uint8_t *)&tmp_str);

			if (retval != ERROR_OK) {
				LOG_WARNING("Error reading FreeRTOS thread 0x%" PRIx64 " name!",
					rtos->thread_details[index].threadid);
			} else {
				LOG_DEBUG(
					"FreeRTOS: Read Thread Name at 0x%" PRIx64 ", value \"%s\"",
					rtos->thread_details[index].threadid +
					thread_name_offset,
					tmp_str);

				if (tmp_str[0] == '\x00')
					strcpy(tmp_str, "No Name");

				rtos->thread_details[index].thread_name_str = strdup(tmp_str);
				if (rtos->thread_details[index].thread_name_str == NULL) {
					LOG_ERROR("Failed to alloc mem for thread name!");
					/* Sever error. Smth went wrong on host */
					return ERROR_FAIL;
				}
				rtos->thread_details[index].exists = true;
				const struct freertos_tls_info *tls_info = rtos_freertos_get_tls_info(target);
				if (tls_info &&
					!rtos->thread_details[index].tls_addr) {
					struct rtos_reg reg;
					retval = freertos_get_thread_reg(rtos,
						rtos->thread_details[index].threadid,
						tls_info->tls_reg,
						&reg);
					if (retval == ERROR_OK &&
						(reg.size / 8) <=
						sizeof(rtos->thread_details[index].
							tls_addr)) {
						memcpy(
							&rtos->thread_details[index].
							tls_addr,
							reg.value,
							reg.size / 8);
						rtos->thread_details[index].
						tls_addr += tls_info->tls_align;
					}
				}

				if (target->smp) {
					struct target *current_target;
					retval = freertos_find_target_from_threadid(target,
						rtos->thread_details[index].threadid,
						&current_target);
					if (retval == ERROR_OK) {
						retval = asprintf(
							&rtos->thread_details[index].extra_info_str,
							STATE_RUNNING_STR,
							current_target->coreid);
						if (retval == -1) {
							LOG_ERROR(
								"Failed to alloc mem for thread extra info!");
							free(
								rtos->thread_details[index].
								thread_name_str);
							/* Sever error. Smth went wrong on host */
							return ERROR_FAIL;
						}
					}
				}
				/* save info only for those threads which data can be accessed
				 *successfully */
				index++;
			}
			list_task_count--;

			uint64_t cur_list_elem_ptr = list_elem_ptr;
			list_elem_ptr = 0;
			retval = target_buffer_read_uint(target,
				cur_list_elem_ptr + rtos_data->params->list_elem_next_offset,
				rtos_data->params->pointer_width,
				&list_elem_ptr);

			if (retval != ERROR_OK) {
				LOG_WARNING(
					"Error reading next thread item location in FreeRTOS thread list!");
				break;	/* stop list processing */
			}

			LOG_DEBUG(
				"FreeRTOS: Read next thread location at 0x%" PRIx64 ", value 0x%"
				PRIx64,
				cur_list_elem_ptr + rtos_data->params->list_elem_next_offset,
				list_elem_ptr);
		}

		if (list_elem_ptr == list_end_ptr)
			LOG_DEBUG("FreeRTOS: Reached the end of list %d", i);
	}

	*tasks_found = index;
	return ERROR_OK;
}
static int freertos_update_threads(struct rtos *rtos)
{
	int retval = ERROR_FAIL;
	uint32_t tasks_found = 0;
	struct freertos_data *rtos_data = NULL;
	uint64_t thread_list_size = 0;
	struct target_list *head = NULL;
	struct target *target = NULL;

	LOG_DEBUG("freertos_update_threads");

	if (!rtos->rtos_specific_params)
		return ERROR_FAIL;

	rtos_data = (struct freertos_data *)rtos->rtos_specific_params;

	if (!rtos->symbols) {
		LOG_WARNING("No symbols for FreeRTOS!");
		return ERROR_FAIL;
	}

	if (rtos->symbols[FREERTOS_VAL_UX_CURRENT_NUMBER_OF_TASKS].address == 0) {
		LOG_ERROR("Don't have the number of threads in FreeRTOS!");
		return ERROR_FAIL;
	}

	uint8_t top_used_priority = freertos_get_ux_top_used_priority(rtos);
	if (top_used_priority == 0) {
		LOG_DEBUG(
			"FreeRTOS: uxTopUsedPriority may not defined or not loaded into memory yet."
			"Try to read in the next update request");
		return ERROR_FAIL;
	}

	/* Find out how many lists are needed to be read from pxReadyTasksLists, */
	if (top_used_priority > FREERTOS_MAX_PRIORITIES) {
		LOG_ERROR(
			"FreeRTOS maximum used priority is unreasonably big, not proceeding: %"
			PRId32 "",
			top_used_priority);
		return ERROR_FAIL;
	}

	/* It does not matter which target is halted. Take the first */
	if (rtos->target->smp) {
		/* find target in HALTED state */
		foreach_smp_target(head, rtos->target->smp_targets) {
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

	/* sanity check to make scan-build happy */
	if (!target) {
		LOG_ERROR("target is NULL!");
		return ERROR_FAIL;
	}

	if (target->state != TARGET_HALTED)
		LOG_WARNING("Target [%s] not HALTED!", target->cmd_name);

	retval = target_buffer_read_uint(target,
		rtos->symbols[FREERTOS_VAL_UX_CURRENT_NUMBER_OF_TASKS].address,
		rtos_data->params->thread_count_width,
		&thread_list_size);

	if (retval != ERROR_OK) {
		LOG_ERROR("Could not read FreeRTOS thread count from target!");
		return retval;
	}

	LOG_DEBUG("Read uxCurrentNumberOfTasks at 0x%" PRIx64 ", value %" PRIu64,
		rtos->symbols[FREERTOS_VAL_UX_CURRENT_NUMBER_OF_TASKS].address,
		thread_list_size);

	if (thread_list_size > FREERTOS_MAX_TASKS_NUM) {
		LOG_ERROR("Too large number of threads %" PRIu64 "!", thread_list_size);
		return ERROR_FAIL;
	}

	/* Read uxTaskNumber to detect the task lists changes */
	int64_t uxTaskNumber = 0;
	retval = target_buffer_read_uint(target,
		rtos->symbols[FREERTOS_VAL_UX_TASK_NUMBER].address,
		rtos_data->params->thread_counter_width,
		(uint64_t *)&uxTaskNumber);

	if (retval != ERROR_OK) {
		LOG_ERROR("Could not read FreeRTOS uxTaskNumber from target!");
		return retval;
	}

	LOG_DEBUG("Read uxTaskNumber at 0x%" PRIx64 ", value %" PRIu64,
		rtos->symbols[FREERTOS_VAL_UX_TASK_NUMBER].address,
		uxTaskNumber);

	if (uxTaskNumber < rtos_data->thread_counter) {
		LOG_ERROR("FreeRTOS uxTaskNumber seems to be corrupted!");
		return ERROR_FAIL;
	}

	/* read the current threads */
	retval = target_read_buffer(target,
		freertos_current_tcb_address(rtos),
		rtos_data->params->pointer_width * rtos_data->nr_cpus,
		rtos_data->curr_threads_handles_buff);

	if (retval != ERROR_OK) {
		LOG_ERROR("Could not read FreeRTOS current threads!");
		return retval;
	}

	rtos->current_thread = freertos_smp_get_current_thread(rtos);

	LOG_DEBUG("FreeRTOS: Read pxCurrentTCB at 0x%" PRIx64 ", value 0x%" PRIx64,
		freertos_current_tcb_address(rtos),
		rtos->current_thread);

	if (rtos->thread_count != 0 && uxTaskNumber == rtos_data->thread_counter) {
		/* There is no new added or deleted RTOS task. Just update the state of the current
		 * task */
		return freertos_update_extra_details(target);
	}

	threadid_t temp = rtos->current_thread;
	rtos_free_threadlist(rtos);
	rtos->current_thread = temp;

	/* read scheduler running */
	uint32_t scheduler_running;
	retval = target_read_u32(rtos->target,
		rtos->symbols[FREERTOS_VAL_X_SCHEDULER_RUNNING].address,
		&scheduler_running);
	if (retval != ERROR_OK) {
		LOG_ERROR("Error reading FreeRTOS scheduler state");
		return retval;
	}
	LOG_DEBUG("FreeRTOS: Read xSchedulerRunning at 0x%" PRIx64 ", value 0x%" PRIx32,
		rtos->symbols[FREERTOS_VAL_X_SCHEDULER_RUNNING].address,
		scheduler_running);

	if ((thread_list_size == 0) || (rtos->current_thread == 0) || !scheduler_running) {
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

	/* TODO: check!!! num_lists < top_used_priority or num_lists < config_max_priorities */
	unsigned int num_lists;
	for (num_lists = 0; num_lists < config_max_priorities; num_lists++)
		list_of_lists[num_lists] =
			rtos->symbols[FREERTOS_VAL_PX_READY_TASKS_LISTS].address +
			num_lists * rtos_data->params->list_width;

	list_of_lists[num_lists++] = rtos->symbols[FREERTOS_VAL_X_DELAYED_TASK_LIST1].address;
	list_of_lists[num_lists++] = rtos->symbols[FREERTOS_VAL_X_DELAYED_TASK_LIST2].address;
	list_of_lists[num_lists++] = rtos->symbols[FREERTOS_VAL_X_PENDING_READY_LIST].address;
	list_of_lists[num_lists++] = rtos->symbols[FREERTOS_VAL_X_SUSPENDED_TASK_LIST].address;
	list_of_lists[num_lists++] =
		rtos->symbols[FREERTOS_VAL_X_TASKS_WAITING_TERMINATION].address;

	if (freertos_get_tasks_details(target, list_of_lists, num_lists, thread_list_size,
			&tasks_found) != ERROR_OK) {
		free(list_of_lists);
		return ERROR_FAIL;
	}

	free(list_of_lists);
	rtos->thread_count = tasks_found;
	rtos_data->thread_counter = uxTaskNumber;
	LOG_DEBUG("Task Number updated to:%d", rtos_data->thread_counter);

	return ERROR_OK;
}

static int freertos_get_current_thread_registers(struct rtos *rtos, int64_t thread_id,
	enum target_register_class reg_class, bool *is_curr_thread,
	struct rtos_reg **reg_list, int *num_regs)
{
	int retval;
	struct target *current_target = NULL;

	LOG_DEBUG("freertos_get_current_thread_registers thread_id=0x%x", (uint32_t)thread_id);

	*is_curr_thread = false;
	if (rtos->target->smp)
		freertos_find_target_from_threadid(rtos->target, thread_id, &current_target);
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

	*reg_list = calloc(*num_regs, sizeof(struct rtos_reg));
	if (*reg_list == NULL) {
		LOG_ERROR("Failed to alloc mem for %d", *num_regs);
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

static int freertos_get_thread_registers_from_stack(struct rtos *rtos, int64_t thread_id,
	struct rtos_reg **reg_list, int *num_regs)
{
	int64_t stack_ptr = 0;
	struct freertos_data *rtos_data = (struct freertos_data *)rtos->rtos_specific_params;

	if (rtos_data == NULL)
		return -1;

	/* Read the stack pointer */
	int thread_stack_offset = freertos_get_thread_stack_offset(rtos);

	int retval = target_read_buffer(rtos->target,
		thread_id + thread_stack_offset,
		rtos_data->params->pointer_width,
		(uint8_t *)&stack_ptr);

	if (retval != ERROR_OK) {
		LOG_ERROR("Error reading stack frame from FreeRTOS thread");
		return retval;
	}

	LOG_DEBUG("FreeRTOS: Read stack pointer at 0x%" PRIx64 ", value 0x%" PRIx64,
		thread_id + thread_stack_offset,
		stack_ptr);

	if (rtos_data->params->stacking_info_pick_fn) {
		retval = rtos_generic_stack_read(rtos->target,
			rtos_data->params->stacking_info_pick_fn(rtos, thread_id,
				thread_id + thread_stack_offset),
			stack_ptr, reg_list, num_regs);
		return retval;
	}

	/* Check for armv7m with *enabled* FPU, i.e. a Cortex-M4F */
	int cm4_fpu_enabled = 0;
	struct armv7m_common *armv7m_target = target_to_armv7m(rtos->target);
	if (is_armv7m(armv7m_target)) {
		if ((armv7m_target->fp_feature == FPV4_SP) || (armv7m_target->fp_feature == FPV5_SP) ||
			(armv7m_target->fp_feature == FPV5_DP)) {
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
		retval = target_read_buffer(rtos->target,
			stack_ptr + 0x20,
			rtos_data->params->pointer_width,
			(uint8_t *)&lr_svc);
		if (retval != ERROR_OK) {
			LOG_OUTPUT("Error reading stack frame from FreeRTOS thread");
			return retval;
		}
		if ((lr_svc & 0x10) == 0) {
			return rtos_generic_stack_read(rtos->target,
				rtos_data->params->stacking_info_cm4f_fpu,
				stack_ptr,
				reg_list,
				num_regs);
		} else {
			return rtos_generic_stack_read(rtos->target,
				rtos_data->params->stacking_info_cm4f,
				stack_ptr,
				reg_list,
				num_regs);
		}
	} else {
		return rtos_generic_stack_read(rtos->target,
			rtos_data->params->stacking_info_cm3,
			stack_ptr,
			reg_list,
			num_regs);
	}

	return -1;
}

static int freertos_get_thread_reg_list(struct rtos *rtos, int64_t thread_id,
	struct rtos_reg **reg_list, int *num_regs)
{
	int retval;
	bool is_curr_thread = false;

	if (!rtos)
		return -1;

	if (thread_id == 0)
		return -2;

	LOG_DEBUG("freertos_get_thread_reg_list thread_id=0x%x", (uint32_t)thread_id);

	retval = freertos_get_current_thread_registers(rtos,
		thread_id,
		REG_CLASS_GENERAL,
		&is_curr_thread,
		reg_list,
		num_regs);
	if (retval != ERROR_OK)
		return retval;

	if (is_curr_thread)
		return ERROR_OK;

	return freertos_get_thread_registers_from_stack(rtos, thread_id, reg_list, num_regs);
}

static int freertos_get_thread_reg(struct rtos *rtos, int64_t thread_id,
	uint32_t reg_num, struct rtos_reg *reg)
{
	int retval;
	bool is_curr_thread = false;
	struct rtos_reg *reg_list;
	int num_regs;

	if (rtos == NULL)
		return -1;

	if (thread_id == 0)
		return -2;

	LOG_DEBUG("freertos_get_thread_reg thread_id=0x%x", (uint32_t)thread_id);

	retval = freertos_get_current_thread_registers(rtos,
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
		retval = freertos_get_thread_reg_list(rtos, thread_id, &reg_list, &num_regs);
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

static int freertos_get_symbol_list_to_lookup(struct symbol_table_elem *symbol_list[])
{
	unsigned int i;

	*symbol_list = calloc(ARRAY_SIZE(freertos_symbol_list), sizeof(struct symbol_table_elem));

	for (i = 0; i < ARRAY_SIZE(freertos_symbol_list); i++) {
		(*symbol_list)[i].symbol_name = freertos_symbol_list[i].name;
		(*symbol_list)[i].optional = freertos_symbol_list[i].optional;
	}

	return 0;
}

static int freertos_post_reset_cleanup(struct target *target)
{
	LOG_DEBUG("freertos_post_reset_cleanup");

	struct freertos_data *rtos_data =
		(struct freertos_data *)target->rtos->rtos_specific_params;

	if ((target->rtos->symbols != NULL) &&
		(target->rtos->symbols[FREERTOS_VAL_UX_CURRENT_NUMBER_OF_TASKS].address != 0)) {
		int ret = target_buffer_write_uint(target,
			target->rtos->symbols[FREERTOS_VAL_UX_CURRENT_NUMBER_OF_TASKS].address,
			rtos_data->params->thread_count_width,
			0);
		if (ret != ERROR_OK) {
			LOG_ERROR("Failed to clear uxCurrentNumberOfTasks!");
			return ret;
		}
		if (target->smp) {
			/* clear pxCurrentTCB for all cores */
			struct target_list *head;
			foreach_smp_target(head, target->smp_targets) {
				struct target *current_target = head->target;
				if (!target_was_examined(current_target))
					continue;
				ret = target_buffer_write_uint(
					target,
					freertos_current_tcb_address(target->rtos)
					+
					current_target->coreid * rtos_data->params->pointer_width,
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
				freertos_current_tcb_address(target->rtos),
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
	free(rtos_data->esp_symbols);
	rtos_data->esp_symbols = NULL;
	return ERROR_OK;
}

static int freertos_clean(struct target *target)
{
	struct freertos_data *rtos_data =
		(struct freertos_data *)target->rtos->rtos_specific_params;

	LOG_DEBUG("freertos_clean");
	/* if rtos_auto_detect is true freertos_create() will be called upon receiption of the first
	 * 'qSymbol', */
	/* so we can free resources
	 * if rtos_auto_detect is false freertos_create() is called only once upon target creation
	 * freertos_clean() is called every time GDB initiates new connection
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
	LOG_DEBUG("freertos_create");

	size_t i = 0;
	while ((i < FREERTOS_NUM_PARAMS) &&
		(0 != strcmp(freertos_params_list[i].target_name, target->type->name))) {
		i++;
	}
	if (i >= FREERTOS_NUM_PARAMS) {
		LOG_ERROR("Could not find target in FreeRTOS compatibility list");
		return JIM_ERR;
	}

	if (!(freertos_params_list[i].pointer_width == 4 ||
			freertos_params_list[i].pointer_width == 8)) {
		LOG_ERROR("Unsupported OS pointer width %u!",
			freertos_params_list[i].pointer_width);
		return JIM_ERR;
	}

	struct freertos_data *rtos_data = calloc(1, sizeof(struct freertos_data));
	if (!rtos_data) {
		LOG_ERROR("Failed to allocate OS data!");
		return JIM_ERR;
	}
	rtos_data->nr_cpus = 1;
	rtos_data->thread_counter = 0;
	rtos_data->params = &freertos_params_list[i];
	rtos_data->curr_threads_handles_buff = calloc(rtos_data->nr_cpus, rtos_data->params->pointer_width);
	if (!rtos_data->curr_threads_handles_buff) {
		free(rtos_data);
		LOG_ERROR("Failed to allocate current threads handles buffer!");
		return JIM_ERR;
	}
	target->rtos->rtos_specific_params = rtos_data;
	target->rtos->thread_details = NULL;
	target->rtos->gdb_target_for_threadid = freertos_target_for_threadid;
	return JIM_OK;
}
