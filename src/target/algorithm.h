/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
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

#ifndef OPENOCD_TARGET_ALGORITHM_H
#define OPENOCD_TARGET_ALGORITHM_H

#include <stdarg.h>
#include "log.h"
#include "image.h"
#include "helper/binarybuffer.h"

enum param_direction {
	PARAM_IN,
	PARAM_OUT,
	PARAM_IN_OUT
};

struct mem_param {
	target_addr_t address;
	uint32_t size;
	uint8_t *value;
	enum param_direction direction;
};

struct reg_param {
	const char *reg_name;
	uint32_t size;
	uint8_t *value;
	enum param_direction direction;
};

void init_mem_param(struct mem_param *param,
		uint32_t address, uint32_t size, enum param_direction dir);
void destroy_mem_param(struct mem_param *param);

void init_reg_param(struct reg_param *param,
		char *reg_name, uint32_t size, enum param_direction dir);
void destroy_reg_param(struct reg_param *param);


/**
 * Algorithm image data.
 * Helper struct to work with algorithms consisting of code and data segments.
 */
struct algorithm_image {
	/** Image. */
	struct image image;
	/** BSS section size. */
	uint32_t bss_size;
};

/**
 * Algorithm stub data.
 */
struct algorithm_stub {
	/** Entry addr. */
	target_addr_t entry;
	/** Working area for code segment. */
	struct working_area *code;
	/** Working area for data segment. */
	struct working_area *data;
	/** Working area for tramploline. */
	struct working_area *tramp;
	/** Address of the target buffer for stub trampoline. If zero tramp->address will be used. */
	target_addr_t tramp_addr;
	/** Working area for stack. */
	struct working_area *stack;
	/** Address of the target buffer for stack. If zero tramp->address will be used. */
	target_addr_t stack_addr;
	/** Algorithm's arch-specific info. */
	void *ainfo;
};

/**
 * Algorithm stub in-memory arguments.
 */
struct algorithm_mem_args {
	/** Memory params. */
	struct mem_param *	params;
	/** Number of memory params. */
	uint32_t 			count;
};

/**
 * Algorithm stub register arguments.
 */
struct algorithm_reg_args {
	/** Algorithm register params. User args start from user_first_reg_param */
	struct reg_param *	params;
	/** Number of register params. */
	uint32_t 			count;
	/** The first several reg_params can be used by stub itself (e.g. for trampoline).
	 * This is the index of the first reg_param available for user to pass args to algorithm stub. */
	uint32_t 			first_user_param;
};

struct algorithm_run_data;

/**
 * @brief Algorithm run function.
 *
 * @param target Pointer to target.
 * @param run    Pointer to algo run data.
 * @param arg    Function specific argument.
 *
 * @return ERROR_OK on success, otherwise ERROR_XXX.
 */
typedef int (*algorithm_func_t)(struct target *target, struct algorithm_run_data *run,
	void *arg);

/**
 * @brief Host part of algorithm.
 *        This function will be called while stub is running on target.
 *        It can be used for communication with stub.
 *
 * @param target  Pointer to target.
 * @param usr_arg Function specific argument.
 *
 * @return ERROR_OK on success, otherwise ERROR_XXX.
 */
typedef int (*algorithm_usr_func_t)(struct target *target, void *usr_arg);

/**
 * @brief Algorithm's arguments setup function.
 *        This function will be called just before stub start.
 *        It must return when all operations with running stub are completed.
 *        It can be used to prepare stub memory parameters.
 *
 * @param target  Pointer to target.
 * @param run     Pointer to algo run data.
 * @param usr_arg Function specific argument. The same as for algorithm_usr_func_t.
 *
 * @return ERROR_OK on success, otherwise ERROR_XXX.
 */
typedef int (*algorithm_usr_func_init_t)(struct target *target, struct algorithm_run_data *run,
	void *usr_arg);

/**
 * @brief Algorithm's arguments cleanup function.
 *        This function will be called just after stub exit.
 *        It can be used to cleanup stub memory parameters.
 *
 * @param target  Pointer to target.
 * @param run     Pointer to algo run data.
 * @param usr_arg Function specific argument. The same as for algorithm_usr_func_t.
 *
 * @return ERROR_OK on success, otherwise ERROR_XXX.
 */
typedef void (*algorithm_usr_func_done_t)(struct target *target, struct algorithm_run_data *run,
	void *usr_arg);

struct algorithm_hw {
	int (*algo_init)(struct target *target, struct algorithm_run_data *run,
		void *arch_info,
		uint32_t num_args,
		va_list ap);
	int (*algo_cleanup)(struct algorithm_run_data *run);
	const uint8_t *(*stub_tramp_get)(struct target *target, size_t *size);
};

/**
 * Algorithm run data.
 */
struct algorithm_run_data {
	/** Algoritm completion timeout in ms. If 0, default value will be used */
	uint32_t tmo;
	/** Algorithm stack size. */
	uint32_t stack_size;
	/** Algorithm register arguments. */
	struct algorithm_reg_args reg_args;
	/** Algorithm memory arguments. */
	struct algorithm_mem_args mem_args;
	/** Algorithm arch-specific info. For Xtensa this should point to struct xtensa_algorithm. */
	void *arch_info;
	/** Algorithm return code. */
	int64_t ret_code;
	/** Stub. */
	struct algorithm_stub stub;
	union {
		struct {
			/** Size of the pre-alocated on-board buffer for stub's code. */
			uint32_t code_buf_size;
			/** Address of pre-compiled target buffer for stub trampoline. */
			target_addr_t code_buf_addr;
			/** Size of the pre-alocated on-board buffer for stub's stack. */
			uint32_t min_stack_size;
			/** Pre-compiled target buffer's addr for stack. */
			target_addr_t min_stack_addr;
			/** Address of malloc-like function to allocate buffer on target. */
			target_addr_t alloc_func;
			/** Address of free-like function to free buffer allocated with on_board_alloc_func. */
			target_addr_t free_func;
		} on_board;
		struct algorithm_image image;
	};
	/** Host side algorithm function argument. */
	void *usr_func_arg;
	/** Host side algorithm function. */
	algorithm_usr_func_t usr_func;
	/** Host side algorithm function setup routine. */
	algorithm_usr_func_init_t usr_func_init;
	/** Host side algorithm function cleanup routine. */
	algorithm_usr_func_done_t usr_func_done;
	/** Algorithm run function: see algorithm_run_xxx for example. */
	algorithm_func_t algo_func;
	/** HW specific API */
	const struct algorithm_hw *hw;
};

int algorithm_load_func_image(struct target *target,
	struct algorithm_run_data *run);

int algorithm_unload_func_image(struct target *target,
	struct algorithm_run_data *run);

int algorithm_exec_func_image_va(struct target *target,
	struct algorithm_run_data *run,
	uint32_t num_args,
	va_list ap);

static inline int algorithm_exec_func_image(struct target *target,
	struct algorithm_run_data *run,
	uint32_t num_args,
	...)
{
	va_list ap;
	va_start(ap, num_args);
	int retval = algorithm_exec_func_image_va(target, run, num_args, ap);
	va_end(ap);
	return retval;
}

/**
 * @brief Loads and runs stub from specified image.
 *        This function should be used to run external stub code on target.
 *
 * @param target   Pointer to target.
 * @param run      Pointer to algo run data.
 * @param num_args Number of stub arguments that follow.
 *
 * @return ERROR_OK on success, otherwise ERROR_XXX. Stub return code is in run->ret_code.
 */
static inline int algorithm_run_func_image_va(struct target *target,
	struct algorithm_run_data *run,
	uint32_t num_args,
	va_list ap)
{
	int ret = algorithm_load_func_image(target, run);
	if (ret != ERROR_OK)
		return ret;
	ret = algorithm_exec_func_image_va(target, run, num_args, ap);
	if (ret != ERROR_OK)
		return ret;
	return algorithm_unload_func_image(target, run);
}

static inline int algorithm_run_func_image(struct target *target,
	struct algorithm_run_data *run,
	uint32_t num_args,
	...)
{
	va_list ap;
	va_start(ap, num_args);
	int retval = algorithm_run_func_image_va(target, run, num_args, ap);
	va_end(ap);
	return retval;
}

int algorithm_load_onboard_func(struct target *target,
	target_addr_t func_addr,
	struct algorithm_run_data *run);

int algorithm_unload_onboard_func(struct target *target,
	struct algorithm_run_data *run);

int algorithm_exec_onboard_func_va(struct target *target,
	struct algorithm_run_data *run,
	uint32_t num_args,
	va_list ap);

/**
 * @brief Runs pre-compiled on-board function.
 *        This function should be used to run on-board stub code.
 *
 * @param target     Pointer to target.
 * @param run        Pointer to algo run data.
 * @param func_entry Address of the function to run.
 * @param num_args   Number of function arguments that follow.
 *
 * @return ERROR_OK on success, otherwise ERROR_XXX. Stub return code is in run->ret_code.
 */
static inline int algorithm_run_onboard_func_va(struct target *target,
	struct algorithm_run_data *run,
	target_addr_t func_addr,
	uint32_t num_args,
	va_list ap)
{
	int ret = algorithm_load_onboard_func(target, func_addr, run);
	if (ret != ERROR_OK)
		return ret;
	ret = algorithm_exec_onboard_func_va(target, run, num_args, ap);
	if (ret != ERROR_OK)
		return ret;
	return algorithm_unload_onboard_func(target, run);
}

static inline int algorithm_run_onboard_func(struct target *target,
	struct algorithm_run_data *run,
	target_addr_t func_addr,
	uint32_t num_args,
	...)
{
	va_list ap;
	va_start(ap, num_args);
	int retval = algorithm_run_onboard_func_va(target, run, func_addr, num_args, ap);
	va_end(ap);
	return retval;
}

static inline void algorithm_user_arg_set_uint(struct algorithm_run_data *run, int arg_num, uint64_t val)
{
	struct reg_param *param = &run->reg_args.params[run->reg_args.first_user_param + arg_num];
	assert(param->size <= 64);
	if (param->size <= 32) {
		buf_set_u32(param->value,
			0,
			param->size,
			val);
	} else {
		buf_set_u64(param->value,
			0,
			param->size,
			val);
	}
}

static inline uint64_t algorithm_user_arg_get_uint(struct algorithm_run_data *run, int arg_num)
{
	struct reg_param *param = &run->reg_args.params[run->reg_args.first_user_param + arg_num];
	assert(param->size <= 64);
	if (param->size <= 32) {
		return buf_get_u32(
				param->value,
				0,
				param->size);
	}
	return buf_get_u64(
			param->value,
			0,
			param->size);
}

#endif /* OPENOCD_TARGET_ALGORITHM_H */
