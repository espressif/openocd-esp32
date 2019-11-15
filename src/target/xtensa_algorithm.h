/***************************************************************************
 *   Module to run arbitrary code on Xtensa using OpenOCD                  *
 *   Copyright (C) 2019 Espressif Systems Ltd.                             *
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

#ifndef XTENSA_ALGO_H
#define XTENSA_ALGO_H

/**
 * API defined below allows to execute pieces of code on target without breaking execution of running program.
 * This functionality can be useful for various debugging and maintenance procedures.
 * @note As of now ESP32 flashing code to load flasher stub on target and write/read/erase flash.
 * Also GCOV command uses some of these functions to run onboard routines to dump coverage info.
 * Stub entry function can take up to 5 arguments and should be of the following form:
 *
 * int stub_entry([uint32_t a1 [, uint32_t a2 [, uint32_t a3 [, uint32_t a4 [, uint32_t a5]]]]]);
 *
 * General scheme of stub code execution is shown below.
 *
 *  -------                                                    -----------   (initial frame)    ----
 * |       | ---(sp,ps,window state,stub entry,stub args)---> |trampoline | ---(stub args)---> |    |
 * |       |                                                  |           |                    |    |
 * |OpenOCD| <----------(stub-specific communications)---------------------------------------> |stub|
 * |       |                                                  |           |                    |    |
 * |       | <---------(target halted event, ret code)------- |tramp break| <---(ret code)---- |    |
 *  -------                                                    -----------                      ----
 *
 * Procedure of executing stub on target includes:
 * 1) User prepares struct xtensa_algo_run_data and calls one of esp32_run_xxx() functions.
 * 2) Routine allocates all necessary stub code and data sections.
 * 3) If user specified arguments initializer func xtensa_algo_usr_func_init_t it is called just before stub start.
 * 4) If user specified stub communication func xtensa_algo_usr_func_t (@see esp32_write/read in ESP32 flash driver)
 *    it is called just after the stub start. When communication with stub is finished this function must return.
 * 5) OpenOCD waits for stub to finish (hit exit breakpoint).
 * 6) If user specified arguments cleanup func xtensa_algo_usr_func_done_t it is called just after stub finishes.
 *
 * There are two options to run code on ESP32 under OpenOCD control:
 * - Run externally compiled stub code.
 * - Run onboard pre-compiled code. @note Debug stubs must be enabled in target code @see ESP IDF docs.
 * The main difference between execution of external stub code and target built-in functions is that in latter case
 * working areas can not be used to allocate target memory for code and data because it they can overlap
 * with code and data involved into onboard function execution. For example if memory allocated in working area
 * for stub stack will overlap with some on-board data used by the stub the stack will get overwritten.
 * The same stands for allocations in target code space.
 *
 * External Code Execution
 * -----------------------
 * To run external code on ESP32 user should use xtensa_run_func_image() or xtensa_run_algorithm_image().
 * In this case all necesarry memory (code/data) is alocated in working areas which have fixed configuration
 * defined in TCL ESP32 file. Stub code is actually standalone program, so all its segments must have known
 * addresses due to position dependent code nature. So stub must be linked in such way that its code segment
 * starts at the beginning of code space working area defined in TCL. The same restriction must be applied
 * to stub data segment and data space working area base addresses. @see ESP32 flasher LD script.
 * Also in order to simplify memory allocation BSS section must follow DATA section in stub image.
 * The size of BSS section must be specofied in bss_size field of struct xtensa_algo_image.
 * Sample stub memory map is shown below.
 *  ___________________________________________
 * | data space working area start             |
 * |                                           |
 * | <stub .data segment>                      |
 * |___________________________________________|
 * | stub .bss start                           |
 * |                                           |
 * | <stub .bss segment of size 'bss_size'>    |
 * |___________________________________________|
 * | stub stack base                           |
 * |                                           |
 * | <stub stack>                              |
 * |___________________________________________|
 * |                                           |
 * | <stub mem arg1>                           |
 * |___________________________________________|
 * |                                           |
 * | <stub mem arg2>                           |
 * |___________________________________________|
 *  ___________________________________________
 * | code space working area start             |
 * |                                           |
 * | <stub .text segment>                      |
 * |___________________________________________|
 * |                                           |
 * | <stub trampoline with exit breakpoint>    |
 * |___________________________________________|
 *
 * For example on how to execute external code with memory arguments @see esp32_blank_check in ESP32 flash driver.
 *
 * On-Board Code Execution
 * -----------------------
 * To run on-board code on ESP32 user should use xtensa_run_onboard_func() or xtensa_run_algorithm_onboard().
 * On-board code execution process does not need to allocate target memory for stub code and data, but it still needs
 * memory for stub trampoline, stack and memory arguments. Working areas can not be used due to possible memory layout
 * conflicts with on-board stub code and data.
 * Debug stubs functionality provided by ESP IDF allows to overcome above problem. It provides special descriptor
 * which provides info necessary to safely allocate memory on target @see struct esp_dbg_stubs_desc.
 * That info is also used to locate memory for stub trampoline code.
 * User can execute target function at any address, but @see ESP IDF debug stubs provide way to pass to the host entry address
 * info for registered stubs. For example of on-board code execution @see esp32_cmd_gcov() in ESP32 apptrace module.
 *
*/

#include <target/image.h>
#include <target/algorithm.h>
#include <target/xtensa.h>

/** Index of the first user-defined algo arg. @see xtensa_stub */
#define XTENSA_STUB_ARGS_FUNC_START             6
/** Max number of all algo args. @see xtensa_stub */
#define XTENSA_STUB_ARGS_MAX                    (XTENSA_STUB_ARGS_FUNC_START+5)

/**
 * Xtensa algorithm data.
 */
struct xtensa_algorithm {
	enum xtensa_mode core_mode;
	xtensa_reg_val_t context[XT_NUM_REGS];
	enum target_debug_reason ctx_debug_reason;
};

/**
 * Xtensa stub data.
 */
struct xtensa_stub {
	/** Entry addr. */
	target_addr_t entry;
	/** Working area for code segment. */
	struct working_area *code;
	/** Working area for data segment. */
	struct working_area *data;
	/** Working area for tramploline. */
	struct working_area *tramp;
	/** Address of the target buffer for stub trampoline. */
	target_addr_t tramp_addr;
	/** Working area for stack. */
	struct working_area *stack;
	/** Address of the target buffer for stack. */
	target_addr_t stack_addr;
	/** Algorithm's arch-specific info. */
	struct xtensa_algorithm ainfo;
	/** Stub register params. User args start from XTENSA_STUB_ARGS_FUNC_START */
	struct reg_param reg_params[XTENSA_STUB_ARGS_MAX];
	/** Number of register params. */
	uint32_t reg_params_count;
};

/**
 * Xtensa stub arguments.
 */
struct xtensa_algo_mem_args {
	/** Memory params. */
	struct mem_param *params;
	/** Number of memory params. */
	uint32_t count;
};

/**
 * Xtensa algorithm image data.
 */
struct xtensa_algo_image {
	/** Stub image. */
	struct image image;
	/** Stub BSS section size. */
	uint32_t bss_size;
};

struct xtensa_algo_run_data;

/**
 * @brief Algorithm run function.
 *
 * @param target Pointer to target.
 * @param run    Pointer to algo run data.
 * @param arg    Function specific argument.
 *
 * @return ERROR_OK on success, otherwise ERROR_XXX.
 */
typedef int (*xtensa_algo_func_t)(struct target *target, struct xtensa_algo_run_data *run,
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
typedef int (*xtensa_algo_usr_func_t)(struct target *target, void *usr_arg);

/**
 * @brief Algorithm's arguments setup function.
 *        This function will be called just before stub start.
 *        It must return when all operations with running stub are completed.
 *        It can be used to prepare stub memory parameters.
 *
 * @param target  Pointer to target.
 * @param run     Pointer to algo run data.
 * @param usr_arg Function specific argument. The same as for xtensa_algo_usr_func_t.
 *
 * @return ERROR_OK on success, otherwise ERROR_XXX.
 */
typedef int (*xtensa_algo_usr_func_init_t)(struct target *target, struct xtensa_algo_run_data *run,
	void *usr_arg);

/**
 * @brief Algorithm's arguments cleanup function.
 *        This function will be called just after stub exit.
 *        It can be used to cleanup stub memory parameters.
 *
 * @param target  Pointer to target.
 * @param run     Pointer to algo run data.
 * @param usr_arg Function specific argument. The same as for xtensa_algo_usr_func_t.
 *
 * @return ERROR_OK on success, otherwise ERROR_XXX.
 */
typedef void (*xtensa_algo_usr_func_done_t)(struct target *target, struct xtensa_algo_run_data *run,
	void *usr_arg);

/**
 * Xtensa algorithm run data.
 */
struct xtensa_algo_run_data {
	/** Algoritm completion timeout in ms. If 0, default value will be used */
	uint32_t tmo;
	/** Stub stack size. */
	uint32_t stack_size;
	struct {
		/** Size of the pre-alocated on-board buffer for stub's code. */
		uint32_t code_buf_size;
		/** Address of pre-compiled target buffer for stub trampoline. The size of buffer
		 * the is ESP_DBG_STUBS_CODE_BUF_SIZE. */
		target_addr_t code_buf_addr;
		/** Size of the pre-alocated on-board buffer for stub's stack. */
		uint32_t min_stack_size;
		/** Pre-compiled target buffer's addr for stack. */
		target_addr_t min_stack_addr;
		/** Address of malloc-like function to allocate buffer on target. */
		target_addr_t alloc_func;
		/** Address of free-like function to free buffer allocated with on_board_alloc_func.
		 * */
		target_addr_t free_func;
	} on_board;
	/** Stub stack size. */
	struct xtensa_algo_mem_args mem_args;
	/** Host side algorithm function argument. */
	void *usr_func_arg;
	/** Host side algorithm function. */
	xtensa_algo_usr_func_t usr_func;
	/** Host side algorithm function setup routine. */
	xtensa_algo_usr_func_init_t usr_func_init;
	/** Host side algorithm function cleanup routine. */
	xtensa_algo_usr_func_done_t usr_func_done;
	/** Stub return code. */
	int32_t ret_code;
	/** Algorithm run function: esp32_run_algorithm_xxx. */
	xtensa_algo_func_t algo_func;
	/* user should init to zero the following fields and should not use them anyhow else.
	 * only for internal uses */
	struct {
		/** Stub. */
		struct xtensa_stub stub;
	} priv;
};

/**
 * @brief Loads and runs stub from specified image.
 *        This function should be used to run external stub code on target.
 *
 * @param target   Pointer to target.
 * @param run      Pointer to algo run data.
 * @param image    Pointer to algo image.
 * @param num_args Number of stub arguments that follow.
 *
 * @return ERROR_OK on success, otherwise ERROR_XXX. Stub return code is in run->ret_code.
 */
int xtensa_run_func_image_va(struct target *target,
	struct xtensa_algo_run_data *run,
	struct xtensa_algo_image *image,
	uint32_t num_args,
	va_list ap);

static inline int xtensa_run_func_image(struct target *target,
	struct xtensa_algo_run_data *run,
	struct xtensa_algo_image *image,
	uint32_t num_args,
	...)
{
	va_list ap;
	va_start(ap, num_args);
	int retval = xtensa_run_func_image_va(target, run, image, num_args, ap);
	va_end(ap);
	return retval;
}

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
int xtensa_run_onboard_func_va(struct target *target,
	struct xtensa_algo_run_data *run,
	uint32_t func_addr,
	uint32_t num_args,
	va_list ap);

static inline int xtensa_run_onboard_func(struct target *target,
	struct xtensa_algo_run_data *run,
	uint32_t func_addr,
	uint32_t num_args,
	...)
{
	va_list ap;
	va_start(ap, num_args);
	int retval = xtensa_run_onboard_func_va(target, run, func_addr, num_args, ap);
	va_end(ap);
	return retval;
}

/**
 * @brief External stub code run function.
 *
 * @param target Pointer to target.
 * @param run    Pointer to algo run data.
 * @param image  Pointer to algo image.
 *
 * @return ERROR_OK on success, otherwise ERROR_XXX.
 */
int xtensa_run_algorithm_image(struct target *target,
	struct xtensa_algo_run_data *run,
	struct xtensa_algo_image *image);

/**
 * @brief On-board stub code run function.
 *
 * @param target     Pointer to target.
 * @param run        Pointer to algo run data.
 * @param func_entry Address of the function to run.
 *
 * @return ERROR_OK on success, otherwise ERROR_XXX.
 */
int xtensa_run_algorithm_onboard(struct target *target,
	struct xtensa_algo_run_data *run,
	void *func_entry);

#endif	/* XTENSA_ESP32_H */
