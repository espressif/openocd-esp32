/***************************************************************************
 *   ESP32 target for OpenOCD                                              *
 *   Copyright (C) 2017 Espressif Systems Ltd.                             *
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

#ifndef XTENSA_ESP32_H
#define XTENSA_ESP32_H

#include <target/image.h>
#include "esp108_common.h"

#define ESP32_CPU_COUNT		2
#define ESP32_PRO_CPU_ID	0
#define ESP32_APP_CPU_ID	1

enum esp32_isrmasking_mode {
	ESP32_ISRMASK_OFF,
	ESP32_ISRMASK_ON,
};

// must be in sync with ESP-IDF version
/** Size of the pre-compiled target buffer for stub trampoline.
 * @note Must be in sync with ESP-IDF version */
#define ESP32_DBG_STUBS_CODE_BUF_SIZE       32
/** Size of the pre-compiled target buffer for stack.
 * @note Must be in sync with ESP-IDF version */
#define ESP32_DBG_STUBS_STACK_MIN_SIZE      2048

#define ESP32_FLASH_SW_BREAKPOINTS_MAX_NUM  32
#define ESP32_SW_BREAKPOINTS_MAX_NUM      	32

/**
 * Debug stubs table entries IDs
 *
 * @note Must be in sync with ESP-IDF version
 */
typedef enum {
	ESP32_DBG_STUB_TABLE_START,
	ESP32_DBG_STUB_DESC = ESP32_DBG_STUB_TABLE_START, 		///< Stubs descriptor ID
	ESP32_DBG_STUB_ENTRY_FIRST,
	ESP32_DBG_STUB_ENTRY_GCOV = ESP32_DBG_STUB_ENTRY_FIRST,	///< GCOV stub ID
	// add new stub entries here
	ESP32_DBG_STUB_ENTRY_MAX,
} esp32_dbg_stub_id_t;

/**
 * Debug stubs descriptor. ID: ESP32_DBG_STUB_DESC
 *
 * @note Must be in sync with ESP-IDF version
 */
struct esp32_dbg_stubs_desc {
	/** Address of pre-compiled target buffer for stub trampoline. The size of buffer the is ESP32_DBG_STUBS_CODE_BUF_SIZE. */
	uint32_t    tramp_addr;
	/** Pre-compiled target buffer's addr for stack. The size of the buffer is ESP32_DBG_STUBS_STACK_MIN_SIZE.
	    Target has the buffer which is used for the stack of onboard algorithms.
	If stack size required by algorithm exceeds ESP32_DBG_STUBS_STACK_MIN_SIZE,
	it should be allocated using onboard function pointed by 'data_alloc' and
	freed by 'data_free'. They fit to the minimal stack. See below. */
	uint32_t    min_stack_addr;
	/** Address of malloc-like function to allocate buffer on target. */
	uint32_t    data_alloc;
	/** Address of free-like function to free buffer allocated with data_alloc. */
	uint32_t    data_free;
};

/**
 * Debug stubs info.
 */
struct esp32_dbg_stubs {
	/** Address. */
	uint32_t 					base;
	/** Table contents. */
	uint32_t 					entries[ESP32_DBG_STUB_ENTRY_MAX];
	/** Number of table entries. */
	uint32_t 					entries_count;
	/** Debug stubs decsriptor. */
	struct esp32_dbg_stubs_desc desc;
};

struct esp32_sw_breakpoint {
	struct breakpoint *oocd_bp;
	// insn
	uint8_t insn[3];
	// insn size
	uint8_t insn_sz; // 2 or 3 bytes
};

struct esp32_flash_sw_breakpoint {
	struct flash_bank *bank;
	struct esp32_sw_breakpoint data;
};

struct esp32_common {

	// Common fields definition for all esp108 targets
	ESP108_COMMON_FIELDS;

	uint8_t                             prevpwrstat[ESP32_CPU_COUNT];
	struct target*                      esp32_targets[ESP32_CPU_COUNT];
	size_t                              active_cpu;
	struct reg_cache *                  core_caches[ESP32_CPU_COUNT];
	size_t                              cores_num;
	uint32_t                            core_poweron_mask;
	// TODO: Below are candidates to be moved to ESP108_COMMON_FIELDS
	int64_t                             current_threadid;
	enum esp32_isrmasking_mode          isrmasking_mode;
	struct esp32_sw_breakpoint **       sw_brps;
	struct esp32_flash_sw_breakpoint ** flash_sw_brps;
	struct esp32_dbg_stubs              dbg_stubs;
	uint32_t                            appimage_flash_base;
};

struct esp32_flash_sw_breakpoint * esp32_add_flash_breakpoint(struct target *target, struct breakpoint *breakpoint);
int esp32_remove_flash_breakpoint(struct target *target, struct esp32_flash_sw_breakpoint *breakpoint);


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
 * 1) User prepares struct esp32_algo_run_data and calls one of esp32_run_xxx() functions.
 * 2) Routine allocates all necessary stub code and data sections.
 * 3) If user specified arguments initializer func esp32_algo_usr_func_init_t it is called just before stub start.
 * 4) If user specified stub communication func esp32_algo_usr_func_t (@see esp32_write/read in ESP32 flash driver)
 *    it is called just after the stub start. When communication with stub is finished this function must return.
 * 5) OpenOCD waits for stub to finish (hit exit breakpoint).
 * 6) If user specified arguments cleanup func esp32_algo_usr_func_done_t it is called just after stub finishes.
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
 * To run external code on ESP32 user should use esp32_run_func_image() or esp32_run_algorithm_image().
 * In this case all necesarry memory (code/data) is alocated in working areas which have fixed configuration
 * defined in TCL ESP32 file. Stub code is actually standalone program, so all its segments must have known
 * addresses due to position dependent code nature. So stub must be linked in such way that its code segment
 * starts at the beginning of code space working area defined in TCL. The same restriction must be applied
 * to stub data segment and data space working area base addresses. @see ESP32 flasher LD script.
 * Also in order to simplify memory allocation BSS section must follow DATA section in stub image.
 * The size of BSS section must be specofied in bss_size field of struct esp32_algo_image.
 * Sample stub memory map is shown below.
 *  ___________________________________________
 * | 0x3FFC0000: data space working area start |
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
 * | 0x40090000: code space working area start |
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
 * To run on-board code on ESP32 user should use esp32_run_onboard_func() or esp32_run_algorithm_onboard().
 * On-board code execution process does not need to allocate target memory for stub code and data, but it still needs
 * memory for stub trampoline, stack and memory arguments. Working areas can not be used due to possible memory layout
 * conflicts with on-board stub code and data.
 * Debug stubs functionality provided by ESP IDF allows to overcome above problem. It provides special descriptor
 * which provides info necessary to safely allocate memory on target @see struct esp32_dbg_stubs_desc.
 * That info is also used to locate memory for stub trampoline code.
 * User can execute target function at any address, but @see ESP IDF debug stubs provide way to pass to the host entry address
 * info for registered stubs. For example of on-board code execution @see esp_cmd_gcov() in ESP32 apptrace module.
 *
*/

/** Index of the first user-defined algo arg. @see esp32_stub */
#define ESP32_STUB_ARGS_FUNC_START  		6
/** Max number of all algo args. @see esp32_stub */
#define ESP32_STUB_ARGS_MAX         		(ESP32_STUB_ARGS_FUNC_START+5)

/**
 * ESP32 stub data.
 */
struct esp32_stub {
	/** Entry addr. */
	uint32_t 				entry;
	/** Working area for code segment. */
	struct working_area *	code;
	/** Working area for data segment. */
	struct working_area *	data;
	/** Working area for tramploline. */
	struct working_area *	tramp;
	/** Address of the target buffer for stub trampoline. */
	uint32_t 				tramp_addr;
	/** Working area for stack. */
	struct working_area *	stack;
	/** Address of the target buffer for stack. */
	uint32_t 				stack_addr;
	/** Algorithm's arch-specific info. */
	struct 					xtensa_algorithm ainfo;
	/** Stub register params. User args start from ESP32_STUB_ARGS_FUNC_START */
	struct 					reg_param reg_params[ESP32_STUB_ARGS_MAX];
	/** Number of register params. */
	uint32_t 				reg_params_count;
};

/**
 * ESP32 stub arguments.
 */
struct esp32_algo_mem_args {
	/** Memory params. */
	struct mem_param *	params;
	/** Number of memory params. */
	uint32_t			count;
};

/**
 * ESP32 algorithm image data.
 */
struct esp32_algo_image {
	/** Stub image. */
	struct 		image image;
	/** Stub BSS section size. */
	uint32_t 	bss_size;
};

struct esp32_algo_run_data;

/**
 * @brief Algorithm run function.
 *
 * @param target Pointer to target.
 * @param run    Pointer to algo run data.
 * @param arg    Function specific argument.
 *
 * @return ERROR_OK on success, otherwise ERROR_XXX.
 */
typedef int (*esp32_algo_func_t)(struct target *target, struct esp32_algo_run_data *run, void *arg);

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
typedef int (*esp32_algo_usr_func_t)(struct target *target, void *usr_arg);

/**
 * @brief Algorithm's arguments setup function.
 *        This function will be called just before stub start. \
 *        It must return when all operations with running stub are completed.
 *        It can be used to prepare stub memory parameters.
 *
 * @param target  Pointer to target.
 * @param run     Pointer to algo run data.
 * @param usr_arg Function specific argument. The same as for esp32_algo_usr_func_t.
 *
 * @return ERROR_OK on success, otherwise ERROR_XXX.
 */
typedef int (*esp32_algo_usr_func_init_t)(struct target *target, struct esp32_algo_run_data *run, void *usr_arg);

/**
 * @brief Algorithm's arguments cleanup function.
 *        This function will be called just after stub exit.
 *        It can be used to cleanup stub memory parameters.
 *
 * @param target  Pointer to target.
 * @param run     Pointer to algo run data.
 * @param usr_arg Function specific argument. The same as for esp32_algo_usr_func_t.
 *
 * @return ERROR_OK on success, otherwise ERROR_XXX.
 */
typedef void (*esp32_algo_usr_func_done_t)(struct target *target, struct esp32_algo_run_data *run, void *usr_arg);

/**
 * ESP32 algorithm run data.
 */
struct esp32_algo_run_data {
	/** Algoritm completion timeout in ms. If 0, default value will be used */
	uint32_t 					tmo;
	/** Stub stack size. */
	uint32_t 					stack_size;
	/** Stub stack size. */
	struct esp32_algo_mem_args 	mem_args;
	/** Host side algorithm function argument. */
	void *						usr_func_arg;
	/** Host side algorithm function. */
	esp32_algo_usr_func_t 		usr_func;
	/** Host side algorithm function setup routine. */
	esp32_algo_usr_func_init_t	usr_func_init;
	/** Host side algorithm function cleanup routine. */
	esp32_algo_usr_func_done_t	usr_func_done;
	/** Stub return code. */
	int32_t 					ret_code;
	/** Algorithm run function: esp32_run_algorithm_xxx. */
	esp32_algo_func_t 			algo_func;
	// user should init to zero the following fields and should not use them anyhow else.
	// only for internal uses
	struct {
		/** Stub. */
		struct esp32_stub 		stub;
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
int esp32_run_func_image(struct target *target, struct esp32_algo_run_data *run, struct esp32_algo_image *image, uint32_t num_args, ...);

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
int esp32_run_onboard_func(struct target *target, struct esp32_algo_run_data *run, uint32_t func_entry, uint32_t num_args, ...);

/**
 * @brief External stub code run function.
 *
 * @param target Pointer to target.
 * @param run    Pointer to algo run data.
 * @param image  Pointer to algo image.
 *
 * @return ERROR_OK on success, otherwise ERROR_XXX.
 */
int esp32_run_algorithm_image(struct target *target, struct esp32_algo_run_data *run, struct esp32_algo_image *image);

/**
 * @brief On-board stub code run function.
 *
 * @param target     Pointer to target.
 * @param run        Pointer to algo run data.
 * @param func_entry Address of the function to run.
 *
 * @return ERROR_OK on success, otherwise ERROR_XXX.
 */
int esp32_run_algorithm_onboard(struct target *target, struct esp32_algo_run_data *run, uint32_t func_entry);

#endif /* XTENSA_ESP32_H */
