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

#include "algorithm.h"
#include "xtensa.h"

/** Index of the first user-defined algo arg. @see algorithm_stub */
#define XTENSA_STUB_ARGS_FUNC_START             6

/**
 * Xtensa algorithm data.
 */
struct xtensa_algorithm {
	/** User can set this to specify which core mode algorithm should be run in. */
	enum xtensa_mode core_mode;
	/** Used internally to backup and restore debug_reason. */
	enum target_debug_reason ctx_debug_reason;
};

extern const struct algorithm_hw xtensa_algo_hw;

#endif	/* XTENSA_ESP32_H */
