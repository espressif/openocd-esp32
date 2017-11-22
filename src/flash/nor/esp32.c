/***************************************************************************
 *	 ESP32 flash driver for OpenOCD										   *
 *	 Copyright (C) 2017 Espressif Systems Ltd.							   *
 *   Author: Alexey Gerenkov <alexey@espressif.com>                        *
 *																		   *
 *	 This program is free software; you can redistribute it and/or modify  *
 *	 it under the terms of the GNU General Public License as published by  *
 *	 the Free Software Foundation; either version 2 of the License, or	   *
 *	 (at your option) any later version.								   *
 *																		   *
 *	 This program is distributed in the hope that it will be useful,	   *
 *	 but WITHOUT ANY WARRANTY; without even the implied warranty of		   *
 *	 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the		   *
 *	 GNU General Public License for more details.						   *
 *																		   *
 *	 You should have received a copy of the GNU General Public License	   *
 *	 along with this program; if not, write to the						   *
 *	 Free Software Foundation, Inc.,									   *
 *	 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.		   *
 ***************************************************************************/

/*
 * Overview
 * --------
 * Like many other flash drivers this one uses special binary program (stub) running on target
 * to perform all operations and communicate to the host. Stub has entry function which accepts
 * variable number of arguments and therefore can handle different flash operation requests.
 * Only the first argument of the stub entry function is mandatory for all operations it must
 * specify the type of flash function to perform (read, write etc.). Actually stub main function
 * is a dispatcher which determines the type of flash operation to perform, retrieves other
 * arguments and calls corresponding handler. In C notation entry function looks like the following:

 * int stub_main(int cmd, ...);

 * In general every flash operation consists of the following steps:
 * 1) Stub is loaded to target.
 * 2) Necessary arguments are prepared and stub's main function is called.
 * 3) Stub does the work and returns the result.

 * Stub Loading
 * ------------
 * To run stub its code and data sections must be loaded to the target. It is done using working area API.
 * But since code and data address spaces are separated in ESP32 it is necessary to have two configured
 * working areas: one in code address space and another one in data space. So driver allocates chunks
 * in respective pools and writes stub sections to them. It is important that the both stub sections reside
 * at the beginning of respective working areas because stub code is linked as ELF and therefore it is
 * position dependent. So target memory for stub code and data must be allocated first.

 * Stub Execution
 * --------------
 * Special wrapping code is used to enter and exit the stub's main function. It prepares register arguments
 * before Windowed ABI call to stub entry and upon return from it executes break command to indicate to OpenOCD
 * that operation is finished.

 * Flash Data Transfers
 * --------------------
 * To transfer data from/to target a buffer should be allocated at ESP32 side. Also during the data transfer
 * target and host must maintain the state of that buffer (read/write pointers etc.). So host needs to check
 * the state of that buffer periodically and write to or read from it (depending on flash operation type).
 * ESP32 does not support access to its memory via JTAG when it is not halted, so accessing target memory would
 * requires halting the CPUs every time the host needs to check if there are incoming data or free space available
 * in the buffer. This fact can slow down flash write/read operations dramatically. To avoid this flash driver and
 * stub use application level tracing module API to transfer the data in 'non-stop' mode.
 *
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "imp.h"
#include <helper/binarybuffer.h>
#include <target/algorithm.h>
#include <target/image.h>
#include <target/register.h>
#include <target/esp108.h>
#include <target/esp108_apptrace.h>
#include <target/esp32.h>
#include "time_support.h"
#include <contrib/loaders/flash/esp32/stub_flasher.h>

#define ESP32_STUB_DEBUG			0

#if ESP32_STUB_DEBUG
#define ESP32_STUB_STACK_STAMP		0xCE
#define ESP32_STUB_STACK_DEBUG		32
#define ESP32_STUB_PATH				"contrib/loaders/flash/esp32/build/stub_flasher.elf"
#else
#define ESP32_STUB_STACK_DEBUG		0
#endif

#define ESP32_STUB_STACK_SZ			(1536)
#define ESP32_FLASH_MIN_OFFSET 		0x1000 // protect secure boot digest data
#define ESP32_ALGORITHM_EXIT_TMO	10000 // ms
#define ESP32_TARGET_STATE_TMO		1000 // ms
#define ESP32_RW_TMO				3000 // ms

#define SPI_FLASH_SEC_SIZE  		4096 // SPI Flash sector size

#define STUB_ARGS_MAX         		10 // total args max num
#define STUB_ARGS_FUNC_START  		5  // flash operation specific args start idx

#define ELF_PHF_EXEC				0x1

#include "contrib/loaders/flash/esp32/stub_flasher_image.h"

struct esp32_flash_bank {
	int 		probed;
};

struct esp32_flash_stub {
	uint32_t entry;
	struct working_area *code;
	struct working_area *data;
	struct working_area *algo;
	struct working_area *stack;
	struct xtensa_algorithm ainfo;
	struct reg_param reg_params[STUB_ARGS_MAX];
	uint32_t reg_params_count;
	struct mem_param *mem_params;
	uint32_t mem_params_count;
};

struct esp32_rw_args {
	struct target *	master_target;
	int 			(*xfer)(struct target *target, uint32_t block_id, uint32_t len, void *priv);
	uint8_t *		buffer;
	uint32_t 		count;
	uint32_t 		total_count;
};

struct esp32_write_state {
	struct esp32_rw_args rw;
	uint32_t prev_block_id;
	struct working_area *target_buf;
};

struct esp32_read_state {
	struct esp32_rw_args rw;
	uint8_t *rd_buf;
};

struct esp32_erase_check_args {
	struct working_area *erased_state_buf;
	uint32_t num_sectors;
};

static void esp32_algo_init_args(struct esp32_flash_stub *stub)
{
	uint32_t stack_addr = stub->stack->address + ESP32_STUB_STACK_SZ;
	if (stack_addr % 16) {
		LOG_DEBUG("Adjust stack addr 0x%x", stack_addr);
		stack_addr &= ~0xFUL;
	}
	buf_set_u32(stub->reg_params[0].value, 0, 32, stub->entry);
	buf_set_u32(stub->reg_params[1].value, 0, 32, stack_addr);
	buf_set_u32(stub->reg_params[2].value, 0, 32, 0x0); // initial window base
	buf_set_u32(stub->reg_params[3].value, 0, 32, 0x1); // initial window start
	buf_set_u32(stub->reg_params[4].value, 0, 32, 0x60021); // enable WOE, UM and debug interrupts level
}

static int esp32_stub_load(struct target *target, struct esp32_flash_stub *stub)
{
	int retval;
	static const uint8_t esp32_stub_wrapper[] = {
		#include "contrib/loaders/flash/esp32/stub_flasher_wrapper.inc"
	};
#if ESP32_STUB_DEBUG == 1
	struct image image;
	uint8_t buf[512];

	image.base_address_set = 1;
	image.base_address = 0;
	image.start_address_set = 0;
	retval = image_open(&image, ESP32_STUB_PATH, NULL);
	if (retval != ERROR_OK) {
		LOG_ERROR("Failed to open stub image (%d)!", retval);
		return retval;
	}

	LOG_DEBUG("stub: base 0x%x, start 0x%x, %d sections", (unsigned)image.base_address, image.start_address, image.num_sections);
	stub->entry = image.start_address;
	for (int i = 0; i < image.num_sections; i++) {
		struct imagesection *section = &image.sections[i];
		LOG_DEBUG("addr %x, sz %d, flags %x", section->base_address, section->size, section->flags);
		if (section->flags & ELF_PHF_EXEC) {
			if (target_alloc_working_area(target, section->size, &stub->code) != ERROR_OK) {
				LOG_ERROR("no working area available, can't alloc space for stub code!");
				image_close(&image);
				retval = ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
				goto _on_error;
			}
			// sanity check, stub is compiled to be run from working area
			if (stub->code->address != section->base_address) {
				LOG_ERROR("working area 0x%x and stub code section 0x%x address mismatch!", section->base_address, stub->code->address);
				image_close(&image);
				retval = ERROR_FAIL;
				goto _on_error;
			}
		} else {
			if (target_alloc_alt_working_area(target, section->size + ESP32_STUB_BSS_SIZE, &stub->data) != ERROR_OK) {
				LOG_ERROR("no working area available, can't alloc space for stub data!");
				image_close(&image);
				retval = ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
				goto _on_error;
			}
			// sanity check, stub is compiled to be run from working area
			if (stub->data->address != section->base_address) {
				LOG_ERROR("working area 0x%x and stub data section 0x%x address mismatch!", section->base_address, stub->data->address);
				image_close(&image);
				retval = ERROR_FAIL;
				goto _on_error;
			}
		}
		uint32_t sec_wr = 0;
		while (sec_wr < section->size) {
			uint32_t nb = section->size - sec_wr > sizeof(buf) ? sizeof(buf) : section->size - sec_wr;
			size_t size_read = 0;
			retval = image_read_section(&image, i, sec_wr, nb, buf, &size_read);
			if (retval != ERROR_OK) {
				LOG_ERROR("Failed to read stub section (%d)!", retval);
				image_close(&image);
				goto _on_error;
			}
			retval = target_write_buffer(target, section->base_address + sec_wr, size_read, buf);
			if (retval != ERROR_OK) {
				LOG_ERROR("Failed to write stub section!");
				image_close(&image);
				goto _on_error;
			}
			sec_wr += size_read;
		}
	}
	image_close(&image);
#else
	static const uint8_t esp32_stub_code[] = {
		#include "contrib/loaders/flash/esp32/stub_flasher_code.inc"
	};
	static const uint8_t esp32_stub_data[] = {
		#include "contrib/loaders/flash/esp32/stub_flasher_data.inc"
	};
	stub->entry = ESP32_STUB_ENTRY_ADDR;
	if (target_alloc_working_area(target, sizeof(esp32_stub_code), &stub->code) != ERROR_OK) {
		LOG_ERROR("no working area available, can't alloc space for stub code!");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}
	retval = target_write_buffer(target, stub->code->address, sizeof(esp32_stub_code), esp32_stub_code);
	if (retval != ERROR_OK) {
		LOG_ERROR("Failed to write stub code section!");
		target_free_working_area(target, stub->code);
		return retval;
	}
	if (target_alloc_alt_working_area(target, sizeof(esp32_stub_data) + ESP32_STUB_BSS_SIZE, &stub->data) != ERROR_OK) {
		LOG_ERROR("no working area available, can't alloc space for stub data!");
		target_free_working_area(target, stub->code);
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}
	retval = target_write_buffer(target, stub->data->address, sizeof(esp32_stub_data), esp32_stub_data);
	if (retval != ERROR_OK) {
		LOG_ERROR("Failed to write stub data section!");
		target_free_working_area(target, stub->code);
		target_free_alt_working_area(target, stub->data);
		return retval;
	}
#endif
	if (target_alloc_alt_working_area(target, ESP32_STUB_STACK_SZ, &stub->stack) != ERROR_OK) {
		LOG_ERROR("no working area available, can't alloc stub stack!");
		retval = ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		goto _on_error;
	}

	if (target_alloc_working_area(target, sizeof(esp32_stub_wrapper), &stub->algo) != ERROR_OK) {
		LOG_ERROR("no working area available, can't alloc space for stub jumper!");
		retval = ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		goto _on_error;
	}

	retval = target_write_buffer(target, stub->algo->address, sizeof(esp32_stub_wrapper), esp32_stub_wrapper);
	if (retval != ERROR_OK) {
		LOG_ERROR("Failed to write stub jumper!");
		goto _on_error;
	}

	init_reg_param(&stub->reg_params[0], "a0", 			32, PARAM_OUT);
	init_reg_param(&stub->reg_params[1], "a1", 			32, PARAM_OUT);
	init_reg_param(&stub->reg_params[2], "windowbase", 	32, PARAM_OUT);
	init_reg_param(&stub->reg_params[3], "windowstart", 32, PARAM_OUT);
	init_reg_param(&stub->reg_params[4], "ps", 			32, PARAM_OUT);

	return ERROR_OK;

_on_error:
	if (stub->code) {
		target_free_working_area(target, stub->code);
	}
	if (stub->algo) {
		target_free_working_area(target, stub->algo);
	}
	if (stub->data) {
		target_free_alt_working_area(target, stub->data);
	}
	if (stub->stack) {
		target_free_alt_working_area(target, stub->stack);
	}

	return retval;
}

static void esp32_stub_cleanup(struct target *target, struct esp32_flash_stub *stub)
{
	destroy_reg_param(&stub->reg_params[4]);
	destroy_reg_param(&stub->reg_params[3]);
	destroy_reg_param(&stub->reg_params[2]);
	destroy_reg_param(&stub->reg_params[1]);
	destroy_reg_param(&stub->reg_params[0]);
	target_free_working_area(target, stub->algo);
	target_free_working_area(target, stub->code);
	target_free_alt_working_area(target, stub->stack);
	if (stub->data) {
		target_free_alt_working_area(target, stub->data);
	}
}

#if ESP32_STUB_STACK_DEBUG
static int esp32_stub_fill_stack(struct target *target, uint32_t stack_addr, uint32_t sz)
{
	uint8_t buf[256];

	// fill stub stack with canary bytes
	memset(buf, ESP32_STUB_STACK_STAMP, sizeof(buf));
	for (uint32_t i = 0; i < sz;) {
		uint32_t wr_sz = ESP32_STUB_STACK_SZ - i >= sizeof(buf) ? sizeof(buf) : ESP32_STUB_STACK_SZ - i;
		int retval = target_write_buffer(target, stack_addr + i, wr_sz, buf);
		if (retval != ERROR_OK) {
			LOG_ERROR("Failed to init stub stack (%d)!", retval);
			return retval;
		}
		i += wr_sz;
	}
	return ERROR_OK;
}

static int esp32_stub_check_stack(struct target *target, uint32_t stack_addr, uint32_t sz)
{
	int retval = ERROR_OK;
	uint8_t buf[256];

	// check stub stack for overflow
	for (uint32_t i = 0; i < sz;) {
		uint32_t rd_sz = sz - i >= sizeof(buf) ? sizeof(buf) : sz - i;
		retval = target_read_buffer(target, stack_addr + i, rd_sz, buf);
		if (retval != ERROR_OK) {
			LOG_ERROR("Failed to read stub stack (%d)!", retval);
			return retval;
		}
		int checked = 0;
		LOG_DEBUG("STK[%x]:", stack_addr+i);
		uint32_t j;
		for (j = 0; j < rd_sz; j++) {
			LOG_DEBUG("STK[%x]: %x", stack_addr+(i+j), buf[j]);
			if (buf[j] != ESP32_STUB_STACK_STAMP) {
				if (i+j > 0) {
					LOG_INFO("Stub stack bytes unused %d / %d", i+j, ESP32_STUB_STACK_SZ);
				} else {
					LOG_ERROR("Stub stack OVF!!!");
					retval = ERROR_FAIL;
				}
				checked = 1;
				break;
			}
			if (((i+j) % 80) == 0) {
				LOG_DEBUG("\nSTK[%x]: %x", stack_addr-(i+j), buf[j]);
			} else {
				LOG_DEBUG(" %x", buf[j]);
			}
		}
		if (checked) {
			break;
		}
		i += rd_sz;
	}
	return retval;
}
#endif

static int esp32_run_algo(struct target *target, struct esp32_flash_stub *stub,
		int (*usr_func)(struct target *target, void *priv), void *usr_arg,
		int (*usr_arg_prep)(struct target *target, struct esp32_flash_stub *stub, void *usr_arg))
{
	int retval;
	int other_cpu_was_running = 1;
	struct target *other_cpu = NULL;
	struct duration algo_time;
	struct target *core_target;

	if (memcmp(target_type_name(target), "esp32", 6) == 0){//get_targets_count() == 1) {
		struct esp32_common *esp32 = (struct esp32_common *)target->arch_info;
		LOG_INFO("Use core%d of target '%s'", esp32->active_cpu, target_type_name(target));
		core_target = esp32->esp32_targets[esp32->active_cpu];
	} else {
		LOG_INFO("Use target '%s'", target_type_name(target));
		core_target = target;
	}

	if (duration_start(&algo_time) != 0) {
		LOG_ERROR("Failed to start algo time measurement!");
		return ERROR_FAIL;
	}
	retval = esp32_stub_load(target, stub);
	if (retval != ERROR_OK) {
		LOG_ERROR("Failed to load stub (%d)!", retval);
		return retval;
	}
	if (duration_measure(&algo_time) != 0) {
		LOG_ERROR("Failed to stop algo run measurement!");
		return ERROR_FAIL;
	}
	LOG_DEBUG("Stub loaded in %g ms", duration_elapsed(&algo_time)*1000);

	esp32_algo_init_args(stub);
	if (usr_arg_prep) {
		retval = usr_arg_prep(target, stub, usr_arg);
		if (retval != ERROR_OK) {
			LOG_ERROR("Failed to prepare algorithm host side args stub (%d)!", retval);
			esp32_stub_cleanup(target, stub);
			return retval;
		}
	}

#if ESP32_STUB_STACK_DEBUG
	retval = esp32_stub_fill_stack(target, stub->stack->address, ESP32_STUB_STACK_DEBUG);
	if (retval != ERROR_OK) {
		LOG_ERROR("Failed to init stub stack (%d)!", retval);
		esp32_stub_cleanup(target, stub);
		return retval;
	}
#endif
	LOG_DEBUG("Algorithm start @ 0x%x, stack %d bytes @ 0x%x ", stub->algo->address,
			ESP32_STUB_STACK_SZ, stub->stack->address + ESP32_STUB_STACK_SZ);
	if (get_targets_count() == 2) {
		// we need to run other CPU in single core mode
		if (memcmp(target_name(target), "esp32.cpu0", 6) == 0) {
			other_cpu = get_target("esp32.cpu1");
		} else {
			other_cpu = get_target("esp32.cpu0");
		}
		if (other_cpu->state == TARGET_HALTED) {
			retval = target_resume(other_cpu, 1, 0, 0, 0);
			if (retval != ERROR_OK) {
				LOG_ERROR("Failed to resume target '%s' (%d)!", target_state_name(other_cpu), retval);
				esp32_stub_cleanup(target, stub);
				return retval;
			}
			other_cpu_was_running = 0;
		} else if (other_cpu->state != TARGET_RUNNING) {
			LOG_ERROR("Invalid target state '%s'! Must be 'halted' or 'running'.", target_state_name(other_cpu));
			esp32_stub_cleanup(target, stub);
			return ERROR_TARGET_NOT_HALTED;
		}
	}
	retval = target_start_algorithm(target,
			stub->mem_params_count, stub->mem_params,
			stub->reg_params_count, stub->reg_params,
			stub->algo->address, 0,
			&stub->ainfo);
	if (retval != ERROR_OK) {
		LOG_ERROR("Faied to start algorithm (%d)!", retval);
		esp32_stub_cleanup(target, stub);
		return retval;
	}

	if (usr_func) {
		// give target algorithm stub time to init itself, then user func can communicate to it safely
		alive_sleep(100);
		retval = usr_func(core_target, usr_arg);
		if (retval != ERROR_OK) {
			LOG_ERROR("Failed to exec algorithm user func (%d)!", retval);
		}
	}

	LOG_DEBUG("Wait algorithm completion");
	int rc = target_wait_algorithm(target,
			stub->mem_params_count, stub->mem_params,
			stub->reg_params_count, stub->reg_params,
			0, ESP32_ALGORITHM_EXIT_TMO,
			&stub->ainfo);
	if (rc != ERROR_OK) {
		LOG_ERROR("Faied to wait algorithm (%d)!", retval);
		target_halt(target);
		target_wait_state(target, TARGET_HALTED, ESP32_TARGET_STATE_TMO);
		if (retval == ERROR_OK) {
			retval = rc;
		}
	}
	if (!other_cpu_was_running) {
		// halt resumed CPU
		rc = target_halt(other_cpu);
		if (rc != ERROR_OK) {
			LOG_ERROR("Failed to halt target '%s' (%d)!", target_state_name(other_cpu), rc);
			if (retval == ERROR_OK) {
				retval = rc;
			}
		} else {
			target_wait_state(other_cpu, TARGET_HALTED, ESP32_TARGET_STATE_TMO);
		}
	}

#if ESP32_STUB_STACK_DEBUG
	retval = esp32_stub_check_stack(target, stub->stack->address, ESP32_STUB_STACK_DEBUG);
	if (retval != ERROR_OK) {
		LOG_ERROR("Failed to check stub stack (%d)!", retval);
	}
#endif
	esp32_stub_cleanup(target, stub);

	return retval;
}

/* flash bank <bank_name> esp32 <base> <size> 0 0 <target#>
   If <size> is zero flash size will be autodetected, otherwise user value will be used
 */
FLASH_BANK_COMMAND_HANDLER(esp32_flash_bank_command)
{
	struct esp32_flash_bank *esp32_info;

	if (CMD_ARGC < 6)
		return ERROR_COMMAND_SYNTAX_ERROR;

	esp32_info = malloc(sizeof(struct esp32_flash_bank));
	bank->driver_priv = esp32_info;

	esp32_info->probed = 0;

	return ERROR_OK;
}

static int esp32_protect(struct flash_bank *bank, int set, int first, int last)
{
	return ERROR_FAIL;
}

static int esp32_protect_check(struct flash_bank *bank)
{
	return ERROR_OK;
}

static int esp32_erase_check_args_prepare(struct target *target, struct esp32_flash_stub *stub, void *usr_arg)
{
	struct esp32_erase_check_args *args = (struct esp32_erase_check_args *)usr_arg;

	int retval = target_alloc_alt_working_area(target, args->num_sectors, &args->erased_state_buf);
	if (retval != ERROR_OK) {
		LOG_ERROR("Failed to alloc target buffer!");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	stub->mem_params = malloc(sizeof(struct mem_param));
	if (!stub->mem_params) {
		LOG_ERROR("Failed to alloc mem params!");
		target_free_alt_working_area(target, args->erased_state_buf);
		return ERROR_FAIL;
	}
	stub->mem_params_count = 1;
	init_mem_param(&stub->mem_params[0], args->erased_state_buf->address, args->num_sectors, PARAM_IN);

	buf_set_u32(stub->reg_params[STUB_ARGS_FUNC_START+2].value, 0, 32, args->num_sectors); // number of sectors
	buf_set_u32(stub->reg_params[STUB_ARGS_FUNC_START+3].value, 0, 32, args->erased_state_buf->address); // address to store sectors' state

	return ERROR_OK;
}

static void esp32_erase_check_args_cleanup(struct target *target, struct esp32_flash_stub *stub, struct esp32_erase_check_args *args)
{
	destroy_mem_param(&stub->mem_params[0]);
	free(stub->mem_params);
	target_free_alt_working_area(target, args->erased_state_buf);
}

static int esp32_blank_check(struct flash_bank *bank)
{
	int retval;
	struct esp32_flash_stub stub;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted!");
		return ERROR_TARGET_NOT_HALTED;
	}

	memset(&stub, 0, sizeof(struct esp32_flash_stub));
	init_reg_param(&stub.reg_params[STUB_ARGS_FUNC_START+0], "a2", 32, PARAM_IN_OUT);
	init_reg_param(&stub.reg_params[STUB_ARGS_FUNC_START+1], "a3", 32, PARAM_OUT);
	init_reg_param(&stub.reg_params[STUB_ARGS_FUNC_START+2], "a4", 32, PARAM_OUT);
	init_reg_param(&stub.reg_params[STUB_ARGS_FUNC_START+3], "a5", 32, PARAM_OUT);

	buf_set_u32(stub.reg_params[STUB_ARGS_FUNC_START+0].value, 0, 32, ESP32_STUB_CMD_FLASH_ERASE_CHECK);
	buf_set_u32(stub.reg_params[STUB_ARGS_FUNC_START+1].value, 0, 32, 0); // start sector

	stub.ainfo.core_mode = XT_MODE_RING0;
	stub.reg_params_count = STUB_ARGS_FUNC_START+4;

	struct esp32_erase_check_args args;
	args.num_sectors = bank->num_sectors;
	retval = esp32_run_algo(bank->target, &stub, NULL, &args, esp32_erase_check_args_prepare);
	if (retval != ERROR_OK) {
		LOG_ERROR("Algorithm run faied (%d)!", retval);
	} else {
		LOG_DEBUG("Check algorithm RC");
		int flasher_rc = buf_get_u32(stub.reg_params[STUB_ARGS_FUNC_START+0].value, 0, 32);
		if (flasher_rc != ESP32_STUB_ERR_OK) {
			LOG_ERROR("Failed to check erase flash (%d)!", flasher_rc);
			retval = ERROR_FAIL;
		} else {
			for (int i = 0; i < bank->num_sectors; i++) {
				bank->sectors[i].is_erased = stub.mem_params[0].value[i];
			}
		}
	}

	esp32_erase_check_args_cleanup(bank->target, &stub, &args);
	destroy_reg_param(&stub.reg_params[STUB_ARGS_FUNC_START+0]);
	destroy_reg_param(&stub.reg_params[STUB_ARGS_FUNC_START+1]);
	destroy_reg_param(&stub.reg_params[STUB_ARGS_FUNC_START+2]);
	destroy_reg_param(&stub.reg_params[STUB_ARGS_FUNC_START+3]);

	return ERROR_OK;
}

static uint32_t esp32_get_size(struct flash_bank *bank)
{
	int retval = ERROR_OK;
	uint32_t size = 0;
	struct esp32_flash_stub stub;

	memset(&stub, 0, sizeof(struct esp32_flash_stub));
	init_reg_param(&stub.reg_params[STUB_ARGS_FUNC_START+0], "a2", 32, PARAM_IN_OUT);

	buf_set_u32(stub.reg_params[STUB_ARGS_FUNC_START+0].value, 0, 32, ESP32_STUB_CMD_FLASH_SIZE);

	stub.ainfo.core_mode = XT_MODE_RING0;
	stub.reg_params_count = STUB_ARGS_FUNC_START+1;

	retval = esp32_run_algo(bank->target, &stub, NULL, NULL, NULL);
	if (retval != ERROR_OK) {
		LOG_ERROR("Algorithm run faied (%d)!", retval);
	} else {
		LOG_DEBUG("Check algorithm RC");
		size = buf_get_u32(stub.reg_params[STUB_ARGS_FUNC_START+0].value, 0, 32);
		if (size == 0) {
			LOG_ERROR("Failed to get flash size!");
		}
	}

	destroy_reg_param(&stub.reg_params[STUB_ARGS_FUNC_START+0]);

	return  size;
}

static int esp32_erase(struct flash_bank *bank, int first, int last)
{
	int retval = ERROR_OK;
	struct esp32_flash_stub stub;

	if (first*SPI_FLASH_SEC_SIZE < ESP32_FLASH_MIN_OFFSET) {
		LOG_ERROR("Invalid offset!");
		return ERROR_FAIL;
	}

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}
	assert((0 <= first) && (first <= last) && (last < bank->num_sectors));

	memset(&stub, 0, sizeof(struct esp32_flash_stub));
	init_reg_param(&stub.reg_params[STUB_ARGS_FUNC_START+0], "a2", 32, PARAM_IN_OUT);
	init_reg_param(&stub.reg_params[STUB_ARGS_FUNC_START+1], "a3", 32, PARAM_OUT);
	init_reg_param(&stub.reg_params[STUB_ARGS_FUNC_START+2], "a4", 32, PARAM_OUT);

	buf_set_u32(stub.reg_params[STUB_ARGS_FUNC_START+0].value, 0, 32, ESP32_STUB_CMD_FLASH_ERASE);
	buf_set_u32(stub.reg_params[STUB_ARGS_FUNC_START+1].value, 0, 32, first*SPI_FLASH_SEC_SIZE); // flash addr
	buf_set_u32(stub.reg_params[STUB_ARGS_FUNC_START+2].value, 0, 32, (last-first+1)*SPI_FLASH_SEC_SIZE); // size

	stub.ainfo.core_mode = XT_MODE_RING0;
	stub.reg_params_count = STUB_ARGS_FUNC_START+3;

	retval = esp32_run_algo(bank->target, &stub, NULL, NULL, NULL);
	if (retval != ERROR_OK) {
		LOG_ERROR("Algorithm run faied (%d)!", retval);
	} else {
		LOG_DEBUG("Check algorithm RC");
		int flasher_rc = buf_get_u32(stub.reg_params[STUB_ARGS_FUNC_START+0].value, 0, 32);
		if (flasher_rc != ESP32_STUB_ERR_OK) {
			LOG_ERROR("Failed to erase flash (%d)!", flasher_rc);
			retval = ERROR_FAIL;
		}
	}

	destroy_reg_param(&stub.reg_params[STUB_ARGS_FUNC_START+0]);
	destroy_reg_param(&stub.reg_params[STUB_ARGS_FUNC_START+1]);
	destroy_reg_param(&stub.reg_params[STUB_ARGS_FUNC_START+2]);

	return  retval;
}

static int esp32_rw_do(struct target *target, void *priv)
{
	struct duration algo_time;
	struct esp32_rw_args *rw = (struct esp32_rw_args *)priv;
	int retval = ERROR_OK;

	if (duration_start(&algo_time) != 0) {
		LOG_ERROR("Failed to start data write time measurement!");
		return ERROR_FAIL;
	}
	while (rw->total_count < rw->count) {
		uint32_t block_id = 0, len = 0;
		LOG_DEBUG("Transfer block on %p", target);
		retval = esp108_apptrace_read_data_len(target, &block_id, &len);
		if (retval != ERROR_OK) {
			LOG_ERROR("Failed to read apptrace status (%d)!", retval);
			return retval;
		}
		// transfer block
		LOG_DEBUG("Transfer block %d, %d bytes", block_id, len);
		retval = rw->xfer(target, block_id, len, rw);
		if (retval == ERROR_WAIT) {
			LOG_DEBUG("Block not ready");
			// if no transfer check tmo
			if (duration_measure(&algo_time) != 0) {
				LOG_ERROR("Failed to stop algo run measurement!");
				return ERROR_FAIL;
			}
			if (1000*duration_elapsed(&algo_time) > ESP32_RW_TMO) {
				LOG_ERROR("Transfer data tmo!");
				return ERROR_WAIT;
			}
		} else if (retval != ERROR_OK) {
			LOG_ERROR("Failed to transfer flash data block (%d)!", retval);
			return retval;
		}
		alive_sleep(10);
		if (rw->master_target->state != TARGET_DEBUG_RUNNING) {
			LOG_ERROR("Algorithm accidentally stopped (%d)!", rw->master_target->state);
			return ERROR_FAIL;
		}
	}
	if (duration_measure(&algo_time) != 0) {
		LOG_ERROR("Failed to stop data write measurement!");
		return ERROR_FAIL;
	}
	LOG_DEBUG("PROF: Data transffered in %g ms @ %g KB/s", duration_elapsed(&algo_time)*1000, duration_kbps(&algo_time, rw->total_count));

	return ERROR_OK;
}

static int esp32_write_xfer(struct target *target, uint32_t block_id, uint32_t len, void *priv)
{
	struct esp32_write_state *state = (struct esp32_write_state *)priv;

	if (state->prev_block_id == block_id) {
		return ERROR_WAIT;
	}

	uint32_t wr_sz = state->rw.count - state->rw.total_count < ESP32_USR_BLOCK_SZ_MAX ? state->rw.count - state->rw.total_count : ESP32_USR_BLOCK_SZ_MAX;
	int retval = esp108_apptrace_usr_block_write(target, block_id, state->rw.buffer + state->rw.total_count, wr_sz);
	if (retval != ERROR_OK) {
		LOG_ERROR("Failed to write apptrace data (%d)!", retval);
		return retval;
	}
	state->rw.total_count += wr_sz;
	state->prev_block_id = block_id;

	return ERROR_OK;
}

static int esp32_write_state_prepare(struct target *target, struct esp32_flash_stub *stub, void *usr_arg)
{
	struct duration algo_time;
	struct esp32_write_state *state = (struct esp32_write_state *)usr_arg;

	/* memory buffer */
	if (duration_start(&algo_time) != 0) {
		LOG_ERROR("Failed to start workarea alloc time measurement!");
		return ERROR_FAIL;
	}
	uint32_t buffer_size = 64*1024;
	while (target_alloc_alt_working_area_try(target, buffer_size, &state->target_buf) != ERROR_OK) {
		buffer_size /= 2;
		if (buffer_size == 0) {
			LOG_ERROR("Failed to alloc target buffer for flash data!");
			return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		}
	}
	if (duration_measure(&algo_time) != 0) {
		LOG_ERROR("Failed to stop workarea alloc measurement!");
		return ERROR_FAIL;
	}
	LOG_DEBUG("PROF: Allocated target buffer %d bytes in %g ms", buffer_size, duration_elapsed(&algo_time)*1000);

	buf_set_u32(stub->reg_params[STUB_ARGS_FUNC_START+3].value, 0, 32, state->target_buf->address); // down buffer
	buf_set_u32(stub->reg_params[STUB_ARGS_FUNC_START+4].value, 0, 32, state->target_buf->size); // down buffer size

	return ERROR_OK;
}

static void esp32_write_state_cleanup(struct target *target, struct esp32_write_state *state)
{
	struct duration algo_time;

	if (!state->target_buf) {
		return;
	}
	if (duration_start(&algo_time) != 0) {
		LOG_ERROR("Failed to start workarea alloc time measurement!");
	}
	target_free_alt_working_area(target, state->target_buf);
	if (duration_measure(&algo_time) != 0) {
		LOG_ERROR("Failed to stop data write measurement!");
	}
	LOG_DEBUG("PROF: Workarea freed in %g ms", duration_elapsed(&algo_time)*1000);
}

static int esp32_write(struct flash_bank *bank, const uint8_t *buffer,
		uint32_t offset, uint32_t count)
{
	int retval = ERROR_OK;
	struct esp32_flash_stub stub;

	if (offset < ESP32_FLASH_MIN_OFFSET) {
		LOG_ERROR("Invalid offset!");
		return ERROR_FAIL;
	}

	if (offset & 0x3UL) {
		LOG_ERROR("Unaligned offset!");
		return ERROR_FAIL;
	}

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	memset(&stub, 0, sizeof(struct esp32_flash_stub));
	init_reg_param(&stub.reg_params[STUB_ARGS_FUNC_START+0], "a2", 32, PARAM_IN_OUT);
	init_reg_param(&stub.reg_params[STUB_ARGS_FUNC_START+1], "a3", 32, PARAM_OUT);
	init_reg_param(&stub.reg_params[STUB_ARGS_FUNC_START+2], "a4", 32, PARAM_OUT);
	init_reg_param(&stub.reg_params[STUB_ARGS_FUNC_START+3], "a5", 32, PARAM_OUT);
	init_reg_param(&stub.reg_params[STUB_ARGS_FUNC_START+4], "a6", 32, PARAM_OUT);

	buf_set_u32(stub.reg_params[STUB_ARGS_FUNC_START+0].value, 0, 32, ESP32_STUB_CMD_FLASH_WRITE);
	buf_set_u32(stub.reg_params[STUB_ARGS_FUNC_START+1].value, 0, 32, offset); // flash addr
	buf_set_u32(stub.reg_params[STUB_ARGS_FUNC_START+2].value, 0, 32, count); // size

	stub.ainfo.core_mode = XT_MODE_RING0;
	stub.reg_params_count = STUB_ARGS_FUNC_START+5;

	struct esp32_write_state wr_state;
	memset(&wr_state, 0, sizeof(struct esp32_write_state));
	wr_state.rw.buffer = (uint8_t *)buffer;
	wr_state.rw.count = count;
	wr_state.rw.xfer = esp32_write_xfer;
	wr_state.rw.master_target = bank->target;
	wr_state.prev_block_id = (uint32_t)-1;
	retval = esp32_run_algo(bank->target, &stub, esp32_rw_do, &wr_state, esp32_write_state_prepare);
	if (retval != ERROR_OK) {
		LOG_ERROR("Algorithm run faied (%d)!", retval);
	} else {
		LOG_DEBUG("Check algorithm RC");
		int flasher_rc = buf_get_u32(stub.reg_params[STUB_ARGS_FUNC_START+0].value, 0, 32);
		if (flasher_rc != ESP32_STUB_ERR_OK) {
			LOG_ERROR("Failed to write flash (%d)!", flasher_rc);
			retval = ERROR_FAIL;
		}
	}

	esp32_write_state_cleanup(bank->target, &wr_state);
	destroy_reg_param(&stub.reg_params[STUB_ARGS_FUNC_START+0]);
	destroy_reg_param(&stub.reg_params[STUB_ARGS_FUNC_START+1]);
	destroy_reg_param(&stub.reg_params[STUB_ARGS_FUNC_START+2]);
	destroy_reg_param(&stub.reg_params[STUB_ARGS_FUNC_START+3]);
	destroy_reg_param(&stub.reg_params[STUB_ARGS_FUNC_START+4]);

	return  retval;
}

static int esp32_read_xfer(struct target *target, uint32_t block_id, uint32_t len, void *priv)
{
	struct esp32_read_state *state = (struct esp32_read_state *)priv;

	if (len == 0) {
		return ERROR_WAIT;
	}

	int retval = esp108_apptrace_read_data(target, len, state->rd_buf, block_id, 1/*ack*/, NULL);
	if (retval != ERROR_OK) {
		LOG_ERROR("Failed to read apptrace status (%d)!", retval);
		return retval;
	}
	LOG_DEBUG("DATA (%d/%d): %x %x %x %x %x %x %x %x", len, ESP32_TRACEMEM_BLOCK_SZ,
		state->rd_buf[0], state->rd_buf[1], state->rd_buf[2], state->rd_buf[3],
		state->rd_buf[4], state->rd_buf[5], state->rd_buf[6], state->rd_buf[7]);

	uint8_t *ptr = state->rd_buf;
	while (ptr < state->rd_buf + len) {
		uint32_t data_sz = 0;
		ptr = esp108_apptrace_usr_block_get(ptr, &data_sz);
		if (data_sz > 0) {
			memcpy(state->rw.buffer + state->rw.total_count, ptr, data_sz);
		}
		ptr += data_sz;
		state->rw.total_count += data_sz;
	}

	return ERROR_OK;
}

static int esp32_read(struct flash_bank *bank, uint8_t *buffer,
		uint32_t offset, uint32_t count)
{
	int retval = ERROR_OK;
	struct esp32_flash_stub stub;

	if (offset & 0x3UL) {
		LOG_ERROR("Unaligned offset!");
		return ERROR_FAIL;
	}

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	struct esp32_read_state rd_state;
	memset(&rd_state, 0, sizeof(struct esp32_read_state));
	rd_state.rw.buffer = buffer;
	rd_state.rw.count = count;
	rd_state.rw.xfer = esp32_read_xfer;
	rd_state.rw.master_target = bank->target;
	rd_state.rd_buf = malloc(ESP32_TRACEMEM_BLOCK_SZ);
	if (!rd_state.rd_buf) {
		LOG_ERROR("Failed to alloc read buffer!");
		return ERROR_FAIL;
	}

	memset(&stub, 0, sizeof(struct esp32_flash_stub));
	init_reg_param(&stub.reg_params[STUB_ARGS_FUNC_START+0], "a2", 32, PARAM_IN_OUT);
	init_reg_param(&stub.reg_params[STUB_ARGS_FUNC_START+1], "a3", 32, PARAM_OUT);
	init_reg_param(&stub.reg_params[STUB_ARGS_FUNC_START+2], "a4", 32, PARAM_OUT);
	init_reg_param(&stub.reg_params[STUB_ARGS_FUNC_START+3], "a5", 32, PARAM_OUT);

	buf_set_u32(stub.reg_params[STUB_ARGS_FUNC_START+0].value, 0, 32, ESP32_STUB_CMD_FLASH_READ);
	buf_set_u32(stub.reg_params[STUB_ARGS_FUNC_START+1].value, 0, 32, offset); // flash addr
	buf_set_u32(stub.reg_params[STUB_ARGS_FUNC_START+2].value, 0, 32, count); // size
	buf_set_u32(stub.reg_params[STUB_ARGS_FUNC_START+3].value, 0, 32, 0); // buffer

	stub.ainfo.core_mode = XT_MODE_RING0;
	stub.reg_params_count = STUB_ARGS_FUNC_START+4;

	retval = esp32_run_algo(bank->target, &stub, esp32_rw_do, &rd_state, NULL);
	if (retval != ERROR_OK) {
		LOG_ERROR("Algorithm run faied (%d)!", retval);
	} else {
		LOG_DEBUG("Check algorithm RC");
		int flasher_rc = buf_get_u32(stub.reg_params[STUB_ARGS_FUNC_START+0].value, 0, 32);
		if (flasher_rc != ESP32_STUB_ERR_OK) {
			LOG_ERROR("Failed to read flash (%d)!", flasher_rc);
			retval = ERROR_FAIL;
		}
	}

	destroy_reg_param(&stub.reg_params[STUB_ARGS_FUNC_START+0]);
	destroy_reg_param(&stub.reg_params[STUB_ARGS_FUNC_START+1]);
	destroy_reg_param(&stub.reg_params[STUB_ARGS_FUNC_START+2]);
	destroy_reg_param(&stub.reg_params[STUB_ARGS_FUNC_START+3]);
	free(rd_state.rd_buf);

	return  retval;
}

static int esp32_probe(struct flash_bank *bank)
{
	struct esp32_flash_bank *esp32_info = bank->driver_priv;

	esp32_info->probed = 0;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	LOG_DEBUG("Flash size = %d KB @ 0x%x '%s' - '%s'", bank->size/1024, bank->base,
		target_name(bank->target), target_state_name(bank->target));

	if (bank->sectors) {
		free(bank->sectors);
		bank->sectors = NULL;
	}

	if (bank->size == 0 /*autodetect*/) {
		bank->size = esp32_get_size(bank);
		LOG_INFO("Auto-detected flash size %d KB", bank->size/1024);
	}
	if (bank->size > 0) {
		LOG_INFO("Using flash size %d KB", bank->size/1024);
		bank->num_sectors = bank->size / SPI_FLASH_SEC_SIZE;
		bank->sectors = malloc(sizeof(struct flash_sector) * bank->num_sectors);
		for (int i = 0; i < bank->num_sectors; i++) {
			bank->sectors[i].offset = i*SPI_FLASH_SEC_SIZE;
			bank->sectors[i].size = SPI_FLASH_SEC_SIZE;
			bank->sectors[i].is_erased = -1;
			bank->sectors[i].is_protected = 0;
		}
		LOG_DEBUG("allocated %d sectors", bank->num_sectors);
	} else {
		LOG_ERROR("Failed to probe flash, size %d KB", bank->size/1024);
	}
	esp32_info->probed = 1;

	return ERROR_OK;
}

static int esp32_auto_probe(struct flash_bank *bank)
{
	struct esp32_flash_bank *esp32_info = bank->driver_priv;
	if (esp32_info->probed)
		return ERROR_OK;
	return esp32_probe(bank);
}

static int get_esp32_info(struct flash_bank *bank, char *buf, int buf_size)
{
	snprintf(buf, buf_size, "ESP32");
	return ERROR_OK;
}

static const struct command_registration esp32_exec_command_handlers[] = {
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration esp32_command_handlers[] = {
	{
		.name = "esp32",
		.mode = COMMAND_ANY,
		.help = "esp32 flash command group",
		.usage = "",
		.chain = esp32_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

struct flash_driver esp32_flash = {
	.name = "esp32",
	.commands = esp32_command_handlers,
	.flash_bank_command = esp32_flash_bank_command,
	.erase = esp32_erase,
	.protect = esp32_protect,
	.write = esp32_write,
	.read = esp32_read,
	.probe = esp32_probe,
	.auto_probe = esp32_auto_probe,
	.erase_check = esp32_blank_check,
	.protect_check = esp32_protect_check,
	.info = get_esp32_info,
};
