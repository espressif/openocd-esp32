/***************************************************************************
 *                                                                         *
 *   Copyright (C) 2019 by Alexey Gerenkov                                 *
 *   alexey@espressif.com                                                  *
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

#include "xtensa.h"
#include "xtensa_algorithm.h"
#include "time_support.h"

#define XTENSA_STUB_DEBUG            0
#define XTENSA_ALGORITHM_EXIT_TMO    40000	/* ms */

#if XTENSA_STUB_DEBUG
#define XTENSA_STUB_STACK_STAMP      0xCE
#define XTENSA_STUB_STACK_DEBUG      128
#else
#define XTENSA_STUB_STACK_DEBUG      0
#endif

struct xtensa_stub_tramp {
	uint32_t size;
	const uint8_t *code;
};

static void xtensa_stub_tramp_get(struct xtensa *xtensa, struct xtensa_stub_tramp *tramp)
{
	static const uint8_t xtensa_stub_tramp_win[] = {
	#include "src/target/xtensa_stub_tramp_win.inc"
	};

	tramp->size = 0;

	if (!xtensa->core_config->windowed) {
		LOG_ERROR(
			"Running stubs is not supported for cores without windowed registers option!");
		return;
	} else {
		tramp->size = sizeof(xtensa_stub_tramp_win);
		tramp->code = xtensa_stub_tramp_win;
	}
}

static void xtensa_algo_args_init(struct xtensa_stub *stub, uint32_t stack_addr)
{
	LOG_DEBUG("Check stack addr 0x%x", stack_addr);
	if (stack_addr & 0xFUL) {
		LOG_DEBUG("Adjust stack addr 0x%x", stack_addr);
		stack_addr &= ~0xFUL;
	}
	stack_addr -= 16;
	init_reg_param(&stub->reg_params[0], "a0",          32, PARAM_OUT);	/*TODO: move to
										 * tramp */
	init_reg_param(&stub->reg_params[1], "a1",          32, PARAM_OUT);
	init_reg_param(&stub->reg_params[2], "a8",          32, PARAM_OUT);
	init_reg_param(&stub->reg_params[3], "windowbase",  32, PARAM_OUT);	/*TODO: move to
										 * tramp */
	init_reg_param(&stub->reg_params[4], "windowstart", 32, PARAM_OUT);	/*TODO: move to
										 * tramp */
	init_reg_param(&stub->reg_params[5], "ps",          32, PARAM_OUT);
	buf_set_u32(stub->reg_params[0].value, 0, 32, 0);	/* a0 TODO: move to tramp */
	buf_set_u32(stub->reg_params[1].value, 0, 32, stack_addr);	/* a1 */
	buf_set_u32(stub->reg_params[2].value, 0, 32, stub->entry);	/* a8 */
	buf_set_u32(stub->reg_params[3].value, 0, 32, 0x0);	/* initial window base TODO: move to
								 * tramp */
	buf_set_u32(stub->reg_params[4].value, 0, 32, 0x1);	/* initial window start TODO: move
								 * to tramp */
	buf_set_u32(stub->reg_params[5].value, 0, 32, 0x60025);	/* enable WOE, UM and debug
								 * interrupts level (6) */
}

static int xtensa_stub_load(struct target *target,
	struct xtensa_algo_image *algo_image,
	struct xtensa_stub *stub,
	uint32_t stack_size)
{
	struct xtensa *xtensa = target_to_xtensa(target);
	struct xtensa_stub_tramp stub_tramp;
	int retval;

	xtensa_stub_tramp_get(xtensa, &stub_tramp);
	if (stub_tramp.size == 0)
		return ERROR_FAIL;

	if (algo_image) {
		/*TODO: add description of how to build proper ELF image to to be loaded to
		 * workspace */
		LOG_DEBUG(
			"stub: base 0x%x, start 0x%x, %d sections",
			algo_image->image.base_address_set ? (unsigned) algo_image->image.
			base_address : 0,
			algo_image->image.start_address,
			algo_image->image.num_sections);
		stub->entry = algo_image->image.start_address;
		for (int i = 0; i < algo_image->image.num_sections; i++) {
			struct imagesection *section = &algo_image->image.sections[i];
			LOG_DEBUG("addr " TARGET_ADDR_FMT ", sz %d, flags %x",
				section->base_address,
				section->size,
				section->flags);
			if (section->flags & IMAGE_ELF_PHF_EXEC) {
				if (target_alloc_working_area(target, section->size,
						&stub->code) != ERROR_OK) {
					LOG_ERROR(
						"no working area available, can't alloc space for stub code!");
					retval = ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
					goto _on_error;
				}
				if (section->base_address == 0) {
					section->base_address = stub->code->address;
					/* sanity check, stub is compiled to be run from working
					 * area */
				} else if (stub->code->address != section->base_address) {
					LOG_ERROR(
						"working area " TARGET_ADDR_FMT
						" and stub code section " TARGET_ADDR_FMT
						" address mismatch!",
						section->base_address,
						stub->code->address);
					retval = ERROR_FAIL;
					goto _on_error;
				}
			} else {
				if (target_alloc_alt_working_area(target,
						section->size + algo_image->bss_size,
						&stub->data) != ERROR_OK) {
					LOG_ERROR(
						"no working area available, can't alloc space for stub data!");
					retval = ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
					goto _on_error;
				}
				if (section->base_address == 0) {
					section->base_address = stub->data->address;
					/* sanity check, stub is compiled to be run from working
					 * area */
				} else if (stub->data->address != section->base_address) {
					LOG_ERROR(
						"working area " TARGET_ADDR_FMT
						" and stub data section " TARGET_ADDR_FMT
						" address mismatch!",
						section->base_address,
						stub->data->address);
					retval = ERROR_FAIL;
					goto _on_error;
				}
			}
			uint32_t sec_wr = 0;
			uint8_t buf[512];
			while (sec_wr < section->size) {
				uint32_t nb = section->size - sec_wr >
					sizeof(buf) ? sizeof(buf) : section->size - sec_wr;
				size_t size_read = 0;
				retval = image_read_section(&(algo_image->image),
					i,
					sec_wr,
					nb,
					buf,
					&size_read);
				if (retval != ERROR_OK) {
					LOG_ERROR("Failed to read stub section (%d)!", retval);
					goto _on_error;
				}
				retval = target_write_buffer(target,
					section->base_address + sec_wr,
					size_read,
					buf);
				if (retval != ERROR_OK) {
					LOG_ERROR("Failed to write stub section!");
					goto _on_error;
				}
				sec_wr += size_read;
			}
		}
	}
	if (stub->tramp_addr == 0) {
		/* alloc trampoline in code working area */
		if (target_alloc_working_area(target, stub_tramp.size, &stub->tramp) != ERROR_OK) {
			LOG_ERROR("no working area available, can't alloc space for stub jumper!");
			return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		}
		stub->tramp_addr = stub->tramp->address;
	}
	if (stub->stack_addr == 0) {
		/* alloc stack in data working area */
		if (target_alloc_alt_working_area(target, stack_size, &stub->stack) != ERROR_OK) {
			LOG_ERROR("no working area available, can't alloc stub stack!");
			target_free_working_area(target, stub->tramp);
			return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		}
		stub->stack_addr = stub->stack->address + stack_size;
	}
	retval = target_write_buffer(target, stub->tramp_addr, stub_tramp.size, stub_tramp.code);
	if (retval != ERROR_OK) {
		LOG_ERROR("Failed to write stub jumper!");
		goto _on_error;
	}

	return ERROR_OK;

_on_error:
	if (stub->tramp)
		target_free_working_area(target, stub->tramp);
	if (stub->stack)
		target_free_alt_working_area(target, stub->stack);
	if (stub->code)
		target_free_working_area(target, stub->code);
	if (stub->data)
		target_free_alt_working_area(target, stub->data);
	return retval;
}

static void xtensa_stub_cleanup(struct target *target, struct xtensa_stub *stub)
{
	destroy_reg_param(&stub->reg_params[5]);
	destroy_reg_param(&stub->reg_params[4]);
	destroy_reg_param(&stub->reg_params[3]);
	destroy_reg_param(&stub->reg_params[2]);
	destroy_reg_param(&stub->reg_params[1]);
	destroy_reg_param(&stub->reg_params[0]);
	if (stub->tramp)
		target_free_working_area(target, stub->tramp);
	if (stub->stack)
		target_free_alt_working_area(target, stub->stack);
	if (stub->code)
		target_free_working_area(target, stub->code);
	if (stub->data)
		target_free_alt_working_area(target, stub->data);
}

#if XTENSA_STUB_STACK_DEBUG
static int xtensa_stub_fill_stack(struct target *target,
	uint32_t stack_addr,
	uint32_t stack_size,
	uint32_t sz)
{
	uint8_t buf[256];

	/* fill stub stack with canary bytes */
	memset(buf, XTENSA_STUB_STACK_STAMP, sizeof(buf));
	for (uint32_t i = 0; i < sz; ) {
		uint32_t wr_sz = stack_size - i >= sizeof(buf) ? sizeof(buf) : stack_size - i;
		/* int retval = target_write_buffer(target, stack_addr + i, wr_sz, buf); */
		int retval = target_write_memory(target, stack_addr + i, 1, wr_sz, buf);
		if (retval != ERROR_OK) {
			LOG_ERROR("Failed to init stub stack (%d)!", retval);
			return retval;
		}
		i += wr_sz;
	}
	return ERROR_OK;
}

static int xtensa_stub_check_stack(struct target *target,
	uint32_t stack_addr,
	uint32_t stack_size,
	uint32_t sz)
{
	int retval = ERROR_OK;
	uint8_t buf[256];

	/* check stub stack for overflow */
	for (uint32_t i = 0; i < sz; ) {
		uint32_t rd_sz = sz - i >= sizeof(buf) ? sizeof(buf) : sz - i;
		retval = target_read_memory(target, stack_addr + i, 1, rd_sz, buf);
		if (retval != ERROR_OK) {
			LOG_ERROR("Failed to read stub stack (%d)!", retval);
			return retval;
		}
		int checked = 0;
		LOG_DEBUG("STK[%x]: check", stack_addr+i);
		uint32_t j;
		for (j = 0; j < rd_sz; j++) {
			if (buf[j] != XTENSA_STUB_STACK_STAMP) {
				if (i+j > 0)
					LOG_WARNING("Stub stack bytes unused %d / %d",
						i+j,
						stack_size);
				else {
					LOG_ERROR("Stub stack OVF!!!");
					retval = ERROR_FAIL;
				}
				checked = 1;
				break;
			}
		}
		if (checked)
			break;
		i += rd_sz;
	}
	return retval;
}
#endif

static int xtensa_algo_run(struct target *target, struct xtensa_algo_image *image,
	struct xtensa_algo_run_data *run)
{
	int retval;
	struct duration algo_time;
	void **mem_handles = NULL;

	if (duration_start(&algo_time) != 0) {
		LOG_ERROR("Failed to start algo time measurement!");
		return ERROR_FAIL;
	}

	retval = xtensa_stub_load(target, image, &run->priv.stub, run->stack_size);
	if (retval != ERROR_OK) {
		LOG_ERROR("Failed to load stub (%d)!", retval);
		return retval;
	}
	if (duration_measure(&algo_time) != 0) {
		LOG_ERROR("Failed to stop algo run measurement!");
		return ERROR_FAIL;
	}
	LOG_DEBUG("Stub loaded in %g ms", duration_elapsed(&algo_time)*1000);

	xtensa_algo_args_init(&run->priv.stub, run->priv.stub.stack_addr);
	/* allocate memory arguments and fill respective reg params */
	if (run->mem_args.count > 0) {
		mem_handles = malloc(sizeof(void *)*run->mem_args.count);
		if (mem_handles == NULL) {
			LOG_ERROR("Failed to alloc target mem handles!");
			retval = ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
			goto _cleanup;
		}
		memset(mem_handles, 0, sizeof(void *)*run->mem_args.count);
		/* alloc memory args target buffers */
		for (uint32_t i = 0; i < run->mem_args.count; i++) {
			/* small hack: if we need to update some reg param this field holds
			 * appropriate user argument number, */
			/* otherwise should hold UINT_MAX */
			uint32_t usr_param_num = run->mem_args.params[i].address;
			if (image) {
				static struct working_area *area;	/* see TODO at target.c:2018
									 *for why this is static */
				retval =
					target_alloc_alt_working_area(target,
					run->mem_args.params[i].size,
					&area);
				if (retval != ERROR_OK) {
					LOG_ERROR("Failed to alloc target buffer!");
					retval = ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
					goto _cleanup;
				}
				mem_handles[i] = area;
				run->mem_args.params[i].address = area->address;
			} else {
				struct xtensa_algo_run_data alloc_run;
				memset(&alloc_run, 0, sizeof(alloc_run));
				alloc_run.stack_size = run->on_board.min_stack_size;
				alloc_run.on_board.min_stack_addr = run->on_board.min_stack_addr;
				alloc_run.on_board.code_buf_size = run->on_board.code_buf_size;
				alloc_run.on_board.code_buf_addr = run->on_board.code_buf_addr;
				retval = xtensa_run_onboard_func(target,
					&alloc_run,
					run->on_board.alloc_func,
					1,
					run->mem_args.params[i].size);
				if (retval != ERROR_OK) {
					LOG_ERROR("Failed to run mem arg alloc onboard algo (%d)!",
						retval);
					goto _cleanup;
				}
				if (alloc_run.ret_code == 0) {
					LOG_ERROR("Failed to alloc onboard memory (%d)!", retval);
					retval = ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
					goto _cleanup;
				}
				mem_handles[i] = (void *)((long)alloc_run.ret_code);
				run->mem_args.params[i].address = alloc_run.ret_code;
			}
			if (usr_param_num != UINT_MAX) {/* if we need update some register param
							 * with to mem param addr */
				buf_set_u32(run->priv.stub.reg_params[XTENSA_STUB_ARGS_FUNC_START+
						usr_param_num].value,
					0, 32, run->mem_args.params[i].address);
			}
		}
	}

	if (run->usr_func_init) {
		retval = run->usr_func_init(target, run, run->usr_func_arg);
		if (retval != ERROR_OK) {
			LOG_ERROR("Failed to prepare algorithm host side args stub (%d)!", retval);
			goto _cleanup;
		}
	}

#if XTENSA_STUB_STACK_DEBUG
	LOG_DEBUG("Fill stack 0x%x", run->priv.stub.stack_addr);
	retval = xtensa_stub_fill_stack(target,
		run->priv.stub.stack_addr - run->stack_size,
		run->stack_size,
		XTENSA_STUB_STACK_DEBUG);
	if (retval != ERROR_OK) {
		LOG_ERROR("Failed to init stub stack (%d)!", retval);
		goto _cleanup;
	}
#endif
	LOG_DEBUG("Algorithm start @ 0x" TARGET_ADDR_FMT ", stack %d bytes @ 0x" TARGET_ADDR_FMT,
		run->priv.stub.tramp_addr, run->stack_size, run->priv.stub.stack_addr);
	retval = target_start_algorithm(target,
		run->mem_args.count, run->mem_args.params,
		run->priv.stub.reg_params_count, run->priv.stub.reg_params,
		run->priv.stub.tramp_addr, 0,
		&run->priv.stub.ainfo);
	if (retval != ERROR_OK) {
		LOG_ERROR("Failed to start algorithm (%d)!", retval);
		goto _cleanup;
	}

	if (run->usr_func) {
		/* give target algorithm stub time to init itself, then user func can communicate to
		 * it safely */
		alive_sleep(100);
		retval = run->usr_func(target, run->usr_func_arg);
		if (retval != ERROR_OK)
			LOG_ERROR("Failed to exec algorithm user func (%d)!", retval);
	}

	LOG_DEBUG("Wait algorithm completion");
	retval = target_wait_algorithm(target,
		run->mem_args.count, run->mem_args.params,
		run->priv.stub.reg_params_count, run->priv.stub.reg_params,
		0, run->tmo ? run->tmo : XTENSA_ALGORITHM_EXIT_TMO,
		&run->priv.stub.ainfo);
	if (retval != ERROR_OK) {
		LOG_ERROR("Failed to wait algorithm (%d)!", retval);
		/* target has been forced to stop in target_wait_algorithm() */
	}
#if XTENSA_STUB_STACK_DEBUG
	retval = xtensa_stub_check_stack(target,
		run->priv.stub.stack_addr - run->stack_size,
		run->stack_size,
		XTENSA_STUB_STACK_DEBUG);
	if (retval != ERROR_OK)
		LOG_ERROR("Failed to check stub stack (%d)!", retval);

#endif
	if (run->usr_func_done)
		run->usr_func_done(target, run, run->usr_func_arg);
_cleanup:
	/* free memory arguments */
	if (mem_handles) {
		for (uint32_t i = 0; i < run->mem_args.count; i++) {
			if (mem_handles[i]) {
				if (image)
					target_free_alt_working_area(target, mem_handles[i]);
				else {
					struct xtensa_algo_run_data free_run;
					memset(&free_run, 0, sizeof(free_run));
					free_run.stack_size = run->on_board.min_stack_size;
					free_run.on_board.min_stack_addr =
						run->on_board.min_stack_addr;
					free_run.on_board.code_buf_size =
						run->on_board.code_buf_size;
					free_run.on_board.code_buf_addr =
						run->on_board.code_buf_addr;
					retval = xtensa_run_onboard_func(target,
						&free_run,
						run->on_board.free_func,
						1,
						mem_handles[i]);
					if (retval != ERROR_OK)
						LOG_ERROR(
							"Failed to run mem arg free onboard algo (%d)!",
							retval);
				}
			}
		}
		free(mem_handles);
	}
	xtensa_stub_cleanup(target, &run->priv.stub);

	return retval;
}

int xtensa_run_algorithm_image(struct target *target,
	struct xtensa_algo_run_data *run,
	struct xtensa_algo_image *algo_image)
{
	if (!algo_image->image.start_address_set || algo_image->image.start_address == 0)
		return ERROR_FAIL;
	/* to be allocated automatically in corresponding working areas */
	run->priv.stub.stack_addr = 0;
	run->priv.stub.tramp_addr = 0;
	/* entry will be set from algo_image->image */
	return xtensa_algo_run(target, algo_image, run);
}

int xtensa_run_algorithm_onboard(struct target *target,
	struct xtensa_algo_run_data *run,
	void *func_entry)
{
	int res;
	struct xtensa *xtensa = target_to_xtensa(target);
	struct xtensa_stub_tramp stub_tramp;

	xtensa_stub_tramp_get(xtensa, &stub_tramp);
	if (stub_tramp.size == 0)
		return ERROR_FAIL;

	if (stub_tramp.size > run->on_board.code_buf_size) {
		LOG_ERROR("Stub tramp size %u bytes exceeds target buf size %d bytes!",
			stub_tramp.size, run->on_board.code_buf_size);
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	if (run->stack_size > run->on_board.min_stack_size) {
		if (run->on_board.alloc_func == 0 || run->on_board.free_func == 0) {
			LOG_ERROR("No stubs stack funcs found!");
			return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		}
		/* alloc stack */
		struct xtensa_algo_run_data alloc_run;
		memset(&alloc_run, 0, sizeof(alloc_run));
		alloc_run.stack_size = run->on_board.min_stack_size;
		alloc_run.on_board.min_stack_addr = run->on_board.min_stack_addr;
		alloc_run.on_board.code_buf_size = run->on_board.code_buf_size;
		alloc_run.on_board.code_buf_addr = run->on_board.code_buf_addr;
		res = xtensa_run_onboard_func(target,
			&alloc_run,
			run->on_board.alloc_func,
			1,
			run->stack_size);
		if (res != ERROR_OK) {
			LOG_ERROR("Failed to run stack alloc onboard algo (%d)!", res);
			return res;
		}
		LOG_DEBUG("RETCODE: %x!", alloc_run.ret_code);
		if (alloc_run.ret_code == 0) {
			LOG_ERROR("Failed to alloc onboard stack (%d)!", res);
			return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		}
		run->priv.stub.stack_addr = alloc_run.ret_code + run->stack_size;
	} else
		run->priv.stub.stack_addr = run->on_board.min_stack_addr + run->stack_size;
	run->priv.stub.tramp_addr = run->on_board.code_buf_addr;
	run->priv.stub.entry = (uintptr_t)func_entry;

	res = xtensa_algo_run(target, NULL, run);

	if (run->stack_size > run->on_board.min_stack_size) {
		/* free stack */
		struct xtensa_algo_run_data free_run;
		memset(&free_run, 0, sizeof(free_run));
		free_run.stack_size = run->on_board.min_stack_size;
		free_run.on_board.min_stack_addr = run->on_board.min_stack_addr;
		free_run.on_board.code_buf_size = run->on_board.code_buf_size;
		free_run.on_board.code_buf_addr = run->on_board.code_buf_addr;
		res = xtensa_run_onboard_func(target,
			&free_run,
			run->on_board.free_func,
			1,
			run->priv.stub.stack_addr - run->stack_size);
		if (res != ERROR_OK) {
			LOG_ERROR("Failed to run stack free onboard algo (%d)!", res);
			return res;
		}
	}
	return res;
}

static int xtensa_run_do(struct target *target,
	struct xtensa_algo_run_data *run,
	void *algo_arg,
	uint32_t num_args,
	va_list ap)
{
	char *arg_regs[] = {"a3", "a4", "a5", "a6"};

	init_reg_param(&run->priv.stub.reg_params[XTENSA_STUB_ARGS_FUNC_START+0],
		"a2",
		32,
		PARAM_IN_OUT);
	if (num_args > 0) {
		uint32_t arg = va_arg(ap, uint32_t);
		buf_set_u32(run->priv.stub.reg_params[XTENSA_STUB_ARGS_FUNC_START+0].value,
			0,
			32,
			arg);
		LOG_DEBUG("Set arg[0] = %d", arg);
	} else
		buf_set_u32(run->priv.stub.reg_params[XTENSA_STUB_ARGS_FUNC_START+0].value, 0, 32,
			0);
	for (uint32_t i = 1; i < num_args; i++) {
		uint32_t arg = va_arg(ap, uint32_t);
		init_reg_param(&run->priv.stub.reg_params[XTENSA_STUB_ARGS_FUNC_START+i],
			arg_regs[i-1], 32, PARAM_OUT);
		buf_set_u32(run->priv.stub.reg_params[XTENSA_STUB_ARGS_FUNC_START+i].value,
			0,
			32,
			arg);
		LOG_DEBUG("Set arg[%d] = %d", i, arg);
	}

	run->priv.stub.ainfo.core_mode = XT_MODE_ANY;
	run->priv.stub.reg_params_count = XTENSA_STUB_ARGS_FUNC_START + num_args;

	int retval = run->algo_func(target, run, algo_arg);
	if (retval != ERROR_OK)
		LOG_ERROR("Algorithm run failed (%d)!", retval);
	else {
		run->ret_code = buf_get_u32(
			run->priv.stub.reg_params[XTENSA_STUB_ARGS_FUNC_START+0].value,
			0,
			32);
		LOG_DEBUG("Got algorithm RC %x", run->ret_code);
	}

	for (uint32_t i = 0; i < num_args; i++)
		destroy_reg_param(&run->priv.stub.reg_params[XTENSA_STUB_ARGS_FUNC_START+i]);

	return retval;
}

int xtensa_run_func_image_va(struct target *target,
	struct xtensa_algo_run_data *run,
	struct xtensa_algo_image *image,
	uint32_t num_args,
	va_list ap)
{
	run->algo_func = (xtensa_algo_func_t)xtensa_run_algorithm_image;
	return xtensa_run_do(target, run, image, num_args, ap);
}

int xtensa_run_onboard_func_va(struct target *target,
	struct xtensa_algo_run_data *run,
	uint32_t func_addr,
	uint32_t num_args,
	va_list ap)
{
	run->algo_func = (xtensa_algo_func_t)xtensa_run_algorithm_onboard;
	return xtensa_run_do(target, run, (void *)(unsigned long)func_addr, num_args, ap);
}
