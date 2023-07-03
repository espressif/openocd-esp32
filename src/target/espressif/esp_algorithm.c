// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   Espressif chips common algorithm API for OpenOCD                      *
 *   Copyright (C) 2022 Espressif Systems Ltd.                             *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <helper/align.h>
#include <target/algorithm.h>
#include <target/target.h>
#include "esp_algorithm.h"

#define ALGO_STUB_DEBUG            0
#define ALGO_ALGORITHM_EXIT_TMO    40000	/* ms */

#if ALGO_STUB_DEBUG
#define ALGO_STUB_STACK_STAMP      0xCE
#define ALGO_STUB_STACK_DEBUG      128
#else
#define ALGO_STUB_STACK_DEBUG      0
#endif

#if ALGO_STUB_STACK_DEBUG
static int algorithm_stub_fill_stack(struct target *target,
	uint32_t stack_addr,
	uint32_t stack_size,
	uint32_t sz)
{
	uint8_t buf[256];

	/* fill stub stack with canary bytes */
	memset(buf, ALGO_STUB_STACK_STAMP, sizeof(buf));
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

static int algorithm_stub_check_stack(struct target *target,
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
		LOG_DEBUG("STK[%x]: check", stack_addr + i);
		uint32_t j;
		for (j = 0; j < rd_sz; j++) {
			if (buf[j] != ALGO_STUB_STACK_STAMP) {
				if (i + j > 0) {
					LOG_WARNING("Stub stack bytes unused %d / %d",
						i + j,
						stack_size);
				} else {
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

static int algorithm_read_stub_logs(struct target *target, struct algorithm_stub *stub)
{
	if (stub->log_buff_addr == 0 || stub->log_buff_size == 0)
		return ERROR_FAIL;

	uint32_t len = 0;
	int retval = target_read_u32(target, stub->log_buff_addr, &len);
	if (retval != ERROR_OK)
		return retval;

	/* sanity check. log_buff_size = sizeof(len) + sizeof(log_buff) */
	if (len == 0 || len > stub->log_buff_size - 4)
		return ERROR_FAIL;

	uint8_t *log_buff = calloc(1, len);
	if (!log_buff) {
		LOG_ERROR("Failed to allocate memory for the stub log (%d)!", retval);
		return retval;
	}
	retval = target_read_memory(target, stub->log_buff_addr + 4, 1, len, log_buff);
	if (retval == ERROR_OK)
		LOG_OUTPUT("%*.*s", len, len, log_buff);
	free(log_buff);
	return retval;
}

static int algorithm_run(struct target *target, struct algorithm_image *image,
	struct algorithm_run_data *run,
	uint32_t num_args,
	va_list ap)
{
	void **mem_handles = NULL;

	int retval = run->hw->algo_init(target, run, num_args, ap);
	if (retval != ERROR_OK)
		return retval;

	/* allocate memory arguments and fill respective reg params */
	if (run->mem_args.count > 0) {
		mem_handles = calloc(run->mem_args.count, sizeof(void *));
		if (!mem_handles) {
			LOG_ERROR("Failed to alloc target mem handles!");
			retval = ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
			goto _cleanup;
		}
		/* alloc memory args target buffers */
		for (uint32_t i = 0; i < run->mem_args.count; i++) {
			/* small hack: if we need to update some reg param this field holds
			 * appropriate user argument number, */
			/* otherwise should hold UINT_MAX */
			uint32_t usr_param_num = run->mem_args.params[i].address;
			if (image) {
				static struct working_area *area;
				retval = target_alloc_working_area(target, run->mem_args.params[i].size, &area);
				if (retval != ERROR_OK) {
					LOG_ERROR("Failed to alloc target buffer!");
					retval = ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
					goto _cleanup;
				}
				mem_handles[i] = area;
				run->mem_args.params[i].address = area->address;
			} else {
				struct algorithm_run_data alloc_run;
				memset(&alloc_run, 0, sizeof(alloc_run));
				alloc_run.hw = run->hw;
				alloc_run.stack_size = run->on_board.min_stack_size;
				alloc_run.on_board.min_stack_addr = run->on_board.min_stack_addr;
				alloc_run.on_board.code_buf_size = run->on_board.code_buf_size;
				alloc_run.on_board.code_buf_addr = run->on_board.code_buf_addr;
				retval = algorithm_run_onboard_func(target,
					&alloc_run,
					run->on_board.alloc_func,
					1,
					run->mem_args.params[i].size);
				if (retval != ERROR_OK) {
					LOG_ERROR("Failed to run mem arg alloc onboard algo (%d)!", retval);
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
			if (usr_param_num != UINT_MAX) /* if we need update some register param with mem param value */
				algorithm_user_arg_set_uint(run, usr_param_num, run->mem_args.params[i].address);
		}
	}

	if (run->usr_func_init) {
		retval = run->usr_func_init(target, run, run->usr_func_arg);
		if (retval != ERROR_OK) {
			LOG_ERROR("Failed to prepare algorithm host side args stub (%d)!", retval);
			goto _cleanup;
		}
	}

#if ALGO_STUB_STACK_DEBUG
	LOG_DEBUG("Fill stack " TARGET_ADDR_FMT, run->stub.stack_addr);
	retval = algorithm_stub_fill_stack(target,
		run->stub.stack_addr - run->stack_size,
		run->stack_size,
		ALGO_STUB_STACK_DEBUG);
	if (retval != ERROR_OK) {
		LOG_ERROR("Failed to init stub stack (%d)!", retval);
		goto _cleanup;
	}
#endif

	LOG_DEBUG("Algorithm start @ " TARGET_ADDR_FMT ", stack %d bytes @ " TARGET_ADDR_FMT,
		run->stub.tramp_mapped_addr, run->stack_size, run->stub.stack_addr);
	retval = target_start_algorithm(target,
		run->mem_args.count, run->mem_args.params,
		run->reg_args.count, run->reg_args.params,
		run->stub.tramp_mapped_addr, 0,
		run->stub.ainfo);
	if (retval != ERROR_OK) {
		LOG_ERROR("Failed to start algorithm (%d)!", retval);
		goto _cleanup;
	}

	if (run->usr_func) {
		/* give target algorithm stub time to init itself, then user func can communicate to it safely */
		alive_sleep(100);
		retval = run->usr_func(target, run->usr_func_arg);
		if (retval != ERROR_OK)
			LOG_ERROR("Failed to exec algorithm user func (%d)!", retval);
	}
	uint32_t tmo = 0;	/* do not wait if 'usr_func' returned error */
	if (retval == ERROR_OK)
		tmo = run->tmo ? run->tmo : ALGO_ALGORITHM_EXIT_TMO;
	LOG_DEBUG("Wait algorithm completion");
	retval = target_wait_algorithm(target,
		run->mem_args.count, run->mem_args.params,
		run->reg_args.count, run->reg_args.params,
		0, tmo,
		run->stub.ainfo);
	if (retval != ERROR_OK) {
		LOG_ERROR("Failed to wait algorithm (%d)!", retval);
		/* target has been forced to stop in target_wait_algorithm() */
	}
	algorithm_read_stub_logs(target, &run->stub);
#if ALGO_STUB_STACK_DEBUG
	ret = algorithm_stub_check_stack(target,
		run->stub.stack_addr - run->stack_size,
		run->stack_size,
		ALGO_STUB_STACK_DEBUG);
	if (ret != ERROR_OK) {
		LOG_ERROR("Failed to check stub stack (%d)!", ret);
		if (retval == ERROR_OK)
			retval = ret;
	}
#endif
	if (run->usr_func_done)
		run->usr_func_done(target, run, run->usr_func_arg);

	if (retval != ERROR_OK) {
		LOG_ERROR("Algorithm run failed (%d)!", retval);
	} else {
		run->ret_code = algorithm_user_arg_get_uint(run, 0);
		LOG_DEBUG("Got algorithm RC 0x%" PRIx32, run->ret_code);
	}

_cleanup:
	/* free memory arguments */
	if (mem_handles) {
		for (uint32_t i = 0; i < run->mem_args.count; i++) {
			if (mem_handles[i]) {
				if (image) {
					target_free_working_area(target, mem_handles[i]);
				} else {
					struct algorithm_run_data free_run;
					memset(&free_run, 0, sizeof(free_run));
					free_run.hw = run->hw;
					free_run.stack_size = run->on_board.min_stack_size;
					free_run.on_board.min_stack_addr = run->on_board.min_stack_addr;
					free_run.on_board.code_buf_size = run->on_board.code_buf_size;
					free_run.on_board.code_buf_addr = run->on_board.code_buf_addr;
					int ret = algorithm_run_onboard_func(target,
						&free_run,
						run->on_board.free_func,
						1,
						mem_handles[i]);
					if (ret != ERROR_OK)
						LOG_ERROR("Failed to run mem arg free onboard algo (%d)!", ret);
				}
			}
		}
		free(mem_handles);
	}
	run->hw->algo_cleanup(target, run);

	return retval;
}

static void reverse_binary(const uint8_t *src, uint8_t *dest, size_t length)
{
	size_t i;

	/* Put extra bytes to the beginning with padding */
	size_t remaining = length % 4;
	if (remaining > 0) {
		memset(dest, 0xFF, 4);
		for (i = 0; i < remaining; i++)
			dest[i] = src[length - remaining + i];
	}

	for (i = remaining; i < length - remaining; i += 4) {
		dest[i + 0] = src[length - i - 4];
		dest[i + 1] = src[length - i - 3];
		dest[i + 2] = src[length - i - 2];
		dest[i + 3] = src[length - i - 1];
	}
}

static int load_section_from_image(struct target *target,
	struct algorithm_run_data *run, int section_num, bool reverse)
{
	struct imagesection *section = &run->image.image.sections[section_num];
	uint32_t sec_wr = 0;
	uint8_t buf[1024];

	assert(sizeof(buf) % 4 == 0);

	while (sec_wr < section->size) {
		uint32_t nb = section->size - sec_wr > sizeof(buf) ? sizeof(buf) : section->size - sec_wr;
		size_t size_read = 0;
		int retval = image_read_section(&run->image.image, section_num, sec_wr, nb, buf, &size_read);
		if (retval != ERROR_OK) {
			LOG_ERROR("Failed to read stub section (%d)!", retval);
			return retval;
		}

		if (reverse) {
			size_t aligned_len = ALIGN_UP(size_read, 4);
			uint8_t reversed_buf[aligned_len];

			/* Send original size to allow padding */
			reverse_binary(buf, reversed_buf, size_read);

			retval = target_write_buffer(target, run->image.dram_org - sec_wr - size_read, size_read, reversed_buf);
			if (retval != ERROR_OK) {
				LOG_ERROR("Failed to write stub section!");
				return retval;
			}
		} else {
			retval = target_write_buffer(target, section->base_address + sec_wr, size_read, buf);
			if (retval != ERROR_OK) {
				LOG_ERROR("Failed to write stub section!");
				return retval;
			}
		}

		sec_wr += size_read;
	}

	return ERROR_OK;
}

int algorithm_load_func_image(struct target *target, struct algorithm_run_data *run)
{
	int retval;
	size_t tramp_sz = 0;
	const uint8_t *tramp = NULL;
	struct duration algo_time;

	if (duration_start(&algo_time) != 0) {
		LOG_ERROR("Failed to start algo time measurement!");
		return ERROR_FAIL;
	}

	if (run->hw->stub_tramp_get) {
		tramp = run->hw->stub_tramp_get(target, &tramp_sz);
		if (!tramp)
			return ERROR_FAIL;
	}

	/*TODO: add description of how to build proper ELF image to be loaded to workspace */
	LOG_DEBUG("stub: base 0x%x, start 0x%x, %d sections",
		run->image.image.base_address_set ? (unsigned int)run->image.image.base_address : 0,
		run->image.image.start_address,
		run->image.image.num_sections);
	run->stub.entry = run->image.image.start_address;

	/* [code + trampoline] --- padding --- [data] */

	/* Load code section */

	/* Will help to calculate the padding size between code and data regions */
	uint32_t total_code_size = 0;

	for (unsigned int i = 0; i < run->image.image.num_sections; i++) {
		struct imagesection *section = &run->image.image.sections[i];

		if (section->size == 0)
			continue;

		if (section->flags & ESP_IMAGE_ELF_PHF_EXEC) {
			LOG_DEBUG("addr " TARGET_ADDR_FMT ", sz %d, flags %" PRIx64,
				section->base_address, section->size, section->flags);

			if (target_alloc_working_area(target, section->size, &run->stub.code) != ERROR_OK) {
				LOG_ERROR("no working area available, can't alloc space for stub code!");
				retval = ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
				goto _on_error;
			}
			if (section->base_address == 0) {
				section->base_address = run->stub.code->address;
				/* sanity check, stub is compiled to be run from working area */
			} else if (run->stub.code->address != section->base_address) {
				LOG_ERROR("working area " TARGET_ADDR_FMT " and stub code section " TARGET_ADDR_FMT
					" address mismatch!",
					section->base_address,
					run->stub.code->address);
				retval = ERROR_FAIL;
				goto _on_error;
			}

			retval = load_section_from_image(target, run, i, run->image.reverse);
			if (retval != ERROR_OK)
				goto _on_error;
			total_code_size += ALIGN_UP(section->size, 4);
		}
	}

	/* If exists, load trampoline to the code area */
	if (tramp) {
		if (run->stub.tramp_addr == 0) {
			/* alloc trampoline in code working area */
			if (target_alloc_working_area(target, tramp_sz, &run->stub.tramp) != ERROR_OK) {
				LOG_ERROR("no working area available, can't alloc space for stub jumper!");
				retval = ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
				goto _on_error;
			}
			run->stub.tramp_addr = run->stub.tramp->address;
			total_code_size += ALIGN_UP(tramp_sz, 4);
		}

		uint32_t offset = run->stub.tramp_addr - target->working_area_phys;

		if (run->image.reverse) {
			target_addr_t reversed_tramp_addr = run->image.dram_org - offset;
			size_t aligned_len = ALIGN_UP(tramp_sz, 4);
			uint8_t reversed_tramp[aligned_len];

			/* Send original size to allow padding */
			reverse_binary(tramp, reversed_tramp, tramp_sz);

			retval = target_write_buffer(target, reversed_tramp_addr - tramp_sz, tramp_sz, reversed_tramp);
			if (retval != ERROR_OK) {
				LOG_ERROR("Failed to write stub jumper!");
				goto _on_error;
			}

		} else {
			LOG_DEBUG("Write tramp to addr " TARGET_ADDR_FMT ", sz %zu", run->stub.tramp_addr, tramp_sz);
			retval = target_write_buffer(target, run->stub.tramp_addr, tramp_sz, tramp);
			if (retval != ERROR_OK) {
				LOG_ERROR("Failed to write stub jumper!");
				goto _on_error;
			}
		}

		run->stub.tramp_mapped_addr = run->image.iram_org + offset;

		LOG_DEBUG("Tramp mapped to addr " TARGET_ADDR_FMT, run->stub.tramp_mapped_addr);
	}

	/* allocate dummy space until the data address */

	/* we dont need to restore padding area. TODO: check for reversed regions */
	uint32_t backup_working_area_prev = target->backup_working_area;
	target->backup_working_area = 0;
	if (target_alloc_working_area(target, run->image.iram_len - total_code_size, &run->stub.padding) != ERROR_OK) {
		LOG_ERROR("no working area available, can't alloc space for stub code!");
		retval = ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		goto _on_error;
	}
	target->backup_working_area = backup_working_area_prev;

	/*  Load the data section */
	for (unsigned int i = 0; i < run->image.image.num_sections; i++) {
		struct imagesection *section = &run->image.image.sections[i];

		if (section->size == 0)
			continue;

		if (!(section->flags & ESP_IMAGE_ELF_PHF_EXEC)) {
			LOG_DEBUG("addr " TARGET_ADDR_FMT ", sz %d, flags %" PRIx64, section->base_address, section->size,
				section->flags);
			/* target_alloc_working_area() aligns the whole working area size to 4-byte boundary.
			   We alloc one area for both DATA and BSS, so align each of them ourselves. */
			uint32_t data_sec_sz = ALIGN_UP(section->size, 4);
			LOG_DEBUG("DATA sec size %" PRIu32 " -> %" PRIu32, section->size, data_sec_sz);
			uint32_t bss_sec_sz = ALIGN_UP(run->image.bss_size, 4);
			LOG_DEBUG("BSS sec size %" PRIu32 " -> %" PRIu32, run->image.bss_size, bss_sec_sz);
			if (target_alloc_working_area(target, data_sec_sz + bss_sec_sz, &run->stub.data) != ERROR_OK) {
				LOG_ERROR("no working area available, can't alloc space for stub data!");
				retval = ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
				goto _on_error;
			}
			if (section->base_address == 0) {
				section->base_address = run->stub.data->address;
				/* sanity check, stub is compiled to be run from working area */
			} else if (run->stub.data->address != section->base_address) {
				LOG_ERROR("working area " TARGET_ADDR_FMT
					" and stub data section " TARGET_ADDR_FMT
					" address mismatch!",
					section->base_address,
					run->stub.data->address);
				retval = ERROR_FAIL;
				goto _on_error;
			}

			retval = load_section_from_image(target, run, i, false);
			if (retval != ERROR_OK)
				goto _on_error;
		}
	}

	/* stack */
	if (run->stub.stack_addr == 0 && run->stack_size > 0) {
		/* allocate stack in data working area */
		if (target_alloc_working_area(target, run->stack_size, &run->stub.stack) != ERROR_OK) {
			LOG_ERROR("no working area available, can't alloc stub stack!");
			retval = ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
			goto _on_error;
		}
		run->stub.stack_addr = run->stub.stack->address + run->stack_size;
	}

	if (duration_measure(&algo_time) != 0) {
		LOG_ERROR("Failed to stop algo run measurement!");
		retval = ERROR_FAIL;
		goto _on_error;
	}
	LOG_DEBUG("Stub loaded in %g ms", duration_elapsed(&algo_time) * 1000);
	return ERROR_OK;

_on_error:
	algorithm_unload_func_image(target, run);
	return retval;
}

int algorithm_unload_func_image(struct target *target, struct algorithm_run_data *run)
{
	target_free_all_working_areas(target);

	return ERROR_OK;
}

int algorithm_exec_func_image_va(struct target *target,
	struct algorithm_run_data *run,
	uint32_t num_args,
	va_list ap)
{
	if (!run->image.image.start_address_set || run->image.image.start_address == 0)
		return ERROR_FAIL;

	return algorithm_run(target, &run->image, run, num_args, ap);
}

int algorithm_load_onboard_func(struct target *target, target_addr_t func_addr, struct algorithm_run_data *run)
{
	int res;
	const uint8_t *tramp = NULL;
	size_t tramp_sz = 0;
	struct duration algo_time;

	if (duration_start(&algo_time) != 0) {
		LOG_ERROR("Failed to start algo time measurement!");
		return ERROR_FAIL;
	}

	if (run->hw->stub_tramp_get) {
		tramp = run->hw->stub_tramp_get(target, &tramp_sz);
		if (!tramp)
			return ERROR_FAIL;
	}

	if (tramp_sz > run->on_board.code_buf_size) {
		LOG_ERROR("Stub tramp size %u bytes exceeds target buf size %d bytes!",
			(uint32_t)tramp_sz, run->on_board.code_buf_size);
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	if (run->stack_size > run->on_board.min_stack_size) {
		if (run->on_board.alloc_func == 0 || run->on_board.free_func == 0) {
			LOG_ERROR("No stubs memory funcs found!");
			return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		}
		/* alloc stack */
		struct algorithm_run_data alloc_run;
		memset(&alloc_run, 0, sizeof(alloc_run));
		alloc_run.hw = run->hw;
		alloc_run.stack_size = run->on_board.min_stack_size;
		alloc_run.on_board.min_stack_addr = run->on_board.min_stack_addr;
		alloc_run.on_board.code_buf_size = run->on_board.code_buf_size;
		alloc_run.on_board.code_buf_addr = run->on_board.code_buf_addr;
		res = algorithm_run_onboard_func(target,
			&alloc_run,
			run->on_board.alloc_func,
			1,
			run->stack_size);
		if (res != ERROR_OK) {
			LOG_ERROR("Failed to run stack alloc onboard algo (%d)!", res);
			return res;
		}
		LOG_DEBUG("RETCODE: 0x%" PRIx32 "!", alloc_run.ret_code);
		if (alloc_run.ret_code == 0) {
			LOG_ERROR("Failed to alloc onboard stack (%d)!", res);
			return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		}
		run->stub.stack_addr = alloc_run.ret_code + run->stack_size;
	} else {
		run->stub.stack_addr = run->on_board.min_stack_addr + run->stack_size;
	}

	run->stub.tramp_addr = run->on_board.code_buf_addr;
	run->stub.tramp_mapped_addr = run->stub.tramp_addr;
	run->stub.entry = func_addr;

	if (tramp) {
		res = target_write_buffer(target, run->stub.tramp_addr, tramp_sz, tramp);
		if (res != ERROR_OK) {
			LOG_ERROR("Failed to write stub jumper!");
			algorithm_unload_onboard_func(target, run);
			return res;
		}
	}

	if (duration_measure(&algo_time) != 0) {
		LOG_ERROR("Failed to stop algo run measurement!");
		return ERROR_FAIL;
	}
	LOG_DEBUG("Stub loaded in %g ms", duration_elapsed(&algo_time) * 1000);

	return ERROR_OK;
}

int algorithm_unload_onboard_func(struct target *target, struct algorithm_run_data *run)
{
	if (run->stack_size > run->on_board.min_stack_size) {
		/* free stack */
		struct algorithm_run_data free_run;
		memset(&free_run, 0, sizeof(free_run));
		free_run.hw = run->hw;
		free_run.stack_size = run->on_board.min_stack_size;
		free_run.on_board.min_stack_addr = run->on_board.min_stack_addr;
		free_run.on_board.code_buf_size = run->on_board.code_buf_size;
		free_run.on_board.code_buf_addr = run->on_board.code_buf_addr;
		int res = algorithm_run_onboard_func(target,
			&free_run,
			run->on_board.free_func,
			1,
			run->stub.stack_addr - run->stack_size);
		if (res != ERROR_OK) {
			LOG_ERROR("Failed to run stack free onboard algo (%d)!", res);
			return res;
		}
	}

	return ERROR_OK;
}

int algorithm_exec_onboard_func_va(struct target *target,
	struct algorithm_run_data *run,
	uint32_t num_args,
	va_list ap)
{
	return algorithm_run(target, NULL, run, num_args, ap);
}
