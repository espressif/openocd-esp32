/***************************************************************************
 *   Generic flash driver for Espressif Xtensa chips                       *
 *   Copyright (C) 2017 Espressif Systems Ltd.                             *
 *   Author: Alexey Gerenkov <alexey@espressif.com>                        *
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
#include <target/register.h>
#include <target/esp_xtensa_apptrace.h>
#include "esp_xtensa.h"
#include "time_support.h"
#include "contrib/loaders/flash/esp/stub_flasher.h"

#define ESP_XTENSA_FLASH_MIN_OFFSET      0x1000	/* protect secure boot digest data */
#define ESP_XTENSA_RW_TMO                20000	/* ms */
#define ESP_XTENSA_ERASE_TMO             60000	/* ms */

struct esp_xtensa_rw_args {
	int (*xfer)(struct target *target, uint32_t block_id, uint32_t len, void *priv);
	uint8_t *buffer;
	uint32_t count;
	uint32_t total_count;
	bool connected;
};

struct esp_xtensa_write_state {
	struct esp_xtensa_rw_args rw;
	uint32_t prev_block_id;
	struct working_area *target_buf;
	struct esp_xtensa_flash_bank *esp_xtensa_info;
};

struct esp_xtensa_read_state {
	struct esp_xtensa_rw_args rw;
	uint8_t *rd_buf;
};

struct esp_xtensa_erase_check_args {
	struct working_area *erased_state_buf;
	uint32_t num_sectors;
};

struct esp_xtensa_flash_bp_op_state {
	struct working_area *target_buf;
	struct esp_xtensa_flash_bank *esp_xtensa_info;
};

static int esp_xtensa_flasher_image_init(struct xtensa_algo_image *flasher_image,
	const struct esp_xtensa_flasher_stub_config *stub_cfg)
{
	if (!stub_cfg) {
		LOG_ERROR("Invalid stub!");
		return ERROR_FAIL;
	}

	flasher_image->bss_size = stub_cfg->bss_sz;
	memset(&flasher_image->image, 0, sizeof(flasher_image->image));
	int ret = image_open(&flasher_image->image, NULL, "build");
	if (ret != ERROR_OK) {
		LOG_ERROR("Failed to create image (%d)!", ret);
		return ret;
	}
	flasher_image->image.start_address_set = 1;
	flasher_image->image.start_address = stub_cfg->entry_addr;
	ret = image_add_section(&flasher_image->image,
		0,
		stub_cfg->code_sz,
		IMAGE_ELF_PHF_EXEC,
		stub_cfg->code);
	if (ret != ERROR_OK) {
		LOG_ERROR("Failed to create image (%d)!", ret);
		image_close(&flasher_image->image);
		return ret;
	}
	ret = image_add_section(&flasher_image->image, 0, stub_cfg->data_sz, 0, stub_cfg->data);
	if (ret != ERROR_OK) {
		LOG_ERROR("Failed to create image (%d)!", ret);
		image_close(&flasher_image->image);
		return ret;
	}
	LOG_DEBUG("base=%08x set=%d",
		(unsigned) flasher_image->image.base_address,
		flasher_image->image.base_address_set);
	return ret;
}

int esp_xtensa_flash_init(struct esp_xtensa_flash_bank *esp_xtensa_info, uint32_t sec_sz,
	int (*run_func_image)(struct target *target, struct xtensa_algo_run_data *run,
		struct xtensa_algo_image *image, uint32_t num_args, ...),
	bool (*is_irom_address)(target_addr_t addr),
	bool (*is_drom_address)(target_addr_t addr),
	const struct esp_xtensa_flasher_stub_config *(*get_stub)(struct flash_bank *bank))
{

	esp_xtensa_info->probed = 0;
	esp_xtensa_info->sec_sz = sec_sz;
	esp_xtensa_info->get_stub = get_stub;
	esp_xtensa_info->run_func_image = run_func_image;
	esp_xtensa_info->is_irom_address = is_irom_address;
	esp_xtensa_info->is_drom_address = is_drom_address;
	esp_xtensa_info->hw_flash_base = 0;
	esp_xtensa_info->appimage_flash_base = (uint32_t)-1;
	return ERROR_OK;
}

int esp_xtensa_protect(struct flash_bank *bank, int set, int first, int last)
{
	return ERROR_FAIL;
}

int esp_xtensa_protect_check(struct flash_bank *bank)
{
	return ERROR_OK;
}

int esp_xtensa_blank_check(struct flash_bank *bank)
{
	struct esp_xtensa_flash_bank *esp_xtensa_info = bank->driver_priv;
	struct xtensa_algo_run_data run;
	struct xtensa_algo_image flasher_image;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted!");
		return ERROR_TARGET_NOT_HALTED;
	}

	int ret = esp_xtensa_flasher_image_init(&flasher_image, esp_xtensa_info->get_stub(bank));
	if (ret != ERROR_OK)
		return ret;

	memset(&run, 0, sizeof(run));
	run.stack_size = 1300;
	struct mem_param mp;
	init_mem_param(&mp, 3 /*3rd usr arg*/, bank->num_sectors /*size in bytes*/, PARAM_IN);
	run.mem_args.params = &mp;
	run.mem_args.count = 1;

	ret = esp_xtensa_info->run_func_image(bank->target,
		&run,
		&flasher_image,
		4,
		ESP_XTENSA_STUB_CMD_FLASH_ERASE_CHECK /*cmd*/,
		esp_xtensa_info->hw_flash_base/esp_xtensa_info->sec_sz /*start*/,
		bank->num_sectors /*sectors num*/,
		0 /*address to store sectors' state*/);
	if (ret != ERROR_OK) {
		LOG_ERROR("Failed to run flasher stub (%d)!", ret);
		destroy_mem_param(&mp);
		return ret;
	}
	if (run.ret_code != ESP_XTENSA_STUB_ERR_OK) {
		LOG_ERROR("Failed to check erase flash (%d)!", run.ret_code);
		ret = ERROR_FAIL;
	} else {
		for (int i = 0; i < bank->num_sectors; i++)
			bank->sectors[i].is_erased = mp.value[i];
	}
	destroy_mem_param(&mp);
	return ret;
}

static uint32_t esp_xtensa_get_size(struct flash_bank *bank)
{
	struct esp_xtensa_flash_bank *esp_xtensa_info = bank->driver_priv;
	uint32_t size = 0;
	struct xtensa_algo_run_data run;
	struct xtensa_algo_image flasher_image;

	int ret = esp_xtensa_flasher_image_init(&flasher_image, esp_xtensa_info->get_stub(bank));
	if (ret != ERROR_OK)
		return ret;

	memset(&run, 0, sizeof(run));
	run.stack_size = 1024;
	ret = esp_xtensa_info->run_func_image(bank->target,
		&run,
		&flasher_image,
		1,
		ESP_XTENSA_STUB_CMD_FLASH_SIZE);
	if (ret != ERROR_OK) {
		LOG_ERROR("Failed to run flasher stub (%d)!", ret);
		return 0;
	}
	size = run.ret_code;
	if (size == 0)
		LOG_ERROR("Failed to get flash size!");
	LOG_DEBUG("%s size 0x%x", __func__, size);
	return size;
}

static int esp_xtensa_get_mappings(struct flash_bank *bank,
	struct esp_xtensa_flash_bank *esp_xtensa_info,
	struct esp_xtensa_flash_mapping *flash_map,
	uint32_t appimage_flash_base)
{
	struct xtensa_algo_run_data run;
	struct xtensa_algo_image flasher_image;

	int ret = esp_xtensa_flasher_image_init(&flasher_image, esp_xtensa_info->get_stub(bank));
	if (ret != ERROR_OK)
		return ret;

	memset(&run, 0, sizeof(run));
	run.stack_size = 1300;

	struct mem_param mp;
	init_mem_param(&mp,
		2 /*2nd usr arg*/,
		sizeof(struct esp_xtensa_flash_mapping)	/*size in bytes*/,
		PARAM_IN);
	run.mem_args.params = &mp;
	run.mem_args.count = 1;

	ret = esp_xtensa_info->run_func_image(bank->target,
		&run,
		&flasher_image,
		3 /*args num*/,
		ESP_XTENSA_STUB_CMD_FLASH_MAP_GET /*cmd*/,
		appimage_flash_base,
		0 /*address to store mappings*/);
	if (ret != ERROR_OK) {
		LOG_ERROR("Failed to run flasher stub (%d)!", ret);
		destroy_mem_param(&mp);
		return ret;
	}
	if (run.ret_code != ESP_XTENSA_STUB_ERR_OK) {
		LOG_ERROR("Failed to get flash maps (%d)!", run.ret_code);
		ret = ERROR_FAIL;
	} else {
		memcpy(flash_map, mp.value, sizeof(struct esp_xtensa_flash_mapping));
		if (flash_map->maps_num == 0)
			LOG_WARNING("Empty flash mapping!");
		for (uint32_t i = 0; i < flash_map->maps_num; i++)
			LOG_INFO("Flash mapping %d: 0x%x -> 0x%x, %d KB",
				i,
				flash_map->maps[i].phy_addr,
				flash_map->maps[i].load_addr,
				flash_map->maps[i].size/1024);
	}
	destroy_mem_param(&mp);
	return ret;
}

int esp_xtensa_erase(struct flash_bank *bank, int first, int last)
{
	struct esp_xtensa_flash_bank *esp_xtensa_info = bank->driver_priv;
	struct xtensa_algo_run_data run;
	struct xtensa_algo_image flasher_image;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}
	assert((0 <= first) && (first <= last) && (last < bank->num_sectors));
	if (esp_xtensa_info->hw_flash_base + first*esp_xtensa_info->sec_sz <
		ESP_XTENSA_FLASH_MIN_OFFSET) {
		LOG_ERROR("Invalid offset!");
		return ERROR_FAIL;
	}

	int ret = esp_xtensa_flasher_image_init(&flasher_image, esp_xtensa_info->get_stub(bank));
	if (ret != ERROR_OK)
		return ret;

	memset(&run, 0, sizeof(run));
	run.stack_size = 1024;
	run.tmo = ESP_XTENSA_ERASE_TMO;
	ret = esp_xtensa_info->run_func_image(bank->target,
		&run,
		&flasher_image,
		3,
		ESP_XTENSA_STUB_CMD_FLASH_ERASE,
		/* cmd */
		esp_xtensa_info->hw_flash_base + first*esp_xtensa_info->sec_sz,
		/* start addr */
		(last-first+1)*esp_xtensa_info->sec_sz);		/* size */
	if (ret != ERROR_OK) {
		LOG_ERROR("Failed to run flasher stub (%d)!", ret);
		return ret;
	}
	if (run.ret_code != ESP_XTENSA_STUB_ERR_OK) {
		LOG_ERROR("Failed to erase flash (%d)!", run.ret_code);
		ret = ERROR_FAIL;
	}
	return ret;
}

static int esp_xtensa_rw_do(struct target *target, void *priv)
{
	struct duration algo_time, tmo_time;
	struct esp_xtensa_rw_args *rw = (struct esp_xtensa_rw_args *)priv;
	int retval = ERROR_OK, busy_num = 0;

	if (duration_start(&algo_time) != 0) {
		LOG_ERROR("Failed to start data write time measurement!");
		return ERROR_FAIL;
	}
	while (rw->total_count < rw->count) {
		uint32_t block_id = 0, len = 0;
		LOG_DEBUG("Transfer block on %s", target_name(target));
		retval = esp_xtensa_apptrace_data_len_read(target, &block_id, &len);
		if (retval != ERROR_OK) {
			LOG_ERROR("Failed to read apptrace status (%d)!", retval);
			return retval;
		}
		/* transfer block */
		LOG_DEBUG("Transfer block %d, %d bytes", block_id, len);
		retval = rw->xfer(target, block_id, len, rw);
		if (retval == ERROR_WAIT) {
			LOG_DEBUG("Block not ready");
			if (busy_num++ == 0) {
				if (duration_start(&tmo_time) != 0) {
					LOG_ERROR("Failed to start data write time measurement!");
					return ERROR_FAIL;
				}
			} else {
				/* if no transfer check tmo */
				if (duration_measure(&tmo_time) != 0) {
					LOG_ERROR("Failed to stop algo run measurement!");
					return ERROR_FAIL;
				}
				if (1000*duration_elapsed(&tmo_time) > ESP_XTENSA_RW_TMO) {
					LOG_ERROR("Transfer data tmo!");
					return ERROR_WAIT;
				}
			}
		} else if (retval != ERROR_OK) {
			LOG_ERROR("Failed to transfer flash data block (%d)!", retval);
			return retval;
		} else
			busy_num = 0;
		alive_sleep(10);
		if (target->state != TARGET_DEBUG_RUNNING) {
			LOG_ERROR("Algorithm accidentally stopped (%d)!", target->state);
			return ERROR_FAIL;
		}
	}
	if (duration_measure(&algo_time) != 0) {
		LOG_ERROR("Failed to stop data write measurement!");
		return ERROR_FAIL;
	}
	LOG_DEBUG("PROF: Data transffered in %g ms @ %g KB/s",
		duration_elapsed(&algo_time)*1000,
		duration_kbps(&algo_time, rw->total_count));

	return ERROR_OK;
}

static int esp_xtensa_write_xfer(struct target *target, uint32_t block_id, uint32_t len, void *priv)
{
	struct esp_xtensa_write_state *state = (struct esp_xtensa_write_state *)priv;
	int retval;

	/* check for target to get connected */
	if (!state->rw.connected) {
		retval =
			esp_xtensa_apptrace_ctrl_reg_read(target, NULL, NULL, &state->rw.connected);
		if (retval != ERROR_OK) {
			LOG_ERROR("Failed to read apptrace control reg (%d)!", retval);
			return retval;
		}
		if (!state->rw.connected)
			return ERROR_WAIT;
	}

	if (state->prev_block_id == block_id)
		return ERROR_WAIT;

	uint32_t wr_sz = state->rw.count - state->rw.total_count <
		esp_xtensa_apptrace_usr_block_max_size_get(target) ?
		state->rw.count -
		state->rw.total_count : esp_xtensa_apptrace_usr_block_max_size_get(target);
	retval = esp_xtensa_apptrace_usr_block_write(target,
		block_id,
		state->rw.buffer + state->rw.total_count,
		wr_sz);
	if (retval != ERROR_OK) {
		LOG_ERROR("Failed to write apptrace data (%d)!", retval);
		return retval;
	}
	state->rw.total_count += wr_sz;
	state->prev_block_id = block_id;

	return ERROR_OK;
}

static int esp_xtensa_write_state_init(struct target *target,
	struct xtensa_algo_run_data *run,
	struct esp_xtensa_write_state *state)
{
	struct duration algo_time;

	/* clear control register, stub will set XTENSA_APPTRACE_HOST_CONNECT bit when it will be
	 * ready */
	int ret = esp_xtensa_apptrace_ctrl_reg_write(target,
		0 /*block_id*/,
		0 /*len*/,
		false /*conn*/,
		false /*data*/);
	if (ret != ERROR_OK) {
		LOG_ERROR("Failed to clear apptrace ctrl reg (%d)!", ret);
		return ret;
	}

	/* memory buffer */
	if (duration_start(&algo_time) != 0) {
		LOG_ERROR("Failed to start workarea alloc time measurement!");
		return ERROR_FAIL;
	}
	uint32_t buffer_size = 64*1024;
	while (target_alloc_alt_working_area_try(target, buffer_size,
			&state->target_buf) != ERROR_OK) {
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
	LOG_DEBUG("PROF: Allocated target buffer %d bytes in %g ms", buffer_size,
		duration_elapsed(&algo_time)*1000);

	buf_set_u32(run->priv.stub.reg_params[XTENSA_STUB_ARGS_FUNC_START+3].value,
		0,
		32,
		state->target_buf->address);										/*
															 * down
															 * buffer */
	buf_set_u32(run->priv.stub.reg_params[XTENSA_STUB_ARGS_FUNC_START+4].value,
		0,
		32,
		state->target_buf->size);									/*
														 * down
														 * buffer
														 * size */

	return ERROR_OK;
}

static void esp_xtensa_write_state_cleanup(struct target *target,
	struct xtensa_algo_run_data *run,
	struct esp_xtensa_write_state *state)
{
	struct duration algo_time;

	if (!state->target_buf)
		return;
	if (duration_start(&algo_time) != 0)
		LOG_ERROR("Failed to start workarea alloc time measurement!");
	target_free_alt_working_area(target, state->target_buf);
	if (duration_measure(&algo_time) != 0)
		LOG_ERROR("Failed to stop data write measurement!");
	LOG_DEBUG("PROF: Workarea freed in %g ms", duration_elapsed(&algo_time)*1000);
}

int esp_xtensa_write(struct flash_bank *bank, const uint8_t *buffer,
	uint32_t offset, uint32_t count)
{
	struct esp_xtensa_flash_bank *esp_xtensa_info = bank->driver_priv;
	struct xtensa_algo_run_data run;
	struct esp_xtensa_write_state wr_state;
	struct xtensa_algo_image flasher_image;

	if (esp_xtensa_info->hw_flash_base + offset < ESP_XTENSA_FLASH_MIN_OFFSET) {
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

	int ret = esp_xtensa_flasher_image_init(&flasher_image, esp_xtensa_info->get_stub(bank));
	if (ret != ERROR_OK)
		return ret;

	memset(&run, 0, sizeof(run));
	run.stack_size = 1024;
	run.usr_func = esp_xtensa_rw_do;
	run.usr_func_arg = &wr_state;
	run.usr_func_init = (xtensa_algo_usr_func_init_t)esp_xtensa_write_state_init;
	run.usr_func_done = (xtensa_algo_usr_func_done_t)esp_xtensa_write_state_cleanup;
	memset(&wr_state, 0, sizeof(struct esp_xtensa_write_state));
	wr_state.rw.buffer = (uint8_t *)buffer;
	wr_state.rw.count = count;
	wr_state.rw.xfer = esp_xtensa_write_xfer;
	wr_state.prev_block_id = (uint32_t)-1;
	wr_state.esp_xtensa_info = esp_xtensa_info;

	ret = esp_xtensa_info->run_func_image(bank->target,
		&run,
		&flasher_image,
		5,
		ESP_XTENSA_STUB_CMD_FLASH_WRITE,
		/* cmd */
		esp_xtensa_info->hw_flash_base + offset,
		/* start addr */
		count,
		/* size */
		0,
		/* down buf addr */
		0);						/* down buf size */
	if (ret != ERROR_OK) {
		LOG_ERROR("Failed to run flasher stub (%d)!", ret);
		return ret;
	}
	if (run.ret_code != ESP_XTENSA_STUB_ERR_OK) {
		LOG_ERROR("Failed to write flash (%d)!", run.ret_code);
		ret = ERROR_FAIL;
	}
	return ret;
}

static int esp_xtensa_read_xfer(struct target *target, uint32_t block_id, uint32_t len, void *priv)
{
	struct esp_xtensa_read_state *state = (struct esp_xtensa_read_state *)priv;
	int retval;

	/* check for target to get connected */
	if (!state->rw.connected) {
		retval =
			esp_xtensa_apptrace_ctrl_reg_read(target, NULL, NULL, &state->rw.connected);
		if (retval != ERROR_OK) {
			LOG_ERROR("Failed to read apptrace control reg (%d)!", retval);
			return retval;
		}
		if (!state->rw.connected)
			return ERROR_WAIT;
	}

	if (len == 0)
		return ERROR_WAIT;

	retval = esp_xtensa_apptrace_data_read(target, len, state->rd_buf, block_id, 1 /*ack*/);
	if (retval != ERROR_OK) {
		LOG_ERROR("Failed to read apptrace status (%d)!", retval);
		return retval;
	}
	LOG_DEBUG("DATA %d bytes: %x %x %x %x %x %x %x %x", len,
		state->rd_buf[0], state->rd_buf[1], state->rd_buf[2], state->rd_buf[3],
		state->rd_buf[4], state->rd_buf[5], state->rd_buf[6], state->rd_buf[7]);

	uint8_t *ptr = state->rd_buf;
	while (ptr < state->rd_buf + len) {
		uint32_t data_sz = 0;
		ptr = esp_xtensa_apptrace_usr_block_get(ptr, &data_sz);
		if (data_sz > 0)
			memcpy(state->rw.buffer + state->rw.total_count, ptr, data_sz);
		ptr += data_sz;
		state->rw.total_count += data_sz;
	}

	return ERROR_OK;
}

static int esp_xtensa_read_state_init(struct target *target,
	struct xtensa_algo_run_data *run,
	struct esp_xtensa_read_state *state)
{
	/* clear control register, stub will set XTENSA_APPTRACE_HOST_CONNECT bit when it will be
	 * ready */
	int ret = esp_xtensa_apptrace_ctrl_reg_write(target,
		0 /*block_id*/,
		0 /*len*/,
		false /*conn*/,
		false /*data*/);
	if (ret != ERROR_OK) {
		LOG_ERROR("Failed to clear apptrace ctrl reg (%d)!", ret);
		return ret;
	}

	state->rd_buf = malloc(esp_xtensa_apptrace_block_max_size_get(target));
	if (!state->rd_buf) {
		LOG_ERROR("Failed to alloc read buffer!");
		return ERROR_FAIL;
	}
	return ERROR_OK;
}

int esp_xtensa_read(struct flash_bank *bank, uint8_t *buffer,
	uint32_t offset, uint32_t count)
{
	struct esp_xtensa_flash_bank *esp_xtensa_info = bank->driver_priv;
	struct xtensa_algo_run_data run;
	struct esp_xtensa_read_state rd_state;
	struct xtensa_algo_image flasher_image;

	if (offset & 0x3UL) {
		LOG_ERROR("Unaligned offset!");
		return ERROR_FAIL;
	}
	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	int ret = esp_xtensa_flasher_image_init(&flasher_image, esp_xtensa_info->get_stub(bank));
	if (ret != ERROR_OK)
		return ret;

	memset(&run, 0, sizeof(run));
	run.stack_size = 1024;
	run.usr_func_init = (xtensa_algo_usr_func_init_t)esp_xtensa_read_state_init;
	run.usr_func = esp_xtensa_rw_do;
	run.usr_func_arg = &rd_state;
	memset(&rd_state, 0, sizeof(struct esp_xtensa_read_state));
	rd_state.rw.buffer = buffer;
	rd_state.rw.count = count;
	rd_state.rw.xfer = esp_xtensa_read_xfer;

	ret = esp_xtensa_info->run_func_image(bank->target,
		&run,
		&flasher_image,
		3,
		ESP_XTENSA_STUB_CMD_FLASH_READ,
		/* cmd */
		esp_xtensa_info->hw_flash_base + offset,
		/* start addr */
		count);						/* size */

	free(rd_state.rd_buf);
	if (ret != ERROR_OK) {
		LOG_ERROR("Failed to run flasher stub (%d)!", ret);
		return ret;
	}
	if (run.ret_code != ESP_XTENSA_STUB_ERR_OK) {
		LOG_ERROR("Failed to read flash (%d)!", run.ret_code);
		ret = ERROR_FAIL;
	}
	return ret;
}

int esp_xtensa_probe(struct flash_bank *bank)
{
	struct esp_xtensa_flash_bank *esp_xtensa_info = bank->driver_priv;
	struct esp_xtensa_flash_mapping flash_map = {.maps_num = 0};
	uint32_t irom_base = 0, irom_sz = 0, drom_base = 0, drom_sz = 0, irom_flash_base = 0,
		drom_flash_base = 0;

	esp_xtensa_info->probed = 0;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	LOG_DEBUG("Flash size = %d KB @ "TARGET_ADDR_FMT " '%s' - '%s'",
		bank->size/1024,
		bank->base,
		target_name(bank->target),
		target_state_name(bank->target));

	if (bank->sectors) {
		free(bank->sectors);
		bank->sectors = NULL;
	}

	int ret = esp_xtensa_get_mappings(bank,
		esp_xtensa_info,
		&flash_map,
		esp_xtensa_info->appimage_flash_base);
	if (ret != ERROR_OK || flash_map.maps_num == 0) {
		LOG_WARNING("Failed to get flash mappings (%d)!", ret);
		/* if no DROM/IROM mappings so pretend they are at the end of the HW flash bank and
		 * have zero size to allow correct memory map with non zero RAM region */
		irom_base = drom_base = esp_xtensa_get_size(bank);
	} else {
		for (uint32_t i = 0; i < flash_map.maps_num; i++) {
			if (esp_xtensa_info->is_irom_address(flash_map.maps[i].load_addr)) {
				irom_flash_base = flash_map.maps[i].phy_addr &
					~(esp_xtensa_info->sec_sz-1);
				irom_base = flash_map.maps[i].load_addr &
					~(esp_xtensa_info->sec_sz-1);
				irom_sz = flash_map.maps[i].size;
				if (irom_sz & (esp_xtensa_info->sec_sz-1))
					irom_sz = (irom_sz & ~(esp_xtensa_info->sec_sz-1)) +
						esp_xtensa_info->sec_sz;
			} else if (esp_xtensa_info->is_drom_address(flash_map.maps[i].load_addr)) {
				drom_flash_base = flash_map.maps[i].phy_addr &
					~(esp_xtensa_info->sec_sz-1);
				drom_base = flash_map.maps[i].load_addr &
					~(esp_xtensa_info->sec_sz-1);
				drom_sz = flash_map.maps[i].size;
				if (drom_sz & (esp_xtensa_info->sec_sz-1))
					drom_sz = (drom_sz & ~(esp_xtensa_info->sec_sz-1)) +
						esp_xtensa_info->sec_sz;
			}
		}
	}

	if (strcmp(bank->name + strlen(target_name(bank->target)), ".irom") == 0) {
		esp_xtensa_info->hw_flash_base = irom_flash_base;
		bank->base = irom_base;
		bank->size = irom_sz;
	} else if (strcmp(bank->name + strlen(target_name(bank->target)), ".drom") == 0) {
		esp_xtensa_info->hw_flash_base = drom_flash_base;
		bank->base = drom_base;
		bank->size = drom_sz;
	} else {
		esp_xtensa_info->hw_flash_base = 0;
		bank->size = esp_xtensa_get_size(bank);
		if (bank->size == 0) {
			LOG_ERROR("Failed to probe flash, size %d KB", bank->size/1024);
			return ERROR_FAIL;
		}
		LOG_INFO("Auto-detected flash bank '%s' size %d KB", bank->name, bank->size/1024);
	}
	LOG_INFO("Using flash bank '%s' size %d KB", bank->name, bank->size/1024);

	if (bank->size) {
		/* Bank size can be 0 for IRON/DROM emulated banks when there is no app in flash */
		bank->num_sectors = bank->size / esp_xtensa_info->sec_sz;
		bank->sectors = malloc(sizeof(struct flash_sector) * bank->num_sectors);
		if (bank->sectors == NULL) {
			LOG_ERROR("Failed to alloc mem for sectors!");
			return ERROR_FAIL;
		}
		for (int i = 0; i < bank->num_sectors; i++) {
			bank->sectors[i].offset = i*esp_xtensa_info->sec_sz;
			bank->sectors[i].size = esp_xtensa_info->sec_sz;
			bank->sectors[i].is_erased = -1;
			bank->sectors[i].is_protected = 0;
		}
	}
	LOG_DEBUG("allocated %d sectors", bank->num_sectors);
	esp_xtensa_info->probed = 1;

	return ERROR_OK;
}

int esp_xtensa_auto_probe(struct flash_bank *bank)
{
	struct esp_xtensa_flash_bank *esp_xtensa_info = bank->driver_priv;
	if (esp_xtensa_info->probed)
		return ERROR_OK;
	return esp_xtensa_probe(bank);
}

static int esp_xtensa_flash_bp_op_state_init(struct target *target,
	struct xtensa_algo_run_data *run,
	struct esp_xtensa_flash_bp_op_state *state)
{
	/* aloocate target buffer for temp storage of flash sections contents when modifying
	 * instruction */
	LOG_DEBUG("SEC_SIZE %d", state->esp_xtensa_info->sec_sz);
	int ret = target_alloc_alt_working_area(target,
		2*(state->esp_xtensa_info->sec_sz),
		&state->target_buf);
	if (ret != ERROR_OK) {
		LOG_ERROR("Failed to alloc target buffer for insn sectors!");
		return ret;
	}
	buf_set_u32(run->priv.stub.reg_params[XTENSA_STUB_ARGS_FUNC_START+3].value,
		0,
		32,
		state->target_buf->address);										/*
															 * insn
															 * sectors */

	return ERROR_OK;
}

static void esp_xtensa_flash_bp_op_state_cleanup(struct target *target,
	struct xtensa_algo_run_data *run,
	struct esp_xtensa_flash_bp_op_state *state)
{
	if (!state->target_buf)
		return;
	target_free_alt_working_area(target, state->target_buf);
}

int esp_xtensa_flash_breakpoint_add(struct target *target,
	struct breakpoint *breakpoint,
	struct esp_xtensa_special_breakpoint *sw_bp)
{
	struct esp_xtensa_flash_bank *esp_xtensa_info;
	struct xtensa_algo_run_data run;
	struct flash_bank *bank;
	struct esp_xtensa_flash_bp_op_state op_state;
	struct mem_param mp;
	struct esp_xtensa_common *esp_xtensa = target_to_esp_xtensa(target);
	struct xtensa_algo_image flasher_image;

	/* flash belongs to root target, so we need to find flash using it instead of core
	 * sub-target */
	int ret = get_flash_bank_by_addr(esp_xtensa->chip_target, breakpoint->address, true, &bank);
	if (ret != ERROR_OK) {
		LOG_ERROR("%s: Failed to get flash bank (%d)!", target_name(target), ret);
		return ret;
	}

	/* can set set breakpoints in mapped app regions only */
	if (strcmp(bank->name + strlen(target_name(bank->target)), ".irom") != 0) {
		LOG_ERROR("%s: Can not set BP outside of IROM (BP addr " TARGET_ADDR_FMT ")!",
			target_name(target),
			breakpoint->address);
		return ERROR_FAIL;
	}

	esp_xtensa_info = bank->driver_priv;

	op_state.esp_xtensa_info = esp_xtensa_info;
	LOG_DEBUG("SEC_SIZE %d", esp_xtensa_info->sec_sz);
	memset(&run, 0, sizeof(run));
	run.stack_size = 1300;
	run.usr_func_arg = &op_state;
	run.usr_func_init = (xtensa_algo_usr_func_init_t)esp_xtensa_flash_bp_op_state_init;
	run.usr_func_done = (xtensa_algo_usr_func_done_t)esp_xtensa_flash_bp_op_state_cleanup;

	sw_bp->data.oocd_bp = breakpoint;
	sw_bp->priv = bank;

	ret = esp_xtensa_flasher_image_init(&flasher_image, esp_xtensa_info->get_stub(bank));
	if (ret != ERROR_OK)
		return ret;

	init_mem_param(&mp, 2 /*2nd usr arg*/, 3 /*size in bytes*/, PARAM_IN);
	run.mem_args.params = &mp;
	run.mem_args.count = 1;
	uint32_t bp_flash_addr = esp_xtensa_info->hw_flash_base +
		(breakpoint->address - bank->base);
	ret = esp_xtensa_info->run_func_image(esp_xtensa->chip_target,
		&run,
		&flasher_image,
		4 /*args num*/,
		ESP_XTENSA_STUB_CMD_FLASH_BP_SET /*cmd*/,
		bp_flash_addr /*bp_addr*/,
		0 /*address to store insn*/,
		0 /*address to store insn sectors*/);
	if (ret != ERROR_OK) {
		LOG_ERROR("%s: Failed to run flasher stub (%d)!", target_name(target), ret);
		destroy_mem_param(&mp);
		return ret;
	}
	if (run.ret_code == 0) {
		LOG_ERROR("%s: Failed to set bp (%d)!", target_name(target), run.ret_code);
		destroy_mem_param(&mp);
		return ERROR_FAIL;
	}
	sw_bp->data.insn_sz = run.ret_code;
	memcpy(sw_bp->data.insn, mp.value, 3);
	destroy_mem_param(&mp);
	LOG_DEBUG(
		"%s: Placed flash SW breakpoint at " TARGET_ADDR_FMT
		", insn [%02x %02x %02x] %d bytes",
		target_name(target),
		breakpoint->address,
		sw_bp->data.insn[0],
		sw_bp->data.insn[1],
		sw_bp->data.insn[2],
		sw_bp->data.insn_sz);

	return ERROR_OK;
}

int esp_xtensa_flash_breakpoint_remove(struct target *target,
	struct esp_xtensa_special_breakpoint *sw_bp)
{
	struct flash_bank *bank = (struct flash_bank *)(sw_bp->priv);
	struct esp_xtensa_flash_bank *esp_xtensa_info = bank->driver_priv;
	struct xtensa_algo_run_data run;
	struct esp_xtensa_flash_bp_op_state op_state;
	struct mem_param mp;
	struct esp_xtensa_common *esp_xtensa = target_to_esp_xtensa(target);
	struct xtensa_algo_image flasher_image;

	int ret = esp_xtensa_flasher_image_init(&flasher_image, esp_xtensa_info->get_stub(bank));
	if (ret != ERROR_OK)
		return ret;

	op_state.esp_xtensa_info = esp_xtensa_info;
	memset(&run, 0, sizeof(run));
	run.stack_size = 1300;
	run.usr_func_arg = &op_state;
	run.usr_func_init = (xtensa_algo_usr_func_init_t)esp_xtensa_flash_bp_op_state_init;
	run.usr_func_done = (xtensa_algo_usr_func_done_t)esp_xtensa_flash_bp_op_state_cleanup;

	init_mem_param(&mp, 2 /*2nd usr arg*/, 3 /*size in bytes*/, PARAM_OUT);
	memcpy(mp.value, sw_bp->data.insn, 3);
	run.mem_args.params = &mp;
	run.mem_args.count = 1;

	uint32_t bp_flash_addr = esp_xtensa_info->hw_flash_base +
		(sw_bp->data.oocd_bp->address - bank->base);
	LOG_DEBUG(
		"%s: Remove flash SW breakpoint at " TARGET_ADDR_FMT
		", insn [%02x %02x %02x] %d bytes",
		target_name(target),
		sw_bp->data.oocd_bp->address,
		sw_bp->data.insn[0],
		sw_bp->data.insn[1],
		sw_bp->data.insn[2],
		sw_bp->data.insn_sz);
	ret = esp_xtensa_info->run_func_image(esp_xtensa->chip_target,
		&run,
		&flasher_image,
		4 /*args num*/,
		ESP_XTENSA_STUB_CMD_FLASH_BP_CLEAR /*cmd*/,
		bp_flash_addr /*bp_addr*/,
		0 /*address with insn*/,
		0 /*address to store insn sectors*/);
	destroy_mem_param(&mp);
	if (ret != ERROR_OK) {
		LOG_ERROR("Failed to run flasher stub (%d)!", ret);
		return ret;
	}
	if (run.ret_code != ESP_XTENSA_STUB_ERR_OK) {
		LOG_ERROR("Failed to clear bp (%d)!", run.ret_code);
		return ERROR_FAIL;
	}

	sw_bp->data.oocd_bp = NULL;
	return ret;
}

static int esp_xtensa_appimage_flash_base_update(struct target *target,
	char *bank_name_suffix,
	uint32_t appimage_flash_base)
{
	struct flash_bank *bank;
	struct esp_xtensa_flash_bank *esp_xtensa_info;
	char bank_name[64];

	int ret = snprintf(bank_name,
		sizeof(bank_name),
		"%s.%s",
		target_name(target),
		bank_name_suffix);
	if (ret == sizeof(bank_name)) {
		LOG_ERROR("Failed to build bank name string!");
		return ERROR_FAIL;
	}
	ret = get_flash_bank_by_name(bank_name, &bank);
	if (ret != ERROR_OK)
		return ret;
	esp_xtensa_info = (struct esp_xtensa_flash_bank *)bank->driver_priv;
	esp_xtensa_info->probed = 0;
	esp_xtensa_info->appimage_flash_base = appimage_flash_base;
	ret = bank->driver->auto_probe(bank);
	if (ret != ERROR_OK)
		return ret;
	return ERROR_OK;
}

COMMAND_HANDLER(esp_xtensa_cmd_appimage_flashoff)
{
	struct target *target = get_current_target(CMD_CTX);
	if (CMD_ARGC != 1) {
		command_print(CMD, "Flash offset not specified!");
		return ERROR_FAIL;
	}

	/* update app image base */
	uint32_t appimage_flash_base = strtoul(CMD_ARGV[0], NULL, 16);

	int ret = esp_xtensa_appimage_flash_base_update(target, "irom", appimage_flash_base);
	if (ret != ERROR_OK)
		return ret;
	ret = esp_xtensa_appimage_flash_base_update(target, "drom", appimage_flash_base);
	if (ret != ERROR_OK)
		return ret;

	return ERROR_OK;
}

const struct command_registration esp_xtensa_exec_command_handlers[] = {
	{
		.name = "appimage_offset",
		.handler = esp_xtensa_cmd_appimage_flashoff,
		.mode = COMMAND_ANY,
		.help =
			"Set offset of application image in flash. Use -1 to debug the first application image from partition table.",
		.usage = "offset",
	},
	COMMAND_REGISTRATION_DONE
};
