/***************************************************************************
 *   ESP RISCV application tracing module for OpenOCD                      *
 *   Copyright (C) 2020 Espressif Systems Ltd.                             *
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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <helper/log.h>
#include <helper/bits.h>
#include "esp_riscv.h"
#include "esp_riscv_apptrace.h"

/* TRAX is disabled, so we use its registers for our own purposes */
/* | 31..XXXXXX..24 | 23 .(host_connect). 23 | 22 .(host_data). 22| 21..(block_id)..15 |
 * 14..(block_len)..0 | */
#define RISCV_APPTRACE_BLOCK_ID_MSK             0x7FUL
#define RISCV_APPTRACE_BLOCK_ID_MAX             RISCV_APPTRACE_BLOCK_ID_MSK

#define RISCV_APPTRACE_BLOCK_LEN_MSK            0x7FFFUL
#define RISCV_APPTRACE_BLOCK_LEN(_l_)           ((_l_) & RISCV_APPTRACE_BLOCK_LEN_MSK)
#define RISCV_APPTRACE_BLOCK_LEN_GET(_v_)  ((_v_) & RISCV_APPTRACE_BLOCK_LEN_MSK)
#define RISCV_APPTRACE_BLOCK_ID(_id_)    (((_id_) & RISCV_APPTRACE_BLOCK_ID_MSK) << 15)
#define RISCV_APPTRACE_BLOCK_ID_GET(_v_)        (((_v_) >> 15) & RISCV_APPTRACE_BLOCK_ID_MSK)
#define RISCV_APPTRACE_HOST_DATA                BIT(22)
#define RISCV_APPTRACE_HOST_CONNECT             BIT(23)

struct esp_apptrace_riscv_ctrl_regs {
	uint32_t ctrl;
	uint32_t stat;
};

struct esp_apptrace_riscv_ctrl_block {
	/* - Guard field. If this register is not zero then CPU is changing this struct and */
	/*   this guard field holds address of the instruction which application will execute when
	 * CPU finishes with those modifications. */
	struct esp_apptrace_riscv_ctrl_regs control;
	uint32_t mem_blocks_cfg_addr;	/* 4 bytes for target pointer */
};

static int esp_riscv_apptrace_status_reg_read(struct target *target, uint32_t *stat);
static int esp_riscv_apptrace_buffs_write(struct target *target,
	uint32_t bufs_num,
	uint32_t buf_sz[],
	const uint8_t *bufs[],
	uint32_t block_id,
	bool ack,
	bool data);

struct esp32_apptrace_hw esp_riscv_apptrace_hw = {
	.max_block_id = RISCV_APPTRACE_BLOCK_ID_MAX,
	.max_block_size_get = esp_riscv_apptrace_block_max_size_get,
	.status_reg_read = esp_riscv_apptrace_status_reg_read,
	.ctrl_reg_write = esp_riscv_apptrace_ctrl_reg_write,
	.ctrl_reg_read = esp_riscv_apptrace_ctrl_reg_read,
	.data_len_read = esp_riscv_apptrace_data_len_read,
	.data_read = esp_riscv_apptrace_data_read,
	.usr_block_max_size_get = esp_riscv_apptrace_usr_block_max_size_get,
	.buffs_write = esp_riscv_apptrace_buffs_write,
	.leave_trace_crit_section_start = NULL,
	.leave_trace_crit_section_stop = NULL
};


int esp_riscv_apptrace_info_init(struct target *target,
	target_addr_t ctrl_addr,
	target_addr_t *old_ctrl_addr)
{
	struct esp_riscv_common *esp_riscv = target_to_esp_riscv(target);
	uint32_t mem_cfg_addr;

	int res = target_read_u32(target,
		ctrl_addr + sizeof(struct esp_apptrace_riscv_ctrl_regs),
		&mem_cfg_addr);
	if (res != ERROR_OK) {
		LOG_ERROR("Failed to read control block @ "TARGET_ADDR_FMT "!", ctrl_addr);
		return res;
	}
	for (int i = 0; i < 2; i++) {
		LOG_DEBUG("memory block %d start @ 0x%x!", i, mem_cfg_addr);
		res =
			target_read_u32(target, mem_cfg_addr,
			&(esp_riscv->apptrace.mem_blocks[i].start));
		if (res != ERROR_OK) {
			LOG_ERROR("Failed to read memory blocks config @ 0x%x!", mem_cfg_addr);
			return res;
		}
		mem_cfg_addr += sizeof(uint32_t);
		LOG_DEBUG("memory block %d size @ 0x%x!", i, mem_cfg_addr);
		res = target_read_u32(target, mem_cfg_addr, &(esp_riscv->apptrace.mem_blocks[i].sz));
		if (res != ERROR_OK) {
			LOG_ERROR("Failed to read memory blocks config @ 0x%x!", mem_cfg_addr);
			return res;
		}
		mem_cfg_addr += sizeof(uint32_t);
	}
	/* TODO: add checks for memory blocks ranges */
	LOG_DEBUG("Detected memory blocks: [0] %d bytes @ 0x%x, [1] %d bytes @ 0x%x",
		esp_riscv->apptrace.mem_blocks[0].sz, esp_riscv->apptrace.mem_blocks[0].start,
		esp_riscv->apptrace.mem_blocks[1].sz, esp_riscv->apptrace.mem_blocks[1].start);

	if (old_ctrl_addr)
		*old_ctrl_addr = esp_riscv->apptrace.ctrl_addr;
	esp_riscv->apptrace.ctrl_addr = ctrl_addr;
	return ERROR_OK;
}

uint32_t esp_riscv_apptrace_block_max_size_get(struct target *target)
{
	struct esp_riscv_common *esp_riscv = target_to_esp_riscv(target);
	return esp_riscv->apptrace.mem_blocks[0].sz;	/* assume blocks are of equal size */
}

uint32_t esp_riscv_apptrace_usr_block_max_size_get(struct target *target)
{
	return (esp_riscv_apptrace_block_max_size_get(target) -
		sizeof(struct esp_apptrace_host2target_hdr));
}

int esp_riscv_apptrace_usr_block_write(struct target *target,
	uint32_t block_id,
	const uint8_t *data,
	uint32_t size)
{
	return esp_apptrace_usr_block_write(&esp_riscv_apptrace_hw,
		target, block_id, data, size);
}

int esp_riscv_apptrace_data_len_read(struct target *target,
	uint32_t *block_id,
	uint32_t *len)
{
	return esp_riscv_apptrace_ctrl_reg_read(target, block_id, len, NULL);
}

int esp_riscv_apptrace_data_read(struct target *target,
	uint32_t size,
	uint8_t *buffer,
	uint32_t block_id,
	bool ack)
{
	struct esp_riscv_common *esp_riscv = target_to_esp_riscv(target);
	int blk_idx = block_id % 2 ? 0 : 1;

	int res = target_read_buffer(target,
		esp_riscv->apptrace.mem_blocks[blk_idx].start,
		size,
		buffer);
	if (res != ERROR_OK)
		return res;
	if (ack) {
		res = esp_riscv_apptrace_ctrl_reg_write(target,
			block_id,
			0 /*all data were read*/,
			true /*host connected*/,
			false /*no host data*/);
	}
	return res;
}

int esp_riscv_apptrace_ctrl_reg_read(struct target *target,
	uint32_t *block_id,
	uint32_t *len,
	bool *conn)
{
	struct esp_riscv_common *esp_riscv = target_to_esp_riscv(target);
	uint32_t ctrl;

	int res = target_read_u32(target, esp_riscv->apptrace.ctrl_addr, &ctrl);
	if (res != ERROR_OK) {
		LOG_ERROR("Failed to read control block @ "TARGET_ADDR_FMT "!",
			esp_riscv->apptrace.ctrl_addr);
		return res;
	}
	if (block_id)
		*block_id = RISCV_APPTRACE_BLOCK_ID_GET(ctrl);
	if (len)
		*len = RISCV_APPTRACE_BLOCK_LEN_GET(ctrl);
	if (conn)
		*conn = ctrl & RISCV_APPTRACE_HOST_CONNECT;

	return ERROR_OK;
}

int esp_riscv_apptrace_ctrl_reg_write(struct target *target,
	uint32_t block_id,
	uint32_t len,
	bool conn,
	bool data)
{
	struct esp_riscv_common *esp_riscv = target_to_esp_riscv(target);
	uint32_t ctrl = (conn ? RISCV_APPTRACE_HOST_CONNECT : 0) |
		(data ? RISCV_APPTRACE_HOST_DATA : 0) | RISCV_APPTRACE_BLOCK_ID(block_id) |
		RISCV_APPTRACE_BLOCK_LEN(len);
	return target_write_u32(target, esp_riscv->apptrace.ctrl_addr, ctrl);
}

static int esp_riscv_apptrace_status_reg_read(struct target *target, uint32_t *stat)
{
	struct esp_riscv_common *esp_riscv = target_to_esp_riscv(target);
	int res = target_read_u32(target, esp_riscv->apptrace.ctrl_addr + sizeof(uint32_t), stat);
	if (res != ERROR_OK)
		return res;
	return ERROR_OK;
}

static int esp_riscv_apptrace_buffs_write(struct target *target,
	uint32_t bufs_num,
	uint32_t buf_sz[],
	const uint8_t *bufs[],
	uint32_t block_id,
	bool ack,
	bool data)
{
	int res;
	int blk_idx = block_id % 2 ? 0 : 1;
	struct esp_riscv_common *esp_riscv = target_to_esp_riscv(target);

	LOG_DEBUG("Block ID %d block @ 0x%x",
		block_id,
		esp_riscv->apptrace.mem_blocks[blk_idx].start);
	for (uint32_t i = 0, curr_addr = esp_riscv->apptrace.mem_blocks[blk_idx].start;
		i < bufs_num; i++) {
		res = target_write_buffer(target, curr_addr, buf_sz[i], bufs[i]);
		if (res != ERROR_OK)
			return res;
		curr_addr += buf_sz[i];
	}
	if (ack) {
		res = esp_riscv_apptrace_ctrl_reg_write(target,
			block_id,
			0 /*all data were read*/,
			true /*host connected*/,
			data);
	}
	return res;
}
