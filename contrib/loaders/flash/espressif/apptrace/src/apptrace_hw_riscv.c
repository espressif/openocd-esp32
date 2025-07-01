// SPDX-License-Identifier: Apache-2.0 OR MIT
// SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD

#include <stdint.h>
#include <stddef.h>
#include <inttypes.h>

#include "stub_logger.h"

#include "apptrace_hw.h"
#include "apptrace_err.h"

struct apptrace_riscv_ctrl_block {
	volatile uint32_t ctrl;
	uint32_t stat;
	struct apptrace_mem_block *mem_blocks;
};

static uint8_t *s_trace_mem;
static uint16_t s_trace_mem_sz;
static struct apptrace_riscv_ctrl_block s_apptrace_ctrl;

void apptrace_hw_init(void)
{
	struct apptrace_mem_block mem_blocks[APPTRACE_MEM_BLOCK_NUM] = {
		{
			.start = s_trace_mem,
			.sz = s_trace_mem_sz / 2,
		},
		{
			.start = s_trace_mem + s_trace_mem_sz / 2,
			.sz = s_trace_mem_sz / 2,
		}
	};

	STUB_LOGI("mem_blocks[0]: %x, %d\n", mem_blocks[0].start, mem_blocks[0].sz);
	STUB_LOGI("mem_blocks[1]: %x, %d\n", mem_blocks[1].start, mem_blocks[1].sz);

	s_apptrace_ctrl.mem_blocks = apptrace_memory_init(mem_blocks)->blocks;
}

void apptrace_hw_connect(void)
{
	/* imply that host is auto-connected */
	s_apptrace_ctrl.ctrl |= APPTRACE_HOST_CONNECT;
}

void apptrace_hw_set_trace_mem(uint8_t *trace_mem, uint16_t trace_mem_sz)
{
	STUB_LOGD("trace_mem: %x, trace_mem_sz: %d\n", trace_mem, trace_mem_sz);

	s_trace_mem = trace_mem;
	s_trace_mem_sz = trace_mem_sz;
}

int apptrace_hw_host_data_is_present(void)
{
	return (s_apptrace_ctrl.ctrl & APPTRACE_HOST_DATA) ? 1 : 0;
}

int apptrace_hw_downlink_put(uint8_t *buf)
{
	(void)buf;

	return APPTRACE_ERR_OK;
}

int apptrace_hw_swap_start(uint32_t current_block_id)
{
	STUB_LOGD("from current block id: %d (%d)\n", current_block_id % 2, current_block_id);

	uint32_t ctrl_reg = s_apptrace_ctrl.ctrl;
	uint32_t host_connected = APPTRACE_HOST_CONNECT & ctrl_reg;
	if (host_connected) {
		uint32_t block_id = APPTRACE_BLOCK_ID_GET(ctrl_reg);
		uint32_t block_len = APPTRACE_BLOCK_LEN_GET(ctrl_reg);
		/* When host finishes reading, block_len should be 0 and block_id should match the latest exposed block */
		if (block_len != 0 || block_id != (current_block_id & APPTRACE_BLOCK_ID_MSK)) {
			STUB_LOGW("Can not swap %x %d %d/%d\n", ctrl_reg, block_len, block_id, current_block_id);
			return APPTRACE_ERR_CANNOT_SWAP;
		}
	}
	return APPTRACE_ERR_OK;
}

int apptrace_hw_swap(int new_block_id, uint32_t prev_block_len)
{
	(void)new_block_id;
	(void)prev_block_len;

	/* nothing to do */
	return APPTRACE_ERR_OK;
}

int apptrace_hw_swap_end(uint32_t new_block_id, uint32_t prev_block_len)
{
	STUB_LOGD("new_block_id: %d, prev_block_len: %d\n", new_block_id, prev_block_len);

	uint32_t ctrl_reg = s_apptrace_ctrl.ctrl;
	uint32_t host_connected = APPTRACE_HOST_CONNECT & ctrl_reg;
	s_apptrace_ctrl.ctrl = APPTRACE_BLOCK_ID(new_block_id) |
		host_connected | APPTRACE_BLOCK_LEN(prev_block_len);
	return APPTRACE_ERR_OK;
}

uint16_t apptrace_hw_get_buf_size(void)
{
	/* No TRAX like module on RISCV. This buffer will be used as software TRAX buffer */
	return APPTRACE_BLOCK_SIZE;
}
