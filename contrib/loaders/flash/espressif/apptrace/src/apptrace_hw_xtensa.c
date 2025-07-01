// SPDX-License-Identifier: Apache-2.0 OR MIT
// SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD

#include <stdint.h>
#include <stddef.h>
#include <inttypes.h>

#include "stub_logger.h"
#include "trax_mem.h"

#include "apptrace_hw.h"
#include "apptrace_err.h"

/* Coming from the target linker script */
extern uint8_t apptrace_block0_org[];
extern uint8_t apptrace_block1_org[];

/* Rom function */
extern uint16_t crc16_le(uint16_t crc, const uint8_t *buf, uint32_t len);

#define APPTRACE_TRAX_CTRL_REG      (ERI_TRAX_DELAYCNT)
#define APPTRACE_TRAX_STAT_REG      (ERI_TRAX_TRIGGERPC)
#define APPTRACE_TRAX_CRC16_REG     (ERI_PERFMON_PM1)
#define APPTRACE_CRC_INDICATOR      (0xA55AU << 16)

#define TRAX_REG_READ(addr)       esp_stub_lib_trax_reg_read(addr)
#define TRAX_REG_WRITE(addr, val) esp_stub_lib_trax_reg_write(addr, val)
#define TRAX_MEM_ENABLE()         esp_stub_lib_trax_mem_enable()
#define TRAX_SEL_MEM_BLK(block)   esp_stub_lib_trax_select_mem_block(block)

static uint8_t * const s_trax_blocks[] = {
	apptrace_block0_org,
	apptrace_block1_org
};

static struct apptrace_mem_ctrl *s_mem_ctrl_ptr;

static void apptrace_trax_hw_init(void)
{
	// Stop trace, if any (on the current CPU)
	TRAX_REG_WRITE(ERI_TRAX_TRAXCTRL, TRAXCTRL_TRSTP);	/* stop trace */
	TRAX_REG_WRITE(ERI_TRAX_TRAXCTRL, TRAXCTRL_TMEN);	/* enable local trace memory */
	TRAX_REG_WRITE(APPTRACE_TRAX_CTRL_REG, APPTRACE_BLOCK_ID(0));
}

int apptrace_hw_host_data_is_present(void)
{
	return TRAX_REG_READ(APPTRACE_TRAX_CTRL_REG) & APPTRACE_HOST_DATA ? 1 : 0;
}

void apptrace_hw_init(void)
{
	struct apptrace_mem_block mem_blocks[APPTRACE_MEM_BLOCK_NUM] = {
		{
			.start = s_trax_blocks[0],
			.sz = APPTRACE_BLOCK_SIZE,
		},
		{
			.start = s_trax_blocks[1],
			.sz = APPTRACE_BLOCK_SIZE,
		}
	};

	STUB_LOGI("mem_blocks[0]: %x, %d\n", mem_blocks[0].start, mem_blocks[0].sz);
	STUB_LOGI("mem_blocks[1]: %x, %d\n", mem_blocks[1].start, mem_blocks[1].sz);

	s_mem_ctrl_ptr = apptrace_memory_init(mem_blocks);

	TRAX_MEM_ENABLE();
	TRAX_SEL_MEM_BLK(0);
	apptrace_trax_hw_init();
}

void apptrace_hw_connect(void)
{
	/* imply that host is auto-connected */
	uint32_t ctrl_reg = TRAX_REG_READ(APPTRACE_TRAX_CTRL_REG);
	TRAX_REG_WRITE(APPTRACE_TRAX_CTRL_REG, ctrl_reg | APPTRACE_HOST_CONNECT);
}

void apptrace_hw_set_trace_mem(uint8_t *trace_mem, uint16_t trace_mem_sz)
{
	(void)trace_mem;
	(void)trace_mem_sz;

	/* nothing to do for XTENSA. It has TRAX memory blocks */
}

int apptrace_hw_swap_start(uint32_t current_block_id)
{
	STUB_LOGD("swap start from current block id: %d\n", current_block_id);

	uint32_t ctrl_reg = TRAX_REG_READ(APPTRACE_TRAX_CTRL_REG);
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
	STUB_LOGD("swap to new block id: %d, prev_block_len: %d\n", new_block_id, prev_block_len);

	/* calculate CRC16 of the next block to be swapped */
	if (prev_block_len > 0) {
		const uint8_t *prev_block_start = s_trax_blocks[!new_block_id];
		uint16_t crc16 = crc16_le(0, prev_block_start, prev_block_len);
		TRAX_REG_WRITE(APPTRACE_TRAX_CRC16_REG, crc16 | APPTRACE_CRC_INDICATOR);
		STUB_LOGI("CRC16:%x %d @%x\n", crc16, prev_block_len, prev_block_start);
	}

	TRAX_SEL_MEM_BLK(new_block_id);

	return APPTRACE_ERR_OK;
}

int apptrace_hw_swap_end(uint32_t new_block_id, uint32_t prev_block_len)
{
	STUB_LOGD("new_block_id: %d, prev_block_len: %d\n", new_block_id, prev_block_len);

	uint32_t ctrl_reg = TRAX_REG_READ(APPTRACE_TRAX_CTRL_REG);
	uint32_t host_connected = APPTRACE_HOST_CONNECT & ctrl_reg;

	TRAX_REG_WRITE(APPTRACE_TRAX_CTRL_REG, APPTRACE_BLOCK_ID(new_block_id) |
				host_connected | APPTRACE_BLOCK_LEN(prev_block_len));
	return APPTRACE_ERR_OK;
}

uint16_t apptrace_hw_get_buf_size(void)
{
	/* Xtensa has memory blocks managed by TRAX module. There is no need to use software buffer */
	return 0;
}
