// SPDX-License-Identifier: Apache-2.0 OR MIT
// SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD

#include <stdint.h>

#include <esp-stub-lib/log.h>
#include <esp-stub-lib/err.h>
#include <esp-stub-lib/flash.h>
#include <esp-stub-lib/bit_utils.h>

#include "flash_test.h"

#define CHUNK_BUF_SZ 1024

static uint8_t s_chunk_buf[CHUNK_BUF_SZ];

static uint32_t count_chunks(uint32_t full_size)
{
	return (full_size + CHUNK_BUF_SZ - 1) / CHUNK_BUF_SZ;
}

static uint32_t remain_size(uint32_t chunk_i, uint32_t full_size)
{
	return ((chunk_i + 1) * CHUNK_BUF_SZ > full_size) ? full_size % CHUNK_BUF_SZ : CHUNK_BUF_SZ;
}

static int run_read_bench(const uint32_t start_addr, const uint32_t data_size)
{
	int rc = STUB_LIB_FAIL;
	for (uint32_t i = 0; i < count_chunks(data_size); ++i) {
		uint32_t addr = start_addr + i * CHUNK_BUF_SZ;
		uint32_t size = remain_size(i, data_size);
		rc = stub_lib_flash_read_buff(addr, s_chunk_buf, size);
		if (rc != STUB_LIB_OK)
			return rc;

		STUB_LOGD("bench read chunk: %x, size: %d\n", addr, size);
		for (uint32_t j = 0; j < MIN(size, 16); j++)
			STUB_LOG("%x ", s_chunk_buf[j]);
		STUB_LOG("\n");
	}
	STUB_LOGD("bench read all: %x, size: %d\n", start_addr, data_size);
	return rc;
}

int stub_flash_test(uint32_t start_addr, uint32_t data_size)
{
	return run_read_bench(start_addr, data_size);
}
