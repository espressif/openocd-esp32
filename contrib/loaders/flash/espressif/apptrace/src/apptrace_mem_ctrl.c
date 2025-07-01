// SPDX-License-Identifier: Apache-2.0 OR MIT
// SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD

#include <stdint.h>
#include <stddef.h>
#include <string.h>

#include "stub_logger.h"
#include "stub_flasher_int.h"

#include "apptrace_mem_ctrl.h"
#include "apptrace_hw.h"
#include "apptrace_err.h"

#define FORCE_INLINE_ATTR static inline __attribute__((always_inline))

struct apptrace_user_header {
	uint16_t   block_sz; // size of allocated block for user (target) data
	uint16_t   wr_sz;    // size of actually written data
};

struct apptrace_host_header {
	uint16_t   block_sz; // size of host data
};

static struct apptrace_mem_ctrl s_mem_ctrl;

/*
* ============================================================================
*                                  RING BUFFER
* ============================================================================
*/

/**
* @brief Reserve a space to write data into the ring buffer.
*
* @param size The size of the data to produce
* @return uint8_t* Pointer to start of the reserved region or NULL if no space is available
*/
static uint8_t *apptrace_memory_rb_produce(uint32_t size)
{
	struct apptrace_mem_rb *rb = &s_mem_ctrl.rb_down;

	STUB_LOGD("wr:%d rd:%d cur_size:%d rb-size:%d data-size:%d\n", rb->wr, rb->rd, rb->cur_size, rb->size, size);

	uint8_t *ptr = rb->data + rb->wr;
	// check for available space
	if (rb->rd <= rb->wr) {
		// |?R......W??|
		if (rb->wr + size >= rb->size) {
			if (rb->rd == 0)
				return NULL; // cannot wrap wr
			if (rb->wr + size == rb->size) {
				rb->wr = 0;
				rb->cur_size = rb->size;
			} else {
				// check if we can wrap wr earlier to get space for requested size
				if (size > rb->rd - 1)
					return NULL; // cannot wrap wr
				// shrink buffer a bit, full size will be restored at rd wrapping
				rb->cur_size = rb->wr;
				rb->wr = 0;
				ptr = rb->data;
				if (rb->rd == rb->cur_size) {
					rb->rd = 0;
					if (rb->cur_size < rb->size)
						rb->cur_size = rb->size;
				}
				rb->wr += size;
			}
		} else {
			rb->wr += size;
		}
	} else {
		// |?W......R??|
		if (size > rb->rd - rb->wr - 1)
			return NULL;
		rb->wr += size;
	}
	return ptr;
}

/**
* @brief Read and remove (consume) a region of memory from the ring buffer
*
* @param size The size of the data to consume
* @return uint8_t* Pointer to the start of the region to be consumed or NULL if no data is available
*/
static uint8_t *apptrace_memory_rb_consume(uint32_t size)
{
	struct apptrace_mem_rb *rb = &s_mem_ctrl.rb_down;

	STUB_LOGD("wr:%d rd:%d cur_size:%d size:%d\n", rb->wr, rb->rd, rb->cur_size, size);

	uint8_t *ptr = rb->data + rb->rd;
	if (rb->rd <= rb->wr) {
		// |?R......W??|
		if (rb->rd + size > rb->wr)
			return NULL;
		rb->rd += size;
	} else {
		// |?W......R??|
		if (rb->rd + size > rb->cur_size) {
			return NULL;
		} else if (rb->rd + size == rb->cur_size) {
			// restore full size usage
			if (rb->cur_size < rb->size)
				rb->cur_size = rb->size;
			rb->rd = 0;
		} else {
			rb->rd += size;
		}
	}
	return ptr;
}

/**
* @brief Get the size of the data available to read from the ring buffer
*
* @return uint32_t The size of the data available to read
*/
static uint32_t apptrace_memory_rb_read_size_get(void)
{
	struct apptrace_mem_rb *rb = &s_mem_ctrl.rb_down;

	if (rb->rd <= rb->wr)
		return rb->wr - rb->rd;

	// |?W......R??|
	return rb->cur_size - rb->rd;
}

/**
* @brief Get the size of the data available to write to the ring buffer.
* Leave one slot empty to avoid wrap-around issues.
*
* @return uint32_t The size of the data available to write
*/
static uint32_t apptrace_memory_rb_write_size_get(void)
{
	struct apptrace_mem_rb *rb = &s_mem_ctrl.rb_down;
	uint32_t size = 0;

	if (rb->rd <= rb->wr) {
		// |?R......W??|
		size = rb->size - rb->wr;
		if (size && rb->rd == 0)
			size--;
	} else {
		// |?W......R??|
		size = rb->rd - rb->wr - 1;
	}
	return size;
}

void apptrace_memory_rb_init(uint8_t *data, uint32_t size)
{
	STUB_LOGD("data: %x, size: %d\n", data, size);

	s_mem_ctrl.rb_down.data = data;
	s_mem_ctrl.rb_down.size = size;
	s_mem_ctrl.rb_down.cur_size = size;
	s_mem_ctrl.rb_down.rd = 0;
	s_mem_ctrl.rb_down.wr = 0;
}

/*
* ============================================================================
*                          APPTRACE MEMORY CONTROL
* ============================================================================
*/
FORCE_INLINE_ATTR uint8_t apptrace_curr_block_num(void)
{
	return s_mem_ctrl.current_block;
}

FORCE_INLINE_ATTR uint32_t apptrace_curr_block_pos(void)
{
	return s_mem_ctrl.blocks[apptrace_curr_block_num()].wr_pos;
}

FORCE_INLINE_ATTR void apptrace_increment_block_pos(uint16_t value)
{
	s_mem_ctrl.blocks[apptrace_curr_block_num()].wr_pos += value;
}

FORCE_INLINE_ATTR struct apptrace_mem_block *apptrace_curr_block(void)
{
	return &s_mem_ctrl.blocks[apptrace_curr_block_num()];
}

FORCE_INLINE_ATTR uint16_t apptrace_usr_block_core(uint16_t core_id)
{
	return core_id << 15;
}

FORCE_INLINE_ATTR uint32_t apptrace_usr_block_len(uint16_t value)
{
	return ~BIT(15) & value;
}

FORCE_INLINE_ATTR uint16_t apptrace_usr_data_max_len(void)
{
	return (uint16_t)(apptrace_curr_block()->sz - sizeof(struct apptrace_user_header));
}

FORCE_INLINE_ATTR uint16_t apptrace_usr_block_raw_size(uint32_t size)
{
	return (uint16_t)(size + sizeof(struct apptrace_user_header));
}

static uint32_t apptrace_memory_downlink_write(uint8_t *data, uint32_t size)
{
	uint32_t total_sz = 0;

	while (total_sz < size) {
		uint32_t wr_sz = apptrace_memory_rb_write_size_get();
		if (wr_sz == 0)
			break;
		wr_sz = MIN(wr_sz, size - total_sz);
		uint8_t *ptr = apptrace_memory_rb_produce(wr_sz);
		if (!ptr) {
			STUB_LOGE("Failed to produce bytes to ring buffer!\n");
			return total_sz;
		}
		memcpy(ptr, data + total_sz, wr_sz);
		total_sz += wr_sz;
		STUB_LOGV("wrote %d, total %d\n", wr_sz, total_sz);
	}
	return total_sz;
}

static int apptrace_memory_swap_internal(void)
{
	int prev_block_num = s_mem_ctrl.current_block;
	int new_block_num = !prev_block_num;

	int res = apptrace_hw_swap_start(s_mem_ctrl.current_block);
	if (res != APPTRACE_ERR_OK)
		return res;

	s_mem_ctrl.blocks[new_block_num].wr_pos = 0;
	s_mem_ctrl.current_block = new_block_num;

	apptrace_hw_swap(new_block_num, s_mem_ctrl.blocks[prev_block_num].wr_pos);

	// handle data from host
	struct apptrace_host_header *host_data = (struct apptrace_host_header *)s_mem_ctrl.blocks[new_block_num].start;
	if (apptrace_hw_host_data_is_present() && host_data->block_sz > 0) {
		uint8_t *data = (uint8_t *)(host_data + 1);
		__attribute__((unused)) uint8_t *p = s_mem_ctrl.blocks[new_block_num].start +
			s_mem_ctrl.blocks[new_block_num].sz;
		STUB_LOGD("Recvd %d bytes from host (@ %x) [%x %x %x %x .. %x %x %x %x]\n",
			host_data->block_sz, s_mem_ctrl.blocks[new_block_num].start,
			data[0], data[1], data[2], data[3], *(p - 4), *(p - 3), *(p - 2), *(p - 1));
		uint32_t sz = apptrace_memory_downlink_write(data, host_data->block_sz);
		if (sz != host_data->block_sz) {
			STUB_LOGE("Failed to write %d bytes to ring buffer (%d %d)!\n",
				host_data->block_sz - sz, host_data->block_sz, sz);
		}
		host_data->block_sz = 0;
	}

	return apptrace_hw_swap_end(s_mem_ctrl.current_block, s_mem_ctrl.blocks[prev_block_num].wr_pos);
}

static int apptrace_memory_swap(void)
{
	while (apptrace_memory_swap_internal() != APPTRACE_ERR_OK) {
#if defined(CONFIG_IDF_TARGET_ESP32S3)
		/*
		* ESP32S3 has a serious data corruption issue with the transferred data to host.
		* This delay helps reduce the failure rate by temporarily reducing heavy memory writes
		* from RTOS-level tracing and giving OpenOCD more time to read trace memory before
		* the current thread continues execution. While this doesn't completely prevent
		* memory access from other threads/cores/ISRs, it has shown to significantly improve
		* reliability when combined with CRC checks in OpenOCD. In practice, this reduces the
		* number of retries needed to read an entire block without corruption.
		*/
		extern void ets_delay_us(uint32_t us);
		ets_delay_us(100); /* avoid busy-loop and wait for OpenOCD to read data */
#endif
	}

	return APPTRACE_ERR_OK;
}

static inline uint8_t *apptrace_memory_pkt_start(uint8_t *ptr, uint16_t size)
{
	/* It is safe to use core 0 because OpenOCD always runs the stub code from core 0 */
	((struct apptrace_user_header *)ptr)->block_sz = apptrace_usr_block_core(0) | size;
	((struct apptrace_user_header *)ptr)->wr_sz = 0;
	return ptr + sizeof(struct apptrace_user_header);
}

static inline void apptrace_memory_pkt_end(uint8_t *ptr)
{
	struct apptrace_user_header *hdr = (struct apptrace_user_header *)(ptr - sizeof(struct apptrace_user_header));
	hdr->wr_sz = hdr->block_sz;
}

struct apptrace_mem_ctrl *apptrace_memory_init(struct apptrace_mem_block *mem_block)
{
	/* disable by default */
	apptrace_memory_rb_init(NULL, 0);

	for (int i = 0; i < APPTRACE_MEM_BLOCK_NUM; i++) {
		s_mem_ctrl.blocks[i].start = mem_block[i].start;
		s_mem_ctrl.blocks[i].sz = mem_block[i].sz;
		s_mem_ctrl.blocks[i].wr_pos = 0;
	}
	s_mem_ctrl.current_block = 0;
	s_mem_ctrl.initialized = 1;

	return &s_mem_ctrl;
}

int apptrace_memory_downlink_put(uint8_t *buf)
{
	(void)buf;

	/* nothing to do */
	return APPTRACE_ERR_OK;
}

uint8_t *apptrace_memory_downlink_get(uint32_t *size)
{
	uint32_t sz = 0;
	while ((sz = apptrace_memory_rb_read_size_get()) == 0) {
		if (apptrace_hw_host_data_is_present()) {
			STUB_LOGD("force flush\n");
			int res = apptrace_memory_swap();
			if (res != APPTRACE_ERR_OK) {
				STUB_LOGE("Failed to switch to another block to recv data from host!\n");
				/* do not return error because data can be in down buffer already */
			}
		}
	}

	STUB_LOGD("downlink get %d bytes rd:%d wr:%d cur_size:%d\n",
		sz, s_mem_ctrl.rb_down.rd, s_mem_ctrl.rb_down.wr, s_mem_ctrl.rb_down.cur_size);
	*size = MIN(*size, sz);
	uint8_t *ptr = apptrace_memory_rb_consume(*size);
	if (!ptr)
		STUB_LOGE("Failed to consume bytes from down buffer!\n");

	return ptr;
}

uint8_t *apptrace_memory_uplink_get(uint32_t size)
{
	STUB_LOGD("size:%d\n", size);

	if (size > apptrace_usr_data_max_len()) {
		STUB_LOGE("Too large user data size %d!\n", size);
		return NULL;
	}

	STUB_LOGV("Block curr pos:%d total size:%d\n", apptrace_curr_block_pos(), apptrace_usr_block_raw_size(size));

	if (apptrace_curr_block_pos() + apptrace_usr_block_raw_size(size) > apptrace_curr_block()->sz) {
		if (apptrace_memory_swap() != APPTRACE_ERR_OK)
			return NULL;
	}

	uint8_t *buf_ptr = apptrace_curr_block()->start + apptrace_curr_block_pos();
	apptrace_increment_block_pos(apptrace_usr_block_raw_size(size));
	STUB_LOGV("Reserved uplink buffer %d bytes\n", size);

	return apptrace_memory_pkt_start(buf_ptr, (uint16_t)size);
}

void apptrace_memory_uplink_put(uint8_t *buf)
{
	apptrace_memory_pkt_end(buf);
}

int apptrace_memory_flush(uint32_t min_sz)
{
	int res = APPTRACE_ERR_OK;

	STUB_LOGD("apptrace flush min_sz: %d\n", min_sz);

	if (apptrace_curr_block_pos() < min_sz) {
		STUB_LOGI("Ignore flush request for min %d bytes. Bytes in block: %d\n",
			min_sz, apptrace_curr_block_pos());
		return APPTRACE_ERR_OK;
	}

	while (apptrace_curr_block_pos() > min_sz) {
		STUB_LOGD("Try to flush %d bytes\n", apptrace_curr_block_pos());
		res = apptrace_memory_swap();
		if (res != APPTRACE_ERR_OK) {
			STUB_LOGW("Failed to switch to another block!\n");
			break;
		}
	}

	return res;
}

uint16_t apptrace_memory_get_max_user_data_size(void)
{
	return apptrace_usr_data_max_len();
}
