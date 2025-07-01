/* SPDX-License-Identifier: Apache-2.0 OR MIT */
/* SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD */

#pragma once

#include <stdint.h>
#include <stddef.h>

#define APPTRACE_MEM_BLOCK_NUM 2

/**
* @brief Memory block configuration for tracing
*
* This structure defines the memory blocks used for tracing data transfer between
* the target and host. Each block represents a fixed-size memory region where:
* - One block is used by the target to write trace data
* - The other block is read by the host
*
* The blocks are swapped when both conditions below are met:
* - The current write block is full
* - The host has finished reading the other block
*
* This dual-buffer approach allows for continuous tracing without data loss,
* as the target can write to one block while the host reads from the other.
*
* @note This structure should be packed, because it is read from the host
*/
struct apptrace_mem_block {
	uint8_t *start;
	uint16_t sz;
	uint16_t wr_pos;
};

/**
* @brief Data structure for managing the ring buffer
*
*  @data pointer to data storage
*  @size size of data storage
*  @cur_size current size of data storage
*  @rd read pointer
*  @wr write pointer
*/
struct apptrace_mem_rb {
	uint8_t *data;
	uint32_t size;
	uint32_t cur_size;
	uint32_t rd;
	uint32_t wr;
};

/**
*  @brief Data structure for managing the state of the dual buffer system in apptrace
*
*  The system uses two memory blocks that are switched between CPU and host access.
*  @blocks array tracks the fill level of each block, allowing the system to know
*  how much data has been written to each block.
*  @rb_down ring buffer for downlink data (from host to target)
*/
struct apptrace_mem_ctrl {
	uint16_t current_block;
	struct apptrace_mem_block blocks[APPTRACE_MEM_BLOCK_NUM];
	struct apptrace_mem_rb rb_down;
	int initialized;
};

/**
* @brief Initialize the downlink ring buffer
*/
void apptrace_memory_rb_init(uint8_t *data, uint32_t size);

/**
* @brief Initialize trace memory blocks
*/
struct apptrace_mem_ctrl *apptrace_memory_init(struct apptrace_mem_block *mem_block);

/**
* @brief Get the downlink buffer filled with host data
*/
uint8_t *apptrace_memory_downlink_get(uint32_t *size);

/**
* @brief Nothing to do for downlink buffer
*/
int apptrace_memory_downlink_put(uint8_t *buf);

/**
* @brief Get the uplink buffer to be filled by user data
*/
uint8_t *apptrace_memory_uplink_get(uint32_t size);

/**
* @brief Fill user data header with actual written size
*/
void apptrace_memory_uplink_put(uint8_t *buf);

/**
* @brief Swap the current block
*/
int apptrace_memory_flush(uint32_t min_sz);

/**
* @brief Get the maximum user data size
*/
uint16_t apptrace_memory_get_max_user_data_size(void);
