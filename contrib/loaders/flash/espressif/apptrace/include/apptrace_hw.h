/* SPDX-License-Identifier: Apache-2.0 OR MIT */
/* SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD */

#pragma once

#include <stdint.h>

#include "stub_flasher_int.h"

#include "apptrace_mem_ctrl.h"

// Control Register Layout
// | 31..(unused)..24 | 23 .(host_connect). 23| 22..(host_data)..22 | 21..(block_id)..15 | 14..(block_len)..0 |

#define APPTRACE_BLOCK_LEN_MSK        0x7FFFUL
#define APPTRACE_BLOCK_LEN(_l_)       ((_l_) & APPTRACE_BLOCK_LEN_MSK)
#define APPTRACE_BLOCK_LEN_GET(_v_)   ((_v_) & APPTRACE_BLOCK_LEN_MSK)
#define APPTRACE_BLOCK_ID_MSK         0x7FUL
#define APPTRACE_BLOCK_ID(_id_)       (((_id_) & APPTRACE_BLOCK_ID_MSK) << 15)
#define APPTRACE_BLOCK_ID_GET(_v_)    (((_v_) >> 15) & APPTRACE_BLOCK_ID_MSK)
#define APPTRACE_HOST_DATA            BIT(22)
#define APPTRACE_HOST_CONNECT         BIT(23)

#define APPTRACE_BLOCK_SIZE           16384

static inline void apptrace_hw_prep_downlink(uint8_t *buf, uint32_t size)
{
	apptrace_memory_rb_init(buf, size);
}

static inline uint8_t *apptrace_hw_downlink_get(uint32_t *size)
{
	return apptrace_memory_downlink_get(size);
}

static inline uint8_t *apptrace_hw_uplink_get(uint32_t size)
{
	return apptrace_memory_uplink_get(size);
}

static inline void apptrace_hw_uplink_put(uint8_t *buf)
{
	apptrace_memory_uplink_put(buf);
}

static inline int apptrace_hw_flush(void)
{
	return apptrace_memory_flush(0);
}

static inline uint16_t apptrace_hw_get_max_user_data_size(void)
{
	return apptrace_memory_get_max_user_data_size();
}

/* Architecture specific functions */
void apptrace_hw_init(void);
void apptrace_hw_connect(void);
void apptrace_hw_set_trace_mem(uint8_t *trace_mem, uint16_t trace_mem_sz);
int apptrace_hw_host_data_is_present(void);
int apptrace_hw_swap_start(uint32_t current_block_id);
int apptrace_hw_swap(int new_block_id, uint32_t prev_block_len);
int apptrace_hw_swap_end(uint32_t new_block_id, uint32_t prev_block_len);
uint16_t apptrace_hw_get_buf_size(void);
