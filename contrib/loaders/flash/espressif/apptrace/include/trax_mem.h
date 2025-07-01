/* SPDX-License-Identifier: Apache-2.0 OR MIT */
/* SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD */

#pragma once

#include <stdint.h>

#include "stub_flasher_int.h"

#define ERI_DEBUG_OFFSET        0x100000
#define ERI_TRAX_OFFSET         (ERI_DEBUG_OFFSET + 0x0)
#define ERI_PERFMON_OFFSET      (ERI_DEBUG_OFFSET + 0x1000)
#define ERI_TRAX_TRAXCTRL       (ERI_TRAX_OFFSET + 0x4)
#define ERI_TRAX_TRIGGERPC      (ERI_TRAX_OFFSET + 0x14)
#define ERI_TRAX_DELAYCNT       (ERI_TRAX_OFFSET + 0x1C)

#define ERI_PERFMON_PM1         (ERI_PERFMON_OFFSET + 0x84) /* used in apptrace module to store CRC16 */

#define TRAXCTRL_TRSTP          BIT(1)  // Trace Stop. Make 1 to stop trace.
#define TRAXCTRL_TMEN           BIT(7)  // Trace Memory Enable. Always set.

#ifdef __cplusplus
extern "C" {
#endif

void stub_target_trax_mem_enable(void);
void stub_target_trax_select_mem_block(int block);

static inline uint32_t eri_read(int addr)
{
	uint32_t ret;

	asm volatile("RER %0,%1" : "=r"(ret) : "r"(addr));

	return ret;
}

static inline void eri_write(int addr, uint32_t data)
{
	asm volatile("WER %0,%1" :: "r"(data), "r"(addr));
}

/*
 * Enable the TRAX memory. For ESP32 only.
 */
static inline void esp_stub_lib_trax_mem_enable(void)
{
	stub_target_trax_mem_enable();
}

/*
 * Select the memory block (0 or 1) to use.
 */
static inline void esp_stub_lib_trax_select_mem_block(int block)
{
	stub_target_trax_select_mem_block(block);
}

/*
 * Read a TRAX register using ERI.
 */
static inline uint32_t esp_stub_lib_trax_reg_read(int addr)
{
	return eri_read(addr);
}

/*
 * Write a TRAX register using ERI.
 */
static inline void esp_stub_lib_trax_reg_write(int addr, uint32_t data)
{
	eri_write(addr, data);
}

#ifdef __cplusplus
}
#endif
