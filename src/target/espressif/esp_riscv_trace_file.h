/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   RISC-V trace encoder control for Espressif chips                      *
 *   Copyright (C) 2026 Espressif Systems Ltd.                             *
 ***************************************************************************/

#ifndef OPENOCD_TARGET_ESP_RISCV_TRACE_FILE_H
#define OPENOCD_TARGET_ESP_RISCV_TRACE_FILE_H

#include <stdint.h>
#include <target/target.h>
#include "esp_riscv_trace_desc.h"

/* .esptrace END record status */
#define ESPTRACE_END_OK             0
#define ESPTRACE_END_FIFO_NOT_EMPTY 1

int esp_riscv_trace_file_write(struct target *target, const struct esp_riscv_trace_target_state *state,
	const char *path, uint32_t core_mask, uint32_t end_status, uint32_t *written_mask, uint32_t *wrapped_mask);

#endif	/* OPENOCD_TARGET_ESP_RISCV_TRACE_FILE_H */
