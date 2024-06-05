/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   RiscV specific flasher stub functions                                 *
 *   Copyright (C) 2023 Espressif Systems Ltd.                             *
 ***************************************************************************/
#ifndef OPENOCD_LOADERS_FLASH_ESPRESSIF_STUB_RISCV_COMMON_H
#define OPENOCD_LOADERS_FLASH_ESPRESSIF_STUB_RISCV_COMMON_H

#include <sdkconfig.h>
#include <esp_app_trace_membufs_proto.h>

#define ESP_APPTRACE_USR_DATA_LEN_MAX   (CONFIG_APPTRACE_BUF_SIZE - 2)

#ifndef SOC_MMU_PAGE_SIZE
#define SOC_MMU_PAGE_SIZE				CONFIG_MMU_PAGE_SIZE
#endif

int esp_apptrace_advertise_ctrl_block(void *ctrl_block_addr);
int stub_apptrace_prepare(void);
uint32_t stub_flash_get_id(void);
uint32_t stub_get_break_insn(uint8_t insn_sz);
uint8_t stub_get_insn_size(uint8_t *insn);
uint8_t stub_get_max_insn_size(void);

#if CONFIG_STUB_STACK_DATA_POOL_SIZE > 0
void stub_stack_data_pool_init(uint8_t *data, size_t sz);
void esp_apptrace_get_up_buffers(esp_apptrace_mem_block_t mem_blocks_cfg[2]);
#endif

struct stub_flash_state {
	uint32_t cache_flags[2];
	bool cache_enabled;
};

void stub_flash_state_prepare(struct stub_flash_state *state);
void stub_flash_state_restore(struct stub_flash_state *state);
uint64_t stub_get_time(void);

#endif /* OPENOCD_LOADERS_FLASH_ESPRESSIF_STUB_RISCV_COMMON_H */
