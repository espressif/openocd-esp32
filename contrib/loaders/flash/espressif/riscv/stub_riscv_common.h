/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   RiscV specific flasher stub functions                                 *
 *   Copyright (C) 2023 Espressif Systems Ltd.                             *
 ***************************************************************************/
#ifndef OPENOCD_LOADERS_FLASH_ESPRESSIF_STUB_RISCV_COMMON_H
#define OPENOCD_LOADERS_FLASH_ESPRESSIF_STUB_RISCV_COMMON_H

#include <sdkconfig.h>

#ifndef SOC_MMU_PAGE_SIZE
#define SOC_MMU_PAGE_SIZE				CONFIG_MMU_PAGE_SIZE
#endif

uint32_t stub_flash_get_id(void);
uint32_t stub_get_break_insn(uint8_t insn_sz);
uint8_t stub_get_insn_size(uint8_t *insn);
uint8_t stub_get_max_insn_size(void);

struct stub_flash_state {
	uint32_t cache_flags[2];
	bool cache_enabled;
};

void stub_flash_state_prepare(struct stub_flash_state *state);
void stub_flash_state_restore(struct stub_flash_state *state);
uint64_t stub_get_time(void);

#endif /* OPENOCD_LOADERS_FLASH_ESPRESSIF_STUB_RISCV_COMMON_H */
