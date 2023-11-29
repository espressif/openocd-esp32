/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   ESP32 flasher stub definitions                                        *
 *   Copyright (C) 2019 Espressif Systems Ltd.                             *
 *   Author: Alexey Gerenkov <alexey@espressif.com>                        *
 ***************************************************************************/
#ifndef ESP32_FLASHER_STUB_H
#define ESP32_FLASHER_STUB_H

#include <stdint.h>

#define STUB_FLASH_SECTOR_SIZE  4096
/* Flash geometry constants */
#define STUB_FLASH_BLOCK_SIZE   65536
#define STUB_FLASH_PAGE_SIZE    256
#define STUB_FLASH_STATUS_MASK  0xFFFF

#define ESP32_STUB_FLASH_STATE_SPI_USER_REG_ID    0
#define ESP32_STUB_FLASH_STATE_SPI_USER1_REG_ID   1
#define ESP32_STUB_FLASH_STATE_SPI_USER2_REG_ID   2
#define ESP32_STUB_FLASH_STATE_SPI_SLAVE_REG_ID   3
#define ESP32_STUB_FLASH_STATE_SPI_CTRL_REG_ID    4
#define ESP32_STUB_FLASH_STATE_SPI_CLOCK_REG_ID   5
#define ESP32_STUB_FLASH_STATE_REGS_NUM           6

struct stub_flash_state {
	uint32_t cache_flags[2];
	bool other_cache_enabled;
	bool cache_enabled;
	uint32_t spi_regs[ESP32_STUB_FLASH_STATE_REGS_NUM];
	uint32_t dummy_len_plus;
};
void stub_flash_state_prepare(struct stub_flash_state *state);
void stub_flash_state_restore(struct stub_flash_state *state);

uint32_t stub_esp_clk_cpu_freq(void);

#include "stub_xtensa_chips.h"

#endif	/*ESP32_FLASHER_STUB_H */
