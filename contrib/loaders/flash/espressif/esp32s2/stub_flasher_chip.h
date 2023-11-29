/***************************************************************************
 *   ESP32-S2 flasher stub definitions                                     *
 *   Copyright (C) 2021 Espressif Systems Ltd.                             *
 *   Author: Alexey Gerenkov <alexey@espressif.com>                        *
 ***************************************************************************/
#ifndef ESP32_S2_FLASHER_STUB_H
#define ESP32_S2_FLASHER_STUB_H

#include <stdint.h>

#define STUB_FLASH_SECTOR_SIZE  4096
/* Flash geometry constants */
#define STUB_FLASH_BLOCK_SIZE   65536
#define STUB_FLASH_PAGE_SIZE    256
#define STUB_FLASH_STATUS_MASK  0xFFFF

struct stub_flash_state {
	uint32_t cache_flags[2];
	bool cache_enabled;
};
void stub_flash_state_prepare(struct stub_flash_state *state);
void stub_flash_state_restore(struct stub_flash_state *state);

uint32_t stub_esp_clk_cpu_freq(void);

#include "stub_xtensa_chips.h"

#endif	/*ESP32_S2_FLASHER_STUB_H */
