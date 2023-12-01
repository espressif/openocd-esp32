/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   ESP32 flasher stub definitions                                        *
 *   Copyright (C) 2023 Espressif Systems Ltd.                             *
 ***************************************************************************/
#ifndef OPENOCD_LOADERS_FLASH_ESPRESSIF_ESP32_STUB_FLASHER_CHIP_H
#define OPENOCD_LOADERS_FLASH_ESPRESSIF_ESP32_STUB_FLASHER_CHIP_H

#include <esp32/rom/spi_flash.h>
#include <esp32/rom/miniz.h>

#include <stub_xtensa_common.h>

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

#endif /* OPENOCD_LOADERS_FLASH_ESPRESSIF_ESP32_STUB_FLASHER_CHIP_H */
