/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   ESP xtensa chips flasher stub internal definitions                    *
 *   Copyright (C) 2017 Espressif Systems Ltd.                             *
 ***************************************************************************/
#ifndef ESP_FLASHER_STUB_INT_H
#define ESP_FLASHER_STUB_INT_H

#include "stub_rom_chip.h"

#ifndef MIN
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#endif

#ifndef ALIGN_UP_BY
#define ALIGN_UP_BY(num, align) (((num) + ((align) - 1)) & ~((align) - 1))
#endif

/*
    Flash encryption mode based on efuse values
*/
typedef enum {
	/* flash encryption is not enabled (flash crypt cnt=0) */
	ESP_FLASH_ENC_MODE_DISABLED,
	/* flash encryption is enabled but for Development (reflash over UART allowed) */
	ESP_FLASH_ENC_MODE_DEVELOPMENT,
	/* flash encryption is enabled for Release (reflash over UART disabled) */
	ESP_FLASH_ENC_MODE_RELEASE
} esp_flash_enc_mode_t;
esp_flash_enc_mode_t stub_get_flash_encryption_mode(void);

extern uint32_t g_stub_cpu_freq_hz;

void stub_sha256_start(void);
void stub_sha256_data(const void *data, size_t data_len);
void stub_sha256_finish(uint8_t *digest);
uint32_t stub_flash_get_id(void);
void stub_flash_cache_flush(void);
void stub_uart_console_configure(int dest);
int stub_cpu_clock_configure(int cpu_freq_mhz);
uint32_t stub_esp_clk_cpu_freq(void);
void stub_print_cache_mmu_registers(void);
int stub_flash_read_buff(uint32_t addr, void *buffer, uint32_t size);

#endif	/*ESP_FLASHER_STUB_INT_H */
