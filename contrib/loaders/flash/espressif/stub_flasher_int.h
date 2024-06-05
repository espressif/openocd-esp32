/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   ESP chips flasher stub internal definitions                           *
 *   Copyright (C) 2023 Espressif Systems Ltd.                             *
 ***************************************************************************/
#ifndef OPENOCD_LOADERS_FLASH_ESPRESSIF_STUB_FLASHER_INT_H
#define OPENOCD_LOADERS_FLASH_ESPRESSIF_STUB_FLASHER_INT_H

#include <stdint.h>

#ifndef MIN
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#endif

#ifndef ALIGN_UP_BY
#define ALIGN_UP_BY(num, a) (((num) + ((a) - 1)) & ~((a) - 1))
#endif

#ifndef BIT
#define BIT(nr) (1UL << (nr))
#endif

#ifndef MHZ
#define MHZ                             (1000000)
#endif

#define __maybe_unused  __attribute__((unused))

/* Flash geometry constants */
#define STUB_FLASH_SECTOR_SIZE          0x1000
#define STUB_FLASH_BLOCK_SIZE           0x10000
#define STUB_FLASH_PAGE_SIZE            0x100
#define STUB_FLASH_STATUS_MASK          0xFFFF

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

/* SPI Flash map request data */
struct spiflash_map_req {
	/* Request mapping SPI Flash base address */
	uint32_t src_addr;
	/* Request mapping SPI Flash size */
	uint32_t size;
	/* Mapped memory pointer */
	void *ptr;
	/* Mapped started MMU page index */
	uint32_t start_page;
	/* Mapped MMU page count */
	uint32_t page_cnt;
	/* Virtual addr */
	uint32_t vaddr_start;
	/* ID of the core currently executing this code */
	int core_id;
};

esp_flash_enc_mode_t stub_get_flash_encryption_mode(void);
void stub_flash_cache_flush(void);
void stub_uart_console_configure(int dest);
int stub_cpu_clock_configure(int cpu_freq_mhz);
uint32_t stub_esp_clk_cpu_freq(void);
int stub_flash_read_buff(uint32_t addr, void *buffer, uint32_t size);

#endif	/* OPENOCD_LOADERS_FLASH_ESPRESSIF_STUB_FLASHER_INT_H */
