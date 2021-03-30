/***************************************************************************
 *   ESP32-S2 specific flasher stub functions                              *
 *   Copyright (C) 2019 Espressif Systems Ltd.                             *
 *   Author: Alexey Gerenkov <alexey@espressif.com>                        *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.           *
 ***************************************************************************/

#include "sdkconfig.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/rtc.h"
#include "soc/efuse_periph.h"
#include "soc/gpio_reg.h"
#include "esp32s2/spiram.h"
#include "stub_rom_chip.h"
#include "stub_flasher_int.h"
#include "stub_flasher_chip.h"

uint32_t g_stub_cpu_freq_hz = CONFIG_ESP32_DEFAULT_CPU_FREQ_MHZ * MHZ;

extern esp_rom_spiflash_chip_t g_rom_spiflash_chip;

extern void spi_flash_attach(uint32_t spiconfig, uint32_t arg2);

static void esp32_s2_flash_disable_cache(uint32_t *saved_state)
{
	saved_state[0] = Cache_Suspend_ICache();
}

static void esp32_s2_flash_restore_cache(uint32_t *saved_state)
{
	Cache_Resume_ICache(saved_state[0]);
}

uint32_t stub_flash_get_id(void)
{
	uint32_t ret;

	STUB_LOGD("flash %x, cs %x, bs %x, ss %x, ps %x, sm %x\n",
		g_rom_spiflash_chip.device_id,
		g_rom_spiflash_chip.chip_size,
		g_rom_spiflash_chip.block_size,
		g_rom_spiflash_chip.sector_size,
		g_rom_spiflash_chip.page_size,
		g_rom_spiflash_chip.status_mask);
	WRITE_PERI_REG(PERIPHS_SPI_FLASH_C0, 0);/* clear regisrter */
	WRITE_PERI_REG(PERIPHS_SPI_FLASH_CMD, SPI_MEM_FLASH_RDID);
	while (READ_PERI_REG(PERIPHS_SPI_FLASH_CMD) != 0) ;
	ret = READ_PERI_REG(PERIPHS_SPI_FLASH_C0) & 0xffffff;
	STUB_LOGD("Flash ID read %x\n", ret);
	return ret >> 16;
}

void stub_flash_cache_flush(void)
{
	/* we do not know breakpoint program address here, so pass wrong addr to invalidate the
	 * whole ICache */
	Cache_Invalidate_ICache_Items(0, 4*1024*1024);
}

void stub_flash_state_prepare(struct stub_flash_state *state)
{
	uint32_t spiconfig = ets_efuse_get_spiconfig();
	uint32_t strapping = REG_READ(GPIO_STRAP_REG);
	/*  If GPIO1 (U0TXD) is pulled low and flash pin configuration is not set in efuse, assume
	 * HSPI flash mode (same as normal boot) */
	if (spiconfig == 0 && (strapping & 0x1c) == 0x08)
		spiconfig = 1;	/* HSPI flash mode */

	esp32_s2_flash_disable_cache(state->cache_flags);
	spi_flash_attach(spiconfig, 0);
}

void stub_flash_state_restore(struct stub_flash_state *state)
{
	esp32_s2_flash_restore_cache(state->cache_flags);
}

int stub_cpu_clock_configure(int cpu_freq_mhz)
{
	rtc_cpu_freq_config_t old_config;
	rtc_clk_cpu_freq_get_config(&old_config);

#if STUB_LOG_LOCAL_LEVEL > STUB_LOG_NONE
	uart_tx_wait_idle(CONFIG_CONSOLE_UART_NUM);
#endif

	/* set to maximum possible value */
	if (cpu_freq_mhz == -1)
		cpu_freq_mhz = CONFIG_ESP32_DEFAULT_CPU_FREQ_MHZ;

	/* Set CPU to configured value. Keep other clocks unmodified. */
	if (cpu_freq_mhz > 0) {
		rtc_clk_config_t clk_cfg = RTC_CLK_CONFIG_DEFAULT();
		/* ESP32-S2 doesn't have XTAL_FREQ choice, always 40MHz.
		   So using default value is fine */
		clk_cfg.cpu_freq_mhz = cpu_freq_mhz;
		clk_cfg.slow_freq = rtc_clk_slow_freq_get();
		clk_cfg.fast_freq = rtc_clk_fast_freq_get();
		rtc_clk_init(clk_cfg);

		g_stub_cpu_freq_hz = cpu_freq_mhz * MHZ;
	}

	return old_config.freq_mhz;
}

#if STUB_LOG_LOCAL_LEVEL > STUB_LOG_NONE
void stub_uart_console_configure(void)
{
	uartAttach(NULL);
	ets_install_uart_printf();
	/* Set configured UART console baud rate */
	uart_div_modify(CONFIG_CONSOLE_UART_NUM,
		(rtc_clk_apb_freq_get() << 4) / CONFIG_CONSOLE_UART_BAUDRATE);
}
#endif

esp_rom_spiflash_result_t esp_rom_spiflash_erase_area(uint32_t start_addr, uint32_t area_len)
{
	int32_t total_sector_num;
	int32_t head_sector_num;
	uint32_t sector_no;
	uint32_t sector_num_per_block;

	/* set read mode to Fastmode ,not QDIO mode for erase
	 *
	 * TODO: this is probably a bug as it doesn't re-enable QIO mode, not serious as this
	 * function is not used in IDF.
	 * esp_rom_spiflash_config_readmode(ESP_ROM_SPIFLASH_SLOWRD_MODE); */

	/* check if area is oversize of flash */
	if ((start_addr + area_len) > g_rom_spiflash_chip.chip_size)
		return ESP_ROM_SPIFLASH_RESULT_ERR;

	/* start_addr is aligned as sector boundary */
	if (0 != (start_addr % g_rom_spiflash_chip.sector_size))
		return ESP_ROM_SPIFLASH_RESULT_ERR;

	/* Unlock flash to enable erase */
	if (ESP_ROM_SPIFLASH_RESULT_OK != esp_rom_spiflash_unlock(/*&g_rom_spiflash_chip*/))
		return ESP_ROM_SPIFLASH_RESULT_ERR;

	sector_no = start_addr / g_rom_spiflash_chip.sector_size;
	sector_num_per_block = g_rom_spiflash_chip.block_size / g_rom_spiflash_chip.sector_size;
	total_sector_num =
		(0 ==
		(area_len %
			g_rom_spiflash_chip.sector_size)) ? area_len /
		g_rom_spiflash_chip.sector_size :
		1 + (area_len / g_rom_spiflash_chip.sector_size);

	/* check if erase area reach over block boundary */
	head_sector_num = sector_num_per_block - (sector_no % sector_num_per_block);

	head_sector_num =
		(head_sector_num >= total_sector_num) ? total_sector_num : head_sector_num;

	/* JJJ, BUG of 6.0 erase
	 * middle part of area is aligned by blocks */
	total_sector_num -= head_sector_num;

	/* head part of area is erased */
	while (0 != head_sector_num) {
		if (ESP_ROM_SPIFLASH_RESULT_OK != esp_rom_spiflash_erase_sector(sector_no))
			return ESP_ROM_SPIFLASH_RESULT_ERR;
		sector_no++;
		head_sector_num--;
	}
	while (total_sector_num > sector_num_per_block) {
		if (ESP_ROM_SPIFLASH_RESULT_OK !=
			esp_rom_spiflash_erase_block(sector_no / sector_num_per_block))
			return ESP_ROM_SPIFLASH_RESULT_ERR;
		sector_no += sector_num_per_block;
		total_sector_num -= sector_num_per_block;
	}

	/* tail part of area burn */
	while (0 < total_sector_num) {
		if (ESP_ROM_SPIFLASH_RESULT_OK != esp_rom_spiflash_erase_sector(sector_no))
			return ESP_ROM_SPIFLASH_RESULT_ERR;
		sector_no++;
		total_sector_num--;
	}

	return ESP_ROM_SPIFLASH_RESULT_OK;
}
