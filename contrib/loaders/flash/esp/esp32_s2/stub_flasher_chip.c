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
#include "soc/efuse_reg.h"
#include "soc/gpio_reg.h"
#include "rom/efuse.h"
#include "rom/cache.h"
#include "rom/spi_flash.h"
#include "stub_flasher_int.h"
#include "stub_flasher_chip.h"

extern esp_rom_spiflash_chip_t g_rom_spiflash_chip;

extern void spi_flash_attach(uint32_t spiconfig, uint32_t arg2);

static void esp32_s2_flash_disable_cache(uint32_t *saved_state)
{
	saved_state[0] = Cache_Suspend_ICache();
	if (!Cache_Drom0_Using_ICache())
		saved_state[1] = Cache_Suspend_DCache();
}

static void esp32_s2_flash_restore_cache(uint32_t *saved_state)
{
	Cache_Resume_ICache(saved_state[0]);
	if (!Cache_Drom0_Using_ICache())
		Cache_Resume_DCache(saved_state[1]);
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

#if STUB_LOG_LOCAL_LEVEL > STUB_LOG_NONE
void stub_clock_configure(void)
{
	/* Set CPU to 80MHz. Keep other clocks unmodified. */
	rtc_cpu_freq_t cpu_freq = RTC_CPU_FREQ_80M;

	rtc_clk_config_t clk_cfg = RTC_CLK_CONFIG_DEFAULT();
	clk_cfg.xtal_freq = CONFIG_ESP32_XTAL_FREQ;
	clk_cfg.cpu_freq = cpu_freq;
	clk_cfg.slow_freq = rtc_clk_slow_freq_get();
	clk_cfg.fast_freq = rtc_clk_fast_freq_get();
	rtc_clk_init(clk_cfg);
}
#endif

uint32_t esp_clk_cpu_freq(void)
{
	return (CONFIG_ESP32_DEFAULT_CPU_FREQ_MHZ * 1000000);
}
