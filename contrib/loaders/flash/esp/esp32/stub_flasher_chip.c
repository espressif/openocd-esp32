/***************************************************************************
 *   ESP32 specific flasher stub functions                                 *
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

#include <stdlib.h>
#include "soc/rtc_cntl_reg.h"
#include "soc/rtc.h"
#include "soc/efuse_reg.h"
#include "soc/dport_reg.h"
#include "soc/spi_reg.h"
#include "soc/gpio_reg.h"
#include "rom/spi_flash.h"
#include "rom/cache.h"
#include "rom/efuse.h"
#include "stub_flasher_int.h"
#include "stub_flasher_chip.h"


#define ESP32_STUB_FLASH_STATE_SPI_USER_REG_VAL     0x80000040
#define ESP32_STUB_FLASH_STATE_SPI_USER1_REG_VAL    0x5c000007	/*0x8c000007 ? */
#define ESP32_STUB_FLASH_STATE_SPI_USER2_REG_VAL    0x70000000
#define ESP32_STUB_FLASH_STATE_SPI_SLAVE_REG_VAL    0x00000200	/* 0x0 ?? */

#define ESP32_STUB_SPI_FLASH_RDID                   0x9FUL

#define PERIPHS_SPI_MOSI_DLEN_REG                   SPI_MOSI_DLEN_REG(1)
#define PERIPHS_SPI_MISO_DLEN_REG                   SPI_MISO_DLEN_REG(1)
#define SPI_USR2_DLEN_SHIFT                         SPI_USR_COMMAND_BITLEN_S

extern esp_rom_spiflash_chip_t g_rom_spiflash_chip;
extern uint8_t g_rom_spiflash_dummy_len_plus[];

/**
 * The following two functions are replacements for Cache_Read_Disable and Cache_Read_Enable
 * function in ROM. They are used to work around a bug where Cache_Read_Disable requires a call to
 * Cache_Flush before Cache_Read_Enable, even if cached data was not modified.
 */
static const uint32_t cache_mask  = DPORT_APP_CACHE_MASK_OPSDRAM | DPORT_APP_CACHE_MASK_DROM0 |
	DPORT_APP_CACHE_MASK_DRAM1 | DPORT_APP_CACHE_MASK_IROM0 |
	DPORT_APP_CACHE_MASK_IRAM1 | DPORT_APP_CACHE_MASK_IRAM0;

static void esp32_flash_disable_cache_for_cpu(uint32_t cpuid, uint32_t *saved_state)
{
	uint32_t ret = 0;
	if (cpuid == 0) {
		ret |= DPORT_GET_PERI_REG_BITS2(DPORT_PRO_CACHE_CTRL1_REG, cache_mask, 0);
		while (DPORT_GET_PERI_REG_BITS2(DPORT_PRO_DCACHE_DBUG0_REG, DPORT_PRO_CACHE_STATE,
				DPORT_PRO_CACHE_STATE_S) != 1) {
			;
		}
		DPORT_SET_PERI_REG_BITS(DPORT_PRO_CACHE_CTRL_REG, 1, 0, DPORT_PRO_CACHE_ENABLE_S);
	} else {
		ret |= DPORT_GET_PERI_REG_BITS2(DPORT_APP_CACHE_CTRL1_REG, cache_mask, 0);
		while (DPORT_GET_PERI_REG_BITS2(DPORT_APP_DCACHE_DBUG0_REG, DPORT_APP_CACHE_STATE,
				DPORT_APP_CACHE_STATE_S) != 1) {
			;
		}
		DPORT_SET_PERI_REG_BITS(DPORT_APP_CACHE_CTRL_REG, 1, 0, DPORT_APP_CACHE_ENABLE_S);
	}
	*saved_state = ret;
}

static void esp32_flash_restore_cache_for_cpu(uint32_t cpuid, uint32_t saved_state)
{
	if (cpuid == 0) {
		DPORT_SET_PERI_REG_BITS(DPORT_PRO_CACHE_CTRL_REG, 1, 1, DPORT_PRO_CACHE_ENABLE_S);
		DPORT_SET_PERI_REG_BITS(DPORT_PRO_CACHE_CTRL1_REG, cache_mask, saved_state, 0);
	} else {
		DPORT_SET_PERI_REG_BITS(DPORT_APP_CACHE_CTRL_REG, 1, 1, DPORT_APP_CACHE_ENABLE_S);
		DPORT_SET_PERI_REG_BITS(DPORT_APP_CACHE_CTRL1_REG, cache_mask, saved_state, 0);
	}
}

static bool esp32_flash_cache_enabled(uint32_t cpuid)
{
	bool result = false;

	if (cpuid == 0)
		result = (DPORT_REG_GET_BIT(DPORT_PRO_CACHE_CTRL_REG, DPORT_PRO_CACHE_ENABLE) != 0);
	else
		result = (DPORT_REG_GET_BIT(DPORT_APP_CACHE_CTRL_REG, DPORT_APP_CACHE_ENABLE) != 0);
	return result;
}

static inline uint32_t stub_get_coreid()
{
	int id;
	__asm__ volatile (
		"rsr.prid %0\n"
		" extui %0,%0,13,1"
		: "=r" (id));
	return id;
}

static uint32_t esp32_flash_exec_usr_cmd(uint32_t cmd)
{
	uint32_t status_value = ESP_ROM_SPIFLASH_BUSY_FLAG;

	while (ESP_ROM_SPIFLASH_BUSY_FLAG == (status_value & ESP_ROM_SPIFLASH_BUSY_FLAG)) {
		WRITE_PERI_REG(PERIPHS_SPI_FLASH_STATUS, 0);	/* clear register */
		WRITE_PERI_REG(PERIPHS_SPI_FLASH_CMD, SPI_USR | cmd);
		while (READ_PERI_REG(PERIPHS_SPI_FLASH_CMD) != 0) ;

		status_value = READ_PERI_REG(PERIPHS_SPI_FLASH_STATUS) &
			g_rom_spiflash_chip.status_mask;
	}

	return status_value;
}

static void esp32_flash_spi_wait_ready()
{
	uint32_t status_value = ESP_ROM_SPIFLASH_BUSY_FLAG;

	while (ESP_ROM_SPIFLASH_BUSY_FLAG == (status_value & ESP_ROM_SPIFLASH_BUSY_FLAG)) {
		WRITE_PERI_REG(PERIPHS_SPI_FLASH_STATUS, 0);	/* clear register */
		WRITE_PERI_REG(PERIPHS_SPI_FLASH_CMD, SPI_FLASH_RDSR);
		while (READ_PERI_REG(PERIPHS_SPI_FLASH_CMD) != 0) ;
		status_value = READ_PERI_REG(PERIPHS_SPI_FLASH_STATUS) &
			(g_rom_spiflash_chip.status_mask);
	}
}

static uint32_t esp32_flash_spi_cmd_run(uint32_t cmd,
	uint8_t data_bits[],
	uint32_t data_bits_num,
	uint32_t read_bits_num)
{
	uint32_t old_spi_usr = READ_PERI_REG(PERIPHS_SPI_FLASH_USRREG);
	uint32_t old_spi_usr2 = READ_PERI_REG(PERIPHS_SPI_FLASH_USRREG2);
	uint32_t flags = SPI_USR_COMMAND;

	esp32_flash_spi_wait_ready();

	if (read_bits_num > 0) {
		flags |= SPI_USR_MISO;
		WRITE_PERI_REG(PERIPHS_SPI_MISO_DLEN_REG, read_bits_num - 1);
	}
	if (data_bits_num > 0) {
		flags |= SPI_USR_MOSI;
		WRITE_PERI_REG(PERIPHS_SPI_MOSI_DLEN_REG, data_bits_num - 1);
	}

	WRITE_PERI_REG(PERIPHS_SPI_FLASH_USRREG, flags);
	WRITE_PERI_REG(PERIPHS_SPI_FLASH_USRREG2, (7 << SPI_USR2_DLEN_SHIFT) | cmd);
	if (data_bits_num == 0)
		WRITE_PERI_REG(PERIPHS_SPI_FLASH_C0, 0);
	else {
		for (uint32_t i = 0; i <= data_bits_num / 32; i += 32)
			WRITE_PERI_REG(PERIPHS_SPI_FLASH_C0 + i / 8,
				*((uint32_t *)&data_bits[i / 8]));
	}
	esp32_flash_exec_usr_cmd(0);
	uint32_t status = READ_PERI_REG(PERIPHS_SPI_FLASH_C0);
	/* restore some SPI controller registers */
	WRITE_PERI_REG(PERIPHS_SPI_FLASH_USRREG, old_spi_usr);
	WRITE_PERI_REG(PERIPHS_SPI_FLASH_USRREG2, old_spi_usr2);

	return status;
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
	ret = esp32_flash_spi_cmd_run(ESP32_STUB_SPI_FLASH_RDID, NULL, 0, 24);
	STUB_LOGD("Flash ID read %x\n", ret);
	return ret >> 16;
}

void stub_flash_cache_flush(void)
{
	Cache_Flush(0);
	Cache_Flush(1);
}

void stub_flash_state_prepare(struct stub_flash_state *state)
{
	uint32_t core_id = stub_get_coreid();
	/* TODO: generic support for multi-core (pass maximum number of cores as the first param) */
	uint32_t other_core_id = core_id == 0 ? 1 : 0;

	ets_efuse_read_op();

	uint32_t spiconfig = ets_efuse_get_spiconfig();
	uint32_t strapping = REG_READ(GPIO_STRAP_REG);
	/*  If GPIO1 (U0TXD) is pulled low and flash pin configuration is not set in efuse, assume
	 * HSPI flash mode (same as normal boot) */
	if (spiconfig == 0 && (strapping & 0x1c) == 0x08)
		spiconfig = 1;	/* HSPI flash mode */

	state->other_cache_enabled = esp32_flash_cache_enabled(other_core_id);
	if (state->other_cache_enabled) {
		esp32_flash_disable_cache_for_cpu(other_core_id,
			&state->cache_flags[other_core_id]);
		STUB_LOGI("Cache disable CPU%d: 0x%x %d\n", other_core_id,
			state->cache_flags[other_core_id],
			esp32_flash_cache_enabled(other_core_id));
	}
	esp32_flash_disable_cache_for_cpu(core_id, &state->cache_flags[core_id]);
	STUB_LOGI("Cache disable CPU%d: 0x%x %d\n",
		core_id,
		state->cache_flags[core_id],
		esp32_flash_cache_enabled(core_id));

	state->spi_regs[ESP32_STUB_FLASH_STATE_SPI_USER_REG_ID] = READ_PERI_REG(SPI_USER_REG(1));
	state->spi_regs[ESP32_STUB_FLASH_STATE_SPI_USER1_REG_ID] = READ_PERI_REG(SPI_USER1_REG(1));
	state->spi_regs[ESP32_STUB_FLASH_STATE_SPI_USER2_REG_ID] = READ_PERI_REG(SPI_USER2_REG(1));
	state->spi_regs[ESP32_STUB_FLASH_STATE_SPI_SLAVE_REG_ID] = READ_PERI_REG(SPI_SLAVE_REG(1));
	WRITE_PERI_REG(SPI_USER_REG(1), ESP32_STUB_FLASH_STATE_SPI_USER_REG_VAL);
	WRITE_PERI_REG(SPI_USER1_REG(1), ESP32_STUB_FLASH_STATE_SPI_USER1_REG_VAL);
	WRITE_PERI_REG(SPI_USER2_REG(1), ESP32_STUB_FLASH_STATE_SPI_USER2_REG_VAL);
	WRITE_PERI_REG(SPI_SLAVE_REG(1), ESP32_STUB_FLASH_STATE_SPI_SLAVE_REG_VAL);

	esp_rom_spiflash_attach(spiconfig, 0);
}

void stub_flash_state_restore(struct stub_flash_state *state)
{
	uint32_t core_id = stub_get_coreid();
	/* TODO: generic support for multi-core (pass maximum number of cores as the first param) */
	uint32_t other_core_id = core_id == 0 ? 1 : 0;

	WRITE_PERI_REG(SPI_USER_REG(1), state->spi_regs[ESP32_STUB_FLASH_STATE_SPI_USER_REG_ID]);
	WRITE_PERI_REG(SPI_USER1_REG(1), state->spi_regs[ESP32_STUB_FLASH_STATE_SPI_USER1_REG_ID]);
	WRITE_PERI_REG(SPI_USER2_REG(1), state->spi_regs[ESP32_STUB_FLASH_STATE_SPI_USER2_REG_ID]);
	WRITE_PERI_REG(SPI_SLAVE_REG(1), state->spi_regs[ESP32_STUB_FLASH_STATE_SPI_SLAVE_REG_ID]);
	if (state->other_cache_enabled) {
		esp32_flash_restore_cache_for_cpu(other_core_id, state->cache_flags[other_core_id]);
		STUB_LOGI("Cache restored CPU%d: 0x%x %d\n", other_core_id,
			state->cache_flags[other_core_id],
			esp32_flash_cache_enabled(other_core_id));
	}
	esp32_flash_restore_cache_for_cpu(core_id, state->cache_flags[core_id]);
	STUB_LOGI("Cache restored CPU%d: 0x%x %d\n",
		core_id,
		state->cache_flags[core_id],
		esp32_flash_cache_enabled(core_id));
}

#if STUB_LOG_LOCAL_LEVEL > STUB_LOG_NONE
void stub_clock_configure(void)
{
	/* Set CPU to 80MHz. Keep other clocks unmodified. */
	rtc_cpu_freq_t cpu_freq = RTC_CPU_FREQ_80M;

	/* On ESP32 rev 0, switching to 80MHz if clock was previously set to
	 * 240 MHz may cause the chip to lock up (see section 3.5 of the errata
	 * document). For rev. 0, switch to 240 instead if it was chosen in
	 * menuconfig.
	 */
	uint32_t chip_ver_reg = REG_READ(EFUSE_BLK0_RDATA3_REG);
	if ((chip_ver_reg & EFUSE_RD_CHIP_VER_REV1_M) == 0 &&
		CONFIG_ESP32_DEFAULT_CPU_FREQ_MHZ == 240)
		cpu_freq = RTC_CPU_FREQ_240M;

	/* uart_tx_wait_idle(CONFIG_CONSOLE_UART_NUM); */
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
