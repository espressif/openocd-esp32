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
#include <string.h>
#include "soc/rtc_cntl_reg.h"
#include "soc/rtc.h"
#include "soc/efuse_periph.h"
#include "soc/dport_reg.h"
#include "soc/spi_reg.h"
#include "soc/gpio_reg.h"
#include "soc/mmu.h"
#include "esp_spi_flash.h"
#include "esp32/spiram.h"
#include "stub_rom_chip.h"
#include "stub_logger.h"
#include "stub_flasher_int.h"
#include "stub_flasher_chip.h"

uint32_t g_stub_cpu_freq_hz = CONFIG_ESP32_DEFAULT_CPU_FREQ_MHZ * MHZ;

#define ESP32_STUB_FLASH_STATE_SPI_USER_REG_VAL     0x80000040UL
#define ESP32_STUB_FLASH_STATE_SPI_USER1_REG_VAL    0x5c000007UL
#define ESP32_STUB_FLASH_STATE_SPI_USER2_REG_VAL    0x70000000UL
#define ESP32_STUB_FLASH_STATE_SPI_SLAVE_REG_VAL    0x00000200UL
#define ESP32_STUB_FLASH_STATE_SPI_CTRL_REG_VAL     0x208000UL
#define ESP32_STUB_FLASH_STATE_SPI_CLOCK_REG_VAL    0x3043UL

#define ESP32_STUB_SPI_FLASH_RDID                   0x9FUL

#define PERIPHS_SPI_MOSI_DLEN_REG                   SPI_MOSI_DLEN_REG(1)
#define PERIPHS_SPI_MISO_DLEN_REG                   SPI_MISO_DLEN_REG(1)
#define SPI_USR2_DLEN_SHIFT                         SPI_USR_COMMAND_BITLEN_S

/* Cache MMU related definitions */
#define STUB_CACHE_BUS_PRO              DPORT_PRO_CACHE_MASK_DROM0
#define STUB_CACHE_BUS_APP              DPORT_APP_CACHE_MASK_DROM0
#define STUB_MMU_DROM_VADDR             SOC_MMU_VADDR0_START_ADDR
#define STUB_MMU_DROM_PAGES_START       SOC_MMU_DROM0_PAGES_START	/* 0 */
#define STUB_MMU_DROM_PAGES_END         SOC_MMU_DROM0_PAGES_END		/* 64 */
#define STUB_PRO_MMU_TABLE              ((volatile uint32_t *)0x3FF10000)
#define STUB_APP_MMU_TABLE              ((volatile uint32_t *)0x3FF12000)
#define STUB_MMU_INVALID_ENTRY_VAL      SOC_MMU_INVALID_ENTRY_VAL	/* 0x100 */

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
	/* ID of the core currently executing this code */
	int core_id;
};

static volatile uint32_t *mmu_table_s[2] = { STUB_PRO_MMU_TABLE, STUB_APP_MMU_TABLE };

extern esp_rom_spiflash_chip_t g_rom_spiflash_chip;
extern uint8_t g_rom_spiflash_dummy_len_plus[];

void vPortEnterCritical(void *mux)
{
}

void vPortExitCritical(void *mux)
{
}

bool ets_efuse_flash_octal_mode(void)
{
	return false;
}

#if STUB_LOG_ENABLE == 1
void stub_print_cache_mmu_registers(void)
{
	uint32_t ctrl_reg = DPORT_READ_PERI_REG(DPORT_PRO_CACHE_CTRL_REG);
	uint32_t ctrl1_reg = DPORT_READ_PERI_REG(DPORT_PRO_CACHE_CTRL1_REG);
	uint32_t dbug0_reg = DPORT_READ_PERI_REG(DPORT_PRO_DCACHE_DBUG0_REG);

	STUB_LOGD("pro ctrl_reg: 0x%x ctrl1_reg: 0x%x dbug0_reg: 0x%x\n",
		ctrl_reg,
		ctrl1_reg,
		dbug0_reg);

	ctrl_reg = DPORT_READ_PERI_REG(DPORT_APP_CACHE_CTRL_REG);
	ctrl1_reg = DPORT_READ_PERI_REG(DPORT_APP_CACHE_CTRL1_REG);
	dbug0_reg = DPORT_READ_PERI_REG(DPORT_APP_DCACHE_DBUG0_REG);

	STUB_LOGD("app ctrl_reg: 0x%x ctrl1_reg: 0x%x dbug0_reg: 0x%x\n",
		ctrl_reg,
		ctrl1_reg,
		dbug0_reg);
}
#endif

static const uint32_t cache_mask = DPORT_APP_CACHE_MASK_OPSDRAM | DPORT_APP_CACHE_MASK_DROM0 |
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

static void stub_cache_init(int cpuid)
{
	Cache_Read_Disable(cpuid);
	Cache_Flush(cpuid);
	/* we are mapping external flash to the DROM so don't need to enable other buses */
	if (cpuid == 0)
		DPORT_REG_CLR_BIT(DPORT_PRO_CACHE_CTRL1_REG, STUB_CACHE_BUS_PRO);
	else
		DPORT_REG_CLR_BIT(DPORT_APP_CACHE_CTRL1_REG, STUB_CACHE_BUS_APP);
	Cache_Read_Enable(cpuid);
}

static bool esp32_flash_cache_bus_enabled(uint32_t cpuid)
{
	uint32_t cache_bus = 0;
	uint32_t cache_mask = 0;

	if (cpuid == 0) {
		cache_bus = DPORT_READ_PERI_REG(DPORT_PRO_CACHE_CTRL1_REG);
		cache_mask = STUB_CACHE_BUS_PRO;
	} else {
		cache_bus = DPORT_READ_PERI_REG(DPORT_APP_CACHE_CTRL1_REG);
		cache_mask = STUB_CACHE_BUS_APP;
	}

	return !(cache_bus & cache_mask);
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
	if (data_bits_num == 0) {
		WRITE_PERI_REG(PERIPHS_SPI_FLASH_C0, 0);
	} else {
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

void stub_spiram_writeback_cache(void)
{
	int x;
	volatile int i = 0;
	volatile uint8_t *psram = (volatile uint8_t *)SOC_EXTRAM_DATA_LOW;
	int cache_was_disabled = 0;

	/* We need cache enabled for this to work. Re-enable it if needed; make sure we
	 * disable it again on exit as well. */
	if (DPORT_REG_GET_BIT(DPORT_PRO_CACHE_CTRL_REG, DPORT_PRO_CACHE_ENABLE) == 0) {
		cache_was_disabled |= (1 << 0);
		DPORT_SET_PERI_REG_BITS(DPORT_PRO_CACHE_CTRL_REG, 1, 1, DPORT_PRO_CACHE_ENABLE_S);
	}
	if (DPORT_REG_GET_BIT(DPORT_APP_CACHE_CTRL_REG, DPORT_APP_CACHE_ENABLE) == 0) {
		cache_was_disabled |= (1 << 1);
		DPORT_SET_PERI_REG_BITS(DPORT_APP_CACHE_CTRL_REG, 1, 1, DPORT_APP_CACHE_ENABLE_S);
	}

	/*
	Note: this assumes the amount of external RAM is >2M. If it is 2M or less, what this code does is undefined. If
	we ever support external RAM chips of 2M or smaller, this may need adjusting.
	*/
	for (x = 0; x < 1024 * 64; x += 32) {
		i += psram[x];
		i += psram[x + (1024 * 1024 * 2)];
	}

	if (cache_was_disabled & (1 << 0)) {
		while (DPORT_GET_PERI_REG_BITS2(DPORT_PRO_DCACHE_DBUG0_REG, DPORT_PRO_CACHE_STATE,
				DPORT_PRO_CACHE_STATE_S) != 1) ;
		DPORT_SET_PERI_REG_BITS(DPORT_PRO_CACHE_CTRL_REG, 1, 0, DPORT_PRO_CACHE_ENABLE_S);
	}
	if (cache_was_disabled & (1 << 1)) {
		while (DPORT_GET_PERI_REG_BITS2(DPORT_APP_DCACHE_DBUG0_REG, DPORT_APP_CACHE_STATE,
				DPORT_APP_CACHE_STATE_S) != 1) ;
		DPORT_SET_PERI_REG_BITS(DPORT_APP_CACHE_CTRL_REG, 1, 0, DPORT_APP_CACHE_ENABLE_S);
	}
}

void stub_flash_cache_flush(void)
{
	if (DPORT_GET_PERI_REG_MASK(DPORT_PRO_CACHE_CTRL1_REG,
			DPORT_PRO_CACHE_MASK_OPSDRAM) == 0 ||
		DPORT_GET_PERI_REG_MASK(DPORT_APP_CACHE_CTRL1_REG,
			DPORT_APP_CACHE_MASK_OPSDRAM) == 0)
		stub_spiram_writeback_cache();
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
	state->cache_enabled = esp32_flash_cache_enabled(core_id) && esp32_flash_cache_bus_enabled(
		core_id);
	if (!state->cache_enabled) {
		STUB_LOGI("Cache needs to be enabled for CPU%d\n", core_id);
		stub_cache_init(core_id);
	}

	state->spi_regs[ESP32_STUB_FLASH_STATE_SPI_USER_REG_ID] = READ_PERI_REG(SPI_USER_REG(1));
	state->spi_regs[ESP32_STUB_FLASH_STATE_SPI_USER1_REG_ID] = READ_PERI_REG(SPI_USER1_REG(1));
	state->spi_regs[ESP32_STUB_FLASH_STATE_SPI_USER2_REG_ID] = READ_PERI_REG(SPI_USER2_REG(1));
	state->spi_regs[ESP32_STUB_FLASH_STATE_SPI_SLAVE_REG_ID] = READ_PERI_REG(SPI_SLAVE_REG(1));
	state->spi_regs[ESP32_STUB_FLASH_STATE_SPI_CLOCK_REG_ID] = READ_PERI_REG(SPI_CLOCK_REG(1));
	state->spi_regs[ESP32_STUB_FLASH_STATE_SPI_CTRL_REG_ID] = READ_PERI_REG(SPI_CTRL_REG(1));
	state->dummy_len_plus = g_rom_spiflash_dummy_len_plus[1];

	WRITE_PERI_REG(SPI_USER_REG(1), ESP32_STUB_FLASH_STATE_SPI_USER_REG_VAL);
	WRITE_PERI_REG(SPI_USER1_REG(1), ESP32_STUB_FLASH_STATE_SPI_USER1_REG_VAL);
	WRITE_PERI_REG(SPI_USER2_REG(1), ESP32_STUB_FLASH_STATE_SPI_USER2_REG_VAL);
	WRITE_PERI_REG(SPI_SLAVE_REG(1), ESP32_STUB_FLASH_STATE_SPI_SLAVE_REG_VAL);

	if ((READ_PERI_REG(SPI_CACHE_FCTRL_REG(0)) & SPI_CACHE_FLASH_USR_CMD) == 0) {
		STUB_LOGI("Attach spi flash...\n");
		esp_rom_spiflash_attach(spiconfig, 0);
	} else {
		WRITE_PERI_REG(SPI_CTRL_REG(1), 0x208000);
		WRITE_PERI_REG(SPI_CLOCK_REG(1), 0x3043);
		g_rom_spiflash_dummy_len_plus[1] = 0;
	}
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
	WRITE_PERI_REG(SPI_CLOCK_REG(1), state->spi_regs[ESP32_STUB_FLASH_STATE_SPI_CLOCK_REG_ID]);
	WRITE_PERI_REG(SPI_CTRL_REG(1), state->spi_regs[ESP32_STUB_FLASH_STATE_SPI_CTRL_REG_ID]);
	g_rom_spiflash_dummy_len_plus[1] = state->dummy_len_plus;

	if (state->other_cache_enabled) {
		esp32_flash_restore_cache_for_cpu(other_core_id, state->cache_flags[other_core_id]);
		STUB_LOGI("Cache restored CPU%d: 0x%x %d\n", other_core_id,
			state->cache_flags[other_core_id],
			esp32_flash_cache_enabled(other_core_id));
	}
	if (state->cache_enabled) {
		/* we are managing the running core's cache while map unmap. So nothing to do here..
		 **/
	}
}

int stub_cpu_clock_configure(int cpu_freq_mhz)
{
	rtc_cpu_freq_config_t old_config;
	rtc_clk_cpu_freq_get_config(&old_config);

	if (stub_get_log_dest() == STUB_LOG_DEST_UART)
		uart_tx_wait_idle(CONFIG_CONSOLE_UART_NUM);

	/* set to maximum possible value */
	if (cpu_freq_mhz == -1)
		cpu_freq_mhz = CONFIG_ESP32_DEFAULT_CPU_FREQ_MHZ;

	/* Set CPU to configured value. Keep other clocks unmodified. */
	if (cpu_freq_mhz > 0) {
		/* On ESP32 rev 0, switching to 80MHz if clock was previously set to
		 * 240 MHz may cause the chip to lock up (see section 3.5 of the errata
		 * document). For rev. 0, switch to 240 instead if it was chosen in
		 * menuconfig.
		 */
		if (cpu_freq_mhz == 80 && old_config.freq_mhz == 240) {
			uint32_t chip_ver_reg = REG_READ(EFUSE_BLK0_RDATA3_REG);
			if ((chip_ver_reg & EFUSE_RD_CHIP_VER_REV1_M) == 0)
				cpu_freq_mhz = 240;
		}

		rtc_clk_config_t clk_cfg = RTC_CLK_CONFIG_DEFAULT();
		clk_cfg.xtal_freq = RTC_XTAL_FREQ_AUTO;
		clk_cfg.cpu_freq_mhz = cpu_freq_mhz;
		clk_cfg.slow_freq = rtc_clk_slow_freq_get();
		clk_cfg.fast_freq = rtc_clk_fast_freq_get();
		rtc_clk_init(clk_cfg);

		g_stub_cpu_freq_hz = cpu_freq_mhz * MHZ;
	}

	return old_config.freq_mhz;
}

#if STUB_LOG_ENABLE == 1
void stub_uart_console_configure(int dest)
{
	uartAttach();
	ets_install_uart_printf();
	/* Set configured UART console baud rate */
	uart_div_modify(CONFIG_CONSOLE_UART_NUM,
		(rtc_clk_apb_freq_get() << 4) / CONFIG_CONSOLE_UART_BAUDRATE);
}
#endif

uint32_t stub_esp_clk_cpu_freq(void)
{
	return g_stub_cpu_freq_hz;
}

static inline bool esp_flash_encryption_enabled(void)
{
	uint32_t flash_crypt_cnt = REG_GET_FIELD(EFUSE_BLK0_RDATA0_REG,
		EFUSE_RD_FLASH_CRYPT_CNT);

	/* __builtin_parity is in flash, so we calculate parity inline */
	bool enabled = false;
	while (flash_crypt_cnt) {
		if (flash_crypt_cnt & 1)
			enabled = !enabled;
		flash_crypt_cnt >>= 1;
	}
	return enabled;
}

esp_flash_enc_mode_t stub_get_flash_encryption_mode(void)
{
	static esp_flash_enc_mode_t mode = ESP_FLASH_ENC_MODE_DEVELOPMENT;
	static bool first = true;

	if (first) {
		if (esp_flash_encryption_enabled()) {
			/* Check if FLASH CRYPT CNT is write protected */
			bool flash_crypt_cnt_wr_dis = REG_READ(EFUSE_BLK0_RDATA0_REG) &
				EFUSE_WR_DIS_FLASH_CRYPT_CNT;
			if (!flash_crypt_cnt_wr_dis) {
				uint8_t flash_crypt_cnt = REG_GET_FIELD(EFUSE_BLK0_RDATA0_REG,
					EFUSE_RD_FLASH_CRYPT_CNT);
				/* Check if FLASH_CRYPT_CNT set for permanent encryption */
				if (flash_crypt_cnt == EFUSE_RD_FLASH_CRYPT_CNT_V)
					flash_crypt_cnt_wr_dis = true;
			}

			if (flash_crypt_cnt_wr_dis) {
				uint8_t dis_dl_cache = REG_GET_FIELD(EFUSE_BLK0_RDATA6_REG,
					EFUSE_RD_DISABLE_DL_CACHE);
				uint8_t dis_dl_enc = REG_GET_FIELD(EFUSE_BLK0_RDATA6_REG,
					EFUSE_RD_DISABLE_DL_ENCRYPT);
				uint8_t dis_dl_dec = REG_GET_FIELD(EFUSE_BLK0_RDATA6_REG,
					EFUSE_RD_DISABLE_DL_DECRYPT);
				/* Check if DISABLE_DL_DECRYPT, DISABLE_DL_ENCRYPT & DISABLE_DL_CACHE are
				        set */
				if (dis_dl_cache && dis_dl_enc && dis_dl_dec)
					mode = ESP_FLASH_ENC_MODE_RELEASE;
			}
		} else {
			mode = ESP_FLASH_ENC_MODE_DISABLED;
		}
		first = false;
		STUB_LOGD("flash encryption mode: %d\n", mode);
	}

	return mode;
}

static int stub_flash_mmap(struct spiflash_map_req *req)
{
	uint32_t map_src = req->src_addr & (~(SPI_FLASH_MMU_PAGE_SIZE - 1));
	uint32_t map_size = req->size + (req->src_addr - map_src);
	uint32_t flash_page = map_src / SPI_FLASH_MMU_PAGE_SIZE;
	uint32_t page_cnt = (map_size + SPI_FLASH_MMU_PAGE_SIZE - 1) / SPI_FLASH_MMU_PAGE_SIZE;
	int start_page, ret = ESP_ROM_SPIFLASH_RESULT_ERR;
	uint32_t saved_state = 0;
	esp32_flash_disable_cache_for_cpu(req->core_id, &saved_state);

	for (start_page = STUB_MMU_DROM_PAGES_START; start_page < STUB_MMU_DROM_PAGES_END;
		++start_page) {
		if (mmu_table_s[req->core_id][start_page] == STUB_MMU_INVALID_ENTRY_VAL)
			break;
	}

	if (start_page == STUB_MMU_DROM_PAGES_END)
		start_page = STUB_MMU_DROM_PAGES_START;

	if (start_page + page_cnt < STUB_MMU_DROM_PAGES_END) {
		for (int i = 0; i < page_cnt; i++)
			mmu_table_s[req->core_id][start_page + i] = SOC_MMU_PAGE_IN_FLASH(
				flash_page + i);

		req->start_page = start_page;
		req->page_cnt = page_cnt;
		req->ptr = (void *)(STUB_MMU_DROM_VADDR +
			(start_page - STUB_MMU_DROM_PAGES_START) * SPI_FLASH_MMU_PAGE_SIZE +
			(req->src_addr - map_src));
		Cache_Flush(req->core_id);
		ret = ESP_ROM_SPIFLASH_RESULT_OK;
	}

	STUB_LOGD(
		"start_page: %d map_src: %x map_size: %x page_cnt: %d flash_page: %d map_ptr: %x\n",
		start_page,
		map_src,
		map_size,
		page_cnt,
		flash_page,
		req->ptr);

	esp32_flash_restore_cache_for_cpu(req->core_id, saved_state);

	return ret;
}

static void stub_flash_ummap(const struct spiflash_map_req *req)
{
	uint32_t saved_state = 0;

	esp32_flash_disable_cache_for_cpu(req->core_id, &saved_state);

	for (int i = req->start_page; i < req->start_page + req->page_cnt; ++i)
		mmu_table_s[req->core_id][i] = STUB_MMU_INVALID_ENTRY_VAL;

	esp32_flash_restore_cache_for_cpu(req->core_id, saved_state);
}

int stub_flash_read_buff(uint32_t addr, void *buffer, uint32_t size)
{
	struct spiflash_map_req req = {
		.src_addr = addr,
		.size = size,
		.core_id = stub_get_coreid()
	};

	int ret = stub_flash_mmap(&req);

	if (ret)
		return ret;

	memcpy(buffer, req.ptr, size);

	stub_flash_ummap(&req);

	return ESP_ROM_SPIFLASH_RESULT_OK;
}
