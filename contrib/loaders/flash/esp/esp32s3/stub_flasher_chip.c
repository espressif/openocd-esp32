/***************************************************************************
 *   ESP32-S3 specific flasher stub functions                              *
 *   Copyright (C) 2021 Espressif Systems Ltd.                             *
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
#include <string.h>
#include "sdkconfig.h"
#include "soc/rtc.h"
#include "soc/efuse_periph.h"
#include "soc/spi_mem_reg.h"
#include "soc/extmem_reg.h"
#include "soc/system_reg.h"
#include "soc/gpio_reg.h"
#include "soc/mmu.h"
#include "xtensa/hal.h"
#include "esp_spi_flash.h"
#include "rtc_clk_common.h"
#include "stub_rom_chip.h"
#include "stub_flasher_int.h"
#include "stub_flasher_chip.h"

#define EFUSE_WR_DIS_SPI_BOOT_CRYPT_CNT          (1 << 4)

/* Cache MMU related definitions */
#define STUB_CACHE_BUS_PRO              EXTMEM_DCACHE_SHUT_CORE0_BUS
#define STUB_CACHE_BUS_APP              EXTMEM_DCACHE_SHUT_CORE1_BUS
#define STUB_MMU_DROM_VADDR             SOC_MMU_VADDR0_START_ADDR
#define STUB_MMU_DROM_PAGES_START       SOC_MMU_DROM0_PAGES_START
#define STUB_MMU_DROM_PAGES_END         SOC_MMU_DROM0_PAGES_END
#define STUB_MMU_TABLE                  SOC_MMU_DPORT_PRO_FLASH_MMU_TABLE	/* 0x600c5000 */
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
};

uint32_t g_stub_cpu_freq_hz = CONFIG_ESP32S3_DEFAULT_CPU_FREQ_MHZ * MHZ;

extern bool ets_efuse_flash_octal_mode(void);

void vPortEnterCritical(void *mux)
{
}

void vPortExitCritical(void *mux)
{
}

#if STUB_LOG_LOCAL_LEVEL > STUB_LOG_INFO
void stub_print_cache_mmu_registers(void)
{
	uint32_t icache_ctrl1_reg = REG_READ(EXTMEM_DCACHE_CTRL1_REG);

	STUB_LOGD("dcache_ctrl1_reg: 0x%x\n",
		icache_ctrl1_reg);
}
#endif

uint32_t stub_flash_get_id(void)
{
	uint32_t ret;

	STUB_LOGD("flash %x, cs %x, bs %x, ss %x, ps %x, sm %x\n",
		rom_spiflash_legacy_data->chip.device_id,
		rom_spiflash_legacy_data->chip.chip_size,
		rom_spiflash_legacy_data->chip.block_size,
		rom_spiflash_legacy_data->chip.sector_size,
		rom_spiflash_legacy_data->chip.page_size,
		rom_spiflash_legacy_data->chip.status_mask);

	if (rom_spiflash_legacy_data->dummy_len_plus[1] == 0)
		REG_CLR_BIT(PERIPHS_SPI_FLASH_USRREG, SPI_MEM_USR_DUMMY);
	else {
		REG_SET_BIT(PERIPHS_SPI_FLASH_USRREG, SPI_MEM_USR_DUMMY);
		REG_WRITE(
			PERIPHS_SPI_FLASH_USRREG1,
			(rom_spiflash_legacy_data->dummy_len_plus[1] -
				1) << SPI_MEM_USR_DUMMY_CYCLELEN_S);

	}
	WRITE_PERI_REG(PERIPHS_SPI_FLASH_C0, 0);/* clear register */
	WRITE_PERI_REG(PERIPHS_SPI_FLASH_CMD, SPI_MEM_FLASH_RDID);
	while (READ_PERI_REG(PERIPHS_SPI_FLASH_CMD) != 0) ;
	ret = READ_PERI_REG(PERIPHS_SPI_FLASH_C0) & 0xffffff;
	STUB_LOGD("Flash ID read %x\n", ret);
	return ret >> 16;
}

void stub_flash_cache_flush(void)
{
	/* we do not know breakpoint program address here, so invalidate the
	 * whole ICache */
	Cache_Invalidate_ICache_All();
}

static void stub_cache_init(uint32_t cpuid)
{
	STUB_LOGD("stub_cache_init\n");
	/* init cache mmu, set cache mode, invalidate cache tags, enable cache*/
	REG_SET_BIT(SYSTEM_CACHE_CONTROL_REG, SYSTEM_DCACHE_CLK_ON);
	REG_SET_BIT(SYSTEM_CACHE_CONTROL_REG, SYSTEM_DCACHE_RESET);
	REG_CLR_BIT(SYSTEM_CACHE_CONTROL_REG, SYSTEM_DCACHE_RESET);
	/* init cache owner bit */
	Cache_Owner_Init();
	/* clear mmu entry */
	Cache_MMU_Init();
	/* config cache mode */
	Cache_Set_Default_Mode();
	Cache_Enable_DCache(0);
	if (cpuid == 0)
		REG_CLR_BIT(EXTMEM_DCACHE_CTRL1_REG, STUB_CACHE_BUS_PRO);
	else
		REG_CLR_BIT(EXTMEM_DCACHE_CTRL1_REG, STUB_CACHE_BUS_APP);
}

static bool stub_is_cache_enabled(uint32_t cpuid)
{
	bool is_enabled = REG_GET_BIT(EXTMEM_DCACHE_CTRL_REG, EXTMEM_DCACHE_ENABLE) != 0;
	int cache_bus_disabled = REG_READ(EXTMEM_DCACHE_CTRL1_REG) &
		(cpuid == 0 ? STUB_CACHE_BUS_PRO : STUB_CACHE_BUS_APP);
	return is_enabled && !cache_bus_disabled;
}

void stub_flash_state_prepare(struct stub_flash_state *state)
{
	uint32_t core_id = stub_get_coreid();
	uint32_t spiconfig = ets_efuse_get_spiconfig();

	state->cache_enabled = stub_is_cache_enabled(core_id);
	if (!state->cache_enabled) {
		STUB_LOGI("Cache needs to be enabled for CPU%d\n", core_id);
		stub_cache_init(core_id);
	}

	if (ets_efuse_flash_octal_mode())
		STUB_LOGE("Octal flah not supported yet!\n");

	esp_rom_spiflash_attach(spiconfig, 0);
}

void stub_flash_state_restore(struct stub_flash_state *state)
{
	/* we do not disable or store the cache settings. So, nothing to restore*/
}

#define RTC_PLL_FREQ_320M   320
#define RTC_PLL_FREQ_480M   480

rtc_xtal_freq_t stub_rtc_clk_xtal_freq_get(void)
{
	uint32_t xtal_freq_reg = READ_PERI_REG(RTC_XTAL_FREQ_REG);
	if (!clk_val_is_valid(xtal_freq_reg))
		return RTC_XTAL_FREQ_40M;
	return reg_val_to_clk_val(xtal_freq_reg);
}

/* Obviously we can call rtc_clk_cpu_freq_get_config() from esp-idf
But this call may cause undesired locks due to ets_printf or abort
*/
int stub_rtc_clk_cpu_freq_get_config(rtc_cpu_freq_config_t *out_config)
{
	rtc_cpu_freq_src_t source;
	uint32_t source_freq_mhz;
	uint32_t div;
	uint32_t freq_mhz;
	uint32_t soc_clk_sel = REG_GET_FIELD(SYSTEM_SYSCLK_CONF_REG, SYSTEM_SOC_CLK_SEL);
	switch (soc_clk_sel) {
		case DPORT_SOC_CLK_SEL_XTAL: {
			source = RTC_CPU_FREQ_SRC_XTAL;
			div = REG_GET_FIELD(SYSTEM_SYSCLK_CONF_REG, SYSTEM_PRE_DIV_CNT) + 1;
			source_freq_mhz = (uint32_t) stub_rtc_clk_xtal_freq_get();
			freq_mhz = source_freq_mhz / div;
		}
		break;
		case DPORT_SOC_CLK_SEL_PLL: {
			source = RTC_CPU_FREQ_SRC_PLL;
			uint32_t cpuperiod_sel = REG_GET_FIELD(SYSTEM_CPU_PER_CONF_REG,
				SYSTEM_CPUPERIOD_SEL);
			uint32_t pllfreq_sel = REG_GET_FIELD(SYSTEM_CPU_PER_CONF_REG,
				SYSTEM_PLL_FREQ_SEL);
			source_freq_mhz = (pllfreq_sel) ? RTC_PLL_FREQ_480M : RTC_PLL_FREQ_320M;
			if (cpuperiod_sel == DPORT_CPUPERIOD_SEL_80) {
				div = (source_freq_mhz == RTC_PLL_FREQ_480M) ? 6 : 4;
				freq_mhz = 80;
			} else if (cpuperiod_sel == DPORT_CPUPERIOD_SEL_160) {
				div = (source_freq_mhz == RTC_PLL_FREQ_480M) ? 3 : 2;
				div = 3;
				freq_mhz = 160;
			} else if (cpuperiod_sel == DPORT_CPUPERIOD_SEL_240) {
				div = 2;
				freq_mhz = 240;
			} else {
				/* unsupported frequency configuration */
				return -1;
			}
			break;
		}
		case DPORT_SOC_CLK_SEL_8M:
			source = RTC_CPU_FREQ_SRC_8M;
			source_freq_mhz = 8;
			div = 1;
			freq_mhz = source_freq_mhz;
			break;
		default:
			/* unsupported frequency configuration */
			return -2;
	}
	*out_config = (rtc_cpu_freq_config_t) {
		.source = source,
		.source_freq_mhz = source_freq_mhz,
		.div = div,
		.freq_mhz = freq_mhz
	};
	return 0;
}

/* this function has almost the same implementation for ESP32 and ESP32-S2
 * TODO: move to common file */
int stub_cpu_clock_configure(int cpu_freq_mhz)
{
	rtc_cpu_freq_config_t old_config;
	int ret = stub_rtc_clk_cpu_freq_get_config(&old_config);
	if (ret < 0) {
		/* this return value will avoid undesired restore requests for unsupported frequency
		 *configuration */
		old_config.freq_mhz = 0;
	}

#if STUB_LOG_LOCAL_LEVEL > STUB_LOG_NONE
	uart_tx_wait_idle(CONFIG_CONSOLE_UART_NUM);
#endif

	/* set to maximum possible value */
	if (cpu_freq_mhz == -1)
		cpu_freq_mhz = CONFIG_ESP32S3_DEFAULT_CPU_FREQ_MHZ;

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
void stub_uart_console_configure()
{
	extern bool g_uart_print;
	/* set the default parameter to UART module, but don't enable RX interrupt */
	uartAttach(NULL);
	/* first enable uart0 as printf channel */
	uint32_t clock = ets_get_apb_freq();
	ets_update_cpu_frequency(clock/1000000);

	Uart_Init(ets_efuse_get_uart_print_channel(), UART_CLK_FREQ_ROM);
	/* install to print later
	 * Non-Flash Boot can print
	 * Flash Boot can print when RTC_CNTL_STORE4_REG bit0 is 0 (can be 1 after deep sleep, software reset) and printf boot.
	 * print boot determined by GPIO and efuse, see ets_is_print_boot
	 */
	g_uart_print = true;
	ets_install_uart_printf();
}
#endif

uint32_t stub_esp_clk_cpu_freq(void)
{
	return g_stub_cpu_freq_hz;
}

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
	if ((start_addr + area_len) > rom_spiflash_legacy_data->chip.chip_size)
		return ESP_ROM_SPIFLASH_RESULT_ERR;

	/* start_addr is aligned as sector boundary */
	if (0 != (start_addr % rom_spiflash_legacy_data->chip.sector_size))
		return ESP_ROM_SPIFLASH_RESULT_ERR;

	/* Unlock flash to enable erase */
	if (ESP_ROM_SPIFLASH_RESULT_OK != esp_rom_spiflash_unlock(/*&rom_spiflash_legacy_data->chip*/))
		return ESP_ROM_SPIFLASH_RESULT_ERR;

	sector_no = start_addr / rom_spiflash_legacy_data->chip.sector_size;
	sector_num_per_block = rom_spiflash_legacy_data->chip.block_size /
		rom_spiflash_legacy_data->chip.sector_size;
	total_sector_num =
		(0 ==
		(area_len %
			rom_spiflash_legacy_data->chip.sector_size)) ? area_len /
		rom_spiflash_legacy_data->chip.sector_size :
		1 + (area_len / rom_spiflash_legacy_data->chip.sector_size);

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

static inline bool esp_flash_encryption_enabled(void)
{
	uint32_t flash_crypt_cnt = REG_GET_FIELD(EFUSE_RD_REPEAT_DATA1_REG,
		EFUSE_SPI_BOOT_CRYPT_CNT);

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
	static esp_flash_enc_mode_t s_mode = ESP_FLASH_ENC_MODE_DEVELOPMENT;
	static bool s_first = true;

	if (s_first) {
		if (esp_flash_encryption_enabled()) {
			/* Check if SPI_BOOT_CRYPT_CNT is write protected */
			bool flash_crypt_cnt_wr_dis = REG_READ(EFUSE_RD_WR_DIS_REG) &
				EFUSE_WR_DIS_SPI_BOOT_CRYPT_CNT;
			if (!flash_crypt_cnt_wr_dis) {
				uint8_t flash_crypt_cnt = REG_GET_FIELD(EFUSE_RD_REPEAT_DATA1_REG,
					EFUSE_SPI_BOOT_CRYPT_CNT);
				/* Check if SPI_BOOT_CRYPT_CNT set for permanent encryption */
				if (flash_crypt_cnt == EFUSE_SPI_BOOT_CRYPT_CNT_V)
					flash_crypt_cnt_wr_dis = true;
			}

			if (flash_crypt_cnt_wr_dis) {
				uint8_t dis_dl_enc = REG_GET_FIELD(EFUSE_RD_REPEAT_DATA0_REG,
					EFUSE_DIS_DOWNLOAD_MANUAL_ENCRYPT);
				uint8_t dis_dl_icache = REG_GET_FIELD(EFUSE_RD_REPEAT_DATA0_REG,
					EFUSE_DIS_DOWNLOAD_ICACHE);
				if (dis_dl_enc && dis_dl_icache)
					s_mode = ESP_FLASH_ENC_MODE_RELEASE;
			}

		} else
			s_mode = ESP_FLASH_ENC_MODE_DISABLED;

		s_first = false;
	}

	STUB_LOGD("flash_encryption_mode: %d\n", s_mode);

	return s_mode;
}

static int stub_flash_mmap(struct spiflash_map_req *req)
{
	uint32_t map_src = req->src_addr & (~(SPI_FLASH_MMU_PAGE_SIZE - 1));
	uint32_t map_size = req->size + (req->src_addr - map_src);
	uint32_t flash_page = map_src / SPI_FLASH_MMU_PAGE_SIZE;
	uint32_t page_cnt = (map_size + SPI_FLASH_MMU_PAGE_SIZE - 1) / SPI_FLASH_MMU_PAGE_SIZE;
	int start_page, ret = ESP_ROM_SPIFLASH_RESULT_ERR;
	uint32_t icache_state, dcache_state;

	icache_state = Cache_Suspend_ICache();
	dcache_state = Cache_Suspend_DCache();

	for (start_page = STUB_MMU_DROM_PAGES_START; start_page < STUB_MMU_DROM_PAGES_END;
		++start_page) {
		if (STUB_MMU_TABLE[start_page] == STUB_MMU_INVALID_ENTRY_VAL)
			break;
	}

	if (start_page == STUB_MMU_DROM_PAGES_END) {
		STUB_LOGW("Failed to find free MMU page! Use the first one.\n");
		start_page = STUB_MMU_DROM_PAGES_START;
	}

	if (start_page + page_cnt < STUB_MMU_DROM_PAGES_END) {
		for (int i = 0; i < page_cnt; i++)
			STUB_MMU_TABLE[start_page + i] = SOC_MMU_PAGE_IN_FLASH(
				flash_page + i);

		req->start_page = start_page;
		req->page_cnt = page_cnt;
		req->ptr = (void *)(STUB_MMU_DROM_VADDR +
			(start_page - STUB_MMU_DROM_PAGES_START) * SPI_FLASH_MMU_PAGE_SIZE +
			(req->src_addr - map_src));
		Cache_Invalidate_Addr((uint32_t)(STUB_MMU_DROM_VADDR +
				(start_page - STUB_MMU_DROM_PAGES_START) * SPI_FLASH_MMU_PAGE_SIZE),
			page_cnt * SPI_FLASH_MMU_PAGE_SIZE);
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

	Cache_Resume_DCache(dcache_state);
	Cache_Resume_ICache(icache_state);

	return ret;
}

static void stub_flash_ummap(const struct spiflash_map_req *req)
{
	uint32_t icache_state, dcache_state;

	icache_state = Cache_Suspend_ICache();
	dcache_state = Cache_Suspend_DCache();

	for (int i = req->start_page; i < req->start_page + req->page_cnt; ++i)
		STUB_MMU_TABLE[i] = STUB_MMU_INVALID_ENTRY_VAL;

	Cache_Resume_DCache(dcache_state);
	Cache_Resume_ICache(icache_state);
}


int stub_flash_read_buff(uint32_t addr, void *buffer, uint32_t size)
{
	struct spiflash_map_req req = {
		.src_addr = addr,
		.size = size,
	};

	int ret = stub_flash_mmap(&req);

	if (ret)
		return ret;

	memcpy(buffer, req.ptr, size);

	stub_flash_ummap(&req);

	return ESP_ROM_SPIFLASH_RESULT_OK;
}
