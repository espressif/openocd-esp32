/***************************************************************************
 *   ESP32-H2 specific flasher stub functions                              *
 *   Copyright (C) 2022 Espressif Systems Ltd.                             *
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
#include "soc/gpio_reg.h"
#include "soc/mmu.h"
#include "soc/system_reg.h"
#include "spi_flash_mmap.h"
#include "soc/clk_tree_defs.h"
#include "hal/clk_tree_ll.h"
#include "hal/mmu_ll.h"
#include "esp_app_trace_membufs_proto.h"
#include "esp_rom_efuse.h"
#include "stub_rom_chip.h"
#include "stub_logger.h"
#include "stub_flasher_int.h"
#include "stub_flasher_chip.h"
#include "stub_flasher.h"

#define EFUSE_WR_DIS_SPI_BOOT_CRYPT_CNT                     (1 << 4)

/* Cache MMU related definitions */
#define STUB_CACHE_BUS                                  EXTMEM_ICACHE_SHUT_DBUS
#define STUB_MMU_DROM_VADDR                             0x3C010000
#define STUB_MMU_DROM_PAGES_START                       1
#define STUB_MMU_DROM_PAGES_END                         128
#define STUB_MMU_TABLE                                  ((volatile uint32_t *)0x600c5000)	/* TODO check this */
#define STUB_MMU_INVALID_ENTRY_VAL                      0x100
#define STUB_MMU_PAGE_SIZE                              0x10000	/* On esp32h2, MMU page size is always 64KB */

#define ESP_APPTRACE_RISCV_BLOCK_LEN_MSK                0x7FFFUL
#define ESP_APPTRACE_RISCV_BLOCK_LEN(_l_)               ((_l_) & ESP_APPTRACE_RISCV_BLOCK_LEN_MSK)
#define ESP_APPTRACE_RISCV_BLOCK_LEN_GET(_v_)           ((_v_) & ESP_APPTRACE_RISCV_BLOCK_LEN_MSK)
#define ESP_APPTRACE_RISCV_BLOCK_ID_MSK                 0x7FUL
#define ESP_APPTRACE_RISCV_BLOCK_ID(_id_)               (((_id_) & ESP_APPTRACE_RISCV_BLOCK_ID_MSK) << 15)
#define ESP_APPTRACE_RISCV_BLOCK_ID_GET(_v_)            (((_v_) >> 15) & ESP_APPTRACE_RISCV_BLOCK_ID_MSK)
#define ESP_APPTRACE_RISCV_HOST_DATA                    (1 << 22)
#define ESP_APPTRACE_RISCV_HOST_CONNECT                 (1 << 23)

/** RISCV memory host iface control block */
typedef struct {
	uint32_t ctrl;
	/* - Guard field. If this register is not zero then CPU is changing this struct and */
	/*   this guard field holds address of the instruction which application will execute when
	 * CPU finishes with those modifications. */
	uint32_t stat;
	esp_apptrace_mem_block_t *mem_blocks;
} esp_apptrace_riscv_ctrl_block_t;

static esp_apptrace_riscv_ctrl_block_t *s_apptrace_ctrl;
#if CONFIG_STUB_STACK_DATA_POOL_SIZE > 0
static uint8_t *s_stack_data_pool;
static size_t s_stack_data_pool_sz;
#endif

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

extern void spi_flash_attach(uint32_t, bool);

uint32_t g_stub_cpu_freq_hz = CONFIG_ESP32H2_DEFAULT_CPU_FREQ_MHZ * MHZ;

void vPortEnterCritical(void)
{
}

void vPortExitCritical(void)
{
}

int xPortInIsrContext(void)
{
	return 0;
}

void *esp_apptrace_uart_hw_get(int num, void **data)
{
	return NULL;
}

#if STUB_LOG_ENABLE == 1
void stub_print_cache_mmu_registers(void)
{
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
	WRITE_PERI_REG(PERIPHS_SPI_FLASH_C0, 0);/* clear regisrter */
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

void stub_cache_init(void)
{
	STUB_LOGD("stub_cache_init\n");
	/* init cache mmu, set cache mode, invalidate cache tags, enable cache*/
#if 0	/* TODO */
	REG_SET_BIT(SYSTEM_CACHE_CONTROL_REG, SYSTEM_ICACHE_CLK_ON);
	REG_SET_BIT(SYSTEM_CACHE_CONTROL_REG, SYSTEM_ICACHE_RESET);
	REG_CLR_BIT(SYSTEM_CACHE_CONTROL_REG, SYSTEM_ICACHE_RESET);
	/* init cache owner bit */
	Cache_Owner_Init();
	/* clear mmu entry */
	Cache_MMU_Init();
	/* config cache mode */
	Cache_Set_Default_Mode();
	Cache_Enable_ICache(0);
	REG_CLR_BIT(EXTMEM_ICACHE_CTRL1_REG, STUB_CACHE_BUS);
#endif
}

bool stub_is_cache_enabled(void)
{
#if 0	/* TODO */
	bool is_enabled = REG_GET_BIT(EXTMEM_ICACHE_CTRL_REG, EXTMEM_ICACHE_ENABLE) != 0;
	int cache_bus = REG_READ(EXTMEM_ICACHE_CTRL1_REG);
	return is_enabled && !(cache_bus & STUB_CACHE_BUS);
#endif
	return false;
}

void stub_flash_state_prepare(struct stub_flash_state *state)
{
	const uint32_t spiconfig = 0;	/* esp_rom_efuse_get_flash_gpio_info(); */

	state->cache_enabled = stub_is_cache_enabled();
	if (!state->cache_enabled) {
		STUB_LOGI("Cache needs to be enabled\n");
		stub_cache_init();
	}

	spi_flash_attach(spiconfig, false);
}

void stub_flash_state_restore(struct stub_flash_state *state)
{
	/* we do not disable or store the cache settings. So, nothing to restore*/
}

#define RTC_PLL_FREQ_320M   320
#define RTC_PLL_FREQ_480M   480

rtc_xtal_freq_t stub_rtc_clk_xtal_freq_get(void)
{
	STUB_LOGD("rtc_clk_xtal_freq_get() has not been implemented yet");
	return 40;
#if 0
	uint32_t xtal_freq_mhz = clk_ll_xtal_load_freq_mhz();
	if (xtal_freq_mhz == 0)
		/* invalid RTC_XTAL_FREQ_REG value, assume 40MHz */
		return RTC_XTAL_FREQ_32M;
	return xtal_freq_mhz;
#endif
}

/* Obviously we can call rtc_clk_cpu_freq_get_config() from esp-idf
But this call may cause undesired locks due to ets_printf or abort
*/
#define RTC_OSC_FREQ_RC8M   8
int stub_rtc_clk_cpu_freq_get_config(rtc_cpu_freq_config_t *out_config)
{
#if 0
	soc_cpu_clk_src_t source = clk_ll_cpu_get_src();
	uint32_t source_freq_mhz;
	uint32_t div;
	uint32_t freq_mhz;
	switch (source) {
	case SOC_CPU_CLK_SRC_XTAL: {
		div = clk_ll_cpu_get_divider();
		source_freq_mhz = (uint32_t)rtc_clk_xtal_freq_get();
		freq_mhz = source_freq_mhz / div;
		break;
	}
	case SOC_CPU_CLK_SRC_PLL: {
		div = clk_ll_cpu_get_divider();
		source_freq_mhz = CLK_LL_PLL_96M_FREQ_MHZ;
		freq_mhz = source_freq_mhz / div;
		break;
	}
	case SOC_CPU_CLK_SRC_RC_FAST: {
		source_freq_mhz = RTC_OSC_FREQ_RC8M;
		div = clk_ll_cpu_get_divider();
		freq_mhz = source_freq_mhz / div;
		break;
	}
	case SOC_CPU_CLK_SRC_XTAL_D2: {
		div = clk_ll_cpu_get_divider();
		source_freq_mhz = (uint32_t)rtc_clk_xtal_freq_get();
		freq_mhz = source_freq_mhz / div / 2;
		break;
	}
	default: {
		/* unsupported frequency configuration */
		return -1;
	}
	}
	*out_config = (rtc_cpu_freq_config_t) {
		.source = source,
		.source_freq_mhz = source_freq_mhz,
		.div = div,
		.freq_mhz = freq_mhz
	};
#endif
	return 0;
}

#if STUB_LOG_ENABLE == 1
extern uint32_t ets_clk_get_xtal_freq(void);
void stub_uart_console_configure(int dest)
{
	/* set the default parameter to UART module, but don't enable RX interrupt */
	uartAttach(NULL);
	/* first enable uart0 as printf channel */
	uint32_t clock = ets_clk_get_xtal_freq();
	ets_update_cpu_frequency(clock / 1000000);

	Uart_Init(0, APB_CLK_FREQ_ROM);
	/* install to print later
	 * Non-Flash Boot can print
	 * Flash Boot can print when RTC_CNTL_STORE4_REG bit0 is 0 (can be 1 after deep sleep, software reset) and printf boot.
	 * print boot determined by GPIO and efuse, see ets_is_print_boot
	 */
	ets_install_uart_printf();
}
#endif

uint32_t stub_esp_clk_cpu_freq(void)
{
	return CONFIG_ESP32H2_DEFAULT_CPU_FREQ_MHZ * 1000000;
}

/* override apptrace control block advertising func, IDF's implementation issues syscall */
int esp_apptrace_advertise_ctrl_block(void *ctrl_block_addr)
{
	s_apptrace_ctrl = ctrl_block_addr;
	return 0;
}

#if CONFIG_STUB_STACK_DATA_POOL_SIZE > 0
void stub_stack_data_pool_init(uint8_t *data, size_t sz)
{
	STUB_LOGD("stack data pool %lu bytes @ 0x%x\n", sz, data);
	s_stack_data_pool = data;
	s_stack_data_pool_sz = sz;
}

void esp_apptrace_get_up_buffers(esp_apptrace_mem_block_t mem_blocks_cfg[2])
{
	/* use whole stack data pool for apptrace up buffers */
	mem_blocks_cfg[0].start = s_stack_data_pool;
	mem_blocks_cfg[0].sz = s_stack_data_pool_sz / 2;
	mem_blocks_cfg[1].start = s_stack_data_pool + mem_blocks_cfg[0].sz;
	mem_blocks_cfg[1].sz = mem_blocks_cfg[0].sz;
}
#endif

int stub_apptrace_prepare()
{
	/* imply that host is auto-connected */
	s_apptrace_ctrl->ctrl |= ESP_APPTRACE_RISCV_HOST_CONNECT;
	return ESP_STUB_ERR_OK;
}

int64_t esp_timer_get_time(void)
{
	/* this function is used by apptrace code to implement timeouts.
	   unfortunately esp32h2 does not support CPU cycle counter, so we have two options:
	   1) Use some HW timer. It can be hard, because we need to ensure that it is initialized and possibly restore its state.
	   2) Emulate timer by incrementing some var on every call.
	          Stub flasher uses ESP_APPTRACE_TMO_INFINITE only, so this function won't be called by apptrace at all. */
	return 0;
}

uint64_t stub_get_time(void)
{
	/* this function is used for perf measurements only.
	   unfortunately esp32h2 does not support CPU cycle counter and usage of HW timer is problematic */
	return 0;
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
	uint32_t flash_crypt_cnt = REG_GET_FIELD(EFUSE_RD_REPEAT_DATA1_REG, EFUSE_SPI_BOOT_CRYPT_CNT);

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
#if 0 /* flash encryption is not supported yet. IDF-6282 */
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
		} else {
			s_mode = ESP_FLASH_ENC_MODE_DISABLED;
		}
		s_first = false;
		STUB_LOGD("flash_encryption_mode: %d\n", s_mode);
	}

	return s_mode;
#endif
	return ESP_FLASH_ENC_MODE_DISABLED;
}

static int __attribute__((unused)) stub_flash_mmap(struct spiflash_map_req *req)
{
	uint32_t map_src = req->src_addr & (~(SPI_FLASH_MMU_PAGE_SIZE - 1));
	uint32_t map_size = req->size + (req->src_addr - map_src);
	uint32_t flash_page = map_src / SPI_FLASH_MMU_PAGE_SIZE;
	uint32_t page_cnt = (map_size + SPI_FLASH_MMU_PAGE_SIZE - 1) / SPI_FLASH_MMU_PAGE_SIZE;
	int start_page, ret = ESP_ROM_SPIFLASH_RESULT_ERR;
	uint32_t saved_state = Cache_Suspend_ICache() << 16;

	for (start_page = STUB_MMU_DROM_PAGES_START; start_page < STUB_MMU_DROM_PAGES_END;
		++start_page) {
		if (STUB_MMU_TABLE[start_page] == STUB_MMU_INVALID_ENTRY_VAL)
			break;
	}

	if (start_page == STUB_MMU_DROM_PAGES_END)
		start_page = STUB_MMU_DROM_PAGES_START;

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

	Cache_Resume_ICache(saved_state >> 16);

	return ret;
}

static void __attribute__((unused)) stub_flash_ummap(const struct spiflash_map_req *req)
{
	uint32_t saved_state = Cache_Suspend_ICache() << 16;

	for (int i = req->start_page; i < req->start_page + req->page_cnt; ++i)
		STUB_MMU_TABLE[i] = STUB_MMU_INVALID_ENTRY_VAL;

	Cache_Resume_ICache(saved_state >> 16);
}

int stub_flash_read_buff(uint32_t addr, void *buffer, uint32_t size)
{
	return esp_rom_spiflash_read(addr, buffer, size);

#if 0	/* cache is not supported yet. IDF-6255 */
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
#endif
}
