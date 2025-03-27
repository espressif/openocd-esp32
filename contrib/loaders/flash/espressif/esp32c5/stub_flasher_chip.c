// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   ESP32-C5 specific flasher stub functions                              *
 *   Copyright (C) 2022 Espressif Systems Ltd.                             *
 ***************************************************************************/
#include <string.h>

#include <sdkconfig.h>

#include <esp32c5/rom/cache.h>
#include <esp32c5/rom/uart.h>
#include <esp32c5/rom/rtc.h>

#include <soc/rtc.h>
#include <soc/efuse_periph.h>
#include <soc/cache_reg.h>
#include <soc/gpio_reg.h>
#include <soc/system_reg.h>
#include <soc/pcr_reg.h>

#include <hal/mmu_ll.h>
#include <hal/cache_types.h>

#include <esp_rom_efuse.h>

#include <stub_flasher_int.h>
#include <stub_logger.h>
#include "stub_flasher_chip.h"

/* RTC related definitios */
#define PCR_SOC_CLK_MAX                         3 // CPU_CLK frequency is 240 MHz (source is PLL_CLK)

/* Cache MMU related definitions */
#define STUB_CACHE_CTRL_REG                     CACHE_L1_CACHE_CTRL_REG
#define STUB_CACHE_BUS                          (CACHE_BUS_IBUS1 | CACHE_BUS_IBUS2 | CACHE_BUS_DBUS1 | CACHE_BUS_DBUS2)
#define STUB_MMU_DROM_PAGES_END                 SOC_MMU_ENTRY_NUM
#define STUB_MMU_DROM_PAGES_START               (STUB_MMU_DROM_PAGES_END - 8) /* 8 pages will be more than enough */

typedef struct {
	mmu_page_size_t page_size;
	uint32_t vaddr0_start_addr;
	uint32_t drom_page_start;
	uint32_t drom_page_end;
	int shift_count;
} cache_mmu_config_t;

static cache_mmu_config_t s_cache_mmu_config;

extern void spi_flash_attach(uint32_t ishspi, bool legacy);

uint32_t g_stub_cpu_freq_hz = CONFIG_ESP32C5_DEFAULT_CPU_FREQ_MHZ * MHZ;

int xPortInIsrContext(void)
{
	return 0;
}

void *esp_apptrace_uart_hw_get(int num, void **data)
{
	return NULL;
}

static inline uint32_t __attribute__((always_inline)) stub_mmu_hal_pages_to_bytes(uint32_t page_num)
{
	return page_num << s_cache_mmu_config.shift_count;
}

static inline uint32_t __attribute__((always_inline)) stub_mmu_ll_format_paddr(uint32_t paddr)
{
	return paddr >> s_cache_mmu_config.shift_count;
}

#define STUB_MMU_VADDR_MASK (s_cache_mmu_config.page_size * SOC_MMU_MAX_PADDR_PAGE_NUM - 1)
static inline uint32_t __attribute__((always_inline)) stub_mmu_ll_get_entry_id(uint32_t vaddr)
{
	return (vaddr & STUB_MMU_VADDR_MASK) >> s_cache_mmu_config.shift_count;
}

static inline void __attribute__((always_inline)) stub_mmu_ll_write_entry(uint32_t entry_id, uint32_t mmu_val)
{
	uint32_t mmu_raw_value;

	if (stub_get_flash_encryption_mode() != ESP_FLASH_ENC_MODE_DISABLED)
		mmu_val |= SOC_MMU_SENSITIVE;

	mmu_raw_value = mmu_val | SOC_MMU_VALID;
	REG_WRITE(SPI_MEM_MMU_ITEM_INDEX_REG(0), entry_id);
	REG_WRITE(SPI_MEM_MMU_ITEM_CONTENT_REG(0), mmu_raw_value);
}

static inline void __attribute__((always_inline)) stub_mmu_ll_set_entry_invalid(uint32_t entry_id)
{
	REG_WRITE(SPI_MEM_MMU_ITEM_INDEX_REG(0), entry_id);
	REG_WRITE(SPI_MEM_MMU_ITEM_CONTENT_REG(0), SOC_MMU_INVALID);
}

static inline int __attribute__((always_inline)) stub_mmu_ll_read_entry(uint32_t entry_id)
{
	uint32_t mmu_raw_value;
	REG_WRITE(SPI_MEM_MMU_ITEM_INDEX_REG(0), entry_id);
	mmu_raw_value = REG_READ(SPI_MEM_MMU_ITEM_CONTENT_REG(0));

	if (stub_get_flash_encryption_mode() != ESP_FLASH_ENC_MODE_DISABLED)
		mmu_raw_value &= ~SOC_MMU_SENSITIVE;

	return  mmu_raw_value;
}

void stub_flash_cache_flush(void)
{
	/* we do not know breakpoint program address here, so invalidate the
	 * whole ICache */
	Cache_WriteBack_Invalidate_All();
}

void stub_cache_configure(void)
{
	s_cache_mmu_config.page_size = mmu_ll_get_page_size(0);
	s_cache_mmu_config.drom_page_start = STUB_MMU_DROM_PAGES_START;
	s_cache_mmu_config.drom_page_end = STUB_MMU_DROM_PAGES_END;	/* 256 */

	switch (s_cache_mmu_config.page_size) {
	case MMU_PAGE_64KB:
		s_cache_mmu_config.shift_count = 16;
		break;
	case MMU_PAGE_32KB:
		s_cache_mmu_config.shift_count = 15;
		break;
	case MMU_PAGE_16KB:
		s_cache_mmu_config.shift_count = 14;
		break;
	case MMU_PAGE_8KB:
		s_cache_mmu_config.shift_count = 13;
		break;
	default:
		STUB_LOGE("Unknown page size!");
		return;
	}

	s_cache_mmu_config.vaddr0_start_addr = SOC_DROM_LOW +
		(s_cache_mmu_config.drom_page_start * s_cache_mmu_config.page_size);

	STUB_LOGI("MMU page size:%X drom_page_start:%d drom_page_end:%d vaddr0_start_addr:%X\n",
		s_cache_mmu_config.page_size,
		s_cache_mmu_config.drom_page_start,
		s_cache_mmu_config.drom_page_end,
		s_cache_mmu_config.vaddr0_start_addr);
}

void stub_cache_init(void)
{
	STUB_LOGD("%s\n", __func__);

	esp_rom_spiflash_attach(0, false);

	SET_PERI_REG_MASK(PCR_CACHE_CONF_REG, PCR_CACHE_CLK_EN_M);
	SET_PERI_REG_MASK(PCR_CACHE_CONF_REG, PCR_CACHE_RST_EN_M);
	CLEAR_PERI_REG_MASK(PCR_CACHE_CONF_REG, PCR_CACHE_RST_EN_M);

	REG_CLR_BIT(STUB_CACHE_CTRL_REG, STUB_CACHE_BUS);
	mmu_ll_set_page_size(0, CONFIG_MMU_PAGE_SIZE);
	Cache_MMU_Init();
	Cache_Enable_Cache(0);
}

static bool stub_is_cache_enabled(void)
{
	int cache_ctrl_reg = REG_READ(STUB_CACHE_CTRL_REG);
	STUB_LOGD("cache_ctrl_reg:%X MMU_VALID:%x\n", cache_ctrl_reg, SOC_MMU_VALID);

	/* if any of the entry is valid and busses are enabled we can consider that cache is enabled */
	for (int i = 0; i < SOC_MMU_ENTRY_NUM; ++i) {
		uint32_t mmu_raw_value = stub_mmu_ll_read_entry(i);
		if ((mmu_raw_value & SOC_MMU_VALID) == SOC_MMU_VALID)
			return !(cache_ctrl_reg & STUB_CACHE_BUS);
	}
	return false;
}

void stub_flash_state_prepare(struct stub_flash_state *state)
{
	state->cache_enabled = stub_is_cache_enabled();
	if (!state->cache_enabled) {
		STUB_LOGI("Cache needs to be enabled\n");
		stub_cache_init();
	}
	stub_cache_configure();
}

void stub_flash_state_restore(struct stub_flash_state *state)
{
	/* we do not disable or store the cache settings. So, nothing to restore*/
}

int stub_cpu_clock_configure(int conf_reg_val)
{
	uint32_t pcr_sysclk_conf_reg = 0;

	/* set to maximum possible value */
	if (conf_reg_val == -1) {
		pcr_sysclk_conf_reg = REG_READ(PCR_SYSCLK_CONF_REG);
		REG_WRITE(PCR_SYSCLK_CONF_REG,
			(pcr_sysclk_conf_reg & ~PCR_SOC_CLK_SEL_M) | (PCR_SOC_CLK_MAX << PCR_SOC_CLK_SEL_S));
	} else { // restore old value
		pcr_sysclk_conf_reg = conf_reg_val;
		REG_WRITE(PCR_SYSCLK_CONF_REG,
			(REG_READ(PCR_SYSCLK_CONF_REG) & ~PCR_SOC_CLK_SEL_M) | (pcr_sysclk_conf_reg & PCR_SOC_CLK_SEL_M));
	}

	STUB_LOGD("pcr_sysclk_conf_reg %x\n", pcr_sysclk_conf_reg);

	return pcr_sysclk_conf_reg;
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

	Uart_Init(0, APB_CLK_FREQ);
	/* install to print later
	 * Non-Flash Boot can print
	 * Flash Boot can print when RTC_CNTL_STORE4_REG bit0 is 0 (can be 1 after deep sleep, software reset)
	 * and printf boot.
	 * print boot determined by GPIO and efuse, see ets_is_print_boot
	 */
	ets_install_uart_printf();
}
#endif

int64_t esp_timer_get_time(void)
{
	/*
		This function is used by apptrace code to implement timeouts.
		unfortunately esp32c5 does not support CPU cycle counter, so we have two options:
		1) Use some HW timer. It can be hard, because we need to ensure that it is initialized and
		possibly restore its state.
		2) Emulate timer by incrementing some var on every call.
		Stub flasher uses ESP_APPTRACE_TMO_INFINITE only, so this function won't be called by apptrace at all.
	*/
	return 0;
}

uint64_t stub_get_time(void)
{
	/* this function is used for perf measurements only.
		unfortunately esp32c5 does not support CPU cycle counter and usage of HW timer is problematic */
	return 0;
}

static inline bool esp_flash_encryption_enabled(void)
{
	uint32_t cnt = REG_GET_FIELD(EFUSE_RD_REPEAT_DATA1_REG, EFUSE_SPI_BOOT_CRYPT_CNT);
	// 3 bits wide, any odd number - 1 or 3 - bits set means encryption is on
	cnt = ((cnt >> 2) ^ (cnt >> 1) ^ cnt) & 0x1;
	return (cnt == 1);
}

esp_flash_enc_mode_t stub_get_flash_encryption_mode(void)
{
	static esp_flash_enc_mode_t s_mode = ESP_FLASH_ENC_MODE_DEVELOPMENT;
	static bool s_first = true;

	if (s_first) {
		if (!esp_flash_encryption_enabled())
			s_mode = ESP_FLASH_ENC_MODE_DISABLED;
		s_first = false;
		STUB_LOGD("flash_encryption_mode: %d\n", s_mode);
	}
	return s_mode;
}

static void stub_mmu_hal_map_region(uint32_t vaddr, uint32_t paddr, uint32_t len)
{
	uint32_t page_size_in_bytes = stub_mmu_hal_pages_to_bytes(1);
	uint32_t page_num = (len + page_size_in_bytes - 1) / page_size_in_bytes;
	uint32_t entry_id = 0;
	uint32_t mmu_val = stub_mmu_ll_format_paddr(paddr);	/* This is the physical address in the format that MMU
								 * supported */

	while (page_num) {
		entry_id = stub_mmu_ll_get_entry_id(vaddr);
		stub_mmu_ll_write_entry(entry_id, mmu_val);
		Cache_Invalidate_Addr(vaddr, page_size_in_bytes);
		STUB_LOGD("mmap page_num:%d entry_id:%d vaddr:%x mmu_val:%x size:%d page_size_in_bytes:%x\n",
			page_num, entry_id, vaddr, mmu_val, len, page_size_in_bytes);
		vaddr += page_size_in_bytes;
		mmu_val++;
		page_num--;
	}
}

static void stub_mmu_hal_unmap_region(uint32_t vaddr, uint32_t len)
{
	uint32_t page_size_in_bytes = stub_mmu_hal_pages_to_bytes(1);
	uint32_t page_num = (len + page_size_in_bytes - 1) / page_size_in_bytes;
	uint32_t entry_id = 0;

	while (page_num) {
		entry_id = stub_mmu_ll_get_entry_id(vaddr);
		stub_mmu_ll_set_entry_invalid(entry_id);
		STUB_LOGD("unmap page_num:%d entry_id:%d vaddr:%x page_size_in_bytes:%x\n",
			page_num, entry_id, vaddr, page_size_in_bytes);
		vaddr += page_size_in_bytes;
		page_num--;
	}
}

static int stub_flash_mmap(struct spiflash_map_req *req)
{
	uint32_t map_src = req->src_addr & (~(s_cache_mmu_config.page_size - 1));	/* start of the page */
	uint32_t map_size = req->src_addr - map_src + req->size;
	uint32_t saved_state = Cache_Suspend_Cache();

	req->vaddr_start = s_cache_mmu_config.vaddr0_start_addr;
	req->ptr = (void *)req->vaddr_start + req->src_addr - map_src;

	STUB_LOGD("map_ptr: %x size:%d req->src_addr:%x map_src:%x map_size:%x\n",
		req->ptr, req->size, req->src_addr, map_src, map_size);

	stub_mmu_hal_map_region(req->vaddr_start, req->src_addr, map_size);

	REG_CLR_BIT(STUB_CACHE_CTRL_REG, STUB_CACHE_BUS);

	Cache_Resume_Cache(saved_state);

	return 0;
}

static void stub_flash_ummap(const struct spiflash_map_req *req)
{
	uint32_t map_src = req->src_addr & (~(s_cache_mmu_config.page_size - 1));	/* start of the page */
	uint32_t map_size = req->src_addr - map_src + req->size;
	uint32_t saved_state = Cache_Suspend_Cache();
	stub_mmu_hal_unmap_region(req->vaddr_start, map_size);
	Cache_Resume_Cache(saved_state);
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
