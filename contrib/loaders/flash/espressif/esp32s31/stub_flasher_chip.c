// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   ESP32-S31 specific flasher stub functions                              *
 *   Copyright (C) 2026 Espressif Systems Ltd.                             *
 ***************************************************************************/
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include <sdkconfig.h>

#include <esp32s31/rom/cache.h>
#include <esp32s31/rom/uart.h>

#include <soc/efuse_reg.h>
#include <soc/cache_reg.h>
#include <soc/ext_mem_defs.h>
#include <hal/mmu_ll.h>

#include <stub_flasher_int.h>
#include <stub_logger.h>
#include "stub_flasher_chip.h"

/* Cache MMU related definitions */
#define STUB_ICACHE_BUS                 (CACHE_L1_ICACHE_SHUT_IBUS0 | CACHE_L1_ICACHE_SHUT_IBUS1)
#define STUB_DCACHE_BUS                 (CACHE_L1_DCACHE_SHUT_DBUS0 | CACHE_L1_DCACHE_SHUT_DBUS1)
#define STUB_DROM_LOW                   SOC_DROM_LOW
#define STUB_MMU_DROM_PAGES_END         SOC_MMU_ENTRY_NUM
#define STUB_MMU_DROM_PAGES_START       (STUB_MMU_DROM_PAGES_END - 8) /* 8 pages will be more than enough */

// Compatibility
#define SOC_MMU_VALID			SOC_MMU_FLASH_VALID
#define SOC_MMU_SENSITIVE		SOC_MMU_FLASH_SENSITIVE
#define SOC_MMU_INVALID			SOC_MMU_FLASH_INVALID
#define SOC_MMU_VALID_VAL_MASK	SOC_MMU_FLASH_VALID_VAL_MASK
#define CACHE_MAP_ALL			(CACHE_MAP_L1_ICACHE_0 | CACHE_MAP_L1_ICACHE_1 | CACHE_MAP_L1_DCACHE)

struct cache_mmu_config {
	uint32_t page_size;
	uint32_t vaddr_base_addr;
	uint32_t drom_page_start;
	uint32_t drom_page_end;
	int shift_count;
};

static struct cache_mmu_config s_cache_mmu_config;

uint32_t g_stub_cpu_freq_hz = CONFIG_ESP32S31_DEFAULT_CPU_FREQ_MHZ * MHZ;

void stub_flash_cache_flush(void)
{
	/* we do not know breakpoint program address here, so invalidate the whole cache */
	Cache_Invalidate_All(CACHE_MAP_ALL);
}

static inline bool esp_flash_encryption_enabled(void)
{
	uint32_t flash_crypt_cnt = REG_GET_FIELD(EFUSE_RD_REPEAT_DATA0_REG, EFUSE_SPI_BOOT_CRYPT_CNT);
	// 3 bits wide, any odd number - 1 or 3 - bits set means encryption is on
	flash_crypt_cnt = ((flash_crypt_cnt >> 2) ^ (flash_crypt_cnt >> 1) ^ flash_crypt_cnt) & 0x1;
	return (flash_crypt_cnt == 1);
}

static inline uint32_t __attribute__((always_inline)) stub_mmu_hal_pages_to_bytes(uint32_t page_num)
{
	return page_num << s_cache_mmu_config.shift_count;
}

static inline uint32_t __attribute__((always_inline)) stub_mmu_ll_format_paddr(uint32_t paddr)
{
	return paddr >> s_cache_mmu_config.shift_count;
}

static inline uint32_t __attribute__((always_inline)) stub_mmu_ll_read_entry_raw(uint32_t entry_id)
{
	REG_WRITE(SPI_MEM_C_MMU_ITEM_INDEX_REG, entry_id);
	return REG_READ(SPI_MEM_C_MMU_ITEM_CONTENT_REG);
}

static inline __attribute__((always_inline)) uint32_t stub_mmu_ll_read_entry(uint32_t entry_id)
{
	uint32_t mmu_raw_value = stub_mmu_ll_read_entry_raw(entry_id);

	if (esp_flash_encryption_enabled())
		mmu_raw_value &= ~SOC_MMU_SENSITIVE;

	if (!(mmu_raw_value & SOC_MMU_VALID))
		return 0;

	return mmu_raw_value & SOC_MMU_VALID_VAL_MASK;
}

#define STUB_MMU_VADDR_MASK (s_cache_mmu_config.page_size * SOC_MMU_ENTRY_NUM - 1)
static inline uint32_t __attribute__((always_inline)) stub_mmu_ll_get_entry_id(uint32_t vaddr)
{
	return (vaddr & STUB_MMU_VADDR_MASK) >> s_cache_mmu_config.shift_count;
}

__attribute__((always_inline)) static inline void stub_mmu_ll_write_entry(uint32_t entry_id, uint32_t mmu_val)
{
	mmu_val |= (SOC_MMU_VALID | SOC_MMU_ACCESS_FLASH);

	if (esp_flash_encryption_enabled())
		mmu_val |= SOC_MMU_SENSITIVE;

	REG_WRITE(SPI_MEM_C_MMU_ITEM_INDEX_REG, entry_id);
	REG_WRITE(SPI_MEM_C_MMU_ITEM_CONTENT_REG, mmu_val);
}

static inline void __attribute__((always_inline)) stub_mmu_ll_set_entry_invalid(uint32_t entry_id)
{
	REG_WRITE(SPI_MEM_C_MMU_ITEM_INDEX_REG, entry_id);
	REG_WRITE(SPI_MEM_C_MMU_ITEM_CONTENT_REG, SOC_MMU_INVALID);
}

static inline __attribute__((always_inline)) void stub_mmu_ll_unmap_all(void)
{
	for (int i = 0; i < SOC_MMU_ENTRY_NUM; i++)
		stub_mmu_ll_set_entry_invalid(i);
}

static bool stub_is_cache_enabled(void)
{
	uint32_t icache_ctrl = REG_READ(CACHE_L1_ICACHE_CTRL_REG);
	uint32_t dcache_ctrl = REG_READ(CACHE_L1_DCACHE_CTRL_REG);

	STUB_LOGD("icache_ctrl_reg:%X dcache_ctrl_reg:%X\n", icache_ctrl, dcache_ctrl);

	bool bus_up = (icache_ctrl & STUB_ICACHE_BUS) == 0 && (dcache_ctrl & STUB_DCACHE_BUS) == 0;

	if (!bus_up)
		return false;

	for (int i = 0; i < SOC_MMU_ENTRY_NUM; i++) {
		uint32_t raw = stub_mmu_ll_read_entry_raw(i);

		if (esp_flash_encryption_enabled())
			raw &= ~SOC_MMU_SENSITIVE;

		if ((raw & SOC_MMU_VALID) && !(raw & SOC_MMU_ACCESS_PSRAM))
			return true;
	}
	return false;
}

static void stub_cache_configure(void)
{
	s_cache_mmu_config.page_size = mmu_ll_get_page_size(0);
	s_cache_mmu_config.drom_page_start = STUB_MMU_DROM_PAGES_START;
	s_cache_mmu_config.drom_page_end = STUB_MMU_DROM_PAGES_END;

	switch (s_cache_mmu_config.page_size) {
	case MMU_PAGE_256KB:
		s_cache_mmu_config.shift_count = 18;
		break;
	case MMU_PAGE_128KB:
		s_cache_mmu_config.shift_count = 17;
		break;
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

	s_cache_mmu_config.vaddr_base_addr = STUB_DROM_LOW +
		(s_cache_mmu_config.drom_page_start * s_cache_mmu_config.page_size);

	STUB_LOGI("MMU page size:%X drom_page_start:%d drom_page_end:%d vaddr_base_addr:%X\n",
		s_cache_mmu_config.page_size,
		s_cache_mmu_config.drom_page_start,
		s_cache_mmu_config.drom_page_end,
		s_cache_mmu_config.vaddr_base_addr);
}

static void stub_cache_init(void)
{
	STUB_LOGD("%s\n", __func__);

	esp_rom_spiflash_attach(0, false);
	ROM_Boot_Cache_Init();
}

void stub_flash_state_prepare(struct stub_flash_state *state)
{
	state->cache_enabled = stub_is_cache_enabled();
	STUB_LOGD("stub_is_cache_enabled=%d\n", state->cache_enabled);

	if (!state->cache_enabled) {
		STUB_LOGI("Cache needs to be enabled\n");
		stub_cache_init();
	}
	stub_cache_configure();
}

void stub_flash_state_restore(struct stub_flash_state *state)
{
	/* Nothing to restore*/
}

int stub_cpu_clock_configure(int conf_reg_val)
{
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

uint64_t stub_get_time(void)
{
	/* this function is used for perf measurements only.
		unfortunately esp32h4 does not support CPU cycle counter and usage of HW timer is problematic */
	return 0;
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

static uint32_t stub_mmu_cache_suspend(void)
{
	uint32_t cache_state = 0;

	cache_state |= Cache_Suspend_L1_CORE0_ICache();
	cache_state |= Cache_Suspend_L1_CORE1_ICache();
	cache_state |= Cache_Suspend_L1_DCache();
	return cache_state;
}

static void stub_mmu_cache_resume(uint32_t cache_state)
{
	Cache_Resume_L1_CORE0_ICache(cache_state);
	Cache_Resume_L1_CORE1_ICache(cache_state);
	Cache_Resume_L1_DCache(cache_state);
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
		Cache_Invalidate_Addr(CACHE_MAP_ALL, vaddr, page_size_in_bytes);
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
		Cache_Invalidate_Addr(CACHE_MAP_ALL, vaddr, page_size_in_bytes);
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
	uint32_t saved_state = stub_mmu_cache_suspend();

	req->vaddr_start = s_cache_mmu_config.vaddr_base_addr;
	req->ptr = (void *)req->vaddr_start + req->src_addr - map_src;

	STUB_LOGD("map_ptr: %x size:%d req->src_addr:%x map_src:%x map_size:%x\n",
		req->ptr, req->size, req->src_addr, map_src, map_size);

	stub_mmu_hal_map_region(req->vaddr_start, req->src_addr, map_size);

	stub_mmu_cache_resume(saved_state);

	return 0;
}

static void stub_flash_ummap(const struct spiflash_map_req *req)
{
	uint32_t map_src = req->src_addr & (~(s_cache_mmu_config.page_size - 1));	/* start of the page */
	uint32_t map_size = req->src_addr - map_src + req->size;
	uint32_t saved_state = stub_mmu_cache_suspend();
	stub_mmu_hal_unmap_region(req->vaddr_start, map_size);
	stub_mmu_cache_resume(saved_state);
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
