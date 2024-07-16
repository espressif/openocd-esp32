// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   ESP32-C2 specific flasher stub functions                              *
 *   Copyright (C) 2021 Espressif Systems Ltd.                             *
 ***************************************************************************/
#include <string.h>

#include <sdkconfig.h>

#include <esp32c2/rom/cache.h>
#include <esp32c2/rom/uart.h>
#include <esp32c2/rom/rtc.h>

#include <soc/rtc.h>
#include <soc/efuse_periph.h>
#include <soc/extmem_reg.h>
#include <soc/gpio_reg.h>
#include <soc/mmu.h>
#include <soc/system_reg.h>

#include <hal/mmu_ll.h>
#include <hal/systimer_ll.h>

#include <systimer.h>
#include <esp_rom_efuse.h>
#include <esp_app_trace_membufs_proto.h>

#include <stub_flasher_int.h>
#include <stub_logger.h>
#include "stub_flasher_chip.h"

/* RTC related definitios */
#define SYSTEM_SOC_CLK_MAX              1
#define SYSTEM_CPUPERIOD_MAX            1  // CPU_CLK frequency is 120 MHz

/* Cache MMU related definitions */
#define STUB_CACHE_BUS                  EXTMEM_ICACHE_SHUT_DBUS
#define STUB_MMU_TABLE                  ((volatile uint32_t *)DR_REG_MMU_TABLE)	/* 0x600c5000 */
#define STUB_MMU_INVALID_ENTRY_VAL      SOC_MMU_INVALID	/* BIT(6) */

typedef struct {
	mmu_page_size_t page_size;
	uint32_t vaddr0_start_addr;
	uint32_t drom_page_start;
	uint32_t drom_page_end;
} cache_mmu_config_t;

static cache_mmu_config_t s_cache_mmu_config;

static systimer_dev_t *s_sys_timer_dev = &SYSTIMER;
static uint32_t s_sys_timer_conf;

uint32_t g_stub_cpu_freq_hz = CONFIG_ESP32C2_DEFAULT_CPU_FREQ_MHZ * MHZ;

int xPortInIsrContext(void)
{
	return 0;
}

void *esp_apptrace_uart_hw_get(int num, void **data)
{
	return NULL;
}

void stub_flash_cache_flush(void)
{
	/* we do not know breakpoint program address here, so invalidate the whole ICache */
	Cache_Invalidate_ICache_All();
}

void stub_cache_init(void)
{
	STUB_LOGD("%s\n", __func__);

	esp_rom_spiflash_attach(0, false);

	Cache_Mask_All();
	/* init cache mmu, set cache mode, invalidate cache tags, enable cache*/
	REG_SET_BIT(SYSTEM_CACHE_CONTROL_REG, SYSTEM_ICACHE_CLK_ON);
	REG_SET_BIT(SYSTEM_CACHE_CONTROL_REG, SYSTEM_ICACHE_RESET);
	REG_CLR_BIT(SYSTEM_CACHE_CONTROL_REG, SYSTEM_ICACHE_RESET);
	/* init cache owner bit */
	Cache_Owner_Init();
	/* set page mode */
	mmu_ll_set_page_size(0, CONFIG_MMU_PAGE_SIZE);
	/* clear mmu entry */
	Cache_MMU_Init();
	/* config cache mode */
	Cache_Set_Default_Mode();
	Cache_Enable_ICache(0);
	REG_CLR_BIT(EXTMEM_ICACHE_CTRL1_REG, STUB_CACHE_BUS);
}

bool stub_is_cache_enabled(void)
{
	bool is_enabled = REG_GET_BIT(EXTMEM_ICACHE_CTRL_REG, EXTMEM_ICACHE_ENABLE) != 0;
	int cache_bus = REG_READ(EXTMEM_ICACHE_CTRL1_REG);
	STUB_LOGD("Cache is_enabled:%d cache_bus:%X\n", is_enabled, cache_bus);
	return is_enabled && !(cache_bus & STUB_CACHE_BUS);
}

void stub_cache_configure(void)
{
	s_cache_mmu_config.page_size = mmu_ll_get_page_size(0);

	switch (s_cache_mmu_config.page_size) {
	case MMU_PAGE_64KB:
		s_cache_mmu_config.drom_page_start = 2;
		break;
	case MMU_PAGE_32KB:
		s_cache_mmu_config.drom_page_start = 1;
		break;
	case MMU_PAGE_16KB:
		s_cache_mmu_config.drom_page_start = 3;
		break;
	default:
		STUB_LOGE("Unknown page size!");
		return;
	}

	s_cache_mmu_config.drom_page_end = 64;
	s_cache_mmu_config.vaddr0_start_addr =
		(SOC_DROM_LOW + (s_cache_mmu_config.drom_page_start * s_cache_mmu_config.page_size));

	STUB_LOGI("MMU page size:%X drom_page_start:%d drom_page_end:%d vaddr0_start_addr:%X\n",
		s_cache_mmu_config.page_size,
		s_cache_mmu_config.drom_page_start,
		s_cache_mmu_config.drom_page_end,
		s_cache_mmu_config.vaddr0_start_addr);
}

void stub_systimer_init(void)
{
	/* Enable APB_CLK signal if not enabled yet */
	if (!GET_PERI_REG_MASK(SYSTEM_PERIP_CLK_EN0_REG, SYSTEM_SYSTIMER_CLK_EN)) {
		SET_PERI_REG_MASK(SYSTEM_PERIP_CLK_EN0_REG, SYSTEM_SYSTIMER_CLK_EN);
		CLEAR_PERI_REG_MASK(SYSTEM_PERIP_RST_EN0_REG, SYSTEM_SYSTIMER_RST);
		STUB_LOGI("Systimer clock enabled\n");
	}

	s_sys_timer_conf = s_sys_timer_dev->conf.val;

	/* enable SYSTIMER_LL_COUNTER_CLOCK */
	s_sys_timer_dev->conf.clk_en = 1;
	s_sys_timer_dev->conf.timer_unit0_work_en = 1;
	s_sys_timer_dev->conf.timer_unit0_core0_stall_en = 0;
}

void stub_flash_state_prepare(struct stub_flash_state *state)
{
	state->cache_enabled = stub_is_cache_enabled();
	if (!state->cache_enabled) {
		STUB_LOGI("Cache needs to be enabled\n");
		stub_cache_init();
	}
	stub_cache_configure();
	stub_systimer_init();
}

void stub_flash_state_restore(struct stub_flash_state *state)
{
	/* we do not disable or store the cache settings. So, nothing to restore*/

	/* restore timer settings */
	s_sys_timer_dev->conf.val = s_sys_timer_conf;
}

int stub_cpu_clock_configure(int conf_reg_val)
{
	uint32_t sysclk_conf_reg = 0;

	/* set to maximum possible value */
	if (conf_reg_val == -1) {
		sysclk_conf_reg = REG_READ(SYSTEM_SYSCLK_CONF_REG) & 0xFFF;
		REG_WRITE(SYSTEM_SYSCLK_CONF_REG,
			(sysclk_conf_reg & ~SYSTEM_SOC_CLK_SEL_M) | (SYSTEM_SOC_CLK_MAX << SYSTEM_SOC_CLK_SEL_S));
	} else { // restore old value
		sysclk_conf_reg = conf_reg_val;
		REG_WRITE(SYSTEM_SYSCLK_CONF_REG,
			(REG_READ(SYSTEM_SYSCLK_CONF_REG) & ~SYSTEM_SOC_CLK_SEL_M) | (sysclk_conf_reg & SYSTEM_SOC_CLK_SEL_M));
	}

	STUB_LOGD("sysclk_conf_reg %x\n", sysclk_conf_reg);

	return conf_reg_val;
}

#if STUB_LOG_ENABLE == 1
void stub_uart_console_configure(int dest)
{
	extern bool g_uart_print;
	/* set the default parameter to UART module, but don't enable RX interrupt */
	uartAttach(NULL);
	/* first enable uart0 as printf channel */
	uint32_t clock = ets_get_apb_freq();
	ets_update_cpu_frequency(clock / 1000000);

	Uart_Init(0, APB_CLK_FREQ_ROM);
	/* install to print later
	 * Non-Flash Boot can print
	 * Flash Boot can print when RTC_CNTL_STORE4_REG bit0 is 0 (can be 1 after deep sleep, software reset)
	 * and printf boot.
	 * print boot determined by GPIO and efuse, see ets_is_print_boot
	 */
	g_uart_print = true;
	ets_install_uart_printf();
}
#endif

/* this function is used for perf measurements only.*/
uint64_t stub_get_time(void)
{
	uint32_t lo, lo_start, hi;

	/* Set the "update" bit and wait for acknowledgment */
	systimer_ll_counter_snapshot(s_sys_timer_dev, SYSTIMER_COUNTER_OS_TICK);
	while (!systimer_ll_is_counter_value_valid(s_sys_timer_dev, SYSTIMER_COUNTER_OS_TICK))
		;

	/* Read LO, HI, then LO again, check that LO returns the same value.
	* This accounts for the case when an interrupt may happen between reading
	* HI and LO values, and this function may get called from the ISR.
	* In this case, the repeated read will return consistent values.
	*/
	lo_start = systimer_ll_get_counter_value_low(s_sys_timer_dev, SYSTIMER_COUNTER_OS_TICK);
	do {
		lo = lo_start;
		hi = systimer_ll_get_counter_value_high(s_sys_timer_dev, SYSTIMER_COUNTER_OS_TICK);
		lo_start = systimer_ll_get_counter_value_low(s_sys_timer_dev, SYSTIMER_COUNTER_OS_TICK);
	} while (lo_start != lo);

	return systimer_ticks_to_us(((uint64_t)hi << 32) | lo);
}

/* this function is used by apptrace code to implement timeouts */
int64_t esp_timer_get_time(void)
{
	return (int64_t)stub_get_time();
}

static inline bool esp_flash_encryption_enabled(void)
{
	uint32_t flash_crypt_cnt = REG_GET_FIELD(EFUSE_RD_REPEAT_DATA0_REG, EFUSE_SPI_BOOT_CRYPT_CNT);

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
		if (!esp_flash_encryption_enabled())
			s_mode = ESP_FLASH_ENC_MODE_DISABLED;
		s_first = false;
		STUB_LOGD("flash_encryption_mode: %d\n", s_mode);
	}
	return s_mode;
}

static int stub_flash_mmap(struct spiflash_map_req *req)
{
	uint32_t map_src = req->src_addr & (~(s_cache_mmu_config.page_size - 1));
	uint32_t map_size = req->size + (req->src_addr - map_src);
	uint32_t flash_page = map_src / s_cache_mmu_config.page_size;
	uint32_t page_cnt = (map_size + s_cache_mmu_config.page_size - 1) / s_cache_mmu_config.page_size;
	int start_page, ret = ESP_ROM_SPIFLASH_RESULT_ERR;
	uint32_t saved_state = Cache_Suspend_ICache() << 16;

	for (start_page = s_cache_mmu_config.drom_page_start; start_page < s_cache_mmu_config.drom_page_end;
		++start_page) {
		if (STUB_MMU_TABLE[start_page] == STUB_MMU_INVALID_ENTRY_VAL)
			break;
	}

	if (start_page == s_cache_mmu_config.drom_page_end)
		start_page = s_cache_mmu_config.drom_page_start;

	if (start_page + page_cnt < s_cache_mmu_config.drom_page_end) {
		for (int i = 0; i < page_cnt; i++)
			STUB_MMU_TABLE[start_page + i] = SOC_MMU_PAGE_IN_FLASH(flash_page + i);

		req->start_page = start_page;
		req->page_cnt = page_cnt;
		req->ptr = (void *)(s_cache_mmu_config.vaddr0_start_addr +
			(start_page - s_cache_mmu_config.drom_page_start) * s_cache_mmu_config.page_size +
			(req->src_addr - map_src));
		Cache_Invalidate_Addr((uint32_t)(s_cache_mmu_config.vaddr0_start_addr +
				(start_page - s_cache_mmu_config.drom_page_start) * s_cache_mmu_config.page_size),
			page_cnt * s_cache_mmu_config.page_size);
		ret = ESP_ROM_SPIFLASH_RESULT_OK;
	}

	STUB_LOGD("start_page: %d map_src: %x map_size: %x page_cnt: %d flash_page: %d map_ptr: %x\n",
		start_page,
		map_src,
		map_size,
		page_cnt,
		flash_page,
		req->ptr);

	Cache_Resume_ICache(saved_state >> 16);

	return ret;
}

static void stub_flash_ummap(const struct spiflash_map_req *req)
{
	uint32_t saved_state = Cache_Suspend_ICache() << 16;

	for (int i = req->start_page; i < req->start_page + req->page_cnt; ++i)
		STUB_MMU_TABLE[i] = STUB_MMU_INVALID_ENTRY_VAL;

	Cache_Resume_ICache(saved_state >> 16);
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
