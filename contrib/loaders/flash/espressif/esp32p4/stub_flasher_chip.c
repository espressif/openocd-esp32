// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   ESP32-P4 specific flasher stub functions                              *
 *   Copyright (C) 2022 Espressif Systems Ltd.                             *
 ***************************************************************************/
#include <string.h>

#include <sdkconfig.h>

#include <esp32p4/rom/cache.h>
#include <esp32p4/rom/uart.h>
#include <esp32p4/rom/rtc.h>

#include <soc/rtc.h>
#include <soc/efuse_periph.h>
#include <soc/gpio_reg.h>
#include "soc/lp_clkrst_reg.h"
#include <soc/system_reg.h>

#include <hal/mmu_ll.h>

#include <esp_app_trace_membufs_proto.h>
#include <esp_rom_efuse.h>

#include <stub_flasher_int.h>
#include <stub_logger.h>
#include "stub_flasher_chip.h"

/* RTC related definitios */
#define PCR_SOC_CLK_MAX                 1 //CPU_CLK frequency is 160 MHz (source is PLL_CLK)

extern void spi_flash_attach(uint32_t ishspi, bool legacy);

uint32_t g_stub_cpu_freq_hz = CONFIG_ESP32P4_DEFAULT_CPU_FREQ_MHZ * MHZ;

int xPortInIsrContext(void)
{
	return 0;
}

void *esp_apptrace_uart_hw_get(int num, void **data)
{
	return NULL;
}

#define CACHE_MAP_L1_ICACHE_0 BIT(0)
#define CACHE_MAP_L1_ICACHE_1 BIT(1)
#define CACHE_MAP_L2_CACHE    BIT(5)
void stub_flash_cache_flush(void)
{
	/* we do not know breakpoint program address here, so invalidate the whole ICache */
	Cache_Invalidate_All(CACHE_MAP_L1_ICACHE_0 | CACHE_MAP_L1_ICACHE_1 | CACHE_MAP_L2_CACHE);
}

void stub_cache_configure(void) {}

void stub_cache_init(void) {}

void stub_flash_state_prepare(struct stub_flash_state *state)
{
	const uint32_t spiconfig = 0;	/* esp_rom_efuse_get_flash_gpio_info(); */

	spi_flash_attach(spiconfig, false);
}

void stub_flash_state_restore(struct stub_flash_state *state)
{
	/* we do not disable or store the cache settings. So, nothing to restore*/
	//Cache_WriteBack_Addr(CACHE_MAP_L2_CACHE, 0x4FF00000, 0x24000);
}

int stub_cpu_clock_configure(int conf_reg_val)
{
	uint32_t hp_clk_ctrl_reg = 0;

	/* set to maximum possible value */
	if (conf_reg_val == -1) {
		hp_clk_ctrl_reg = REG_READ(LP_CLKRST_HP_CLK_CTRL_REG);
		REG_WRITE(LP_CLKRST_HP_CLK_CTRL_REG,
			(hp_clk_ctrl_reg & ~LP_CLKRST_HP_ROOT_CLK_SRC_SEL_M)
				| (PCR_SOC_CLK_MAX << LP_CLKRST_HP_ROOT_CLK_SRC_SEL_S));
	} else { // restore old value
		hp_clk_ctrl_reg = conf_reg_val;
		REG_WRITE(LP_CLKRST_HP_CLK_CTRL_REG,
			(REG_READ(LP_CLKRST_HP_CLK_CTRL_REG) & ~LP_CLKRST_HP_ROOT_CLK_SRC_SEL_M)
				| (hp_clk_ctrl_reg & LP_CLKRST_HP_ROOT_CLK_SRC_SEL_M));
	}

	STUB_LOGD("hp_clk_ctrl_reg %x\n", hp_clk_ctrl_reg);

	return hp_clk_ctrl_reg;
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
		unfortunately esp32p4 does not support CPU cycle counter, so we have two options:
		1) Use some HW timer. It can be hard, because we need to ensure that it is initialized
		and possibly restore its state.
		2) Emulate timer by incrementing some var on every call.
		Stub flasher uses ESP_APPTRACE_TMO_INFINITE only, so this function won't be called by apptrace at all.
	*/
	return 0;
}

uint64_t stub_get_time(void)
{
	/* this function is used for perf measurements only.
		unfortunately esp32p4 does not support CPU cycle counter and usage of HW timer is problematic */
	return 0;
}

static inline bool esp_flash_encryption_enabled(void)
{
	return false;
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

int stub_flash_read_buff(uint32_t addr, void *buffer, uint32_t size)
{
	return esp_rom_spiflash_read(addr, buffer, size);
}
