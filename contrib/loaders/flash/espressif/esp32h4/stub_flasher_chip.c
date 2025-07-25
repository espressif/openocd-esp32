// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   ESP32-H4 specific flasher stub functions                              *
 *   Copyright (C) 2025 Espressif Systems Ltd.                             *
 ***************************************************************************/
#include <string.h>

#include <sdkconfig.h>

#include <esp32h4/rom/cache.h>
#include <esp32h4/rom/uart.h>
#include <esp32h4/rom/rtc.h>

#include <soc/rtc.h>
#include <soc/efuse_periph.h>
#include <soc/gpio_reg.h>
#include "soc/lp_clkrst_reg.h"
#include <soc/system_reg.h>
#include <soc/spi_mem_c_reg.h>
#include <soc/cache_reg.h>
#include <soc/ext_mem_defs.h>

#include <esp_rom_efuse.h>

#include <stub_flasher_int.h>
#include <stub_logger.h>
#include "stub_flasher_chip.h"

uint32_t g_stub_cpu_freq_hz = CONFIG_ESP32H4_DEFAULT_CPU_FREQ_MHZ * MHZ;

void stub_flash_cache_flush(void)
{
	/* we do not know breakpoint program address here, so invalidate the whole cache */
	Cache_Invalidate_All(CACHE_MAP_ICACHE0 | CACHE_MAP_ICACHE1 | CACHE_MAP_DCACHE);
}

void stub_flash_state_prepare(struct stub_flash_state *state)
{
	esp_rom_spiflash_attach(0, false);
}

void stub_flash_state_restore(struct stub_flash_state *state)
{
	/* Nothing to restore*/
}

int stub_cpu_clock_configure(int conf_reg_val)
{
	// TODO OCD-1148
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
	// TODO OCD-1147
	return false;
}

int stub_flash_read_buff(uint32_t addr, void *buffer, uint32_t size)
{
	return esp_rom_spiflash_read(addr, buffer, size);
}
