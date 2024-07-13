// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   ESP32-S3 specific flasher stub functions                              *
 *   Copyright (C) 2021 Espressif Systems Ltd.                             *
 ***************************************************************************/
#include <string.h>

#include <sdkconfig.h>

#include <esp32s3/rom/cache.h>
#include <esp32s3/rom/efuse.h>
#include <esp32s3/rom/uart.h>
#include <esp32s3/rom/rtc.h>

#include <soc/rtc.h>
#include <soc/efuse_periph.h>
#include <soc/spi_mem_reg.h>
#include <soc/spi_reg.h>
#include <soc/extmem_reg.h>
#include <soc/system_reg.h>
#include <soc/gpio_reg.h>
#include <soc/mmu.h>

#include <xtensa/hal.h>
#include <esp_spi_flash.h>
#include <rtc_clk_common.h>

#include <stub_logger.h>
#include <stub_flasher_int.h>
#include "stub_flasher_chip.h"

// this works for SPI0 and SPI1 only
#define REG_SPI_BASE(i)     (DR_REG_SPI0_BASE - (i) * 0x1000)

#define ESP_FLASH_CHIP_MXIC_OCT         0xC2 /* Supported Octal Flash chip vendor id */
#define SPI_BUFF_BYTE_WRITE_NUM         32
#define SPI_BUFF_BYTE_READ_NUM          16

#define EFUSE_WR_DIS_SPI_BOOT_CRYPT_CNT BIT(4)

/* Cache MMU related definitions */
#define STUB_CACHE_BUS_PRO              EXTMEM_DCACHE_SHUT_CORE0_BUS
#define STUB_CACHE_BUS_APP              EXTMEM_DCACHE_SHUT_CORE1_BUS
#define STUB_MMU_DROM_VADDR             SOC_MMU_VADDR0_START_ADDR
#define STUB_MMU_DROM_PAGES_START       SOC_MMU_DROM0_PAGES_START
#define STUB_MMU_DROM_PAGES_END         SOC_MMU_DROM0_PAGES_END
#define STUB_MMU_TABLE                  SOC_MMU_DPORT_PRO_FLASH_MMU_TABLE /* 0x600c5000 */
#define STUB_MMU_INVALID_ENTRY_VAL      SOC_MMU_INVALID_ENTRY_VAL         /* 0x4000 */

uint32_t g_stub_cpu_freq_hz = CONFIG_ESP32S3_DEFAULT_CPU_FREQ_MHZ * MHZ;

extern bool ets_efuse_flash_octal_mode(void);
extern void spi_cache_mode_switch(uint32_t  modebit);
extern void spi_common_set_flash_cs_timing(void);

static void stub_spi_read_mode_config(void)
{
	uint32_t  modebit = 0;
	//clear old mode bit
	WRITE_PERI_REG(SPI_MEM_DDR_REG(0), 0);
	WRITE_PERI_REG(SPI_MEM_DDR_REG(1), 0);
	CLEAR_PERI_REG_MASK(PERIPHS_SPI_FLASH_CTRL, SPI_MEM_FCMD_OCT | SPI_MEM_FCMD_DUAL | SPI_MEM_FCMD_QUAD
			| SPI_MEM_FREAD_QIO | SPI_MEM_FREAD_QUAD | SPI_MEM_FREAD_DIO | SPI_MEM_FREAD_DUAL | SPI_MEM_FASTRD_MODE
			| SPI_MEM_FADDR_OCT | SPI_MEM_FDIN_OCT | SPI_MEM_FDOUT_OCT);
	CLEAR_PERI_REG_MASK(SPI_MEM_CTRL_REG(0), SPI_MEM_FCMD_OCT | SPI_MEM_FCMD_DUAL | SPI_MEM_FCMD_QUAD
			| SPI_MEM_FREAD_QIO | SPI_MEM_FREAD_QUAD | SPI_MEM_FREAD_DIO | SPI_MEM_FREAD_DUAL | SPI_MEM_FASTRD_MODE
			| SPI_MEM_FADDR_OCT | SPI_MEM_FDIN_OCT | SPI_MEM_FDOUT_OCT);

	SET_PERI_REG_MASK(PERIPHS_SPI_FLASH_CTRL, modebit);
	SET_PERI_REG_MASK(SPI_MEM_CTRL_REG(0), modebit);
	spi_cache_mode_switch(modebit);
}

static void stub_spi_init(void)
{
	uint32_t modebit = 0;
	uint32_t freqbits;
	uint8_t freqdiv = 4;

	// Modified version of SPI_init(SpiFlashRdMode mode, uint8_t freqdiv) from esp_rom project
	// We do no reset the SPI module in order not to break communication with the PSRAM
	// Settings are done for mode SPI_FLASH_SLOWRD_MODE (5) and freqdiv SPI_CLK_DIV (4)

	REG_CLR_BIT(SPI_MEM_MISC_REG(0), SPI_MEM_CS0_DIS);
	REG_SET_BIT(SPI_MEM_MISC_REG(0), SPI_MEM_CS1_DIS);

	spi_common_set_flash_cs_timing();

	freqbits = (((freqdiv - 1) << SPI_MEM_CLKCNT_N_S)) |
		(((freqdiv / 2 - 1) << SPI_MEM_CLKCNT_H_S)) | ((freqdiv - 1) << SPI_MEM_CLKCNT_L_S);
	WRITE_PERI_REG(SPI_MEM_CLOCK_REG(1), freqbits);
	WRITE_PERI_REG(SPI_MEM_CLOCK_REG(0), freqbits);

	WRITE_PERI_REG(PERIPHS_SPI_FLASH_CTRL,  SPI_MEM_WP_REG | SPI_MEM_RESANDRES | modebit);
	WRITE_PERI_REG(SPI_MEM_CTRL_REG(0),  SPI_MEM_WP_REG | modebit);
	REG_SET_FIELD(SPI_MEM_MISO_DLEN_REG(0), SPI_MEM_USR_MISO_DBITLEN, 0xff);
	REG_SET_FIELD(SPI_MEM_MOSI_DLEN_REG(0), SPI_MEM_USR_MOSI_DBITLEN, 0xff);
	REG_SET_FIELD(SPI_MEM_USER2_REG(0), SPI_MEM_USR_COMMAND_BITLEN, 0x7);
	REG_SET_BIT(SPI_MEM_CACHE_FCTRL_REG(0), SPI_MEM_CACHE_REQ_EN);

	WRITE_PERI_REG(SPI_MEM_DDR_REG(0), 0);
	WRITE_PERI_REG(SPI_MEM_DDR_REG(1), 0);
	spi_cache_mode_switch(modebit);

	REG_SET_BIT(SPI_MEM_CACHE_FCTRL_REG(0), SPI_MEM_CACHE_FLASH_USR_CMD);
}

uint32_t stub_flash_get_id(void)
{
	STUB_LOGD("flash %x, cs %x, bs %x, ss %x, ps %x, sm %x\n",
		rom_spiflash_legacy_data->chip.device_id,
		rom_spiflash_legacy_data->chip.chip_size,
		rom_spiflash_legacy_data->chip.block_size,
		rom_spiflash_legacy_data->chip.sector_size,
		rom_spiflash_legacy_data->chip.page_size,
		rom_spiflash_legacy_data->chip.status_mask);

	if (rom_spiflash_legacy_data->dummy_len_plus[1] == 0) {
		REG_CLR_BIT(PERIPHS_SPI_FLASH_USRREG, SPI_MEM_USR_DUMMY);
	} else {
		REG_SET_BIT(PERIPHS_SPI_FLASH_USRREG, SPI_MEM_USR_DUMMY);
		REG_WRITE(PERIPHS_SPI_FLASH_USRREG1,
			(rom_spiflash_legacy_data->dummy_len_plus[1] - 1) << SPI_MEM_USR_DUMMY_CYCLELEN_S);
	}
	WRITE_PERI_REG(PERIPHS_SPI_FLASH_C0, 0); /* clear register */
	WRITE_PERI_REG(PERIPHS_SPI_FLASH_CMD, SPI_MEM_FLASH_RDID);
	while (READ_PERI_REG(PERIPHS_SPI_FLASH_CMD) != 0)
		;
	uint32_t ret = READ_PERI_REG(PERIPHS_SPI_FLASH_C0) & 0xffffff;
	STUB_LOGD("Flash ID read %x\n", ret);
	if (ets_efuse_flash_octal_mode() && (ret & 0xFF) != ESP_FLASH_CHIP_MXIC_OCT) {
		STUB_LOGE("Unsupported octal flash manufacturer\n");
		return 0;
	}
	return ret >> 16;
}

void stub_flash_cache_flush(void)
{
	/* we do not know breakpoint program address here, so invalidate the whole ICache */
	Cache_Invalidate_ICache_All();
}

static void stub_cache_init(uint32_t cpuid)
{
	STUB_LOGD("%s\n", __func__);

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

void stub_save_spi_regs(struct stub_flash_state *state)
{
	state->spi_regs[ESP32S3_STUB_FLASH_STATE_SPI_USER_REG_ID] = READ_PERI_REG(SPI_USER_REG(1));
	state->spi_regs[ESP32S3_STUB_FLASH_STATE_SPI_USER1_REG_ID] = READ_PERI_REG(SPI_USER1_REG(1));
	state->spi_regs[ESP32S3_STUB_FLASH_STATE_SPI_USER2_REG_ID] = READ_PERI_REG(SPI_USER2_REG(1));
	state->spi_regs[ESP32S3_STUB_FLASH_STATE_SPI_SLAVE_REG_ID] = READ_PERI_REG(SPI_SLAVE_REG(1));
	state->spi_regs[ESP32S3_STUB_FLASH_STATE_SPI_CLOCK_REG_ID] = READ_PERI_REG(SPI_CLOCK_REG(1));
	state->spi_regs[ESP32S3_STUB_FLASH_STATE_SPI_CTRL_REG_ID] = READ_PERI_REG(SPI_CTRL_REG(1));
	state->spi_regs[ESP32S3_STUB_FLASH_STATE_SPI_CMD_REG_ID] = READ_PERI_REG(SPI_CMD_REG(1));
	state->spi_regs[ESP32S3_STUB_FLASH_STATE_SPI_ADDR_REG_ID] = READ_PERI_REG(SPI_ADDR_REG(1));
	state->spi_regs[ESP32S3_STUB_FLASH_STATE_SPI_MS_DLEN_REG_ID] = READ_PERI_REG(SPI_MS_DLEN_REG(1));
	state->spi_regs[ESP32S3_STUB_FLASH_STATE_SPI_MISC_REG_ID] = READ_PERI_REG(SPI_MISC_REG(1));
	state->spi_regs[ESP32S3_STUB_FLASH_STATE_SPI_SLAVE1_REG_ID] = READ_PERI_REG(SPI_SLAVE1_REG(1));
	state->spi_regs[ESP32S3_STUB_FLASH_STATE_SPI_CLK_GATE_REG_ID] = READ_PERI_REG(SPI_CLK_GATE_REG(1));
	state->spi_regs[ESP32S3_STUB_FLASH_STATE_SPI_DIN_MODE_REG_ID] = READ_PERI_REG(SPI_DIN_MODE_REG(1));
	state->spi_regs[ESP32S3_STUB_FLASH_STATE_SPI_DIN_NUM_REG_ID] = READ_PERI_REG(SPI_DIN_NUM_REG(1));
	state->spi_regs[ESP32S3_STUB_FLASH_STATE_SPI_DOUT_MODE_REG_ID] = READ_PERI_REG(SPI_DOUT_MODE_REG(1));
	state->spi_regs[ESP32S3_STUB_FLASH_STATE_SPI_DMA_CONF_REG_ID] = READ_PERI_REG(SPI_DMA_CONF_REG(1));
	state->spi_regs[ESP32S3_STUB_FLASH_STATE_SPI_DMA_INT_ENA_REG_ID] = READ_PERI_REG(SPI_DMA_INT_ENA_REG(1));
	state->spi_regs[ESP32S3_STUB_FLASH_STATE_SPI_DMA_INT_CLR_REG_ID] = READ_PERI_REG(SPI_DMA_INT_CLR_REG(1));
	state->spi_regs[ESP32S3_STUB_FLASH_STATE_SPI_DMA_INT_RAW_REG_ID] = READ_PERI_REG(SPI_DMA_INT_RAW_REG(1));
	state->spi_regs[ESP32S3_STUB_FLASH_STATE_SPI_DMA_INT_ST_REG_ID] = READ_PERI_REG(SPI_DMA_INT_ST_REG(1));
	state->spi_regs[ESP32S3_STUB_FLASH_STATE_SPI_DMA_INT_SET_REG_ID] = READ_PERI_REG(SPI_DMA_INT_SET_REG(1));
	state->spi_regs[ESP32S3_STUB_FLASH_STATE_SPI_W0_REG_ID] = READ_PERI_REG(SPI_W0_REG(1));
	state->spi_regs[ESP32S3_STUB_FLASH_STATE_SPI_W1_REG_ID] = READ_PERI_REG(SPI_W1_REG(1));
	state->spi_regs[ESP32S3_STUB_FLASH_STATE_SPI_W2_REG_ID] = READ_PERI_REG(SPI_W2_REG(1));
	state->spi_regs[ESP32S3_STUB_FLASH_STATE_SPI_W3_REG_ID] = READ_PERI_REG(SPI_W3_REG(1));
	state->spi_regs[ESP32S3_STUB_FLASH_STATE_SPI_W4_REG_ID] = READ_PERI_REG(SPI_W4_REG(1));
	state->spi_regs[ESP32S3_STUB_FLASH_STATE_SPI_W5_REG_ID] = READ_PERI_REG(SPI_W5_REG(1));
	state->spi_regs[ESP32S3_STUB_FLASH_STATE_SPI_W6_REG_ID] = READ_PERI_REG(SPI_W6_REG(1));
	state->spi_regs[ESP32S3_STUB_FLASH_STATE_SPI_W7_REG_ID] = READ_PERI_REG(SPI_W7_REG(1));
	state->spi_regs[ESP32S3_STUB_FLASH_STATE_SPI_W8_REG_ID] = READ_PERI_REG(SPI_W8_REG(1));
	state->spi_regs[ESP32S3_STUB_FLASH_STATE_SPI_W9_REG_ID] = READ_PERI_REG(SPI_W9_REG(1));
	state->spi_regs[ESP32S3_STUB_FLASH_STATE_SPI_W10_REG_ID] = READ_PERI_REG(SPI_W10_REG(1));
	state->spi_regs[ESP32S3_STUB_FLASH_STATE_SPI_W11_REG_ID] = READ_PERI_REG(SPI_W11_REG(1));
	state->spi_regs[ESP32S3_STUB_FLASH_STATE_SPI_W12_REG_ID] = READ_PERI_REG(SPI_W12_REG(1));
	state->spi_regs[ESP32S3_STUB_FLASH_STATE_SPI_W13_REG_ID] = READ_PERI_REG(SPI_W13_REG(1));
	state->spi_regs[ESP32S3_STUB_FLASH_STATE_SPI_W14_REG_ID] = READ_PERI_REG(SPI_W14_REG(1));
	state->spi_regs[ESP32S3_STUB_FLASH_STATE_SPI_W15_REG_ID] = READ_PERI_REG(SPI_W15_REG(1));
	state->spi_regs[ESP32S3_STUB_FLASH_STATE_SPI_DATE_REG_ID] = READ_PERI_REG(SPI_DATE_REG(1));
	state->dummy_len_plus = g_rom_spiflash_dummy_len_plus[1];
	state->spi_regs_saved = true;
}

void stub_restore_spi_regs(struct stub_flash_state *state)
{
	WRITE_PERI_REG(SPI_USER_REG(1), state->spi_regs[ESP32S3_STUB_FLASH_STATE_SPI_USER_REG_ID]);
	WRITE_PERI_REG(SPI_USER1_REG(1), state->spi_regs[ESP32S3_STUB_FLASH_STATE_SPI_USER1_REG_ID]);
	WRITE_PERI_REG(SPI_USER2_REG(1), state->spi_regs[ESP32S3_STUB_FLASH_STATE_SPI_USER2_REG_ID]);
	WRITE_PERI_REG(SPI_SLAVE_REG(1), state->spi_regs[ESP32S3_STUB_FLASH_STATE_SPI_SLAVE_REG_ID]);
	WRITE_PERI_REG(SPI_CLOCK_REG(1), state->spi_regs[ESP32S3_STUB_FLASH_STATE_SPI_CLOCK_REG_ID]);
	WRITE_PERI_REG(SPI_CTRL_REG(1), state->spi_regs[ESP32S3_STUB_FLASH_STATE_SPI_CTRL_REG_ID]);
	WRITE_PERI_REG(SPI_CMD_REG(1), state->spi_regs[ESP32S3_STUB_FLASH_STATE_SPI_CMD_REG_ID]);
	WRITE_PERI_REG(SPI_ADDR_REG(1), state->spi_regs[ESP32S3_STUB_FLASH_STATE_SPI_ADDR_REG_ID]);
	WRITE_PERI_REG(SPI_MS_DLEN_REG(1), state->spi_regs[ESP32S3_STUB_FLASH_STATE_SPI_MS_DLEN_REG_ID]);
	WRITE_PERI_REG(SPI_MISC_REG(1), state->spi_regs[ESP32S3_STUB_FLASH_STATE_SPI_MISC_REG_ID]);
	WRITE_PERI_REG(SPI_SLAVE1_REG(1), state->spi_regs[ESP32S3_STUB_FLASH_STATE_SPI_SLAVE1_REG_ID]);
	WRITE_PERI_REG(SPI_CLK_GATE_REG(1), state->spi_regs[ESP32S3_STUB_FLASH_STATE_SPI_CLK_GATE_REG_ID]);
	WRITE_PERI_REG(SPI_DIN_MODE_REG(1), state->spi_regs[ESP32S3_STUB_FLASH_STATE_SPI_DIN_MODE_REG_ID]);
	WRITE_PERI_REG(SPI_DIN_NUM_REG(1), state->spi_regs[ESP32S3_STUB_FLASH_STATE_SPI_DIN_NUM_REG_ID]);
	WRITE_PERI_REG(SPI_DOUT_MODE_REG(1), state->spi_regs[ESP32S3_STUB_FLASH_STATE_SPI_DOUT_MODE_REG_ID]);
	WRITE_PERI_REG(SPI_DMA_CONF_REG(1), state->spi_regs[ESP32S3_STUB_FLASH_STATE_SPI_DMA_CONF_REG_ID]);
	WRITE_PERI_REG(SPI_DMA_INT_ENA_REG(1), state->spi_regs[ESP32S3_STUB_FLASH_STATE_SPI_DMA_INT_ENA_REG_ID]);
	WRITE_PERI_REG(SPI_DMA_INT_CLR_REG(1), state->spi_regs[ESP32S3_STUB_FLASH_STATE_SPI_DMA_INT_CLR_REG_ID]);
	WRITE_PERI_REG(SPI_DMA_INT_RAW_REG(1), state->spi_regs[ESP32S3_STUB_FLASH_STATE_SPI_DMA_INT_RAW_REG_ID]);
	WRITE_PERI_REG(SPI_DMA_INT_ST_REG(1), state->spi_regs[ESP32S3_STUB_FLASH_STATE_SPI_DMA_INT_ST_REG_ID]);
	WRITE_PERI_REG(SPI_DMA_INT_SET_REG(1), state->spi_regs[ESP32S3_STUB_FLASH_STATE_SPI_DMA_INT_SET_REG_ID]);
	WRITE_PERI_REG(SPI_W0_REG(1), state->spi_regs[ESP32S3_STUB_FLASH_STATE_SPI_W0_REG_ID]);
	WRITE_PERI_REG(SPI_W1_REG(1), state->spi_regs[ESP32S3_STUB_FLASH_STATE_SPI_W1_REG_ID]);
	WRITE_PERI_REG(SPI_W2_REG(1), state->spi_regs[ESP32S3_STUB_FLASH_STATE_SPI_W2_REG_ID]);
	WRITE_PERI_REG(SPI_W3_REG(1), state->spi_regs[ESP32S3_STUB_FLASH_STATE_SPI_W3_REG_ID]);
	WRITE_PERI_REG(SPI_W4_REG(1), state->spi_regs[ESP32S3_STUB_FLASH_STATE_SPI_W4_REG_ID]);
	WRITE_PERI_REG(SPI_W5_REG(1), state->spi_regs[ESP32S3_STUB_FLASH_STATE_SPI_W5_REG_ID]);
	WRITE_PERI_REG(SPI_W6_REG(1), state->spi_regs[ESP32S3_STUB_FLASH_STATE_SPI_W6_REG_ID]);
	WRITE_PERI_REG(SPI_W7_REG(1), state->spi_regs[ESP32S3_STUB_FLASH_STATE_SPI_W7_REG_ID]);
	WRITE_PERI_REG(SPI_W8_REG(1), state->spi_regs[ESP32S3_STUB_FLASH_STATE_SPI_W8_REG_ID]);
	WRITE_PERI_REG(SPI_W9_REG(1), state->spi_regs[ESP32S3_STUB_FLASH_STATE_SPI_W9_REG_ID]);
	WRITE_PERI_REG(SPI_W10_REG(1), state->spi_regs[ESP32S3_STUB_FLASH_STATE_SPI_W10_REG_ID]);
	WRITE_PERI_REG(SPI_W11_REG(1), state->spi_regs[ESP32S3_STUB_FLASH_STATE_SPI_W11_REG_ID]);
	WRITE_PERI_REG(SPI_W12_REG(1), state->spi_regs[ESP32S3_STUB_FLASH_STATE_SPI_W12_REG_ID]);
	WRITE_PERI_REG(SPI_W13_REG(1), state->spi_regs[ESP32S3_STUB_FLASH_STATE_SPI_W13_REG_ID]);
	WRITE_PERI_REG(SPI_W14_REG(1), state->spi_regs[ESP32S3_STUB_FLASH_STATE_SPI_W14_REG_ID]);
	WRITE_PERI_REG(SPI_W15_REG(1), state->spi_regs[ESP32S3_STUB_FLASH_STATE_SPI_W15_REG_ID]);
	WRITE_PERI_REG(SPI_DATE_REG(1), state->spi_regs[ESP32S3_STUB_FLASH_STATE_SPI_DATE_REG_ID]);

	g_rom_spiflash_dummy_len_plus[1] = state->dummy_len_plus;
	state->spi_regs_saved = false;
}

void stub_flash_state_prepare(struct stub_flash_state *state)
{
	uint32_t core_id = stub_get_coreid();
	uint32_t spiconfig = ets_efuse_get_spiconfig();

	state->spi_regs_saved = false;
	state->cache_enabled = stub_is_cache_enabled(core_id);
	if (!state->cache_enabled) {
		STUB_LOGI("Cache needs to be enabled for CPU%d\n", core_id);
		stub_cache_init(core_id);
	}

	if (ets_efuse_flash_octal_mode()) {
		static spiflash_legacy_funcs_t rom_default_spiflash_legacy_funcs = {
			.se_addr_bit_len = 24,
			.be_addr_bit_len = 24,
			.pp_addr_bit_len = 24,
			.rd_addr_bit_len = 24,
			.read_sub_len = SPI_BUFF_BYTE_READ_NUM,
			.write_sub_len = SPI_BUFF_BYTE_WRITE_NUM,
		};
		rom_spiflash_legacy_funcs = &rom_default_spiflash_legacy_funcs;
	}

	if ((READ_PERI_REG(SPI_MEM_CACHE_FCTRL_REG(0)) & SPI_MEM_CACHE_FLASH_USR_CMD) == 0) {
		STUB_LOGI("Attach spi flash...\n");
		esp_rom_spiflash_attach(spiconfig, 0);
	} else {
		stub_save_spi_regs(state);
		stub_spi_init();
		stub_spi_read_mode_config();
		if (ets_efuse_flash_octal_mode())
			esp_rom_opiflash_mode_reset(1);
	}

	STUB_LOGI("Flash state prepared...\n");
}

void stub_flash_state_restore(struct stub_flash_state *state)
{
	/* we do not disable or store the cache settings. So, nothing to restore*/
	if (state->spi_regs_saved)
		stub_restore_spi_regs(state);
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
		source_freq_mhz = (uint32_t)stub_rtc_clk_xtal_freq_get();
		freq_mhz = source_freq_mhz / div;
	}
	break;
	case DPORT_SOC_CLK_SEL_PLL: {
		source = RTC_CPU_FREQ_SRC_PLL;
		uint32_t cpuperiod_sel = REG_GET_FIELD(SYSTEM_CPU_PER_CONF_REG, SYSTEM_CPUPERIOD_SEL);
		uint32_t pllfreq_sel = REG_GET_FIELD(SYSTEM_CPU_PER_CONF_REG, SYSTEM_PLL_FREQ_SEL);
		source_freq_mhz = pllfreq_sel ? RTC_PLL_FREQ_480M : RTC_PLL_FREQ_320M;
		if (cpuperiod_sel == DPORT_CPUPERIOD_SEL_80) {
			div = source_freq_mhz == RTC_PLL_FREQ_480M ? 6 : 4;
			freq_mhz = 80;
		} else if (cpuperiod_sel == DPORT_CPUPERIOD_SEL_160) {
			div = source_freq_mhz == RTC_PLL_FREQ_480M ? 3 : 2;
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
		/* this return value will avoid undesired restore requests for unsupported frequency configuration */
		old_config.freq_mhz = 0;
	}

#if STUB_LOG_ENABLE == 1
	if (stub_get_log_dest() == STUB_LOG_DEST_UART)
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

#if STUB_LOG_ENABLE == 1
void stub_uart_console_configure(int dest)
{
	extern bool g_uart_print;
	/* set the default parameter to UART module, but don't enable RX interrupt */
	uartAttach(NULL);
	/* first enable uart0 as printf channel */
	uint32_t clock = ets_get_apb_freq();
	ets_update_cpu_frequency(clock / 1000000);

	Uart_Init(0, UART_CLK_FREQ_ROM);
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
			bool flash_crypt_cnt_wr_dis = REG_READ(EFUSE_RD_WR_DIS_REG) & EFUSE_WR_DIS_SPI_BOOT_CRYPT_CNT;
			if (!flash_crypt_cnt_wr_dis) {
				uint8_t flash_crypt_cnt = REG_GET_FIELD(EFUSE_RD_REPEAT_DATA1_REG, EFUSE_SPI_BOOT_CRYPT_CNT);
				/* Check if SPI_BOOT_CRYPT_CNT set for permanent encryption */
				if (flash_crypt_cnt == EFUSE_SPI_BOOT_CRYPT_CNT_V)
					flash_crypt_cnt_wr_dis = true;
			}

			if (flash_crypt_cnt_wr_dis) {
				uint8_t dis_dl_enc = REG_GET_FIELD(EFUSE_RD_REPEAT_DATA0_REG, EFUSE_DIS_DOWNLOAD_MANUAL_ENCRYPT);
				uint8_t dis_dl_icache = REG_GET_FIELD(EFUSE_RD_REPEAT_DATA0_REG, EFUSE_DIS_DOWNLOAD_ICACHE);
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
			STUB_MMU_TABLE[start_page + i] = SOC_MMU_PAGE_IN_FLASH(flash_page + i);

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

	STUB_LOGD("start_page: %d map_src: %x map_size: %x page_cnt: %d flash_page: %d map_ptr: %x\n",
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
