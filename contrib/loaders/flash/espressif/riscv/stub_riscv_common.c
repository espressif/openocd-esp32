// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   RiscV specific flasher stub functions                                 *
 *   Copyright (C) 2023 Espressif Systems Ltd.                             *
 ***************************************************************************/
#include <stub_flasher.h>
#include <stub_flasher_chip.h>
#include <stub_flasher_int.h>
#include <stub_logger.h>

#define RISCV_EBREAK        0x9002

void vPortEnterCritical(void)
{
}

void vPortExitCritical(void)
{
}

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
	WRITE_PERI_REG(PERIPHS_SPI_FLASH_C0, 0);	/* clear register */
	WRITE_PERI_REG(PERIPHS_SPI_FLASH_CMD, SPI_MEM_FLASH_RDID);
	while (READ_PERI_REG(PERIPHS_SPI_FLASH_CMD) != 0)
		;
	ret = READ_PERI_REG(PERIPHS_SPI_FLASH_C0) & 0xffffff;
	STUB_LOGD("Flash ID read %x\n", ret);
	return ret >> 16;
}

uint32_t stub_get_break_insn(uint8_t insn_sz)
{
	return RISCV_EBREAK;
}

uint8_t stub_get_insn_size(uint8_t *insn)
{
	/* we use 16bit `c.ebreak`. it works perfectly with either 32bit and 16bit code */
	return 2;
}

uint8_t stub_get_max_insn_size(void)
{
	return 2;
}
