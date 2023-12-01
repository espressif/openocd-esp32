// SPDX-License-Identifier: GPL-2.0-or-later

#include "esp32/rom/spi_flash.h"
#include "soc/spi_periph.h"
#include "spi_flash_defs.h"

#define SPI_IDX   1
#define OTH_IDX   0

extern esp_rom_spiflash_chip_t g_rom_spiflash_chip;

static inline bool is_issi_chip(const esp_rom_spiflash_chip_t *chip)
{
	return ((chip->device_id >> 16) & 0xff) == 0x9D;
}

esp_rom_spiflash_result_t esp_rom_spiflash_wait_idle(esp_rom_spiflash_chip_t *spi)
{
	uint32_t status;
	/* wait for spi control ready */
	while ((REG_READ(SPI_EXT2_REG(1)) & SPI_ST))
		;
	while ((REG_READ(SPI_EXT2_REG(0)) & SPI_ST))
		;
	/* wait for flash status ready */
	if (esp_rom_spiflash_read_status(spi, &status) != ESP_ROM_SPIFLASH_RESULT_OK)
		return ESP_ROM_SPIFLASH_RESULT_ERR;
	return ESP_ROM_SPIFLASH_RESULT_OK;
}

/* Modified version of esp_rom_spiflash_unlock() that replaces version in ROM.

   This works around a bug where esp_rom_spiflash_unlock sometimes reads the wrong
   high status byte (RDSR2 result) and then copies it back to the
   flash status, which can cause the CMP bit or Status Register
   Protect bit to become set.

   Like other ROM SPI functions, this function is not designed to be
   called directly from an RTOS environment without taking precautions
   about interrupts, CPU coordination, flash mapping. However some of
   the functions in esp_spi_flash.c call it.
 */
esp_rom_spiflash_result_t esp_rom_spiflash_unlock(void)
{
	uint32_t status;
	uint32_t new_status;

	esp_rom_spiflash_wait_idle(&g_rom_spiflash_chip);

	if (is_issi_chip(&g_rom_spiflash_chip)) {
		/* ISSI chips have different QE position */

		if (esp_rom_spiflash_read_status(&g_rom_spiflash_chip, &status) != ESP_ROM_SPIFLASH_RESULT_OK)
			return ESP_ROM_SPIFLASH_RESULT_ERR;

		/* Clear all bits in the mask.
		(This is different from ROM esp_rom_spiflash_unlock, which keeps all bits as-is.)
		*/
		new_status = status & (~ESP_ROM_SPIFLASH_BP_MASK_ISSI);
		/* Skip if nothing needs to be cleared. Otherwise will waste time waiting for the
		 * flash to clear nothing. */
		if (new_status == status)
			return ESP_ROM_SPIFLASH_RESULT_OK;

		CLEAR_PERI_REG_MASK(SPI_CTRL_REG(SPI_IDX), SPI_WRSR_2B);
	} else {
		if (esp_rom_spiflash_read_statushigh(&g_rom_spiflash_chip, &status) != ESP_ROM_SPIFLASH_RESULT_OK)
			return ESP_ROM_SPIFLASH_RESULT_ERR;

		/* Clear all bits except QE, if it is set.
		(This is different from ROM esp_rom_spiflash_unlock, which keeps all bits as-is.)
		*/
		new_status = status & ESP_ROM_SPIFLASH_QE;
		SET_PERI_REG_MASK(SPI_CTRL_REG(SPI_IDX), SPI_WRSR_2B);
	}

	esp_rom_spiflash_wait_idle(&g_rom_spiflash_chip);
	REG_WRITE(SPI_CMD_REG(SPI_IDX), SPI_FLASH_WREN);
	while (REG_READ(SPI_CMD_REG(SPI_IDX)) != 0)
		;

	esp_rom_spiflash_wait_idle(&g_rom_spiflash_chip);
	esp_rom_spiflash_result_t ret = esp_rom_spiflash_write_status(&g_rom_spiflash_chip, new_status);

	/* WEL bit should be cleared after operations regardless of writing succeed or not. */
	esp_rom_spiflash_wait_idle(&g_rom_spiflash_chip);
	REG_WRITE(SPI_CMD_REG(SPI_IDX), SPI_FLASH_WRDI);
	while (REG_READ(SPI_CMD_REG(SPI_IDX)) != 0)
		;

	return ret;
}
