/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) 2008 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
 *                                                                         *
 *   Copyright (C) 2011 Øyvind Harboe                                      *
 *   oyvind.harboe@zylin.com                                               *
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
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>. *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "imp.h"
#include <helper/binarybuffer.h>
#include <target/algorithm.h>
#include <target/image.h>
#include <target/register.h>
#include <target/esp108.h>

/* Regarding performance:
 *
 * Short story - it might be best to leave the performance at
 * current levels.
 *
 * You may see a jump in speed if you change to using
 * 32bit words for the block programming.
 *
 * Its a shame you cannot use the double word as its
 * even faster - but you require external VPP for that mode.
 *
 * Having said all that 16bit writes give us the widest vdd
 * operating range, so may be worth adding a note to that effect.
 *
 */

/* Danger!!!! The STM32F1x and STM32F2x series actually have
 * quite different flash controllers.
 *
 * What's more scary is that the names of the registers and their
 * addresses are the same, but the actual bits and what they do are
 * can be very different.
 *
 * To reduce testing complexity and dangers of regressions,
 * a seperate file is used for stm32fx2x.
 *
 * Sector sizes in kiBytes:
 * 1 MiByte part with 4 x 16, 1 x 64, 7 x 128.
 * 2 MiByte part with 4 x 16, 1 x 64, 7 x 128, 4 x 16, 1 x 64, 7 x 128.
 * 1 MiByte STM32F42x/43x part with DB1M Option set:
 *                    4 x 16, 1 x 64, 3 x 128, 4 x 16, 1 x 64, 3 x 128.
 *
 * STM32F7[4|5]
 * 1 MiByte part with 4 x 32, 1 x 128, 3 x 256.
 *
 * STM32F7[6|7]
 * 1 MiByte part in single bank mode with 4 x 32, 1 x 128, 3 x 256.
 * 1 MiByte part in dual-bank mode two banks with 4 x 16, 1 x 64, 3 x 128 each.
 * 2 MiByte part in single-bank mode with 4 x 32, 1 x 128, 7 x 256.
 * 2 MiByte part in dual-bank mode two banks with 4 x 16, 1 x 64, 7 x 128 each.
 *
 * Protection size is sector size.
 *
 * Tested with STM3220F-EVAL board.
 *
 * STM32F4xx series for reference.
 *
 * RM0090
 * http://www.st.com/web/en/resource/technical/document/reference_manual/DM00031020.pdf
 *
 * PM0059
 * www.st.com/internet/com/TECHNICAL_RESOURCES/TECHNICAL_LITERATURE/
 * PROGRAMMING_MANUAL/CD00233952.pdf
 *
 * STM32F7xx series for reference.
 *
 * RM0385
 * http://www.st.com/web/en/resource/technical/document/reference_manual/DM00124865.pdf
 *
 * RM0410
 * http://www.st.com/resource/en/reference_manual/dm00224583.pdf
 *
 * STM32F1x series - notice that this code was copy, pasted and knocked
 * into a stm32f2x driver, so in case something has been converted or
 * bugs haven't been fixed, here are the original manuals:
 *
 * RM0008 - Reference manual
 *
 * RM0042, the Flash programming manual for low-, medium- high-density and
 * connectivity line STM32F10x devices
 *
 * PM0068, the Flash programming manual for XL-density STM32F10x devices.
 *
 */

#define SPI_FLASH_SEC_SIZE  4096    /**< SPI Flash sector size */

#if 0
/* Erase time can be as high as 1000ms, 10x this and it's toast... */
#define FLASH_ERASE_TIMEOUT 10000
#define FLASH_WRITE_TIMEOUT 5

#define STM32_FLASH_BASE    0x40023c00
#define STM32_FLASH_ACR     0x40023c00
#define STM32_FLASH_KEYR    0x40023c04
#define STM32_FLASH_OPTKEYR 0x40023c08
#define STM32_FLASH_SR      0x40023c0C
#define STM32_FLASH_CR      0x40023c10
#define STM32_FLASH_OPTCR   0x40023c14
#define STM32_FLASH_OPTCR1  0x40023c18

/* FLASH_CR register bits */
#define FLASH_PG       (1 << 0)
#define FLASH_SER      (1 << 1)
#define FLASH_MER      (1 << 2)		/* MER/MER1 for f76x/77x */
#define FLASH_MER1     (1 << 15)	/* MER2 for f76x/77x, confusing ... */
#define FLASH_STRT     (1 << 16)
#define FLASH_PSIZE_8  (0 << 8)
#define FLASH_PSIZE_16 (1 << 8)
#define FLASH_PSIZE_32 (2 << 8)
#define FLASH_PSIZE_64 (3 << 8)
/* The sector number encoding is not straight binary for dual bank flash.
 * Warning: evaluates the argument multiple times */
#define FLASH_SNB(a)   ((((a) >= 12) ? 0x10 | ((a) - 12) : (a)) << 3)
#define FLASH_LOCK     (1 << 31)

/* FLASH_SR register bits */
#define FLASH_BSY      (1 << 16)
#define FLASH_PGSERR   (1 << 7) /* Programming sequence error */
#define FLASH_PGPERR   (1 << 6) /* Programming parallelism error */
#define FLASH_PGAERR   (1 << 5) /* Programming alignment error */
#define FLASH_WRPERR   (1 << 4) /* Write protection error */
#define FLASH_OPERR    (1 << 1) /* Operation error */

#define FLASH_ERROR (FLASH_PGSERR | FLASH_PGPERR | FLASH_PGAERR | FLASH_WRPERR | FLASH_OPERR)

/* STM32_FLASH_OPTCR register bits */
#define OPTCR_LOCK     (1 << 0)
#define OPTCR_START    (1 << 1)
#define OPTCR_NDBANK   (1 << 29)	/* not dual bank mode */
#define OPTCR_DB1M     (1 << 30)	/* 1 MiB devices dual flash bank option */

/* register unlock keys */
#define KEY1           0x45670123
#define KEY2           0xCDEF89AB

/* option register unlock key */
#define OPTKEY1        0x08192A3B
#define OPTKEY2        0x4C5D6E7F

struct stm32x_options {
	uint8_t RDP;
	uint16_t user_options;	/* bit 0-7 usual options, bit 8-11 extra options */
	uint32_t protection;
	uint32_t boot_addr;
};
#endif

struct esp32_flash_bank {
	//struct stm32x_options option_bytes;
	int probed;
	// bool has_large_mem;		/* F42x/43x/469/479/7xx in dual bank mode */
	// bool has_boot_addr;     /* F7xx */
	// bool has_extra_options; /* F42x/43x/469/479/7xx */
	uint32_t user_bank_size;
};

/* flash bank stm32x <base> <size> 0 0 <target#>
 */
FLASH_BANK_COMMAND_HANDLER(esp32_flash_bank_command)
{
	struct esp32_flash_bank *esp32_info;

	if (CMD_ARGC < 6)
		return ERROR_COMMAND_SYNTAX_ERROR;

	esp32_info = malloc(sizeof(struct esp32_flash_bank));
	bank->driver_priv = esp32_info;

	esp32_info->probed = 0;
	esp32_info->user_bank_size = bank->size;

	return ERROR_OK;
}

#if 0
static inline int stm32x_get_flash_reg(struct flash_bank *bank, uint32_t reg)
{
	return reg;
}

static inline int stm32x_get_flash_status(struct flash_bank *bank, uint32_t *status)
{
	struct target *target = bank->target;
	return target_read_u32(target, stm32x_get_flash_reg(bank, STM32_FLASH_SR), status);
}

static int stm32x_wait_status_busy(struct flash_bank *bank, int timeout)
{
	struct target *target = bank->target;
	uint32_t status;
	int retval = ERROR_OK;

	/* wait for busy to clear */
	for (;;) {
		retval = stm32x_get_flash_status(bank, &status);
		if (retval != ERROR_OK)
			return retval;
		LOG_DEBUG("status: 0x%" PRIx32 "", status);
		if ((status & FLASH_BSY) == 0)
			break;
		if (timeout-- <= 0) {
			LOG_ERROR("timed out waiting for flash");
			return ERROR_FAIL;
		}
		alive_sleep(1);
	}


	if (status & FLASH_WRPERR) {
		LOG_ERROR("stm32x device protected");
		retval = ERROR_FAIL;
	}

	/* Clear but report errors */
	if (status & FLASH_ERROR) {
		/* If this operation fails, we ignore it and report the original
		 * retval
		 */
		target_write_u32(target, stm32x_get_flash_reg(bank, STM32_FLASH_SR),
				status & FLASH_ERROR);
	}
	return retval;
}

static int stm32x_unlock_reg(struct target *target)
{
	uint32_t ctrl;

	/* first check if not already unlocked
	 * otherwise writing on STM32_FLASH_KEYR will fail
	 */
	int retval = target_read_u32(target, STM32_FLASH_CR, &ctrl);
	if (retval != ERROR_OK)
		return retval;

	if ((ctrl & FLASH_LOCK) == 0)
		return ERROR_OK;

	/* unlock flash registers */
	retval = target_write_u32(target, STM32_FLASH_KEYR, KEY1);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u32(target, STM32_FLASH_KEYR, KEY2);
	if (retval != ERROR_OK)
		return retval;

	retval = target_read_u32(target, STM32_FLASH_CR, &ctrl);
	if (retval != ERROR_OK)
		return retval;

	if (ctrl & FLASH_LOCK) {
		LOG_ERROR("flash not unlocked STM32_FLASH_CR: %" PRIx32, ctrl);
		return ERROR_TARGET_FAILURE;
	}

	return ERROR_OK;
}

static int stm32x_unlock_option_reg(struct target *target)
{
	uint32_t ctrl;

	int retval = target_read_u32(target, STM32_FLASH_OPTCR, &ctrl);
	if (retval != ERROR_OK)
		return retval;

	if ((ctrl & OPTCR_LOCK) == 0)
		return ERROR_OK;

	/* unlock option registers */
	retval = target_write_u32(target, STM32_FLASH_OPTKEYR, OPTKEY1);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u32(target, STM32_FLASH_OPTKEYR, OPTKEY2);
	if (retval != ERROR_OK)
		return retval;

	retval = target_read_u32(target, STM32_FLASH_OPTCR, &ctrl);
	if (retval != ERROR_OK)
		return retval;

	if (ctrl & OPTCR_LOCK) {
		LOG_ERROR("options not unlocked STM32_FLASH_OPTCR: %" PRIx32, ctrl);
		return ERROR_TARGET_FAILURE;
	}

	return ERROR_OK;
}

static int stm32x_read_options(struct flash_bank *bank)
{
	uint32_t optiondata;
	struct stm32x_flash_bank *stm32x_info = NULL;
	struct target *target = bank->target;

	stm32x_info = bank->driver_priv;

	/* read current option bytes */
	int retval = target_read_u32(target, STM32_FLASH_OPTCR, &optiondata);
	if (retval != ERROR_OK)
		return retval;

    /* caution: F2 implements 5 bits (WDG_SW only)
     * whereas F7 6 bits (IWDG_SW and WWDG_SW) in user_options */
	stm32x_info->option_bytes.user_options = optiondata & 0xfc;
	stm32x_info->option_bytes.RDP = (optiondata >> 8) & 0xff;
	stm32x_info->option_bytes.protection = (optiondata >> 16) & 0xfff;

	if (stm32x_info->has_extra_options) {
		/* F42x/43x/469/479 and 7xx have up to 4 bits of extra options */
		stm32x_info->option_bytes.user_options |= (optiondata >> 20) & 0xf00;
	}

	if (stm32x_info->has_large_mem || stm32x_info->has_boot_addr) {
		retval = target_read_u32(target, STM32_FLASH_OPTCR1, &optiondata);
		if (retval != ERROR_OK)
			return retval;

		/* FLASH_OPTCR1 has quite diffent meanings ... */
		if (stm32x_info->has_boot_addr) {
			/* for F7xx it contains boot0 and boot1 */
			stm32x_info->option_bytes.boot_addr = optiondata;
		} else {
			/* for F42x/43x/469/479 it contains 12 additional protection bits */
			stm32x_info->option_bytes.protection |= (optiondata >> 4) & 0x00fff000;
		}
	}

	if (stm32x_info->option_bytes.RDP != 0xAA)
		LOG_INFO("Device Security Bit Set");

	return ERROR_OK;
}

static int stm32x_write_options(struct flash_bank *bank)
{
	struct stm32x_flash_bank *stm32x_info = NULL;
	struct target *target = bank->target;
	uint32_t optiondata, optiondata2;

	stm32x_info = bank->driver_priv;

	int retval = stm32x_unlock_option_reg(target);
	if (retval != ERROR_OK)
		return retval;

	/* rebuild option data */
	optiondata = stm32x_info->option_bytes.user_options & 0xfc;
	optiondata |= stm32x_info->option_bytes.RDP << 8;
	optiondata |= (stm32x_info->option_bytes.protection & 0x0fff) << 16;

	if (stm32x_info->has_extra_options) {
		/* F42x/43x/469/479 and 7xx have up to 4 bits of extra options */
		optiondata |= (stm32x_info->option_bytes.user_options & 0xf00) << 20;
	}

	if (stm32x_info->has_large_mem || stm32x_info->has_boot_addr) {
		if (stm32x_info->has_boot_addr) {
			/* F7xx uses FLASH_OPTCR1 for boot0 and boot1 ... */
			optiondata2 = stm32x_info->option_bytes.boot_addr;
		} else {
			/* F42x/43x/469/479 uses FLASH_OPTCR1 for additional protection bits */
			optiondata2 = (stm32x_info->option_bytes.protection & 0x00fff000) << 4;
		}

		retval = target_write_u32(target, STM32_FLASH_OPTCR1, optiondata2);
		if (retval != ERROR_OK)
			return retval;
	}

	/* program options */
	retval = target_write_u32(target, STM32_FLASH_OPTCR, optiondata);
	if (retval != ERROR_OK)
		return retval;

	/* start programming cycle */
	retval = target_write_u32(target, STM32_FLASH_OPTCR, optiondata | OPTCR_START);
	if (retval != ERROR_OK)
		return retval;

	/* wait for completion */
	retval = stm32x_wait_status_busy(bank, FLASH_ERASE_TIMEOUT);
	if (retval != ERROR_OK)
		return retval;

	/* relock registers */
	retval = target_write_u32(target, STM32_FLASH_OPTCR, optiondata | OPTCR_LOCK);
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}
#endif

static int esp32_protect_check(struct flash_bank *bank)
{
#if 0
	struct stm32x_flash_bank *stm32x_info = bank->driver_priv;

	/* read write protection settings */
	int retval = stm32x_read_options(bank);
	if (retval != ERROR_OK) {
		LOG_DEBUG("unable to read option bytes");
		return retval;
	}

	if (stm32x_info->has_boot_addr && stm32x_info->has_large_mem) {
		/* F76x/77x: bit k protects sectors 2*k and 2*k+1 */
		for (int i = 0; i < (bank->num_sectors >> 1); i++) {
			if (stm32x_info->option_bytes.protection & (1 << i)) {
				bank->sectors[i << 1].is_protected = 0;
				bank->sectors[(i << 1) + 1].is_protected = 0;
			} else {
				bank->sectors[i << 1].is_protected = 1;
				bank->sectors[(i << 1) + 1].is_protected = 1;
			}
		}
	} else {
		/* one protection bit per sector */
		for (int i = 0; i < bank->num_sectors; i++) {
			if (stm32x_info->option_bytes.protection & (1 << i))
				bank->sectors[i].is_protected = 0;
			else
				bank->sectors[i].is_protected = 1;
		}
	}
#endif
	return ERROR_FAIL;
}

static int esp32_erase(struct flash_bank *bank, int first, int last)
{
	// struct target *target = bank->target;
	// int i;

	LOG_DEBUG("esp32_erase %d KB @ 0x%x %d..%d", bank->size/1024, bank->base, first, last);
#if 0
	assert((0 <= first) && (first <= last) && (last < bank->num_sectors));

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	int retval;
	retval = stm32x_unlock_reg(target);
	if (retval != ERROR_OK)
		return retval;

	/*
	Sector Erase
	To erase a sector, follow the procedure below:
	1. Check that no Flash memory operation is ongoing by checking the BSY bit in the
	  FLASH_SR register
	2. Set the SER bit and select the sector
	  you wish to erase (SNB) in the FLASH_CR register
	3. Set the STRT bit in the FLASH_CR register
	4. Wait for the BSY bit to be cleared
	 */

	for (i = first; i <= last; i++) {
		retval = target_write_u32(target,
				stm32x_get_flash_reg(bank, STM32_FLASH_CR), FLASH_SER | FLASH_SNB(i) | FLASH_STRT);
		if (retval != ERROR_OK)
			return retval;

		retval = stm32x_wait_status_busy(bank, FLASH_ERASE_TIMEOUT);
		if (retval != ERROR_OK)
			return retval;

		bank->sectors[i].is_erased = 1;
	}

	retval = target_write_u32(target, stm32x_get_flash_reg(bank, STM32_FLASH_CR), FLASH_LOCK);
	if (retval != ERROR_OK)
		return retval;
#endif
	return ERROR_FAIL;
}

static int esp32_protect(struct flash_bank *bank, int set, int first, int last)
{
#if 0
	struct target *target = bank->target;
	struct stm32x_flash_bank *stm32x_info = bank->driver_priv;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* read protection settings */
	int retval = stm32x_read_options(bank);
	if (retval != ERROR_OK) {
		LOG_DEBUG("unable to read option bytes");
		return retval;
	}

	if (stm32x_info->has_boot_addr && stm32x_info->has_large_mem) {
		/* F76x/77x: bit k protects sectors 2*k and 2*k+1 */
		if ((first & 1) != 0 || (last & 1) != 1) {
			LOG_ERROR("sector protection must be double sector aligned");
			return ERROR_FAIL;
		} else {
			first >>= 1;
			last >>= 1;
		}
	}

	for (int i = first; i <= last; i++) {
		if (set)
			stm32x_info->option_bytes.protection &= ~(1 << i);
		else
			stm32x_info->option_bytes.protection |= (1 << i);
	}

	retval = stm32x_write_options(bank);
	if (retval != ERROR_OK)
		return retval;
#endif
	return ERROR_FAIL;
}

#if 0
static int stm32x_write_block(struct flash_bank *bank, const uint8_t *buffer,
		uint32_t offset, uint32_t count)
{
	struct target *target = bank->target;
	uint32_t buffer_size = 16384;
	struct working_area *write_algorithm;
	struct working_area *source;
	uint32_t address = bank->base + offset;
	struct reg_param reg_params[5];
	struct armv7m_algorithm armv7m_info;
	int retval = ERROR_OK;

	/* see contrib/loaders/flash/stm32f2x.S for src */

	static const uint8_t stm32x_flash_write_code[] = {
									/* wait_fifo: */
		0xD0, 0xF8, 0x00, 0x80,		/* ldr		r8, [r0, #0] */
		0xB8, 0xF1, 0x00, 0x0F,		/* cmp		r8, #0 */
		0x1A, 0xD0,					/* beq		exit */
		0x47, 0x68,					/* ldr		r7, [r0, #4] */
		0x47, 0x45,					/* cmp		r7, r8 */
		0xF7, 0xD0,					/* beq		wait_fifo */

		0xDF, 0xF8, 0x34, 0x60,		/* ldr		r6, STM32_PROG16 */
		0x26, 0x61,					/* str		r6, [r4, #STM32_FLASH_CR_OFFSET] */
		0x37, 0xF8, 0x02, 0x6B,		/* ldrh		r6, [r7], #0x02 */
		0x22, 0xF8, 0x02, 0x6B,		/* strh		r6, [r2], #0x02 */
		0xBF, 0xF3, 0x4F, 0x8F,		/* dsb		sy */
									/* busy: */
		0xE6, 0x68,					/* ldr		r6, [r4, #STM32_FLASH_SR_OFFSET] */
		0x16, 0xF4, 0x80, 0x3F,		/* tst		r6, #0x10000 */
		0xFB, 0xD1,					/* bne		busy */
		0x16, 0xF0, 0xF0, 0x0F,		/* tst		r6, #0xf0 */
		0x07, 0xD1,					/* bne		error */

		0x8F, 0x42,					/* cmp		r7, r1 */
		0x28, 0xBF,					/* it		cs */
		0x00, 0xF1, 0x08, 0x07,		/* addcs	r7, r0, #8 */
		0x47, 0x60,					/* str		r7, [r0, #4] */
		0x01, 0x3B,					/* subs		r3, r3, #1 */
		0x13, 0xB1,					/* cbz		r3, exit */
		0xDF, 0xE7,					/* b		wait_fifo */
									/* error: */
		0x00, 0x21,					/* movs		r1, #0 */
		0x41, 0x60,					/* str		r1, [r0, #4] */
									/* exit: */
		0x30, 0x46,					/* mov		r0, r6 */
		0x00, 0xBE,					/* bkpt		#0x00 */

		/* <STM32_PROG16>: */
		0x01, 0x01, 0x00, 0x00,		/* .word	0x00000101 */
	};

	if (target_alloc_working_area(target, sizeof(stm32x_flash_write_code),
			&write_algorithm) != ERROR_OK) {
		LOG_WARNING("no working area available, can't do block memory writes");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	retval = target_write_buffer(target, write_algorithm->address,
			sizeof(stm32x_flash_write_code),
			stm32x_flash_write_code);
	if (retval != ERROR_OK)
		return retval;

	/* memory buffer */
	while (target_alloc_working_area_try(target, buffer_size, &source) != ERROR_OK) {
		buffer_size /= 2;
		if (buffer_size <= 256) {
			/* we already allocated the writing code, but failed to get a
			 * buffer, free the algorithm */
			target_free_working_area(target, write_algorithm);

			LOG_WARNING("no large enough working area available, can't do block memory writes");
			return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		}
	}

	armv7m_info.common_magic = ARMV7M_COMMON_MAGIC;
	armv7m_info.core_mode = ARM_MODE_THREAD;

	init_reg_param(&reg_params[0], "r0", 32, PARAM_IN_OUT);		/* buffer start, status (out) */
	init_reg_param(&reg_params[1], "r1", 32, PARAM_OUT);		/* buffer end */
	init_reg_param(&reg_params[2], "r2", 32, PARAM_OUT);		/* target address */
	init_reg_param(&reg_params[3], "r3", 32, PARAM_OUT);		/* count (halfword-16bit) */
	init_reg_param(&reg_params[4], "r4", 32, PARAM_OUT);		/* flash base */

	buf_set_u32(reg_params[0].value, 0, 32, source->address);
	buf_set_u32(reg_params[1].value, 0, 32, source->address + source->size);
	buf_set_u32(reg_params[2].value, 0, 32, address);
	buf_set_u32(reg_params[3].value, 0, 32, count);
	buf_set_u32(reg_params[4].value, 0, 32, STM32_FLASH_BASE);

	retval = target_run_flash_async_algorithm(target, buffer, count, 2,
			0, NULL,
			5, reg_params,
			source->address, source->size,
			write_algorithm->address, 0,
			&armv7m_info);

	if (retval == ERROR_FLASH_OPERATION_FAILED) {
		LOG_ERROR("error executing stm32x flash write algorithm");

		uint32_t error = buf_get_u32(reg_params[0].value, 0, 32) & FLASH_ERROR;

		if (error & FLASH_WRPERR)
			LOG_ERROR("flash memory write protected");

		if (error != 0) {
			LOG_ERROR("flash write failed = %08" PRIx32, error);
			/* Clear but report errors */
			target_write_u32(target, STM32_FLASH_SR, error);
			retval = ERROR_FAIL;
		}
	}

	target_free_working_area(target, source);
	target_free_working_area(target, write_algorithm);

	destroy_reg_param(&reg_params[0]);
	destroy_reg_param(&reg_params[1]);
	destroy_reg_param(&reg_params[2]);
	destroy_reg_param(&reg_params[3]);
	destroy_reg_param(&reg_params[4]);

	return retval;
}
#endif

static int esp32_write(struct flash_bank *bank, const uint8_t *buffer,
		uint32_t offset, uint32_t count)
{
#if 0
	struct target *target = bank->target;
	uint32_t words_remaining = (count / 2);
	uint32_t bytes_remaining = (count & 0x00000001);
	uint32_t address = bank->base + offset;
	uint32_t bytes_written = 0;
	int retval;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (offset & 0x1) {
		LOG_WARNING("offset 0x%" PRIx32 " breaks required 2-byte alignment", offset);
		return ERROR_FLASH_DST_BREAKS_ALIGNMENT;
	}

	retval = stm32x_unlock_reg(target);
	if (retval != ERROR_OK)
		return retval;

	/* multiple half words (2-byte) to be programmed? */
	if (words_remaining > 0) {
		/* try using a block write */
		retval = stm32x_write_block(bank, buffer, offset, words_remaining);
		if (retval != ERROR_OK) {
			if (retval == ERROR_TARGET_RESOURCE_NOT_AVAILABLE) {
				/* if block write failed (no sufficient working area),
				 * we use normal (slow) single dword accesses */
				LOG_WARNING("couldn't use block writes, falling back to single memory accesses");
			}
		} else {
			buffer += words_remaining * 2;
			address += words_remaining * 2;
			words_remaining = 0;
		}
	}

	if ((retval != ERROR_OK) && (retval != ERROR_TARGET_RESOURCE_NOT_AVAILABLE))
		return retval;

	/*
	Standard programming
	The Flash memory programming sequence is as follows:
	1. Check that no main Flash memory operation is ongoing by checking the BSY bit in the
	  FLASH_SR register.
	2. Set the PG bit in the FLASH_CR register
	3. Perform the data write operation(s) to the desired memory address (inside main
	  memory block or OTP area):
	– – Half-word access in case of x16 parallelism
	– Word access in case of x32 parallelism
	–
	4.
	Byte access in case of x8 parallelism
	Double word access in case of x64 parallelism
	Wait for the BSY bit to be cleared
	*/
	while (words_remaining > 0) {
		uint16_t value;
		memcpy(&value, buffer + bytes_written, sizeof(uint16_t));

		retval = target_write_u32(target, stm32x_get_flash_reg(bank, STM32_FLASH_CR),
				FLASH_PG | FLASH_PSIZE_16);
		if (retval != ERROR_OK)
			return retval;

		retval = target_write_u16(target, address, value);
		if (retval != ERROR_OK)
			return retval;

		retval = stm32x_wait_status_busy(bank, FLASH_WRITE_TIMEOUT);
		if (retval != ERROR_OK)
			return retval;

		bytes_written += 2;
		words_remaining--;
		address += 2;
	}

	if (bytes_remaining) {
		retval = target_write_u32(target, stm32x_get_flash_reg(bank, STM32_FLASH_CR),
				FLASH_PG | FLASH_PSIZE_8);
		if (retval != ERROR_OK)
			return retval;
		retval = target_write_u8(target, address, buffer[bytes_written]);
		if (retval != ERROR_OK)
			return retval;

		retval = stm32x_wait_status_busy(bank, FLASH_WRITE_TIMEOUT);
		if (retval != ERROR_OK)
			return retval;
	}

	return target_write_u32(target, STM32_FLASH_CR, FLASH_LOCK);
#endif
	return ERROR_FAIL;
}

#if 0
static int stm32x_get_device_id(struct flash_bank *bank, uint32_t *device_id)
{
	/* this checks for a stm32f4x errata issue where a
	 * stm32f2x DBGMCU_IDCODE is incorrectly returned.
	 * If the issue is detected target is forced to stm32f4x Rev A.
	 * Only effects Rev A silicon */

	struct target *target = bank->target;
	uint32_t cpuid;

	/* read stm32 device id register */
	int retval = target_read_u32(target, 0xE0042000, device_id);
	if (retval != ERROR_OK)
		return retval;

	if ((*device_id & 0xfff) == 0x411) {
		/* read CPUID reg to check core type */
		retval = target_read_u32(target, 0xE000ED00, &cpuid);
		if (retval != ERROR_OK)
			return retval;

		/* check for cortex_m4 */
		if (((cpuid >> 4) & 0xFFF) == 0xC24) {
			*device_id &= ~((0xFFFF << 16) | 0xfff);
			*device_id |= (0x1000 << 16) | 0x413;
			LOG_INFO("stm32f4x errata detected - fixing incorrect MCU_IDCODE");
		}
	}
	return retval;
}
#endif

#define ESP32_STUB_STACK_SZ		(256+16)
//TODO: retrive stack addr automatically
#define ESP32_STUB_STACK_ADDR	0x3FFFF000UL
static int esp32_run_stub(struct target *target)
{
#if ESP32_STUB_STACK_ADDR == 0
	struct working_area *stack;
#endif
	struct working_area *write_algorithm;
	struct reg_param reg_params[6];
	struct xtensa_algorithm algorithm_info;
	uint32_t stack_addr;
	static const uint8_t esp32_stub[] = {
		#include "esp32_stub_code.inc"
	};
	struct image image;
	uint8_t sec_buf[512];

	int retval = target_halt(target);
	if (retval != ERROR_OK) {
		LOG_ERROR("Failed to halt target (%d)!", retval);
		return retval;
	}
	retval = target_wait_state(target, TARGET_HALTED, 1000);
	if (retval != ERROR_OK) {
		LOG_ERROR("Failed to wait halted target (%d)!", retval);
		return retval;
	}

	image.base_address_set = 1;
	image.base_address = 0;
	image.start_address_set = 0;
	retval = image_open(&image, "/home/alexey/projects/esp/esp-idf/components/openocd_stub/build/stub_flasher_32.elf", NULL);
	if (retval != ERROR_OK) {
		LOG_ERROR("Failed to open stub image (%d)!", retval);
		return retval;
	}
	//TODO: add parsing entry func prolog to select proper startup stub (callx0 or calxN)
	//TODO: put main stub into working area
	LOG_INFO("stub: base 0x%x, start 0x%x, %d sections", (unsigned)image.base_address, image.start_address, image.num_sections);
	for (int i = 0; i < image.num_sections; i++) {
		struct imagesection *section = &image.sections[i];
		LOG_INFO("addr %x, sz %d, flags %x", section->base_address, section->size, section->flags);
		uint32_t sec_wr = 0;
		while (sec_wr < section->size) {
			uint32_t nb = section->size - sec_wr > sizeof(sec_buf) ? sizeof(sec_buf) : section->size - sec_wr;
			size_t size_read = 0;
			retval = image_read_section(&image, i, sec_wr, nb, sec_buf, &size_read);
			if (retval != ERROR_OK) {
				LOG_ERROR("Failed to read stub section (%d)!", retval);
				image_close(&image);
				return retval;
			}
			retval = target_write_memory(target, section->base_address + sec_wr, 1, size_read, sec_buf);
			if (retval != ERROR_OK) {
				LOG_ERROR("Failed to write target memory (%d)!", retval);
				image_close(&image);
				return retval;
			}
			sec_wr += size_read;
		}
	}
	image_close(&image);
#if 1
	if (target_alloc_working_area(target, sizeof(esp32_stub), &write_algorithm) != ERROR_OK) {
		LOG_WARNING("no working area available, can't do block memory writes");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	retval = target_write_buffer(target, write_algorithm->address, sizeof(esp32_stub), esp32_stub);
	if (retval != ERROR_OK) {
		LOG_ERROR("Failed to write stub!");
		target_free_working_area(target, write_algorithm);
		return retval;
	}
#if ESP32_STUB_STACK_ADDR
	stack_addr = ESP32_STUB_STACK_ADDR + ESP32_STUB_STACK_SZ;
#else
	if (target_alloc_working_area(target, ESP32_STUB_STACK_SZ, &stack) != ERROR_OK) {
		LOG_WARNING("no working area available, can't alloc stack!");
		target_free_working_area(target, write_algorithm);
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}
	stack_addr = stack->address + ESP32_STUB_STACK_SZ;
#endif
	if (stack_addr % 16) {
		LOG_INFO("Adjust stack addr 0x%x", stack_addr);
		stack_addr &= ~0xFUL;
	}
#if 0
	/* memory buffer */
	while (target_alloc_working_area_try(target, buffer_size, &source) != ERROR_OK) {
		buffer_size /= 2;
		if (buffer_size <= 256) {
			/* we already allocated the writing code, but failed to get a
			 * buffer, free the algorithm */
			target_free_working_area(target, write_algorithm);

			LOG_WARNING("no large enough working area available, can't do block memory writes");
			return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		}
	}
	armv7m_info.common_magic = ARMV7M_COMMON_MAGIC;
	armv7m_info.core_mode = ARM_MODE_THREAD;
#endif
	algorithm_info.core_mode = XT_MODE_ANY;

	init_reg_param(&reg_params[0], "a2", 32, PARAM_IN_OUT);
	init_reg_param(&reg_params[1], "a1", 32, PARAM_OUT);
	init_reg_param(&reg_params[2], "a5", 32, PARAM_OUT);
	init_reg_param(&reg_params[3], "windowbase", 32, PARAM_OUT);
	init_reg_param(&reg_params[4], "windowstart", 32, PARAM_OUT);
	init_reg_param(&reg_params[5], "ps", 32, PARAM_OUT);

	buf_set_u32(reg_params[0].value, 0, 32, 5);
	buf_set_u32(reg_params[1].value, 0, 32, stack_addr);
	buf_set_u32(reg_params[2].value, 0, 32, image.start_address);//write_algorithm->address + 0x14);
	buf_set_u32(reg_params[3].value, 0, 32, 0x0);
	buf_set_u32(reg_params[4].value, 0, 32, 0x1);
	buf_set_u32(reg_params[5].value, 0, 32, 0x60021); // enable WOE, UM and debug interrupts level

	/*retval = target_run_flash_async_algorithm(target, buffer, count, 2,
			0, NULL,
			5, reg_params,
			source->address, source->size,
			write_algorithm->address, 0,
			&armv7m_info);*/

	LOG_INFO("Algorithm run @ 0x%x, stack @ 0x%x", write_algorithm->address, stack_addr);
	retval = target_run_algorithm(target,
			0, NULL,
			// sizeof(reg_params)/sizeof(struct reg_param), reg_params,
			6, reg_params,
			write_algorithm->address, 0,
			1000, &algorithm_info);
	// uint8_t ret_buf[4];
	// retval = target_read_buffer(target, stack_addr-4, 4, ret_buf);
	// if (retval != ERROR_OK) {
	// 	LOG_ERROR("Failed to read stub!");
	// }
	// LOG_INFO("Algorithm returned %d ret 0x%x / 0x%x", retval, buf_get_u32(reg_params[0].value, 0, 32), buf_get_u32(ret_buf, 0, 32));
	LOG_INFO("Algorithm returned %d ret 0x%x", retval, buf_get_u32(reg_params[0].value, 0, 32));
#if 0
	if (retval == ERROR_FLASH_OPERATION_FAILED) {
		LOG_ERROR("error executing stm32l4 flash write algorithm");

		uint32_t error = buf_get_u32(reg_params[0].value, 0, 32);

		if (error & FLASH_WRPERR)
			LOG_ERROR("flash memory write protected");

		if (error != 0) {
			LOG_ERROR("flash write failed = %08" PRIx32, error);
			/* Clear but report errors */
			target_write_u32(target, STM32_FLASH_SR, error);
			retval = ERROR_FAIL;
		}
	}
#endif
#if ESP32_STUB_STACK_ADDR == 0
	target_free_working_area(target, stack);
#endif
	target_free_working_area(target, write_algorithm);

	destroy_reg_param(&reg_params[5]);
	destroy_reg_param(&reg_params[4]);
	destroy_reg_param(&reg_params[3]);
	destroy_reg_param(&reg_params[2]);
	destroy_reg_param(&reg_params[1]);
	destroy_reg_param(&reg_params[0]);
#endif
	retval = target_resume(target, 1, 0, 1, 0);
	if (retval != ERROR_OK) {
		LOG_ERROR("Failed to resume target (%d)!", retval);
	}
	// retval = target_wait_state(target, TARGET_HALTED, 1000);
	// if (retval != ERROR_OK) {
	// 	LOG_ERROR("Failed to wait halted target (%d)!", retval);
	// } else {
	// 	LOG_INFO("Target HALTED!");
	// }
	// for (unsigned i = 0; i < target->reg_cache->num_regs; i++) {
	// 	LOG_INFO("REG '%s' = 0x%x!", target->reg_cache->reg_list[i].name, buf_get_u32(target->reg_cache->reg_list[i].value, 0, 32));
	// }

	return  retval;
}

static int esp32_probe(struct flash_bank *bank)
{
	struct esp32_flash_bank *esp32_info = bank->driver_priv;

	esp32_info->probed = 0;

	LOG_INFO("flash size = %dkbytes @ 0x%x", bank->size/1024, bank->base);

	if (bank->sectors) {
		free(bank->sectors);
		bank->sectors = NULL;
	}

	bank->num_sectors = bank->size / SPI_FLASH_SEC_SIZE;
	bank->sectors = malloc(sizeof(struct flash_sector) * bank->num_sectors);
	for (int i = 0; i < bank->num_sectors; i++) {
		bank->sectors[i].offset = i*SPI_FLASH_SEC_SIZE;
		bank->sectors[i].size = SPI_FLASH_SEC_SIZE;
		bank->sectors[i].is_erased = -1;
		bank->sectors[i].is_protected = 0;
	}
	LOG_DEBUG("allocated %d sectors", bank->num_sectors);

	int res = esp32_run_stub(bank->target);

	esp32_info->probed = 1;

	return res;
}

static int esp32_auto_probe(struct flash_bank *bank)
{
	struct esp32_flash_bank *esp32_info = bank->driver_priv;
	if (esp32_info->probed)
		return ERROR_OK;
	return esp32_probe(bank);
}

static int get_esp32_info(struct flash_bank *bank, char *buf, int buf_size)
{
	snprintf(buf, buf_size, "ESP32");
	return ERROR_OK;
}

#if 0
COMMAND_HANDLER(stm32x_handle_lock_command)
{
	struct target *target = NULL;
	struct stm32x_flash_bank *stm32x_info = NULL;

	if (CMD_ARGC < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (ERROR_OK != retval)
		return retval;

	stm32x_info = bank->driver_priv;
	target = bank->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (stm32x_read_options(bank) != ERROR_OK) {
		command_print(CMD_CTX, "%s failed to read options", bank->driver->name);
		return ERROR_OK;
	}

	/* set readout protection */
	stm32x_info->option_bytes.RDP = 0;

	if (stm32x_write_options(bank) != ERROR_OK) {
		command_print(CMD_CTX, "%s failed to lock device", bank->driver->name);
		return ERROR_OK;
	}

	command_print(CMD_CTX, "%s locked", bank->driver->name);

	return ERROR_OK;
}

COMMAND_HANDLER(stm32x_handle_unlock_command)
{
	struct target *target = NULL;
	struct stm32x_flash_bank *stm32x_info = NULL;

	if (CMD_ARGC < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (ERROR_OK != retval)
		return retval;

	stm32x_info = bank->driver_priv;
	target = bank->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (stm32x_read_options(bank) != ERROR_OK) {
		command_print(CMD_CTX, "%s failed to read options", bank->driver->name);
		return ERROR_OK;
	}

	/* clear readout protection and complementary option bytes
	 * this will also force a device unlock if set */
	stm32x_info->option_bytes.RDP = 0xAA;

	if (stm32x_write_options(bank) != ERROR_OK) {
		command_print(CMD_CTX, "%s failed to unlock device", bank->driver->name);
		return ERROR_OK;
	}

	command_print(CMD_CTX, "%s unlocked.\n"
			"INFO: a reset or power cycle is required "
			"for the new settings to take effect.", bank->driver->name);

	return ERROR_OK;
}

static int stm32x_mass_erase(struct flash_bank *bank)
{
	int retval;
	uint32_t flash_mer;
	struct target *target = bank->target;
	struct stm32x_flash_bank *stm32x_info = NULL;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	stm32x_info = bank->driver_priv;

	retval = stm32x_unlock_reg(target);
	if (retval != ERROR_OK)
		return retval;

	/* mass erase flash memory */
	if (stm32x_info->has_large_mem)
		flash_mer = FLASH_MER | FLASH_MER1;
	else
		flash_mer = FLASH_MER;

	retval = target_write_u32(target, stm32x_get_flash_reg(bank, STM32_FLASH_CR), flash_mer);
	if (retval != ERROR_OK)
		return retval;
	retval = target_write_u32(target, stm32x_get_flash_reg(bank, STM32_FLASH_CR),
		flash_mer | FLASH_STRT);
	if (retval != ERROR_OK)
		return retval;

	retval = stm32x_wait_status_busy(bank, 30000);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u32(target, stm32x_get_flash_reg(bank, STM32_FLASH_CR), FLASH_LOCK);
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}

COMMAND_HANDLER(stm32x_handle_mass_erase_command)
{
	int i;

	if (CMD_ARGC < 1) {
		command_print(CMD_CTX, "stm32x mass_erase <bank>");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (ERROR_OK != retval)
		return retval;

	retval = stm32x_mass_erase(bank);
	if (retval == ERROR_OK) {
		/* set all sectors as erased */
		for (i = 0; i < bank->num_sectors; i++)
			bank->sectors[i].is_erased = 1;

		command_print(CMD_CTX, "stm32x mass erase complete");
	} else {
		command_print(CMD_CTX, "stm32x mass erase failed");
	}

	return retval;
}

COMMAND_HANDLER(stm32f2x_handle_options_read_command)
{
	int retval;
	struct flash_bank *bank;
	struct stm32x_flash_bank *stm32x_info = NULL;

	if (CMD_ARGC != 1) {
		command_print(CMD_CTX, "stm32f2x options_read <bank>");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (ERROR_OK != retval)
		return retval;

	retval = stm32x_read_options(bank);
	if (ERROR_OK != retval)
		return retval;

	stm32x_info = bank->driver_priv;
	if (stm32x_info->has_extra_options) {
		if (stm32x_info->has_boot_addr) {
			uint32_t boot_addr = stm32x_info->option_bytes.boot_addr;

			command_print(CMD_CTX, "stm32f2x user_options 0x%03X,"
				" boot_add0 0x%04X, boot_add1 0x%04X",
				stm32x_info->option_bytes.user_options,
				boot_addr & 0xffff, (boot_addr & 0xffff0000) >> 16);
		} else {
			command_print(CMD_CTX, "stm32f2x user_options 0x%03X,",
				stm32x_info->option_bytes.user_options);
		}
	} else {
		command_print(CMD_CTX, "stm32f2x user_options 0x%02X",
			stm32x_info->option_bytes.user_options);

	}

	return retval;
}

COMMAND_HANDLER(stm32f2x_handle_options_write_command)
{
	int retval;
	struct flash_bank *bank;
	struct stm32x_flash_bank *stm32x_info = NULL;
	uint16_t user_options, boot_addr0, boot_addr1;

	if (CMD_ARGC < 1) {
		command_print(CMD_CTX, "stm32f2x options_write <bank> ...");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (ERROR_OK != retval)
		return retval;

	retval = stm32x_read_options(bank);
	if (ERROR_OK != retval)
		return retval;

	stm32x_info = bank->driver_priv;
	if (stm32x_info->has_boot_addr) {
		if (CMD_ARGC != 4) {
			command_print(CMD_CTX, "stm32f2x options_write <bank> <user_options>"
				" <boot_addr0> <boot_addr1>");
			return ERROR_COMMAND_SYNTAX_ERROR;
		}
		COMMAND_PARSE_NUMBER(u16, CMD_ARGV[2], boot_addr0);
		COMMAND_PARSE_NUMBER(u16, CMD_ARGV[3], boot_addr1);
		stm32x_info->option_bytes.boot_addr = boot_addr0 | (((uint32_t) boot_addr1) << 16);
	} else {
		if (CMD_ARGC != 2) {
			command_print(CMD_CTX, "stm32f2x options_write <bank> <user_options>");
			return ERROR_COMMAND_SYNTAX_ERROR;
		}
	}

	COMMAND_PARSE_NUMBER(u16, CMD_ARGV[1], user_options);
	if (user_options & (stm32x_info->has_extra_options ? ~0xffc : ~0xfc)) {
		command_print(CMD_CTX, "stm32f2x invalid user_options");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	stm32x_info->option_bytes.user_options = user_options;

	if (stm32x_write_options(bank) != ERROR_OK) {
		command_print(CMD_CTX, "stm32f2x failed to write options");
		return ERROR_OK;
	}

	/* switching between single- and dual-bank modes requires re-probe */
	/* ... and reprogramming of whole flash */
	stm32x_info->probed = 0;

	command_print(CMD_CTX, "stm32f2x write options complete.\n"
				"INFO: a reset or power cycle is required "
				"for the new settings to take effect.");
	return retval;
}

static const struct command_registration stm32x_exec_command_handlers[] = {
	{
		.name = "lock",
		.handler = stm32x_handle_lock_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id",
		.help = "Lock entire flash device.",
	},
	{
		.name = "unlock",
		.handler = stm32x_handle_unlock_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id",
		.help = "Unlock entire protected flash device.",
	},
	{
		.name = "mass_erase",
		.handler = stm32x_handle_mass_erase_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id",
		.help = "Erase entire flash device.",
	},
	{
		.name = "options_read",
		.handler = stm32f2x_handle_options_read_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id",
		.help = "Read and display device option bytes.",
	},
	{
		.name = "options_write",
		.handler = stm32f2x_handle_options_write_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id user_options [ boot_add0 boot_add1]",
		.help = "Write option bytes",
	},
	COMMAND_REGISTRATION_DONE
};
#endif

static const struct command_registration esp32_command_handlers[] = {
#if 0
	{
		.name = "stm32f2x",
		.mode = COMMAND_ANY,
		.help = "stm32f2x flash command group",
		.usage = "",
		.chain = stm32x_exec_command_handlers,
	},
#endif
	COMMAND_REGISTRATION_DONE
};

struct flash_driver esp32_flash = {
	.name = "esp32",
	.commands = esp32_command_handlers,
	.flash_bank_command = esp32_flash_bank_command,
	.erase = esp32_erase,
	.protect = esp32_protect,
	.write = esp32_write,
	.read = default_flash_read,
	.probe = esp32_probe,
	.auto_probe = esp32_auto_probe,
	.erase_check = default_flash_blank_check,
	.protect_check = esp32_protect_check,
	.info = get_esp32_info,
};
