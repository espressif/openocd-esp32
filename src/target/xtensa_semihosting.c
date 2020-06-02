/***************************************************************************
 *   Generic Xtensa Semihosting API                                        *
 *   Copyright (C) 2020 Espressif Systems Ltd.                             *
 *   Author: Alexey Gerenkov <alexey@espressif.com>                        *
 *   Author: Andrei Gramakov <andrei.gramakov@espressif.com>               *
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

#include "xtensa_semihosting.h"

static int xtensa_semihosting_read_fields(struct target *target, size_t number, uint8_t *fields)
{
	uint32_t reg_id = target->semihosting->param;	/* containts regnumber based on enum
							 * xtensa_reg_id */
	uint32_t r;
	for (size_t i = 0; i < number; i++) {
		r = xtensa_reg_get(target, reg_id + i);
		h_u32_to_le(&fields[i*4],r);
	}
	return ERROR_OK;
}

static int xtensa_semihosting_write_fields(struct target *target, size_t number, uint8_t *fields)
{
	uint32_t reg_id = target->semihosting->param;	/* containts regnumber based on enum
							 * xtensa_reg_id */
	uint32_t r;
	for (size_t i = 0; i < number; i++) {
		r = le_to_h_u32(&fields[i*4]);
		xtensa_reg_set(target, reg_id + i, r);
	}
	return ERROR_OK;
}

static int xtensa_semihosting_setup(struct target *target)
{
	target->semihosting->param = XT_REG_IDX_A3;	/* used to specify where
							 * to read fields. xtensa
							 * uses registers in contrast
							 * with general memory-based approach*/
	target->semihosting->read_fields = xtensa_semihosting_read_fields;
	target->semihosting->write_fields = xtensa_semihosting_write_fields;
	return ERROR_OK;
}

static int xtensa_semihosting_post_result(struct target *target)
{
	xtensa_reg_set(target, XTENSA_SYSCALL_RETVAL_REG, target->semihosting->result);
	xtensa_reg_set(target, XTENSA_SYSCALL_ERRNO_REG, target->semihosting->sys_errno);
	return ERROR_OK;
}

/**
 * Initialize esp_xtensa semihosting support.
 *
 * @param target Pointer to the ESP_XTENSA target to initialize.
 * @return An error status if there is a problem during initialization.
 */
int xtensa_semihosting_init(struct target *target)
{
	int retval = semihosting_common_init(target,
			xtensa_semihosting_setup,
			xtensa_semihosting_post_result);
	if (retval != ERROR_OK)
		return retval;
	return xtensa_semihosting_setup(target);
}
