/***************************************************************************
 *   Semihosting API for Espressif chips                                   *
 *   Copyright (C) 2022 Espressif Systems Ltd.                             *
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
#include <helper/log.h>
#include "target.h"
#include "semihosting_common.h"
#include "esp_semihosting.h"
#include "esp_riscv.h"
#include "esp_xtensa.h"

struct esp_semihost_data *target_to_esp_semihost_data(struct target *target)
{
	const char *arch = target_get_gdb_arch(target);
	if (arch) {
		if (strncmp(arch, "riscv", 5) == 0)
			return &target_to_esp_riscv(target)->semihost;
		else if (strncmp(arch, "xtensa", 6) == 0)
			return &target_to_esp_xtensa(target)->semihost;
	}
	LOG_ERROR("Unknown target arch!");
	return NULL;
}

int esp_semihosting_sys_seek(struct target *target, uint64_t fd, uint32_t pos, size_t whence)
{
	struct semihosting *semihosting = target->semihosting;

	semihosting->result = lseek(fd, pos, whence);
	semihosting->sys_errno = errno;
	LOG_DEBUG("lseek(%" PRIx64 ", %d %d)=%d", fd, (int)pos,
		(int)semihosting->result, errno);
	return ERROR_OK;
}

int esp_semihosting_common(struct target *target)
{
	struct semihosting *semihosting = target->semihosting;
	if (!semihosting) {
		/* Silently ignore if the semihosting field was not set. */
		return ERROR_OK;
	}

	int retval = ERROR_FAIL;
	/* Enough space to hold 4 long words. */
	uint8_t fields[4*8];

	LOG_DEBUG("op=0x%x, param=0x%" PRIx64, semihosting->op,
		semihosting->param);

	switch (semihosting->op) {
		case ESP_SYS_DRV_INFO_LEGACY:
		case ESP_SEMIHOSTING_SYS_DRV_INFO:
			/* TODO */
			break;
		case ESP_SEMIHOSTING_SYS_SEEK:
			retval = semihosting_read_fields(target, 3, fields);
			if (retval == ERROR_OK) {
				uint64_t fd = semihosting_get_field(target, 0, fields);
				uint32_t pos = semihosting_get_field(target, 1, fields);
				size_t whence = semihosting_get_field(target, 2, fields);
				retval = esp_semihosting_sys_seek(target, fd, pos, whence);
			}
			break;
		case ESP_SEMIHOSTING_SYS_APPTRACE_INIT:
		case ESP_SEMIHOSTING_SYS_DEBUG_STUBS_INIT:
		case ESP_SEMIHOSTING_SYS_BREAKPOINT_SET:
		case ESP_SEMIHOSTING_SYS_WATCHPOINT_SET:
		/* TODO remove old codes when the new ones backported to IDF 4.4 */
		case ESP_RISCV_APPTRACE_SYSNR:
			/* For the time being only riscv chips support these commands */
			return esp_riscv_semihosting(target);
	}

	return retval;
}
