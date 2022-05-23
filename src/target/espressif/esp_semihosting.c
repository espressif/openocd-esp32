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
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>. *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <helper/log.h>
#include <target/target.h>
#include <target/semihosting_common.h>
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

static int esp_semihosting_sys_drv_info(struct target *target, int addr, int size)
{
	struct semihosting *semihosting = target->semihosting;

	semihosting->result = -1;
	semihosting->sys_errno = EINVAL;

	uint8_t *buf = malloc(size);
	if (!buf) {
		LOG_ERROR("Memory alloc failed drv info!");
		return ERROR_FAIL;
	}
	int retval = target_read_buffer(target, addr, size, buf);
	if (retval == ERROR_OK) {
		struct esp_semihost_data *semihost_data = target_to_esp_semihost_data(target);
		semihosting->result = 0;
		semihosting->sys_errno = 0;
		semihost_data->version = le_to_h_u32(&buf[0]);
		LOG_DEBUG("semihost.version: %d", semihost_data->version);
	}
	free(buf);
	LOG_DEBUG("drv_info res=%d errno=%d", (int)semihosting->result, semihosting->sys_errno);
	return retval;
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
	uint8_t fields[4 * 8];

	LOG_DEBUG("op=0x%x, param=0x%" PRIx64, semihosting->op,
		semihosting->param);

	switch (semihosting->op) {
	case ESP_SEMIHOSTING_SYS_DRV_INFO:
		retval = semihosting_read_fields(target, 2, fields);
		if (retval == ERROR_OK) {
			int addr = semihosting_get_field(target, 0, fields);
			int size = semihosting_get_field(target, 1, fields);
			retval = esp_semihosting_sys_drv_info(target, addr, size);
		}
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
		/* For the time being only riscv chips support these commands */
		return esp_riscv_semihosting(target);
	}

	return retval;
}

int esp_semihosting_basedir_command(struct command_invocation *cmd)
{
	struct target *target = get_current_target(CMD_CTX);

	if (!target) {
		LOG_ERROR("No target selected");
		return ERROR_FAIL;
	}

	struct semihosting *semihosting = target->semihosting;
	if (!semihosting) {
		command_print(CMD, "semihosting not supported for current target");
		return ERROR_FAIL;
	}

	if (!semihosting->is_active) {
		if (semihosting->setup(target, true) != ERROR_OK) {
			LOG_ERROR("Failed to Configure semihosting");
			return ERROR_FAIL;
		}
		semihosting->is_active = true;
	}

	if (CMD_ARGC > 0) {
		free(semihosting->basedir);
		semihosting->basedir = strdup(CMD_ARGV[0]);
		if (!semihosting->basedir) {
			command_print(CMD, "semihosting failed to allocate memory for basedir!");
			return ERROR_FAIL;
		}
	}

	command_print(CMD, "DEPRECATED! semihosting base dir: %s",
		semihosting->basedir ? semihosting->basedir : "");

	return ERROR_OK;
}
