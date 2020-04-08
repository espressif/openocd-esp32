/***************************************************************************
 *   Copyright (c) 2020 Espressif Systems (Shanghai) Co. Ltd.                                *
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

#include "esp_xtensa_semihosting.h"

#define ESP_FD_MIN          2
#define ESP_O_RDONLY        0
#define ESP_O_WRONLY        1
#define ESP_O_RDWR          2
#define ESP_O_APPEND        0x0008
#define ESP_O_CREAT         0x0200
#define ESP_O_TRUNC         0x0400
#define ESP_O_EXCL          0x0800
#define ESP_O_SEMIHOST_ABSPATH  0x80000000

/* semihosting driver information */
char *esp_xtensa_semihosting_get_file_name(struct target *target,
	target_addr_t addr_fn,
	size_t len,
	uint32_t *mode)
{
	if (len > PATH_MAX ) {
		LOG_ERROR("Wrong length of file name!");
		return NULL;
	}
	int base_len = 0;
	int retval = 0;
	struct esp_xtensa_common *esp_xtensa = target_to_esp_xtensa(target);
	if (esp_xtensa->semihost.basedir && (*mode & ESP_O_SEMIHOST_ABSPATH) == 0)
		base_len = strlen(esp_xtensa->semihost.basedir);
	char *fn = malloc(base_len + len + 1);
	if (!fn) {
		LOG_ERROR("Failed to alloc memory for file name!");
		return NULL;
	}
	strncpy(fn, esp_xtensa->semihost.basedir, base_len);
	retval = target_read_buffer(target, addr_fn, len, (uint8_t *)fn + base_len);
	if (retval != ERROR_OK) {
		free(fn);
		LOG_ERROR("Failed to read name of file to open!");
		return NULL;
	}
	fn[base_len + len] = 0;
	*mode &= ~(XT_FILE_FLAGS_MASK);	/* clear xtensa flags after processing -
					 * at next steps semihosting_common will not use them*/
	return fn;
}

int esp_xtensa_semihosting_init(struct target *target)
{
	int retval = xtensa_semihosting_init(target);
	if (retval != ERROR_OK)
		return retval;
	target->semihosting->get_filename = esp_xtensa_semihosting_get_file_name;
	target->semihosting->lseek = lseek;
	target->semihosting->word_size_bytes = 4;			/* 32 bits */
	return retval;
}

static inline int semihosting_drv_info(struct target *target)
{
	/* setting semihosting*/
	int ret = 0;
	struct esp_xtensa_common *esp_xtensa = target_to_esp_xtensa(target);
	int addr = xtensa_reg_get(target, XT_REG_IDX_A3);
	int sz = xtensa_reg_get(target, XT_REG_IDX_A4);
	if (sz == -1) {
		LOG_ERROR("Wrong length of file name!");
		xtensa_reg_set(target, SYSCALL_RETVAL_REG, -1);
		xtensa_reg_set(target, SYSCALL_ERRNO_REG, EINVAL);
		return ERROR_FAIL;
	}
	uint8_t *buf = malloc(sz);
	ret = target_read_buffer(target, addr, sz, buf);
	if (ret != ERROR_OK) {
		free(buf);
		xtensa_reg_set(target, SYSCALL_RETVAL_REG, -1);
		xtensa_reg_set(target, SYSCALL_ERRNO_REG, EINVAL);
		return ERROR_FAIL;
	}

	/* read ver from drv_info_t mapped onto buf */
	esp_xtensa->semihost.version = le_to_h_u32(&buf[0]);

	xtensa_reg_set(target, SYSCALL_RETVAL_REG, 0);
	xtensa_reg_set(target, SYSCALL_ERRNO_REG, 0);
	return ERROR_OK;
}

static inline int esp_xtensa_semihosting_v0(
	struct target *target,
	struct esp_xtensa_common *esp_xtensa,
	xtensa_reg_val_t a2,
	xtensa_reg_val_t a3,
	xtensa_reg_val_t a4,
	xtensa_reg_val_t a5,
	xtensa_reg_val_t a6	)
{
	int syscall_ret = 0, syscall_errno = 0, retval;
	switch (a2) {
		case SEMIHOSTING_SYS_OPEN:
		{
			int mode = 0;

			if (a4 == 0) {
				LOG_ERROR("Zero file name length!");
				syscall_ret = -1;
				syscall_errno = EINVAL;
				break;
			}
			if (a4 > PATH_MAX) {
				LOG_ERROR("File name length if greater then the maximum possible value!");
				syscall_ret = -1;
				syscall_errno = EINVAL;
				break;
			}
			char *file_name = esp_xtensa_semihosting_get_file_name(target, a3, a4, (uint32_t * )&mode);
			if (a5 & ESP_O_RDWR)
				mode = O_RDWR;
			else if (a5 & ESP_O_WRONLY)
				mode = O_WRONLY;
			else
				mode = O_RDONLY;
			if (a5 & ESP_O_APPEND)
				mode |= O_APPEND;
			if (a5 & ESP_O_CREAT)
				mode |= O_CREAT;
			if (a5 & ESP_O_TRUNC)
				mode |= O_TRUNC;
			if (a5 & ESP_O_EXCL)
				mode |= O_EXCL;

#ifdef _WIN32
			/* Windows needs O_BINARY flag for proper handling of EOLs */
			mode |= O_BINARY;
#endif
			/* cygwin requires the permission setting
			 * otherwise it will fail to reopen a previously
			 * written file */
			syscall_ret = open(file_name, mode, 0644);
			syscall_errno = errno;
			LOG_DEBUG("Open file '%s' -> %d. Error %d.",
				file_name,
				syscall_ret,
				syscall_errno);
			free(file_name);
			break;
		}
		case SEMIHOSTING_SYS_CLOSE:
			if (a3 <= ESP_FD_MIN) {
				LOG_ERROR("Invalid file desc %d!", a3);
				syscall_ret = -1;
				syscall_errno = EINVAL;
				break;
			}
			syscall_ret = close(a3);
			syscall_errno = errno;
			LOG_DEBUG("Close file %d. Ret %d. Error %d.", a3, syscall_ret,
				syscall_errno);
			break;
		case SEMIHOSTING_SYS_WRITE: {
			LOG_DEBUG("Req write file %d. %d bytes.", a3, a5);
			if (a3 <= ESP_FD_MIN) {
				LOG_ERROR("Invalid file desc %d!", a3);
				syscall_ret = -1;
				syscall_errno = EINVAL;
				break;
			}
			if (a5 == 0) {
				syscall_ret = 0;
				syscall_errno = 0;
				break;
			}
			uint8_t *buf = malloc(a5);
			if (!buf) {
				syscall_ret = -1;
				syscall_errno = ENOMEM;
				break;
			}
			retval = target_read_buffer(target, a4, a5, buf);
			if (retval != ERROR_OK) {
				free(buf);
				syscall_ret = -1;
				syscall_errno = EINVAL;
				break;
			}
			syscall_ret = write(a3, buf, a5);
			syscall_errno = errno;
			LOG_DEBUG("Wrote file %d. %d bytes.", a3, a5);
			free(buf);
			break;
		}
		case SEMIHOSTING_SYS_READ: {
			LOG_DEBUG("Req read file %d. %d bytes.", a3, a5);
			if (a3 <= ESP_FD_MIN) {
				LOG_ERROR("Invalid file desc %d!", a3);
				syscall_ret = -1;
				syscall_errno = EINVAL;
				break;
			}
			if (a5 == 0) {
				syscall_ret = 0;
				syscall_errno = 0;
				break;
			}
			uint8_t *buf = malloc(a5);
			if (!buf) {
				syscall_ret = -1;
				syscall_errno = ENOMEM;
				break;
			}
			syscall_ret = read(a3, buf, a5);
			syscall_errno = errno;
			LOG_DEBUG("Read file %d. %d bytes.", a3, a5);
			if (syscall_ret >= 0) {
				retval = target_write_buffer(target, a4, syscall_ret, buf);
				if (retval != ERROR_OK) {
					free(buf);
					syscall_ret = -1;
					syscall_errno = EINVAL;
					break;
				}
			}
			free(buf);
			break;
		}
		case SEMIHOSTING_SYS_SEEK: {
			LOG_DEBUG("Req seek file %d. To %x, mode %d.", a3, a4, a5);
			if (a3 <= ESP_FD_MIN) {
				LOG_ERROR("Invalid file desc %d!", a3);
				syscall_ret = -1;
				syscall_errno = EINVAL;
				break;
			}
			syscall_ret = lseek(a3, a4, a5);
			syscall_errno = errno;
			LOG_DEBUG("Seek file %d. To %x, mode %d.", a3, a4, a5);
			break;
		}
		default:
			LOG_WARNING("Unsupported syscall %x!", a2);
			syscall_ret = -1;
			syscall_errno = ENOTSUP;
	}

	xtensa_reg_set(target, SYSCALL_RETVAL_REG, syscall_ret);
	xtensa_reg_set(target, SYSCALL_ERRNO_REG, syscall_errno);

	return ERROR_OK;
}

int esp_xtensa_semihosting(struct target *target)
{
	struct esp_xtensa_common *esp_xtensa = target_to_esp_xtensa(target);
	xtensa_reg_val_t a2 = xtensa_reg_get(target, XT_REG_IDX_A2);
	xtensa_reg_val_t a3 = xtensa_reg_get(target, XT_REG_IDX_A3);
	xtensa_reg_val_t a4 = xtensa_reg_get(target, XT_REG_IDX_A4);
	xtensa_reg_val_t a5 = xtensa_reg_get(target, XT_REG_IDX_A5);
	xtensa_reg_val_t a6 = xtensa_reg_get(target, XT_REG_IDX_A6);

	LOG_DEBUG("Call 0x%x 0x%x 0x%x 0x%x 0x%x %s",
		a2,
		a3,
		a4,
		a5,
		a6,
		esp_xtensa->semihost.basedir ? esp_xtensa->semihost.basedir : "");

	target->semihosting->op = xtensa_reg_get(target, XT_REG_IDX_A2);/* operation */

	/* Catching our custom SYSCALL*/
	if (target->semihosting->op == SEMIHOSTING_SYS_DRV_INFO) {
		semihosting_drv_info(target);
		return ERROR_OK;
	}
	if (esp_xtensa->semihost.version > 0)
		return semihosting_common(target);
	else
		return esp_xtensa_semihosting_v0(target, esp_xtensa, a2, a3, a4, a5, a6);
}
