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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "esp_xtensa_semihosting.h"

#define ESP_XTENSA_SYSCALL_LEGACY       XT_INS_BREAK(1, 1)
#define ESP_XTENSA_SYSCALL              XT_INS_BREAK(1, 14)
#define ESP_XTENSA_SYSCALL_SZ           3

#define ESP_FD_MIN                      2

#define ESP_O_RDONLY                    0
#define ESP_O_WRONLY                    1
#define ESP_O_RDWR                      2
#define ESP_O_APPEND                    0x0008
#define ESP_O_CREAT                     0x0200
#define ESP_O_TRUNC                     0x0400
#define ESP_O_EXCL                      0x0800
#define ESP_O_SEMIHOST_ABSPATH          0x80000000

#define ESP_FILE_FLAGS_MASK             0xFFF00000

#define SYSCALL_PARAM2_REG              XT_REG_IDX_A3

#define XTENSA_SYSCALL_OP_REG           XT_REG_IDX_A2
#define XTENSA_SYSCALL_RETVAL_REG       XT_REG_IDX_A2
#define XTENSA_SYSCALL_ERRNO_REG        XT_REG_IDX_A3

static int esp_xtensa_semihosting_setup(struct target *target, int enable)
{
	LOG_DEBUG("[%s] semihosting enable=%d", target_name(target), enable);

	target->semihosting->param = XT_REG_IDX_A3;	/* used to specify where
							 * to read fields. xtensa
							 * uses registers in contrast
							 * with general memory-based approach*/
	return ERROR_OK;
}

static int esp_xtensa_semihosting_post_result(struct target *target)
{
	/* Even with the v2 and later, errno will not retrieved from A3 reg, it is safe to set */
	xtensa_reg_set(target, XTENSA_SYSCALL_RETVAL_REG, target->semihosting->result);
	xtensa_reg_set(target, XTENSA_SYSCALL_ERRNO_REG, target->semihosting->sys_errno);
	return ERROR_OK;
}

static char *esp_xtensa_semihosting_get_file_name(struct target *target,
	target_addr_t addr_fn,
	size_t len,
	uint32_t *mode)
{
	if (len > PATH_MAX) {
		LOG_ERROR("Wrong length of file name!");
		return NULL;
	}
	int base_len = 0;
	int retval = 0;
	if (target->semihosting->basedir && (*mode & ESP_O_SEMIHOST_ABSPATH) == 0)
		base_len = strlen(target->semihosting->basedir);
	char *fn = malloc(base_len + len + 1);
	if (!fn) {
		LOG_ERROR("Failed to alloc memory for file name!");
		return NULL;
	}
	strncpy(fn, target->semihosting->basedir, base_len);
	retval = target_read_buffer(target, addr_fn, len, (uint8_t *)fn + base_len);
	if (retval != ERROR_OK) {
		free(fn);
		LOG_ERROR("Failed to read name of file to open!");
		return NULL;
	}
	fn[base_len + len] = 0;
	*mode &= ~(ESP_FILE_FLAGS_MASK);	/* clear xtensa flags after processing -
						* at next steps semihosting_common will not use them*/
	return fn;
}

static inline int esp_xtensa_semihosting_drv_info_v01(struct target *target)
{
	int syscall_ret = -1, syscall_errno = EINVAL;
	int addr = xtensa_reg_get(target, XT_REG_IDX_A3);
	int sz = xtensa_reg_get(target, XT_REG_IDX_A4);
	if (sz < 0) {
		LOG_ERROR("Wrong length of drv info!");
		goto _exit;
	}
	uint8_t *buf = malloc(sz);
	if (!buf) {
		LOG_ERROR("Memory alloc failed drv info!");
		goto _exit;
	}
	int retval = target_read_buffer(target, addr, sz, buf);
	if (retval != ERROR_OK) {
		free(buf);
		LOG_ERROR("Read memory failed drv info!");
		goto _exit;
	}
	/* read ver from drv_info_t mapped onto buf */
	struct esp_xtensa_common *esp_xtensa = target_to_esp_xtensa(target);
	esp_xtensa->semihost.version = le_to_h_u32(&buf[0]);
	syscall_ret = syscall_errno = 0;
	LOG_DEBUG("semihost.version: %d", esp_xtensa->semihost.version);
	free(buf);
_exit:
	xtensa_reg_set(target, XTENSA_SYSCALL_RETVAL_REG, syscall_ret);
	xtensa_reg_set(target, XTENSA_SYSCALL_ERRNO_REG, syscall_errno);
	return ERROR_OK;
}

static inline int esp_xtensa_semihosting_v0(
	struct target *target,
	xtensa_reg_val_t a2,
	xtensa_reg_val_t a3,
	xtensa_reg_val_t a4,
	xtensa_reg_val_t a5,
	xtensa_reg_val_t a6)
{
	int syscall_ret = 0, syscall_errno = 0, retval;
	switch (a2) {
	case SEMIHOSTING_SYS_OPEN:
	{
		int mode = 0;

		if (a4 == 0) {
			LOG_ERROR("Zero file name length!");
			syscall_ret = -1;
			syscall_errno = ENOMEM;
			break;
		}
		if (a4 > PATH_MAX) {
			LOG_ERROR(
					"File name length if greater then the maximum possible value!");
			syscall_ret = -1;
			syscall_errno = ENOMEM;
			break;
		}
		char *file_name =
				esp_xtensa_semihosting_get_file_name(target,
				a3,
				a4,
				(uint32_t * )&mode);
		if (!file_name) {
			syscall_ret = -1;
			syscall_errno = ENOMEM;
			break;
		}

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
		LOG_DEBUG("Req write file %d. %" PRIu32 " bytes.", a3, a5);
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
			syscall_errno = EIO;
			break;
		}
		syscall_ret = write(a3, buf, a5);
		syscall_errno = errno;
		LOG_DEBUG("Wrote file %d. %d bytes.", a3, a5);
		free(buf);
		break;
	}
	case SEMIHOSTING_SYS_READ: {
		LOG_DEBUG("Req read file %d. %" PRIu32 " bytes.", a3, a5);
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
		LOG_DEBUG("Read file %d. %" PRIu32 " bytes.", a3, a5);
		if (syscall_ret >= 0) {
			retval = target_write_buffer(target, a4, syscall_ret, buf);
			if (retval != ERROR_OK) {
				free(buf);
				syscall_ret = -1;
				syscall_errno = EIO;
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

	xtensa_reg_set(target, XTENSA_SYSCALL_RETVAL_REG, syscall_ret);
	xtensa_reg_set(target, XTENSA_SYSCALL_ERRNO_REG, syscall_errno);

	return ERROR_OK;
}

static const int open_modeflags[12] = {
	O_RDONLY,
	O_RDONLY | O_BINARY,
	O_RDWR,
	O_RDWR | O_BINARY,
	O_WRONLY | O_CREAT | O_TRUNC,
	O_WRONLY | O_CREAT | O_TRUNC | O_BINARY,
	O_RDWR | O_CREAT | O_TRUNC,
	O_RDWR | O_CREAT | O_TRUNC | O_BINARY,
	O_WRONLY | O_CREAT | O_APPEND,
	O_WRONLY | O_CREAT | O_APPEND | O_BINARY,
	O_RDWR | O_CREAT | O_APPEND,
	O_RDWR | O_CREAT | O_APPEND | O_BINARY
};

static inline int esp_xtensa_semihosting_v1(
	struct target *target,
	xtensa_reg_val_t a2,
	xtensa_reg_val_t a3,
	xtensa_reg_val_t a4,
	xtensa_reg_val_t a5,
	xtensa_reg_val_t a6)
{
	int syscall_ret = 0, syscall_errno = 0, retval;
	switch (a2) {
	case SEMIHOSTING_SYS_OPEN:
	{
		uint64_t addr = a3;
		uint32_t mode = a4;
		size_t len = a5;

		if (mode > 11) {
			syscall_ret = -1;
			syscall_errno = EINVAL;
			break;
		}

		char *file_name =
				esp_xtensa_semihosting_get_file_name(target,
				addr,
				len,
				(uint32_t * )&mode);
		if (!file_name) {
			syscall_ret = -1;
			syscall_errno = ENOMEM;
			break;
		}
		len = strlen(file_name);	/* updated len_size based on gotten file name */
		uint32_t flags = open_modeflags[mode];

#ifdef _WIN32
		/* Windows needs O_BINARY flag for proper handling of EOLs */
		flags |= O_BINARY;
#endif
		/* cygwin requires the permission setting
		 * otherwise it will fail to reopen a previously
		 * written file */
		syscall_ret = open(file_name, flags, 0644);
		syscall_errno = errno;
		LOG_DEBUG("Open file '%s' -> %d. Error %d.",
				file_name,
				syscall_ret,
				syscall_errno);
		free(file_name);
		break;
	}
	case SEMIHOSTING_SYS_CLOSE:
	{
		int fd = a3;
		if (fd == 0 || fd == 1 || fd == 2) {
			LOG_DEBUG("ignoring semihosting attempt to close %s",
					(fd == 0) ? "stdin" :
					(fd == 1) ? "stdout" : "stderr");
			syscall_ret = 0;
			syscall_errno = 0;
			break;
		}
		syscall_ret = close(fd);
		syscall_errno = errno;
		LOG_DEBUG("close(%d)=%d errno=%d", fd, syscall_ret, syscall_errno);
		break;
	}
	case SEMIHOSTING_SYS_WRITE:
	{
		int fd = a3;
		uint64_t addr = a4;
		size_t len = a5;
		uint8_t *buf = malloc(len);
		if (!buf) {
			syscall_ret = -1;
			syscall_errno = ENOMEM;
			break;
		}
		retval = target_read_buffer(target, addr, len, buf);
		if (retval != ERROR_OK) {
			free(buf);
			syscall_ret = -1;
			syscall_errno = EIO;
			break;
		}
		syscall_ret = write(fd, buf, len);
		syscall_errno = errno;
		LOG_DEBUG("write(%d, 0x%" PRIx64 ", %zu)=%d",
				fd,
				addr,
				len,
				syscall_ret);
		if (syscall_ret >= 0) {
			/* The number of bytes that are NOT written */
			syscall_ret = len - syscall_ret;
		}
		free(buf);
		break;
	}
	case SEMIHOSTING_SYS_READ:
	{
		int fd = a3;
		uint64_t addr = a4;
		size_t len = a5;
		uint8_t *buf = malloc(len);
		if (!buf) {
			syscall_ret = -1;
			syscall_errno = ENOMEM;
			break;
		}
		syscall_ret = read(a3, buf, a5);
		syscall_errno = errno;
		LOG_DEBUG("read(%d, 0x%" PRIx64 ", %zu)=%d",
				fd,
				addr,
				len,
				syscall_ret);
		if (syscall_ret >= 0) {
			retval = target_write_buffer(target, addr, syscall_ret, buf);
			if (retval != ERROR_OK) {
				free(buf);
				syscall_ret = -1;
				syscall_errno = EIO;
				break;
			}
			/* the number of bytes NOT filled in */
			syscall_ret = len - syscall_ret;
		}
		free(buf);
		break;
	}
	case ESP_SEMIHOSTING_SYS_SEEK:
	case SEMIHOSTING_SYS_SEEK:
	{
		int fd = a3;
		off_t pos = a4;
		int whence = a5;
		syscall_ret = lseek(fd, pos, whence);
		syscall_errno = errno;
		LOG_DEBUG("lseek(%d, %d)=%d", fd, (int)pos, syscall_ret);
		break;
	}
	default:
		LOG_WARNING("Unsupported syscall %x!", a2);
		syscall_ret = -1;
		syscall_errno = ENOTSUP;
	}

	xtensa_reg_set(target, XTENSA_SYSCALL_RETVAL_REG, syscall_ret);
	xtensa_reg_set(target, XTENSA_SYSCALL_ERRNO_REG, syscall_errno);

	return ERROR_OK;
}

/**
 * Checks for and processes an ESP Xtensa semihosting request.  This is meant
 * to be called when the target is stopped due to a debug mode entry.
 * If the value 0 is returned then there was nothing to process. A non-zero
 * return value signifies that a request was processed and the target resumed,
 * or an error was encountered, in which case the caller must return
 * immediately.
 *
 * @param target Pointer to the ESP Xtensa target to process.
 * @param retval Pointer to a location where the return code will be stored
 * @return non-zero value if a request was processed or an error encountered
 */
int esp_xtensa_semihosting(struct target *target, int *retval)
{
	struct esp_xtensa_common *esp_xtensa = target_to_esp_xtensa(target);

	xtensa_reg_val_t dbg_cause = xtensa_reg_get(target, XT_REG_IDX_DEBUGCAUSE);
	if ((dbg_cause & (DEBUGCAUSE_BI | DEBUGCAUSE_BN)) == 0)
		return 0;

	uint8_t brk_insn_buf[sizeof(uint32_t)] = { 0 };
	xtensa_reg_val_t pc = xtensa_reg_get(target, XT_REG_IDX_PC);
	*retval = target_read_memory(target,
		pc,
		ESP_XTENSA_SYSCALL_SZ,
		1,
		(uint8_t *)brk_insn_buf);
	if (*retval != ERROR_OK) {
		LOG_ERROR("Failed to read break instruction!");
		return 0;
	}

	uint32_t syscall_ins = buf_get_u32(brk_insn_buf, 0, 32);
	if ((syscall_ins != ESP_XTENSA_SYSCALL) && (syscall_ins != ESP_XTENSA_SYSCALL_LEGACY))
		return 0;

	if (esp_xtensa->semihost.ops && esp_xtensa->semihost.ops->prepare)
		esp_xtensa->semihost.ops->prepare(target);

	xtensa_reg_val_t a2 = xtensa_reg_get(target, XT_REG_IDX_A2);
	xtensa_reg_val_t a3 = xtensa_reg_get(target, XT_REG_IDX_A3);
	LOG_DEBUG("%s: Semihosting call 0x%x 0x%x Base dir '%s'",
		target_name(target),
		a2,
		a3,
		target->semihosting->basedir ? target->semihosting->basedir : "");

	target->semihosting->op = a2;
	target->semihosting->param = a3;

	if (target->semihosting->op == ESP_SEMIHOSTING_SYS_DRV_INFO ||
		esp_xtensa->semihost.version > 1) {
		*retval = semihosting_common(target);
	} else if (target->semihosting->op == ESP_SYS_DRV_INFO_LEGACY) {
		target->semihosting->is_resumable = true;
		*retval = esp_xtensa_semihosting_drv_info_v01(target);
		/*
		        It is safe to update opcode with the new one.
		        So that we don't need to check different opcodes in the other modules
		*/
		target->semihosting->op = ESP_SEMIHOSTING_SYS_DRV_INFO;
	} else {
		target->semihosting->is_resumable = true;
		xtensa_reg_val_t a4 = xtensa_reg_get(target, XT_REG_IDX_A4);
		xtensa_reg_val_t a5 = xtensa_reg_get(target, XT_REG_IDX_A5);
		xtensa_reg_val_t a6 = xtensa_reg_get(target, XT_REG_IDX_A6);

		LOG_DEBUG("%s: Semihosting. Call 0x%x 0x%x 0x%x 0x%x 0x%x. Base dir '%s'",
			target_name(target),
			a2,
			a3,
			a4,
			a5,
			a6,
			target->semihosting->basedir ? target->semihosting->basedir : "");

		if (esp_xtensa->semihost.version == 1)
			*retval = esp_xtensa_semihosting_v1(target, a2, a3, a4, a5, a6);
		else
			*retval = esp_xtensa_semihosting_v0(target, a2, a3, a4, a5, a6);

		LOG_DEBUG("%s: Semihosting. retval: %d, target_ret: %d, target_errno: %d",
			target_name(target),
			*retval,
			xtensa_reg_get(target, XTENSA_SYSCALL_RETVAL_REG),
			xtensa_reg_get(target, XTENSA_SYSCALL_ERRNO_REG)
			);
	}
	/* Most operations are resumable, except the two exit calls. */
	if (*retval != ERROR_OK) {
		LOG_ERROR("Semihosting operation (op: 0x%x) error! Code: %d",
			target->semihosting->op,
			*retval);
	}

	/* Resume if target it is resumable and we are not waiting on a fileio
	 * operation to complete:
	 */
	if (target->semihosting->is_resumable && !target->semihosting->hit_fileio)
		target_to_esp_xtensa(target)->semihost.need_resume = true;
	return 1;
}

static int xtensa_semihosting_init(struct target *target)
{
	int retval = semihosting_common_init(target,
		esp_xtensa_semihosting_setup,
		esp_xtensa_semihosting_post_result);
	return retval;
}

int esp_xtensa_semihosting_init(struct target *target)
{
	int retval = xtensa_semihosting_init(target);
	if (retval != ERROR_OK)
		return retval;
	target->semihosting->word_size_bytes = 4;			/* 32 bits */
	target->semihosting->user_command_handler = esp_semihosting_common;
	return retval;
}
