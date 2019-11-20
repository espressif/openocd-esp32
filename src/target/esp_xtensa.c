/***************************************************************************
 *   Espressif Xtensa target API for OpenOCD                               *
 *   Copyright (C) 2019 Espressif Systems Ltd.                             *
 *   Author: Alexey Gerenkov <alexey@espressif.com>                        *
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

#include <stdbool.h>
#include <stdint.h>
#include "register.h"
#include "esp_xtensa.h"
#include "xtensa_mcore.h"
#include "esp_xtensa_apptrace.h"

#define ESP_XTENSA_SYSCALL     XT_INS_BREAK(1,1)
#define ESP_XTENSA_SYSCALL_SZ  3

#define ESP_SYS_OPEN        0x01
#define ESP_SYS_CLOSE       0x02
#define ESP_SYS_WRITE       0x05
#define ESP_SYS_READ        0x06
#define ESP_SYS_SEEK        0x0A

#define ESP_FD_MIN          2

#define ESP_O_RDONLY        0
#define ESP_O_WRONLY        1
#define ESP_O_RDWR          2
#define ESP_O_APPEND        0x0008
#define ESP_O_CREAT         0x0200
#define ESP_O_TRUNC         0x0400
#define ESP_O_EXCL          0x0800
#define ESP_O_SEMIHOST_ABSPATH  0x80000000

#define SYSCALL_PARAM2_REG  XT_REG_IDX_A3
#define SYSCALL_RETVAL_REG  XT_REG_IDX_A2
#define SYSCALL_ERRNO_REG   SYSCALL_PARAM2_REG

#define ESP_XTENSA_DBGSTUBS_UPDATE_DATA_ENTRY(_e_) \
	do { \
		(_e_) = buf_get_u32((uint8_t *)&(_e_), 0, 32); \
		if (!xtensa_data_addr_valid(target, (_e_))) { \
			LOG_ERROR("No valid stub data entry found (0x%x)!", (uint32_t)(_e_)); \
			return;	\
		} \
	} while (0)

#define ESP_XTENSA_DBGSTUBS_UPDATE_CODE_ENTRY(_e_) \
	do { \
		(_e_) = buf_get_u32((uint8_t *)&(_e_), 0, 32); \
		if ((_e_) == 0) { \
			LOG_ERROR("No valid stub code entry found (0x%x)!", (uint32_t)(_e_)); \
			return;	\
		} \
	} while (0)

static void esp_xtensa_dbgstubs_info_update(struct target *target);
static void esp_xtensa_dbgstubs_addr_check(struct target *target);
static int esp_xtensa_do_semihosting(struct target *target);


static int esp_xtensa_dbgstubs_restore(struct target *target)
{
	struct esp_xtensa_common *esp_xtensa = target_to_esp_xtensa(target);

	if (esp_xtensa->dbg_stubs.base == 0)
		return ERROR_OK;

	LOG_INFO("%s: Restore debug stubs address %x",
		target_name(target),
		esp_xtensa->dbg_stubs.base);
	int res = esp_xtensa_apptrace_status_reg_write(target, esp_xtensa->dbg_stubs.base);
	if (res != ERROR_OK) {
		LOG_ERROR("Failed to write trace status (%d)!", res);
		return res;
	}
	return ERROR_OK;
}

static int esp_xtensa_on_exit(struct target *target, void *priv)
{
	if (target != priv)
		return ERROR_OK;

	LOG_DEBUG("start");
	int ret = esp_xtensa_dbgstubs_restore(target);
	if (ret != ERROR_OK)
		return ret;
	return ERROR_OK;
}

static int esp_xtensa_handle_target_event(struct target *target, enum target_event event,
	void *priv)
{
	if (target != priv)
		return ERROR_OK;

	LOG_DEBUG("%d", event);
	switch (event) {
		case TARGET_EVENT_HALTED:
			/* debug stubs can be used in HALTED state only, so it is OK to get info
			 * about them here */
			esp_xtensa_dbgstubs_info_update(target);
			break;
		case TARGET_EVENT_GDB_DETACH:
		{
			enum target_state old_state = target->state;
			if (target->state != TARGET_HALTED) {
				int ret = target_halt(target);
				if (ret != ERROR_OK) {
					LOG_ERROR(
						"%s: Failed to halt target to remove flash BPs (%d)!",
						target_name(target),
						ret);
					return ret;
				}
				ret = target_wait_state(target, TARGET_HALTED, 3000);
				if (ret != ERROR_OK) {
					LOG_ERROR(
						"%s: Failed to wait halted target to remove flash BPs (%d)!",
						target_name(target),
						ret);
					return ret;
				}
			}
			struct esp_xtensa_common *esp_xtensa = target_to_esp_xtensa(target);
			for (size_t slot = 0; slot < ESP_XTENSA_SPECIAL_BREAKPOINTS_MAX_NUM;
				slot++) {
				struct esp_xtensa_special_breakpoint *spec_bp =
					&esp_xtensa->spec_brps[slot];
				if (spec_bp->data.oocd_bp != NULL) {
					int ret = esp_xtensa->spec_brps_ops.breakpoint_remove(
						target,
						spec_bp);
					if (ret != ERROR_OK) {
						LOG_ERROR(
							"%s: Failed to remove SW flash BP @ "
							TARGET_ADDR_FMT " (%d)!",
							target_name(target),
							spec_bp->data.oocd_bp->address,
							ret);
						return ret;
					}
				}
			}
			memset(esp_xtensa->spec_brps,
				0,
				ESP_XTENSA_SPECIAL_BREAKPOINTS_MAX_NUM*
				sizeof(struct esp_xtensa_special_breakpoint));
			if (old_state == TARGET_RUNNING) {
				int ret = target_resume(target, 1, 0, 1, 0);
				if (ret != ERROR_OK) {
					LOG_ERROR(
						"%s: Failed to resume target after flash BPs removal (%d)!",
						target_name(target),
						ret);
					return ret;
				}
			}
			break;
		}
		default:
			break;
	}
	return ERROR_OK;
}

int esp_xtensa_init_arch_info(struct target *target, struct target *chip_target,
	void *arch_info,
	const struct xtensa_config *xtensa_cfg,
	struct xtensa_debug_module_config *dm_cfg,
	const struct xtensa_chip_ops *chip_ops,
	const struct esp_xtensa_special_breakpoint_ops *spec_brps_ops)
{
	struct esp_xtensa_common *esp_xtensa;

	int ret = target_register_exit_callback(esp_xtensa_on_exit, NULL);
	if (ret != ERROR_OK)
		return ret;
	ret = target_register_event_callback(esp_xtensa_handle_target_event, target);
	if (ret != ERROR_OK)
		return ret;
	if (target != chip_target) {
		/* if this target is a sub-core of the chip, allocate arch data */
		esp_xtensa = calloc(1, sizeof(struct esp_xtensa_common));
		if (esp_xtensa == NULL)
			return ERROR_FAIL;
	} else {
		esp_xtensa = arch_info;
		memset(esp_xtensa, 0, sizeof(*esp_xtensa));
	}
	if (dm_cfg->queue_tdi_idle == NULL) {
		dm_cfg->queue_tdi_idle = esp_xtensa_queue_tdi_idle;
		dm_cfg->queue_tdi_idle_arg = target;
	}
	ret = xtensa_init_arch_info(target, &esp_xtensa->xtensa, xtensa_cfg, dm_cfg, chip_ops);
	if (ret != ERROR_OK) {
		if (target != chip_target)
			free(esp_xtensa);
		return ret;
	}
	memcpy(&esp_xtensa->spec_brps_ops, spec_brps_ops, sizeof(esp_xtensa->spec_brps_ops));
	esp_xtensa->spec_brps =
		calloc(ESP_XTENSA_SPECIAL_BREAKPOINTS_MAX_NUM,
		sizeof(struct esp_xtensa_special_breakpoint));
	if (esp_xtensa->spec_brps == NULL) {
		if (target != chip_target)
			free(esp_xtensa);
		return ERROR_FAIL;
	}
	esp_xtensa->chip_target = chip_target;
	esp_xtensa->flash_bootstrap = FBS_DONTCARE;
	return ERROR_OK;
}


int esp_xtensa_target_init(struct command_context *cmd_ctx, struct target *target)
{
	xtensa_build_reg_cache(target);
	return ERROR_OK;
}

int esp_xtensa_arch_state(struct target *target)
{
	return ERROR_OK;
}

void esp_xtensa_on_reset(struct target *target)
{
	struct esp_xtensa_common *esp_xtensa = target_to_esp_xtensa(target);

	LOG_DEBUG("start");
	memset(&esp_xtensa->dbg_stubs, 0, sizeof(esp_xtensa->dbg_stubs));
}

void esp_xtensa_on_poll(struct target *target)
{
	xtensa_on_poll(target);
	if (target->state != TARGET_DEBUG_RUNNING)
		esp_xtensa_dbgstubs_addr_check(target);
}

bool esp_xtensa_on_halt(struct target *target)
{
	int res;

	xtensa_reg_val_t dbg_cause = xtensa_reg_get(target, XT_REG_IDX_DEBUGCAUSE);
	if (dbg_cause & (DEBUGCAUSE_BI|DEBUGCAUSE_BN)) {
		uint8_t brk_insn_buf[sizeof(uint32_t)] = {0};
		xtensa_reg_val_t pc = xtensa_reg_get(target, XT_REG_IDX_PC);
		res = target_read_memory(target,
			pc,
			ESP_XTENSA_SYSCALL_SZ,
			1,
			(uint8_t *)brk_insn_buf);
		if (res != ERROR_OK)
			LOG_ERROR("Failed to read break instruction!");
		else if (buf_get_u32(brk_insn_buf, 0, 32) == ESP_XTENSA_SYSCALL) {
			if (esp_xtensa_do_semihosting(target) == ERROR_OK)
				return true;
		}
	}
	return false;
}

static void esp_xtensa_dbgstubs_addr_check(struct target *target)
{
	struct esp_xtensa_common *esp_xtensa = target_to_esp_xtensa(target);
	uint32_t vec_addr = 0;

	if (esp_xtensa->dbg_stubs.base != 0)
		return;

	int res = esp_xtensa_apptrace_status_reg_read(target, &vec_addr);
	if (res != ERROR_OK) {
		LOG_ERROR("Failed to read debug stubs address location (%d)!", res);
		return;
	}
	if (xtensa_data_addr_valid(target, vec_addr)) {
		LOG_INFO("%s: Detected debug stubs @ %x", target_name(target), vec_addr);
		res = esp_xtensa_apptrace_status_reg_write(target, 0);
		if (res != ERROR_OK)
			LOG_ERROR("Failed to clear debug stubs address location (%d)!", res);
		esp_xtensa->dbg_stubs.base = vec_addr;
	}
}

static void esp_xtensa_dbgstubs_info_update(struct target *target)
{
	struct esp_xtensa_common *esp_xtensa = target_to_esp_xtensa(target);

	if (esp_xtensa->dbg_stubs.base == 0 || esp_xtensa->dbg_stubs.entries_count != 0)
		return;

	int res = target_read_memory(target, esp_xtensa->dbg_stubs.base, sizeof(uint32_t),
		ESP_DBG_STUB_ENTRY_MAX-ESP_DBG_STUB_TABLE_START,
		(uint8_t *)&esp_xtensa->dbg_stubs.entries);
	if (res != ERROR_OK) {
		LOG_ERROR("%s: Failed to read debug stubs info!", target_name(target));
		return;
	}
	for (enum esp_dbg_stub_id i = ESP_DBG_STUB_TABLE_START; i < ESP_DBG_STUB_ENTRY_MAX; i++) {
		LOG_DEBUG("Check dbg stub %d", i);
		if (esp_xtensa->dbg_stubs.entries[i]) {
			esp_xtensa->dbg_stubs.entries[i] = buf_get_u32(
				(uint8_t *)&esp_xtensa->dbg_stubs.entries[i],
				0,
				32);
			LOG_DEBUG("New dbg stub %d at %x",
				esp_xtensa->dbg_stubs.entries_count,
				esp_xtensa->dbg_stubs.entries[i]);
			esp_xtensa->dbg_stubs.entries_count++;
		}
	}
	if (esp_xtensa->dbg_stubs.entries_count <
		(ESP_DBG_STUB_ENTRY_MAX-ESP_DBG_STUB_TABLE_START)) {
		LOG_DEBUG("Not full dbg stub table %d of %d", esp_xtensa->dbg_stubs.entries_count,
			(ESP_DBG_STUB_ENTRY_MAX-ESP_DBG_STUB_TABLE_START));
		esp_xtensa->dbg_stubs.entries_count = 0;
		return;
	}
	/* read debug stubs descriptor */
	ESP_XTENSA_DBGSTUBS_UPDATE_DATA_ENTRY(esp_xtensa->dbg_stubs.entries[ESP_DBG_STUB_DESC]);
	res =
		target_read_memory(target, esp_xtensa->dbg_stubs.entries[ESP_DBG_STUB_DESC],
		sizeof(struct esp_dbg_stubs_desc), 1,
		(uint8_t *)&esp_xtensa->dbg_stubs.desc);
	if (res != ERROR_OK) {
		LOG_ERROR("Failed to read target memory (%d)!", res);
		return;
	}
	ESP_XTENSA_DBGSTUBS_UPDATE_CODE_ENTRY(esp_xtensa->dbg_stubs.desc.tramp_addr);
	ESP_XTENSA_DBGSTUBS_UPDATE_DATA_ENTRY(esp_xtensa->dbg_stubs.desc.min_stack_addr);
	ESP_XTENSA_DBGSTUBS_UPDATE_CODE_ENTRY(esp_xtensa->dbg_stubs.desc.data_alloc);
	ESP_XTENSA_DBGSTUBS_UPDATE_CODE_ENTRY(esp_xtensa->dbg_stubs.desc.data_free);
}

bool esp_xtensa_is_special_breakpoint(struct target *target, struct breakpoint *breakpoint)
{
	struct esp_xtensa_common *esp_xtensa = target_to_esp_xtensa(target);
	for (uint32_t slot = 0; slot < ESP_XTENSA_SPECIAL_BREAKPOINTS_MAX_NUM; slot++) {
		if (esp_xtensa->spec_brps[slot].data.oocd_bp == breakpoint)
			return true;
	}
	return false;
}

static int esp_xtensa_special_breakpoint_add(struct target *target, struct breakpoint *breakpoint)
{
	struct esp_xtensa_common *esp_xtensa = target_to_esp_xtensa(target);
	uint32_t slot;

	for (slot = 0; slot < ESP_XTENSA_SPECIAL_BREAKPOINTS_MAX_NUM; slot++) {
		if (esp_xtensa->spec_brps[slot].data.oocd_bp == NULL ||
			esp_xtensa->spec_brps[slot].data.oocd_bp == breakpoint)
			break;
	}
	if (slot == ESP_XTENSA_SPECIAL_BREAKPOINTS_MAX_NUM) {
		LOG_WARNING("%s: max SW flash slot reached, slot=%u", target_name(target), slot);
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}
	return esp_xtensa->spec_brps_ops.breakpoint_add(target, breakpoint,
		&esp_xtensa->spec_brps[slot]);
}

int esp_xtensa_breakpoint_add(struct target *target, struct breakpoint *breakpoint)
{
	int res = xtensa_breakpoint_add(target, breakpoint);
	if (res == ERROR_TARGET_RESOURCE_NOT_AVAILABLE && breakpoint->type == BKPT_HARD)
		return esp_xtensa_special_breakpoint_add(target, breakpoint);
	return res;
}

static int esp_xtensa_special_breakpoint_remove(struct target *target,
	struct breakpoint *breakpoint)
{
	struct esp_xtensa_common *esp_xtensa = target_to_esp_xtensa(target);
	uint32_t slot;

	for (slot = 0; slot < ESP_XTENSA_SPECIAL_BREAKPOINTS_MAX_NUM; slot++) {
		if (esp_xtensa->spec_brps[slot].data.oocd_bp != NULL &&
			esp_xtensa->spec_brps[slot].data.oocd_bp == breakpoint)
			break;
	}
	if (slot == ESP_XTENSA_SPECIAL_BREAKPOINTS_MAX_NUM) {
		LOG_WARNING("%s: max SW flash slot reached, slot=%u", target_name(target), slot);
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}
	return esp_xtensa->spec_brps_ops.breakpoint_remove(target, &esp_xtensa->spec_brps[slot]);
}

int esp_xtensa_breakpoint_remove(struct target *target, struct breakpoint *breakpoint)
{
	int res = xtensa_breakpoint_remove(target, breakpoint);
	if (res == ERROR_TARGET_RESOURCE_NOT_AVAILABLE && breakpoint->type == BKPT_HARD)
		return esp_xtensa_special_breakpoint_remove(target, breakpoint);
	return res;
}

/*
The TDI pin is also used as a flash Vcc bootstrap pin. If we reset the CPU externally, the last state of the TDI pin can
allow the power to an 1.8V flash chip to be raised to 3.3V, or the other way around. Users can use the
esp32 flashbootstrap command to set a level, and this routine will make sure the tdi line will return to
that when the jtag port is idle.
*/
void esp_xtensa_queue_tdi_idle(struct target *target)
{
	struct esp_xtensa_common *esp_xtensa = target_to_esp_xtensa(target);
	static uint8_t value;
	uint8_t t[4] = { 0, 0, 0, 0 };

	if (esp_xtensa->flash_bootstrap == FBS_TMSLOW) {
		/*Make sure tdi is 0 at the exit of queue execution */
		value = 0;
	} else if (esp_xtensa->flash_bootstrap == FBS_TMSHIGH) {
		/*Make sure tdi is 1 at the exit of queue execution */
		value = 1;
	} else
		return;

	/*Scan out 1 bit, do not move from IRPAUSE after we're done. */
	buf_set_u32(t, 0, 1, value);
	jtag_add_plain_ir_scan(1, t, NULL, TAP_IRPAUSE);
}

static int esp_xtensa_do_semihosting(struct target *target)
{
	struct esp_xtensa_common *esp_xtensa = target_to_esp_xtensa(target);
	int syscall_ret = 0, syscall_errno = 0, retval;
	/* TODO: use a2, a3, a4, a5, a6 for syscall params when problem with a3 corruption will be
	 * solved */
	xtensa_reg_val_t a2 = xtensa_reg_get(target, XT_REG_IDX_A2);
	xtensa_reg_val_t a3 = xtensa_reg_get(target, SYSCALL_PARAM2_REG);
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
	switch (a2) {
		case ESP_SYS_OPEN:
		{
			int mode, base_len = 0;

			if (a4 == 0) {
				LOG_ERROR("Zero file name length!");
				syscall_ret = -1;
				/* TODO: check errno corectness, here and in other places */
				syscall_errno = EINVAL;
				break;
			}
			if (esp_xtensa->semihost.basedir && (a5 & ESP_O_SEMIHOST_ABSPATH) == 0)
				base_len = strlen(esp_xtensa->semihost.basedir);
			char *file_name = malloc(base_len+a4+1);
			if (!file_name) {
				LOG_ERROR("Failed to alloc memory for file name!");
				syscall_ret = -1;
				syscall_errno = EINVAL;
				break;
			}
			if (esp_xtensa->semihost.basedir)
				strcpy(file_name, esp_xtensa->semihost.basedir);
			retval = target_read_buffer(target, a3, a4, (uint8_t *)file_name+base_len);
			if (retval != ERROR_OK) {
				free(file_name);
				LOG_ERROR("Failed to read name of file to open!");
				syscall_ret = -1;
				syscall_errno = EINVAL;
				break;
			}
			file_name[base_len+a4] = 0;

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
		case ESP_SYS_CLOSE:
			if (a3 <= ESP_FD_MIN) {
				LOG_ERROR("Invalid file desc %d!", a3);
				syscall_ret = -1;
				/* TODO: check errno corectness, here and in other places */
				syscall_errno = EINVAL;
				break;
			}
			syscall_ret = close(a3);
			syscall_errno = errno;
			LOG_DEBUG("Close file %d. Ret %d. Error %d.", a3, syscall_ret,
			syscall_errno);
			break;
		case ESP_SYS_WRITE:
		{
			LOG_DEBUG("Req write file %d. %d bytes.", a3, a5);
			if (a3 <= ESP_FD_MIN) {
				LOG_ERROR("Invalid file desc %d!", a3);
				syscall_ret = -1;
				/* TODO: check errno corectness, here and in other places */
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
		case ESP_SYS_READ:
		{
			LOG_DEBUG("Req read file %d. %d bytes.", a3, a5);
			if (a3 <= ESP_FD_MIN) {
				LOG_ERROR("Invalid file desc %d!", a3);
				syscall_ret = -1;
				/* TODO: check errno corectness, here and in other places */
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
		case ESP_SYS_SEEK:
		{
			LOG_DEBUG("Req seek file %d. To %x, mode %d.", a3, a4, a5);
			if (a3 <= ESP_FD_MIN) {
				LOG_ERROR("Invalid file desc %d!", a3);
				syscall_ret = -1;
				/* TODO: check errno corectness, here and in other places */
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
			syscall_errno = EINVAL;
	}

	xtensa_reg_set(target, SYSCALL_RETVAL_REG, syscall_ret);
	xtensa_reg_set(target, SYSCALL_PARAM2_REG, syscall_errno);

	return ERROR_OK;
}

COMMAND_HELPER(esp_xtensa_cmd_flashbootstrap_do, struct esp_xtensa_common *esp_xtensa)
{
	int state = -1;

	if (CMD_ARGC < 1) {
		const char *st;
		state = esp_xtensa->flash_bootstrap;
		if (state == FBS_DONTCARE)
			st = "Don't care";
		else if (state == FBS_TMSLOW)
			st = "Low (3.3V)";
		else if (state == FBS_TMSHIGH)
			st = "High (1.8V)";
		else
			st = "None";
		command_print(CMD, "Current idle tms state: %s", st);
		return ERROR_OK;
	}

	if (!strcasecmp(CMD_ARGV[0], "none"))
		state = FBS_DONTCARE;
	else if (!strcasecmp(CMD_ARGV[0], "1.8"))
		state = FBS_TMSHIGH;
	else if (!strcasecmp(CMD_ARGV[0], "3.3"))
		state = FBS_TMSLOW;
	else if (!strcasecmp(CMD_ARGV[0], "high"))
		state = FBS_TMSHIGH;
	else if (!strcasecmp(CMD_ARGV[0], "low"))
		state = FBS_TMSLOW;

	if (state == -1) {
		command_print(CMD,
			"Argument unknown. Please pick one of none, high, low, 1.8 or 3.3");
		return ERROR_FAIL;
	}
	esp_xtensa->flash_bootstrap = state;
	return ERROR_OK;
}

COMMAND_HELPER(esp_xtensa_cmd_semihost_basedir_do, struct esp_xtensa_common *esp_xtensa)
{
	if (CMD_ARGC != 1) {
		command_print(CMD,
			"Current semihosting base dir: %s",
			esp_xtensa->semihost.basedir ? esp_xtensa->semihost.basedir : "");
		return ERROR_OK;
	}

	char *s = strdup(CMD_ARGV[0]);
	if (!s) {
		command_print(CMD, "Failed to allocate memory!");
		return ERROR_FAIL;
	}
	if (esp_xtensa->semihost.basedir)
		free(esp_xtensa->semihost.basedir);
	esp_xtensa->semihost.basedir = s;

	return ERROR_OK;
}

COMMAND_HANDLER(esp_xtensa_cmd_flashbootstrap)
{
	if (CMD_ARGC < 1)
		return CALL_COMMAND_HANDLER(esp_xtensa_cmd_flashbootstrap_do,
			target_to_esp_xtensa(get_current_target(CMD_CTX)));
	return CALL_COMMAND_HANDLER(esp_xtensa_cmd_flashbootstrap_do,
		target_to_esp_xtensa(get_current_target(CMD_CTX)));
}

COMMAND_HANDLER(esp_xtensa_cmd_semihost_basedir)
{
	return CALL_COMMAND_HANDLER(esp_xtensa_cmd_semihost_basedir_do,
		target_to_esp_xtensa(get_current_target(CMD_CTX)));
}

const struct command_registration esp_xtensa_command_handlers[] = {
	{
		.name = "flashbootstrap",
		.handler = esp_xtensa_cmd_flashbootstrap,
		.mode = COMMAND_ANY,
		.help =
			"Set the idle state of the TMS pin, which at reset also is the voltage selector for the flash chip.",
		.usage = "none|1.8|3.3|high|low",
	},
	{
		.name = "semihost_basedir",
		.handler = esp_xtensa_cmd_semihost_basedir,
		.mode = COMMAND_ANY,
		.help = "Set the base directory for semohosting I/O.",
		.usage = "dir",
	},
	COMMAND_REGISTRATION_DONE
};
