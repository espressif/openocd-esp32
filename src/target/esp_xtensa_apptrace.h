/***************************************************************************
 *   Application level tracing API for Espressif Xtensa chips              *
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
#ifndef ESP_XTENSA_APPTRACE_H__
#define ESP_XTENSA_APPTRACE_H__

#include "target.h"
#include "xtensa.h"

#define XTENSA_APPTRACE_BLOCK_ID_MSK          0x7FUL
#define XTENSA_APPTRACE_BLOCK_ID_MAX          XTENSA_APPTRACE_BLOCK_ID_MSK

struct esp_xtensa_apptrace_target2host_hdr {
	union {
		struct {
			uint16_t block_sz;
			uint16_t wr_sz;
		} gen;
		struct {
			uint8_t block_sz;
			uint8_t wr_sz;
		} sys_view;
	};
};

struct esp_xtensa_apptrace_host2target_hdr {
	uint16_t block_sz;
};

int esp_xtensa_apptrace_ctrl_reg_write(struct target *target,
	uint32_t block_id,
	uint32_t len,
	bool conn,
	bool data);
int esp_xtensa_apptrace_ctrl_reg_read(struct target *target,
	uint32_t *block_id,
	uint32_t *len,
	bool *conn);
int esp_xtensa_apptrace_status_reg_read(struct target *target, uint32_t *stat);
int esp_xtensa_apptrace_status_reg_write(struct target *target, uint32_t stat);
int esp_xtensa_apptrace_data_read(struct target *target,
	uint32_t size,
	uint8_t *buffer,
	uint32_t block_id,
	bool ack);
static inline int esp_xtensa_apptrace_data_len_read(struct target *target,
	uint32_t *block_id,
	uint32_t *len)
{
	return esp_xtensa_apptrace_ctrl_reg_read(target, block_id, len, NULL);
}
int esp_xtensa_apptrace_buffs_write(struct target *target,
	uint32_t bufs_num,
	uint32_t buf_sz[],
	const uint8_t *bufs[],
	uint32_t block_id,
	int ack,
	int data);
int esp_xtensa_swdbg_activate(struct target *target, int enab);

uint8_t *esp_xtensa_apptrace_usr_block_get(uint8_t *buffer, uint32_t *size);
int esp_xtensa_apptrace_usr_block_write(struct target *target,
	uint32_t block_id,
	const uint8_t *data,
	uint32_t size);
int esp_xtensa_sysview_cmds_queue(struct target *core_target,
	uint8_t *cmds,
	uint32_t cmds_num,
	uint32_t block_id);

static inline uint32_t esp_xtensa_apptrace_block_max_size_get(struct target *target)
{
	struct xtensa *xtensa = target_to_xtensa(target);
	return xtensa->core_config->trace.mem_sz;
}

static inline uint32_t esp_xtensa_apptrace_usr_block_max_size_get(struct target *target)
{
	struct xtensa *xtensa = target_to_xtensa(target);
	return (xtensa->core_config->trace.mem_sz -
		sizeof(struct esp_xtensa_apptrace_host2target_hdr));
}

#endif	/*ESP_XTENSA_APPTRACE_H__*/
