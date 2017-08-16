/***************************************************************************
 *   ESP108 application trace module                                       *
 *   Copyright (C) 2017 Espressif Systems Ltd.                             *
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

#ifndef XTENSA_ESP108_APPTRACE_H
#define XTENSA_ESP108_APPTRACE_H

#include "command.h"
#include "time_support.h"

//TODO: remove from header
#define ESP32_TRACEMEM_BLOCK_SZ		(0x4000)
#define ESP32_USR_BLOCK_SZ_MAX		(ESP32_TRACEMEM_BLOCK_SZ - sizeof(struct esp_apptrace_host2target_hdr))

//TODO: remove from header
struct esp_apptrace_host2target_hdr {
	uint16_t   block_sz;
};

int esp_cmd_apptrace_generic(struct target *target, int sys_view, const char **argv, int argc);
int esp_cmd_gcov(struct target *target, const char **argv, int argc);
__COMMAND_HANDLER(esp108_cmd_apptrace);
__COMMAND_HANDLER(esp108_cmd_sysview);

int esp108_apptrace_write_ctrl_reg(struct target *target, uint32_t block_id, uint32_t len, int conn, int data);
int esp108_apptrace_read_data_len(struct target *target, uint32_t *block_id, uint32_t *len);
int esp108_apptrace_read_data(struct target *target, uint32_t size, uint8_t *buffer, uint32_t block_id, int ack, struct duration *dur);
uint8_t *esp108_apptrace_usr_block_get(uint8_t *buffer, uint32_t *size);
int esp108_apptrace_usr_block_write(struct target *target, uint32_t block_id, const uint8_t *data, uint32_t size);

#endif // XTENSA_ESP108_APPTRACE_H
