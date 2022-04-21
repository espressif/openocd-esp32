/***************************************************************************
 *   ESP32 sysview tracing module                                          *
 *   Copyright (C) 2020 Espressif Systems Ltd.                             *
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

#ifndef OPENOCD_TARGET_ESP32_SYSVIEW_H
#define OPENOCD_TARGET_ESP32_SYSVIEW_H

typedef enum {
	SEGGER_SYSVIEW_COMMAND_ID_START = 1,
	SEGGER_SYSVIEW_COMMAND_ID_STOP,
	SEGGER_SYSVIEW_COMMAND_ID_GET_SYSTIME,
	SEGGER_SYSVIEW_COMMAND_ID_GET_TASKLIST,
	SEGGER_SYSVIEW_COMMAND_ID_GET_SYSDESC,
	SEGGER_SYSVIEW_COMMAND_ID_GET_NUMMODULES,
	SEGGER_SYSVIEW_COMMAND_ID_GET_MODULEDESC,
	/* Extended commands: Commands >= 128 have a second parameter */
	SEGGER_SYSVIEW_COMMAND_ID_GET_MODULE = 128
} SEGGER_SYSVIEW_COMMAND_ID;

struct esp32_sysview_cmd_data {
	/* Should be the first field. Generic apptrace command handling code accesses it */
	struct esp32_apptrace_cmd_data apptrace;
	struct esp32_apptrace_dest data_dests[ESP32_APPTRACE_MAX_CORES_NUM];
	bool mcore_format;
	uint32_t sv_acc_time_delta;
	int sv_last_core_id;
	int sv_trace_running;
};

struct esp32_apptrace_cmd_ctx;

int esp32_sysview_cmd_init(struct target *target,
	struct esp32_apptrace_cmd_ctx *cmd_ctx,
	int mode,
	bool mcore_format,
	const char **argv,
	int argc);
int esp32_sysview_cmd_cleanup(struct esp32_apptrace_cmd_ctx *cmd_ctx);
int esp32_sysview_process_data(struct esp32_apptrace_cmd_ctx *ctx,
	int core_id,
	uint8_t *data,
	uint32_t data_len);

#endif	/* OPENOCD_TARGET_ESP32_SYSVIEW_H */
