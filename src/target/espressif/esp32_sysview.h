/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   ESP32 sysview tracing module                                          *
 *   Copyright (C) 2020 Espressif Systems Ltd.                             *
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
	unsigned int sv_last_core_id;
	int sv_trace_running;
};

struct esp32_apptrace_cmd_ctx;

int esp32_sysview_cmd_init(struct esp32_apptrace_cmd_ctx *cmd_ctx,
	struct command_invocation *cmd,
	int mode,
	bool mcore_format,
	const char **argv,
	int argc);
int esp32_sysview_cmd_cleanup(struct esp32_apptrace_cmd_ctx *cmd_ctx);
int esp32_sysview_process_data(struct esp32_apptrace_cmd_ctx *ctx,
	unsigned int core_id,
	uint8_t *data,
	uint32_t data_len);

#endif	/* OPENOCD_TARGET_ESP32_SYSVIEW_H */
