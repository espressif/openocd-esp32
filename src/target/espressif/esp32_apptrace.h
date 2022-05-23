/***************************************************************************
 *   ESP32 application trace module                                        *
 *   Copyright (C) 2017-2019 Espressif Systems Ltd.                        *
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
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>. *
 ***************************************************************************/

#ifndef OPENOCD_TARGET_ESP32_APPTRACE_H
#define OPENOCD_TARGET_ESP32_APPTRACE_H

#include <pthread.h>
#include <helper/command.h>
#include <helper/time_support.h>
#include <target/target.h>

#define ESP32_APPTRACE_MAX_CORES_NUM 2

struct esp32_apptrace_hw {
	uint32_t max_block_id;
	uint32_t (*max_block_size_get)(struct target *target);
	int (*status_reg_read)(struct target *target, uint32_t *stat);
	int (*ctrl_reg_write)(struct target *target,
		uint32_t block_id,
		uint32_t len,
		bool conn,
		bool data);
	int (*ctrl_reg_read)(struct target *target,
		uint32_t *block_id,
		uint32_t *len,
		bool *conn);
	int (*data_len_read)(struct target *target,
		uint32_t *block_id,
		uint32_t *len);
	int (*data_read)(struct target *target,
		uint32_t size,
		uint8_t *buffer,
		uint32_t block_id,
		bool ack);
	uint32_t (*usr_block_max_size_get)(struct target *target);
	int (*buffs_write)(struct target *target,
		uint32_t bufs_num,
		uint32_t buf_sz[],
		const uint8_t *bufs[],
		uint32_t block_id,
		bool ack,
		bool data);
	int (*leave_trace_crit_section_start)(struct target *target);
	int (*leave_trace_crit_section_stop)(struct target *target);
};

struct esp_apptrace_host2target_hdr {
	uint16_t block_sz;
};

struct esp32_apptrace_dest {
	void *priv;
	int (*write)(void *priv, uint8_t *data, uint32_t size);
	int (*clean)(void *priv);
	bool log_progress;
};

struct esp32_apptrace_format {
	uint32_t hdr_sz;
	int (*core_id_get)(uint8_t *hdr_buf);
	uint32_t (*usr_block_len_get)(uint8_t *hdr_buf, uint32_t *wr_len);
};

struct esp32_apptrace_cmd_stats {
	uint32_t incompl_blocks;
	uint32_t lost_bytes;
	float min_blk_read_time;
	float max_blk_read_time;
	float min_blk_proc_time;
	float max_blk_proc_time;
};

struct esp32_apptrace_cmd_ctx {
	volatile int running;
	int mode;
	/* TODO: use subtargets from target arch info */
	struct target *cpus[ESP32_APPTRACE_MAX_CORES_NUM];
	/* TODO: use cores num from target */
	int cores_num;
	const struct esp32_apptrace_hw *hw;
	const struct algorithm_hw *algo_hw;
	enum target_state target_state;
	uint32_t last_blk_id;
	pthread_mutex_t trax_blocks_mux;
	struct hlist_head free_trace_blocks;
	struct hlist_head ready_trace_blocks;
	uint32_t max_trace_block_sz;
	pthread_t data_processor;
	struct esp32_apptrace_format trace_format;
	int (*process_data)(struct esp32_apptrace_cmd_ctx *ctx, int core_id,
		uint8_t *data, uint32_t data_len);
	void (*auto_clean)(struct esp32_apptrace_cmd_ctx *ctx);
	uint32_t tot_len;
	uint32_t raw_tot_len;
	float stop_tmo;
	struct esp32_apptrace_cmd_stats stats;
	struct duration read_time;
	struct duration idle_time;
	void *cmd_priv;
};

struct esp32_apptrace_cmd_data {
	struct esp32_apptrace_dest data_dest;
	uint32_t poll_period;
	uint32_t max_len;
	uint32_t skip_len;
	bool wait4halt;
};

int esp32_apptrace_cmd_ctx_init(struct target *target,
	struct esp32_apptrace_cmd_ctx *cmd_ctx,
	int mode);
int esp32_apptrace_cmd_ctx_cleanup(struct esp32_apptrace_cmd_ctx *cmd_ctx);
void esp32_apptrace_cmd_args_parse(struct esp32_apptrace_cmd_ctx *cmd_ctx,
	struct esp32_apptrace_cmd_data *cmd_data,
	const char **argv,
	int argc);
int esp32_apptrace_dest_init(struct esp32_apptrace_dest dest[],
	const char *dest_paths[],
	int max_dests);
int esp32_apptrace_dest_cleanup(struct esp32_apptrace_dest dest[], int max_dests);
int esp_apptrace_usr_block_write(const struct esp32_apptrace_hw *hw, struct target *target,
	uint32_t block_id,
	const uint8_t *data,
	uint32_t size);
uint8_t *esp_apptrace_usr_block_get(uint8_t *buffer, uint32_t *size);

extern const struct command_registration esp32_apptrace_command_handlers[];

#endif	/* OPENOCD_TARGET_ESP32_APPTRACE_H */
