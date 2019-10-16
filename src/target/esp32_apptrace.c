/***************************************************************************
 *   ESP32 application tracing module for OpenOCD                         *
 *   Copyright (C) 2017 Espressif Systems Ltd.                             *
 *   Author: Alexey Gerenkov <alexey@espressif.com>                        *
 *                                                                         *
 *   Derived from original ESP8266 target.                                 *
 *   Copyright (C) 2015 by Angus Gratton                                   *
 *   gus@projectgus.com                                                    *
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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <pthread.h>
#include "target.h"
#include "target_type.h"
#include "time_support.h"
#include "list.h"
#include "xtensa_mcore.h"
#include "esp_xtensa.h"
#include "esp_xtensa_apptrace.h"
#include "esp32_apptrace.h"


#define ESP_APPTRACE_MAX_CORES_NUM 2

#define ESP32_APPTRACE_USER_BLOCK_CORE(_v_) ((_v_) >> 15)
#define ESP32_APPTRACE_USER_BLOCK_LEN(_v_)  ((_v_) & ~(1 << 15))
/* in SystemView mode core ID is passed in event ID field */
#define ESP32_SYSVIEW_USER_BLOCK_CORE(_v_)  (0)	/* not used */
#define ESP32_SYSVIEW_USER_BLOCK_LEN(_v_)       (_v_)

#define ESP32_APPTRACE_USER_BLOCK_HDR_SZ        4
#define ESP32_SYSVIEW_USER_BLOCK_HDR_SZ     2

#define ESP_APPTRACE_CMD_MODE_GEN           0
#define ESP_APPTRACE_CMD_MODE_SYSVIEW       1
#define ESP_APPTRACE_CMD_MODE_SYNC          2

#define ESP32_APPTRACE_TGT_STATE_TMO            5000
#define ESP_APPTRACE_TIME_STATS_ENABLE      1
#define ESP_APPTRACE_BLOCKS_POOL_SZ         10

#define ESP_APPTRACE_FILE_CMD_FOPEN     0x0
#define ESP_APPTRACE_FILE_CMD_FCLOSE    0x1
#define ESP_APPTRACE_FILE_CMD_FWRITE    0x2
#define ESP_APPTRACE_FILE_CMD_FREAD     0x3
#define ESP_APPTRACE_FILE_CMD_FSEEK     0x4
#define ESP_APPTRACE_FILE_CMD_FTELL     0x5
#define ESP_APPTRACE_FILE_CMD_FTELL     0x5
#define ESP_APPTRACE_FILE_CMD_STOP      0x6	/* indicates that there is no files to transfer */

#define ESP_GCOV_FILES_MAX_NUM          512

/* grabbed from SystemView target sources */
#define   SYSVIEW_EVTID_NOP                 0	/* Dummy packet. */
#define   SYSVIEW_EVTID_OVERFLOW            1
#define   SYSVIEW_EVTID_ISR_ENTER           2
#define   SYSVIEW_EVTID_ISR_EXIT            3
#define   SYSVIEW_EVTID_TASK_START_EXEC     4
#define   SYSVIEW_EVTID_TASK_STOP_EXEC      5
#define   SYSVIEW_EVTID_TASK_START_READY    6
#define   SYSVIEW_EVTID_TASK_STOP_READY     7
#define   SYSVIEW_EVTID_TASK_CREATE         8
#define   SYSVIEW_EVTID_TASK_INFO           9
#define   SYSVIEW_EVTID_TRACE_START         10
#define   SYSVIEW_EVTID_TRACE_STOP          11
#define   SYSVIEW_EVTID_SYSTIME_CYCLES      12
#define   SYSVIEW_EVTID_SYSTIME_US          13
#define   SYSVIEW_EVTID_SYSDESC             14
#define   SYSVIEW_EVTID_USER_START          15
#define   SYSVIEW_EVTID_USER_STOP           16
#define   SYSVIEW_EVTID_IDLE                17
#define   SYSVIEW_EVTID_ISR_TO_SCHEDULER    18
#define   SYSVIEW_EVTID_TIMER_ENTER         19
#define   SYSVIEW_EVTID_TIMER_EXIT          20
#define   SYSVIEW_EVTID_STACK_INFO          21
#define   SYSVIEW_EVTID_MODULEDESC          22

#define   SYSVIEW_EVTID_INIT                24
#define   SYSVIEW_EVTID_NAME_RESOURCE       25
#define   SYSVIEW_EVTID_PRINT_FORMATTED     26
#define   SYSVIEW_EVTID_NUMMODULES          27

#define   SYSVIEW_SYNC_LEN                  10

#define   SYSVIEW_EVENT_ID_MAX             (200)

#define SYSVIEW_ENCODE_U32(dest, val) {					    \
		uint8_t *sv_ptr;			 \
		uint32_t sv_data;			 \
		sv_ptr = dest;				 \
		sv_data = val;				 \
		while (sv_data > 0x7F) {		  \
			*sv_ptr++ = (uint8_t)(sv_data | 0x80); \
			sv_data >>= 7;			       \
		};					 \
		*sv_ptr++ = (uint8_t)sv_data;		 \
		dest = sv_ptr;				 \
};

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

struct esp32_apptrace_target_state {
	int running;
	uint32_t block_id;
	uint32_t data_len;
};

struct esp32_apptrace_cmd_stats {
	uint32_t incompl_blocks;
	uint32_t lost_bytes;
#if ESP_APPTRACE_TIME_STATS_ENABLE
	float min_blk_read_time;
	float max_blk_read_time;
	float min_blk_proc_time;
	float max_blk_proc_time;
#endif
};

struct esp32_apptrace_dest_file_data {
	int fout;
};

typedef int (*esp32_apptrace_dest_write_t)(void *priv, uint8_t *data, uint32_t size);
typedef int (*esp32_apptrace_dest_cleanup_t)(void *priv);

struct esp32_apptrace_dest {
	void *priv;
	esp32_apptrace_dest_write_t write;
	esp32_apptrace_dest_cleanup_t clean;
};

struct esp32_apptrace_cmd_ctx;

typedef int (*esp32_apptrace_process_data_t)(struct esp32_apptrace_cmd_ctx *ctx, int core_id,
	uint8_t *data, uint32_t data_len);

struct esp32_apptrace_block {
	struct hlist_node node;
	uint8_t *data;
	uint32_t data_len;
};

struct esp32_apptrace_cmd_ctx {
	volatile int running;
	int mode;
	/* TODO: use subtargets from target arch info */
	struct target *cpus[ESP_APPTRACE_MAX_CORES_NUM];
	/* TODO: use cores num from target */
	int cores_num;
	uint32_t last_blk_id;
	pthread_mutex_t trax_blocks_mux;
	struct hlist_head free_trax_blocks;
	struct hlist_head ready_trax_blocks;
	uint8_t *trax_block_data;
	uint32_t trax_block_sz;
	pthread_t data_processor;
	esp32_apptrace_process_data_t process_data;
	float stop_tmo;
	uint32_t tot_len;
	uint32_t raw_tot_len;
	struct esp32_apptrace_cmd_stats stats;
	struct duration read_time;
	struct duration idle_time;
	void *cmd_priv;
};

struct esp32_apptrace_cmd_data {
	struct esp32_apptrace_dest data_dests[ESP_APPTRACE_MAX_CORES_NUM];
	uint32_t poll_period;
	uint32_t sv_acc_time_delta;
	int sv_last_core_id;
	int sv_trace_running;
	uint32_t max_len;
	uint32_t skip_len;
	bool wait4halt;
};

struct esp32_gcov_cmd_data {
	FILE *files[ESP_GCOV_FILES_MAX_NUM];
	uint32_t files_num;
	bool wait4halt;
};

/* need to check `shutdown_openocd` when poll period is less then 1 ms in order to react on CTRL+C
 * etc. */
/* Actually `shutdown_openocd` is an enum type var. Non-zero value tells that shutdown is requested,
 * for now this hasck works. */
/* TODO: Currently for periods less then 1ms we loop in command handler until CTRL+C is pressed.
 *       Another trace data polling mechanism is necessary for small periods. */
extern int shutdown_openocd;

static int esp32_sysview_process_data(struct esp32_apptrace_cmd_ctx *ctx,
	int core_id,
	uint8_t *data,
	uint32_t data_len);
static int esp_gcov_process_data(struct esp32_apptrace_cmd_ctx *ctx,
	int core_id,
	uint8_t *data,
	uint32_t data_len);
static void *esp32_apptrace_data_processor(void *arg);
static int esp32_apptrace_handle_trace_block(struct esp32_apptrace_cmd_ctx *ctx,
	struct esp32_apptrace_block *block);
static int esp32_apptrace_cmd_ctx_cleanup(struct esp32_apptrace_cmd_ctx *cmd_ctx);
static int esp32_apptrace_get_data_info(struct esp32_apptrace_cmd_ctx *ctx,
	struct esp32_apptrace_target_state *target_state,
	uint32_t *fired_target_num);


/*********************************************************************
*                       Trace destination API
**********************************************************************/

static int esp32_apptrace_file_dest_write(void *priv, uint8_t *data, uint32_t size)
{
	struct esp32_apptrace_dest_file_data *dest_data =
		(struct esp32_apptrace_dest_file_data *)priv;

	ssize_t wr_sz = write(dest_data->fout, data, size);
	if (wr_sz != (ssize_t)size) {
		LOG_ERROR("Failed to write %u bytes to out file (%d)! Written %d.", size, errno,
			(int)wr_sz);
		return ERROR_FAIL;
	}
	return ERROR_OK;
}

static int esp32_apptrace_file_dest_cleanup(void *priv)
{
	struct esp32_apptrace_dest_file_data *dest_data =
		(struct esp32_apptrace_dest_file_data *)priv;

	if (dest_data->fout > 0)
		close(dest_data->fout);
	free(dest_data);
	return ERROR_OK;
}

static int esp32_apptrace_file_dest_init(struct esp32_apptrace_dest *dest, const char *dest_name)
{
	struct esp32_apptrace_dest_file_data *dest_data =
		malloc(sizeof(struct esp32_apptrace_dest_file_data));
	if (!dest_data) {
		LOG_ERROR("Failed to alloc mem for file dest!");
		return ERROR_FAIL;
	}
	memset(dest_data, 0, sizeof(struct esp32_apptrace_dest_file_data));

	LOG_INFO("Open file %s", dest_name);
	dest_data->fout = open(dest_name, O_WRONLY|O_CREAT|O_TRUNC|O_BINARY, 0666);
	if (dest_data->fout <= 0) {
		LOG_ERROR("Failed to open file %s", dest_name);
		return ERROR_FAIL;
	}

	dest->priv = dest_data;
	dest->write = esp32_apptrace_file_dest_write;
	dest->clean = esp32_apptrace_file_dest_cleanup;

	return ERROR_OK;
}

static int esp32_apptrace_dest_init(struct esp32_apptrace_dest dest[],
	const char *dest_paths[],
	int max_dests)
{
	int res = ERROR_OK, i;

	for (i = 0; i < max_dests; i++) {
		if (strncmp(dest_paths[i], "file://", 7) == 0) {
			res = esp32_apptrace_file_dest_init(&dest[i], &dest_paths[i][7]);
			if (res != ERROR_OK) {
				LOG_ERROR("Failed to init destination '%s'!", dest_paths[i]);
				return 0;
			}
		} else
			break;
	}

	return i;
}

static int esp32_apptrace_dest_cleanup(struct esp32_apptrace_dest dest[], int max_dests)
{
	for (int i = 0; i < max_dests; i++) {
		if (dest[i].clean)
			return dest[i].clean(dest[i].priv);
	}
	return ERROR_OK;
}

/*********************************************************************
*                 Trace data blocks management API
**********************************************************************/

static void esp32_apptrace_blocks_pool_cleanup(struct esp32_apptrace_cmd_ctx *ctx)
{
	struct esp32_apptrace_block *cur;
	struct hlist_node *pos, *tmp;
	hlist_for_each_entry_safe(cur, pos, tmp, &ctx->free_trax_blocks, node) {
		if (cur) {
			hlist_del(&cur->node);
			if (cur->data)
				free(cur->data);
			free(cur);
		}
	}
	hlist_for_each_entry_safe(cur, pos, tmp, &ctx->ready_trax_blocks, node) {
		if (cur) {
			hlist_del(&cur->node);
			if (cur->data)
				free(cur->data);
			free(cur);
		}
	}
}

static struct esp32_apptrace_block *esp32_apptrace_free_block_get(
	struct esp32_apptrace_cmd_ctx *ctx)
{
	struct esp32_apptrace_block *block = NULL;

	int res = pthread_mutex_lock(&ctx->trax_blocks_mux);
	if (res == 0) {
		if (!hlist_empty(&ctx->free_trax_blocks)) {
			/*get first */
			block = hlist_entry(ctx->free_trax_blocks.first,
				struct esp32_apptrace_block,
				node);
			hlist_del(&block->node);
		}
		res = pthread_mutex_unlock(&ctx->trax_blocks_mux);
		if (res)
			LOG_ERROR("Failed to unlock blocks pool (%d)!", res);
	}

	return block;
}

static int esp32_apptrace_ready_block_put(struct esp32_apptrace_cmd_ctx *ctx,
	struct esp32_apptrace_block *block)
{
	int res;

	res = pthread_mutex_lock(&ctx->trax_blocks_mux);
	if (res == 0) {
		LOG_DEBUG("esp32_apptrace_ready_block_put");
		/* add to ready blocks list */
		INIT_HLIST_NODE(&block->node);
		hlist_add_head(&block->node, &ctx->ready_trax_blocks);
		res = pthread_mutex_unlock(&ctx->trax_blocks_mux);
		if (res) {
			LOG_ERROR("Failed to unlock blocks pool (%d)!", res);
			res = ERROR_FAIL;
		}
		res = ERROR_OK;
	} else {
		LOG_ERROR("Failed to lock blocks pool (%d)!", res);
		res = ERROR_FAIL;
	}

	return res;
}

static struct esp32_apptrace_block *esp32_apptrace_ready_block_get(
	struct esp32_apptrace_cmd_ctx *ctx)
{
	struct esp32_apptrace_block *block = NULL;

	if (pthread_mutex_trylock(&ctx->trax_blocks_mux) == 0) {
		if (!hlist_empty(&ctx->ready_trax_blocks)) {
			struct hlist_node *tmp;
			/* find to the last */
			hlist_for_each_entry(block, tmp, &ctx->ready_trax_blocks, node);
			/* remove it from ready list */
			hlist_del(&block->node);
		}
		int res = pthread_mutex_unlock(&ctx->trax_blocks_mux);
		if (res)
			LOG_ERROR("Failed to unlock blocks pool (%d)!", res);
	}

	return block;
}

static int esp32_apptrace_block_free(struct esp32_apptrace_cmd_ctx *ctx,
	struct esp32_apptrace_block *block)
{
	int res;

	res = pthread_mutex_lock(&ctx->trax_blocks_mux);
	if (res == 0) {
		/* add to free blocks list */
		INIT_HLIST_NODE(&block->node);
		hlist_add_head(&block->node, &ctx->free_trax_blocks);
		res = pthread_mutex_unlock(&ctx->trax_blocks_mux);
		if (res) {
			LOG_ERROR("Failed to unlock blocks pool (%d)!", res);
			res = ERROR_FAIL;
		}
		res = ERROR_OK;
	} else {
		LOG_ERROR("Failed to lock blocks pool (%d)!", res);
		res = ERROR_FAIL;
	}

	return res;
}

static int esp32_apptrace_wait_tracing_finished(struct esp32_apptrace_cmd_ctx *ctx)
{
	int i = 0, tries = LOG_LEVEL_IS(LOG_LVL_DEBUG) ? 700 : 50;
	while (!hlist_empty(&ctx->ready_trax_blocks)) {
		alive_sleep(100);
		if (i++ == tries) {
			LOG_ERROR("Failed to wait for pended TRAX blocks!");
			return ERROR_FAIL;
		}
	}
	/* signal thread to stop */
	ctx->running = 0;
	/* wait for the processor thread to finish */
	if (ctx->data_processor != (pthread_t)-1) {
		void *thr_res;
		int res = pthread_join(ctx->data_processor, (void *)&thr_res);
		if (res)
			LOG_ERROR("Failed to join trace data processor thread (%d)!", res);
		else
			LOG_INFO("Trace data processor thread exited with %ld", (long)thr_res);
	}

	return ERROR_OK;
}

/*********************************************************************
*                          Trace commands
**********************************************************************/

static int esp32_apptrace_cmd_ctx_init(struct target *target,
	struct esp32_apptrace_cmd_ctx *cmd_ctx,
	int mode)
{
	int res;

	memset(cmd_ctx, 0, sizeof(struct esp32_apptrace_cmd_ctx));

	cmd_ctx->data_processor = (pthread_t)-1;
	cmd_ctx->stop_tmo = -1.0;	/* infinite */
	cmd_ctx->mode = mode;

	if (target->type->get_cores_count) {	/* single core chips have no get_cores_count()
						 * callback */
		struct xtensa_mcore_common *xtensa_mcore = target_to_xtensa_mcore(target);
		/* HACK: OOCD has no attach event for telnet session so
		 * halt and resume target if numbers of working cores has not been detected yet,
		 * e.g. when we connected via telnet to running target */
		if (xtensa_mcore->cores_num == 0 && target->state == TARGET_RUNNING) {
			res = target_halt(target);
			if (res != ERROR_OK) {
				LOG_ERROR("Failed to halt target (%d)!", res);
				return res;
			}
			res =
				target_wait_state(target,
				TARGET_HALTED,
				ESP32_APPTRACE_TGT_STATE_TMO);
			if (res != ERROR_OK) {
				LOG_ERROR("Failed to halt target (%d)!", res);
				return res;
			}
			res = target_resume(target, 1, 0, 1, 0);
			if (res != ERROR_OK) {
				LOG_ERROR("Failed to halt target (%d)!", res);
				return res;
			}
		}
		cmd_ctx->cores_num = target_get_core_count(target);
		for (int i = 0; i < cmd_ctx->cores_num; i++)
			cmd_ctx->cpus[i] = &xtensa_mcore->cores_targets[i];
	} else {
		cmd_ctx->cores_num = 1;
		cmd_ctx->cpus[0] = target;
	}

	struct xtensa *xtensa = target_to_xtensa(cmd_ctx->cpus[0]);
	struct xtensa_trace_status trace_status;
	res = xtensa_dm_trace_status_read(&xtensa->dbg_mod, &trace_status);
	if (res) {
		LOG_ERROR("Failed to read TRAX status (%d)!", res);
		return res;
	}

	cmd_ctx->trax_block_sz = 1 << (((trace_status.stat >> 8) & 0x1f) - 2);
	cmd_ctx->trax_block_sz *= 4;
	LOG_INFO("Total trace memory: %d bytes", cmd_ctx->trax_block_sz);
	struct xtensa_trace_config trace_config;
	res = xtensa_dm_trace_config_read(&xtensa->dbg_mod, &trace_config);
	if (res) {
		LOG_ERROR("Failed to read TRAX config (%d)!", res);
		return res;
	}
	LOG_DEBUG("ctrl=0x%x memadrstart=0x%x memadrend=0x%x traxadr=0x%x",
		trace_config.ctrl,
		trace_config.memaddr_start,
		trace_config.memaddr_end,
		trace_config.addr);

	INIT_HLIST_HEAD(&cmd_ctx->ready_trax_blocks);
	INIT_HLIST_HEAD(&cmd_ctx->free_trax_blocks);
	for (int i = 0; i < ESP_APPTRACE_BLOCKS_POOL_SZ; i++) {
		struct esp32_apptrace_block *block = malloc(sizeof(struct esp32_apptrace_block));
		if (!block) {
			LOG_ERROR("Failed to alloc trace buffer entry!");
			esp32_apptrace_blocks_pool_cleanup(cmd_ctx);
			return ERROR_FAIL;
		}
		block->data = malloc(cmd_ctx->trax_block_sz);
		if (!block->data) {
			free(block);
			LOG_ERROR("Failed to alloc trace buffer %d bytes!", cmd_ctx->trax_block_sz);
			esp32_apptrace_blocks_pool_cleanup(cmd_ctx);
			return ERROR_FAIL;
		}
		INIT_HLIST_NODE(&block->node);
		hlist_add_head(&block->node, &cmd_ctx->free_trax_blocks);
	}

	cmd_ctx->running = 1;

	res = pthread_mutex_init(&cmd_ctx->trax_blocks_mux, NULL);
	if (res) {
		LOG_ERROR("Failed to blocks pool mux (%d)!", res);
		esp32_apptrace_blocks_pool_cleanup(cmd_ctx);
		return ERROR_FAIL;
	}
	if (cmd_ctx->mode != ESP_APPTRACE_CMD_MODE_SYNC) {
		res = pthread_create(&cmd_ctx->data_processor,
			NULL,
			esp32_apptrace_data_processor,
			cmd_ctx);
		if (res) {
			LOG_ERROR("Failed to start trace data processor thread (%d)!", res);
			cmd_ctx->data_processor = (pthread_t)-1;
			pthread_mutex_destroy(&cmd_ctx->trax_blocks_mux);
			esp32_apptrace_blocks_pool_cleanup(cmd_ctx);
			return ERROR_FAIL;
		}
	}

#if ESP_APPTRACE_TIME_STATS_ENABLE
	cmd_ctx->stats.min_blk_read_time = 1000000.0;
	cmd_ctx->stats.min_blk_proc_time = 1000000.0;
#endif
	if (duration_start(&cmd_ctx->idle_time) != 0) {
		LOG_ERROR("Failed to start idle time measurement!");
		esp32_apptrace_cmd_ctx_cleanup(cmd_ctx);
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

static int esp32_apptrace_cmd_ctx_cleanup(struct esp32_apptrace_cmd_ctx *cmd_ctx)
{
	pthread_mutex_destroy(&cmd_ctx->trax_blocks_mux);
	esp32_apptrace_blocks_pool_cleanup(cmd_ctx);
	return ERROR_OK;
}

#define ESP32_APPTRACE_CMD_NUM_ARG_CHECK(_arg_, _start_, _end_)	   \
	do { \
		if ((_arg_) == 0 && (_start_) == (_end_)) { \
			LOG_ERROR("Invalid '" # _arg_ "' arg!"); \
			res = ERROR_FAIL; \
			goto on_error; \
		} \
	} while (0)

static int esp32_apptrace_cmd_init(struct target *target,
	struct esp32_apptrace_cmd_ctx *cmd_ctx,
	int mode,
	const char **argv,
	int argc)
{
	int res;

	if (argc < 1) {
		LOG_ERROR("Not enough args! Need trace data destination!");
		return ERROR_FAIL;
	}

	res = esp32_apptrace_cmd_ctx_init(target, cmd_ctx, mode);
	if (res)
		return res;
	if (cmd_ctx->mode == ESP_APPTRACE_CMD_MODE_SYSVIEW && argc < cmd_ctx->cores_num) {
		LOG_ERROR("Not enough args! Need %d trace data destinations!", cmd_ctx->cores_num);
		cmd_ctx->running = 0;
		esp32_apptrace_cmd_ctx_cleanup(cmd_ctx);
		return ERROR_FAIL;
	}
	struct esp32_apptrace_cmd_data *cmd_data = malloc(sizeof(struct esp32_apptrace_cmd_data));
	if (!cmd_data) {
		LOG_ERROR("Failed to alloc cmd data!");
		cmd_ctx->running = 0;
		esp32_apptrace_cmd_ctx_cleanup(cmd_ctx);
		return ERROR_FAIL;
	}
	memset(cmd_data, 0, sizeof(struct esp32_apptrace_cmd_data));
	cmd_ctx->cmd_priv = cmd_data;

	/*outfile1 [outfile2] [poll_period [trace_size [stop_tmo [wait4halt [skip_size]]]]] */
	cmd_data->max_len = (uint32_t)-1;
	cmd_data->poll_period = 1 /*ms*/;
	int dests_num = esp32_apptrace_dest_init(cmd_data->data_dests,
		argv,
		cmd_ctx->mode == ESP_APPTRACE_CMD_MODE_SYSVIEW ? cmd_ctx->cores_num : 1);
	if (cmd_ctx->mode == ESP_APPTRACE_CMD_MODE_SYSVIEW && dests_num < cmd_ctx->cores_num) {
		LOG_ERROR("Not enough args! Need %d trace data destinations!", cmd_ctx->cores_num);
		res = ERROR_FAIL;
		goto on_error;
	}
	if (argc > dests_num) {
		char *end;
		cmd_data->poll_period = strtoul(argv[dests_num+0], &end, 10);
		ESP32_APPTRACE_CMD_NUM_ARG_CHECK(cmd_data->poll_period, argv[dests_num+0], end);
		if (argc > dests_num+1) {
			cmd_data->max_len = strtoul(argv[dests_num+1], &end, 10);
			ESP32_APPTRACE_CMD_NUM_ARG_CHECK(cmd_data->max_len, argv[dests_num+1], end);
			if (argc > dests_num+2) {
				int32_t tmo = strtol(argv[dests_num+2], &end, 10);
				ESP32_APPTRACE_CMD_NUM_ARG_CHECK(tmo, argv[dests_num+2], end);
				cmd_ctx->stop_tmo = 1.0*tmo;
				if (argc > dests_num+3) {
					cmd_data->wait4halt = strtoul(argv[dests_num+3], &end, 10);
					ESP32_APPTRACE_CMD_NUM_ARG_CHECK(cmd_data->wait4halt,
						argv[dests_num+3], end);
					if (argc > dests_num+4) {
						cmd_data->skip_len = strtoul(argv[dests_num+4],
							&end,
							10);
						ESP32_APPTRACE_CMD_NUM_ARG_CHECK(cmd_data->skip_len,
							argv[dests_num+4],
							end);
					}
				}
			}
		}
	}
	LOG_USER(
		"App trace params: from %d cores, size %u bytes, stop_tmo %g s, poll period %u ms, wait_rst %d, skip %u bytes",
		cmd_ctx->cores_num,
		cmd_data->max_len,
		cmd_ctx->stop_tmo,
		cmd_data->poll_period,
		cmd_data->wait4halt,
		cmd_data->skip_len);

	return ERROR_OK;
on_error:
	LOG_ERROR("Not enough args! Need %d trace data destinations!", cmd_ctx->cores_num);
	free(cmd_data);
	cmd_ctx->running = 0;
	esp32_apptrace_cmd_ctx_cleanup(cmd_ctx);
	return res;
}

static int esp32_apptrace_cmd_cleanup(struct esp32_apptrace_cmd_ctx *cmd_ctx)
{
	struct esp32_apptrace_cmd_data *cmd_data = cmd_ctx->cmd_priv;

	esp32_apptrace_dest_cleanup(cmd_data->data_dests, cmd_ctx->cores_num);
	free(cmd_data);
	esp32_apptrace_cmd_ctx_cleanup(cmd_ctx);
	memset(cmd_ctx, 0, sizeof(*cmd_ctx));
	return ERROR_OK;
}

static void esp32_apptrace_print_stats(struct esp32_apptrace_cmd_ctx *ctx)
{
	struct esp32_apptrace_cmd_data *cmd_data = ctx->cmd_priv;
	uint32_t trace_sz = 0;

	if (cmd_data)
		trace_sz = ctx->tot_len >
			cmd_data->skip_len ? ctx->tot_len - cmd_data->skip_len : 0;
	LOG_USER("Tracing is %s. Size is %u of %u @ %f (%f) KB/s",
		!ctx->running ? "STOPPED" : "RUNNING",
		trace_sz,
		cmd_data ? cmd_data->max_len : 0,
		duration_kbps(&ctx->read_time, ctx->tot_len),
		duration_kbps(&ctx->read_time, ctx->raw_tot_len));
	LOG_USER("Data: blocks incomplete %u, lost bytes: %u",
		ctx->stats.incompl_blocks,
		ctx->stats.lost_bytes);
#if ESP_APPTRACE_TIME_STATS_ENABLE
	LOG_USER("TRAX: block read time [%f..%f] ms",
		1000*ctx->stats.min_blk_read_time,
		1000*ctx->stats.max_blk_read_time);
	LOG_USER("TRAX: block proc time [%f..%f] ms",
		1000*ctx->stats.min_blk_proc_time,
		1000*ctx->stats.max_blk_proc_time);
#endif
}

static int esp32_apptrace_wait4halt(struct esp32_apptrace_cmd_ctx *ctx, struct target *target)
{
	int halted = 0;

	LOG_USER("Wait for halt...");
	while (!shutdown_openocd) {
		int res = target_poll(target);
		if (res != ERROR_OK) {
			LOG_ERROR("Failed to poll target (%d)!", res);
			return res;
		}
		if (target->state == TARGET_HALTED) {
			LOG_USER("%s: HALTED", target->cmd_name);
			break;
		}
		if (halted)
			break;
		alive_sleep(500);
	}
	return ERROR_OK;
}

static int esp32_apptrace_safe_halt_targets(struct esp32_apptrace_cmd_ctx *ctx,
	struct target *target,
	struct esp32_apptrace_target_state *targets)
{
	int res = ERROR_OK;

	memset(targets, 0, ctx->cores_num*sizeof(struct esp32_apptrace_target_state));
	/* halt all CPUs */
	LOG_DEBUG("Halt all targets!");
	if (target->state != TARGET_HALTED) {
		res = target_halt(target);
		if (res != ERROR_OK) {
			LOG_ERROR("Failed to halt target (%d)!", res);
			return res;
		}
		res = target_wait_state(target, TARGET_HALTED, ESP32_APPTRACE_TGT_STATE_TMO);
		if (res != ERROR_OK) {
			LOG_ERROR("Failed to wait halt target %s / %d (%d)!",
				target_name(target),
				target->state,
				res);
			return res;
		}
	}
	/* read current block statuses from CPUs */
	LOG_DEBUG("Read current block statuses");
	for (int k = 0; k < ctx->cores_num; k++) {
		uint32_t stat;
		res = esp_xtensa_apptrace_status_reg_read(ctx->cpus[k], &stat);
		if (res != ERROR_OK) {
			LOG_ERROR("Failed to read trace status (%d)!", res);
			return res;
		}
		/* check if some CPU stopped inside TRAX reg update critical section */
		if (stat) {
			res = esp_xtensa_swdbg_activate(ctx->cpus[k], 1	/*enable*/);
			if (res != ERROR_OK) {
				LOG_ERROR("Failed to activate SW debug (%d)!", res);
				return res;
			}
			uint32_t bp_addr = stat;
			res = breakpoint_add(target, bp_addr, 1, BKPT_HARD);
			if (res != ERROR_OK) {
				LOG_ERROR("Failed to set breakpoint (%d)!", res);
				return res;
			}
			while (stat) {
				/* allow this CPU to leave ERI write critical section */
				res = target_resume(target, 1, 0, 1, 0);
				if (res != ERROR_OK) {
					LOG_ERROR("Failed to resume target (%d)!", res);
					breakpoint_remove(target, bp_addr);
					return res;
				}
				/* wait for CPU to be halted on BP */
				enum target_debug_reason debug_reason = DBG_REASON_UNDEFINED;
				while (debug_reason != DBG_REASON_BREAKPOINT) {
					res = target_wait_state(target,
						TARGET_HALTED,
						ESP32_APPTRACE_TGT_STATE_TMO);
					if (res != ERROR_OK) {
						LOG_ERROR("Failed to wait halt on bp (%d)!", res);
						breakpoint_remove(target, bp_addr);
						return res;
					}
					debug_reason = target->debug_reason;
				}
				res = esp_xtensa_apptrace_status_reg_read(ctx->cpus[k], &stat);
				if (res != ERROR_OK) {
					LOG_ERROR("Failed to read trace status (%d)!", res);
					breakpoint_remove(target, bp_addr);
					return res;
				}
			}
			breakpoint_remove(target, bp_addr);
			res = esp_xtensa_swdbg_activate(ctx->cpus[k], 0	/*disable*/);
			if (res != ERROR_OK) {
				LOG_ERROR("Failed to de-activate SW debug (%d)!", res);
				return res;
			}
		}
		res = esp_xtensa_apptrace_data_len_read(ctx->cpus[k],
			&targets[k].block_id,
			&targets[k].data_len);
		if (res != ERROR_OK) {
			LOG_ERROR("Failed to read trace status (%d)!", res);
			return res;
		}
	}

	return ERROR_OK;
}

static int esp32_apptrace_connect_targets(struct esp32_apptrace_cmd_ctx *ctx,
	struct target *target,
	bool conn,
	bool resume_target)
{
	int res = ERROR_OK;
	struct esp32_apptrace_target_state target_to_connect[ESP_APPTRACE_MAX_CORES_NUM];

	if (conn)
		LOG_USER("Connect targets...");
	else
		LOG_USER("Disconnect targets...");

	res = esp32_apptrace_safe_halt_targets(ctx, target, target_to_connect);
	if (res != ERROR_OK) {
		LOG_ERROR("Failed to halt targets (%d)!", res);
		return res;
	}
	if (ctx->cores_num > 1) {
		/* set block ids to the highest value */
		uint32_t max_id = 0;
		for (int k = 0; k < ctx->cores_num; k++) {
			if (target_to_connect[k].block_id > max_id)
				max_id = target_to_connect[k].block_id;
		}
		for (int k = 0; k < ctx->cores_num; k++)
			target_to_connect[k].block_id = max_id;
	}
	for (int k = 0; k < ctx->cores_num; k++) {
		/* update host connected status */
		res = esp_xtensa_apptrace_ctrl_reg_write(ctx->cpus[k],
			target_to_connect[k].block_id,
			0 /*ack target data*/,
			conn,
			0 /*no host data*/);
		if (res != ERROR_OK) {
			LOG_ERROR("Failed to read trace status (%d)!", res);
			return res;
		}
	}
	if (resume_target) {
		LOG_DEBUG("Resume targets");
		res = target_resume(target, 1, 0, 1, 0);
		if (res != ERROR_OK) {
			LOG_ERROR("Failed to resume target (%d)!", res);
			return res;
		}
	}
	if (conn)
		LOG_INFO("Targets connected.");
	else
		LOG_INFO("Targets disconnected.");
	return res;
}

static int esp_sysview_write_trace_header(struct esp32_apptrace_cmd_ctx *ctx)
{
	struct esp32_apptrace_cmd_data *cmd_data = ctx->cmd_priv;

	char hdr_str[] = ";\n"
		"; Version     SEGGER SystemViewer V2.42\n"
		"; Author      Espressif Inc\n"
		";\n";
	int hdr_len = strlen(hdr_str);
	for (int i = 0; i < ctx->cores_num; i++) {
		int res = cmd_data->data_dests[i].write(cmd_data->data_dests[i].priv,
			(uint8_t *)hdr_str,
			hdr_len);
		if (res != ERROR_OK) {
			LOG_ERROR("SEGGER: Failed to write %u bytes to dest %d!", hdr_len, i);
			return ERROR_FAIL;
		}
	}
	return ERROR_OK;
}

/* this function must be called after connecting to targets */
static int esp_sysview_start(struct esp32_apptrace_cmd_ctx *ctx)
{
	uint8_t cmds[] = {SEGGER_SYSVIEW_COMMAND_ID_START};
	uint32_t fired_target_num = 0;
	struct esp32_apptrace_target_state target_state[ESP_APPTRACE_MAX_CORES_NUM];
	struct esp32_apptrace_cmd_data *cmd_data = (struct esp32_apptrace_cmd_data *)ctx->cmd_priv;

	/* get current block id */
	int res = esp32_apptrace_get_data_info(ctx, target_state, &fired_target_num);
	if (res != ERROR_OK) {
		LOG_ERROR("SEGGER: Failed to read target data info!");
		return res;
	}
	if (fired_target_num == (uint32_t)-1) {
		/* it can happen that there is no pending target data, but block was switched
		 * in this case block_ids on both CPUs are equal, so select the first one */
		fired_target_num = 0;
	}
	/* start tracing */
	res =
		esp_xtensa_sysview_cmds_queue(ctx->cpus[fired_target_num], cmds,
		sizeof(cmds)/sizeof(cmds[0]),
		target_state[fired_target_num].block_id);
	if (res != ERROR_OK) {
		LOG_ERROR("SEGGER: Failed to start tracing!");
		return res;
	}
	cmd_data->sv_trace_running = 1;
	return res;
}

static int esp_sysview_stop(struct esp32_apptrace_cmd_ctx *ctx, struct target *target)
{
	uint32_t old_block_id, fired_target_num = 0, empty_target_num = 0;
	struct esp32_apptrace_target_state target_state[ESP_APPTRACE_MAX_CORES_NUM];
	struct esp32_apptrace_cmd_data *cmd_data = (struct esp32_apptrace_cmd_data *)ctx->cmd_priv;
	uint8_t cmds[] = {SEGGER_SYSVIEW_COMMAND_ID_STOP};
	struct duration wait_time;

	struct esp32_apptrace_block *block = esp32_apptrace_free_block_get(ctx);
	if (!block) {
		LOG_ERROR("Failed to get free block for data on (%s)!",
			target_name(ctx->cpus[fired_target_num]));
		return ERROR_FAIL;
	}

	/* halt all CPUs, otherwise it can happen that there is no target data and
	 * while we are queueing commands on one CPU another CPU switches TRAX block */
	int res = esp32_apptrace_safe_halt_targets(ctx, target, target_state);
	if (res != ERROR_OK) {
		LOG_ERROR("SEGGER: Failed to halt targets (%d)!", res);
		return res;
	}
	/* it can happen that there is no pending target data
	 * in this case block_ids on both CPUs are equal, so the first one will be selected */
	for (int k = 0; k < ctx->cores_num; k++) {
		if (target_state[k].data_len) {
			fired_target_num = k;
			break;
		}
	}
	if (target_state[fired_target_num].data_len) {
		/* read pending data without ack, they will be acked when stop command is queued */
		res =
			esp_xtensa_apptrace_data_read(ctx->cpus[fired_target_num],
			target_state[fired_target_num].data_len,
			block->data,
			target_state[fired_target_num].block_id,
			0 /*no ack target data*/);
		if (res != ERROR_OK) {
			LOG_ERROR("SEGGER: Failed to read data on (%s)!",
				target_name(ctx->cpus[fired_target_num]));
			return res;
		} else {
			/* process data */
			block->data_len = target_state[fired_target_num].data_len;
			res = esp32_apptrace_handle_trace_block(ctx, block);
			if (res != ERROR_OK) {
				LOG_ERROR("Failed to process trace block %d bytes!",
					block->data_len);
				return res;
			}
		}
	}
	/* stop tracing and ack target data */
	res =
		esp_xtensa_sysview_cmds_queue(ctx->cpus[fired_target_num], cmds,
		sizeof(cmds)/sizeof(cmds[0]), target_state[fired_target_num].block_id);
	if (res != ERROR_OK) {
		LOG_ERROR("SEGGER: Failed to stop tracing!");
		return res;
	}
	if (ctx->cores_num > 1) {
		empty_target_num = fired_target_num ? 0 : 1;
		/* ack target data on another CPU */
		res =
			esp_xtensa_apptrace_ctrl_reg_write(ctx->cpus[empty_target_num],
			target_state[fired_target_num].block_id,
			0 /*target data ack*/,
			1 /*host connected*/,
			0 /*no host data*/);
		if (res != ERROR_OK) {
			LOG_ERROR("SEGGER: Failed to ack data on target '%s' (%d)!",
				target_name(ctx->cpus[empty_target_num]), res);
			return res;
		}
	}
	/* resume targets to allow command processing */
	LOG_INFO("Resume targets");
	res = target_resume(target, 1, 0, 1, 0);
	if (res != ERROR_OK) {
		LOG_ERROR("SEGGER: Failed to resume target '%s' (%d)!", target_name(target), res);
		return res;
	}
	/* wait for block switch (command sent), so we can disconnect from targets */
	old_block_id = target_state[fired_target_num].block_id;
	if (duration_start(&wait_time) != 0) {
		LOG_ERROR("Failed to start trace stop timeout measurement!");
		return ERROR_FAIL;
	}
	/* we are waiting for the last data from TRAX block and also there can be data in the pended
	 * data buffer */
	/* so we are expecting two TRX block switches at most or stopping due to timeout */
	while (cmd_data->sv_trace_running) {
		res = esp32_apptrace_get_data_info(ctx, target_state, &fired_target_num);
		if (res != ERROR_OK) {
			LOG_ERROR("SEGGER: Failed to read targets data info!");
			return res;
		}
		if (fired_target_num == (uint32_t)-1) {
			/* it can happen that there is no pending (last) target data, but block was
			 * switched */
			/* in this case block_ids on both CPUs are equal, so select the first one */
			fired_target_num = 0;
		}
		if (target_state[fired_target_num].block_id != old_block_id) {
			if (target_state[fired_target_num].data_len) {
				/* read last data and ack them */
				res = esp_xtensa_apptrace_data_read(ctx->cpus[fired_target_num],
					target_state[fired_target_num].data_len,
					block->data,
					target_state[fired_target_num].block_id,
					1 /*ack target data*/);
				if (res != ERROR_OK)
					LOG_ERROR("SEGGER: Failed to read last data on (%s)!",
						target_name(ctx->cpus[fired_target_num]));
				else {
					if (ctx->cores_num > 1) {
						/* ack target data on another CPU */
						empty_target_num = fired_target_num ? 0 : 1;
						res =
							esp_xtensa_apptrace_ctrl_reg_write(
							ctx->cpus[empty_target_num],
							target_state[fired_target_num].block_id,
							0 /*all read*/,
							1 /*host connected*/,
							0 /*no host data*/);
						if (res != ERROR_OK) {
							LOG_ERROR(
								"SEGGER: Failed to ack data on target '%s' (%d)!",
								target_name(ctx->cpus[
										empty_target_num]),
								res);
							return res;
						}
					}
					/* process data */
					block->data_len = target_state[fired_target_num].data_len;
					res = esp32_apptrace_handle_trace_block(ctx, block);
					if (res != ERROR_OK) {
						LOG_ERROR("Failed to process trace block %d bytes!",
							block->data_len);
						return res;
					}
				}
				old_block_id = target_state[fired_target_num].block_id;
			}
		}
		if (duration_measure(&wait_time) != 0) {
			LOG_ERROR("Failed to start trace stop timeout measurement!");
			return ERROR_FAIL;
		}
		const float stop_tmo = LOG_LEVEL_IS(LOG_LVL_DEBUG) ? 30.0 : 0.5;
		if (duration_elapsed(&wait_time) >= stop_tmo) {
			LOG_INFO("Stop waiting for the last data due to timeout.");
			break;
		}
	}
	return res;
}

static uint32_t esp32_apptrace_usr_block_check(struct esp32_apptrace_cmd_ctx *ctx,
	struct esp_xtensa_apptrace_target2host_hdr *hdr)
{
	uint32_t wr_len = 0, usr_len = 0;
	if (ctx->mode == ESP_APPTRACE_CMD_MODE_SYSVIEW) {
		wr_len = ESP32_SYSVIEW_USER_BLOCK_LEN(hdr->sys_view.wr_sz);
		usr_len = ESP32_SYSVIEW_USER_BLOCK_LEN(hdr->sys_view.block_sz);
	} else {
		wr_len = ESP32_APPTRACE_USER_BLOCK_LEN(hdr->gen.wr_sz);
		usr_len = ESP32_APPTRACE_USER_BLOCK_LEN(hdr->gen.block_sz);
	}
	if (usr_len != wr_len) {
		LOG_ERROR("Incomplete block sz %u, wr %u", usr_len, wr_len);
		ctx->stats.incompl_blocks++;
		ctx->stats.lost_bytes += usr_len - wr_len;
	}
	return usr_len;
}

static int esp32_apptrace_get_data_info(struct esp32_apptrace_cmd_ctx *ctx,
	struct esp32_apptrace_target_state *target_state,
	uint32_t *fired_target_num)
{
	if (fired_target_num)
		*fired_target_num = (uint32_t)-1;

	for (int i = 0; i < ctx->cores_num; i++) {
		int res = esp_xtensa_apptrace_data_len_read(ctx->cpus[i],
			&target_state[i].block_id,
			&target_state[i].data_len);
		if (res != ERROR_OK) {
			LOG_ERROR("Failed to read data len on (%s)!", target_name(ctx->cpus[i]));
			return res;
		}
		if (target_state[i].data_len) {
			LOG_DEBUG("Block %d, len %d bytes on fired target (%s)!",
				target_state[i].block_id, target_state[i].data_len,
				target_name(ctx->cpus[i]));
			if (fired_target_num)
				*fired_target_num = (uint32_t)i;
			break;
		}
	}
	return ERROR_OK;
}

static int esp32_apptrace_process_data(struct esp32_apptrace_cmd_ctx *ctx,
	int core_id,
	uint8_t *data,
	uint32_t data_len)
{
	struct esp32_apptrace_cmd_data *cmd_data = ctx->cmd_priv;

	LOG_DEBUG("Got block %d bytes [%x %x...%x %x]", data_len, data[12], data[13],
		data[data_len-2], data[data_len-1]);
	if (ctx->tot_len + data_len > cmd_data->skip_len) {
		uint32_t wr_idx = 0, wr_chunk_len = data_len;
		if (ctx->tot_len < cmd_data->skip_len) {
			wr_chunk_len = (ctx->tot_len + wr_chunk_len) - cmd_data->skip_len;
			wr_idx = cmd_data->skip_len - ctx->tot_len;
		}
		if (ctx->tot_len + wr_chunk_len > cmd_data->max_len)
			wr_chunk_len -= (ctx->tot_len + wr_chunk_len - cmd_data->skip_len) -
				cmd_data->max_len;
		if (wr_chunk_len > 0) {
			int res = cmd_data->data_dests[0].write(cmd_data->data_dests[0].priv,
				data + wr_idx,
				wr_chunk_len);
			if (res != ERROR_OK) {
				LOG_ERROR("Failed to write %u bytes to dest 0!", data_len);
				return res;
			}
		}
		ctx->tot_len += wr_chunk_len;
	} else
		ctx->tot_len += data_len;
	LOG_USER("%u ", ctx->tot_len);
	/* check for stop condition */
	if ((ctx->tot_len > cmd_data->skip_len) &&
		(ctx->tot_len - cmd_data->skip_len >= cmd_data->max_len)) {
		ctx->running = 0;
		if (duration_measure(&ctx->read_time) != 0) {
			LOG_ERROR("Failed to stop trace read time measure!");
			return ERROR_FAIL;
		}
	}
	return ERROR_OK;
}

static uint32_t esp_sysview_decode_u32(uint8_t **ptr)
{
	uint32_t val = 0;
	for (int k = 0;; k++, (*ptr)++) {
		if (**ptr & 0x80)
			val |= (uint32_t)(**ptr & ~0x80) << 7*k;
		else {
			val |= (uint32_t)**ptr << 7*k;
			(*ptr)++;
			break;
		}
	}
	return val;
}

static uint16_t esp_sysview_decode_plen(uint8_t **ptr)
{
	uint16_t payload_len = 0;
	uint8_t *p = *ptr;
	/* here pkt points to encoded payload length */
	if (*p & 0x80) {
		payload_len = *(p + 1);	/* higher part */
		payload_len = (payload_len << 7) | (*p & ~0x80);/* lower 7 bits */
		p += 2;	/* payload len (2 bytes) */
	} else {
		payload_len = *p;
		p++;	/* payload len (1 byte) */
	}
	*ptr = p;

	return payload_len;
}

static uint16_t esp_sysview_get_predef_payload_len(uint16_t id, uint8_t *pkt)
{
	uint16_t len;
	uint8_t *ptr = pkt;

	switch (id) {
		case SYSVIEW_EVTID_OVERFLOW:
		case SYSVIEW_EVTID_ISR_ENTER:
		case SYSVIEW_EVTID_TASK_START_EXEC:
		case SYSVIEW_EVTID_TASK_START_READY:
		case SYSVIEW_EVTID_TASK_CREATE:
		case SYSVIEW_EVTID_SYSTIME_CYCLES:
		case SYSVIEW_EVTID_USER_START:
		case SYSVIEW_EVTID_USER_STOP:
		case SYSVIEW_EVTID_TIMER_ENTER:
			/*ENCODE_U32 */
			esp_sysview_decode_u32(&ptr);
			len = ptr - pkt;
			break;
		case SYSVIEW_EVTID_TASK_STOP_READY:
		case SYSVIEW_EVTID_SYSTIME_US:
			/*2*ENCODE_U32 */
			esp_sysview_decode_u32(&ptr);
			esp_sysview_decode_u32(&ptr);
			len = ptr - pkt;
			break;
		case SYSVIEW_EVTID_SYSDESC:
			/*str(128 + 1) */
			len = *ptr + 1;
			break;
		case SYSVIEW_EVTID_TASK_INFO:
		case SYSVIEW_EVTID_MODULEDESC:
			/*2*ENCODE_U32 + str */
			esp_sysview_decode_u32(&ptr);
			esp_sysview_decode_u32(&ptr);
			/* TODO: add support for strings longer then 255 bytes */
			len = ptr - pkt + *ptr + 1;
			break;
		case SYSVIEW_EVTID_STACK_INFO:
			/*4*ENCODE_U32 */
			esp_sysview_decode_u32(&ptr);
			esp_sysview_decode_u32(&ptr);
			esp_sysview_decode_u32(&ptr);
			esp_sysview_decode_u32(&ptr);
			len = ptr - pkt;
			break;
		case SYSVIEW_EVTID_ISR_EXIT:
		case SYSVIEW_EVTID_TASK_STOP_EXEC:
		case SYSVIEW_EVTID_TRACE_START:
		case SYSVIEW_EVTID_TRACE_STOP:
		case SYSVIEW_EVTID_IDLE:
		case SYSVIEW_EVTID_ISR_TO_SCHEDULER:
		case SYSVIEW_EVTID_TIMER_EXIT:
			len = 0;
			break;

		/*case SYSVIEW_EVTID_NOP: */
		default:
			LOG_ERROR("SEGGER: Unsupported predef event %d!", id);
			len = 0;
	}
	return len;
}

static uint16_t esp_sysview_parse_packet(uint8_t *pkt_buf,
	uint32_t *pkt_len,
	int *pkt_core_id,
	uint32_t *delta,
	uint32_t *delta_len)
{
	uint8_t *pkt = pkt_buf;
	uint16_t event_id = 0, payload_len = 0;

	*pkt_core_id = 0;
	*pkt_len = 0;
	/* 1-2 byte of message type, 0-2  byte of payload length, payload, 1-5 bytes of timestamp.
	 * */
	if (*pkt & 0x80) {
		if (*(pkt + 1) & (1 << 6)) {
			*(pkt + 1) &= ~(1 << 6);/* clear core_id bit */
			*pkt_core_id = 1;
		}
		event_id = *(pkt + 1);	/* higher part */
		event_id = (event_id << 7) | (*pkt & ~0x80);	/* lower 7 bits */
		pkt += 2;	/* event_id (2 bytes) */
		/* here pkt points to encoded payload length */
		payload_len = esp_sysview_decode_plen(&pkt);
	} else {
		if (*pkt & (1 << 6)) {
			*pkt &= ~(1 << 6);	/* clear core_id bit */
			*pkt_core_id = 1;
		}
		/* event_id (1 byte) */
		event_id = *pkt;
		pkt++;
		if (event_id < 24)
			payload_len = esp_sysview_get_predef_payload_len(event_id, pkt);
		else
			payload_len = esp_sysview_decode_plen(&pkt);
	}
	pkt += payload_len;
	uint8_t *delta_start = pkt;
	*delta = esp_sysview_decode_u32(&pkt);
	*delta_len = pkt - delta_start;
	*pkt_len = pkt - pkt_buf;
	LOG_DEBUG("SEGGER: evt %d len %d plen %d dlen %d",
		event_id,
		*pkt_len,
		payload_len,
		*delta_len);
	return event_id;
}

static int esp32_sysview_process_data(struct esp32_apptrace_cmd_ctx *ctx,
	int core_id,
	uint8_t *data,
	uint32_t data_len)
{
	struct esp32_apptrace_cmd_data *cmd_data = ctx->cmd_priv;

	LOG_DEBUG("SEGGER: Read from target %d bytes [%x %x %x %x]",
		data_len,
		data[0],
		data[1],
		data[2],
		data[3]);
	int res;
	uint32_t processed = 0;
	if (core_id >= ctx->cores_num) {
		LOG_ERROR("SEGGER: Invalid core id %d in user block!", core_id);
		return ERROR_FAIL;
	}
	if (ctx->tot_len == 0) {
		/* handle sync seq */
		if (data_len < SYSVIEW_SYNC_LEN) {
			LOG_ERROR("SEGGER: Invalid init seq len %d!", data_len);
			return ERROR_FAIL;
		}
		LOG_DEBUG("SEGGER: Process %d sync bytes", SYSVIEW_SYNC_LEN);
		uint8_t sync_seq[SYSVIEW_SYNC_LEN] =
		{0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};
		if (memcmp(data, sync_seq, SYSVIEW_SYNC_LEN) != 0) {
			LOG_ERROR("SEGGER: Invalid init seq [%x %x %x %x %x %x %x %x %x %x]",
				data[0], data[1], data[2], data[3], data[4], data[5], data[6],
				data[7], data[8], data[9]);
			return ERROR_FAIL;
		}
		res = cmd_data->data_dests[core_id].write(cmd_data->data_dests[core_id].priv,
			data,
			SYSVIEW_SYNC_LEN);
		if (res != ERROR_OK) {
			LOG_ERROR("SEGGER: Failed to write %u sync bytes to dest %d!",
				SYSVIEW_SYNC_LEN,
				core_id);
			return res;
		}
		if (ctx->cores_num > 1) {
			res =
				cmd_data->data_dests[core_id ? 0 : 1].write(cmd_data->data_dests[
					core_id
					? 0 : 1].priv, data, SYSVIEW_SYNC_LEN);
			if (res != ERROR_OK) {
				LOG_ERROR("SEGGER: Failed to write %u sync bytes to dest %d!",
					SYSVIEW_SYNC_LEN,
					core_id ? 0 : 1);
				return res;
			}
		}
		ctx->tot_len += SYSVIEW_SYNC_LEN;
		processed += SYSVIEW_SYNC_LEN;
	}
	while (processed < data_len) {
		int pkt_core_id, pkt_core_changed = 0;
		uint32_t delta_len = 0, new_delta_len = 0;
		uint8_t new_delta_buf[10];
		uint32_t pkt_len = 0, delta = 0, wr_len;
		uint16_t event_id = esp_sysview_parse_packet(data + processed,
			&pkt_len,
			&pkt_core_id,
			&delta,
			&delta_len);
		LOG_DEBUG("SEGGER: Process packet %d id %d bytes [%x %x %x %x]",
			event_id,
			pkt_len,
			data[processed+0],
			data[processed+1],
			data[processed+2],
			data[processed+3]);
		wr_len = pkt_len;
		if (ctx->cores_num > 1) {
			if (cmd_data->sv_last_core_id == pkt_core_id) {
				/* if this packet is for the same core as the prev one acc delta and
				 * write packet unmodified */
				cmd_data->sv_acc_time_delta += delta;
			} else {
				/* if this packet is for another core then prev one set acc delta to
				 * the packet's delta */
				uint8_t *delta_ptr = new_delta_buf;
				SYSVIEW_ENCODE_U32(delta_ptr, delta + cmd_data->sv_acc_time_delta);
				cmd_data->sv_acc_time_delta = delta;
				wr_len -= delta_len;
				new_delta_len = delta_ptr - new_delta_buf;
				pkt_core_changed = 1;
			}
			cmd_data->sv_last_core_id = pkt_core_id;
		}
		res = cmd_data->data_dests[pkt_core_id].write(
			cmd_data->data_dests[pkt_core_id].priv,
			data + processed,
			wr_len);
		if (res != ERROR_OK) {
			LOG_ERROR("SEGGER: Failed to write %u bytes to dest %d!", wr_len, core_id);
			return res;
		}
		if (new_delta_len) {
			/* write packet with modified delta */
			res =
				cmd_data->data_dests[pkt_core_id].write(
				cmd_data->data_dests[pkt_core_id].priv,
				new_delta_buf,
				new_delta_len);
			if (res != ERROR_OK) {
				LOG_ERROR("SEGGER: Failed to write %u bytes of delta to dest %d!",
					new_delta_len,
					core_id);
				return res;
			}
		}
		if (ctx->cores_num > 1) {
			/* handle other core dest */
			int other_core_id = pkt_core_id ? 0 : 1;
			switch (event_id) {
				/* messages below should be sent to trace destinations for all cores
				 * */
				case SYSVIEW_EVTID_TRACE_START:
				case SYSVIEW_EVTID_TRACE_STOP:
				case SYSVIEW_EVTID_SYSTIME_CYCLES:
				case SYSVIEW_EVTID_SYSTIME_US:
				case SYSVIEW_EVTID_SYSDESC:
				case SYSVIEW_EVTID_TASK_INFO:
				case SYSVIEW_EVTID_STACK_INFO:
				case SYSVIEW_EVTID_MODULEDESC:
				case SYSVIEW_EVTID_INIT:
				case SYSVIEW_EVTID_NUMMODULES:
				case SYSVIEW_EVTID_OVERFLOW:
				case SYSVIEW_EVTID_TASK_START_READY:
					/* if packet's source core has changed */
					wr_len = pkt_len;
					if (pkt_core_changed) {
						/* clone packet with unmodified delta */
						new_delta_len = 0;
					} else {
						/* clone packet with modified delta */
						uint8_t *delta_ptr = new_delta_buf;
						SYSVIEW_ENCODE_U32(delta_ptr,
						cmd_data->sv_acc_time_delta /*delta has been
									             * accumulated
									             * above*/);
						wr_len -= delta_len;
						new_delta_len = delta_ptr - new_delta_buf;
					}
					LOG_DEBUG(
					"SEGGER: Redirect %d bytes of event %d to dest %d",
					wr_len,
					event_id,
					other_core_id);
					res = cmd_data->data_dests[other_core_id].write(
					cmd_data->data_dests[other_core_id].priv,
					data + processed,
					wr_len);
					if (res != ERROR_OK) {
						LOG_ERROR(
						"SEGGER: Failed to write %u bytes to dest %d!",
						wr_len,
						other_core_id);
						return res;
					}
					if (new_delta_len) {
						/* write packet with modified delta */
						res = cmd_data->data_dests[other_core_id].write(
						cmd_data->data_dests[other_core_id].priv,
						new_delta_buf,
						new_delta_len);
						if (res != ERROR_OK) {
							LOG_ERROR(
							"SEGGER: Failed to write %u bytes of delta to dest %d!",
							new_delta_len,
							other_core_id);
							return res;
						}
					}
					/* messages above are cloned to trace files for both cores,
					 * so reset acc time delta, both files have actual delta
					 * info */
					cmd_data->sv_acc_time_delta = 0;
					break;
				default:
					break;
			}
		}
		if (event_id == SYSVIEW_EVTID_TRACE_STOP)
			cmd_data->sv_trace_running = 0;
		ctx->tot_len += pkt_len;
		processed += pkt_len;
	}
	LOG_USER("%u ", ctx->tot_len);
	/* check for stop condition */
	if ((ctx->tot_len > cmd_data->skip_len) &&
		(ctx->tot_len - cmd_data->skip_len >= cmd_data->max_len)) {
		ctx->running = 0;
		if (duration_measure(&ctx->read_time) != 0) {
			LOG_ERROR("Failed to stop trace read time measure!");
			return ERROR_FAIL;
		}
	}
	return ERROR_OK;
}

static int esp32_apptrace_handle_trace_block(struct esp32_apptrace_cmd_ctx *ctx,
	struct esp32_apptrace_block *block)
{
	uint32_t processed = 0;
	uint32_t hdr_sz = ctx->mode ==
		ESP_APPTRACE_CMD_MODE_SYSVIEW ? ESP32_SYSVIEW_USER_BLOCK_HDR_SZ :
		ESP32_APPTRACE_USER_BLOCK_HDR_SZ;
	LOG_DEBUG("Got block %d bytes", block->data_len);
	/* process user blocks one by one */
	while (processed < block->data_len) {
		LOG_DEBUG("Process usr block %d/%d", processed, block->data_len);
		struct esp_xtensa_apptrace_target2host_hdr tmp_hdr;
		memcpy(&tmp_hdr, block->data + processed, sizeof(tmp_hdr));
		struct esp_xtensa_apptrace_target2host_hdr *hdr = &tmp_hdr;
		/* process user block */
		uint32_t usr_len = esp32_apptrace_usr_block_check(ctx, hdr);
		int core_id;
		if (ctx->mode == ESP_APPTRACE_CMD_MODE_SYSVIEW)
			core_id = ESP32_SYSVIEW_USER_BLOCK_CORE(hdr->sys_view.block_sz);
		else
			core_id = ESP32_APPTRACE_USER_BLOCK_CORE(hdr->gen.block_sz);
		/* process user data */
		int res =
			ctx->process_data(ctx, core_id, block->data + processed + hdr_sz, usr_len);
		if (res != ERROR_OK) {
			LOG_ERROR("Failed to process %d bytes!", usr_len);
			return res;
		}
		processed += usr_len + hdr_sz;
	}
	return ERROR_OK;
}

static void *esp32_apptrace_data_processor(void *arg)
{
	long res = ERROR_OK;
	struct esp32_apptrace_cmd_ctx *ctx = (struct esp32_apptrace_cmd_ctx *)arg;

	while (ctx->running) {
		struct esp32_apptrace_block *block = esp32_apptrace_ready_block_get(ctx);
		if (!block)
			continue;
		res = esp32_apptrace_handle_trace_block(ctx, block);
		if (res != ERROR_OK) {
			ctx->running = 0;
			LOG_ERROR("Failed to process trace block %d bytes!", block->data_len);
			break;
		}
		res = esp32_apptrace_block_free(ctx, block);
		if (res != ERROR_OK) {
			ctx->running = 0;
			LOG_ERROR("Failed to free ready block!");
			break;
		}
	}

	return (void *)res;
}

static int esp32_apptrace_poll(void *priv)
{
	struct esp32_apptrace_cmd_ctx *ctx = (struct esp32_apptrace_cmd_ctx *)priv;
	int res;
	uint32_t fired_target_num = 0;
	struct esp32_apptrace_target_state target_state[ESP_APPTRACE_MAX_CORES_NUM];
#if ESP_APPTRACE_TIME_STATS_ENABLE
	struct duration blk_proc_time;
#endif

	if (!ctx->running)
		return ERROR_FAIL;

	/* check for data from target */
	res = esp32_apptrace_get_data_info(ctx, target_state, &fired_target_num);
	if (res != ERROR_OK) {
		ctx->running = 0;
		LOG_ERROR("Failed to read data len!");
		return res;
	}
	/* LOG_DEBUG("Block %d (%d bytes) on target (%s)!", target_state[0].block_id,
	 * target_state[0].data_len, target_name(ctx->cpus[0])); */
	if (fired_target_num == (uint32_t)-1) {
		/* no data has been received, but block could be switched due to the data transfered
		 * from host to target */
		if (ctx->cores_num > 1) {
			uint32_t max_block_id = 0, min_block_id = XTENSA_APPTRACE_BLOCK_ID_MAX;
			/* find maximum block ID and set the same ID in control reg for both blocks;
			 * */
			for (int i = 0; i < ctx->cores_num; i++) {
				if (max_block_id < target_state[i].block_id)
					max_block_id = target_state[i].block_id;
				if (min_block_id > target_state[i].block_id)
					min_block_id = target_state[i].block_id;
			}
			/* handle block ID overflow */
			if (max_block_id == XTENSA_APPTRACE_BLOCK_ID_MAX && min_block_id == 0)
				max_block_id = 0;
			for (int i = 0; i < ctx->cores_num; i++) {
				if (max_block_id != target_state[i].block_id) {
					LOG_DEBUG("Ack empty block %d on target (%s)!",
						max_block_id,
						target_name(ctx->cpus[i]));
					res = esp_xtensa_apptrace_ctrl_reg_write(ctx->cpus[i],
						max_block_id,
						0 /*all read*/,
						1 /*host connected*/,
						0 /*no host data*/);
					if (res != ERROR_OK) {
						ctx->running = 0;
						LOG_ERROR("Failed to ack empty data block on (%s)!",
							target_name(ctx->cpus[i]));
						return res;
					}
				}
			}
			ctx->last_blk_id = max_block_id;
		}
		if (ctx->stop_tmo != -1.0) {
			if (duration_measure(&ctx->idle_time) != 0) {
				ctx->running = 0;
				LOG_ERROR("Failed to measure idle time!");
				return ERROR_FAIL;
			}
			if (duration_elapsed(&ctx->idle_time) >= ctx->stop_tmo) {
				ctx->running = 0;
				LOG_ERROR("Data timeout!");
				return ERROR_FAIL;
			}
		}
		return ERROR_OK;/* no data */
	}
	/* sanity check */
	if (target_state[fired_target_num].data_len > ctx->trax_block_sz) {
		ctx->running = 0;
		LOG_ERROR("Too large block size %d!", target_state[fired_target_num].data_len);
		return ERROR_FAIL;
	}
	if (ctx->tot_len == 0) {
		if (duration_start(&ctx->read_time) != 0) {
			ctx->running = 0;
			LOG_ERROR("Failed to start trace read time measurement!");
			return ERROR_FAIL;
		}
	}
	struct esp32_apptrace_block *block = esp32_apptrace_free_block_get(ctx);
	if (!block) {
		ctx->running = 0;
		LOG_ERROR("Failed to get free block for data on (%s)!",
			target_name(ctx->cpus[fired_target_num]));
		return ERROR_FAIL;
	}
#if ESP_APPTRACE_TIME_STATS_ENABLE
	/* read block */
	if (duration_start(&blk_proc_time) != 0) {
		ctx->running = 0;
		LOG_ERROR("Failed to start block read time measurement!");
		return ERROR_FAIL;
	}
#endif
	res =
		esp_xtensa_apptrace_data_read(ctx->cpus[fired_target_num],
		target_state[fired_target_num].data_len,
		block->data,
		target_state[fired_target_num].block_id,
		1 /*ack target data*/);
	if (res != ERROR_OK) {
		ctx->running = 0;
		LOG_ERROR("Failed to read data on (%s)!", target_name(ctx->cpus[fired_target_num]));
		return res;
	}
	ctx->last_blk_id = target_state[fired_target_num].block_id;
#if ESP_APPTRACE_TIME_STATS_ENABLE
	if (duration_measure(&blk_proc_time) != 0) {
		ctx->running = 0;
		LOG_ERROR("Failed to measure block read time!");
		return ERROR_FAIL;
	}
	/* update stats */
	float brt = duration_elapsed(&blk_proc_time);
	if (brt > ctx->stats.max_blk_read_time)
		ctx->stats.max_blk_read_time = brt;
	if (brt < ctx->stats.min_blk_read_time)
		ctx->stats.min_blk_read_time = brt;

	if (duration_start(&blk_proc_time) != 0) {
		ctx->running = 0;
		LOG_ERROR("Failed to start block proc time measurement!");
		return ERROR_FAIL;
	}
#endif
	if (ctx->cores_num > 1) {
		res = esp_xtensa_apptrace_ctrl_reg_write(ctx->cpus[fired_target_num ? 0 : 1],
			ctx->last_blk_id,
			0 /*all read*/,
			1 /*host connected*/,
			0 /*no host data*/);
		if (res != ERROR_OK) {
			ctx->running = 0;
			LOG_ERROR("Failed to ack data on (%s)!",
				target_name(ctx->cpus[fired_target_num ? 0 : 1]));
			return res;
		}
		LOG_DEBUG("Ack block %d target (%s)!", ctx->last_blk_id,
			target_name(ctx->cpus[fired_target_num ? 0 : 1]));
	}
	ctx->raw_tot_len += target_state[fired_target_num].data_len;

	block->data_len = target_state[fired_target_num].data_len;
	if (ctx->mode == ESP_APPTRACE_CMD_MODE_SYNC) {
		res = esp32_apptrace_handle_trace_block(ctx, block);
		if (res != ERROR_OK) {
			ctx->running = 0;
			LOG_ERROR("Failed to process trace block %d bytes!", block->data_len);
			return res;
		}
		res = esp32_apptrace_block_free(ctx, block);
		if (res != ERROR_OK) {
			ctx->running = 0;
			LOG_ERROR("Failed to free ready block!");
			return res;
		}
	} else {
		res = esp32_apptrace_ready_block_put(ctx, block);
		if (res != ERROR_OK) {
			ctx->running = 0;
			LOG_ERROR("Failed to put ready block of data from (%s)!",
				target_name(ctx->cpus[fired_target_num]));
			return res;
		}
	}
	if (ctx->stop_tmo != -1.0) {
		/* start idle time measurement */
		if (duration_start(&ctx->idle_time) != 0) {
			ctx->running = 0;
			LOG_ERROR("Failed to start idle time measure!");
			return ERROR_FAIL;
		}
	}
#if ESP_APPTRACE_TIME_STATS_ENABLE
	if (duration_measure(&blk_proc_time) != 0) {
		ctx->running = 0;
		LOG_ERROR("Failed to stop block proc time measure!");
		return ERROR_FAIL;
	}
	/* update stats */
	float bt = duration_elapsed(&blk_proc_time);
	if (bt > ctx->stats.max_blk_proc_time)
		ctx->stats.max_blk_proc_time = bt;
	if (bt < ctx->stats.min_blk_proc_time)
		ctx->stats.min_blk_proc_time = bt;
#endif
	return ERROR_OK;
}

int esp32_cmd_apptrace_generic(struct target *target, int mode, const char **argv, int argc)
{
	static struct esp32_apptrace_cmd_ctx s_at_cmd_ctx;
	struct esp32_apptrace_cmd_data *cmd_data;
	int res = ERROR_OK;
	enum target_state old_state = target->state;

	if (argc < 1) {
		LOG_ERROR("Action missed!");
		return ERROR_FAIL;
	}

	if (strcmp(argv[0], "start") == 0) {
		/* init cmd context */
		res = esp32_apptrace_cmd_init(target, &s_at_cmd_ctx, mode, &argv[1], argc-1);
		if (res != ERROR_OK) {
			LOG_ERROR("Failed to init cmd ctx (%d)!", res);
			return res;
		}
		cmd_data = s_at_cmd_ctx.cmd_priv;
		if (mode == ESP_APPTRACE_CMD_MODE_SYSVIEW) {
			if (cmd_data->skip_len != 0) {
				LOG_ERROR("Data skipping not supported!");
				s_at_cmd_ctx.running = 0;
				esp32_apptrace_cmd_cleanup(&s_at_cmd_ctx);
				return ERROR_FAIL;
			}
			s_at_cmd_ctx.process_data = esp32_sysview_process_data;
			res = esp_sysview_write_trace_header(&s_at_cmd_ctx);
			if (res != ERROR_OK) {
				LOG_ERROR("Failed to write trace header (%d)!", res);
				s_at_cmd_ctx.running = 0;
				esp32_apptrace_cmd_cleanup(&s_at_cmd_ctx);
				return res;
			}
		} else
			s_at_cmd_ctx.process_data = esp32_apptrace_process_data;
		if (cmd_data->wait4halt) {
			res = esp32_apptrace_wait4halt(&s_at_cmd_ctx, target);
			if (res != ERROR_OK) {
				LOG_ERROR("Failed to wait for halt target (%d)!", res);
				s_at_cmd_ctx.running = 0;
				esp32_apptrace_cmd_cleanup(&s_at_cmd_ctx);
				return res;
			}
		}
		res = esp32_apptrace_connect_targets(&s_at_cmd_ctx,
			target,
			true,
			old_state == TARGET_RUNNING);
		if (res != ERROR_OK) {
			LOG_ERROR("Failed to connect to targets (%d)!", res);
			s_at_cmd_ctx.running = 0;
			esp32_apptrace_cmd_cleanup(&s_at_cmd_ctx);
			return res;
		}
		if (mode == ESP_APPTRACE_CMD_MODE_SYSVIEW) {
			/* start tracing */
			res = esp_sysview_start(&s_at_cmd_ctx);
			if (res != ERROR_OK) {
				LOG_ERROR("SEGGER: Failed to start tracing!");
				esp32_apptrace_connect_targets(&s_at_cmd_ctx,
					target,
					false,
					old_state == TARGET_RUNNING);
				s_at_cmd_ctx.running = 0;
				esp32_apptrace_cmd_cleanup(&s_at_cmd_ctx);
				return res;
			}
		}
		/* openocd timer_callback min period is 1 ms, if we need to poll target for trace
		 * data more frequently polling loop will be used */
		if (cmd_data->poll_period >= 1) {
			res = target_register_timer_callback(esp32_apptrace_poll,
				cmd_data->poll_period,
				1,
				&s_at_cmd_ctx);
			if (res != ERROR_OK) {
				LOG_ERROR("Failed to register target timer handler (%d)!", res);
				return res;
			}
		} else {/* //////////////// POLLING MODE ///////////////////////////
			 * check for exit signal and comand completion */
			while (!shutdown_openocd && s_at_cmd_ctx.running) {
				res = esp32_apptrace_poll(&s_at_cmd_ctx);
				if (res != ERROR_OK) {
					LOG_ERROR("Failed to poll target for trace data (%d)!",
						res);
					break;
				}
				/* let registered timer callbacks to run */
				target_call_timer_callbacks();
			}
			/* if we stopped due to user pressed CTRL+C */
			if (shutdown_openocd) {
				if (duration_measure(&s_at_cmd_ctx.read_time) != 0)
					LOG_ERROR("Failed to stop trace read time measurement!");
			}
			if (mode == ESP_APPTRACE_CMD_MODE_SYSVIEW) {
				/* stop tracing */
				res = esp_sysview_stop(&s_at_cmd_ctx, target);
				if (res != ERROR_OK)
					LOG_ERROR("SEGGER: Failed to stop tracing!");
			}
			if (s_at_cmd_ctx.running) {
				/* data processor is alive, so wait for all received blocks to be
				 * processed */
				res = esp32_apptrace_wait_tracing_finished(&s_at_cmd_ctx);
				if (res != ERROR_OK)
					LOG_ERROR("Failed to wait for pended blocks (%d)!", res);
			}
			res = esp32_apptrace_connect_targets(&s_at_cmd_ctx,
				target,
				false,
				old_state == TARGET_RUNNING);
			if (res != ERROR_OK)
				LOG_ERROR("Failed to disconnect targets (%d)!", res);
			esp32_apptrace_print_stats(&s_at_cmd_ctx);
			res = esp32_apptrace_cmd_cleanup(&s_at_cmd_ctx);
			if (res != ERROR_OK)
				LOG_ERROR("Failed to cleanup cmd ctx (%d)!", res);
		}
	} else if (strcmp(argv[0], "stop") == 0) {
		if (!s_at_cmd_ctx.running) {
			LOG_WARNING("Tracing is not running!");
			return ERROR_FAIL;
		}
		if (duration_measure(&s_at_cmd_ctx.read_time) != 0)
			LOG_ERROR("Failed to stop trace read time measurement!");
		res = target_unregister_timer_callback(esp32_apptrace_poll, &s_at_cmd_ctx);
		if (res != ERROR_OK) {
			LOG_ERROR("Failed to unregister target timer handler (%d)!", res);
			return res;
		}
		if (mode == ESP_APPTRACE_CMD_MODE_SYSVIEW) {
			/* stop tracing */
			res = esp_sysview_stop(&s_at_cmd_ctx, target);
			if (res != ERROR_OK)
				LOG_ERROR("SEGGER: Failed to stop tracing!");
		}
		/* data processor is alive, so wait for all received blocks to be processed */
		res = esp32_apptrace_wait_tracing_finished(&s_at_cmd_ctx);
		if (res != ERROR_OK)
			LOG_ERROR("Failed to wait for pended blocks (%d)!", res);
		res = esp32_apptrace_connect_targets(&s_at_cmd_ctx,
			target,
			false,
			old_state == TARGET_RUNNING);
		if (res != ERROR_OK)
			LOG_ERROR("Failed to disconnect targets (%d)!", res);
		esp32_apptrace_print_stats(&s_at_cmd_ctx);
		res = esp32_apptrace_cmd_cleanup(&s_at_cmd_ctx);
		if (res != ERROR_OK)
			LOG_ERROR("Failed to cleanup cmd ctx (%d)!", res);
	} else if (strcmp(argv[0], "status") == 0) {
		if (s_at_cmd_ctx.running && duration_measure(&s_at_cmd_ctx.read_time) != 0)
			LOG_ERROR("Failed to measure trace read time!");
		esp32_apptrace_print_stats(&s_at_cmd_ctx);
	} else if (strcmp(argv[0], "dump") == 0) {
		if (mode == ESP_APPTRACE_CMD_MODE_SYSVIEW) {
			LOG_ERROR("Not supported!");
			return ERROR_FAIL;
		}
		/* [dump outfile] - post-mortem dump without connection to targets */
		res = esp32_apptrace_cmd_init(target, &s_at_cmd_ctx, mode, &argv[1], argc-1);
		if (res != ERROR_OK) {
			LOG_ERROR("Failed to init cmd ctx (%d)!", res);
			return res;
		}
		s_at_cmd_ctx.stop_tmo = 0.01;	/* use small stop tmo */
		s_at_cmd_ctx.process_data = esp32_apptrace_process_data;
		/* check for exit signal and comand completion */
		while (!shutdown_openocd && s_at_cmd_ctx.running) {
			res = esp32_apptrace_poll(&s_at_cmd_ctx);
			if (res != ERROR_OK) {
				LOG_ERROR("Failed to poll target for trace data (%d)!", res);
				break;
			}
			/* let registered timer callbacks to run */
			target_call_timer_callbacks();
		}
		if (s_at_cmd_ctx.running) {
			/* data processor is alive, so wait for all received blocks to be processed
			 * */
			res = esp32_apptrace_wait_tracing_finished(&s_at_cmd_ctx);
			if (res != ERROR_OK)
				LOG_ERROR("Failed to wait for pended blocks (%d)!", res);
		}
		esp32_apptrace_print_stats(&s_at_cmd_ctx);
		res = esp32_apptrace_cmd_cleanup(&s_at_cmd_ctx);
		if (res != ERROR_OK)
			LOG_ERROR("Failed to cleanup cmd ctx (%d)!", res);
	} else
		LOG_ERROR("Invalid action '%s'!", argv[0]);

	return res;
}

COMMAND_HANDLER(esp32_cmd_apptrace)
{
	return esp32_cmd_apptrace_generic(get_current_target(CMD_CTX),
		ESP_APPTRACE_CMD_MODE_GEN,
		CMD_ARGV,
		CMD_ARGC);
}

COMMAND_HANDLER(esp32_cmd_sysview)
{
	return esp32_cmd_apptrace_generic(get_current_target(CMD_CTX),
		ESP_APPTRACE_CMD_MODE_SYSVIEW,
		CMD_ARGV,
		CMD_ARGC);
}

static int esp_gcov_cmd_init(struct target *target,
	struct esp32_apptrace_cmd_ctx *cmd_ctx,
	const char **argv,
	int argc)
{
	int res;

	res = esp32_apptrace_cmd_ctx_init(target, cmd_ctx, ESP_APPTRACE_CMD_MODE_SYNC);
	if (res)
		return res;
	cmd_ctx->stop_tmo = 3.0;
	cmd_ctx->process_data = esp_gcov_process_data;

	struct esp32_gcov_cmd_data *cmd_data = malloc(sizeof(struct esp32_gcov_cmd_data));
	if (!cmd_data) {
		LOG_ERROR("Failed to alloc cmd data!");
		esp32_apptrace_cmd_ctx_cleanup(cmd_ctx);
		return ERROR_FAIL;
	}
	memset(cmd_data, 0, sizeof(struct esp32_gcov_cmd_data));
	cmd_ctx->cmd_priv = cmd_data;

	if (argc > 0)
		cmd_data->wait4halt = strtoul(argv[0], NULL, 10);

	return ERROR_OK;
}

static int esp_gcov_cmd_cleanup(struct esp32_apptrace_cmd_ctx *cmd_ctx)
{
	struct esp32_gcov_cmd_data *cmd_data = cmd_ctx->cmd_priv;
	int res = ERROR_OK;

	for (int i = 0; i < ESP_GCOV_FILES_MAX_NUM; i++) {
		if (cmd_data->files[i] && fclose(cmd_data->files[i])) {
			LOG_ERROR("Failed to close file 0x%p (%d)!", cmd_data->files[i], errno);
			res = ERROR_FAIL;
		}
	}
	free(cmd_data);
	esp32_apptrace_cmd_ctx_cleanup(cmd_ctx);
	return res;
}

#ifdef _WIN32
#define DIR_SEPARATORS      ("\\/")
#define IS_DIR_SEPARATOR(c) (c == '\\' || c == '/')
#else
#define DIR_SEPARATORS      ("/")
#define IS_DIR_SEPARATOR(c) (c == '/')
#endif

static const char *esp_gcov_filename_alloc(const char *orig_fname)
{
	const char *gcov_prefix;
	size_t prefix_length;
	int strip = 0, orig_fname_len = strlen(orig_fname);

	if (orig_fname_len == 0)
		return NULL;
	/* Check if the level of dirs to strip off specified. */
	char *tmp = getenv("OPENOCD_GCOV_PREFIX_STRIP");
	if (tmp) {
		strip = atoi (tmp);
		/* Do not consider negative values. */
		if (strip < 0)
			strip = 0;
	}

	/* Get file name relocation prefix. Non-absolute values are ignored. */
	gcov_prefix = getenv("OPENOCD_GCOV_PREFIX");
	prefix_length = gcov_prefix ? strlen (gcov_prefix) : 0;

	/* Remove an unnecessary trailing '/' */
	if (prefix_length && IS_DIR_SEPARATOR(gcov_prefix[prefix_length - 1]))
		prefix_length--;

	/* If no prefix was specified and a prefix stip, then we assume relative.  */
	if (!prefix_length && strip) {
		gcov_prefix = ".";
		prefix_length = 1;
	}

	/* Allocate and initialize the filename scratch space.  */
	char *filename = (char *)malloc(prefix_length + orig_fname_len + 1);
	if (prefix_length)
		memcpy(filename, gcov_prefix, prefix_length);
	const char *striped_fname = orig_fname;
	while (strip--) {
		/* assume that file path starts with '/', it is generated by GCC
		 * under Windows we skip over drive letter,
		 * path can contain mixed Windows and Unix and */
		/* can look like
		 * `c:\esp\esp-idf\examples\system\gcov\build/esp-idf/main/CMakeFiles/__idf_main.dir/gcov_example.c.gcda` */
		tmp = strpbrk(striped_fname+1, DIR_SEPARATORS);
		if (tmp == NULL)
			break;
		striped_fname = tmp;
	}
	if (strip > 0)
		LOG_WARNING("Failed to srip %d dir names in Gcov file path '%s'!", strip,
			orig_fname);
	strcpy(&filename[prefix_length], striped_fname);

	return filename;
}


static int esp_gcov_fopen(struct esp32_gcov_cmd_data *cmd_data,
	uint8_t *data,
	uint32_t data_len,
	uint8_t **resp,
	uint32_t *resp_len)
{
	*resp_len = 0;
	if (cmd_data->files_num == ESP_GCOV_FILES_MAX_NUM) {
		LOG_ERROR("Max gcov files num exceeded!");
		return ERROR_FAIL;
	}

	if (data_len == 0) {
		LOG_ERROR("Missed FOPEN args!");
		return ERROR_FAIL;
	}
	int len = strlen((char *)data);
	if (len == 0) {
		LOG_ERROR("Missed FOPEN path arg!");
		return ERROR_FAIL;
	}
	if (data_len - len - 1 == 0) {
		LOG_ERROR("Missed FOPEN mode arg!");
		return ERROR_FAIL;
	}

	uint32_t fd = cmd_data->files_num;
	char *mode = (char *)data + len + 1;
	const char *fname = esp_gcov_filename_alloc((const char *)data);
	if (fname == NULL) {
		LOG_ERROR("Failed to alloc memory for file name!");
		return ERROR_FAIL;
	}
	LOG_INFO("Open file 0x%x '%s'", fd+1, fname);
	cmd_data->files[fd] = fopen(fname, mode);
	if (!cmd_data->files[fd]) {
		/* do not report error on reading non-existent file */
		if (errno != ENOENT || strchr(mode, 'r') == NULL)
			LOG_ERROR("Failed to open file '%s', mode '%s' (%d)!", fname, mode, errno);
		fd = 0;
	} else
		fd++;	/* 1-based, 0 indicates error */

	*resp_len = sizeof(fd);
	*resp = malloc(*resp_len);
	if (!*resp) {
		LOG_ERROR("Failed to alloc mem for resp!");
		if (fd != 0)
			fclose(cmd_data->files[fd-1]);
		free((void *)fname);
		return ERROR_FAIL;
	}
	memcpy(*resp, &fd, sizeof(fd));

	if (fd != 0)
		cmd_data->files_num++;

	free((void *)fname);
	return ERROR_OK;
}

static int esp_gcov_fclose(struct esp32_gcov_cmd_data *cmd_data,
	uint8_t *data,
	uint32_t data_len,
	uint8_t **resp,
	uint32_t *resp_len)
{
	*resp_len = 0;
	if (data_len < sizeof(uint32_t)) {
		LOG_ERROR("Missed FCLOSE args!");
		return ERROR_FAIL;
	}
	uint32_t fd;
	memcpy(&fd, data, sizeof(fd));
	fd--;
	if (fd >= ESP_GCOV_FILES_MAX_NUM) {
		LOG_ERROR("Invalid file desc received 0x%x!", fd);
		return ERROR_FAIL;
	}
	if (!cmd_data->files[fd]) {
		LOG_ERROR("FCLOSE for not open file!");
		return ERROR_FAIL;
	}

	int32_t fret = fclose(cmd_data->files[fd]);
	if (fret)
		LOG_ERROR("Failed to close file %d (%d)!", fd, errno);
	else
		cmd_data->files[fd] = NULL;

	*resp_len = sizeof(fret);
	*resp = malloc(*resp_len);
	if (!*resp) {
		LOG_ERROR("Failed to alloc mem for resp!");
		return ERROR_FAIL;
	}
	memcpy(*resp, &fret, sizeof(fret));

	return ERROR_OK;
}

static int esp_gcov_fwrite(struct esp32_gcov_cmd_data *cmd_data,
	uint8_t *data,
	uint32_t data_len,
	uint8_t **resp,
	uint32_t *resp_len)
{
	*resp_len = 0;
	if (data_len < sizeof(uint32_t)) {
		LOG_ERROR("Missed FWRITE args!");
		return ERROR_FAIL;
	}
	uint32_t fd;
	memcpy(&fd, data, sizeof(fd));
	fd--;
	if (fd >= ESP_GCOV_FILES_MAX_NUM) {
		LOG_ERROR("Invalid file desc received 0x%x!", fd);
		return ERROR_FAIL;
	}
	if (!cmd_data->files[fd]) {
		LOG_ERROR("FWRITE for not open file!");
		return ERROR_FAIL;
	}

	uint32_t fret = fwrite(data + sizeof(fd), data_len - sizeof(fd), 1, cmd_data->files[fd]);
	if (fret != 1)
		LOG_ERROR("Failed to write %ld byte (%d)!", (long)(data_len - sizeof(fd)), errno);

	*resp_len = sizeof(fret);
	*resp = malloc(*resp_len);
	if (!*resp) {
		LOG_ERROR("Failed to alloc mem for resp!");
		return ERROR_FAIL;
	}
	memcpy(*resp, &fret, sizeof(fret));

	return ERROR_OK;
}

static int esp_gcov_fread(struct esp32_gcov_cmd_data *cmd_data,
	uint8_t *data,
	uint32_t data_len,
	uint8_t **resp,
	uint32_t *resp_len)
{
	uint32_t fret;

	*resp_len = 0;
	if (data_len < 2*sizeof(uint32_t)) {
		LOG_ERROR("Missed FREAD args!");
		return ERROR_FAIL;
	}
	uint32_t fd;
	memcpy(&fd, data, sizeof(fd));
	fd--;
	if (fd >= ESP_GCOV_FILES_MAX_NUM) {
		LOG_ERROR("Invalid file desc received 0x%x!", fd);
		return ERROR_FAIL;
	}
	if (!cmd_data->files[fd]) {
		LOG_ERROR("FREAD for not open file!");
		return ERROR_FAIL;
	}
	uint32_t len;
	memcpy(&len, data + sizeof(fd), sizeof(len));

	*resp_len = sizeof(fret) + len;
	*resp = malloc(*resp_len);
	if (!*resp) {
		LOG_ERROR("Failed to alloc mem for resp!");
		return ERROR_FAIL;
	}
	fret = fread(*resp + sizeof(fret), 1, len, cmd_data->files[fd]);
	if (fret == 0)
		LOG_ERROR("Failed to read %d byte (%d)!", len, errno);
	*resp_len = sizeof(fret) + fret;
	memcpy(*resp, &fret, sizeof(fret));

	return ERROR_OK;
}

static int esp_gcov_fseek(struct esp32_gcov_cmd_data *cmd_data,
	uint8_t *data,
	uint32_t data_len,
	uint8_t **resp,
	uint32_t *resp_len)
{
	*resp_len = 0;
	if (data_len < (sizeof(uint32_t) + 2*sizeof(int32_t))) {
		LOG_ERROR("Missed FSEEK args!");
		return ERROR_FAIL;
	}
	uint32_t fd;
	memcpy(&fd, data, sizeof(fd));
	fd--;
	if (fd >= ESP_GCOV_FILES_MAX_NUM) {
		LOG_ERROR("Invalid file desc received 0x%x!", fd);
		return ERROR_FAIL;
	}
	if (!cmd_data->files[fd]) {
		LOG_ERROR("FSEEK for not open file!");
		return ERROR_FAIL;
	}

	int32_t off;
	memcpy(&off, data + sizeof(fd), sizeof(off));
	int32_t whence;
	memcpy(&whence, data + sizeof(fd) + sizeof(off), sizeof(whence));

	int32_t fret = fseek(cmd_data->files[fd], off, whence);
	*resp_len = sizeof(fret);
	*resp = malloc(*resp_len);
	if (!*resp) {
		LOG_ERROR("Failed to alloc mem for resp!");
		return ERROR_FAIL;
	}
	memcpy(*resp, &fret, sizeof(fret));

	return ERROR_OK;
}

static int esp_gcov_ftell(struct esp32_gcov_cmd_data *cmd_data,
	uint8_t *data,
	uint32_t data_len,
	uint8_t **resp,
	uint32_t *resp_len)
{
	*resp_len = 0;
	if (data_len < sizeof(uint32_t)) {
		LOG_ERROR("Missed FTELL args!");
		return ERROR_FAIL;
	}
	uint32_t fd;
	memcpy(&fd, data, sizeof(fd));
	fd--;
	if (fd >= ESP_GCOV_FILES_MAX_NUM) {
		LOG_ERROR("Invalid file desc received 0x%x!", fd);
		return ERROR_FAIL;
	}
	if (!cmd_data->files[fd]) {
		LOG_ERROR("FTELL for not open file!");
		return ERROR_FAIL;
	}

	int32_t fret = ftell(cmd_data->files[fd]);
	*resp_len = sizeof(fret);
	*resp = malloc(*resp_len);
	if (!*resp) {
		LOG_ERROR("Failed to alloc mem for resp!");
		return ERROR_FAIL;
	}
	memcpy(*resp, &fret, sizeof(fret));

	return ERROR_OK;
}

/*TODO: support for multi-block data transfers */
static int esp_gcov_process_data(struct esp32_apptrace_cmd_ctx *ctx,
	int core_id,
	uint8_t *data,
	uint32_t data_len)
{
	int ret = ERROR_OK;
	struct esp32_gcov_cmd_data *cmd_data = ctx->cmd_priv;
	uint8_t *resp;
	uint32_t resp_len = 0;

	LOG_DEBUG("Got block %d bytes [%x %x]", data_len, data[0], data[1]);

	if (data_len < 1) {
		LOG_ERROR("Too small data length %d!", data_len);
		return ERROR_FAIL;
	}

	switch (*data) {
		case ESP_APPTRACE_FILE_CMD_FOPEN:
			ret = esp_gcov_fopen(cmd_data, data+1, data_len-1, &resp, &resp_len);
			break;
		case ESP_APPTRACE_FILE_CMD_FCLOSE:
			ret = esp_gcov_fclose(cmd_data, data+1, data_len-1, &resp, &resp_len);
			break;
		case ESP_APPTRACE_FILE_CMD_FWRITE:
			ret = esp_gcov_fwrite(cmd_data, data+1, data_len-1, &resp, &resp_len);
			break;
		case ESP_APPTRACE_FILE_CMD_FREAD:
			ret = esp_gcov_fread(cmd_data, data+1, data_len-1, &resp, &resp_len);
			break;
		case ESP_APPTRACE_FILE_CMD_FSEEK:
			ret = esp_gcov_fseek(cmd_data, data+1, data_len-1, &resp, &resp_len);
			break;
		case ESP_APPTRACE_FILE_CMD_FTELL:
			ret = esp_gcov_ftell(cmd_data, data+1, data_len-1, &resp, &resp_len);
			break;
		case ESP_APPTRACE_FILE_CMD_STOP:
			ctx->running = 0;
			break;
		default:
			LOG_ERROR("Invalid FCMD 0x%x!", *data);
			ret = ERROR_FAIL;
	}
	if (ret != ERROR_OK)
		return ret;

	if (resp_len) {
		struct esp32_apptrace_target_state target_state[ESP_APPTRACE_MAX_CORES_NUM];
		uint32_t fired_target_num = 0;
		/* get current block id */
		int res = esp32_apptrace_get_data_info(ctx, target_state, &fired_target_num);
		if (res != ERROR_OK) {
			LOG_ERROR("Failed to read target data info!");
			free(resp);
			return res;
		}
		if (fired_target_num == (uint32_t)-1) {
			/* it can happen that there is no pending target data, but block was
			 * switched */
			/* in this case block_ids on both CPUs are equal, so select the first one */
			fired_target_num = 0;
		}
		/* write response */
		res =
			esp_xtensa_apptrace_usr_block_write(ctx->cpus[fired_target_num],
			target_state[fired_target_num].block_id,
			resp,
			resp_len);
		if (res != ERROR_OK) {
			LOG_ERROR("Failed to write data to (%s)!",
				target_name(ctx->cpus[fired_target_num]));
			free(resp);
			return res;
		}
		free(resp);
	}

	return ERROR_OK;
}

int esp_gcov_poll(struct target *target, void *priv)
{
	int res = ERROR_OK;
	struct esp32_apptrace_cmd_ctx *cmd_ctx = (struct esp32_apptrace_cmd_ctx *)priv;

	while (!shutdown_openocd && target->state != TARGET_HALTED && cmd_ctx->running) {
		res = esp32_apptrace_poll(cmd_ctx);
		if (res != ERROR_OK) {
			LOG_ERROR("Failed to poll target for gcov data (%d)!", res);
			break;
		}
		/* let registered timer callbacks to run */
		target_call_timer_callbacks();
	}
	return res;
}

COMMAND_HANDLER(esp32_cmd_gcov)
{
	static struct esp32_apptrace_cmd_ctx s_at_cmd_ctx;
	int res = ERROR_OK;
	struct target *target = get_current_target(CMD_CTX);
	enum target_state old_state = target->state;
	struct xtensa_algo_run_data run;
	uint32_t func_addr;
	bool dump = false;

	if (CMD_ARGC > 0) {
		if (strcmp(CMD_ARGV[0], "dump") == 0)
			dump = true;
		else {
			LOG_ERROR("Invalid action!");
			return ERROR_FAIL;
		}
	}

	/* init cmd context */
	res = esp_gcov_cmd_init(target, &s_at_cmd_ctx, CMD_ARGV, CMD_ARGC);
	if (res != ERROR_OK) {
		LOG_ERROR("Failed to init cmd ctx (%d)!", res);
		return res;
	}
	/* connect */
	res = esp32_apptrace_connect_targets(&s_at_cmd_ctx, target, true, dump);
	if (res != ERROR_OK) {
		LOG_ERROR("Failed to connect to targets (%d)!", res);
		esp_gcov_cmd_cleanup(&s_at_cmd_ctx);
		return res;
	}
	if (dump)
		esp_gcov_poll(target, &s_at_cmd_ctx);
	else {
		struct target *active_core_target = NULL;
		if (target_get_core_count(target) != 0) {
			struct xtensa_mcore_common *xtensa_mcore = target_to_xtensa_mcore(target);
			active_core_target =
				&xtensa_mcore->cores_targets[xtensa_mcore->active_core];
		} else
			active_core_target = target;
		struct esp_dbg_stubs *dbg_stubs =
			&(target_to_esp_xtensa(active_core_target)->dbg_stubs);
		if (dbg_stubs->entries_count < 1 || dbg_stubs->desc.data_alloc == 0) {
			LOG_ERROR("No dbg stubs found!");
			return ERROR_FAIL;
		}
		if (dbg_stubs->entries_count < ESP_DBG_STUB_ENTRY_GCOV+1) {
			LOG_ERROR("No GCOV stubs found!");
			return ERROR_FAIL;
		}
		func_addr = dbg_stubs->entries[ESP_DBG_STUB_ENTRY_GCOV];
		LOG_DEBUG("GCOV_FUNC = 0x%x", func_addr);
		if (func_addr == 0) {
			LOG_ERROR("GCOV stub not found!");
			return ERROR_FAIL;
		}
		memset(&run, 0, sizeof(run));
		run.stack_size      = 1024;
		run.usr_func_arg    = &s_at_cmd_ctx;
		run.usr_func        = esp_gcov_poll;
		run.on_board.min_stack_addr = dbg_stubs->desc.min_stack_addr;
		run.on_board.min_stack_size = ESP_DBG_STUBS_STACK_MIN_SIZE;
		run.on_board.code_buf_addr = dbg_stubs->desc.tramp_addr;
		run.on_board.code_buf_size = ESP_DBG_STUBS_CODE_BUF_SIZE;
		xtensa_run_onboard_func(active_core_target, &run, func_addr, 0);
		LOG_DEBUG("FUNC RET = 0x%x", run.ret_code);
	}
	/* disconnect */
	res = esp32_apptrace_connect_targets(&s_at_cmd_ctx,
		target,
		false,
		old_state == TARGET_RUNNING);
	if (res != ERROR_OK)
		LOG_ERROR("Failed to disconnect targets (%d)!", res);
	res = esp_gcov_cmd_cleanup(&s_at_cmd_ctx);
	if (res != ERROR_OK)
		LOG_ERROR("Failed to cleanup cmd ctx (%d)!", res);
	return res;
}

const struct command_registration esp32_apptrace_command_handlers[] = {
	{
		.name = "apptrace",
		.handler = esp32_cmd_apptrace,
		.mode = COMMAND_EXEC,
		.help =
			"App Tracing: application level trace control. Starts, stops or queries tracing process status.",
		.usage =
			"[start file://<outfile> [poll_period [trace_size [stop_tmo [wait4halt [skip_size]]]]] | [stop] | [status] | [dump file://<outfile>]",
	},
	{
		.name = "sysview",
		.handler = esp32_cmd_sysview,
		.mode = COMMAND_EXEC,
		.help =
			"App Tracing: SEGGER SystemView compatible trace control. Starts, stops or queries tracing process status.",
		.usage =
			"[start file://<outfile1> [file://<outfile2>] [poll_period [trace_size [stop_tmo [wait4halt [skip_size]]]]] | [stop] | [status]",
	},
	{
		.name = "gcov",
		.handler = esp32_cmd_gcov,
		.mode = COMMAND_ANY,
		.help = "GCOV: Dumps gcov info collected on target.",
		.usage = "",
	},
	COMMAND_REGISTRATION_DONE
};
