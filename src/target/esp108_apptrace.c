/***************************************************************************
 *	 ESP108 application tracing module	for OpenOCD						   *
 *	 Copyright (C) 2017 Espressif Systems Ltd.							   *
 *	 <alexey@espressif.com>												   *
 *																		   *
 *	 Derived from original ESP8266 target.								   *
 *	 Copyright (C) 2015 by Angus Gratton								   *
 *	 gus@projectgus.com													   *
 *																		   *
 *	 This program is free software; you can redistribute it and/or modify  *
 *	 it under the terms of the GNU General Public License as published by  *
 *	 the Free Software Foundation; either version 2 of the License, or	   *
 *	 (at your option) any later version.								   *
 *																		   *
 *	 This program is distributed in the hope that it will be useful,	   *
 *	 but WITHOUT ANY WARRANTY; without even the implied warranty of		   *
 *	 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the		   *
 *	 GNU General Public License for more details.						   *
 *																		   *
 *	 You should have received a copy of the GNU General Public License	   *
 *	 along with this program; if not, write to the						   *
 *	 Free Software Foundation, Inc.,									   *
 *	 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.		   *
 ***************************************************************************/

// Hot It Works
// ************

// 1. Components Overview
// ======================

// Xtensa has useful feature: TRAX debug module. It allows recording program execution flow during run-time without disturbing CPU execution.
// Exectution flow data are written to configurable Trace RAM block. Besides accessing RAM itself TRAX module also allows to read/write
// trace memory via its registers by means of JTAG, APB or ERI transactions.
// ESP32 has two Xtensa cores with separate TRAX modules on them and provides two special memory regions to be used as trace memory.
// ESP32 allows muxing access to trace memory blocks in such way that while one block is accessed by CPUs another can be accessed via JTAG by host
// by means of reading/writing TRAX registers. Block muxing is configurable at run-time and allows switching trace memory blocks between
// accessors in round-robin way so they can read/write separate memory blocks without disturbing each other.
// This moduile implements application tracing feature basing on above mechanisms. This feature allows to transfer arbitrary user data to
// host via JTAG with minimal impact on system performance. This module is implied to be used in the following tracing scheme.

//													  ------>------											----- (host components) -----
//													  |			  |											|							|
// ---------------	 -----------------------	 -----------------------	 ----------------	 ------		---------	-----------------
// |apptrace user|-->|target tracing module|<--->|TRAX_MEM0 | TRAX_MEM1|---->|TRAX_DATA_REGS|<-->|JTAG|<--->|OpenOCD|-->|trace data file|
// ---------------	 -----------------------	 -----------------------	 ----------------	 ------		---------	-----------------
//							   |					  |			  |								   |
//							   |					  ------<------			 ----------------	   |
//							   |<------------------------------------------->|TRAX_CTRL_REGS|<---->|
//																			 ----------------

// In general tracing happens in the following way. User aplication requests tracing module to send some data by calling esp_apptrace_buffer_get(),
// moduile allocates necessary buffer in current input trace block. Then user fills received buffer with data and calls esp_apptrace_buffer_put().
// When current input trace block is filled with app data it is exposed to host and the second block becomes input one and block filling restarts.
// While target application fills one memory block host reads another block via JTAG.
// To control buffer switching and for other communication purposes this implementation uses some TRAX registers. It is safe since HW TRAX tracing
// can not be used along with application tracing feature so these registers are freely readable/writeable via JTAG from host and via ERI from ESP32 cores.
// So this implementation's target CPU overhead is produced only by calls to allocate and manage buffers and data copying.
// On host special OpenOCD command must be used to read trace data.

// 2. Registers layout
// ======================

// This module uses two TRAX HW registers to communicate with host SW (OpenOCD).
//	- Control register uses TRAX_DELAYCNT as storage. Only lower 24 bits of TRAX_DELAYCNT are writable. Register has the following bitfields:
//	 | 31..XXXXXX..24 | 23 .(host_connect). 23| 22..(block_id)..15 | 14..(block_len)..0 |
//	  14..0	 bits - actual length of user data in trace memory block. Target updates it every time it fills memory block and exposes it to host.
//					Host writes zero to this field when it finishes reading exposed block;
//	  22..15 bits - trace memory block transfer ID. Block counter. It can overflow. Updated by target, host should not modify it. Actually can be 1-2 bits;
//	  23	 bit  - 'host connected' flag. If zero then host is not connected and tracing module works in post-mortem mode, otherwise in streaming mode;
// - Status register uses TRAX_TRIGGERPC as storage. If this register is not zero then currentlly CPU is changing TRAX registers and
//	 this register holds address of the instruction which application will execute when it will finish with those registers modification.
//	 See 'Targets Connection' setion for details.

// 3. Modes of operation
// ======================

// This module supports two modes of operation:
//	- Post-mortem mode. This is the default mode. In this mode application tracing module does not check whether host read all the data from block
//	  exposed to it and switches block in any case. The mode is useful when only the latest trace data are necessary, e.g. for analyzing crashes.
//	  On panic data from current input block are exposed to host and host can read them.
//	- Streaming mode. Tracing module enters this mode when host connects to targets and sets respective bit in control register. In this mode tracing
//	  module waits for specified time until host read all the data from exposed block.
//	  On panic tracing module waits (timeout is configured via menuconfig) for the host to read all data from the previously exposed block.

// 4. Communication Protocol
// =========================

// 4.1 HW Transport Level
// ----------------------------

// 4.1.1 Trace Memory Blocks
// ^^^^^^^^^^^^^^^^^^^^^^^^^

// Communication is controlled via special register. Host periodically polls control register on each core to find out if there are any data avalable.
// When current input trace memory block is filled tracing module exposes block to host and updates block_len and block_id fields in control register.
// Host reads new register value and according to it starts reading data from exposed block. Meanwhile target starts filling another trace block.
// When host finishes reading the block it clears block_len field in control register indicating to target that it is ready to accept next block.

// 4.2 User Data Chunks Level
// --------------------------

// Since trace memory block is shared between user data chunks and data copying is performed on behalf of the API user (in its normal context) in
// multithreading environment it can happen that task/ISR which copies data is preempted by another high prio task/ISR. So it is possible situation
// that task/ISR will fail to complete writing its data chunk before the whole trace block is exposed to the host. To handle such conditions tracing
// module prepends all user data chunks with 4 bytes header which contains allocated buffer size and actual data length within it. OpenOCD command
// to read application traces will report error when it will read incompleted user data block.

// 4.3 Targets Connection/Disconnection
// ------------------------------------

// When host is going to start tracing in streaming mode it needs to put both ESP32 cores into certain initial state when 'host connected' bit is set
// on both cores. To accomplish it host halts both cores and sets this bit in TRAX registers. But target code can be in state when it read control
// register and before updating its value. To handle such situations target code indicates to the host that it is updating control register by writing
// non-zero value to status register. Actually target code write address of the instruction which application will execute when it will finish with
// the update. When target is halted during control register update host sets breakpoint at the address from status register and resumes CPU.
// After target code finishes with register update it is halted on breakpoint? host detects it and safely sets 'host connected' bit. When both cores
// are set up they are resumed. Tracing starts without further disturbing of CPUs.
// When host is going to stop tracing in streaming mode it needs to disconnect from targets. Disconnection process is done using the same algorithm
// as for connecting, but 'host connected' bits are cleared on ESP32 cores.

// 4.4 Application Tracing Command Options
// ---------------------------------------

// Command usage:
// ``esp108 apptrace [start <outfile> [options] | [stop] | [status] | [dump <outfile>]``

// Sub-commands:
//	 * ``start``.  Start tracing (continuous streaming).
//	 * ``stop``.   Stop tracing.
//	 * ``status``. Get tracing status.
//	 * ``dump``.   Dump as much data as possible without waiting for trace memory block switch (post-mortem dump).

// Start command syntax:
//	 ``start <outfile> [trace_size [stop_tmo [skip_size [poll_period [wait4halt]]]]]``
//	   * - outfile
//		 - Path to log trace file to save data
//	   * - trace_size
//		 - Maximum size of data to collect (in bytes). Tracing is stopped after specified amount of data is received. By default -1 (trace size stop trigger is disabled).
//	   * - stop_tmo
//		 - Idle timeout (in ms). Tracing is stopped if there is no data for specified period of time. By default 10 s (-1 to disable this stop trigger).
//	   * - skip_size
//		 - Number of bytes to skip at the start. By default 0.
//	   * - poll_period
//		 - Data polling period (in ms). If greater then 0 then command runs in non-blocking mode, otherwise command line will not be avalable until tracing is stopped. By default 1 ms.
//	   * - wait4halt
//		 - If 0 start tracing immediately, otherwise command waits for the target to be halted (after reset, by breakpoint etc) and then automatically resumes it and starts tracing. By default 0.

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "target.h"
#include "time_support.h"

#include "esp108.h"
#include "esp108_dbg_regs.h"

// TRAX is disabled, so we use its registers for our own purposes
// | 31..XXXXXX..24 | 23 .(host_connect). 23| 22..(block_id)..15 | 14..(block_len)..0 |
#define ESP_APPTRACE_TRAX_CTRL_REG			 NARADR_DELAYCNT
#define ESP_APPTRACE_TRAX_STAT_REG			 NARADR_TRIGGERPC

#define ESP_APPTRACE_BLOCK_LEN_MSK		   0x7FFFUL
#define ESP_APPTRACE_BLOCK_LEN(_l_)		   ((_l_) & ESP_APPTRACE_BLOCK_LEN_MSK)
#define ESP_APPTRACE_BLOCK_LEN_GET(_v_)	   ((_v_) & ESP_APPTRACE_BLOCK_LEN_MSK)
#define ESP_APPTRACE_BLOCK_ID_MSK		   0xFFUL
#define ESP_APPTRACE_BLOCK_ID(_id_)		   (((_id_) & ESP_APPTRACE_BLOCK_ID_MSK) << 15)
#define ESP_APPTRACE_BLOCK_ID_GET(_v_)	   (((_v_) >> 15) & ESP_APPTRACE_BLOCK_ID_MSK)
#define ESP_APPTRACE_HOST_CONNECT		   (1 << 23)

#define ESP_APPTRACE_TGT_STATE_TMO	5000

// need to check it when poll period is less then 1 ms in order to react on CTRL+C etc
extern int shutdown_openocd;

static int esp108_apptrace_read_data(struct target *target, uint32_t size, uint8_t *buffer, uint32_t block_id, struct duration *dur)
{
	int res = 0;
	uint32_t i;
	uint32_t tmp = ESP_APPTRACE_HOST_CONNECT | ESP_APPTRACE_BLOCK_ID(block_id) | ESP_APPTRACE_BLOCK_LEN(0);

	// start read from the beginning
	esp108_queue_nexus_reg_write(target, NARADR_TRAXADDR, 0);
	//FIXME: why data order is reversed???
	for (i = size/4; i > 0; i--) { // size is always 16KB
		esp108_queue_nexus_reg_read(target, NARADR_TRAXDATA, &buffer[(i-1)*4]);
	}
	esp108_queue_nexus_reg_write(target, ESP_APPTRACE_TRAX_CTRL_REG, tmp);
	esp108_queue_tdi_idle(target);
	if (dur && duration_start(dur) != 0) {
		LOG_ERROR("Failed to start data read measurement!");
		return ERROR_FAIL;
	}
	res = jtag_execute_queue();
	if (dur && duration_measure(dur) != 0) {
		LOG_ERROR("Failed to stop data read measurement!");
		return ERROR_FAIL;
	}
	if (res != ERROR_OK) {
		LOG_ERROR("Failed to exec JTAG queue!");
		return res;
	}

	return ERROR_OK;
}

static int esp108_apptrace_read_data_len(struct target *target, uint32_t *block_id, uint32_t *len)
{
	int res = 0;
	uint8_t tmp[4];

	esp108_queue_nexus_reg_read(target, ESP_APPTRACE_TRAX_CTRL_REG, tmp);
	esp108_queue_tdi_idle(target);
	res = jtag_execute_queue();
	if (res != ERROR_OK) {
		LOG_ERROR("Failed to exec JTAG queue!");
		return res;
	}
	uint32_t val = intfromchars(tmp);
	*block_id = ESP_APPTRACE_BLOCK_ID_GET(val);
	*len = ESP_APPTRACE_BLOCK_LEN_GET(val);
	return ERROR_OK;
}

static int esp108_apptrace_write_data_len(struct target *target, uint32_t block_id, uint32_t len, int conn)
{
	int res = 0;
	uint32_t tmp = (conn ? ESP_APPTRACE_HOST_CONNECT : 0) | ESP_APPTRACE_BLOCK_ID(block_id) | ESP_APPTRACE_BLOCK_LEN(len);

	esp108_queue_nexus_reg_write(target, ESP_APPTRACE_TRAX_CTRL_REG, tmp);
	esp108_queue_tdi_idle(target);
	res = jtag_execute_queue();
	if (res != ERROR_OK) {
		LOG_ERROR("Failed to exec JTAG queue!");
		return res;
	}

	return ERROR_OK;
}

static int esp108_apptrace_read_status(struct target *target, uint32_t *stat)
{
	int res = 0;
	uint8_t tmp[4];

	esp108_queue_nexus_reg_read(target, ESP_APPTRACE_TRAX_STAT_REG, tmp);
	esp108_queue_tdi_idle(target);
	res = jtag_execute_queue();
	if (res != ERROR_OK) {
		LOG_ERROR("Failed to exec JTAG queue!");
		return res;
	}
	*stat = intfromchars(tmp);
	return ERROR_OK;
}

struct apptrace_data_hdr {
	uint16_t   block_sz;
	uint16_t   wr_sz;
};

struct apptrace_cmd_stats {
	uint32_t		incompl_blocks;
	uint32_t		lost_bytes;
	float			min_blk_read_time;
	float			max_blk_read_time;
};

struct apptrace_cmd_ctx {
	int				running;
	uint8_t *		trax_block_data;
	uint32_t		trax_block_sz;
	int				fout;
	uint32_t		tot_len;
	uint32_t		max_len;
	uint32_t		skip_len;
	float			stop_tmo;
	uint32_t		poll_period;
	int				wait4halt;
	struct duration idle_time;
	struct duration read_time;
	struct apptrace_cmd_stats stats;
};

static int esp108_activate_swdbg(struct target *target, int enab)
{
	int res;

	if (enab)
		esp108_queue_nexus_reg_write(target, NARADR_DCRSET, OCDDCR_DEBUGSWACTIVE);
	else
		esp108_queue_nexus_reg_write(target, NARADR_DCRCLR, OCDDCR_DEBUGSWACTIVE);
	esp108_queue_tdi_idle(target);
	res=jtag_execute_queue();
	if (res!=ERROR_OK) {
		LOG_ERROR("%s: writing DCR failed!", target->cmd_name);
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

static int esp108_cmd_apptrace_wait4halt(void)
{
	int halted = 0;

	LOG_USER("Wait for halt...");
	while (!shutdown_openocd) {
		for (int k = 0; k < 2; k++) {
			struct target *target = get_target_by_num(k);
			if (!target) {
				LOG_ERROR("Failed to get target%d!", k);
				return ERROR_FAIL;
			}
			int res = target_poll(target);
			if (res != ERROR_OK) {
				LOG_ERROR("Failed to poll target%d (%d)!", k, res);
				return res;
			}
			if (target->state == TARGET_HALTED) {
				LOG_USER("%s: HALTED", target->cmd_name);
				if (++halted == 2)
					break;
			}
		}
		if(halted)
			break;
		alive_sleep(100);
	}
	return ERROR_OK;
}

static int esp108_cmd_apptrace_connect_targets(int conn)
{
	int res = ERROR_OK;
	struct apptrace_target_state {
		struct target *cpu;
		//int runstall;
		int running;
		uint32_t block_id;
	} target_to_connect[2];

	if (conn)
		LOG_USER("Connect targets...");
	else
		LOG_USER("Disconnect targets...");

	memset(target_to_connect, 0, sizeof(target_to_connect));
	// halt all CPUs
	for (int k = 0; k < 2; k++) {
		target_to_connect[k].cpu = get_target_by_num(k);
		if (!target_to_connect[k].cpu) {
			LOG_ERROR("Failed to get target%d!", k);
			return ERROR_FAIL;
		}
		target_to_connect[k].running = 1;
		res = target_halt(target_to_connect[k].cpu);
		if (res != ERROR_OK) {
			LOG_ERROR("Failed to halt target%d (%d)!", k, res);
			return res;
		}
		res = target_wait_state(target_to_connect[k].cpu, TARGET_HALTED, ESP_APPTRACE_TGT_STATE_TMO);
		if (res != ERROR_OK) {
			LOG_ERROR("Failed to wait halt target %s / %d (%d)!", target_to_connect[k].cpu->cmd_name, target_to_connect[k].cpu->state, res);
			return res;
		}
	}
	// read current block statuses from CPU
	for (int k = 0; k < 2; k++) {
		uint32_t stat;
		res = esp108_apptrace_read_status(target_to_connect[k].cpu, &stat);
		if (res != ERROR_OK) {
			LOG_ERROR("Failed to read trace status (%d)!", res);
			return res;
		}
		// check if some CPU stopped inside TRAX reg update critical section
		if (stat) {
			res = esp108_activate_swdbg(target_to_connect[k].cpu, 1);
			if (res != ERROR_OK) {
				LOG_ERROR("Failed to activate SW debug (%d)!", res);
				return res;
			}
			uint32_t bp_addr = stat;
			res = breakpoint_add(target_to_connect[k].cpu, bp_addr, 1, BKPT_HARD);
			if (res != ERROR_OK) {
				LOG_ERROR("Failed to set breakpoint (%d)!", res);
				return res;
			}
			while (stat) {
				// allow this CPU to leave ERI write critical section
				res = target_resume(target_to_connect[k].cpu, 1, 0, 1, 0);
				if (res != ERROR_OK) {
					LOG_ERROR("Failed to resume target (%d)!", res);
					breakpoint_remove(target_to_connect[k].cpu, bp_addr);
					return res;
				}
				do {
					res = target_wait_state(target_to_connect[k].cpu, TARGET_HALTED, ESP_APPTRACE_TGT_STATE_TMO);
					if (res != ERROR_OK) {
						LOG_ERROR("Failed to wait halt on bp target (%d)!", res);
						breakpoint_remove(target_to_connect[k].cpu, bp_addr);
						return res;
					}
				} while (target_to_connect[k].cpu->debug_reason != DBG_REASON_BREAKPOINT);
				res = esp108_apptrace_read_status(target_to_connect[k].cpu, &stat);
				if (res != ERROR_OK) {
					LOG_ERROR("Failed to read trace status (%d)!", res);
					breakpoint_remove(target_to_connect[k].cpu, bp_addr);
					return res;
				}
			}
			breakpoint_remove(target_to_connect[k].cpu, bp_addr);
			res = esp108_activate_swdbg(target_to_connect[k].cpu, 0);
			if (res != ERROR_OK) {
				LOG_ERROR("Failed to de-activate SW debug (%d)!", res);
				return res;
			}
		}
		uint32_t len;
		res = esp108_apptrace_read_data_len(target_to_connect[k].cpu, &target_to_connect[k].block_id, &len);
		if (res != ERROR_OK) {
			LOG_ERROR("Failed to read trace status (%d)!", res);
			return res;
		}
	}
	// set block id to higher value
	if (target_to_connect[0].block_id != target_to_connect[1].block_id) {
		if (target_to_connect[1].block_id > target_to_connect[0].block_id)
			target_to_connect[0].block_id = target_to_connect[1].block_id;
		else
			target_to_connect[1].block_id = target_to_connect[0].block_id;
	}
	LOG_INFO("Resume targets");
	for (int k = 0; k < 2; k++) {
		// update host connected status
		res = esp108_apptrace_write_data_len(target_to_connect[k].cpu, target_to_connect[k].block_id, 0, conn);
		if (res != ERROR_OK) {
			LOG_ERROR("Failed to read trace status (%d)!", res);
			return res;
		}
		res = target_resume(target_to_connect[k].cpu, 1, 0, 1, 0);
		if (res != ERROR_OK) {
			LOG_ERROR("Failed to resume target %d (%d)!", k, res);
			return res;
		}
	}
	if (conn)
		LOG_INFO("Targets connected.");
	else
		LOG_INFO("Targets disconnected.");
	return res;
}

static int esp108_cmd_apptrace_ctx_init(struct apptrace_cmd_ctx *cmd_ctx, struct target *target, const char **argv, int argc)
{
	uint8_t traxstat[4], memadrstart[4], memadrend[4], adr[4], traxctl[4];
	uint32_t stop_tmo = (uint32_t)-1;
	int res;

	if (argc < 1) {
		LOG_ERROR("Need filename to dump to as output!");
		return ERROR_FAIL;
	}

	memset(cmd_ctx, 0, sizeof(struct apptrace_cmd_ctx));

	cmd_ctx->max_len = (uint32_t)-1;
	cmd_ctx->stop_tmo = 1000000.0;
	cmd_ctx->poll_period = 1/*ms*/;
	if (argc > 1) {
		cmd_ctx->max_len = strtoul(argv[1], NULL, 10);
		if (argc > 2) {
			stop_tmo = strtoul(argv[2], NULL, 10);
			if (argc > 3) {
				cmd_ctx->skip_len = strtoul(argv[3], NULL, 10);
				if (argc > 4) {
					cmd_ctx->poll_period = strtoul(argv[4], NULL, 10);
					if (argc > 5) {
						cmd_ctx->wait4halt = strtoul(argv[5], NULL, 10);
					}
				}
			}
		}
	}
	cmd_ctx->stop_tmo = (float)stop_tmo/1000;
	cmd_ctx->stats.min_blk_read_time = 1000000.0;

	LOG_USER("Start %d app trace to %s, size %u bytes, stop_tmo %u ms, skip %u bytes, poll period %u ms, wait_rst %d",
			argc, argv[0], cmd_ctx->max_len, stop_tmo, cmd_ctx->skip_len, cmd_ctx->poll_period, cmd_ctx->wait4halt);

	esp108_queue_nexus_reg_read(target, NARADR_TRAXSTAT, traxstat);
	esp108_queue_nexus_reg_read(target, NARADR_TRAXCTRL, traxctl);
	esp108_queue_nexus_reg_read(target, NARADR_MEMADDRSTART, memadrstart);
	esp108_queue_nexus_reg_read(target, NARADR_MEMADDREND, memadrend);
	esp108_queue_nexus_reg_read(target, NARADR_TRAXADDR, adr);
	esp108_queue_tdi_idle(target);
	res = jtag_execute_queue();
	if (res) {
		LOG_ERROR("Failed to read TRAX config (%d)!", res);
		return res;
	}

	cmd_ctx->trax_block_sz = 1 << (((intfromchars(traxstat) >> 8) & 0x1f) - 2);
	cmd_ctx->trax_block_sz *= 4;
	LOG_INFO("stat=%0x ctrl=%0x", intfromchars(traxstat), intfromchars(traxctl));
	LOG_INFO("memadrstart=%x memadrend=%x traxadr=%x memsz=%x", intfromchars(memadrstart), intfromchars(memadrend), intfromchars(adr), cmd_ctx->trax_block_sz);

	LOG_INFO("Total trace memory: %d bytes", cmd_ctx->trax_block_sz);

	cmd_ctx->trax_block_data = malloc(cmd_ctx->trax_block_sz);
	if (!cmd_ctx->trax_block_data) {
		LOG_ERROR("Failed to alloc trace buffer %d bytes!", cmd_ctx->trax_block_sz);
		return ERROR_FAIL;
	}

	cmd_ctx->fout = open(argv[0], O_WRONLY|O_CREAT|O_TRUNC, 0666);
	if (cmd_ctx->fout <= 0) {
		LOG_ERROR("Failed to open file %s", argv[0]);
		free(cmd_ctx->trax_block_data);
		return ERROR_FAIL;
	}


	if (duration_start(&cmd_ctx->idle_time) != 0) {
		LOG_ERROR("Failed to start idle time measurement!");
		close(cmd_ctx->fout);
		free(cmd_ctx->trax_block_data);
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

static int esp108_cmd_apptrace_ctx_cleanup(struct apptrace_cmd_ctx *cmd_ctx)
{
	if (cmd_ctx->fout > 0) {
		close(cmd_ctx->fout);
	}
	if (cmd_ctx->trax_block_data) {
		free(cmd_ctx->trax_block_data);
	}
	return ERROR_OK;
}

static int esp108_apptrace_poll(void *priv)
{
	struct apptrace_cmd_ctx *at_cmd_ctx = (struct apptrace_cmd_ctx *)priv;
	int res = ERROR_OK;
	struct target *fired_target, *not_fired_target;
	uint32_t block_id, data_len = 0;
	struct duration blk_read_time;

	if (!at_cmd_ctx->running)
		return ERROR_FAIL;

	for (int k = 0; k < 2; k++) {
		fired_target = get_target_by_num(k);
		if (!fired_target)
			continue;
		not_fired_target = get_target_by_num(k ? 0 : 1);
		if (fired_target->state != TARGET_RUNNING || not_fired_target->state != TARGET_RUNNING) {
			return ERROR_OK; // do not poll if any of targets is not running
		}
		res = esp108_apptrace_read_data_len(fired_target, &block_id, &data_len);
		if (res != ERROR_OK) {
			at_cmd_ctx->running = 0;
			LOG_ERROR("Failed to read trace block len (%d)!", res);
			return res;
		}
		if (data_len != 0) {
			//LOG_USER("Fired target %s len %x stat %x", fired_target->cmd_name, data_len, stat);
			if (data_len > at_cmd_ctx->trax_block_sz) {
				at_cmd_ctx->running = 0;
				LOG_ERROR("Too large block size %d!", data_len);
				return ERROR_FAIL;
			}
			if (at_cmd_ctx->tot_len == 0) {
				if (duration_start(&at_cmd_ctx->read_time) != 0) {
					at_cmd_ctx->running = 0;
					LOG_ERROR("Failed to start trace read time measurement!");
					return ERROR_FAIL;
				}
			}
			// read block
			if (duration_start(&blk_read_time) != 0) {
				at_cmd_ctx->running = 0;
				LOG_ERROR("Failed to start block read time measurement!");
				return ERROR_FAIL;
			}
			res = esp108_apptrace_read_data(fired_target, at_cmd_ctx->trax_block_sz, at_cmd_ctx->trax_block_data, block_id, NULL);
			if (res != ERROR_OK) {
				at_cmd_ctx->running = 0;
				LOG_ERROR("Failed to read trace data (%d)!", res);
				return res;
			}
			if (duration_measure(&blk_read_time) != 0) {
				LOG_ERROR("Failed to measure block read time!");
				return ERROR_FAIL;
			}
			res = esp108_apptrace_write_data_len(not_fired_target, block_id, 0, 1);
			if (res != ERROR_OK) {
				at_cmd_ctx->running = 0;
				LOG_ERROR("Failed to clear trace len on other CPU (%d)!", res);
				return res;
			}

			float brt = duration_elapsed(&blk_read_time);
			if (brt > at_cmd_ctx->stats.max_blk_read_time)
				at_cmd_ctx->stats.max_blk_read_time = brt;
			if (brt < at_cmd_ctx->stats.min_blk_read_time)
				at_cmd_ctx->stats.min_blk_read_time = brt;

			for (uint32_t i = 0; i < data_len;) {
				struct apptrace_data_hdr *hdr = (struct apptrace_data_hdr *)&at_cmd_ctx->trax_block_data[i];
				if (hdr->block_sz != hdr->wr_sz) {
					LOG_ERROR("Incomplete block sz %u, wr %u", hdr->block_sz, hdr->wr_sz);
					at_cmd_ctx->stats.incompl_blocks++;
					at_cmd_ctx->stats.lost_bytes += hdr->block_sz - hdr->wr_sz;
				}
				if (at_cmd_ctx->tot_len + hdr->block_sz > at_cmd_ctx->skip_len) {
					uint32_t wr_idx = 0, wr_chunk_len = hdr->block_sz;
					if (at_cmd_ctx->tot_len < at_cmd_ctx->skip_len) {
						wr_chunk_len = (at_cmd_ctx->tot_len + wr_chunk_len) - at_cmd_ctx->skip_len;
						wr_idx = at_cmd_ctx->skip_len - at_cmd_ctx->tot_len;
					}
					if (at_cmd_ctx->tot_len + wr_chunk_len > at_cmd_ctx->max_len)
						wr_chunk_len -= (at_cmd_ctx->tot_len + wr_chunk_len - at_cmd_ctx->skip_len) - at_cmd_ctx->max_len;
					if (wr_chunk_len > 0) {
						if (write(at_cmd_ctx->fout, (uint8_t *)(hdr + 1) + wr_idx, wr_chunk_len) != wr_chunk_len) {
							at_cmd_ctx->running = 0;
							LOG_ERROR("Failed to write %u bytes to out file!", wr_chunk_len);
							return ERROR_FAIL;
						}
					}
					at_cmd_ctx->tot_len += wr_chunk_len;
				}
				else {
					at_cmd_ctx->tot_len += hdr->block_sz;
				}
				i += sizeof(struct apptrace_data_hdr) + hdr->block_sz;
			}
			LOG_INFO("%u", at_cmd_ctx->tot_len);
			if ((at_cmd_ctx->tot_len > at_cmd_ctx->skip_len) && (at_cmd_ctx->tot_len - at_cmd_ctx->skip_len >= at_cmd_ctx->max_len)) {
				at_cmd_ctx->running = 0;
				if (duration_measure(&at_cmd_ctx->read_time) != 0) {
					LOG_ERROR("Failed to stop trace read time measurement!");
					return ERROR_FAIL;
				}
			}
			break;
		}
	}
	if (data_len != 0) {
		if (duration_start(&at_cmd_ctx->idle_time) != 0) {
			at_cmd_ctx->running = 0;
			LOG_ERROR("Failed to start idle time measurement!");
			return ERROR_FAIL;
		}
	}
	else {
		if (duration_measure(&at_cmd_ctx->idle_time) != 0) {
			at_cmd_ctx->running = 0;
			LOG_ERROR("Failed to measure idle time!");
			return ERROR_FAIL;
		}
		if (duration_elapsed(&at_cmd_ctx->idle_time) >= at_cmd_ctx->stop_tmo) {
			at_cmd_ctx->running = 0;
			LOG_ERROR("Data timeout!");
			return ERROR_FAIL;
		}
	}
	return res;
}

static void esp108_cmd_apptrace_print_stats(struct command_context *ctx, struct apptrace_cmd_ctx *at_ctx)
{
	command_print(ctx, "Tracing is %s. Size is %u of %u @ %f KB/s", !at_ctx->running ? "STOPPED" : "RUNNING",
			at_ctx->tot_len > at_ctx->skip_len ? at_ctx->tot_len - at_ctx->skip_len : 0,
					at_ctx->max_len, duration_kbps(&at_ctx->read_time, at_ctx->tot_len));
	command_print(ctx, "Data: blocks incomplete %u, lost bytes: %u", at_ctx->stats.incompl_blocks, at_ctx->stats.lost_bytes);
	command_print(ctx, "TRAX: block read time [%f..%f] ms", 1000*at_ctx->stats.min_blk_read_time, 1000*at_ctx->stats.max_blk_read_time);
}

// FIXME: define handler manually, otherwise complier complanis about unused function
//COMMAND_HANDLER(esp108_cmd_apptrace)
int esp108_cmd_apptrace(struct command_invocation *cmd)
{
	static struct apptrace_cmd_ctx s_at_cmd_ctx;
	struct target *target = get_current_target(CMD_CTX);
	int res = ERROR_OK;

	if (CMD_ARGC < 1) {
		command_print(CMD_CTX, "Action missed!");
		return ERROR_FAIL;
	}

	if (strcmp(CMD_ARGV[0], "start") == 0) {
		// [start outfile [trace_size [stop_tmo [skip_size [poll_period [wait4halt]]]]]] | [stop] | [status]
		// init cmd context
		res = esp108_cmd_apptrace_ctx_init(&s_at_cmd_ctx, target, &CMD_ARGV[1], CMD_ARGC-1);
		if (res != ERROR_OK) {
			command_print(CMD_CTX, "Failed to init cmd ctx (%d)!", res);
			return res;
		}
		s_at_cmd_ctx.running = 0;
		if (s_at_cmd_ctx.wait4halt) {
			res = esp108_cmd_apptrace_wait4halt();
			if (res != ERROR_OK) {
				LOG_ERROR("Failed to wait for halt target (%d)!", res);
				return res;
			}
		}
		res = esp108_cmd_apptrace_connect_targets(1);
		if (res != ERROR_OK) {
			command_print(CMD_CTX, "Failed to connect to targets (%d)!", res);
			return res;
		}
		s_at_cmd_ctx.running = 1;
		// openocd timer_callback min period is 1 ms, if we need to poll target for trace data more frequently polling loop will be used
		if (s_at_cmd_ctx.poll_period >= 1) {
			res = target_register_timer_callback(esp108_apptrace_poll, s_at_cmd_ctx.poll_period, 1, &s_at_cmd_ctx);
			if (res != ERROR_OK) {
				command_print(CMD_CTX, "Failed to register target timer handler (%d)!", res);
				return res;
			}
		}
		else {
			/* check for exit signal and comand completion */
			while (!shutdown_openocd && s_at_cmd_ctx.running) {
				res = esp108_apptrace_poll(&s_at_cmd_ctx);
				if (res != ERROR_OK) {
					command_print(CMD_CTX, "Failed to poll target for trace data (%d)!", res);
					break;
				}
				/* let registered timer callbacks to run */
				target_call_timer_callbacks();
			}
			res = esp108_cmd_apptrace_connect_targets(0);
			if (res != ERROR_OK) {
				command_print(CMD_CTX, "Failed to disconnect targets (%d)!", res);
				return res;
			}
			esp108_cmd_apptrace_print_stats(CMD_CTX, &s_at_cmd_ctx);
			res = esp108_cmd_apptrace_ctx_cleanup(&s_at_cmd_ctx);
			if (res != ERROR_OK) {
				command_print(CMD_CTX, "Failed to cleanup cmd ctx (%d)!", res);
			}
		}
	}
	else if (strcmp(CMD_ARGV[0], "stop") == 0) {
		if (s_at_cmd_ctx.running) {
			s_at_cmd_ctx.running = 0;
			res = target_unregister_timer_callback(esp108_apptrace_poll, &s_at_cmd_ctx);
			if (res != ERROR_OK) {
				command_print(CMD_CTX, "Failed to unregister target timer handler (%d)!", res);
			}
			res = esp108_cmd_apptrace_connect_targets(0);
			if (res != ERROR_OK) {
				command_print(CMD_CTX, "Failed to disconnect targets (%d)!", res);
			}
			esp108_cmd_apptrace_print_stats(CMD_CTX, &s_at_cmd_ctx);
			res = esp108_cmd_apptrace_ctx_cleanup(&s_at_cmd_ctx);
			if (res != ERROR_OK) {
				command_print(CMD_CTX, "Failed to cleanup cmd ctx (%d)!", res);
			}
		}
	}
	else if (strcmp(CMD_ARGV[0], "status") == 0) {
		if (s_at_cmd_ctx.running && duration_measure(&s_at_cmd_ctx.read_time) != 0) {
			LOG_ERROR("Failed to measure trace read time!");
		}
		esp108_cmd_apptrace_print_stats(CMD_CTX, &s_at_cmd_ctx);
	}
	else if (strcmp(CMD_ARGV[0], "dump") == 0) {
		// [dump outfile] - post-mortem dump without connection to targets
		res = esp108_cmd_apptrace_ctx_init(&s_at_cmd_ctx, target, &CMD_ARGV[1], CMD_ARGC-1);
		if (res != ERROR_OK) {
			command_print(CMD_CTX, "Failed to init cmd ctx (%d)!", res);
			return res;
		}
		s_at_cmd_ctx.stop_tmo = 0.01; // use small stop tmo
		s_at_cmd_ctx.running = 1;
		/* check for exit signal and comand completion */
		while (!shutdown_openocd && s_at_cmd_ctx.running) {
			res = esp108_apptrace_poll(&s_at_cmd_ctx);
			if (res != ERROR_OK) {
				command_print(CMD_CTX, "Failed to poll target for trace data (%d)!", res);
				break;
			}
			/* let registered timer callbacks to run */
			target_call_timer_callbacks();
		}
		esp108_cmd_apptrace_print_stats(CMD_CTX, &s_at_cmd_ctx);
		res = esp108_cmd_apptrace_ctx_cleanup(&s_at_cmd_ctx);
		if (res != ERROR_OK) {
			command_print(CMD_CTX, "Failed to cleanup cmd ctx (%d)!", res);
		}
	}
	else {
		command_print(CMD_CTX, "Invalid action '%s'!", CMD_ARGV[0]);
	}

	return res;
}
