/***************************************************************************
 *   ESP108 application tracing module for OpenOCD                         *
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

// How It Works
// ************

// 1. Components Overview
// ======================

// Xtensa has useful feature: TRAX debug module. It allows recording program execution flow at run-time without disturbing CPU.
// Exectution flow data are written to configurable Trace RAM block. Besides accessing Trace RAM itself TRAX module also allows to read/write
// trace memory via its registers by means of JTAG, APB or ERI transactions.
// ESP32 has two Xtensa cores with separate TRAX modules on them and provides two special memory regions to be used as trace memory.
// Chip allows muxing access to those trace memory blocks in such a way that while one block is accessed by CPUs another one can be accessed by host
// by means of reading/writing TRAX registers via JTAG. Blocks muxing is configurable at run-time and allows switching trace memory blocks between
// accessors in round-robin fashion so they can read/write separate memory blocks without disturbing each other.
// This moduile implements application tracing feature based on above mechanisms. It allows to transfer arbitrary user data to/from
// host via JTAG with minimal impact on system performance. This module is implied to be used in the following tracing scheme.

//														  ------>------											----- (host components) -----
//														  |			  |											|							|
// -------------------	 -----------------------	 -----------------------	 ----------------	 ------		---------	-----------------
// |trace data source|-->|target tracing module|<--->|TRAX_MEM0 | TRAX_MEM1|---->|TRAX_DATA_REGS|<-->|JTAG|<--->|OpenOCD|-->|trace data sink|
// -------------------	 -----------------------	 -----------------------	 ----------------	 ------		---------	-----------------
//								   |					  |			  |								   |
//								   |					  ------<------			 ----------------	   |
//								   |<------------------------------------------->|TRAX_CTRL_REGS|<---->|
//																				 ----------------

// In general tracing goes in the following way. User aplication requests tracing module to send some data by calling esp_apptrace_buffer_get(),
// moduile allocates necessary buffer in current input trace block. Then user fills received buffer with data and calls esp_apptrace_buffer_put().
// When current input trace block is filled with app data it is exposed to host and the second block becomes input one and buffer filling restarts.
// While target application fills one TRAX block host reads another one via JTAG.
// This module also allows communication in the opposite direction: from host to target. As it was said ESP32 and host can access different TRAX blocks
// simultaneously, so while target writes trace data to one block host can write its own data (e.g. tracing commands) to another one then when
// blocks are switched host receives trace data and target receives data written by host application. Target user application can read host data
// by calling esp_apptrace_read() API.
// To control buffer switching and for other communication purposes this implementation uses some TRAX registers. It is safe since HW TRAX tracing
// can not be used along with application tracing feature so these registers are freely readable/writeable via JTAG from host and via ERI from ESP32 cores.
// Overhead of this implementation on target CPU is produced only by allocating/managing buffers and copying of data.
// On the host side special OpenOCD command must be used to read trace data.

// 2. TRAX Registers layout
// ========================

// This module uses two TRAX HW registers to communicate with host SW (OpenOCD).
//	- Control register uses TRAX_DELAYCNT as storage. Only lower 24 bits of TRAX_DELAYCNT are writable. Control register has the following bitfields:
//	 | 31..XXXXXX..24 | 23 .(host_connect). 23| 22..(block_id)..15 | 14..(block_len)..0 |
//	  14..0	 bits - actual length of user data in trace memory block. Target updates it every time it fills memory block and exposes it to host.
//					Host writes zero to this field when it finishes reading exposed block;
//	  21..15 bits - trace memory block transfer ID. Block counter. It can overflow. Updated by target, host should not modify it. Actually can be 2 bits;
//	  22	 bit  - 'host data present' flag. If set to one there is data from host, otherwise - no host data;
//	  23	 bit  - 'host connected' flag. If zero then host is not connected and tracing module works in post-mortem mode, otherwise in streaming mode;
// - Status register uses TRAX_TRIGGERPC as storage. If this register is not zero then currentlly CPU is changing TRAX registers and
//	 this register holds address of the instruction which application will execute when it finishes with those registers modifications.
//	 See 'Targets Connection' setion for details.

// 3. Modes of operation
// =====================

// This module supports two modes of operation:
//	- Post-mortem mode. This is the default mode. In this mode application tracing module does not check whether host has read all the data from block
//	  exposed to it and switches block in any case. The mode does not need host interaction for operation and so can be useful when only the latest
//	  trace data are necessary, e.g. for analyzing crashes. On panic the latest data from current input block are exposed to host and host can read them.
//	  It can happen that system panic occurs when there are very small amount of data not read by host yet (e.g. crash just after the TRAX block switch).
//	  In this case the previous 16KB of collected data will be dropped and host will see the latest, but very small piece of trace. It can be insufficient
//	  to diagnose the problem. To avoid such situations there is menuconfig option CONFIG_ESP32_APPTRACE_POSTMORTEM_FLUSH_TRAX_THRESH which controls
//	  the threshold for flushing data in case of panic.
//	- Streaming mode. Tracing module enters this mode when host connects to target and sets respective bits in control registers (per core).
//	  In this mode before switching the block tracing module waits for the host to read all the data from the previously exposed block.
//	  On panic tracing module also waits (timeout is configured via menuconfig via CONFIG_ESP32_APPTRACE_ONPANIC_HOST_FLUSH_TMO) for the host to read all data.

// 4. Communication Protocol
// =========================

// 4.1 Trace Memory Blocks
// -----------------------

// Communication is controlled via special register. Host periodically polls control register on each core to find out if there are any data avalable.
// When current input memory block is filled it is exposed to host and 'block_len' and 'block_id' fields are updated in the control register.
// Host reads new register value and according to it's value starts reading data from exposed block. Meanwhile target starts filling another trace block.
// When host finishes reading the block it clears 'block_len' field in control register indicating to the target that it is ready to accept the next one.
// If the host has some data to transfer to the target it writes them to trace memory block before clearing 'block_len' field. Then it sets
// 'host_data_present' bit and clears 'block_len' field in control register. Upon every block switch target checks 'host_data_present' bit and if it is set
// reads them to down buffer before writing any trace data to switched TRAX block.

// 4.2 User Data Chunks Level
// --------------------------

// Since trace memory block is shared between user data chunks and data copying is performed on behalf of the API user (in its normal context) in
// multithreading environment it can happen that task/ISR which copies data is preempted by another high prio task/ISR. So it is possible situation
// that task/ISR will fail to complete filling its data chunk before the whole trace block is exposed to the host. To handle such conditions tracing
// module prepends all user data chunks with header which contains allocated buffer size and actual data length within it. OpenOCD command
// which reads application traces reports error when it reads incompleted user data block.
// Data which are transfered from host to target are also prepended with such header.

// 4.3 Data Buffering
// ------------------

// It takes some time for the host to read TRAX memory block via JTAG. In streaming mode it can happen that target has filled its TRAX block, but host
// has not completed reading of the previous one yet. So in this case time critical tracing calls (which can not be delayed for too long time due to
// the lack of free memory in TRAX block) can be dropped. To avoid such scenarios tracing module implements data buffering. Buffered data will be sent
// to the host later when TRAX block switch occurs. The maximum size of the buffered data is controlled by menuconfig option
// CONFIG_ESP32_APPTRACE_PENDED_DATA_SIZE_MAX.

// 4.3 Target Connection/Disconnection
// -----------------------------------

// When host is going to start tracing in streaming mode it needs to put both ESP32 cores into initial state when 'host connected' bit is set
// on both cores. To accomplish this host halts both cores and sets this bit in TRAX registers. But target code can be halted in state when it has read control
// register but has not updated its value. To handle such situations target code indicates to the host that it is updating control register by writing
// non-zero value to status register. Actually it writes address of the instruction which it will execute when it finishes with
// the registers update. When target is halted during control register update host sets breakpoint at the address from status register and resumes CPU.
// After target code finishes with register update it is halted on breakpoint, host detects it and safely sets 'host connected' bit. When both cores
// are set up they are resumed. Tracing starts without further intrusion into CPUs work.
// When host is going to stop tracing in streaming mode it needs to disconnect targets. Disconnection process is done using the same algorithm
// as for connecting, but 'host connected' bits are cleared on ESP32 cores.

// 5. Data Procesing
// =================

// Target is polled for data periodically. Period depends on tracing command arguments (see the next section). When there are data available they are copied to allocated memory block
// and that block is added to the queue for processing by a separate thread. This is done in order to achive the highest possible polling rate and not to lose data due to delays caused
// by data processing algorithm. When tracing is stopped OpenOCD waits for all pendded memory blocks to be processed by the thread.

// 6. Application Tracing Commands
// ===============================

// 6.1 Application Specific Tracing
// --------------------------------

// Command usage:

//	 esp108 apptrace [start <options>] | [stop] | [status] | [dump <cores_num> <outfile>]

// Sub-commands:

//	   * start	- Start tracing (continuous streaming).
//	   * stop	- Stop tracing.
//	   * status - Get tracing status.
//	   * dump	- Dump all data from *HW UP BUFFER* (post-mortem dump).

// Start command syntax:

//	 start <outfile1> [outfile2] [poll_period [trace_size [stop_tmo [wait4halt [skip_size]]]]

//	   * outfile1	 - Path to file to save data from PRO CPU. This argument should have the following format: `file://path/to/file`.
//	   * outfile2	 - Path to file to save data from APP CPU. This argument should have the following format: `file://path/to/file`.
//	   * poll_period - Data polling period (in ms). If greater then 0 then command runs in non-blocking mode. By default 1 ms.
//	   * trace_size	 - Maximum size of data to collect (in bytes). Tracing is stopped after specified amount of data is received. By default -1 (trace size stop trigger is disabled).
//	   * stop_tmo	 - Idle timeout (in sec). Tracing is stopped if there is no data for specified period of time. By default -1 (disable this stop trigger).
//	   * wait4halt	 - If 0 start tracing immediately, otherwise command waits for the target to be halted (after reset, by breakpoint etc.) and then automatically resumes it and starts tracing. By default 0.
//	   * skip_size	 - Number of bytes to skip at the start. By default 0.

// 6.2 SystemView Compatible Tracing
// ---------------------------------

// Command usage:

//	 esp108 sysview [start <options>] | [stop] | [status]

// Sub-commands:

//	   * start	- Start tracing (continuous streaming).
//	   * stop	- Stop tracing.
//	   * status - Get tracing status.

// Start command syntax:

//	 start <outfile1> [outfile2] [poll_period [trace_size [stop_tmo]]]

//	   * outfile1	 - Path to file to save data from PRO CPU. This argument should have the following format: `file://path/to/file`.
//	   * outfile2	 - Path to file to save data from APP CPU. This argument should have the following format: `file://path/to/file`.
//	   * poll_period - Data polling period (in ms). If greater then 0 then command runs in non-blocking mode. By default 1 ms.
//	   * trace_size	 - Maximum size of data to collect (in bytes). Tracing is stopped after specified amount of data is received. By default -1 (trace size stop trigger is disabled).
//	   * stop_tmo	 - Idle timeout (in sec). Tracing is stopped if there is no data for specified period of time. By default -1 (disable this stop trigger).

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "target.h"
#include "time_support.h"
#include "list.h"

#include "esp108.h"
#include "esp108_dbg_regs.h"
#include "esp32.h"
#include "esp108_apptrace.h"
#include <pthread.h>

#define ESP_SYSVIEW_OPTIMIZE	1

// TRAX is disabled, so we use its registers for our own purposes
// | 31..XXXXXX..24 | 23 .(host_connect). 23 | 22 .(host_data). 22| 21..(block_id)..15 | 14..(block_len)..0 |
#define ESP_APPTRACE_TRAX_CTRL_REG			 NARADR_DELAYCNT
#define ESP_APPTRACE_TRAX_STAT_REG			 NARADR_TRIGGERPC

#define ESP_APPTRACE_BLOCK_LEN_MSK		   0x7FFFUL
#define ESP_APPTRACE_BLOCK_LEN(_l_)		   ((_l_) & ESP_APPTRACE_BLOCK_LEN_MSK)
#define ESP_APPTRACE_BLOCK_LEN_GET(_v_)	   ((_v_) & ESP_APPTRACE_BLOCK_LEN_MSK)
#define ESP_APPTRACE_BLOCK_ID_MSK		   0x7FUL
#define ESP_APPTRACE_BLOCK_ID(_id_)		   (((_id_) & ESP_APPTRACE_BLOCK_ID_MSK) << 15)
#define ESP_APPTRACE_BLOCK_ID_GET(_v_)	   (((_v_) >> 15) & ESP_APPTRACE_BLOCK_ID_MSK)
#define ESP_APPTRACE_HOST_DATA			   (1 << 22)
#define ESP_APPTRACE_HOST_CONNECT		   (1 << 23)

#define ESP_APPTRACE_USER_BLOCK_CORE(_v_)	((_v_) >> 15)
#define ESP_APPTRACE_USER_BLOCK_LEN(_v_)	((_v_) & ~(1 << 15))
// in SystemView mode core ID is passed in event ID field
#define ESP_SYSVIEW_USER_BLOCK_CORE(_v_)	(0) // not used
#define ESP_SYSVIEW_USER_BLOCK_LEN(_v_)		(_v_)

#define ESP_APPTRACE_USER_BLOCK_HDR_SZ		4
#if ESP_SYSVIEW_OPTIMIZE
#define ESP_SYSVIEW_USER_BLOCK_HDR_SZ		2
#else
#define ESP_SYSVIEW_USER_BLOCK_HDR_SZ		ESP_APPTRACE_USER_BLOCK_HDR_SZ
#endif

#define ESP_APPTRACE_CMD_MODE_GEN			0
#define ESP_APPTRACE_CMD_MODE_SYSVIEW		1
#define ESP_APPTRACE_CMD_MODE_SYNC			2

#define ESP_APPTRACE_TGT_STATE_TMO			5000
#define ESP_APPTRACE_TARGETS_NUM_MAX		2
#define ESP_APPTRACE_TIME_STATS_ENABLE		1
#define ESP_APPTRACE_BLOCKS_POOL_SZ			10

#define ESP_APPTRACE_FILE_CMD_FOPEN     0x0
#define ESP_APPTRACE_FILE_CMD_FCLOSE    0x1
#define ESP_APPTRACE_FILE_CMD_FWRITE    0x2
#define ESP_APPTRACE_FILE_CMD_FREAD     0x3
#define ESP_APPTRACE_FILE_CMD_FSEEK     0x4
#define ESP_APPTRACE_FILE_CMD_FTELL     0x5
#define ESP_APPTRACE_FILE_CMD_FTELL     0x5
#define ESP_APPTRACE_FILE_CMD_STOP      0x6 // indicates that there is no files to transfer

#define ESP_GCOV_FILES_MAX_NUM     		512

// grabbed from SystemView target sources
#define	  SYSVIEW_EVTID_NOP					0  // Dummy packet.
#define	  SYSVIEW_EVTID_OVERFLOW			1
#define	  SYSVIEW_EVTID_ISR_ENTER			2
#define	  SYSVIEW_EVTID_ISR_EXIT			3
#define	  SYSVIEW_EVTID_TASK_START_EXEC		4
#define	  SYSVIEW_EVTID_TASK_STOP_EXEC		5
#define	  SYSVIEW_EVTID_TASK_START_READY	6
#define	  SYSVIEW_EVTID_TASK_STOP_READY		7
#define	  SYSVIEW_EVTID_TASK_CREATE			8
#define	  SYSVIEW_EVTID_TASK_INFO			9
#define	  SYSVIEW_EVTID_TRACE_START			10
#define	  SYSVIEW_EVTID_TRACE_STOP			11
#define	  SYSVIEW_EVTID_SYSTIME_CYCLES		12
#define	  SYSVIEW_EVTID_SYSTIME_US			13
#define	  SYSVIEW_EVTID_SYSDESC				14
#define	  SYSVIEW_EVTID_USER_START			15
#define	  SYSVIEW_EVTID_USER_STOP			16
#define	  SYSVIEW_EVTID_IDLE				17
#define	  SYSVIEW_EVTID_ISR_TO_SCHEDULER	18
#define	  SYSVIEW_EVTID_TIMER_ENTER			19
#define	  SYSVIEW_EVTID_TIMER_EXIT			20
#define	  SYSVIEW_EVTID_STACK_INFO			21
#define	  SYSVIEW_EVTID_MODULEDESC			22

#define	  SYSVIEW_EVTID_INIT				24
#define	  SYSVIEW_EVTID_NAME_RESOURCE		25
#define	  SYSVIEW_EVTID_PRINT_FORMATTED		26
#define	  SYSVIEW_EVTID_NUMMODULES			27

#define	  SYSVIEW_SYNC_LEN					10

#define	  SYSVIEW_EVENT_ID_MAX			   (200)

#define SYSVIEW_ENCODE_U32(dest, val) {										\
								   uint8_t* sv_ptr;							\
								   uint32_t sv_data;						\
								   sv_ptr = dest;							\
								   sv_data = val;							\
								   while(sv_data > 0x7F) {					\
									 *sv_ptr++ = (uint8_t)(sv_data | 0x80); \
									 sv_data >>= 7;							\
								   };										\
								   *sv_ptr++ = (uint8_t)sv_data;			\
								   dest = sv_ptr;							\
								 };

typedef enum {
  SEGGER_SYSVIEW_COMMAND_ID_START = 1,
  SEGGER_SYSVIEW_COMMAND_ID_STOP,
  SEGGER_SYSVIEW_COMMAND_ID_GET_SYSTIME,
  SEGGER_SYSVIEW_COMMAND_ID_GET_TASKLIST,
  SEGGER_SYSVIEW_COMMAND_ID_GET_SYSDESC,
  SEGGER_SYSVIEW_COMMAND_ID_GET_NUMMODULES,
  SEGGER_SYSVIEW_COMMAND_ID_GET_MODULEDESC,
  // Extended commands: Commands >= 128 have a second parameter
  SEGGER_SYSVIEW_COMMAND_ID_GET_MODULE = 128
} SEGGER_SYSVIEW_COMMAND_ID;

struct esp108_apptrace_target_state {
	int			running;
	uint32_t	block_id;
	uint32_t	data_len;
};

struct esp_apptrace_target2host_hdr {
	union {
		struct {
			uint16_t	block_sz;
			uint16_t	wr_sz;
		} gen;
		struct {
#if ESP_SYSVIEW_OPTIMIZE
			uint8_t		block_sz;
			uint8_t	wr_sz;
#else
			uint16_t	block_sz;
			uint16_t	wr_sz;
#endif
		} sys_view;
	};
};

struct esp_apptrace_host2target_msg {
	struct esp_apptrace_host2target_hdr hdr;
	uint8_t							data[32];
};

struct esp_apptrace_cmd_stats {
	uint32_t		incompl_blocks;
	uint32_t		lost_bytes;
#if ESP_APPTRACE_TIME_STATS_ENABLE
	float			min_blk_read_time;
	float			max_blk_read_time;
	float			min_blk_proc_time;
	float			max_blk_proc_time;
#endif
};

struct esp_apptrace_dest_file_data {
	int fout;
};

typedef int (*esp_apptrace_dest_write_t)(void *priv, uint8_t *data, uint32_t size);
typedef int (*esp_apptrace_dest_cleanup_t)(void *priv);

struct esp_apptrace_dest {
	void *						priv;
	esp_apptrace_dest_write_t	write;
	esp_apptrace_dest_cleanup_t	clean;
};

struct esp_apptrace_cmd_ctx;

typedef int (*esp_apptrace_process_data_t)(struct esp_apptrace_cmd_ctx *ctx, int core_id, uint8_t *data, uint32_t data_len);

struct esp_apptrace_block {
	struct hlist_node	node;
	uint8_t	*			data;
	uint32_t			data_len;
};

struct esp_apptrace_cmd_ctx {
	volatile int					running;
	int								mode;
	struct target *					esp32_target;
	struct target *					cpus[ESP_APPTRACE_TARGETS_NUM_MAX];
	int								cores_num;
	pthread_mutex_t					trax_blocks_mux;
	struct hlist_head				free_trax_blocks;
	struct hlist_head				ready_trax_blocks;
	uint8_t *						trax_block_data;
	uint32_t						trax_block_sz;
	pthread_t						data_processor;
	esp_apptrace_process_data_t		process_data;
	float							stop_tmo;
	uint32_t						tot_len;
	uint32_t						raw_tot_len;
	struct esp_apptrace_cmd_stats	stats;
	struct duration					read_time;
	struct duration					idle_time;
	void *							cmd_priv;
};

struct esp_apptrace_cmd_data {
	struct esp_apptrace_dest		data_dests[ESP_APPTRACE_TARGETS_NUM_MAX];
	uint32_t						poll_period;
	uint32_t						sv_acc_time_delta;
	int								sv_last_core_id;
	uint32_t						max_len;
	uint32_t						skip_len;
	bool							wait4halt;
};

struct esp_gcov_cmd_data {
	FILE *		files[ESP_GCOV_FILES_MAX_NUM];
	uint32_t 	files_num;
	bool		wait4halt;
};

// need to check it when poll period is less then 1 ms in order to react on CTRL+C etc
extern int shutdown_openocd;

static int esp_sysview_process_data(struct esp_apptrace_cmd_ctx *ctx, int core_id, uint8_t *data, uint32_t data_len);
static int esp_gcov_process_data(struct esp_apptrace_cmd_ctx *ctx, int core_id, uint8_t *data, uint32_t data_len);
static void *esp_apptrace_data_processor(void *arg);
static int esp_apptrace_handle_trace_block(struct esp_apptrace_cmd_ctx *ctx, struct esp_apptrace_block *block);
static int esp_apptrace_cmd_ctx_cleanup(struct esp_apptrace_cmd_ctx *cmd_ctx);

/*********************************************************************
*                       ESP108 Specific Functions
**********************************************************************/

int esp108_apptrace_read_data_len(struct target *target, uint32_t *block_id, uint32_t *len)
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

int esp108_apptrace_read_data(struct target *target, uint32_t size, uint8_t *buffer, uint32_t block_id, int ack, struct duration *dur)
{
	int res = 0;
	uint32_t i, rd_sz = size;
    uint8_t unal_bytes[4];
	uint32_t tmp = ESP_APPTRACE_HOST_CONNECT | ESP_APPTRACE_BLOCK_ID(block_id) | ESP_APPTRACE_BLOCK_LEN(0);

	LOG_DEBUG("Read data on target (%s)", target_name(target));
    if (size & 0x3UL) {
        rd_sz = (size + 0x3UL) & ~0x3UL;
    }
	esp108_queue_nexus_reg_write(target, NARADR_TRAXADDR, (ESP32_TRACEMEM_BLOCK_SZ-rd_sz)/4);
    if (size & 0x3UL) {
        esp108_queue_nexus_reg_read(target, NARADR_TRAXDATA, unal_bytes);
    }
	for (i = size/4; i > 0; i--) {
		esp108_queue_nexus_reg_read(target, NARADR_TRAXDATA, &buffer[(i-1)*4]);
	}
	if (ack) {
		LOG_DEBUG("Ack block %d target (%s)!", block_id, target_name(target));
		esp108_queue_nexus_reg_write(target, ESP_APPTRACE_TRAX_CTRL_REG, tmp);
	}
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

    if (size & 0x3UL) {
        // copy the last unaligned bytes
        memcpy(buffer + size - (size & 0x3UL), unal_bytes, size & 0x3UL);
    }

	return ERROR_OK;
}

int esp108_apptrace_write_buffs(struct target *target, uint32_t bufs_num, uint32_t buf_sz[], const uint8_t *bufs[],
                                uint32_t block_id, int ack, int data)
{
    int res = ERROR_OK;
    uint32_t tmp = ESP_APPTRACE_HOST_CONNECT | (data ? ESP_APPTRACE_HOST_DATA : 0) | ESP_APPTRACE_BLOCK_ID(block_id) | ESP_APPTRACE_BLOCK_LEN(0);
    uint32_t wr_dw = 0, total_sz = 0, cached_bytes = 0;
    union {
        uint8_t data8[4];
        uint32_t data32;
    } dword_cache;

    dword_cache.data32 = 0;
    for (uint32_t i = 0; i < bufs_num; i++) {
        total_sz += buf_sz[i];
    }
    if (total_sz & 0x3UL) {
        cached_bytes = 4 - (total_sz & 0x3UL);
        total_sz = (total_sz + 0x3UL) & ~0x3UL;
    }
    esp108_queue_nexus_reg_write(target, NARADR_TRAXADDR, (ESP32_TRACEMEM_BLOCK_SZ-total_sz)/4);
    for (uint32_t i = bufs_num; i > 0; i--) {
        uint32_t bs = buf_sz[i-1];
        const uint8_t *cur_buf = bufs[i-1];
        // if there are cached bytes from the previous buffer, combine them with the last from the current buffer
        if (cached_bytes) {
            memcpy(dword_cache.data8, &cur_buf[bs - (4 - cached_bytes)], 4 - cached_bytes);
            LOG_DEBUG("Write PADDED DWORD[%d] %x", wr_dw, dword_cache.data32);
            esp108_queue_nexus_reg_write(target, NARADR_TRAXDATA, dword_cache.data32);
            bs -= 4 - cached_bytes;
            cached_bytes = 0;
            wr_dw++;
        }
        // write full dwords
        for (uint32_t k = bs; k >= 4; k -= 4) {
            uint32_t dword;
            memcpy(&dword, &cur_buf[k-4], sizeof(dword));
            LOG_DEBUG("Write DWORD[%d] %x", wr_dw, dword);
            esp108_queue_nexus_reg_write(target, NARADR_TRAXDATA, dword);
            wr_dw++;
        }
        // if there are bytes to be cached
        if (bs & 0x3UL) {
            cached_bytes = bs & 0x3UL;
            memcpy(&dword_cache.data8[4-cached_bytes], cur_buf, cached_bytes);
        }
    }
    if (ack) {
        LOG_DEBUG("Ack block %d on target (%s)!", block_id, target_name(target));
        esp108_queue_nexus_reg_write(target, ESP_APPTRACE_TRAX_CTRL_REG, tmp);
    }
    esp108_queue_tdi_idle(target);
    res = jtag_execute_queue();
    if (res != ERROR_OK) {
        LOG_ERROR("Failed to exec JTAG queue!");
        return res;
    }

    return ERROR_OK;
}

int esp108_apptrace_write_ctrl_reg(struct target *target, uint32_t block_id, uint32_t len, int conn, int data)
{
	int res = ERROR_OK;
	uint32_t tmp = (conn ? ESP_APPTRACE_HOST_CONNECT : 0) | (data ? ESP_APPTRACE_HOST_DATA : 0) | ESP_APPTRACE_BLOCK_ID(block_id) | ESP_APPTRACE_BLOCK_LEN(len);

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

/*********************************************************************
*                       ESP32/108 Common Functions
**********************************************************************/

static uint32_t esp_apptrace_get_cores_num(struct target *target)
{
	int res;
	uint32_t cores_num = 1;
	bool resume = false;

	if (target->state != TARGET_HALTED) {
		res = target_halt(target);
		if (res != ERROR_OK) {
			LOG_ERROR("Failed to halt target (%d)!", res);
			return 0;
		}
		res = target_wait_state(target, TARGET_HALTED, ESP_APPTRACE_TGT_STATE_TMO);
		if (res != ERROR_OK) {
			LOG_ERROR("Failed to wait for target '%s' to halt: state %d (%d)!", target_name(target), target->state, res);
			return 0;
		}
		resume = true;
	}

	uint32_t appcpu_ctrl = 0;
	res = target_read_memory(target, ESP32_DPORT_APPCPU_CTRL_B_REG, sizeof(uint32_t), 1, (uint8_t *)&appcpu_ctrl);
	if (res != ERROR_OK) {
		LOG_ERROR("Failed to read target memory (%d)!", res);
		return 0;
	}
	if (appcpu_ctrl & ESP32_DPORT_APPCPU_CLKGATE_EN) {
		cores_num++;
	}

	if (resume) {
		// continue execution
		res = target_resume(target, 1, 0, 1, 0);
		if (res != ERROR_OK) {
			LOG_ERROR("Failed to resume target (%d)!", res);
			return 0;
		}
	}

	return cores_num;
}

static int esp_apptrace_file_dest_write(void *priv, uint8_t *data, uint32_t size)
{
	struct esp_apptrace_dest_file_data *dest_data = (struct esp_apptrace_dest_file_data *)priv;

	if (write(dest_data->fout, data, size) != (ssize_t)size) {
		LOG_ERROR("Failed to write %u bytes to out file!", size);
		return ERROR_FAIL;
	}
	return ERROR_OK;
}

static int esp_apptrace_file_dest_cleanup(void *priv)
{
	struct esp_apptrace_dest_file_data *dest_data = (struct esp_apptrace_dest_file_data *)priv;

	if (dest_data->fout > 0) {
		close(dest_data->fout);
	}
	free(dest_data);
	return ERROR_OK;
}

static int esp_apptrace_file_dest_init(struct esp_apptrace_dest *dest, const char *dest_name)
{
	struct esp_apptrace_dest_file_data *dest_data = malloc(sizeof(struct esp_apptrace_dest_file_data));
	if (!dest_data) {
		LOG_ERROR("Failed to alloc mem for file dest!");
		return ERROR_FAIL;
	}
	memset(dest_data, 0, sizeof(struct esp_apptrace_dest_file_data));

	LOG_INFO("Open file %s", dest_name);
	dest_data->fout = open(dest_name, O_WRONLY|O_CREAT|O_TRUNC|O_BINARY, 0666);
	if (dest_data->fout <= 0) {
		LOG_ERROR("Failed to open file %s", dest_name);
		return ERROR_FAIL;
	}

	dest->priv = dest_data;
	dest->write = esp_apptrace_file_dest_write;
	dest->clean = esp_apptrace_file_dest_cleanup;

	return ERROR_OK;
}

static int esp_apptrace_dest_init(struct esp_apptrace_dest dest[], const char *dest_paths[], int max_dests)
{
	int res = ERROR_OK, i;

	for (i = 0; i < max_dests; i++) {
		if (strncmp(dest_paths[i], "file://", 7) == 0) {
			res = esp_apptrace_file_dest_init(&dest[i], &dest_paths[i][7]);
			if (res != ERROR_OK) {
				LOG_ERROR("Failed to init destination '%s'!", dest_paths[i]);
				return 0;
			}
		}
		else {
			break;
		}
	}

	return i;
}

static int esp_apptrace_dest_cleanup(struct esp_apptrace_dest dest[], int max_dests)
{
	for (int i = 0; i < max_dests; i++) {
		if (dest[i].clean) {
			return dest[i].clean(dest[i].priv);
		}
	}
	return ERROR_OK;
}

static void esp_apptrace_blocks_pool_cleanup(struct esp_apptrace_cmd_ctx *ctx)
{
	struct esp_apptrace_block *cur;
	struct hlist_node *pos, *tmp;
	hlist_for_each_entry_safe(cur, pos, tmp, &ctx->free_trax_blocks, node) {
		if (cur) {
			hlist_del(&cur->node);
			if (cur->data) {
				free(cur->data);
			}
			free(cur);
		}
	}
	hlist_for_each_entry_safe(cur, pos, tmp, &ctx->ready_trax_blocks, node) {
		if (cur) {
			hlist_del(&cur->node);
			if (cur->data) {
				free(cur->data);
			}
			free(cur);
		}
	}
}

static struct esp_apptrace_block *esp_apptrace_free_block_get(struct esp_apptrace_cmd_ctx *ctx)
{
	struct esp_apptrace_block *block = NULL;

	int res = pthread_mutex_lock(&ctx->trax_blocks_mux);
	if (res == 0) {
		if (!hlist_empty(&ctx->free_trax_blocks)) {
			//get first
			block = hlist_entry(ctx->free_trax_blocks.first, struct esp_apptrace_block, node);
			hlist_del(&block->node);
		}
		res = pthread_mutex_unlock(&ctx->trax_blocks_mux);
		if (res) {
			LOG_ERROR("Failed to unlock blocks pool (%d)!", res);
		}
	}

	return block;
}

static int esp_apptrace_ready_block_put(struct esp_apptrace_cmd_ctx *ctx, struct esp_apptrace_block *block)
{
	int res;

	res = pthread_mutex_lock(&ctx->trax_blocks_mux);
	if (res == 0) {
		// add to ready blocks list
		INIT_HLIST_NODE(&block->node);
		hlist_add_head(&block->node, &ctx->ready_trax_blocks);
		res = pthread_mutex_unlock(&ctx->trax_blocks_mux);
		if (res) {
			LOG_ERROR("Failed to unlock blocks pool (%d)!", res);
			res = ERROR_FAIL;
		}
		res = ERROR_OK;
	}
	else {
		LOG_ERROR("Failed to lock blocks pool (%d)!", res);
		res = ERROR_FAIL;
	}

	return res;
}

static struct esp_apptrace_block *esp_apptrace_ready_block_get(struct esp_apptrace_cmd_ctx *ctx)
{
	struct esp_apptrace_block *block = NULL;

	if (pthread_mutex_trylock(&ctx->trax_blocks_mux) == 0) {
		if (!hlist_empty(&ctx->ready_trax_blocks)) {
			struct hlist_node *tmp;
			// find to the last
			hlist_for_each_entry(block, tmp, &ctx->ready_trax_blocks, node);
			// remove it from ready list
			hlist_del(&block->node);
		}
		int res = pthread_mutex_unlock(&ctx->trax_blocks_mux);
		if (res) {
			LOG_ERROR("Failed to unlock blocks pool (%d)!", res);
		}
	}

	return block;
}

static int esp_apptrace_block_free(struct esp_apptrace_cmd_ctx *ctx, struct esp_apptrace_block *block)
{
	int res;

	res = pthread_mutex_lock(&ctx->trax_blocks_mux);
	if (res == 0) {
		// add to free blocks list
		INIT_HLIST_NODE(&block->node);
		hlist_add_head(&block->node, &ctx->free_trax_blocks);
		res = pthread_mutex_unlock(&ctx->trax_blocks_mux);
		if (res) {
			LOG_ERROR("Failed to unlock blocks pool (%d)!", res);
			res = ERROR_FAIL;
		}
		res = ERROR_OK;
	}
	else {
		LOG_ERROR("Failed to lock blocks pool (%d)!", res);
		res = ERROR_FAIL;
	}

	return res;
}

static int esp_apptrace_wait_pended_blocks(struct esp_apptrace_cmd_ctx *ctx)
{
	int i = 0;
	while (!hlist_empty(&ctx->ready_trax_blocks)) {
		alive_sleep(100);
		if (i++ == 50) {
			return ERROR_FAIL;
		}
	}

	return ERROR_OK;
}

static int esp_apptrace_cmd_ctx_init(struct target *target, struct esp_apptrace_cmd_ctx *cmd_ctx, int mode)
{
	uint8_t traxstat[4], memadrstart[4], memadrend[4], adr[4], traxctl[4];
	int res;

	memset(cmd_ctx, 0, sizeof(struct esp_apptrace_cmd_ctx));

	cmd_ctx->data_processor = (pthread_t)-1;
	cmd_ctx->stop_tmo = -1.0; // infinite
	cmd_ctx->mode = mode;
	cmd_ctx->cores_num = esp_apptrace_get_cores_num(target);
	cmd_ctx->esp32_target = cmd_ctx->cores_num == 2 ? target : NULL;

	for (int i = 0; i < cmd_ctx->cores_num; i++) {
		if (cmd_ctx->esp32_target) {
			struct esp32_common *esp32 = (struct esp32_common *)target->arch_info;
			cmd_ctx->cpus[i] = esp32->esp32_targets[i];
		} else {
			cmd_ctx->cpus[i] = get_target_by_num(i);
		}
		if (!cmd_ctx->cpus[i]) {
			LOG_ERROR("Failed to get CPU%d target!", i);
			return ERROR_FAIL;
		}
	}

	esp108_queue_nexus_reg_read(cmd_ctx->cpus[0], NARADR_TRAXSTAT, traxstat);
	esp108_queue_nexus_reg_read(cmd_ctx->cpus[0], NARADR_TRAXCTRL, traxctl);
	esp108_queue_nexus_reg_read(cmd_ctx->cpus[0], NARADR_MEMADDRSTART, memadrstart);
	esp108_queue_nexus_reg_read(cmd_ctx->cpus[0], NARADR_MEMADDREND, memadrend);
	esp108_queue_nexus_reg_read(cmd_ctx->cpus[0], NARADR_TRAXADDR, adr);
	esp108_queue_tdi_idle(cmd_ctx->cpus[0]);
	res = jtag_execute_queue();
	if (res) {
		LOG_ERROR("Failed to read TRAX config (%d)!", res);
		return res;
	}

	cmd_ctx->trax_block_sz = 1 << (((intfromchars(traxstat) >> 8) & 0x1f) - 2);
	cmd_ctx->trax_block_sz *= 4;
	LOG_DEBUG("stat=%0x ctrl=%0x", intfromchars(traxstat), intfromchars(traxctl));
	LOG_DEBUG("memadrstart=%x memadrend=%x traxadr=%x memsz=%x", intfromchars(memadrstart), intfromchars(memadrend), intfromchars(adr), cmd_ctx->trax_block_sz);
	LOG_INFO("Total trace memory: %d bytes", cmd_ctx->trax_block_sz);

	INIT_HLIST_HEAD(&cmd_ctx->ready_trax_blocks);
	INIT_HLIST_HEAD(&cmd_ctx->free_trax_blocks);
	for (int i = 0; i < ESP_APPTRACE_BLOCKS_POOL_SZ; i++) {
		struct esp_apptrace_block *block = malloc(sizeof(struct esp_apptrace_block));
		if (!block) {
			LOG_ERROR("Failed to alloc trace buffer entry!");
			esp_apptrace_blocks_pool_cleanup(cmd_ctx);
			return ERROR_FAIL;
		}
		block->data = malloc(cmd_ctx->trax_block_sz);
		if (!block->data) {
			free(block);
			LOG_ERROR("Failed to alloc trace buffer %d bytes!", cmd_ctx->trax_block_sz);
			esp_apptrace_blocks_pool_cleanup(cmd_ctx);
			return ERROR_FAIL;
		}
		INIT_HLIST_NODE(&block->node);
		hlist_add_head(&block->node, &cmd_ctx->free_trax_blocks);
	}

	cmd_ctx->running = 1;

	if (cmd_ctx->mode != ESP_APPTRACE_CMD_MODE_SYNC) {
		res = pthread_mutex_init(&cmd_ctx->trax_blocks_mux, NULL);
		if (res) {
			LOG_ERROR("Failed to blocks pool mux (%d)!", res);
			esp_apptrace_blocks_pool_cleanup(cmd_ctx);
			return ERROR_FAIL;
		}
		res = pthread_create(&cmd_ctx->data_processor, NULL, esp_apptrace_data_processor, cmd_ctx);
		if (res) {
			LOG_ERROR("Failed to start trace data processor thread (%d)!", res);
			cmd_ctx->data_processor = (pthread_t)-1;
			pthread_mutex_destroy(&cmd_ctx->trax_blocks_mux);
			esp_apptrace_blocks_pool_cleanup(cmd_ctx);
			return ERROR_FAIL;
		}
	}

#if ESP_APPTRACE_TIME_STATS_ENABLE
	cmd_ctx->stats.min_blk_read_time = 1000000.0;
	cmd_ctx->stats.min_blk_proc_time = 1000000.0;
#endif
	if (duration_start(&cmd_ctx->idle_time) != 0) {
		LOG_ERROR("Failed to start idle time measurement!");
		esp_apptrace_cmd_ctx_cleanup(cmd_ctx);
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

static int esp_apptrace_cmd_ctx_cleanup(struct esp_apptrace_cmd_ctx *cmd_ctx)
{
	if (cmd_ctx->data_processor != (pthread_t)-1) {
		void *thr_res;
		int res = pthread_join(cmd_ctx->data_processor, (void *)&thr_res);
		if (res) {
			LOG_ERROR("Failed to join trace data processor thread (%d)!", res);
		}
		else {
			LOG_INFO("Trace data processor thread exited with %ld", (long)thr_res);
		}
		pthread_mutex_destroy(&cmd_ctx->trax_blocks_mux);
	}
	esp_apptrace_blocks_pool_cleanup(cmd_ctx);
	return ERROR_OK;
}

#define ESP_APPTRACE_CMD_NUM_ARG_CHECK(_arg_, _start_, _end_)    \
    do { \
        if ((_arg_) == 0 && (_start_) == (_end_)) { \
            LOG_ERROR("Invalid '" #_arg_ "' arg!"); \
            res = ERROR_FAIL; \
            goto on_error; \
        } \
    } while (0)

static int esp_apptrace_cmd_init(struct target *target, struct esp_apptrace_cmd_ctx *cmd_ctx, int mode, const char **argv, int argc)
{
	int res;

	if (argc < 1) {
		LOG_ERROR("Not enough args! Need trace data destination!");
		return ERROR_FAIL;
	}

	res = esp_apptrace_cmd_ctx_init(target, cmd_ctx, mode);
	if (res) {
		return res;
	}
	if (cmd_ctx->mode == ESP_APPTRACE_CMD_MODE_SYSVIEW && argc < cmd_ctx->cores_num) {
		LOG_ERROR("Not enough args! Need %d trace data destinations!", cmd_ctx->cores_num);
		cmd_ctx->running = 0;
		esp_apptrace_cmd_ctx_cleanup(cmd_ctx);
		return ERROR_FAIL;
	}
	struct esp_apptrace_cmd_data *cmd_data = malloc(sizeof(struct esp_apptrace_cmd_data));
	if (!cmd_data) {
		LOG_ERROR("Failed to alloc cmd data!");
		cmd_ctx->running = 0;
		esp_apptrace_cmd_ctx_cleanup(cmd_ctx);
		return ERROR_FAIL;
	}
	memset(cmd_data, 0, sizeof(struct esp_apptrace_cmd_data));
	cmd_ctx->cmd_priv = cmd_data;

	//outfile1 [outfile2] [poll_period [trace_size [stop_tmo [wait4halt [skip_size]]]]]
	cmd_data->max_len = (uint32_t)-1;
	cmd_data->poll_period = 1/*ms*/;
	int dests_num = esp_apptrace_dest_init(cmd_data->data_dests, argv, cmd_ctx->mode == ESP_APPTRACE_CMD_MODE_SYSVIEW ? cmd_ctx->cores_num : 1);
	if (cmd_ctx->mode == ESP_APPTRACE_CMD_MODE_SYSVIEW && dests_num < cmd_ctx->cores_num) {
		LOG_ERROR("Not enough args! Need %d trace data destinations!", cmd_ctx->cores_num);
		res = ERROR_FAIL;
        goto on_error;
	}
	if (argc > dests_num) {
        char *end;
		cmd_data->poll_period = strtoul(argv[dests_num+0], &end, 10);
        ESP_APPTRACE_CMD_NUM_ARG_CHECK(cmd_data->poll_period, argv[dests_num+0], end);
		if (argc > dests_num+1) {
			cmd_data->max_len = strtoul(argv[dests_num+1], &end, 10);
            ESP_APPTRACE_CMD_NUM_ARG_CHECK(cmd_data->max_len, argv[dests_num+1], end);
			if (argc > dests_num+2) {
				int32_t tmo = strtol(argv[dests_num+2], &end, 10);
                ESP_APPTRACE_CMD_NUM_ARG_CHECK(tmo, argv[dests_num+2], end);
				cmd_ctx->stop_tmo = 1.0*tmo;
				if (argc > dests_num+3) {
					cmd_data->wait4halt = strtoul(argv[dests_num+3], &end, 10);
                    ESP_APPTRACE_CMD_NUM_ARG_CHECK(cmd_data->wait4halt, argv[dests_num+3], end);
					if (argc > dests_num+4) {
						cmd_data->skip_len = strtoul(argv[dests_num+4], &end, 10);
                        ESP_APPTRACE_CMD_NUM_ARG_CHECK(cmd_data->skip_len, argv[dests_num+4], end);
					}
				}
			}
		}
	}
	LOG_USER("App trace params: from %d cores, size %u bytes, stop_tmo %g s, poll period %u ms, wait_rst %d, skip %u bytes",
			cmd_ctx->cores_num, cmd_data->max_len, cmd_ctx->stop_tmo, cmd_data->poll_period, cmd_data->wait4halt, cmd_data->skip_len);

	return ERROR_OK;
on_error:
    LOG_ERROR("Not enough args! Need %d trace data destinations!", cmd_ctx->cores_num);
    free(cmd_data);
    cmd_ctx->running = 0;
    esp_apptrace_cmd_ctx_cleanup(cmd_ctx);
    return res;
}

static int esp_apptrace_cmd_cleanup(struct esp_apptrace_cmd_ctx *cmd_ctx)
{
	struct esp_apptrace_cmd_data *cmd_data = cmd_ctx->cmd_priv;

	esp_apptrace_dest_cleanup(cmd_data->data_dests, cmd_ctx->cores_num);
	free(cmd_data);
	esp_apptrace_cmd_ctx_cleanup(cmd_ctx);
	return ERROR_OK;
}

static void esp_apptrace_print_stats(struct esp_apptrace_cmd_ctx *ctx)
{
	struct esp_apptrace_cmd_data *cmd_data = ctx->cmd_priv;

	LOG_USER("Tracing is %s. Size is %u of %u @ %f (%f) KB/s", !ctx->running ? "STOPPED" : "RUNNING",
			ctx->tot_len > cmd_data->skip_len ? ctx->tot_len - cmd_data->skip_len : 0,
			cmd_data->max_len, duration_kbps(&ctx->read_time, ctx->tot_len),
			duration_kbps(&ctx->read_time, ctx->raw_tot_len));
	LOG_USER("Data: blocks incomplete %u, lost bytes: %u", ctx->stats.incompl_blocks, ctx->stats.lost_bytes);
#if ESP_APPTRACE_TIME_STATS_ENABLE
	LOG_USER("TRAX: block read time [%f..%f] ms", 1000*ctx->stats.min_blk_read_time, 1000*ctx->stats.max_blk_read_time);
	LOG_USER("TRAX: block proc time [%f..%f] ms", 1000*ctx->stats.min_blk_proc_time, 1000*ctx->stats.max_blk_proc_time);
#endif
}

static int esp_apptrace_wait4halt(struct esp_apptrace_cmd_ctx *ctx)
{
	int halted = 0;

	LOG_USER("Wait for halt...");
	while (!shutdown_openocd) {
	 	if (ctx->esp32_target) {
			int res = target_poll(ctx->esp32_target);
			if (res != ERROR_OK) {
				LOG_ERROR("Failed to poll target (%d)!", res);
				return res;
			}
			if (ctx->esp32_target->state == TARGET_HALTED) {
				LOG_USER("%s: HALTED", ctx->esp32_target->cmd_name);
				break;
			}
	 	} else {
			for (int k = 0; k < ctx->cores_num; k++) {
				int res = target_poll(ctx->cpus[k]);
				if (res != ERROR_OK) {
					LOG_ERROR("Failed to poll target%d (%d)!", k, res);
					return res;
				}
				if (ctx->cpus[k]->state == TARGET_HALTED) {
					LOG_USER("%s: HALTED", ctx->cpus[k]->cmd_name);
					if (++halted == ctx->cores_num)
						break;
				}
			}
	 	}
		if(halted)
			break;
		alive_sleep(500);
	}
	return ERROR_OK;
}

static int esp_apptrace_safe_halt_targets(struct esp_apptrace_cmd_ctx *ctx, struct esp108_apptrace_target_state *targets)
{
	int res = ERROR_OK;

	memset(targets, 0, ctx->cores_num*sizeof(struct esp108_apptrace_target_state));
	// halt all CPUs
	LOG_DEBUG("Halt all targets!");
 	if (ctx->esp32_target) {
		res = target_halt(ctx->esp32_target);
		if (res != ERROR_OK) {
			LOG_ERROR("Failed to halt target (%d)!", res);
			return res;
		}
		res = target_wait_state(ctx->esp32_target, TARGET_HALTED, ESP_APPTRACE_TGT_STATE_TMO);
		if (res != ERROR_OK) {
			LOG_ERROR("Failed to wait halt target %s / %d (%d)!", target_name(ctx->esp32_target), ctx->esp32_target->state, res);
			return res;
		}
 	} else {
		for (int k = 0; k < ctx->cores_num; k++) {
		 	targets[k].running = 1;
			res = target_halt(ctx->cpus[k]);
			if (res != ERROR_OK) {
				LOG_ERROR("Failed to halt target%d (%d)!", k, res);
				return res;
			}
			res = target_wait_state(ctx->cpus[k], TARGET_HALTED, ESP_APPTRACE_TGT_STATE_TMO);
			if (res != ERROR_OK) {
				LOG_ERROR("Failed to wait halt target %s / %d (%d)!", target_name(ctx->cpus[k]), ctx->cpus[k]->state, res);
				return res;
			}
		}
 	}
	// read current block statuses from CPUs
	LOG_DEBUG("Read current block statuses");
	for (int k = 0; k < ctx->cores_num; k++) {
		uint32_t stat;
		res = esp108_apptrace_read_status(ctx->cpus[k], &stat);
		if (res != ERROR_OK) {
			LOG_ERROR("Failed to read trace status (%d)!", res);
			return res;
		}
		// check if some CPU stopped inside TRAX reg update critical section
		if (stat) {
			res = esp108_activate_swdbg(ctx->cpus[k], 1/*enable*/);
			if (res != ERROR_OK) {
				LOG_ERROR("Failed to activate SW debug (%d)!", res);
				return res;
			}
			uint32_t bp_addr = stat;
			res = breakpoint_add(ctx->esp32_target ? ctx->esp32_target : ctx->cpus[k], bp_addr, 1, BKPT_HARD);
			if (res != ERROR_OK) {
				LOG_ERROR("Failed to set breakpoint (%d)!", res);
				return res;
			}
			while (stat) {
				// allow this CPU to leave ERI write critical section
				res = target_resume(ctx->esp32_target ? ctx->esp32_target : ctx->cpus[k], 1, 0, 1, 0);
				if (res != ERROR_OK) {
					LOG_ERROR("Failed to resume target (%d)!", res);
					breakpoint_remove(ctx->esp32_target ? ctx->esp32_target : ctx->cpus[k], bp_addr);
					return res;
				}
				// wait for CPU to be halted on BP
				enum target_debug_reason debug_reason = DBG_REASON_UNDEFINED;
				while (debug_reason != DBG_REASON_BREAKPOINT) {
					res = target_wait_state(ctx->esp32_target ? ctx->esp32_target : ctx->cpus[k], TARGET_HALTED, ESP_APPTRACE_TGT_STATE_TMO);
					if (res != ERROR_OK) {
						LOG_ERROR("Failed to wait halt on bp target (%d)!", res);
						breakpoint_remove(ctx->esp32_target ? ctx->esp32_target : ctx->cpus[k], bp_addr);
						return res;
					}
					debug_reason = ctx->esp32_target ? ctx->esp32_target->debug_reason : ctx->cpus[k]->debug_reason;
				}
				res = esp108_apptrace_read_status(ctx->cpus[k], &stat);
				if (res != ERROR_OK) {
					LOG_ERROR("Failed to read trace status (%d)!", res);
					breakpoint_remove(ctx->esp32_target ? ctx->esp32_target : ctx->cpus[k], bp_addr);
					return res;
				}
			}
			breakpoint_remove(ctx->esp32_target ? ctx->esp32_target : ctx->cpus[k], bp_addr);
			res = esp108_activate_swdbg(ctx->cpus[k], 0/*disable*/);
			if (res != ERROR_OK) {
				LOG_ERROR("Failed to de-activate SW debug (%d)!", res);
				return res;
			}
		}
		res = esp108_apptrace_read_data_len(ctx->cpus[k], &targets[k].block_id, &targets[k].data_len);
		if (res != ERROR_OK) {
			LOG_ERROR("Failed to read trace status (%d)!", res);
			return res;
		}
	}

	return ERROR_OK;
}

static int esp_apptrace_connect_targets(struct esp_apptrace_cmd_ctx *ctx, int conn)
{
	int res = ERROR_OK;
	struct esp108_apptrace_target_state target_to_connect[ESP_APPTRACE_TARGETS_NUM_MAX];

	if (conn)
		LOG_USER("Connect targets...");
	else
		LOG_USER("Disconnect targets...");

	res = esp_apptrace_safe_halt_targets(ctx, target_to_connect);
	if (res != ERROR_OK) {
		LOG_ERROR("Failed to halt targets (%d)!", res);
		return res;
	}
	if (ctx->cores_num > 1) {
		// set block ids to the highest value
		if (target_to_connect[0].block_id != target_to_connect[1].block_id) {
			if (target_to_connect[1].block_id > target_to_connect[0].block_id)
				target_to_connect[0].block_id = target_to_connect[1].block_id;
			else
				target_to_connect[1].block_id = target_to_connect[0].block_id;
		}
	}
	LOG_INFO("Resume targets");
	for (int k = 0; k < ctx->cores_num; k++) {
		// update host connected status
		res = esp108_apptrace_write_ctrl_reg(ctx->cpus[k], target_to_connect[k].block_id,
											0/*ack target data*/, conn, 0/*no host data*/);
		if (res != ERROR_OK) {
			LOG_ERROR("Failed to read trace status (%d)!", res);
			return res;
		}
		if (!ctx->esp32_target) {
			res = target_resume(ctx->cpus[k], 1, 0, 1, 0);
			if (res != ERROR_OK) {
				LOG_ERROR("Failed to resume target %d (%d)!", k, res);
				return res;
			}
		}
	}
	if (ctx->esp32_target) {
		res = target_resume(ctx->esp32_target, 1, 0, 1, 0);
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

static int esp_sysview_queue_cmds(struct target *target, uint8_t *cmds, uint32_t cmds_num, uint32_t block_id)
{
	struct esp_apptrace_host2target_msg msg;

	msg.hdr.block_sz = 0;
	for (uint32_t i = 0; i < cmds_num; i++) {
		msg.hdr.block_sz++;
		msg.data[i] = cmds[i];
		LOG_DEBUG("SEGGER: Send command %d, b %d", cmds[i], msg.hdr.block_sz);
	}
	// write header with or without data
	int res = esp108_apptrace_usr_block_write(target, block_id, (uint8_t *)&msg, sizeof(struct esp_apptrace_host2target_hdr) + msg.hdr.block_sz);
	if (res != ERROR_OK) {
		LOG_ERROR("SEGGER: Failed to write data to (%s)!", target_name(target));
		return res;
	}
	return res;
}

static int esp_apptrace_get_data_info(struct esp_apptrace_cmd_ctx *ctx, struct esp108_apptrace_target_state *target_state, uint32_t *fired_target_num)
{
	if (fired_target_num) {
		*fired_target_num = (uint32_t)-1;
	}

	for (int i = 0; i < ctx->cores_num; i++) {
		int res = esp108_apptrace_read_data_len(ctx->cpus[i], &target_state[i].block_id, &target_state[i].data_len);
		if (res != ERROR_OK) {
			LOG_ERROR("Failed to read data len on (%s)!", target_name(ctx->cpus[i]));
			return res;
		}
		if (target_state[i].data_len) {
			LOG_DEBUG("Block %d, len %d bytes on fired target (%s)!",
				target_state[i].block_id, target_state[i].data_len, target_name(ctx->cpus[i]));
			if (fired_target_num) {
				*fired_target_num = (uint32_t)i;
			}
			break;
		}
	}
	return ERROR_OK;
}

static int esp_sysview_write_trace_header(struct esp_apptrace_cmd_ctx *ctx)
{
	struct esp_apptrace_cmd_data *cmd_data = ctx->cmd_priv;

	char hdr_str[] = ";\n"
					 "; Version		SEGGER SystemViewer V2.42\n"
					 "; Author		Espressif Inc\n"
					 ";\n";
	int hdr_len = strlen(hdr_str);
	for (int i = 0; i < ctx->cores_num; i++) {
		int res = cmd_data->data_dests[i].write(cmd_data->data_dests[i].priv, (uint8_t *)hdr_str, hdr_len);
		if (res != ERROR_OK) {
			LOG_ERROR("SEGGER: Failed to write %u bytes to dest %d!", hdr_len, i);
			return ERROR_FAIL;
		}
	}
	return ERROR_OK;
}

// this function must be called after connecting to targets
static int esp_sysview_start(struct esp_apptrace_cmd_ctx *ctx)
{
	uint8_t cmds[] = {SEGGER_SYSVIEW_COMMAND_ID_START};
	uint32_t fired_target_num = 0, old_block_id;
	struct esp108_apptrace_target_state target_state[ESP_APPTRACE_TARGETS_NUM_MAX];
	struct duration wait_time;

	// get current block id
	int res = esp_apptrace_get_data_info(ctx, target_state, &fired_target_num);
	if (res != ERROR_OK) {
		LOG_ERROR("SEGGER: Failed to read target data info!");
		return res;
	}
	if (fired_target_num == (uint32_t)-1) {
		// it can happen that there is no pending target data, but block was switched
		// in this case block_ids on both CPUs are equal, so select the first one
		fired_target_num = 0;
	}
	// start tracing
	res = esp_sysview_queue_cmds(ctx->cpus[fired_target_num], cmds, sizeof(cmds)/sizeof(cmds[0]),
										target_state[fired_target_num].block_id);
	if (res != ERROR_OK) {
		LOG_ERROR("SEGGER: Failed to start tracing!");
		return res;
	}
	// wait for block switch (command sent)
	old_block_id = target_state[fired_target_num].block_id;
	if (duration_start(&wait_time) != 0) {
		LOG_ERROR("Failed to start trace start timeout measurement!");
		return ERROR_FAIL;
	}
	while (1) {
		res = esp_apptrace_get_data_info(ctx, target_state, &fired_target_num);
		if (res != ERROR_OK) {
			LOG_ERROR("SEGGER: Failed to read target data info!");
			return res;
		}
		if (fired_target_num == (uint32_t)-1) {
			// it can happen that there is no pending target data, but block was switched
			// in this case block_ids on both CPUs are equal, so select the first one
			fired_target_num = 0;
		}
		if (target_state[fired_target_num].block_id != old_block_id) {
			// do not read data, they will be read when polling callback is called
			LOG_DEBUG("Start command sent on %s (blk_id %x->%x len %x).", target_name(ctx->cpus[fired_target_num]),
				old_block_id, target_state[fired_target_num].block_id, target_state[fired_target_num].data_len);
			break;
		}
		if (duration_measure(&wait_time) != 0) {
			LOG_ERROR("Failed to start trace start timeout measurement!");
			return ERROR_FAIL;
		}
		if (duration_elapsed(&wait_time) >= 0.1) {
			LOG_INFO("Stop waiting for trace start due to timeout.");
			return ERROR_FAIL;
		}
	}
	return res;
}

static int esp_sysview_stop(struct esp_apptrace_cmd_ctx *ctx)
{
	uint32_t old_block_id, fired_target_num = 0, empty_target_num = 0;
	struct esp108_apptrace_target_state target_state[ESP_APPTRACE_TARGETS_NUM_MAX];
	uint8_t cmds[] = {SEGGER_SYSVIEW_COMMAND_ID_STOP};
	struct duration wait_time;
	int last_blocks_num = 0;

	struct esp_apptrace_block *block = esp_apptrace_free_block_get(ctx);
	if (!block) {
		LOG_ERROR("Failed to get free block for data on (%s)!", target_name(ctx->cpus[fired_target_num]));
		return ERROR_FAIL;
	}

	// halt all CPUs, otherwise it can happen that there is no target data and
	// while we are queueing commands on one CPU another CPU switches TRAX block
	int res = esp_apptrace_safe_halt_targets(ctx, target_state);
	if (res != ERROR_OK) {
		LOG_ERROR("SEGGER: Failed to halt targets (%d)!", res);
		return res;
	}
	// it can happen that there is no pending target data
	// in this case block_ids on both CPUs are equal, so the first one will be selected
	for (int k = 0; k < ctx->cores_num; k++) {
		if (target_state[k].data_len) {
			// data_len = target_state[k].data_len;
			// block_id = target_state[k].block_id;
			fired_target_num = k;
			break;
		}
	}
	if (target_state[fired_target_num].data_len) {
		// read pending data without ack, they will be acked when stop command is queued
		res = esp108_apptrace_read_data(ctx->cpus[fired_target_num], target_state[fired_target_num].data_len,
										block->data, target_state[fired_target_num].block_id,
										0/*no ack target data*/, NULL);
		if (res != ERROR_OK) {
			LOG_ERROR("SEGGER: Failed to read data on (%s)!", target_name(ctx->cpus[fired_target_num]));
			return res;
		} else {
			// process data
			block->data_len = target_state[fired_target_num].data_len;
			res = esp_apptrace_handle_trace_block(ctx, block);
			if (res != ERROR_OK) {
				LOG_ERROR("Failed to process trace block %d bytes!", block->data_len);
				return res;
			}
		}
	}
	// stop tracing and ack target data
	res = esp_sysview_queue_cmds(ctx->cpus[fired_target_num], cmds, sizeof(cmds)/sizeof(cmds[0]), target_state[fired_target_num].block_id);
	if (res != ERROR_OK) {
		LOG_ERROR("SEGGER: Failed to stop tracing!");
		return res;
	}
	if (ctx->cores_num > 1) {
		empty_target_num = fired_target_num ? 0 : 1;
		// ack target data on another CPU
		res = esp108_apptrace_write_ctrl_reg(ctx->cpus[empty_target_num], target_state[fired_target_num].block_id,
											0/*target data ack*/, 1/*host connected*/, 0 /*no host data*/);
		if (res != ERROR_OK) {
			LOG_ERROR("SEGGER: Failed to ack data on target '%s' (%d)!", target_name(ctx->cpus[empty_target_num]), res);
			return res;
		}
	}
	// resume targets to allow command processing
	LOG_INFO("Resume targets");
	for (int k = 0; k < ctx->cores_num; k++) {
		if (!ctx->esp32_target) {
			res = target_resume(ctx->cpus[k], 1, 0, 1, 0);
			if (res != ERROR_OK) {
				LOG_ERROR("SEGGER: Failed to resume target '%s' (%d)!", target_name(ctx->cpus[k]), res);
				return res;
			}
		}
	}
	if (ctx->esp32_target) {
		res = target_resume(ctx->esp32_target, 1, 0, 1, 0);
		if (res != ERROR_OK) {
			LOG_ERROR("SEGGER: Failed to resume target '%s' (%d)!", target_name(ctx->esp32_target), res);
			return res;
		}
	}
	// wait for block switch (command sent), so we can disconnect from targets
	old_block_id = target_state[fired_target_num].block_id;
	if (duration_start(&wait_time) != 0) {
		LOG_ERROR("Failed to start trace stop timeout measurement!");
		return ERROR_FAIL;
	}
	// we are waiting for the last data from TRAX block and also there can be data in the pended data buffer
	// so we are expectign two TRX block switches at most or stopping due to timeout
	while (last_blocks_num < 2) {
		res = esp_apptrace_get_data_info(ctx, target_state, &fired_target_num);
		if (res != ERROR_OK) {
			LOG_ERROR("SEGGER: Failed to read targets data info!");
			return res;
		}
		if (fired_target_num == (uint32_t)-1) {
			// it can happen that there is no pending (last) target data, but block was switched
			// in this case block_ids on both CPUs are equal, so select the first one
			fired_target_num = 0;
		}
		if (target_state[fired_target_num].block_id != old_block_id) {
			if (target_state[fired_target_num].data_len) {
				// read last data and ack them
				res = esp108_apptrace_read_data(ctx->cpus[fired_target_num], target_state[fired_target_num].data_len,
												block->data, target_state[fired_target_num].block_id,
												1/*ack target data*/, NULL);
				if (res != ERROR_OK) {
					LOG_ERROR("SEGGER: Failed to read last data on (%s)!", target_name(ctx->cpus[fired_target_num]));
				} else {
					if (ctx->cores_num > 1) {
						// ack target data on another CPU
						empty_target_num = fired_target_num ? 0 : 1;
						res = esp108_apptrace_write_ctrl_reg(ctx->cpus[empty_target_num], target_state[fired_target_num].block_id,
															0/*all read*/, 1/*host connected*/, 0 /*no host data*/);
						if (res != ERROR_OK) {
							LOG_ERROR("SEGGER: Failed to ack data on target '%s' (%d)!", target_name(ctx->cpus[empty_target_num]), res);
							return res;
						}
					}
					// process data
					block->data_len = target_state[fired_target_num].data_len;
					res = esp_apptrace_handle_trace_block(ctx, block);
					if (res != ERROR_OK) {
						LOG_ERROR("Failed to process trace block %d bytes!", block->data_len);
						return res;
					}
				}
				old_block_id = target_state[fired_target_num].block_id;
				last_blocks_num++;
			}
		}
		if (duration_measure(&wait_time) != 0) {
			LOG_ERROR("Failed to start trace stop timeout measurement!");
			return ERROR_FAIL;
		}
		if (duration_elapsed(&wait_time) >= 0.1) {
			LOG_INFO("Stop waiting for the last data due to timeout.");
			break;
		}
	}
	return res;
}

static uint32_t esp_apptrace_usr_block_check(struct esp_apptrace_cmd_ctx *ctx, struct esp_apptrace_target2host_hdr *hdr)
{
	uint32_t wr_len = 0, usr_len = 0;
	if (ctx->mode == ESP_APPTRACE_CMD_MODE_SYSVIEW) {
		wr_len = ESP_SYSVIEW_USER_BLOCK_LEN(hdr->sys_view.wr_sz);
		usr_len = ESP_SYSVIEW_USER_BLOCK_LEN(hdr->sys_view.block_sz);
	} else {
		wr_len = ESP_APPTRACE_USER_BLOCK_LEN(hdr->gen.wr_sz);
		usr_len = ESP_APPTRACE_USER_BLOCK_LEN(hdr->gen.block_sz);
	}
	if (usr_len != wr_len) {
		LOG_ERROR("Incomplete block sz %u, wr %u", usr_len, wr_len);
		ctx->stats.incompl_blocks++;
		ctx->stats.lost_bytes += usr_len - wr_len;
	}
	return usr_len;
}

uint8_t *esp108_apptrace_usr_block_get(uint8_t *buffer, uint32_t *size)
{
    struct esp_apptrace_target2host_hdr tmp_hdr;
    memcpy(&tmp_hdr, buffer, sizeof(tmp_hdr));

    *size = tmp_hdr.gen.wr_sz;

    return buffer + sizeof(struct esp_apptrace_target2host_hdr);
}

int esp108_apptrace_usr_block_write(struct target *target, uint32_t block_id, const uint8_t *data, uint32_t size)
{
    struct esp_apptrace_host2target_hdr hdr = {.block_sz = size};
    uint32_t buf_sz[2] = {sizeof(struct esp_apptrace_host2target_hdr), size};
    const uint8_t *bufs[2] = {(const uint8_t *)&hdr, data};

    if (size > ESP32_USR_BLOCK_SZ_MAX) {
        LOG_ERROR("Too large user block %u", size);
        return ERROR_FAIL;
    }

    return esp108_apptrace_write_buffs(target, 2, buf_sz, bufs, block_id, 1/*ack target data*/, 1/*host data*/);
}

static int esp_apptrace_process_data(struct esp_apptrace_cmd_ctx *ctx, int core_id, uint8_t *data, uint32_t data_len)
{
	struct esp_apptrace_cmd_data *cmd_data = ctx->cmd_priv;

	LOG_DEBUG("Got block %d bytes [%x %x...%x %x]", data_len, data[12], data[13], data[data_len-2], data[data_len-1]);
	if (ctx->tot_len + data_len > cmd_data->skip_len) {
		uint32_t wr_idx = 0, wr_chunk_len = data_len;
		if (ctx->tot_len < cmd_data->skip_len) {
			wr_chunk_len = (ctx->tot_len + wr_chunk_len) - cmd_data->skip_len;
			wr_idx = cmd_data->skip_len - ctx->tot_len;
		}
		if (ctx->tot_len + wr_chunk_len > cmd_data->max_len)
			wr_chunk_len -= (ctx->tot_len + wr_chunk_len - cmd_data->skip_len) - cmd_data->max_len;
		if (wr_chunk_len > 0) {
			int res = cmd_data->data_dests[0].write(cmd_data->data_dests[0].priv, data + wr_idx, wr_chunk_len);
			if (res != ERROR_OK) {
				LOG_ERROR("Failed to write %u bytes to dest 0!", data_len);
				return res;
			}
		}
		ctx->tot_len += wr_chunk_len;
	}
	else {
		ctx->tot_len += data_len;
	}
	LOG_USER("%u ", ctx->tot_len);
	// check for stop condition
	if ((ctx->tot_len > cmd_data->skip_len) && (ctx->tot_len - cmd_data->skip_len >= cmd_data->max_len)) {
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
	for (int k = 0; ; k++, (*ptr)++) {
		if (**ptr & 0x80) {
			val |= (uint32_t)(**ptr & ~0x80) << 7*k;
		} else {
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
  // here pkt points to encoded payload length
  if (*p & 0x80) {
	payload_len = *(p + 1); // higher part
	payload_len = (payload_len << 7) | (*p & ~0x80); // lower 7 bits
	p += 2; // payload len (2 bytes)
  } else {
	payload_len = *p;
	p++; // payload len (1 byte)
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
			//ENCODE_U32
			esp_sysview_decode_u32(&ptr);
			len = ptr - pkt;
			break;
		case SYSVIEW_EVTID_TASK_STOP_READY:
		case SYSVIEW_EVTID_SYSTIME_US:
			//2*ENCODE_U32
			esp_sysview_decode_u32(&ptr);
			esp_sysview_decode_u32(&ptr);
			len = ptr - pkt;
			break;
		case SYSVIEW_EVTID_SYSDESC:
			//str(128 + 1)
			len = *ptr + 1;
			break;
		case SYSVIEW_EVTID_TASK_INFO:
		case SYSVIEW_EVTID_MODULEDESC:
			//2*ENCODE_U32 + str
			esp_sysview_decode_u32(&ptr);
			esp_sysview_decode_u32(&ptr);
			// TODO: add support for strings longer then 255 bytes
			len = ptr - pkt + *ptr + 1;
			break;
		case SYSVIEW_EVTID_STACK_INFO:
			//4*ENCODE_U32
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

		//case SYSVIEW_EVTID_NOP:
		default:
			LOG_ERROR("SEGGER: Unsupported predef event %d!", id);
			len = 0;
	}
	return len;
}

static uint16_t esp_sysview_parse_packet(uint8_t *pkt_buf, uint32_t *pkt_len, int *pkt_core_id, uint32_t *delta, uint32_t *delta_len)
{
	uint8_t *pkt = pkt_buf;
	uint16_t event_id = 0, payload_len = 0;

	*pkt_core_id = 0;
	*pkt_len = 0;
	// 1-2 byte of message type, 0-2  byte of payload length, payload, 1-5 bytes of timestamp.
	if (*pkt & 0x80) {
		if (*(pkt + 1) & (1 << 6)) {
			*(pkt + 1) &= ~(1 << 6); // clear core_id bit
			*pkt_core_id = 1;
		}
		event_id = *(pkt + 1); // higher part
		event_id = (event_id << 7) | (*pkt & ~0x80); // lower 7 bits
		pkt += 2; // event_id (2 bytes)
		// here pkt points to encoded payload length
		payload_len = esp_sysview_decode_plen(&pkt);
	} else {
		if (*pkt & (1 << 6)) {
			*pkt &= ~(1 << 6); // clear core_id bit
			*pkt_core_id = 1;
		}
		// event_id (1 byte)
		event_id = *pkt;
		pkt++;
		if (event_id < 24) {
			payload_len = esp_sysview_get_predef_payload_len(event_id, pkt);
		} else {
			payload_len = esp_sysview_decode_plen(&pkt);
		}
	}
	pkt += payload_len;
	uint8_t *delta_start = pkt;
	*delta = esp_sysview_decode_u32(&pkt);
	*delta_len = pkt - delta_start;
	*pkt_len = pkt - pkt_buf;
	LOG_DEBUG("SEGGER: evt %d len %d plen %d dlen %d", event_id, *pkt_len, payload_len, *delta_len);
	return event_id;
}

static int esp_sysview_process_data(struct esp_apptrace_cmd_ctx *ctx, int core_id, uint8_t *data, uint32_t data_len)
{
	struct esp_apptrace_cmd_data *cmd_data = ctx->cmd_priv;

	LOG_DEBUG("SEGGER: Read from target %d bytes [%x %x %x %x]", data_len, data[0], data[1], data[2], data[3]);
	int res;
	uint32_t processed = 0;
	if (core_id >= ctx->cores_num) {
		LOG_ERROR("SEGGER: Invalid core id %d in user block!", core_id);
		return ERROR_FAIL;
	}
	if (ctx->tot_len == 0) {
		// handle sync seq
		if (data_len < SYSVIEW_SYNC_LEN) {
			LOG_ERROR("SEGGER: Invalid init seq len %d!", data_len);
			return ERROR_FAIL;
		}
		LOG_DEBUG("SEGGER: Process %d sync bytes", SYSVIEW_SYNC_LEN);
		uint8_t sync_seq[SYSVIEW_SYNC_LEN] = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};
		if (memcmp(data, sync_seq, SYSVIEW_SYNC_LEN) != 0) {
			LOG_ERROR("SEGGER: Invalid init seq [%x %x %x %x %x %x %x %x %x %x]",
				data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7], data[8], data[9]);
			return ERROR_FAIL;
		}
		res = cmd_data->data_dests[core_id].write(cmd_data->data_dests[core_id].priv, data, SYSVIEW_SYNC_LEN);
		if (res != ERROR_OK) {
			LOG_ERROR("SEGGER: Failed to write %u sync bytes to dest %d!", SYSVIEW_SYNC_LEN, core_id);
			return res;
		}
		if (ctx->cores_num > 1) {
			res = cmd_data->data_dests[core_id ? 0 : 1].write(cmd_data->data_dests[core_id ? 0 : 1].priv, data, SYSVIEW_SYNC_LEN);
			if (res != ERROR_OK) {
				LOG_ERROR("SEGGER: Failed to write %u sync bytes to dest %d!", SYSVIEW_SYNC_LEN, core_id ? 0 : 1);
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
		uint16_t event_id = esp_sysview_parse_packet(data + processed, &pkt_len, &pkt_core_id, &delta, &delta_len);
		LOG_DEBUG("SEGGER: Process packet %d id %d bytes [%x %x %x %x]", event_id, pkt_len, data[processed+0], data[processed+1], data[processed+2], data[processed+3]);
		if (event_id > SYSVIEW_EVENT_ID_MAX) {
			LOG_ERROR("SEGGER: Unsupported event ID %d!", event_id);
			return res;
		}
		wr_len = pkt_len;
		if (ctx->cores_num > 1) {
			if (cmd_data->sv_last_core_id == pkt_core_id) {
				// if this packet is for the same core as the prev one acc delta and write packet unmodified
				cmd_data->sv_acc_time_delta += delta;
			} else {
				// if this packet is for another core then prev one set acc delta to the packet's delta
				uint8_t *delta_ptr = new_delta_buf;
				SYSVIEW_ENCODE_U32(delta_ptr, delta + cmd_data->sv_acc_time_delta);
				cmd_data->sv_acc_time_delta = delta;
				wr_len -= delta_len;
				new_delta_len = delta_ptr - new_delta_buf;
				pkt_core_changed = 1;
			}
			cmd_data->sv_last_core_id = pkt_core_id;
		}
		res = cmd_data->data_dests[pkt_core_id].write(cmd_data->data_dests[pkt_core_id].priv, data + processed, wr_len);
		if (res != ERROR_OK) {
			LOG_ERROR("SEGGER: Failed to write %u bytes to dest %d!", wr_len, core_id);
			return res;
		}
		if (new_delta_len) {
			// write packet with modified delta
			res = cmd_data->data_dests[pkt_core_id].write(cmd_data->data_dests[pkt_core_id].priv, new_delta_buf, new_delta_len);
			if (res != ERROR_OK) {
				LOG_ERROR("SEGGER: Failed to write %u bytes of delta to dest %d!", new_delta_len, core_id);
				return res;
			}
		}
		if (ctx->cores_num > 1) {
			// handle other core dest
			int other_core_id = pkt_core_id ? 0 : 1;
			switch (event_id)
			{
				/* messages below should be sent to trace destinations for all cores */
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
					// if packet's source core has changed
					wr_len = pkt_len;
					if (pkt_core_changed) {
						// clone packet with unmodified delta
						new_delta_len = 0;
					} else {
						// clone packet with modified delta
						uint8_t *delta_ptr = new_delta_buf;
						SYSVIEW_ENCODE_U32(delta_ptr, cmd_data->sv_acc_time_delta /*delta has been accumulated above*/);
						wr_len -= delta_len;
						new_delta_len = delta_ptr - new_delta_buf;
					}
					LOG_DEBUG("SEGGER: Redirect %d bytes of event %d to dest %d", wr_len, event_id, other_core_id);
					res = cmd_data->data_dests[other_core_id].write(cmd_data->data_dests[other_core_id].priv, data + processed, wr_len);
					if (res != ERROR_OK) {
						LOG_ERROR("SEGGER: Failed to write %u bytes to dest %d!", wr_len, other_core_id);
						return res;
					}
					if (new_delta_len) {
						// write packet with modified delta
						res = cmd_data->data_dests[other_core_id].write(cmd_data->data_dests[other_core_id].priv, new_delta_buf, new_delta_len);
						if (res != ERROR_OK) {
							LOG_ERROR("SEGGER: Failed to write %u bytes of delta to dest %d!", new_delta_len, other_core_id);
							return res;
						}
					}
					// messages above are cloned to trace files for both cores, so reset acc time delta, both files have actual delta info
					cmd_data->sv_acc_time_delta = 0;
					break;
				default:
					break;
			}
		}
		ctx->tot_len += pkt_len;
		processed += pkt_len;
	}
	LOG_USER("%u ", ctx->tot_len);
	// check for stop condition
	if ((ctx->tot_len > cmd_data->skip_len) && (ctx->tot_len - cmd_data->skip_len >= cmd_data->max_len)) {
		ctx->running = 0;
		if (duration_measure(&ctx->read_time) != 0) {
			LOG_ERROR("Failed to stop trace read time measure!");
			return ERROR_FAIL;
		}
	}
	return ERROR_OK;
}

static int esp_apptrace_handle_trace_block(struct esp_apptrace_cmd_ctx *ctx, struct esp_apptrace_block *block)
{
	uint32_t processed = 0;
	uint32_t hdr_sz = ctx->mode == ESP_APPTRACE_CMD_MODE_SYSVIEW ? ESP_SYSVIEW_USER_BLOCK_HDR_SZ : ESP_APPTRACE_USER_BLOCK_HDR_SZ;
	LOG_DEBUG("Got block %d bytes", block->data_len);
	// process user blocks one by one
	while (processed < block->data_len) {
		LOG_DEBUG("Process usr block %d/%d", processed, block->data_len);
		struct esp_apptrace_target2host_hdr tmp_hdr;
		memcpy(&tmp_hdr, block->data + processed, sizeof(tmp_hdr));
		struct esp_apptrace_target2host_hdr *hdr = &tmp_hdr;
		// process user block
		uint32_t usr_len = esp_apptrace_usr_block_check(ctx, hdr);
		int core_id;
		if (ctx->mode == ESP_APPTRACE_CMD_MODE_SYSVIEW) {
			core_id = ESP_SYSVIEW_USER_BLOCK_CORE(hdr->sys_view.block_sz);
		} else {
			core_id = ESP_APPTRACE_USER_BLOCK_CORE(hdr->gen.block_sz);
		}
		// process user data
		int res = ctx->process_data(ctx, core_id, block->data + processed + hdr_sz, usr_len);
		if (res != ERROR_OK) {
			LOG_ERROR("Failed to process %d bytes!", usr_len);
			return res;
		}
		processed += usr_len + hdr_sz;
	}
	return ERROR_OK;
}

static void *esp_apptrace_data_processor(void *arg)
{
	long res = ERROR_OK;
	struct esp_apptrace_cmd_ctx *ctx = (struct esp_apptrace_cmd_ctx *)arg;

	while(ctx->running) {
		struct esp_apptrace_block *block = esp_apptrace_ready_block_get(ctx);
		if (!block) {
			continue;
		}
		res = esp_apptrace_handle_trace_block(ctx, block);
		if (res != ERROR_OK) {
			ctx->running = 0;
			LOG_ERROR("Failed to process trace block %d bytes!", block->data_len);
			break;
		}
		res = esp_apptrace_block_free(ctx, block);
		if (res != ERROR_OK) {
			ctx->running = 0;
			LOG_ERROR("Failed to free ready block!");
			break;
		}
	}

	return (void *)res;
}

static int esp_apptrace_poll(void *priv)
{
	struct esp_apptrace_cmd_ctx *ctx = (struct esp_apptrace_cmd_ctx *)priv;
	int res;
	uint32_t fired_target_num = 0;
	struct esp108_apptrace_target_state target_state[ESP_APPTRACE_TARGETS_NUM_MAX];
#if ESP_APPTRACE_TIME_STATS_ENABLE
	struct duration blk_proc_time;
#endif

	if (!ctx->running)
		return ERROR_FAIL;

	// check for data from target
	res = esp_apptrace_get_data_info(ctx, target_state, &fired_target_num);
	if (res != ERROR_OK) {
		ctx->running = 0;
		LOG_ERROR("Failed to read data len!");
		return res;
	}
	if (fired_target_num == (uint32_t)-1) {
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
		return ERROR_OK; // no data
	}
	// sanity check
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
	struct esp_apptrace_block *block = esp_apptrace_free_block_get(ctx);
	if (!block) {
		ctx->running = 0;
		LOG_ERROR("Failed to get free block for data on (%s)!", target_name(ctx->cpus[fired_target_num]));
		return ERROR_FAIL;
	}
#if ESP_APPTRACE_TIME_STATS_ENABLE
	// read block
	if (duration_start(&blk_proc_time) != 0) {
		ctx->running = 0;
		LOG_ERROR("Failed to start block read time measurement!");
		return ERROR_FAIL;
	}
#endif
	res = esp108_apptrace_read_data(ctx->cpus[fired_target_num], target_state[fired_target_num].data_len,
									block->data, target_state[fired_target_num].block_id,
									1/*ack target data*/, NULL);
	if (res != ERROR_OK) {
		ctx->running = 0;
		LOG_ERROR("Failed to read data on (%s)!", target_name(ctx->cpus[fired_target_num]));
		return res;
	}
#if ESP_APPTRACE_TIME_STATS_ENABLE
	if (duration_measure(&blk_proc_time) != 0) {
		ctx->running = 0;
		LOG_ERROR("Failed to measure block read time!");
		return ERROR_FAIL;
	}
	// update stats
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
		res = esp108_apptrace_write_ctrl_reg(ctx->cpus[fired_target_num ? 0 : 1], target_state[fired_target_num].block_id,
											 0/*all read*/, 1/*host connected*/, 0/*no host data*/);
		if (res != ERROR_OK) {
			ctx->running = 0;
			LOG_ERROR("Failed to ack data on (%s)!", target_name(ctx->cpus[fired_target_num ? 0 : 1]));
			return res;
		}
		LOG_DEBUG("Ack block %d target (%s)!", target_state[fired_target_num].block_id, target_name(ctx->cpus[fired_target_num ? 0 : 1]));
	}
	ctx->raw_tot_len += target_state[fired_target_num].data_len;

	block->data_len = target_state[fired_target_num].data_len;
	if (ctx->mode == ESP_APPTRACE_CMD_MODE_SYNC) {
		res = esp_apptrace_handle_trace_block(ctx, block);
		if (res != ERROR_OK) {
			ctx->running = 0;
			LOG_ERROR("Failed to process trace block %d bytes!", block->data_len);
			return res;
		}
		res = esp_apptrace_block_free(ctx, block);
		if (res != ERROR_OK) {
			ctx->running = 0;
			LOG_ERROR("Failed to free ready block!");
			return res;
		}
	} else {
		res = esp_apptrace_ready_block_put(ctx, block);
		if (res != ERROR_OK) {
			ctx->running = 0;
			LOG_ERROR("Failed to put ready block of data from (%s)!", target_name(ctx->cpus[fired_target_num]));
			return res;
		}
	}
	if (ctx->stop_tmo != -1.0) {
		// start idle time measurement
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
	// update stats
	float bt = duration_elapsed(&blk_proc_time);
	if (bt > ctx->stats.max_blk_proc_time)
		ctx->stats.max_blk_proc_time = bt;
	if (bt < ctx->stats.min_blk_proc_time)
		ctx->stats.min_blk_proc_time = bt;
#endif
	return ERROR_OK;
}

int esp_cmd_apptrace_generic(struct target *target, int mode, const char **argv, int argc)
{
	static struct esp_apptrace_cmd_ctx s_at_cmd_ctx;
	struct esp_apptrace_cmd_data *cmd_data;
	int res = ERROR_OK;

	if (argc < 1) {
		LOG_ERROR("Action missed!");
		return ERROR_FAIL;
	}

	if (strcmp(argv[0], "start") == 0) {
		// init cmd context
		res = esp_apptrace_cmd_init(target, &s_at_cmd_ctx, mode, &argv[1], argc-1);
		if (res != ERROR_OK) {
			LOG_ERROR("Failed to init cmd ctx (%d)!", res);
			return res;
		}
		cmd_data = s_at_cmd_ctx.cmd_priv;
		if (mode == ESP_APPTRACE_CMD_MODE_SYSVIEW) {
			if (cmd_data->skip_len != 0) {
				LOG_ERROR("Data skipping not supported!");
				s_at_cmd_ctx.running = 0;
				esp_apptrace_cmd_cleanup(&s_at_cmd_ctx);
				return ERROR_FAIL;
			}
			s_at_cmd_ctx.process_data = esp_sysview_process_data;
			res = esp_sysview_write_trace_header(&s_at_cmd_ctx);
			if (res != ERROR_OK) {
				LOG_ERROR("Failed to write trace header (%d)!", res);
				s_at_cmd_ctx.running = 0;
				esp_apptrace_cmd_cleanup(&s_at_cmd_ctx);
				return res;
			}
		}
		else {
			s_at_cmd_ctx.process_data = esp_apptrace_process_data;
		}
		if (cmd_data->wait4halt) {
			res = esp_apptrace_wait4halt(&s_at_cmd_ctx);
			if (res != ERROR_OK) {
				LOG_ERROR("Failed to wait for halt target (%d)!", res);
				s_at_cmd_ctx.running = 0;
				esp_apptrace_cmd_cleanup(&s_at_cmd_ctx);
				return res;
			}
		}
		res = esp_apptrace_connect_targets(&s_at_cmd_ctx, 1);
		if (res != ERROR_OK) {
			LOG_ERROR("Failed to connect to targets (%d)!", res);
			s_at_cmd_ctx.running = 0;
			esp_apptrace_cmd_cleanup(&s_at_cmd_ctx);
			return res;
		}
		if (mode == ESP_APPTRACE_CMD_MODE_SYSVIEW) {
			// start tracing
			res = esp_sysview_start(&s_at_cmd_ctx);
			if (res != ERROR_OK) {
				LOG_ERROR("SEGGER: Failed to start tracing!");
				esp_apptrace_connect_targets(&s_at_cmd_ctx, 0);
				s_at_cmd_ctx.running = 0;
				esp_apptrace_cmd_cleanup(&s_at_cmd_ctx);
				return res;
			}
		}
		// openocd timer_callback min period is 1 ms, if we need to poll target for trace data more frequently polling loop will be used
		if (cmd_data->poll_period >= 1) {
			res = target_register_timer_callback(esp_apptrace_poll, cmd_data->poll_period, 1, &s_at_cmd_ctx);
			if (res != ERROR_OK) {
				LOG_ERROR("Failed to register target timer handler (%d)!", res);
				return res;
			}
		}
		else { ////////////////// POLLING MODE ///////////////////////////
			/* check for exit signal and comand completion */
			while (!shutdown_openocd && s_at_cmd_ctx.running) {
				res = esp_apptrace_poll(&s_at_cmd_ctx);
				if (res != ERROR_OK) {
					LOG_ERROR("Failed to poll target for trace data (%d)!", res);
					break;
				}
				/* let registered timer callbacks to run */
				target_call_timer_callbacks();
			}
			// if we stopped due to user pressed CTRL+C
			if (shutdown_openocd) {
				if (duration_measure(&s_at_cmd_ctx.read_time) != 0) {
					LOG_ERROR("Failed to stop trace read time measurement!");
				}
			}
			if (s_at_cmd_ctx.running) {
				// data processor is alive, so wait for all received blocks to be processed
				res = esp_apptrace_wait_pended_blocks(&s_at_cmd_ctx);
				if (res != ERROR_OK) {
					LOG_ERROR("Failed to wait for pended blocks (%d)!", res);
				}
				// signal thread to stop
				s_at_cmd_ctx.running = 0;
			}
			if (mode == ESP_APPTRACE_CMD_MODE_SYSVIEW) {
				// stop tracing
				res = esp_sysview_stop(&s_at_cmd_ctx);
				if (res != ERROR_OK) {
					LOG_ERROR("SEGGER: Failed to stop tracing!");
				}
			}
			res = esp_apptrace_connect_targets(&s_at_cmd_ctx, 0);
			if (res != ERROR_OK) {
				LOG_ERROR("Failed to disconnect targets (%d)!", res);
			}
			esp_apptrace_print_stats(&s_at_cmd_ctx);
			res = esp_apptrace_cmd_cleanup(&s_at_cmd_ctx);
			if (res != ERROR_OK) {
				LOG_ERROR("Failed to cleanup cmd ctx (%d)!", res);
			}
		}
	}
	else if (strcmp(argv[0], "stop") == 0) {
		if (s_at_cmd_ctx.running) {
			if (duration_measure(&s_at_cmd_ctx.read_time) != 0) {
				LOG_ERROR("Failed to stop trace read time measurement!");
			}
			// data processor is alive, so wait for all received blocks to be processed
			res = esp_apptrace_wait_pended_blocks(&s_at_cmd_ctx);
			if (res != ERROR_OK) {
				LOG_ERROR("Failed to wait for pended blocks (%d)!", res);
			}
			// signal thread to stop
			s_at_cmd_ctx.running = 0;
		}
		res = target_unregister_timer_callback(esp_apptrace_poll, &s_at_cmd_ctx);
		if (res != ERROR_OK) {
			LOG_ERROR("Failed to unregister target timer handler (%d)!", res);
		}
		if (mode == ESP_APPTRACE_CMD_MODE_SYSVIEW) {
			// stop tracing
			res = esp_sysview_stop(&s_at_cmd_ctx);
			if (res != ERROR_OK) {
				LOG_ERROR("SEGGER: Failed to stop tracing!");
			}
		}
		res = esp_apptrace_connect_targets(&s_at_cmd_ctx, 0);
		if (res != ERROR_OK) {
			LOG_ERROR("Failed to disconnect targets (%d)!", res);
		}
		esp_apptrace_print_stats(&s_at_cmd_ctx);
		res = esp_apptrace_cmd_cleanup(&s_at_cmd_ctx);
		if (res != ERROR_OK) {
			LOG_ERROR("Failed to cleanup cmd ctx (%d)!", res);
		}
	}
	else if (strcmp(argv[0], "status") == 0) {
		if (s_at_cmd_ctx.running && duration_measure(&s_at_cmd_ctx.read_time) != 0) {
			LOG_ERROR("Failed to measure trace read time!");
		}
		esp_apptrace_print_stats(&s_at_cmd_ctx);
	}
	else if (strcmp(argv[0], "dump") == 0) {
		if (mode == ESP_APPTRACE_CMD_MODE_SYSVIEW) {
			LOG_ERROR("Not supported!");
			return ERROR_FAIL;
		}
		// [dump outfile] - post-mortem dump without connection to targets
		res = esp_apptrace_cmd_init(target, &s_at_cmd_ctx, mode, &argv[1], argc-1);
		if (res != ERROR_OK) {
			LOG_ERROR("Failed to init cmd ctx (%d)!", res);
			return res;
		}
		s_at_cmd_ctx.stop_tmo = 0.01; // use small stop tmo
		s_at_cmd_ctx.process_data = esp_apptrace_process_data;
		/* check for exit signal and comand completion */
		while (!shutdown_openocd && s_at_cmd_ctx.running) {
			res = esp_apptrace_poll(&s_at_cmd_ctx);
			if (res != ERROR_OK) {
				LOG_ERROR("Failed to poll target for trace data (%d)!", res);
				break;
			}
			/* let registered timer callbacks to run */
			target_call_timer_callbacks();
		}
	if (s_at_cmd_ctx.running) {
	  // data processor is alive, so wait for all received blocks to be processed
	  res = esp_apptrace_wait_pended_blocks(&s_at_cmd_ctx);
	  if (res != ERROR_OK) {
		LOG_ERROR("Failed to wait for pended blocks (%d)!", res);
	  }
	  // signal thread to stop
	  s_at_cmd_ctx.running = 0;
	}
		esp_apptrace_print_stats(&s_at_cmd_ctx);
		res = esp_apptrace_cmd_cleanup(&s_at_cmd_ctx);
		if (res != ERROR_OK) {
			LOG_ERROR("Failed to cleanup cmd ctx (%d)!", res);
		}
	}
	else {
		LOG_ERROR("Invalid action '%s'!", argv[0]);
	}

	return res;
}

__COMMAND_HANDLER(esp108_cmd_apptrace)
{
	return esp_cmd_apptrace_generic(get_current_target(CMD_CTX), ESP_APPTRACE_CMD_MODE_GEN, CMD_ARGV, CMD_ARGC);
}

__COMMAND_HANDLER(esp108_cmd_sysview)
{
	return esp_cmd_apptrace_generic(get_current_target(CMD_CTX), ESP_APPTRACE_CMD_MODE_SYSVIEW, CMD_ARGV, CMD_ARGC);
}

static int esp_gcov_cmd_init(struct target *target, struct esp_apptrace_cmd_ctx *cmd_ctx, const char **argv, int argc)
{
	int res;

	res = esp_apptrace_cmd_ctx_init(target, cmd_ctx, ESP_APPTRACE_CMD_MODE_SYNC);
	if (res) {
		return res;
	}
	cmd_ctx->stop_tmo = 3.0;
	cmd_ctx->process_data = esp_gcov_process_data;

	struct esp_gcov_cmd_data *cmd_data = malloc(sizeof(struct esp_gcov_cmd_data));
	if (!cmd_data) {
		LOG_ERROR("Failed to alloc cmd data!");
		esp_apptrace_cmd_ctx_cleanup(cmd_ctx);
		return ERROR_FAIL;
	}
	memset(cmd_data, 0, sizeof(struct esp_gcov_cmd_data));
	cmd_ctx->cmd_priv = cmd_data;

	if (argc > 0) {
		cmd_data->wait4halt = strtoul(argv[0], NULL, 10);
	}

	return ERROR_OK;
}

static int esp_gcov_cmd_cleanup(struct esp_apptrace_cmd_ctx *cmd_ctx)
{
	struct esp_gcov_cmd_data *cmd_data = cmd_ctx->cmd_priv;
	int res = ERROR_OK;

	for (int i = 0; i < ESP_GCOV_FILES_MAX_NUM; i++) {
		if (cmd_data->files[i] && fclose(cmd_data->files[i])) {
			LOG_ERROR("Failed to close file 0x%p (%d)!", cmd_data->files[i], errno);
			res = ERROR_FAIL;
		}
	}
	free(cmd_data);
	esp_apptrace_cmd_ctx_cleanup(cmd_ctx);
	return res;
}

static int esp_gcov_fopen(struct esp_gcov_cmd_data *cmd_data, uint8_t *data, uint32_t data_len, uint8_t **resp, uint32_t *resp_len)
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

	LOG_INFO("Open file '%s'", data);
	uint32_t fd = cmd_data->files_num;
	char *mode = (char *)data + len + 1;
	cmd_data->files[fd] = fopen((char *)data, mode);
	if (!cmd_data->files[fd]) {
		// do not report error on reading non-existent file
		if (errno != ENOENT || strchr(mode, 'r') == NULL) {
			LOG_ERROR("Failed to open file '%s', mode '%s' (%d)!", data, mode, errno);
		}
		fd = 0;
	} else {
		fd++; // 1-based, 0 indicates error
	}

	*resp_len = sizeof(fd);
	*resp = malloc(*resp_len);
	if (!*resp) {
		LOG_ERROR("Failed to alloc mem for resp!");
		if (fd != 0) {
			fclose(cmd_data->files[fd-1]);
		}
		return ERROR_FAIL;
	}
	memcpy(*resp, &fd, sizeof(fd));

	if (fd != 0) {
		cmd_data->files_num++;
	}

	return ERROR_OK;
}

static int esp_gcov_fclose(struct esp_gcov_cmd_data *cmd_data, uint8_t *data, uint32_t data_len, uint8_t **resp, uint32_t *resp_len)
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
	if (fret) {
		LOG_ERROR("Failed to close file %d (%d)!", fd, errno);
	} else {
		cmd_data->files[fd] = NULL;
	}

	*resp_len = sizeof(fret);
	*resp = malloc(*resp_len);
	if (!*resp) {
		LOG_ERROR("Failed to alloc mem for resp!");
		return ERROR_FAIL;
	}
	memcpy(*resp, &fret, sizeof(fret));

	return ERROR_OK;
}

static int esp_gcov_fwrite(struct esp_gcov_cmd_data *cmd_data, uint8_t *data, uint32_t data_len, uint8_t **resp, uint32_t *resp_len)
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
	if (fret != 1) {
		LOG_ERROR("Failed to write %ld byte (%d)!", (long)(data_len - sizeof(fd)), errno);
	}

	*resp_len = sizeof(fret);
	*resp = malloc(*resp_len);
	if (!*resp) {
		LOG_ERROR("Failed to alloc mem for resp!");
		return ERROR_FAIL;
	}
	memcpy(*resp, &fret, sizeof(fret));

	return ERROR_OK;
}

static int esp_gcov_fread(struct esp_gcov_cmd_data *cmd_data, uint8_t *data, uint32_t data_len, uint8_t **resp, uint32_t *resp_len)
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
	fret = fread(*resp + sizeof(fret), len, 1, cmd_data->files[fd]);
	if (fret != 1) {
		LOG_ERROR("Failed to read %d byte (%d)!", len, errno);
	}
	memcpy(*resp, &fret, sizeof(fret));

	return ERROR_OK;
}

static int esp_gcov_fseek(struct esp_gcov_cmd_data *cmd_data, uint8_t *data, uint32_t data_len, uint8_t **resp, uint32_t *resp_len)
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

static int esp_gcov_ftell(struct esp_gcov_cmd_data *cmd_data, uint8_t *data, uint32_t data_len, uint8_t **resp, uint32_t *resp_len)
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

//TODO: support for multi-block data transfers
static int esp_gcov_process_data(struct esp_apptrace_cmd_ctx *ctx, int core_id, uint8_t *data, uint32_t data_len)
{
	int ret = ERROR_OK;
	struct esp_gcov_cmd_data *cmd_data = ctx->cmd_priv;
	uint8_t *resp;
	uint32_t resp_len = 0;

	LOG_DEBUG("Got block %d bytes [%x %x...%x %x]", data_len, data[12], data[13], data[data_len-2], data[data_len-1]);

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
	if (ret != ERROR_OK) {
		return ret;
	}

	if (resp_len) {
		struct esp108_apptrace_target_state target_state[ESP_APPTRACE_TARGETS_NUM_MAX];
		uint32_t fired_target_num = 0;
		// get current block id
		int res = esp_apptrace_get_data_info(ctx, target_state, &fired_target_num);
		if (res != ERROR_OK) {
			LOG_ERROR("Failed to read target data info!");
			free(resp);
			return res;
		}
		if (fired_target_num == (uint32_t)-1) {
			// it can happen that there is no pending target data, but block was switched
			// in this case block_ids on both CPUs are equal, so select the first one
			fired_target_num = 0;
		}
		// write response
		res = esp108_apptrace_usr_block_write(ctx->cpus[fired_target_num], target_state[fired_target_num].block_id, resp, resp_len);
		if (res != ERROR_OK) {
			LOG_ERROR("Failed to write data to (%s)!", target_name(ctx->cpus[fired_target_num]));
			free(resp);
			return res;
		}
		free(resp);
	}

	return ERROR_OK;
}

int esp_cmd_gcov(struct target *target, const char **argv, int argc)
{
	struct esp_gcov_cmd_data *cmd_data;
	static struct esp_apptrace_cmd_ctx s_at_cmd_ctx;
	int res = ERROR_OK;

	// init cmd context
	res = esp_gcov_cmd_init(target, &s_at_cmd_ctx, argv, argc);
	if (res != ERROR_OK) {
		LOG_ERROR("Failed to init cmd ctx (%d)!", res);
		return res;
	}
	cmd_data = s_at_cmd_ctx.cmd_priv;
	if (cmd_data->wait4halt) {
		res = esp_apptrace_wait4halt(&s_at_cmd_ctx);
		if (res != ERROR_OK) {
			LOG_ERROR("Failed to wait for halt target (%d)!", res);
			esp_gcov_cmd_cleanup(&s_at_cmd_ctx);
			return res;
		}
	}
	res = esp_apptrace_connect_targets(&s_at_cmd_ctx, 1);
	if (res != ERROR_OK) {
		LOG_ERROR("Failed to connect to targets (%d)!", res);
		s_at_cmd_ctx.running = 0;
		esp_gcov_cmd_cleanup(&s_at_cmd_ctx);
		return res;
	}
	/* check for exit signal and command completion */
	while (!shutdown_openocd && s_at_cmd_ctx.running) {
		res = esp_apptrace_poll(&s_at_cmd_ctx);
		if (res != ERROR_OK) {
			LOG_ERROR("Failed to poll target for trace data (%d)!", res);
			break;
		}
		/* let registered timer callbacks to run */
		target_call_timer_callbacks();
	}
	res = esp_apptrace_connect_targets(&s_at_cmd_ctx, 0);
	if (res != ERROR_OK) {
		LOG_ERROR("Failed to disconnect targets (%d)!", res);
	}
	res = esp_gcov_cmd_cleanup(&s_at_cmd_ctx);
	if (res != ERROR_OK) {
		LOG_ERROR("Failed to cleanup cmd ctx (%d)!", res);
	}
	return res;
}
