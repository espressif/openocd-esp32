/***************************************************************************
 *   Xtensa application tracing module for OpenOCD                         *
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

/* How It Works
 * ************ */

/* 1. Components Overview
 * ====================== */

/* Xtensa has useful feature: TRAX debug module. It allows recording program execution flow at
 * run-time without disturbing CPU. */
/* Exectution flow data are written to configurable Trace RAM block. Besides accessing Trace RAM
 * itself TRAX module also allows to read/write */
/* trace memory via its registers by means of JTAG, APB or ERI transactions. */
/* ESP32 has two Xtensa cores with separate TRAX modules on them and provides two special memory
 * regions to be used as trace memory. */
/* Chip allows muxing access to those trace memory blocks in such a way that while one block is
 * accessed by CPUs another one can be accessed by host */
/* by means of reading/writing TRAX registers via JTAG. Blocks muxing is configurable at run-time
 * and allows switching trace memory blocks between */
/* accessors in round-robin fashion so they can read/write separate memory blocks without disturbing
 * each other. */
/* This moduile implements application tracing feature based on above mechanisms. It allows to
 * transfer arbitrary user data to/from */
/* host via JTAG with minimal impact on system performance. This module is implied to be used in the
 * following tracing scheme. */

/*                                                        ------>------
 *                                         ----- (host components) ----- */
/*                                                        |           |
 *                                         |                           | */
/* -------------------   -----------------------     -----------------------     ----------------
 *    ------     ---------   ----------------- */
/* |trace data source|-->|target tracing module|<--->|TRAX_MEM0 |
 * TRAX_MEM1|---->|TRAX_DATA_REGS|<-->|JTAG|<--->|OpenOCD|-->|trace data sink| */
/* -------------------   -----------------------     -----------------------     ----------------
 *    ------     ---------   ----------------- */
/*                                 |                      |           |
 *                                | */
/*                                 |                      ------<------          ----------------
 *      | */
/*
 *
 *                            |<------------------------------------------->|TRAX_CTRL_REGS|<---->| */
/*                                                                               ---------------- */

/* In general tracing goes in the following way. User aplication requests tracing module to send
 * some data by calling esp32_apptrace_buffer_get(), */
/* moduile allocates necessary buffer in current input trace block. Then user fills received buffer
 * with data and calls esp32_apptrace_buffer_put(). */
/* When current input trace block is filled with app data it is exposed to host and the second block
 * becomes input one and buffer filling restarts. */
/* While target application fills one TRAX block host reads another one via JTAG. */
/* This module also allows communication in the opposite direction: from host to target. As it was
 * said ESP32 and host can access different TRAX blocks */
/* simultaneously, so while target writes trace data to one block host can write its own data (e.g.
 * tracing commands) to another one then when */
/* blocks are switched host receives trace data and target receives data written by host
 * application. Target user application can read host data */
/* by calling esp32_apptrace_read() API. */
/* To control buffer switching and for other communication purposes this implementation uses some
 * TRAX registers. It is safe since HW TRAX tracing */
/* can not be used along with application tracing feature so these registers are freely
 * readable/writeable via JTAG from host and via ERI from ESP32 cores. */
/* Overhead of this implementation on target CPU is produced only by allocating/managing buffers and
 * copying of data. */
/* On the host side special OpenOCD command must be used to read trace data. */

/* 2. TRAX Registers layout
 * ======================== */

/* This module uses two TRAX HW registers to communicate with host SW (OpenOCD). */
/*  - Control register uses TRAX_DELAYCNT as storage. Only lower 24 bits of TRAX_DELAYCNT are
 * writable. Control register has the following bitfields: */
/*   | 31..XXXXXX..24 | 23 .(host_connect). 23| 22..(block_id)..15 | 14..(block_len)..0 | */
/*    14..0  bits - actual length of user data in trace memory block. Target updates it every time
 * it fills memory block and exposes it to host. */
/*                  Host writes zero to this field when it finishes reading exposed block; */
/*    21..15 bits - trace memory block transfer ID. Block counter. It can overflow. Updated by
 * target, host should not modify it. Actually can be 2 bits; */
/*    22     bit  - 'host data present' flag. If set to one there is data from host, otherwise - no
 * host data; */
/*    23     bit  - 'host connected' flag. If zero then host is not connected and tracing module
 * works in post-mortem mode, otherwise in streaming mode; */
/* - Status register uses TRAX_TRIGGERPC as storage. If this register is not zero then currentlly
 * CPU is changing TRAX registers and */
/*   this register holds address of the instruction which application will execute when it finishes
 * with those registers modifications. */
/*   See 'Targets Connection' setion for details. */

/* 3. Modes of operation
 * ===================== */

/* This module supports two modes of operation: */
/*  - Post-mortem mode. This is the default mode. In this mode application tracing module does not
 * check whether host has read all the data from block */
/*    exposed to it and switches block in any case. The mode does not need host interaction for
 * operation and so can be useful when only the latest */
/*    trace data are necessary, e.g. for analyzing crashes. On panic the latest data from current
 * input block are exposed to host and host can read them. */
/*    It can happen that system panic occurs when there are very small amount of data not read by
 * host yet (e.g. crash just after the TRAX block switch). */
/*    In this case the previous 16KB of collected data will be dropped and host will see the latest,
 * but very small piece of trace. It can be insufficient */
/*    to diagnose the problem. To avoid such situations there is menuconfig option
 * CONFIG_ESP32_APPTRACE_POSTMORTEM_FLUSH_TRAX_THRESH which controls */
/*    the threshold for flushing data in case of panic. */
/*  - Streaming mode. Tracing module enters this mode when host connects to target and sets
 * respective bits in control registers (per core). */
/*    In this mode before switching the block tracing module waits for the host to read all the data
 * from the previously exposed block. */
/*    On panic tracing module also waits (timeout is configured via menuconfig via
 * CONFIG_ESP32_APPTRACE_ONPANIC_HOST_FLUSH_TMO) for the host to read all data. */

/* 4. Communication Protocol
 * ========================= */

/* 4.1 Trace Memory Blocks
 * ----------------------- */

/* Communication is controlled via special register. Host periodically polls control register on
 * each core to find out if there are any data avalable. */
/* When current input memory block is filled it is exposed to host and 'block_len' and 'block_id'
 * fields are updated in the control register. */
/* Host reads new register value and according to it's value starts reading data from exposed block.
 * Meanwhile target starts filling another trace block. */
/* When host finishes reading the block it clears 'block_len' field in control register indicating
 * to the target that it is ready to accept the next one. */
/* If the host has some data to transfer to the target it writes them to trace memory block before
 * clearing 'block_len' field. Then it sets */
/* 'host_data_present' bit and clears 'block_len' field in control register. Upon every block switch
 * target checks 'host_data_present' bit and if it is set */
/* reads them to down buffer before writing any trace data to switched TRAX block. */

/* 4.2 User Data Chunks Level
 * -------------------------- */

/* Since trace memory block is shared between user data chunks and data copying is performed on
 * behalf of the API user (in its normal context) in */
/* multithreading environment it can happen that task/ISR which copies data is preempted by another
 * high prio task/ISR. So it is possible situation */
/* that task/ISR will fail to complete filling its data chunk before the whole trace block is
 * exposed to the host. To handle such conditions tracing */
/* module prepends all user data chunks with header which contains allocated buffer size and actual
 * data length within it. OpenOCD command */
/* which reads application traces reports error when it reads incompleted user data block.
 * Data which are transfered from host to target are also prepended with such header. */

/* 4.3 Data Buffering
 * ------------------ */

/* It takes some time for the host to read TRAX memory block via JTAG. In streaming mode it can
 * happen that target has filled its TRAX block, but host */
/* has not completed reading of the previous one yet. So in this case time critical tracing calls
 * (which can not be delayed for too long time due to */
/* the lack of free memory in TRAX block) can be dropped. To avoid such scenarios tracing module
 * implements data buffering. Buffered data will be sent */
/* to the host later when TRAX block switch occurs. The maximum size of the buffered data is
 * controlled by menuconfig option */
/* CONFIG_ESP32_APPTRACE_PENDED_DATA_SIZE_MAX. */

/* 4.3 Target Connection/Disconnection
 * ----------------------------------- */

/* When host is going to start tracing in streaming mode it needs to put both ESP32 cores into
 * initial state when 'host connected' bit is set */
/* on both cores. To accomplish this host halts both cores and sets this bit in TRAX registers. But
 * target code can be halted in state when it has read control */
/* register but has not updated its value. To handle such situations target code indicates to the
 * host that it is updating control register by writing */
/* non-zero value to status register. Actually it writes address of the instruction which it will
 * execute when it finishes with */
/* the registers update. When target is halted during control register update host sets breakpoint
 * at the address from status register and resumes CPU. */
/* After target code finishes with register update it is halted on breakpoint, host detects it and
 * safely sets 'host connected' bit. When both cores */
/* are set up they are resumed. Tracing starts without further intrusion into CPUs work. */
/* When host is going to stop tracing in streaming mode it needs to disconnect targets.
 * Disconnection process is done using the same algorithm */
/* as for connecting, but 'host connected' bits are cleared on ESP32 cores. */

/* 5. Data Procesing
 * ================= */

/* Target is polled for data periodically. Period depends on tracing command arguments (see the next
 * section). When there are data available they are copied to allocated memory block */
/* and that block is added to the queue for processing by a separate thread. This is done in order
 * to achive the highest possible polling rate and not to lose data due to delays caused */
/* by data processing algorithm. When tracing is stopped OpenOCD waits for all pendded memory blocks
 * to be processed by the thread. */

#include "xtensa.h"
#include "xtensa_debug_module.h"
#include "esp_xtensa_apptrace.h"

/* TRAX is disabled, so we use its registers for our own purposes */
/* | 31..XXXXXX..24 | 23 .(host_connect). 23 | 22 .(host_data). 22| 21..(block_id)..15 |
 * 14..(block_len)..0 | */
#define XTENSA_APPTRACE_CTRL_REG        NARADR_DELAYCNT
/* if non-zero then apptrace code entered the critical section and the value is an address of the
 * critical section's exit point */
#define XTENSA_APPTRACE_STAT_REG        NARADR_TRIGGERPC

#define XTENSA_APPTRACE_BLOCK_LEN_MSK         0x7FFFUL
#define XTENSA_APPTRACE_BLOCK_LEN(_l_)        ((_l_) & XTENSA_APPTRACE_BLOCK_LEN_MSK)
#define XTENSA_APPTRACE_BLOCK_LEN_GET(_v_)    ((_v_) & XTENSA_APPTRACE_BLOCK_LEN_MSK)
#define XTENSA_APPTRACE_BLOCK_ID(_id_)        (((_id_) & XTENSA_APPTRACE_BLOCK_ID_MSK) << 15)
#define XTENSA_APPTRACE_BLOCK_ID_GET(_v_)     (((_v_) >> 15) & XTENSA_APPTRACE_BLOCK_ID_MSK)
#define XTENSA_APPTRACE_HOST_DATA             (1 << 22)
#define XTENSA_APPTRACE_HOST_CONNECT          (1 << 23)


static int esp_xtensa_apptrace_data_reverse_read(struct xtensa *xtensa,
	uint32_t size,
	uint8_t *buffer,
	uint8_t *unal_bytes)
{
	int res = 0;
	uint32_t i, rd_sz = size;

	if (size & 0x3UL)
		rd_sz = (size + 0x3UL) & ~0x3UL;
	res =
		xtensa_queue_dbg_reg_write(xtensa, NARADR_TRAXADDR,
		(xtensa->core_config->trace.mem_sz-rd_sz)/4);
	if (res != ERROR_OK)
		return res;
	if (size & 0x3UL) {
		res = xtensa_queue_dbg_reg_read(xtensa, NARADR_TRAXDATA, unal_bytes);
		if (res != ERROR_OK)
			return res;
	}
	for (i = size/4; i > 0; i--) {
		res = xtensa_queue_dbg_reg_read(xtensa, NARADR_TRAXDATA, &buffer[(i-1)*4]);
		if (res != ERROR_OK)
			return res;
	}
	return ERROR_OK;
}

static int esp_xtensa_apptrace_data_normal_read(struct xtensa *xtensa,
	uint32_t size,
	uint8_t *buffer,
	uint8_t *unal_bytes)
{
	int res = xtensa_queue_dbg_reg_write(xtensa, NARADR_TRAXADDR, 0);
	if (res != ERROR_OK)
		return res;
	for (uint32_t i = 0; i < size/4; i++) {
		res = xtensa_queue_dbg_reg_read(xtensa, NARADR_TRAXDATA, &buffer[i*4]);
		if (res != ERROR_OK)
			return res;
	}
	if (size & 0x3UL) {
		res = xtensa_queue_dbg_reg_read(xtensa, NARADR_TRAXDATA, unal_bytes);
		if (res != ERROR_OK)
			return res;
	}
	return ERROR_OK;
}

int esp_xtensa_apptrace_data_read(struct target *target,
	uint32_t size,
	uint8_t *buffer,
	uint32_t block_id,
	bool ack)
{
	struct xtensa *xtensa = target_to_xtensa(target);
	int res = 0;
	uint32_t tmp = XTENSA_APPTRACE_HOST_CONNECT | XTENSA_APPTRACE_BLOCK_ID(block_id) |
		XTENSA_APPTRACE_BLOCK_LEN(0);
	uint8_t unal_bytes[4];

	LOG_DEBUG("Read data on target (%s)", target_name(target));
	if (xtensa->core_config->trace.reversed_mem_access)
		res = esp_xtensa_apptrace_data_reverse_read(xtensa, size, buffer, unal_bytes);
	else
		res = esp_xtensa_apptrace_data_normal_read(xtensa, size, buffer, unal_bytes);
	if (res != ERROR_OK)
		return res;
	if (ack) {
		LOG_DEBUG("Ack block %d target (%s)!", block_id, target_name(target));
		res = xtensa_queue_dbg_reg_write(xtensa, XTENSA_APPTRACE_CTRL_REG, tmp);
		if (res != ERROR_OK)
			return res;
	}
	xtensa_dm_queue_tdi_idle(&xtensa->dbg_mod);
	res = jtag_execute_queue();
	if (res != ERROR_OK) {
		LOG_ERROR("Failed to exec JTAG queue!");
		return res;
	}
	if (size & 0x3UL) {
		/* copy the last unaligned bytes */
		memcpy(buffer + size - (size & 0x3UL), unal_bytes, size & 0x3UL);
	}
	return ERROR_OK;
}

int esp_xtensa_apptrace_ctrl_reg_write(struct target *target,
	uint32_t block_id,
	uint32_t len,
	bool conn,
	bool data)
{
	struct xtensa *xtensa = target_to_xtensa(target);
	int res = ERROR_OK;
	uint32_t tmp = (conn ? XTENSA_APPTRACE_HOST_CONNECT : 0) |
		(data ? XTENSA_APPTRACE_HOST_DATA : 0) | XTENSA_APPTRACE_BLOCK_ID(block_id) |
		XTENSA_APPTRACE_BLOCK_LEN(len);

	xtensa_queue_dbg_reg_write(xtensa, XTENSA_APPTRACE_CTRL_REG, tmp);
	xtensa_dm_queue_tdi_idle(&xtensa->dbg_mod);
	res = jtag_execute_queue();
	if (res != ERROR_OK) {
		LOG_ERROR("Failed to exec JTAG queue!");
		return res;
	}

	return ERROR_OK;
}

int esp_xtensa_apptrace_ctrl_reg_read(struct target *target,
	uint32_t *block_id,
	uint32_t *len,
	bool *conn)
{
	struct xtensa *xtensa = target_to_xtensa(target);
	int res = 0;
	uint8_t tmp[4];

	xtensa_queue_dbg_reg_read(xtensa, XTENSA_APPTRACE_CTRL_REG, tmp);
	xtensa_dm_queue_tdi_idle(&xtensa->dbg_mod);
	res = jtag_execute_queue();
	if (res != ERROR_OK)
		return res;
	uint32_t val = buf_get_u32(tmp, 0, 32);
	if (block_id)
		*block_id = XTENSA_APPTRACE_BLOCK_ID_GET(val);
	if (len)
		*len = XTENSA_APPTRACE_BLOCK_LEN_GET(val);
	if (conn)
		*conn = val & XTENSA_APPTRACE_HOST_CONNECT;
	return ERROR_OK;
}

int esp_xtensa_apptrace_status_reg_read(struct target *target, uint32_t *stat)
{
	struct xtensa *xtensa = target_to_xtensa(target);
	int res = 0;
	uint8_t tmp[4];

	xtensa_queue_dbg_reg_read(xtensa, XTENSA_APPTRACE_STAT_REG, tmp);
	xtensa_dm_queue_tdi_idle(&xtensa->dbg_mod);
	res = jtag_execute_queue();
	if (res != ERROR_OK) {
		LOG_ERROR("Failed to exec JTAG queue!");
		return res;
	}
	*stat = buf_get_u32(tmp, 0, 32);
	return ERROR_OK;
}

int esp_xtensa_apptrace_status_reg_write(struct target *target, uint32_t stat)
{
	struct xtensa *xtensa = target_to_xtensa(target);
	int res = 0;

	xtensa_queue_dbg_reg_write(xtensa, XTENSA_APPTRACE_STAT_REG, stat);
	xtensa_dm_queue_tdi_idle(&xtensa->dbg_mod);
	res = jtag_execute_queue();
	if (res != ERROR_OK) {
		LOG_ERROR("Failed to exec JTAG queue!");
		return res;
	}
	return ERROR_OK;
}

int esp_xtensa_swdbg_activate(struct target *target, int enab)
{
	struct xtensa *xtensa = target_to_xtensa(target);
	int res;

	xtensa_queue_dbg_reg_write(xtensa,
		enab ? NARADR_DCRSET : NARADR_DCRCLR,
		OCDDCR_DEBUGSWACTIVE);
	xtensa_dm_queue_tdi_idle(&xtensa->dbg_mod);
	res = jtag_execute_queue();
	if (res != ERROR_OK) {
		LOG_ERROR("%s: writing DCR failed!", target->cmd_name);
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

/* TODO: for now this function assumes the same endianness on the target and host */
static int esp_xtensa_apptrace_queue_reverse_write(struct xtensa *xtensa, uint32_t bufs_num,
	uint32_t buf_sz[], const uint8_t *bufs[])
{
	int res = ERROR_OK;
	uint32_t cached_bytes = 0, total_sz = 0;
	union {
		uint8_t data8[4];
		uint32_t data32;
	} dword_cache;

	for (uint32_t i = 0; i < bufs_num; i++)
		total_sz += buf_sz[i];
	if (total_sz & 0x3UL) {
		cached_bytes = 4 - (total_sz & 0x3UL);
		total_sz = (total_sz + 0x3UL) & ~0x3UL;
	}
	dword_cache.data32 = 0;
	xtensa_queue_dbg_reg_write(xtensa, NARADR_TRAXADDR,
		(xtensa->core_config->trace.mem_sz - total_sz) / 4);
	for (uint32_t i = bufs_num; i > 0; i--) {
		uint32_t bsz = buf_sz[i-1];
		const uint8_t *cur_buf = &bufs[i-1][bsz];
		uint32_t bytes_to_cache;
		/* if there are cached bytes from the previous buffer, combine them with the last
		 * from the current buffer */
		if (cached_bytes) {
			bytes_to_cache = 0;
			if ((cached_bytes + bsz) < sizeof(uint32_t))
				bytes_to_cache = bsz;
			else
				bytes_to_cache = sizeof(uint32_t) - cached_bytes;
			memcpy(&dword_cache.data8[sizeof(uint32_t) - cached_bytes - bytes_to_cache],
				cur_buf-bytes_to_cache,
				bytes_to_cache);
			cached_bytes += bytes_to_cache;
			if (cached_bytes < sizeof(uint32_t))
				continue;
			res =
				xtensa_queue_dbg_reg_write(xtensa, NARADR_TRAXDATA,
				dword_cache.data32);
			if (res != ERROR_OK)
				return res;
			bsz -= bytes_to_cache;
			cur_buf -= bytes_to_cache;
			dword_cache.data32 = 0;
			cached_bytes = 0;
		}
		/* write full dwords */
		for (uint32_t k = bsz; k >= sizeof(uint32_t); k -= sizeof(uint32_t)) {
			res =
				xtensa_queue_dbg_reg_write(xtensa, NARADR_TRAXDATA,
				*((uint32_t *)(void *)(cur_buf-sizeof(uint32_t))));
			if (res != ERROR_OK)
				return res;
			cur_buf -= sizeof(uint32_t);
		}
		/* if there are bytes to be cached (1..3) */
		bytes_to_cache = bsz & 0x3UL;
		if (bytes_to_cache > 0) {
			if (bytes_to_cache + cached_bytes >= sizeof(uint32_t)) {
				/* filling the cache buffer from the end to beginning */
				uint32_t to_copy = sizeof(uint32_t) - cached_bytes;
				memcpy(&dword_cache.data8, cur_buf-to_copy, to_copy);
				/* write full word of cached bytes */
				res = xtensa_queue_dbg_reg_write(xtensa,
					NARADR_TRAXDATA,
					dword_cache.data32);
				if (res != ERROR_OK)
					return res;
				/* cache remaining bytes */
				dword_cache.data32 = 0;
				cur_buf -= to_copy;
				to_copy = bytes_to_cache + cached_bytes - sizeof(uint32_t);
				memcpy(&dword_cache.data8[sizeof(uint32_t) - to_copy],
					cur_buf-to_copy,
					to_copy);
				cached_bytes = to_copy;
			} else {
				/* filling the cache buffer from the end to beginning */
				memcpy(&dword_cache.data8[sizeof(uint32_t) - cached_bytes -
						bytes_to_cache],
					cur_buf-bytes_to_cache,
					bytes_to_cache);
				cached_bytes += bytes_to_cache;
			}
		}
	}
	return ERROR_OK;
}

/* TODO: for now this function assumes the same endianness on the target and host */
static int esp_xtensa_apptrace_queue_normal_write(struct xtensa *xtensa, uint32_t bufs_num,
	uint32_t buf_sz[], const uint8_t *bufs[])
{
	int res = ERROR_OK;
	uint32_t cached_bytes = 0;
	union {
		uint8_t data8[4];
		uint32_t data32;
	} dword_cache;

	/* | 1 |   2   | 1 | 2     |       4       |.......|
	 * |       4       |       4       |       4       | */
	dword_cache.data32 = 0;
	xtensa_queue_dbg_reg_write(xtensa, NARADR_TRAXADDR, 0);
	for (uint32_t i = 0; i < bufs_num; i++) {
		uint32_t bsz = buf_sz[i];
		const uint8_t *cur_buf = bufs[i];
		uint32_t bytes_to_cache;
		/* if there are cached bytes from the previous buffer, combine them with the last
		 * from the current buffer */
		if (cached_bytes) {
			bytes_to_cache = 0;
			if ((cached_bytes + bsz) < sizeof(uint32_t))
				bytes_to_cache = bsz;
			else
				bytes_to_cache = sizeof(uint32_t) - cached_bytes;
			memcpy(&dword_cache.data8[cached_bytes], cur_buf, bytes_to_cache);
			cached_bytes += bytes_to_cache;
			if (cached_bytes < sizeof(uint32_t))
				continue;
			res =
				xtensa_queue_dbg_reg_write(xtensa, NARADR_TRAXDATA,
				dword_cache.data32);
			if (res != ERROR_OK)
				return res;
			bsz -= bytes_to_cache;
			cur_buf += bytes_to_cache;
			dword_cache.data32 = 0;
			cached_bytes = 0;
		}
		/* write full dwords */
		for (uint32_t k = 0; (k+sizeof(uint32_t)) <= bsz; k += sizeof(uint32_t)) {
			res =
				xtensa_queue_dbg_reg_write(xtensa, NARADR_TRAXDATA,
				*((uint32_t *)(void *)cur_buf));
			if (res != ERROR_OK)
				return res;
			cur_buf += sizeof(uint32_t);
		}
		/* if there are bytes to be cached (1..3) */
		bytes_to_cache = bsz & 0x3UL;
		if (bytes_to_cache > 0) {
			if (bytes_to_cache + cached_bytes >= sizeof(uint32_t)) {
				memcpy(&dword_cache.data8[cached_bytes],
					cur_buf,
					sizeof(uint32_t) - cached_bytes);
				/* write full word of cached bytes */
				res = xtensa_queue_dbg_reg_write(xtensa,
					NARADR_TRAXDATA,
					dword_cache.data32);
				if (res != ERROR_OK)
					return res;
				/* cache remaining bytes */
				dword_cache.data32 = 0;
				cur_buf += sizeof(uint32_t) - cached_bytes;
				memcpy(dword_cache.data8,
					cur_buf,
					bytes_to_cache + cached_bytes - sizeof(uint32_t));
				cached_bytes = bytes_to_cache + cached_bytes - sizeof(uint32_t);
			} else {
				memcpy(&dword_cache.data8[cached_bytes], cur_buf, bytes_to_cache);
				cached_bytes += bytes_to_cache;
			}
		}
	}
	if (cached_bytes) {
		/* write remaining cached bytes */
		res = xtensa_queue_dbg_reg_write(xtensa, NARADR_TRAXDATA, dword_cache.data32);
		if (res != ERROR_OK)
			return res;
	}
	return ERROR_OK;
}

int esp_xtensa_apptrace_buffs_write(struct target *target,
	uint32_t bufs_num,
	uint32_t buf_sz[],
	const uint8_t *bufs[],
	uint32_t block_id,
	int ack,
	int data)
{
	struct xtensa *xtensa = target_to_xtensa(target);
	int res = ERROR_OK;
	uint32_t tmp = (1 << 24) | XTENSA_APPTRACE_HOST_CONNECT |
		(data ? XTENSA_APPTRACE_HOST_DATA : 0) | XTENSA_APPTRACE_BLOCK_ID(block_id) |
		XTENSA_APPTRACE_BLOCK_LEN(0);

	if (xtensa->core_config->trace.reversed_mem_access)
		res = esp_xtensa_apptrace_queue_reverse_write(xtensa, bufs_num, buf_sz, bufs);
	else
		res = esp_xtensa_apptrace_queue_normal_write(xtensa, bufs_num, buf_sz, bufs);
	if (res != ERROR_OK)
		return res;
	if (ack) {
		LOG_DEBUG("Ack block %d on target (%s)!", block_id, target_name(target));
		res = xtensa_queue_dbg_reg_write(xtensa, XTENSA_APPTRACE_CTRL_REG, tmp);
		if (res != ERROR_OK)
			return res;
	}
	xtensa_dm_queue_tdi_idle(&xtensa->dbg_mod);
	res = jtag_execute_queue();
	if (res != ERROR_OK) {
		LOG_ERROR("Failed to exec JTAG queue!");
		return res;
	}
	return ERROR_OK;
}

uint8_t *esp_xtensa_apptrace_usr_block_get(uint8_t *buffer, uint32_t *size)
{
	struct esp_xtensa_apptrace_target2host_hdr tmp_hdr;
	memcpy(&tmp_hdr, buffer, sizeof(tmp_hdr));

	*size = tmp_hdr.gen.wr_sz;

	return buffer + sizeof(struct esp_xtensa_apptrace_target2host_hdr);
}

int esp_xtensa_apptrace_usr_block_write(struct target *core_target,
	uint32_t block_id,
	const uint8_t *data,
	uint32_t size)
{
	struct esp_xtensa_apptrace_host2target_hdr hdr = {.block_sz = size};
	uint32_t buf_sz[2] = {sizeof(struct esp_xtensa_apptrace_host2target_hdr), size};
	const uint8_t *bufs[2] = {(const uint8_t *)&hdr, data};
	/* uint8_t test_data1[1] = {0xa};
	 * uint16_t test_data2 = 0xbecf;
	 * uint8_t test_data3[1] = {0x5};
	 * uint8_t test_data4[3] = {0xd, 0x6, 0x9};
	 * struct esp_xtensa_apptrace_host2target_hdr hdr = {.block_sz = 7}; */
	/* uint32_t buf_sz[] = {sizeof(struct esp_xtensa_apptrace_host2target_hdr),
	 * sizeof(test_data1), sizeof(test_data2), sizeof(test_data3), sizeof(test_data4)}; */
	/* const uint8_t *bufs[] = {(const uint8_t *)&hdr, test_data1, (uint8_t *)&test_data2,
	 * test_data3, test_data4}; */

	if (size > esp_xtensa_apptrace_usr_block_max_size_get(core_target)) {
		LOG_ERROR("Too large user block %u", size);
		return ERROR_FAIL;
	}

	return esp_xtensa_apptrace_buffs_write(core_target,
		sizeof(buf_sz)/sizeof(buf_sz[0]),
		buf_sz,
		bufs,
		block_id,
		1 /*ack target data*/,
		1 /*host data*/);
}

int esp_xtensa_sysview_cmds_queue(struct target *core_target,
	uint8_t *cmds,
	uint32_t cmds_num,
	uint32_t block_id)
{
	for (uint32_t i = 0; i < cmds_num; i++)
		LOG_DEBUG("SEGGER: Send command %d", cmds[i]);
	/* write header with or without data */
	int res = esp_xtensa_apptrace_usr_block_write(core_target, block_id, cmds, cmds_num);
	if (res != ERROR_OK) {
		LOG_ERROR("SEGGER: Failed to write data to (%s)!", target_name(core_target));
		return res;
	}
	return res;
}
