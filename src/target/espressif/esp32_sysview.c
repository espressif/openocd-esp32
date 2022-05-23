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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <helper/log.h>
#include "esp32_apptrace.h"
#include "esp32_sysview.h"

/* in SystemView mode core ID is passed in event ID field */
#define ESP32_SYSVIEW_USER_BLOCK_CORE(_v_)  (0)	/* not used */
#define ESP32_SYSVIEW_USER_BLOCK_LEN(_v_)       (_v_)
#define ESP32_SYSVIEW_USER_BLOCK_HDR_SZ     2

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
}

struct esp_sysview_target2host_hdr {
	uint8_t block_sz;
	uint8_t wr_sz;
};

static int esp_sysview_trace_header_write(struct esp32_apptrace_cmd_ctx *ctx, bool mcore_format);
static int esp32_sysview_core_id_get(uint8_t *hdr_buf);
static uint32_t esp32_sysview_usr_block_len_get(uint8_t *hdr_buf, uint32_t *wr_len);


int esp32_sysview_cmd_init(struct target *target,
	struct esp32_apptrace_cmd_ctx *cmd_ctx,
	int mode,
	bool mcore_format,
	const char **argv,
	int argc)
{
	int res;
	struct esp32_sysview_cmd_data *cmd_data;

	if (argc < 1) {
		LOG_ERROR("Not enough args! Need trace data destination!");
		return ERROR_FAIL;
	}

	res = esp32_apptrace_cmd_ctx_init(target, cmd_ctx, mode);
	if (res)
		return res;

	if (!mcore_format && argc < cmd_ctx->cores_num) {
		LOG_ERROR("Not enough args! Need %d trace data destinations!", cmd_ctx->cores_num);
		res = ERROR_FAIL;
		goto on_error;
	}

	cmd_data = calloc(1, sizeof(*cmd_data));
	assert(cmd_data && "No memory for command data!");
	cmd_ctx->cmd_priv = cmd_data;
	cmd_data->mcore_format = mcore_format;

	/*outfile1 [outfile2] [poll_period [trace_size [stop_tmo [wait4halt [skip_size]]]]] */
	int dests_num = esp32_apptrace_dest_init(cmd_data->data_dests,
		argv,
		!mcore_format ? cmd_ctx->cores_num : 1);
	if (!mcore_format && dests_num < cmd_ctx->cores_num) {
		LOG_ERROR("Not enough args! Need %d trace data destinations!", cmd_ctx->cores_num);
		free(cmd_data);
		res = ERROR_FAIL;
		goto on_error;
	}
	cmd_data->apptrace.max_len = (uint32_t)-1;
	cmd_data->apptrace.poll_period = 0 /*ms*/;
	cmd_ctx->stop_tmo = -1.0;	/* infinite */
	if (argc > dests_num) {
		/* parse remaining args */
		esp32_apptrace_cmd_args_parse(cmd_ctx,
			&cmd_data->apptrace,
			&argv[dests_num],
			argc - dests_num);
	}
	LOG_USER(
		"App trace params: from %d cores, size %u bytes, stop_tmo %g s, poll period %u ms, wait_rst %d, skip %u bytes",
		cmd_ctx->cores_num,
		cmd_data->apptrace.max_len,
		cmd_ctx->stop_tmo,
		cmd_data->apptrace.poll_period,
		cmd_data->apptrace.wait4halt,
		cmd_data->apptrace.skip_len);

	cmd_ctx->trace_format.hdr_sz = ESP32_SYSVIEW_USER_BLOCK_HDR_SZ;
	cmd_ctx->trace_format.core_id_get = esp32_sysview_core_id_get;
	cmd_ctx->trace_format.usr_block_len_get = esp32_sysview_usr_block_len_get;

	res = esp_sysview_trace_header_write(cmd_ctx, mcore_format);
	if (res != ERROR_OK) {
		LOG_ERROR("Failed to write trace header (%d)!", res);
		esp32_apptrace_dest_cleanup(cmd_data->data_dests, cmd_ctx->cores_num);
		free(cmd_data);
		return res;
	}
	return ERROR_OK;
on_error:
	cmd_ctx->running = 0;
	esp32_apptrace_cmd_ctx_cleanup(cmd_ctx);
	return res;
}

int esp32_sysview_cmd_cleanup(struct esp32_apptrace_cmd_ctx *cmd_ctx)
{
	struct esp32_sysview_cmd_data *cmd_data = cmd_ctx->cmd_priv;

	esp32_apptrace_dest_cleanup(cmd_data->data_dests, cmd_ctx->cores_num);
	free(cmd_data);
	cmd_ctx->cmd_priv = NULL;
	esp32_apptrace_cmd_ctx_cleanup(cmd_ctx);
	return ERROR_OK;
}

static int esp32_sysview_core_id_get(uint8_t *hdr_buf)
{
	/* for sysview compressed apptrace header is used, so core id is encoded in sysview packet
	 **/
	return 0;
}

static uint32_t esp32_sysview_usr_block_len_get(uint8_t *hdr_buf, uint32_t *wr_len)
{
	struct esp_sysview_target2host_hdr tmp_hdr;
	memcpy(&tmp_hdr, hdr_buf, sizeof(tmp_hdr));
	*wr_len = ESP32_SYSVIEW_USER_BLOCK_LEN(tmp_hdr.wr_sz);
	return ESP32_SYSVIEW_USER_BLOCK_LEN(tmp_hdr.block_sz);
}

static int esp_sysview_trace_header_write(struct esp32_apptrace_cmd_ctx *ctx, bool mcore_format)
{
	struct esp32_sysview_cmd_data *cmd_data = ctx->cmd_priv;
	char *hdr_str;
	int dests_num;

	if (!mcore_format) {
		hdr_str = ";\n"
			"; Version     SEGGER SystemViewer V2.42\n"
			"; Author      Espressif Inc\n"
			";\n";
		dests_num = ctx->cores_num;
	} else {
		hdr_str = ";\n"
			"; Version     SEGGER SystemViewer V2.42\n"
			"; Author      Espressif Inc\n"
			"; ESP_Extension\n"
			";\n";
		dests_num = 1;
	}

	int hdr_len = strlen(hdr_str);
	for (int i = 0; i < dests_num; i++) {
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

static uint32_t esp_sysview_decode_u32(uint8_t **ptr)
{
	uint32_t val = 0;
	for (int k = 0;; k++, (*ptr)++) {
		if (**ptr & 0x80) {
			val |= (uint32_t)(**ptr & ~0x80) << 7 * k;
		} else {
			val |= (uint32_t)**ptr << 7 * k;
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
	uint32_t *delta_len,
	bool clear_core_bit)
{
	uint8_t *pkt = pkt_buf;
	uint16_t event_id = 0, payload_len = 0;

	*pkt_core_id = 0;
	*pkt_len = 0;
	/* 1-2 byte of message type, 0-2  byte of payload length, payload, 1-5 bytes of timestamp.
	 * */
	if (*pkt & 0x80) {
		if (*(pkt + 1) & (1 << 6)) {
			if (clear_core_bit)
				*(pkt + 1) &= ~(1 << 6);	/* clear core_id bit */
			*pkt_core_id = 1;
		}
		event_id = *(pkt + 1) & ~(1 << 6);	/* higher part */
		event_id = (event_id << 7) | (*pkt & ~0x80);	/* lower 7 bits */
		pkt += 2;	/* event_id (2 bytes) */
		/* here pkt points to encoded payload length */
		payload_len = esp_sysview_decode_plen(&pkt);
	} else {
		if (*pkt & (1 << 6)) {
			if (clear_core_bit)
				*pkt &= ~(1 << 6);	/* clear core_id bit */
			*pkt_core_id = 1;
		}
		/* event_id (1 byte) */
		event_id = *pkt & ~(1 << 6);
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

static int esp32_sysview_write_packet(struct esp32_sysview_cmd_data *cmd_data,
	int pkt_core_id, uint32_t pkt_len, uint8_t *pkt_buf, uint32_t delta_len, uint8_t *delta_buf)
{
	int res = cmd_data->data_dests[pkt_core_id].write(
		cmd_data->data_dests[pkt_core_id].priv,
		pkt_buf,
		pkt_len);
	if (res != ERROR_OK) {
		LOG_ERROR("SEGGER: Failed to write %u bytes to dest %d!", pkt_len, pkt_core_id);
		return res;
	}
	if (delta_len) {
		/* write packet with modified delta */
		res =
			cmd_data->data_dests[pkt_core_id].write(
			cmd_data->data_dests[pkt_core_id].priv,
			delta_buf,
			delta_len);
		if (res != ERROR_OK) {
			LOG_ERROR("SEGGER: Failed to write %u bytes of delta to dest %d!",
				delta_len,
				pkt_core_id);
			return res;
		}
	}
	return ERROR_OK;
}

static int esp32_sysview_process_packet(struct esp32_apptrace_cmd_ctx *ctx,
	int pkt_core_id, uint16_t event_id, uint32_t delta, uint32_t delta_len,
	uint32_t pkt_len, uint8_t *pkt_buf)
{
	struct esp32_sysview_cmd_data *cmd_data = ctx->cmd_priv;
	int pkt_core_changed = 0;
	uint32_t new_delta_len = 0;
	uint8_t new_delta_buf[10];
	uint32_t wr_len = pkt_len;

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
	if (pkt_core_id >= ctx->cores_num) {
		LOG_WARNING(
			"SEGGER: invalid core ID in packet %d, must be less then %d! Event id %d",
			pkt_core_id,
			ctx->cores_num,
			event_id);
		return ERROR_FAIL;
	}
	int res = esp32_sysview_write_packet(cmd_data,
		pkt_core_id,
		wr_len,
		pkt_buf,
		new_delta_len,
		new_delta_buf);
	if (res != ERROR_OK)
		return res;
	for (int i = 0; i < ctx->cores_num; i++) {
		if (pkt_core_id == i)
			continue;
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
				i);
			res = esp32_sysview_write_packet(cmd_data,
				i,
				wr_len,
				pkt_buf,
				new_delta_len,
				new_delta_buf);
			if (res != ERROR_OK)
				return res;
			/* messages above are cloned to trace files for both cores,
			* so reset acc time delta, both files have actual delta
			* info */
			cmd_data->sv_acc_time_delta = 0;
			break;
		default:
			break;
		}
	}
	return ERROR_OK;
}

int esp32_sysview_process_data(struct esp32_apptrace_cmd_ctx *ctx,
	int core_id,
	uint8_t *data,
	uint32_t data_len)
{
	struct esp32_sysview_cmd_data *cmd_data = ctx->cmd_priv;

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
	if (cmd_data->mcore_format)
		core_id = 0;
	if (ctx->tot_len == 0) {
		/* handle sync seq */
		if (data_len < SYSVIEW_SYNC_LEN) {
			LOG_ERROR("SEGGER: Invalid init seq len %d!", data_len);
			return ERROR_FAIL;
		}
		LOG_DEBUG("SEGGER: Process %d sync bytes", SYSVIEW_SYNC_LEN);
		uint8_t sync_seq[SYSVIEW_SYNC_LEN] =
		{ 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0 };
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
		if (!cmd_data->mcore_format) {
			for (int i = 0; i < ctx->cores_num; i++) {
				if (core_id == i)
					continue;
				res =
					cmd_data->data_dests[i].write(cmd_data->data_dests[i].priv,
					data,
					SYSVIEW_SYNC_LEN);
				if (res != ERROR_OK) {
					LOG_ERROR(
						"SEGGER: Failed to write %u sync bytes to dest %d!",
						SYSVIEW_SYNC_LEN,
						core_id ? 0 : 1);
					return res;
				}
			}
		}
		ctx->tot_len += SYSVIEW_SYNC_LEN;
		processed += SYSVIEW_SYNC_LEN;
	}
	while (processed < data_len) {
		int pkt_core_id;
		uint32_t delta_len = 0;
		uint32_t pkt_len = 0, delta = 0;
		uint16_t event_id = esp_sysview_parse_packet(data + processed,
			&pkt_len,
			&pkt_core_id,
			&delta,
			&delta_len,
			!cmd_data->mcore_format);
		LOG_DEBUG("SEGGER: Process packet: core %d, %d id, %d bytes [%x %x %x %x]",
			pkt_core_id,
			event_id,
			pkt_len,
			data[processed + 0],
			data[processed + 1],
			data[processed + 2],
			data[processed + 3]);
		if (!cmd_data->mcore_format) {
			res = esp32_sysview_process_packet(ctx,
				pkt_core_id,
				event_id,
				delta,
				delta_len,
				pkt_len,
				data + processed);
			if (res != ERROR_OK)
				return res;
		} else {
			res = cmd_data->data_dests[0].write(
				cmd_data->data_dests[0].priv,
				data + processed,
				pkt_len);
			if (res != ERROR_OK) {
				LOG_ERROR("SEGGER: Failed to write %u bytes to dest %d!",
					pkt_len,
					0);
				return res;
			}
		}
		if (event_id == SYSVIEW_EVTID_TRACE_STOP)
			cmd_data->sv_trace_running = 0;
		ctx->tot_len += pkt_len;
		processed += pkt_len;
	}
	LOG_USER("%u ", ctx->tot_len);
	/* check for stop condition */
	if ((ctx->tot_len > cmd_data->apptrace.skip_len) &&
		(ctx->tot_len - cmd_data->apptrace.skip_len >= cmd_data->apptrace.max_len)) {
		ctx->running = 0;
		if (duration_measure(&ctx->read_time) != 0) {
			LOG_ERROR("Failed to stop trace read time measure!");
			return ERROR_FAIL;
		}
	}
	return ERROR_OK;
}
