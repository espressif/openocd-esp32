// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   RISC-V trace encoder control for Espressif chips                      *
 *   Copyright (C) 2026 Espressif Systems Ltd.                             *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <helper/bits.h>
#include <helper/fileio.h>
#include <helper/log.h>
#include <target/target.h>
#include "esp_riscv_trace_desc.h"
#include "esp_riscv_trace_file.h"

/*
 * The .esptrace file is a header followed by records. Little-endian.
 *   header  magic, version, header size, RVTS root, encoder params, app ELF SHA-256
 *   record  type, core, flags, payload length, payload
 */
#define ESPTRACE_MAGIC             0x54505345
#define ESPTRACE_VERSION           1
#define ESPTRACE_OFF_MAGIC         0
#define ESPTRACE_OFF_VERSION       4
#define ESPTRACE_OFF_HDR_SIZE      6
#define ESPTRACE_OFF_SNAPSHOT      8
#define ESPTRACE_OFF_PARAMS        (ESPTRACE_OFF_SNAPSHOT + RVTS_DESC_SIZE)
#define ESPTRACE_OFF_SHA256        (ESPTRACE_OFF_PARAMS + RVTS_PARAMS_SIZE)
#define ESPTRACE_HDR_SIZE          (ESPTRACE_OFF_SHA256 + RVTS_SHA256_SIZE)

#define ESPTRACE_REC_OFF_TYPE      0
#define ESPTRACE_REC_OFF_CORE      1
#define ESPTRACE_REC_OFF_FLAGS     2
#define ESPTRACE_REC_OFF_LEN       4
#define ESPTRACE_REC_HDR_SIZE      8

#define ESPTRACE_REC_CORE_DESC     1
#define ESPTRACE_REC_DATA          2
#define ESPTRACE_REC_END           3
#define ESPTRACE_REC_NO_CORE       0xFF

#define ESP_RISCV_TRACE_READ_CHUNK 4096

static int esptrace_write(struct fileio *file, const void *buf, size_t size)
{
	size_t written;
	int res = fileio_write(file, size, buf, &written);
	if (res != ERROR_OK || written != size) {
		LOG_ERROR("Failed to write the trace file");
		return ERROR_FAIL;
	}
	return ERROR_OK;
}

static int esptrace_write_record_hdr(struct fileio *file, uint8_t type, uint8_t core, uint16_t flags,
	uint32_t len)
{
	uint8_t hdr[ESPTRACE_REC_HDR_SIZE];

	hdr[ESPTRACE_REC_OFF_TYPE] = type;
	hdr[ESPTRACE_REC_OFF_CORE] = core;
	h_u16_to_le(hdr + ESPTRACE_REC_OFF_FLAGS, flags);
	h_u32_to_le(hdr + ESPTRACE_REC_OFF_LEN, len);
	return esptrace_write(file, hdr, sizeof(hdr));
}

/* Builds the file header from the snapshot root, encoder params and app ELF SHA-256. */
static int esptrace_build_hdr(struct target *target, const struct esp_riscv_trace_snapshot_desc *snap,
	const struct esp_riscv_trace_snapshot_raw *raw, uint8_t *hdr)
{
	memset(hdr, 0, ESPTRACE_HDR_SIZE);
	h_u32_to_le(hdr + ESPTRACE_OFF_MAGIC, ESPTRACE_MAGIC);
	h_u16_to_le(hdr + ESPTRACE_OFF_VERSION, ESPTRACE_VERSION);
	h_u16_to_le(hdr + ESPTRACE_OFF_HDR_SIZE, ESPTRACE_HDR_SIZE);
	memcpy(hdr + ESPTRACE_OFF_SNAPSHOT, raw->desc, RVTS_DESC_SIZE);

	if (snap->encoder_params_addr == 0 || snap->encoder_params_size != RVTS_PARAMS_SIZE) {
		LOG_ERROR("RISC-V trace snapshot has no encoder parameters");
		return ERROR_FAIL;
	}
	uint8_t *params = hdr + ESPTRACE_OFF_PARAMS;
	int res = target_read_buffer(target, snap->encoder_params_addr, RVTS_PARAMS_SIZE, params);
	if (res != ERROR_OK)
		return res;
	if (params[RVTS_PARAMS_OFF_VERSION] != RVTS_PARAMS_VERSION ||
		params[RVTS_PARAMS_OFF_SIZE] != RVTS_PARAMS_SIZE) {
		LOG_ERROR("Unsupported RISC-V trace encoder parameters (version %u, size %u)",
			params[RVTS_PARAMS_OFF_VERSION], params[RVTS_PARAMS_OFF_SIZE]);
		return ERROR_FAIL;
	}

	res = esp_riscv_trace_read_app_sha256(target, snap, hdr + ESPTRACE_OFF_SHA256);
	if (res != ERROR_OK)
		LOG_ERROR("RISC-V trace app ELF SHA-256 not available");
	return res;
}

/* Writes one DATA record with the whole trace buffer of a core. *wrapped is set when data follows
 * the head. Start clears the buffer, so a loop capture that wrapped has no zero tail. */
static int esptrace_write_core_data(struct target *target, struct fileio *file,
	const struct esp_riscv_trace_snapshot_core_desc *core, bool *wrapped)
{
	*wrapped = false;
	uint8_t chunk[ESP_RISCV_TRACE_READ_CHUNK];

	int res = esptrace_write_record_hdr(file, ESPTRACE_REC_DATA, core->core_id, 0, core->capacity);
	if (res != ERROR_OK)
		return res;

	for (uint32_t off = 0; off < core->capacity; off += sizeof(chunk)) {
		uint32_t len = MIN(core->capacity - off, sizeof(chunk));
		res = target_read_buffer(target, core->buffer_addr + off, len, chunk);
		if (res != ERROR_OK)
			return res;
		for (uint32_t i = 0; core->head_valid && !*wrapped && i < len; i++)
			*wrapped = off + i >= core->head_offset && chunk[i];
		res = esptrace_write(file, chunk, len);
		if (res != ERROR_OK)
			return res;
	}
	return ERROR_OK;
}

static int esptrace_write_contents(struct target *target, struct fileio *file,
	const struct esp_riscv_trace_target_state *state, uint32_t core_mask, uint32_t end_status, uint32_t *wrapped_mask)
{
	const struct esp_riscv_trace_snapshot_desc *snap = &state->snap;
	const struct esp_riscv_trace_snapshot_core_desc *cores = state->cores;
	*wrapped_mask = 0;
	uint8_t hdr[ESPTRACE_HDR_SIZE];

	int res = esptrace_build_hdr(target, snap, &state->raw, hdr);
	if (res != ERROR_OK)
		return res;
	res = esptrace_write(file, hdr, sizeof(hdr));
	if (res != ERROR_OK)
		return res;

	for (unsigned int i = 0; i < snap->core_count; i++) {
		if (cores[i].buffer_addr == 0 || !(core_mask & BIT(cores[i].core_id)))
			continue;
		res = esptrace_write_record_hdr(file, ESPTRACE_REC_CORE_DESC, cores[i].core_id, 0, RVTS_CORE_DESC_SIZE);
		if (res != ERROR_OK)
			return res;
		res = esptrace_write(file, state->raw.cores[i], RVTS_CORE_DESC_SIZE);
		if (res != ERROR_OK)
			return res;
		bool wrapped;
		res = esptrace_write_core_data(target, file, &cores[i], &wrapped);
		if (res != ERROR_OK)
			return res;
		if (wrapped)
			*wrapped_mask |= BIT(cores[i].core_id);
	}

	/* A capture started while reading changes write_seq and rewrites the buffers. */
	uint32_t write_seq;
	res = target_read_u32(target, state->ocd.hdr.snapshot + RVTS_OFF_WRITE_SEQ, &write_seq);
	if (res != ERROR_OK)
		return res;
	if (write_seq != snap->write_seq) {
		LOG_ERROR("RISC-V trace capture changed during dump");
		return ERROR_FAIL;
	}

	uint8_t status[sizeof(uint32_t)];
	h_u32_to_le(status, end_status);
	res = esptrace_write_record_hdr(file, ESPTRACE_REC_END, ESPTRACE_REC_NO_CORE, 0, sizeof(status));
	if (res != ERROR_OK)
		return res;
	return esptrace_write(file, status, sizeof(status));
}

/* Writes the stopped or frozen cores of core_mask to an .esptrace file. Other cores are skipped,
 * except a capturing core, which fails the write. written_mask and wrapped_mask get BIT(core_id)
 * of the written and of the wrapped cores. */
int esp_riscv_trace_file_write(struct target *target, const struct esp_riscv_trace_target_state *state,
	const char *path, uint32_t core_mask, uint32_t end_status, uint32_t *written_mask, uint32_t *wrapped_mask)
{
	uint32_t write_mask = 0;
	uint32_t bytes = 0;
	uint32_t file_bytes = ESPTRACE_HDR_SIZE + ESPTRACE_REC_HDR_SIZE + sizeof(uint32_t);
	for (unsigned int i = 0; i < state->snap.core_count; i++) {
		const struct esp_riscv_trace_snapshot_core_desc *core = &state->cores[i];
		if (core->buffer_addr == 0 || !(core_mask & BIT(core->core_id)))
			continue;
		if (core->state == RVTS_STATE_CAPTURING) {
			LOG_ERROR("RISC-V trace core %u is capturing. Stop the capture first.", core->core_id);
			return ERROR_FAIL;
		}
		if (!esp_riscv_trace_core_is_stopped(core)) {
			LOG_INFO("RISC-V trace core %u has no capture, skipped", core->core_id);
			continue;
		}
		write_mask |= BIT(core->core_id);
		bytes += core->capacity;
		file_bytes += 2 * ESPTRACE_REC_HDR_SIZE + RVTS_CORE_DESC_SIZE + core->capacity;
	}
	if (write_mask == 0) {
		LOG_ERROR("No stopped RISC-V trace capture to write");
		return ERROR_FAIL;
	}

	struct fileio *file;
	int res = fileio_open(&file, path, FILEIO_WRITE, FILEIO_BINARY);
	if (res != ERROR_OK)
		return res;
	res = esptrace_write_contents(target, file, state, write_mask, end_status, wrapped_mask);
	int close_res = fileio_close(file);
	if (res == ERROR_OK)
		res = close_res;
	if (res != ERROR_OK) {
		remove(path);
		return res;
	}

	LOG_INFO("Wrote cores 0x%" PRIx32 ", %" PRIu32 " trace bytes to %s (%" PRIu32 " B)", write_mask, bytes,
		path, file_bytes);
	*written_mask = write_mask;
	return ERROR_OK;
}
