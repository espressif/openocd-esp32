// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   RISC-V trace encoder control for Espressif chips                      *
 *   Copyright (C) 2026 Espressif Systems Ltd.                             *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <helper/log.h>
#include <target/target.h>
#include "esp.h"
#include "esp_riscv.h"
#include "esp_riscv_trace_desc.h"

/* Reads the descriptor address from the live stubs table. 0 when the entry is absent. */
static int read_ocd_desc_addr_from_stubs(struct target *target, uint32_t *addr)
{
	uint32_t base = target_to_esp_riscv(target)->esp.dbg_stubs.base;
	uint32_t table_size;

	*addr = 0;
	int res = target_read_u32(target, base + ESP_DBG_STUB_TABLE_SIZE * sizeof(uint32_t), &table_size);
	if (res != ERROR_OK)
		return res;
	if (table_size <= ESP_DBG_STUB_ENTRY_RISCV_TRACE)
		return ERROR_OK;
	return target_read_u32(target, base + ESP_DBG_STUB_ENTRY_RISCV_TRACE * sizeof(uint32_t), addr);
}

static void parse_ocd_core(struct target *target, const uint8_t *buf, struct esp_riscv_trace_ocd_core *core)
{
	for (unsigned int i = 0; i < RVOC_BANK_COUNT; i++) {
		uint32_t off = i * sizeof(uint32_t);
		core->bank_addr[i] = target_buffer_get_u32(target, buf + RVOC_CORE_OFF_BANK_ADDR + off);
		core->bank_len[i] = target_buffer_get_u32(target, buf + RVOC_CORE_OFF_BANK_LEN + off);
		core->bank_seq[i] = target_buffer_get_u32(target, buf + RVOC_CORE_OFF_BANK_SEQ + off);
	}
	core->bank_size = target_buffer_get_u32(target, buf + RVOC_CORE_OFF_BANK_SIZE);
	core->seq = target_buffer_get_u32(target, buf + RVOC_CORE_OFF_SEQ);
	core->ack_seq = target_buffer_get_u32(target, buf + RVOC_CORE_OFF_ACK_SEQ);
	core->mode = buf[RVOC_CORE_OFF_MODE];
	core->state = buf[RVOC_CORE_OFF_STATE];
	core->error = buf[RVOC_CORE_OFF_ERROR];
	core->active_bank = buf[RVOC_CORE_OFF_ACTIVE_BANK];
}

static void parse_ocd_hdr(struct target *target, const uint8_t *buf, struct esp_riscv_trace_ocd_hdr *hdr)
{
	hdr->magic = target_buffer_get_u32(target, buf + RVOC_HDR_OFF_MAGIC);
	hdr->abi_version = buf[RVOC_HDR_OFF_ABI_VERSION];
	hdr->core_count = buf[RVOC_HDR_OFF_CORE_COUNT];
	hdr->snapshot = target_buffer_get_u32(target, buf + RVOC_HDR_OFF_SNAPSHOT);
	hdr->start = target_buffer_get_u32(target, buf + RVOC_HDR_OFF_START);
	hdr->stop = target_buffer_get_u32(target, buf + RVOC_HDR_OFF_STOP);
}

static int read_ocd_desc(struct target *target, uint32_t *addr, struct esp_riscv_trace_ocd *ocd)
{
	uint8_t buf[ESP_RISCV_TRACE_MAX_CORES * RVOC_CORE_SIZE];

	memset(ocd, 0, sizeof(*ocd));
	int res = read_ocd_desc_addr_from_stubs(target, addr);
	if (res != ERROR_OK)
		return res;
	if (*addr == 0) {
		LOG_ERROR("Firmware has no RISC-V trace descriptor (CONFIG_ESP_RISCV_TRACE_OPENOCD)");
		return ERROR_FAIL;
	}

	res = target_read_buffer(target, *addr, RVOC_HDR_SIZE, buf);
	if (res != ERROR_OK)
		return res;
	parse_ocd_hdr(target, buf, &ocd->hdr);

	const struct esp_riscv_trace_ocd_hdr *hdr = &ocd->hdr;
	if (hdr->magic != RVOC_MAGIC || hdr->abi_version != RVOC_ABI_VERSION ||
		hdr->core_count == 0 || hdr->core_count > ESP_RISCV_TRACE_MAX_CORES) {
		LOG_ERROR("Unsupported RISC-V trace descriptor @ 0x%" PRIx32
			" (magic 0x%08" PRIx32 ", ABI %u, cores %u)", *addr, hdr->magic, hdr->abi_version,
			hdr->core_count);
		return ERROR_FAIL;
	}
	if (hdr->snapshot == 0) {
		LOG_ERROR("RISC-V trace descriptor @ 0x%" PRIx32 " has no snapshot address", *addr);
		return ERROR_FAIL;
	}

	res = target_read_buffer(target, *addr + RVOC_HDR_SIZE, hdr->core_count * RVOC_CORE_SIZE, buf);
	if (res != ERROR_OK)
		return res;
	for (unsigned int i = 0; i < hdr->core_count; i++)
		parse_ocd_core(target, buf + i * RVOC_CORE_SIZE, &ocd->cores[i]);

	return ERROR_OK;
}

static void parse_snapshot_core_desc(struct target *target, const uint8_t *buf,
	struct esp_riscv_trace_snapshot_core_desc *core)
{
	core->buffer_addr = target_buffer_get_u32(target, buf + RVTS_CORE_OFF_BUFFER_ADDR);
	core->capacity = target_buffer_get_u32(target, buf + RVTS_CORE_OFF_CAPACITY);
	core->head_offset = target_buffer_get_u32(target, buf + RVTS_CORE_OFF_HEAD_OFFSET);
	core->resync_threshold = target_buffer_get_u32(target, buf + RVTS_CORE_OFF_RESYNC_THRESHOLD);
	core->fifo_status_raw = target_buffer_get_u32(target, buf + RVTS_CORE_OFF_FIFO_STATUS_RAW);
	core->intr_status_raw = target_buffer_get_u32(target, buf + RVTS_CORE_OFF_INTR_STATUS_RAW);
	core->core_id = buf[RVTS_CORE_OFF_CORE_ID];
	core->state = buf[RVTS_CORE_OFF_STATE];
	core->memory_mode = buf[RVTS_CORE_OFF_MEMORY_MODE];
	core->packet_format = buf[RVTS_CORE_OFF_PACKET_FORMAT];
	core->address_mode = buf[RVTS_CORE_OFF_ADDRESS_MODE];
	core->resync_mode = buf[RVTS_CORE_OFF_RESYNC_MODE];
	core->reserved0 = buf[RVTS_CORE_OFF_RESERVED0];
	core->head_valid = buf[RVTS_CORE_OFF_HEAD_VALID];
	core->fifo_empty = buf[RVTS_CORE_OFF_FIFO_EMPTY];
	core->memory_full = buf[RVTS_CORE_OFF_MEMORY_FULL];
	core->fifo_overflow = buf[RVTS_CORE_OFF_FIFO_OVERFLOW];
	core->reserved1 = buf[RVTS_CORE_OFF_RESERVED1];
}

static void parse_snapshot_desc(struct target *target, const uint8_t *buf, struct esp_riscv_trace_snapshot_desc *snap)
{
	snap->magic = target_buffer_get_u32(target, buf + RVTS_OFF_MAGIC);
	snap->write_seq = target_buffer_get_u32(target, buf + RVTS_OFF_WRITE_SEQ);
	snap->cores_addr = target_buffer_get_u32(target, buf + RVTS_OFF_CORES_ADDR);
	snap->app_elf_sha256_addr = target_buffer_get_u32(target, buf + RVTS_OFF_APP_ELF_SHA256_ADDR);
	snap->encoder_params_addr = target_buffer_get_u32(target, buf + RVTS_OFF_ENCODER_PARAMS_ADDR);
	snap->target_id = target_buffer_get_u16(target, buf + RVTS_OFF_TARGET_ID);
	snap->chip_revision = target_buffer_get_u16(target, buf + RVTS_OFF_CHIP_REVISION);
	snap->abi_major = buf[RVTS_OFF_ABI_MAJOR];
	snap->abi_minor = buf[RVTS_OFF_ABI_MINOR];
	snap->snapshot_desc_size = buf[RVTS_OFF_SNAPSHOT_DESC_SIZE];
	snap->core_desc_size = buf[RVTS_OFF_CORE_DESC_SIZE];
	snap->core_count = buf[RVTS_OFF_CORE_COUNT];
	snap->capture_reason = buf[RVTS_OFF_CAPTURE_REASON];
	snap->app_elf_sha256_size = buf[RVTS_OFF_APP_ELF_SHA256_SIZE];
	snap->encoder_params_size = buf[RVTS_OFF_ENCODER_PARAMS_SIZE];
}

/* Reads a consistent snapshot. Retries while write_seq is odd or changes during the read. */
static int read_snapshot(struct target *target, uint32_t addr, struct esp_riscv_trace_snapshot_desc *snap,
	struct esp_riscv_trace_snapshot_core_desc *cores, struct esp_riscv_trace_snapshot_raw *raw)
{
	uint8_t buf[ESP_RISCV_TRACE_MAX_CORES * RVTS_CORE_DESC_SIZE];

	for (int attempt = 0; attempt < RVTS_READ_RETRIES; attempt++) {
		int res = target_read_buffer(target, addr, RVTS_DESC_SIZE, buf);
		if (res != ERROR_OK)
			return res;
		parse_snapshot_desc(target, buf, snap);

		if (snap->magic != RVTS_MAGIC || snap->abi_major != RVTS_ABI_MAJOR ||
			snap->snapshot_desc_size != RVTS_DESC_SIZE || snap->core_desc_size != RVTS_CORE_DESC_SIZE ||
			snap->core_count == 0 || snap->core_count > ESP_RISCV_TRACE_MAX_CORES) {
			LOG_ERROR("Unsupported RISC-V trace snapshot @ 0x%" PRIx32 " (magic 0x%08" PRIx32 ")",
				addr, snap->magic);
			return ERROR_FAIL;
		}
		if (snap->write_seq & RVTS_WRITE_SEQ_BUSY)
			continue;
		if (snap->cores_addr == 0) {
			LOG_ERROR("RISC-V trace snapshot @ 0x%" PRIx32 " has no core array address", addr);
			return ERROR_FAIL;
		}

		res = target_read_buffer(target, snap->cores_addr, snap->core_count * RVTS_CORE_DESC_SIZE, buf);
		if (res != ERROR_OK)
			return res;

		uint32_t seq_after;
		res = target_read_u32(target, addr + RVTS_OFF_WRITE_SEQ, &seq_after);
		if (res != ERROR_OK)
			return res;
		if (seq_after != snap->write_seq)
			continue;

		for (unsigned int i = 0; i < snap->core_count; i++) {
			parse_snapshot_core_desc(target, buf + i * RVTS_CORE_DESC_SIZE, &cores[i]);
			if (cores[i].core_id >= ESP_RISCV_TRACE_MAX_CORES) {
				LOG_ERROR("RISC-V trace snapshot core %u has invalid core_id %u", i, cores[i].core_id);
				return ERROR_FAIL;
			}
		}
		res = target_read_buffer(target, addr, RVTS_DESC_SIZE, raw->desc);
		if (res != ERROR_OK)
			return res;
		for (unsigned int i = 0; i < snap->core_count; i++)
			memcpy(raw->cores[i], buf + i * RVTS_CORE_DESC_SIZE, RVTS_CORE_DESC_SIZE);
		return ERROR_OK;
	}

	LOG_ERROR("RISC-V trace snapshot kept changing while reading");
	return ERROR_FAIL;
}

/* Reads the OpenOCD descriptor and the snapshot it points at. */
int esp_riscv_trace_read_target_state(struct target *target, struct esp_riscv_trace_target_state *state)
{
	int res = read_ocd_desc(target, &state->ocd_addr, &state->ocd);
	if (res != ERROR_OK)
		return res;
	return read_snapshot(target, state->ocd.hdr.snapshot, &state->snap, state->cores, &state->raw);
}

/* Reads the app ELF SHA-256. Fails when the firmware does not publish it. */
int esp_riscv_trace_read_app_sha256(struct target *target, const struct esp_riscv_trace_snapshot_desc *snap,
	uint8_t *sha)
{
	if (!snap->app_elf_sha256_addr || snap->app_elf_sha256_size != RVTS_SHA256_SIZE)
		return ERROR_FAIL;
	return target_read_buffer(target, snap->app_elf_sha256_addr, RVTS_SHA256_SIZE, sha);
}

bool esp_riscv_trace_core_is_stopped(const struct esp_riscv_trace_snapshot_core_desc *core)
{
	return core->state == RVTS_STATE_STOPPED || core->state == RVTS_STATE_FROZEN;
}
