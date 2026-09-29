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
#include "esp_riscv_trace.h"
#include "esp_riscv_trace_desc.h"
#include "esp_riscv_trace_file.h"

#define ESP_RISCV_TRACE_UNKNOWN_MS      (-1)

static const char *const s_ocd_mode_str[] = { "none", "stream", "loop", NULL };
static const char *const s_ocd_state_str[] = { "idle", "streaming", "looping", "stopped", "overrun", "error", NULL };
static const char *const s_ocd_error_str[] = {
	"none", "overrun", "fifo-overflow", "head-invalid", "fifo-timeout", NULL
};
static const char *const s_snap_state_str[] = { "unavailable", "ready", "capturing", "stopped", "frozen", NULL };
static const char *const s_snap_mem_mode_str[] = { "unknown", "linear", "loop", NULL };
static const char *const s_snap_reason_str[] = { "unknown", "explicit-stop", "panic", NULL };

/* Returns the name of an enum value from a NULL-terminated table, or "?" when it is out of range. */
static const char *enum_str(const char *const *names, unsigned int value)
{
	for (unsigned int i = 0; names[i]; i++)
		if (i == value)
			return names[i];
	return "?";
}

/* Describes the resync setting, e.g. "resync every 64 packets". */
static void format_resync(char *buf, size_t size, const struct esp_riscv_trace_snapshot_core_desc *core)
{
	switch (core->resync_mode) {
	case RVTS_RESYNC_PACKET:
		snprintf(buf, size, "resync every %" PRIu32 " packets", core->resync_threshold);
		break;
	case RVTS_RESYNC_CYCLE:
		snprintf(buf, size, "resync every %" PRIu32 " cycles", core->resync_threshold);
		break;
	case RVTS_RESYNC_DISABLED:
		snprintf(buf, size, "no periodic resync");
		break;
	default:
		snprintf(buf, size, "resync unknown");
		break;
	}
}

/* Formats bytes per second as "N B/s" or "N.N KiB/s". */
static void format_rate(char *buf, size_t size, uint64_t bytes, int64_t ms)
{
	uint64_t bps = bytes * 1000 / (uint64_t)ms;
	if (bps < 1024)
		snprintf(buf, size, "%" PRIu64 " B/s", bps);
	else
		snprintf(buf, size, "%" PRIu64 ".%" PRIu64 " KiB/s", bps / 1024, bps % 1024 * 10 / 1024);
}

/* Logs one core of a written trace file. The rate is shown only when captured_ms is known. */
static void log_core_stats(const struct esp_riscv_trace_snapshot_core_desc *core, bool wrapped, int64_t captured_ms)
{
	const char *lost = core->fifo_overflow ? ", FIFO overflow: packets were lost" : "";
	char resync[40];
	char rate[32] = "";

	format_resync(resync, sizeof(resync), core);
	if (!core->head_valid) {
		LOG_INFO("  core %u: head unknown, %" PRIu32 " bytes written as is, %s%s",
			core->core_id, core->capacity, resync, lost);
	} else if (wrapped) {
		LOG_INFO("  core %u: head %" PRIu32 "/%" PRIu32 ", wrapped, %s%s",
			core->core_id, core->head_offset, core->capacity, resync, lost);
	} else {
		if (captured_ms > 0)
			format_rate(rate, sizeof(rate), core->head_offset, captured_ms);
		LOG_INFO("  core %u: head %" PRIu32 "/%" PRIu32 ", %" PRIu32 "%% full%s%s, %s%s",
			core->core_id, core->head_offset, core->capacity,
			(uint32_t)((uint64_t)core->head_offset * 100 / core->capacity),
			captured_ms > 0 ? ", " : "", rate, resync, lost);
	}
}

/* Writes the stopped cores of core_mask to an .esptrace file and logs per-core stats. captured_ms is
 * the capture time, or ESP_RISCV_TRACE_UNKNOWN_MS. */
static int write_trace_file(struct target *target, const char *path, uint32_t core_mask, uint32_t end_status,
	int64_t captured_ms)
{
	struct esp_riscv_trace_target_state state;
	int res = esp_riscv_trace_read_target_state(target, &state);
	if (res != ERROR_OK)
		return res;

	uint32_t written_mask;
	uint32_t wrapped_mask;
	res = esp_riscv_trace_file_write(target, &state, path, core_mask, end_status, &written_mask, &wrapped_mask);
	if (res != ERROR_OK)
		return res;

	for (unsigned int i = 0; i < state.snap.core_count; i++) {
		const struct esp_riscv_trace_snapshot_core_desc *core = &state.cores[i];
		if (written_mask & BIT(core->core_id))
			log_core_stats(core, wrapped_mask & BIT(core->core_id), captured_ms);
	}
	return ERROR_OK;
}

static const char *packet_format_str(uint8_t format)
{
	switch (format) {
	case RVTS_PACKET_FORMAT_PT10: return "PT10";
	case RVTS_PACKET_FORMAT_ET20: return "ET20";
	default: return "unknown";
	}
}

static struct target *get_trace_target(struct command_invocation *cmd)
{
	struct target *target = esp_common_dbg_stubs_target(get_current_target(cmd->ctx));
	if (!target)
		command_print(cmd, "Debug stubs table not found. Reset the target with OpenOCD connected.");
	return target;
}

COMMAND_HANDLER(esp_riscv_trace_cmd_dump)
{
	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;
	struct target *target = get_trace_target(CMD);
	if (!target)
		return ERROR_FAIL;
	return write_trace_file(target, CMD_ARGV[0], UINT32_MAX, ESPTRACE_END_OK, ESP_RISCV_TRACE_UNKNOWN_MS);
}

COMMAND_HANDLER(esp_riscv_trace_cmd_info)
{
	if (CMD_ARGC != 0)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct target *target = get_trace_target(CMD);
	if (!target)
		return ERROR_FAIL;

	struct esp_riscv_trace_target_state state;
	int res = esp_riscv_trace_read_target_state(target, &state);
	if (res != ERROR_OK)
		return res;

	command_print(CMD, "OpenOCD control (RVOC) ABI %u @ 0x%08" PRIx32, state.ocd.hdr.abi_version, state.ocd_addr);
	command_print(CMD, "trace snapshot  (RVTS) ABI %u.%u @ 0x%08" PRIx32, state.snap.abi_major, state.snap.abi_minor,
		state.ocd.hdr.snapshot);
	/* The capture reason is recorded only at stop or panic freeze. */
	bool recorded = false;
	for (unsigned int i = 0; i < state.snap.core_count; i++)
		if (esp_riscv_trace_core_is_stopped(&state.cores[i]))
			recorded = true;
	if (recorded)
		command_print(CMD, "chip id %u rev %u, capture reason %s", state.snap.target_id, state.snap.chip_revision,
			enum_str(s_snap_reason_str, state.snap.capture_reason));
	else
		command_print(CMD, "chip id %u rev %u", state.snap.target_id, state.snap.chip_revision);

	uint8_t sha[RVTS_SHA256_SIZE];
	res = esp_riscv_trace_read_app_sha256(target, &state.snap, sha);
	if (res == ERROR_OK) {
		char sha_str[2 * RVTS_SHA256_SIZE + 1];
		for (unsigned int i = 0; i < sizeof(sha); i++)
			sprintf(&sha_str[2 * i], "%02x", sha[i]);
		command_print(CMD, "app ELF SHA-256 %s", sha_str);
	} else {
		command_print(CMD, "app ELF SHA-256 not available");
	}

	for (unsigned int i = 0; i < state.snap.core_count; i++) {
		const struct esp_riscv_trace_snapshot_core_desc *sc = &state.cores[i];
		if (sc->buffer_addr == 0) {
			command_print(CMD, "core %u: not traced", sc->core_id);
			continue;
		}
		char resync[40];
		format_resync(resync, sizeof(resync), sc);
		command_print(CMD, "core %u: buffer 0x%08" PRIx32 " size %" PRIu32 ", %s, %s, %s, %s",
			sc->core_id, sc->buffer_addr, sc->capacity,
			enum_str(s_snap_state_str, sc->state),
			enum_str(s_snap_mem_mode_str, sc->memory_mode),
			packet_format_str(sc->packet_format), resync);
		if (esp_riscv_trace_core_is_stopped(sc))
			command_print(CMD, "        head %" PRIu32 "%s, fifo %s%s%s",
				sc->head_offset, sc->head_valid ? "" : " (invalid)",
				sc->fifo_empty ? "empty" : "not-empty",
				sc->memory_full ? ", memory full" : "",
				sc->fifo_overflow ? ", overflowed" : "");

		if (sc->core_id >= state.ocd.hdr.core_count)
			continue;
		const struct esp_riscv_trace_ocd_core *oc = &state.ocd.cores[sc->core_id];
		command_print(CMD, "        mode %s, state %s, error %s",
			enum_str(s_ocd_mode_str, oc->mode),
			enum_str(s_ocd_state_str, oc->state),
			enum_str(s_ocd_error_str, oc->error));
		if (oc->bank_size)
			command_print(CMD, "        banks 0x%08" PRIx32 "/0x%08" PRIx32 " size %" PRIu32
				", active %u, seq %" PRIu32 ", ack %" PRIu32,
				oc->bank_addr[0], oc->bank_addr[1], oc->bank_size,
				oc->active_bank, oc->seq, oc->ack_seq);
	}

	return ERROR_OK;
}

static const struct command_registration esp_riscv_trace_subcommand_handlers[] = {
	{
		.name = "info",
		.handler = esp_riscv_trace_cmd_info,
		.mode = COMMAND_EXEC,
		.help = "Show RISC-V trace encoder state",
		.usage = "",
	},
	{
		.name = "dump",
		.handler = esp_riscv_trace_cmd_dump,
		.mode = COMMAND_EXEC,
		.help = "Write the stopped RISC-V trace buffers to an .esptrace file",
		.usage = "<file>",
	},
	COMMAND_REGISTRATION_DONE
};

const struct command_registration esp_riscv_trace_command_handlers[] = {
	{
		.name = "riscv_trace",
		.mode = COMMAND_ANY,
		.help = "RISC-V trace encoder commands",
		.usage = "",
		.chain = esp_riscv_trace_subcommand_handlers,
	},
	COMMAND_REGISTRATION_DONE
};
