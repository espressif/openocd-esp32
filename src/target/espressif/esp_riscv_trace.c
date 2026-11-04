// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   RISC-V trace encoder control for Espressif chips                      *
 *   Copyright (C) 2026 Espressif Systems Ltd.                             *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <helper/log.h>
#include <helper/time_support.h>
#include <target/target.h>
#include "esp.h"
#include "esp_algorithm.h"
#include "esp_riscv.h"
#include "esp_riscv_trace.h"
#include "esp_riscv_trace_desc.h"
#include "esp_riscv_trace_file.h"

#define ESP_RISCV_TRACE_STUB_STACK_SIZE 1024
#define ESP_RISCV_TRACE_STUB_RETRIES    5
#define ESP_RISCV_TRACE_STUB_RETRY_MS   10
#define ESP_RISCV_TRACE_UNKNOWN_MS      (-1)

/* PT 1.0 encoder trigger register, the same on every PT 1.0 chip (ESP32-C6, ESP32-H2, ESP32-H21) */
#define ESP_RISCV_TRACE_PT10_TRIGGER_REG         0x600C0020
#define ESP_RISCV_TRACE_PT10_TRIGGER_ON          BIT(0)
#define ESP_RISCV_TRACE_PT10_TRIGGER_OFF         BIT(1)
#define ESP_RISCV_TRACE_PT10_TRIGGER_RESTART_ENA BIT(3)

/* One OpenOCD capture at a time. */
struct esp_riscv_trace_session {
	bool active;
	bool timer_registered; /* Duration timer is still registered */
	struct target *target; /* Target that runs the firmware start/stop functions */
	char *file;
	uint32_t core_mask;    /* BIT(core_id) for every captured core */
	uint32_t stop_func;
	uint32_t duration_ms;  /* 0 = until the stop command */
	int64_t start_ms;      /* Host time when the capture started */
	/* PT 1.0 requires stopping the encoder before halting the hart. */
	bool stop_encoder_before_halt;
};

static struct esp_riscv_trace_session s_session;

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

/* Runs a firmware function on the halted target. */
static int stub_run(struct target *target, uint32_t func_addr, uint32_t core_mask, uint32_t mode, int32_t *ret_code)
{
	struct esp_riscv_common *esp_riscv = target_to_esp_riscv(target);
	const struct esp_dbg_stubs_ctl_data *ctl = &esp_riscv->esp.dbg_stubs.ctl_data;
	if (ctl->tramp_addr == 0 || ctl->min_stack_addr == 0) {
		LOG_ERROR("Debug stubs have no trampoline or stack");
		return ERROR_FAIL;
	}

	struct esp_algorithm_run_data run = { 0 };
	run.hw = esp_riscv->esp.algo_hw;
	run.stack_size = ESP_RISCV_TRACE_STUB_STACK_SIZE;
	run.on_board.min_stack_addr = ctl->min_stack_addr;
	run.on_board.min_stack_size = ESP_DBG_STUBS_STACK_MIN_SIZE;
	run.on_board.code_buf_addr = ctl->tramp_addr;
	run.on_board.code_buf_size = ESP_DBG_STUBS_CODE_BUF_SIZE;
	int res = run.hw->run_onboard_func(target, &run, func_addr, 2, core_mask, mode);
	*ret_code = run.ret_code;
	return res;
}

/*
 * Runs a firmware start/stop function for core_mask on the halted target. While it reports a halted
 * app call in progress, the target is resumed for a short time so that call can finish, then halted
 * and the function runs again. The target is halted on return.
 */
static int stub_call(struct target *target, enum target_state old_state, uint32_t func_addr, uint32_t core_mask,
	uint32_t mode, int32_t *ret_code)
{
	int res = stub_run(target, func_addr, core_mask, mode, ret_code);
	for (int attempt = 1; res == ERROR_OK && *ret_code == RVOC_ERR_NOT_FINISHED &&
		old_state == TARGET_RUNNING && attempt < ESP_RISCV_TRACE_STUB_RETRIES; attempt++) {
		res = esp_common_restore_target_state(target, TARGET_RUNNING);
		alive_sleep(ESP_RISCV_TRACE_STUB_RETRY_MS);
		enum target_state retry_state;
		if (res == ERROR_OK)
			res = esp_common_halt_target(target, &retry_state);
		if (res == ERROR_OK)
			res = stub_run(target, func_addr, core_mask, mode, ret_code);
	}
	return res;
}

/* Halts the target for a firmware call. GDB is not told, so it keeps seeing the state it had. */
static int trace_halt(struct target *target, enum target_state *old_state)
{
	esp_common_pause_gdb_event_callbacks(target, true);
	int res = esp_common_halt_target(target, old_state);
	if (res != ERROR_OK)
		esp_common_pause_gdb_event_callbacks(target, false);
	return res;
}

/* Ends the halt started by trace_halt() and resumes the target if it was running */
static int trace_resume(struct target *target, enum target_state old_state)
{
	int res = esp_common_restore_target_state(target, old_state);
	esp_common_pause_gdb_event_callbacks(target, false);
	return res;
}

/* Tells GDB about a halt it was not told about */
static void trace_report_halt(struct target *target)
{
	struct target *gdb_target = target->gdb_service ? target->gdb_service->target : target;
	target_call_event_callbacks(gdb_target, TARGET_EVENT_GDB_HALT);
}

static int session_timer_cb(void *priv);
static int session_reset_cb(struct target *target, enum target_reset_mode reset_mode, void *priv);

static void session_clear(void)
{
	if (s_session.timer_registered)
		target_unregister_timer_callback(session_timer_cb, NULL);
	free(s_session.file);
	memset(&s_session, 0, sizeof(s_session));
}

/* A reset or reflash ends the capture. The old firmware functions must not be called again.
 * Registered once and never removed, because the reset callback list cannot drop entries while it runs. */
static int session_reset_cb(struct target *target, enum target_reset_mode reset_mode, void *priv)
{
	if (s_session.active) {
		LOG_WARNING("Target reset. RISC-V trace capture to %s was dropped.", s_session.file);
		session_clear();
	}
	return ERROR_OK;
}

static void session_begin(struct target *target, char *file, const struct esp_riscv_trace_target_state *state,
	uint32_t core_mask, uint32_t duration_ms)
{
	static bool reset_cb_registered;

	s_session.active = true;
	s_session.target = target;
	s_session.file = file;
	s_session.stop_func = state->ocd.hdr.stop;
	s_session.core_mask = core_mask;
	s_session.duration_ms = duration_ms;
	s_session.start_ms = timeval_ms();
	s_session.stop_encoder_before_halt = false;
	for (unsigned int i = 0; i < state->snap.core_count; i++) {
		const struct esp_riscv_trace_snapshot_core_desc *core = &state->cores[i];
		if ((core_mask & BIT(core->core_id)) && core->packet_format == RVTS_PACKET_FORMAT_PT10)
			s_session.stop_encoder_before_halt = true;
	}
	if (!reset_cb_registered) {
		int res = target_register_reset_callback(session_reset_cb, NULL);
		reset_cb_registered = (res == ERROR_OK);
	}
}

/* Maps a stub stop result to the released and stopped core masks and logs the outcome.
 * The firmware ends every OpenOCD capture it stops, so the result applies to all of them. */
static void session_stop_result(int res, int32_t ret, uint32_t *released, uint32_t *stopped,
	bool *fifo_not_empty)
{
	*released = 0;
	*stopped = 0;
	*fifo_not_empty = false;
	if (res == ERROR_OK && (ret == RVOC_ERR_OK || ret == RVOC_ERR_TIMEOUT)) {
		*released = s_session.core_mask;
		*stopped = *released;
		*fifo_not_empty = (ret == RVOC_ERR_TIMEOUT);
	} else if (res == ERROR_OK && ret == RVOC_ERR_INVALID_STATE) {
		LOG_WARNING("RISC-V trace cores 0x%" PRIx32 " are no longer in an OpenOCD capture. The capture is lost.",
			s_session.core_mask);
		*released = s_session.core_mask;
	} else if (res == ERROR_OK) {
		LOG_ERROR("RISC-V trace stop failed (esp_err 0x%" PRIx32 ")", (uint32_t)ret);
	}
}

/* Reads which session cores a panic froze, which are stopped or frozen, and which of those left
 * data in the FIFO. */
static int session_read_frozen(struct target *target, uint32_t *frozen, uint32_t *stopped,
	uint32_t *fifo_mask)
{
	struct esp_riscv_trace_target_state state;
	int res = esp_riscv_trace_read_target_state(target, &state);
	if (res != ERROR_OK)
		return res;

	*frozen = 0;
	*stopped = 0;
	*fifo_mask = 0;
	for (unsigned int i = 0; i < state.snap.core_count; i++) {
		const struct esp_riscv_trace_snapshot_core_desc *core = &state.cores[i];
		uint32_t bit = BIT(core->core_id);
		bool core_stopped = esp_riscv_trace_core_is_stopped(core);
		if (!(s_session.core_mask & bit) || !core_stopped)
			continue;
		*stopped |= bit;
		if (core->state == RVTS_STATE_FROZEN)
			*frozen |= bit;
		if (!core->fifo_empty)
			*fifo_mask |= bit;
	}
	return ERROR_OK;
}

/* Stops a PT 1.0 encoder while the target runs, so tracing during the coming halt does not overwrite
 * the capture. PT 1.0 chips are single-core. A failure is logged and the stop function still stops the
 * encoder. */
static void session_stop_pt10_encoder(struct target *target)
{
	uint32_t trigger;
	int res = target_read_u32(target, ESP_RISCV_TRACE_PT10_TRIGGER_REG, &trigger);
	if (res != ERROR_OK) {
		LOG_WARNING("Cannot read the RISC-V trace trigger register. The halt may overwrite the capture.");
		return;
	}
	trigger &= ~(ESP_RISCV_TRACE_PT10_TRIGGER_ON | ESP_RISCV_TRACE_PT10_TRIGGER_RESTART_ENA);
	trigger |= ESP_RISCV_TRACE_PT10_TRIGGER_OFF;
	res = target_write_u32(target, ESP_RISCV_TRACE_PT10_TRIGGER_REG, trigger);
	if (res != ERROR_OK)
		LOG_WARNING("Cannot write the RISC-V trace trigger register. The halt may overwrite the capture.");
}

/* Stops the session cores and writes the file while the target stays halted, so the app cannot
 * restart tracing before the buffers are copied. If the stop function fails, the session stays and
 * the stop can be run again. After a panic the firmware must not run, so frozen cores are written
 * without the stop function. */
static int session_finish(const char *reason)
{
	if (s_session.timer_registered) {
		target_unregister_timer_callback(session_timer_cb, NULL);
		s_session.timer_registered = false;
	}

	struct target *target = s_session.target;
	if (s_session.stop_encoder_before_halt && target->state == TARGET_RUNNING)
		session_stop_pt10_encoder(target);

	enum target_state old_state;
	int res = trace_halt(target, &old_state);
	if (res != ERROR_OK)
		return res;
	bool report_halt = false;
	int64_t captured_ms = timeval_ms() - s_session.start_ms;

	uint32_t frozen = 0;
	uint32_t released = 0;
	uint32_t stopped = 0;
	uint32_t fifo_mask = 0;
	bool fifo_not_empty = false;
	res = session_read_frozen(target, &frozen, &stopped, &fifo_mask);
	if (res != ERROR_OK) {
		/* The session stays as it was. The read error is already logged. */
		LOG_INFO("Run 'esp riscv_trace stop' to retry.");
		trace_resume(target, old_state);
		return res;
	}
	if (frozen) {
		released = s_session.core_mask;
		if (released & ~stopped)
			LOG_WARNING("RISC-V trace cores 0x%" PRIx32 " did not stop at the panic. Their data is lost.",
				released & ~stopped);
		fifo_not_empty = (fifo_mask != 0);
		/* The target sat halted at the panic, so the capture time is not known. */
		captured_ms = ESP_RISCV_TRACE_UNKNOWN_MS;
		/* A panicked target stays halted. Resuming would run the faulting code again. */
		report_halt = (old_state == TARGET_RUNNING);
		old_state = TARGET_HALTED;
	} else {
		int32_t ret;
		res = stub_call(target, old_state, s_session.stop_func, s_session.core_mask, 0, &ret);
		session_stop_result(res, ret, &released, &stopped, &fifo_not_empty);
	}
	if (fifo_not_empty)
		LOG_WARNING("RISC-V trace FIFO did not empty. The capture tail may be incomplete.");

	int write_res = ERROR_OK;
	if (stopped) {
		if (frozen)
			LOG_INFO("RISC-V trace loop capture on cores 0x%" PRIx32 " was frozen by a panic (%s)",
				frozen, reason);
		else
			LOG_INFO("RISC-V trace loop capture stopped (%s), %" PRId64 ".%03" PRId64 " s captured", reason,
				captured_ms / 1000, captured_ms % 1000);
		write_res = write_trace_file(target, s_session.file, stopped,
			fifo_not_empty ? ESPTRACE_END_FIFO_NOT_EMPTY : ESPTRACE_END_OK, captured_ms);
	}

	int resume_res = trace_resume(target, old_state);
	if (report_halt)
		trace_report_halt(target);
	if (res == ERROR_OK)
		res = write_res;
	if (res == ERROR_OK)
		res = resume_res;

	if (!released) {
		LOG_ERROR("RISC-V trace cores 0x%" PRIx32 " are still capturing. Run 'esp riscv_trace stop' again.",
			s_session.core_mask);
		return ERROR_FAIL;
	}
	session_clear();
	/* Released cores with nothing written mean the capture is lost */
	if (res == ERROR_OK && !stopped)
		res = ERROR_FAIL;
	return res;
}

static int session_timer_cb(void *priv)
{
	/* One-shot timers are unregistered by the caller after this returns. */
	s_session.timer_registered = false;
	session_finish("duration elapsed");
	return ERROR_OK;
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
	if (s_session.active) {
		command_print(CMD, "Cannot dump: an OpenOCD capture to %s is active.", s_session.file);
		command_print(CMD, "Run 'esp riscv_trace stop' to write it.");
		return ERROR_FAIL;
	}

	struct target *target = get_trace_target(CMD);
	if (!target)
		return ERROR_FAIL;
	return write_trace_file(target, CMD_ARGV[0], UINT32_MAX, ESPTRACE_END_OK, ESP_RISCV_TRACE_UNKNOWN_MS);
}

COMMAND_HANDLER(esp_riscv_trace_cmd_stop)
{
	if (CMD_ARGC != 0)
		return ERROR_COMMAND_SYNTAX_ERROR;
	if (!s_session.active) {
		command_print(CMD, "Nothing to stop: no RISC-V trace capture was started");
		return ERROR_FAIL;
	}
	return session_finish("stop command");
}

COMMAND_HANDLER(esp_riscv_trace_cmd_start)
{
	if (CMD_ARGC < 2 || CMD_ARGC > 4)
		return ERROR_COMMAND_SYNTAX_ERROR;

	if (s_session.active) {
		command_print(CMD, "Cannot start: a RISC-V trace capture is already running. Stop it first.");
		return ERROR_FAIL;
	}

	uint32_t mode;
	if (!strcmp(CMD_ARGV[0], "loop")) {
		mode = RVOC_MODE_LOOP;
	} else if (!strcmp(CMD_ARGV[0], "stream")) {
		command_print(CMD, "Stream mode is not supported yet");
		return ERROR_FAIL;
	} else {
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	uint32_t duration_ms = 0;
	if (CMD_ARGC > 2)
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[2], duration_ms);
	uint32_t core_mask = UINT32_MAX;
	if (CMD_ARGC > 3)
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[3], core_mask);

	struct target *target = get_trace_target(CMD);
	if (!target)
		return ERROR_FAIL;

	struct esp_riscv_trace_target_state state;
	int res = esp_riscv_trace_read_target_state(target, &state);
	if (res != ERROR_OK)
		return res;
	if (state.ocd.hdr.start == 0) {
		command_print(CMD, "RISC-V trace start/stop is disabled in firmware (ESP_RISCV_TRACE_AUTOSTART).");
		command_print(CMD, "Use 'esp riscv_trace dump' to read the trace.");
		return ERROR_FAIL;
	}

	uint32_t traced_mask = 0;
	for (unsigned int i = 0; i < state.snap.core_count; i++)
		if (state.cores[i].buffer_addr)
			traced_mask |= BIT(state.cores[i].core_id);
	if (CMD_ARGC > 3 && (core_mask & ~traced_mask)) {
		command_print(CMD, "Core mask 0x%" PRIx32 " includes cores that are not traced (traced 0x%" PRIx32 ")",
			core_mask, traced_mask);
		return ERROR_FAIL;
	}
	core_mask &= traced_mask;
	if (core_mask == 0) {
		command_print(CMD, "No traced core to capture");
		return ERROR_FAIL;
	}

	char *file = strdup(CMD_ARGV[1]);
	if (!file) {
		LOG_ERROR("Out of memory");
		return ERROR_FAIL;
	}

	enum target_state old_state;
	res = trace_halt(target, &old_state);
	if (res != ERROR_OK) {
		free(file);
		return res;
	}

	int32_t ret;
	res = stub_call(target, old_state, state.ocd.hdr.start, core_mask, mode, &ret);
	if (res != ERROR_OK || ret != RVOC_ERR_OK) {
		/* The firmware starts all cores or none. A failed run leaves the target state unknown. */
		if (res == ERROR_OK)
			command_print(CMD, "RISC-V trace start failed (esp_err 0x%" PRIx32 ")", (uint32_t)ret);
		else
			command_print(CMD, "RISC-V trace start failed, target state unknown. Reset the target.");
		trace_resume(target, old_state);
		free(file);
		return res != ERROR_OK ? res : ERROR_FAIL;
	}
	session_begin(target, file, &state, core_mask, duration_ms);

	res = trace_resume(target, old_state);
	if (res == ERROR_OK && duration_ms) {
		res = target_register_timer_callback(session_timer_cb, duration_ms, TARGET_TIMER_TYPE_ONESHOT, NULL);
		if (res == ERROR_OK)
			s_session.timer_registered = true;
	}
	if (res != ERROR_OK) {
		session_finish("start failed");
		return res;
	}

	command_print(CMD, "RISC-V trace loop capture started on cores 0x%" PRIx32 "%s", core_mask,
		duration_ms ? "" : ". Run 'esp riscv_trace stop' to write the file.");
	return ERROR_OK;
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

	bool control = state.ocd.hdr.start != 0;
	command_print(CMD, "OpenOCD control (RVOC) ABI %u @ 0x%08" PRIx32 ", start/stop %s", state.ocd.hdr.abi_version,
		state.ocd_addr, control ? "available" : "disabled in firmware");
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

	if (s_session.active) {
		int64_t elapsed_ms = timeval_ms() - s_session.start_ms;
		command_print(CMD, "OpenOCD capture: loop, cores 0x%" PRIx32 ", file %s", s_session.core_mask,
			s_session.file);
		if (s_session.duration_ms)
			command_print(CMD, "        elapsed %" PRId64 " ms, remaining %" PRId64 " ms", elapsed_ms,
				MAX((int64_t)s_session.duration_ms - elapsed_ms, 0));
		else
			command_print(CMD, "        elapsed %" PRId64 " ms, until stop", elapsed_ms);
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
		/* A panic freezes the snapshot but leaves the RVOC state as it was. */
		command_print(CMD, "        mode %s, state %s, error %s%s",
			enum_str(s_ocd_mode_str, oc->mode),
			enum_str(s_ocd_state_str, oc->state),
			enum_str(s_ocd_error_str, oc->error),
			sc->state == RVTS_STATE_FROZEN ? ", frozen by panic" : "");
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
		.name = "start",
		.handler = esp_riscv_trace_cmd_start,
		.mode = COMMAND_EXEC,
		.help = "Start a RISC-V trace capture from OpenOCD",
		.usage = "<loop|stream> <file> [duration_ms] [core_mask]",
	},
	{
		.name = "stop",
		.handler = esp_riscv_trace_cmd_stop,
		.mode = COMMAND_EXEC,
		.help = "Stop the RISC-V trace capture and write its file",
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
