// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) 2007-2010 Øyvind Harboe                                 *
 *   oyvind.harboe@zylin.com                                               *
 *                                                                         *
 *   Copyright (C) 2008 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "log.h"
#include "command.h"
#include "replacements.h"
#include "time_support.h"
#include <server/gdb_server.h>
#include <server/server.h>

#include <stdarg.h>

#ifdef _DEBUG_FREE_SPACE_
#ifdef HAVE_MALLOC_H
#include <malloc.h>
#else
#error "malloc.h is required to use --enable-malloc-logging"
#endif

#ifdef __GLIBC__
#if __GLIBC_PREREQ(2, 33)
#define FORDBLKS_FORMAT " %zu"
#else
/* glibc older than 2.33 (2021-02-01) use mallinfo(). Overwrite it */
#define mallinfo2 mallinfo
#define FORDBLKS_FORMAT " %d"
#endif
#else
#error "GNU glibc is required to use --enable-malloc-logging"
#endif
#endif

#define DEFAULT_LOG_OUTPUT	stderr

int debug_level = LOG_LVL_INFO;

static FILE *log_output;
static struct log_callback *log_callbacks;
static bool log_non_error_levels_to_stdout;

static int64_t last_time;

static int64_t start;

static const char * const log_strings[6] = {
	"User : ",
	"Error: ",
	"Warn : ",	/* want a space after each colon, all same width, colons aligned */
	"Info : ",
	"Debug: ",
	"Debug: "
};

static int count;

/* forward the log to the listeners */
static void log_forward(const char *file, unsigned int line, const char *function, const char *string)
{
	struct log_callback *cb, *next;
	cb = log_callbacks;
	/* DANGER!!!! the log callback can remove itself!!!! */
	while (cb) {
		next = cb->next;
		cb->fn(cb->priv, file, line, function, string);
		cb = next;
	}
}

/* The log_puts() serves two somewhat different goals:
 *
 * - logging
 * - feeding low-level info to the user in GDB or Telnet
 *
 * The latter dictates that strings without newline are not logged, lest there
 * will be *MANY log lines when sending one char at the time(e.g.
 * target_request.c).
 *
 */
static void log_puts(enum log_levels level,
	const char *file,
	int line,
	const char *function,
	const char *string)
{
	char *f;

	if (!log_output) {
		/* log_init() not called yet; print on DEFAULT_LOG_OUTPUT */
		fputs(string, DEFAULT_LOG_OUTPUT);
		fflush(DEFAULT_LOG_OUTPUT);
		return;
	}

	/* global log_output value can be stderr(default), stdout or a file descriptor */
	FILE *current_log_output = log_output;
	if (log_non_error_levels_to_stdout) {
		/* default log output is stderr for all levels but now we are redirect non-error logs to the stdout */
		if (log_output == stderr && level != LOG_LVL_ERROR)
			current_log_output = stdout;
		/* If the default value set as stdout, just error logs will be redirected */
		else if (log_output == stdout && level == LOG_LVL_ERROR)
			current_log_output = stderr;
	}

	if (level == LOG_LVL_OUTPUT) {
		/* do not prepend any headers, just print out what we were given and return */
		fputs(string, current_log_output);
		fflush(current_log_output);
		return;
	}

	f = strrchr(file, '/');
	if (f)
		file = f + 1;

	if (debug_level >= LOG_LVL_DEBUG) {
		/* print with count and time information */
		int64_t t = timeval_ms() - start;
#ifdef _DEBUG_FREE_SPACE_
		struct mallinfo2 info = mallinfo2();
#endif
		fprintf(current_log_output, "%s%d %" PRId64 " %s:%d %s()"
#ifdef _DEBUG_FREE_SPACE_
			FORDBLKS_FORMAT
#endif
			": %s", log_strings[level + 1], count, t, file, line, function,
#ifdef _DEBUG_FREE_SPACE_
			info.fordblks,
#endif
			string);
	} else {
		/* if we are using gdb through pipes then we do not want any output
		 * to the pipe otherwise we get repeated strings */
		fprintf(current_log_output, "%s%s",
			(level > LOG_LVL_USER) ? log_strings[level + 1] : "", string);
	}

	fflush(current_log_output);

	/* Never forward LOG_LVL_DEBUG, too verbose and they can be found in the log if need be */
	if (level <= LOG_LVL_INFO)
		log_forward(file, line, function, string);
}

void log_printf(enum log_levels level,
	const char *file,
	unsigned int line,
	const char *function,
	const char *format,
	...)
{
	char *string;
	va_list ap;

	count++;
	if (level > debug_level)
		return;

	va_start(ap, format);

	string = alloc_vprintf(format, ap);
	if (string) {
		log_puts(level, file, line, function, string);
		free(string);
	}

	va_end(ap);
}

void log_vprintf_lf(enum log_levels level, const char *file, unsigned int line,
		const char *function, const char *format, va_list args)
{
	char *tmp;

	count++;

	if (level > debug_level)
		return;

	tmp = alloc_vprintf(format, args);

	if (!tmp)
		return;

	/*
	 * Note: alloc_vprintf() guarantees that the buffer is at least one
	 * character longer.
	 */
	strcat(tmp, "\n");
	log_puts(level, file, line, function, tmp);
	free(tmp);
}

void log_printf_lf(enum log_levels level,
	const char *file,
	unsigned int line,
	const char *function,
	const char *format,
	...)
{
	va_list ap;

	va_start(ap, format);
	log_vprintf_lf(level, file, line, function, format, ap);
	va_end(ap);
}

COMMAND_HANDLER(handle_debug_level_command)
{
	if (CMD_ARGC == 1) {
		int new_level;
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], new_level);
		if ((new_level > LOG_LVL_DEBUG_IO) || (new_level < LOG_LVL_SILENT)) {
			LOG_ERROR("level must be between %d and %d", LOG_LVL_SILENT, LOG_LVL_DEBUG_IO);
			return ERROR_COMMAND_SYNTAX_ERROR;
		}
		debug_level = new_level;
	} else if (CMD_ARGC > 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	command_print(CMD, "debug_level: %i", debug_level);

	return ERROR_OK;
}

COMMAND_HANDLER(handle_log_output_command)
{
	if (CMD_ARGC > 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	FILE *file;
	if (CMD_ARGC == 1 && strcmp(CMD_ARGV[0], "default") != 0) {
		file = fopen(CMD_ARGV[0], "w");
		if (!file) {
			command_print(CMD, "failed to open output log \"%s\"", CMD_ARGV[0]);
			return ERROR_FAIL;
		}
		command_print(CMD, "set log_output to \"%s\"", CMD_ARGV[0]);
	} else {
		file = DEFAULT_LOG_OUTPUT;
		command_print(CMD, "set log_output to default");
	}

	if (log_output != DEFAULT_LOG_OUTPUT && log_output) {
		/* Close previous log file, if it was open and wasn't DEFAULT_LOG_OUTPUT. */
		fclose(log_output);
	}
	log_output = file;
	return ERROR_OK;
}

COMMAND_HANDLER(handle_log_non_error_levels_to_stdout)
{
	if (CMD_ARGC > 0)
		COMMAND_PARSE_BOOL(CMD_ARGV[0], log_non_error_levels_to_stdout, "on", "off");

	if (log_non_error_levels_to_stdout)
		LOG_INFO("non-error logs will be re-directed to the stdout");
	else
		LOG_INFO("non-error logs will be re-directed to the %s", DEFAULT_LOG_OUTPUT == stdout ? "stdout" : "stderr");

	return ERROR_OK;
}

static const struct command_registration log_command_handlers[] = {
	{
		.name = "log_output",
		.handler = handle_log_output_command,
		.mode = COMMAND_ANY,
		.help = "redirect logging to a file (default: stderr)",
		.usage = "[file_name | 'default']",
	},
	{
		.name = "debug_level",
		.handler = handle_debug_level_command,
		.mode = COMMAND_ANY,
		.help = "Sets the verbosity level of debugging output. "
			"0 shows errors only; 1 adds warnings; "
			"2 (default) adds other info; 3 adds debugging; "
			"4 adds extra verbose debugging.",
		.usage = "number",
	},
	{
		.name = "log_non_error_levels_to_stdout",
		.handler = handle_log_non_error_levels_to_stdout,
		.mode = COMMAND_ANY,
		.help = "redirect non-error logs to the stdout and errors to the default channel",
		.usage = "[on | off]",
	},
	COMMAND_REGISTRATION_DONE
};

int log_register_commands(struct command_context *cmd_ctx)
{
	return register_commands(cmd_ctx, NULL, log_command_handlers);
}

void log_init(void)
{
	/* set defaults for daemon configuration,
	 * if not set by cmdline or cfgfile */
	char *debug_env = getenv("OPENOCD_DEBUG_LEVEL");
	if (debug_env) {
		int value;
		int retval = parse_int(debug_env, &value);
		if (retval == ERROR_OK
				&& debug_level >= LOG_LVL_SILENT
				&& debug_level <= LOG_LVL_DEBUG_IO)
			debug_level = value;
	}

	if (!log_output)
		log_output = DEFAULT_LOG_OUTPUT;

	start = last_time = timeval_ms();
}

void log_exit(void)
{
	if (log_output && log_output != DEFAULT_LOG_OUTPUT) {
		/* Close log file, if it was open and wasn't default. */
		fclose(log_output);
	}
	log_output = NULL;
}

/* add/remove log callback handler */
int log_add_callback(log_callback_fn fn, void *priv)
{
	struct log_callback *cb;

	/* prevent the same callback to be registered more than once, just for sure */
	for (cb = log_callbacks; cb; cb = cb->next) {
		if (cb->fn == fn && cb->priv == priv)
			return ERROR_COMMAND_SYNTAX_ERROR;
	}

	/* alloc memory, it is safe just to return in case of an error, no need for the caller to
	 *check this */
	cb = malloc(sizeof(struct log_callback));
	if (!cb)
		return ERROR_BUF_TOO_SMALL;

	/* add item to the beginning of the linked list */
	cb->fn = fn;
	cb->priv = priv;
	cb->next = log_callbacks;
	log_callbacks = cb;

	return ERROR_OK;
}

int log_remove_callback(log_callback_fn fn, void *priv)
{
	struct log_callback *cb, **p;

	for (p = &log_callbacks; (cb = *p); p = &(*p)->next) {
		if (cb->fn == fn && cb->priv == priv) {
			*p = cb->next;
			free(cb);
			return ERROR_OK;
		}
	}

	/* no such item */
	return ERROR_COMMAND_SYNTAX_ERROR;
}

/* return allocated string w/printf() result */
char *alloc_vprintf(const char *fmt, va_list ap)
{
	va_list ap_copy;
	int len;
	char *string;

	/* determine the length of the buffer needed */
	va_copy(ap_copy, ap);
	len = vsnprintf(NULL, 0, fmt, ap_copy);
	va_end(ap_copy);

	/* allocate and make room for terminating zero. */
	/* FIXME: The old version always allocated at least one byte extra and
	 * other code depend on that. They should be probably be fixed, but for
	 * now reserve the extra byte. */
	string = malloc(len + 2);
	if (!string)
		return NULL;

	/* do the real work */
	vsnprintf(string, len + 1, fmt, ap);

	return string;
}

char *alloc_printf(const char *format, ...)
{
	char *string;
	va_list ap;
	va_start(ap, format);
	string = alloc_vprintf(format, ap);
	va_end(ap);
	return string;
}

/* Code must return to the server loop before 1000ms has returned or invoke
 * this function.
 *
 * The GDB connection will time out if it spends >2000ms and you'll get nasty
 * error messages from GDB:
 *
 * Ignoring packet error, continuing...
 * Reply contains invalid hex digit 116
 *
 * While it is possible use "set remotetimeout" to more than the default 2000ms
 * in GDB, OpenOCD guarantees that it sends keep-alive packages on the
 * GDB protocol and it is a bug in OpenOCD not to either return to the server
 * loop or invoke keep_alive() every 1000ms.
 *
 * This function will send a keep alive packet if >500ms has passed since last time
 * it was invoked.
 *
 * Note that this function can be invoked often, so it needs to be relatively
 * fast when invoked more often than every 500ms.
 *
 */
#define KEEP_ALIVE_KICK_TIME_MS  500
#define KEEP_ALIVE_TIMEOUT_MS   1000

static void gdb_timeout_warning(int64_t delta_time)
{
	if (gdb_get_actual_connections())
		LOG_WARNING("keep_alive() was not invoked in the "
			"%d ms timelimit. GDB alive packet not "
			"sent! (%" PRId64 " ms). Workaround: increase "
			"\"set remotetimeout\" in GDB",
			KEEP_ALIVE_TIMEOUT_MS,
			delta_time);
	else
		LOG_DEBUG("keep_alive() was not invoked in the "
			"%d ms timelimit (%" PRId64 " ms). This may cause "
			"trouble with GDB connections.",
			KEEP_ALIVE_TIMEOUT_MS,
			delta_time);
}

void keep_alive(void)
{
	int64_t current_time = timeval_ms();
	int64_t delta_time = current_time - last_time;

	if (delta_time > KEEP_ALIVE_TIMEOUT_MS) {
		last_time = current_time;

		gdb_timeout_warning(delta_time);
	}

	if (delta_time > KEEP_ALIVE_KICK_TIME_MS) {
		last_time = current_time;

		/* this will keep the GDB connection alive */
		server_keep_clients_alive();

		/* DANGER!!!! do not add code to invoke e.g. target event processing,
		 * jim timer processing, etc. it can cause infinite recursion +
		 * jim event callbacks need to happen at a well defined time,
		 * not anywhere keep_alive() is invoked.
		 *
		 * These functions should be invoked at a well defined spot in server.c
		 */
	}
}

/* reset keep alive timer without sending message */
void kept_alive(void)
{
	int64_t current_time = timeval_ms();

	int64_t delta_time = current_time - last_time;

	last_time = current_time;

	if (delta_time > KEEP_ALIVE_TIMEOUT_MS)
		gdb_timeout_warning(delta_time);
}

/* if we sleep for extended periods of time, we must invoke keep_alive() intermittently */
void alive_sleep(uint64_t ms)
{
	uint64_t nap_time = 10;
	for (uint64_t i = 0; i < ms; i += nap_time) {
		uint64_t sleep_a_bit = ms - i;
		if (sleep_a_bit > nap_time)
			sleep_a_bit = nap_time;

		usleep(sleep_a_bit * 1000);
		keep_alive();
	}
}

void busy_sleep(uint64_t ms)
{
	uint64_t then = timeval_ms();
	while (timeval_ms() - then < ms) {
		/*
		 * busy wait
		 */
	}
}

/* Maximum size of socket error message retrieved from operation system */
#define MAX_SOCKET_ERR_MSG_LENGTH 256

/* Provide log message for the last socket error.
   Uses errno on *nix and WSAGetLastError() on Windows */
void log_socket_error(const char *socket_desc)
{
	int error_code;
#ifdef _WIN32
	error_code = WSAGetLastError();
	char error_message[MAX_SOCKET_ERR_MSG_LENGTH];
	error_message[0] = '\0';
	DWORD retval = FormatMessage(FORMAT_MESSAGE_FROM_SYSTEM, NULL, error_code, 0,
		error_message, MAX_SOCKET_ERR_MSG_LENGTH, NULL);
	error_message[MAX_SOCKET_ERR_MSG_LENGTH - 1] = '\0';
	const bool have_message = (retval != 0) && (error_message[0] != '\0');
	LOG_ERROR("Error on socket '%s': WSAGetLastError==%d%s%s.", socket_desc, error_code,
		(have_message ? ", message: " : ""),
		(have_message ? error_message : ""));
#else
	error_code = errno;
	LOG_ERROR("Error on socket '%s': errno==%d, message: %s.", socket_desc, error_code, strerror(error_code));
#endif
}

/**
 * Find the first non-printable character in the char buffer, return a pointer to it.
 * If no such character exists, return NULL.
 */
const char *find_nonprint_char(const char *buf, unsigned int buf_len)
{
	for (unsigned int i = 0; i < buf_len; i++) {
		if (!isprint(buf[i]))
			return buf + i;
	}
	return NULL;
}
