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
 *                                                                         *
 *   Copyright (C) 2011 by Broadcom Corporation                            *
 *   Evan Hunter - ehunter@broadcom.com                                    *
 *                                                                         *
 *   Copyright (C) ST-Ericsson SA 2011                                     *
 *   michel.jaouen@stericsson.com : smp minimum support                    *
 *                                                                         *
 *   Copyright (C) 2013 Andes Technology                                   *
 *   Hsiangkai Wang <hkwang@andestech.com>                                 *
 *                                                                         *
 *   Copyright (C) 2013 Franck Jullien                                     *
 *   elec4fun@gmail.com                                                    *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <target/breakpoints.h>
#include <target/target_request.h>
#include <target/register.h>
#include <target/target.h>
#include <target/target_type.h>
#include <target/semihosting_common.h>
#include "server.h"
#include <flash/nor/core.h>
#include "gdb_server.h"
#include <target/image.h>
#include <jtag/jtag.h>
#include "rtos/rtos.h"
#include "target/smp.h"

/**
 * @file
 * GDB server implementation.
 *
 * This implements the GDB Remote Serial Protocol, over TCP connections,
 * giving GDB access to the JTAG or other hardware debugging facilities
 * found in most modern embedded processors.
 */

#define CTRL(c) ((c) - '@')

enum gdb_output_flag {
	/* GDB doesn't accept 'O' packets */
	GDB_OUTPUT_NO,
	/* GDB doesn't accept 'O' packets but accepts notifications */
	GDB_OUTPUT_NOTIF,
	/* GDB accepts 'O' packets */
	GDB_OUTPUT_ALL,
};

struct target_desc_format {
	char *tdesc;
	uint32_t tdesc_length;
};

/* private connection data for GDB */
struct gdb_connection {
	char buffer[GDB_BUFFER_SIZE + 1]; /* Extra byte for null-termination */
	char *buf_p;
	int buf_cnt;
	bool ctrl_c;
	enum target_state frontend_state;
	struct image *vflash_image;
	bool closed;
	/* set to prevent re-entrance from log messages during gdb_get_packet()
	 * and gdb_put_packet(). */
	bool busy;
	int noack_mode;
	/* set flag to true if you want the next stepi to return immediately.
	 * allowing GDB to pick up a fresh set of register values from the target
	 * without modifying the target state. */
	bool sync;
	/* We delay reporting memory write errors until next step/continue or memory
	 * write. This improves performance of gdb load significantly as the GDB packet
	 * can be replied immediately and a new GDB packet will be ready without delay
	 * (ca. 10% or so...). */
	bool mem_write_error;
	/* with extended-remote it seems we need to better emulate attach/detach.
	 * what this means is we reply with a W stop reply after a kill packet,
	 * normally we reply with a S reply via gdb_last_signal_packet.
	 * as a side note this behaviour only effects gdb > 6.8 */
	bool attached;
	/* set when extended protocol is used */
	bool extended_protocol;
	/* temporarily used for target description support */
	struct target_desc_format target_desc;
	/* temporarily used for thread list support */
	char *thread_list;
	/* flag to mask the output from gdb_log_callback() */
	enum gdb_output_flag output_flag;
	/* Unique index for this GDB connection. */
	unsigned int unique_index;
};

#if 0
#define _DEBUG_GDB_IO_
#endif

static struct gdb_connection *current_gdb_connection;

static int gdb_breakpoint_override;
static enum breakpoint_type gdb_breakpoint_override_type;

static int gdb_error(struct connection *connection, int retval);
static char *gdb_port;
static char *gdb_port_next;

static void gdb_log_callback(void *priv, const char *file, unsigned int line,
		const char *function, const char *string);

static void gdb_sig_halted(struct connection *connection);

/* number of gdb connections, mainly to suppress gdb related debugging spam
 * in helper/log.c when no gdb connections are actually active */
static int gdb_actual_connections;

/* set if we are sending a memory map to gdb
 * via qXfer:memory-map:read packet */
/* enabled by default*/
static bool gdb_use_memory_map = true;
/* enabled by default*/
static bool gdb_flash_program = true;

/* if set, data aborts cause an error to be reported in memory read packets
 * see the code in gdb_read_memory_packet() for further explanations.
 * Disabled by default.
 */
static int gdb_report_data_abort;
/* If set, errors when accessing registers are reported to gdb. Disabled by
 * default. */
static int gdb_report_register_access_error;

/* set if we are sending target descriptions to gdb
 * via qXfer:features:read packet */
/* enabled by default */
static bool gdb_use_target_description = true;

/* current processing free-run type, used by file-I/O */
static char gdb_running_type;

/* Find an available target in the SMP group that gdb is connected to. For
 * commands that affect an entire SMP group (like memory access and run control)
 * this will give better results than returning the unavailable target and having
 * the command fail.  If gdb was aware that targets can be unavailable we
 * wouldn't need this logic.
 */
struct target *get_available_target_from_connection(struct connection *connection)
{
	struct gdb_service *gdb_service = connection->service->priv;
	struct target *target = gdb_service->target;
	if (target->state == TARGET_UNAVAILABLE && target->smp) {
		struct target_list *tlist;
		foreach_smp_target(tlist, target->smp_targets) {
			struct target *t = tlist->target;
			if (t->state != TARGET_UNAVAILABLE)
				return t;
		}
		/* If we can't find an available target, just return the
		 * original. */
	}
	return target;
}

static int gdb_last_signal(struct target *target)
{
	LOG_TARGET_DEBUG(target, "Debug reason is: %s",
			target_debug_reason_str(target->debug_reason));

	switch (target->debug_reason) {
		case DBG_REASON_DBGRQ:
			return 0x2;		/* SIGINT */
		case DBG_REASON_BREAKPOINT:
		case DBG_REASON_WATCHPOINT:
		case DBG_REASON_WPTANDBKPT:
			return 0x05;	/* SIGTRAP */
		case DBG_REASON_SINGLESTEP:
			return 0x05;	/* SIGTRAP */
		case DBG_REASON_EXC_CATCH:
			return 0x05;
		case DBG_REASON_NOTHALTED:
			return 0x0;		/* no signal... shouldn't happen */
		default:
			LOG_USER("undefined debug reason %d (%s) - target needs reset",
					target->debug_reason,
					target_debug_reason_str(target->debug_reason));
			return 0x0;
	}
}

static int check_pending(struct connection *connection,
		int timeout_s, int *got_data)
{
	/* a non-blocking socket will block if there is 0 bytes available on the socket,
	 * but return with as many bytes as are available immediately
	 */
	struct timeval tv;
	fd_set read_fds;
	struct gdb_connection *gdb_con = connection->priv;
	int t;
	if (!got_data)
		got_data = &t;
	*got_data = 0;

	if (gdb_con->buf_cnt > 0) {
		*got_data = 1;
		return ERROR_OK;
	}

	FD_ZERO(&read_fds);
	FD_SET(connection->fd, &read_fds);

	tv.tv_sec = timeout_s;
	tv.tv_usec = 0;
	if (socket_select(connection->fd + 1, &read_fds, NULL, NULL, &tv) == 0) {
		/* This can typically be because a "monitor" command took too long
		 * before printing any progress messages
		 */
		if (timeout_s > 0)
			return ERROR_GDB_TIMEOUT;
		else
			return ERROR_OK;
	}
	*got_data = FD_ISSET(connection->fd, &read_fds) != 0;
	return ERROR_OK;
}

static int gdb_get_char_inner(struct connection *connection, int *next_char)
{
	struct gdb_connection *gdb_con = connection->priv;
	int retval = ERROR_OK;

#ifdef _DEBUG_GDB_IO_
	char *debug_buffer;
#endif
	for (;; ) {
		if (connection->service->type != CONNECTION_TCP)
			gdb_con->buf_cnt = read(connection->fd, gdb_con->buffer, GDB_BUFFER_SIZE);
		else {
			retval = check_pending(connection, 1, NULL);
			if (retval != ERROR_OK)
				return retval;
			gdb_con->buf_cnt = read_socket(connection->fd,
					gdb_con->buffer,
					GDB_BUFFER_SIZE);
		}

		if (gdb_con->buf_cnt > 0)
			break;
		if (gdb_con->buf_cnt == 0) {
			LOG_DEBUG("GDB connection closed by the remote client");
			gdb_con->closed = true;
			return ERROR_SERVER_REMOTE_CLOSED;
		}

#ifdef _WIN32
		bool retry = (WSAGetLastError() == WSAEWOULDBLOCK);
#else
		bool retry = (errno == EAGAIN);
#endif

		if (retry) {
			// Try again after a delay
			usleep(1000);
		} else {
			// Print error and close the socket
			log_socket_error("GDB");
			gdb_con->closed = true;
			return ERROR_SERVER_REMOTE_CLOSED;
		}
	}

#ifdef _DEBUG_GDB_IO_
	debug_buffer = strndup(gdb_con->buffer, gdb_con->buf_cnt);
	LOG_DEBUG("received '%s'", debug_buffer);
	free(debug_buffer);
#endif

	gdb_con->buf_p = gdb_con->buffer;
	gdb_con->buf_cnt--;
	*next_char = *(gdb_con->buf_p++);
	if (gdb_con->buf_cnt > 0)
		connection->input_pending = true;
	else
		connection->input_pending = false;
#ifdef _DEBUG_GDB_IO_
	LOG_DEBUG("returned char '%c' (0x%2.2x)", *next_char, *next_char);
#endif

	return retval;
}

/**
 * The cool thing about this fn is that it allows buf_p and buf_cnt to be
 * held in registers in the inner loop.
 *
 * For small caches and embedded systems this is important!
 */
static inline int gdb_get_char_fast(struct connection *connection,
		int *next_char, char **buf_p, int *buf_cnt)
{
	int retval = ERROR_OK;

	if ((*buf_cnt)-- > 0) {
		*next_char = **buf_p;
		(*buf_p)++;
		if (*buf_cnt > 0)
			connection->input_pending = true;
		else
			connection->input_pending = false;

#ifdef _DEBUG_GDB_IO_
		LOG_DEBUG("returned char '%c' (0x%2.2x)", *next_char, *next_char);
#endif

		return ERROR_OK;
	}

	struct gdb_connection *gdb_con = connection->priv;
	gdb_con->buf_p = *buf_p;
	gdb_con->buf_cnt = *buf_cnt;
	retval = gdb_get_char_inner(connection, next_char);
	*buf_p = gdb_con->buf_p;
	*buf_cnt = gdb_con->buf_cnt;

	return retval;
}

static int gdb_get_char(struct connection *connection, int *next_char)
{
	struct gdb_connection *gdb_con = connection->priv;
	return gdb_get_char_fast(connection, next_char, &gdb_con->buf_p, &gdb_con->buf_cnt);
}

static int gdb_putback_char(struct connection *connection, int last_char)
{
	struct gdb_connection *gdb_con = connection->priv;

	if (gdb_con->buf_p > gdb_con->buffer) {
		*(--gdb_con->buf_p) = last_char;
		gdb_con->buf_cnt++;
	} else
		LOG_ERROR("BUG: couldn't put character back");

	return ERROR_OK;
}

/* The only way we can detect that the socket is closed is the first time
 * we write to it, we will fail. Subsequent write operations will
 * succeed. Shudder! */
static int gdb_write(struct connection *connection, const void *data, int len)
{
	struct gdb_connection *gdb_con = connection->priv;
	if (gdb_con->closed) {
		LOG_DEBUG("GDB socket marked as closed, cannot write to it.");
		return ERROR_SERVER_REMOTE_CLOSED;
	}

	if (connection_write(connection, data, len) == len)
		return ERROR_OK;

	LOG_WARNING("Error writing to GDB socket. Dropping the connection.");
	gdb_con->closed = true;
	return ERROR_SERVER_REMOTE_CLOSED;
}

static void gdb_log_incoming_packet(struct connection *connection, const char *packet)
{
	if (!LOG_LEVEL_IS(LOG_LVL_DEBUG))
		return;

	struct target *target = get_target_from_connection(connection);
	struct gdb_connection *gdb_connection = connection->priv;

	/* Avoid dumping non-printable characters to the terminal */
	const unsigned int packet_len = strlen(packet);
	const char *nonprint = find_nonprint_char(packet, packet_len);
	if (nonprint) {
		/* Does packet at least have a prefix that is printable?
		 * Look within the first 50 chars of the packet. */
		const char *colon = memchr(packet, ':', MIN(50, packet_len));
		const bool packet_has_prefix = (colon);
		const bool packet_prefix_printable = (packet_has_prefix && nonprint > colon);

		if (packet_prefix_printable) {
			const unsigned int prefix_len = colon - packet + 1;  /* + 1 to include the ':' */
			const unsigned int payload_len = packet_len - prefix_len;
			LOG_TARGET_DEBUG(target, "{%d} received packet: %.*s<binary-data-%u-bytes>",
				gdb_connection->unique_index, prefix_len, packet, payload_len);
		} else {
			LOG_TARGET_DEBUG(target, "{%d} received packet: <binary-data-%u-bytes>",
				gdb_connection->unique_index, packet_len);
		}
	} else {
		/* All chars printable, dump the packet as is */
		LOG_TARGET_DEBUG(target, "{%d} received packet: %s", gdb_connection->unique_index, packet);
	}
}

static void gdb_log_outgoing_packet(struct connection *connection, const char *packet_buf,
	unsigned int packet_len, unsigned char checksum)
{
	if (!LOG_LEVEL_IS(LOG_LVL_DEBUG))
		return;

	struct target *target = get_target_from_connection(connection);
	struct gdb_connection *gdb_connection = connection->priv;

	if (find_nonprint_char(packet_buf, packet_len))
		LOG_TARGET_DEBUG(target, "{%d} sending packet: $<binary-data-%u-bytes>#%2.2x",
			gdb_connection->unique_index, packet_len, checksum);
	else
		LOG_TARGET_DEBUG(target, "{%d} sending packet: $%.*s#%2.2x",
			gdb_connection->unique_index, packet_len, packet_buf, checksum);
}

static int gdb_put_packet_inner(struct connection *connection,
		const char *buffer, int len)
{
	int i;
	unsigned char my_checksum = 0;
	int reply;
	int retval;
	struct gdb_connection *gdb_con = connection->priv;

	for (i = 0; i < len; i++)
		my_checksum += buffer[i];

#ifdef _DEBUG_GDB_IO_
	/*
	 * At this point we should have nothing in the input queue from GDB,
	 * however sometimes '-' is sent even though we've already received
	 * an ACK (+) for everything we've sent off.
	 */
	int gotdata;
	for (;; ) {
		retval = check_pending(connection, 0, &gotdata);
		if (retval != ERROR_OK)
			return retval;
		if (!gotdata)
			break;
		retval = gdb_get_char(connection, &reply);
		if (retval != ERROR_OK)
			return retval;
		if (reply == '$') {
			/* fix a problem with some IAR tools */
			gdb_putback_char(connection, reply);
			LOG_DEBUG("Unexpected start of new packet");
			break;
		} else if (reply == CTRL('C')) {
			/* do not discard Ctrl-C */
			gdb_putback_char(connection, reply);
			break;
		}

		LOG_WARNING("Discard unexpected char %c", reply);
	}
#endif

	while (1) {
		gdb_log_outgoing_packet(connection, buffer, len, my_checksum);

		char local_buffer[1024];
		local_buffer[0] = '$';
		if ((size_t)len + 4 <= sizeof(local_buffer)) {
			/* performance gain on smaller packets by only a single call to gdb_write() */
			memcpy(local_buffer + 1, buffer, len++);
			len += snprintf(local_buffer + len, sizeof(local_buffer) - len, "#%02x", my_checksum);
			retval = gdb_write(connection, local_buffer, len);
			if (retval != ERROR_OK)
				return retval;
		} else {
			/* larger packets are transmitted directly from caller supplied buffer
			 * by several calls to gdb_write() to avoid dynamic allocation */
			snprintf(local_buffer + 1, sizeof(local_buffer) - 1, "#%02x", my_checksum);
			retval = gdb_write(connection, local_buffer, 1);
			if (retval != ERROR_OK)
				return retval;
			retval = gdb_write(connection, buffer, len);
			if (retval != ERROR_OK)
				return retval;
			retval = gdb_write(connection, local_buffer + 1, 3);
			if (retval != ERROR_OK)
				return retval;
		}

		if (gdb_con->noack_mode)
			break;

		retval = gdb_get_char(connection, &reply);
		if (retval != ERROR_OK)
			return retval;

		if (reply == '+') {
			gdb_log_incoming_packet(connection, "+");
			break;
		} else if (reply == '-') {
			/* Stop sending output packets for now */
			gdb_con->output_flag = GDB_OUTPUT_NO;
			gdb_log_incoming_packet(connection, "-");
			LOG_WARNING("negative reply, retrying");
		} else if (reply == CTRL('C')) {
			gdb_con->ctrl_c = true;
			gdb_log_incoming_packet(connection, "<Ctrl-C>");
			retval = gdb_get_char(connection, &reply);
			if (retval != ERROR_OK)
				return retval;
			if (reply == '+') {
				gdb_log_incoming_packet(connection, "+");
				break;
			} else if (reply == '-') {
				/* Stop sending output packets for now */
				gdb_con->output_flag = GDB_OUTPUT_NO;
				gdb_log_incoming_packet(connection, "-");
				LOG_WARNING("negative reply, retrying");
			} else if (reply == '$') {
				LOG_ERROR("GDB missing ack(1) - assumed good");
				gdb_putback_char(connection, reply);
				return ERROR_OK;
			} else {
				LOG_ERROR("unknown character(1) 0x%2.2x in reply, dropping connection", reply);
				gdb_con->closed = true;
				return ERROR_SERVER_REMOTE_CLOSED;
			}
		} else if (reply == '$') {
			LOG_ERROR("GDB missing ack(2) - assumed good");
			gdb_putback_char(connection, reply);
			return ERROR_OK;
		} else {
			LOG_ERROR("unknown character(2) 0x%2.2x in reply, dropping connection",
				reply);
			gdb_con->closed = true;
			return ERROR_SERVER_REMOTE_CLOSED;
		}
	}
	if (gdb_con->closed)
		return ERROR_SERVER_REMOTE_CLOSED;

	return ERROR_OK;
}

int gdb_put_packet(struct connection *connection, const char *buffer, int len)
{
	struct gdb_connection *gdb_con = connection->priv;
	gdb_con->busy = true;
	int retval = gdb_put_packet_inner(connection, buffer, len);
	gdb_con->busy = false;

	/* we sent some data, reset timer for keep alive messages */
	kept_alive();

	return retval;
}

static inline int fetch_packet(struct connection *connection,
		int *checksum_ok, int noack, int *len, char *buffer)
{
	unsigned char my_checksum = 0;
	char checksum[3];
	int character;
	int retval = ERROR_OK;

	struct gdb_connection *gdb_con = connection->priv;
	my_checksum = 0;
	int count = 0;
	count = 0;

	/* move this over into local variables to use registers and give the
	 * more freedom to optimize */
	char *buf_p = gdb_con->buf_p;
	int buf_cnt = gdb_con->buf_cnt;

	for (;; ) {
		/* The common case is that we have an entire packet with no escape chars.
		 * We need to leave at least 2 bytes in the buffer to have
		 * gdb_get_char() update various bits and bobs correctly.
		 */
		if ((buf_cnt > 2) && ((buf_cnt + count) < *len)) {
			/* The compiler will struggle a bit with constant propagation and
			 * aliasing, so we help it by showing that these values do not
			 * change inside the loop
			 */
			int i;
			char *buf = buf_p;
			int run = buf_cnt - 2;
			i = 0;
			int done = 0;
			while (i < run) {
				character = *buf++;
				i++;
				if (character == '#') {
					/* Danger! character can be '#' when esc is
					 * used so we need an explicit boolean for done here. */
					done = 1;
					break;
				}

				if (character == '}') {
					/* data transmitted in binary mode (X packet)
					 * uses 0x7d as escape character */
					my_checksum += character & 0xff;
					character = *buf++;
					i++;
					my_checksum += character & 0xff;
					buffer[count++] = (character ^ 0x20) & 0xff;
				} else {
					my_checksum += character & 0xff;
					buffer[count++] = character & 0xff;
				}
			}
			buf_p += i;
			buf_cnt -= i;
			if (done)
				break;
		}
		if (count > *len) {
			LOG_ERROR("packet buffer too small");
			retval = ERROR_GDB_BUFFER_TOO_SMALL;
			break;
		}

		retval = gdb_get_char_fast(connection, &character, &buf_p, &buf_cnt);
		if (retval != ERROR_OK)
			break;

		if (character == '#')
			break;

		if (character == '}') {
			/* data transmitted in binary mode (X packet)
			 * uses 0x7d as escape character */
			my_checksum += character & 0xff;

			retval = gdb_get_char_fast(connection, &character, &buf_p, &buf_cnt);
			if (retval != ERROR_OK)
				break;

			my_checksum += character & 0xff;
			buffer[count++] = (character ^ 0x20) & 0xff;
		} else {
			my_checksum += character & 0xff;
			buffer[count++] = character & 0xff;
		}
	}

	gdb_con->buf_p = buf_p;
	gdb_con->buf_cnt = buf_cnt;

	if (retval != ERROR_OK)
		return retval;

	*len = count;

	retval = gdb_get_char(connection, &character);
	if (retval != ERROR_OK)
		return retval;
	checksum[0] = character;
	retval = gdb_get_char(connection, &character);
	if (retval != ERROR_OK)
		return retval;
	checksum[1] = character;
	checksum[2] = 0;

	if (!noack)
		*checksum_ok = (my_checksum == strtoul(checksum, NULL, 16));

	return ERROR_OK;
}

static int gdb_get_packet_inner(struct connection *connection,
		char *buffer, int *len)
{
	int character;
	int retval;
	struct gdb_connection *gdb_con = connection->priv;

	while (1) {
		do {
			retval = gdb_get_char(connection, &character);
			if (retval != ERROR_OK)
				return retval;

#ifdef _DEBUG_GDB_IO_
			LOG_DEBUG("character: '%c'", character);
#endif

			switch (character) {
				case '$':
					break;
				case '+':
					gdb_log_incoming_packet(connection, "+");
					/* According to the GDB documentation
					 * (https://sourceware.org/gdb/onlinedocs/gdb/Packet-Acknowledgment.html):
					 * "gdb sends a final `+` acknowledgment of the stub's `OK`
					 * response, which can be safely ignored by the stub."
					 * However OpenOCD server already is in noack mode at this
					 * point and instead of ignoring this it was emitting a
					 * warning. This code makes server ignore the first ACK
					 * that will be received after going into noack mode,
					 * warning only about subsequent ACK's. */
					if (gdb_con->noack_mode > 1) {
						LOG_WARNING("acknowledgment received, but no packet pending");
					} else if (gdb_con->noack_mode) {
						LOG_DEBUG("Received first acknowledgment after entering noack mode. Ignoring it.");
						gdb_con->noack_mode = 2;
					}
					break;
				case '-':
					gdb_log_incoming_packet(connection, "-");
					LOG_WARNING("negative acknowledgment, but no packet pending");
					break;
				case CTRL('C'):
					gdb_log_incoming_packet(connection, "<Ctrl-C>");
					gdb_con->ctrl_c = true;
					*len = 0;
					return ERROR_OK;
				default:
					LOG_WARNING("ignoring character 0x%x", character);
					break;
			}
		} while (character != '$');

		int checksum_ok = 0;
		/* explicit code expansion here to get faster inlined code in -O3 by not
		 * calculating checksum */
		if (gdb_con->noack_mode) {
			retval = fetch_packet(connection, &checksum_ok, 1, len, buffer);
			if (retval != ERROR_OK)
				return retval;
		} else {
			retval = fetch_packet(connection, &checksum_ok, 0, len, buffer);
			if (retval != ERROR_OK)
				return retval;
		}

		if (gdb_con->noack_mode) {
			/* checksum is not checked in noack mode */
			break;
		}
		if (checksum_ok) {
			retval = gdb_write(connection, "+", 1);
			if (retval != ERROR_OK)
				return retval;
			break;
		}
	}
	if (gdb_con->closed)
		return ERROR_SERVER_REMOTE_CLOSED;

	return ERROR_OK;
}

static int gdb_get_packet(struct connection *connection, char *buffer, int *len)
{
	struct gdb_connection *gdb_con = connection->priv;
	gdb_con->busy = true;
	int retval = gdb_get_packet_inner(connection, buffer, len);
	gdb_con->busy = false;
	return retval;
}

static int gdb_output_con(struct connection *connection, const char *line)
{
	char *hex_buffer;
	int bin_size;

	bin_size = strlen(line);

	hex_buffer = malloc(bin_size * 2 + 2);
	if (!hex_buffer)
		return ERROR_GDB_BUFFER_TOO_SMALL;

	hex_buffer[0] = 'O';
	size_t pkt_len = hexify(hex_buffer + 1, (const uint8_t *)line, bin_size,
		bin_size * 2 + 1);
	int retval = gdb_put_packet(connection, hex_buffer, pkt_len + 1);

	free(hex_buffer);
	return retval;
}

static int gdb_output(struct command_context *context, const char *line)
{
	/* this will be dumped to the log and also sent as an O packet if possible */
	LOG_USER_N("%s", line);
	return ERROR_OK;
}

static void gdb_signal_reply(struct target *target, struct connection *connection)
{
	struct gdb_connection *gdb_connection = connection->priv;
	char sig_reply[65];
	char stop_reason[32];
	char current_thread[25];
	int sig_reply_len;
	int signal_var;

	rtos_update_threads(target);

	if (target->debug_reason == DBG_REASON_EXIT) {
		sig_reply_len = snprintf(sig_reply, sizeof(sig_reply), "W00");
	} else {
		struct target *ct;
		if (target->rtos) {
			target->rtos->current_threadid = target->rtos->current_thread;
			target->rtos->gdb_target_for_threadid(connection, target->rtos->current_threadid, &ct);
		} else {
			ct = target;
		}

		if (gdb_connection->ctrl_c) {
			LOG_TARGET_DEBUG(target, "Responding with signal 2 (SIGINT) to debugger due to Ctrl-C");
			signal_var = 0x2;
		} else
			signal_var = gdb_last_signal(ct);

		stop_reason[0] = '\0';
		if (ct->debug_reason == DBG_REASON_WATCHPOINT) {
			enum watchpoint_rw hit_wp_type;
			target_addr_t hit_wp_address;

			if (watchpoint_hit(ct, &hit_wp_type, &hit_wp_address) == ERROR_OK) {

				switch (hit_wp_type) {
					case WPT_WRITE:
						snprintf(stop_reason, sizeof(stop_reason),
								"watch:%08" TARGET_PRIxADDR ";", hit_wp_address);
						break;
					case WPT_READ:
						snprintf(stop_reason, sizeof(stop_reason),
								"rwatch:%08" TARGET_PRIxADDR ";", hit_wp_address);
						break;
					case WPT_ACCESS:
						snprintf(stop_reason, sizeof(stop_reason),
								"awatch:%08" TARGET_PRIxADDR ";", hit_wp_address);
						break;
					default:
						break;
				}
			}
		}

		current_thread[0] = '\0';
		if (target->rtos)
			snprintf(current_thread, sizeof(current_thread), "thread:%" PRIx64 ";",
					target->rtos->current_thread);

		sig_reply_len = snprintf(sig_reply, sizeof(sig_reply), "T%2.2x%s%s",
				signal_var, stop_reason, current_thread);

		gdb_connection->ctrl_c = false;
	}

	gdb_put_packet(connection, sig_reply, sig_reply_len);
	gdb_connection->frontend_state = TARGET_HALTED;
}

static void gdb_fileio_reply(struct target *target, struct connection *connection)
{
	struct gdb_connection *gdb_connection = connection->priv;
	char fileio_command[256];
	int command_len;
	bool program_exited = false;

	if (strcmp(target->fileio_info->identifier, "open") == 0)
		sprintf(fileio_command, "F%s,%" PRIx64 "/%" PRIx64 ",%" PRIx64 ",%" PRIx64, target->fileio_info->identifier,
				target->fileio_info->param_1,
				target->fileio_info->param_2 + 1,	/* len + trailing zero */
				target->fileio_info->param_3,
				target->fileio_info->param_4);
	else if (strcmp(target->fileio_info->identifier, "close") == 0)
		sprintf(fileio_command, "F%s,%" PRIx64, target->fileio_info->identifier,
				target->fileio_info->param_1);
	else if (strcmp(target->fileio_info->identifier, "read") == 0)
		sprintf(fileio_command, "F%s,%" PRIx64 ",%" PRIx64 ",%" PRIx64, target->fileio_info->identifier,
				target->fileio_info->param_1,
				target->fileio_info->param_2,
				target->fileio_info->param_3);
	else if (strcmp(target->fileio_info->identifier, "write") == 0)
		sprintf(fileio_command, "F%s,%" PRIx64 ",%" PRIx64 ",%" PRIx64, target->fileio_info->identifier,
				target->fileio_info->param_1,
				target->fileio_info->param_2,
				target->fileio_info->param_3);
	else if (strcmp(target->fileio_info->identifier, "lseek") == 0)
		sprintf(fileio_command, "F%s,%" PRIx64 ",%" PRIx64 ",%" PRIx64, target->fileio_info->identifier,
				target->fileio_info->param_1,
				target->fileio_info->param_2,
				target->fileio_info->param_3);
	else if (strcmp(target->fileio_info->identifier, "rename") == 0)
		sprintf(fileio_command, "F%s,%" PRIx64 "/%" PRIx64 ",%" PRIx64 "/%" PRIx64, target->fileio_info->identifier,
				target->fileio_info->param_1,
				target->fileio_info->param_2 + 1,	/* len + trailing zero */
				target->fileio_info->param_3,
				target->fileio_info->param_4 + 1);	/* len + trailing zero */
	else if (strcmp(target->fileio_info->identifier, "unlink") == 0)
		sprintf(fileio_command, "F%s,%" PRIx64 "/%" PRIx64, target->fileio_info->identifier,
				target->fileio_info->param_1,
				target->fileio_info->param_2 + 1);	/* len + trailing zero */
	else if (strcmp(target->fileio_info->identifier, "stat") == 0)
		sprintf(fileio_command, "F%s,%" PRIx64 "/%" PRIx64 ",%" PRIx64, target->fileio_info->identifier,
				target->fileio_info->param_1,
				target->fileio_info->param_2,
				target->fileio_info->param_3);
	else if (strcmp(target->fileio_info->identifier, "fstat") == 0)
		sprintf(fileio_command, "F%s,%" PRIx64 ",%" PRIx64, target->fileio_info->identifier,
				target->fileio_info->param_1,
				target->fileio_info->param_2);
	else if (strcmp(target->fileio_info->identifier, "gettimeofday") == 0)
		sprintf(fileio_command, "F%s,%" PRIx64 ",%" PRIx64, target->fileio_info->identifier,
				target->fileio_info->param_1,
				target->fileio_info->param_2);
	else if (strcmp(target->fileio_info->identifier, "isatty") == 0)
		sprintf(fileio_command, "F%s,%" PRIx64, target->fileio_info->identifier,
				target->fileio_info->param_1);
	else if (strcmp(target->fileio_info->identifier, "system") == 0)
		sprintf(fileio_command, "F%s,%" PRIx64 "/%" PRIx64, target->fileio_info->identifier,
				target->fileio_info->param_1,
				target->fileio_info->param_2 + 1);	/* len + trailing zero */
	else if (strcmp(target->fileio_info->identifier, "exit") == 0) {
		/* If target hits exit syscall, report to GDB the program is terminated.
		 * In addition, let target run its own exit syscall handler. */
		program_exited = true;
		sprintf(fileio_command, "W%02" PRIx64, target->fileio_info->param_1);
	} else {
		LOG_DEBUG("Unknown syscall: %s", target->fileio_info->identifier);

		/* encounter unknown syscall, continue */
		gdb_connection->frontend_state = TARGET_RUNNING;
		target_resume(target, true, 0x0, false, false);
		return;
	}

	command_len = strlen(fileio_command);
	gdb_put_packet(connection, fileio_command, command_len);

	if (program_exited) {
		/* Use target_resume() to let target run its own exit syscall handler. */
		gdb_connection->frontend_state = TARGET_RUNNING;
		target_resume(target, true, 0x0, false, false);
	} else {
		gdb_connection->frontend_state = TARGET_HALTED;
		rtos_update_threads(target);
	}
}

static void gdb_frontend_halted(struct target *target, struct connection *connection)
{
	struct gdb_connection *gdb_connection = connection->priv;

	/* In the GDB protocol when we are stepping or continuing execution,
	 * we have a lingering reply. Upon receiving a halted event
	 * when we have that lingering packet, we reply to the original
	 * step or continue packet.
	 *
	 * Executing monitor commands can bring the target in and
	 * out of the running state so we'll see lots of TARGET_EVENT_XXX
	 * that are to be ignored.
	 */
	if (gdb_connection->frontend_state == TARGET_RUNNING) {
		/* stop forwarding log packets! */
		gdb_connection->output_flag = GDB_OUTPUT_NO;

		/* check fileio first */
		if (target_get_gdb_fileio_info(target, target->fileio_info) == ERROR_OK)
			gdb_fileio_reply(target, connection);
		else
			gdb_signal_reply(target, connection);
	}
}

static int gdb_target_callback_event_handler(struct target *target,
		enum target_event event, void *priv)
{
	struct connection *connection = priv;
	struct target *gdb_target = get_available_target_from_connection(connection);

	if (gdb_target != target)
		return ERROR_OK;

	switch (event) {
		case TARGET_EVENT_GDB_HALT:
			gdb_frontend_halted(target, connection);
			break;
		case TARGET_EVENT_HALTED:
			target_call_event_callbacks(target, TARGET_EVENT_GDB_END);
			break;
		default:
			break;
	}

	return ERROR_OK;
}

static int gdb_new_connection(struct connection *connection)
{
	struct gdb_connection *gdb_connection = malloc(sizeof(struct gdb_connection));
	struct target *target;
	int retval;
	int initial_ack;
	static unsigned int next_unique_id = 1;

	target = get_target_from_connection(connection);
	connection->priv = gdb_connection;
	connection->cmd_ctx->current_target = target;

	/* initialize gdb connection information */
	gdb_connection->buf_p = gdb_connection->buffer;
	gdb_connection->buf_cnt = 0;
	gdb_connection->ctrl_c = false;
	gdb_connection->frontend_state = TARGET_HALTED;
	gdb_connection->vflash_image = NULL;
	gdb_connection->closed = false;
	gdb_connection->busy = false;
	gdb_connection->noack_mode = 0;
	gdb_connection->sync = false;
	gdb_connection->mem_write_error = false;
	gdb_connection->attached = true;
	gdb_connection->extended_protocol = false;
	gdb_connection->target_desc.tdesc = NULL;
	gdb_connection->target_desc.tdesc_length = 0;
	gdb_connection->thread_list = NULL;
	gdb_connection->output_flag = GDB_OUTPUT_NO;
	gdb_connection->unique_index = next_unique_id++;

	/* output goes through gdb connection */
	command_set_output_handler(connection->cmd_ctx, gdb_output, connection);

	/* we must remove all breakpoints registered to the target as a previous
	 * GDB session could leave dangling breakpoints if e.g. communication
	 * timed out.
	 */
	breakpoint_clear_target(target);
	watchpoint_clear_target(target);

	/* Since version 3.95 (gdb-19990504), with the exclusion of 6.5~6.8, GDB
	 * sends an ACK at connection with the following comment in its source code:
	 * "Ack any packet which the remote side has already sent."
	 * LLDB does the same since the first gdb-remote implementation.
	 * Remove the initial ACK from the incoming buffer.
	 */
	retval = gdb_get_char(connection, &initial_ack);
	if (retval != ERROR_OK)
		return retval;

	if (initial_ack != '+')
		gdb_putback_char(connection, initial_ack);

	target_call_event_callbacks(target, TARGET_EVENT_GDB_ATTACH);

	if (target->rtos) {
		/* clean previous rtos session if supported*/
		if (target->rtos->type->clean)
			target->rtos->type->clean(target);

		/* update threads */
		rtos_update_threads(target);
	}

	if (gdb_use_memory_map) {
		/* Connect must fail if the memory map can't be set up correctly.
		 *
		 * This will cause an auto_probe to be invoked, which is either
		 * a no-op or it will fail when the target isn't ready(e.g. not halted).
		 */
		for (unsigned int i = 0; i < flash_get_bank_count(); i++) {
			struct flash_bank *p;
			p = get_flash_bank_by_num_noprobe(i);
			if (p->target != target)
				continue;
			retval = get_flash_bank_by_num(i, &p);
			if (retval != ERROR_OK) {
				LOG_ERROR("Connect failed. Consider setting up a gdb-attach event for the target "
						"to prepare target for GDB connect, or use 'gdb_memory_map disable'.");
				return retval;
			}
		}
	}

	log_printf_lf(all_targets->next ? LOG_LVL_INFO : LOG_LVL_DEBUG,
			__FILE__, __LINE__, __func__,
			"New GDB Connection: %d, Target %s, state: %s",
			gdb_connection->unique_index,
			target_name(target),
			target_state_name(target));

	if (!target_was_examined(target)) {
		LOG_TARGET_ERROR(target, "Target not examined yet, refuse gdb connection %d!",
				  gdb_connection->unique_index);
		return ERROR_TARGET_NOT_EXAMINED;
	}
	gdb_actual_connections++;

	if (target->state != TARGET_HALTED)
		LOG_TARGET_WARNING(target, "GDB connection %d not halted",
					gdb_actual_connections);

	/* DANGER! If we fail subsequently, we must remove this handler,
	 * otherwise we occasionally see crashes as the timer can invoke the
	 * callback fn.
	 *
	 * register callback to be informed about target events */
	target_register_event_callback(gdb_target_callback_event_handler, connection);

	log_add_callback(gdb_log_callback, connection);

	return ERROR_OK;
}

static int gdb_connection_closed(struct connection *connection)
{
	struct target *target;
	struct gdb_connection *gdb_connection = connection->priv;

	target = get_target_from_connection(connection);

	/* we're done forwarding messages. Tear down callback before
	 * cleaning up connection.
	 */
	log_remove_callback(gdb_log_callback, connection);

	gdb_actual_connections--;
	LOG_TARGET_DEBUG(target, "{%d} GDB Close, state: %s, gdb_actual_connections=%d",
		gdb_connection->unique_index,
		target_state_name(target),
		gdb_actual_connections);

	/* see if an image built with vFlash commands is left */
	if (gdb_connection->vflash_image) {
		image_close(gdb_connection->vflash_image);
		free(gdb_connection->vflash_image);
		gdb_connection->vflash_image = NULL;
	}

	/* if this connection registered a debug-message receiver delete it */
	delete_debug_msg_receiver(connection->cmd_ctx, target);

	free(connection->priv);
	connection->priv = NULL;

	target_unregister_event_callback(gdb_target_callback_event_handler, connection);

	target_call_event_callbacks(target, TARGET_EVENT_GDB_END);

	target_call_event_callbacks(target, TARGET_EVENT_GDB_DETACH);

	return ERROR_OK;
}

static void gdb_send_error(struct connection *connection, uint8_t the_error)
{
	char err[4];
	snprintf(err, 4, "E%2.2X", the_error);
	gdb_put_packet(connection, err, 3);
}

static int gdb_last_signal_packet(struct connection *connection,
		char const *packet, int packet_size)
{
	struct target *target = get_target_from_connection(connection);
	struct gdb_connection *gdb_con = connection->priv;
	char sig_reply[4];
	int signal_var;

	if (!gdb_con->attached) {
		/* if we are here we have received a kill packet
		 * reply W stop reply otherwise gdb gets very unhappy */
		gdb_put_packet(connection, "W00", 3);
		return ERROR_OK;
	}

	signal_var = gdb_last_signal(target);

	snprintf(sig_reply, 4, "S%2.2x", signal_var);
	gdb_put_packet(connection, sig_reply, 3);

	return ERROR_OK;
}

static inline int gdb_reg_pos(struct target *target, int pos, int len)
{
	if (target->endianness == TARGET_LITTLE_ENDIAN)
		return pos;
	else
		return len - 1 - pos;
}

/* Convert register to string of bytes. NB! The # of bits in the
 * register might be non-divisible by 8(a byte), in which
 * case an entire byte is shown.
 *
 * NB! the format on the wire is the target endianness
 *
 * The format of reg->value is little endian
 *
 */
static void gdb_str_to_target(struct target *target,
		char *tstr, struct reg *reg)
{
	int i;

	uint8_t *buf;
	int buf_len;
	buf = reg->value;
	buf_len = DIV_ROUND_UP(reg->size, 8);

	for (i = 0; i < buf_len; i++) {
		int j = gdb_reg_pos(target, i, buf_len);
		tstr += sprintf(tstr, "%02x", buf[j]);
	}
}

/* copy over in register buffer */
static void gdb_target_to_reg(struct target *target,
		char const *tstr, int str_len, uint8_t *bin)
{
	if (str_len % 2) {
		LOG_ERROR("BUG: gdb value with uneven number of characters encountered");
		exit(-1);
	}

	int i;
	for (i = 0; i < str_len; i += 2) {
		unsigned int t;
		if (sscanf(tstr + i, "%02x", &t) != 1) {
			LOG_ERROR("BUG: unable to convert register value");
			exit(-1);
		}

		int j = gdb_reg_pos(target, i/2, str_len/2);
		bin[j] = t;
	}
}

/* get register value if needed and fill the buffer accordingly */
static int gdb_get_reg_value_as_str(struct target *target, char *tstr, struct reg *reg)
{
	int retval = ERROR_OK;

	if (!reg->valid)
		retval = reg->type->get(reg);

	const unsigned int len = DIV_ROUND_UP(reg->size, 8) * 2;
	switch (retval) {
		case ERROR_OK:
			gdb_str_to_target(target, tstr, reg);
			return ERROR_OK;
		case ERROR_TARGET_RESOURCE_NOT_AVAILABLE:
			memset(tstr, 'x', len);
			tstr[len] = '\0';
			return ERROR_OK;
	}
	memset(tstr, '0', len);
	tstr[len] = '\0';
	return ERROR_FAIL;
}

static int gdb_get_registers_packet(struct connection *connection,
		char const *packet, int packet_size)
{
	struct target *target = get_target_from_connection(connection);
	struct reg **reg_list;
	int reg_list_size;
	int retval;
	int reg_packet_size = 0;
	char *reg_packet;
	char *reg_packet_p;
	int i;

#ifdef _DEBUG_GDB_IO_
	LOG_DEBUG("-");
#endif

	if ((target->rtos) && (rtos_get_gdb_reg_list(connection) == ERROR_OK))
		return ERROR_OK;

	/*  `target_get_gdb_reg_list()` should read registers from target,
		but it can happen that packet is received when target is running.
		Register access error reporting is controlled by `gdb_report_register_access_error`,
		so use `target_get_gdb_reg_list_noread()` here and get cached registers values.
		If `gdb_report_register_access_error` is enabled we will fail
		below when trying to updated non-valid registers.
		For targets that do not support `target_get_gdb_reg_list_noread` (currently only RISCV support xxx_noread)
		`target_get_gdb_reg_list` will be effectively called.
		*/
	retval = target_get_gdb_reg_list_noread(target, &reg_list, &reg_list_size,
			REG_CLASS_GENERAL);
	if (retval != ERROR_OK)
		return gdb_error(connection, retval);

	for (i = 0; i < reg_list_size; i++) {
		if (!reg_list[i] || !reg_list[i]->exist || reg_list[i]->hidden)
			continue;
		reg_packet_size += DIV_ROUND_UP(reg_list[i]->size, 8) * 2;
	}

	assert(reg_packet_size > 0);

	reg_packet = malloc(reg_packet_size + 1); /* plus one for string termination null */
	if (!reg_packet)
		return ERROR_FAIL;

	reg_packet_p = reg_packet;

	for (i = 0; i < reg_list_size; i++) {
		if (!reg_list[i] || !reg_list[i]->exist || reg_list[i]->hidden)
			continue;
		retval = gdb_get_reg_value_as_str(target, reg_packet_p, reg_list[i]);
		if (retval != ERROR_OK && gdb_report_register_access_error) {
			LOG_DEBUG("Couldn't get register %s.", reg_list[i]->name);
			free(reg_packet);
			free(reg_list);
			return gdb_error(connection, retval);
		}
		reg_packet_p += DIV_ROUND_UP(reg_list[i]->size, 8) * 2;
	}

#ifdef _DEBUG_GDB_IO_
	{
		char *reg_packet_p_debug;
		reg_packet_p_debug = strndup(reg_packet, reg_packet_size);
		LOG_DEBUG("reg_packet: %s", reg_packet_p_debug);
		free(reg_packet_p_debug);
	}
#endif

	gdb_put_packet(connection, reg_packet, reg_packet_size);
	free(reg_packet);

	free(reg_list);

	return ERROR_OK;
}

static int gdb_set_registers_packet(struct connection *connection,
		char const *packet, int packet_size)
{
	struct target *target = get_target_from_connection(connection);
	int i;
	struct reg **reg_list;
	int reg_list_size;
	int retval;
	char const *packet_p;

#ifdef _DEBUG_GDB_IO_
	LOG_DEBUG("-");
#endif

	/* skip command character */
	packet++;
	packet_size--;

	if (packet_size % 2) {
		LOG_WARNING("GDB set_registers packet with uneven characters received, dropping connection");
		return ERROR_SERVER_REMOTE_CLOSED;
	}

	retval = target_get_gdb_reg_list(target, &reg_list, &reg_list_size,
			REG_CLASS_GENERAL);
	if (retval != ERROR_OK)
		return gdb_error(connection, retval);

	packet_p = packet;
	for (i = 0; i < reg_list_size; i++) {
		uint8_t *bin_buf;
		if (!reg_list[i] || !reg_list[i]->exist || reg_list[i]->hidden)
			continue;
		int chars = (DIV_ROUND_UP(reg_list[i]->size, 8) * 2);

		if (packet_p + chars > packet + packet_size)
			LOG_ERROR("BUG: register packet is too small for registers");

		bin_buf = malloc(DIV_ROUND_UP(reg_list[i]->size, 8));
		gdb_target_to_reg(target, packet_p, chars, bin_buf);

		retval = reg_list[i]->type->set(reg_list[i], bin_buf);
		if (retval != ERROR_OK && gdb_report_register_access_error) {
			LOG_DEBUG("Couldn't set register %s.", reg_list[i]->name);
			free(reg_list);
			free(bin_buf);
			return gdb_error(connection, retval);
		}

		/* advance packet pointer */
		packet_p += chars;

		free(bin_buf);
	}

	/* free struct reg *reg_list[] array allocated by get_gdb_reg_list */
	free(reg_list);

	gdb_put_packet(connection, "OK", 2);

	return ERROR_OK;
}

static int gdb_get_register_packet(struct connection *connection,
	char const *packet, int packet_size)
{
	struct target *target = get_target_from_connection(connection);
	char *reg_packet;
	int reg_num = strtoul(packet + 1, NULL, 16);
	struct reg **reg_list;
	int reg_list_size;
	int retval;

#ifdef _DEBUG_GDB_IO_
	LOG_DEBUG("-");
#endif

	if ((target->rtos) && (rtos_get_gdb_reg(connection, reg_num) == ERROR_OK))
		return ERROR_OK;

	retval = target_get_gdb_reg_list_noread(target, &reg_list, &reg_list_size,
			REG_CLASS_ALL);
	if (retval != ERROR_OK)
		return gdb_error(connection, retval);

	if ((reg_list_size <= reg_num) || !reg_list[reg_num] ||
		!reg_list[reg_num]->exist || reg_list[reg_num]->hidden) {
		LOG_ERROR("gdb requested a non-existing register (reg_num=%d)", reg_num);
		return ERROR_SERVER_REMOTE_CLOSED;
	}

	reg_packet = calloc(DIV_ROUND_UP(reg_list[reg_num]->size, 8) * 2 + 1, 1); /* plus one for string termination null */

	retval = gdb_get_reg_value_as_str(target, reg_packet, reg_list[reg_num]);
	if (retval != ERROR_OK && gdb_report_register_access_error) {
		LOG_DEBUG("Couldn't get register %s.", reg_list[reg_num]->name);
		free(reg_packet);
		free(reg_list);
		return gdb_error(connection, retval);
	}

	gdb_put_packet(connection, reg_packet, DIV_ROUND_UP(reg_list[reg_num]->size, 8) * 2);

	free(reg_list);
	free(reg_packet);

	return ERROR_OK;
}

static int gdb_set_register_packet(struct connection *connection,
	char const *packet, int packet_size)
{
	struct target *target = get_target_from_connection(connection);
	char *separator;
	int reg_num = strtoul(packet + 1, &separator, 16);
	struct reg **reg_list;
	int reg_list_size;
	int retval;

#ifdef _DEBUG_GDB_IO_
	LOG_DEBUG("-");
#endif

	if (*separator != '=') {
		LOG_ERROR("GDB 'set register packet', but no '=' following the register number");
		return ERROR_SERVER_REMOTE_CLOSED;
	}
	size_t chars = strlen(separator + 1);
	uint8_t *bin_buf = malloc(chars / 2);
	gdb_target_to_reg(target, separator + 1, chars, bin_buf);

	if ((target->rtos) &&
			(rtos_set_reg(connection, reg_num, bin_buf) == ERROR_OK)) {
		free(bin_buf);
		gdb_put_packet(connection, "OK", 2);
		return ERROR_OK;
	}

	retval = target_get_gdb_reg_list_noread(target, &reg_list, &reg_list_size,
			REG_CLASS_ALL);
	if (retval != ERROR_OK) {
		free(bin_buf);
		return gdb_error(connection, retval);
	}

	if ((reg_list_size <= reg_num) || !reg_list[reg_num] ||
		!reg_list[reg_num]->exist || reg_list[reg_num]->hidden) {
		LOG_ERROR("gdb requested a non-existing register (reg_num=%d)", reg_num);
		free(bin_buf);
		free(reg_list);
		return ERROR_SERVER_REMOTE_CLOSED;
	}

	if (chars != (DIV_ROUND_UP(reg_list[reg_num]->size, 8) * 2)) {
		LOG_ERROR("gdb sent %zu bits for a %" PRIu32 "-bit register (%s)",
				chars * 4, reg_list[reg_num]->size, reg_list[reg_num]->name);
		free(bin_buf);
		free(reg_list);
		return ERROR_SERVER_REMOTE_CLOSED;
	}

	gdb_target_to_reg(target, separator + 1, chars, bin_buf);

	retval = reg_list[reg_num]->type->set(reg_list[reg_num], bin_buf);
	if (retval != ERROR_OK && gdb_report_register_access_error) {
		LOG_DEBUG("Couldn't set register %s.", reg_list[reg_num]->name);
		free(bin_buf);
		free(reg_list);
		return gdb_error(connection, retval);
	}

	gdb_put_packet(connection, "OK", 2);

	free(bin_buf);
	free(reg_list);

	return ERROR_OK;
}

/* No attempt is made to translate the "retval" to
 * GDB speak. This has to be done at the calling
 * site as no mapping really exists.
 */
static int gdb_error(struct connection *connection, int retval)
{
	LOG_DEBUG("Reporting %i to GDB as generic error", retval);
	gdb_send_error(connection, EFAULT);
	return ERROR_OK;
}

static int gdb_read_memory_packet(struct connection *connection,
		char const *packet, int packet_size)
{
	struct target *target = get_available_target_from_connection(connection);
	char *separator;
	uint64_t addr = 0;
	uint32_t len = 0;

	uint8_t *buffer;
	char *hex_buffer;

	int retval = ERROR_OK;

	/* skip command character */
	packet++;

	addr = strtoull(packet, &separator, 16);

	if (*separator != ',') {
		LOG_ERROR("incomplete read memory packet received, dropping connection");
		return ERROR_SERVER_REMOTE_CLOSED;
	}

	len = strtoul(separator + 1, NULL, 16);

	if (!len) {
		LOG_WARNING("invalid read memory packet received (len == 0)");
		gdb_put_packet(connection, "", 0);
		return ERROR_OK;
	}

	buffer = malloc(len);

	LOG_DEBUG("addr: 0x%16.16" PRIx64 ", len: 0x%8.8" PRIx32 "", addr, len);

	retval = ERROR_NOT_IMPLEMENTED;
	if (target->rtos)
		retval = rtos_read_buffer(target, addr, len, buffer);
	if (retval == ERROR_NOT_IMPLEMENTED)
		retval = target_read_buffer(target, addr, len, buffer);

	if ((retval != ERROR_OK) && !gdb_report_data_abort) {
		/* TODO : Here we have to lie and send back all zero's lest stack traces won't work.
		 * At some point this might be fixed in GDB, in which case this code can be removed.
		 *
		 * OpenOCD developers are acutely aware of this problem, but there is nothing
		 * gained by involving the user in this problem that hopefully will get resolved
		 * eventually
		 *
		 * http://sourceware.org/cgi-bin/gnatsweb.pl? \
		 * cmd = view%20audit-trail&database = gdb&pr = 2395
		 *
		 * For now, the default is to fix up things to make current GDB versions work.
		 * This can be overwritten using the "gdb report_data_abort <'enable'|'disable'>" command.
		 */
		memset(buffer, 0, len);
		retval = ERROR_OK;
	}

	if (retval == ERROR_OK) {
		hex_buffer = malloc(len * 2 + 1);

		size_t pkt_len = hexify(hex_buffer, buffer, len, len * 2 + 1);

		gdb_put_packet(connection, hex_buffer, pkt_len);

		free(hex_buffer);
	} else
		retval = gdb_error(connection, retval);

	free(buffer);

	return retval;
}

static int gdb_write_memory_packet(struct connection *connection,
		char const *packet, int packet_size)
{
	struct target *target = get_available_target_from_connection(connection);
	char *separator;
	uint64_t addr = 0;
	uint32_t len = 0;

	uint8_t *buffer;
	int retval;

	/* skip command character */
	packet++;

	addr = strtoull(packet, &separator, 16);

	if (*separator != ',') {
		LOG_ERROR("incomplete write memory packet received, dropping connection");
		return ERROR_SERVER_REMOTE_CLOSED;
	}

	len = strtoul(separator + 1, &separator, 16);

	if (*(separator++) != ':') {
		LOG_ERROR("incomplete write memory packet received, dropping connection");
		return ERROR_SERVER_REMOTE_CLOSED;
	}

	buffer = malloc(len);

	LOG_DEBUG("addr: 0x%" PRIx64 ", len: 0x%8.8" PRIx32 "", addr, len);

	if (unhexify(buffer, separator, len) != len)
		LOG_ERROR("unable to decode memory packet");

	retval = ERROR_NOT_IMPLEMENTED;
	if (target->rtos)
		retval = rtos_write_buffer(target, addr, len, buffer);
	if (retval == ERROR_NOT_IMPLEMENTED)
		retval = target_write_buffer(target, addr, len, buffer);

	if (retval == ERROR_OK)
		gdb_put_packet(connection, "OK", 2);
	else
		retval = gdb_error(connection, retval);

	free(buffer);

	return retval;
}

static int gdb_write_memory_binary_packet(struct connection *connection,
		char const *packet, int packet_size)
{
	struct target *target = get_available_target_from_connection(connection);
	char *separator;
	uint64_t addr = 0;
	uint32_t len = 0;

	int retval = ERROR_OK;
	/* Packets larger than fast_limit bytes will be acknowledged instantly on
	 * the assumption that we're in a download and it's important to go as fast
	 * as possible. */
	uint32_t fast_limit = 8;

	/* skip command character */
	packet++;

	addr = strtoull(packet, &separator, 16);

	if (*separator != ',') {
		LOG_ERROR("incomplete write memory binary packet received, dropping connection");
		return ERROR_SERVER_REMOTE_CLOSED;
	}

	len = strtoul(separator + 1, &separator, 16);

	if (*(separator++) != ':') {
		LOG_ERROR("incomplete write memory binary packet received, dropping connection");
		return ERROR_SERVER_REMOTE_CLOSED;
	}

	struct gdb_connection *gdb_connection = connection->priv;

	if (gdb_connection->mem_write_error)
		retval = ERROR_FAIL;

	if (retval == ERROR_OK) {
		if (len >= fast_limit) {
			/* By replying the packet *immediately* GDB will send us a new packet
			 * while we write the last one to the target.
			 * We only do this for larger writes, so that users who do something like:
			 * p *((int*)0xdeadbeef)=8675309
			 * will get immediate feedback that that write failed.
			 */
			gdb_put_packet(connection, "OK", 2);
		}
	} else {
		retval = gdb_error(connection, retval);
		/* now that we have reported the memory write error, we can clear the condition */
		gdb_connection->mem_write_error = false;
		if (retval != ERROR_OK)
			return retval;
	}

	if (len) {
		LOG_DEBUG("addr: 0x%" PRIx64 ", len: 0x%8.8" PRIx32 "", addr, len);

		retval = ERROR_NOT_IMPLEMENTED;
		if (target->rtos)
			retval = rtos_write_buffer(target, addr, len, (uint8_t *)separator);
		if (retval == ERROR_NOT_IMPLEMENTED)
			retval = target_write_buffer(target, addr, len, (uint8_t *)separator);

		if (retval != ERROR_OK)
			gdb_connection->mem_write_error = true;
	}

	if (len < fast_limit) {
		if (retval != ERROR_OK) {
			gdb_error(connection, retval);
			gdb_connection->mem_write_error = false;
		} else {
			gdb_put_packet(connection, "OK", 2);
		}
	}

	return ERROR_OK;
}

static int gdb_step_continue_packet(struct connection *connection,
		char const *packet, int packet_size)
{
	struct target *target = get_available_target_from_connection(connection);
	bool current = false;
	uint64_t address = 0x0;
	int retval = ERROR_OK;

	LOG_DEBUG("-");

	if (packet_size > 1)
		address = strtoull(packet + 1, NULL, 16);
	else
		current = true;

	gdb_running_type = packet[0];
	if (packet[0] == 'c') {
		LOG_DEBUG("continue");
		/* resume at current address, don't handle breakpoints, not debugging */
		retval = target_resume(target, current, address, false, false);
	} else if (packet[0] == 's') {
		LOG_DEBUG("step");
		/* step at current or address, don't handle breakpoints */
		retval = target_step(target, current, address, false);
	}
	return retval;
}

static int gdb_breakpoint_watchpoint_packet(struct connection *connection,
		char const *packet, int packet_size)
{
	struct target *target = get_available_target_from_connection(connection);
	int type;
	enum breakpoint_type bp_type = BKPT_SOFT /* dummy init to avoid warning */;
	enum watchpoint_rw wp_type = WPT_READ /* dummy init to avoid warning */;
	uint64_t address;
	uint32_t size;
	char *separator;
	int retval;

	LOG_DEBUG("[%s]", target_name(target));

	type = strtoul(packet + 1, &separator, 16);

	if (type == 0)	/* memory breakpoint */
		bp_type = BKPT_SOFT;
	else if (type == 1)	/* hardware breakpoint */
		bp_type = BKPT_HARD;
	else if (type == 2)	/* write watchpoint */
		wp_type = WPT_WRITE;
	else if (type == 3)	/* read watchpoint */
		wp_type = WPT_READ;
	else if (type == 4)	/* access watchpoint */
		wp_type = WPT_ACCESS;
	else {
		LOG_ERROR("invalid gdb watch/breakpoint type(%d), dropping connection", type);
		return ERROR_SERVER_REMOTE_CLOSED;
	}

	if (gdb_breakpoint_override && ((bp_type == BKPT_SOFT) || (bp_type == BKPT_HARD)))
		bp_type = gdb_breakpoint_override_type;

	if (*separator != ',') {
		LOG_ERROR("incomplete breakpoint/watchpoint packet received, dropping connection");
		return ERROR_SERVER_REMOTE_CLOSED;
	}

	address = strtoull(separator + 1, &separator, 16);

	if (*separator != ',') {
		LOG_ERROR("incomplete breakpoint/watchpoint packet received, dropping connection");
		return ERROR_SERVER_REMOTE_CLOSED;
	}

	size = strtoul(separator + 1, &separator, 16);

	switch (type) {
		case 0:
		case 1:
			if (packet[0] == 'Z') {
				retval = breakpoint_add(target, address, size, bp_type);
			} else {
				assert(packet[0] == 'z');
				retval = breakpoint_remove(target, address);
			}
			break;
		case 2:
		case 3:
		case 4:
		{
			if (packet[0] == 'Z') {
				retval = watchpoint_add(target, address, size, wp_type, 0, WATCHPOINT_IGNORE_DATA_VALUE_MASK);
			} else {
				assert(packet[0] == 'z');
				retval = watchpoint_remove(target, address);
			}
			break;
		}
		default:
		{
			retval = ERROR_NOT_IMPLEMENTED;
			break;
		}
	}

	if (retval == ERROR_NOT_IMPLEMENTED) {
		/* Send empty reply to report that watchpoints of this type are not supported */
		return gdb_put_packet(connection, "", 0);
	}
	if (retval != ERROR_OK)
		return gdb_error(connection, retval);
	return gdb_put_packet(connection, "OK", 2);
}

/* print out a string and allocate more space as needed,
 * mainly used for XML at this point
 */
static __attribute__ ((format (PRINTF_ATTRIBUTE_FORMAT, 5, 6))) void xml_printf(int *retval,
		char **xml, int *pos, int *size, const char *fmt, ...)
{
	if (*retval != ERROR_OK)
		return;
	int first = 1;

	for (;; ) {
		if ((!*xml) || (!first)) {
			/* start by 0 to exercise all the code paths.
			 * Need minimum 2 bytes to fit 1 char and 0 terminator. */

			*size = *size * 2 + 2;
			char *t = *xml;
			*xml = realloc(*xml, *size);
			if (!*xml) {
				free(t);
				*retval = ERROR_SERVER_REMOTE_CLOSED;
				return;
			}
		}

		va_list ap;
		int ret;
		va_start(ap, fmt);
		ret = vsnprintf(*xml + *pos, *size - *pos, fmt, ap);
		va_end(ap);
		if ((ret > 0) && ((ret + 1) < *size - *pos)) {
			*pos += ret;
			return;
		}
		/* there was just enough or not enough space, allocate more. */
		first = 0;
	}
}

static int decode_xfer_read(char const *buf, char **annex, int *ofs, unsigned int *len)
{
	/* Locate the annex. */
	const char *annex_end = strchr(buf, ':');
	if (!annex_end)
		return ERROR_FAIL;

	/* After the read marker and annex, qXfer looks like a
	 * traditional 'm' packet. */
	char *separator;
	*ofs = strtoul(annex_end + 1, &separator, 16);

	if (*separator != ',')
		return ERROR_FAIL;

	*len = strtoul(separator + 1, NULL, 16);

	/* Extract the annex if needed */
	if (annex) {
		*annex = strndup(buf, annex_end - buf);
		if (!*annex)
			return ERROR_FAIL;
	}

	return ERROR_OK;
}

static int generate_memory_map_xml(struct target_memory_map *memory_map, char **xml, int *pos, int *size)
{
	int retval = ERROR_OK;

	xml_printf(&retval, xml, pos, size, "<memory-map>\n");

	for (unsigned int i = 0; i < memory_map->num_regions; i++) {
		struct target_memory_region *region = &memory_map->regions[i];

		switch (region->type) {
			case MEMORY_TYPE_RAM:
			case MEMORY_TYPE_RW:
				xml_printf(&retval, xml, pos, size,
					"<memory type=\"ram\" start=\"" TARGET_ADDR_FMT "\" "
					"length=\"0x%x\"/>\n",
					region->start, region->length);
				break;

			case MEMORY_TYPE_FLASH:
				xml_printf(&retval, xml, pos, size,
					"<memory type=\"flash\" start=\"" TARGET_ADDR_FMT "\" "
					"length=\"0x%x\">\n"
					"<property name=\"blocksize\">0x%x</property>\n"
					"</memory>\n",
					region->start, region->length, region->block_size);
				break;

			case MEMORY_TYPE_ROM:
				xml_printf(&retval, xml, pos, size,
					"<memory type=\"rom\" start=\"" TARGET_ADDR_FMT "\" "
					"length=\"0x%x\"/>\n",
					region->start, region->length);
				break;
		}
	}

	xml_printf(&retval, xml, pos, size, "</memory-map>\n");

	return retval;
}

static void print_memory_map(const struct target_memory_map *memory_map)
{
	if (!memory_map || !memory_map->regions) {
		LOG_ERROR("Invalid memory map");
		return;
	}

	LOG_USER("Memory Map (%d regions):", memory_map->num_regions);
	LOG_USER("----------------------------------------");
	LOG_USER("Type    Start Addr    Length    BlockSize");
	LOG_USER("----------------------------------------");

	for (unsigned int i = 0; i < memory_map->num_regions; i++) {
		const struct target_memory_region *region = &memory_map->regions[i];
		const char *type_str;

		switch (region->type) {
			case MEMORY_TYPE_RAM:
				type_str = "RAM";
				break;
			case MEMORY_TYPE_RW:
				type_str = "R/W";
				break;
			case MEMORY_TYPE_FLASH:
				type_str = "FLASH";
				break;
			case MEMORY_TYPE_ROM:
				type_str = "ROM";
				break;
			default:
				type_str = "?";
				break;
		}

		LOG_USER("%-7s 0x%08" PRIx64 "  0x%08x  0x%08x",
			type_str,
			region->start,
			region->length,
			region->block_size);
	}
	LOG_USER("----------------------------------------");
}

static int compare_memory_regions(const void *a, const void *b)
{
	const struct target_memory_region *r1 = a;
	const struct target_memory_region *r2 = b;

	if (r1->start == r2->start)
		return 0;
	else if (r1->start > r2->start)
		return 1;
	else
		return -1;
}

static int gdb_memory_map(struct connection *connection, char const *packet, int packet_size)
{
	/* We get away with only specifying flash here. Regions that are not
	 * specified are treated as if we provided no memory map(if not we
	 * could detect the holes and mark them as RAM).
	 * Normally we only execute this code once, but no big deal if we
	 * have to regenerate it a couple of times.
	 */

	struct target *target = get_available_target_from_connection(connection);
	struct flash_bank *p;
	char *xml = NULL;
	int size = 0;
	int pos = 0;
	int retval = ERROR_OK;
	struct flash_bank **banks;
	int offset;
	int length;
	char *separator;
	unsigned int target_flash_banks = 0;
	struct target_memory_map memory_map = { 0 };

	/* skip command character */
	packet += 23;

	offset = strtoul(packet, &separator, 16);
	length = strtoul(separator + 1, &separator, 16);

	if (target->type->get_gdb_memory_map)
		target->type->get_gdb_memory_map(target, &memory_map);

	banks = malloc(sizeof(struct flash_bank *)*flash_get_bank_count());

	for (unsigned int i = 0; i < flash_get_bank_count(); i++) {
		p = get_flash_bank_by_num_noprobe(i);
		if (p->target != target)
			continue;
		retval = get_flash_bank_by_num(i, &p);
		if (retval != ERROR_OK) {
			free(banks);
			gdb_error(connection, retval);
			return retval;
		}
		banks[target_flash_banks++] = p;
	}

	for (unsigned int i = 0; i < target_flash_banks; i++) {
		unsigned int sector_size = 0;
		unsigned int group_len = 0;
		struct target_memory_region region = { 0 };

		p = banks[i];

		/* Report adjacent groups of same-size sectors.  So for
		 * example top boot CFI flash will list an initial region
		 * with several large sectors (maybe 128KB) and several
		 * smaller ones at the end (maybe 32KB).  STR7 will have
		 * regions with 8KB, 32KB, and 64KB sectors; etc.
		 */
		for (unsigned int j = 0; j < p->num_sectors; j++) {

			/* Maybe start a new group of sectors. */
			if (sector_size == 0) {
				if (p->sectors[j].offset + p->sectors[j].size > p->size) {
					LOG_WARNING("The flash sector at offset 0x%08" PRIx32
						" overflows the end of %s bank.",
						p->sectors[j].offset, p->name);
					LOG_WARNING("The rest of bank will not show in gdb memory map.");
					break;
				}
				region.type = MEMORY_TYPE_FLASH;
				region.start = p->base + p->sectors[j].offset;
				sector_size = p->sectors[j].size;
				group_len = sector_size;
			} else {
				group_len += sector_size; /* equal to p->sectors[j].size */
			}

			/* Does this finish a group of sectors?
			 * If not, continue an already-started group.
			 */
			if (j < p->num_sectors - 1
					&& p->sectors[j + 1].size == sector_size
					&& p->sectors[j + 1].offset == p->sectors[j].offset + sector_size
					&& p->sectors[j + 1].offset + p->sectors[j + 1].size <= p->size)
				continue;

			region.length = group_len;
			region.block_size = sector_size;
			target_add_memory_region(&memory_map, &region);
			sector_size = 0;
		}
	}

	free(banks);

	/* Sort all regions by start address in ascending order */
	qsort(memory_map.regions, memory_map.num_regions, sizeof(struct target_memory_region), compare_memory_regions);

	/* We need to report non-flash memory as ram (or rather read/write) by default for GDB,
	 * since it has no concept of non-cacheable read/write memory (i/o etc).
	 */
	target_addr_t current_addr = 0;
	struct target_memory_region ram_regions[memory_map.num_regions + 1]; /* max gap count between added regions */
	unsigned int ram_region_cnt = 0;

	for (unsigned int i = 0; i < memory_map.num_regions; i++) {
		struct target_memory_region *existing = &memory_map.regions[i];

		if (current_addr < existing->start) {
			ram_regions[ram_region_cnt].type = MEMORY_TYPE_RW;
			ram_regions[ram_region_cnt].start = current_addr;
			ram_regions[ram_region_cnt].length = existing->start - current_addr;
			ram_regions[ram_region_cnt].block_size = 0;
			ram_region_cnt++;
		}

		current_addr = existing->start + existing->length;
	}

	/* Add final RAM region if needed */
	if (current_addr < target_address_max(target)) {
		ram_regions[ram_region_cnt].type = MEMORY_TYPE_RW;
		ram_regions[ram_region_cnt].start = current_addr;
		ram_regions[ram_region_cnt].length = target_address_max(target) - current_addr + 1;
		ram_regions[ram_region_cnt].block_size = 0;
		ram_region_cnt++;
	}

	for (unsigned int i = 0; i < ram_region_cnt; i++)
		target_add_memory_region(&memory_map, &ram_regions[i]);

	/* Sort is not needed here, but for the pretty print we do it anyway */
	qsort(memory_map.regions, memory_map.num_regions, sizeof(struct target_memory_region), compare_memory_regions);
	print_memory_map(&memory_map);

	if (memory_map.num_regions > 0) {
		retval = generate_memory_map_xml(&memory_map, &xml, &pos, &size);
		if (retval != ERROR_OK) {
			free(xml);
			free(memory_map.regions);
			gdb_error(connection, retval);
			return retval;
		}
	}

	if (offset + length > pos)
		length = pos - offset;

	char *t = malloc(length + 1);
	t[0] = 'l';
	memcpy(t + 1, xml + offset, length);
	gdb_put_packet(connection, t, length + 1);

	free(t);
	free(xml);
	free(memory_map.regions);
	return ERROR_OK;
}

static const char *gdb_get_reg_type_name(enum reg_type type)
{
	switch (type) {
		case REG_TYPE_BOOL:
			return "bool";
		case REG_TYPE_INT:
			return "int";
		case REG_TYPE_INT8:
			return "int8";
		case REG_TYPE_INT16:
			return "int16";
		case REG_TYPE_INT32:
			return "int32";
		case REG_TYPE_INT64:
			return "int64";
		case REG_TYPE_INT128:
			return "int128";
		case REG_TYPE_UINT:
			return "uint";
		case REG_TYPE_UINT8:
			return "uint8";
		case REG_TYPE_UINT16:
			return "uint16";
		case REG_TYPE_UINT32:
			return "uint32";
		case REG_TYPE_UINT64:
			return "uint64";
		case REG_TYPE_UINT128:
			return "uint128";
		case REG_TYPE_CODE_PTR:
			return "code_ptr";
		case REG_TYPE_DATA_PTR:
			return "data_ptr";
		case REG_TYPE_FLOAT:
			return "float";
		case REG_TYPE_IEEE_SINGLE:
			return "ieee_single";
		case REG_TYPE_IEEE_DOUBLE:
			return "ieee_double";
		case REG_TYPE_ARCH_DEFINED:
			return "int"; /* return arbitrary string to avoid compile warning. */
	}

	return "int"; /* "int" as default value */
}

static int lookup_add_arch_defined_types(char const **arch_defined_types_list[], const char *type_id,
					int *num_arch_defined_types)
{
	int tbl_sz = *num_arch_defined_types;

	if (type_id && (strcmp(type_id, ""))) {
		for (int j = 0; j < (tbl_sz + 1); j++) {
			if (!((*arch_defined_types_list)[j])) {
				(*arch_defined_types_list)[tbl_sz++] = type_id;
				*arch_defined_types_list = realloc(*arch_defined_types_list,
								sizeof(char *) * (tbl_sz + 1));
				(*arch_defined_types_list)[tbl_sz] = NULL;
				*num_arch_defined_types = tbl_sz;
				return 1;
			} else {
				if (!strcmp((*arch_defined_types_list)[j], type_id))
					return 0;
			}
		}
	}

	return -1;
}

static int gdb_generate_reg_type_description(struct target *target,
		char **tdesc, int *pos, int *size, struct reg_data_type *type,
		char const **arch_defined_types_list[], int *num_arch_defined_types)
{
	int retval = ERROR_OK;

	if (type->type_class == REG_TYPE_CLASS_VECTOR) {
		struct reg_data_type *data_type = type->reg_type_vector->type;
		if (data_type->type == REG_TYPE_ARCH_DEFINED) {
			if (lookup_add_arch_defined_types(arch_defined_types_list, data_type->id,
							num_arch_defined_types))
				gdb_generate_reg_type_description(target, tdesc, pos, size, data_type,
								arch_defined_types_list,
								num_arch_defined_types);
		}
		/* <vector id="id" type="type" count="count"/> */
		xml_printf(&retval, tdesc, pos, size,
				"<vector id=\"%s\" type=\"%s\" count=\"%" PRIu32 "\"/>\n",
				type->id, type->reg_type_vector->type->id,
				type->reg_type_vector->count);

	} else if (type->type_class == REG_TYPE_CLASS_UNION) {
		struct reg_data_type_union_field *field;
		field = type->reg_type_union->fields;
		while (field) {
			struct reg_data_type *data_type = field->type;
			if (data_type->type == REG_TYPE_ARCH_DEFINED) {
				if (lookup_add_arch_defined_types(arch_defined_types_list, data_type->id,
								num_arch_defined_types))
					gdb_generate_reg_type_description(target, tdesc, pos, size, data_type,
									arch_defined_types_list,
									num_arch_defined_types);
			}

			field = field->next;
		}
		/* <union id="id">
		 *  <field name="name" type="type"/> ...
		 * </union> */
		xml_printf(&retval, tdesc, pos, size,
				"<union id=\"%s\">\n",
				type->id);

		field = type->reg_type_union->fields;
		while (field) {
			xml_printf(&retval, tdesc, pos, size,
					"<field name=\"%s\" type=\"%s\"/>\n",
					field->name, field->type->id);

			field = field->next;
		}

		xml_printf(&retval, tdesc, pos, size,
				"</union>\n");

	} else if (type->type_class == REG_TYPE_CLASS_STRUCT) {
		struct reg_data_type_struct_field *field;
		field = type->reg_type_struct->fields;

		if (field->use_bitfields) {
			/* <struct id="id" size="size">
			 *  <field name="name" start="start" end="end"/> ...
			 * </struct> */
			xml_printf(&retval, tdesc, pos, size,
					"<struct id=\"%s\" size=\"%" PRIu32 "\">\n",
					type->id, type->reg_type_struct->size);
			while (field) {
				xml_printf(&retval, tdesc, pos, size,
						"<field name=\"%s\" start=\"%" PRIu32 "\" end=\"%" PRIu32 "\" type=\"%s\" />\n",
						field->name, field->bitfield->start, field->bitfield->end,
						gdb_get_reg_type_name(field->bitfield->type));

				field = field->next;
			}
		} else {
			while (field) {
				struct reg_data_type *data_type = field->type;
				if (data_type->type == REG_TYPE_ARCH_DEFINED) {
					if (lookup_add_arch_defined_types(arch_defined_types_list, data_type->id,
									num_arch_defined_types))
						gdb_generate_reg_type_description(target, tdesc, pos, size, data_type,
										arch_defined_types_list,
										num_arch_defined_types);
				}
			}

			/* <struct id="id">
			 *  <field name="name" type="type"/> ...
			 * </struct> */
			xml_printf(&retval, tdesc, pos, size,
					"<struct id=\"%s\">\n",
					type->id);
			while (field) {
				xml_printf(&retval, tdesc, pos, size,
						"<field name=\"%s\" type=\"%s\"/>\n",
						field->name, field->type->id);

				field = field->next;
			}
		}

		xml_printf(&retval, tdesc, pos, size,
				"</struct>\n");

	} else if (type->type_class == REG_TYPE_CLASS_FLAGS) {
		/* <flags id="id" size="size">
		 *  <field name="name" start="start" end="end"/> ...
		 * </flags> */
		xml_printf(&retval, tdesc, pos, size,
				"<flags id=\"%s\" size=\"%" PRIu32 "\">\n",
				type->id, type->reg_type_flags->size);

		struct reg_data_type_flags_field *field;
		field = type->reg_type_flags->fields;
		while (field) {
			xml_printf(&retval, tdesc, pos, size,
					"<field name=\"%s\" start=\"%" PRIu32 "\" end=\"%" PRIu32 "\" type=\"%s\" />\n",
					field->name, field->bitfield->start, field->bitfield->end,
					gdb_get_reg_type_name(field->bitfield->type));

			field = field->next;
		}

		xml_printf(&retval, tdesc, pos, size,
				"</flags>\n");

	}

	return ERROR_OK;
}

/* Get a list of available target registers features. feature_list must
 * be freed by caller.
 */
static int get_reg_features_list(struct target *target, char const **feature_list[], int *feature_list_size,
		struct reg **reg_list, int reg_list_size)
{
	int tbl_sz = 0;

	/* Start with only one element */
	*feature_list = calloc(1, sizeof(char *));

	for (int i = 0; i < reg_list_size; i++) {
		if (!reg_list[i]->exist || reg_list[i]->hidden)
			continue;

		if (reg_list[i]->feature
			&& reg_list[i]->feature->name
			&& (strcmp(reg_list[i]->feature->name, ""))) {
			/* We found a feature, check if the feature is already in the
			 * table. If not, allocate a new entry for the table and
			 * put the new feature in it.
			 */
			for (int j = 0; j < (tbl_sz + 1); j++) {
				if (!((*feature_list)[j])) {
					(*feature_list)[tbl_sz++] = reg_list[i]->feature->name;
					*feature_list = realloc(*feature_list, sizeof(char *) * (tbl_sz + 1));
					(*feature_list)[tbl_sz] = NULL;
					break;
				} else {
					if (!strcmp((*feature_list)[j], reg_list[i]->feature->name))
						break;
				}
			}
		}
	}

	if (feature_list_size)
		*feature_list_size = tbl_sz;

	return ERROR_OK;
}

/* Create a register list that's the union of all the registers of the SMP
 * group this target is in. If the target is not part of an SMP group, this
 * returns the same as target_get_gdb_reg_list_noread().
 */
static int smp_reg_list_noread(struct target *target,
		struct reg **combined_list[], int *combined_list_size,
		enum target_register_class reg_class)
{
	if (!target->smp)
		return target_get_gdb_reg_list_noread(target, combined_list,
				combined_list_size, REG_CLASS_ALL);

	unsigned int combined_allocated = 256;
	struct reg **local_list = malloc(combined_allocated * sizeof(struct reg *));
	if (!local_list) {
		LOG_ERROR("malloc(%zu) failed", combined_allocated * sizeof(struct reg *));
		return ERROR_FAIL;
	}
	unsigned int local_list_size = 0;

	struct target_list *head;
	foreach_smp_target(head, target->smp_targets) {
		if (!target_was_examined(head->target))
			continue;

		struct reg **reg_list = NULL;
		int reg_list_size;
		int result = target_get_gdb_reg_list_noread(head->target, &reg_list,
				&reg_list_size, reg_class);
		if (result != ERROR_OK) {
			free(local_list);
			return result;
		}
		for (int i = 0; i < reg_list_size; i++) {
			bool found = false;
			struct reg *a = reg_list[i];
			if (a->exist) {
				/* Nested loop makes this O(n^2), but this entire function with
				 * 5 RISC-V targets takes just 2ms on my computer. Fast enough
				 * for me. */
				for (unsigned int j = 0; j < local_list_size; j++) {
					struct reg *b = local_list[j];
					if (!strcmp(a->name, b->name)) {
						found = true;
						if (a->size != b->size) {
							LOG_ERROR("SMP register %s is %d bits on one "
									"target, but %d bits on another target.",
									a->name, a->size, b->size);
							free(reg_list);
							free(local_list);
							return ERROR_FAIL;
						}
						break;
					}
				}
				if (!found) {
					LOG_TARGET_DEBUG(target, "%s not found in combined list", a->name);
					if (local_list_size >= combined_allocated) {
						combined_allocated *= 2;
						local_list = realloc(local_list, combined_allocated * sizeof(struct reg *));
						if (!local_list) {
							LOG_ERROR("realloc(%zu) failed", combined_allocated * sizeof(struct reg *));
							free(reg_list);
							return ERROR_FAIL;
						}
					}
					local_list[local_list_size] = a;
					local_list_size++;
				}
			}
		}
		free(reg_list);
	}

	if (local_list_size == 0) {
		LOG_ERROR("Unable to get register list");
		free(local_list);
		return ERROR_FAIL;
	}

	/* Now warn the user about any registers that weren't found in every target. */
	foreach_smp_target(head, target->smp_targets) {
		if (!target_was_examined(head->target))
			continue;

		struct reg **reg_list = NULL;
		int reg_list_size;
		int result = target_get_gdb_reg_list_noread(head->target, &reg_list,
				&reg_list_size, reg_class);
		if (result != ERROR_OK) {
			free(local_list);
			return result;
		}
		for (unsigned int i = 0; i < local_list_size; i++) {
			bool found = false;
			struct reg *a = local_list[i];
			for (int j = 0; j < reg_list_size; j++) {
				struct reg *b = reg_list[j];
				if (b->exist && !strcmp(a->name, b->name)) {
					found = true;
					break;
				}
			}
			if (!found) {
				LOG_TARGET_WARNING(head->target, "Register %s does not exist, which is part of an SMP group where "
					    "this register does exist.", a->name);
			}
		}
		free(reg_list);
	}

	*combined_list = local_list;
	*combined_list_size = local_list_size;
	return ERROR_OK;
}

static int gdb_generate_target_description(struct target *target, char **tdesc_out)
{
	int retval = ERROR_OK;
	struct reg **reg_list = NULL;
	int reg_list_size;
	char const *architecture;
	char const **features = NULL;
	int feature_list_size = 0;
	char *tdesc = NULL;
	int pos = 0;
	int size = 0;


	retval = smp_reg_list_noread(target, &reg_list, &reg_list_size,
			REG_CLASS_ALL);

	if (retval != ERROR_OK) {
		LOG_ERROR("get register list failed");
		retval = ERROR_FAIL;
		goto error;
	}

	if (reg_list_size <= 0) {
		LOG_ERROR("get register list failed");
		retval = ERROR_FAIL;
		goto error;
	}

	/* Get a list of available target registers features */
	retval = get_reg_features_list(target, &features, &feature_list_size, reg_list, reg_list_size);
	if (retval != ERROR_OK) {
		LOG_ERROR("Can't get the registers feature list");
		retval = ERROR_FAIL;
		goto error;
	}

	/* If we found some features associated with registers, create sections */
	int current_feature = 0;

	xml_printf(&retval, &tdesc, &pos, &size,
			"<?xml version=\"1.0\"?>\n"
			"<!DOCTYPE target SYSTEM \"gdb-target.dtd\">\n"
			"<target version=\"1.0\">\n");

	/* generate architecture element if supported by target */
	architecture = target_get_gdb_arch(target);
	if (architecture)
		xml_printf(&retval, &tdesc, &pos, &size,
				"<architecture>%s</architecture>\n", architecture);

	/* generate target description according to register list */
	if (features) {
		while (features[current_feature]) {
			char const **arch_defined_types = NULL;
			int num_arch_defined_types = 0;

			arch_defined_types = calloc(1, sizeof(char *));
			xml_printf(&retval, &tdesc, &pos, &size,
					"<feature name=\"%s\">\n",
					features[current_feature]);

			int i;
			for (i = 0; i < reg_list_size; i++) {

				if (!reg_list[i]->exist || reg_list[i]->hidden)
					continue;

				if (strcmp(reg_list[i]->feature->name, features[current_feature]))
					continue;

				const char *type_str;
				if (reg_list[i]->reg_data_type) {
					if (reg_list[i]->reg_data_type->type == REG_TYPE_ARCH_DEFINED) {
						/* generate <type... first, if there are architecture-defined types. */
						if (lookup_add_arch_defined_types(&arch_defined_types,
										reg_list[i]->reg_data_type->id,
										&num_arch_defined_types))
							gdb_generate_reg_type_description(target, &tdesc, &pos, &size,
											reg_list[i]->reg_data_type,
											&arch_defined_types,
											&num_arch_defined_types);

						type_str = reg_list[i]->reg_data_type->id;
					} else {
						/* predefined type */
						type_str = gdb_get_reg_type_name(
								reg_list[i]->reg_data_type->type);
					}
				} else {
					/* Default type is "int" */
					type_str = "int";
				}

				xml_printf(&retval, &tdesc, &pos, &size,
						"<reg name=\"%s\"", reg_list[i]->name);
				xml_printf(&retval, &tdesc, &pos, &size,
						" bitsize=\"%" PRIu32 "\"", reg_list[i]->size);
				xml_printf(&retval, &tdesc, &pos, &size,
						" regnum=\"%" PRIu32 "\"", reg_list[i]->number);
				if (reg_list[i]->caller_save)
					xml_printf(&retval, &tdesc, &pos, &size,
							" save-restore=\"yes\"");
				else
					xml_printf(&retval, &tdesc, &pos, &size,
							" save-restore=\"no\"");

				xml_printf(&retval, &tdesc, &pos, &size,
						" type=\"%s\"", type_str);

				if (reg_list[i]->group)
					xml_printf(&retval, &tdesc, &pos, &size,
							" group=\"%s\"", reg_list[i]->group);

				xml_printf(&retval, &tdesc, &pos, &size,
						"/>\n");
			}

			xml_printf(&retval, &tdesc, &pos, &size,
					"</feature>\n");

			current_feature++;
			free(arch_defined_types);
		}
	}

	xml_printf(&retval, &tdesc, &pos, &size,
			"</target>\n");

error:
	free(features);
	free(reg_list);

	if (retval == ERROR_OK)
		*tdesc_out = tdesc;
	else
		free(tdesc);

	return retval;
}

static int gdb_get_target_description_chunk(struct target *target, struct target_desc_format *target_desc,
		char **chunk, int32_t offset, uint32_t length)
{
	if (!target_desc) {
		LOG_ERROR("Unable to Generate Target Description");
		return ERROR_FAIL;
	}

	char *tdesc = target_desc->tdesc;
	uint32_t tdesc_length = target_desc->tdesc_length;

	if (!tdesc) {
		int retval = gdb_generate_target_description(target, &tdesc);
		if (retval != ERROR_OK) {
			LOG_ERROR("Unable to Generate Target Description");
			return ERROR_FAIL;
		}

		tdesc_length = strlen(tdesc);
	}

	char transfer_type;

	if (length < (tdesc_length - offset))
		transfer_type = 'm';
	else
		transfer_type = 'l';

	*chunk = malloc(length + 2);
	if (!*chunk) {
		LOG_ERROR("Unable to allocate memory");
		return ERROR_FAIL;
	}

	(*chunk)[0] = transfer_type;
	if (transfer_type == 'm') {
		strncpy((*chunk) + 1, tdesc + offset, length);
		(*chunk)[1 + length] = '\0';
	} else {
		strncpy((*chunk) + 1, tdesc + offset, tdesc_length - offset);
		(*chunk)[1 + (tdesc_length - offset)] = '\0';

		/* After gdb-server sends out last chunk, invalidate tdesc. */
		free(tdesc);
		tdesc = NULL;
		tdesc_length = 0;
	}

	target_desc->tdesc = tdesc;
	target_desc->tdesc_length = tdesc_length;

	return ERROR_OK;
}

static int gdb_target_description_supported(struct target *target, bool *supported)
{
	int retval = ERROR_OK;
	struct reg **reg_list = NULL;
	int reg_list_size = 0;
	char const **features = NULL;
	int feature_list_size = 0;

	char const *architecture = target_get_gdb_arch(target);

	retval = target_get_gdb_reg_list_noread(target, &reg_list,
			&reg_list_size, REG_CLASS_ALL);
	if (retval != ERROR_OK) {
		LOG_ERROR("get register list failed");
		goto error;
	}

	if (reg_list_size <= 0) {
		LOG_ERROR("get register list failed");
		retval = ERROR_FAIL;
		goto error;
	}

	/* Get a list of available target registers features */
	retval = get_reg_features_list(target, &features, &feature_list_size, reg_list, reg_list_size);
	if (retval != ERROR_OK) {
		LOG_ERROR("Can't get the registers feature list");
		goto error;
	}

	if (supported) {
		if (architecture || feature_list_size)
			*supported = true;
		else
			*supported = false;
	}

error:
	free(features);

	free(reg_list);

	return retval;
}

static int gdb_generate_thread_list(struct target *target, char **thread_list_out)
{
	struct rtos *rtos = target->rtos;
	int retval = ERROR_OK;
	char *thread_list = NULL;
	int pos = 0;
	int size = 0;

	xml_printf(&retval, &thread_list, &pos, &size,
		   "<?xml version=\"1.0\"?>\n"
		   "<threads>\n");

	if (rtos) {
		for (int i = 0; i < rtos->thread_count; i++) {
			struct thread_detail *thread_detail = &rtos->thread_details[i];

			if (!thread_detail->exists)
				continue;

			if (thread_detail->thread_name_str)
				xml_printf(&retval, &thread_list, &pos, &size,
					   "<thread id=\"%" PRIx64 "\" name=\"%s\">",
					   thread_detail->threadid,
					   thread_detail->thread_name_str);
			else
				xml_printf(&retval, &thread_list, &pos, &size,
					   "<thread id=\"%" PRIx64 "\">", thread_detail->threadid);

			if (thread_detail->thread_name_str)
				xml_printf(&retval, &thread_list, &pos, &size,
					   "Name: %s", thread_detail->thread_name_str);

			if (thread_detail->extra_info_str) {
				if (thread_detail->thread_name_str)
					xml_printf(&retval, &thread_list, &pos, &size,
						   ", ");
				xml_printf(&retval, &thread_list, &pos, &size,
					   "%s", thread_detail->extra_info_str);
			}

			xml_printf(&retval, &thread_list, &pos, &size,
				   "</thread>\n");
		}
	}

	xml_printf(&retval, &thread_list, &pos, &size,
		   "</threads>\n");

	if (retval == ERROR_OK)
		*thread_list_out = thread_list;
	else
		free(thread_list);

	return retval;
}

static int gdb_get_thread_list_chunk(struct target *target, char **thread_list,
		char **chunk, int32_t offset, uint32_t length)
{
	if (!*thread_list) {
		int retval = gdb_generate_thread_list(target, thread_list);
		if (retval != ERROR_OK) {
			LOG_ERROR("Unable to Generate Thread List");
			return ERROR_FAIL;
		}
	}

	size_t thread_list_length = strlen(*thread_list);
	char transfer_type;

	length = MIN(length, thread_list_length - offset);
	if (length < (thread_list_length - offset))
		transfer_type = 'm';
	else
		transfer_type = 'l';

	*chunk = malloc(length + 2 + 3);
	/* Allocating extra 3 bytes prevents false positive valgrind report
	 * of strlen(chunk) word access:
	 * Invalid read of size 4
	 * Address 0x4479934 is 44 bytes inside a block of size 45 alloc'd */
	if (!*chunk) {
		LOG_ERROR("Unable to allocate memory");
		return ERROR_FAIL;
	}

	(*chunk)[0] = transfer_type;
	strncpy((*chunk) + 1, (*thread_list) + offset, length);
	(*chunk)[1 + length] = '\0';

	/* After gdb-server sends out last chunk, invalidate thread list. */
	if (transfer_type == 'l') {
		free(*thread_list);
		*thread_list = NULL;
	}

	return ERROR_OK;
}

static int gdb_query_packet(struct connection *connection,
		char const *packet, int packet_size)
{
	struct command_context *cmd_ctx = connection->cmd_ctx;
	struct gdb_connection *gdb_connection = connection->priv;
	struct target *target = get_target_from_connection(connection);

	if (strncmp(packet, "qRcmd,", 6) == 0) {
		if (packet_size > 6) {
			Jim_Interp *interp = cmd_ctx->interp;
			char *cmd;
			cmd = malloc((packet_size - 6) / 2 + 1);
			size_t len = unhexify((uint8_t *)cmd, packet + 6, (packet_size - 6) / 2);
			cmd[len] = 0;

			/* We want to print all debug output to GDB connection */
			gdb_connection->output_flag = GDB_OUTPUT_ALL;
			target_call_timer_callbacks_now();
			/* some commands need to know the GDB connection, make note of current
			 * GDB connection. */
			current_gdb_connection = gdb_connection;

			struct target *saved_target_override = cmd_ctx->current_target_override;
			cmd_ctx->current_target_override = NULL;

			struct command_context *old_context = Jim_GetAssocData(interp, "context");
			Jim_DeleteAssocData(interp, "context");
			int retval = Jim_SetAssocData(interp, "context", NULL, cmd_ctx);
			if (retval == JIM_OK) {
				retval = Jim_EvalObj(interp, Jim_NewStringObj(interp, cmd, -1));
				Jim_DeleteAssocData(interp, "context");
			}
			int inner_retval = Jim_SetAssocData(interp, "context", NULL, old_context);
			if (retval == JIM_OK)
				retval = inner_retval;

			cmd_ctx->current_target_override = saved_target_override;

			current_gdb_connection = NULL;
			target_call_timer_callbacks_now();
			gdb_connection->output_flag = GDB_OUTPUT_NO;
			free(cmd);
			if (retval == JIM_RETURN)
				retval = interp->returnCode;
			int lenmsg;
			const char *cretmsg = Jim_GetString(Jim_GetResult(interp), &lenmsg);
			char *retmsg;
			if (lenmsg && cretmsg[lenmsg - 1] != '\n') {
				retmsg = alloc_printf("%s\n", cretmsg);
				lenmsg++;
			} else {
				retmsg = strdup(cretmsg);
			}
			if (!retmsg)
				return ERROR_GDB_BUFFER_TOO_SMALL;

			if (retval == JIM_OK) {
				if (lenmsg) {
					char *hex_buffer = malloc(lenmsg * 2 + 1);
					if (!hex_buffer) {
						free(retmsg);
						return ERROR_GDB_BUFFER_TOO_SMALL;
					}

					size_t pkt_len = hexify(hex_buffer, (const uint8_t *)retmsg, lenmsg,
											lenmsg * 2 + 1);
					gdb_put_packet(connection, hex_buffer, pkt_len);
					free(hex_buffer);
				} else {
					gdb_put_packet(connection, "OK", 2);
				}
			} else {
				if (lenmsg)
					gdb_output_con(connection, retmsg);
				gdb_send_error(connection, retval);
			}
			free(retmsg);
			return ERROR_OK;
		}
		gdb_put_packet(connection, "OK", 2);
		return ERROR_OK;
	} else if (strncmp(packet, "qCRC:", 5) == 0) {
		if (packet_size > 5) {
			int retval;
			char gdb_reply[10];
			char *separator;
			uint32_t checksum;
			target_addr_t addr = 0;
			uint32_t len = 0;

			/* skip command character */
			packet += 5;

			addr = strtoull(packet, &separator, 16);

			if (*separator != ',') {
				LOG_ERROR("incomplete read memory packet received, dropping connection");
				return ERROR_SERVER_REMOTE_CLOSED;
			}

			len = strtoul(separator + 1, NULL, 16);

			gdb_connection->output_flag = GDB_OUTPUT_NOTIF;
			retval = target_checksum_memory(target, addr, len, &checksum);
			gdb_connection->output_flag = GDB_OUTPUT_NO;

			if (retval == ERROR_OK) {
				snprintf(gdb_reply, 10, "C%8.8" PRIx32 "", checksum);
				gdb_put_packet(connection, gdb_reply, 9);
			} else {
				retval = gdb_error(connection, retval);
				if (retval != ERROR_OK)
					return retval;
			}

			return ERROR_OK;
		}
	} else if (strncmp(packet, "qSupported", 10) == 0) {
		/* we currently support packet size and qXfer:memory-map:read (if enabled)
		 * qXfer:features:read is supported for some targets */
		int retval = ERROR_OK;
		char *buffer = NULL;
		int pos = 0;
		int size = 0;
		bool gdb_target_desc_supported = false;

		/* we need to test that the target supports target descriptions */
		retval = gdb_target_description_supported(target, &gdb_target_desc_supported);
		if (retval != ERROR_OK) {
			LOG_INFO("Failed detecting Target Description Support, disabling");
			gdb_target_desc_supported = false;
		}

		/* support may be disabled globally */
		if (!gdb_use_target_description) {
			if (gdb_target_desc_supported)
				LOG_WARNING("Target Descriptions Supported, but disabled");
			gdb_target_desc_supported = false;
		}

		xml_printf(&retval,
			&buffer,
			&pos,
			&size,
			"PacketSize=%x;qXfer:memory-map:read%c;qXfer:features:read%c;qXfer:threads:read+;QStartNoAckMode+;vContSupported+",
			GDB_BUFFER_SIZE,
			(gdb_use_memory_map && (flash_get_bank_count() > 0)) ? '+' : '-',
			gdb_target_desc_supported ? '+' : '-');

		if (retval != ERROR_OK) {
			gdb_send_error(connection, 01);
			return ERROR_OK;
		}

		gdb_put_packet(connection, buffer, strlen(buffer));
		free(buffer);

		return ERROR_OK;
	} else if ((strncmp(packet, "qXfer:memory-map:read::", 23) == 0)
		   && (flash_get_bank_count() > 0))
		return gdb_memory_map(connection, packet, packet_size);
	else if (strncmp(packet, "qXfer:features:read:", 20) == 0) {
		char *xml = NULL;
		int retval = ERROR_OK;

		int offset;
		unsigned int length;

		/* skip command character */
		packet += 20;

		if (decode_xfer_read(packet, NULL, &offset, &length) < 0) {
			gdb_send_error(connection, 01);
			return ERROR_OK;
		}

		/* Target should prepare correct target description for annex.
		 * The first character of returned xml is 'm' or 'l'. 'm' for
		 * there are *more* chunks to transfer. 'l' for it is the *last*
		 * chunk of target description.
		 */
		retval = gdb_get_target_description_chunk(target, &gdb_connection->target_desc,
				&xml, offset, length);
		if (retval != ERROR_OK) {
			gdb_error(connection, retval);
			return retval;
		}

		gdb_put_packet(connection, xml, strlen(xml));

		free(xml);
		return ERROR_OK;
	} else if (strncmp(packet, "qXfer:threads:read:", 19) == 0) {
		char *xml = NULL;
		int retval = ERROR_OK;

		int offset;
		unsigned int length;

		/* skip command character */
		packet += 19;

		if (decode_xfer_read(packet, NULL, &offset, &length) < 0) {
			gdb_send_error(connection, 01);
			return ERROR_OK;
		}

		/* Target should prepare correct thread list for annex.
		 * The first character of returned xml is 'm' or 'l'. 'm' for
		 * there are *more* chunks to transfer. 'l' for it is the *last*
		 * chunk of target description.
		 */
		retval = gdb_get_thread_list_chunk(target, &gdb_connection->thread_list,
						   &xml, offset, length);
		if (retval != ERROR_OK) {
			gdb_error(connection, retval);
			return retval;
		}

		gdb_put_packet(connection, xml, strlen(xml));

		free(xml);

		/* ESPRESSIF: will be used to process lazy breakpoints */
		target_call_event_callbacks(target, TARGET_EVENT_QXFER_THREAD_READ_END);

		return ERROR_OK;
	} else if (strncmp(packet, "QStartNoAckMode", 15) == 0) {
		gdb_connection->noack_mode = 1;
		gdb_put_packet(connection, "OK", 2);
		return ERROR_OK;
	} else if (target->type->gdb_query_custom) {
		char *buffer = NULL;
		int ret = target->type->gdb_query_custom(target, packet, &buffer);
		gdb_put_packet(connection, buffer, strlen(buffer));
		return ret;
	}

	gdb_put_packet(connection, "", 0);
	return ERROR_OK;
}

static bool gdb_handle_vcont_packet(struct connection *connection, const char *packet,
	__attribute__((unused)) int packet_size)
{
	struct gdb_connection *gdb_connection = connection->priv;
	struct target *target = get_available_target_from_connection(connection);
	const char *parse = packet;
	int retval;

	/* query for vCont supported */
	if (parse[0] == '?') {
		if (target->type->step) {
			/* gdb doesn't accept c without C and s without S */
			gdb_put_packet(connection, "vCont;c;C;s;S", 13);
			return true;
		}
		return false;
	}

	if (parse[0] == ';') {
		++parse;
	}

	/* simple case, a continue packet */
	if (parse[0] == 'c') {
		gdb_running_type = 'c';
		LOG_TARGET_DEBUG(target, "target continue");
		gdb_connection->output_flag = GDB_OUTPUT_ALL;
		retval = target_resume(target, true, 0, false, false);
		if (retval == ERROR_TARGET_NOT_HALTED)
			LOG_TARGET_INFO(target, "target was not halted when resume was requested");

		/* poll target in an attempt to make its internal state consistent */
		if (retval != ERROR_OK) {
			retval = target_poll(target);
			if (retval != ERROR_OK)
				LOG_TARGET_DEBUG(target, "error polling target after failed resume");
		}

		/*
		 * We don't report errors to gdb here, move frontend_state to
		 * TARGET_RUNNING to stay in sync with gdb's expectation of the
		 * target state
		 */
		gdb_connection->frontend_state = TARGET_RUNNING;
		target_call_event_callbacks(target, TARGET_EVENT_GDB_START);

		return true;
	}

	/* single-step or step-over-breakpoint */
	if (parse[0] == 's') {
		gdb_running_type = 's';
		bool fake_step = false;

		struct target *ct = target;
		bool current_pc = true;
		int64_t thread_id;
		parse++;
		if (parse[0] == ':') {
			char *endp;
			parse++;
			thread_id = strtoll(parse, &endp, 16);
			if (endp) {
				parse = endp;
			}
		} else {
			thread_id = 0;
		}

		if (target->rtos) {
			/* FIXME: why is this necessary? rtos state should be up-to-date here already! */
			rtos_update_threads(target);

			target->rtos->gdb_target_for_threadid(connection, thread_id, &ct);

			/*
			 * check if the thread to be stepped is the current rtos thread
			 * if not, we must fake the step
			 */
			if (target->rtos->current_thread != thread_id)
				fake_step = true;
		}

		if (parse[0] == ';') {
			++parse;

			if (parse[0] == 'c') {
				parse += 1;

				/* check if thread-id follows */
				if (parse[0] == ':') {
					int64_t tid;
					parse += 1;

					tid = strtoll(parse, NULL, 16);
					if (tid == thread_id) {
						/*
						 * Special case: only step a single thread (core),
						 * keep the other threads halted. Currently, only
						 * aarch64 target understands it. Other target types don't
						 * care (nobody checks the actual value of 'current')
						 * and it doesn't really matter. This deserves
						 * a symbolic constant and a formal interface documentation
						 * at a later time.
						 */
						LOG_DEBUG("request to step current core only");
						/* uncomment after checking that indeed other targets are safe */
						/*current_pc = 2;*/
					}
				}
			}
		}

		LOG_TARGET_DEBUG(ct, "single-step thread %" PRIx64, thread_id);
		gdb_connection->output_flag = GDB_OUTPUT_ALL;
		target_call_event_callbacks(ct, TARGET_EVENT_GDB_START);

		/*
		 * work around an annoying gdb behaviour: when the current thread
		 * is changed in gdb, it assumes that the target can follow and also
		 * make the thread current. This is an assumption that cannot hold
		 * for a real target running a multi-threading OS. We just fake
		 * the step to not trigger an internal error in gdb. See
		 * https://sourceware.org/bugzilla/show_bug.cgi?id=22925 for details
		 */
		if (fake_step) {
			int sig_reply_len;
			char sig_reply[128];

			LOG_DEBUG("fake step thread %"PRIx64, thread_id);

			sig_reply_len = snprintf(sig_reply, sizeof(sig_reply),
									"T05thread:%016"PRIx64";", thread_id);

			gdb_put_packet(connection, sig_reply, sig_reply_len);
			gdb_connection->output_flag = GDB_OUTPUT_NO;

			return true;
		}

		/* support for gdb_sync command */
		if (gdb_connection->sync) {
			gdb_connection->sync = false;
			if (ct->state == TARGET_HALTED) {
				LOG_DEBUG("stepi ignored. GDB will now fetch the register state "
								"from the target.");
				gdb_sig_halted(connection);
				gdb_connection->output_flag = GDB_OUTPUT_NO;
			} else
				gdb_connection->frontend_state = TARGET_RUNNING;
			return true;
		}

		retval = target_step(ct, current_pc, 0, false);
		if (retval == ERROR_TARGET_NOT_HALTED)
			LOG_TARGET_INFO(ct, "target was not halted when step was requested");

		/* if step was successful send a reply back to gdb */
		if (retval == ERROR_OK) {
			retval = target_poll(ct);
			if (retval != ERROR_OK)
				LOG_TARGET_DEBUG(ct, "error polling target after successful step");
			/* send back signal information */
			gdb_signal_reply(ct, connection);
			/* stop forwarding log packets! */
			gdb_connection->output_flag = GDB_OUTPUT_NO;
		} else
			gdb_connection->frontend_state = TARGET_RUNNING;
		return true;
	}
	LOG_ERROR("Unknown vCont packet");
	return false;
}

static char *next_hex_encoded_field(const char **str, char sep)
{
	size_t hexlen;
	const char *hex = *str;
	if (hex[0] == '\0')
		return NULL;

	const char *end = strchr(hex, sep);
	if (!end)
		hexlen = strlen(hex);
	else
		hexlen = end - hex;
	*str = hex + hexlen + 1;

	if (hexlen % 2 != 0) {
		/* Malformed hex data */
		return NULL;
	}

	size_t count = hexlen / 2;
	char *decoded = malloc(count + 1);
	if (!decoded)
		return NULL;

	size_t converted = unhexify((void *)decoded, hex, count);
	if (converted != count) {
		free(decoded);
		return NULL;
	}

	decoded[count] = '\0';
	return decoded;
}

/* handle extended restart packet */
static void gdb_restart_inferior(struct connection *connection, const char *packet, int packet_size)
{
	struct gdb_connection *gdb_con = connection->priv;
	struct target *target = get_target_from_connection(connection);

	breakpoint_clear_target(target);
	watchpoint_clear_target(target);
	command_run_linef(connection->cmd_ctx, "ocd_gdb_restart %s",
			target_name(target));
	/* set connection as attached after reset */
	gdb_con->attached = true;
	/*  info rtos parts */
	gdb_thread_packet(connection, packet, packet_size);
}

static bool gdb_handle_vrun_packet(struct connection *connection, const char *packet, int packet_size)
{
	struct target *target = get_available_target_from_connection(connection);
	const char *parse = packet;

	/* Skip "vRun" */
	parse += 4;

	if (parse[0] != ';')
		return false;
	parse++;

	/* Skip first field "filename"; don't know what to do with it. */
	free(next_hex_encoded_field(&parse, ';'));

	char *cmdline = next_hex_encoded_field(&parse, ';');
	while (cmdline) {
		char *arg = next_hex_encoded_field(&parse, ';');
		if (!arg)
			break;
		char *new_cmdline = alloc_printf("%s %s", cmdline, arg);
		free(cmdline);
		free(arg);
		cmdline = new_cmdline;
	}

	if (cmdline) {
		if (target->semihosting) {
			LOG_INFO("GDB set inferior command line to '%s'", cmdline);
			free(target->semihosting->cmdline);
			target->semihosting->cmdline = cmdline;
		} else {
			LOG_INFO("GDB set inferior command line to '%s' but semihosting is unavailable", cmdline);
			free(cmdline);
		}
	}

	gdb_restart_inferior(connection, packet, packet_size);
	gdb_put_packet(connection, "S00", 3);
	return true;
}

static int gdb_v_packet(struct connection *connection,
		char const *packet, int packet_size)
{
	struct gdb_connection *gdb_connection = connection->priv;
	int result;

	struct target *target = get_available_target_from_connection(connection);

	if (strncmp(packet, "vCont", 5) == 0) {
		bool handled;

		packet += 5;
		packet_size -= 5;

		handled = gdb_handle_vcont_packet(connection, packet, packet_size);
		if (!handled)
			gdb_put_packet(connection, "", 0);

		return ERROR_OK;
	}

	if (strncmp(packet, "vRun", 4) == 0) {
		bool handled;

		handled = gdb_handle_vrun_packet(connection, packet, packet_size);
		if (!handled)
			gdb_put_packet(connection, "", 0);

		return ERROR_OK;
	}

	/* if flash programming disabled - send a empty reply */

	if (!gdb_flash_program) {
		gdb_put_packet(connection, "", 0);
		return ERROR_OK;
	}

	if (strncmp(packet, "vFlashErase:", 12) == 0) {
		target_addr_t addr;
		unsigned long length;

		char const *parse = packet + 12;
		if (*parse == '\0') {
			LOG_ERROR("incomplete vFlashErase packet received, dropping connection");
			return ERROR_SERVER_REMOTE_CLOSED;
		}

		addr = strtoull(parse, (char **)&parse, 16);

		if (*(parse++) != ',' || *parse == '\0') {
			LOG_ERROR("incomplete vFlashErase packet received, dropping connection");
			return ERROR_SERVER_REMOTE_CLOSED;
		}

		length = strtoul(parse, (char **)&parse, 16);

		if (*parse != '\0') {
			LOG_ERROR("incomplete vFlashErase packet received, dropping connection");
			return ERROR_SERVER_REMOTE_CLOSED;
		}

		/* assume all sectors need erasing - stops any problems
		 * when flash_write is called multiple times */
		flash_set_dirty();

		/* perform any target specific operations before the erase */
		target_call_event_callbacks(target,
			TARGET_EVENT_GDB_FLASH_ERASE_START);

		/* vFlashErase:addr,length messages require region start and
		 * end to be "block" aligned ... if padding is ever needed,
		 * GDB will have become dangerously confused.
		 */
		result = flash_erase_address_range(target, false, addr,
			length);

		/* perform any target specific operations after the erase */
		target_call_event_callbacks(target,
			TARGET_EVENT_GDB_FLASH_ERASE_END);

		/* perform erase */
		if (result != ERROR_OK) {
			/* GDB doesn't evaluate the actual error number returned,
			 * treat a failed erase as an I/O error
			 */
			gdb_send_error(connection, EIO);
			LOG_ERROR("flash_erase returned %i", result);
		} else
			gdb_put_packet(connection, "OK", 2);

		return ERROR_OK;
	}

	if (strncmp(packet, "vFlashWrite:", 12) == 0) {
		int retval;
		target_addr_t addr;
		unsigned long length;
		char const *parse = packet + 12;

		if (*parse == '\0') {
			LOG_ERROR("incomplete vFlashErase packet received, dropping connection");
			return ERROR_SERVER_REMOTE_CLOSED;
		}

		addr = strtoull(parse, (char **)&parse, 16);
		if (*(parse++) != ':') {
			LOG_ERROR("incomplete vFlashErase packet received, dropping connection");
			return ERROR_SERVER_REMOTE_CLOSED;
		}
		length = packet_size - (parse - packet);

		/* create a new image if there isn't already one */
		if (!gdb_connection->vflash_image) {
			gdb_connection->vflash_image = malloc(sizeof(struct image));
			image_open(gdb_connection->vflash_image, "", "build");
		}

		/* create new section with content from packet buffer */
		retval = image_add_section(gdb_connection->vflash_image,
				addr, length, 0x0, (uint8_t const *)parse);
		if (retval != ERROR_OK)
			return retval;

		gdb_put_packet(connection, "OK", 2);

		return ERROR_OK;
	}

	if (strncmp(packet, "vFlashDone", 10) == 0) {
		uint32_t written;

		/* GDB command 'flash-erase' does not send a vFlashWrite,
		 * so nothing to write here. */
		if (!gdb_connection->vflash_image) {
			gdb_put_packet(connection, "OK", 2);
			return ERROR_OK;
		}

		/* process the flashing buffer. No need to erase as GDB
		 * always issues a vFlashErase first. */
		target_call_event_callbacks(target,
				TARGET_EVENT_GDB_FLASH_WRITE_START);
		result = flash_write(target, gdb_connection->vflash_image,
			&written, false);
		target_call_event_callbacks(target,
			TARGET_EVENT_GDB_FLASH_WRITE_END);
		if (result != ERROR_OK) {
			if (result == ERROR_FLASH_DST_OUT_OF_BANK)
				gdb_put_packet(connection, "E.memtype", 9);
			else
				gdb_send_error(connection, EIO);
		} else {
			LOG_DEBUG("wrote %u bytes from vFlash image to flash", (unsigned)written);
			gdb_put_packet(connection, "OK", 2);
		}

		image_close(gdb_connection->vflash_image);
		free(gdb_connection->vflash_image);
		gdb_connection->vflash_image = NULL;

		return ERROR_OK;
	}

	gdb_put_packet(connection, "", 0);
	return ERROR_OK;
}

static int gdb_detach(struct connection *connection)
{
	/*
	 * Only reply "OK" to GDB
	 * it will close the connection and this will trigger a call to
	 * gdb_connection_closed() that will in turn trigger the event
	 * TARGET_EVENT_GDB_DETACH
	 */
	return gdb_put_packet(connection, "OK", 2);
}

/* The format of 'F' response packet is
 * Fretcode,errno,Ctrl-C flag;call-specific attachment
 */
static int gdb_fileio_response_packet(struct connection *connection,
		char const *packet, int packet_size)
{
	struct target *target = get_available_target_from_connection(connection);
	char *separator;
	char *parsing_point;
	int fileio_retcode = strtoul(packet + 1, &separator, 16);
	int fileio_errno = 0;
	bool fileio_ctrl_c = false;
	int retval;

	LOG_DEBUG("-");

	if (*separator == ',') {
		parsing_point = separator + 1;
		fileio_errno = strtoul(parsing_point, &separator, 16);
		if (*separator == ',') {
			if (*(separator + 1) == 'C') {
				/* TODO: process ctrl-c */
				fileio_ctrl_c = true;
			}
		}
	}

	LOG_DEBUG("File-I/O response, retcode: 0x%x, errno: 0x%x, ctrl-c: %s",
			fileio_retcode, fileio_errno, fileio_ctrl_c ? "true" : "false");

	retval = target_gdb_fileio_end(target, fileio_retcode, fileio_errno, fileio_ctrl_c);
	if (retval != ERROR_OK)
		return ERROR_FAIL;

	/* After File-I/O ends, keep continue or step */
	if (gdb_running_type == 'c')
		retval = target_resume(target, true, 0x0, false, false);
	else if (gdb_running_type == 's')
		retval = target_step(target, true, 0x0, false);
	else
		retval = ERROR_FAIL;

	if (retval != ERROR_OK)
		return ERROR_FAIL;

	return ERROR_OK;
}

static void gdb_log_callback(void *priv, const char *file, unsigned int line,
		const char *function, const char *string)
{
	struct connection *connection = priv;
	struct gdb_connection *gdb_con = connection->priv;

	if (gdb_con->output_flag != GDB_OUTPUT_ALL)
		/* No out allowed */
		return;

	if (gdb_con->busy) {
		/* do not reply this using the O packet */
		return;
	}

	gdb_output_con(connection, string);
}

static void gdb_sig_halted(struct connection *connection)
{
	char sig_reply[4];
	snprintf(sig_reply, 4, "T%2.2x", 2);
	gdb_put_packet(connection, sig_reply, 3);
}

static int gdb_input_inner(struct connection *connection)
{
	/* Do not allocate this on the stack */
	static char gdb_packet_buffer[GDB_BUFFER_SIZE + 1]; /* Extra byte for null-termination */

	struct target *target;
	char const *packet = gdb_packet_buffer;
	int packet_size;
	int retval;
	struct gdb_connection *gdb_con = connection->priv;
	static bool warn_use_ext;

	target = get_target_from_connection(connection);

	/* drain input buffer. If one of the packets fail, then an error
	 * packet is replied, if applicable.
	 *
	 * This loop will terminate and the error code is returned.
	 *
	 * The calling fn will check if this error is something that
	 * can be recovered from, or if the connection must be closed.
	 *
	 * If the error is recoverable, this fn is called again to
	 * drain the rest of the buffer.
	 */
	do {
		packet_size = GDB_BUFFER_SIZE;
		retval = gdb_get_packet(connection, gdb_packet_buffer, &packet_size);
		if (retval != ERROR_OK)
			return retval;

		/* terminate with zero */
		gdb_packet_buffer[packet_size] = '\0';

		if (packet_size > 0) {
			gdb_log_incoming_packet(connection, gdb_packet_buffer);

			retval = ERROR_OK;
			switch (packet[0]) {
				case 'T':	/* Is thread alive? */
					gdb_thread_packet(connection, packet, packet_size);
					break;
				case 'H':	/* Set current thread ( 'c' for step and continue,
							 * 'g' for all other operations ) */
					gdb_thread_packet(connection, packet, packet_size);
					break;
				case 'q':
				case 'Q':
					retval = gdb_thread_packet(connection, packet, packet_size);
					if (retval == GDB_THREAD_PACKET_NOT_CONSUMED)
						retval = gdb_query_packet(connection, packet, packet_size);
					break;
				case 'g':
					retval = gdb_get_registers_packet(connection, packet, packet_size);
					break;
				case 'G':
					retval = gdb_set_registers_packet(connection, packet, packet_size);
					break;
				case 'p':
					retval = gdb_get_register_packet(connection, packet, packet_size);
					break;
				case 'P':
					retval = gdb_set_register_packet(connection, packet, packet_size);
					break;
				case 'm':
					gdb_con->output_flag = GDB_OUTPUT_NOTIF;
					retval = gdb_read_memory_packet(connection, packet, packet_size);
					gdb_con->output_flag = GDB_OUTPUT_NO;
					break;
				case 'M':
					gdb_con->output_flag = GDB_OUTPUT_NOTIF;
					retval = gdb_write_memory_packet(connection, packet, packet_size);
					gdb_con->output_flag = GDB_OUTPUT_NO;
					break;
				case 'z':
				case 'Z':
					retval = gdb_breakpoint_watchpoint_packet(connection, packet, packet_size);
					break;
				case '?':
					gdb_last_signal_packet(connection, packet, packet_size);
					/* '?' is sent after the eventual '!' */
					if (!warn_use_ext && !gdb_con->extended_protocol) {
						warn_use_ext = true;
						LOG_WARNING("Prefer GDB command \"target extended-remote :%s\" instead of \"target remote :%s\"",
									connection->service->port, connection->service->port);
					}
					break;
				case 'c':
				case 's':
				{
					gdb_thread_packet(connection, packet, packet_size);
					gdb_con->output_flag = GDB_OUTPUT_ALL;

					if (gdb_con->mem_write_error) {
						LOG_ERROR("Memory write failure!");

						/* now that we have reported the memory write error,
						 * we can clear the condition */
						gdb_con->mem_write_error = false;
					}

					bool nostep = false;
					bool already_running = false;
					if (target->state == TARGET_RUNNING) {
						LOG_WARNING("WARNING! The target is already running. "
								"All changes GDB did to registers will be discarded! "
								"Waiting for target to halt.");
						already_running = true;
					} else if (target->state != TARGET_HALTED) {
						LOG_WARNING("The target is not in the halted nor running stated, "
								"stepi/continue ignored.");
						nostep = true;
					} else if ((packet[0] == 's') && gdb_con->sync) {
						/* Hmm..... when you issue a continue in GDB, then a "stepi" is
						 * sent by GDB first to OpenOCD, thus defeating the check to
						 * make only the single stepping have the sync feature...
						 */
						nostep = true;
						LOG_DEBUG("stepi ignored. GDB will now fetch the register state "
								"from the target.");
					}
					gdb_con->sync = false;

					if (!already_running && nostep) {
						/* Either the target isn't in the halted state, then we can't
						 * step/continue. This might be early setup, etc.
						 *
						 * Or we want to allow GDB to pick up a fresh set of
						 * register values without modifying the target state.
						 *
						 */
						gdb_sig_halted(connection);

						/* stop forwarding log packets! */
						gdb_con->output_flag = GDB_OUTPUT_NO;
					} else {
						/* We're running/stepping, in which case we can
						 * forward log output until the target is halted
						 */
						gdb_con->frontend_state = TARGET_RUNNING;
						target_call_event_callbacks(target, TARGET_EVENT_GDB_START);

						if (!already_running) {
							/* Here we don't want packet processing to stop even if this fails,
							 * so we use a local variable instead of retval. */
							retval = gdb_step_continue_packet(connection, packet, packet_size);
							if (retval != ERROR_OK) {
								/* we'll never receive a halted
								 * condition... issue a false one..
								 */
								gdb_frontend_halted(target, connection);
							}
						}
					}
				}
				break;
				case 'v':
					retval = gdb_v_packet(connection, packet, packet_size);
					break;
				case 'D':
					retval = gdb_detach(connection);
					break;
				case 'X':
					gdb_con->output_flag = GDB_OUTPUT_NOTIF;
					retval = gdb_write_memory_binary_packet(connection, packet, packet_size);
					gdb_con->output_flag = GDB_OUTPUT_NO;
					break;
				case 'k':
					if (gdb_con->extended_protocol) {
						gdb_con->attached = false;
						break;
					}
					gdb_put_packet(connection, "OK", 2);
					return ERROR_SERVER_REMOTE_CLOSED;
				case '!':
					/* handle extended remote protocol */
					gdb_con->extended_protocol = true;
					gdb_put_packet(connection, "OK", 2);
					break;
				case 'R':
					/* handle extended restart packet */
					gdb_restart_inferior(connection, packet, packet_size);
					break;

				case 'j':
					/* DEPRECATED */
					/* packet supported only by smp target i.e cortex_a.c*/
					/* handle smp packet replying coreid played to gbd */
					gdb_read_smp_packet(connection, packet, packet_size);
					break;

				case 'J':
					/* DEPRECATED */
					/* packet supported only by smp target i.e cortex_a.c */
					/* handle smp packet setting coreid to be played at next
					 * resume to gdb */
					gdb_write_smp_packet(connection, packet, packet_size);
					break;

				case 'F':
					/* File-I/O extension */
					/* After gdb uses host-side syscall to complete target file
					 * I/O, gdb sends host-side syscall return value to target
					 * by 'F' packet.
					 * The format of 'F' response packet is
					 * Fretcode,errno,Ctrl-C flag;call-specific attachment
					 */
					gdb_con->frontend_state = TARGET_RUNNING;
					gdb_con->output_flag = GDB_OUTPUT_ALL;
					gdb_fileio_response_packet(connection, packet, packet_size);
					break;

				default:
					/* ignore unknown packets */
					LOG_DEBUG("ignoring 0x%2.2x packet", packet[0]);
					gdb_put_packet(connection, "", 0);
					break;
			}

			/* if a packet handler returned an error, exit input loop */
			if (retval != ERROR_OK)
				return retval;
		}

		if (gdb_con->ctrl_c) {
			struct target *available_target = get_available_target_from_connection(connection);
			if (available_target->state == TARGET_RUNNING) {
				struct target *t = available_target;
				if (available_target->rtos)
					available_target->rtos->gdb_target_for_threadid(connection, target->rtos->current_threadid, &t);
				retval = target_halt(t);
				if (retval == ERROR_OK)
					retval = target_poll(t);
				if (retval != ERROR_OK)
					target_call_event_callbacks(target, TARGET_EVENT_GDB_HALT);
				gdb_con->ctrl_c = false;
			} else {
				LOG_TARGET_INFO(target, "Not running when halt was requested, stopping GDB. (state=%d)",
						target->state);
				target_call_event_callbacks(target, TARGET_EVENT_GDB_HALT);
			}
		}

	} while (gdb_con->buf_cnt > 0);

	return ERROR_OK;
}

static int gdb_input(struct connection *connection)
{
	int retval = gdb_input_inner(connection);
	struct gdb_connection *gdb_con = connection->priv;
	if (retval == ERROR_SERVER_REMOTE_CLOSED)
		return retval;

	/* logging does not propagate the error, yet can set the gdb_con->closed flag */
	if (gdb_con->closed)
		return ERROR_SERVER_REMOTE_CLOSED;

	/* we'll recover from any other errors(e.g. temporary timeouts, etc.) */
	return ERROR_OK;
}

/*
 * Send custom notification packet as keep-alive during memory read/write.
 *
 * From gdb 7.0 (released 2009-10-06) an unknown notification received during
 * memory read/write would be silently dropped.
 * Before gdb 7.0 any character, with exclusion of "+-$", would be considered
 * as junk and ignored.
 * In both cases the reception will reset the timeout counter in gdb, thus
 * working as a keep-alive.
 * Check putpkt_binary() and getpkt_sane() in gdb commit
 * 74531fed1f2d662debc2c209b8b3faddceb55960
 *
 * Enable remote debug in gdb with 'set debug remote 1' to either dump the junk
 * characters in gdb pre-7.0 and the notification from gdb 7.0.
 */
static void gdb_async_notif(struct connection *connection)
{
	static unsigned char count;
	unsigned char checksum = 0;
	char buf[22];

	int len = sprintf(buf, "%%oocd_keepalive:%2.2x", count++);
	for (int i = 1; i < len; i++)
		checksum += buf[i];
	len += sprintf(buf + len, "#%2.2x", checksum);

#ifdef _DEBUG_GDB_IO_
	LOG_DEBUG("sending packet '%s'", buf);
#endif

	gdb_write(connection, buf, len);
}

static void gdb_keep_client_alive(struct connection *connection)
{
	struct gdb_connection *gdb_con = connection->priv;

	if (!gdb_con)
		return;

	switch (gdb_con->output_flag) {
	case GDB_OUTPUT_NO:
		/* no need for keep-alive */
		break;
	case GDB_OUTPUT_NOTIF:
		/* send asynchronous notification */
		gdb_async_notif(connection);
		break;
	case GDB_OUTPUT_ALL:
		/* send an empty O packet */
		gdb_output_con(connection, "");
		break;
	default:
		break;
	}
}

static const struct service_driver gdb_service_driver = {
	.name = "gdb",
	.new_connection_during_keep_alive_handler = NULL,
	.new_connection_handler = gdb_new_connection,
	.input_handler = gdb_input,
	.connection_closed_handler = gdb_connection_closed,
	.keep_client_alive_handler = gdb_keep_client_alive,
};

static int gdb_target_start(struct target *target, const char *port)
{
	struct gdb_service *gdb_service;
	int ret;
	gdb_service = malloc(sizeof(struct gdb_service));

	if (!gdb_service)
		return -ENOMEM;

	LOG_TARGET_INFO(target, "starting gdb server on %s", port);

	gdb_service->target = target;
	gdb_service->core[0] = -1;
	gdb_service->core[1] = -1;
	target->gdb_service = gdb_service;

	ret = add_service(&gdb_service_driver, port, target->gdb_max_connections, gdb_service);
	/* initialize all targets gdb service with the same pointer */
	{
		struct target_list *head;
		foreach_smp_target(head, target->smp_targets) {
			struct target *curr = head->target;
			if (curr != target)
				curr->gdb_service = gdb_service;
		}
	}
	return ret;
}

static int gdb_target_add_one(struct target *target)
{
	/*  one gdb instance per smp list */
	if ((target->smp) && (target->gdb_service))
		return ERROR_OK;

	/* skip targets that cannot handle a gdb connections (e.g. mem_ap) */
	if (!target_supports_gdb_connection(target)) {
		LOG_TARGET_DEBUG(target, "skip gdb server");
		return ERROR_OK;
	}

	if (target->gdb_port_override) {
		if (strcmp(target->gdb_port_override, "disabled") == 0) {
			LOG_TARGET_INFO(target, "gdb port disabled");
			return ERROR_OK;
		}
		return gdb_target_start(target, target->gdb_port_override);
	}

	if (strcmp(gdb_port_next, "disabled") == 0) {
		LOG_TARGET_INFO(target, "gdb port disabled");
		return ERROR_OK;
	}

	int retval = gdb_target_start(target, gdb_port_next);
	if (retval == ERROR_OK) {
		/* save the port number so can be queried with
		 * $target_name cget -gdb-port
		 */
		target->gdb_port_override = strdup(gdb_port_next);

		long portnumber;
		/* If we can parse the port number
		 * then we increment the port number for the next target.
		 */
		char *end;
		portnumber = strtol(gdb_port_next, &end, 0);
		if (!*end) {
			if (parse_long(gdb_port_next, &portnumber) == ERROR_OK) {
				free(gdb_port_next);
				if (portnumber) {
					gdb_port_next = alloc_printf("%ld", portnumber+1);
				} else {
					/* Don't increment if gdb_port is 0, since we're just
					 * trying to allocate an unused port. */
					gdb_port_next = strdup("0");
				}
			}
		} else if (strcmp(gdb_port_next, "pipe") == 0) {
			free(gdb_port_next);
			gdb_port_next = strdup("disabled");
		}
	}
	return retval;
}

int gdb_target_add_all(struct target *target)
{
	if (!target) {
		LOG_WARNING("gdb services need one or more targets defined");
		return ERROR_OK;
	}

	while (target) {
		int retval = gdb_target_add_one(target);
		if (retval != ERROR_OK)
			return retval;

		target = target->next;
	}

	return ERROR_OK;
}

COMMAND_HANDLER(handle_gdb_sync_command)
{
	if (CMD_ARGC != 0)
		return ERROR_COMMAND_SYNTAX_ERROR;

	if (!current_gdb_connection) {
		command_print(CMD,
			"gdb sync command can only be run from within gdb using \"monitor gdb sync\"");
		return ERROR_FAIL;
	}

	current_gdb_connection->sync = true;

	return ERROR_OK;
}

COMMAND_HANDLER(handle_gdb_port_command)
{
	int retval = CALL_COMMAND_HANDLER(server_pipe_command, &gdb_port);
	if (retval == ERROR_OK) {
		free(gdb_port_next);
		gdb_port_next = strdup(gdb_port);
	}
	return retval;
}

COMMAND_HANDLER(handle_gdb_memory_map_command)
{
	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	COMMAND_PARSE_ENABLE(CMD_ARGV[0], gdb_use_memory_map);
	return ERROR_OK;
}

COMMAND_HANDLER(handle_gdb_flash_program_command)
{
	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	COMMAND_PARSE_ENABLE(CMD_ARGV[0], gdb_flash_program);
	return ERROR_OK;
}

COMMAND_HANDLER(handle_gdb_report_data_abort_command)
{
	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	COMMAND_PARSE_ENABLE(CMD_ARGV[0], gdb_report_data_abort);
	return ERROR_OK;
}

COMMAND_HANDLER(handle_gdb_report_register_access_error)
{
	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	COMMAND_PARSE_ENABLE(CMD_ARGV[0], gdb_report_register_access_error);
	return ERROR_OK;
}

COMMAND_HANDLER(handle_gdb_breakpoint_override_command)
{
	if (CMD_ARGC == 0) {
		/* nothing */
	} else if (CMD_ARGC == 1) {
		gdb_breakpoint_override = 1;
		if (strcmp(CMD_ARGV[0], "hard") == 0)
			gdb_breakpoint_override_type = BKPT_HARD;
		else if (strcmp(CMD_ARGV[0], "soft") == 0)
			gdb_breakpoint_override_type = BKPT_SOFT;
		else if (strcmp(CMD_ARGV[0], "disable") == 0)
			gdb_breakpoint_override = 0;
	} else
		return ERROR_COMMAND_SYNTAX_ERROR;
	if (gdb_breakpoint_override)
		LOG_USER("force %s breakpoints",
			(gdb_breakpoint_override_type == BKPT_HARD) ? "hard" : "soft");
	else
		LOG_USER("breakpoint type is not overridden");

	return ERROR_OK;
}

COMMAND_HANDLER(handle_gdb_target_description_command)
{
	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	COMMAND_PARSE_ENABLE(CMD_ARGV[0], gdb_use_target_description);
	return ERROR_OK;
}

COMMAND_HANDLER(handle_gdb_save_tdesc_command)
{
	char *tdesc;
	uint32_t tdesc_length;
	struct target *target = get_current_target(CMD_CTX);

	int retval = gdb_generate_target_description(target, &tdesc);
	if (retval != ERROR_OK) {
		LOG_ERROR("Unable to Generate Target Description");
		return ERROR_FAIL;
	}

	tdesc_length = strlen(tdesc);

	struct fileio *fileio;
	size_t size_written;

	char *tdesc_filename = alloc_printf("%s.xml", target_type_name(target));
	if (!tdesc_filename) {
		retval = ERROR_FAIL;
		goto out;
	}

	retval = fileio_open(&fileio, tdesc_filename, FILEIO_WRITE, FILEIO_TEXT);

	if (retval != ERROR_OK) {
		LOG_ERROR("Can't open %s for writing", tdesc_filename);
		goto out;
	}

	retval = fileio_write(fileio, tdesc_length, tdesc, &size_written);

	fileio_close(fileio);

	if (retval != ERROR_OK)
		LOG_ERROR("Error while writing the tdesc file");

out:
	free(tdesc_filename);
	free(tdesc);

	return retval;
}

static const struct command_registration gdb_subcommand_handlers[] = {
	{
		.name = "sync",
		.handler = handle_gdb_sync_command,
		.mode = COMMAND_ANY,
		.help = "next stepi will return immediately allowing "
			"GDB to fetch register state without affecting "
			"target state",
		.usage = ""
	},
	{
		.name = "port",
		.handler = handle_gdb_port_command,
		.mode = COMMAND_CONFIG,
		.help = "Normally gdb listens to a TCP/IP port. Each subsequent GDB "
			"server listens for the next port number after the "
			"base port number specified. "
			"No arguments reports GDB port. \"pipe\" means listen to stdin "
			"output to stdout, an integer is base port number, \"disabled\" disables "
			"port. Any other string is are interpreted as named pipe to listen to. "
			"Output pipe is the same name as input pipe, but with 'o' appended.",
		.usage = "[port_num]",
	},
	{
		.name = "memory_map",
		.handler = handle_gdb_memory_map_command,
		.mode = COMMAND_CONFIG,
		.help = "enable or disable memory map",
		.usage = "('enable'|'disable')"
	},
	{
		.name = "flash_program",
		.handler = handle_gdb_flash_program_command,
		.mode = COMMAND_CONFIG,
		.help = "enable or disable flash program",
		.usage = "('enable'|'disable')"
	},
	{
		.name = "report_data_abort",
		.handler = handle_gdb_report_data_abort_command,
		.mode = COMMAND_CONFIG,
		.help = "enable or disable reporting data aborts",
		.usage = "('enable'|'disable')"
	},
	{
		.name = "report_register_access_error",
		.handler = handle_gdb_report_register_access_error,
		.mode = COMMAND_CONFIG,
		.help = "enable or disable reporting register access errors",
		.usage = "('enable'|'disable')"
	},
	{
		.name = "breakpoint_override",
		.handler = handle_gdb_breakpoint_override_command,
		.mode = COMMAND_ANY,
		.help = "Display or specify type of breakpoint "
			"to be used by gdb 'break' commands.",
		.usage = "('hard'|'soft'|'disable')"
	},
	{
		.name = "target_description",
		.handler = handle_gdb_target_description_command,
		.mode = COMMAND_CONFIG,
		.help = "enable or disable target description",
		.usage = "('enable'|'disable')"
	},
	{
		.name = "save_tdesc",
		.handler = handle_gdb_save_tdesc_command,
		.mode = COMMAND_EXEC,
		.help = "Save the target description file",
		.usage = "",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration gdb_command_handlers[] = {
	{
		.name = "gdb",
		.mode = COMMAND_ANY,
		.help = "GDB commands",
		.chain = gdb_subcommand_handlers,
		.usage = "",
	},
	COMMAND_REGISTRATION_DONE
};

int gdb_register_commands(struct command_context *cmd_ctx)
{
	gdb_port = strdup("3333");
	gdb_port_next = strdup("3333");
	return register_commands(cmd_ctx, NULL, gdb_command_handlers);
}

void gdb_service_free(void)
{
	free(gdb_port);
	free(gdb_port_next);
}

int gdb_get_actual_connections(void)
{
	return gdb_actual_connections;
}
