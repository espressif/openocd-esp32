/*
 * JTAG to ESP remote driver
 *
 * Based on jtag_vpi driver, Copyright (C) 2013 Franck Jullien, <elec4fun@gmail.com>.
 * Copyright 2020 Espressif Systems (Shanghai) Co. Ltd.
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <jtag/interface.h>
#ifdef HAVE_ARPA_INET_H
#include <arpa/inet.h>
#endif

#ifndef _WIN32
#include <netinet/tcp.h>
#endif

#include "jtag_usb_common.h"
#include "libusb_common.h"

#define NO_TAP_SHIFT	0
#define TAP_SHIFT	1

#define SERVER_ADDRESS	"127.0.0.1"
#define SERVER_PORT	5555

#define	XFERT_MAX_SIZE		512

//For an USB interface
#define USB_VID 0x303A
#define USB_PID 0x1001
#define USB_CONFIGURATION 0
//todo: get this from descriptors?
#define USB_INTERFACE 2
#define USB_IN_EP 0x83
#define USB_OUT_EP 0x2

int server_port = SERVER_PORT;
char *server_address;

/*
 Note: this all possibly is quite efficient, as each JTAG command is sent as a separate TCP packet
 or USB bulk write. ToDo: queue up commands and flush when we need to receive something
*/
int sockfd;
struct sockaddr_in serv_addr;
struct jtag_libusb_device_handle *usb_device;

struct esp_remote_cmd {
	uint16_t reserved: 4;
	uint16_t ver: 4;
		#define ESP_REMOTE_CMD_VER_1	1

	uint16_t function: 8;
		#define ESP_REMOTE_CMD_RESET	1
		#define ESP_REMOTE_CMD_SCAN	2
		#define ESP_REMOTE_CMD_TMS_SEQ	3
	union {
		uint16_t function_specific;
		struct {
			uint16_t bits: 12;
				#define MAX_BITS 4095
			uint16_t read: 1;
			uint16_t flip_tms: 1;
			uint16_t reserved: 2;
		} scan;
		struct {
			uint16_t srst: 1;
			uint16_t trst: 1;
			uint16_t reserved: 14;
		} reset;
		struct {
			uint16_t bits: 12;
			uint16_t reserved: 4;
		} tms_seq;
	};
	uint32_t data[0];
};

#define ESP_REMOTE_CMD_DECL(var, func, data_len) \
	const size_t var ## _len = (sizeof(struct esp_remote_cmd) + data_len + 3) / 4; \
	uint32_t var ## _storage[var ## _len]; \
	memset(var ## _storage, 0, var ## _len); \
	struct esp_remote_cmd *var = (struct esp_remote_cmd *) var ## _storage; \
	var->ver = ESP_REMOTE_CMD_VER_1; \
	var->function = func

/* Returns the variable-length data size, in bytes, based on the command header */
static inline size_t cmd_data_len_bytes(const struct esp_remote_cmd *cmd) {
	if (cmd->function == ESP_REMOTE_CMD_SCAN)
		return DIV_ROUND_UP(cmd->scan.bits, 8);
	else if (cmd->function == ESP_REMOTE_CMD_TMS_SEQ)
		return DIV_ROUND_UP(cmd->tms_seq.bits, 8);
	else
		return 0;
}

static int jtag_esp_remote_send_cmd(struct esp_remote_cmd *cmd)
{
	size_t data_len = cmd_data_len_bytes(cmd);
	if (usb_device) {
		size_t n=jtag_libusb_bulk_write(usb_device, USB_OUT_EP, (char*)cmd, sizeof(struct esp_remote_cmd) + data_len, 1000 /*ms*/); 
		if (n!=sizeof(struct esp_remote_cmd) + data_len) {
			LOG_ERROR("jtag_esp_remote: usb sent only %d out of %d bytes.", (int)n, (int)(sizeof(struct esp_remote_cmd) + data_len));
			return ERROR_FAIL;
		}
	} else {
		int retval = write_socket(sockfd, cmd, sizeof(struct esp_remote_cmd) + data_len);
		if (retval <= 0) return ERROR_FAIL;
	}
	return ERROR_OK;
}

static int jtag_esp_remote_receive_cmd(struct esp_remote_cmd *cmd)
{
#if 0
	/* replies don't include the header. do we need headers in replies? */
	int retval = read_socket(sockfd, cmd, sizeof(struct esp_remote_cmd));
	if (retval < (int)sizeof(struct esp_remote_cmd))
		return ERROR_FAIL;
#endif

	size_t data_len = cmd_data_len_bytes(cmd);
	if (usb_device) {
		if (data_len!=0) {
			size_t n=jtag_libusb_bulk_read(usb_device, USB_IN_EP, (char*)cmd->data, data_len, 1000 /*ms*/); 
			if (n!=data_len) {
				LOG_ERROR("jtag_esp_remote: usb received only %d out of %d bytes.", (int)n, (int)(data_len));
				return ERROR_FAIL;
			}
		}
	} else {
		read_socket(sockfd, cmd->data, data_len);
	}
	return ERROR_OK;
}

/**
 * jtag_esp_remote_reset - ask to reset the JTAG device
 * @trst: 1 if TRST is to be asserted
 * @srst: 1 if SRST is to be asserted
 */
static int jtag_esp_remote_reset(int trst, int srst)
{
	ESP_REMOTE_CMD_DECL(cmd, ESP_REMOTE_CMD_RESET, 0);
	cmd->reset.srst = srst;
	cmd->reset.trst = trst;

	return jtag_esp_remote_send_cmd(cmd);
}

/**
 * jtag_esp_remote_tms_seq - ask a TMS sequence transition to JTAG
 * @bits: TMS bits to be written (bit0, bit1 .. bitN)
 * @nb_bits: number of TMS bits (between 1 and 8)
 *
 * Write a series of TMS transitions, where each transition consists in :
 *  - writing out TCK=0, TMS=<new_state>, TDI=<???>
 *  - writing out TCK=1, TMS=<new_state>, TDI=<???> which triggers the transition
 * The function ensures that at the end of the sequence, the clock (TCK) is put
 * low.
 */
static int jtag_esp_remote_tms_seq(const uint8_t *bits, int nb_bits)
{
	if (nb_bits > MAX_BITS) {
		LOG_ERROR("%s: nb_bits too large: %d, max %d", __func__, nb_bits, MAX_BITS);
		return ERROR_FAIL;
	}
	int nb_bytes = DIV_ROUND_UP(nb_bits, 8);

	ESP_REMOTE_CMD_DECL(cmd, ESP_REMOTE_CMD_TMS_SEQ, nb_bytes);

	memcpy(cmd->data, bits, nb_bytes);
	cmd->tms_seq.bits = nb_bits;

	return jtag_esp_remote_send_cmd(cmd);
}

/**
 * jtag_esp_remote_path_move - ask a TMS sequence transition to JTAG
 * @cmd: path transition
 *
 * Write a serie of TMS transitions, where each transition consists in :
 *  - writing out TCK=0, TMS=<new_state>, TDI=<???>
 *  - writing out TCK=1, TMS=<new_state>, TDI=<???> which triggers the transition
 * The function ensures that at the end of the sequence, the clock (TCK) is put
 * low.
 */

static int jtag_esp_remote_path_move(struct pathmove_command *cmd)
{
	int cb = DIV_ROUND_UP(cmd->num_states, 8);
	uint8_t trans[cb];
	memset(trans, 0, cb);

	for (int i = 0; i < cmd->num_states; i++) {
		if (tap_state_transition(tap_get_state(), true) == cmd->path[i])
			buf_set_u32(trans, i, 1, 1);
		tap_set_state(cmd->path[i]);
	}

	return jtag_esp_remote_tms_seq(trans, cmd->num_states);
}

/**
 * jtag_esp_remote_tms - ask a tms command
 * @cmd: tms command
 */
static int jtag_esp_remote_tms(struct tms_command *cmd)
{
	return jtag_esp_remote_tms_seq(cmd->bits, cmd->num_bits);
}

static int jtag_esp_remote_state_move(tap_state_t state)
{
	if (tap_get_state() == state)
		return ERROR_OK;

	uint8_t tms_scan = tap_get_tms_path(tap_get_state(), state);
	int tms_len = tap_get_tms_path_len(tap_get_state(), state);

	int retval = jtag_esp_remote_tms_seq(&tms_scan, tms_len);
	if (retval != ERROR_OK)
		return retval;

	tap_set_state(state);

	return ERROR_OK;
}

static int s_read_bits_queued;

static int jtag_esp_remote_queue_tdi_xfer(uint8_t *bits, int nb_bits, int tap_shift, bool need_read)
{
	if (nb_bits > MAX_BITS) {
		LOG_ERROR("%s: nb_bits too large: %d, max %d", __func__, nb_bits, MAX_BITS);
		return ERROR_FAIL;
	}

	int nb_bytes = DIV_ROUND_UP(nb_bits, 8);
	ESP_REMOTE_CMD_DECL(cmd, ESP_REMOTE_CMD_SCAN, nb_bytes);

	if (tap_shift)
		cmd->scan.flip_tms = 1;

	if (need_read) {
		cmd->scan.read = 1;
		s_read_bits_queued += nb_bits;
	}

	if (bits)
		memcpy(cmd->data, bits, nb_bytes);
	else
		memset(cmd->data, 0xff, nb_bytes);

	cmd->scan.bits = nb_bits;

	cmd->scan.read = 1; //always on for now

	int retval = jtag_esp_remote_send_cmd(cmd);
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}

static int jtag_esp_remote_get_tdi_xfer_result(uint8_t *bits, int nb_bits)
{
	int nb_bytes = DIV_ROUND_UP(nb_bits, 8);
	ESP_REMOTE_CMD_DECL(cmd, ESP_REMOTE_CMD_SCAN, nb_bytes);
	cmd->scan.bits = nb_bits;
	memset(cmd->data, 0xcc, nb_bytes);

	int retval = jtag_esp_remote_receive_cmd(cmd);
	if (retval != ERROR_OK)
		return retval;

	if (bits)
		memcpy(bits, cmd->data, nb_bytes);

	assert (s_read_bits_queued >= nb_bits);
	s_read_bits_queued -= nb_bits;

	return ERROR_OK;
}

/**
 * jtag_esp_remote_queue_tdi - short description
 * @bits: bits to be queued on TDI (or NULL if 0 are to be queued)
 * @nb_bits: number of bits
 */
static int jtag_esp_remote_queue_tdi(uint8_t *bits, int nb_bits, int tap_shift, bool need_read)
{
	int nb_xfer = DIV_ROUND_UP(nb_bits, XFERT_MAX_SIZE * 8);
	int retval;

	while (nb_xfer) {
		if (nb_xfer ==  1) {
			retval = jtag_esp_remote_queue_tdi_xfer(bits, nb_bits, tap_shift, need_read);
			if (retval != ERROR_OK)
				return retval;
		} else {
			retval = jtag_esp_remote_queue_tdi_xfer(bits, XFERT_MAX_SIZE * 8, NO_TAP_SHIFT, need_read);
			if (retval != ERROR_OK)
				return retval;
			nb_bits -= XFERT_MAX_SIZE * 8;
			if (bits)
				bits += XFERT_MAX_SIZE;
		}

		nb_xfer--;
	}

	return ERROR_OK;
}

/**
 * jtag_esp_remote_clock_tms - clock a TMS transition
 * @tms: the TMS to be sent
 *
 * Triggers a TMS transition (ie. one JTAG TAP state move).
 */
static int jtag_esp_remote_clock_tms(int tms)
{
	const uint8_t tms_0 = 0;
	const uint8_t tms_1 = 1;

	return jtag_esp_remote_tms_seq(tms ? &tms_1 : &tms_0, 1);
}

/**
 * jtag_esp_remote_scan - launches a DR-scan or IR-scan
 * @cmd: the command to launch
 * out_read_size: output, number of bits to be read by this command
 *
 * Launch a JTAG IR-scan or DR-scan
 *
 * Returns ERROR_OK if OK, ERROR_xxx if a read/write error occurred.
 */
static int jtag_esp_remote_scan(struct scan_command *cmd, size_t *out_read_size)
{
	int scan_bits;
	uint8_t *buf = NULL;
	int retval = ERROR_OK;

	scan_bits = jtag_build_buffer(cmd, &buf);

	*out_read_size = 0;
	for (int i = 0; i < cmd->num_fields; ++i) {
		struct scan_field *sf = cmd->fields + i;
		if (sf->in_value) {
			*out_read_size += sf->num_bits;
		}
	}
	bool need_read = (*out_read_size > 0);

	if (cmd->ir_scan) {
		retval = jtag_esp_remote_state_move(TAP_IRSHIFT);
		if (retval != ERROR_OK)
			return retval;
	} else {
		retval = jtag_esp_remote_state_move(TAP_DRSHIFT);
		if (retval != ERROR_OK)
			return retval;
	}

	if (cmd->end_state == TAP_DRSHIFT) {
		retval = jtag_esp_remote_queue_tdi(buf, scan_bits, NO_TAP_SHIFT, need_read);
		if (retval != ERROR_OK)
			return retval;
	} else {
		retval = jtag_esp_remote_queue_tdi(buf, scan_bits, TAP_SHIFT, need_read);
		if (retval != ERROR_OK)
			return retval;
	}

	if (cmd->end_state != TAP_DRSHIFT) {
		/*
		 * As our JTAG is in an unstable state (IREXIT1 or DREXIT1), move it
		 * forward to a stable IRPAUSE or DRPAUSE.
		 */
		retval = jtag_esp_remote_clock_tms(0);
		if (retval != ERROR_OK)
			return retval;

		if (cmd->ir_scan)
			tap_set_state(TAP_IRPAUSE);
		else
			tap_set_state(TAP_DRPAUSE);
	}

	if (buf)
		free(buf);

	if (cmd->end_state != TAP_DRSHIFT) {
		retval = jtag_esp_remote_state_move(cmd->end_state);
		if (retval != ERROR_OK)
			return retval;
	}

	return ERROR_OK;
}

static int jtag_esp_remote_scan_read(struct scan_command *cmd)
{
	int retval;
	uint8_t *buf = NULL;

	size_t read_size = 0;
	for (int i = 0; i < cmd->num_fields; ++i) {
		struct scan_field *sf = cmd->fields + i;
		if (sf->in_value) {
			read_size += sf->num_bits;
		}
	}
	if (read_size == 0)
		return ERROR_OK;

	int nbits = jtag_build_buffer(cmd, &buf);
	int nbytes = DIV_ROUND_UP(nbits, 8);
	memset(buf, 0xcc, nbytes);
	retval = jtag_esp_remote_get_tdi_xfer_result(buf, nbits);
	if (retval != ERROR_OK) {
		free(buf);
		return retval;
	}
	retval = jtag_read_buffer(buf, cmd);
	if (retval != ERROR_OK) {
		free(buf);
		return retval;
	}

	return ERROR_OK;
}

static int jtag_esp_remote_runtest(int cycles, tap_state_t state)
{
	int retval;

	retval = jtag_esp_remote_state_move(TAP_IDLE);
	if (retval != ERROR_OK)
		return retval;

	retval = jtag_esp_remote_queue_tdi(NULL, cycles, TAP_SHIFT, false);
	if (retval != ERROR_OK)
		return retval;

	return jtag_esp_remote_state_move(state);
}

static int jtag_esp_remote_stableclocks(int cycles)
{
	return jtag_esp_remote_queue_tdi(NULL, cycles, TAP_SHIFT, false);
}

static int jtag_esp_remote_execute_queue(void)
{
	struct jtag_command *cmd;
	int retval = ERROR_OK;
	size_t read_size = 0;
	size_t cmd_read_size;


	for (cmd = jtag_command_queue; retval == ERROR_OK && cmd != NULL;
		 cmd = cmd->next) {
		switch (cmd->type) {
		case JTAG_RESET:
			LOG_DEBUG("JTAG_RESET");
			retval = jtag_esp_remote_reset(cmd->cmd.reset->trst, cmd->cmd.reset->srst);
			break;
		case JTAG_RUNTEST:
			LOG_DEBUG("JTAG_RUNTEST");
			retval = jtag_esp_remote_runtest(cmd->cmd.runtest->num_cycles,
						cmd->cmd.runtest->end_state);
			break;
		case JTAG_STABLECLOCKS:
			LOG_DEBUG("JTAG_STABLECLOCKS");
			retval = jtag_esp_remote_stableclocks(cmd->cmd.stableclocks->num_cycles);
			break;
		case JTAG_TLR_RESET:
			LOG_DEBUG("JTAG_TLR_RESET");
			retval = jtag_esp_remote_state_move(cmd->cmd.statemove->end_state);
			break;
		case JTAG_PATHMOVE:
			LOG_DEBUG("JTAG_PATHMOVE");
			retval = jtag_esp_remote_path_move(cmd->cmd.pathmove);
			break;
		case JTAG_TMS:
			LOG_DEBUG("JTAG_TMS");
			retval = jtag_esp_remote_tms(cmd->cmd.tms);
			break;
		case JTAG_SLEEP:
			LOG_DEBUG("JTAG_SLEEP");
			jtag_sleep(cmd->cmd.sleep->us);
			break;
		case JTAG_SCAN:
			// LOG_DEBUG("JTAG_SCAN");
			retval = jtag_esp_remote_scan(cmd->cmd.scan, &cmd_read_size);
			read_size += cmd_read_size;
			break;
		}
	}

	if (read_size > 0) {
			for (cmd = jtag_command_queue; retval == ERROR_OK && cmd != NULL;
				 cmd = cmd->next) {
				if (cmd->type == JTAG_SCAN) {
					jtag_esp_remote_scan_read(cmd->cmd.scan);
				}
			}
	}
	assert(s_read_bits_queued == 0);

	return retval;
}
static int jtag_esp_remote_init_tcp(void)
{
	int flag = 1;

	sockfd = socket(AF_INET, SOCK_STREAM, 0);
	if (sockfd < 0) {
		LOG_ERROR("Could not create socket");
		return ERROR_FAIL;
	}

	memset(&serv_addr, 0, sizeof(serv_addr));

	serv_addr.sin_family = AF_INET;
	serv_addr.sin_port = htons(server_port);

	if (!server_address)
		server_address = strdup(SERVER_ADDRESS);

	serv_addr.sin_addr.s_addr = inet_addr(server_address);

	if (serv_addr.sin_addr.s_addr == INADDR_NONE) {
		LOG_ERROR("inet_addr error occurred");
		return ERROR_FAIL;
	}

	if (connect(sockfd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) {
		close(sockfd);
		LOG_ERROR("Can't connect to %s : %u", server_address, server_port);
		return ERROR_COMMAND_CLOSE_CONNECTION;
	}

	if (serv_addr.sin_addr.s_addr == htonl(INADDR_LOOPBACK)) {
		/* This increases performance dramatically for local
		 * connections, which is the most likely arrangement
		 * for a ESP remote connection. */
		setsockopt(sockfd, IPPROTO_TCP, TCP_NODELAY, (char *)&flag, sizeof(int));
	}

	LOG_INFO("Connection to %s : %u succeed", server_address, server_port);

	return ERROR_OK;
}

static int jtag_esp_remote_init_usb(void)
{
	const uint16_t vids[]={USB_VID};
	const uint16_t pids[]={USB_PID};
	int r=jtag_libusb_open(vids, pids, NULL, &usb_device);
	if (r!=ERROR_OK) {
		if (r==ERROR_FAIL) {
			return ERROR_JTAG_INVALID_INTERFACE; //we likely can't find the USB device
		} else {
			return r; //some other error
		}
	}

	jtag_libusb_set_configuration(usb_device, USB_CONFIGURATION);
	jtag_libusb_claim_interface(usb_device, USB_INTERFACE);
	return ERROR_OK;
}

static int jtag_esp_remote_init(void)
{
	if (!server_address || strcmp(server_address, "")==0) {
		int r=jtag_esp_remote_init_usb();
		//Note: if we succeed, usb_device is also non-NULL.
		if (r!=ERROR_JTAG_INVALID_INTERFACE) return r;
	}
	return jtag_esp_remote_init_tcp();
}

static int jtag_esp_remote_quit(void)
{
	if (usb_device) {
		jtag_libusb_close(usb_device);
	} else {
		free(server_address);
		return close(sockfd);
	}
	return ERROR_OK;
}

COMMAND_HANDLER(jtag_esp_remote_set_port)
{
	if (CMD_ARGC == 0)
		LOG_WARNING("You need to set a port number");
	else
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], server_port);

	LOG_INFO("Set server port to %u", server_port);

	return ERROR_OK;
}

COMMAND_HANDLER(jtag_esp_remote_set_address)
{
	free(server_address);

	if (CMD_ARGC == 0) {
		LOG_WARNING("You need to set an address");
		server_address = strdup(SERVER_ADDRESS);
	} else
		server_address = strdup(CMD_ARGV[0]);

	LOG_INFO("Set server address to %s", server_address);

	return ERROR_OK;
}

static const struct command_registration jtag_esp_remote_command_handlers[] = {
	{
		.name = "jtag_esp_remote_set_port",
		.handler = &jtag_esp_remote_set_port,
		.mode = COMMAND_CONFIG,
		.help = "set the port of the ESP remote server",
		.usage = "description_string",
	},
	{
		.name = "jtag_esp_remote_set_address",
		.handler = &jtag_esp_remote_set_address,
		.mode = COMMAND_CONFIG,
		.help = "set the address of the ESP remote server",
		.usage = "description_string",
	},
	COMMAND_REGISTRATION_DONE
};

struct jtag_interface jtag_esp_remote_interface = {
	.name = "jtag_esp_remote",
	.supported = DEBUG_CAP_TMS_SEQ,
	.commands = jtag_esp_remote_command_handlers,
	.transports = jtag_only,

	.init = jtag_esp_remote_init,
	.quit = jtag_esp_remote_quit,
	.execute_queue = jtag_esp_remote_execute_queue,
};
