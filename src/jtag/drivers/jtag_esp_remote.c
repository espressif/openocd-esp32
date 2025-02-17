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

#include <jtag/adapter.h>
#include <jtag/interface.h>
#ifdef HAVE_ARPA_INET_H
#include <arpa/inet.h>
#endif

#ifndef _WIN32
#include <netinet/tcp.h>
#endif

#include "libusb_helper.h"

#define NO_TAP_SHIFT    0
#define TAP_SHIFT       1

#define XFERT_MAX_SIZE          512

#define DEFAULT_SERVER_PORT     5555
#define DEFAULT_SERVER_ADDRESS  "127.0.0.1"

/*For an USB interface */
#define USB_CONFIGURATION 0
#define USB_INTERFACE 2
#define USB_IN_EP 0x83
#define USB_OUT_EP 0x3

struct esp_remote_cmd {
	uint16_t reserved : 4;
	uint16_t ver : 4;
		#define ESP_REMOTE_CMD_VER_1    1

	uint16_t function : 8;
		#define ESP_REMOTE_CMD_RESET    1
		#define ESP_REMOTE_CMD_SCAN     2
		#define ESP_REMOTE_CMD_TMS_SEQ  3
		#define ESP_REMOTE_CMD_SET_CLK  4
	union {
		uint16_t function_specific;
		struct {
			uint16_t bits : 12;
				#define MAX_BITS 4095
			uint16_t read : 1;
			uint16_t flip_tms : 1;
			uint16_t reserved : 2;
		} scan;
		struct {
			uint16_t srst : 1;
			uint16_t trst : 1;
			uint16_t reserved : 14;
		} reset;
		struct {
			uint16_t bits : 12;
			uint16_t reserved : 4;
		} tms_seq;
	};
	uint32_t data[0];
};

#define ESP_REMOTE_CMD_DECL(var, func, data_len) \
	const size_t var ## _len = (sizeof(struct esp_remote_cmd) + data_len + 3) / 4; \
	uint32_t var ## _storage[var ## _len]; \
	memset(var ## _storage, 0, var ## _len); \
	struct esp_remote_cmd *var = (struct esp_remote_cmd *)var ## _storage; \
	var->ver = ESP_REMOTE_CMD_VER_1; \
	var->function = func

typedef int (*jtag_esp_remote_send_cmd_t)(struct esp_remote_cmd *);
typedef int (*jtag_esp_remote_receive_cmd_t)(struct esp_remote_cmd *);

static jtag_esp_remote_send_cmd_t jtag_esp_remote_send_cmd = NULL;
static jtag_esp_remote_receive_cmd_t jtag_esp_remote_receive_cmd = NULL;

enum esp_remote_protocols {
	ESP_REMOTE_TCP,
	ESP_REMOTE_USB,
	ESP_REMOTE_UNKNOWN_PROT,
};

static int esp_remote_protocol = ESP_REMOTE_UNKNOWN_PROT;

static int server_port = 5555;
static char *server_address = NULL;
static int sockfd;
static struct sockaddr_in serv_addr;

static int usb_vid = 0;
static int usb_pid = 0;
static struct libusb_device_handle *usb_device = NULL;

static int s_read_bits_queued = 0;

/* Returns the variable-length data size, in bytes, based on the command header */
static inline size_t cmd_data_len_bytes(const struct esp_remote_cmd *cmd)
{
	if (cmd->function == ESP_REMOTE_CMD_SCAN)
		return DIV_ROUND_UP(cmd->scan.bits, 8);
	else if (cmd->function == ESP_REMOTE_CMD_TMS_SEQ)
		return DIV_ROUND_UP(cmd->tms_seq.bits, 8);
	else if (cmd->function == ESP_REMOTE_CMD_SET_CLK)
		return 4;
	else
		return 0;
}

static int jtag_esp_remote_send_cmd_tcp(struct esp_remote_cmd *cmd)
{
	const size_t data_len = cmd_data_len_bytes(cmd);
	const size_t size = sizeof(struct esp_remote_cmd) + data_len;
	const size_t retval = write_socket(sockfd, cmd, size);
	if (retval != size)
		return ERROR_FAIL;
	return ERROR_OK;
}

static int jtag_esp_remote_send_cmd_usb(struct esp_remote_cmd *cmd)
{
	const size_t data_len = cmd_data_len_bytes(cmd);
	const int size = sizeof(struct esp_remote_cmd) + data_len;
	int tr, ret = jtag_libusb_bulk_write(usb_device,
		USB_OUT_EP,
		(char *)cmd,
		size,
		1000 /*ms*/,
		&tr);
	if (ret != ERROR_OK)
		return ERROR_FAIL;
	if (tr != size) {
		LOG_ERROR("jtag_esp_remote: usb sent only %d out of %d bytes.",
			(int)tr,
			(int)size);
		return ERROR_FAIL;
	}
	return ERROR_OK;
}

static int jtag_esp_remote_receive_cmd_tcp(struct esp_remote_cmd *cmd)
{
	const size_t data_len = cmd_data_len_bytes(cmd);
	const size_t ret_val = read_socket(sockfd, cmd->data, data_len);
	if (ret_val != data_len)
		return ERROR_FAIL;
	return ERROR_OK;
}

static int jtag_esp_remote_receive_cmd_usb(struct esp_remote_cmd *cmd)
{
	size_t data_len = cmd_data_len_bytes(cmd);
	if (usb_device) {
		if (data_len != 0) {
			/* Need to keep an internal buffer because libusb can read the same amount
			 * of bytes sent together. */
			/* E.g. If 2 bytes were sent then it cannot read 1 byte. */
			static char internal_buffer[64];/* 64 is the size of VENDOR class USB buffer
							                                **/
			static size_t internal_buffer_occupied = 0;
			for (size_t i = 0; i < data_len; ) {
				if (internal_buffer_occupied > 0) {
					const size_t t =
						MIN(data_len - i, internal_buffer_occupied);
					memcpy(((char *)cmd->data) + i, internal_buffer, t);
					memmove(internal_buffer,
						internal_buffer + t,
						internal_buffer_occupied - t);
					i += t;
					internal_buffer_occupied -= t;

					if (i >= data_len)
						break;
				}
				assert(internal_buffer_occupied == 0);
				int tr, ret = jtag_libusb_bulk_read(usb_device,
					USB_IN_EP,
					internal_buffer,
					sizeof(internal_buffer),
					1000,	/*ms*/
					&tr);
				/* libusb will read groups of bytes which were send toghether and
				 * not everything in the receive buffer */
				if (ret != ERROR_OK || tr == 0) {
					LOG_ERROR("jtag_esp_remote: usb receive error");
					return ERROR_FAIL;
				}
				internal_buffer_occupied = tr;
			}
		}
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
 * jtag_esp_remote_tms_seq - send a TMS sequence transition to JTAG
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
 * jtag_esp_remote_path_move - send a TMS sequence transition to JTAG
 * @cmd: path transition
 *
 * Write a series of TMS transitions, where each transition consists of:
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

	for (unsigned int i = 0; i < cmd->num_states; i++) {
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

static int jtag_esp_remote_state_move(enum tap_state state)
{
	enum tap_state cur_state = tap_get_state();
	if (cur_state == state)
		return ERROR_OK;

	uint8_t tms_scan = tap_get_tms_path(cur_state, state);
	int tms_len = tap_get_tms_path_len(cur_state, state);

	assert(((unsigned)tms_len) <= sizeof(tms_scan) * 8);

	int retval = jtag_esp_remote_tms_seq(&tms_scan, tms_len);
	if (retval != ERROR_OK)
		return retval;

	tap_set_state(state);

	return ERROR_OK;
}

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
	else
		cmd->scan.flip_tms = 0;

	if (need_read) {
		cmd->scan.read = 1;
		s_read_bits_queued += nb_bits;
	} else {
		cmd->scan.read = 0;
	}

	if (bits)
		memcpy(cmd->data, bits, nb_bytes);
	else
		memset(cmd->data, 0xff, nb_bytes);

	cmd->scan.bits = nb_bits;

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

	assert(s_read_bits_queued >= nb_bits);
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
		if (nb_xfer == 1) {
			retval =
				jtag_esp_remote_queue_tdi_xfer(bits, nb_bits, tap_shift, need_read);
			if (retval != ERROR_OK)
				return retval;
		} else {
			retval = jtag_esp_remote_queue_tdi_xfer(bits,
				XFERT_MAX_SIZE * 8,
				NO_TAP_SHIFT,
				need_read);
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
	for (unsigned int i = 0; i < cmd->num_fields; ++i) {
		struct scan_field *sf = cmd->fields + i;
		if (sf->in_value)
			*out_read_size += sf->num_bits;
	}
	bool need_read = (*out_read_size > 0);

	retval = jtag_esp_remote_state_move(cmd->ir_scan ? TAP_IRSHIFT : TAP_DRSHIFT);
	if (retval != ERROR_OK)
		goto jtag_esp_remote_scan_exit;

	retval = jtag_esp_remote_queue_tdi(buf, scan_bits, TAP_SHIFT, need_read);
	if (retval != ERROR_OK)
		goto jtag_esp_remote_scan_exit;

	/* TAP_SHIFT moved the state into IREXIT/DREXIT. Another TMS=0 will move it to
	 * IRPAUSE/DRPAUSE */

	retval = jtag_esp_remote_clock_tms(0);
	if (retval != ERROR_OK)
		goto jtag_esp_remote_scan_exit;

	tap_set_state(cmd->ir_scan ? TAP_IRPAUSE : TAP_DRPAUSE);

	retval = jtag_esp_remote_state_move(cmd->end_state);

jtag_esp_remote_scan_exit:
	if (buf)
		free(buf);

	return retval;
}

static int jtag_esp_remote_scan_read(struct scan_command *cmd)
{
	int retval = ERROR_OK;
	uint8_t *buf = NULL;

	size_t read_size = 0;
	for (unsigned int i = 0; i < cmd->num_fields; ++i) {
		struct scan_field *sf = cmd->fields + i;
		if (sf->in_value)
			read_size += sf->num_bits;
	}
	if (read_size == 0)
		return retval;

	int nbits = jtag_build_buffer(cmd, &buf);
	int nbytes = DIV_ROUND_UP(nbits, 8);
	memset(buf, 0xcc, nbytes);
	retval = jtag_esp_remote_get_tdi_xfer_result(buf, nbits);
	if (retval != ERROR_OK)
		goto jtag_esp_remote_scan_read_exit;
	retval = jtag_read_buffer(buf, cmd);

jtag_esp_remote_scan_read_exit:
	free(buf);
	return retval;
}

static int jtag_esp_remote_runtest(int cycles, enum tap_state end_state)
{
	int retval;

	retval = jtag_esp_remote_state_move(TAP_IDLE);
	if (retval != ERROR_OK)
		return retval;

	for (int i = 0; i < cycles; i++)
		jtag_esp_remote_clock_tms(0);

	return jtag_esp_remote_state_move(end_state);
}

static int jtag_esp_remote_stableclocks(int cycles)
{
	uint8_t tms_bits[4];
	int cycles_remain = cycles;
	int nb_bits;
	int retval;
	const int cycles_one_batch = sizeof(tms_bits) * 8;

	assert(cycles >= 0);

	/* use TMS=1 in TAP RESET state, TMS=0 in all other stable states */
	memset(tms_bits, (tap_get_state() == TAP_RESET) ? 0xff : 0x00, sizeof(tms_bits));

	/* send the TMS bits */
	while (cycles_remain > 0) {
		nb_bits = (cycles_remain < cycles_one_batch) ? cycles_remain : cycles_one_batch;
		retval = jtag_esp_remote_tms_seq(tms_bits, nb_bits);
		if (retval != ERROR_OK)
			return retval;
		cycles_remain -= nb_bits;
	}

	return ERROR_OK;
}

static int jtag_esp_remote_execute_queue(struct jtag_command *cmd_queue)
{
	struct jtag_command *cmd;
	int retval = ERROR_OK;
	size_t read_size = 0;
	size_t cmd_read_size;

	for (cmd = cmd_queue; retval == ERROR_OK && cmd; cmd = cmd->next) {
		switch (cmd->type) {
		case JTAG_RESET:
			retval = jtag_esp_remote_reset(cmd->cmd.reset->trst,
				cmd->cmd.reset->srst);
			break;
		case JTAG_RUNTEST:
			retval = jtag_esp_remote_runtest(cmd->cmd.runtest->num_cycles,
				cmd->cmd.runtest->end_state);
			break;
		case JTAG_STABLECLOCKS:
			retval = jtag_esp_remote_stableclocks(
				cmd->cmd.stableclocks->num_cycles);
			break;
		case JTAG_TLR_RESET:
			retval = jtag_esp_remote_state_move(cmd->cmd.statemove->end_state);
			break;
		case JTAG_PATHMOVE:
			retval = jtag_esp_remote_path_move(cmd->cmd.pathmove);
			break;
		case JTAG_TMS:
			retval = jtag_esp_remote_tms(cmd->cmd.tms);
			break;
		case JTAG_SLEEP:
			jtag_sleep(cmd->cmd.sleep->us);
			break;
		case JTAG_SCAN:
			retval = jtag_esp_remote_scan(cmd->cmd.scan, &cmd_read_size);
			read_size += cmd_read_size;
			break;
		default:
			LOG_ERROR("Unknown JTAG command type 0x%X", cmd->type);
			retval = ERROR_FAIL;
			break;
		}
	}

	if (read_size > 0) {
		for (cmd = cmd_queue; retval == ERROR_OK && cmd; cmd = cmd->next) {
			if (cmd->type == JTAG_SCAN)
				retval = jtag_esp_remote_scan_read(cmd->cmd.scan);
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
	const uint16_t vids[] = { usb_vid, 0 };		/* must be null terminated */
	const uint16_t pids[] = { usb_pid, 0 };		/* must be null terminated */
	int r = jtag_libusb_open(vids, pids, NULL, &usb_device, NULL);
	if (r != ERROR_OK) {
		if (r == ERROR_FAIL)
			return ERROR_JTAG_INVALID_INTERFACE;	/*we likely can't find the USB
								                                        * device */
		else
			return r;	/*some other error */
	}

	jtag_libusb_set_configuration(usb_device, USB_CONFIGURATION);
	if (libusb_kernel_driver_active(usb_device, USB_INTERFACE))
		libusb_detach_kernel_driver(usb_device, USB_INTERFACE);
	libusb_claim_interface(usb_device, USB_INTERFACE);
	return ERROR_OK;
}

static int jtag_esp_remote_init(void)
{
	if (esp_remote_protocol == ESP_REMOTE_USB) {
		int r = jtag_esp_remote_init_usb();
		/*Note: if we succeed, usb_device is also non-NULL. */
		if (r != ERROR_JTAG_INVALID_INTERFACE)
			return r;
	} else if (esp_remote_protocol == ESP_REMOTE_TCP) {
		return jtag_esp_remote_init_tcp();
	}
	return ERROR_FAIL;
}

static int jtag_esp_remote_quit(void)
{
	if (usb_device) {
		libusb_release_interface(usb_device, USB_INTERFACE);
		jtag_libusb_close(usb_device);
	} else {
		if (server_address) {
			free(server_address);
			server_address = NULL;
		}
		return close(sockfd);
	}
	return ERROR_OK;
}

static int jtag_esp_remote_jtag_speed_div(int speed, int *khz)
{
	*khz = speed / 1000;

	return ERROR_OK;
}

static int jtag_esp_remote_jtag_khz(int khz, int *speed)
{
	if (khz == 0) {
		LOG_ERROR("jtag_esp_remote: Adaptive clocking is not supported.");
		return ERROR_JTAG_NOT_IMPLEMENTED;
	}

	*speed = khz * 1000;

	return ERROR_OK;
}

static int jtag_esp_remote_jtag_speed(int speed)
{
	if (speed == 0) {
		LOG_ERROR("jtag_esp_remote: Adaptive clocking is not supported.");
		return ERROR_JTAG_NOT_IMPLEMENTED;
	}

	uint8_t speed_buff[4] = {
		((uint32_t)speed >> 24) & 0xFF,
		((uint32_t)speed >> 16) & 0xFF,
		((uint32_t)speed >> 8) & 0xFF,
		((uint32_t)speed >> 0) & 0xFF
	};

	ESP_REMOTE_CMD_DECL(cmd, ESP_REMOTE_CMD_SET_CLK, sizeof(speed_buff));

	memcpy(cmd->data, &speed_buff, sizeof(speed_buff));

	return jtag_esp_remote_send_cmd(cmd);
}

COMMAND_HANDLER(jtag_esp_remote_protocol)
{
	if (CMD_ARGC > 0) {
		if (strcmp(CMD_ARGV[0], "usb") == 0) {
			esp_remote_protocol = ESP_REMOTE_USB;
			jtag_esp_remote_send_cmd = jtag_esp_remote_send_cmd_usb;
			jtag_esp_remote_receive_cmd = jtag_esp_remote_receive_cmd_usb;
			LOG_INFO("USB protocol set for esp remote");
			return ERROR_OK;
		}
		if (strcmp(CMD_ARGV[0], "tcp") == 0) {
			esp_remote_protocol = ESP_REMOTE_TCP;
			jtag_esp_remote_send_cmd = jtag_esp_remote_send_cmd_tcp;
			jtag_esp_remote_receive_cmd = jtag_esp_remote_receive_cmd_tcp;
			if (!server_address)
				server_address = strdup(DEFAULT_SERVER_ADDRESS);
			LOG_INFO("TCP protocol set for esp remote");
			return ERROR_OK;
		}
	}
	LOG_ERROR("You need to set an esp remote protocol (tcp or usb)");
	esp_remote_protocol = ESP_REMOTE_UNKNOWN_PROT;
	return ERROR_FAIL;
}

COMMAND_HANDLER(jtag_esp_remote_vid_pid)
{
	if (esp_remote_protocol != ESP_REMOTE_USB) {
		LOG_ERROR("USB protocol must be set up for jtag_esp_remote_vid_pid");
		return ERROR_FAIL;
	}

	if (CMD_ARGC < 2) {
		LOG_ERROR("You need to supply the vendor and product IDs");
		return ERROR_FAIL;
	}

	COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], usb_vid);
	COMMAND_PARSE_NUMBER(int, CMD_ARGV[1], usb_pid);
	LOG_INFO("VID set to 0x%x and PID to 0x%x", usb_vid, usb_pid);
	return ERROR_OK;
}

COMMAND_HANDLER(jtag_esp_remote_set_port)
{
	if (esp_remote_protocol != ESP_REMOTE_TCP) {
		LOG_ERROR("TCP protocol must be set up for jtag_esp_remote_set_port");
		return ERROR_FAIL;
	}

	if (CMD_ARGC == 0) {
		LOG_ERROR("You need to set a port number");
		return ERROR_FAIL;
	}

	COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], server_port);
	LOG_INFO("Set server port to %u", server_port);
	return ERROR_OK;
}

COMMAND_HANDLER(jtag_esp_remote_set_address)
{
	if (esp_remote_protocol != ESP_REMOTE_TCP) {
		LOG_ERROR("TCP protocol must be set up for jtag_esp_remote_set_address");
		return ERROR_FAIL;
	}

	if (server_address) {
		free(server_address);
		server_address = NULL;
	}

	if (CMD_ARGC == 0) {
		LOG_ERROR("You need to set an address");
		return ERROR_FAIL;
	}

	server_address = strdup(CMD_ARGV[0]);
	LOG_INFO("Set server address to %s", server_address);
	return ERROR_OK;
}

static const struct command_registration jtag_esp_remote_command_handlers[] = {
	{
		.name = "jtag_esp_remote_protocol",
		.handler = &jtag_esp_remote_protocol,
		.mode = COMMAND_CONFIG,
		.help = "set communication protocol for ESP remote driver (tcp or usb)",
		.usage = "description_string",
	},
	{
		.name = "jtag_esp_remote_vid_pid",
		.handler = &jtag_esp_remote_vid_pid,
		.mode = COMMAND_CONFIG,
		.help = "set vendor ID and product ID for ESP remote driver over USB",
		.usage = "description_string",
	},
	{
		.name = "jtag_esp_remote_set_port",
		.handler = &jtag_esp_remote_set_port,
		.mode = COMMAND_CONFIG,
		.help = "set the port of the ESP remote TCP server",
		.usage = "description_string",
	},
	{
		.name = "jtag_esp_remote_set_address",
		.handler = &jtag_esp_remote_set_address,
		.mode = COMMAND_CONFIG,
		.help = "set the address of the ESP remote TCP server",
		.usage = "description_string",
	},
	COMMAND_REGISTRATION_DONE
};

static struct jtag_interface esp_remote_jtag_interface = {
	.supported = DEBUG_CAP_TMS_SEQ,
	.execute_queue = jtag_esp_remote_execute_queue,
};

struct adapter_driver esp_remote_adapter_driver = {
	.name = "jtag_esp_remote",
	.commands = jtag_esp_remote_command_handlers,
	.transports = jtag_only,

	.init = jtag_esp_remote_init,
	.quit = jtag_esp_remote_quit,
	.speed_div = jtag_esp_remote_jtag_speed_div,
	.speed = jtag_esp_remote_jtag_speed,
	.khz = jtag_esp_remote_jtag_khz,

	.jtag_ops = &esp_remote_jtag_interface,
};
