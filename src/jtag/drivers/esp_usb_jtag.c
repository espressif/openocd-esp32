/***************************************************************************
 *   Espressif USB to Jtag adapter                                         *
 *   Copyright (C) 2020 Espressif Systems (Shanghai) Co. Ltd.              *
 *   Author: Jeroen Domburg <jeroen@espressif.com>                         *
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

#if IS_CYGWIN == 1
#include "windows.h"
#endif

#include <jtag/interface.h>
#include <helper/time_support.h>
#include "bitq.h"
#include "jtag_usb_common.h"
#include "libusb_common.h"

#define __packed __attribute__((packed))

/*
  Holy Crap, it's protocol documentation, and it's even vendor-provided!

  A device that speaks this protocol has two endpoints intended for JTAG debugging: one
  OUT for the host to send encoded commands to, one IN from which the host can read any read
  TDO bits. The device will also respond to vendor-defined interface requests on ep0.

  The main communication method is over the IN/OUT endpoints. The commands that are expected
  on the OUT endpoint are one nibble wide and are processed high-nibble-first, low-nibble-second,
  and in the order the bytes come in. Commands are defined as follows:

      bit     3   2    1    0
  CMD_CLK   [ 0   cap  tdi  tms  ]
  CMD_RST   [ 1   0    0    srst ]
  CMD_FLUSH [ 1   0    1    0    ]
  CMD_RSV   [ 1   0    1    1    ]
  CMD_REP   [ 1   1    R1   R0   ]

  CMD_CLK sets the TDI and TMS lines to the value of `tdi` and `tms` and lowers, then raises, TCK. If
  `cap` is 1, the value of TDO is captured and can be retrieved over the IN endpoint. The bytes read from
  the IN endpoint specifically are these bits, with the lowest it in every byte captured first and the
  bytes returned in the order the data in them was captured. The durations of TCK being high / low can
  be set using the VEND_JTAG_SETDIV vendor-specific interface request.

  CMD_RST controls the SRST line; as soon as the command is processed, the SRST line will be set
  to the value of `srst`.

  CMD_FLUSH flushes the IN endpoint; zeroes will be added to the amount of bits in the endpoint until
  the payload is a multiple of bytes, and the data is offered to the host. If the IN endpoint has
  no data, this effectively becomes a no-op; the endpoint won't send any 0-byte payloads.

  CMD_RSV is reserved for future use.

  CMD_REP repeats the last command that is not CMD_REP. The amount of times a CMD_REP command will
  re-execute this command is (r1*2+r0)<<(2*n), where n is the amount of previous repeat commands executed
  since the command to be repeated.

  An example for CMD_REP: Say the host queues:
  1. CMD_CLK - This will execute one CMD_CLK.
  2. CMD_REP with r1=0 and r0=1 - This will execute 1. another (0*2+1)<<(2*0)=1 time.
  3. CMD_REP with r1=1 and r0=0 - This will execute 1. another (1*2+0)<<(2*1)=4 times.
  4. CMD_REP with r1=0 and r0=1 - This will execute 1. another (0*2+1)<<(2*2)=8 time.
  5. CMD_FLUSH - This will flush the IN pipeline.
  6. CMD_CLK - This will execute one CMD_CLK
  7. CMD_REP with r1=1 and r0=0 - This will execute 1. another (1*2+0)<<(2*0)=2 times.
  8. CMD_FLUSH - This will flush the IN pipeline.

  Note that the net effect of the repetitions is that command 1 is executed (1+1+4+8=) 14 times and
  command 6 is executed (1+2=) 3 times.

  Note that the device only has a fairly limited amount of endpoint RAM. It's probably best to keep
  an eye on the amount of bytes that are supposed to be in the IN endpoint and grab those before stuffing
  more commands into the OUT endpoint: the OUT endpoint will not accept any more commands (writes will
  time out) when the IN endpoint buffers are all filled up.

  The device also supports some vendor-specific interface requests. These requests are sent as control
  transfers on endpoint 0 to the JTAG endpoint. Note that these commands bypass the data in the OUT
  endpoint; if timing is important, it's important that this endpoint is empty. This can be done by
  e.g sending one CMD_CLK capturing TDI, then one CMD_FLUSH, then waiting until the bit appears on the
  IN endpoint.

  bmRequestType bRequest         wValue   wIndex    wLength Data
  01000000b     VEND_JTAG_SETDIV [divide] interface 0       None
  01000000b     VEND_JTAG_SETIO  [iobits] interface 0       None
  11000000b     VEND_JTAG_GETTDO  0       interface 1       [iostate]
  10000000b     GET_DESCRIPTOR(6) 0x2000  0         256     [jtag cap desc]

  VEND_JTAG_SETDIV indirectly controls the speed of the TCK clock. The value written here is the length
        of a TCK cycle, in ticks of the adapters base clock. Both the base clock value as well as the
        minimum and maximum divider can be read from the jtag capabilities descriptor, as explained
        below. Note that this should not be set to a value outside of the range described there,
        otherwise results are undefined.

  VEND_JTAG_SETIO can be controlled to directly set the IO pins. The format of [iobits] normally is
        {11'b0, srst, trst, tck, tms, tdi}
        Note that the first 11 0 bits are reserved for future use, current hardware ignores them.

  VEND_JTAG_GETTDO returns one byte, of which bit 0 indicates the current state of the TDO input.
        Note that other bits are reserved for future use and should be ignored.

  To describe the capabilities of the JTAG adapter, a specific descriptor (0x20) can be retrieved.
  The format of the descriptor documented below. The descriptor works in the same fashion as USB
  descriptors: a header indicating the version and total length followed by descriptors with a
  specific type and size. Forward compatibility is guaranteed as software can skip over an unknown
  descriptor.
*/
#define JTAG_PROTO_CAPS_VER 1	/*Version field. At the moment, only version 1 is defined. */
struct jtag_proto_caps_hdr {
	uint8_t proto_ver;	/*Protocol version. Expects JTAG_PROTO_CAPS_VER for now. */
	uint8_t length;	/*of this plus any following descriptors */
} __packed;

/*
Note: At the moment, there is only a speed_caps version indicating the base speed of the JTAG
hardware is derived from the APB bus speed of the SoC. If later on, there are standalone
converters using the protocol, we should define e.g. JTAG_PROTO_CAPS_SPEED_FIXED_TYPE to distinguish
between the two.

Note: If the JTAG device has larger buffers than endpoint-size-plus-a-bit, we should have some kind
of caps header to assume this. If no such caps exist, assume a minimum (in) buffer of endpoint size + 4.
*/

#define JTAG_PROTO_CAPS_SPEED_APB_TYPE 1
struct jtag_gen_hdr {
	uint8_t type;
	uint8_t length;
} __packed;

struct jtag_proto_caps_speed_apb {
	uint8_t type;	/*Type, always JTAG_PROTO_CAPS_SPEED_APB_TYPE */
	uint8_t length;	/*Length of this */
	uint16_t apb_speed_10khz;	/*ABP bus speed, in 10KHz increments. Base speed is half
					 * this. */
	uint16_t div_min;	/*minimum divisor (to base speed), inclusive */
	uint16_t div_max;	/*maximum divisor (to base speed), inclusive */
} __packed;

#define VEND_DESCRIPTOR_JTAG_CAPS 0x2000

#define VEND_JTAG_SETDIV 0
#define VEND_JTAG_SETIO 1
#define VEND_JTAG_GETTDO 2

#define VEND_JTAG_SETIO_TDI             (1<<0)
#define VEND_JTAG_SETIO_TMS             (1<<1)
#define VEND_JTAG_SETIO_TCK             (1<<2)
#define VEND_JTAG_SETIO_TRST    (1<<3)
#define VEND_JTAG_SETIO_SRST    (1<<4)

#define CMD_CLK(cap, tdi, tms) ((cap ? 4 : 0)|(tms ? 2 : 0)|(tdi ? 1 : 0))
#define CMD_RST(srst) (8|(srst ? 1 : 0))
#define CMD_FLUSH 0xA
#define CMD_RSVD 0xB
#define CMD_REP(r) (0xC+(r))

/*The internal repeats register is 10 bits, which means we can have 5 repeat commands in a
 *row at max. This translates to ('b1111111111+1=)1024 reps max. */
#define CMD_REP_MAX_REPS 1024

/*Currently we only support one USB device. Its PID/VID are defined here. */
#define USB_VID 0x303A
#define USB_PID 0x1001
#define USB_CONFIGURATION 0

/*Buffer size; is equal to the endpoint size. In bytes
 *ToDo for future adapters: read from device configuration? */
#define OUT_EP_SZ 64
/*Out data can be buffered for longer without issues (as long as the in buffer does not overflow),
 *so we'll use an out buffer that is much larger than the out ep size. */
#define OUT_BUF_SZ (OUT_EP_SZ*32)
/*The in buffer cannot be larger than the device can offer, though. */
#define IN_BUF_SZ 64

/*Because a series of out commands can lead to a multitude of IN_BUF_SZ-sized in packets
 *to be read, we have multiple buffers to store those before the bitq interface reads them out. */
#define IN_BUF_CT 8

/*Private data */
struct esp_usb_jtag {
	struct jtag_libusb_device_handle *usb_device;
	int base_speed_khz;
	int div_min;
	int div_max;
	uint8_t out_buf[OUT_BUF_SZ];
	int out_buf_pos_nibbles;			/*write position in out_buf */

	uint8_t in_buf[IN_BUF_CT][IN_BUF_SZ];
	int in_buf_size_bits[IN_BUF_CT];	/*size in bits of the data stored in an in_buf */
	int cur_in_buf_rd, cur_in_buf_wr;	/*read/write index */
	int in_buf_pos_bits;				/*which bit in the in buf needs to be
							 * returned to bitq next */

	unsigned int read_ep;
	unsigned int write_ep;

	int prev_cmd;						/*previous command, stored here for
								 * RLEing. */
	int prev_cmd_repct;					/*Amount of repetitions of that
								 * command we have seen until now */

	/*This is the total number of in bits we need to read, including in unsent commands */
	int pending_in_bits;
	FILE *logfile;						/*If non-NULL, we log communication
								 * traces here. */

	int hw_in_fifo_len;
};

/*For now, we only use one static private struct. Technically, we can re-work this, but I don't
 * think */
/*OpenOCD supports multiple JTAG adapters anyway. */
static struct esp_usb_jtag esp_usb_jtag_priv;
static struct esp_usb_jtag *priv= &esp_usb_jtag_priv;

/*The JTAG adapter can drop a logfile detailing all low-level USB transactions that are done.
 *If this is defined, the log will have entries that allow replay on a testbed. */
#define LOG_REPLAYABLE

const char *cmds_string[]= {"000", "001", "010", "011", "100", "101", "110", "111",
			    "srst0", "srst1", "flush", "rsv", "rep0", "rep1", "rep2", "rep3"};

static void log_cmds(uint8_t *buf, int ct, int wr)
{
	fprintf(priv->logfile, "Wrote %d of %d bytes:%s\n", wr, ct, wr != ct ? "***ERROR***" : "");
#ifdef LOG_REPLAYABLE
	fprintf(priv->logfile, ":w %02X ", ct);
	for (int n= 0; n < ct; n++)
		fprintf(priv->logfile, "%02X ", buf[n]);
	fprintf(priv->logfile, "\n");
#endif
	for (int n= 0; n < ct*2; n++) {
		int c= (n&1) ? (buf[n/2]&0xf) : (buf[n/2]>>4);
		fprintf(priv->logfile, "%s ", cmds_string[c]);
	}
	fprintf(priv->logfile, "\n");
	fflush(priv->logfile);
}

static void log_resp(uint8_t *buf, int ct, int recvd)
{
	fprintf(priv->logfile,
		"Read %d of %d bytes (%d (%d bits) were pending): %s\n",
		recvd,
		ct,
		(priv->pending_in_bits+7)/8,
		priv->pending_in_bits,
		recvd != ct ? "***ERROR***" : "");
#ifdef LOG_REPLAYABLE
	fprintf(priv->logfile, "%cr %02X\n",recvd == 0 ? 'X' : ':', ct);
#endif
	for (int n= 0; n < recvd; n++)
		fprintf(priv->logfile, "%02X ", buf[n]);
	fprintf(priv->logfile, "\n");
	fflush(priv->logfile);
}


/*Try to receive from USB endpoint into the current priv->in_buf */
static int esp_usb_jtag_recv_buf(void)
{
	if (priv->in_buf_size_bits[priv->cur_in_buf_wr] != 0)
		LOG_ERROR("esp_usb_jtag: IN buffer overflow! (%d, size %d)",
			priv->cur_in_buf_wr,
			priv->in_buf_size_bits[priv->cur_in_buf_wr]);

	size_t ct= (priv->pending_in_bits+7)/8;
	if (ct > IN_BUF_SZ)
		ct= IN_BUF_SZ;
	if (ct == 0) {
		/*Note that the adapters IN EP specifically does *not* usually generate 0-byte in
		 * packets if there has */
		/*been no data since the last flush. As such, we don't need (and shouldn't) try to
		 * read it. */
		return ERROR_OK;
	}
	size_t n=
		jtag_libusb_bulk_read(priv->usb_device,
		priv->read_ep,
		(char *)priv->in_buf[priv->cur_in_buf_wr],
		ct,
		5000 /*ms*/);
	if (priv->logfile)
		log_resp(priv->in_buf[priv->cur_in_buf_wr], ct, n);
	if (n == 0) {
		/*Sometimes the hardware returns 0 bytes instead of NAKking the transaction. Ignore
		 * this. */
		return ERROR_FAIL;
	}
	if (n != ct) {
		/*Huh, short read? */
		LOG_ERROR("esp_usb_jtag: usb received only %d out of %d bytes.", (int)n, (int)ct);
		return ERROR_FAIL;
	}
	/*Adjust the amount of bits we still expect to read from the USB device after this. */
	int bits_in_buf= priv->pending_in_bits;			/*initially assume we read
								 * everything that was pending */
	if (bits_in_buf > (int)ct*8)
		bits_in_buf= ct*8;			/*...but correct that if that was not the
							 * case. */
	priv->pending_in_bits-= bits_in_buf;
	priv->in_buf_size_bits[priv->cur_in_buf_wr]= bits_in_buf;
	/*next in buffer for the next time. */
	priv->cur_in_buf_wr++;
	if (priv->cur_in_buf_wr == IN_BUF_CT)
		priv->cur_in_buf_wr= 0;
	LOG_DEBUG_IO("esp_usb_jtag: In ep: received %d bytes; %d bytes (%d bits) left.", (int)n,
		(priv->pending_in_bits+7)/8, priv->pending_in_bits);
	return ERROR_OK;
}

/*Sends priv->out_buf to the USB device. */
static int esp_usb_jtag_send_buf(void)
{
	size_t ct= priv->out_buf_pos_nibbles/2;
	size_t n= 0;
	n= jtag_libusb_bulk_write(priv->usb_device,
		priv->write_ep,
		((char *)priv->out_buf),
		ct,
		5000 /*ms*/);
	LOG_DEBUG_IO("esp_usb_jtag: sent %d bytes.", (int)n);
	if (priv->logfile)
		log_cmds(priv->out_buf, ct, n);
	if (n != ct) {
		LOG_ERROR("esp_usb_jtag: usb sent only %d out of %d bytes.", (int)n, (int)ct);
		return ERROR_FAIL;
	}
	priv->out_buf_pos_nibbles= 0;

	/*If there's more than a bufferful of data queing up in the jtag adapters IN endpoint, empty
	 *
	 *all but one buffer. */
	while (priv->pending_in_bits > (IN_BUF_SZ + priv->hw_in_fifo_len - 1) * 8) {
		esp_usb_jtag_recv_buf();
	}

	return ERROR_OK;
}

/*Simply adds a command to the buffer. Is called by the RLE encoding mechanism.
 *Also sends the intermediate buffer if there's enough to go into one USB packet. */
static void esp_usb_jtag_command_add_raw(int cmd)
{
	if ((priv->out_buf_pos_nibbles & 1) == 0)
		priv->out_buf[priv->out_buf_pos_nibbles/2] = (cmd<<4);
	else
		priv->out_buf[priv->out_buf_pos_nibbles/2] |= cmd;
	priv->out_buf_pos_nibbles++;

	if (priv->out_buf_pos_nibbles == OUT_BUF_SZ*2)
		esp_usb_jtag_send_buf();
	if (priv->out_buf_pos_nibbles%(OUT_EP_SZ*2) == 0) {
		if (priv->pending_in_bits > (IN_BUF_SZ + priv->hw_in_fifo_len - 1) * 8)
			esp_usb_jtag_send_buf();
	}
}

/*Writes a command stream equivalent to writing `cmd` `ct` times. */
static void esp_usb_jtag_write_rlestream(int cmd, int ct)
{
	/*Special case: stacking flush commands does not make sense (and may not make the hardware
	 * very happy) */
	if (cmd == CMD_FLUSH)
		ct= 1;
	/*Output previous command and repeat commands */
	esp_usb_jtag_command_add_raw(cmd);
	ct--;	/*as the previous line already executes the command one time */
	while (ct > 0) {
		esp_usb_jtag_command_add_raw(CMD_REP(ct&3));
		ct>>= 2;
	}
}


/*Adds a command to the buffer of things to be sent. Transparently handles RLE compression using
 *the CMD_REP_x commands */
static void esp_usb_jtag_command_add(int cmd)
{
	if (cmd == priv->prev_cmd && priv->prev_cmd_repct < CMD_REP_MAX_REPS)
		priv->prev_cmd_repct++;
	else {
		/*We can now write out the previous command plus repeat count. */
		if (priv->prev_cmd_repct)
			esp_usb_jtag_write_rlestream(priv->prev_cmd, priv->prev_cmd_repct);
		/*Ready for new command. */
		priv->prev_cmd= cmd;
		priv->prev_cmd_repct= 1;
	}
}

/*Called by bitq interface to output a bit on tdi and perhaps read a bit from tdo */
static int esp_usb_jtag_out(int tms, int tdi, int tdo_req)
{
	esp_usb_jtag_command_add(CMD_CLK(tdo_req, tdi, tms));
	if (tdo_req)
		priv->pending_in_bits++;
	return ERROR_OK;
}

/*Called by bitq interface to flush all output commands and get returned data ready to read */
static int esp_usb_jtag_flush(void)
{
	/*Make sure last command is written */
	if (priv->prev_cmd_repct)
		esp_usb_jtag_write_rlestream(priv->prev_cmd, priv->prev_cmd_repct);
	priv->prev_cmd_repct= 0;
	/*Flush in buffer */
	esp_usb_jtag_command_add_raw(CMD_FLUSH);
	/*Make sure we have an even amount of commands, as we can't write a nibble by itself. */
	if (priv->out_buf_pos_nibbles & 1) {
		/*If not, pad with an extra FLUSH */
		esp_usb_jtag_command_add_raw(CMD_FLUSH);
	}
	LOG_DEBUG_IO("esp_usb_jtag: Flush!");
	/*Send off the buffer. */
	int r= esp_usb_jtag_send_buf();
	if (r != ERROR_OK)
		return r;

	/*Immediately fetch the response bits. */
	while (priv->pending_in_bits > 0) {
		esp_usb_jtag_recv_buf();
	}

	return ERROR_OK;
}

/*Called by bitq interface to sleep for a determined amount of time */
static int esp_usb_jtag_sleep(unsigned long us)
{
	esp_usb_jtag_flush();
	/*ToDo: we can sleep more precisely (for small amounts of sleep at least) by sending dummy
	 * commands to the adapter. */
	jtag_sleep(us);
	return 0;
}

/*Called by the bitq interface to set the various resets */
static int esp_usb_jtag_reset(int trst, int srst)
{
	/*ToDo: handle trst using setup commands. Kind-of superfluous, however, as we can also do */
	/*a tap reser using tms, and it's also not implemented on other ESP32 chips with external
	 * JTAG. */
	esp_usb_jtag_command_add(CMD_RST(srst));
	return ERROR_OK;
}

/*Called by bitq to see if the IN data already is returned to the host. */
static int esp_usb_jtag_in_rdy(void)
{
	/*We read all bits in the flush() routine, so if we're here, we have bits or are at EOF. */
	return 1;
}

/*Read one bit from the IN data */
static int esp_usb_jtag_in(void)
{
	int r;
	if (!esp_usb_jtag_in_rdy()) {
		LOG_ERROR("esp_usb_jtag: Eeek! bitq asked us for in data while not ready!");
		return -1;
	}
	if (priv->cur_in_buf_rd == priv->cur_in_buf_wr &&
		priv->in_buf_size_bits[priv->cur_in_buf_rd] == 0)
		return -1;

	/*Extract the bit */
	r=
		(priv->in_buf[priv->cur_in_buf_rd][priv->in_buf_pos_bits/8]&
		(1<<(priv->in_buf_pos_bits&7))) ? 1 : 0;
	/*Move to next bit. */
	priv->in_buf_pos_bits++;
	if (priv->in_buf_pos_bits == priv->in_buf_size_bits[priv->cur_in_buf_rd]) {
		/*No more bits in this buffer; mark as re-usable and move to next buffer. */
		priv->in_buf_pos_bits= 0;
		priv->in_buf_size_bits[priv->cur_in_buf_rd]= 0;	/*indicate it is free again */
		priv->cur_in_buf_rd++;
		if (priv->cur_in_buf_rd == IN_BUF_CT)
			priv->cur_in_buf_rd= 0;
	}
	return r;
}

static int esp_usb_jtag_init(void)
{
	memset(priv, 0, sizeof(struct esp_usb_jtag));

	const uint16_t vids[]= {USB_VID, 0};	/* must be null terminated */
	const uint16_t pids[]= {USB_PID, 0};	/* must be null terminated */

	bitq_interface= calloc(sizeof(struct bitq_interface), 1);
	bitq_interface->out= esp_usb_jtag_out;
	bitq_interface->flush= esp_usb_jtag_flush;
	bitq_interface->sleep= esp_usb_jtag_sleep;
	bitq_interface->reset= esp_usb_jtag_reset;
	bitq_interface->in_rdy= esp_usb_jtag_in_rdy;
	bitq_interface->in= esp_usb_jtag_in;

	int r= jtag_libusb_open(vids, pids, NULL, &priv->usb_device);
	if (r != ERROR_OK) {
		LOG_ERROR("esp_usb_jtag: could not find or open device!");
		goto out;
	}

	jtag_libusb_set_configuration(priv->usb_device, USB_CONFIGURATION);

	r= jtag_libusb_choose_interface(priv->usb_device, &priv->read_ep, &priv->write_ep,
		0xff, 0xff, 0x01, 0x2);
	if (r != ERROR_OK) {
		LOG_ERROR("esp_usb_jtag: error finding/claiming JTAG interface on device!");
		goto out;
	}

	char jtag_caps_desc[256];
	r= jtag_libusb_control_transfer(priv->usb_device,
		LIBUSB_ENDPOINT_IN|LIBUSB_REQUEST_TYPE_STANDARD|LIBUSB_RECIPIENT_DEVICE,
		LIBUSB_REQUEST_GET_DESCRIPTOR, VEND_DESCRIPTOR_JTAG_CAPS, 0,
		jtag_caps_desc, 255, 1000);
	if (r <= 0) {
		LOG_ERROR("esp_usb_jtag: could not retrieve jtag_caps descriptor!");
		goto out;
	}

	/*defaults for values we normally get from the jtag caps descriptor */
	priv->base_speed_khz= -1;
	priv->div_min= 1;
	priv->div_max= 1;

	struct jtag_proto_caps_hdr *hdr= (struct jtag_proto_caps_hdr *)jtag_caps_desc;
	if (hdr->proto_ver != JTAG_PROTO_CAPS_VER) {
		LOG_ERROR("esp_usb_jtag: unknown jtag_caps descriptor version 0x%X!",
			hdr->proto_ver);
		goto out;
	}
	int p= 2;	/*start of rest of descriptors */
	while (p < hdr->length) {
		struct jtag_gen_hdr *dhdr= (struct jtag_gen_hdr *)&jtag_caps_desc[p];
		if (dhdr->type == JTAG_PROTO_CAPS_SPEED_APB_TYPE) {
			struct jtag_proto_caps_speed_apb *spcap=
				(struct jtag_proto_caps_speed_apb *)dhdr;
			/*base speed always is half APB speed */
			priv->base_speed_khz= (spcap->apb_speed_10khz*10)/2;
			priv->div_min= spcap->div_min;
			priv->div_max= spcap->div_max;
			/*todo: mark in priv that this is apb-derived and as such may change if apb
			 * ever changes? */
		} else
			LOG_WARNING("esp_usb_jtag: unknown caps type 0x%X", dhdr->type);
		p+= dhdr->length;
	}

	if (priv->base_speed_khz == -1) {
		LOG_WARNING("esp_usb_jtag: No speed caps found... using sane-ish defaults.");
		priv->base_speed_khz= 1000;
	}
	LOG_INFO("esp_usb_jtag: Device found. Base speed %dKHz, div range %d to %d",
		priv->base_speed_khz, priv->div_min, priv->div_max);

	/*ToDo: grab from (future) descriptor if we ever have a device with larger IN buffers */
	priv->hw_in_fifo_len= 4;

	return ERROR_OK;
out:
	jtag_libusb_close(priv->usb_device);
	free(bitq_interface);
	bitq_interface= NULL;
	priv->usb_device= NULL;
	return ERROR_FAIL;
}

static int esp_usb_jtag_quit(void)
{
	if (priv->usb_device == NULL)
		return ERROR_OK;
	jtag_libusb_close(priv->usb_device);
	bitq_cleanup();
	free(bitq_interface);
	bitq_interface= NULL;
	if (priv->logfile)
		fclose(priv->logfile);
	return ERROR_OK;
}

static int esp_usb_jtag_speed_div(int divisor, int *khz)
{
	*khz = priv->base_speed_khz / divisor;
	return ERROR_OK;
}

static int esp_usb_jtag_khz(int khz, int *divisor)
{
	if (khz == 0) {
		LOG_WARNING("esp_usb_jtag: RCLK not supported");
		return ERROR_FAIL;
	}

	*divisor= priv->base_speed_khz/khz;
	LOG_DEBUG("Divisor for %d KHz with base clock of %d khz is %d",
		khz,
		priv->base_speed_khz,
		*divisor);
	if (*divisor < priv->div_min)
		*divisor= priv->div_min;
	if (*divisor > priv->div_max)
		*divisor= priv->div_max;

	return ERROR_OK;
}

static int esp_usb_jtag_speed(int divisor)
{
	if (divisor == 0) {
		LOG_ERROR("esp_usb_jtag: Adaptive clocking is not supported.");
		return ERROR_JTAG_NOT_IMPLEMENTED;
	}

	LOG_DEBUG("esp_usb_jtag: setting divisor %d", divisor);
	if (priv->logfile)
		fprintf(priv->logfile, "Setting divisor to %d\n.", divisor);
	jtag_libusb_control_transfer(priv->usb_device,
		0x40, VEND_JTAG_SETDIV, divisor, 0, NULL, 0, 1000);

	return ERROR_OK;
}

COMMAND_HANDLER(esp_usb_jtag_tdo_cmd)
{
	char tdo;
	if (!priv->usb_device)
		return ERROR_FAIL;
	int r= jtag_libusb_control_transfer(priv->usb_device,
		0xC0, VEND_JTAG_GETTDO, 0, 0, &tdo, 1, 1000);
	if (r < 1)
		return r;

	command_print(CMD, "%d", tdo);
	if (priv->logfile)
		fprintf(priv->logfile, "Retrieved TDO: %d\n.", tdo);

	return ERROR_OK;
}

COMMAND_HANDLER(esp_usb_jtag_setio_cmd)
{
	uint32_t tdi, tms, tck, trst, srst;
	uint16_t d= 0;
	if (!priv->usb_device)
		return ERROR_FAIL;

	if (CMD_ARGC != 5)
		return ERROR_COMMAND_SYNTAX_ERROR;
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], tdi);
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], tms);
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[2], tck);
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[3], trst);
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[4], srst);
	if (tdi)
		d|= VEND_JTAG_SETIO_TDI;
	if (tms)
		d|= VEND_JTAG_SETIO_TMS;
	if (tck)
		d|= VEND_JTAG_SETIO_TCK;
	if (trst)
		d|= VEND_JTAG_SETIO_TRST;
	if (srst)
		d|= VEND_JTAG_SETIO_SRST;

	if (priv->logfile)
		fprintf(priv->logfile, "SetIO command 0x%X\n.", d);
	jtag_libusb_control_transfer(priv->usb_device,
		0x40, VEND_JTAG_SETIO, d, 0, NULL, 0, 1000);

	return ERROR_OK;
}

COMMAND_HANDLER(esp_usb_jtag_log_cmd)
{
	if (!priv->usb_device)
		return ERROR_FAIL;
	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	priv->logfile= fopen(CMD_ARGV[0], "w");
	if (!priv->logfile)
		command_print(CMD, "Could not open log file: %s.", strerror(errno));

	return ERROR_OK;
}


static const struct command_registration esp_usb_jtag_subcommands[] = {
	{
		.name = "tdo",
		.handler = &esp_usb_jtag_tdo_cmd,
		.mode = COMMAND_EXEC,
		.help = "Returns the current state of the TDO line",
		.usage = "",
	},
	{
		.name = "setio",
		.handler = &esp_usb_jtag_setio_cmd,
		.mode = COMMAND_EXEC,
		.help = "Manually set the status of the output lines",
		.usage = "tdi tms tck trst srst"
	},
	{
		.name = "log",
		.handler = &esp_usb_jtag_log_cmd,
		.mode = COMMAND_EXEC,
		.help = "Log USB comms to file",
		.usage = "logfile.txt"
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration esp_usb_jtag_commands[] = {
	{
		.name = "espusbjtag",
		.mode = COMMAND_ANY,
		.help = "ESP-USB-JTAG commands",
		.chain = esp_usb_jtag_subcommands,
		.usage = "",
	},
	COMMAND_REGISTRATION_DONE
};


struct jtag_interface esp_usb_jtag_interface = {
	.name = "esp_usb_jtag",
	.supported = DEBUG_CAP_TMS_SEQ,
	.transports = jtag_only,
	.commands = esp_usb_jtag_commands,

	.init = esp_usb_jtag_init,
	.quit = esp_usb_jtag_quit,
	.execute_queue = bitq_execute_queue,
	.speed_div = esp_usb_jtag_speed_div,
	.speed = esp_usb_jtag_speed,
	.khz = esp_usb_jtag_khz,
};
