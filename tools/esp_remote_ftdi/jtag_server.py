#!/usr/bin/env python3
#
# This script sets up a server on port 5555, which can talk "ESP remote" protocol.
# In OpenOCD, this corresponds to the "jtag_esp_remote" interface (interfaces/jtag_esp_remote.cfg)
# This script uses pyftdi package to bit-bang JTAG protocol using an FT2232H chip.
# The default pin mapping corresponds to an ESP-WROVER-KIT board.
#
# Run "pip install -r requirements.txt" before running this script.


try:
    import socketserver
except ImportError:
    print("this script works with python3 only")
    raise SystemExit(1)
from pyftdi.gpio import GpioSyncController
import struct
from binascii import hexlify
import logging
import threading
import queue
import socket
import time


# ESP-WROVER-KIT FT2232H pin mapping:
# TCK, TDO, TDI, TMS: ADBUS0-3
# LEDs: ACBUS4-7 (unused as GpioSyncController only supports low 8 bits)
TCK_MASK = 0x0001
TDO_MASK = 0x0002
TDI_MASK = 0x0004
TMS_MASK = 0x0008

# Bits which are output
PORT_DIRECTION = TCK_MASK | TDO_MASK | TMS_MASK

HOST, PORT = "localhost", 5555
FTDI_PATH = "ftdi:///1"

# Protocol definitions
CMD_HEADER_LEN = 2
CMD_PARAM_LEN = 2
CMD_RESET = 1
CMD_SCAN = 2
CMD_TMS_SEQ = 3
VER_1 = 1

CMD_NAME = {
    CMD_SCAN: "scan",
    CMD_RESET: "reset",
    CMD_TMS_SEQ: "tms_seq"
}

gpio = GpioSyncController()


# Utility functions to convert between bits packed into a bytestring, and a list of 0/1 ints.
# This is similar to the bitstring package, except that bits within byte are counted from the LSB.
def list_to_bits(list_of_ints):
    n_bits = len(list_of_ints)
    n_bytes = (n_bits + 7) // 8
    pos = 0
    res = b''
    for i_byte in range(n_bytes):
        byte_val = 0
        for i_bit in range(8):
            if pos == n_bits:
                break
            if list_of_ints[pos]:
                byte_val |= (1 << i_bit)
            pos += 1
        res += struct.pack('B', byte_val)
    return res


def bits_to_list(b, n_bits):
    n_bytes = len(b)
    res = [0] * n_bits
    assert n_bytes == (n_bits + 7) // 8
    pos = 0
    for i_byte in range(n_bytes):
        byte_val = b[i_byte]
        for i_bit in range(8):
            if pos == n_bits:
                break
            if byte_val & (1 << i_bit):
                res[pos] = 1
            pos += 1
    return res


TEST_BITS = [1, 0, 0, 1, 0, 0, 0, 1, 1]
TEST_BYTES = b'\x89\x01'
assert list_to_bits(TEST_BITS) == TEST_BYTES
assert bits_to_list(TEST_BYTES, len(TEST_BITS)) == TEST_BITS


# JTAG related
def init_jtag():
    gpio.exchange(b'\x00')


class JTAGQueueItem(object):
    def __init__(self, tms, tdo, bits, do_read, tms_flip):
        self.tms = tms
        self.tdo = tdo
        self.bits = bits
        self.do_read = do_read
        self.tms_flip = tms_flip


JTAG_QUEUE = queue.Queue()


class JTAGThread(threading.Thread):
    def __init__(self, jtag_queue, req):
        super(JTAGThread, self).__init__()
        self.jtag_queue = jtag_queue
        self.req = req
        self.done = False

    def run(self):
        last_tms = 0
        while not self.done:
            items = [self.jtag_queue.get(block=True)]
            total_len = items[0].bits

            while total_len < 512:
                try:
                    it = self.jtag_queue.get(block=False)
                    total_len += it.bits
                    items.append(it)
                except queue.Empty:
                    break

            all_tms = []
            all_tdo = []
            n_bits = 0
            for i in items:
                if i.tms is None:
                    tms_list = [last_tms] * i.bits
                    if i.tms_flip:
                        tms_list[-1] = 1 - last_tms
                else:
                    assert not i.tms_flip
                    tms_list = bits_to_list(i.tms, i.bits)
                last_tms = tms_list[-1]

                if i.tdo is None:
                    tdo_list = [1] * i.bits
                else:
                    tdo_list = bits_to_list(i.tdo, i.bits)

                all_tms += tms_list
                all_tdo += tdo_list
                n_bits += i.bits

            tdi_list = self.do_jtag_inner(n_bits, all_tms, all_tdo)

            result = b''
            for i in items:
                tdi_part = tdi_list[0:i.bits]
                tdi_list = tdi_list[i.bits:]
                if i.do_read:
                    result += list_to_bits(tdi_part)

            assert len(tdi_list) == 0

            if len(result) > 0 and not self.done:
                self.req.sendall(result)

    @staticmethod
    def do_jtag_inner(n_bits, tms_list, tdo_list):
        gpio_out = [0] * (2 * n_bits + 1)

        # prepare the list of GPIO port values from TMS and TDO vectors
        for i in range(n_bits):
            data = tms_list[i] * TMS_MASK + tdo_list[i] * TDO_MASK
            # data is clocked in on posedge
            gpio_out[2 * i] = data
            gpio_out[2 * i + 1] = data + TCK_MASK

        # Move TCK back to 0
        gpio_out[-1] = data

        def data_to_str(n):
            s = 'C' if n & TCK_MASK else '-'
            s += 'M' if n & TMS_MASK else '-'
            s += 'I' if n & TDI_MASK else '-'
            s += 'O' if n & TDO_MASK else '-'
            return s

        gpio_in = gpio.exchange(bytearray(gpio_out))
        logging.debug("GPIO output {}".format(' '.join([data_to_str(i) for i in gpio_out])))
        logging.debug("GPIO input  {}".format(' '.join([data_to_str(i) for i in gpio_in])))

        # extract TDI
        tdi_list = [0] * n_bits
        for i in range(n_bits):
            # TDI (TDO at the target side) changes at falling TCK and should be captured at the next raising TCK.
            # This can be observed and the the data is not stable at gpio_in[2 * i + 1].
            if gpio_in[2 * i + 2] & TDI_MASK != 0:
                tdi_list[i] = 1

        return tdi_list


def do_jtag(n_bits, tms_bytes=None, tdo_bytes=None, do_read=False, tms_flip=False):
    JTAG_QUEUE.put(JTAGQueueItem(tms_bytes, tdo_bytes, n_bits, do_read, tms_flip))


# Protocol commands
def cmd_scan(n_bits, do_read, do_flip_tms, data):
    logging.debug("scan n_bits={}, do_read={}, do_flip_tms={}, data={}".format(
        n_bits, do_read, do_flip_tms, hexlify(data)
    ))
    do_jtag(n_bits, tdo_bytes=data, tms_flip=do_flip_tms, do_read=do_read)


def cmd_tms_seq(n_bits, data):
    logging.debug("tms_seq n_bits={}, data={}".format(
        n_bits, hexlify(data)
    ))
    do_jtag(n_bits, tms_bytes=data)


def cmd_reset(srst, trst):
    logging.debug("reset srst={}, trst={}".format(
        srst, trst
    ))


# Main loop, handles the incoming TCP connection
class JTAGTCPHandler(socketserver.BaseRequestHandler):
    def handle(self):
        init_jtag()
        jtag_thread = JTAGThread(JTAG_QUEUE, self.request)
        jtag_thread.start()
        try:
            request_count = 0
            self.request.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, True)

            def receive(length):
                """
                socket recv() can return less data than requested no matter what flags are selected. This function
                ensures that the required amount is received.
                """
                buf = b''
                to_read = length
                while to_read > 0:
                    buf += self.request.recv(to_read, socket.MSG_WAITALL)
                    done = len(buf)
                    to_read = length - done
                    if to_read > 0:
                        logging.info("socket receive got only {} bytes out of {}".format(done, length))
                        time.sleep(0.5)
                return buf

            while True:
                # reserved:4, ver:4, function:8
                header_bytes = receive(CMD_HEADER_LEN + CMD_PARAM_LEN)
                request_count += 1
                if request_count % 100 == 0:
                    print("\r{} requests processed".format(request_count), end='')
                logging.debug("header: {}".format(hexlify(header_bytes)))

                rsv_ver, cmd = struct.unpack_from("BB", header_bytes, 0)
                ver = rsv_ver >> 4

                logging.debug("cmd {}, args {}".format(
                    CMD_NAME.get(cmd, "%d" % cmd),
                    hexlify(header_bytes[CMD_HEADER_LEN:])))

                assert ver == VER_1
                val = struct.unpack_from("H", header_bytes, CMD_HEADER_LEN)[0]

                if cmd == CMD_SCAN:
                    # bits:12, read:1, flip_tms:1, reserved:2
                    n_bits = val & 0xfff
                    do_read = (val >> 12) & 1
                    do_flip_tms = (val >> 13) & 1
                    n_bytes = (n_bits + 7) // 8
                    data = receive(n_bytes)
                    cmd_scan(n_bits, do_read, do_flip_tms, data)

                elif cmd == CMD_TMS_SEQ:
                    # bits:12, reserved:4
                    n_bits = val & 0xfff
                    n_bytes = (n_bits + 7) // 8
                    data = receive(n_bytes)
                    cmd_tms_seq(n_bits, data)

                elif cmd == CMD_RESET:
                    # srst:1, trst:1, reserved:14
                    srst = val & 1
                    trst = (val >> 1) & 1
                    cmd_reset(srst, trst)

                else:
                    logging.error("Unknown command: {}".format(cmd))
        finally:
            print('')
            jtag_thread.done = True
            jtag_thread.join(timeout=10)


def main():
    logging.basicConfig(level=logging.INFO)
    gpio.configure(FTDI_PATH, direction=PORT_DIRECTION, frequency=10000000)
    try:
        with socketserver.TCPServer((HOST, PORT), JTAGTCPHandler) as server:
            server.serve_forever()
    except KeyboardInterrupt:
        print('\nExiting...')
        logging.shutdown()


if __name__ == "__main__":
    main()
