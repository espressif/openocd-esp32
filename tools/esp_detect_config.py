#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later

import subprocess
import json
import argparse
import logging
import os.path
import socket
import time
import copy
import tempfile


class OpenOcd:
    COMMAND_TOKEN           = '\x1a'
    TAPINS_NARSEL           = 0x1C
    TAPINS_NARSEL_ADRLEN    = 8
    TAPINS_NARSEL_DATALEN   = 32
    XDMREG_OCDID_NAR        = 0x40

    def __init__(self, oocd_path, scripts_dir, iface_config, host="127.0.0.1", port=6666, start_tmo=1, iface_cmd="", env=None):
        self.host       = host
        self.port       = port
        self.buf_size   = 4096
        self.sock       = None
        self.iface_cmd  = iface_cmd

        run_args = [oocd_path]
        if len(scripts_dir):
            run_args += ["-s", scripts_dir]
        run_args += ["-f", iface_config]
        logging.debug("Run OpenOCD with args: %s", run_args)
        self.proc = subprocess.Popen(args=run_args, bufsize=0, stdout=subprocess.PIPE, stdin=None,
                                     stderr=subprocess.STDOUT, universal_newlines=True, env=env)
        if self.proc.poll():
            logging.error("Failed to start OpenOCD %d!", self.proc.returncode)
            self.readout_all_output()
            raise RuntimeError
        time.sleep(start_tmo)
        # logging.error(self.proc.stdout.read(20))

    def readout_all_output(self):
        try:
            outs,_ = self.proc.communicate(timeout=5)
        except subprocess.TimeoutExpired:
            logging.error("Failed to wait for OpenOCD output! Kill it.")
            self.proc.kill()
            outs,_ = self.proc.communicate()
        logging.debug(outs)

    def connect(self):
        try:
            self._connect()
        except ConnectionRefusedError:
            logging.warning("Failed to connect to OpenOCD!")
            self.proc.kill()
            self.readout_all_output()
            raise ConnectionRefusedError
        return self

    def disconnect(self):
        self._disconnect()
        self.proc.kill()
        self.readout_all_output()

    def __enter__(self):
        self.connect()

    def __exit__(self, type, value, traceback):
        self.disconnect()

    def _connect(self):
        logging.debug("Try to connect to %s:%d", self.host, self.port)
        self.sock = socket.create_connection((self.host, self.port), timeout=5)
        logging.debug("Connected to %s:%d", self.host, self.port)

    def _disconnect(self):
        if not self.sock:
            return
        try:
            self.send("exit")
        finally:
            if self.sock:
                self.sock.close()

    def send(self, cmd):
        """Send a command string to TCL RPC. Return the result that was read."""
        data = (cmd + OpenOcd.COMMAND_TOKEN).encode("utf-8")
        logging.debug("-> {%s}", data)
        self.sock.send(data)
        return self._recv()

    def _recv(self):
        """Read from the stream until the token (\x1a) was received."""
        data = bytes()
        while True:
            chunk = self.sock.recv(self.buf_size)
            data += chunk
            if bytes(OpenOcd.COMMAND_TOKEN, encoding="utf-8") in chunk:
                break

        logging.debug("<- {%s}", data)

        data = data.decode("utf-8").strip()
        data = data[:-1] # strip trailing \x1a

        return data

    def is_xtensa(self, idcode):
        return idcode == "0x120034e5"

    def xtensa_dm_read_reg(self, tap_id, reg_addr):
        # scan NARSEL instruction to IR
        self.send("irscan {} 0x{:x}".format(tap_id, OpenOcd.TAPINS_NARSEL))
        # scan reg addr to DR
        self.send("drscan {} 0x{:x} 0x{:x}".format(tap_id, OpenOcd.TAPINS_NARSEL_ADRLEN, reg_addr << 1))
        # scan reg value to DR
        return "0x" + self.send("drscan {} 0x{:x} 0x0".format(tap_id, OpenOcd.TAPINS_NARSEL_DATALEN))

    def taps(self):
        logging.debug("TAPs")
        taps_str = self.send("scan_chain")
        logging.debug(taps_str)
        lines = taps_str.splitlines()
        # Skip 2 heading lines
        #    TapName             Enabled  IdCode     Expected   IrLen IrCap IrMask
        # -- ------------------- -------- ---------- ---------- ----- ----- ------
        if len(lines) < 3:
            return []
        taps = []
        for line in lines[2:]:
            comps = line.strip().split()
            tap = {"name": comps[1], "idcode": comps[3], "ocdid": None}
            if self.is_xtensa(tap["idcode"]):
                tap["ocdid"] = self.xtensa_dm_read_reg(tap["name"], OpenOcd.XDMREG_OCDID_NAR)
            taps.append(tap)
        return taps

    def usb_devices(self):
        devs_paths = self.send(self.iface_cmd + " list_devs")
        devs_paths = devs_paths.splitlines()
        logging.debug(devs_paths)
        return devs_paths

    def default_usb_device(self):
        dev_path = self.send(self.iface_cmd + " get_location")
        logging.debug(dev_path)
        return dev_path

class OpenOcdRunError(RuntimeError):
    pass

def detect_and_populate_config(oocd, scripts, config_file, host, port, start_delay,
                               iface_id, iface_cmd, usb_location, esp_cfg):
    try:
        my_env = os.environ.copy()
        if usb_location:
            my_env["OPENOCD_USB_ADAPTER_LOCATION"] = usb_location
        ocd = OpenOcd(oocd, scripts, config_file, host, port, start_delay, iface_cmd, env=my_env)
        ocd.connect()
    except:
        raise OpenOcdRunError()

    try:
        devices = ocd.usb_devices()
        curr_dev = ocd.default_usb_device()
        logging.info("Default device %s", curr_dev)
        devices.remove(curr_dev)
        logging.info("Found other devices %s", devices)

        iface_boards = []
        for tap in ocd.taps():
            logging.info("Found TAP %s, idcode %s, ocdid %s", tap["name"], tap["idcode"], tap["ocdid"])
            for tgt in esp_cfg["targets"]:
                if tgt["idcode"] != tap["idcode"]:
                    continue
                if tgt.get("ocdid") != tap["ocdid"]:
                    continue
                logging.info("Matched idcode %s, ocdid %s for target %s", tap["idcode"], tap["ocdid"], tgt["name"])

                found = False
                for cfg_board in esp_cfg["boards"]:
                    if cfg_board["target"] == tgt["id"] and cfg_board["interface"] == iface_id:
                        # Check if we already have similar board entry.
                        # Similar board entries for the same target and USB
                        # location mean multiple cores (one per TAP)
                        duplicate = False
                        for b in iface_boards:
                            if b["name"] == cfg_board["name"]:
                                duplicate = True
                                break
                        if not duplicate:
                            found = True
                            logging.info("Found board %s", cfg_board["name"])
                            iface_boards.append(cfg_board)
                            iface_boards[-1]["location"] = "usb://" + (usb_location if usb_location else curr_dev)
                if not found:
                    logging.debug("No board entry found for idcode %s target %s @ iface %s!",
                                    tgt["idcode"], tgt["name"], iface_id)
    finally:
        ocd.disconnect()
    return iface_boards,devices


def main():  # type: () -> None

    parser = argparse.ArgumentParser('Espressif OpenOCD Config Detecting Tool')

    parser.add_argument('--esp-config', '-c', help='Path to Espressif OpenOCD config json file', type=str)
    parser.add_argument('--oocd', '-b', help='Path to OpenOCD binary', type=str, default="openocd")
    parser.add_argument('--scripts', '-s', help='Path to OpenOCD scripts', type=str, default="")
    parser.add_argument('--debug', '-d', help='Debug level: 0-4', type=int, default=1)
    parser.add_argument('--start-delay', '-r', help='OpenOCD startup time', type=int, default=1)
    parser.add_argument('--host', '-a', help='OpenOCD host IP addr', type=str, default="127.0.0.1")
    parser.add_argument('--port', '-p', help='OpenOCD TCL port number', type=int, default=6666)
    parser.add_argument('--output', '-o', help='Output filename', type=str, default="stdout")

    args = parser.parse_args()

    if args.debug == 0:
        log_level = logging.CRITICAL
    elif args.debug == 1:
        log_level = logging.ERROR
    elif args.debug == 2:
        log_level = logging.WARNING
    elif args.debug == 3:
        log_level = logging.INFO
    else:
        log_level = logging.DEBUG
    logging.basicConfig(format='%(levelname)s: %(message)s', level=log_level)

    with open(args.esp_config, "r") as f:
        esp_cfg = json.load(f)

    cfg_out = copy.deepcopy(esp_cfg)
    cfg_out["boards"] = []
    cfg_out.pop("interfaces", None)

    for iface in esp_cfg["interfaces"]:
        logging.info("Check interface %s", iface)
        extra_devs = []
        try:
            iface_boards,extra_devs = detect_and_populate_config(args.oocd, args.scripts,
                                                                 os.path.join("interface", iface["config_file"]),
                                                                 args.host, args.port, args.start_delay,
                                                                 iface["id"], iface["command"], None, esp_cfg)
        except OpenOcdRunError:
            logging.warning("Skip interface %s", iface)
            continue
        cfg_out["boards"].extend(iface_boards)

        for dev in extra_devs:
            logging.info("Check interface %s @ %s", iface, dev)
            try:
                iface_boards,_ = detect_and_populate_config(args.oocd, args.scripts,
                                                            os.path.join("interface", iface["config_file"]),
                                                            args.host, args.port, args.start_delay,
                                                            iface["id"], iface["command"], dev, esp_cfg)
            except OpenOcdRunError:
                logging.warning("Skip interface %s @ %s", iface, dev)
                continue
            cfg_out["boards"].extend(iface_boards)

    if args.output == "stdout":
        print(json.dumps(cfg_out))
    else:
        with open(args.output, "w") as f:
            json.dump(cfg_out, f, indent=2)


if __name__ == '__main__':
    main()
