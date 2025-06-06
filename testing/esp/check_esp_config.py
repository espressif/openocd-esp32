#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later

import sys
import json
import argparse
import logging


def main():  # type: () -> None

    parser = argparse.ArgumentParser('Espressif OpenOCD Config Checking Tool')

    parser.add_argument('config_file', help='Path to Espressif OpenOCD config json file', type=str)
    parser.add_argument('--debug', '-d', help='Debug level: 0-4', type=int, default=1)
    parser.add_argument('--chip', '-c', help='Chip', type=str)
    parser.add_argument('--interface', '-i', help='Interface', type=str)

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

    logging.info("Check file %s", args.config_file)
    with open(args.config_file, "r") as f:
        esp_cfg = json.load(f)
    if args.interface == "jtag":
        args.interface = "ftdi"
    elif args.interface == "usb_serial_jtag":
        args.interface = "esp_usb_jtag"
    for board in esp_cfg["boards"]:
        if board["target"] == args.chip and board["interface"] == args.interface:
            logging.info("Found target entry: %s", board)
            assert board["location"]
            sys.exit(0)
    logging.critical("Target entry for \"%s/%s\" was not found", args.chip, args.interface)
    sys.exit(1)


if __name__ == '__main__':
    main()
