#!/usr/bin/env python

import sys
import os
import logging
import argparse
import unittest
import debug_backend
import debug_backend_tests
import test_exec_ctrl
import test_bp
import test_step
from pprint import pprint


BOARD_TCL_FILES = {
'esp-wrover-kit' : [
        os.path.join('interface', 'ftdi', 'esp32_devkitj_v1.cfg'),
        os.path.join('board', 'esp-wroom-32.cfg')
    ]
}

def main():
    parser = argparse.ArgumentParser(description='run_tests.py - Run auto-tests', prog='run_tests')

    parser.add_argument('--gdb', '-g',
                        help='Path to GDB binary',
                        default=os.environ.get('OOCD_TEST_GDB_BIN_PATH', 'xtensa-esp32-elf-gdb'))
    parser.add_argument('--oocd', '-o',
                        help='Path to OpenOCD binary',
                        default=os.environ.get('OOCD_TEST_BIN_PATH', os.path.join(os.getcwd(), 'src', 'openocd')))
    parser.add_argument('--oocd-tcl', '-s',
                        help='Path to OpenOCD TCL scripts',
                        default=os.environ.get('OOCD_TEST_TCL_DIR', os.path.join(os.getcwd(), 'tcl')))
    parser.add_argument('--board-type', '-b',
                        help='Type of the board to run tests on',
                        choices=['esp-wrover-kit'],
                        default=os.environ.get('OOCD_TEST_BOARD', 'esp-wrover-kit'))
    parser.add_argument('--apps-dir', '-a',
                        help='Path to test apps',
                        default=os.environ.get('OOCD_TEST_APPS_DIR', os.path.join(os.getcwd(), 'testing', 'esp', 'test_apps')))
    parser.add_argument('--pattern', '-p', nargs='*',
                        help='Pattern of test cases to run. Format: <module>.<test_case>.<test_method>. Several strings can be specified.',
                        default='*')
    parser.add_argument('--debug', '-d',
                        help='Debug level (0-4)', 
                        type=int, default=2)
    parser.add_argument('--log-file', '-l',
                        help='Path to log file')
    args = parser.parse_args()

    log_formatter = logging.Formatter('%(name)s: %(levelname)s - %(message)s')
    ch = logging.StreamHandler()
    ch.setFormatter(log_formatter)
    ch.setLevel(logging.WARNING)
    fh = None
    if args.log_file:
        fh = logging.FileHandler(args.log_file)
        fh.setFormatter(log_formatter)
        if args.debug == 0:
            fh.setLevel(logging.CRITICAL)
        elif args.debug == 1:
            fh.setLevel(logging.ERROR)
        elif args.debug == 2:
            fh.setLevel(logging.WARNING)
        elif args.debug == 3:
            fh.setLevel(logging.INFO)
        else:
            fh.setLevel(logging.DEBUG)
    # config log levels in modules
    def setup_logger(logger, conh, fileh):
        logger.setLevel(logging.DEBUG)
        logger.addHandler(conh)
        if fileh:
            logger.addHandler(fileh)
    setup_logger(debug_backend.Oocd.get_logger(), ch, fh)
    setup_logger(debug_backend.Gdb.get_logger(), ch, fh)
    setup_logger(debug_backend_tests.get_logger(), ch, fh)
    # start debugger, ideally we should run all tests w/o restarting it
    debug_backend.start(args.gdb, args.oocd, args.oocd_tcl, BOARD_TCL_FILES[args.board_type])
    debug_backend_tests.set_apps_dir(args.apps_dir)
    # run tests from the same directory this file is
    loader = unittest.TestLoader()
    loader.suiteClass = debug_backend_tests.DebuggerTestsBunch
    if isinstance(args.pattern, list):
        suite = loader.loadTestsFromNames(args.pattern)
    elif args.pattern != '*':
        suite = loader.loadTestsFromName(args.pattern)
    else:
        suite = loader.discover(os.path.dirname(__file__))
    # setup loggers in test modules
    for m in suite.modules:
        setup_logger(suite.modules[m].get_logger(), ch, fh)
    try:  
        res = unittest.TextTestRunner(verbosity=2).run(suite)
    finally:
        # stop debugger
        debug_backend.stop()
    # check results
    if len(res.errors) or len(res.failures):
        sys.exit(-1)

if __name__ == '__main__':
    main()
