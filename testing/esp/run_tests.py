#!/usr/bin/env python3

import os, sys
import re
import logging
import argparse
import unittest
import tempfile
import serial
import threading
import xmlrunner
import cProfile, pstats
import debug_backend as dbg
import debug_backend_tests
import traceback

_oocd_inst = None
_gdb_inst = None

# Keys in this map are special names for boards to run tests on
# tets can be declared to be sckipped for some types boards using 'skip_for_hw_id()' decorator.
# For usage example see 'test_special.PsramTestsSingle'
BOARD_TCL_CONFIG = {
    'esp32-wrover-kit-3.3v' :  {
        'files' : [
            os.path.join('board', 'esp32-wrover-kit-3.3v.cfg')
        ],
        'commands' : [],
        'chip_name' : 'esp32'
    },
    'esp32-solo-devkitj' :  {
        'files' : [
            os.path.join('interface', 'ftdi', 'esp32_devkitj_v1.cfg'),
            os.path.join('target', 'esp32-solo-1.cfg')
        ],
        'commands' : [],
        'chip_name' : 'esp32-solo'
    },
    'esp32s2-devkitj' :  {
        'files' : [
            os.path.join('interface', 'ftdi', 'esp32_devkitj_v1.cfg'),
            os.path.join('target', 'esp32s2.cfg')
        ],
        'commands' : [],
        'chip_name' : 'esp32s2'
    }
}

class SerialPortReader(threading.Thread):
    def __init__(self, port_name):
        threading.Thread.__init__(self, name='serial_reader')
        # connect to serial port
        self.ser = serial.serial_for_url(port_name, do_not_open=True)
        self.ser.baudrate = 115200
        # self.ser.parity = serial.PARITY_NONE
        self.ser.dtr = False
        self.ser.rts = False
        # self.ser.rtscts = False
        # self.ser.xonxoff = False
        self.ser.timeout = 0
        self.ser.open()
        self.do_work = True
        self._logger = logging.getLogger('BOARD_UART')

    def get_logger(self):
        return self._logger

    def stop(self):
        self.do_work = False
        self._logger.debug('Wait for reader thread to finish...')
        self.join()
        self.ser.close()
        self._logger.debug('Reader thread to finished')

    def run(self):
        self._logger.debug('Start reading from "{}"'.format(self.ser.name))
        line = b''
        while self.do_work:
            line += self.ser.read(1)
            if line.endswith(b'\n'):
                self._logger.info(line.rstrip(b'\r\n'))
                line = b''


def dbg_start(toolchain, oocd, oocd_tcl, oocd_cfg_files, oocd_cfg_cmds, debug_oocd,
              chip_name, log_level, log_stream, log_file):
    global _oocd_inst, _gdb_inst
    connect_tmo = 5 if debug_oocd <= 2 else 15
    # Start OpenOCD
    _oocd_inst = dbg.create_oocd(chip_name=chip_name,
                        oocd_exec=oocd,
                        oocd_scripts=oocd_tcl,
                        oocd_cfg_files=oocd_cfg_files,
                        oocd_cfg_cmds=oocd_cfg_cmds,
                        oocd_debug=debug_oocd,
                        log_level=log_level,
                        log_stream_handler=log_stream,
                        log_file_handler=log_file)
    _oocd_inst.start()
    try:
        # reset the board if it is stuck from the previous test run
        _oocd_inst.cmd_exec('reset halt')
        # Start GDB
        _gdb_inst = dbg.create_gdb(chip_name=chip_name,
                            gdb_path='%sgdb' % toolchain,
                            remote_target='127.0.0.1:%d' % dbg.Oocd.GDB_PORT,
                            log_level=log_level,
                            log_stream_handler=log_stream,
                            log_file_handler=log_file)
        _gdb_inst.connect(tmo=connect_tmo)
    except Exception as e:
        _oocd_inst.stop()
        raise e


def dbg_stop():
    global _oocd_inst, _gdb_inst
    _oocd_inst.stop()
    print("Debug backend finished")


# config log levels in modules
def setup_logger(logger, conh, fileh, lev):
    logger.setLevel(lev)
    fileh.setLevel(lev)
    logger.addHandler(conh)
    if fileh:
        logger.addHandler(fileh)
    logger.propagate = False


# loads tests using pattern <module>[.<test_case>[.<test_method>]] with wildcards (*) in <module> and <test_case> parts
def load_tests_by_pattern(loader, search_dir, pattern):
    if pattern.find('*') == -1:
        return loader.loadTestsFromName(pattern)
    parts = pattern.split('.')
    if len(parts) == 1:
        # '*' or 'te*' or test_*' etc
        return loader.discover(search_dir, '%s.py' % parts[0])
    if parts[0].find('*') == -1:
        # if module part w/o wildcard
        # 'xxx.??*'
        mod_tests = loader.loadTestsFromName(parts[0])
    else:
        # if module part has wildcard
        # 'xx*.???'
        mod_tests = loader.discover(search_dir, '%s.py' % parts[0])
    if parts[1].find('*') != -1:
        case_name_pattern = '^%s$' % parts[1].replace('*', '.*')
    else:
        case_name_pattern = '^%s$' % parts[1]
    suite = debug_backend_tests.DebuggerTestsBunch()
    # look for concrete matched test cases
    for test in mod_tests:
        if not issubclass(type(test), debug_backend_tests.DebuggerTestsBase):
            continue
        if re.match(case_name_pattern, test.__class__.__name__):
            if len(parts) == 2 or parts[2] == '*':
                suite.addTest(test)
            elif hasattr(test, parts[2]):
                test_method = getattr(test, parts[2])
                test1 = type(test)(test_method.__name__)
                suite.addTest(debug_backend_tests.DebuggerTestsBunch([test1]))
    return suite

# excludes tests using pattern <module>[.<test_case>[.<test_method>]] with wildcards (*) in <module> and <test_case> parts
def exclude_tests_by_patterns(suite, patterns):
    for test in suite:
        if not issubclass(type(test), debug_backend_tests.DebuggerTestsBase):
            continue
        for pattern in patterns:
            parts = pattern.split('.')
            if len(parts) == 1:
                pattern = '%s.*.*' % parts[0]
            elif len(parts) == 2:
                pattern = '%s.%s.*' % (parts[0], parts[1])
            re_pattern = '^%s$' % pattern.replace('.', '\.').replace('*', '.*')
            if re.match(re_pattern, test.id()):
                setattr(test, 'setUp', lambda: test.skipTest('Excluded by pattern'))
                break
    return suite

def main():
    board_uart_reader = None
    if args.serial_port:
        try:
            board_uart_reader = SerialPortReader(args.serial_port)
        except serial.SerialException as e:
            sys.stderr.write('Could not start reader for serial port {}: {}\n'.format(args.serial_port, e))
            sys.exit(1)
    log_formatter = logging.Formatter('%(asctime)-15s %(name)s: %(levelname)s - %(message)s')
    ch = logging.StreamHandler()
    ch.setFormatter(log_formatter)
    ch.setLevel(logging.WARNING)
    fh = None
    log_lev = logging.WARNING
    if args.log_file:
        if args.log_file == 'stdout':
            fh = ch
        else:
            fh = logging.FileHandler(args.log_file, 'w')
        fh.setFormatter(log_formatter)
        if args.debug == 0:
            log_lev = logging.CRITICAL
        elif args.debug == 1:
            log_lev = logging.ERROR
        elif args.debug == 2:
            log_lev = logging.WARNING
        elif args.debug == 3:
            log_lev = logging.INFO
        else:
            log_lev = logging.DEBUG

    setup_logger(debug_backend_tests.get_logger(), ch, fh, log_lev)
    if board_uart_reader:
        setup_logger(board_uart_reader.get_logger(), ch, fh, log_lev)
        board_uart_reader.start()
    board_tcl = BOARD_TCL_CONFIG[args.board_type]

    # init testee info
    debug_backend_tests.testee_info.hw_id = args.board_type
    if args.idf_ver_min != 'auto':
        debug_backend_tests.testee_info.idf_ver = debug_backend_tests.IdfVersion.fromstr(args.idf_ver_min)
    debug_backend_tests.test_apps_dir = args.apps_dir

    if args.test_runner == 'x':
        test_runner = xmlrunner.XMLTestRunner(verbosity=2, output=args.test_outdir)
    elif args.test_runner == 't':
        test_runner = unittest.TextTestRunner(verbosity=2)
    else:
        print("Wrong test-runner argument")
        return

    # start debugger, ideally we should run all tests w/o restarting it
    dbg_start(args.toolchain, args.oocd, args.oocd_tcl, board_tcl['files'], board_tcl['commands'],
                        args.debug_oocd, board_tcl['chip_name'], log_lev, ch, fh)
    res = None
    try:
        # run tests from the same directory this file is
        loader = unittest.TestLoader()
        loader.suiteClass = debug_backend_tests.DebuggerTestsBunch
        # load tests by patterns
        if not isinstance(args.pattern, list):
            tests_patterns = [args.pattern, ]
        else:
            tests_patterns = args.pattern
        suite = None
        for pattern in tests_patterns:
            pattern_suite = load_tests_by_pattern(loader, os.path.dirname(__file__), pattern)
            if suite:
                suite.addTest(pattern_suite)
            else:
                suite = pattern_suite
        # exclude tests by patterns
        if not isinstance(args.exclude, list):
            tests_exclude = [args.exclude, ]
        else:
            tests_exclude = args.exclude
        suite = exclude_tests_by_patterns(suite, tests_exclude)
        # setup loggers in test modules
        for m in suite.modules:
            setup_logger(suite.modules[m].get_logger(), ch, fh, log_lev)
        suite.load_app_bins = not args.no_load
        global _oocd_inst, _gdb_inst
        suite.config_tests(_oocd_inst, _gdb_inst, args.toolchain)
        # RUN TESTS
        res = test_runner.run(suite)
        if not res.wasSuccessful() and args.retry:
            print("===========================================")
            print("Re-run failed tests. Give a second chance.")
            print("===========================================")
            # restart debugger
            dbg_stop()
            dbg_start(args.toolchain, args.oocd, args.oocd_tcl, board_tcl['files'], board_tcl['commands'],
                                args.debug_oocd, board_tcl['chip_name'], log_lev, ch, fh)
            err_suite = debug_backend_tests.DebuggerTestsBunch()
            for e in res.errors:
                err_suite.addTest(e[0])
            for f in res.failures:
                err_suite.addTest(f[0])
            res = test_runner.run(err_suite)
    except:
        traceback.print_exc()
    finally:
        # stop debugger
        dbg_stop()
        if board_uart_reader:
           board_uart_reader.stop()
    # check results
    if not res or not res.wasSuccessful():
        sys.exit(-1)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='run_tests.py - Run auto-tests', prog='run_tests')

    parser.add_argument('--toolchain', '-t',
                        help='Toolchain prefix',
                        default=os.environ.get('OOCD_TEST_GDB_BIN_PATH', 'xtensa-esp32-elf-'))
    parser.add_argument('--oocd', '-o',
                        help='Path to OpenOCD binary',
                        default=os.environ.get('OOCD_TEST_BIN_PATH', os.path.join(os.getcwd(), 'src', 'openocd')))
    parser.add_argument('--oocd-tcl', '-s',
                        help='Path to OpenOCD TCL scripts',
                        default=os.environ.get('OOCD_TEST_TCL_DIR', os.path.join(os.getcwd(), 'tcl')))
    parser.add_argument('--board-type', '-b',
                        help='Type of the board to run tests on',
                        choices=list(BOARD_TCL_CONFIG.keys()),
                        default=os.environ.get('OOCD_TEST_BOARD', 'esp32-wrover-kit-3.3v'))
    parser.add_argument('--apps-dir', '-a',
                        help='Path to test apps',
                        default=os.environ.get('OOCD_TEST_APPS_DIR', os.path.join(os.getcwd(), 'testing', 'esp', 'test_apps')))
    parser.add_argument('--pattern', '-p', nargs='*',
                        help="""Pattern of test cases to run. Format: <module>[.<test_case>[.<test_method>]].
                                User can specify several strings separated by space. Wildcards (*) are supported in <module> and <test_case> parts""",
                        default='test_*')
    parser.add_argument('--exclude', '-e', nargs='*',
                        help="""Pattern of test cases to exclude. Format: <module>[.<test_case>[.<test_method>]].
                                User can specify several strings separated by space. Wildcards (*) are supported in <module> and <test_case> parts""",
                        default='')
    parser.add_argument('--no-load', '-n',
                        help='Do not load test app binaries',
                        action='store_true', default=False)
    parser.add_argument('--retry', '-r',
                        help='Try to re0run failed tests',
                        action='store_true', default=False)
    parser.add_argument('--stats-file', '-k',
                        help='Path to log file to store profiler stats. Use "stdout" to print.',
                        default='')
    parser.add_argument('--debug', '-d',
                        help='Verbosity level (0-4)',
                        type=int, default=2)
    parser.add_argument('--debug-oocd', '-w',
                        help='OpenOCD verbosity level (0-3)',
                        type=int, default=2)
    parser.add_argument('--log-file', '-l',
                        help='Path to log file. Use "stdout" to log to console.')
    parser.add_argument('--serial-port', '-u',
                        help='Name of serial port to grab board\'s UART output.')
    parser.add_argument('--idf-ver-min', '-i',
                        help='Minimal IDF version to run tests for. Format: x[.y[.z]]. Use "latest" to run all tests. Use "auto" to read version from target.',
                        type=str, default='auto')
    parser.add_argument('--test-runner', '-tr',
                        help='x - for for XMLTestRunner, t - for TextTestRunner', type=str, default='t')
    parser.add_argument('--test-outdir', '-to',
                        help='Output dir for runners needed to it',
                        type=str, default='./results')
    args = parser.parse_args()
    if len(args.stats_file) > 0:
        if args.stats_file == 'stdout':
            fhnd,fname = tempfile.mkstemp()
            fbin = os.fdopen(fhnd, 'wb')
        else:
            fname = args.stats_file
        cProfile.run('main()', fname)
        if args.stats_file == 'stdout':
            stats = pstats.Stats(fname)
            stats.print_stats()
            fbin.close()
    else:
        main()
