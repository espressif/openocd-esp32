#!/usr/bin/env python3

import os, sys
import re
import logging
import argparse
import unittest
import tempfile
import serial
import threading
import time
import xmlrunner
import cProfile, pstats
import debug_backend as dbg
import debug_backend_tests
import traceback

_oocd_inst = None
_gdb_inst = None

# Keys in this map are special names for boards to run tests on
# tests can be declared to be sckipped for:
# - some types of boards using 'skip_for_hw_id()' decorator.
# - some types of chips using 'skip_for_chip()' decorator.
# - some types of archs using 'skip_for_arch()' decorator.
# For usage example see 'test_special.PsramTestsSingle'
BOARD_TCL_CONFIG = {
    'esp32-wrover-kit-3.3v' :  {
        'files' : [
            os.path.join('board', 'esp32-wrover-kit-3.3v.cfg')
        ],
        'commands' : [],
        'chip_name' : 'esp32',
        'target_triple' : 'xtensa-esp32-elf'
    },
    'esp32-solo-devkitj' :  {
        'files' : [
            os.path.join('interface', 'ftdi', 'esp_ftdi.cfg'),
            os.path.join('target', 'esp32-solo-1.cfg')
        ],
        'commands' : [],
        'chip_name' : 'esp32-solo',
        'target_triple' : 'xtensa-esp32-elf'
    },
    'esp32s2-devkitj' :  {
        'files' : [
            os.path.join('interface', 'ftdi', 'esp_ftdi.cfg'),
            os.path.join('target', 'esp32s2.cfg')
        ],
        'commands' : [],
        'chip_name' : 'esp32s2',
        'target_triple' : 'xtensa-esp32s2-elf'
    },
    'esp32s2-kaluga-1' :  {
        'files' : [
            os.path.join('board', 'esp32s2-kaluga-1.cfg')
        ],
        'commands' : [],
        'chip_name' : 'esp32s2',
        'target_triple' : 'xtensa-esp32s2-elf'
    },
    'esp32c2-ftdi' :  {
        'files' : [
            os.path.join('board', 'esp32c2-ftdi.cfg')
        ],
        'commands' : [],
        'chip_name' : 'esp32c2',
        'target_triple' : 'riscv32-esp-elf'
    },
    'esp32c3-ftdi' :  {
        'files' : [
            os.path.join('board', 'esp32c3-ftdi.cfg')
        ],
        'commands' : [],
        'chip_name' : 'esp32c3',
        'target_triple' : 'riscv32-esp-elf'
    },
    'esp32c3-builtin' :  {
        'files' : [
            os.path.join('board', 'esp32c3-builtin.cfg')
        ],
        'commands' : [],
        'chip_name' : 'esp32c3',
        'target_triple' : 'riscv32-esp-elf'
    },
    'esp32c5-ftdi' :  {
        'files' : [
            os.path.join('board', 'esp32c5-ftdi.cfg')
        ],
        'commands' : [],
        'chip_name' : 'esp32c5',
        'target_triple' : 'riscv32-esp-elf'
    },
    'esp32c5-builtin' :  {
        'files' : [
            os.path.join('board', 'esp32c5-builtin.cfg')
        ],
        'commands' : [],
        'chip_name' : 'esp32c5',
        'target_triple' : 'riscv32-esp-elf'
    },
    'esp32c5-lpcore-ftdi' :  {
        'files' : [
            os.path.join('board', 'esp32c5-lpcore-ftdi.cfg')
        ],
        'commands' : [],
        'chip_name' : 'esp32c5',
        'target_triple' : 'riscv32-esp-elf'
    },
    'esp32c5-lpcore-builtin' :  {
        'files' : [
            os.path.join('board', 'esp32c5-lpcore-builtin.cfg')
        ],
        'commands' : [],
        'chip_name' : 'esp32c5',
        'target_triple' : 'riscv32-esp-elf'
    },
    'esp32c6-ftdi' :  {
        'files' : [
            os.path.join('board', 'esp32c6-ftdi.cfg')
        ],
        'commands' : [],
        'chip_name' : 'esp32c6',
        'target_triple' : 'riscv32-esp-elf'
    },
    'esp32c6-builtin' :  {
        'files' : [
            os.path.join('board', 'esp32c6-builtin.cfg')
        ],
        'commands' : [],
        'chip_name' : 'esp32c6',
        'target_triple' : 'riscv32-esp-elf'
    },
    'esp32c6-lpcore-ftdi' :  {
        'files' : [
            os.path.join('board', 'esp32c6-lpcore-ftdi.cfg')
        ],
        'commands' : [],
        'chip_name' : 'esp32c6',
        'target_triple' : 'riscv32-esp-elf'
    },
    'esp32c6-lpcore-builtin' :  {
        'files' : [
            os.path.join('board', 'esp32c6-lpcore-builtin.cfg')
        ],
        'commands' : [],
        'chip_name' : 'esp32c6',
        'target_triple' : 'riscv32-esp-elf'
    },
    'esp32c61-ftdi' :  {
        'files' : [
            os.path.join('board', 'esp32c61-ftdi.cfg')
        ],
        'commands' : [],
        'chip_name' : 'esp32c61',
        'target_triple' : 'riscv32-esp-elf'
    },
    'esp32c61-builtin' :  {
        'files' : [
            os.path.join('board', 'esp32c61-builtin.cfg')
        ],
        'commands' : [],
        'chip_name' : 'esp32c61',
        'target_triple' : 'riscv32-esp-elf'
    },
    'esp32h2-ftdi' :  {
        'files' : [
            os.path.join('board', 'esp32h2-ftdi.cfg')
        ],
        'commands' : [],
        'chip_name' : 'esp32h2',
        'target_triple' : 'riscv32-esp-elf'
    },
    'esp32h2-builtin' :  {
        'files' : [
            os.path.join('board', 'esp32h2-builtin.cfg')
        ],
        'commands' : [],
        'chip_name' : 'esp32h2',
        'target_triple' : 'riscv32-esp-elf'
    },
    'esp32h4-ftdi' :  {
        'files' : [
            os.path.join('board', 'esp32h4-ftdi.cfg')
        ],
        'commands' : [],
        'chip_name' : 'esp32h4',
        'target_triple' : 'riscv32-esp-elf'
    },
    'esp32h4-builtin' :  {
        'files' : [
            os.path.join('board', 'esp32h4-builtin.cfg')
        ],
        'commands' : [],
        'chip_name' : 'esp32h4',
        'target_triple' : 'riscv32-esp-elf'
    },
    'esp32p4-ftdi' :  {
        'files' : [
            os.path.join('board', 'esp32p4-ftdi.cfg')
        ],
        'commands' : [],
        'chip_name' : 'esp32p4',
        'target_triple' : 'riscv32-esp-elf'
    },
    'esp32p4-builtin' :  {
        'files' : [
            os.path.join('board', 'esp32p4-builtin.cfg')
        ],
        'commands' : [],
        'chip_name' : 'esp32p4',
        'target_triple' : 'riscv32-esp-elf'
    },
    'esp32p4-lpcore-ftdi' :  {
        'files' : [
            os.path.join('board', 'esp32p4-lpcore-ftdi.cfg')
        ],
        'commands' : [],
        'chip_name' : 'esp32p4',
        'target_triple' : 'riscv32-esp-elf'
    },
    'esp32p4-lpcore-builtin' :  {
        'files' : [
            os.path.join('board', 'esp32p4-lpcore-builtin.cfg')
        ],
        'commands' : [],
        'chip_name' : 'esp32p4',
        'target_triple' : 'riscv32-esp-elf'
    },
    'esp32s3-ftdi' :  {
        'files' : [
            os.path.join('board', 'esp32s3-ftdi.cfg')
        ],
        'commands' : [],
        'chip_name' : 'esp32s3',
        'target_triple' : 'xtensa-esp32s3-elf'
    },
    'esp32s3-builtin' :  {
        'files' : [
            os.path.join('board', 'esp32s3-builtin.cfg')
        ],
        'commands' : [],
        'chip_name' : 'esp32s3',
        'target_triple' : 'xtensa-esp32s3-elf'
    },
}

class SerialPortReader(threading.Thread):
    def __init__(self, port_name):
        self._logger = logging.getLogger('BOARD_UART')
        threading.Thread.__init__(self, name='serial_reader')
        self.port_name = port_name
        # connect to serial port
        self.ser = serial.serial_for_url(port_name, do_not_open=True)
        self.ser.baudrate = 115200
        # self.ser.parity = serial.PARITY_NONE
        self.ser.dtr = False
        self.ser.rts = False
        # self.ser.rtscts = False
        # self.ser.xonxoff = False
        self.ser.timeout = 0

    def get_logger(self):
        return self._logger

    def is_connected(self):
        self.ser.is_open()

    def start(self):
        self.ser.open()
        self.do_work = True
        self.paused = False
        threading.Thread.start(self)

    def stop(self):
        self.do_work = False
        self._logger.debug('Wait for reader thread to finish...')
        self.join()
        self.ser.close()
        self._logger.debug('Reader thread to finished')

    def pause(self):
        self.paused = True

    def resume(self):
        self.paused = False

    def run(self):
        self._logger.debug('Start reading from "{}"'.format(self.ser.name))
        line = b''
        while self.do_work:
            if not self.paused:
                try:
                    line += self.ser.read(1)
                except serial.SerialException:
                    line += b'<exc>'
                if line.endswith(b'\n'):
                    self._logger.info(line.rstrip(b'\r\n'))
                    line = b''


def dbg_start(toolchain, oocd, oocd_tcl, oocd_cfg_files, oocd_cfg_cmds, debug_oocd,
              chip_name, target_triple, log_level, log_stream, log_file, gdb_log, no_gdb):
    global _oocd_inst, _gdb_inst
    connect_tmo = 15
    remote_tmo = 10
    # Start OpenOCD
    _oocd_inst = dbg.create_oocd(chip_name=chip_name,
                        target_triple=target_triple,
                        oocd_exec=oocd,
                        oocd_scripts=oocd_tcl,
                        oocd_cfg_files=oocd_cfg_files,
                        oocd_cfg_cmds=oocd_cfg_cmds,
                        oocd_debug=debug_oocd,
                        log_level=log_level,
                        log_stream_handler=log_stream,
                        log_file_handler=log_file)
    _oocd_inst.start()
    if no_gdb:
        return
    try:
        # reset the board if it is stuck from the previous test run
        _oocd_inst.cmd_exec('reset halt')
        # Enable GDB fix
        # TODO: Remove
        os.environ["ESP_XTENSA_GDB_PRIV_REGS_FIX"] = "1"
        gdb_utils = debug_backend_tests.GDBUtils()
        _gdb_inst = gdb_utils.create_gdb(chip_name, target_triple, toolchain, log_level,
                                            log_stream, log_file, gdb_log, debug_oocd)
        _gdb_inst.connect(tmo=connect_tmo)
    except Exception as e:
        _oocd_inst.stop()
        if type(e) == dbg.DebuggerTargetStateTimeoutError:
            sys.exit(os.EX_TEMPFAIL)
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
            if len(parts) == 2 or parts[2] in ['*', test._testMethodName]:
                suite.addTest(test)
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
                # '__esp_unittest_skip_reason__' flag is used by test suite to check if app binaries need to be flashed,
                # so we can skip individual tests w/o need for having binaries for them. See DebuggerTestsBunch.run() for details
                setattr(test, '__esp_unittest_skip_reason__', 'Excluded by pattern')
                break
    return suite

def main():
    board_uart_reader = None
    if args.log_uart:
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
        time.sleep(1)
    board_tcl = BOARD_TCL_CONFIG[args.board_type]
    board_tcl['commands'] = args.oocd_cmds.split(",")

    # init testee info
    debug_backend_tests.testee_info.hw_id = args.board_type
    debug_backend_tests.testee_info.chip = board_tcl['chip_name']
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
    try:
        dbg_start(args.toolchain, args.oocd, args.oocd_tcl, board_tcl['files'], board_tcl['commands'],
                            args.debug_oocd, board_tcl['chip_name'], board_tcl['target_triple'],
                            log_lev, ch, fh, args.gdb_log_folder, args.no_gdb)
    except RuntimeError:
        # flash an app and try again
        import json, subprocess
        output_dir = None
        for dir, _, files in os.walk(os.getcwd()):
            if dir.endswith('single_core') and 'flasher_args.json' in files:
                output_dir = dir
                break
        if output_dir is None:
            raise
        with open(os.path.join(output_dir, 'flasher_args.json'), 'rb') as f:
            json_args = json.load(f)
            # replace all arguments with'-' with '_', for compatibility with both esptool v4/v5
            flasher_args = [x.replace('-','_').replace('__','--') for x in json_args['write_flash_args']]
            for addr, bin in json_args['flash_files'].items():
                flasher_args += [addr, bin]
        if board_uart_reader:
            board_uart_reader.stop()
        cmd = ['esptool.py', '-p', args.serial_port, '--no-stub', 'write_flash', *flasher_args]
        proc = subprocess.run(cmd, cwd=output_dir)
        proc.check_returncode()
        # flashing succeeded, return special code EX_TEMPFAIL (75), configured in CI to retry the job
        sys.exit(os.EX_TEMPFAIL)
    res = None
    try:
        # run tests from the same directory this file is
        loader = unittest.TestLoader()
        loader.suiteClass = debug_backend_tests.DebuggerTestsBunch
        # load tests by patterns
        if not args.pattern:
            args.pattern = ['test_*']
        suite = None
        for pattern in args.pattern:
            pattern_suite = load_tests_by_pattern(loader, os.path.dirname(__file__), pattern)
            if suite:
                suite.addTest(pattern_suite)
            else:
                suite = pattern_suite
        # exclude tests by patterns
        if args.exclude:
            suite = exclude_tests_by_patterns(suite, args.exclude)
        # setup loggers in test modules
        for m in suite.modules:
            setup_logger(suite.modules[m].get_logger(), ch, fh, log_lev)
        suite.load_app_bins = not args.no_load
        global _oocd_inst, _gdb_inst
        arg_list = [args.debug_oocd, log_lev, args.gdb_log_folder, ch, fh]
        suite.config_tests(_oocd_inst, _gdb_inst, args.toolchain, board_uart_reader, args.serial_port, arg_list)
        # RUN TESTS
        res = test_runner.run(suite)
        if not res.wasSuccessful() and args.retry:
            print("===========================================")
            print("Re-run failed tests. Give a second chance.")
            print("===========================================")
            # restart debugger
            dbg_stop()
            time.sleep(1)
            dbg_start(args.toolchain, args.oocd, args.oocd_tcl, board_tcl['files'], board_tcl['commands'],
                                args.debug_oocd, board_tcl['chip_name'], board_tcl['target_triple'],
                                log_lev, ch, fh, args.gdb_log_folder, args.no_gdb)
            err_suite = debug_backend_tests.DebuggerTestsBunch()

            if not board_uart_reader:
                try:
                    board_uart_reader = SerialPortReader(args.serial_port)
                    setup_logger(board_uart_reader.get_logger(), ch, fh, log_lev)
                    board_uart_reader.start()
                    time.sleep(1)
                except serial.SerialException as e:
                    sys.stderr.write('Could not start reader for serial port {}: {}\n'.format(args.serial_port, e))
                    board_uart_reader = None

            ids = [x[0].id() for x in res.errors + res.failures]
            for t in suite._tests:
                if t.id() in ids:
                    err_suite.addTest(t)
            err_suite.load_app_bins = not args.no_load
            arg_list = [args.debug_oocd, log_lev, args.gdb_log_folder, ch, fh]
            err_suite.config_tests(_oocd_inst, _gdb_inst, args.toolchain, board_uart_reader, args.serial_port, arg_list)

            # to output new report instead of overwriting previous one
            if args.test_runner == 'x':
                test_runner.outsuffix += "_retry"

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

class ExtendAction(argparse.Action):
    def __call__(self, parser, namespace, values, option_string=None):
        items = getattr(namespace, self.dest) or []
        items.extend(values)
        setattr(namespace, self.dest, items)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='run_tests.py - Run auto-tests', prog='run_tests')
    parser.register('action', 'extend', ExtendAction)

    parser.add_argument('--toolchain', '-t',
                        help='Toolchain prefix',
                        default=os.environ.get('OOCD_TEST_GDB_BIN_PATH', 'xtensa-esp32-elf-'))
    parser.add_argument('--oocd', '-o',
                        help='Path to OpenOCD binary',
                        default=os.environ.get('OOCD_TEST_BIN_PATH', os.path.join(os.getcwd(), 'src', 'openocd')))
    parser.add_argument('--oocd-tcl', '-s',
                        help='Path to OpenOCD TCL scripts',
                        default=os.environ.get('OOCD_TEST_TCL_DIR', os.path.join(os.getcwd(), 'tcl')))
    parser.add_argument('--oocd-cmds', '-c',
                        help='Additional, comma separated, OpenOCD commands',
                        default='')
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
                        action='extend', default=[])
    parser.add_argument('--exclude', '-e', nargs='*',
                        help="""Pattern of test cases to exclude. Format: <module>[.<test_case>[.<test_method>]].
                                User can specify several strings separated by space. Wildcards (*) are supported in <module> and <test_case> parts""",
                        action='extend', default=[])
    parser.add_argument('--no-load', '-n',
                        help='Do not load test app binaries',
                        action='store_true', default=False)
    parser.add_argument('--no-gdb', '-ng',
                        help='Do not connect GDB',
                        action='store_true', default=False)
    parser.add_argument('--retry', '-r',
                        help='Try to rerun failed tests',
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
    parser.add_argument('--gdb-log-folder', '-gl',
                        help='Path to folder for GDB log files.', default='')
    parser.add_argument('--serial-port', '-u',
                        help='Name of serial port to grab board\'s UART output.')
    parser.add_argument('--log-uart', '-lu',
                        help='Connect to UART and log data from it.',
                        action='store_true', default=False)
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
