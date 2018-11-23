#!/usr/bin/env python

import os, sys
import re
import logging
import argparse
import unittest
import tempfile
import cProfile, pstats
import debug_backend, debug_backend_tests


BOARD_TCL_CONFIG = {
    'esp-wrover-kit' :  {
        'files' : [
            os.path.join('interface', 'ftdi', 'esp32_devkitj_v1.cfg'),
            os.path.join('board', 'esp-wroom-32.cfg')
        ],
        'commands' : []
    },
    'esp32_solo-devkitj' :  {
        'files' : [
            os.path.join('interface', 'ftdi', 'esp32_devkitj_v1.cfg'),
            os.path.join('board', 'esp32-solo-1.cfg')
        ],
        'commands' : []
    }
}

def main():
    log_formatter = logging.Formatter('%(asctime)-15s %(name)s: %(levelname)s - %(message)s')
    ch = logging.StreamHandler()
    ch.setFormatter(log_formatter)
    ch.setLevel(logging.WARNING)
    fh = None
    if args.log_file:
        if args.log_file == 'stdout':
            fh = ch
        else:
            fh = logging.FileHandler(args.log_file, 'w')
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
    board_tcl = BOARD_TCL_CONFIG[args.board_type]
    # start debugger, ideally we should run all tests w/o restarting it
    debug_backend.start(args.toolchain, args.oocd, args.oocd_tcl, board_tcl['files'], board_tcl['commands'])
    debug_backend_tests.test_apps_dir = args.apps_dir
    # loads tests using pattern <module>[.<test_case>[.<test_method>]] with wildcards (*) in <module> and <test_case> parts
    def load_tests_by_pattern(search_dir, pattern):
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

    # run tests from the same directory this file is
    loader = unittest.TestLoader()
    loader.suiteClass = debug_backend_tests.DebuggerTestsBunch
    if not isinstance(args.pattern, list):
        tests_patterns = [args.pattern, ]
    else:
        tests_patterns = args.pattern
    suite = None
    for pattern in tests_patterns:
        pattern_suite = load_tests_by_pattern(os.path.dirname(__file__), pattern)
        if suite:
            suite.addTest(pattern_suite)
        else:
            suite = pattern_suite
    # setup loggers in test modules
    for m in suite.modules:
        setup_logger(suite.modules[m].get_logger(), ch, fh)
    suite.load_app_bins = not args.no_load
    try: 
        res = unittest.TextTestRunner(verbosity=2).run(suite)
        if not res.wasSuccessful() and args.retry:
            print '==========================================='
            print 'Re-run failed tests. Give a second chance.'
            print '==========================================='
            # restart debugger
            debug_backend.stop()
            debug_backend.start(args.toolchain, args.oocd, args.oocd_tcl, board_tcl['files'], board_tcl['commands'])
            err_suite = debug_backend_tests.DebuggerTestsBunch()
            for e in res.errors:
                err_suite.addTest(e[0])
            for f in res.failures:
                err_suite.addTest(f[0])
            res = unittest.TextTestRunner(verbosity=2).run(err_suite)
    finally:
        # stop debugger
        debug_backend.stop()
    # check results
    if not res.wasSuccessful():
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
                        choices=['esp-wrover-kit', 'esp32_solo-devkitj'],
                        default=os.environ.get('OOCD_TEST_BOARD', 'esp-wrover-kit'))
    parser.add_argument('--apps-dir', '-a',
                        help='Path to test apps',
                        default=os.environ.get('OOCD_TEST_APPS_DIR', os.path.join(os.getcwd(), 'testing', 'esp', 'test_apps')))
    parser.add_argument('--pattern', '-p', nargs='*',
                        help="""Pattern of test cases to run. Format: <module>[.<test_case>[.<test_method>]].
                                User can specify several strings separated by space. Wildcards (*) are supported in <module> and <test_case> parts""",
                        default='*')
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
                        help='Debug level (0-4)', 
                        type=int, default=2)
    parser.add_argument('--log-file', '-l',
                        help='Path to log file. Use "stdout" to log to console.')
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
