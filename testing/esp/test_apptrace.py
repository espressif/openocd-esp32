import logging
import unittest
import debug_backend as dbg
from debug_backend_tests import *
import os
import os.path
import re
import time
import tempfile
import sys
import traceback
from time import sleep

idf_path = os.getenv('IDF_PATH')
if idf_path:
    sys.path.append(os.path.join(idf_path, 'tools', 'esp_app_trace'))

from espytrace.apptrace import reader_create, ReaderTimeoutError


def get_logger():
    return logging.getLogger(__name__)


########################################################################
#                         TESTS IMPLEMENTATION                         #
########################################################################
@skip_for_chip(['esp32s3'])
class ApptraceTestsImpl:
    """
    Tests for "raw" apptrace.
    """

    def setUp(self):
        pass

    def tearDown(self):
        pass

    def test_apptrace_dest_tcp(self):
        self.select_sub_test(503)
        self.add_bp('raw_trace_log_done')
        trace_src = 'tcp://localhost:53535'
        reader = reader_create(trace_src, 1.0)

        self.gdb.apptrace_start("%s" % trace_src)
        self.resume_exec()
        rsn = self.gdb.wait_target_state(dbg.TARGET_STATE_STOPPED, 5)
        self.assertEqual(rsn, dbg.TARGET_STOP_REASON_BP)
        self.gdb.apptrace_stop()
        lines = []
        while True:
            try:
                lines.append(reader.readline())
            except ReaderTimeoutError:
                break
        reader.cleanup()
        self.assertEqual(len(lines), 10)
        for i, line in enumerate(lines):
            self.assertEqual(line, "[%d %s]\n" % (i, " " * (i * 20)))

    def test_apptrace_autostop(self):
        self.select_sub_test(504)
        trace_file = tempfile.NamedTemporaryFile(delete=False)
        trace_file_name = trace_file.name
        trace_file.close()
        trace_src = 'file://%s' % trace_file_name
        reader = reader_create(trace_src, 1.0)
        # 0 ms poll period, stop when 9000 bytes are received or due to 5 s timeout
        self.oocd.apptrace_start("%s 0 10000 5" % trace_src)
        self.resume_exec()
        self.oocd.apptrace_wait_stop(tmo=30)
        lines = []
        while True:
            try:
                lines.append(reader.readline())
            except ReaderTimeoutError:
                break
        reader.cleanup()
        for i, line in enumerate(lines):
            self.assertEqual(line, "[%d %s]\n" % (i, " " * (i * 20)))
        os.remove(trace_file_name)

    def test_apptrace_reset(self):
        """
            This test checks that apptracing continue to work if target resets between start and stop
        """
        self.select_sub_test(505)
        trace_file = tempfile.NamedTemporaryFile(delete=False)
        trace_file_name = trace_file.name
        trace_file.close()
        trace_src = 'file://%s' % trace_file_name
        reader = reader_create(trace_src, 1.0)
        # 0 ms poll period, stop when 400 bytes are received or due to 3 s timeout
        self.oocd.apptrace_start("%s 0 400 3" % trace_src)
        self.resume_exec()
        sleep(1) #  let it works some time
        self.stop_exec()
        self.gdb.target_reset()

        print("lines before reset")

        lines_before_reset = []
        while True:
            try:
                line = reader.readline()
                if (len(line)):
                    lines_before_reset.append(line)
                    print(line)
            except ReaderTimeoutError:
                break

        self.gdb.add_bp('app_main')
        self.run_to_bp(dbg.TARGET_STOP_REASON_BP, 'app_main')
        self.select_sub_test(505)
        self.resume_exec()
        self.oocd.apptrace_wait_stop(tmo=30)

        print("lines after reset")

        lines_after_reset = []
        while True:
            try:
                line = reader.readline()
                if (len(line)):
                    lines_after_reset.append(line)
                    print(line)
            except ReaderTimeoutError:
                break
        reader.cleanup()
        os.remove(trace_file_name)

        # compare first 5 lines. But before that make sure first line includes number zero
        self.assertEqual(lines_before_reset[0].rstrip().split('#')[1], '0')
        self.assertEqual(lines_before_reset[:5], lines_after_reset[:5])

########################################################################
#              TESTS DEFINITION WITH SPECIAL TESTS                     #
########################################################################

class ApptraceTestAppTestsDual(DebuggerGenericTestAppTests):
    """ Base class to run tests which use gcov test app in dual core mode
    """
    def __init__(self, methodName='runTest'):
        super(ApptraceTestAppTestsDual, self).__init__(methodName)
        self.test_app_cfg.bin_dir = os.path.join('output', 'apptrace_gcov_dual')
        self.test_app_cfg.build_dir = os.path.join('builds', 'apptrace_gcov_dual')


class ApptraceTestAppTestsSingle(DebuggerGenericTestAppTests):
    """ Base class to run tests which use gcov test app in single core mode.
    """
    def __init__(self, methodName='runTest'):
        super(ApptraceTestAppTestsSingle, self).__init__(methodName)
        self.test_app_cfg.bin_dir = os.path.join('output', 'apptrace_gcov_single')
        self.test_app_cfg.build_dir = os.path.join('builds', 'apptrace_gcov_single')


class ApptraceTestsDual(ApptraceTestAppTestsDual, ApptraceTestsImpl):
    """ Test cases via GDB in dual core mode
    """
    def setUp(self):
        ApptraceTestAppTestsDual.setUp(self)
        ApptraceTestsImpl.setUp(self)

    def tearDown(self):
        ApptraceTestAppTestsDual.tearDown(self)
        ApptraceTestsImpl.tearDown(self)

class ApptraceTestsSingle(ApptraceTestAppTestsSingle, ApptraceTestsImpl):
    """ Test cases via GDB in single core mode
    """
    def setUp(self):
        ApptraceTestAppTestsSingle.setUp(self)
        ApptraceTestsImpl.setUp(self)

    def tearDown(self):
        ApptraceTestAppTestsSingle.tearDown(self)
        ApptraceTestsImpl.tearDown(self)
