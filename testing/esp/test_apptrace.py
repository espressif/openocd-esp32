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

idf_path = os.getenv('IDF_PATH')
if idf_path:
    sys.path.append(os.path.join(idf_path, 'tools', 'esp_app_trace'))

from espytrace.apptrace import reader_create, ReaderTimeoutError


def get_logger():
    return logging.getLogger(__name__)


########################################################################
#                         TESTS IMPLEMENTATION                         #
########################################################################

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
