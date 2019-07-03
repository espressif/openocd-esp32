import logging
import unittest
import time
import debug_backend as dbg
from debug_backend_tests import *


def get_logger():
    return logging.getLogger(__name__)


########################################################################
#                         TESTS IMPLEMENTATION                         #
########################################################################

class PerfMonTestsImpl:
    """ Test cases for performance monitor
    """

    def test_perfmon(self):
        self.select_sub_test(100)
        self.oocd.perfmon_enable(0, 0)  # count cycles
        self.oocd.perfmon_enable(1, 2)  # count retired instructions
        self.resume_exec()
        time.sleep(1)
        self.gdb.exec_interrupt()
        self.gdb.wait_target_state(dbg.Gdb.TARGET_STATE_STOPPED, 5)
        time.sleep(1)
        counts = self.oocd.perfmon_dump()
        for core in range(0, self.CORE_COUNT):
            self.assertGreater(counts[core][0], 0) # number of cycles >0
            self.assertGreater(counts[core][1], 0) # number of retired instructions >0
            self.assertGreaterEqual(counts[core][0], counts[core][1]) # sanity check: cycles >= instructions


########################################################################
#              TESTS DEFINITION WITH SPECIAL TESTS                     #
########################################################################

class PerfMonTestsDual(DebuggerGenericTestAppTestsDual, PerfMonTestsImpl):
    """ Test cases in dual core mode
    """
    CORE_COUNT = 2

class PerfMonTestsSingle(DebuggerGenericTestAppTestsSingle, PerfMonTestsImpl):
    """ Test cases in single core mode
    """
    CORE_COUNT = 1
