import logging
import unittest
import debug_backend as dbg
from debug_backend_tests import *


def get_logger():
    """ Returns logger for this module
    """
    return logging.getLogger(__name__)


########################################################################
#                         TESTS IMPLEMENTATION                         #
########################################################################

class DebuggerBreakpointTestsImpl:
    """ Breakpoints test cases which are common for dual and single core modes
    """

    @unittest.expectedFailure
    def test_multi_reset_break(self):
        for i in range(3):
            self.gdb.target_reset()
            rsn = self.gdb.wait_target_state(dbg.Gdb.TARGET_STATE_STOPPED, 5)
            bp = self.gdb.add_bp('app_main')
            self.resume_exec()
            rsn = self.gdb.wait_target_state(dbg.Gdb.TARGET_STATE_STOPPED, 5)
            self.assertEqual(rsn, dbg.Gdb.TARGET_STOP_REASON_BP)
            self.gdb.delete_bp(bp)

    def test_simple(self):
        """
            This simple test does the following:
            1) Selects appropriate sub-test number on target.
            2) Set breakpoint.
            3) Resumes target and waits for brekpoint hit.
            4) Checks that target has stopped in right place.
            5) Checks backtrace at stop point.
            6) Repeats steps 3-5 several times.
            7) Finally it deletes breakpoint and resumes execution.
        """
        get_logger().debug('Start test')
        self.select_sub_test(1)
        bp = self.gdb.add_bp('gpio_set_level')
        for i in range(3):
            self.resume_exec()
            rsn = self.gdb.wait_target_state(dbg.Gdb.TARGET_STATE_STOPPED, 5)
            self.assertEqual(rsn, dbg.Gdb.TARGET_STOP_REASON_BP)
            cur_frame = self.gdb.get_current_frame()
            self.assertEqual(cur_frame['func'], 'gpio_set_level')
            self.assertEqual(cur_frame['line'], '188')
            frames = self.gdb.get_backtrace()
            self.assertEqual(len(frames), 2)
            self.assertEqual(frames[0]['func'], cur_frame['func'])
            self.assertEqual(frames[0]['line'], cur_frame['line'])
            self.assertEqual(frames[1]['func'], 'blink_task')
            if i % 2:
                self.assertEqual(frames[1]['line'], '43')
            else:
                self.assertEqual(frames[1]['line'], '40')
        self.gdb.delete_bp(bp)
        self.resume_exec()

    @unittest.skip('not implemented')
    def test_condition(self):
        pass

########################################################################
#              TESTS DEFINITION WITH SPECIAL TESTS                     #
########################################################################

class DebuggerBreakpointTestsDual(DebuggerGenericTestAppTestsDual, DebuggerBreakpointTestsImpl):
    """ Test cases for breakpoints in dual core mode
    """

    @unittest.skip('not implemented')
    def test_something_special_for_dual_core_mode(self):
        pass

class DebuggerBreakpointTestsSingle(DebuggerGenericTestAppTestsSingle, DebuggerBreakpointTestsImpl):
    """ Test cases for breakpoints in single core mode
    """

    @unittest.expectedFailure
    def test_simple(self):
        """
            Override test method to decorated its single core version as expected failure
        """
        super(DebuggerBreakpointTestsSingle, self).test_simple()
