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

class DebuggerStepTestsImpl:
    """ Stepping test cases generic for dual and single core modes
    """
    
    @unittest.skip('not implemented')
    def test_step(self):
        pass

    @unittest.skip('not implemented')
    def test_step_over_breakpoint(self):
        pass

    @unittest.skip('not implemented')
    def test_step_after_active_thread_swicth(self):
        pass

    def test_step_window_exception(self):
        # start the test, stopping at the window_exception_test function
        self.select_sub_test(200)
        bp = self.gdb.add_bp('window_exception_test')
        self.resume_exec()
        rsn = self.gdb.wait_target_state(dbg.Gdb.TARGET_STATE_STOPPED, 5)
        self.assertEqual(rsn, dbg.Gdb.TARGET_STOP_REASON_BP)
        self.gdb.delete_bp(bp)

        # do "step in", 3 steps per recursion level
        for i in range(0, 59):
            get_logger().info('Step in {}'.format(i))
            self.step_in()

        # check that we have reached the end of recursion
        self.assertEqual(int(self.gdb.data_eval_expr('levels')), 1)

        # do "step out" once per recursion level
        for i in range(0, 20):
            get_logger().info('Step out {}'.format(i))
            self.step_out()

        cur_frame = self.gdb.get_current_frame()
        self.assertEqual(cur_frame['func'], 'window_exception_test')


########################################################################
#              TESTS DEFINITION WITH SPECIAL TESTS                     #
########################################################################

class DebuggerStepTestsDual(DebuggerGenericTestAppTestsDual, DebuggerStepTestsImpl):
    """ Test cases for dual core mode
    """

    @unittest.skip('not implemented')
    def test_something_special_for_dual_core_mode(self):
        pass


class DebuggerStepTestsSingle(DebuggerGenericTestAppTestsSingle, DebuggerStepTestsImpl):
    """ Test cases for single core mode
    """
    # no special tests for single core mode yet
    pass
