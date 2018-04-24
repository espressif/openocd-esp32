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

    def test_step_over_insn_using_scratch_reg(self):
        """
            This test checks that scratch register (A3) used by OpenOCD for its internal purposes 
            (e.g. target memory reads) is not corrupted by stepping over instructions which use that reg as operand.
            1) Select appropriate sub-test number on target.
            2) Set breakpoint just before test instructions.
            3) Resume target and wait for breakpoint hit.
            4) Check that target stopped at the correct location.
            5) Step over instruction which write counter value to A3.
            6) Check that A3 has correct value.
            7) Step over instruction which moves A3 to A4.
            8) Check that A4 has correct value.
            9) Increment counter.
            10) Repeat steps 3-9 several times.
        """
        self.select_sub_test(102)
        val = 100
        bp = self.gdb.add_bp('_scratch_reg_using_task_break')
        for i in range(5):
            self.resume_exec()
            rsn = self.gdb.wait_target_state(dbg.Gdb.TARGET_STATE_STOPPED, 5)
            self.assertEqual(rsn, dbg.Gdb.TARGET_STOP_REASON_BP)
            cur_frame = self.gdb.get_current_frame()
            self.assertEqual(cur_frame['func'], 'scratch_reg_using_task')
            self.gdb.exec_next_insn()
            self.gdb.wait_target_state(dbg.Gdb.TARGET_STATE_RUNNING, 5)
            rsn = self.gdb.wait_target_state(dbg.Gdb.TARGET_STATE_STOPPED, 5)
            self.assertEqual(rsn, dbg.Gdb.TARGET_STOP_REASON_STEPPED)
            reg_val = int(self.gdb.data_eval_expr('$a3'))
            self.assertEqual(reg_val, val)
            self.gdb.exec_next_insn()
            self.gdb.wait_target_state(dbg.Gdb.TARGET_STATE_RUNNING, 5)
            rsn = self.gdb.wait_target_state(dbg.Gdb.TARGET_STATE_STOPPED, 5)
            self.assertEqual(rsn, dbg.Gdb.TARGET_STOP_REASON_STEPPED)
            reg_val = int(self.gdb.data_eval_expr('$a4'))
            self.assertEqual(reg_val, val)
            val += 1
        self.gdb.delete_bp(bp)
        self.resume_exec()


########################################################################
#              TESTS DEFINITION WITH SPECIAL TESTS                     #
########################################################################

class DebuggerStepTestsDual(DebuggerGenericTestAppTestsDual, DebuggerStepTestsImpl):
    """ Test cases for dual core mode
    """
    pass


class DebuggerStepTestsSingle(DebuggerGenericTestAppTestsSingle, DebuggerStepTestsImpl):
    """ Test cases for single core mode
    """
    pass
