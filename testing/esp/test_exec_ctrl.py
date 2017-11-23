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

class DebuggerExecControlTestsImpl:
    """ Execution control test cases generic for dual and single core modes
    """

    def test_exec_stop(self):
        # select non-existing sub-test to go into infinite while loop on target
        self.select_sub_test(0)
        for i in range(3):
            state,rsn = self.gdb.get_target_state()
            if state == dbg.Gdb.TARGET_STATE_RUNNING:
                self.gdb.exec_interrupt()
                self.gdb.wait_target_state(dbg.Gdb.TARGET_STATE_STOPPED, 5)
            else:
                self.gdb.exec_continue()
                self.gdb.wait_target_state(dbg.Gdb.TARGET_STATE_RUNNING, 5)

    @unittest.skip('not implemented')
    def test_active_thread_switch(self):
        pass


########################################################################
#              TESTS DEFINITION WITH SPECIAL TESTS                     #
########################################################################

class DebuggerExecControlTestsDual(DebuggerGenericTestAppTestsDual, DebuggerExecControlTestsImpl):
    """ Test cases for dual core mode
    """

    @unittest.skip('not implemented')
    def test_something_special_for_dual_core_mode(self):
        pass


@unittest.skip('not implemented')
class DebuggerExecControlTestsSingle(DebuggerGenericTestAppTestsSingle, DebuggerExecControlTestsImpl):
    """ Test cases for single core mode
    """

    @unittest.expectedFailure
    def test_exec_stop(self):
        """
            Override test method to decorated its single core version as expected failure
        """
        super(DebuggerExecControlTestsSingle, self).test_exec_stop()
