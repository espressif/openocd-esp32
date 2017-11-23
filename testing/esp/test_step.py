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
