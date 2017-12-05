# HOW TO ADD NEW TESTS
#
# There are three general cases when you need to add new test:
# 1) Add new test to existing test case class. It can be done by adding new test_xxx method 
#    to the existing child of unittest.TestCase. 
#    - If test is common for dual and single core modes add it to test iplementation class (e.g. DebuggerTemplateTestsImpl)
#    - If test should be run only in dual or single core mode add it to the respective specialized test class (e.g. DebuggerBreakpointTests(Dual|Single)
# 2) Add new test which requires new test case class. It can be done by adding new test class inherited from DebuggerTestsBase or one of its childs 
# 3) Add new test which requires new test module. The minimalistic template module is shown here. Create test module file. Use pattern 'test_*.py' for the name of file.
#    Copy contents of this file to new module and follow items 1-2.
import logging
import unittest
import debug_backend as dbg
from debug_backend_tests import *


def get_logger():
    return logging.getLogger(__name__)


########################################################################
#                         TESTS IMPLEMENTATION                         #
########################################################################

class DebuggerTemplateTestsImpl:
    """ Test cases which are common for dual and single core modes
    """

    def test_something(self):
        pass


########################################################################
#              TESTS DEFINITION WITH SPECIAL TESTS                     #
########################################################################

class DebuggerTemplateTestsDual(DebuggerGenericTestAppTestsDual, DebuggerTemplateTestsImpl):
    """ Test cases in dual core mode
    """

    def test_something_special_for_dual_core_mode(self):
        pass

class DebuggerTemplateTestsSingle(DebuggerGenericTestAppTestsSingle, DebuggerTemplateTestsImpl):
    """ Test cases in single core mode
    """
    # no special tests for single core mode yet
    pass
