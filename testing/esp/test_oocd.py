import logging
import debug_backend as dbg
from debug_backend_tests import *


def get_logger():
    return logging.getLogger(__name__)


class OocdTestsSimple(DebuggerGenericTestAppTests):
    """ Simple
    """
    def setUp(self):
        self.oocd.cmd_exec('reset halt')

    def tearDown(self):
        self.oocd.process_lazy_bps()

class StubTestsSimple(OocdTestsSimple):
    """ Test cases for flasher stub
    """
    def setUp(self):
        OocdTestsSimple.setUp(self)
        self.oocd.cmd_exec('esp stub_log on')

    def tearDown(self):
        self.oocd.cmd_exec('esp stub_log off')
        OocdTestsSimple.tearDown(self)

    def test_stub_lib(self):
        """
            This test simply calls `esp stub_lib_test` command and checks the return code.
        """
        res = self.oocd.cmd_exec('esp stub_lib_test')
        print('\n' + res)
        self.assertTrue('ret_code: 0' in res)
