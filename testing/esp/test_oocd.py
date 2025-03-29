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

    def run_stub_test(self, test):
        res = self.oocd.cmd_exec(f'esp {test}')
        self.assertTrue(f'Called {test}: test passed' in res)

    def test_stub_lib(self):
        """
            This test simply calls `esp stub_lib_test` command and checks the return code.
        """
        self.run_stub_test('stub_lib_test')

    def test_apptrace_wr(self):
        """
            This test simply calls `esp stub_apptrace_wr_test` command and checks the return code.
        """
        self.run_stub_test('stub_apptrace_wr_test')

    def test_apptrace_rd(self):
        """
            This test simply calls `esp stub_apptrace_rd_test` command and checks the return code.
        """
        self.oocd.cmd_exec('esp stub_log off') # doesnt work (in CI) with enabled stub logs
        self.run_stub_test('stub_apptrace_rd_test')

