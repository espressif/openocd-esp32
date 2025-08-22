import logging
import re
import time
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

    def tearDown(self):
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
        self.run_stub_test('stub_apptrace_rd_test')

    def run_flash_mapping_test(self, reset_cmds):
        for cmd in reset_cmds:
            self.oocd.cmd_exec(cmd)

        target_output = self.oocd.cmd_exec('flash probe 0')

        expected_mapping = ["Flash mapping 0:",
                            "Flash mapping 1:"]
        for expected_str in expected_mapping:
            self.assertIn(expected_str, target_output,
                f"Expected string '{expected_str}' not found in output")

        # Validate the full format: "Flash mapping N: 0xHEX -> 0xHEX, N KB"
        mapping_pattern = r'Flash mapping \d+: (0x[0-9a-fA-F]+ -> 0x[0-9a-fA-F]+, \d+ KB)'
        matches = re.findall(mapping_pattern, target_output)
        self.assertEqual(len(matches), 2,
            f"Expected 2 flash mapping lines, found {len(matches)} in output:\n{target_output}")
        self.assertNotEqual(matches[0], matches[1],
            f"Flash mapping 0 and 1 must differ, but both are:\n{matches[0]}")

    def test_flash_mapping_reset_halt(self):
        """
            This test runs `flash probe 0` command after reset halt.
        """
        self.run_flash_mapping_test(['reset halt'])

    def test_flash_mapping_reset_resume(self):
        """
            This test runs `flash probe 0` command after reset run.
        """
        self.oocd.cmd_exec('reset run')
        time.sleep(2)
        self.run_flash_mapping_test(['halt'])
