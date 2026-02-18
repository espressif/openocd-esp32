import filecmp
import logging
import os
import re
import tempfile
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

    def test_flash_read_bank(self):
        """
            This test runs `flash read_bank` command and validates the response and output file.
        """
        size = 512000
        offset = 0x1000
        bank = 0
        tmp = tempfile.NamedTemporaryFile(suffix='.bin', delete=False)
        tmp.close()
        try:
            res = self.oocd.cmd_exec(f'flash read_bank {bank} {tmp.name} {offset} {size}')
            expected = (f'wrote {size} bytes to file {tmp.name} '
                        f'from flash bank {bank} at offset {offset:#010x}')
            self.assertIn(expected, res, f"Expected '{expected}' in output:\n{res}")
            self.assertTrue(os.path.isfile(tmp.name), f"Output file {tmp.name} was not created")
            actual_size = os.path.getsize(tmp.name)
            self.assertEqual(actual_size, size, f"Expected file size {size}, got {actual_size}")
        finally:
            if os.path.exists(tmp.name):
                os.unlink(tmp.name)

    def test_flash_write_bank(self):
        """
            This test reads a flash region, erases it, writes back, reads again
            and compares to verify the write was successful.
        """
        size = 0x20000
        offset = 0x2000
        bank = 0
        tmp_orig = tempfile.NamedTemporaryFile(suffix='_orig.bin', delete=False)
        tmp_orig.close()
        tmp_verify = tempfile.NamedTemporaryFile(suffix='_verify.bin', delete=False)
        tmp_verify.close()

        try:
            res = self.oocd.cmd_exec(f'flash read_bank {bank} {tmp_orig.name} {offset} {size}')
            self.assertIn(f'wrote {size} bytes to file', res, f"Read original failed:\n{res}")

            res = self.oocd.cmd_exec(f'flash erase_address {offset} {size}')
            self.assertIn('erased address', res, f"Erase failed:\n{res}")

            self.oocd.cmd_exec('esp compression off')
            res = self.oocd.cmd_exec(f'flash write_bank {bank} {tmp_orig.name} {offset}')
            self.assertIn(f'wrote', res, f"Write back failed:\n{res}")

            res = self.oocd.cmd_exec(f'flash read_bank {bank} {tmp_verify.name} {offset} {size}')
            self.assertIn(f'wrote {size} bytes to file', res, f"Read verify failed:\n{res}")

            self.assertTrue(filecmp.cmp(tmp_orig.name, tmp_verify.name, shallow=False),
                "Verification failed: written data does not match original")
        finally:
            for path in (tmp_orig.name, tmp_verify.name):
                if os.path.exists(path):
                    os.unlink(path)

    def test_flash_erase_check(self):
        """
            This test runs `flash erase_check 0` and validates the PROF output.
        """
        res = self.oocd.cmd_exec('flash erase_check 0')
        pattern = r'PROF: Erased check (\d+) sectors in [\d.]+ ms'
        match = re.search(pattern, res)
        self.assertIsNotNone(match, f"Expected 'PROF: Erased check <N> sectors in <T> ms' in output:\n{res}")
        sectors = int(match.group(1))
        self.assertGreater(sectors, 0, f"Expected sector count > 0, got {sectors}")

    def run_flash_mapping_test(self):
        target_output = self.oocd.cmd_exec('flash probe 0')

        expected_mapping = ["Flash mapping 0:",
                            "Flash mapping 1:"]
        for expected_str in expected_mapping:
            self.assertIn(expected_str, target_output,
                f"Expected string '{expected_str}' not found in output")

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
        self.oocd.cmd_exec('reset halt')
        self.run_flash_mapping_test()

    def test_flash_mapping_reset_resume(self):
        """
            This test runs `flash probe 0` command after reset run.
        """
        self.oocd.cmd_exec('reset run')
        time.sleep(2)
        self.oocd.cmd_exec('halt')
        self.run_flash_mapping_test()

class StubTestsWithLog(StubTestsSimple):
    """ Runs all StubTestsSimple tests with esp stub_log enabled.
    """
    def setUp(self):
        StubTestsSimple.setUp(self)
        self.oocd.cmd_exec('esp stub_log on')

    def tearDown(self):
        self.oocd.cmd_exec('esp stub_log off')
        StubTestsSimple.tearDown(self)
