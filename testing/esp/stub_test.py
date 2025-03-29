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

    def test_stub_lib(self):
        """
            This test runs `esp stub_lib_test` command for exception handling and checks the return code.
        """
        # TODO: check why xtensa fails to handle exceptions after 'reset halt'
        self.oocd.cmd_exec('reset')
        time.sleep(1)
        self.oocd.cmd_exec('halt')
        res = self.oocd.cmd_exec(f'esp stub_lib_test')
        self.assertRegex(res, r'Stub exception on hart \d+ @ 0x[0-9a-fA-F]+: \w+')
        self.assertTrue(f'Failed to run stub test1: (0x2000) Exception happened during stub execution' in res)

    def test_apptrace_wr(self):
        """
            This test runs `esp stub_apptrace_wr_test` command and checks the return code.
        """
        res = self.oocd.cmd_exec(f'esp stub_apptrace_wr_test')
        self.assertTrue(f'Apptrace write test passed' in res)

    def test_apptrace_rd(self):
        """
            This test runs `esp stub_apptrace_rd_test` command and checks the return code.
        """
        res = self.oocd.cmd_exec(f'esp stub_apptrace_rd_test')
        self.assertTrue(f'Apptrace read test passed' in res)

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
            self.assertRegex(res,
                r'wrote \d+ bytes to file .+ from flash bank 0 at offset 0x[0-9a-fA-F]+',
                f"Read failed:\n{res}")
            self.assertTrue(os.path.isfile(tmp.name), f"Output file {tmp.name} was not created")
            actual_size = os.path.getsize(tmp.name)
            self.assertEqual(actual_size, size, f"Expected file size {size}, got {actual_size}")
        finally:
            if os.path.exists(tmp.name):
                os.unlink(tmp.name)

    def _flash_write_bank(self, compressed, size=0x100000, offset=0x20000):
        """
            Reads a flash region, erases it, writes back (with compression
            on or off), then verifies via SHA256 hash comparison on the target.
        """
        tmp_orig = tempfile.NamedTemporaryFile(suffix='_orig.bin', delete=False)
        tmp_orig.close()

        try:
            res = self.oocd.cmd_exec(f'flash read_bank 0 {tmp_orig.name} {offset} {size}')
            self.assertRegex(res,
                r'wrote \d+ bytes to file .+ from flash bank 0 at offset 0x[0-9a-fA-F]+',
                f"Read original failed:\n{res}")

            res = self.oocd.cmd_exec(f'flash erase_address {offset} {size}')
            self.assertIn('erased address', res, f"Erase failed:\n{res}")

            self.oocd.cmd_exec(f'esp compression {compressed}')
            res = self.oocd.cmd_exec(f'flash write_bank 0 {tmp_orig.name} {offset}')
            self.assertRegex(res,
                r'wrote \d+ bytes from file .+ to flash bank 0 at offset 0x[0-9a-fA-F]+',
                f"Write back failed (compression {compressed}):\n{res}")

            res = self.oocd.cmd_exec(f'esp verify_bank_hash 0 {tmp_orig.name} {offset}')
            self.assertRegex(res,
                r'PROF: Flash verified in [\d.]+ ms',
                f"Hash verification failed (compression {compressed}):\n{res}")
            self.assertNotIn('Verification failure', res,
                f"Hash verification failed (compression {compressed}):\n{res}")
        finally:
            self.oocd.cmd_exec('esp compression on') # default compression mode
            if os.path.exists(tmp_orig.name):
                os.unlink(tmp_orig.name)

    def test_flash_write_bank_compressed(self):
        """
            Test flash write_bank and verify_bank_hash with compression enabled.
        """
        self._flash_write_bank(compressed='on', offset=0x0)

    def test_flash_write_bank(self):
        """
            Test flash write_bank and verify_bank_hash with compression disabled.
        """
        self._flash_write_bank(compressed='off')

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
