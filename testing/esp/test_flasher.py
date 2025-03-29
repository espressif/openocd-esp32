import logging
import unittest
import tempfile
import filecmp
import json
import os
import shutil
import debug_backend as dbg
from debug_backend_tests import *


def get_logger():
    return logging.getLogger(__name__)


DUMMY_BIN_SIZE = 0x400
BOOTLOADER_OFF = 0x10000
PARTITION_TABLE_OFF = 0x20000
APP_TAIL_OFF = 0x2000


########################################################################
#                         TESTS IMPLEMENTATION                         #
########################################################################
class FlasherTestsImpl:
    """ Test cases which are common for dual and single core modes
    """

    def setUp(self):
        self.gdb.monitor_run('flash probe 0', tmo=10)
        self.flash_sz = self.get_flash_banks()[0][1]

    def _get_image_header(self, len, magic=0xe9, chip_id=None, min_rev=0, max_rev=0xffff):
        if chip_id is None:
            chip_id = {
                'esp32': 0,
                'esp32s2': 2,
                'esp32s3': 9,
                'esp32c2': 12,
                'esp32c3': 5,
                'esp32c5': 23,
                'esp32c6': 13,
                'esp32c61': 20,
                'esp32h2': 16,
                'esp32h21': 25,
                'esp32h4': 28,
                'esp32p4': 18,
                'esp32s31': 32
            }[testee_info.chip]
        data = bytearray(os.urandom(len))
        data[0] = magic
        data[12] = chip_id & 0xFF
        data[13] = (chip_id >> 8) & 0xFF
        data[15] = min_rev & 0xFF
        data[16] = (min_rev >> 8) & 0xFF
        data[17] = max_rev & 0xFF
        data[18] = (max_rev >> 8) & 0xFF
        return data

    def test_rev_checks(self):
        """
            This test checks that flasher correctly handles chip revision checks.
        """
        rev = int(self.oocd.cmd_exec("[target current] cget -revision").strip())

        test_cases = [
            # magic, chip_id, min_rev, max_rev, should_pass
            (None, None, None, None, True),  # default values should pass
            (0xEE, None, None, None, True),  # wrong magic should be ignored
            (None, 0xFFFF, None, None, False),  # unknown chip_id
            (None, None, rev + 1, None, False),  # min_rev too high
            (None, None, None, rev - 1, False),  # max_rev too low
            (None, None, rev, rev, True),  # exact match
            (None, None, rev - 1, rev + 1, True),
        ]

        try:
            for magic, chip_id, min_rev, max_rev, should_pass in test_cases:
                if rev == 0 and (min_rev is not None or max_rev is not None):
                    # dont check revision for v0.0, 'cget -revision' can be unimplemented
                    continue
                kwargs = {}
                if magic is not None:
                    kwargs['magic'] = magic
                if chip_id is not None:
                    kwargs['chip_id'] = chip_id
                if min_rev is not None:
                    kwargs['min_rev'] = min_rev
                if max_rev is not None:
                    kwargs['max_rev'] = max_rev
                data = self._get_image_header(1024, **kwargs)
                fhnd, fname = tempfile.mkstemp()
                with os.fdopen(fhnd, 'wb') as fbin:
                    fbin.write(data)
                actions='encrypt verify' if self.ENCRYPTED else 'verify'
                try:
                    self.gdb.target_program(fname, 0, actions=actions)
                    self.assertTrue(should_pass)
                except dbg.DebuggerError:
                    if should_pass:
                        raise
                    else:
                        self.gdb.target_program(fname, 0, actions=actions + ' force')
        finally:
            # restore flash contents with test app as it was overwritten by test
            # what can lead to the failures when preparing for the next tests
            self.gdb.target_program_bins(self.test_app_cfg.build_bins_dir())

    def program_big_binary(self, actions, overflow=False):
        size = 0x2000 if overflow else self.flash_sz
        offset = self.flash_sz - 0x1000 if overflow else 0
        truncate_size = 0x1000 if overflow else 0

        fhnd, fname1 = tempfile.mkstemp()
        get_logger().debug('Generate random file %dKB "%s"', size / 1024, fname1)
        with os.fdopen(fhnd, 'wb') as fbin:
            for i in range(int(size / 1024)):
                fbin.write(os.urandom(1024) if i != 0 else self._get_image_header(1024))

        self.gdb.target_program(fname1, offset, actions=actions, tmo=480)

        # since we can not get result from OpenOCD (output parsing seems not to be good idea),
        # we need to read written flash and compare data manually
        _, fname2 = tempfile.mkstemp()
        os.truncate(fname1, size - truncate_size)
        self.gdb.monitor_run('flash read_bank 0 %s 0x%x %d' % (dbg.fixup_path(fname2), offset, size - truncate_size), tmo=480)

        # restore flash contents with test app as it was overwritten by test
        # what can lead to the failures when preparing for the next tests
        self.gdb.target_program_bins(self.test_app_cfg.build_bins_dir())

        self.assertTrue(filecmp.cmp(fname1, fname2))

    def test_big_binary(self):
        """
            This test checks flashing big binaries works.
            1) Create test binary file of the most possible size.
            2) Fill it with random data.
            3) Write the file to the flash.
            4) Read written data to another file.
            5) Compare files.
        """
        self.program_big_binary('encrypt verify' if self.ENCRYPTED else 'verify')

    def test_big_binary_compressed(self):
        """
            This test checks flashing big compressed binaries works.
            1) Create test binary file of the most possible size.
            2) Fill it with random data.
            3) Write the file to the flash with compress option.
            4) Read written data to another file.
            5) Compare files.
        """
        self.program_big_binary('encrypt compress' if self.ENCRYPTED else 'compress')

    def test_flash_overflow(self):
        """
            This test checks that flashing binaries which overflow flash boundaries is safe.
            1) Create test binary file with size of flash.
            2) Fill it with random data.
            3) Write the file to the flash with some offset to ensure that write operation crosses flash boundary.
            4) Read written data to another file.
            5) Compare files and ensure that written data size was truncated to fit flash.
        """
        self.program_big_binary('encrypt' if self.ENCRYPTED else '', overflow=True)

    def test_flash_overflow_compressed(self):
        """
            This test checks that flashing compressed binaries which overflow flash boundaries is safe.
            1) Create test binary file with size of flash.
            2) Fill it with random data.
            3) Write the file to the flash with some offset to ensure that write operation crosses flash boundary.
            4) Read written data to another file.
            5) Compare files and ensure that written data size was truncated to fit flash.
        """
        self.program_big_binary('encrypt compress' if self.ENCRYPTED else 'compress', overflow=True)

    def test_cache_handling(self):
        """
            This test checks that flasher does not corrupts cache config registers when setting breakpoints.
            The idea is the following: GDB postpones writing breakpoints to resume.
            So cache config registers should have the same values after resuming with set flash breakpoints.
            It is checked by the test code on target. This test method sets breakpoint and resumes execution several times.
            Before hiting the BP program save cache config and after resuming checks its value.
        """
        # Filling HW breakpoints slots to make test using SW flash breakpoints
        self.fill_hw_bps(keep_avail=2)
        # 2 HW + 1 SW flash BP
        self.bps = ['app_main', 'gpio_set_direction', 'gpio_set_level']
        for f in self.bps:
            self.add_bp(f)
        self.run_to_bp_and_check(dbg.TARGET_STOP_REASON_BP, 'gpio_set_direction', ['gpio_set_direction'], outmost_func_name='cache_handling_task')
        for i in range(5):
            self.run_to_bp_and_check(dbg.TARGET_STOP_REASON_BP, 'gpio_set_level', ['gpio_set_level'], outmost_func_name='cache_handling_task')

    def program_esp_bins(self, actions):
        # Temp Folder where everything will be contained
        tmp = tempfile.mkdtemp(prefix="esp")

        obj = generate_flasher_args_json(self.flash_sz)
        flash_files = obj["flash_files"]

        # Write dummy data to bin files
        for offset in flash_files:
            fname = "esp_%s.bin" % (offset)
            fpath = os.path.join(tmp, fname)

            flash_files[offset] = fname

            fbin = open(fpath, 'wb')
            fbin.write(self._get_image_header(DUMMY_BIN_SIZE))
            fbin.close()

        encrypted = "true" if self.ENCRYPTED else "false"
        obj["partition_table"]["encrypted"] = encrypted
        obj["bootloader"]["encrypted"] = encrypted
        obj["app"]["encrypted"] = encrypted

        # Write the flasher_args file
        json_fname = "flasher_args.json"
        json_fpath = os.path.join(tmp, json_fname)
        json_fp = open(json_fpath, "w")
        json.dump(obj, fp=json_fp, indent=2)
        json_fp.close()

        # Flash the chip
        self.gdb.monitor_run("program_esp_bins %s %s %s" % (tmp, json_fname, actions))
        # Halt Reset
        self.gdb.target_reset(action='halt')

        # Read the chip back to verify if flash was successful
        for offset in flash_files:
            fname = "esp_%s.bin.verify" % (offset)
            fpath = os.path.join(tmp, fname)
            fbin = open(fpath, "wb")
            fbin.close()
            self.gdb.monitor_run("flash read_bank 0 %s %s %d" % (dbg.fixup_path(fpath), offset, DUMMY_BIN_SIZE), tmo=120)

            # Verify the content
            og_fname = "esp_%s.bin" % (offset)
            og_fpath = os.path.join(tmp, og_fname)
            self.assertTrue(filecmp.cmp(og_fpath, fpath))

        # Remove the tmp folder for cleanup
        shutil.rmtree(tmp)

    def test_program_esp_bins(self):
        """
            This test checks flashing complete app works using flasher_args.json.
            1) Generate a dummy flasher_args.json file
            2) Create random binaries based on the flasher_args.json
            3) Write the files to the flash.
            4) Read written data to another file.
            5) Compare files.
        """
        self.program_esp_bins('reset verify')
        # restore flash contents with test app as it was overwritten by test
        # what can lead to the failures when preparing for the next tests
        self.gdb.target_program_bins(self.test_app_cfg.build_bins_dir())

    def test_program_esp_bins_compressed(self):
        """
            This test checks flashing all compressed apps works using flasher_args.json.
            1) Generate a dummy flasher_args.json file
            2) Create random binaries based on the flasher_args.json
            3) Write the files to the flash with compress option.
            4) Read written data to another file.
            5) Compare files.
        """
        self.program_esp_bins('reset verify compress')
        # restore flash contents with test app as it was overwritten by test
        # what can lead to the failures when preparing for the next tests
        self.gdb.target_program_bins(self.test_app_cfg.build_bins_dir())

def generate_flasher_args_json(flash_size_bytes):
    off_app = flash_size_bytes - APP_TAIL_OFF
    min_flash = PARTITION_TABLE_OFF + DUMMY_BIN_SIZE + APP_TAIL_OFF
    if flash_size_bytes < min_flash:
        raise ValueError(
            f"flash size {flash_size_bytes:#x} too small for program_esp_bins test layout: "
            f"partition_table@{PARTITION_TABLE_OFF:#x} + dummy_bin({DUMMY_BIN_SIZE:#x}) "
            f"must fit before app@flash_end-{APP_TAIL_OFF:#x} (need >= {min_flash:#x})"
        )

    return {
        "write_flash_args" : [ "--flash_mode", "dio",
                            "--flash_size", "detect",
                            "--flash_freq", "40m" ],
        "flash_settings" : {
            "flash_mode": "dio",
            "flash_size": "detect",
            "flash_freq": "40m"
        },
        "flash_files" : {
            f"{PARTITION_TABLE_OFF:#x}" : "",
            f"{BOOTLOADER_OFF:#x}" : "",
            f"{off_app:#x}" : ""
        },
        "partition_table" : { "offset" : f"{PARTITION_TABLE_OFF:#x}", "file" : "", "encrypted" : "" },
        "bootloader" : { "offset" : f"{BOOTLOADER_OFF:#x}", "file" : "", "encrypted" : "" },
        "app" : { "offset" : f"{off_app:#x}", "file" : "", "encrypted" : "" },
        "extra_esptool_args" : {
            "after"  : "hard_reset",
            "before" : "default_reset",
            "stub"   : True,
            "chip"   : "esp32"
        }
    }


########################################################################
#              TESTS DEFINITION WITH SPECIAL TESTS                     #
########################################################################

class FlasherTestsDual(DebuggerGenericTestAppTestsDual, FlasherTestsImpl):
    """ Test cases in dual core mode
    """
    def setUp(self):
        DebuggerGenericTestAppTestsDual.setUp(self)
        FlasherTestsImpl.setUp(self)

    @skip_for_chip(['esp32p4' ,'esp32h4'], "skipped - slow test")
    def test_big_binary_compressed(self):
        super(DebuggerGenericTestAppTestsDual, self).test_big_binary_compressed()

class FlasherTestsDualEncrypted(DebuggerGenericTestAppTestsDualEncrypted, FlasherTestsImpl):
    """ Encrypted flash test cases in dual core mode
    """
    def setUp(self):
        DebuggerGenericTestAppTestsDualEncrypted.setUp(self)
        FlasherTestsImpl.setUp(self)

class FlasherTestsSingle(DebuggerGenericTestAppTestsSingle, FlasherTestsImpl):
    """ Test cases in single core mode
    """
    def setUp(self):
        DebuggerGenericTestAppTestsSingle.setUp(self)
        FlasherTestsImpl.setUp(self)

    @skip_for_chip(['esp32p4', 'esp32h4'], "skipped - slow test")
    def test_big_binary(self):
        super(DebuggerGenericTestAppTestsSingle, self).test_big_binary()

class FlasherTestsSingleEncrypted(DebuggerGenericTestAppTestsSingleEncrypted, FlasherTestsImpl):
    """ Encrypted flash test cases in single core mode
    """
    def setUp(self):
        DebuggerGenericTestAppTestsSingleEncrypted.setUp(self)
        FlasherTestsImpl.setUp(self)

@idf_ver_min('5.4')
@only_for_chip(['esp32c6', 'esp32h2'])
class FlasherTestsPreloadedStubSingle(DebuggerGenericTestAppTestsSingle):

    def __init__(self, methodName='runTest'):
        super(FlasherTestsPreloadedStubSingle, self).__init__(methodName)
        self.test_app_cfg.bin_dir = os.path.join('output', 'single_core_preloaded_stub')
        self.test_app_cfg.build_dir = os.path.join('builds', 'single_core_preloaded_stub')

    def test_preloaded_stub_binary(self):
        """
            This test checks if stub codes already loaded to the targets and functioning as expected.
            If there's a version mismatch, the test will pass but log the version difference.
        """
        # Expected strings that should always be present
        common_expected_strings = ["Flash mapping 0:",
                                  "Flash mapping 1:"]

        # Expected string for successful preloaded stub usage
        preloaded_expected_string = "Stub flasher will be running from preloaded image (5C3A9F5A)"

        target_output = ''
        def _target_stream_handler(type, stream, payload):
            nonlocal target_output
            target_output += payload
        self.gdb.stream_handler_add('target', _target_stream_handler)

        self.gdb.monitor_run("esp stub_log off", 5)
        self.gdb.monitor_run("flash probe 0", 5)
        self.gdb.stream_handler_remove('target', _target_stream_handler)

        # Check if neither preloaded nor version mismatch scenario
        if preloaded_expected_string not in target_output:
            import re
            installed_version_match = re.search(r'Installed stub code.*?stub_version\((\d+)\)', target_output)
            expected_version_match = re.search(r'Expected stub code.*?stub_version\((\d+)\)', target_output)

            self.assertTrue(installed_version_match and expected_version_match)
            installed_version = installed_version_match.group(1)
            expected_version = expected_version_match.group(1)
            self.assertTrue(installed_version != expected_version)
            print(f"Version mismatch detected - Installed: {installed_version}, Expected: {expected_version} - Stub was loaded fresh due to version mismatch - this is expected behavior")

        # Always check common functionality regardless of preloaded vs fresh load
        for expected_str in common_expected_strings:
            self.assertIn(expected_str, target_output, f"Expected string '{expected_str}' not found in output")
