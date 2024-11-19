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


########################################################################
#                         TESTS IMPLEMENTATION                         #
########################################################################
class FlasherTestsImpl:
    """ Test cases which are common for dual and single core modes
    """

    def setUp(self):
        self.gdb.monitor_run('flash probe 0', tmo=10)
        _, self.flash_sz = self.get_flash_size(0)

    def get_flash_size(self, bank_num):
        _,target_output = self.gdb.monitor_run('flash banks', tmo=10, output_type='stdout')
        for bank_desc in target_output.split('\\n'):
            # #0 : esp32.cpu0.flash (esp32) at 0x00000000, size 0x00400000, buswidth 0, chipwidth 0
            mo = re.match(r'#(?P<bank_num>\d)+\s*:\s*(?P<tgt_name>\S+).flash\s+\(\w+\)\s+at\s+0x[0-9A-Fa-f]+,\s*size\s+(?P<flash_sz>0x[0-9A-Fa-f]+)', bank_desc)
            if not mo or len(mo.groups()) != 3:
                continue
            if int(mo.group("bank_num")) != bank_num:
                continue
            return mo.group("tgt_name"), int(mo.group("flash_sz"), 16)
        return "", 0

    def program_big_binary(self, actions, overflow=False):
        size = 0x2000 if overflow else self.flash_sz
        offset = self.flash_sz - 0x1000 if overflow else 0
        truncate_size = 0x1000 if overflow else 0

        fhnd, fname1 = tempfile.mkstemp()
        get_logger().debug('Generate random file %dKB "%s"', size / 1024, fname1)
        with os.fdopen(fhnd, 'wb') as fbin:
            for i in range(int(size / 1024)):
                fbin.write(os.urandom(1024))

        self.gdb.target_program(fname1, offset, actions=actions, tmo=130)

        # since we can not get result from OpenOCD (output parsing seems not to be good idea),
        # we need to read written flash and compare data manually
        _, fname2 = tempfile.mkstemp()
        os.truncate(fname1, size - truncate_size)
        self.gdb.monitor_run('flash read_bank 0 %s 0x%x %d' % (dbg.fixup_path(fname2), offset, size - truncate_size), tmo=180)

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

    def test_stub_logs(self):
        """
            This test checks if stub logs are enabled successfully.
        """
        expected_strings = ["STUB_D: cmd 4:FLASH_MAP_GET",
                            "STUB_D: stub_flash_get_size: ENTER",
                            "STUB_I: Found app image: magic 0xe9"]

        self.gdb.monitor_run("esp stub_log on", 5)
        self.gdb.monitor_run("flash probe 0", 5)
        self.gdb.monitor_run("esp stub_log off", 5)

        log_path = get_logger().handlers[1].baseFilename  # 0:StreamHandler 1:FileHandler
        target_output = ''
        with open(log_path, 'r') as file:
            target_output = file.read()

        for expected_str in expected_strings:
            self.assertIn(expected_str, target_output, f"Expected string '{expected_str}' not found in output")

    def program_esp_bins(self, actions):
        # Temp Folder where everything will be contained
        tmp = tempfile.mkdtemp(prefix="esp")

        obj = generate_flasher_args_json()
        flash_files = obj["flash_files"]

        # Write dummy data to bin files
        for offset in flash_files:
            fname = "esp_%s.bin" % (offset)
            fpath = os.path.join(tmp, fname)

            flash_files[offset] = fname

            fbin = open(fpath, 'wb')
            fbin.write(os.urandom(1024))
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
            self.gdb.monitor_run("flash read_bank 0 %s %s 1024" % (dbg.fixup_path(fpath), offset), tmo=120)

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

def generate_flasher_args_json():
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
            "0x118000" : "",
            "0x110000" : "",
            "0x210000" : ""
        },
        "partition_table" : { "offset" : "0x118000", "file" : "", "encrypted" : "" },
        "bootloader" : { "offset" : "0x110000", "file" : "", "encrypted" : "" },
        "app" : { "offset" : "0x210000", "file" : "", "encrypted" : "" },
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

class FlasherTestsSingleEncrypted(DebuggerGenericTestAppTestsSingleEncrypted, FlasherTestsImpl):
    """ Encrypted flash test cases in single core mode
    """
    def setUp(self):
        DebuggerGenericTestAppTestsSingleEncrypted.setUp(self)
        FlasherTestsImpl.setUp(self)

@idf_ver_min('latest')
@only_for_chip(['esp32c6', 'esp32h2'])
class FlasherTestsPreloadedStubSingle(DebuggerGenericTestAppTestsSingle):

    def __init__(self, methodName='runTest'):
        super(FlasherTestsPreloadedStubSingle, self).__init__(methodName)
        self.test_app_cfg.bin_dir = os.path.join('output', 'single_core_preloaded_stub')
        self.test_app_cfg.build_dir = os.path.join('builds', 'single_core_preloaded_stub')

    def test_preloaded_stub_binary(self):
        """
            This test checks if stub codes already loaded to the targets and functioning as expected
        """
        expected_strings = ["Stub flasher will be running from preloaded image (5C3A9F5A)",
                            "Flash mapping 0:",
                            "Flash mapping 1:"]

        target_output = ''
        def _target_stream_handler(type, stream, payload):
            nonlocal target_output
            target_output += payload
        self.gdb.stream_handler_add('target', _target_stream_handler)

        self.gdb.monitor_run("esp stub_log off", 5)
        self.gdb.monitor_run("flash probe 0", 5)
        self.gdb.stream_handler_remove('target', _target_stream_handler)

        for expected_str in expected_strings:
            self.assertIn(expected_str, target_output, f"Expected string '{expected_str}' not found in output")
