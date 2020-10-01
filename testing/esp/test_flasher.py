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

    def test_big_binary(self):
        """
            This test checks flashing big binaries works.
            1) Create test binary file of the most possible size.
            2) Fill it with random data.
            3) Write the file to the flash.
            4) Read written data to another file.
            5) Compare files.
        """
        fhnd,fname1 = tempfile.mkstemp()
        fbin = os.fdopen(fhnd, 'wb')
        size = int((ESP32_FLASH_SZ - (ESP32_APP_FLASH_OFF + ESP32_APP_FLASH_SZ))/1024)
        get_logger().debug('Generate random file %dKB "%s"', size, fname1)
        for i in range(size):
            fbin.write(os.urandom(1024))
        fbin.close()
        self.gdb.target_program(fname1, ESP32_APP_FLASH_OFF + ESP32_APP_FLASH_SZ, actions='', tmo=130)
        # since we can not get result from OpenOCD (output parsing seems not to be good idea),
        # we need to read written flash and compare data manually
        fhnd,fname2 = tempfile.mkstemp()
        fbin = os.fdopen(fhnd, 'wb')
        fbin.close()
        self.gdb.monitor_run('flash read_bank 0 %s 0x%x %d' % (dbg.fixup_path(fname2), ESP32_APP_FLASH_OFF + ESP32_APP_FLASH_SZ, size*1024), tmo=120)
        self.assertTrue(filecmp.cmp(fname1, fname2))

    def test_cache_handling(self):
        """
            This test checks that flasher does not corrupts cache config registers when setting breakpoints.
            The idea is the following: GDB postpones writing breakpoints to resume.
            So cache config registers should have the same values after resuming with set flash breakpoints.
            It is checked by the test code on target. This test method sets breakpoint and resumes execution several times.
            Before hiting the BP program save cache config and after resuming checks its value.
        """
        self.select_sub_test(801)
        # 2 HW + 1 SW flash BP
        self.bps = ['app_main', 'gpio_set_direction', 'gpio_set_level']
        for f in self.bps:
            self.add_bp(f)
        self.run_to_bp_and_check(dbg.TARGET_STOP_REASON_BP, 'gpio_set_direction', ['gpio_set_direction'], outmost_func_name='cache_check_task')
        for i in range(5):
            self.run_to_bp_and_check(dbg.TARGET_STOP_REASON_BP, 'gpio_set_level', ['gpio_set_level'], outmost_func_name='cache_check_task')

    def test_program_esp_bins(self):
        """
            This test checks flashing complete app works using flasher_args.json.
            1) Generate a dummy flasher_args.json file
            2) Create random binaries based on the flasher_args.json
            3) Write the files to the flash.
            4) Read written data to another file.
            5) Compare files.
        """
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
        
        # Write the flasher_args file
        json_fname = "flasher_args.json"
        json_fpath = os.path.join(tmp, json_fname)
        json_fp = open(json_fpath, "w")
        json.dump(obj, fp=json_fp, indent=2)
        json_fp.close()

        # Flash the chip
        self.gdb.monitor_run("program_esp_bins %s %s reset verify" % (tmp, json_fname))
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
        "partition_table" : { "offset" : "0x118000", "file" : "" },
        "bootloader" : { "offset" : "0x110000", "file" : "" },
        "app" : { "offset" : "0x210000", "file" : "" },
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
    # no special tests for single core mode yet
    pass

class FlasherTestsSingle(DebuggerGenericTestAppTestsSingle, FlasherTestsImpl):
    """ Test cases in single core mode
    """
    # no special tests for single core mode yet
    pass
