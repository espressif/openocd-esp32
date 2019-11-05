import logging
import unittest
import tempfile
import filecmp
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
        size = (ESP32_FLASH_SZ - (ESP32_APP_FLASH_OFF + ESP32_APP_FLASH_SZ))/1024
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
        self.run_to_bp_and_check(dbg.Gdb.TARGET_STOP_REASON_BP, 'gpio_set_direction', ['gpio_set_direction'], outmost_func_name='cache_check_task')
        for i in range(5):
            self.run_to_bp_and_check(dbg.Gdb.TARGET_STOP_REASON_BP, 'gpio_set_level', ['gpio_set_level'], outmost_func_name='cache_check_task')


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
