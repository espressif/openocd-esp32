import logging
import debug_backend as dbg
from debug_backend_tests import *
import test_bp


def get_logger():
    return logging.getLogger(__name__)


########################################################################
#              TESTS DEFINITION WITH SPECIAL TESTS                     #
########################################################################

class MultiAppImagesTests(DebuggerGenericTestAppTests):

    def __init__(self, methodName):
        DebuggerGenericTestAppTests.__init__(self, methodName)
        if IdfVersion.get_current() < IdfVersion.fromstr('4.0'):
            self.test_app_cfg.pt_path = 'partitions_multi_apps.bin'
        else:
            self.test_app_cfg.pt_path = os.path.join('partition_table', 'partition-table.bin')
        self.test_app_cfg.bin_dir = os.path.join('output', 'multi_app_images')
        self.test_app_cfg.build_dir = os.path.join('builds', 'multi_app_images')
        # this is very specific and unusual test scenario, the offsets must be in sync with OTA ones from 'partitions_multi_apps.csv'
        self.extra_app_image_offs = [self.test_app_cfg.app_off+1*1024*1024, self.test_app_cfg.app_off+2*1024*1024]

    def setUp(self):
        DebuggerGenericTestAppTests.setUp(self)
        # 2 HW breaks + 1 flash SW break + RAM SW break
        self.bps = ['app_main', 'gpio_set_direction', 'gpio_set_level', 'vTaskDelay']

    def _debug_image(self, off):
        self.select_sub_test(100)
        for f in self.bps:
            self.add_bp(f)
        # break at gpio_set_direction
        self.run_to_bp_and_check(dbg.Gdb.TARGET_STOP_REASON_BP, 'gpio_set_direction', ['gpio_set_direction'])
        # break at gpio_set_level
        self.run_to_bp_and_check(dbg.Gdb.TARGET_STOP_REASON_BP, 'gpio_set_level', ['gpio_set_level0'])
        # break at vTaskDelay
        self.run_to_bp_and_check(dbg.Gdb.TARGET_STOP_REASON_BP, 'vTaskDelay', ['vTaskDelay0'])
        self.clear_bps()
        # erase current image to allow bootloader to start one from the next app partition after reset
        self.oocd.cmd_exec('flash erase_address 0x%x 4096' % off)

    def test_debug_images(self):
        """
            This test checks that it is possible to debug applications from the various locations in flash.
            The test does the following:
            1) Set several breakpoints to cover all types of them (HW, SW).
            2) Resume target and wait for brekpoints to hit.
            3) Check that target has stopped in the right place.
            4) Check backtrace at the stop point.
            5) Removes all breakpoints.
            6) Repeat above steps for different offsets for application images in flash.
        """
        # at first debug factory image and erase it to allow bootloader to run the first OTA image next time
        self._debug_image(self.test_app_cfg.app_off)
        bin_dir = os.path.join(test_apps_dir, self.test_app_cfg.app_name, 'output', 'default')
        for off in self.extra_app_image_offs:
            # program and debug the same app (from default dual core config) with different flash mappings
            self.gdb.target_program(os.path.join(bin_dir, '%s.bin' % self.test_app_cfg.app_name), off)
            self.gdb.exec_file_set(os.path.join(bin_dir, '%s.elf' % self.test_app_cfg.app_name))
            self.prepare_app_for_debugging(off)
            # debug OTA image and erase it to allow bootloader to run the next OTA image after reset
            self._debug_image(off)

