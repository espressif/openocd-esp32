import logging
import unittest
import debug_backend as dbg
from debug_backend_tests import *


def get_logger():
    """ Returns logger for this module
    """
    return logging.getLogger(__name__)


########################################################################
#                         TESTS IMPLEMENTATION                         #
########################################################################

class DebuggerSpecialTestsImpl:
    """ Special test cases generic for dual and single core modes
    """

    def test_restart_debug_from_crash(self):
        """
            This test checks that debugger can operate correctly after SW reset with stalled CPU.
            1) Select appropriate sub-test number on target.
            2) Resume target, wait for the program to crash (in dual core mode one CPU will be stalled).
            3) Re-start debugging (SW reset, set break to app_main(), resume and wait for the stop).
        """
        self.select_sub_test(800)
        # under OOCD panic handler sets BP on instruction which generated the exception and stops execution there
        self.run_to_bp_and_check(dbg.TARGET_STOP_REASON_SIGTRAP, 'crash_task', ['crash'], outmost_func_name='crash_task')
        self.prepare_app_for_debugging(self.test_app_cfg.app_off)


# to be skipped for any board with ESP32-S2 chip
# TODO: enable these tests when PSRAM is supported for ESP32-S2
@skip_for_hw_id([r'esp32s2-[.]*'])
class PsramTestsImpl:
    """ PSRAM specific test cases generic for dual and single core modes
    """
    def setUp(self):
        pass

    def test_psram_with_flash_breakpoints(self):
        """
            This test checks that PSRAM memory contents ard not corrupted when using flash SW breakpoints.
            1) Select appropriate sub-test number on target.
            2) Resume target, wait for the program to stop at the places where we set breakpoints.
            3) Target program checks PSRAM memory contents and calls 'assert()' in case of error,
            so test expects propgram to be stopped on breakpoints only. Stop at the call to 'assert()' is a failure.
        """
        # 2 HW breaks + 1 flash SW break + RAM SW break
        bps = ['app_main', 'gpio_set_direction', 'gpio_set_level', 'vTaskDelay']
        for f in bps:
            self.add_bp(f)
        self.select_sub_test(802)
        self.run_to_bp_and_check(dbg.TARGET_STOP_REASON_BP, 'gpio_set_direction', ['gpio_set_direction'], outmost_func_name='psram_check_task')
        for i in range(10):
            # break at gpio_set_level
            self.run_to_bp_and_check(dbg.TARGET_STOP_REASON_BP, 'gpio_set_level', ['gpio_set_level%d' % (i % 2)], outmost_func_name='psram_check_task')
            # break at vTaskDelay
            self.run_to_bp_and_check(dbg.TARGET_STOP_REASON_BP, 'vTaskDelay', ['vTaskDelay%d' % (i % 2)], outmost_func_name='psram_check_task')

########################################################################
#              TESTS DEFINITION WITH SPECIAL TESTS                     #
########################################################################

class DebuggerSpecialTestsDual(DebuggerGenericTestAppTestsDual, DebuggerSpecialTestsImpl):
    """ Test cases for dual core mode
    """
    pass

class DebuggerSpecialTestsSingle(DebuggerGenericTestAppTestsSingle, DebuggerSpecialTestsImpl):
    """ Test cases for single core mode
    """
    pass


class PsramTestAppTestsDual(DebuggerGenericTestAppTests):
    """ Base class to run tests which use PSRAM test app in dual core mode
    """

    def __init__(self, methodName='runTest'):
        super(PsramTestAppTestsDual, self).__init__(methodName)
        self.test_app_cfg.bin_dir = os.path.join('output', 'psram_dual')
        self.test_app_cfg.build_dir = os.path.join('builds', 'psram_dual')


class PsramTestAppTestsSingle(DebuggerGenericTestAppTests):
    """ Base class to run tests which use PSRAM test app in single core mode
    """

    def __init__(self, methodName='runTest'):
        super(PsramTestAppTestsSingle, self).__init__(methodName)
        self.test_app_cfg.bin_dir = os.path.join('output', 'psram_single')
        self.test_app_cfg.build_dir = os.path.join('builds', 'psram_single')


class PsramTestsDual(PsramTestAppTestsDual, PsramTestsImpl):
    """ Test cases via GDB in dual core mode
    """
    def setUp(self):
        PsramTestAppTestsDual.setUp(self)
        PsramTestsImpl.setUp(self)

# to be skipped for any board with 'esp32-solo' module, but still needs to be ran
# for dual-core version of ESP32 modules even in single-core mode
@skip_for_hw_id([r'esp32-solo[.]*'])
class PsramTestsSingle(PsramTestAppTestsSingle, PsramTestsImpl):
    """ Test cases via GDB in single core mode
    """
    def setUp(self):
        PsramTestAppTestsSingle.setUp(self)
        PsramTestsImpl.setUp(self)
