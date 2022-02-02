import logging
import unittest
import subprocess
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

    @idf_ver_min_for_arch('latest', ['riscv32'])
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

    @skip_for_chip(['esp32s3'])
    def test_gdb_regs_mapping(self):
        """
            This test checks that GDB and OpenOCD has identical registers mapping.
            1) Cycles over registers and assigns them specific values using GDB command
            2) Uses OpenOCD command to check that registers have expected values
        """
        # should fail for any new chip.
        # just to be sure that this test is revised when new chip support is added
        self.fail_if_not_hw_id([r'esp32-[.]*', r'esp32s2-[.]*', r'esp32c3-[.]*', r'esp32s3-[.]*'])
        regs = self.gdb.get_reg_names()
        i = 10
        for reg in regs:
            if (len(reg) == 0 or reg == 'zero'):
                continue

            # TODO: With the new gdb version (gdb9) privileged regs now can be set. So, break condition needs to be changed for each chip.
            # eg. Now mmid is pass but it is failing at a0 for esp32
            if reg == 'mmid' or reg == 'mstatus' or reg == 'q0':
                break

            # set to reasonable value, because GDB tries to read memory @ pc
            val = 0x40000400 if reg == 'pc' else i
            self.gdb.set_reg(reg, val)
            self.gdb.console_cmd_run('flushregs')
            self.assertEqual(self.gdb.get_reg(reg), val)
            _,res_str = self.gdb.monitor_run('reg %s' % reg, output_type='stdout')
            self.assertEqual(self.oocd.parse_reg_val(reg, res_str), val)
            i += 1
        # reset chip to clear all changes in regs, otherwise the next test can fail
        self.gdb.target_reset()

    def _debug_image(self):
        self.select_sub_test(100)
        bps = ['app_main', 'gpio_set_direction', 'gpio_set_level', 'vTaskDelay']
        for f in bps:
            self.add_bp(f)
        # break at gpio_set_direction
        self.run_to_bp_and_check(dbg.TARGET_STOP_REASON_BP, 'gpio_set_direction', ['gpio_set_direction'])
        # break at gpio_set_level
        self.run_to_bp_and_check(dbg.TARGET_STOP_REASON_BP, 'gpio_set_level', ['gpio_set_level0'])
        # break at vTaskDelay
        self.run_to_bp_and_check(dbg.TARGET_STOP_REASON_BP, 'vTaskDelay', ['vTaskDelay0'])
        self.clear_bps()

    def test_debugging_works_after_hw_reset(self):
        """
            This test checks that debugging works after HW reset.
            1) Select appropriate sub-test number on target.
            2) Resume target and wait some time.
            4) Run `esptool.py` to get chip ID and reset target.
            5) Wait some time.
            6) Run simple debug session.
        """
        # avoid simultaneous access to UART with SerialReader
        self.assertIsNone(self.uart_reader, "Can not run this test with UART logging enabled!")
        self.select_sub_test(100)
        self.resume_exec()
        time.sleep(2.0)
        if self.port_name:
            cmd = ['esptool.py', '-p', self.port_name, '-a', 'hard_reset', 'chip_id']
        else:
            cmd = ['esptool.py', '-a', 'hard_reset', 'chip_id']
        proc = subprocess.run(cmd)
        proc.check_returncode()
        time.sleep(2.0)
        self.stop_exec()
        self.prepare_app_for_debugging(self.test_app_cfg.app_off)
        self._debug_image()

    @idf_ver_min('4.3')
    @idf_ver_min_for_arch('latest', ['riscv32'])
    def test_bp_and_wp_set_by_program(self):
        """
            This test checks that breakpoints and watchpoints set by program on target work.
            1) Select appropriate sub-test number on target.
            2) Resume target, wait for the program to hit breakpoints.
        """
        self.select_sub_test(803)
        # breakpoint at 'target_bp_func1' entry
        self.run_to_bp_and_check_location(dbg.TARGET_STOP_REASON_SIGTRAP, 'target_bp_func1', 'target_bp_func1')
        # watchpoint hit on write var in 'target_bp_func1'
        self.run_to_bp_and_check_location(dbg.TARGET_STOP_REASON_SIGTRAP, 'target_bp_func1', 'target_wp_var1_1')
        # watchpoint hit on read var in 'target_bp_func1'
        self.run_to_bp_and_check_location(dbg.TARGET_STOP_REASON_SIGTRAP, 'target_bp_func1', 'target_wp_var1_2')
        # breakpoint at 'target_bp_func2' entry
        self.run_to_bp_and_check_location(dbg.TARGET_STOP_REASON_SIGTRAP, 'target_bp_func2', 'target_bp_func2')
        # watchpoint hit on write var in 'target_bp_func2'
        self.run_to_bp_and_check_location(dbg.TARGET_STOP_REASON_SIGTRAP, 'target_bp_func2', 'target_wp_var2_1')
        # watchpoint hit on read var in 'target_bp_func2'
        self.run_to_bp_and_check_location(dbg.TARGET_STOP_REASON_SIGTRAP, 'target_bp_func2', 'target_wp_var2_2')


# to be skipped for any board with ESP32-S2 chip
# TODO: enable these tests when PSRAM is supported for ESP32-S2
@skip_for_chip(['esp32s2', 'esp32c3'])
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
    @skip_for_chip(['esp32s3'])
    def test_cores_states_after_esptool_connection(self):
        """
            This test checks that cores are in running or halted state after esptool connection.
            1) Select appropriate sub-test number on target.
            2) Resume target and wait some time.
            3) Check that all targets are in state 'running'.
            4) Run `esptool.py` to get chip ID and reset target.
            5) Wait some time.
            6) Check that all targets are in state 'running'.
        """
        # avoid simultaneous access to UART with SerialReader
        self.assertIsNone(self.uart_reader, "Can not run this test with UART logging enabled!")
        self.select_sub_test(100)
        self.resume_exec()
        time.sleep(2.0)
        for target in self.oocd.targets():
            state = self.oocd.target_state(target)
            self.assertEqual(state, 'running')
        if self.port_name:
            cmd = ['esptool.py', '-p', self.port_name, 'chip_id']
        else:
            cmd = ['esptool.py', 'chip_id']
        proc = subprocess.run(cmd)
        proc.check_returncode()
        time.sleep(2.0)
        for target in self.oocd.targets():
            state = self.oocd.target_state(target)
            self.assertEqual(state, 'running')

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
@skip_for_chip(['esp32-solo'])
class PsramTestsSingle(PsramTestAppTestsSingle, PsramTestsImpl):
    """ Test cases via GDB in single core mode
    """
    def setUp(self):
        PsramTestAppTestsSingle.setUp(self)
        PsramTestsImpl.setUp(self)
