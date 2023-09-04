import logging
import unittest
import subprocess
import re
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

    @idf_ver_min_for_arch('5.0', ['riscv32'])
    def test_restart_debug_from_crash(self):
        """
            This test checks that debugger can operate correctly after SW reset with stalled CPU.
            1) Select appropriate sub-test number on target.
            2) Resume target, wait for the program to crash (in dual core mode one CPU will be stalled).
            3) Re-start debugging (SW reset, set break to app_main(), resume and wait for the stop).
        """
        # under OOCD panic handler sets BP on instruction which generated the exception and stops execution there
        self.run_to_bp_and_check(dbg.TARGET_STOP_REASON_SIGTRAP, 'restart_debug_from_crash_task', ['crash'], outmost_func_name='restart_debug_from_crash_task')
        self.prepare_app_for_debugging(self.test_app_cfg.app_off)

    def _debug_image(self):
        self.select_sub_test("blink")
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

    @skip_for_chip(['esp32s3', 'esp32'])
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
        self.select_sub_test("blink")
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

    def _do_test_bp_and_wp_set_by_program(self):
        # breakpoint at 'target_bp_func1' entry
        self.run_to_bp_and_check_location(dbg.TARGET_STOP_REASON_SIGTRAP, 'target_bp_func1', 'target_bp_func1')
        # watchpoint hit on write var in 'target_bp_func1'
        self.run_to_bp_and_check_location(dbg.TARGET_STOP_REASON_SIGTRAP, 'target_bp_func1', 'target_wp_var1_1')
        # watchpoint hit on read var in 'target_bp_func1'
        self.run_to_bp_and_check_location(dbg.TARGET_STOP_REASON_SIGTRAP, 'target_bp_func1', 'target_wp_var1_2')
        # esp32c2 has only 2 hw triggers
        if testee_info.chip != "esp32c2":
            # breakpoint at 'target_bp_func2' entry
            self.run_to_bp_and_check_location(dbg.TARGET_STOP_REASON_SIGTRAP, 'target_bp_func2', 'target_bp_func2')
            # watchpoint hit on write var in 'target_bp_func2'
            self.run_to_bp_and_check_location(dbg.TARGET_STOP_REASON_SIGTRAP, 'target_bp_func2', 'target_wp_var2_1')
            # watchpoint hit on read var in 'target_bp_func2'
            self.run_to_bp_and_check_location(dbg.TARGET_STOP_REASON_SIGTRAP, 'target_bp_func2', 'target_wp_var2_2')

    def test_bp_and_wp_set_by_program(self):
        """
            This test checks that breakpoints and watchpoints set by program on target work.
            1) Select appropriate sub-test number on target.
            2) Resume target, wait for the program to hit breakpoints.
        """
        self._do_test_bp_and_wp_set_by_program()

    def test_wp_reconfigure_by_program(self):
        """
            This test checks that watchpoints can be reconfigured by target w/o removing them.
            1) Select appropriate sub-test number on target.
            2) Resume target, wait for the program to hit breakpoints.
        """
        self._do_test_bp_and_wp_set_by_program()

    def test_exception(self):
        """
        This test checks that expected exception cause string equal to the OpenOCD output.
        """
        bps = ["exception_bp_1", "exception_bp_2", "exception_bp_3"]
        if testee_info.arch == "xtensa":
            bps.append("exception_bp_4");
            sub_tests = ["illegal_instruction", "load_prohibited", "store_prohibited", "divide_by_zero"]
            expected_strings = ["Halt cause (0) - (Illegal instruction)",
                            "Halt cause (28) - (Load prohibited)",
                            "Halt cause (29) - (Store prohibited)",
                            "Halt cause (6) - (Integer divide by zero)"]
        else: 
            sub_tests = ["illegal_instruction", "load_access_fault", "store_access_fault"]
            expected_strings = ["Halt cause (2) - (Illegal Instruction)",
                                "Halt cause (5) - (PMP Load access fault)",
                                "Halt cause (7) - (PMP Store access fault)"]

        #TODO: enable after IDF MR(25708) merged
        if 0:
        #if testee_info.idf_ver >= IdfVersion.fromstr('latest'):
            # Pseudo exeption tests only implemented for xtensa
            if testee_info.arch == "xtensa":
                bps.append("exception_bp_5");
                bps.append("exception_bp_6");
                sub_tests.append("pseudo_debug")
                sub_tests.append("pseudo_coprocessor")
                expected_strings.append("Halt cause (Unhandled debug exception)")
                expected_strings.append("Halt cause (Coprocessor exception)")

            bps.append("assert_failure_bp")
            sub_tests.append("assert_failure")
            expected_strings.append("Halt cause \(assert failed: assert_failure_ex_task special_tests.c:[^\n]+\)")

            bps.append("abort_bp")
            sub_tests.append("abort")
            expected_strings.append("Halt cause \(abort\(\) was called at PC 0x[0-9a-fA-F]+ on core [0-9]+\)")

        for i in range (len(bps)):
            self.add_bp(bps[i])
            self.select_sub_test(self.id() + '_' + sub_tests[i])
            self.resume_exec()
            rsn = self.gdb.wait_target_state(dbg.TARGET_STATE_STOPPED, 5)
            self.assertEqual(rsn, dbg.TARGET_STOP_REASON_BP)
            old_pc = self.gdb.get_reg('pc')
            faddr = self.gdb.extract_exec_addr(self.gdb.data_eval_expr('&%s' % bps[i]))
            self.assertEqual(old_pc, faddr)
            self.clear_bps()
            target_output = ''
            def _target_stream_handler(type, stream, payload):
                nonlocal target_output
                target_output += payload
            self.gdb.stream_handler_add('target', _target_stream_handler)

            try:
                self.resume_exec()
                rsn = self.gdb.wait_target_state(dbg.TARGET_STATE_STOPPED , 5)
                self.assertTrue(rsn == dbg.TARGET_STATE_STOPPED or rsn == dbg.TARGET_STOP_REASON_SIGTRAP)
                self.step() # Without step OpenOCD may not print the exception cause
            finally:
                self.gdb.stream_handler_remove('target', _target_stream_handler)

            if any(word in sub_tests[i] for word in ["assert", "abort"]):
				# On assert and abort panics, file line numbers or PC value can be vary.
                # Therefore, we will use regex pattern for the matching expected strings.
                pattern = re.compile(expected_strings[i])
                if testee_info.arch == "xtensa":
                    match = re.search(pattern, target_output)
                    self.assertTrue(match)
                # On RISC-V, when the SIGTRAP signal is received from GDB, there is no corresponding MI response from the target.
                # As a result, the OpenOCD output is not visible in the gdb logs.
                # Therefore, we will need to search for the expected strings in the OpenOCD log file instead.
                else:
                    log_path = get_logger().handlers[1].baseFilename
                    found_line_count = 0
                    with open(log_path) as file:
                        for line in file:
                            match = re.search(pattern, line)
                            if match:
                                found_line_count += 1
                                break
                    self.assertTrue(found_line_count)
            else:
               self.assertTrue(expected_strings[i] in target_output)
            self.gdb.target_reset()
            self.add_bp('app_main')
            self.run_to_bp(dbg.TARGET_STOP_REASON_BP, 'app_main')

    def test_stub_logs(self):
        """
            This test checks if stub logs are enabled successfully.
        """
        expected_strings = ["STUB_D: cmd 5:FLASH_MAP_GET",
                            "STUB_D: cmd 4:FLASH_SIZE"]

        self.gdb.monitor_run("esp stub_log on", 5)
        self.gdb.monitor_run("flash probe 0", 5)
        self.gdb.monitor_run("esp stub_log off", 5)

        log_path = get_logger().handlers[1].baseFilename # 0:StreamHandler 1:FileHandler
        found_line_count = 0
        with open(log_path) as file:
            for line in file:
                for s in expected_strings:
                    if s in line:
                        found_line_count += 1
        # We expect at least len(expected_strings) for one core.
        self.assertTrue(found_line_count >= len(expected_strings))


# PSRAM is supported for Xtensa chips only
@only_for_arch(['xtensa'])
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
        self.run_to_bp_and_check(dbg.TARGET_STOP_REASON_BP, 'gpio_set_direction', ['gpio_set_direction'], outmost_func_name='psram_with_flash_breakpoints_task')
        for i in range(10):
            # break at gpio_set_level
            self.run_to_bp_and_check(dbg.TARGET_STOP_REASON_BP, 'gpio_set_level', ['gpio_set_level%d' % (i % 2)], outmost_func_name='psram_with_flash_breakpoints_task')
            # break at vTaskDelay
            self.run_to_bp_and_check(dbg.TARGET_STOP_REASON_BP, 'vTaskDelay', ['vTaskDelay%d' % (i % 2)], outmost_func_name='psram_with_flash_breakpoints_task')

    def test_psram_with_flash_breakpoints_gh264(self):
        """
            GH issue reported for ESP32-S3. See https://github.com/espressif/openocd-esp32/issues/264
            This test checks that PSRAM memory contents ard not corrupted when using flash SW breakpoints.
            1) Select appropriate sub-test number on target.
            2) Resume target, wait for the program to stop at the places where we set breakpoints.
            3) Target program checks PSRAM memory contents and calls 'assert()' in case of error,
            so test expects propgram to be stopped on breakpoints only. Stop at the call to 'assert()' is a failure.
        """
        # 2 HW breaks + 1 flash SW break + RAM SW break
        bps = ['gh264_psram_check_bp_1', 'gh264_psram_check_bp_2', 'gh264_psram_check_bp_3']
        for f in bps:
            self.add_bp(f)
        for i in range(3):
            self.run_to_bp_and_check_location(dbg.TARGET_STOP_REASON_BP, 'gh264_psram_check_task', 'gh264_psram_check_1')
            self.run_to_bp_and_check_location(dbg.TARGET_STOP_REASON_BP, 'gh264_psram_check_task', 'gh264_psram_check_2')
            self.run_to_bp_and_check_location(dbg.TARGET_STOP_REASON_BP, 'gh264_psram_check_task', 'gh264_psram_check_3')


########################################################################
#              TESTS DEFINITION WITH SPECIAL TESTS                     #
########################################################################

class DebuggerSpecialTestsDual(DebuggerGenericTestAppTestsDual, DebuggerSpecialTestsImpl):
    """ Test cases for dual core mode
    """
    @skip_for_chip(['esp32s3', 'esp32'])
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
        self.select_sub_test("blink")
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

    def test_gdb_regs_mapping(self):
        """
            This test checks that GDB and OpenOCD has identical registers mapping.
            1) Cycles over registers and assigns them specific values using GDB command
            2) Uses OpenOCD command to check that registers have expected values
        """
        # should fail for any new chip.
        # just to be sure that this test is revised when new chip support is added
        self.fail_if_not_hw_id([r'esp32-[.]*', r'esp32s2-[.]*', r'esp32c2-[.]*', r'esp32c3-[.]*', r'esp32s3-[.]*', r'esp32c6-[.]*', r'esp32h2-[.]*'])
        regs = self.gdb.get_reg_names()
        i = 10

        # GDB sets registers in current thread, here we assume that it always belongs to CPU0
        # select CPU0 as current target in OpenOCD for multi-core chips
        _, res_str = self.gdb.monitor_run('target names', output_type='stdout')
        if res_str.endswith('\\n'):
            res_str = res_str[:-2]
        targets = res_str.split()
        if len(targets) > 1:
            for t in targets:
                if t.endswith('.cpu0'):
                    self.gdb.monitor_run('targets %s' % t, output_type='stdout')
                    break

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
            read_val = self.oocd.parse_reg_val(reg, res_str)
            get_logger().debug("Check reg value '%s': %d == %d", reg, read_val, val)
            self.assertEqual(read_val, val)
            i += 1
        # reset chip to clear all changes in regs, otherwise the next test can fail
        self.gdb.target_reset()


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
