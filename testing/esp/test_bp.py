import logging
import unittest
import debug_backend as dbg
from debug_backend_tests import *
from time import sleep


def get_logger():
    """ Returns logger for this module
    """
    return logging.getLogger(__name__)


########################################################################
#                         TESTS IMPLEMENTATION                         #
########################################################################

class BreakpointTestsImpl:
    """ Breakpoints test cases which are common for dual and single core modes
    """

    def setUp(self):
        self.bps = []
        if testee_info.chip == "esp32c3":
            # esp32c3 has 8 HW breakpoint slots
            # 6 dummy HW breaks to fill in HW breaks slots and make OpenOCD using SW breakpoints in flash (seen as HW ones by GDB)
            self.bps = ['unused_func0', 'unused_func1', 'unused_func2', 'unused_func3', 'unused_func4', 'unused_func5']
        elif testee_info.chip == "esp32c6" or testee_info.chip == "esp32h2":
            # esp32c6 has 4 HW breakpoint slots
            # 2 dummy HW breaks to fill in HW breaks slots and make OpenOCD using SW breakpoints in flash (seen as HW ones by GDB)
            self.bps = ['unused_func0', 'unused_func1']
        # + 2 HW breaks + 1 flash SW break + RAM SW break
        self.bps += ['app_main', 'gpio_set_direction', 'gpio_set_level', 'vTaskDelay']

    def test_multi_reset_break(self):
        """
            This test checks that breakpoint works just after the reset:
            1) Reset the board.
            2) Set breakpoint.
            3) Resume target and wait for brekpoint hit.
            4) Delete breakpoint.
            5) Repeat steps 1-4 several times.
        """
        for i in range(3):
            self.gdb.target_reset()
            rsn = self.gdb.wait_target_state(dbg.TARGET_STATE_STOPPED, 5)
            self.add_bp('app_main')
            self.resume_exec()
            rsn = self.gdb.wait_target_state(dbg.TARGET_STATE_STOPPED, 5)
            self.gdb.delete_bp(self.bpns.pop())
            self.assertEqual(rsn, dbg.TARGET_STOP_REASON_BP)
            cur_frame = self.gdb.get_current_frame()
            self.assertEqual(cur_frame['func'], 'app_main')

    def readd_bps(self):
        # remove all BPs except the first one
        for i in range(1, len(self.bpns)):
            self.gdb.delete_bp(self.bpns[i])
        self.bpns = self.bpns[:1]
        # add removed BPs back
        for i in range(1, len(self.bps)):
            self.add_bp(self.bps[i])

    def test_bp_add_remove_run(self):
        """
            This simple test checks general breakpoints usage scenario.
            1) Select appropriate sub-test number on target.
            2) Set several breakpoints to cover all types of them (HW, SW).
            3) Resume target and wait for brekpoints to hit.
            4) Check that target has stopped in the right place.
            5) Check backtrace at the stop point.
            6) Removes several breakpoints and adds them again.
            7) Repeat steps 3-5 several times.
        """
        self.select_sub_test("blink")
        for f in self.bps:
            self.add_bp(f)
        self.readd_bps()
        # break at gpio_set_direction
        self.run_to_bp_and_check(dbg.TARGET_STOP_REASON_BP, 'gpio_set_direction', ['gpio_set_direction'])
        self.readd_bps()
        for i in range(2):
            # break at gpio_set_level
            self.run_to_bp_and_check(dbg.TARGET_STOP_REASON_BP, 'gpio_set_level', ['gpio_set_level%d' % (i % 2)])
            self.readd_bps()
            # break at vTaskDelay
            self.run_to_bp_and_check(dbg.TARGET_STOP_REASON_BP, 'vTaskDelay', ['vTaskDelay%d' % (i % 2)])
            self.readd_bps()

    def test_bp_ignore_count(self):
        """
            This test checks that the first several breakpoint's hits can be ignored:
            1) Select appropriate sub-test number on target.
            2) Set several breakpoints to cover all types of them (HW, SW). One of the breakpoints uses 'ignore count' GDB option.
            3) Resume target and wait for brekpoints to hit.
            4) Check that target has stopped in the right place.
            5) Check that conditioned breakpoint stopped the target only when its 'ignore count' is exceeded.
            6) Check backtrace at the stop point.
            7) Repeat steps 3-6 several times.
        """
        self.select_sub_test("blink")
        for f in self.bps:
            if f == 'vTaskDelay':
                self.add_bp(f, ignore_count=2)
            else:
                self.add_bp(f)
        # break at gpio_set_direction
        self.run_to_bp_and_check(dbg.TARGET_STOP_REASON_BP, 'gpio_set_direction', ['gpio_set_direction'])
        for i in range(3):
            # break at gpio_set_level
            self.run_to_bp_and_check(dbg.TARGET_STOP_REASON_BP, 'gpio_set_level', ['gpio_set_level%d' % (i % 2)])
            if i > 1:
                # break at vTaskDelay
                self.run_to_bp_and_check(dbg.TARGET_STOP_REASON_BP, 'vTaskDelay', ['vTaskDelay%d' % (i % 2)])

    def test_bp_cond_expr(self):
        """
            This test checks that conditional breakpoint using expression works:
            1) Select appropriate sub-test number on target.
            2) Set several breakpoints to cover all types of them (HW, SW). One of the breakpoints uses 'conditional expression' GDB option.
            3) Resume target and wait for brekpoints to hit.
            4) Check that target has stopped in the right place.
            5) Check that conditioned breakpoint stopped the target only when its 'conditional expression' is true.
            6) Check backtrace at the stop point.
            7) Repeat steps 3-6 several times.
        """
        self.select_sub_test("blink")
        for f in self.bps:
            if f == 'vTaskDelay':
                self.add_bp(f, cond='s_count1 == 1')
            else:
                self.add_bp(f)
        # break at gpio_set_direction
        self.run_to_bp_and_check(dbg.TARGET_STOP_REASON_BP, 'gpio_set_direction', ['gpio_set_direction'])
        for i in range(6):
            # break at gpio_set_level
            self.run_to_bp_and_check(dbg.TARGET_STOP_REASON_BP, 'gpio_set_level', ['gpio_set_level%d' % (i % 2)])
            # 'count' var is increased once per two calls to vTaskDelay
            if i == 2 or i == 3:
                # break at vTaskDelay
                self.run_to_bp_and_check(dbg.TARGET_STOP_REASON_BP, 'vTaskDelay', ['vTaskDelay%d' % (i % 2)])

    def test_bp_and_reconnect(self):
        """
            This test checks that breakpoints work after GDB re-connection.
            Also this test checks that OOCD handles such re-connections safely.
            1) Select appropriate sub-test number on target.
            2) Set several breakpoints to cover all types of them (HW, SW).
            3) Resume target and wait for any brekpoint to hit.
            4) Check that target has stopped in the right place.
            5) Check backtrace at the stop point.
            6) Disconnect GDB from OOCD.
            7) Connect GDB to OOCD.
            8) Repeat steps 3-7 several times.
        """
        self.select_sub_test("blink")
        for f in self.bps:
            self.add_bp(f)
        for i in range(5):
            self.resume_exec()
            rsn = self.gdb.wait_target_state(dbg.TARGET_STATE_STOPPED, 5)
            self.assertEqual(rsn, dbg.TARGET_STOP_REASON_BP)
            cur_frame = self.gdb.get_current_frame()
            self.assertTrue(cur_frame['func'] in self.bps)
            frames = self.gdb.get_backtrace()
            self.assertTrue(len(frames) > 0)
            self.assertEqual(frames[0]['func'], cur_frame['func'])
            self.assertEqual(frames[0]['line'], cur_frame['line'])
            self.gdb.disconnect()
            sleep(0.1) #sleep 100ms
            self.gdb.connect()

    def test_bp_in_isr(self):
        """
            This test checks that the breakpoint's are handled in ISR properly:
            1) Select appropriate sub-test number on target.
            2) Set several breakpoints in ISR code to cover all types of them (HW, SW).
            3) Resume target and wait for brekpoints to hit.
            4) Check that target has stopped in the right place.
            5) Repeat steps 3-4 several times.
        """
        # 2 HW breaks + 1 flash SW break + RAM SW break
        self.bps = ['app_main', 'test_timer_isr', 'test_timer_isr_func', 'test_timer_isr_ram_func']
        self.select_sub_test("blink")
        for f in self.bps:
            self.add_bp(f)

        # riscv gdb11 workaround
        run_bt = True if testee_info.arch == "riscv32" and testee_info.idf_ver >= IdfVersion.fromstr('5.0') else False
        for i in range(3):
            self.run_to_bp_and_check_basic(dbg.TARGET_STOP_REASON_BP, 'test_timer_isr', run_bt)
            self.run_to_bp_and_check_basic(dbg.TARGET_STOP_REASON_BP, 'test_timer_isr_func', run_bt)
            self.run_to_bp_and_check_basic(dbg.TARGET_STOP_REASON_BP, 'test_timer_isr_ram_func', run_bt)

class WatchpointTestsImpl:
    """ Watchpoints test cases which are common for dual and single core modes
    """

    wp_stop_reason = [dbg.TARGET_STOP_REASON_SIGTRAP, dbg.TARGET_STOP_REASON_WP]

    def test_wp_simple(self):
        """
            This simple test checks general watchpoints usage scenario.
            1) Select appropriate sub-test number on target.
            2) Set access watchpoint.
            3) Resume target and wait for watchpoint to hit.
            4) Check that target has stopped in the right place.
            5) Check backtrace at the stop point.
            6) Check that watched expression has correct value.
            7) Repeat steps 3-6 several times.
        """
        self.select_sub_test("blink")
        self.wps = {'s_count1': None}
        for e in self.wps:
            self.add_wp(e, 'rw')
        cnt = 0
        for i in range(3):
            # 'count' read
            self.run_to_bp_and_check(self.wp_stop_reason, 'blink_task', ['s_count10'])
            var_val = int(self.gdb.data_eval_expr('s_count1'))
            self.assertEqual(var_val, cnt)
            # 'count' read
            self.run_to_bp_and_check(self.wp_stop_reason, 'blink_task', ['s_count11'])
            var_val = int(self.gdb.data_eval_expr('s_count1'))
            self.assertEqual(var_val, cnt)
            # 'count' write
            self.run_to_bp_and_check(self.wp_stop_reason, 'blink_task', ['s_count11'])
            var_val = int(self.gdb.data_eval_expr('s_count1'))
            self.assertEqual(var_val, cnt+1)
            cnt += 1

    def test_wp_and_reconnect(self):
        """
            This test checks that watchpoints work after GDB re-connection.
            Also this test checks that OOCD handles such re-connections safely.
            1) Select appropriate sub-test number on target.
            2) Set several 'write' watchpoints.
            3) Resume target and wait for watchpoint to hit.
            4) Check that target has stopped in the right place.
            5) Check the backtrace at the stop point.
            6) Disconnect GDB from OOCD.
            7) Connect GDB to OOCD.
            8) Repeat steps 3-7 several times.
        """
        self.select_sub_test("blink")
        self.wps = {'s_count1': None, 's_count2': None}
        cnt = 0
        cnt2 = 100
        for e in self.wps:
            self.add_wp(e, 'w')
        for i in range(5):
            if (i % 2) == 0:
                self.run_to_bp_and_check(self.wp_stop_reason, 'blink_task', ['s_count11'])
                var_val = int(self.gdb.data_eval_expr('s_count1'))
                self.assertEqual(var_val, cnt+1)
                cnt += 1
            else:
                self.run_to_bp_and_check(self.wp_stop_reason, 'blink_task', ['s_count2'])
                var_val = int(self.gdb.data_eval_expr('s_count2'))
                self.assertEqual(var_val, cnt2-1)
                cnt2 -= 1
            self.gdb.disconnect()
            sleep(0.1) #sleep 100ms
            self.gdb.connect()


########################################################################
#              TESTS DEFINITION WITH SPECIAL TESTS                     #
########################################################################
def two_cores_concurrently_hit_bps(self):
    """
        This test checks that 2 cores can concurrently hit the same set of breakpoints.
        1) Select appropriate sub-test number on target.
        2) Set several breakpoints to cover all types of them (HW, SW).
        3) Resume target and wait for any brekpoint to hit on any core.
        4) Check that target has stopped in the right place.
        5) Check backtrace at the stop point.
        6) Repeat steps 3-5 several times.
        7) Check that all set breakpoints hit one time at least.
    """
    hit_cnt = [0] * len(self.bps)
    for f in self.bps:
        self.add_bp(f)
    for i in range(30):
        self.resume_exec()
        rsn = self.gdb.wait_target_state(dbg.TARGET_STATE_STOPPED, 10)
        self.assertEqual(rsn, dbg.TARGET_STOP_REASON_BP)
        cur_frame = self.gdb.get_current_frame()
        self.assertTrue(cur_frame['func'] in self.bps)
        f_idx = self.bps.index(cur_frame['func'])
        hit_cnt[f_idx] += 1
        frames = self.gdb.get_backtrace()
        self.assertTrue(len(frames) > 0)
        self.assertEqual(frames[0]['func'], cur_frame['func'])
        self.assertEqual(frames[0]['line'], cur_frame['line'])
    for cnt in hit_cnt[1:]:
        self.assertTrue(cnt > 0)

def two_cores_concurrently_hit_wps(self):
    """
        This test checks that 2 cores can concurrently hit the same set of watchpoints.
        1) Select appropriate sub-test number on target.
        2) Set several 'write' watchpoints.
        3) Resume target and wait for any watchpoint to hit on any core.
        4) Check that target has stopped in the right place.
        5) Check backtrace at the stop point.
        6) Repeat steps 3-5 several times.
    """
    self.wps = {'s_count1': None, 's_count2': None}
    for e in self.wps:
        self.add_wp(e, 'w')
    wp_stop_reason = [dbg.TARGET_STOP_REASON_SIGTRAP]
    if testee_info.arch == "xtensa" or testee_info.idf_ver > IdfVersion.fromstr('5.0'):
        wp_stop_reason.append(dbg.TARGET_STOP_REASON_WP)
    for i in range(10):
        self.run_to_bp_and_check(wp_stop_reason, 'blink_task', ['s_count11', 's_count2'])


def appcpu_early_hw_bps(self):
    """
        This test checks if breakpoints set on APP_CPU just after reset work well.
    """
    self.gdb.target_reset()
    rsn = self.gdb.wait_target_state(dbg.TARGET_STATE_STOPPED, 10)
    self.add_bp('call_start_cpu1', hw=True)
    self.resume_exec()
    self.gdb.wait_target_state(dbg.TARGET_STATE_STOPPED, 10)
    # We stopped when FreRTOS is not running yet, so GDB is connected to
    # one core only (most probably core 0) and shows only one thread representing that core.
    # Prepare to switch GDB to core 1.
    self.gdb.monitor_run("esp32 smp_gdb 1", 5)
    try:
        # Switch GDB to core 1.
        self.gdb.monitor_run("resume", 5)
        # Invalidate register cache to re-read them and get proper backtrace
        self.gdb.console_cmd_run("maint flush register-cache", 5)
        # In this scenario we can not check stop reason for core 1,
        # because GDB could be initially connected to core 0.
        # So just check the function name we stopped in on core 1.
        frame = self.gdb.read_current_frame()
        self.assertEqual(frame['func'], 'call_start_cpu1')
    finally:
        # restore default GDB SMP handling to avoid other tests failures
        self.gdb.monitor_run("esp32 smp_gdb -1", 5)

class DebuggerBreakpointTestsDual(DebuggerGenericTestAppTestsDual, BreakpointTestsImpl):
    """ Test cases for breakpoints in dual core mode
    """

    def setUp(self):
        DebuggerGenericTestAppTestsDual.setUp(self)
        BreakpointTestsImpl.setUp(self)

    def test_2cores_concurrently_hit_bps(self):
        two_cores_concurrently_hit_bps(self)

    # OCD-773
    @idf_ver_min_for_chip('5.1', ['esp32'])
    def test_appcpu_early_hw_bps(self):
        appcpu_early_hw_bps(self)

class DebuggerBreakpointTestsDualEncrypted(DebuggerGenericTestAppTestsDualEncrypted, BreakpointTestsImpl):
    """ Breakpoint test cases on encrypted flash in dual core mode
    """
    def setUp(self):
        DebuggerGenericTestAppTestsDual.setUp(self)
        BreakpointTestsImpl.setUp(self)

    def test_2cores_concurrently_hit_bps(self):
        two_cores_concurrently_hit_bps(self)

    # OCD-773
    @idf_ver_min_for_chip('5.1', ['esp32'])
    def test_appcpu_early_hw_bps(self):
        appcpu_early_hw_bps(self)

class DebuggerBreakpointTestsSingle(DebuggerGenericTestAppTestsSingle, BreakpointTestsImpl):
    """ Test cases for breakpoints in single core mode
    """

    def setUp(self):
        DebuggerGenericTestAppTestsSingle.setUp(self)
        BreakpointTestsImpl.setUp(self)

class DebuggerBreakpointTestsSingleEncrypted(DebuggerGenericTestAppTestsSingleEncrypted, BreakpointTestsImpl):
    """ Breakpoint test cases on encrypted flash in single core mode
    """
    def setUp(self):
        DebuggerGenericTestAppTestsSingle.setUp(self)
        BreakpointTestsImpl.setUp(self)

class DebuggerWatchpointTestsDual(DebuggerGenericTestAppTestsDual, WatchpointTestsImpl):
    """ Test cases for watchpoints in dual core mode
    """
    def test_2cores_concurrently_hit_wps(self):
        two_cores_concurrently_hit_wps(self)

class DebuggerWatchpointTestsDualEncrypted(DebuggerGenericTestAppTestsDualEncrypted, WatchpointTestsImpl):
    """ Watchpoint test cases on encrypted flash in dual core mode
    """
    def test_2cores_concurrently_hit_wps(self):
        two_cores_concurrently_hit_wps(self)

class DebuggerWatchpointTestsSingle(DebuggerGenericTestAppTestsSingle, WatchpointTestsImpl):
    """ Test cases for watchpoints in single core mode
    """
    pass

class DebuggerWatchpointTestsSingleEncrypted(DebuggerGenericTestAppTestsSingleEncrypted, WatchpointTestsImpl):
    """ Watchpoint test cases on encrypted flash in single core mode
    """
    pass
