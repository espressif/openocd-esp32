import logging
import unittest
import debug_backend as dbg
from debug_backend_tests import *


def get_logger():
    return logging.getLogger(__name__)


########################################################################
#                         TESTS IMPLEMENTATION                         #
########################################################################

class DebuggerThreadsTestsImpl:

    def test_threads_backtraces(self):
        """
            This test switches between threads and checks that their backtraces are as expected:
            1) Selects test number on target
            2) Resumes app execution
            3) Waits for tasks to go to their pre-defined place in the source code
            4) Stops app execution
            5) Switches between tasks and checks their backtraces
        """
        self.add_bp('check_backtrace_test_done')
        self.run_to_bp(dbg.TARGET_STOP_REASON_BP, 'check_backtrace_test_done', tmo=120)
        test_tasks = {'1': [3, 0], '2': [7, 0], '3': [5, 0]}
        _,threads_info = self.gdb.get_thread_info()
        for ti in threads_info:
            def _check_backtrace(suf, num):
                if not ti['details'].startswith("Name: check_bt_task%s" % suf):
                    return 0
                self.gdb.set_thread(int(ti['id'],0))
                frames = self.gdb.get_backtrace()
                self.assertEqual(len(frames), num+2) # task entry + vPortTaskWrapper
                line_num = int(self.gdb.data_eval_expr('go_to_level_task%s_break_ln' % suf), 0)
                # should be at the loop start or loop body
                self.assertTrue((frames[0]['line'] == '%d' % (line_num+1)) or (frames[0]['line'] == '%d' % (line_num+2)))
                for i in range(num):
                    self.assertEqual(frames[i]['func'], 'go_to_level_task%s' % suf)
                self.assertEqual(frames[num]['func'], 'check_backtrace_task%s' % suf)
                return 1
            for suf in test_tasks:
                test_tasks[suf][1] += _check_backtrace(suf, test_tasks[suf][0])
        for suf in test_tasks:
            self.assertEqual(test_tasks[suf][1], 1)

    def test_thread_switch(self):
        """
            This test switch a threads and check that expected thread id and current thread id are the same.
            1) Read thread information about all threads to get full amount of threads
            2) Set up each thread as an active
            3) Compare that active thread id and expected thread id are the same
            4) Repeat test several times
        """
        self.add_bp('test_check_bp')

        for i in range(10):
            self.resume_exec()
            self.gdb.wait_target_state(dbg.TARGET_STATE_STOPPED, 5)
            _,threads_info = self.gdb.get_thread_info() # get info for all threads
            s = len(threads_info)
            get_logger().debug('Loop = %d', i)
            for k in range(0,s):
                #print 'DebuggerThreadsTestsImpl.test_thread_switch loop [%i,%i] ' % (i,k)
                _,threads_info = self.gdb.get_thread_info() # get info for all threads
                get_logger().debug('Process thread  %d, k=%d', int(threads_info[k]['id'],10), k)
                if threads_info[k]['details'].find("thread_task") >= 0:
                    # Get expected ID
                    expected_id = int(threads_info[k]['id'],10);
                    self.gdb.set_thread(int(threads_info[k]['id'],10))
                    # Get Current ID
                    thread_ids = self.gdb.get_thread_ids()
                    current_id = int(thread_ids['current-thread-id'],10);
                    self.assertTrue(current_id == expected_id)
        get_logger().debug('test 2: DebuggerThreadsTestsImpl done')

    @only_for_arch(['riscv'])
    def test_thread_registers(self):
        """
            This test switches between threads and checks that their registers are as expected:
            1) Selects the test case by setting a breakpoint at 'check_backtrace_test_done'
            2) Resumes app execution until the breakpoint is hit
            3) Waits for tasks to reach their predefined positions in the source code
            4) Stops application execution at the breakpoint
            5) Switches between passive tasks and reads their register values from the stack using the "g" packet
            6) Compares g packet response with the corresponding values read individually using the "p" packet
        """
        self.add_bp('check_backtrace_test_done')
        self.run_to_bp(dbg.TARGET_STOP_REASON_BP, 'check_backtrace_test_done', tmo=120)
        _,threads_info = self.gdb.get_thread_info()
        thread_ids = self.gdb.get_thread_ids()
        current_thread_id = int(thread_ids['current-thread-id'], 10)

        console_output = ''
        def _console_stream_handler(type, stream, payload):
            nonlocal console_output
            console_output += payload

        def parse_gp_packet(console_output):
            match = re.search(r'received: \\"([0-9a-fA-F]+)\\"', console_output)
            if not match:
                raise ValueError("'received:' not found in gdb response")
            all_regs = match.group(1)
            reg_vals = [f"0x{all_regs[i+6:i+8]}{all_regs[i+4:i+6]}{all_regs[i+2:i+4]}{all_regs[i:i+2]}" for i in range(0, len(all_regs), 8)]
            return reg_vals

        try:
            self.gdb.stream_handler_add('console', _console_stream_handler)
            for thread in threads_info:
                thread_id = int(thread['id'], 10)
                if thread_id == current_thread_id:
                    continue
                self.gdb.set_thread(thread_id)
                console_output = ''
                self.gdb.console_cmd_run("maint packet g", 5)
                all_regs = parse_gp_packet(console_output)
                for reg_num in range(0, len(all_regs)):
                    console_output = ''
                    self.gdb.console_cmd_run(f"maint packet p {reg_num:x}", 5)
                    reg = parse_gp_packet(console_output)
                    self.assertEqual(all_regs[reg_num], reg[0])
        finally:
            self.gdb.stream_handler_remove('console', _console_stream_handler)

########################################################################
#              TESTS DEFINITION WITH SPECIAL TESTS                     #
########################################################################

class DebuggerThreadsTestsDual(DebuggerGenericTestAppTestsDual, DebuggerThreadsTestsImpl):
    """ Test cases in dual core mode
    """
    # no special tests for single core mode yet
    pass

class DebuggerThreadsTestsSingle(DebuggerGenericTestAppTestsSingle, DebuggerThreadsTestsImpl):
    """ Test cases in single core mode
    """
    # no special tests for single core mode yet
    pass

@idf_ver_min('5.3')
class DebuggerThreadsTestsAmazonFreeRTOSDual(DebuggerGenericTestAppTestsDual, DebuggerThreadsTestsImpl):

    def __init__(self, methodName='runTest'):
        super(DebuggerGenericTestAppTestsDual, self).__init__(methodName)
        self.test_app_cfg.bin_dir = os.path.join('output', 'default_amazon_freertos')
        self.test_app_cfg.build_dir = os.path.join('builds', 'default_amazon_freertos')

@idf_ver_min('5.3')
class DebuggerThreadsTestsAmazonFreeRTOSSingle(DebuggerGenericTestAppTestsSingle, DebuggerThreadsTestsImpl):

    def __init__(self, methodName='runTest'):
        super(DebuggerGenericTestAppTestsSingle, self).__init__(methodName)
        self.test_app_cfg.bin_dir = os.path.join('output', 'single_core_amazon_freertos')
        self.test_app_cfg.build_dir = os.path.join('builds', 'single_core_amazon_freertos')


@idf_ver_min('latest')
class DebuggerThreadsTestsFreeRTOSListIntegrityDual(DebuggerGenericTestAppTestsDual, DebuggerThreadsTestsImpl):

    def __init__(self, methodName='runTest'):
        super(DebuggerGenericTestAppTestsDual, self).__init__(methodName)
        self.test_app_cfg.bin_dir = os.path.join('output', 'default_freertos_list_integrity')
        self.test_app_cfg.build_dir = os.path.join('builds', 'default_freertos_list_integrity')

@idf_ver_min('latest')
class DebuggerThreadsTestsFreeRTOSListIntegritySingle(DebuggerGenericTestAppTestsSingle, DebuggerThreadsTestsImpl):

    def __init__(self, methodName='runTest'):
        super(DebuggerGenericTestAppTestsSingle, self).__init__(methodName)
        self.test_app_cfg.bin_dir = os.path.join('output', 'single_core_freertos_list_integrity')
        self.test_app_cfg.build_dir = os.path.join('builds', 'single_core_freertos_list_integrity')
