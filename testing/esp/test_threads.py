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
            3) Waits for tssks to go to their pre-defined place in the source code
            4) Stops app execution
            5) Switches between tasks and checks their backtraces
        """
        self.select_sub_test(400)
        self.resume_exec()
        time.sleep(5)
        self.stop_exec()
        test_tasks = {'1': [3, 0], '2': [7, 0], '3': [5, 0]}
        _,threads_info = self.gdb.get_thread_info()
        for ti in threads_info:
            def _check_backtrace(suf, num):
                if not ti['details'].startswith("Name: check_bt_task%s" % suf):
                    return 0
                self.gdb.set_thread(int(ti['id'],0))
                frames = self.gdb.get_backtrace()
                if IdfVersion.get_current() == IdfVersion.fromstr('latest'):
                    self.assertEqual(len(frames), num+2) # task entry + vPortTaskWrapper
                else:
                    self.assertEqual(len(frames), num+1) # task entry
                line_num = self.gdb.data_eval_expr('go_to_level_task%s_break_ln' % suf)
                self.assertEqual(frames[0]['line'], line_num)
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
        self.select_sub_test(401)
        self.add_bp('test_check_bp')

        for i in range(10):
            self.resume_exec()
            self.gdb.wait_target_state(dbg.Gdb.TARGET_STATE_STOPPED, 5)
            _,threads_info = self.gdb.get_thread_info() # get info for all threads
            s = len(threads_info)
            get_logger().debug('Loop = %d', i)
            for k in range(0,s):
                #print 'DebuggerThreadsTestsImpl.test_thread_switch loop [%i,%i] ' % (i,k)
                _,threads_info = self.gdb.get_thread_info() # get info for all threads
                get_logger().debug('Process thread  %d, k=%d', int(threads_info[k]['id'],10), k)
                if threads_info[k]['details'].find("thread_task") == 0:
                    # Get expected ID
                    expected_id = int(threads_info[k]['id'],10);
                    self.gdb.set_thread(int(threads_info[k]['id'],10))
                    # Get Current ID
                    thread_ids = self.gdb.get_thread_ids()
                    current_id = int(thread_ids['current-thread-id'],10);
                    self.assertTrue(current_id == expected_id)
        get_logger().debug('test 2: DebuggerThreadsTestsImpl done')



########################################################################
#              TESTS DEFINITION WITH SPECIAL TESTS                     #
########################################################################

class DebuggerThreadsTestsDual(DebuggerGenericTestAppTestsDual, DebuggerThreadsTestsImpl):
    """ Test cases in dual core mode
    """
    # no special tests for dual core mode yet
    pass

class DebuggerThreadsTestsSingle(DebuggerGenericTestAppTestsSingle, DebuggerThreadsTestsImpl):
    """ Test cases in single core mode
    """
    # no special tests for single core mode yet
    pass
