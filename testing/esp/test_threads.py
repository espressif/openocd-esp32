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
    """ Test cases which are common for dual and single core modes
            This test switch a threads and check that expected thread id and current thread id are the same. 
            1) Read thread information about all threads to get full amount of threads
            2) Set up each thread as an active
            3) Compare that active thread id and expected thread id are the same
            4) Repeat test several times
    """

    def test_thread_switch(self):
        self.gdb.wait_target_state(dbg.Gdb.TARGET_STATE_STOPPED, 5)
        self.select_sub_test(401)
        self.add_bp('test_check_bp')
        
        for i in range(10):
            self.resume_exec()
            self.gdb.wait_target_state(dbg.Gdb.TARGET_STATE_STOPPED, 5)
            threads_info = self.gdb.get_thread_info() # get info for all threads
            s = len(threads_info)
            get_logger().debug('Loop = %d', i)
            for k in range(0,s):
                #print 'DebuggerThreadsTestsImpl.test_thread_switch loop [%i,%i] ' % (i,k)
                threads_info = self.gdb.get_thread_info() # get info for all threads
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
