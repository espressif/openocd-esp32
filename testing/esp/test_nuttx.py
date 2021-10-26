import logging
import unittest
import debug_backend as dbg
from debug_backend_tests import *

def get_logger():
    return logging.getLogger(__name__)


########################################################################
#                         TESTS IMPLEMENTATION                         #
########################################################################

class DebuggerNuttxTestsImpl:
    def test_threads_backtraces(self):
        """
            This test verifies the NuttX support. It switches between threads
            and checks that their backtraces are as expected:
        """

        NAME_START = 6
        NAME_LEN = 13
        tasks_level = [3, 4, 5]
        self.add_bp('tasksbt')
        self.run_to_bp(dbg.TARGET_STOP_REASON_BP, 'tasksbt', tmo=120)
        _,threads_info = self.gdb.get_thread_info()
        i = 1
        for ti in threads_info:
            if ti['details'].startswith("Name: openocd_task"):
                name = ti['details'][NAME_START:NAME_START+NAME_LEN]
                expectedlvl = tasks_level[int(name[-1:]) - 1]
                self.gdb.set_thread(int(ti['id']))
                frames = self.gdb.get_backtrace()

                # The frame should be like the following:
                #   nxtask_start
                #   nxtask_startup
                #   taskX_daemon
                #   taskX_lvl
                #    ...
                #   taskx_lvl
                #   Rest of the frame
                # We extract here from the end (the daemon) up to the last
                # taskX_lvl.  This should be the expected level + 1 for the
                # daemon.
                lvlsframe = frames[-(expectedlvl + 3):]
                extractedlvl = 0
                lvlname = 'task%s_lvls' % i
                for f in lvlsframe:
                    if f['func'] == lvlname:
                        extractedlvl += 1
                self.assertEqual(expectedlvl, extractedlvl)
                i += 1

########################################################################
#              TESTS DEFINITION WITH SPECIAL TESTS                     #
########################################################################

class NuttxAppTests(DebuggerTestAppTests):
    """ Base class to run NuttX's tests
    """

    def __init__(self, methodName='runTest'):
        super(NuttxAppTests, self).__init__(methodName);
        # NuttX's blobs are copied from a different pipeline and are as follows:
        # test_app_dir = nuttx_test
        # app_name = nuttx_openocd
        # bin_dir = ''
        #   ==> bin_dir = nuttx_test/nuttx_openocd/
        # bootloader and partition-table are supposed to be in the same
        # location.
        self.test_app_cfg.app_name = 'nuttx_openocd'
        self.test_app_cfg.entry_point = 'nx_start'
        self.test_app_cfg.bld_path = 'bootloader.bin'
        self.test_app_cfg.pt_path = 'partition-table.bin'
        self.test_app_cfg.startup_script = 'gdbstartup'

@run_with_version('other')
class NuttxAppTestsSingle(NuttxAppTests, DebuggerNuttxTestsImpl):
    """ Test cases in single core mode
    """

    # no special tests for single core mode yet
    pass

