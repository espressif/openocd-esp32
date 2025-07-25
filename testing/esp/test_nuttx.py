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
        NAME_LEN = 15
        tasks_level = [3, 4, 5]
        self.add_bp('threadsbt', hw=True)
        self.run_to_bp(dbg.TARGET_STOP_REASON_BP, 'threadsbt', tmo=120)
        _,threads_info = self.gdb.get_thread_info()
        ocd_threads = [ti for ti in threads_info if ti['details'].startswith("Name: openocd_thread")]
        self.assertEqual(len(ocd_threads), 3)
        for ti in ocd_threads:
            name = ti['details'][NAME_START:NAME_START+NAME_LEN]
            expectedlvl = tasks_level[int(name[-1:])]
            self.gdb.set_thread(int(ti['id']))
            frames = self.gdb.get_backtrace()
            extractedlvl = 0

            for f in frames:
                if f['func'] == "thread_frames":
                    extractedlvl += 1
            self.assertEqual(expectedlvl, extractedlvl)

########################################################################
#              TESTS DEFINITION WITH SPECIAL TESTS                     #
########################################################################

class NuttxAppTests(DebuggerTestAppTests):
    """ Base class to run NuttX's tests
    """

    def __init__(self, methodName='runTest'):
        super(NuttxAppTests, self).__init__(methodName)
        # NuttX's blobs are copied from a different pipeline and are as follows:
        # test_app_dir = nuttx_test
        # app_name = nuttx_openocd
        # bin_dir = ''
        #   ==> bin_dir = nuttx_test/nuttx_openocd/
        self.test_app_cfg.app_name = 'nuttx_openocd'
        self.test_app_cfg.entry_point = 'openocd_main'
        if testee_info.chip == 'esp32' or testee_info.chip == 'esp32s2':
            self.test_app_cfg.app_off = 0x1000
        else:
            self.test_app_cfg.app_off = 0x0
        self.test_app_cfg.merged_bin = True

@run_with_version('other')
class NuttxAppTestsSingle(NuttxAppTests, DebuggerNuttxTestsImpl):
    """ Test cases in single core mode
    """

    # no special tests for single core mode yet
    pass

