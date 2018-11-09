import logging
import unittest
import debug_backend as dbg
from debug_backend_tests import *


def get_logger():
    return logging.getLogger(__name__)


########################################################################
#                         TESTS IMPLEMENTATION                         #
########################################################################

class GDBConnectTestsImpl:
    """ Test cases which are common for dual and single core modes
    """

    def test_connect_reset_load_symbols(self):
        """ 
            Check that we can connect and do "monitor reset" when
            symbol table is not loaded yet.
        """
        # When the test starts, symbol table is already loaded and 
        # gdb is connected. Undo that.
        self.gdb.disconnect()
        self.gdb.exec_file_set('')
        # Target is now running, try connecting and doing a reset
        self.gdb.connect()
        self.gdb.target_reset('halt')
        state, _ = self.gdb.get_target_state()
        self.assertEqual(state, dbg.Gdb.TARGET_STATE_STOPPED)
        self.gdb.exec_file_set(self.test_app_cfg.build_app_elf_path())
        self.resume_exec()


########################################################################
#              TESTS DEFINITION WITH SPECIAL TESTS                     #
########################################################################

class GDBConnectTestsDual(DebuggerGenericTestAppTestsDual, GDBConnectTestsImpl):
    """ Test cases in dual core mode
    """
    pass

class GDBConnectTestsSingle(DebuggerGenericTestAppTestsSingle, GDBConnectTestsImpl):
    """ Test cases in single core mode
    """
    pass
