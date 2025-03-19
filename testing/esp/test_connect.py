import logging
import unittest
import debug_backend as dbg
from debug_backend_tests import *
from time import sleep

def get_logger():
    return logging.getLogger(__name__)


########################################################################
#                         TESTS IMPLEMENTATION                         #
########################################################################

class GDBConnectTestsImpl:
    """ Test cases which are common for dual and single core modes
    """

    @run_all_cores
    def test_connect_reset_load_symbols(self):
        """
            Check that we can connect and do "monitor reset" when
            symbol table is not loaded yet.
        """
        # When the test starts, symbol table is already loaded and
        # gdb is connected. Undo that.
        self.gdb.disconnect()
        sleep(0.1) #sleep 100ms
        self.gdb.exec_file_set('')
        # Target is now running, try connecting and doing a reset
        self.gdb.connect()
        self.gdb.target_reset('halt')
        state, _ = self.gdb.get_target_state()
        self.assertEqual(state, dbg.TARGET_STATE_STOPPED)
        self.gdb.exec_file_set(self.test_app_cfg.build_app_elf_path())
        self.resume_exec()

    @run_all_cores
    def test_gdb_detach(self):
        """
            Check that gdb detach command removes flash breakpoints
            1) Add flash SW breakpoints in the appropriate sub-test.
            2) Run to SW breakpoints and expect to hit.
            3) Drop gdb connection with pkill command.
            4) Create a new gdb sesion and connect to OpenOCD.
            5) Reset the target, run to the app_main and set the sub-test again.
            6) Add a breakpoints into latter line of SW breakpoints tagged as gdb_detach3.
            7) Run the test and check that will chip hit to the SW breakpoints. If it will, detach command is not working properly.
        """
        # Filling HW breakpoints slots to make test using SW flash breakpoints
        self.fill_hw_bps()
        # flash SW breakpoints
        bps = ['gdb_detach0', 'gdb_detach1', 'gdb_detach2']

        for each_bp in bps:
            self.add_bp(each_bp)

        # Check if all flash breakpoints hit before dropping GDB connection
        self.run_to_bp_label("gdb_detach0")
        self.run_to_bp_label("gdb_detach1")
        self.run_to_bp_label("gdb_detach2")

        self.gdb_kill()
        sleep(1)
        self.create_gdb_and_reconnect()
        state,_ = self.gdb.get_target_state()
        if state != dbg.TARGET_STATE_STOPPED:
            self.gdb.exec_interrupt()
            self.gdb.wait_target_state(dbg.TARGET_STATE_STOPPED, 5)

        self.gdb.target_reset()
        self.add_bp('app_main')
        self.run_to_bp(dbg.TARGET_STOP_REASON_BP, 'app_main')
        self.select_sub_test(self.id())

        # Add a breakpoint to the last line of the test function.
        self.add_bp('gdb_detach3')
        # Run to hit breakpoint tagged as gdb_detach3 without hitting gdb_detach0 and gdb_detach1 or gdb_detach2 tags
        self.run_to_bp_label("gdb_detach3")


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
