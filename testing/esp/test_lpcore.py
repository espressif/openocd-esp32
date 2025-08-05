import logging
import unittest
import os
import subprocess
import re
import time
import debug_backend as dbg
from debug_backend_tests import *


def get_logger():
    """ Returns logger for this module
    """
    return logging.getLogger(__name__)


########################################################################
#                         TESTS IMPLEMENTATION                         #
########################################################################

class LpCoreTestsImpl:
    """ LP core test cases generic for dual and single core modes
    """

    def setUp(self):
        lp_elf_path = os.path.join(
            test_apps_dir, self.test_app_cfg.app_name,  self.test_app_cfg.build_dir, 'lp_core_main.elf')
        self.gdb.exec_file_set(lp_elf_path)
        # Now we can debug LP core
        self.add_bp('main', hw=True, tmp=True)
        self.run_to_bp_and_check_basic(dbg.TARGET_STOP_REASON_BP, 'main')

    def test_debug_crash(self):
        """
            This test checks that debugger can operate correctly after SW reset with stalled CPU.
            1) Select appropriate sub-test number on target.
            2) Resume target, wait for the program to crash (in dual core mode one CPU will be stalled).
            3) Re-start debugging (SW reset, set break to app_main(), resume and wait for the stop).
        """
        self.add_bp('do_crash')
        self.run_to_bp_and_check_basic(dbg.TARGET_STOP_REASON_BP, 'do_crash')
        self.clear_bps()

        # Call to abort() is optimised to 'ebreak' instruction
        # Force an exception by disabling SW breakpoints in OpenOCD
        lp_target = [t for t in self.oocd.targets() if t.endswith('lp.cpu')][0]
        self.oocd.cmd_exec(f"{lp_target} riscv set_ebreakm off")

        self.add_bp('_panic_handler', hw=True)
        self.run_to_bp_and_check_basic(dbg.TARGET_STOP_REASON_BP, '_panic_handler')
        self.clear_bps()

        self.oocd.cmd_exec(f"{lp_target} riscv set_ebreakm on")

        # Let the panic handler finish
        self.resume_exec()
        time.sleep(1)
        self.stop_exec()

        # Debugging HP core again
        self.gdb.target_reset()
        self.gdb.exec_file_set(self.test_app_cfg.build_app_elf_path())
        self.add_bp('app_main')
        self.run_to_bp_and_check_basic(dbg.TARGET_STOP_REASON_BP, 'app_main')



########################################################################
#              TESTS DEFINITION WITH SPECIAL TESTS                     #
########################################################################


class LpCoreTestAppTestsDual(DebuggerGenericTestAppTests):
    """ Base class to run tests which use LP core test app in dual core mode
    """

    def __init__(self, methodName='runTest'):
        super(LpCoreTestAppTestsDual, self).__init__(methodName)
        self.test_app_cfg.bin_dir = os.path.join('output', 'lpcore_dual')
        self.test_app_cfg.build_dir = os.path.join('builds', 'lpcore_dual')


class LpCoreTestAppTestsSingle(DebuggerGenericTestAppTests):
    """ Base class to run tests which use LP core test app in single core mode
    """

    def __init__(self, methodName='runTest'):
        super(LpCoreTestAppTestsSingle, self).__init__(methodName)
        self.test_app_cfg.bin_dir = os.path.join('output', 'lpcore_single')
        self.test_app_cfg.build_dir = os.path.join('builds', 'lpcore_single')


@idf_ver_min('5.4')
@only_for_chip(['esp32p4'])
class LpCoreTestsDual(LpCoreTestAppTestsDual, LpCoreTestsImpl):
    """ Test cases via GDB in dual core mode
    """
    def setUp(self):
        LpCoreTestAppTestsDual.setUp(self)
        LpCoreTestsImpl.setUp(self)

@idf_ver_min('5.4')
@only_for_chip(['esp32c6', 'esp32c5', 'esp32p4'])
class LpCoreTestsSingle(LpCoreTestAppTestsSingle, LpCoreTestsImpl):
    """ Test cases via GDB in single core mode
    """
    def setUp(self):
        LpCoreTestAppTestsSingle.setUp(self)
        LpCoreTestsImpl.setUp(self)
