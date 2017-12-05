# GDB + OpenOCD tests

import time
import os.path
import logging
import unittest
import importlib
import debug_backend as dbg

# TODO: fixed???
ESP32_BLD_FLASH_OFF = 0x1000
ESP32_PT_FLASH_OFF = 0x8000
# TODO: get from partition table
ESP32_APP_FLASH_OFF = 0x10000

_test_apps_dir = ''


def get_logger():
    """ Returns logger for this module
    """
    return logging.getLogger('debug_backend_tests')


def set_apps_dir(path):
    """ Set test applications base directory
    """
    global _test_apps_dir
    _test_apps_dir = path


class DebuggerTestError(RuntimeError):
    """ Base class for debugger's test errors
    """
    pass


class DebuggerTestAppConfig:
    """ Test application config.
    """

    def __init__(self, bin_dir='', app_name='', app_off=ESP32_APP_FLASH_OFF):
        self.bin_dir = bin_dir
        self.app_name = app_name
        self.app_off = app_off
        self.bld_path = None
        self.bld_off = ESP32_BLD_FLASH_OFF
        self.pt_path = None
        self.pt_off = ESP32_PT_FLASH_OFF
        # name of test app variable which selects sub-test to run
        self.test_select_var = None
    
    def __repr__(self):
        return '%s/%s/%x-%s/%x-%s/%x' % (self.app_name, self.bin_dir, self.app_off, self.bld_path, self.bld_off, self.pt_path, self.pt_off)


class DebuggerTestsBunch(unittest.BaseTestSuite):
    """ Custom suite which supports groupping tests by target app and
        loading necessary binaries before tst group run.
    """

    def __init__(self, tests=()):
        self.modules = {}
        self._groupped_suites = {}
        super(DebuggerTestsBunch, self).__init__(tests)

    def addTest(self, test):
        """ Adds test
        """
        if type(test) is DebuggerTestsBunch:
            for t in test:
                self.addTest(t)
            return
        get_logger().debug('Add test %s', test)
        super(DebuggerTestsBunch, self).addTest(test)
        if test.__module__  not in self.modules:
            get_logger().debug('Add test module %s', test.__module__)
            self.modules[test.__module__] = importlib.import_module(test.__module__)
            # get_logger().debug('Modules: %s', self.modules)

    def run(self, result, debug=False):
        """ Runs tests
        """
        self._group_tests(self)
        for app_cfg_id in self._groupped_suites:
            # check if suite have at least one test to run 
            skip_count = 0
            for test in self._groupped_suites[app_cfg_id][1]:
                if getattr(type(test), '__unittest_skip__', False):
                    skip_count += 1
            if skip_count == self._groupped_suites[app_cfg_id][1].countTestCases():
                get_logger().debug('Skip loading %s for %d tests', app_cfg_id, skip_count)
            else:
                get_logger().debug('Load %s for %d tests', app_cfg_id, self._groupped_suites[app_cfg_id][1].countTestCases())
                # load only if app bins are configured (used) for these tests
                if self._groupped_suites[app_cfg_id][0]:
                    self._load_app(self._groupped_suites[app_cfg_id][0])
            self._groupped_suites[app_cfg_id][1]._run_tests(result, debug)
        return result

    def _run_tests(self, result, debug=False):
        """ Runs groups of tests
        """
        for test in self:
            if result.shouldStop:
                break
            get_logger().debug('<<<<<<<<< START %s >>>>>>>', test.id())
            if not debug:
                test(result)
            else:
                test.debug()
            get_logger().debug('======= END %s =======', test.id())

    def _group_tests(self, tests):
        """ Groups tests by target app
        """
        for test in tests:
            if type(test) is not DebuggerTestsBunch:
                app_cfg = getattr(test, 'test_app_cfg', False)
                if app_cfg:
                    app_cfg_id = str(app_cfg)
                else:
                    app_cfg_id = '' # test does not use app
                if app_cfg_id not in self._groupped_suites:
                    # print 'Add new suite for (%s)' % (app_name)
                    self._groupped_suites[app_cfg_id] = [app_cfg, DebuggerTestsBunch()]
                # print 'Add test %s to (%s)' % (test, app_name)
                self._groupped_suites[app_cfg_id][1].addTest(test)
            else:
                # print 'Group suite %s' % (test)
                self._group_tests(test)

    def _load_app(self, app_cfg):
        """ Loads application binaries to target.
        """
        bins_dir = os.path.join(_test_apps_dir, app_cfg.app_name, app_cfg.bin_dir)
        gdb = dbg.get_gdb()
        state,rsn = gdb.get_target_state()
        # print 'DebuggerTestAppTests.LOAD_APP %s / %s' % (cls, app_bins)
        if state != dbg.Gdb.TARGET_STATE_STOPPED:
            gdb.exec_interrupt()
            gdb.wait_target_state(dbg.Gdb.TARGET_STATE_STOPPED, 5)
        # write bootloader
        gdb.target_program(os.path.join(bins_dir, app_cfg.bld_path), app_cfg.bld_off)
        # write partition table
        gdb.target_program(os.path.join(bins_dir, app_cfg.pt_path), app_cfg.pt_off)
        # write application
        gdb.exec_file_set(os.path.join(bins_dir, '%s.elf' % app_cfg.app_name))
        # Currently we can not use GDB ELF loading facility for ESP32, so write binary image instead
        # _gdb.target_download()
        gdb.target_program(os.path.join(bins_dir, '%s.bin' % app_cfg.app_name), app_cfg.app_off)
        gdb.target_reset()


class DebuggerTestsBase(unittest.TestCase):
    """ Base class for all tests
    """
    def __init__(self, methodName):
        super(DebuggerTestsBase, self).__init__(methodName)
        self.gdb = dbg.get_gdb()

    def stop_exec(self):
        """ Stops target execution and ensures that it is in STOPPED state
        """
        state,rsn = self.gdb.get_target_state()
        if state != dbg.Gdb.TARGET_STATE_STOPPED:
            self.gdb.exec_interrupt()
            self.gdb.wait_target_state(dbg.Gdb.TARGET_STATE_STOPPED, 5)

    def resume_exec(self):
        """ Resumes target execution and ensures that it is in RUNNING state
        """
        state,rsn = self.gdb.get_target_state()
        if state != dbg.Gdb.TARGET_STATE_RUNNING:
            self.gdb.exec_continue()
            self.gdb.wait_target_state(dbg.Gdb.TARGET_STATE_RUNNING, 5)


class DebuggerTestAppTests(DebuggerTestsBase):
    """ Base class for tests which need special app running on target
    """

    def __init__(self, methodName):
        super(DebuggerTestAppTests, self).__init__(methodName)
        self.test_app_cfg = DebuggerTestAppConfig()

    def setUp(self):
        """ Setup test.
            In order to select sub-test all tests of this class need target to be reset and halted.
        """
        self.stop_exec()
        self.gdb.target_reset()
        rsn = self.gdb.wait_target_state(dbg.Gdb.TARGET_STATE_STOPPED, 5)
        self.main_bp = self.gdb.add_bp('app_main')
        self.resume_exec()
        rsn = self.gdb.wait_target_state(dbg.Gdb.TARGET_STATE_STOPPED, 5)
        # workaround for strange debugger's behaviour
        if rsn == dbg.Gdb.TARGET_STOP_REASON_SIGTRAP:
            get_logger().warning('Unexpected SIGTRAP! Apply workaround...')
            cur_frame = self.gdb.get_current_frame()
            self.assertEqual(cur_frame['addr'], '0x40000450')
            self.resume_exec()
            rsn = self.gdb.wait_target_state(dbg.Gdb.TARGET_STATE_STOPPED, 5)
        self.assertEqual(rsn, dbg.Gdb.TARGET_STOP_REASON_BP)
        frame = self.gdb.get_current_frame()
        self.assertEqual(frame['func'], 'app_main')
        self.gdb.delete_bp(self.main_bp)
        # ready to select and start test (should be done in test method)

    def select_sub_test(self, sub_test_num):
        """ Selects sub test in app running on target
        """
        self.gdb.data_eval_expr('%s=%d' % (self.test_app_cfg.test_select_var, sub_test_num))


class DebuggerGenericTestAppTests(DebuggerTestAppTests):
    """ Base class to run tests which use generic test app
    """

    def __init__(self, methodName):
        super(DebuggerGenericTestAppTests, self).__init__(methodName)
        self.test_app_cfg.app_name = 'gen_ut_app'
        self.test_app_cfg.bld_path = os.path.join('bootloader', 'bootloader.bin')
        self.test_app_cfg.pt_path = 'partitions_singleapp.bin'
        self.test_app_cfg.test_select_var = 'run_test'


class DebuggerGenericTestAppTestsDual(DebuggerGenericTestAppTests):
    """ Base class to run tests which use generic test app in dual core mode
    """

    def __init__(self, methodName='runTest'):
        super(DebuggerGenericTestAppTestsDual, self).__init__(methodName)
        # use default config with modified path to binaries
        self.test_app_cfg.bin_dir = os.path.join('output', 'default')


class DebuggerGenericTestAppTestsSingle(DebuggerGenericTestAppTests):
    """ Base class to run tests which use generic test app in single core mode
    """

    def __init__(self, methodName='runTest'):
        super(DebuggerGenericTestAppTestsSingle, self).__init__(methodName)
        # use default config with modified path to binaries
        self.test_app_cfg.bin_dir = os.path.join('output', 'single_core')