# GDB + OpenOCD tests

import time
import os
import logging
import unittest
import importlib
import sys
import re
import debug_backend as dbg

# TODO: fixed???
ESP_RISCV_BLD_FLASH_OFF = 0x0
ESP_XTENSA_BLD_FLASH_OFF = 0x1000
ESP32_PT_FLASH_OFF = 0x8000
# TODO: get from partition table
ESP32_APP_FLASH_OFF = 0x10000

test_apps_dir = ''

xtensa_chips = ["esp32", "esp32s2", "esp32s3"]

def get_logger():
    """ Returns logger for this module
    """
    return logging.getLogger(__name__)


class IdfVersion:
    """ Wrapper class for IDF version
        Keeps IDF ver as 4 bytes int. Format is: x.3.0.2, the most significant byte is not used.
    """
    IDF_VER_LATEST = 0xFFFFFFFF
    IDF_VER_OTHER  = 0xEFFFFFFF

    def __init__(self, ver_num=IDF_VER_LATEST):
        self._idf_ver = ver_num

    @staticmethod
    def fromstr(ver_str):
        if ver_str == 'latest':
            return IdfVersion()
        if ver_str == 'other':
            return IdfVersion(IdfVersion.IDF_VER_OTHER)

        vers = ver_str.split('.')
        # 3 -> 3.0.0
        # 3.1 -> 3.1.0
        while len(vers) < 3:
            vers.append('0')
        idf_ver = 0
        for i in range((len(vers)-1), -1, -1):
            idf_ver |= int(vers[len(vers)-i-1], 0) << i*8
        return IdfVersion(idf_ver)

    def __repr__(self):
        if self._idf_ver == self.IDF_VER_LATEST:
            return 'latest'
        return "%d.%d.%d" % ((self._idf_ver >> 16) & 0xFF, (self._idf_ver >> 8) & 0xFF, self._idf_ver & 0xFF)

    def __cmp__(self, other):
        res = 0
        if self._idf_ver < other._idf_ver:
            res = -1
        elif self._idf_ver > other._idf_ver:
            res = 1
        return res

    def __lt__(self, other):
        return self._idf_ver < other._idf_ver

    def __ge__(self, other):
        return self._idf_ver >= other._idf_ver

    def __eq__(self, other):
        return self._idf_ver == other._idf_ver

class TesteeInfo:
    """ Wrapper class for any info related to the current target.
        It may include target name, IDF version etc.
    """
    def __init__(self):
        self.hw_id = ''
        self.idf_ver = IdfVersion()
        self.__chip = ''
        self.__arch = ''

    @property
    def chip(self):
        return self.__chip

    @chip.setter
    def chip(self, val):
        self.__chip = val
        self.__arch = "xtensa" if val in xtensa_chips else "riscv32"

    @property
    def arch(self):
        return self.__arch

class GDBUtils:

    def get_pkill_arg(self):
        if "xtensa" in testee_info.arch:
            return "xtensa-esp*"
        elif "riscv" in testee_info.arch:
            return "riscv32-esp-elf"
        return "Unknown target"

    def gdb_kill(self):
        pkill_arg = self.get_pkill_arg()
        os.system('pkill %s' % pkill_arg)

    def create_gdb(self, chip_name, target_triple, toolchain, log_level, log_stream, log_file, gdb_log, debug_oocd):
        remote_tmo = 10
        _gdb_inst = dbg.create_gdb(chip_name=chip_name,
                            target_triple=target_triple,
                            gdb_path='%sgdb' % toolchain,
                            extended_remote_mode='127.0.0.1:%d' % dbg.Oocd.GDB_PORT,
                            log_level=log_level,
                            log_stream_handler=log_stream,
                            log_file_handler=log_file)
        if len(gdb_log):
            _gdb_inst.gdb_set('remotelogfile', gdb_log)
        if debug_oocd > 2:
            _gdb_inst.tmo_scale_factor = 5
        else:
            _gdb_inst.tmo_scale_factor = 3
        _gdb_inst.gdb_set('remotetimeout', '%d' % remote_tmo)

        return _gdb_inst

    def create_gdb_and_reconnect(self):
        debug_oocd = self.args[0]
        log_lev = self.args[1]
        gdb_log_file = self.args[2]
        ch = self.args[3]
        fh = self.args[4]
        connect_tmo = 15

        _gdb_inst = self.create_gdb(testee_info.chip, self.toolchain[:-1], self.toolchain, log_lev,
                                        ch, fh, gdb_log_file, debug_oocd)
        _gdb_inst.connect(tmo=connect_tmo)
        _gdb_inst.exec_file_set(self.test_app_cfg.build_app_elf_path())
        self.gdb = _gdb_inst

testee_info = TesteeInfo()


def idf_ver_min(ver_str):
    return unittest.skipIf(testee_info.idf_ver < IdfVersion.fromstr(ver_str), "requires min IDF_VER='%s', current IDF_VER='%s'" % (ver_str, testee_info.idf_ver))

def run_with_version(ver_str):
    return unittest.skipIf(testee_info.idf_ver != IdfVersion.fromstr(ver_str), "Not Applicable to this version")

def skip_for_hw_id(hw_ids_to_skip):
    skip = False
    hw_id_to_skip = ''
    for id in hw_ids_to_skip:
        if re.match(id, testee_info.hw_id):
            skip = True
            hw_id_to_skip = id
            break
    return unittest.skipIf(skip, "skipped due to HW ID '%s' matches to '%s'" % (testee_info.hw_id, hw_id_to_skip))

def skip_for_chip(chips_to_skip):
    skip = False
    for id in chips_to_skip:
        if id == testee_info.chip:
            skip = True
            break
    return unittest.skipIf(skip, "skipped for chip '%s'" % (testee_info.chip))

def skip_for_chip_and_ver(ver_str, chips_to_skip):
    skip = False
    for id in chips_to_skip:
        if id == testee_info.chip:
            v1 = repr(testee_info.idf_ver).split('.')
            v2 = ver_str.split('.')
            # check major and minor numbers only.
            if v1[0] == v2[0] and v1[1] == v2[1]:
                skip = True
    return unittest.skipIf(skip, "for the '%s' for the IDF_VER='%s'" % (id, testee_info.idf_ver))

def skip_for_arch(archs_to_skip):
    skip = False
    arch_to_skip = ''
    for id in archs_to_skip:
        if re.match(id, testee_info.arch):
            skip = True
            arch_to_skip = id
            break
    return unittest.skipIf(skip, "skipped due to arch '%s' matches to '%s'" % (testee_info.arch, arch_to_skip))

def only_for_arch(archs_to_run):
    skip = True
    for id in archs_to_run:
        if re.match(id, testee_info.arch):
            skip = False
            break
    return unittest.skipIf(skip, "skipped due to arch '%s' does not match to '%s'" % (testee_info.arch, archs_to_run))

def idf_ver_min_for_arch(ver_str, archs_to_run):
    skip = True
    for id in archs_to_run:
        if re.match(id, testee_info.arch):
            return idf_ver_min(ver_str)
    # do not skip if arch is not found
    return unittest.skipIf(False, "")

def idf_ver_min_for_chip(ver_str, chips_to_skip):
    for chip in chips_to_skip:
        if chip == testee_info.chip:
            return idf_ver_min(ver_str)
    # do not skip if chip is not found
    return unittest.skipIf(False, "")

class DebuggerTestError(RuntimeError):
    """ Base class for debugger's test errors
    """
    pass


class DebuggerTestAppConfig:
    """ Test application config.
        Application binaries (elf and bin) are implied to be located in $test_apps_dir/$app_name/$bin_dir
    """

    def __init__(self, bin_dir='', build_dir='', src_dir='', app_name='',
            app_off=ESP32_APP_FLASH_OFF, entry_point='app_main'):
        # Base path for app binaries, relative $test_apps_dir/$app_name
        self.bin_dir = bin_dir
        # Base path for app object files, relative $test_apps_dir/$app_name
        self.build_dir = build_dir
        # App name
        self.app_name = app_name
        # App binary offeset in flash
        self.app_off = app_off
        # Path for bootloader binary, relative $test_apps_dir/$app_name/$bin_dir
        self.bld_path = None
        # App binary offeset in flash
        if testee_info.arch == "xtensa":
            self.bld_off = ESP_XTENSA_BLD_FLASH_OFF
        else: #riscv32
            self.bld_off = ESP_RISCV_BLD_FLASH_OFF
        # Path for partitions table binary, relative $test_apps_dir/$app_name/$bin_dir
        self.pt_path = None
        # App binary offeset in flash
        self.pt_off = ESP32_PT_FLASH_OFF
        # name of test app variable which selects sub-test to run
        # used for number-based tests selection
        self.test_select_var = None
        # name of test app variable which holds sub-test string ID (name) to run
        # used for string-based tests selection
        self.test_id_var = None
        # Program's entry point ("app_main" is IDF's default)
        self.entry_point = entry_point
        # File containing commands to execute at startup
        self.startup_script = ''
        # Execute the script only.
        self.only_startup = True

    def __repr__(self):
        return '%s/%x-%s/%x-%s/%x-%s' % (self.bin_dir, self.app_off, self.app_name, self.bld_off, self.bld_path, self.pt_off, self.pt_path)

    def build_src_dir(self):
        return os.path.join(test_apps_dir, self.app_name)

    def build_obj_dir(self):
        return os.path.join(test_apps_dir, self.app_name, self.build_dir)

    def build_bins_dir(self):
        return os.path.join(test_apps_dir, self.app_name, self.bin_dir)

    def build_bld_bin_path(self):
        return os.path.join(self.build_bins_dir(), self.bld_path)

    def build_pt_bin_path(self):
        return os.path.join(self.build_bins_dir(), self.pt_path)

    def build_app_bin_path(self):
        return os.path.join(self.build_bins_dir(), '%s.bin' % self.app_name)

    def build_app_elf_path(self):
        return os.path.join(self.build_bins_dir(), '%s.elf' % self.app_name)

    def startup_script_path(self):
        return os.path.join(self.build_bins_dir(), '%s' % self.startup_script)

class DebuggerTestsBunch(unittest.BaseTestSuite):
    """ Custom suite which supports groupping tests by target app and
        loading necessary binaries before tst group run.
    """

    def __init__(self, tests=()):
        self.load_app_bins = True
        self.modules = {}
        self._groupped_suites = {}
        self.gdb = None
        self.oocd = None
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

    def config_tests(self, oocd, gdb, toolchain, uart_reader, port_name, arg_list):
        self.oocd = oocd
        self.gdb = gdb
        for test in self:
            if not issubclass(type(test), DebuggerTestsBase):
                continue
            test.oocd = oocd
            test.gdb = gdb
            test.toolchain = toolchain
            test.uart_reader = uart_reader
            test.port_name = port_name
            test.args = arg_list

    def change_gdb_in_tests(self, gdb):
        for each_test in self:
            each_test.gdb = gdb

    def run(self, result, debug=False):
        """ Runs tests
        """
        self._group_tests(self)
        for app_cfg_id in self._groupped_suites:
            # check if suite have at least one test to run
            skip_count = 0
            for test in self._groupped_suites[app_cfg_id][1]:
                # if the whole class of tests or particular test is marked to be skipped
                if getattr(type(test), '__unittest_skip__', False) or getattr(test, '__esp_unittest_skip_reason__', '') != '':
                    skip_count += 1
            if skip_count == self._groupped_suites[app_cfg_id][1].countTestCases():
                get_logger().debug('Skip loading %s for %d tests', app_cfg_id, skip_count)
            else:
                get_logger().debug('Load %s for %d tests', app_cfg_id, self._groupped_suites[app_cfg_id][1].countTestCases())
                # load only if app bins are configured (used) for these tests
                if self.load_app_bins and self._groupped_suites[app_cfg_id][0]:
                    try:
                        self._load_app(self._groupped_suites[app_cfg_id][0])
                    except:
                        get_logger().critical('Failed to load %s!', app_cfg_id)
                        for test in self._groupped_suites[app_cfg_id][1]:
                            result.addError(test, sys.exc_info())
                        continue
                self.gdb.exec_file_set(self._groupped_suites[app_cfg_id][0].build_app_elf_path())

                if self._groupped_suites[app_cfg_id][0].startup_script != '':
                    self.gdb.set_prog_startup_script(self._groupped_suites[app_cfg_id][0].startup_script_path())
                    self.gdb.exec_run(only_startup=self._groupped_suites[app_cfg_id][0].only_startup)
            ret = self._groupped_suites[app_cfg_id][1]._run_tests(result, debug)
            if ret is not None:
                self.gdb = ret
                for tmp_app_cfg_id in self._groupped_suites:
                    self._groupped_suites[tmp_app_cfg_id][0].gdb = ret
                    self._groupped_suites[tmp_app_cfg_id][1].gdb = ret
                    self._groupped_suites[tmp_app_cfg_id][1].change_gdb_in_tests(ret)
        return result

    def _run_tests(self, result, debug=False):
        """ Runs groups of tests
        """
        tmp_gdb = None
        for test in self:
            if result.shouldStop:
                break
            get_logger().debug('<<<<<<<<< START %s >>>>>>>', test.id())
            if not debug:
                test(result)
                if "test_gdb_detach" in test.id() and test.gdb != self.gdb:
                    tmp_gdb = test.gdb
                    self.gdb = tmp_gdb
                    self.change_gdb_in_tests(tmp_gdb)
            else:
                test.debug()
            get_logger().debug('======= END %s =======', test.id())
        return tmp_gdb

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
                    self._groupped_suites[app_cfg_id] = [app_cfg, DebuggerTestsBunch()]
                self._groupped_suites[app_cfg_id][1].addTest(test)
            else:
                self._group_tests(test)

    def _load_app(self, app_cfg):
        """ Loads application binaries to target.
        """
        state,_ = self.gdb.get_target_state()
        if state != dbg.TARGET_STATE_STOPPED:
            self.gdb.exec_interrupt()
            self.gdb.wait_target_state(dbg.TARGET_STATE_STOPPED, 5)
        # flash using 'flasher_args.json'
        self.gdb.target_program_bins(app_cfg.build_bins_dir())
        self.gdb.target_reset()


class DebuggerTestsBase(unittest.TestCase, GDBUtils):
    """ Base class for all tests
    """
    def __init__(self, methodName):
        super(DebuggerTestsBase, self).__init__(methodName)
        self.gdb = None
        self.oocd = None
        self.toolchain = ''
        self.__esp_unittest_skip_reason__ = ''

    def setUp(self):
        super(DebuggerTestsBase, self).setUp()
        if self.__esp_unittest_skip_reason__ != '':
            self.skipTest(self.__esp_unittest_skip_reason__)

    def fail_if_not_hw_id(self, hw_ids):
        for id in hw_ids:
            if re.match(id, testee_info.hw_id):
                return
        return self.fail("failed due to HW ID '%s' does not match to any of %s" % (testee_info.hw_id, hw_ids))

    def stop_exec(self):
        """ Stops target execution and ensures that it is in STOPPED state
        """
        state,_ = self.gdb.get_target_state()
        if state != dbg.TARGET_STATE_STOPPED:
            self.gdb.exec_interrupt()
            rsn = self.gdb.wait_target_state(dbg.TARGET_STATE_STOPPED, 10)
            self.assertTrue(rsn == dbg.TARGET_STOP_REASON_SIGINT or rsn == dbg.TARGET_STOP_REASON_SIGTRAP)

    def resume_exec(self, loc=None):
        """ Resumes target execution and ensures that it is in RUNNING state
        """
        state,rsn = self.gdb.get_target_state()
        if state != dbg.TARGET_STATE_RUNNING:
            if loc:
                get_logger().debug('Resume from addr 0x%x', pc)
                self.gdb.exec_jump(loc)
            else:
                self.gdb.exec_continue()
            self.gdb.wait_target_state(dbg.TARGET_STATE_RUNNING, 5)

    def interrupt(self):
        """ Perform CTRL+C
        """
        self.gdb.exec_interrupt()
        rsn = self.gdb.wait_target_state(dbg.TARGET_STATE_STOPPED, 20)
        self.assertEqual(rsn, dbg.TARGET_STOP_REASON_SIGINT)

    def step(self, insn=False, stop_rsn=dbg.TARGET_STOP_REASON_STEPPED):
        """ Performs program step ( "next", "nexti" command in GDB)
        """
        if insn:
            self.gdb.exec_next_insn()
        else:
            self.gdb.exec_next()
        self.gdb.wait_target_state(dbg.TARGET_STATE_RUNNING, 5)
        rsn = self.gdb.wait_target_state(dbg.TARGET_STATE_STOPPED, 5)
        self.assertEqual(rsn, stop_rsn)

    def step_in(self):
        """ Performs program step (step in, "step" command in GDB)
        """
        self.gdb.exec_step()
        self.gdb.wait_target_state(dbg.TARGET_STATE_RUNNING, 5)
        rsn = self.gdb.wait_target_state(dbg.TARGET_STATE_STOPPED, 5)
        self.assertEqual(rsn, dbg.TARGET_STOP_REASON_STEPPED)

    def step_out(self, tmo=None):
        """ Runs until current function retunrs (step out, "finish" command in GDB)
        """
        self.gdb.exec_finish()
        self.gdb.wait_target_state(dbg.TARGET_STATE_RUNNING, 5)
        rsn = self.gdb.wait_target_state(dbg.TARGET_STATE_STOPPED, 5 if tmo is None else tmo)
        self.assertEqual(rsn, dbg.TARGET_STOP_REASON_FN_FINISHED)



class DebuggerTestAppTests(DebuggerTestsBase):
    """ Base class for tests which need special app running on target
    """

    def __init__(self, methodName):
        super(DebuggerTestAppTests, self).__init__(methodName)
        self.test_app_cfg = DebuggerTestAppConfig()
        self.bpns = []
        self.wps = {}

    def setUp(self):
        """ Setup test.
            In order to select sub-test all tests of this class need target to be reset and halted.
        """
        super(DebuggerTestAppTests, self).setUp()
        self.stop_exec()
        self.prepare_app_for_debugging(self.test_app_cfg.app_off)
        # ready to select and start test (should be done in test method)
        self.select_sub_test(self.id())

    def tearDown(self):
        self.clear_bps()
        self.clear_wps()

    def prepare_app_for_debugging(self, app_flash_off):
        self.gdb.target_reset()
        rsn = self.gdb.wait_target_state(dbg.TARGET_STATE_STOPPED, 10)
        # update GDB memory map
        self.gdb.disconnect()
        # TODO: chip dependent
        self.oocd.set_appimage_offset(app_flash_off)
        self.gdb.connect()
        bp = self.gdb.add_bp(self.test_app_cfg.entry_point)
        self.resume_exec()
        rsn = self.gdb.wait_target_state(dbg.TARGET_STATE_STOPPED, 10)
        # workarounds for strange debugger's behaviour
        if rsn == dbg.TARGET_STOP_REASON_SIGINT:
            get_logger().warning('Unexpected SIGINT during setup! Apply workaround...')
            cur_frame = self.gdb.get_current_frame()
            self.resume_exec()
            rsn = self.gdb.wait_target_state(dbg.TARGET_STATE_STOPPED, 10)
        self.assertEqual(rsn, dbg.TARGET_STOP_REASON_BP)
        frame = self.gdb.get_current_frame()
        self.assertEqual(frame['func'], self.test_app_cfg.entry_point)
        self.gdb.delete_bp(bp)


    def add_bp(self, loc, ignore_count=0, cond='', hw=False, tmp=False):
        self.bpns.append(self.gdb.add_bp(loc, ignore_count=ignore_count, cond=cond, hw=hw, tmp=tmp))

    def add_wp(self, exp, tp='w'):
        self.wps[exp] = self.gdb.add_wp(exp, tp=tp)

    def clear_bps(self):
        for bpn in self.bpns:
            self.gdb.delete_bp(bpn)
        self.bpns = []

    def clear_wps(self):
        for _,wpn in self.wps.items():
            self.gdb.delete_bp(wpn)
        self.wps = {}

    def select_sub_test(self, sub_test_id):
        """ Selects sub test in app running on target
        """
        if type(sub_test_id) is str:
            self.gdb.data_eval_expr('%s=%d' % (self.test_app_cfg.test_select_var, -1))
            self.gdb.data_eval_expr('%s=\\"%s\\"' % (self.test_app_cfg.test_id_var, sub_test_id))
        else:
            self.gdb.data_eval_expr('%s=%d' % (self.test_app_cfg.test_select_var, sub_test_id))


    def run_to_bp(self, exp_rsn, func_name, tmo=20):
        self.resume_exec()
        rsn = self.gdb.wait_target_state(dbg.TARGET_STATE_STOPPED, tmo)
        try:
            if type(exp_rsn) is list:
                self.assertTrue(rsn in exp_rsn)
            else:
                self.assertEqual(rsn, exp_rsn)
            cur_frame = self.gdb.get_current_frame()
            self.assertEqual(cur_frame['func'], func_name)
        except AssertionError as e:
            get_logger().error('Invalid stop location! Backtrace:')
            frames = self.gdb.get_backtrace()
            for frame in frames:
                file_name = frame['file'] if 'file' in frame else ''
                get_logger().error('#%s: %s %s - %s:%s', frame['level'], frame['addr'], frame['func'], frame['file'], frame['line'])
            raise e
        return cur_frame

    def run_to_bp_and_check_basic(self, exp_rsn, func_name, run_bt=False):
        cur_frame = self.run_to_bp(exp_rsn, func_name)
        frames = self.gdb.get_backtrace(run_bt)
        self.assertTrue(len(frames) > 0)
        self.assertEqual(frames[0]['func'], cur_frame['func'])
        self.assertEqual(frames[0]['line'], cur_frame['line'])
        return frames

    def run_to_bp_and_check(self, exp_rsn, func_name, lineno_var_prefs, outmost_func_name='blink_task'):
        frames = self.run_to_bp_and_check_basic(exp_rsn, func_name)
        # -2 because our task function is called by FreeRTOS task wrapper for Xtensa or 'prvTaskExitError' for RISCV
        outmost_frame = len(frames) - 2
        get_logger().debug('outmost_frame = %d', outmost_frame)
        self.assertGreaterEqual(outmost_frame, 0)
        # Sometimes GDB does not provide full backtrace. so check this
        # we can only check line numbers in <outmost_func_name>,
        # because its code is under control of test framework
        if outmost_frame == 0 and func_name != outmost_func_name:
            return
        # outermost frame should be in <outmost_func_name> function
        self.assertEqual(frames[outmost_frame]['func'], outmost_func_name)
        self.gdb.select_frame(outmost_frame)
        # read line number from variable and compare with what GDB provides
        if len(lineno_var_prefs) == 1:
            line_num = self.gdb.data_eval_expr('%s_break_ln' % lineno_var_prefs[0])
            self.assertEqual(frames[outmost_frame]['line'], line_num)
        else:
            # for tests which set multiple BPs/WPs and expect hits on both cores
            # it is hard to predict the order of hits,
            # so just check that debugger has stopped on one of the locations
            line_nums = []
            for p in lineno_var_prefs:
                line_nums.append(self.gdb.data_eval_expr('%s_break_ln' % p))
            self.assertTrue(frames[outmost_frame]['line'] in line_nums)

    def run_to_bp_and_check_location(self, exp_rsn, func_name, lineno_var_pref):
        frames = self.run_to_bp_and_check_basic(exp_rsn, func_name)
        line = self.gdb.data_eval_expr('%s_break_ln' % lineno_var_pref)
        self.assertEqual(line, frames[0]['line'])

    def run_to_bp_label(self, label):
        self.resume_exec()
        rsn = self.gdb.wait_target_state(dbg.TARGET_STATE_STOPPED, 5)
        self.assertEqual(rsn, dbg.TARGET_STOP_REASON_BP)
        pc = self.gdb.get_reg('pc')
        faddr = self.gdb.extract_exec_addr(self.gdb.data_eval_expr('&%s' % label))
        self.assertEqual(pc, faddr)

class DebuggerGenericTestAppTests(DebuggerTestAppTests):
    """ Base class to run tests which use generic test app
    """

    def __init__(self, methodName):
        super(DebuggerGenericTestAppTests, self).__init__(methodName)
        self.test_app_cfg.app_name = 'gen_ut_app'
        self.test_app_cfg.bld_path = os.path.join('bootloader', 'bootloader.bin')
        self.test_app_cfg.pt_path = os.path.join('partition_table', 'partition-table.bin')
        self.test_app_cfg.test_select_var = 's_run_test'
        self.test_app_cfg.test_id_var = 's_run_test_str'


class DebuggerGenericTestAppTestsDual(DebuggerGenericTestAppTests):
    """ Base class to run tests which use generic test app in dual core mode
    """
    CORES_NUM = 2
    ENCRYPTED = 0
    def __init__(self, methodName='runTest'):
        super(DebuggerGenericTestAppTestsDual, self).__init__(methodName)
        # use default config with modified path to binaries
        self.test_app_cfg.bin_dir = os.path.join('output', 'default')
        self.test_app_cfg.build_dir = os.path.join('builds', 'default')
        self.args = []


class DebuggerGenericTestAppTestsSingle(DebuggerGenericTestAppTests):
    """ Base class to run tests which use generic test app in single core mode
    """
    CORES_NUM = 1
    ENCRYPTED = 0
    def __init__(self, methodName='runTest'):
        super(DebuggerGenericTestAppTestsSingle, self).__init__(methodName)
        # use default config with modified path to binaries
        self.test_app_cfg.bin_dir = os.path.join('output', 'single_core')
        self.test_app_cfg.build_dir = os.path.join('builds', 'single_core')
        self.args = []

class DebuggerGenericTestAppTestsDualEncrypted(DebuggerGenericTestAppTests):
    """ Base class to run tests which use generic test app in dual core encrypted mode
    """
    CORES_NUM = 2
    ENCRYPTED = 1
    def __init__(self, methodName='runTest'):
        super(DebuggerGenericTestAppTestsDualEncrypted, self).__init__(methodName)
        # use default_encrypted config with modified path to binaries
        self.test_app_cfg.bin_dir = os.path.join('output', 'default_encrypted')
        self.test_app_cfg.build_dir = os.path.join('builds', 'default_encrypted')
        self.test_app_cfg.pt_off = 0x10000
        self.test_app_cfg.app_off = 0x20000
        self.args = []

class DebuggerGenericTestAppTestsSingleEncrypted(DebuggerGenericTestAppTests):
    """ Base class to run tests which use generic test app in single core encrypted mode
    """
    CORES_NUM = 1
    ENCRYPTED = 1
    def __init__(self, methodName='runTest'):
        super(DebuggerGenericTestAppTestsSingleEncrypted, self).__init__(methodName)
        # use single_core_encrypted config with modified path to binaries
        self.test_app_cfg.bin_dir = os.path.join('output', 'single_core_encrypted')
        self.test_app_cfg.build_dir = os.path.join('builds', 'single_core_encrypted')
        self.test_app_cfg.pt_off = 0x10000
        self.test_app_cfg.app_off = 0x20000
        self.args = []

