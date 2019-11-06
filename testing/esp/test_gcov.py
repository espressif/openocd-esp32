import logging
import unittest
import os
import os.path
import subprocess
import debug_backend as dbg
from debug_backend_tests import *


def get_logger():
    return logging.getLogger(__name__)


class GcovDataError(RuntimeError):
    """ GCOV data error
    """
    pass


class GcovDataFile:
    """ GCOV data file wrapper
    """
    GCOV_FILE_TAG = 'file:'
    GCOV_FUNC_TAG = 'function:'
    GCOV_LCOUNT_TAG = 'lcount:'
    GCOV_BRANCH_TAG = 'branch:'
    GCOV_VERSION_TAG = 'version:'

    def __init__(self, path, src_dirs, build_path='', proj_path=''):
        self._path = path
        self.data = {}
        self.src_dirs = src_dirs
        self._cur_file = ''
        get_logger().debug('Process gcov file "%s"', path)
        dir_name,file_name = os.path.split(path)
        get_logger().debug('Gcov file dir "%s" filename "%s"', dir_name, file_name)
        _,file_ext = os.path.splitext(file_name)
        # get_logger().debug('Gcov filename "%s" ext "%s"', fname, file_ext)
        if file_ext == '.gcov':
            gcov_name = path
        else:
            out = subprocess.check_output(['%sgcov' % dbg.toolchain, '-ib', path], stderr=subprocess.STDOUT)
            get_logger().debug('GCOV: %s', out)
            gcov_name = '%s.gcov' % file_name
        f = open(gcov_name)
        for ln in f:
            if ln.startswith(self.GCOV_FILE_TAG):
                fname = ln[len(self.GCOV_FILE_TAG):].rstrip()
                # skip path conversion if it is already done, e.g. in case of reference .gcov file
                if not (fname.startswith('$PROJECT_PATH') or fname.startswith('$IDF_PATH')):
                    if not os.path.isabs(fname):
                        # path relative to the build dir
                        fname = os.path.normpath(os.path.join(build_path, fname))
                    idf_path = os.getenv('IDF_PATH', '')
                    get_logger().debug('SRC FILE "%s"', fname)
                    prefix = os.path.commonprefix([idf_path, fname])
                    get_logger().debug('IDF_PREF "%s" "%s"', idf_path, prefix)
                    if prefix == idf_path:
                        fname = '$IDF_PATH' + fname[len(prefix):]
                    else:
                        prefix = os.path.commonprefix([proj_path, fname])
                        get_logger().debug('PROJ_PREF "%s" "%s"', proj_path, prefix)
                        if prefix == proj_path:
                            fname = '$PROJECT_PATH' + fname[len(prefix):]
                get_logger().debug('Found src file "%s"', fname)
                self.data[fname] = {}
                self.data[fname]['funcs'] = {}
                self.data[fname]['lc'] = []
                self.data[fname]['br'] = []
                self._cur_file = fname
            elif ln.startswith(self.GCOV_FUNC_TAG):
                func = ln[len(self.GCOV_FUNC_TAG):].rstrip().split(',')
                if len(func) < 3:
                    raise GcovDataError('Too short func line')
                get_logger().debug('Found func "%s"', func[2])
                self.data[self._cur_file]['funcs'][func[2]] = {'sl' : func[0], 'ec' : func[1]}
            elif ln.startswith(self.GCOV_LCOUNT_TAG):
                lcount = ln[len(self.GCOV_LCOUNT_TAG):].rstrip().split(',')
                if len(lcount) < 2:
                    raise GcovDataError('Too short LC line')
                self.data[self._cur_file]['lc'].append(lcount)
            elif ln.startswith(self.GCOV_BRANCH_TAG):
                br = ln[len(self.GCOV_BRANCH_TAG):].rstrip().split(',')
                if len(br) < 2:
                    raise GcovDataError('Too short BR line')
                self.data[self._cur_file]['br'].append(br)
            elif ln.startswith(self.GCOV_VERSION_TAG):
                pass
            else:
                raise GcovDataError('Unknown tag in line "%s"' % ln)
        f.close()

    def __eq__(self, other):
        for fname in self.data:
            get_logger().debug('Check Gcov fname: %s', fname)
            if fname not in other.data:
                get_logger().error('Check Gcov fname: %s not in %s (%s)', fname, other.data.keys(), other._path)
                return False
            in_src_dirs = False
            for d in self.src_dirs:
                if os.path.commonprefix([d, fname]) == d:
                    in_src_dirs = True
                    break
            if not in_src_dirs:
                continue
            for func in self.data[fname]['funcs']:
                if func not in other.data[fname]['funcs']:
                    return False
            for i in range(len(self.data[fname]['lc'])):
                if self.data[fname]['lc'][i][0] != other.data[fname]['lc'][i][0] or self.data[fname]['lc'][i][1] != other.data[fname]['lc'][i][1]:
                    return False
            for i in range(len(self.data[fname]['br'])):
                if self.data[fname]['br'][i][0] != other.data[fname]['br'][i][0] or self.data[fname]['br'][i][1] != other.data[fname]['br'][i][1]:
                    return False
        return True

    def __repr__(self):
        return self._path

    def get_lines_coverage(self, fname, start, end):
        lines_cov = []
        if fname in self.data:
            for i in range(len(self.data[fname]['lc'])):
                ln = int(self.data[fname]['lc'][i][0])
                if ln >= start and ln <= end:
                    lines_cov.append((int(self.data[fname]['lc'][i][0]), int(self.data[fname]['lc'][i][1])))
        return lines_cov


########################################################################
#                         TESTS IMPLEMENTATION                         #
########################################################################

class GcovTestsImpl:
    """ Test cases which are common for dual and single core modes

        NOTE: GCOV tests use reference '*.gcov' files which are located in the test app source dir.
        Every test source file have corresponding '*.gcda.gcov' file which is compared against gcov data
        received from the target during tests. If code in any test C file is changed the corresponding
        reference gcov file is also to be regenerated with the following commands:
        1) Build unit test app with updated sources.
        2) Run it on target and collect GCOV data (*.gcda) using OpenOCD.
        3) Generate new ref gcov file 'xtensa-esp32-elf-gcov -ib <path_to_gcda_file>'
        4) Replace old ref file with the new one.
    """
    # lines numbers range which execution count does not change during test: for 'main/gcov_tests.c' and 'main/helper_funcs.gcda'
    CONST_LINES_START = [15, None]
    CONST_LINES_END = [26, None]
    # lines numbers range which execution count changes during test: for 'main/gcov_tests.c' and 'main/helper_funcs.gcda'
    DYN_LINES_START = [28, 6]
    DYN_LINES_END = [39, 10]

    # This function expcets normalized path
    def strip_gcov_path(self, path):
        gcov_prefix_strip = int(os.getenv('OPENOCD_GCOV_PREFIX_STRIP', '0'))
        if gcov_prefix_strip > 0:
            path_comps = path.split(os.sep)
            # del the leading empty string
            if len(path_comps) > 0 and path_comps[0] == '':
                path_comps = path_comps[1:]
            if gcov_prefix_strip > (len(path_comps) - 1):
                gcov_prefix_strip = len(path_comps) - 1
            path = os.path.join(*path_comps[gcov_prefix_strip:])
        return path


    def setUp(self):
        self.gcov_files = []
        # for test app project path is two level higher than build dir
        self.proj_path = os.path.normpath(os.path.join(self.test_app_cfg.build_obj_dir(), '..', '..'))
        self.gcov_prefix = os.getenv('OPENOCD_GCOV_PREFIX', self.test_app_cfg.build_obj_dir())
        self.src_dirs = [self.test_app_cfg.build_src_dir(),]
        src_path = os.path.join(self.test_app_cfg.build_src_dir(), 'main', 'gcov_tests.c')
        if IdfVersion.get_current() < IdfVersion.fromstr('4.0'):
            data_path = os.path.join(self.test_app_cfg.build_obj_dir(), 'main', 'gcov_tests.gcda')
        else:
            # starting from IDF 4.0 test app supports cmake build system which uses another build dir structure
            data_path = os.path.join(self.test_app_cfg.build_obj_dir(), 'esp-idf', 'main', 'CMakeFiles', '__idf_main.dir', 'gcov_tests.c.gcda')
        ref_data = GcovDataFile(os.path.join(self.test_app_cfg.build_src_dir(), 'main', 'gcov_tests.gcda.gcov'), self.src_dirs)
        self.gcov_files.append({
            'src_path' : src_path,
            'data_path' : os.path.join(self.gcov_prefix, self.strip_gcov_path(data_path)),
            'ref_data' : ref_data,
            # lines executed only once
            'c_lines' : ref_data.get_lines_coverage(src_path, self.CONST_LINES_START[0], self.CONST_LINES_END[0]),
            # lines executed every gcov dump cycle
            'd_lines' : ref_data.get_lines_coverage(src_path, self.DYN_LINES_START[0], self.DYN_LINES_END[0])
            })
        src_path = os.path.join(self.test_app_cfg.build_src_dir(), 'main', 'helper_funcs.c')
        if IdfVersion.get_current() < IdfVersion.fromstr('4.0'):
            data_path = os.path.join(self.test_app_cfg.build_obj_dir(), 'main', 'helper_funcs.gcda')
        else:
            # starting from IDF 4.0 test app supports cmake build system which uses another build dir structure
            data_path = os.path.join(self.test_app_cfg.build_obj_dir(), 'esp-idf', 'main', 'CMakeFiles', '__idf_main.dir', 'helper_funcs.c.gcda')
        ref_data = GcovDataFile(os.path.join(self.test_app_cfg.build_src_dir(), 'main', 'helper_funcs.gcda.gcov'), self.src_dirs)
        self.gcov_files.append({
            'src_path' : src_path,
            'data_path' : os.path.join(self.gcov_prefix, self.strip_gcov_path(data_path)),
            'ref_data' : ref_data,
            # lines executed only once
            'c_lines' : None,
            # lines executed every gcov dump cycle
            'd_lines' : ref_data.get_lines_coverage(src_path, self.DYN_LINES_START[1], self.DYN_LINES_END[1])
            })
        # remove old gcov files
        for f in self.gcov_files:
            if os.path.exists(f['data_path']):
                os.remove(f['data_path'])

    def test_simple_gdb(self):
        """
            This test checks that GCOV data can be dumped by means of GDB
            from the target app which uses pre-compiled code calling 'esp_gcov_dump()' for data transfer.
            1) Select appropriate sub-test number on target.
            2) Set breakpoint at 'esp_gcov_dump()'.
            3) Resume target and wait for brekpoints to hit.
            4) Check that target has stopped in the right place.
            5) Run 'mon esp32 gcov dump'.
            6) Compare collected data with the reference one.
              - after the first test iteration gcov data should be equal to the reference one
              - on all the next iterations gcov data should be changed only for lines which are executed in the loop (see source code)
            7) Repeat steps 3-6 several times.
            8) Finally delete breakpoint
        """
        self.select_sub_test(300)
        bp = self.gdb.add_bp('esp_gcov_dump')
        for i in range(5):
            self.resume_exec()
            rsn = self.gdb.wait_target_state(dbg.Gdb.TARGET_STATE_STOPPED, 5)
            self.assertEqual(rsn, dbg.Gdb.TARGET_STOP_REASON_BP)
            cur_frame = self.gdb.get_current_frame()
            self.assertEqual(cur_frame['func'], 'esp_gcov_dump')
            self.step()
            self.gdb.gcov_dump(False)
            # parse and check gcov data
            gcov_data_files = []
            for f in self.gcov_files:
                gcov_data_files.append(GcovDataFile(f['data_path'], self.src_dirs,
                                        self.test_app_cfg.build_obj_dir(), self.proj_path))
            if i == 0:
                # after first test iteration gcov data should be equal to reference ones
                for k in range(len(gcov_data_files)):
                    self.assertEqual(gcov_data_files[k], self.gcov_files[k]['ref_data'])
            else:
                for n in range(len(gcov_data_files)):
                    if self.gcov_files[n]['c_lines']:
                        # check constant lines
                        c_lines = gcov_data_files[n].get_lines_coverage(self.gcov_files[n]['src_path'], self.CONST_LINES_START[n], self.CONST_LINES_END[n])
                        self.assertNotEqual(len(c_lines), 0)
                        self.assertEqual(len(c_lines), len(self.gcov_files[n]['c_lines']))
                        for k in range(len(c_lines)):
                            self.assertEqual(self.gcov_files[n]['c_lines'][k][0], c_lines[k][0])
                            self.assertEqual(self.gcov_files[n]['c_lines'][k][1], c_lines[k][1])
                    if self.gcov_files[n]['d_lines']:
                        # check dynamic lines
                        d_lines = gcov_data_files[n].get_lines_coverage(self.gcov_files[n]['src_path'], self.DYN_LINES_START[n], self.DYN_LINES_END[n])
                        self.assertNotEqual(len(d_lines), 0)
                        self.assertEqual(len(d_lines), len(self.gcov_files[n]['d_lines']))
                        for k in range(len(d_lines)):
                            self.assertEqual(self.gcov_files[n]['d_lines'][k][0], d_lines[k][0])
                            self.assertEqual(self.gcov_files[n]['d_lines'][k][1] + i, d_lines[k][1])
        self.gdb.delete_bp(bp)

    def test_simple_oocd(self):
        """
            This test checks that GCOV data can be dumped by means of OpenOCD
            from the target app which uses pre-compiled code calling 'esp_gcov_dump()' for data transfer.
            1) Select appropriate sub-test number on target.
            3) Resume target.
            4) Wait some time to allow test app to call 'esp_gcov_dump()'.
            5) Halt test app. (Doing it via GDB to simplify test implementation).
            5) Run 'esp32 gcov dump'.
            6) Compare collected data with the reference one.
        """
        self.select_sub_test(300)
        self.resume_exec()
        # TODO: better way to ensure that test app called esp_gcov_dump
        time.sleep(3)
        self.stop_exec()
        self.oocd.gcov_dump(False)
        # parse and check gcov data
        if IdfVersion.get_current() < IdfVersion.fromstr('4.0'):
            data_path = os.path.join(self.test_app_cfg.build_obj_dir(), 'main', 'gcov_tests.gcda')
        else:
            # starting from IDF 4.0 test app supports cmake build system which uses another build dir structure
            data_path = os.path.join(self.test_app_cfg.build_obj_dir(), 'esp-idf', 'main', 'CMakeFiles', '__idf_main.dir', 'gcov_tests.c.gcda')
        f = GcovDataFile(os.path.join(self.gcov_prefix, self.strip_gcov_path(data_path)), self.src_dirs,
                        self.test_app_cfg.build_obj_dir(), self.proj_path)
        f2 = GcovDataFile(os.path.join(self.test_app_cfg.build_src_dir(), 'main', 'gcov_tests.gcda.gcov'), self.src_dirs)
        self.assertEqual(f, f2)
        if IdfVersion.get_current() < IdfVersion.fromstr('4.0'):
            data_path = os.path.join('main', 'helper_funcs.gcda')
        else:
            # starting from IDF 4.0 test app supports cmake build system which uses another build dir structure
            data_path = os.path.join('esp-idf', 'main', 'CMakeFiles', '__idf_main.dir', 'helper_funcs.c.gcda')
        f = GcovDataFile(os.path.join(self.gcov_prefix, self.strip_gcov_path(data_path)), self.src_dirs,
                        self.test_app_cfg.build_obj_dir(), self.proj_path)
        f2 = GcovDataFile(os.path.join(self.test_app_cfg.build_src_dir(), 'main', 'helper_funcs.gcda.gcov'), self.src_dirs)
        self.assertEqual(f, f2)

    def test_on_the_fly_gdb(self):
        """
            This test checks that GCOV data can be dumped by means of GDB
            from the target app which does not use 'esp_gcov_dump()' for data transfer.
            1) Select appropriate sub-test number on target.
            3) Resume target.
            4) Wait some time to allow test app to change GCOV counters.
            5) Halt test app. Otherwise we can not run the next command via GDB.
            6) Run 'mon esp32 gcov'.
            7) Compare collected data with the reference one.
        """
        self.select_sub_test(301)
        self.resume_exec()
        # wait some time to let app run and generate gcov data
        time.sleep(3)
        self.stop_exec()
        self.gdb.gcov_dump(True)
        # do not check gcov data, because its hard to precdict their contents
        # just check that files exist, contents are checked in test_simple_xxx tests
        if IdfVersion.get_current() < IdfVersion.fromstr('4.0'):
            data_path = os.path.join(self.test_app_cfg.build_obj_dir(), 'main', 'gcov_tests.gcda')
        else:
            # starting from IDF 4.0 test app supports cmake build system which uses another build dir structure
            data_path = os.path.join(self.test_app_cfg.build_obj_dir(), 'esp-idf', 'main', 'CMakeFiles', '__idf_main.dir', 'gcov_tests.c.gcda')
        self.assertTrue(os.path.exists(os.path.join(self.gcov_prefix, self.strip_gcov_path(data_path))))
        if IdfVersion.get_current() < IdfVersion.fromstr('4.0'):
            data_path = os.path.join(self.test_app_cfg.build_obj_dir(), 'main', 'helper_funcs.gcda')
        else:
            # starting from IDF 4.0 test app supports cmake build system which uses another build dir structure
            data_path = os.path.join(self.test_app_cfg.build_obj_dir(), 'esp-idf', 'main', 'CMakeFiles', '__idf_main.dir', 'helper_funcs.c.gcda')
        self.assertTrue(os.path.exists(os.path.join(self.gcov_prefix, self.strip_gcov_path(data_path))))

    def test_on_the_fly_oocd(self):
        """
            This test checks that GCOV data can be dumped by means of OpenOCD
            from the target app which does not use 'esp_gcov_dump()' for data transfer.
            1) Select appropriate sub-test number on target.
            3) Resume target.
            4) Wait some time to allow test app to change GCOV counters.
            5) Run 'esp32 gcov'.
            6) Compare collected data with the reference one.
        """
        self.select_sub_test(301)
        self.resume_exec()
        # wait some time to let app run and generate gcov data
        time.sleep(3)
        self.oocd.gcov_dump(True)
        time.sleep(1)
        # do not check gcov data, because its hard to precdict their contents
        # just check that files exist, contents are checked in test_simple_xxx tests
        if IdfVersion.get_current() < IdfVersion.fromstr('4.0'):
            data_path = os.path.join(self.test_app_cfg.build_obj_dir(), 'main', 'gcov_tests.gcda')
        else:
            # starting from IDF 4.0 test app supports cmake build system which uses another build dir structure
            data_path = os.path.join(self.test_app_cfg.build_obj_dir(), 'esp-idf', 'main', 'CMakeFiles', '__idf_main.dir', 'gcov_tests.c.gcda')
        self.assertTrue(os.path.exists(os.path.join(self.gcov_prefix, self.strip_gcov_path(data_path))))
        if IdfVersion.get_current() < IdfVersion.fromstr('4.0'):
            data_path = os.path.join(self.test_app_cfg.build_obj_dir(), 'main', 'helper_funcs.gcda')
        else:
            # starting from IDF 4.0 test app supports cmake build system which uses another build dir structure
            data_path = os.path.join(self.test_app_cfg.build_obj_dir(), 'esp-idf', 'main', 'CMakeFiles', '__idf_main.dir', 'helper_funcs.c.gcda')
        self.assertTrue(os.path.exists(os.path.join(self.gcov_prefix, self.strip_gcov_path(data_path))))


########################################################################
#              TESTS DEFINITION WITH SPECIAL TESTS                     #
########################################################################

class GcovTestAppTestsDual(DebuggerGenericTestAppTests):
    """ Base class to run tests which use gcov test app in dual core mode
    """

    def __init__(self, methodName='runTest'):
        super(GcovTestAppTestsDual, self).__init__(methodName)
        self.test_app_cfg.bin_dir = os.path.join('output', 'gcov_dual')
        self.test_app_cfg.build_dir = os.path.join('builds', 'gcov_dual')


class GcovTestAppTestsSingle(DebuggerGenericTestAppTests):
    """ Base class to run tests which use gcov test app in single core mode
    """

    def __init__(self, methodName='runTest'):
        super(GcovTestAppTestsSingle, self).__init__(methodName)
        self.test_app_cfg.bin_dir = os.path.join('output', 'gcov_single')
        self.test_app_cfg.build_dir = os.path.join('builds', 'gcov_single')


@unittest.expectedFailure
class GcovTestsDual(GcovTestAppTestsDual, GcovTestsImpl):
    """ Test cases via GDB in dual core mode
    """
    def setUp(self):
        GcovTestAppTestsDual.setUp(self)
        GcovTestsImpl.setUp(self)


@unittest.expectedFailure
class GcovTestsSingle(GcovTestAppTestsSingle, GcovTestsImpl):
    """ Test cases via GDB in single core mode
    """

    def setUp(self):
        GcovTestAppTestsSingle.setUp(self)
        GcovTestsImpl.setUp(self)
