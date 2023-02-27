import logging
import tempfile

import os
import os.path
import filecmp
import debug_backend as dbg
from debug_backend_tests import *
import shutil
import platform

import random
import string

REMOVE_TEMP_FILES = True
USE_TEMP_FOLDER = True

def get_logger():
    return logging.getLogger(__name__)


########################################################################
#                         TESTS IMPLEMENTATION                         #
########################################################################
@idf_ver_min_for_arch('5.0', ['riscv32'])
class SemihostTestsImpl:
    """
    Test cases which are common for dual and single core modes. The test's scenario:
    - Creates test_read.* files for each core of random genarated data
    - Waiting for the end odf the test, expecting to get test_write.* files for each core
    - Compare test_read.* vs test_write.*
    - Test is OK if the files in pairs the same
    """
    if USE_TEMP_FOLDER:
        semi_dir = tempfile.gettempdir()
    else:
        semi_dir = os.getcwd()

    def setUp(self):
        def rand_seq(n_vals):
            t = ""
            while(n_vals) :
                t += random.choice(string.ascii_letters)
                n_vals -= 1
            return t
        self.fout_names = []
        self.fin_names = []
        self.fio_names = []
        for i in range(self.CORES_NUM):
            fname = os.path.join(self.semi_dir, 'test_read.%d' % i)
            fout = open(fname, 'w')
            fnametrunc = os.path.join(self.semi_dir, 'truncate_file.%d' % i)
            ftrunc = open(fnametrunc, 'w')
            size = 1
            get_logger().info('Generate random file %dKB', size)
            for k in range(size):
                fout.write(rand_seq(1024))
                ftrunc.write(rand_seq(1024))
            fout.close()
            ftrunc.close()
            self.fout_names.append(fname)
            fname = os.path.join(self.semi_dir, 'test_write.%d' % i)
            get_logger().info('In File %d %s', i, fname)
            self.fin_names.append(fname)

            self.fio_names_tmp = []
            """
            old_file file created to test rename function working correctly
            """
            fname = os.path.join(self.semi_dir, 'old_file.%d' % i)
            fout = open(fname, 'w')
            get_logger().info('IO File %d %s', i, fname)
            self.fio_names_tmp.append(fname)
            """
            test_link link created to test unlink function working correctly
            """
            fname = os.path.join(self.semi_dir, 'test_read.%d' % i)
            link_name = os.path.join(self.semi_dir, 'test_link.%d' % i)
            if os.path.exists(link_name):
                os.remove(link_name)
            os.link(fname, link_name)
            self.fio_names_tmp.append(link_name)
            """
            These for checking some test outputs. Some files needs to exist
            some of them not exist if everything goes right
            """
            fname = os.path.join(self.semi_dir, 'link_file_idf.%d' % i)
            get_logger().info('IO File %d %s', i, fname)
            self.fio_names_tmp.append(fname)
            fname = os.path.join(self.semi_dir, 'renamed_file_idf.%d' % i)
            get_logger().info('IO File %d %s', i, fname)
            self.fio_names_tmp.append(fname)
            fname = os.path.join(self.semi_dir, 'truncate_file.%d' % i)
            get_logger().info('IO File %d %s', i, fname)
            self.fio_names_tmp.append(fname)
            """
            This folder created to check opendir test working correctly.
            After test this folder should not exist
            """
            dir_name = os.path.join(self.semi_dir, 'opendir_test.%d' % i)
            get_logger().info('IO Folder %d %s', i, dir_name)
            if os.path.exists(dir_name):
                shutil.rmtree(dir_name)
            os.mkdir(dir_name)
            self.fio_names_tmp.append(dir_name)
            """
            This folder and file created to check readdir test working correctly.
            Last file name is for checking file renamed correctly during readdir test
            """
            dir_name = os.path.join(self.semi_dir, 'readdir_test.%d' % i)
            get_logger().info('IO Folder %d %s', i, dir_name)
            if os.path.exists(dir_name):
                shutil.rmtree(dir_name)
            os.mkdir(dir_name)
            self.fio_names_tmp.append(dir_name)
            fname = os.path.join(dir_name, 'old_file.%d' % i)
            fout = open(fname, 'w')
            self.fio_names_tmp.append(fname)
            fname = os.path.join(dir_name, 'renamed_file_idf.%d' % i)
            self.fio_names_tmp.append(fname)
            self.fio_names.append(self.fio_names_tmp)

        get_logger().info('Files %s, %s', self.fout_names, self.fin_names)

    def tearDown(self):
        for fname in self.fout_names:
            if os.path.exists(fname) and REMOVE_TEMP_FILES:
                os.remove(fname)
        for fname in self.fin_names:
            if os.path.exists(fname) and REMOVE_TEMP_FILES:
                os.remove(fname)
        for i in range(self.CORES_NUM):
            for fname in self.fio_names[i]:
                if os.path.exists(fname) and REMOVE_TEMP_FILES:
                    if os.path.isdir(fname):
                        shutil.rmtree(fname, ignore_errors=False)
                    else:
                        os.remove(fname)

    def test_semihost_rw(self):
        """
        This test checks that semihost functions working as expected.
        In the 1st loop new `arm semihosting_basedir` command is used.
        In the 2nd loop old `esp semihost_basedir` command is used.
        Remove 2nd loop when the old command support dropped.
        """
        for i in range(2):
            if i == 0:
                self.oocd.set_smp_semihosting_basedir(self.semi_dir)
            else:
                self.oocd.set_semihost_basedir(self.semi_dir)
            self.select_sub_test(700)
            self.add_bp('esp_vfs_semihost_unregister')
            self.run_to_bp(dbg.TARGET_STOP_REASON_BP, 'esp_vfs_semihost_unregister', tmo=120)
            get_logger().info('Files %s, %s', self.fout_names, self.fin_names)
            for i in range(self.CORES_NUM):
                get_logger().info('Compare files [%s, %s]', self.fout_names[i], self.fin_names[i])
                self.assertTrue(filecmp.cmp(self.fout_names[i], self.fin_names[i]))
            self.gdb.target_reset()
            self.gdb.add_bp('app_main')
            self.run_to_bp(dbg.TARGET_STOP_REASON_BP, 'app_main')

    @only_for_arch(['xtensa'])
    def test_semihost_args(self):
        """
        This test checks that semihosting syscalls working properly with wrong argumented functions
        """
        self.oocd.set_smp_semihosting_basedir(self.semi_dir)
        self.select_sub_test(701)
        self.add_bp('esp_vfs_semihost_unregister')
        self.run_to_bp(dbg.TARGET_STOP_REASON_BP, 'esp_vfs_semihost_unregister', tmo=120)

    @idf_ver_min('5.0')
    def test_semihost_custom(self):
        """
        This test checks that custom syscalls working properly
        """
        self.oocd.set_smp_semihosting_basedir(self.semi_dir)
        if platform.system() == "Windows":
            self.select_sub_test(703)
        else:
            self.select_sub_test(702)
        self.add_bp('esp_vfs_semihost_unregister')
        self.run_to_bp(dbg.TARGET_STOP_REASON_BP, 'esp_vfs_semihost_unregister', tmo=120)
        for i in range(self.CORES_NUM):
            # This test checks old_file file not exist after rename operation
            get_logger().info('Checking file not exist [%s]', self.fio_names[i][0])
            self.assertFalse(os.path.exists(self.fio_names[i][0]))
            # This test checks test_link file not exist after unlink operation
            get_logger().info('Checking file not exist [%s]', self.fio_names[i][1])
            self.assertFalse(os.path.exists(self.fio_names[i][1]))
            # This test checks link_file_idf file exist after link operation
            get_logger().info('Checking file not exist [%s]', self.fio_names[i][2])
            self.assertTrue(os.path.exists(self.fio_names[i][2]))
            # This test checks renamed_file_idf file exist after rename operation
            get_logger().info('Checking file exist [%s]', self.fio_names[i][3])
            self.assertTrue(os.path.exists(self.fio_names[i][3]))
            # This test checks truncate_file file length is 15 bytes long after truncate operation
            get_logger().info('Checking file length [%s]', self.fio_names[i][4])
            size = os.path.getsize(self.fio_names[i][4])
            self.assertTrue(size == 15)
            # This test checks opendir_test folder not exist after opendir test
            get_logger().info('Checking file not exist [%s]', self.fio_names[i][5])
            self.assertFalse(os.path.exists(self.fio_names[i][5]))
            # This test checks readdir_test/old_file file not exist after rename operation
            get_logger().info('Checking file not exist [%s]', self.fio_names[i][7])
            self.assertFalse(os.path.exists(self.fio_names[i][7]))
            # This test checks readdir_test/renamed_file_idf file exist after rename operation
            get_logger().info('Checking file exist [%s]', self.fio_names[i][8])
            self.assertTrue(os.path.exists(self.fio_names[i][8]))
            # This test checks test_read file modtime value is changed after utime operation
            get_logger().info('Checking modification time [%s]', self.fout_names[i])
            mtime = os.path.getmtime(self.fout_names[i])
            self.assertTrue(mtime == 456789)

########################################################################
#              TESTS DEFINITION WITH SPECIAL TESTS                     #
########################################################################

class SemihostTestsDual(DebuggerGenericTestAppTestsDual, SemihostTestsImpl):
    """ Test cases in dual core mode
    """

    def setUp(self):
        DebuggerGenericTestAppTestsDual.setUp(self)
        SemihostTestsImpl.setUp(self)

    def tearDown(self):
        DebuggerGenericTestAppTestsDual.tearDown(self)
        SemihostTestsImpl.tearDown(self)

class SemihostTestsSingle(DebuggerGenericTestAppTestsSingle, SemihostTestsImpl):
    """ Test cases in single core mode
    """

    def setUp(self):
        DebuggerGenericTestAppTestsSingle.setUp(self)
        SemihostTestsImpl.setUp(self)

    def tearDown(self):
        DebuggerGenericTestAppTestsSingle.tearDown(self)
        SemihostTestsImpl.tearDown(self)
