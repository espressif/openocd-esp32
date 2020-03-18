import logging
import tempfile

import os
import os.path
import filecmp
import debug_backend as dbg
from debug_backend_tests import *

import random
import string

REMOVE_TEMP_FILES = True
USE_TEMP_FOLDER = True

def get_logger():
    return logging.getLogger(__name__)


########################################################################
#                         TESTS IMPLEMENTATION                         #
########################################################################

@idf_ver_min('4.0')
class SemihostTestsImpl:
    """
    Test cases which are common for dual and single core modes. The test's scenario:
    - Creates test_read.* files for each core of random genarated data
    - Waiting for the end odf the test, expecting to get test_write.* files for each core
    - Compare test_read.* vs test_write.*
    - Test is OK if the files in pairs the same
    """

    def setUp(self):
        def rand_seq(n_vals):
            t = ""
            while(n_vals) :
                t += random.choice(string.ascii_letters)
                n_vals -= 1
            return t

        if USE_TEMP_FOLDER:
            semi_dir = tempfile.gettempdir()
        else:
            semi_dir = os.getcwd()
        self.oocd.set_semihost_basedir(semi_dir)
        self.fout_names = []
        self.fin_names = []
        for i in range(self.CORES_NUM):
            fname = os.path.join(semi_dir, 'test_read.%d' % i)
            fout = open(fname, 'w')
            size = 1
            get_logger().info('Generate random file %dKB', size)
            for k in range(size):
                fout.write(rand_seq(1024))
            fout.close()
            self.fout_names.append(fname)
            fname = os.path.join(semi_dir, 'test_write.%d' % i)
            get_logger().info('In File %d %s', i, fname)
            self.fin_names.append(fname)
        get_logger().info('Files %s, %s', self.fout_names, self.fin_names)

    def tearDown(self):
        for fname in self.fout_names:
            if os.path.exists(fname) and REMOVE_TEMP_FILES:
                os.remove(fname)
        for fname in self.fin_names:
            if os.path.exists(fname) and REMOVE_TEMP_FILES:
                os.remove(fname)

    def test_semihost_rw(self):
        """
        """
        self.select_sub_test(700)
        self.add_bp('esp_vfs_semihost_unregister')
        self.run_to_bp(dbg.TARGET_STOP_REASON_BP, 'esp_vfs_semihost_unregister', tmo=120)
        get_logger().info('Files %s, %s', self.fout_names, self.fin_names)
        for i in range(self.CORES_NUM):
            get_logger().info('Compare files [%s, %s]', self.fout_names[i], self.fin_names[i])
            self.assertTrue(filecmp.cmp(self.fout_names[i], self.fin_names[i]))


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
