import logging
import tempfile
import os
import os.path
import filecmp
import debug_backend as dbg
from debug_backend_tests import *


def get_logger():
    return logging.getLogger(__name__)


########################################################################
#                         TESTS IMPLEMENTATION                         #
########################################################################

@idf_ver_min('latest')
class SemihostTestsImpl:
    """ Test cases which are common for dual and single core modes
    """

    def setUp(self):
        semi_dir = tempfile.gettempdir()
        self.oocd.semihost_basedir_set(semi_dir)
        self.fout_names = []
        self.fin_names = []
        for i in range(self.CORES_NUM):
            fname = os.path.join(semi_dir, 'test_read.%d' % i)
            fout = open(fname, 'w')
            size = 1
            get_logger().info('Generate random file %dKB', size)
            for k in range(size):
                fout.write(os.urandom(1024))
            fout.write(os.urandom(3))
            fout.close()
            self.fout_names.append(fname)
            fname = os.path.join(semi_dir, 'test_write.%d' % i)
            get_logger().info('In File %d %s', i, fname)
            self.fin_names.append(fname)
        get_logger().info('Files0 %s, %s', self.fout_names, self.fin_names)

    def tearDown(self):
        for fname in self.fout_names:
            if os.path.exists(fname):
                os.remove(fname)
        for fname in self.fin_names:
            if os.path.exists(fname):
                os.remove(fname)

    def test_semihost_rw(self):
        """
        """
        self.select_sub_test(700)
        self.add_bp('esp_vfs_semihost_unregister')
        self.resume_exec()
        self.gdb.wait_target_state(dbg.Gdb.TARGET_STATE_STOPPED, 120)
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
