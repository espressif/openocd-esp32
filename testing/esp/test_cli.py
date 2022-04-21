import logging
import debug_backend as dbg
from debug_backend_tests import *


def get_logger():
    return logging.getLogger(__name__)


########################################################################
#                         TESTS IMPLEMENTATION                         #
########################################################################
class MacTestsImpl:
    """
    Test cases which are common for dual and single core modes. The test's scenario:
    ---

    """
    def setUp(self):
        self.oocd.cmd_exec("reset halt")

    def tearDown(self):
        pass

    def get_mac_manually(self):
        """
        Returns
        -------
        List[str]
        """
        if testee_info.arch == "xtensa":
            self.oocd.cmd_exec("xtensa set_permissive 1")
            self.oocd.cmd_exec("set mac_list [read_memory $EFUSE_MAC_ADDR_REG 8 6]")
            self.oocd.cmd_exec("xtensa set_permissive 0")
        else: #riscv32
            self.oocd.cmd_exec("set mac_list [read_memory $EFUSE_MAC_ADDR_REG 8 6]")
        m0 = self.oocd.cmd_exec("format %02x [lindex $mac_list 0]").strip('\n')
        m1 = self.oocd.cmd_exec("format %02x [lindex $mac_list 1]").strip('\n')
        m2 = self.oocd.cmd_exec("format %02x [lindex $mac_list 2]").strip('\n')
        m3 = self.oocd.cmd_exec("format %02x [lindex $mac_list 3]").strip('\n')
        m4 = self.oocd.cmd_exec("format %02x [lindex $mac_list 4]").strip('\n')
        m5 = self.oocd.cmd_exec("format %02x [lindex $mac_list 5]").strip('\n')
        get_logger().debug("Read using read_memory: '%s %s %s %s %s %s'" % (m5, m4, m3, m2, m1, m0))
        return (m5, m4, m3, m2, m1, m0)

    def test_mac_cmd(self):
        """
        Test of esp_get_mac command
        - read mac in 6 strings, each string - value of byte
        - read results of 'esp_get_mac' and 'esp_get_mac format'
        - compare results with expected ones
        """
        m5, m4, m3, m2, m1, m0 = self.get_mac_manually()

        mac_hex = ""
        mac_formatted = ""
        mac_list = self.oocd.cmd_exec("esp_get_mac").splitlines()
        for mac_str in mac_list:
            if mac_str.find("xtensa permissive mode") == -1:
                mac_hex = mac_str
                break
        mac_hex_expected = "0x0000%s%s%s%s%s%s" % (m5, m4, m3, m2, m1, m0)
        get_logger().debug("Read using 'esp_get_mac': '%s'" % mac_hex)
        get_logger().debug("Expected using 'esp_get_mac': '%s'" % mac_hex_expected)

        mac_list = self.oocd.cmd_exec("esp_get_mac format").splitlines()
        for mac_str in mac_list:
            if mac_str.find("xtensa permissive mode") == -1:
                mac_formatted = mac_str
                break
        mac_formatted_expected = "%s:%s:%s:%s:%s:%s" % (m5, m4, m3, m2, m1, m0)
        get_logger().debug("Read using 'esp_get_mac format': '%s'" % mac_formatted)
        get_logger().debug("Expected using 'esp_get_mac format': '%s'" % mac_formatted_expected)

        self.assertEqual(mac_hex, mac_hex_expected)
        self.assertEqual(mac_formatted, mac_formatted_expected)


########################################################################
#              TESTS DEFINITION WITH SPECIAL TESTS                     #
########################################################################


class MacTestsDual(DebuggerGenericTestAppTestsDual, MacTestsImpl):
    """ Test cases in dual core mode
    """
    def setUp(self):
        DebuggerGenericTestAppTestsDual.setUp(self)
        MacTestsImpl.setUp(self)

    def tearDown(self):
        DebuggerGenericTestAppTestsDual.tearDown(self)
        MacTestsImpl.tearDown(self)


class MacTestsSingle(DebuggerGenericTestAppTestsSingle, MacTestsImpl):
    """ Test cases in single core mode
    """
    def setUp(self):
        DebuggerGenericTestAppTestsSingle.setUp(self)
        MacTestsImpl.setUp(self)

    def tearDown(self):
        DebuggerGenericTestAppTestsSingle.tearDown(self)
        MacTestsImpl.tearDown(self)
