import re
from ..oocd import Oocd
from ..gdb import Gdb


class OocdRiscv(Oocd):
    """
        Class to communicate to OpenOCD supporting RISCV-specific features
    """
    chip_name = 'riscv'

    def __init__(self, oocd_exec=None, oocd_scripts=None, oocd_cfg_files=[], oocd_cfg_cmds=[],
                 oocd_debug=2, oocd_args=[], host='localhost', log_level=None, log_stream_handler=None, log_file_handler=None):
        super(OocdRiscv, self).__init__(oocd_exec=oocd_exec, oocd_scripts=oocd_scripts,
                                         oocd_cfg_files=oocd_cfg_files, oocd_cfg_cmds=oocd_cfg_cmds, oocd_debug=oocd_debug,
                                         oocd_args=oocd_args, host=host, log_level=log_level, log_stream_handler=log_stream_handler,
                                         log_file_handler=log_file_handler)


class GdbRiscv(Gdb):
    """
        Class to communicate to GDB supporting RISCV-specific features
    """
    chip_name = 'riscv'

    def __init__(self, gdb_path='gdb', remote_target=None, extended_remote_mode=False, gdb_log_file=None,
                 log_level=None, log_stream_handler=None, log_file_handler=None):
        super(GdbRiscv, self).__init__(gdb_path=gdb_path, remote_target=remote_target, extended_remote_mode=extended_remote_mode,
                                        gdb_log_file=gdb_log_file, log_level=log_level, log_stream_handler=log_stream_handler,
                                        log_file_handler=log_file_handler)
