import re
from ..oocd import Oocd
from ..gdb import Gdb


class OocdXtensa(Oocd):
    """
        Class to communicate to OpenOCD supporting Xtensa-specific features
    """
    chip_name = 'xtensa'

    def __init__(self, oocd_exec=None, oocd_scripts=None, oocd_cfg_files=[], oocd_cfg_cmds=[],
                 oocd_debug=2, oocd_args=[], host='localhost', log_level=None, log_stream_handler=None, log_file_handler=None):
        super(OocdXtensa, self).__init__(oocd_exec=oocd_exec, oocd_scripts=oocd_scripts,
                                         oocd_cfg_files=oocd_cfg_files, oocd_cfg_cmds=oocd_cfg_cmds, oocd_debug=oocd_debug,
                                         oocd_args=oocd_args, host=host, log_level=log_level, log_stream_handler=log_stream_handler,
                                         log_file_handler=log_file_handler)

    def perfmon_enable(self, counter, select, mask=None, kernelcnt=None, tracelevel=None):
        """Run OpenOCD perfmon_enable command, which starts performance counter

        counter: performance counter ID
        select, mask: determine the event being profiled, refer to Xtensa Debug Guide
        kernelcnt: 0 - count events with CINTLEVEL <= tracelevel,
                   1 - count events with CINTLEVEL > tracelevel

        If mask, kernelcnt, tracelevel are not specified, OpenOCD will use default values.
        """
        cmd = 'xtensa perfmon_enable %d %d' % (counter, select)
        if mask is not None:
            cmd += ' 0x%x' % mask
        if kernelcnt is not None:
            cmd += ' %d' % kernelcnt
        if tracelevel is not None:
            cmd += ' %d' % tracelevel
        self.cmd_exec(cmd)

    def perfmon_dump(self, counter = None):
        """Run OpenOCD perfmon_dump command

        Reported results are returned as a dictionary. Each key is the counter id.
        Each value is a tuple of counts.
        If APP CPU is disabled, its count will be None.
        """
        cmd = 'xtensa perfmon_dump'
        if counter is not None:
            cmd += ' %d' % counter
        resp = self.cmd_exec(cmd)
        # Response should have one line for every counter
        result = {}
        lines = resp.split('\n')
        for line in lines:
            if len(line) == 0:
                continue
            tokens = re.match(r'Counter (?P<counter>\d+): (?P<val>\d+)', line)
            val = int(tokens.group('val'))
            counter = int(tokens.group('counter'))
            result[counter] = val
        return result


class GdbXtensa(Gdb):
    """
        Class to communicate to GDB supporting Xtensa-specific features
    """
    chip_name = 'xtensa'

    def __init__(self, gdb_path='gdb', remote_target=None, extended_remote_mode=False, gdb_log_file=None,
                 log_level=None, log_stream_handler=None, log_file_handler=None):
        super(GdbXtensa, self).__init__(gdb_path=gdb_path, remote_target=remote_target, extended_remote_mode=extended_remote_mode,
                                        gdb_log_file=gdb_log_file, log_level=log_level, log_stream_handler=log_stream_handler,
                                        log_file_handler=log_file_handler)
