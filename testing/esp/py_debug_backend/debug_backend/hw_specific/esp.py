import os
import time
from ..defs import *
from ..utils import fixup_path
from .xtensa import *
from .riscv import *


class OocdEspImpl:
    """
        Class to communicate to OpenOCD supporting ESP-specific features
    """

    def set_appimage_offset(self, app_flash_off):
        self.cmd_exec('esp appimage_offset 0x%x' % app_flash_off)

    def set_semihost_basedir(self, semi_dir):
        self.cmd_exec('esp semihost_basedir %s' % (fixup_path(semi_dir)))

    def gcov_dump(self, on_the_fly=True):
        if on_the_fly:
            cmd = 'esp gcov'
        else:
            cmd = 'esp gcov dump'
        self.cmd_exec(cmd)

    def sysview_start(self, file1, file2=''):
        cmd_out = self.cmd_exec('esp sysview start %s %s' % (file1, file2))
        if 'Targets connected.' not in cmd_out:
            raise DebuggerError('Failed to start sysview with args "%s %s"!' % (file1, file2))

    def sysview_stop(self):
        cmd_out = self.cmd_exec('esp sysview stop')
        if 'Targets disconnected.' not in cmd_out:
            raise DebuggerError('Failed to stop sysview!')

    def apptrace_start(self, trace_args):
        cmd_out = self.cmd_exec("esp apptrace start %s" % trace_args)
        if 'Targets connected.' not in cmd_out:
            raise DebuggerError('Failed to start apptrace with args "%s"!' % (trace_args))

    def apptrace_stop(self):
        cmd_out = self.cmd_exec("esp apptrace stop")
        if 'Targets disconnected.' not in cmd_out:
            raise DebuggerError('Failed to stop apptrace!')

    def apptrace_wait_stop(self, tmo=10):
        stopped = False
        end = time.time()
        if tmo:
            end += tmo
        while not stopped:
            cmd_out = self.cmd_exec("esp apptrace status")
            for line in cmd_out.splitlines():
                if line.startswith("Tracing is STOPPED."):
                    stopped = True
                    break
            if not stopped and tmo and time.time() > end:
                raise DebuggerError('Failed to wait for apptrace stop!')


class OocdEspXtensa(OocdXtensa, OocdEspImpl):
    """
        Class to communicate to OpenOCD supporting ESP Xtensa-specific features
    """

    def __init__(self, cores_num=1, oocd_exec=None, oocd_scripts=None, oocd_cfg_files=[], oocd_cfg_cmds=[],
                 oocd_debug=2, oocd_args=[], host='127.0.0.1', log_level=None, log_stream_handler=None,
                 log_file_handler=None):
        super(OocdEspXtensa, self).__init__(oocd_exec=oocd_exec, oocd_scripts=oocd_scripts,
                                            oocd_cfg_files=oocd_cfg_files, oocd_cfg_cmds=oocd_cfg_cmds,
                                            oocd_debug=oocd_debug, oocd_args=oocd_args, host=host, log_level=log_level,
                                            log_stream_handler=log_stream_handler, log_file_handler=log_file_handler)
        self.cores_num = cores_num

    def perfmon_dump(self, counter = None):
        """Run OpenOCD perfmon_dump command

        Reported results are returned as a dictionary. Each key is the counter id.
        Each value is a tuple of counts for every core.
        If some CPU is disabled, its count will be None.
        """
        if self.cores_num == 1:
            # call single core version inmplementation of base class
            return {0: super(OocdEspXtensa, self).perfmon_dump(counter)}
        cmd = 'xtensa perfmon_dump'
        if counter is not None:
            cmd += ' %d' % counter
        resp = self.cmd_exec(cmd)
        # Response should have one line for every counter
        core = None
        result = {}
        lines = resp.split('\n')
        for line in lines:
            if len(line) == 0:
                continue
            tokens = re.match(r'CPU(?P<core>\d+):$', line)
            if tokens:
                core = int(tokens.group('core'))
                if core not in result:
                    result[core] = {}
            else:
                tokens = re.match(r'Counter (?P<counter>\d+): (?P<val>\d+)', line)
                val = int(tokens.group('val'))
                counter = int(tokens.group('counter'))
                result[core][counter] = val
        return result


class OocdEspRiscv(OocdRiscv, OocdEspImpl):
    """
        Class to communicate to OpenOCD supporting ESP RISCV-specific features
    """

    def __init__(self, cores_num=1, oocd_exec=None, oocd_scripts=None, oocd_cfg_files=[], oocd_cfg_cmds=[],
                 oocd_debug=2, oocd_args=[], host='127.0.0.1', log_level=None, log_stream_handler=None,
                 log_file_handler=None):
        super(OocdEspRiscv, self).__init__(oocd_exec=oocd_exec, oocd_scripts=oocd_scripts,
                                            oocd_cfg_files=oocd_cfg_files, oocd_cfg_cmds=oocd_cfg_cmds,
                                            oocd_debug=oocd_debug, oocd_args=oocd_args, host=host, log_level=log_level,
                                            log_stream_handler=log_stream_handler, log_file_handler=log_file_handler)
        self.cores_num = cores_num


class GdbEspImpl:
    """
        Class to communicate to GDB supporting ESP-specific features
    """

    def __init__(self):
        self.app_flash_offset = 0x10000  # default for for ESP xtensa chips
        self.prog_startup_cmdfile = os.path.join(DEFAULT_GDB_INIT_SCRIPT_DIR, "esp_init.gdb")

    def target_program(self, file_name, off, actions='verify', tmo=30):
        """

        actions can be any or both of 'verify reset'

        Parameters
        ----------
        file_name : str
        off : str
        actions : str
        tmo : int

        """
        self.monitor_run('program_esp %s %s 0x%x' % (fixup_path(file_name), actions, int(off)), tmo)

    def _update_memory_map(self):
        self.monitor_run('esp appimage_offset 0x%x' % self.app_flash_offset, 5)
        self.disconnect()
        self.connect()

    def exec_run(self, start_func='app_main', startup_tmo=5, only_startup=False):
        """
        Implements logic of `run` and `start` commands. Executes a startup command file in the beginning if it specified

        Parameters
        ----------
        start_func : str
            if not empty `exec_run` works like `start` stopping on the main function, otherwise - as `run`
        startup_tmo : int
            timeout for startup command file's execution
        only_startup :bool
            execute only a startup command file omitting other method's logic
        """
        if self.prog_startup_cmdfile:
            self.file_cmd_run(self.prog_startup_cmdfile, tmo=startup_tmo)
            if only_startup:
                return
        else:
            self.target_reset()
        self.wait_target_state(TARGET_STATE_STOPPED, 10)
        self._update_memory_map()
        if start_func:
            self.add_bp(start_func, tmp=True)
        self.resume()

    def gcov_dump(self, on_the_fly=True):
        if on_the_fly:
            cmd = 'esp gcov'
        else:
            cmd = 'esp gcov dump'
        self.monitor_run(cmd, tmo=30)

    def sysview_start(self, file1, file2=''):
        self.monitor_run('esp sysview start %s %s' % (file1, file2))

    def sysview_stop(self):
        self.monitor_run('esp sysview stop')

    def sysview_mcore_start(self, file):
        self.monitor_run('esp sysview_mcore start %s' % (file))

    def apptrace_start(self, trace_args):
        self.monitor_run('esp apptrace start %s' % (trace_args))

    def apptrace_stop(self):
        self.monitor_run('esp apptrace stop')


class GdbEspXtensa(GdbEspImpl, GdbXtensa):
    """
        Class to communicate to GDB supporting ESP Xtensa-specific features
    """

    def __init__(self, gdb_path, remote_target='127.0.0.1:3333', extended_remote_mode=False, gdb_log_file=None,
                 log_level=None, log_stream_handler=None, log_file_handler=None):
        GdbXtensa.__init__(self, gdb_path=gdb_path, remote_target=remote_target,
                                           extended_remote_mode=extended_remote_mode, gdb_log_file=gdb_log_file,
                                           log_level=log_level, log_stream_handler=log_stream_handler,
                                           log_file_handler=log_file_handler)
        GdbEspImpl.__init__(self)

    def get_thread_info(self, thread_id=None):
        """
            See Gdb.get_thread_info().
            ESP xtensa chips need to be halted to read memory. This method stops
        """
        self.halt()
        return super(GdbEspXtensa, self).get_thread_info(thread_id)


class GdbEspRiscv(GdbEspImpl, GdbRiscv):
    """
        Class to communicate to GDB supporting ESP RISCV-specific features
    """

    def __init__(self, gdb_path, remote_target='127.0.0.1:3333', extended_remote_mode=False, gdb_log_file=None,
                 log_level=None, log_stream_handler=None, log_file_handler=None):
        GdbRiscv.__init__(self, gdb_path=gdb_path, remote_target=remote_target,
                                           extended_remote_mode=extended_remote_mode, gdb_log_file=gdb_log_file,
                                           log_level=log_level, log_stream_handler=log_stream_handler,
                                           log_file_handler=log_file_handler)
        GdbEspImpl.__init__(self)


class OocdEsp32(OocdEspXtensa):
    """
        Class to communicate to OpenOCD supporting ESP32 specific features
    """
    chip_name = 'esp32'

    def __init__(self, oocd_exec=None, oocd_scripts=None, oocd_cfg_files=[], oocd_cfg_cmds=[], oocd_debug=2,
                 oocd_args=[], host='127.0.0.1', log_level=None, log_stream_handler=None, log_file_handler=None):
        super(OocdEsp32, self).__init__(cores_num=2, oocd_exec=oocd_exec, oocd_scripts=oocd_scripts,
                                        oocd_cfg_files=oocd_cfg_files, oocd_cfg_cmds=oocd_cfg_cmds,
                                        oocd_debug=oocd_debug,
                                        oocd_args=oocd_args, host=host, log_level=log_level,
                                        log_stream_handler=log_stream_handler,
                                        log_file_handler=log_file_handler)


class GdbEsp32(GdbEspXtensa):
    """
        Class to communicate to GDB supporting ESP32 specific features
    """
    chip_name = 'esp32'

    def __init__(self, gdb_path='xtensa-esp32-elf-gdb', remote_target='127.0.0.1:3333', extended_remote_mode=False,
                 gdb_log_file=None, log_level=None, log_stream_handler=None, log_file_handler=None):
        super(GdbEsp32, self).__init__(gdb_path=gdb_path, remote_target=remote_target,
                                       extended_remote_mode=extended_remote_mode,
                                       gdb_log_file=gdb_log_file, log_level=log_level,
                                       log_stream_handler=log_stream_handler,
                                       log_file_handler=log_file_handler)


class OocdEsp32Solo(OocdEspXtensa):
    """
        Class to communicate to OpenOCD supporting ESP32-SOLO specific features
    """
    chip_name = 'esp32-solo'


class GdbEsp32Solo(GdbEspXtensa):
    """
        Class to communicate to GDB supporting ESP32-SOLO specific features
    """
    chip_name = 'esp32-solo'

    def __init__(self, gdb_path='xtensa-esp32-elf-gdb', remote_target='127.0.0.1:3333', extended_remote_mode=False,
                 gdb_log_file=None, log_level=None, log_stream_handler=None, log_file_handler=None):
        super(GdbEsp32Solo, self).__init__(gdb_path=gdb_path, remote_target=remote_target,
                                           extended_remote_mode=extended_remote_mode,
                                           gdb_log_file=gdb_log_file, log_level=log_level,
                                           log_stream_handler=log_stream_handler,
                                           log_file_handler=log_file_handler)


class OocdEsp32s2(OocdEspXtensa):
    """
        Class to communicate to OpenOCD supporting ESP32-S2 specific features
    """
    chip_name = 'esp32s2'


class GdbEsp32s2(GdbEspXtensa):
    """
        Class to communicate to GDB supporting ESP32-S2 specific features
    """
    chip_name = 'esp32s2'

    def __init__(self, gdb_path='xtensa-esp32s2-elf-gdb', remote_target='127.0.0.1:3333', extended_remote_mode=False,
                 gdb_log_file=None, log_level=None, log_stream_handler=None, log_file_handler=None):
        super(GdbEsp32s2, self).__init__(gdb_path=gdb_path, remote_target=remote_target,
                                         extended_remote_mode=extended_remote_mode,
                                         gdb_log_file=gdb_log_file, log_level=log_level,
                                         log_stream_handler=log_stream_handler,
                                         log_file_handler=log_file_handler)


class OocdEsp32c3(OocdEspRiscv):
    """
        Class to communicate to OpenOCD supporting ESP32-C3 specific features
    """
    chip_name = 'esp32c3'


class GdbEsp32c3(GdbEspRiscv):
    """
        Class to communicate to GDB supporting ESP32-C3 specific features
    """
    chip_name = 'esp32c3'

    def __init__(self, gdb_path='riscv32-esp-elf-gdb', remote_target='127.0.0.1:3333', extended_remote_mode=False,
                 gdb_log_file=None, log_level=None, log_stream_handler=None, log_file_handler=None):
        super(GdbEsp32c3, self).__init__(gdb_path=gdb_path, remote_target=remote_target,
                                         extended_remote_mode=extended_remote_mode,
                                         gdb_log_file=gdb_log_file, log_level=log_level,
                                         log_stream_handler=log_stream_handler,
                                         log_file_handler=log_file_handler)
        # self.console_cmd_run('set arch riscv:rv32')
        self.gdb_set('arch', 'riscv:rv32')
