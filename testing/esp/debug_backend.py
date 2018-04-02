import logging
import subprocess
import signal
import threading
import time
import telnetlib
import re
from pygdbmi.gdbcontroller import GdbController
from pygdbmi.gdbcontroller import GdbTimeoutError
from pprint import pformat

OOCD_PORT = 3333
toolchain = 'none'
_oocd_inst   = None
_gdb_inst    = None

def start(toolch, oocd_path, oocd_tcl_dir, oocd_cfg_files):
    global _oocd_inst
    global _gdb_inst
    global toolchain
    toolchain = toolch
    oocd_args = ['-s', oocd_tcl_dir]
    for f in oocd_cfg_files:
        oocd_args += ['-f', f]
    # oocd_args += ['-d', '3']
    _oocd_inst = Oocd(oocd_path, oocd_args)
    _oocd_inst.start()
    try:
        _gdb_inst = Gdb('%sgdb' % toolchain)
        _gdb_inst.connect()
    except Exception as e:
        _oocd_inst.stop()
        _oocd_inst.join()
        raise e

def stop():
    # _gdb_inst.disconnect()
    _oocd_inst.stop()
    _oocd_inst.join()

def get_gdb():
    return _gdb_inst

def get_oocd():
    return _oocd_inst

class DebuggerError(RuntimeError):
    pass


class DebuggerTargetStateTimeoutError(DebuggerError):
    pass


class Oocd(threading.Thread):
    _oocd_proc = None
    _logger = None

    @staticmethod
    def get_logger():
        return logging.getLogger('Oocd')

    def __init__(self, oocd_path = 'openocd', oocd_args=[]):
        super(Oocd, self).__init__()
        self._logger = self.get_logger()
        self._logger.debug('Start OpenOCD')
        self._oocd_proc = subprocess.Popen(
                bufsize = 0, args = [oocd_path] + oocd_args,
                stdin = None, stdout = subprocess.PIPE, stderr = subprocess.STDOUT
                )
        time.sleep(1)
        self._logger.debug('Open telnet conn...')
        try:
            self._tn = telnetlib.Telnet('localhost', 4444, 5)
            self._tn.read_until('>', 5)
        except Exception as e:
            self._logger.error('Failed to open telnet connection!')
            self._oocd_proc.send_signal(signal.SIGINT)
            raise e

    def run(self):
        while True:
            ln = self._oocd_proc.stdout.readline()
            if len(ln) == 0:
                break            
            self._logger.debug(ln.rstrip(' \r\n'))

    def stop(self):
        self._logger.debug('Close telnet conn')
        self._tn.close()
        self._logger.debug('Stop OpenOCD')
        self._oocd_proc.send_signal(signal.SIGINT)

    def join(self):
        super(Oocd, self).join()
        self._oocd_proc.stdout.close()

    def cmd_exec(self, cmd):
        # read all output already sent
        resp = self._tn.read_very_eager()
        self._logger.debug('TELNET <-: %s' % resp)
        self._logger.debug('TELNET ->: %s' % cmd)
        cmd_sent = cmd + '\n'
        self._tn.write(cmd_sent)
        resp = self._tn.read_until('>')
        # remove all '\r' first
        resp = resp.replace('\r','')
        # command we sent will be echoed back - remove it
        index_start = resp.find(cmd_sent)
        if index_start >= 0:
            resp = resp[index_start+len(cmd_sent):]
        # the response will also include '>', next prompt - remove it as well
        index_end = resp.rfind('>')
        if index_end >= 0:
            resp = resp[:index_end]
        self._logger.debug('TELNET <-: %s' % resp)
        return resp

    def perfmon_enable(self, counter, select, mask = None, kernelcnt = None, tracelevel = None):
        """Run OpenOCD perfmon_enable command, which starts performance counter

        counter: performance counter ID
        select, mask: determine the event being profiled, refer to Xtensa Debug Guide
        kernelcnt: 0 - count events with CINTLEVEL <= tracelevel,
                   1 - count events with CINTLEVEL > tracelevel

        If mask, kernelcnt, tracelevel are not specified, OpenOCD will use default values.
        """
        cmd = 'esp32 perfmon_enable %d %d' % (counter, select)
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
        Each value is a tuple of counts for PRO and APP CPUs.
        If APP CPU is disabled, its count will be None.
        """
        cmd = 'esp32 perfmon_dump'
        if counter is not None:
            cmd += '%d' % counter
        resp = self.cmd_exec(cmd)
        # Response should have one line for every counter
        lines = resp.split('\n')
        result = {}
        for line in lines:
            if len(line) == 0:
                continue
            tokens = re.match(r'Counter (?P<counter>\d+): CPU0: (?P<count0>\d+)(\s* CPU1: (?P<count1>\d+))?', line)
            count0 = int(tokens.group('count0'))
            count1 = int(tokens.group('count1')) if tokens.group('count1') is not None else None
            counter = int(tokens.group('counter'))
            result[counter] = (count0, count1)
        return result


class Gdb:
    # Target states
    TARGET_STATE_UNKNOWN = 0
    TARGET_STATE_STOPPED = 1
    TARGET_STATE_RUNNING = 2
    # Target stop reasons
    TARGET_STOP_REASON_UNKNOWN  = 0
    TARGET_STOP_REASON_SIGINT   = 1
    TARGET_STOP_REASON_SIGTRAP  = 2
    TARGET_STOP_REASON_BP       = 3
    TARGET_STOP_REASON_WP       = 4
    TARGET_STOP_REASON_WP_SCOPE = 5
    TARGET_STOP_REASON_STEPPED  = 6
    TARGET_STOP_REASON_FN_FINISHED = 7

    @staticmethod
    def get_logger():
        return logging.getLogger('Gdb')

    def __init__(self, gdb = None):
        # Start gdb process
        self._logger = self.get_logger()
        self._gdbmi = GdbController(gdb_path=gdb)
        self._resp_cache = []
        self._target_state = self.TARGET_STATE_UNKNOWN
        self._target_stop_reason = self.TARGET_STOP_REASON_UNKNOWN
        self._curr_frame = None
        self._curr_wp_val = None

    def _on_notify(self, rec):
        if rec['message'] == 'stopped':
            self._target_state = self.TARGET_STATE_STOPPED
            self._curr_frame = rec['payload']['frame'] 
            if 'reason' in rec['payload']:
                if rec['payload']['reason'] == 'breakpoint-hit':
                    self._target_stop_reason = self.TARGET_STOP_REASON_BP
                elif rec['payload']['reason'] == 'watchpoint-trigger':
                    self._target_stop_reason = self.TARGET_STOP_REASON_WP
                    self._curr_wp_val = rec['payload']['value']
                elif rec['payload']['reason'] == 'watchpoint-scope':
                    self._target_stop_reason = self.TARGET_STOP_REASON_WP_SCOPE
                elif rec['payload']['reason'] == 'end-stepping-range':
                    self._target_stop_reason = self.TARGET_STOP_REASON_STEPPED
                elif rec['payload']['reason'] == 'function-finished':
                    self._target_stop_reason = self.TARGET_STOP_REASON_FN_FINISHED
                elif rec['payload']['reason'] == 'signal-received':
                    if rec['payload']['signal-name'] == 'SIGINT':
                        self._target_stop_reason = self.TARGET_STOP_REASON_SIGINT
                    elif rec['payload']['signal-name'] == 'SIGTRAP':
                        self._target_stop_reason = self.TARGET_STOP_REASON_SIGTRAP
                    else:
                        self._logger.warning('Unknown signal received "%s"!', rec['payload']['signal-name'])
                        self._target_stop_reason = self.TARGET_STOP_REASON_UNKNOWN
                else:
                    self._logger.warning('Unknown target stop reason "%s"!', rec['payload']['reason'])
                    self._target_stop_reason = self.TARGET_STOP_REASON_UNKNOWN
            else:
                self._target_stop_reason = self.TARGET_STOP_REASON_UNKNOWN
        elif rec['message'] == 'running':
            self._target_state = self.TARGET_STATE_RUNNING

    def _parse_mi_resp(self, new_resp, new_tgt_state):
        result = None
        result_body = None
        old_state = self._target_state
        # if any cached records go first
        resp = self._resp_cache + new_resp
        processed_recs = 0
        for rec in resp:
            processed_recs += 1
            if rec['type'] == 'log':
                self._logger.debug('LOG: %s', pformat(rec['payload']))
            elif rec['type'] == 'console':
                self._logger.info('CONS: %s', pformat(rec['payload']))
            elif rec['type'] == 'notify':
                self._logger.info('NOTIFY: %s %s', rec['message'], pformat(rec['payload']))
                self._on_notify(rec)
                # stop upon result receiption if we do not expect target state change
                if self._target_state != old_state and self._target_state == new_tgt_state:
                    self._logger.debug('new target state %d', self._target_state)
                    break
            elif rec['type'] == 'result':
                self._logger.debug('RESULT: %s %s', rec['message'], pformat(rec['payload']))
                result = rec['message']
                result_body = rec['payload']
                # stop upon result receiption if we do not expect target state change
                if not new_tgt_state:
                    break
        # cache unprocessed records
        self._resp_cache = resp[processed_recs:]
        # self._logger.debug('cached recs: %s', pformat(self._resp_cache))
        return result,result_body

    def _mi_cmd_run(self, cmd, new_tgt_state=None, tmo=5):
        self._logger.debug('MI->: %s', cmd)
        response = []
        if tmo:
            end = time.time() + tmo
            try:
                response = self._gdbmi.write(cmd, timeout_sec=tmo)
            except:
                self._gdbmi.verify_valid_gdb_subprocess()
        else:
            while len(response) == 0:
                response = self._gdbmi.write(cmd, raise_error_on_timeout=False)
        self._logger.debug('MI<-:\n%s', pformat(response))
        res,res_body = self._parse_mi_resp(response, new_tgt_state)
        while not res:
            # check for result report from GDB
            response = self._gdbmi.get_gdb_response(1, raise_error_on_timeout = False)
            if len(response) == 0:
                if tmo and (time.time() >= end):
                    raise DebuggerTargetStateTimeoutError('Failed to wait for completion of command "%s" / %s!' % (cmd, tmo))
            else:
                self._logger.debug('MI<-:\n%s', pformat(response))
                res,res_body = self._parse_mi_resp(response, new_tgt_state)
        return res,res_body

    def console_cmd_run(self, cmd):
        self._mi_cmd_run('-interpreter-exec console %s' % cmd)

    def target_select(self, tgt_type, tgt_params):
        # -target-select type parameters
        res,_ = self._mi_cmd_run('-target-select %s %s' % (tgt_type, tgt_params))
        if res != 'connected':
            raise DebuggerError('Failed to connect to "%s %s"!' % (tgt_type, tgt_params))

    def target_disconnect(self):
        # -target-disconnect
        self._mi_cmd_run('-target-disconnect')

    def target_reset(self, action='halt'):
        self.monitor_run('reset %s' % action)
        if action == 'halt':
            self.wait_target_state(self.TARGET_STATE_STOPPED, 5)
            self.console_cmd_run('flushregs')

    def target_download(self):
        raise NotImplementedError('target_download')

    def target_program(self, file_name, off, actions='verify'):
        # actions can be any or both of 'verify reset'
        self.monitor_run('program_esp32 %s %s 0x%x' % (file_name, actions, off), 30)

    def exec_file_set(self, file_path):
        # -file-exec-and-symbols file
        res,_ = self._mi_cmd_run('-file-exec-and-symbols %s' % file_path)
        if res != 'done':
            raise DebuggerError('Failed to set program file!')

    def exec_interrupt(self):
        # Hack, unfortunately GDB does not react on -exec-interrupt,
        # so send CTRL+C to it
        self._logger.debug('MI->: send SIGINT')
        self._gdbmi.gdb_process.send_signal(signal.SIGINT)
        # # -exec-interrupt [--all|--thread-group N]
        # res,_ = self._mi_cmd_run('-exec-interrupt --all')
        # if res != 'done':
        #     raise DebuggerError('Failed to stop program!')

    def exec_continue(self):
        # -exec-continue [--reverse] [--all|--thread-group N]
        res,_ = self._mi_cmd_run('-exec-continue --all')
        if res != 'running':
            raise DebuggerError('Failed to continue program!')

    def exec_jump(self, loc):
        # -exec-jump location
        res,_ = self._mi_cmd_run('-exec-jump %s' % loc)
        if res != 'running':
            raise DebuggerError('Failed to make jump in program!')

    def exec_next(self):
        # -exec-next [--reverse]
        res,_ = self._mi_cmd_run('-exec-next')
        if res != 'running':
            raise DebuggerError('Failed to step over!')

    def exec_step(self):
        # -exec-step [--reverse]
        res,_ = self._mi_cmd_run('-exec-step')
        if res != 'running':
            raise DebuggerError('Failed to step in!')

    def exec_finish(self):
        # -exec-finish [--reverse]
        res,_ = self._mi_cmd_run('-exec-finish')
        if res != 'running':
            raise DebuggerError('Failed to step out!')

    def exec_next_insn(self):
        # -exec-next-instruction [--reverse]
        res,_ = self._mi_cmd_run('-exec-next-instruction')
        if res != 'running':
            raise DebuggerError('Failed to step insn!')

    def get_thread_info(self, thread_id=None):
        # -thread-info [ thread-id ]
        if thread_id:
            cmd = '-thread-info %d' % thread_id
        else:
            cmd = '-thread-info'    
        res,res_body = self._mi_cmd_run(cmd)
        if res != 'done' or not res_body or 'threads' not in res_body or 'current-thread-id' not in res_body:
            raise DebuggerError('Failed to get thread info!')
        return (res_body['current-thread-id'], res_body['threads'])

    def data_eval_expr(self, expr):
        # -data-evaluate-expression expr
        res,res_body = self._mi_cmd_run('-data-evaluate-expression %s' % expr)
        if res != 'done' or not res_body or 'value' not in res_body:
            raise DebuggerError('Failed to eval expression!')
        return res_body['value']

    def get_reg(self, nm):
        sval = self.data_eval_expr('$%s' % nm)
        # for PC we get something like '0x400e0db8 <gpio_set_direction>'
        # TODO: use regexp to extract value
        fn_start = sval.find(' <')
        if fn_start != -1:
            sval = sval[:fn_start]
        return int(sval, 0)

    def get_variables_at_frame(self, thread_num=None, frame_num = 0):
        # -stack-list-variables [ --no-frame-filters ] [ --skip-unavailable ] print-values
        if thread_num:
            cmd = '-stack-list-variables --thread %d --frame %d --all-values' % (thread_num, frame_num)
        else:
            cmd = '-stack-list-variables --all-values'
        res,res_body = self._mi_cmd_run(cmd)
        if res != 'done' or not res_body or 'variables' not in res_body:
            raise DebuggerError('Failed to get variables @ frame %d of thread %d!' % (frame_num, thread_num))
        return res_body['variables']

    def get_backtrace(self):
        # -stack-list-frames [ --no-frame-filters low-frame high-frame ]
        res,res_body = self._mi_cmd_run('-stack-list-frames')
        if res != 'done' or not res_body or 'stack' not in res_body:
            raise DebuggerError('Failed to get backtrace!')
        return res_body['stack']

    def select_frame(self, frame):
        # -stack-select-frame framenum
        res,_ = self._mi_cmd_run('-stack-select-frame %d' % frame)
        if res != 'done':
            raise DebuggerError('Failed to get backtrace!')

    def add_bp(self, loc, ignore_count=0, cond=''):
        # -break-insert [ -t ] [ -h ] [ -f ] [ -d ] [ -a ] [ -c condition ] [ -i ignore-count ] [ -p thread-id ] [ location ]
        cmd_args = '-i %d %s' % (ignore_count, loc)
        if len(cond):
            cmd_args = '-c "%s" %s' % (cond, cmd_args)
        res,res_body = self._mi_cmd_run('-break-insert %s' % cmd_args)
        if res != 'done' or not res_body or 'bkpt' not in res_body or 'number' not in res_body['bkpt']:
            raise DebuggerError('Failed to insert BP!')
        return res_body['bkpt']['number']

    def add_wp(self, exp, tp='w'):
        # -break-watch [ -a | -r ] expr
        cmd_args = '"%s"' % exp
        if tp == 'r':
            cmd_args = '-r %s' % cmd_args
        elif tp == 'rw':
            cmd_args = '-a %s' % cmd_args
        res,res_body = self._mi_cmd_run('-break-watch %s' % cmd_args)
        if res != 'done' or not res_body:
            raise DebuggerError('Failed to insert WP!')
        if tp == 'w':
            if 'wpt' not in res_body or 'number' not in res_body['wpt']:
                raise DebuggerError('Failed to insert WP!')
            return res_body['wpt']['number']
        elif tp == 'r':
            if 'hw-rwpt' not in res_body or 'number' not in res_body['hw-rwpt']:
                raise DebuggerError('Failed to insert RWP!')
            return res_body['hw-rwpt']['number']
        elif tp == 'rw':
            if 'hw-awpt' not in res_body or 'number' not in res_body['hw-awpt']:
                raise DebuggerError('Failed to insert AWP!')
            return res_body['hw-awpt']['number']
        return None

    def delete_bp(self, bp):
        # -break-delete ( breakpoint )+
        res,_ = self._mi_cmd_run('-break-delete %s' % bp)
        if res != 'done':
            raise DebuggerError('Failed to delete BP!')

    def monitor_run(self, cmd, tmo = None):
        res, resp = self._mi_cmd_run('mon %s' % cmd, tmo=tmo)
        if res != 'done':
            raise DebuggerError('Failed to run monitor cmd "%s"!' % cmd)
        return resp

    def wait_target_state(self, state, tmo = None):
        if tmo:
            end = time.time() + tmo
        while self._target_state != state:
            recs = []
            if len(self._resp_cache):
                recs = self._resp_cache
            else :
                # check for target state change report from GDB
                recs = self._gdbmi.get_gdb_response(1, raise_error_on_timeout = False)
                if tmo and len(recs) == 0 and time.time() >= end:
                    raise DebuggerTargetStateTimeoutError("Failed to wait for target state %d!" % state)
            self._parse_mi_resp(recs, state)
        return self._target_stop_reason

    def get_target_state(self):
        return self._target_state, self._target_stop_reason

    def get_current_frame(self):
        return self._curr_frame

    def get_current_wp_val(self):
        return self._curr_wp_val

    def connect(self):
        global OOCD_PORT
        self.target_select('remote', ':%d' % OOCD_PORT)

    def disconnect(self):
        self.target_disconnect()
