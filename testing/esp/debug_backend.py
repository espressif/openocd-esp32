import logging
import subprocess
import signal
import threading
import time
from pygdbmi.gdbcontroller import GdbController
from pygdbmi.gdbcontroller import GdbTimeoutError
from pprint import pformat


_oocd_inst   = None
_gdb_inst    = None

def start(gdb_path, oocd_path, oocd_tcl_dir, oocd_cfg_files):
    global _oocd_inst
    global _gdb_inst
    oocd_args = ['-s', oocd_tcl_dir]
    for f in oocd_cfg_files:
        oocd_args += ['-f', f]
    _oocd_inst = Oocd(oocd_path, oocd_args)
    _oocd_inst.start()
    try:
        _gdb_inst = Gdb(gdb_path)
        _gdb_inst.target_select('remote', ':3333')
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
        self._oocd_proc = subprocess.Popen(
                bufsize = 0, args = [oocd_path] + oocd_args,
                stdin = None, stdout = subprocess.PIPE, stderr = subprocess.STDOUT
                )

    def run(self):
        while True:
            ln = self._oocd_proc.stdout.readline()
            if len(ln) == 0:
                break            
            self._logger.debug(ln.rstrip(' \r\n'))

    def stop(self):
        self._oocd_proc.send_signal(signal.SIGINT)

    def join(self):
        super(Oocd, self).join()
        self._oocd_proc.stdout.close()


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

    def _on_notify(self, rec):
        if rec['message'] == 'stopped':
            self._target_state = self.TARGET_STATE_STOPPED
            self._curr_frame = rec['payload']['frame'] 
            if 'reason' in rec['payload']:
                if rec['payload']['reason'] == 'breakpoint-hit':
                    self._target_stop_reason = self.TARGET_STOP_REASON_BP
                elif rec['payload']['reason'] == 'watchpoint-trigger':
                    self._target_stop_reason = self.TARGET_STOP_REASON_WP
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
                    # self._logger.debug('new target state %d', self._target_state)
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
                if tmo and time.time() >= end:
                    raise DebuggerTargetStateTimeoutError('Failed to wait for completion of command "%s"!' % cmd)
            else:
                self._logger.debug('MI<-:\n%s', pformat(response))
            res,res_body = self._parse_mi_resp(response, new_tgt_state)
        return res,res_body

    def target_select(self, tgt_type, tgt_params):
        # -target-select type parameters
        res,_ = self._mi_cmd_run('-target-select %s %s' % (tgt_type, tgt_params))
        if res != 'connected':
            raise DebuggerError('Failed to connect to "%s %s"!' % (tgt_type, tgt_params))

    def target_reset(self, action='halt'):
        self.monitor_run('reset %s' % action)
        if action == 'halt':
            self.wait_target_state(self.TARGET_STATE_STOPPED, 5)

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

    def data_eval_expr(self, expr):
        # -data-evaluate-expression expr
        res,res_body = self._mi_cmd_run('-data-evaluate-expression %s' % expr)
        if res != 'done' or not res_body:
            raise DebuggerError('Failed to get backtrace!')
        return res_body['value']

    def get_backtrace(self):
        # -stack-list-frames [ --no-frame-filters low-frame high-frame ]
        res,res_body = self._mi_cmd_run('-stack-list-frames')
        if res != 'done' or not res_body:
            raise DebuggerError('Failed to get backtrace!')
        return res_body['stack']

    def add_bp(self, loc):
        # -break-insert [ -t ] [ -h ] [ -f ] [ -d ] [ -a ] [ -c condition ] [ -i ignore-count ] [ -p thread-id ] [ location ]
        res,res_body = self._mi_cmd_run('-break-insert %s' % loc)
        if res != 'done' or not res_body or 'bkpt' not in res_body or 'number' not in res_body['bkpt']:
            raise DebuggerError('Failed to insert BP!')
        return res_body['bkpt']['number']

    def delete_bp(self, bp):
        # -break-delete ( breakpoint )+
        res,_ = self._mi_cmd_run('-break-delete %s' % bp)
        if res != 'done':
            raise DebuggerError('Failed to delete BP!')

    def monitor_run(self, cmd, tmo = None):
        res,_ = self._mi_cmd_run('mon %s' % cmd, tmo)
        if res != 'done':
            raise DebuggerError('Failed to run monitor cmd "%s"!' % cmd)

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
