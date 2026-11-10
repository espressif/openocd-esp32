import hashlib
import logging
import os
import struct
import tempfile
import debug_backend as dbg
from debug_backend_tests import *


def get_logger():
    return logging.getLogger(__name__)


RISCV_TRACE_CHIPS = ['esp32c5', 'esp32c6', 'esp32c61', 'esp32h2', 'esp32h21', 'esp32h4', 'esp32p4', 'esp32s31']
RISCV_TRACE_DUAL_CORE_CHIPS = ['esp32h4', 'esp32p4', 'esp32s31']
WRAP_WAIT_S = 1

# .esptrace v1 layout, mirrors src/target/espressif/esp_riscv_trace_file.c
ESPT_MAGIC = b'ESPT'
ESPT_VERSION = 1
ESPT_HDR = struct.Struct('<4sHH')
ESPT_HDR_SIZE = 100
ESPT_OFF_SNAPSHOT = 8
RVTS_DESC_SIZE = 32
RVTS_PARAMS_SIZE = 28
ESPT_OFF_SHA256 = ESPT_OFF_SNAPSHOT + RVTS_DESC_SIZE + RVTS_PARAMS_SIZE
SHA256_SIZE = 32
REC_HDR = struct.Struct('<BBHI')
REC_CORE_DESC = 1
REC_DATA = 2
REC_END = 3
END_OK = 0

# RVTS root and core descriptor fields, mirrors esp_riscv_trace_desc.h
RVTS_OFF_CORE_COUNT = 28
RVTS_OFF_CAPTURE_REASON = 29
RVTS_CORE_DESC_SIZE = 36
RVTS_CORE_DESC = struct.Struct('<IIIIII' + 'B' * 12)
RVTS_STATE_STOPPED = 3
RVTS_STATE_FROZEN = 4
RVTS_MEMORY_MODE_LOOP = 2
RVTS_REASON_EXPLICIT_STOP = 1
RVTS_REASON_PANIC = 2


class TraceCore:
    def __init__(self, desc):
        fields = RVTS_CORE_DESC.unpack(desc)
        self.desc = desc
        self.capacity = fields[1]
        self.head_offset = fields[2]
        self.core_id, self.state, self.memory_mode = fields[6:9]
        self.head_valid, self.fifo_empty, _memory_full, self.fifo_overflow = fields[13:17]
        self.data = b''


class TraceFile:
    def __init__(self, path):
        with open(path, 'rb') as f:
            blob = f.read()
        magic, version, hdr_size = ESPT_HDR.unpack_from(blob, 0)
        assert magic == ESPT_MAGIC, '%s: bad magic %r' % (path, magic)
        assert version == ESPT_VERSION, '%s: version %d' % (path, version)
        assert hdr_size == ESPT_HDR_SIZE, '%s: header size %d' % (path, hdr_size)
        root = blob[ESPT_OFF_SNAPSHOT:ESPT_OFF_SNAPSHOT + RVTS_DESC_SIZE]
        self.core_count = root[RVTS_OFF_CORE_COUNT]
        self.capture_reason = root[RVTS_OFF_CAPTURE_REASON]
        self.sha256 = blob[ESPT_OFF_SHA256:ESPT_OFF_SHA256 + SHA256_SIZE]
        self.cores = {}
        self.end_status = None

        pos = hdr_size
        while pos < len(blob):
            rec_type, core, _flags, length = REC_HDR.unpack_from(blob, pos)
            payload = blob[pos + REC_HDR.size:pos + REC_HDR.size + length]
            assert len(payload) == length, '%s: truncated record at %d' % (path, pos)
            pos += REC_HDR.size + length
            if rec_type == REC_CORE_DESC:
                assert length == RVTS_CORE_DESC_SIZE, '%s: core descriptor size %d' % (path, length)
                self.cores[core] = TraceCore(payload)
            elif rec_type == REC_DATA:
                self.cores[core].data = payload
            elif rec_type == REC_END:
                self.end_status = struct.unpack('<I', payload)[0]


def core_mask(cores):
    return sum(1 << core for core in cores)


########################################################################
#                         TESTS IMPLEMENTATION                         #
########################################################################
class RiscvTraceTestsImpl:
    """ Tests for OpenOCD control of the RISC-V trace encoder ("esp riscv_trace" commands).
    """
    def setUp(self):
        # Kept after the test so CI can upload the captures. Each run, including a retry, gets its own directory.
        trace_root = os.path.join(test_apps_dir, 'riscv_trace')
        os.makedirs(trace_root, exist_ok=True)
        self.trace_dir = tempfile.mkdtemp(prefix='%s.%s_' % (type(self).__name__, self._testMethodName),
                                          dir=trace_root)

    def tearDown(self):
        # Ends a capture that a failed test left running
        self.oocd.cmd_exec('esp riscv_trace stop')

    def trace_path(self, name):
        return os.path.join(self.trace_dir, name)

    def trace_cmd(self, cmd, expect=None):
        out = self.oocd.cmd_exec('esp riscv_trace %s' % cmd)
        get_logger().debug('riscv_trace %s:\n%s', cmd, out)
        if expect is not None:
            self.assertIn(expect, out)
        return out

    def check_trace_file(self, path, state, reason):
        trace = TraceFile(path)
        with open(self.test_app_cfg.build_app_elf_path(), 'rb') as elf:
            self.assertEqual(trace.sha256, hashlib.sha256(elf.read()).digest())
        self.assertEqual(trace.capture_reason, reason)
        self.assertEqual(trace.end_status, END_OK)
        self.assertEqual(sorted(trace.cores), list(range(self.CORES_NUM)))
        for core_id in range(self.CORES_NUM):
            core = trace.cores[core_id]
            self.assertEqual(core.core_id, core_id)
            self.assertEqual(core.state, state)
            self.assertEqual(core.memory_mode, RVTS_MEMORY_MODE_LOOP)
            self.assertTrue(core.head_valid)
            self.assertTrue(core.fifo_empty)
            self.assertEqual(len(core.data), core.capacity)
            # Start clears the buffer, so data after the head means the loop buffer wrapped
            wrapped = core.head_offset == core.capacity or any(core.data[core.head_offset:])
            self.assertTrue(wrapped, 'core %d buffer did not wrap' % core_id)
        return trace

    def test_riscv_trace_loop_start_stop(self):
        """
            This test checks OpenOCD loop-mode capture of the RISC-V trace encoder
            (esp riscv_trace start/stop/dump/info).
            1) Resume the target so the firmware can run the branchy workload.
            2) Run 'esp riscv_trace info' and check that both descriptors are present
               and that start/stop is available.
            3) Check that start refuses stream mode and cores the firmware does not trace.
            4) Start a loop capture and check that info reports looping on every traced core.
            5) Check that a second start and a dump are refused while the capture runs.
            6) Wait for the loop buffer to wrap, stop the capture and check the .esptrace file
               (ELF SHA, reason, end status, per-core state, wrap and fifo_empty).
            7) Dump the same capture after the stop and check that the file matches the stop output.
            8) Check that stop with nothing running reports "Nothing to stop", and that refused
               commands did not create a file.
        """
        mask = core_mask(range(self.CORES_NUM))
        stop_file = self.trace_path('stop.esptrace')
        dump_file = self.trace_path('dump.esptrace')
        unused_file = self.trace_path('unused.esptrace')
        self.resume_exec()

        # Firmware reports both descriptors and allows OpenOCD control
        info = self.trace_cmd('info', '(RVOC) ABI 1')
        self.assertIn('(RVTS) ABI 1.0', info)
        self.assertIn('start/stop available', info)

        # Start refuses stream mode and cores the firmware does not trace
        self.trace_cmd('start stream %s' % unused_file, 'Stream mode is not supported yet')
        untraced_mask = mask | (1 << self.CORES_NUM)
        self.trace_cmd('start loop %s 0 0x%x' % (unused_file, untraced_mask), 'cores that are not traced')

        # Loop capture runs on every traced core
        self.trace_cmd('start loop %s' % stop_file, 'loop capture started on cores 0x%x' % mask)
        info = self.trace_cmd('info', 'OpenOCD capture: loop')
        self.assertEqual(info.count('state looping'), self.CORES_NUM)

        # A second start and a dump are refused while the capture runs
        self.trace_cmd('start loop %s' % unused_file, 'already running')
        self.trace_cmd('dump %s' % unused_file, 'Cannot dump: an OpenOCD capture to')

        # Stop writes the wrapped buffer of every core and ends the capture
        self.alive_sleep(WRAP_WAIT_S)
        self.trace_cmd('stop')
        stopped = self.check_trace_file(stop_file, RVTS_STATE_STOPPED, RVTS_REASON_EXPLICIT_STOP)
        self.assertNotIn('OpenOCD capture', self.trace_cmd('info'))

        # Dump after the stop reads the same capture
        self.trace_cmd('dump %s' % dump_file)
        dumped = self.check_trace_file(dump_file, RVTS_STATE_STOPPED, RVTS_REASON_EXPLICIT_STOP)
        for core_id in range(self.CORES_NUM):
            self.assertEqual(dumped.cores[core_id].desc, stopped.cores[core_id].desc)
            self.assertEqual(dumped.cores[core_id].data, stopped.cores[core_id].data)

        # Stop without a capture does nothing, and no refused command created a file
        self.trace_cmd('stop', 'Nothing to stop')
        self.assertFalse(os.path.exists(unused_file))


########################################################################
#              TESTS DEFINITION WITH SPECIAL TESTS                     #
########################################################################

class RiscvTraceTestAppTestsSingle(DebuggerGenericTestAppTests):
    """ Base class to run tests which use the RISC-V trace test app in single core mode
    """
    CORES_NUM = 1

    def __init__(self, methodName='runTest'):
        super(RiscvTraceTestAppTestsSingle, self).__init__(methodName)
        self.test_app_cfg.bin_dir = os.path.join('output', 'riscv_trace_single')
        self.test_app_cfg.build_dir = os.path.join('builds', 'riscv_trace_single')


class RiscvTraceTestAppTestsDual(DebuggerGenericTestAppTests):
    """ Base class to run tests which use the RISC-V trace test app in dual core mode
    """
    CORES_NUM = 2

    def __init__(self, methodName='runTest'):
        super(RiscvTraceTestAppTestsDual, self).__init__(methodName)
        self.test_app_cfg.bin_dir = os.path.join('output', 'riscv_trace_dual')
        self.test_app_cfg.build_dir = os.path.join('builds', 'riscv_trace_dual')


@idf_ver_min('latest')
@only_for_chip(RISCV_TRACE_CHIPS)
class RiscvTraceTestsSingle(RiscvTraceTestAppTestsSingle, RiscvTraceTestsImpl):
    """ Test cases in single core mode
    """

    def setUp(self):
        RiscvTraceTestAppTestsSingle.setUp(self)
        RiscvTraceTestsImpl.setUp(self)

    def tearDown(self):
        # Stops a leftover capture while the target still runs, before the base class halts it
        RiscvTraceTestsImpl.tearDown(self)
        RiscvTraceTestAppTestsSingle.tearDown(self)


@idf_ver_min('latest')
@only_for_chip(RISCV_TRACE_DUAL_CORE_CHIPS)
class RiscvTraceTestsDual(RiscvTraceTestAppTestsDual, RiscvTraceTestsImpl):
    """ Test cases in dual core mode
    """

    def setUp(self):
        RiscvTraceTestAppTestsDual.setUp(self)
        RiscvTraceTestsImpl.setUp(self)

    def tearDown(self):
        # Stops a leftover capture while the target still runs, before the base class halts it
        RiscvTraceTestsImpl.tearDown(self)
        RiscvTraceTestAppTestsDual.tearDown(self)
