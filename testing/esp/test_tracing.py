import logging
import unittest
import debug_backend as dbg
from debug_backend_tests import *
import os
import os.path
import re
import time
import tempfile
import sys
import traceback

idf_path = os.getenv('IDF_PATH')
if idf_path:
    sys.path.append(os.path.join(idf_path, 'tools', 'esp_app_trace'))

import espytrace.apptrace as apptrace
import espytrace.sysview as sysview


def get_logger():
    return logging.getLogger(__name__)


def _create_file_reader():
    fhnd,fname = tempfile.mkstemp()
    # Convert filepath from Windows format if needed
    local_file_path = fname;
    local_file_path = local_file_path.replace("\\","/");
    trace_src = 'file://%s' % local_file_path
    reader = apptrace.reader_create(trace_src, 0)
    os.close(fhnd) # not needed, open by reader
    return trace_src,reader


########################################################################
#                         TESTS IMPLEMENTATION                         #
########################################################################

class BaseTracingTestsImpl:
    """ Test cases which are common for dual and single core modes
    """
    allow_lost_events = False

    def setUp(self):
        self.tasks_test_data = {}

    def tearDown(self):
        pass

    def _test_trace_from_file(self, test_func):
        trace_src,self.reader = _create_file_reader()
        if not self.reader:
            self.fail("Failed to create trace reader!")
        if IdfVersion.get_current() != IdfVersion.fromstr('latest'):
            # old style trace source URL
            trace_src = trace_src[len('file://'):]
        test_func(trace_src)

    def _start_tracing(self, trace_src):
        pass

    def _stop_tracing(self):
        pass

    def _create_processor(self, **proc_args):
        pass

    def _process_trace(self):
        pass

    def _get_log_stream(self):
        return None

    def _get_heap_stream(self):
        return None

    def _check_log_stream(self, log_stream, msgs_per_task, allow_lost_events):
        if not allow_lost_events:
            self.assertEqual(len(log_stream.messages), msgs_per_task*self.test_tasks_num)
        # parse and check log messages
        for msg in log_stream.messages:
            rexp = '\[[0-9]+\.[0-9]{9}\] LOG: task\[(0x[0-9a-fA-F]+)\]: Sample print (0x[0-9a-fA-F]+) (0x[0-9a-fA-F]+) (0x[0-9a-fA-F]+) ([0-9]+) ([0-9]+) ([0-9]+) ([0-9]+) ([0-9]+) ([0-9]+) ([A-Z])'
            m = re.match(rexp, msg)
            self.assertNotEqual(m, None)
            curr_task = m.group(1)
            # check that printed task handle is valid
            self.assertTrue(curr_task in self.tasks_test_data)
            if not allow_lost_events:
                for i in range(2, 10):
                    self.assertEqual(self.tasks_test_data[curr_task]['print_num'], int(m.group(i), 0))
                ch = ord('A') + (self.tasks_test_data[curr_task]['print_num']  % (ord('Z') - ord('A')))
                self.assertEqual(str(unichr(ch)), m.group(11))
                self.tasks_test_data[curr_task]['print_num'] += 1

    def _do_test_log_continuous(self, trace_src):
        """
            This test checks that continuous log tracing works.
            This common test method is used for checking log tracing via files, TCP and UDP sockets.
            1) Select appropriate sub-test number on target.
            2) Set breakpoints to execute OpenOCD commands to start and stop tracing.
            3) Set breakpoint to retrieve logging task handle and initial value for logged variable.
            4) Resume target, wait for the first breakpoint to hit and start tracing.
            5) Resume target, wait for the second breakpoint to hit and retrieve logging task handle and initial value for logged variable.
            6) Resume target, wait for the third breakpoint to hit and stop tracing.
            7) Check collected log messages.
        """
        self.select_sub_test(501)
        self.add_bp('_trace_test_log_continuous_start')
        self.add_bp('_do_trace_test_log_continuous_end')
        self.add_bp('_trace_test_log_continuous_stop')
        self.run_to_bp(dbg.Gdb.TARGET_STOP_REASON_BP, 'trace_test_log_continuous_main')
        self._start_tracing(trace_src)
        for k in range(self.test_tasks_num):
            self.run_to_bp(dbg.Gdb.TARGET_STOP_REASON_BP, 'do_trace_test_log_continuous')
            curr_task = self.gdb.data_eval_expr('curr_task')
            self.tasks_test_data[curr_task] = {'print_num': 0}
            self.gdb.select_frame(1)
            self.tasks_test_data[curr_task]['print_num'] = int(self.gdb.data_eval_expr('num'), 0)
        self.run_to_bp(dbg.Gdb.TARGET_STOP_REASON_BP, '_trace_test_log_continuous_stop')
        self._stop_tracing()

        get_logger().info("Process trace from '%s'..." % trace_src)
        elf_file = self.test_app_cfg.build_app_elf_path()
        self._create_processor(toolchain=dbg.toolchain, elf_file=elf_file)
        try:
            self._process_trace();
        except (apptrace.ReaderTimeoutError) as e:
            get_logger().info("Stop processing trace. (%s)" % e)
        except Exception as e:
            traceback.print_exc()
            self.fail("Failed to parse trace (%s)!" % e)
        get_logger().info("Processing completed.")
        # check trace
        self._check_log_stream(self._get_log_stream(), 400, self.allow_lost_events)

    def _do_test_heap_log(self, trace_src):
        """
            This test checks that log and/or heap tracing works.
            This common test method is used for checking tracing via files, TCP and UDP sockets.
            1) Select appropriate sub-test number on target.
            2) Set breakpoints to execute OpenOCD commands to start and stop tracing.
            3) Set breakpoint to retrieve tracing task handle, initial value for logged variable and line numbers for heap API calls.
            4) Resume target, wait for the first breakpoint to hit and start tracing.
            5) Resume target, wait for the second breakpoint to hit and tracing task handle, initial value for logged variable and line numbers for heap API calls.
            6) Resume target, wait for the third breakpoint to hit and stop tracing.
            7) Check collected log messages and heap API calls.
        """
        self.select_sub_test(500)
        self.add_bp('heap_trace_start')
        self.add_bp('heap_trace_stop')
        self.add_bp('_do_trace_test_heap_log_end')
        self.run_to_bp(dbg.Gdb.TARGET_STOP_REASON_BP, 'heap_trace_start')
        self._start_tracing(trace_src)
        for k in range(self.test_tasks_num):
            self.run_to_bp(dbg.Gdb.TARGET_STOP_REASON_BP, 'do_trace_test_heap_log')
            curr_task = self.gdb.data_eval_expr('curr_task')
            self.tasks_test_data[curr_task] = {'print_num': 0, 'leaks': []}
            self.gdb.select_frame(1)
            self.tasks_test_data[curr_task]['print_num'] = int(self.gdb.data_eval_expr('num'), 0)
            # get line number of the outmost caller
            outmost_tasks_test_data = self.gdb.data_eval_expr('trace_test_func_call_break_ln')
            self.gdb.select_frame(0)
            self.tasks_test_data[curr_task]['leaks'].append({'sz': 96, 'callers': [self.gdb.data_eval_expr('malloc1_break_ln'), outmost_tasks_test_data]})
            self.tasks_test_data[curr_task]['leaks'].append({'sz': 10, 'callers': [self.gdb.data_eval_expr('malloc2_break_ln'), outmost_tasks_test_data]})
        self.run_to_bp(dbg.Gdb.TARGET_STOP_REASON_BP, 'heap_trace_stop')
        self.step_out(tmo=20)
        self._stop_tracing()

        get_logger().info("Process trace from '%s'..." % trace_src)
        elf_file = self.test_app_cfg.build_app_elf_path()
        self._create_processor(toolchain=dbg.toolchain, elf_file=elf_file)
        try:
            self._process_trace();
        except (apptrace.ReaderTimeoutError) as e:
            get_logger().info("Stop processing trace. (%s)" % e)
        except Exception as e:
            traceback.print_exc()
            self.fail("Failed to parse trace (%s)!" % e)
        get_logger().info("Processing completed.")
        # check trace
        self._check_log_stream(self._get_log_stream(), 6, False)
        heap_stream = self._get_heap_stream()
        if heap_stream:
            total_leaks = 0
            for t in self.tasks_test_data:
                total_leaks += len(self.tasks_test_data[t]['leaks'])
            self.assertEqual(len(heap_stream.allocs), total_leaks)
            for alloc in heap_stream.allocs:
                alloc_valid = False
                # find alloc with such size in any reference task leaks list and remove respective entry from that list (it is handled)
                # every alloc has unique size
                for t in self.tasks_test_data:
                    if len(self.tasks_test_data[t]['leaks']) > 0 and self.tasks_test_data[t]['leaks'][0]['sz'] == alloc.size:
                        leak = self.tasks_test_data[t]['leaks'][0]
                        self.assertEqual(len(alloc.callers), len(leak['callers']))
                        for i in range(len(alloc.callers)):
                            ln = apptrace.addr2line(dbg.toolchain, elf_file, alloc.callers[i])
                            ln = ln.split(':')[-1].split('(')[0].strip()
                            if int(ln, 0) != int(leak['callers'][i], 0):
                                break
                            elif i == len(alloc.callers)-1:
                                self.tasks_test_data[t]['leaks'].pop(0)
                                alloc_valid = True
                    if alloc_valid:
                        break
                self.assertTrue(alloc_valid)

    @idf_ver_min('latest')
    def test_log_from_file(self):
        self._test_trace_from_file(self._do_test_log_continuous)

    @idf_ver_min('latest')
    def test_heap_log_from_file(self):
        self._test_trace_from_file(self._do_test_heap_log)


class SysViewTracingTestsImpl(BaseTracingTestsImpl):
    """ Test cases which are common for dual and single core modes
    """
    FREERTOS_EVENTS_MAP_FILE = os.path.join(os.path.dirname(__file__), 'SYSVIEW_FreeRTOS.txt')

    def _start_tracing(self, trace_src):
        if self.cores_num == 2:
            self.gdb.sysview_start(trace_src[0], trace_src[1])
        else:
            self.gdb.sysview_start(trace_src[0])

    def _stop_tracing(self):
        self.gdb.sysview_stop()

    def _get_parsers(self):
        parsers = []
        for i in range(self.cores_num):
            parsers.append(self.trace_ctrl[i]['parser'])
        return parsers

    def _create_processor(self, **proc_args):
        toolchain = ''
        elf_file = ''
        keep_all_events = False
        if 'toolchain' in proc_args:
            toolchain = proc_args['toolchain']
        if 'elf_file' in proc_args:
            elf_file = proc_args['elf_file']
        if 'keep_all_events' in proc_args:
            keep_all_events = proc_args['keep_all_events']
        self.processor = sysview.SysViewMultiStreamTraceDataProcessor(traces=self._get_parsers(), print_events=False, keep_all_events=keep_all_events)
        self.processor.add_stream_processor(sysview.SysViewTraceDataParser.STREAMID_HEAP,
                                  sysview.SysViewHeapTraceDataProcessor(toolchain, elf_file, root_proc=self.processor, print_heap_events=False))
        self.processor.add_stream_processor(sysview.SysViewTraceDataParser.STREAMID_LOG,
                                  sysview.SysViewLogTraceDataProcessor(root_proc=self.processor, print_log_events=False))

    def _process_trace(self):
        for i in range(self.cores_num):
            try:
                get_logger().info("Parse trace from '%s'..." % self.trace_ctrl[i]['src'])
                sysview.parse_trace(self.trace_ctrl[i]['reader'], self.trace_ctrl[i]['parser'], self.FREERTOS_EVENTS_MAP_FILE)
                get_logger().info("Parsing completed.")
            except (apptrace.ReaderTimeoutError) as e:
                get_logger().info("Stop parsing trace. (%s)" % e)
            except Exception as e:
                traceback.print_exc()
                self.fail("Failed to parse trace on core %d (%s)!" % (i, e))
        try:
            get_logger().info("Process events...")
            self.processor.merge_and_process()
            get_logger().info("Processing completed.")
        except Exception as e:
            traceback.print_exc()
            self.fail("Failed to process trace ({})!".format(e))

    def _get_log_stream(self):
        return self.processor.stream_procs[sysview.SysViewTraceDataParser.STREAMID_LOG]

    def _get_heap_stream(self):
        return self.processor.stream_procs[sysview.SysViewTraceDataParser.STREAMID_HEAP]

    def setUp(self):
        BaseTracingTestsImpl.setUp(self)
        self.processor = None
        self.trace_ctrl = [{'src': '', 'reader': None, 'parser': None} for i in range(self.cores_num)]
        for i in range(self.cores_num):
            # create parser
            try:
                parser = sysview.SysViewMultiTraceDataParser(print_events=False, core_id=i)
                parser.add_stream_parser(sysview.SysViewTraceDataParser.STREAMID_HEAP,
                                         sysview.SysViewHeapTraceDataParser(print_events=False, core_id=i))
                parser.add_stream_parser(sysview.SysViewTraceDataParser.STREAMID_LOG,
                                         sysview.SysViewLogTraceDataParser(print_events=False, core_id=i))
                self.trace_ctrl[i]['parser'] = parser
            except Exception as e:
                traceback.print_exc()
                self.fail("Failed to create data parser ({})!".format(e))
            # create reader with trace source URL
            self.trace_ctrl[i]['src'],self.trace_ctrl[i]['reader'] = _create_file_reader()
            if not self.trace_ctrl[i]['reader']:
                if i == 1:
                    self.trace_ctrl[0]['reader'].cleanup()
                self.fail("Failed to create trace reader for core %d!" % i)

    def tearDown(self):
        BaseTracingTestsImpl.tearDown(self)
        if self.processor:
            self.processor.cleanup()
        for i in range(self.cores_num):
            if self.trace_ctrl[i]['reader']:
                self.trace_ctrl[i]['reader'].cleanup()

    @idf_ver_min('latest')
    def test_log_from_file(self):
        trace_src = [self.trace_ctrl[0]['src']]
        if self.cores_num > 1:
            trace_src.append(self.trace_ctrl[1]['src'])
        self._do_test_log_continuous(trace_src)

    @idf_ver_min('latest')
    def test_heap_log_from_file(self):
        trace_src = [self.trace_ctrl[0]['src']]
        if self.cores_num > 1:
            trace_src.append(self.trace_ctrl[1]['src'])
        self._do_test_heap_log(trace_src)

    def test_os_tracing(self):
        """
            This test checks that OS level SystemView tracing works.
            This common test method is used for checking tracing via files, TCP and UDP sockets.
            1) Select appropriate sub-test number on target.
            4) Resume target and waits some time to allow test tasks and timers to start working.
            5) Executes OpenOCD command to start collecting SystemView trace data.
            4) Waits some time to allow test tasks and timers to work and generate trace data.
            5) Executes OpenOCD command to stop collecting trace data.
            7) Check that test tasks and timers ISRs are executed at pre-defined frequency.
        """
        trace_tasks = ['trace_task0']
        if self.test_tasks_num > 1:
            trace_tasks.append('trace_task1')
        trace_irqs = ['SysTick', 'TG1_T0_LEVEL']
        if self.test_tasks_num > 1:
            trace_irqs.append('TG1_T1_LEVEL')
        task_ref_data = {}
        task_ref_data['trace_task0'] = {'freq': 1.0/0.5, 'core': 0}
        if self.test_tasks_num > 1:
            task_ref_data['trace_task1'] = {'freq': 1.0/2.0, 'core': 1}
        irq_ref_data = {}
        irq_ref_data['SysTick'] = {'freq': 100.0, 'core': -1}
        irq_ref_data['TG1_T0_LEVEL'] = {'freq': 1.0/0.3, 'core': 0}
        if self.test_tasks_num > 1:
            irq_ref_data['TG1_T1_LEVEL'] = {'freq': 1.0/0.5, 'core': 1}

        self.select_sub_test(502)
        self.resume_exec()
        # colect trace
        time.sleep(3.0)
        if self.cores_num > 1:
            self.oocd.sysview_start(self.trace_ctrl[0]['src'], self.trace_ctrl[1]['src'])
        else:
            self.oocd.sysview_start(self.trace_ctrl[0]['src'])
        time.sleep(8.0)
        self.oocd.sysview_stop()

        self._create_processor(keep_all_events=True)
        try:
            self._process_trace()
        except (apptrace.ReaderTimeoutError) as e:
            get_logger().info("Stop processing trace. (%s)" % e)
        except Exception as e:
            traceback.print_exc()
            self.fail("Failed to parse trace (%s)!" % e)

        # Tasks and IRQs info are identical in traces files for both cores, so use info from CPU0
        task_run_data = dict(zip(trace_tasks, [{'handle': 0, 'run_count': 0, 'core': 0} for n in trace_tasks]))
        irq_run_data = dict(zip(trace_irqs, [{'handle': 0, 'run_count': 0, 'core': 0} for n in trace_irqs]))
        for tid,task_name in self.trace_ctrl[0]['parser'].tasks_info.iteritems():
            if task_name in task_run_data:
                task_run_data[task_name]['handle'] = tid
        for irqn,irq_name in self.trace_ctrl[0]['parser'].irqs_info.iteritems():
            if irq_name in irq_run_data:
                irq_run_data[irq_name]['handle'] = irqn

        for evt in self.processor.events:
            if evt.id == sysview.SYSVIEW_EVTID_ISR_ENTER:
                for name in irq_run_data:
                    if evt.params['irq_num'].value == irq_run_data[name]['handle']:
                        irq_run_data[name]['run_count'] += 1
                        break
            elif evt.id == sysview.SYSVIEW_EVTID_TASK_START_READY:
                for name in task_run_data:
                    if evt.params['tid'].value == task_run_data[name]['handle']:
                        task_run_data[name]['run_count'] += 1
                        break

        def print_run_data(name, run_data, iv):
            get_logger().debug('%s %x ran %d times in %f sec, freq = %f Hz', name, run_data['handle'], run_data['run_count'], iv, run_data['run_count']/iv)

        start_ts = self.processor.events[0].ts
        last_ts = self.processor.events[-1].ts
        iv = last_ts-start_ts
        for name in task_run_data:
            print_run_data('Task "%s"' % name, task_run_data[name], iv)
            freq_dev = 100*(task_ref_data[name]['freq'] - task_run_data[name]['run_count']/iv)/task_ref_data[name]['freq']
            self.assertTrue(freq_dev <= 10) # max event's freq deviation (due to measurement error) is 10%
        for name in irq_run_data:
            print_run_data('IRQ "%s"' % name, irq_run_data[name], iv)
            freq_dev = 100*(irq_ref_data[name]['freq'] - irq_run_data[name]['run_count']/iv)/irq_ref_data[name]['freq']
            self.assertTrue(freq_dev <= 10) # max event's freq deviation (due to measurement error) is 10%


########################################################################
#              TESTS DEFINITION WITH SPECIAL TESTS                     #
########################################################################

class SysViewTraceTestAppTestsDual(DebuggerGenericTestAppTests):
    """ Base class to run tests which use gcov test app in dual core mode
    """
    def __init__(self, methodName='runTest'):
        super(SysViewTraceTestAppTestsDual, self).__init__(methodName)
        self.test_app_cfg.bin_dir = os.path.join('output', 'svtrace_dual')
        self.test_app_cfg.build_dir = os.path.join('builds', 'svtrace_dual')
        self.test_tasks_num = 2
        self.cores_num = 2


class SysViewTraceTestAppTestsSingle(DebuggerGenericTestAppTests):
    """ Base class to run tests which use gcov test app in single core mode
    """
    def __init__(self, methodName='runTest'):
        super(SysViewTraceTestAppTestsSingle, self).__init__(methodName)
        self.test_app_cfg.bin_dir = os.path.join('output', 'svtrace_single')
        self.test_app_cfg.build_dir = os.path.join('builds', 'svtrace_single')
        self.test_tasks_num = 1
        self.cores_num = 1


class SysViewTracingTestsDual(SysViewTraceTestAppTestsDual, SysViewTracingTestsImpl):
    """ Test cases via GDB in dual core mode
    """
    def setUp(self):
        SysViewTraceTestAppTestsDual.setUp(self)
        SysViewTracingTestsImpl.setUp(self)

    def tearDown(self):
        SysViewTraceTestAppTestsDual.tearDown(self)
        SysViewTracingTestsImpl.tearDown(self)

class SysViewTracingTestsSingle(SysViewTraceTestAppTestsSingle, SysViewTracingTestsImpl):
    """ Test cases via GDB in single core mode
    """
    def setUp(self):
        SysViewTraceTestAppTestsSingle.setUp(self)
        SysViewTracingTestsImpl.setUp(self)

    def tearDown(self):
        SysViewTraceTestAppTestsSingle.tearDown(self)
        SysViewTracingTestsImpl.tearDown(self)
