"""运行态标定主机协议与离线证据门槛的故障回归；合成夹具不属于真实实测。"""

import copy
import io
from pathlib import Path
import struct
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import patch
import zlib

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'tools'))
import calibrate_pyocd as cal


class FakeClock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds


class FakeProbe:
    """仅模拟内存读写和请求提交；意外暂停、复位或下载调用会直接失败。"""
    unique_id = 'TEST-EXACT-PROBE'

    def __init__(self, clock=None):
        self.clock = clock or time.monotonic
        self.image = cal.FirmwareImage(0x20001000, 0x20002000, 0x80010000, b'Oct 13 2026 12:34:56\0', 'test-sha256')
        self.memory = bytearray(cal.MAILBOX_BYTES)
        self.log = bytearray(cal.LOG_CAPACITY * cal.RECORD.size)
        self.writes = []
        self.commands = []
        self.fail_payload = False
        self.drop_ack = False
        self.bad_crc = False
        self.snapshot_race = False
        self.persistent_snapshot_race = False
        self.log_read_limit = None
        self.snapshot_reads = 0
        self.auto_ticks = False
        self.on_command = None
        self.on_stop = None
        self.closed = False
        self.set_words(0, [cal.MAGIC, cal.ABI, cal.MAILBOX_BYTES, self.image.build_crc, cal.RECORD.size, cal.LOG_CAPACITY, 3, 0])
        self.set_words(120, [2])
        self.set_words(128, [10000, 1, 10000])
        self.set_words(128 + 11 * 4, [1, 1, 0, 0, 1])
        self.set_words(384, [1])
        self.set_words(400, [1, cal.float_word(100), cal.float_word(200), cal.float_word(2)])

    def set_words(self, offset, words):
        struct.pack_into('<' + str(len(words)) + 'I', self.memory, offset, *words)

    def word(self, offset):
        return struct.unpack_from('<I', self.memory, offset)[0]

    def read_memory_block8(self, address, count):
        if address == self.image.stamp_address:
            return list(self.image.stamp[:count])
        if self.image.log_address <= address < self.image.log_address + len(self.log):
            offset = address - self.image.log_address
            if self.log_read_limit is not None:
                count = min(count, self.log_read_limit)
            return list(self.log[offset:offset + count])
        offset = address - self.image.mailbox_address
        if not 0 <= offset < offset + count <= len(self.memory):
            raise AssertionError('读取越过模拟对象')
        if offset == 0 and count == cal.MAILBOX_BYTES:
            self.snapshot_reads += 1
            if self.auto_ticks:
                self.set_words(128, [self.word(128) + 10000, self.word(132) + 1])
            raw = bytes(self.memory)
            if self.snapshot_race or self.persistent_snapshot_race:
                self.snapshot_race = False
                self.set_words(120, [self.word(120) + 2])
            return list(raw)
        return list(self.memory[offset:offset + count])

    def write_memory(self, address, value, transfer_size):
        assert transfer_size == 32 and address % 4 == 0
        offset = address - self.image.mailbox_address
        self.writes.append((offset, struct.pack('<I', value), 'word32', self.clock()))
        self.set_words(offset, [value])
        if offset == 36:
            payload = bytes(self.memory[32:76])
            _, seq, opcode, *_ = struct.unpack('<11I', payload)
            status = 1 if self.bad_crc or self.word(76) != zlib.crc32(payload) else 0
            if not status:
                self.commands.append(opcode)
                if self.on_command is not None:
                    self.on_command(opcode, seq)
            if not self.drop_ack:
                self.set_words(96, [seq, status, self.word(32)])
                if opcode in (1, 2, 3):
                    self.set_words(108, [seq, opcode])
        if offset == 80:
            self.set_words(112, [8, 2])
            self.set_words(124, [value])
            if self.on_stop is not None:
                self.on_stop()
        if offset == 88 and hasattr(self, 'heartbeat_observed') and sum(item[0] == 88 for item in self.writes) >= 3:
            self.heartbeat_observed.set()

    def write_memory_block8(self, address, values):
        offset = address - self.image.mailbox_address
        self.writes.append((offset, bytes(values), 'block', self.clock()))
        self.memory[offset:offset + len(values)] = bytes(values)
        if self.fail_payload:
            raise OSError('载荷写入中断')

    def flush(self):
        pass

    def get_state(self):
        from pyocd.core.target import Target
        return Target.State.RUNNING

    def halt(self):
        raise AssertionError('禁止暂停')

    def reset(self):
        raise AssertionError('禁止复位')

    def resume(self):
        raise AssertionError('禁止自动恢复')

    def program_page(self):
        raise AssertionError('禁止下载')


class ProtocolTests(unittest.TestCase):
    def setUp(self):
        self.clock = FakeClock()
        self.probe = FakeProbe(self.clock)
        self.transport = cal.MemoryTransport(self.probe, self.probe.image, 0x12345678,
                                             clock=self.clock, sleeper=self.clock.sleep)

    def test_readonly_status_has_no_writes_and_unmeasured_rpm_is_null(self):
        state = self.transport.snapshot()
        self.assertIsNone(state['rpm100_left'])
        self.assertIsNone(state['rpm500_right'])
        self.assertEqual(self.probe.writes, [])

    def test_identity_mismatch_refuses_writes(self):
        for offset, value in ((0, 0), (4, 2), (8, 412), (12, 123), (16, 64), (20, 2048), (24, 0)):
            probe = FakeProbe(self.clock)
            probe.set_words(offset, [value])
            with self.subTest(offset=offset), self.assertRaises(cal.CalibrationError):
                cal.MemoryTransport(probe, probe.image, 3, clock=self.clock, sleeper=self.clock.sleep)
            self.assertFalse(probe.writes)

    def test_symbol_bounds_overlap_and_stamp_shape(self):
        image = self.probe.image
        for kwargs in ({'mailbox_address': 0x20200000}, {'log_address': 0x2006F000},
                       {'log_address': image.mailbox_address}, {'stamp': b'bad'}, {'stamp_address': 0x20050000}):
            with self.subTest(kwargs=kwargs), self.assertRaises(cal.CalibrationError):
                cal.FirmwareImage(**{**image.__dict__, **kwargs}).validate()

    def test_host_writes_are_whitelisted(self):
        for offset in (0, 92, 96, 120, 128, 392, 414):
            with self.subTest(offset=offset), self.assertRaises(cal.CalibrationError):
                self.transport.write(offset, struct.pack('<I', 1))
        with self.assertRaises(cal.CalibrationError):
            self.transport.write(40, bytes(44))
        with self.assertRaises(cal.CalibrationError):
            self.transport.write(40, b'1')
        self.assertFalse(self.probe.writes)

    def test_firmware_changed_during_session_blocks_heartbeat_stop_and_command_writes(self):
        self.probe.set_words(12, [999])
        self.transport.heartbeat_enabled = True
        with self.assertRaises(cal.CalibrationError):
            self.transport.maintenance()
        with self.assertRaises(cal.CalibrationError):
            self.transport.stop()
        with self.assertRaises(cal.CalibrationError):
            self.transport.command(cal.CMD_START, [1])
        self.assertFalse(self.probe.writes)

    def test_payload_crc_then_atomic_commit_and_one_inflight(self):
        self.transport.command(cal.CMD_PREPARE_COUNT, [1])
        offsets = [item[0] for item in self.probe.writes]
        self.assertEqual(offsets, [32, 40, 36])
        self.assertEqual(self.probe.writes[-1][2], 'word32')
        self.assertEqual(self.probe.word(76), zlib.crc32(bytes(self.probe.memory[32:76])))
        self.probe.set_words(36, [9])
        self.probe.set_words(96, [8])
        previous = len(self.probe.writes)
        with self.assertRaises(cal.CalibrationError):
            self.transport.command(cal.CMD_START, [1])
        self.assertEqual(len(self.probe.writes), previous)

    def test_payload_interrupted_never_commits(self):
        self.probe.fail_payload = True
        with self.assertRaises(OSError):
            self.transport.command(cal.CMD_START, [1])
        self.assertEqual(self.probe.word(36), 0)
        self.assertEqual(self.probe.commands, [])

    def test_bad_crc_is_rejected_and_not_retried(self):
        self.probe.bad_crc = True
        with self.assertRaisesRegex(cal.CalibrationError, 'BAD_CRC'):
            self.transport.command(cal.CMD_START, [1])
        self.assertEqual(self.probe.commands, [])
        self.assertEqual(sum(offset == 36 for offset, *_ in self.probe.writes), 1)

    def test_ack_lost_start_only_once_then_stop(self):
        self.probe.drop_ack = True
        with self.assertRaises(cal.AckTimeout):
            self.transport.command(cal.CMD_START, [1], 0.2)
        self.assertEqual(self.probe.commands, [cal.CMD_START])
        self.assertTrue(any(offset == 80 and mode == 'word32' for offset, _, mode, _ in self.probe.writes))
        with self.assertRaises(cal.CalibrationError):
            self.transport.command(cal.CMD_START, [1])
        self.assertEqual(self.probe.commands, [cal.CMD_START])

    def test_stop_cancels_command_before_commit(self):
        self.transport.stop_requested.set()
        with self.assertRaises(cal.CalibrationError):
            self.transport.command(cal.CMD_START, [1])
        self.assertEqual(self.probe.commands, [])
        self.assertEqual([item[0] for item in self.probe.writes], [80])

    def test_snapshot_retries_mixed_cycle(self):
        self.probe.snapshot_race = True
        state = self.transport.snapshot()
        self.assertEqual(self.probe.snapshot_reads, 2)
        self.assertEqual(state['snapshot_seq'], 4)

    def test_start_ack_accepted_but_snapshot_unavailable_immediately_stops(self):
        for opcode in (cal.CMD_START, cal.CMD_COUNT_START):
            probe = FakeProbe(self.clock)
            transport = cal.MemoryTransport(probe, probe.image, 3, clock=self.clock, sleeper=self.clock.sleep)
            probe.persistent_snapshot_race = True
            with self.subTest(opcode=opcode), self.assertRaisesRegex(cal.CalibrationError, 'START 已接受'):
                transport.command(opcode, [1])
            self.assertEqual(probe.commands, [opcode])
            self.assertEqual(sum(offset == 36 for offset, *_ in probe.writes), 1)
            self.assertEqual(sum(offset == 80 for offset, *_ in probe.writes), 1)

    def test_heartbeat_while_awaiting_ack(self):
        self.transport.heartbeat_enabled = True
        self.probe.drop_ack = True
        with self.assertRaises(cal.AckTimeout):
            self.transport.command(cal.CMD_START, [1], 0.35)
        times = [when for offset, _, _, when in self.probe.writes if offset == 88]
        self.assertGreaterEqual(len(times), 4)
        self.assertLessEqual(max(b - a for a, b in zip(times, times[1:])), 0.106)

    def test_worker_idle_input_wait_keeps_heartbeat(self):
        probe = FakeProbe()
        probe.heartbeat_observed = threading.Event()
        transport = cal.MemoryTransport(probe, probe.image, 91, heartbeat=True)
        worker = cal.IOWorker(lambda: (transport, lambda: None))
        try:
            self.assertTrue(probe.heartbeat_observed.wait(1))
        finally:
            worker.close()
        self.assertTrue(any(offset == 80 for offset, *_ in probe.writes))

    def test_attach_options_and_no_implicit_target_actions(self):
        probe = FakeProbe()
        probe.auto_ticks = True
        observed = {}
        class FakeSession:
            def __init__(self, selected, auto_open, options):
                observed.update(probe=selected, auto_open=auto_open, options=options)
                self.target = probe
            def open(self):
                self._load_user_script()
            def _load_user_script(self):
                raise AssertionError('项目调试脚本不得加载')
            def close(self):
                probe.closed = True
        with patch('pyocd.core.session.Session', FakeSession), patch('pyocd.core.helpers.ConnectHelper.get_all_connected_probes', return_value=[probe]):
            transport, close = cal.attach_factory(probe.image, probe.unique_id, False)()
            self.assertTrue(transport.validated)
            close()
        self.assertEqual(observed['options'], cal.ATTACH_OPTIONS)
        self.assertFalse(observed['auto_open'])
        self.assertEqual(observed['options']['connect_mode'], 'attach')
        self.assertFalse(observed['options']['auto_unlock'])
        self.assertFalse(observed['options']['resume_on_disconnect'])
        self.assertFalse(probe.writes)

    def test_frozen_log_reconstructs_totals_and_uses_firmware_pwm_coefficient(self):
        self.probe.set_words(256, [0xFFFFD8F0, 10000, 100, 0, 200, 0, 105, 0, 207, 0])
        self.probe.set_words(364, [9, cal.KIND_SERVO, 1, 1, 2, 1, 2])
        self.probe.set_words(412, [cal.float_word(1.5)])
        first = cal.RECORD.pack(8, 0, 10000, 2, 3, 2, 3, 0, 0, 1000, 17, 1500)
        second = cal.RECORD.pack(9, 10000, 10000, 3, 4, 3, 4, 0, 0, 0, 1, 1500)
        self.probe.log[:64] = first + second
        data = self.transport.frozen_export()
        self.assertTrue(data['metadata']['valid'])
        self.assertEqual(data['records'][0]['elapsed_us'], 10000)
        self.assertEqual(data['records'][-1]['total_left'], 105)
        self.assertEqual(data['records'][0]['servo_nominal_us'], 1500)
        self.assertEqual(data['records'][-1]['servo_nominal_us'], 0)
        self.probe.set_words(280, [106])
        self.assertFalse(self.transport.frozen_export()['metadata']['valid'])
        self.probe.set_words(280, [105])
        struct.pack_into('<H', self.probe.log, 26, 49)
        self.assertFalse(self.transport.frozen_export()['metadata']['valid'])
        self.probe.set_words(384, [0])
        with self.assertRaises(cal.CalibrationError):
            self.transport.frozen_export()

    def test_live_marker_zero_timestamp_is_not_discarded(self):
        self.probe.set_words(256, [0, 10000])
        self.probe.set_words(128 + 27 * 4, [1])
        self.probe.set_words(328, [100])
        markers = self.transport.snapshot()['markers']
        self.assertEqual(markers, [{'index': 0, 'begin_us': 0, 'end_us': 100}])

    def test_short_frozen_log_read_is_rejected_even_if_missing_record_has_zero_delta(self):
        self.probe.set_words(256, [0, 20000, 0, 0, 0, 0, 2, 0, 3, 0])
        self.probe.set_words(364, [9, cal.KIND_SERVO, 1, 1, 2, 1, 2])
        first = cal.RECORD.pack(8, 10000, 10000, 2, 3, 2, 3, 0, 0, 750, 17, 1500)
        second = cal.RECORD.pack(9, 20000, 10000, 0, 0, 0, 0, 0, 0, 0, 1, 0)
        self.probe.log[:64] = first + second
        self.probe.log_read_limit = 32
        with self.assertRaisesRegex(cal.CalibrationError, '短读'):
            self.transport.frozen_export()
        self.assertFalse(self.probe.writes)

    def test_normal_bench_control_end_is_valid_but_frozen_fault_cannot_be_cleared_away(self):
        self.probe.set_words(256, [0, 10000, 0, 0, 0, 0, 2, 0, 3, 0])
        self.probe.set_words(364, [9, cal.KIND_RPM, 1, 1, 1, 1, 6])
        self.probe.log[:32] = cal.RECORD.pack(8, 10000, 10000, 2, 3, 2, 3, 0, 0, 0, 1, 0)
        self.assertTrue(self.transport.frozen_export()['metadata']['valid'])
        struct.pack_into('<H', self.probe.log, 26, 33)
        self.assertFalse(self.transport.frozen_export()['metadata']['valid'])
        self.probe.set_words(368, [cal.KIND_COUNT])
        self.assertFalse(self.transport.frozen_export()['metadata']['valid'])


def synthetic_measurements(directory):
    """测试完整矩阵；标签明确为合成夹具，绝不作为项目默认标定值。"""
    build = '12345678'
    data = {'version': 1, 'firmware_build_id': build, 'center_us': 1500,
            'provenance': {'real_measurements': True, 'operator': '软件测试夹具', 'date': 'synthetic-fixture',
                           'angle_method': '软件合成用于验证', 'encoder_method': '软件合成用于验证',
                           'toe_left_deg': 0.2, 'toe_right_deg': -0.2},
            'encoder_runs': [], 'servo_samples': []}
    for wheel, n in (('left', 100), ('right', 200)):
        for direction, sign in (('forward', 1), ('reverse', -1)):
            for repeat in (1, 2, 3):
                name = f'{wheel}_{direction}_{repeat}.json'
                record = {'metadata': {'build_id': build, 'kind': cal.KIND_COUNT, 'frozen': True, 'valid': True,
                                       'count_delta_' + wheel: sign * n * 10}}
                cal.write_json(directory / name, record)
                data['encoder_runs'].append({'wheel': wheel, 'direction': direction, 'turns': 10,
                                             'count_delta': sign * n * 10, 'run_export': name})
    for pulse, angle in ((1400, 10), (1500, 0), (1600, -10), (1450, 5)):
        for direction in ('increasing', 'decreasing'):
            for repeat in (1, 2, 3):
                name = f'servo_{pulse}_{direction}_{repeat}.json'
                cal.write_json(directory / name, {'metadata': {'build_id': build, 'kind': cal.KIND_SERVO, 'frozen': True, 'valid': True},
                                                  'records': [{'servo_nominal_us': pulse}]})
                data['servo_samples'].append({'nominal_us': pulse, 'left_deg': angle, 'right_deg': angle * 0.9,
                                             'direction': direction, 'repeat': repeat, 'role': 'validation' if pulse == 1450 else 'table',
                                             'run_export': name})
    return data


class CountInteractiveTests(unittest.TestCase):
    def setUp(self):
        build = ROOT / 'tools/tests/_build'
        build.mkdir(parents=True, exist_ok=True)
        self.tmp = tempfile.TemporaryDirectory(prefix='count-session-tests-', dir=build)
        self.directory = Path(self.tmp.name)
        self.probe = FakeProbe()
        self.probe.set_words(224, [100000])
        self.probe.set_words(384, [1])

        def commanded(opcode, seq):
            if opcode == cal.CMD_COUNT_START:
                self.probe.set_words(112, [4, 0])
                self.probe.set_words(364, [seq, 4, 1, 1, 0, 0, 0])
                self.probe.set_words(256, [10000, 0, seq * 10, 0, 0, 0, 0, 0, 0, 0])
            elif opcode == cal.CMD_COUNT_END:
                self.probe.set_words(112, [7, 1])
                self.probe.set_words(260, [20000])
                self.probe.set_words(280, [self.probe.word(264) + 100, 0, 0, 0])
                self.probe.set_words(384, [1, 1])

        self.probe.on_command = commanded
        def stopped():
            self.probe.set_words(384, [1])
            if self.probe.word(36) != self.probe.word(96):
                self.probe.set_words(96, [self.probe.word(36), 11, self.probe.word(32)])

        self.probe.on_stop = stopped

    def tearDown(self):
        self.tmp.cleanup()

    def run_session(self, commands, skip_save=False):
        worker = cal.IOWorker(lambda: (cal.MemoryTransport(self.probe, self.probe.image, 20, heartbeat=True), lambda: None))
        output, errors = io.StringIO(), io.StringIO()
        try:
            with patch('builtins.input', side_effect=commands), patch('sys.stdout', output), patch('sys.stderr', errors):
                cal.interactive(worker, self.directory / 'recovery', count_only=True, skip_save=skip_save)
        finally:
            worker.close()
        return output.getvalue(), errors.getvalue()

    def test_start_end_restart_discards_only_the_previous_count(self):
        _, errors = self.run_session(['start', 'end', 'count-note left forward 10', 'restart', 'end', 'quit'])
        self.assertEqual(errors, '')
        self.assertEqual(self.probe.commands, [1, 6, 7, 1, 6, 7])
        files = list((self.directory / 'recovery').glob('*.json'))
        self.assertEqual(len(files), 1)
        saved = cal.load_json(files[0])
        self.assertEqual(saved['metadata']['run_id'], 5)
        self.assertEqual(saved['manual_measurements'], [])

    def test_skip_save_setting_allows_start_after_end_without_an_export(self):
        _, errors = self.run_session(['start', 'end', 'set skip-save on', 'start', 'end', 'quit'])
        self.assertEqual(errors, '')
        self.assertEqual(self.probe.commands.count(cal.CMD_COUNT_START), 2)
        self.assertEqual(len(list((self.directory / 'recovery').glob('*.json'))), 1)

    def test_default_requires_export_or_explicit_restart_and_rejects_servo(self):
        _, errors = self.run_session(['start', 'end', 'start', 'prepare servo 1500 1400 1600 100', 'quit'])
        self.assertIn('尚未export', errors)
        self.assertIn('只接受手转命令', errors)
        self.assertEqual(self.probe.commands, [1, 6, 7])

    def test_restart_does_not_discard_servo_or_bypass_a_fault(self):
        worker = cal.IOWorker(lambda: (cal.MemoryTransport(self.probe, self.probe.image, 20), lambda: None))
        try:
            self.probe.set_words(112, [5])
            self.probe.set_words(368, [5])
            with self.assertRaisesRegex(cal.CalibrationError, '舵机/RPM'):
                cal.prepare_count_start(worker, discard=True)
            self.assertFalse(self.probe.writes)
            self.probe.set_words(112, [8])
            self.probe.set_words(368, [4])
            self.probe.set_words(184, [1])
            with self.assertRaisesRegex(cal.CalibrationError, '无故障READY'):
                cal.prepare_count_start(worker, discard=True)
            self.assertEqual(self.probe.commands, [])
        finally:
            worker.close()

    def test_count_only_refuses_the_old_time_limited_firmware(self):
        self.probe.set_words(24, [1])
        worker = cal.IOWorker(lambda: (cal.MemoryTransport(self.probe, self.probe.image, 20), lambda: None))
        try:
            with self.assertRaisesRegex(cal.CalibrationError, '旧固件仍有120秒'):
                cal.prepare_count_start(worker)
            self.assertEqual(self.probe.writes, [])
        finally:
            worker.close()

    def test_old_frozen_snapshot_cannot_acknowledge_a_new_stop(self):
        clock = FakeClock()
        probe = FakeProbe(clock)
        probe.set_words(112, [7])
        probe.set_words(384, [1])
        pending_reads = []
        original_read = probe.read_memory_block8

        def read(address, count):
            if probe.word(80) and address == probe.image.mailbox_address and count == cal.MAILBOX_BYTES:
                pending_reads.append(1)
                if len(pending_reads) >= 3:
                    probe.set_words(124, [probe.word(80)])
            return original_read(address, count)

        probe.read_memory_block8 = read
        probe.on_stop = lambda: probe.set_words(124, [0])
        transport = cal.MemoryTransport(probe, probe.image, 20, clock=clock, sleeper=clock.sleep)
        state = transport.stop_and_wait()
        self.assertEqual(state['stop_ack_seq'], transport.stop_sequence)
        self.assertGreaterEqual(len(pending_reads), 3)
        self.assertGreaterEqual(clock.now, 0.04)

    def test_unknown_count_start_can_only_restart_after_confirmed_stop(self):
        original_command, original_stop = self.probe.on_command, self.probe.on_stop
        for failure in ('ack_lost', 'snapshot_lost'):
            first = True

            def commanded(opcode, seq):
                nonlocal first
                original_command(opcode, seq)
                if opcode == cal.CMD_COUNT_START and first:
                    first = False
                    if failure == 'ack_lost':
                        self.probe.drop_ack = True
                    else:
                        self.probe.persistent_snapshot_race = True

            def stopped():
                original_stop()
                self.probe.drop_ack = self.probe.persistent_snapshot_race = False

            self.probe.on_command, self.probe.on_stop = commanded, stopped
            self.probe.commands.clear()
            self.probe.set_words(112, [0])
            self.probe.set_words(368, [4])
            self.probe.set_words(384, [1])
            with self.subTest(failure=failure):
                _, errors = self.run_session(['start', 'restart', 'end', 'quit'])
                self.assertIn('STOP', errors)
                self.assertEqual(self.probe.commands, [1, 6, 1, 6, 7])


class ServoInteractiveTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(prefix='servo-session-tests-', dir=ROOT / 'tools/tests/_build')
        self.directory = Path(self.tmp.name)
        self.probe = FakeProbe()
        self.probe.set_words(24, [11])
        self.probe.set_words(384, [1])
        self.refuse_start = False
        self.delay_pulse = False
        self.unknown_start = None
        self.start_attempts = 0

        def commanded(opcode, seq):
            if opcode == cal.CMD_START:
                self.start_attempts += 1
                if self.refuse_start:
                    self.probe.bad_crc = True
                self.probe.set_words(112, [5, 0])
                self.probe.set_words(172, [17, 2, 2, 0, 1])
                self.probe.set_words(204, [763, cal.float_word(1526), cal.float_word(1526)])
                self.probe.set_words(364, [seq, 5, 1, 1, 0, 0, 0])
                self.probe.set_words(256, [10000, 0, 0, 0, 0, 0, 0, 0, 0, 0])
                if self.unknown_start and self.start_attempts == 1:
                    self.probe.drop_ack = self.unknown_start == 'ack_lost'
                    self.probe.persistent_snapshot_race = self.unknown_start == 'snapshot_lost'
            elif opcode == cal.CMD_PULSE and not self.delay_pulse:
                pulse = cal.word_float(self.probe.word(44))
                self.probe.set_words(204, [int(pulse / 2), cal.float_word(pulse), cal.float_word(int(pulse / 2) * 2)])
            elif opcode == cal.CMD_SERVO_END:
                self.finish(manual=True)

        def stopped():
            manual = self.probe.word(388) == 1 and self.probe.word(384) == 1
            fault = self.probe.word(184)
            self.finish(manual=manual)
            if fault:
                self.probe.set_words(176, [3])
                self.probe.set_words(184, [fault])
            self.probe.drop_ack = self.probe.persistent_snapshot_race = False
            if self.probe.word(36) != self.probe.word(96):
                self.probe.set_words(96, [self.probe.word(36), 11, self.probe.word(32)])

        self.probe.on_command, self.probe.on_stop = commanded, stopped

    def finish(self, manual=False):
        self.probe.set_words(112, [7 if manual else 8, 1 if manual else 2])
        self.probe.set_words(172, [1, 1, 2, 0, 1])
        self.probe.set_words(200, [0, 0, 0, 0])
        self.probe.set_words(260, [40000])
        self.probe.set_words(380, [2, 1, 1 if manual else 2])
        self.probe.log[:64] = cal.RECORD.pack(2, 20000, 10000, 0, 0, 0, 0, 0, 0, 763, 17, 1526) + cal.RECORD.pack(4, 40000, 10000, 0, 0, 0, 0, 0, 0, 0, 1, 0)

    def tearDown(self):
        self.tmp.cleanup()

    def run_session(self, commands):
        worker = cal.IOWorker(lambda: (cal.MemoryTransport(self.probe, self.probe.image, 20, heartbeat=True), lambda: None))
        output, errors = io.StringIO(), io.StringIO()
        try:
            with patch('builtins.input', side_effect=commands), patch('sys.stdout', output), patch('sys.stderr', errors):
                cal.interactive(worker, self.directory / 'recovery', servo_only=True)
        finally:
            worker.close()
        return output.getvalue(), errors.getvalue()

    def test_manual_end_and_restart_drop_only_unsaved_current_group(self):
        commands = ['prepare servo 1526 1516 1536 100', 'CONFIRM', 'start', 'RUN',
                    'angle 0 0 increasing 1 table', 'end', 'restart', 'RUN', 'end', 'quit']
        output, errors = self.run_session(commands)
        self.assertEqual(errors, '')
        self.assertEqual(self.probe.commands, [2, 4, 14, 2, 4, 14])
        self.assertIn('"limit_s": null', output)
        raw = list((self.directory / 'recovery').glob('*.json'))
        self.assertEqual(len(raw), 1)
        exported = cal.load_json(raw[0])
        self.assertTrue(exported['metadata']['valid'])
        self.assertEqual(exported['metadata']['recording_mode'], 'servo_events')
        self.assertEqual(exported['manual_measurements'], [])
        self.assertIsNone(exported['records'][0]['total_left'])

    def test_restart_cancellation_keeps_zero_output_and_does_not_start(self):
        _, errors = self.run_session(['prepare servo 1526 1516 1536 100', 'CONFIRM', 'start', 'RUN', 'restart', 'CANCEL', 'quit'])
        self.assertEqual(errors, '')
        self.assertEqual(self.probe.commands, [2, 4, 2])
        self.assertEqual(self.probe.word(204), 0)

    def test_exported_files_survive_restart_and_start_reuses_confirmed_range(self):
        path = self.directory / 'valid.json'
        commands = ['prepare servo 1526 1516 1536 100', 'CONFIRM', 'start', 'RUN',
                    'end', 'export ' + str(path), 'start', 'RUN', 'restart', 'RUN', 'end', 'quit']
        _, errors = self.run_session(commands)
        self.assertEqual(errors, '')
        self.assertEqual(cal.load_json(path)['metadata']['run_id'], 2)
        self.assertEqual(self.probe.commands.count(cal.CMD_START), 3)

    def test_servo_entry_rejects_rpm_count_and_unprepared_restart(self):
        _, errors = self.run_session(['prepare rpm 5 5', 'prepare count', 'restart', 'start', 'quit'])
        self.assertIn('只允许prepare servo', errors)
        self.assertIn('确认机械范围', errors)
        self.assertFalse(self.probe.commands)

    def test_old_firmware_refused_before_any_write(self):
        self.probe.set_words(24, [3])
        with self.assertRaisesRegex(cal.CalibrationError, '新版'):
            cal.MemoryTransport(self.probe, self.probe.image, 20, heartbeat=True, required_features=8)
        self.assertFalse(self.probe.writes)

    def test_restart_refuses_fault_and_other_trial_before_prepare(self):
        worker = cal.IOWorker(lambda: (cal.MemoryTransport(self.probe, self.probe.image, 20), lambda: None))
        try:
            values = [cal.float_word(value) for value in (1526, 1516, 1536, 100)]
            self.probe.set_words(112, [6])
            self.probe.set_words(368, [6])
            with self.assertRaisesRegex(cal.CalibrationError, 'RPM'):
                cal.prepare_servo_start(worker, values, True)
            self.assertFalse(self.probe.writes)
            self.probe.set_words(112, [8])
            self.probe.set_words(368, [5])
            self.probe.set_words(184, [1])
            with self.assertRaisesRegex(cal.CalibrationError, '故障'):
                cal.prepare_servo_start(worker, values, True)
            self.assertFalse(self.probe.commands)
        finally:
            worker.close()

    def test_angle_refused_until_requested_target_is_reached(self):
        self.delay_pulse = True
        _, errors = self.run_session(['prepare servo 1526 1516 1536 100', 'CONFIRM', 'start', 'RUN',
                                      'pulse 1536', 'angle 0 0 increasing 1 table', 'end', 'quit'])
        self.assertIn('尚未到达目标脉宽', errors)

    def test_unknown_start_requires_confirmed_stop_before_explicit_restart(self):
        for failure in ('ack_lost', 'snapshot_lost'):
            self.unknown_start, self.start_attempts = failure, 0
            self.probe.commands.clear()
            self.probe.set_words(112, [0])
            self.probe.set_words(368, [0])
            with self.subTest(failure=failure):
                _, errors = self.run_session(['prepare servo 1526 1516 1536 100', 'CONFIRM',
                                              'start', 'RUN', 'restart', 'RUN', 'end', 'quit'])
                self.assertIn('STOP', errors)
                self.assertEqual(self.probe.commands, [2, 4, 2, 4, 14])

    def test_sparse_export_rejects_impossible_time_or_repeated_sample(self):
        self.probe.set_words(368, [5])
        self.finish(manual=True)
        transport = cal.MemoryTransport(self.probe, self.probe.image, 20)
        self.assertTrue(transport.frozen_export()['metadata']['valid'])
        for sample, now in ((102, 40000), (2, 40000), (3, 40000), (4, 48000), (5, 40001)):
            self.probe.set_words(260, [now])
            self.probe.log[32:64] = cal.RECORD.pack(sample, now, 10000, 0, 0, 0, 0, 0, 0, 0, 1, 0)
            self.assertFalse(transport.frozen_export()['metadata']['valid'])

    def test_sparse_export_requires_zero_motors_and_final_zero_servo(self):
        self.probe.set_words(368, [5])
        self.finish(manual=True)
        transport = cal.MemoryTransport(self.probe, self.probe.image, 20)
        for motor, servo, flags, now in ((1, 0, 1, 40000), (0, 763, 17, 40000), (0, 0, 1, 44000)):
            self.probe.log[32:64] = cal.RECORD.pack(4, now, 10000, 0, 0, 0, 0, motor, 0, servo, flags, 0)
            self.assertFalse(transport.frozen_export()['metadata']['valid'])


class EvidenceTests(unittest.TestCase):
    def setUp(self):
        build = ROOT / 'tools/tests/_build'
        build.mkdir(parents=True, exist_ok=True)
        self.tmp = tempfile.TemporaryDirectory(prefix='calibration-tests-', dir=build)
        self.directory = Path(self.tmp.name)
        self.data = synthetic_measurements(self.directory)

    def tearDown(self):
        self.tmp.cleanup()

    def test_complete_real_measurement_matrix_generates_c_and_audit_json(self):
        source = self.directory / 'measurements.json'
        output = self.directory / 'candidate.c'
        cal.write_json(source, self.data)
        result = cal.generate_profile(source, output)
        self.assertEqual(result['counts_per_rev'], {'left': 100, 'right': 200})
        self.assertEqual(result['points'][1]['left_deg'], 0)
        self.assertTrue(output.with_suffix('.json').exists())
        self.assertIn('const SteeringMap calibration_profile_steering_map', output.read_text(encoding='utf-8'))

    def test_generated_candidate_compiles_and_passes_actual_firmware_map_validator(self):
        source = self.directory / 'measurements.json'
        generated = self.directory / 'candidate.c'
        cal.write_json(source, self.data)
        cal.generate_profile(source, generated)
        fixture = self.directory / 'check_candidate.c'
        fixture.write_text('#include "calibration_profile.h"\n#include "calibration_debug.h"\n#include <assert.h>\n'
                           'int main(void) { assert(steering_map_validate(&calibration_profile_steering_map)); '
                           'assert(calibration_profile_counts_per_rev_left == 100.0f); '
                           'assert(calibration_profile_counts_per_rev_right == 200.0f); '
                           'assert(sizeof(CalibrationDebugMailbox) == 416u); '
                           'assert(sizeof(CalibrationDebugRecord) == 32u); '
                           'assert(offsetof(CalibrationDebugMailbox, measurement_version) == 400u); '
                           'assert(offsetof(CalibrationDebugMailbox, nominal_us_per_duty_bits) == 412u); return 0; }\n', encoding='utf-8')
        exe = self.directory / 'check_candidate.exe'
        sources = [generated, fixture, ROOT / 'user/src/steering_map.c']
        try:
            subprocess.run(['gcc', '--version'], capture_output=True, check=True)
            command = ['gcc', '-std=c99', '-Wall', '-Wextra', '-Werror', '-pedantic', '-I', str(ROOT / 'user/inc'),
                       *map(str, sources), '-lm', '-o', str(exe)]
        except (OSError, subprocess.CalledProcessError):
            compiler = sorted(Path('C:/Program Files/Microsoft Visual Studio/2022/Community/VC/Tools/MSVC').glob('*'))[-1]
            kit = sorted(Path('C:/Program Files (x86)/Windows Kits/10/Include').glob('*'))[-1]
            command = [str(compiler / 'bin/Hostx64/x64/cl.exe'), '/nologo', '/std:c11', '/utf-8', '/W4', '/WX', '/MD',
                       '/I' + str(ROOT / 'user/inc'), '/I' + str(compiler / 'include'), '/I' + str(kit / 'ucrt'),
                       '/I' + str(kit / 'shared'), '/I' + str(kit / 'um'), '/Fe' + str(exe), *map(str, sources), '/link',
                       '/LIBPATH:' + str(compiler / 'lib/x64'), '/LIBPATH:' + str(kit.parents[1] / 'Lib' / kit.name / 'ucrt/x64'),
                       '/LIBPATH:' + str(kit.parents[1] / 'Lib' / kit.name / 'um/x64')]
        result = subprocess.run(command, cwd=self.directory, capture_output=True, timeout=30)
        self.assertEqual(result.returncode, 0, (result.stdout + result.stderr).decode('utf-8', errors='replace'))
        self.assertEqual(subprocess.run([str(exe)], capture_output=True, timeout=5).returncode, 0)

    def test_counts_can_be_validated_before_servo_measurement(self):
        data = copy.deepcopy(self.data)
        data.pop('servo_samples')
        data.pop('center_us')
        data['provenance'].pop('angle_method')
        data['provenance'].pop('toe_left_deg')
        data['provenance'].pop('toe_right_deg')
        result = cal.validate_encoder_measurements(data, self.directory)
        self.assertEqual(result['counts_per_rev'], {'left': 100, 'right': 200})
        with self.assertRaises(cal.CalibrationError):
            cal.validate_measurements(data, self.directory)

    def test_missing_measurement_invalid_numbers_and_bad_direction(self):
        def change_source(data):
            data['provenance']['real_measurements'] = False
        def incomplete(data):
            data['encoder_runs'].pop()
        def wrong_sign(data):
            data['encoder_runs'][0]['count_delta'] *= -1
        def duplicate_evidence(data):
            data['encoder_runs'][1] = copy.deepcopy(data['encoder_runs'][0])
        def nan_angle(data):
            data['servo_samples'][0]['left_deg'] = float('nan')
        def missing_validation(data):
            data['servo_samples'] = [sample for sample in data['servo_samples'] if sample['role'] == 'table']
        for transform in (change_source, incomplete, wrong_sign, duplicate_evidence, nan_angle, missing_validation):
            data = copy.deepcopy(self.data)
            transform(data)
            with self.subTest(transform=transform.__name__), self.assertRaises(cal.CalibrationError):
                cal.validate_measurements(data, self.directory)

    def test_encoder_repeatability_gate_and_raw_count_comparison(self):
        data = copy.deepcopy(self.data)
        data['encoder_runs'][0]['count_delta'] = 1020
        name = data['encoder_runs'][0]['run_export']
        record = cal.load_json(self.directory / name)
        record['metadata']['count_delta_left'] = 1020
        cal.write_json(self.directory / name, record)
        with self.assertRaisesRegex(cal.CalibrationError, '1%'):
            cal.validate_measurements(data, self.directory)
        data['encoder_runs'][0]['count_delta'] = 1010
        with self.assertRaisesRegex(cal.CalibrationError, '累计计数'):
            cal.validate_measurements(data, self.directory)

    def test_servo_hysteresis_monotonicity_and_heldout_error(self):
        for selection, value in ((0, 13), (6, 20), (18, 9)):
            data = copy.deepcopy(self.data)
            data['servo_samples'][selection]['left_deg'] = value
            with self.subTest(selection=selection), self.assertRaises(cal.CalibrationError):
                cal.validate_measurements(data, self.directory)

    def test_source_mismatch_freeze_and_pulse_not_in_raw_export(self):
        for update in ({'build_id': 'mismatch'}, {'frozen': False}, {'valid': False}):
            path = self.directory / self.data['servo_samples'][0]['run_export']
            original = cal.load_json(path)
            modified = copy.deepcopy(original)
            modified['metadata'].update(update)
            cal.write_json(path, modified)
            with self.subTest(update=update), self.assertRaises(cal.CalibrationError):
                cal.validate_measurements(self.data, self.directory)
            cal.write_json(path, original)
        data = copy.deepcopy(self.data)
        data['servo_samples'][0]['nominal_us'] = 1402
        with self.assertRaisesRegex(cal.CalibrationError, '原始记录'):
            cal.validate_measurements(data, self.directory)

    def test_default_profile_is_never_overwritten(self):
        with self.assertRaisesRegex(cal.CalibrationError, '独立候选'):
            cal.generate_profile(self.directory / 'absent.json', ROOT / 'user/src/calibration_profile.c')
        source = self.directory / 'measurements.json'
        cal.write_json(source, self.data)
        before = source.read_bytes()
        with self.assertRaises(cal.CalibrationError):
            cal.generate_profile(source, source.with_suffix('.c'))
        self.assertEqual(source.read_bytes(), before)

    def test_csv_and_json_export_include_provenance_and_manual_notes(self):
        exported = {'metadata': {'valid': True, 'build_id': 'abc'}, 'records': [], 'manual_measurements': []}
        saved = cal.save_export(exported, self.directory / 'raw.json', [{'left_deg': 12, 'instrument_measured_us': None}])
        self.assertTrue(Path(saved['csv']).exists())
        self.assertEqual(cal.load_json(Path(saved['json']))['manual_measurements'][0]['left_deg'], 12)
        with self.assertRaises(cal.CalibrationError):
            cal.save_export(exported, self.directory / 'raw.json')

    def test_rpm_independent_video_interval_and_quantization_tolerance(self):
        exported = {'metadata': {'valid': True, 'frozen': True, 'kind': cal.KIND_RPM, 'counts_per_rev_left': 100,
                                'markers': [{'index': 0, 'begin_us': 2000000, 'end_us': 2001000}, {'index': 1, 'begin_us': 6000000, 'end_us': 6001000}]},
                    'records': [{'elapsed_us': t, 'total_left': t // 10000, 'total_right': t // 5000}
                                for t in range(0, 8000001, 10000)]}
        reference = {'capture_fps': 240, 'start_frame': 1000, 'end_frame': 1720, 'complete_turns': 3,
                     'wheel': 'left', 'counts_per_rev': 100, 'marker_frames': [760, 1720]}
        result = cal.rpm_reference(exported, reference)
        self.assertAlmostEqual(result['reference_rpm'], 60)
        self.assertAlmostEqual(result['encoder_rpm'], 60)
        self.assertTrue(result['pass'])
        reference['capture_fps'] = 30
        with self.assertRaisesRegex(cal.CalibrationError, '帧率'):
            cal.rpm_reference(exported, reference)

    def test_cli_offline_errors_are_clean_and_never_attach(self):
        source = self.directory / 'no-measurements.json'
        cal.write_json(source, {'provenance': {'real_measurements': False}})
        result = subprocess.run([sys.executable, str(ROOT / 'tools/calibrate_pyocd.py'), 'profile',
                                 '--measurements', str(source), '--output', str(self.directory / 'bad.c')], capture_output=True)
        self.assertEqual(result.returncode, 1)
        self.assertNotIn(b'NameError', result.stderr)
        self.assertNotIn(b'Traceback', result.stderr)
        self.assertFalse((self.directory / 'bad.c').exists())

    def test_exit_recovery_preserves_pending_manual_notes_and_frozen_group(self):
        probe = FakeProbe()
        probe.set_words(364, [10, cal.KIND_SERVO, 1, 1, 0, 1, 2])
        worker = cal.IOWorker(lambda: (cal.MemoryTransport(probe, probe.image, 20, heartbeat=True), lambda: None))
        try:
            notes = [{'type': 'servo', 'left_deg': 1.2, 'right_deg': 1.1}]
            cal.recover_session(worker, self.directory / 'recovery', notes, [], True)
        finally:
            worker.close()
        files = list((self.directory / 'recovery').glob('*.json'))
        self.assertEqual(len(files), 2)
        pending = next(path for path in files if 'pending_measurements' in path.name)
        collection = cal.load_json(pending)
        self.assertFalse(collection['provenance']['real_measurements'])
        self.assertEqual(collection['servo_samples'][0]['left_deg'], 1.2)
        self.assertTrue(Path(collection['servo_samples'][0]['run_export']).exists())

    def test_unknown_start_results_block_new_prepare_and_restart_until_raw_recovery(self):
        for failure in ('ack_lost', 'snapshot_lost'):
            clock = FakeClock()
            probe = FakeProbe(clock)
            probe.set_words(108, [8, 2])
            def started(opcode, seq):
                if opcode == cal.CMD_START:
                    probe.set_words(112, [cal.KIND_SERVO])
                    probe.set_words(364, [seq, cal.KIND_SERVO, 1, 1, 0, 1, 2])
                    if failure == 'snapshot_lost':
                        probe.persistent_snapshot_race = True
                    else:
                        probe.drop_ack = True
            def stopped():
                probe.persistent_snapshot_race = False
            probe.on_command, probe.on_stop = started, stopped
            transport = cal.MemoryTransport(probe, probe.image, 20, heartbeat=True, clock=clock, sleeper=clock.sleep)
            worker = cal.IOWorker(lambda: (transport, lambda: None))
            errors = io.StringIO()
            recovery = self.directory / failure
            try:
                with self.subTest(failure=failure), patch('builtins.input', side_effect=['start', 'RUN', 'prepare count', 'start', 'quit']), patch('sys.stderr', errors), patch('sys.stdout', io.StringIO()):
                    cal.interactive(worker, recovery)
            finally:
                worker.close()
            self.assertEqual(probe.commands, [cal.CMD_START])
            self.assertIn('尚未 export', errors.getvalue())
            self.assertIn('不能重发 START', errors.getvalue())
            raw = [path for path in recovery.glob('*.json') if 'pending_measurements' not in path.name]
            self.assertEqual(len(raw), 1)
            self.assertEqual(cal.load_json(raw[0])['metadata']['run_id'], 1)

    def test_known_negative_start_ack_releases_only_the_pending_evidence_barrier(self):
        clock = FakeClock()
        probe = FakeProbe(clock)
        probe.set_words(108, [8, 2])
        probe.bad_crc = True
        transport = cal.MemoryTransport(probe, probe.image, 20, heartbeat=True, clock=clock, sleeper=clock.sleep)
        worker = cal.IOWorker(lambda: (transport, lambda: None))
        errors = io.StringIO()
        try:
            with patch('builtins.input', side_effect=['start', 'RUN', 'prepare count', 'CONFIRM', 'quit']), patch('sys.stderr', errors), patch('sys.stdout', io.StringIO()):
                cal.interactive(worker, self.directory / 'rejected')
        finally:
            worker.close()
        self.assertNotIn('尚未 export', errors.getvalue())
        self.assertEqual(sum(offset == 36 for offset, *_ in probe.writes), 2)
        self.assertEqual(probe.commands, [])

    def test_float32_node_or_angle_collapse_and_boundary_rounding_are_rejected(self):
        for mode in ('pulse', 'angle', 'bound'):
            data = copy.deepcopy(self.data)
            extra = copy.deepcopy(data['servo_samples'][:6])
            for sample in extra:
                sample['nominal_us'] = 1400.00005 if mode == 'pulse' else 1402
                sample['left_deg'] = 9.99 if mode == 'pulse' else 9.9999999
                sample['right_deg'] = 8.99 if mode == 'pulse' else 8.9999999
                name = self.directory / f'{mode}_{sample["direction"]}_{sample["repeat"]}.json'
                sample['run_export'] = str(name)
                cal.write_json(name, {'metadata': {'build_id': data['firmware_build_id'], 'kind': cal.KIND_SERVO,
                                                   'frozen': True, 'valid': True},
                                     'records': [{'servo_nominal_us': sample['nominal_us']}]})
            if mode == 'bound':
                for sample in data['servo_samples'][:6]:
                    sample['left_deg'] = 88.999999
                for sample in data['servo_samples']:
                    if sample['role'] == 'validation':
                        sample['left_deg'] = 88.999999 / 2
                extra = []
            data['servo_samples'].extend(extra)
            with self.subTest(mode=mode), self.assertRaisesRegex(cal.CalibrationError, 'float32'):
                cal.validate_measurements(data, self.directory)

    def test_elf_parser_rejects_broken_input_and_reads_actual_diagnostic_objects(self):
        broken = self.directory / 'broken.axf'
        broken.write_bytes(b'not-an-elf')
        with self.assertRaises(cal.CalibrationError):
            cal.FirmwareImage.from_elf(broken)
        found = list((ROOT / 'mdk/Objects').glob('calibration_*/*.axf'))
        for path in found:
            with self.subTest(path=path.name):
                image = cal.FirmwareImage.from_elf(path)
                self.assertEqual(len(image.build_id), 8)
                self.assertTrue(cal.DTCM_START <= image.mailbox_address < cal.DTCM_END)
                self.assertEqual(len(image.sha256), 64)


if __name__ == '__main__':
    unittest.main(verbosity=2)
