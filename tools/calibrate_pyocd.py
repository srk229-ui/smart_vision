"""运行态 pyOCD 标定工具；仅交互人工确认可以启动，不下载或暂停固件。"""

from __future__ import annotations

import argparse
from concurrent.futures import Future
import csv
from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
import queue
import secrets
import shlex
import statistics
import struct
import sys
import threading
import time
from typing import Any
import zlib


class CalibrationError(RuntimeError):
    """固件身份、协议或实测证据不满足标定条件。"""


class AckTimeout(CalibrationError):
    """确认丢失后已请求停车；不得自动重发启动。"""


class CommandRejected(CalibrationError):
    """固件已明确返回负确认，保证该请求没有被接受执行。"""


MAGIC = 0x43414C33
ABI = 1
MAILBOX_BYTES = 416
RECORD = struct.Struct('<IIIhhhhHHHHf')
LOG_CAPACITY = 1024
DTCM_START = 0x20000000
DTCM_END = 0x20070000
KIND_COUNT, KIND_SERVO, KIND_RPM = 4, 5, 6
CMD_PREPARE_COUNT, CMD_PREPARE_SERVO, CMD_PREPARE_RPM = 1, 2, 3
CMD_START, CMD_PULSE, CMD_COUNT_START, CMD_COUNT_END = 4, 5, 6, 7
CMD_MARKER, CMD_APPLY_ENCODER, CMD_QUERY_US, CMD_QUERY_ANGLE = 8, 9, 10, 11
CMD_RELEASE, CMD_CLEAR_FAULT = 12, 13
CMD_SERVO_END = 14
ACK_NAMES = ('OK', 'BAD_CRC', 'BAD_SESSION', 'BAD_STATE', 'BAD_ARGUMENT', 'STALE_VERSION',
             'CONTROL_REJECTED', 'NOT_STATIONARY', 'MEASUREMENT_INVALID', 'UNCALIBRATED', 'BAD_COMMAND', 'STOPPED')
PHASE_NAMES = ('IDLE', 'PREPARED_COUNT', 'PREPARED_SERVO', 'PREPARED_RPM',
               'ACTIVE_COUNT', 'ACTIVE_SERVO', 'ACTIVE_RPM', 'COMPLETE', 'ABORTED')
END_NAMES = ('NONE', 'MANUAL', 'STOP', 'LINK_TIMEOUT', 'TIME_LIMIT', 'MEASUREMENT', 'CONTROL', 'LOG_FULL')
ATTACH_OPTIONS = {'target_override': 'mimxrt1064', 'connect_mode': 'attach', 'auto_unlock': False,
                  'cache.enable_memory': False, 'cache.enable_register': False,
                  'resume_on_disconnect': False, 'no_config': True, 'frequency': 1000000}


def float_word(value: Any) -> int:
    try:
        return struct.unpack('<I', struct.pack('<f', finite(value, '命令参数')))[0]
    except (OverflowError, struct.error) as exc:
        raise CalibrationError('命令参数超出 float32 有限范围') from exc


def word_float(value: int) -> float:
    return struct.unpack('<f', struct.pack('<I', value))[0]


def pair_signed(value: int) -> tuple[int, int]:
    return struct.unpack('<hh', struct.pack('<I', value))


def int64(low: int, high: int) -> int:
    return struct.unpack('<q', struct.pack('<II', low, high))[0]


def next_sequence(value: int) -> int:
    return (value + 1) & 0xFFFFFFFF or 1


@dataclass(frozen=True)
class FirmwareImage:
    mailbox_address: int
    log_address: int
    stamp_address: int
    stamp: bytes
    sha256: str

    @property
    def build_crc(self) -> int:
        return zlib.crc32(self.stamp.rstrip(b'\0'))

    @property
    def build_id(self) -> str:
        return f'{self.build_crc:08x}'

    def validate(self) -> None:
        ranges = [(self.mailbox_address, MAILBOX_BYTES), (self.log_address, LOG_CAPACITY * RECORD.size)]
        for address, size in ranges:
            if address % 4 or not DTCM_START <= address < address + size <= DTCM_END:
                raise CalibrationError('标定邮箱或日志地址不在当前工程 DTCM 内')
        if not (ranges[0][0] + ranges[0][1] <= ranges[1][0] or ranges[1][0] + ranges[1][1] <= ranges[0][0]):
            raise CalibrationError('邮箱和日志区重叠')
        if not 1 < len(self.stamp) <= 128 or not self.stamp.endswith(b'\0') or b'\0' in self.stamp[:-1]:
            raise CalibrationError('ELF 构建标识无效')
        if not ((0x70000000 <= self.stamp_address < self.stamp_address + len(self.stamp) <= 0x70400000)
                or (0x80000000 <= self.stamp_address < self.stamp_address + len(self.stamp) <= 0x82000000)):
            raise CalibrationError('构建标识不在当前工程只读执行区')

    @classmethod
    def from_elf(cls, path: Path) -> FirmwareImage:
        """只接受当前 ELF 的全局对象符号和初始构建标识，禁止猜测 RAM 地址。"""
        from elftools.common.exceptions import ELFError
        try:
            return cls._parse_elf(path)
        except ELFError as exc:
            raise CalibrationError('AXF/ELF 数据不完整或格式损坏') from exc

    @classmethod
    def _parse_elf(cls, path: Path) -> FirmwareImage:
        from elftools.elf.elffile import ELFFile
        from elftools.elf.sections import SymbolTableSection
        raw = path.read_bytes()
        with path.open('rb') as stream:
            elf = ELFFile(stream)
            if elf.elfclass != 32 or not elf.little_endian or elf['e_machine'] != 'EM_ARM':
                raise CalibrationError('仅支持 ARM32 小端 AXF/ELF')
            expected = {'calibration_debug_mailbox': MAILBOX_BYTES,
                        'calibration_debug_log': LOG_CAPACITY * RECORD.size,
                        'calibration_debug_build_stamp': None}
            symbols = {}
            for section in elf.iter_sections():
                if not isinstance(section, SymbolTableSection):
                    continue
                for symbol in section.iter_symbols():
                    if symbol.name not in expected or symbol['st_info']['bind'] != 'STB_GLOBAL':
                        continue
                    if symbol['st_info']['type'] != 'STT_OBJECT' or not isinstance(symbol['st_shndx'], int):
                        raise CalibrationError('标定符号不是已定义的全局对象')
                    if symbol.name in symbols:
                        raise CalibrationError('ELF 含重复标定符号')
                    size = expected[symbol.name]
                    if size is not None and symbol['st_size'] != size:
                        raise CalibrationError(symbol.name + ' 大小与工具 ABI 不一致')
                    owner = elf.get_section(symbol['st_shndx'])
                    address = symbol['st_value']
                    if not owner['sh_flags'] & 2 or not owner['sh_addr'] <= address < address + symbol['st_size'] <= owner['sh_addr'] + owner['sh_size']:
                        raise CalibrationError('标定对象超出其 ELF 分配段')
                    if size is not None and not owner['sh_flags'] & 1:
                        raise CalibrationError('邮箱或日志不是可写 RAM 对象')
                    symbols[symbol.name] = symbol
            if set(symbols) != set(expected):
                raise CalibrationError('ELF 缺少标定对象；请选择已启用 CALIBRATION_DEBUG_ENABLE 的诊断版')
            stamp_symbol = symbols['calibration_debug_build_stamp']
            section = elf.get_section(stamp_symbol['st_shndx'])
            offset = stamp_symbol['st_value'] - section['sh_addr']
            size = stamp_symbol['st_size']
            if section['sh_type'] == 'SHT_NOBITS' or offset < 0 or not 1 < size <= 128 or offset + size > section['sh_size']:
                raise CalibrationError('ELF 构建标识缺少可读取的初始化内容')
            stamp = section.data()[offset:offset + size]
            result = cls(symbols['calibration_debug_mailbox']['st_value'], symbols['calibration_debug_log']['st_value'],
                         stamp_symbol['st_value'], stamp, hashlib.sha256(raw).hexdigest())
            result.validate()
            return result


def decode_mailbox(raw: bytes) -> dict:
    if len(raw) != MAILBOX_BYTES:
        raise CalibrationError('邮箱读取长度错误')
    words = struct.unpack('<104I', raw)
    s = words[32:64]
    raw_left, raw_right = pair_signed(s[3])
    delta_left, delta_right = pair_signed(s[4])
    result = {'features': words[6], 'command_ready': bool(words[7]), 'stop_ack_seq': words[31],
              'ack_seq': words[24], 'ack_status': words[25], 'ack_session': words[26], 'prepare_token': words[27],
              'phase': words[28], 'phase_name': PHASE_NAMES[words[28]] if words[28] < len(PHASE_NAMES) else 'UNKNOWN',
              'end_reason': words[29], 'end_name': END_NAMES[words[29]] if words[29] < len(END_NAMES) else 'UNKNOWN',
              'snapshot_seq': words[30], 'now_us': s[0], 'sample_id': s[1], 'dt_us': s[2],
              'raw_left': raw_left, 'raw_right': raw_right, 'delta_left': delta_left, 'delta_right': delta_right,
              'total_left': int64(s[5], s[6]), 'total_right': int64(s[7], s[8]), 'flags': s[11],
              'rpm100_left': word_float(s[9]) if s[11] & 4 else None,
              'rpm100_right': word_float(s[10]) if s[11] & 4 else None,
              'control_state': s[12], 'control_mode': s[13], 'control_fault': s[14], 'params_version': s[15],
              'target_left': word_float(s[16]), 'target_right': word_float(s[17]),
              'pwm_left': s[18] & 0xFFFF, 'pwm_right': s[18] >> 16, 'servo_duty': s[19],
              'servo_request_us': word_float(s[20]), 'servo_nominal_us': word_float(s[21]),
              'servo_enabled': bool(s[11] & 16), 'heartbeat_age_us': s[22], 'active_elapsed_us': s[23],
              'stationary_us': s[24], 'map_version': s[28], 'map_valid': bool(s[29]),
              'query_0': word_float(s[30]), 'query_1': word_float(s[31]),
              'count_start_us': words[64], 'count_end_us': words[65],
              'count_start_left': int64(words[66], words[67]), 'count_start_right': int64(words[68], words[69]),
              'count_end_left': int64(words[70], words[71]), 'count_end_right': int64(words[72], words[73]),
              'log_seq': words[90], 'run_id': words[91], 'kind': words[92], 'log_params_version': words[93],
              'calibration_version': words[94], 'log_count': words[95], 'frozen': bool(words[96]), 'log_end_reason': words[97],
              'rpm500_left': word_float(words[98]) if s[11] & 8 else None,
              'rpm500_right': word_float(words[99]) if s[11] & 8 else None,
              'measurement_version': words[100], 'counts_per_rev_left': word_float(words[101]),
              'counts_per_rev_right': word_float(words[102]), 'nominal_us_per_duty': word_float(words[103])}
    result['markers'] = [{'index': index, 'begin_us': (words[74 + index] - words[64]) & 0xFFFFFFFF,
                          'end_us': (words[82 + index] - words[64]) & 0xFFFFFFFF}
                         for index in range(8) if s[27] & (1 << index)]
    return result


class MemoryTransport:
    """单线程运行态协议；所有主机写入限制在四类明确字段，STOP 独立提交。"""

    def __init__(self, probe: Any, image: FirmwareImage, session_id: int, heartbeat: bool = False,
                 clock=time.monotonic, sleeper=time.sleep, required_features: int = 0):
        image.validate()
        if not 0 < session_id <= 0xFFFFFFFF:
            raise CalibrationError('主机会话必须是非零 uint32')
        self.probe, self.image, self.session_id = probe, image, session_id
        self.clock, self.sleeper = clock, sleeper
        self.heartbeat_enabled = heartbeat
        self.stop_requested = threading.Event()
        self.next_heartbeat = 0.0
        self.heartbeat_sequence = 0
        self.validated = False
        self.required_features = required_features
        self.validate_identity()
        self.stop_sequence = self.read_word(80)

    def read(self, offset: int, count: int) -> bytes:
        if not 0 <= offset < offset + count <= MAILBOX_BYTES:
            raise CalibrationError('邮箱读取超界')
        return bytes(self.probe.read_memory_block8(self.image.mailbox_address + offset, count))

    def read_word(self, offset: int) -> int:
        return struct.unpack('<I', self.read(offset, 4))[0]

    def write(self, offset: int, payload: bytes) -> None:
        # payload不覆盖commit序号；ACK、快照、日志和元数据从不允许主机写入。
        allowed = ((32, 36), (36, 40), (40, 80), (80, 84), (84, 88), (88, 92))
        if not self.validated or not any(begin <= offset < offset + len(payload) <= end for begin, end in allowed):
            raise CalibrationError('写入不在标定协议白名单内')
        if offset % 4 or len(payload) % 4:
            raise CalibrationError('仅允许完整对齐的主机字段写入')
        if len(payload) == 4:
            self.probe.write_memory(self.image.mailbox_address + offset, struct.unpack('<I', payload)[0], 32)
        else:
            self.probe.write_memory_block8(self.image.mailbox_address + offset, list(payload))
        self.probe.flush()

    def validate_identity(self) -> None:
        magic, abi, size, build, record_size, capacity, features, _ = struct.unpack('<8I', self.read(0, 32))
        if (magic, abi, size, build, record_size, capacity) != (MAGIC, ABI, MAILBOX_BYTES, self.image.build_crc, RECORD.size, LOG_CAPACITY) or not features & 1:
            raise CalibrationError('运行固件与 ELF、ABI、构建标识或对象大小不符；拒绝所有写入')
        if features & self.required_features != self.required_features:
            raise CalibrationError('需要新版舵机无时限事件记录固件；未发送心跳或测量命令')
        stamp = bytes(self.probe.read_memory_block8(self.image.stamp_address, len(self.image.stamp)))
        if stamp != self.image.stamp:
            raise CalibrationError('运行固件的构建字符串与 ELF 不符')
        tail = bytes(self.probe.read_memory_block8(self.image.log_address + LOG_CAPACITY * RECORD.size - 4, 4))
        if len(tail) != 4:
            raise CalibrationError('DTCM 日志末端不可完整读取，拒绝标定写入')
        self.validated = True

    def ensure_live_identity(self) -> None:
        expected = (MAGIC, ABI, MAILBOX_BYTES, self.image.build_crc, RECORD.size, LOG_CAPACITY)
        if struct.unpack('<6I', self.read(0, 24)) != expected:
            self.validated = False
            raise CalibrationError('连接期间固件身份改变；停止全部主机写入且不自动重连')

    def maintenance(self) -> None:
        stopping = self.stop_requested.is_set()
        heartbeat_due = self.heartbeat_enabled and self.clock() >= self.next_heartbeat
        if stopping or heartbeat_due:
            self.ensure_live_identity()
        if stopping:
            self.stop_requested.clear()
            self.stop_sequence = next_sequence(self.stop_sequence)
            self.write(80, struct.pack('<I', self.stop_sequence))
        if heartbeat_due:
            self.heartbeat_sequence = next_sequence(self.heartbeat_sequence)
            self.write(84, struct.pack('<I', self.session_id))
            self.write(88, struct.pack('<I', self.heartbeat_sequence))
            self.next_heartbeat = self.clock() + 0.1

    def stop(self) -> None:
        self.stop_requested.set()
        self.maintenance()

    def stop_and_wait(self, timeout: float = 2.0) -> dict:
        """重开先确认本次独立STOP已消费且实际零命令周期已冻结，避免取消下一PREPARE。"""
        if not self.snapshot()['features'] & 2:
            raise CalibrationError('直接重开需要支持STOP确认的新诊断固件，请烧录本轮版本')
        self.stop()
        wanted = self.stop_sequence
        deadline = self.clock() + timeout
        while self.clock() < deadline:
            state = self.snapshot()
            if (state['stop_ack_seq'] == wanted and state['frozen'] and
                    state['phase'] not in (1, 2, 3, 4, 5, 6) and not state['pwm_left'] and
                    not state['pwm_right'] and not state['servo_enabled'] and not state['servo_duty']):
                return state
            self.sleeper(0.02)
        raise CalibrationError('本次STOP尚未确认或冻结；未丢弃或重新启动，请核对状态')

    def snapshot(self, attempts: int = 50) -> dict:
        self.ensure_live_identity()
        for _ in range(attempts):
            self.maintenance()
            before = self.read_word(120)
            if before & 1:
                self.sleeper(0.002)
                continue
            raw = self.read(0, MAILBOX_BYTES)
            after = self.read_word(120)
            if before == after == struct.unpack_from('<I', raw, 120)[0]:
                return decode_mailbox(raw)
            self.sleeper(0.002)
        raise CalibrationError('快照持续更新或不可用，未接受混合周期数据')

    def command(self, opcode: int, args: list[int] | None = None, timeout: float = 0.75) -> dict:
        self.ensure_live_identity()
        interrupted = self.stop_requested.is_set()
        self.maintenance()
        if interrupted:
            raise CalibrationError('独立 STOP 取消本次普通命令，请重新读取状态')
        args = list(args or [])
        if not 1 <= opcode <= CMD_SERVO_END or len(args) > 8 or any(isinstance(value, bool) or not isinstance(value, int) or not 0 <= value <= 0xFFFFFFFF for value in args):
            raise CalibrationError('非法操作码或参数位型')
        old_seq = self.read_word(36)
        if old_seq != self.read_word(96):
            raise CalibrationError('上个命令尚未确认，先 STOP 并读取状态，不覆盖在途载荷')
        seq = next_sequence(old_seq)
        payload = struct.pack('<11I', self.session_id, seq, opcode, *(args + [0] * (8 - len(args))))
        self.write(32, payload[:4])
        self.write(40, payload[8:] + struct.pack('<I', zlib.crc32(payload)))
        if self.stop_requested.is_set():
            self.maintenance()
            raise CalibrationError('STOP 已取消未提交载荷，未执行普通命令')
        # 序号是唯一提交点，完整载荷和CRC刷入探针之后再发布。
        self.write(36, struct.pack('<I', seq))
        deadline = self.clock() + timeout
        while self.clock() < deadline:
            self.maintenance()
            ack1 = self.read_word(96)
            status, session = struct.unpack('<II', self.read(100, 8))
            ack2 = self.read_word(96)
            if ack1 == ack2 == seq and session == self.session_id:
                if status:
                    raise CommandRejected('固件拒绝命令：' + (ACK_NAMES[status] if status < len(ACK_NAMES) else str(status)))
                try:
                    return self.snapshot()
                except Exception as exc:
                    if opcode in (CMD_START, CMD_COUNT_START):
                        try:
                            self.stop()
                        except Exception:
                            self.heartbeat_enabled = False
                            raise CalibrationError('START 已接受但快照失败，STOP 未确认提交；停止心跳，保留待取回试验') from exc
                        raise CalibrationError('START 已接受但快照失败；已提交独立 STOP，须冻结导出后重新准备') from exc
                    raise
            self.sleeper(0.005)
        self.stop()
        # 即便确认只是在链路上丢失，启动也不重发，以防相同动作运行两次。
        raise AckTimeout('命令确认超时；已提交独立 STOP，请核对状态后重新人工准备')

    def frozen_export(self) -> dict:
        state = self.snapshot()
        if not state['frozen'] or state['log_seq'] & 1 or not 0 <= state['log_count'] <= LOG_CAPACITY:
            raise CalibrationError('仅导出已冻结且容量合法的试验')
        count = state['log_count']
        chunks = []
        for index in range(0, count * RECORD.size, 1024):
            self.maintenance()
            expected_bytes = min(1024, count * RECORD.size - index)
            chunk = bytes(self.probe.read_memory_block8(self.image.log_address + index, expected_bytes))
            if len(chunk) != expected_bytes:
                raise CalibrationError('日志分块发生短读或超读，未接受不完整原始记录')
            chunks.append(chunk)
        raw_records = b''.join(chunks)
        if len(raw_records) != count * RECORD.size:
            raise CalibrationError('日志总字节数与冻结元数据不一致')
        after = self.snapshot()
        keys = ('log_seq', 'run_id', 'kind', 'log_params_version', 'calibration_version', 'log_count', 'frozen', 'log_end_reason')
        if any(state[key] != after[key] for key in keys):
            raise CalibrationError('导出期间日志换组；未接受混合试验数据')
        rows = []
        coefficient = finite(state['nominal_us_per_duty'], '固件占空计数到名义脉宽系数')
        if coefficient <= 0:
            raise CalibrationError('固件未发布有效的名义脉宽换算系数')
        left, right = state['count_start_left'], state['count_start_right']
        servo_events = state['kind'] == KIND_SERVO and bool(state['features'] & 8)
        for values in RECORD.iter_unpack(raw_records):
            fields = ('sample_id', 'now_us', 'dt_us', 'raw_left', 'raw_right', 'delta_left', 'delta_right',
                      'pwm_left', 'pwm_right', 'servo_duty', 'flags', 'servo_request_us')
            row = dict(zip(fields, values))
            left += row['delta_left']
            right += row['delta_right']
            row.update(elapsed_us=(row['now_us'] - state['count_start_us']) & 0xFFFFFFFF,
                       total_left=None if servo_events else left, total_right=None if servo_events else right,
                       servo_nominal_us=row['servo_duty'] * coefficient)
            rows.append(row)
        if len(rows) != count:
            raise CalibrationError('解码条数与冻结元数据不一致')
        valid_end = state['log_end_reason'] == 1 if state['kind'] == KIND_COUNT else state['log_end_reason'] in ((1, 2, 6) if servo_events else (2, 4, 6))
        valid_records = (state['kind'] == KIND_COUNT or bool(rows)) and all(row['flags'] & 1 and not row['flags'] & 32 and 5000 < row['dt_us'] < 15000 for row in rows)
        if servo_events and rows:
            valid_records = (valid_records and all(not row['pwm_left'] and not row['pwm_right'] for row in rows) and
                             not rows[-1]['servo_duty'] and not rows[-1]['flags'] & 16 and
                             rows[-1]['now_us'] == state['count_end_us'])
        for previous, current in zip(rows, rows[1:]):
            sample_gap = (current['sample_id'] - previous['sample_id']) & 0xFFFFFFFF
            time_gap = (current['now_us'] - previous['now_us']) & 0xFFFFFFFF
            # 稀疏事件允许跳过稳定周期；足够短的间隔仍须符合每周期5至15ms，长间隔只保留GPT模差。
            if (not 0 < sample_gap < 0x80000000 or
                    (sample_gap == 1 and time_gap != current['dt_us']) or
                    (servo_events and sample_gap > 1 and sample_gap * 15000 < 2**32 and
                     not (sample_gap - 1) * 5001 + current['dt_us'] <= time_gap <= (sample_gap - 1) * 14999 + current['dt_us']) or
                    (not servo_events and (sample_gap != 1 or time_gap != current['dt_us']))):
                valid_records = False
        metadata = {key: state[key] for key in ('run_id', 'kind', 'log_params_version', 'calibration_version', 'map_version',
                                               'frozen', 'log_end_reason', 'count_start_us', 'count_end_us', 'markers')}
        metadata.update(build_id=self.image.build_id, build_stamp=self.image.stamp.rstrip(b'\0').decode('ascii'),
                        elf_sha256=self.image.sha256, abi=ABI, record_count=count, session_id=self.session_id,
                        count_delta_left=state['count_end_left'] - state['count_start_left'],
                        count_delta_right=state['count_end_right'] - state['count_start_right'],
                        valid=bool(valid_end and valid_records and not state['control_fault']),
                        label='运行态原始命令与计数；转角和实际脉宽仍需人工/仪器测量')
        for wheel in ('left', 'right'):
            metadata['counts_per_rev_' + wheel] = state['counts_per_rev_' + wheel] if state['measurement_version'] == state['calibration_version'] else None
        metadata['recording_mode'] = 'servo_events' if servo_events else 'continuous' if state['kind'] != KIND_COUNT else 'endpoints'
        if servo_events:
            metadata['timestamp_semantics'] = 'GPT低32位us；elapsed_us为距起点模2^32差值；事件之间无逐周期日志'
        if rows and not servo_events and (left != state['count_end_left'] or right != state['count_end_right']):
            metadata['valid'] = False
            metadata['count_mismatch'] = True
        return {'schema_version': 1, 'metadata': metadata, 'snapshot': state, 'records': rows, 'manual_measurements': []}


class IOWorker:
    """唯一探针 I/O 线程；人工输入停留时仍更新心跳，STOP 可打断 ACK 等待。"""

    def __init__(self, factory):
        self.tasks = queue.Queue()
        self.ready = Future()
        self.closing = threading.Event()
        self.stop_epoch = 0
        self.transport = None
        self.thread = threading.Thread(target=self._run, args=(factory,), name='calibration-pyocd-io', daemon=True)
        self.thread.start()
        try:
            self.ready.result(timeout=15)
        except TimeoutError as exc:
            self.closing.set()
            self.thread.join(timeout=3)
            raise CalibrationError('建立探针会话超时；已请求关闭I/O线程，未发送测量命令') from exc

    def _run(self, factory) -> None:
        closer = lambda: None
        try:
            self.transport, closer = factory()
            self.ready.set_result(True)
            while not self.closing.is_set():
                self.transport.maintenance()
                try:
                    future, method, args, epoch = self.tasks.get(timeout=0.02)
                except queue.Empty:
                    continue
                if not future.set_running_or_notify_cancel():
                    continue
                if method == 'command' and epoch != self.stop_epoch:
                    future.set_exception(CalibrationError('STOP 已取消排队命令，请重新读取状态'))
                    continue
                try:
                    future.set_result(getattr(self.transport, method)(*args))
                except CalibrationError as exc:
                    future.set_exception(exc)
                except Exception as exc:
                    future.set_exception(CalibrationError('探针通信中断，停止心跳且不自动重连：' + str(exc)))
                    break
        except Exception as exc:
            if not self.ready.done():
                self.ready.set_exception(exc)
        finally:
            self.closing.set()
            while not self.tasks.empty():
                future, _, _, _ = self.tasks.get_nowait()
                if not future.done():
                    future.set_exception(CalibrationError('探针会话已经结束'))
            if self.transport is not None and self.transport.heartbeat_enabled:
                try:
                    self.transport.stop()
                except Exception:
                    pass
            closer()

    def call(self, method: str, *args):
        if self.closing.is_set():
            raise CalibrationError('会话已结束，请重新只读核对后人工准备')
        future = Future()
        self.tasks.put((future, method, args, self.stop_epoch))
        return future.result(timeout=15)

    def request_stop(self) -> None:
        self.stop_epoch += 1
        if self.transport is not None:
            self.transport.stop_requested.set()

    def close(self) -> None:
        if self.transport is not None and self.transport.heartbeat_enabled:
            self.request_stop()
        self.closing.set()
        self.thread.join(timeout=3)
        if self.thread.is_alive():
            raise CalibrationError('探针关闭仍未完成；固件心跳保护应停止输出，需现场核对')


def attach_factory(image: FirmwareImage, probe_id: str, heartbeat: bool, required_features: int = 0):
    """选精确序列号，在 worker 内打开运行态连接，禁用项目配置和用户调试脚本。"""
    def factory():
        import pyocd
        from pyocd.core.helpers import ConnectHelper
        from pyocd.core.session import Session
        from pyocd.core.target import Target
        if pyocd.__version__ != '0.45.1':
            raise CalibrationError('本工具已核对 pyOCD 0.45.1；其他版本需重新核对 attach 语义')
        class SafeAttachSession(Session):
            def _load_user_script(self):
                # 默认pyocd.py也可能包含reset/halt，标定工具始终不加载这些脚本。
                return None
        probes = [probe for probe in ConnectHelper.get_all_connected_probes(blocking=False)
                  if probe.unique_id == probe_id]
        if len(probes) != 1:
            raise CalibrationError('未找到唯一精确匹配的探针序列号')
        session = SafeAttachSession(probes[0], auto_open=False, options=dict(ATTACH_OPTIONS))
        try:
            session.open()
            target = session.target
            if target.get_state() != Target.State.RUNNING:
                raise CalibrationError('CPU 未在运行；工具不会自动恢复、复位或暂停')
            sid = int.from_bytes(secrets.token_bytes(4), 'little') or 1
            transport = MemoryTransport(target, image, sid, heartbeat=heartbeat, required_features=required_features)
            first = transport.snapshot()
            time.sleep(0.03)
            second = transport.snapshot()
            if first['sample_id'] == second['sample_id'] or first['now_us'] == second['now_us']:
                raise CalibrationError('GPT/PIT 未持续推进；拒绝运行态标定')
            if heartbeat and second['features'] & 4:
                # PIT开始不代表上电联网/相机初始化已返回，主循环就绪后才接收交互命令。
                deadline = time.monotonic() + 10.0
                while not second['command_ready']:
                    if time.monotonic() >= deadline:
                        raise CalibrationError('固件主循环尚未就绪；初始化未结束，未发送测量命令')
                    time.sleep(0.05)
                    second = transport.snapshot()
            return transport, session.close
        except Exception:
            session.close()
            raise
    return factory


def save_export(exported: dict, output: Path, notes: list[dict] | None = None) -> dict:
    """保存完整 JSON 和同名 CSV；32 字节原始计数按固件起点累计还原。"""
    output = output.with_suffix('.json').resolve()
    if output.exists() or output.with_suffix('.csv').exists():
        raise CalibrationError('原始导出文件已经存在，请使用新的组名，保留之前记录')
    exported['manual_measurements'] = list(notes or [])
    write_json(output, exported)
    csv_path = output.with_suffix('.csv')
    fields = ('sample_id', 'now_us', 'elapsed_us', 'dt_us', 'raw_left', 'raw_right', 'delta_left', 'delta_right',
              'total_left', 'total_right', 'pwm_left', 'pwm_right', 'servo_duty', 'servo_request_us', 'servo_nominal_us', 'flags')
    with csv_path.open('w', newline='', encoding='utf-8-sig') as stream:
        writer = csv.DictWriter(stream, fields)
        writer.writeheader()
        writer.writerows(exported['records'])
    return {'json': str(output), 'csv': str(csv_path), 'metadata': exported['metadata']}


INTERACTIVE_HELP = """命令：
  status                                      只读状态和两种窗口 RPM
  prepare count                               人工确认手转准备，输出均关闭
  prepare servo <center_us> <min_us> <max_us> <rate_us_s>
  prepare rpm <left_counts_10ms> <right_counts_10ms>
  start                                       每次显示条件，人工另输入 RUN 才提交
  restart                                     丢弃未保存手转/舵机组重开；舵机另输入RUN，不用于RPM
  pulse <us>                                  活动舵机单点，新版固件无固定时限
  end                                         正常停止舵机，或手转静止至少100ms后结束；随后export
  stop | clear | release                      停止／人工清故障／释放准备
  query-us <us> | query-angle <left|right> <deg>  表推算查询
  apply-counts <measurement.json>              验证计数实测后停车应用两轮 N
  angle <left_deg> <right_deg> <increasing|decreasing> <1|2|3> <table|validation> [measured_us]
  count-note <left|right> <forward|reverse> <turns>
  export <name.json>                           冻结后导出原始记录及人工笔记
  collect <measurement.json>                   汇总本次导出的人工条目；来源/零点需人工补齐
  quit                                        先停止再关闭连接
角度以直行零点为基准、左转为正；日志脉宽是占空比反算值，实际脉宽选填。"""

COUNT_HELP = """手转计数（执行器输出关闭，单组无固定时限）：
  start                          开始一组；自动完成手转准备
  end                            两轮静止100ms后结束，显示左右净计数
  restart                        停止并丢弃未保存的当前组，重新开始
  set skip-save on|off            设置是否允许start跳过上一组未保存数据
  status | stop                  查看状态／停止
  count-note left|right forward|reverse 圈数
  export 路径.json | collect 集合.json
  quit                           停止并退出，自动保存未导出的当前组
restart丢弃后无法恢复未导出的原始组；已导出的文件不受影响。"""


def confirm(prompt: str, token: str = 'CONFIRM') -> bool:
    return input(prompt + f'，请输入 {token}：').strip() == token


def recover_session(worker: IOWorker, directory: Path, notes: list[dict], collected: list[dict], started: bool) -> None:
    """退出先停车，保存尚未导出的原始组；断线时至少保留人工笔记及失败原因。"""
    identity = worker.transport.image.build_id
    suffix = secrets.token_hex(4)
    pending = {'schema_version': 1, 'version': 1, 'firmware_build_id': identity,
               'provenance': {'real_measurements': False}, 'encoder_runs': [], 'servo_samples': []}
    if started or notes:
        try:
            worker.call('stop')
            deadline = time.monotonic() + 0.5
            while not worker.call('snapshot')['frozen']:
                if time.monotonic() >= deadline:
                    raise CalibrationError('退出停车后日志尚未冻结，需现场核对')
                time.sleep(0.02)
            exported = worker.call('frozen_export')
            saved = save_export(exported, directory / f'{identity}_{exported["metadata"]["run_id"]:08x}_{suffix}.json', notes)
            collected.extend({**note, 'run_export': saved['json']} for note in notes)
            print('退出已停车并保存本组原始记录：', saved['json'])
        except (CalibrationError, OSError) as exc:
            pending['unexported_manual_notes'] = list(notes)
            pending['recovery_error'] = str(exc)
            print('本组原始日志未取回；保留人工笔记并需现场核对：', exc, file=sys.stderr)
    if collected or pending.get('unexported_manual_notes') or pending.get('recovery_error'):
        for note in collected:
            pending['encoder_runs' if note['type'] == 'encoder' else 'servo_samples'].append(note)
        path = directory / f'{identity}_pending_measurements_{suffix}.json'
        write_json(path, pending)
        print('待补齐来源的测量集合：', path.resolve())


def prepare_count_start(worker: IOWorker, discard: bool = False) -> dict:
    """仅对无输出手转组停止、等待冻结和重新准备，不重发任何START。"""
    state = worker.call('snapshot')
    if not state['features'] & 2:
        raise CalibrationError('计数专用入口需要新版诊断固件；旧固件仍有120秒上限')
    if state['phase'] in (2, 3, 5, 6) or state['kind'] not in (0, KIND_COUNT):
        raise CalibrationError('只能丢弃或重新开始手转组；舵机/RPM组仍需冻结导出')
    if discard:
        state = worker.call('stop_and_wait')
    elif state['phase'] == KIND_COUNT:
        raise CalibrationError('手转组正在测量；请先end，或用restart丢弃后重开')
    if (state['control_state'] != 1 or state['control_fault'] or state['pwm_left'] or
            state['pwm_right'] or state['servo_enabled'] or state['servo_duty']):
        raise CalibrationError('手转开始需要无故障READY及零执行器输出；清故障不会自动开始')
    if state['stationary_us'] < 100000:
        raise CalibrationError('两轮须连续静止至少100ms，再输入start或restart')
    if state['phase'] != 1:
        state = worker.call('command', CMD_PREPARE_COUNT, [1])
    return state


def prepare_servo_start(worker: IOWorker, values: list[int], discard: bool = False) -> dict:
    """重用已人工确认的舵机范围；丢弃前匹配STOP确认与实际零输出，不执行START。"""
    state = worker.call('snapshot')
    if not state['features'] & 8:
        raise CalibrationError('舵机start/end/restart需要新版无时限事件记录固件')
    if state['phase'] in (1, 3, 4, 6) or state['kind'] not in (0, KIND_SERVO):
        raise CalibrationError('舵机重开不能丢弃手转或RPM组')
    if discard:
        state = worker.call('stop_and_wait')
    elif state['phase'] == KIND_SERVO:
        raise CalibrationError('舵机组仍在测量；先end或用restart丢弃重开')
    if (state['control_state'] != 1 or state['control_fault'] or state['pwm_left'] or
            state['pwm_right'] or state['servo_enabled'] or state['servo_duty']):
        raise CalibrationError('舵机准备需要无故障READY及零输出；restart不能清故障')
    if state['phase'] != 2:
        if not state['frozen']:
            raise CalibrationError('上一组尚未完成零输出冻结')
        state = worker.call('command', CMD_PREPARE_SERVO, [*values, state['params_version'], 1])
    return state


def interactive(worker: IOWorker, recovery_directory: Path, *, count_only: bool = False,
                skip_save: bool = False, servo_only: bool = False) -> None:
    if servo_only and not worker.call('snapshot')['features'] & 8:
        raise CalibrationError('舵机专用入口需要新版无时限事件记录固件')
    print(COUNT_HELP if count_only else INTERACTIVE_HELP)
    if servo_only:
        print('舵机专用入口：先prepare servo确认范围；start/restart后输入RUN，end停止，无固定时限。')
    if count_only:
        print('跳过未保存手转组：' + ('开启' if skip_save else '关闭'))
    notes = []
    exported_notes = []
    started = False
    servo_values = None
    servo_target = None
    while True:
        try:
            parts = shlex.split(input('标定> '), posix=False)
            parts = [part.strip('"') for part in parts]
            if not parts:
                continue
            name, args = parts[0], parts[1:]
            if name in ('quit', 'exit'):
                recover_session(worker, recovery_directory, notes, exported_notes, started)
                return
            if name == 'help':
                print(COUNT_HELP if count_only else INTERACTIVE_HELP)
                continue
            if count_only and name not in ('status', 'stop', 'clear', 'release', 'start', 'end',
                                           'restart', 'set', 'count-note', 'export', 'collect'):
                raise CalibrationError('计数专用入口只接受手转命令，请输入help')
            if servo_only and name not in ('status', 'stop', 'clear', 'release', 'prepare', 'start',
                                           'end', 'restart', 'pulse', 'angle', 'export', 'collect', 'query-us', 'query-angle'):
                raise CalibrationError('舵机专用入口只接受舵机命令，不允许电机试验')
            if name == 'restart' and not args and not count_only and servo_values is not None:
                state = prepare_servo_start(worker, servo_values, discard=True)
                notes.clear()
                started = False
                servo_target = word_float(servo_values[0])
                print('上一组未保存的舵机数据已丢弃；使用已确认范围重新准备，无固定时限')
                if confirm('操作者确认现在重新开始舵机测量', 'RUN'):
                    started = True
                    try:
                        state = worker.call('command', CMD_START, [state['prepare_token']])
                    except CommandRejected:
                        started = False
                        raise
                    print('舵机已重新开始；由end结束。run_id =', state['run_id'])
                continue
            if servo_only and name == 'restart':
                raise CalibrationError('先prepare servo并确认机械范围，才能restart')
            if name == 'set' and args[:1] == ['skip-save'] and len(args) == 2:
                if not count_only or args[1] not in ('on', 'off'):
                    raise CalibrationError('计数专用入口使用set skip-save on或off')
                skip_save = args[1] == 'on'
                print('跳过未保存手转组：' + ('开启' if skip_save else '关闭'))
                continue
            if (name == 'restart' or (count_only and name == 'start')) and not args:
                if name == 'start' and (started or notes) and not skip_save:
                    raise CalibrationError('本组尚未export；先导出，或用restart明确丢弃本组')
                previous = worker.call('snapshot')
                discard = name == 'restart' or (skip_save and (started or bool(notes)))
                # START已接受但主机确认失败的组仍须先STOP/冻结，显式丢弃才能解除屏障。
                state = prepare_count_start(worker, discard=discard)
                notes.clear()
                started = True
                try:
                    state = worker.call('command', CMD_COUNT_START, [state['prepare_token']])
                except CommandRejected:
                    started = False
                    raise
                print('手转已开始；由end结束，无固定时限。run_id =', state['run_id'])
                if discard and previous['kind'] == KIND_COUNT:
                    print('上一组未保存的手转数据已丢弃')
                continue
            if name == 'status':
                print(json.dumps(worker.call('snapshot'), ensure_ascii=False, indent=2, allow_nan=False))
                continue
            if name == 'stop':
                worker.request_stop()
                print('已提交高优先级停车请求')
                continue
            if name == 'clear':
                if confirm('人工确认故障原因已排除，清故障不会启动'):
                    worker.call('command', CMD_CLEAR_FAULT)
                continue
            if name == 'release':
                worker.call('command', CMD_RELEASE)
                continue
            if name == 'prepare':
                if notes or started:
                    raise CalibrationError('本组尚未 export，先冻结导出后再准备下一组')
                state = worker.call('snapshot')
                if not args:
                    raise CalibrationError('prepare 缺少模式')
                if servo_only and args[0] != 'servo':
                    raise CalibrationError('舵机专用入口只允许prepare servo')
                if args[0] == 'count' and len(args) == 1:
                    command, words = CMD_PREPARE_COUNT, [1]
                elif args[0] == 'servo' and len(args) == 5:
                    command, words = CMD_PREPARE_SERVO, [*(float_word(value) for value in args[1:]), state['params_version'], 1]
                elif args[0] == 'rpm' and len(args) == 3:
                    command, words = CMD_PREPARE_RPM, [*(float_word(value) for value in args[1:]), state['params_version'], 1]
                else:
                    raise CalibrationError('prepare 参数数量或模式错误')
                print(json.dumps({'mode': args[0], 'values': args[1:], 'params_version': state['params_version']}, ensure_ascii=False))
                if confirm('已核对车轮架空／舵机机械范围、独立停止条件，本次只准备'):
                    state = worker.call('command', command, words)
                    notes.clear()
                    servo_values = words[:4] if command == CMD_PREPARE_SERVO else None
                    servo_target = word_float(servo_values[0]) if servo_values else None
                    print('准备完成；本次 token =', state['prepare_token'])
                continue
            if name == 'start':
                if started:
                    raise CalibrationError('前一次启动尝试尚未冻结导出，不能重发 START 或复用准备')
                state = worker.call('snapshot')
                if servo_values is not None and state['features'] & 8:
                    state = prepare_servo_start(worker, servo_values)
                if state['phase'] not in (1, 2, 3):
                    raise CalibrationError('尚未人工准备')
                print(json.dumps({'phase': state['phase_name'], 'token': state['prepare_token'],
                                  'firmware_params': state['params_version'], 'heartbeat_age_us': state['heartbeat_age_us'],
                                  'limit_s': (None if state['features'] & 2 else 120) if state['phase'] == 1 else
                                  None if state['phase'] == 2 and state['features'] & 8 else 8}, ensure_ascii=False))
                if confirm('操作者确认现在开始本次测量', 'RUN'):
                    opcode = CMD_COUNT_START if state['phase'] == 1 else CMD_START
                    # 提交前即登记待取回证据；超时或ACK后的快照失败均不能解除这个屏障。
                    started = True
                    try:
                        worker.call('command', opcode, [state['prepare_token']])
                    except CommandRejected:
                        started = False
                        raise
                continue
            if name == 'pulse' and len(args) == 1:
                worker.call('command', CMD_PULSE, [float_word(args[0])])
                servo_target = finite(args[0], '舵机目标脉宽')
                continue
            if name == 'end' and not args:
                state = worker.call('snapshot')
                if state['phase'] == KIND_SERVO:
                    if not state['features'] & 8:
                        raise CalibrationError('旧舵机固件不支持end，请stop并导出')
                    worker.call('command', CMD_SERVO_END, [state['prepare_token']])
                    state = worker.call('stop_and_wait')
                    if state['phase'] != 7 or state['log_end_reason'] != 1:
                        raise CalibrationError('舵机end被停止或故障中止，请查看状态并保留记录')
                    print('舵机已正常结束，零输出记录已冻结；export保存，restart可丢弃重开')
                    continue
                state = worker.call('command', CMD_COUNT_END, [state['prepare_token']])
                print('手转已结束；左净计数 =', state['count_end_left'] - state['count_start_left'],
                      '；右净计数 =', state['count_end_right'] - state['count_start_right'])
                continue
            if name == 'query-us' and len(args) == 1:
                state = worker.call('command', CMD_QUERY_US, [float_word(args[0])])
                print('标定表估计左右前轮角：', state['query_0'], state['query_1'])
                continue
            if name == 'query-angle' and len(args) == 2 and args[0] in ('left', 'right'):
                state = worker.call('command', CMD_QUERY_ANGLE, [float_word(args[1]), 0 if args[0] == 'left' else 1])
                print('标定表估计脉宽 us：', state['query_0'])
                continue
            if name in ('apply-counts', 'apply') and len(args) == 1:
                source = Path(args[0]).resolve()
                candidate = validate_encoder_measurements(load_json(source), source.parent)
                if candidate['firmware_build_id'] != worker.transport.image.build_id:
                    raise CalibrationError('计数实测固件与当前运行固件不一致')
                state = worker.call('snapshot')
                if state['control_state'] != 1 or state['phase'] in (1, 2, 3, 4, 5, 6):
                    raise CalibrationError('需停车并释放准备权后应用')
                n = candidate['counts_per_rev']
                print('候选每圈计数：', n)
                if confirm('人工确认本次整组应用左右实测每圈计数，转向表仍需复核候选源文件'):
                    worker.call('command', CMD_APPLY_ENCODER,
                                [float_word(n['left']), float_word(n['right']), state['measurement_version'], 1])
                continue
            if name == 'angle' and len(args) in (5, 6):
                state = worker.call('snapshot')
                if state['phase'] != KIND_SERVO or not state['servo_enabled']:
                    raise CalibrationError('仅在当前舵机单点保持并真实人工读数后录入')
                if servo_target is not None and abs(state['servo_request_us'] - servo_target) > 0.001:
                    raise CalibrationError('舵机名义输出尚未到达目标脉宽，请等待到位后再录入实测角')
                if args[2] not in ('increasing', 'decreasing') or args[3] not in ('1', '2', '3') or args[4] not in ('table', 'validation'):
                    raise CalibrationError('角度方向、重复编号或用途错误')
                note = {'type': 'servo', 'nominal_us': state['servo_nominal_us'],
                        'requested_us': state['servo_request_us'], 'left_deg': finite(args[0], '左前轮角'),
                        'right_deg': finite(args[1], '右前轮角'), 'direction': args[2], 'repeat': int(args[3]), 'role': args[4],
                        'sample_id': state['sample_id'], 'now_us': state['now_us'],
                        'instrument_measured_us': finite(args[5], '仪器实测脉宽') if len(args) == 6 else None}
                notes.append(note)
                print('已记录人工前轮角；停止冻结后 export 留档')
                continue
            if name == 'count-note' and len(args) == 3:
                state = worker.call('snapshot')
                if state['phase'] != 7 or state['kind'] != KIND_COUNT or state['log_end_reason'] != 1:
                    raise CalibrationError('只接受已正常结束的手转组')
                if args[0] not in ('left', 'right') or args[1] not in ('forward', 'reverse'):
                    raise CalibrationError('轮侧或转动方向错误')
                notes.append({'type': 'encoder', 'wheel': args[0], 'direction': args[1], 'turns': finite(args[2], '圈数'),
                              'count_delta': state['count_end_' + args[0]] - state['count_start_' + args[0]]})
                print('已记录准确转动圈数；export 保存原始起止累计计数')
                continue
            if name == 'export' and len(args) == 1:
                data = worker.call('frozen_export')
                saved = save_export(data, Path(args[0]), notes)
                print(json.dumps(saved, ensure_ascii=False, indent=2))
                for note in notes:
                    exported_notes.append({**note, 'run_export': saved['json']})
                notes.clear()
                started = False
                continue
            if name == 'collect' and len(args) == 1:
                path = Path(args[0]).resolve()
                collection = load_json(path) if path.exists() else {'schema_version': 1, 'version': 1, 'provenance': {'real_measurements': False},
                                                                    'encoder_runs': [], 'servo_samples': []}
                if collection.get('firmware_build_id', worker.transport.image.build_id) != worker.transport.image.build_id:
                    raise CalibrationError('测量集合属于其他固件，请使用新的集合文件')
                if any(not isinstance(collection.get(field), list) for field in ('encoder_runs', 'servo_samples')):
                    raise CalibrationError('测量集合缺少 encoder_runs/servo_samples 列表')
                collection['firmware_build_id'] = worker.transport.image.build_id
                for note in exported_notes:
                    collection['encoder_runs' if note['type'] == 'encoder' else 'servo_samples'].append(note)
                write_json(path, collection)
                exported_notes.clear()
                print('已汇总原始人工条目；生成前补齐 provenance、center_us 和真实测量来源')
                continue
            raise CalibrationError('未知命令或参数不匹配，请输入 help')
        except (EOFError, KeyboardInterrupt):
            worker.request_stop()
            recover_session(worker, recovery_directory, notes, exported_notes, started)
            return
        except (CalibrationError, ValueError, OSError, json.JSONDecodeError) as exc:
            print('拒绝／未完成：', exc, file=sys.stderr)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='command', required=True)
    for name in ('status', 'interactive', 'export'):
        item = sub.add_parser(name)
        item.add_argument('--elf', required=True, type=Path, help='本次诊断 AXF/ELF')
        item.add_argument('--probe', required=True, help='探针精确序列号；不使用模糊匹配')
        if name == 'export':
            item.add_argument('--output', required=True, type=Path)
        if name == 'interactive':
            item.add_argument('--recovery-dir', type=Path, default=Path.cwd() / 'calibration_records',
                              help='退出时保存未导出的冻结记录与人工笔记')
            item.add_argument('--count-only', action='store_true', help='只允许手转；start/end直接控制测量')
            item.add_argument('--skip-save', action='store_true', help='计数专用入口允许start丢弃上一组未保存数据')
            item.add_argument('--servo-only', action='store_true', help='舵机专用入口，拒绝手转和电机试验命令')
    profile = sub.add_parser('profile', help='离线校验真实测量并生成 C 常量候选，不连接探针')
    profile.add_argument('--measurements', required=True, type=Path)
    profile.add_argument('--output', required=True, type=Path)
    reference = sub.add_parser('rpm-reference', help='离线核对手机慢动作独立 RPM')
    reference.add_argument('--export', dest='export_file', required=True, type=Path)
    reference.add_argument('--reference', required=True, type=Path)
    reference.add_argument('--output', required=True, type=Path)
    args = parser.parse_args(argv)
    if args.command == 'interactive' and args.skip_save and not args.count_only:
        parser.error('--skip-save须与--count-only一起使用')
    if args.command == 'interactive' and args.servo_only and args.count_only:
        parser.error('--servo-only与--count-only不能同时使用')
    worker = None
    try:
        if args.command == 'profile':
            result = generate_profile(args.measurements, args.output)
        elif args.command == 'rpm-reference':
            result = rpm_reference(load_json(args.export_file), load_json(args.reference))
            write_json(args.output, result)
        else:
            image = FirmwareImage.from_elf(args.elf)
            required = 8 if args.command == 'interactive' and args.servo_only else 0
            worker = IOWorker(attach_factory(image, args.probe, args.command == 'interactive', required_features=required))
            if args.command == 'interactive':
                interactive(worker, args.recovery_dir, count_only=args.count_only, skip_save=args.skip_save,
                            servo_only=args.servo_only)
                result = {'closed': True}
            elif args.command == 'status':
                result = worker.call('snapshot')
                result['build_id'] = image.build_id
            else:
                result = save_export(worker.call('frozen_export'), args.output)
        print(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False))
        return 0
    except (CalibrationError, OSError, ValueError, json.JSONDecodeError) as exc:
        print('标定工具未完成：' + str(exc), file=sys.stderr)
        return 1
    finally:
        if worker is not None:
            worker.close()


def finite(value: Any, label: str) -> float:
    """接受有限数值，拒绝布尔值、NaN 和无穷大。"""
    if isinstance(value, bool):
        raise CalibrationError(label + " 必须是有限数值")
    try:
        result = float(value)
    except (ValueError, TypeError) as exc:
        raise CalibrationError(label + " 必须是有限数值") from exc
    if not math.isfinite(result):
        raise CalibrationError(label + " 必须是有限数值")
    return result


def load_json(path: Path) -> dict:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise CalibrationError(str(path) + " 不是 JSON 对象")
    return value


def write_json(path: Path, value: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def interpolate(points: list[dict], pulse_us: float, wheel: str) -> float:
    """在实测节点间线性插值；端点之外拒绝推算。"""
    if pulse_us < points[0]["nominal_us"] or pulse_us > points[-1]["nominal_us"]:
        raise CalibrationError("验证点位于映射表之外")
    field = wheel + "_deg"
    for low, high in zip(points, points[1:]):
        if low["nominal_us"] <= pulse_us <= high["nominal_us"]:
            weight = (pulse_us - low["nominal_us"]) / (high["nominal_us"] - low["nominal_us"])
            return low[field] + weight * (high[field] - low[field])
    return points[-1][field]


def evidence_export(record: dict, base: Path, build_id: str, kind: int) -> dict:
    """每条人工测量必须关联同固件的冻结原始导出，保留来源而不以模拟值充填。"""
    name = record.get("run_export")
    if not isinstance(name, str) or not name:
        raise CalibrationError("实测条目缺少 run_export 原始记录")
    source = Path(name)
    if not source.is_absolute():
        source = base / source
    data = load_json(source)
    meta = data.get("metadata", {})
    if not isinstance(meta, dict):
        raise CalibrationError('原始导出的 metadata 必须是对象')
    if meta.get("build_id") != build_id or meta.get("kind") != kind:
        raise CalibrationError("实测条目引用不同固件或不同试验类型")
    if meta.get("frozen") is not True or meta.get("valid") is not True:
        raise CalibrationError("实测条目引用尚未冻结或无效的试验")
    return data


def validate_encoder_measurements(data: dict, base: Path) -> dict:
    """独立验证两轮每圈实测计数；无需先取得转向表即可核对 RPM。"""
    provenance = data.get("provenance", {})
    if not isinstance(provenance, dict):
        raise CalibrationError('实测 provenance 必须是对象')
    if provenance.get("real_measurements") is not True:
        raise CalibrationError("必须明确声明真实实测来源，教学或模拟数据不能生成有效配置")
    for field in ("operator", "date", "encoder_method"):
        if not isinstance(provenance.get(field), str) or not provenance[field].strip():
            raise CalibrationError("实测来源缺少 " + field)
    build_id = data.get("firmware_build_id")
    if not isinstance(build_id, str) or len(build_id) != 8 or any(character not in '0123456789abcdef' for character in build_id):
        raise CalibrationError("缺少实测固件标识")
    version = data.get("version")
    if isinstance(version, bool) or not isinstance(version, int) or not 1 <= version <= 0xFFFFFFFF:
        raise CalibrationError("标定版本必须是非零 uint32 整数")
    counts_per_rev = {}
    runs = data.get("encoder_runs", [])
    if not isinstance(runs, list) or any(not isinstance(run, dict) for run in runs):
        raise CalibrationError("encoder_runs 必须是列表")
    for wheel in ("left", "right"):
        values = []
        seen_exports = set()
        for direction, sign in (("forward", 1), ("reverse", -1)):
            selected = [run for run in runs if run.get("wheel") == wheel and run.get("direction") == direction]
            if len(selected) < 3:
                raise CalibrationError(wheel + " " + direction + " 至少需要三组实测")
            for run in selected:
                turns = finite(run.get("turns"), "实测圈数")
                count = run.get("count_delta")
                if turns != 10 or isinstance(count, bool) or not isinstance(count, int) or count * sign <= 0 or abs(count) >= 2**63:
                    raise CalibrationError("每组需准确十圈、整数有效计数及正确前进/反转方向")
                exported = evidence_export(run, base, build_id, KIND_COUNT)
                identity = str((base / run['run_export']).resolve())
                if identity in seen_exports:
                    raise CalibrationError('每个轮侧重复组必须引用独立试验原始记录')
                seen_exports.add(identity)
                metadata = exported["metadata"]
                measured = metadata.get("count_delta_" + wheel)
                if measured != count:
                    raise CalibrationError("人工计数与原始冻结起止累计计数不符")
                values.append(abs(count) / turns)
        median = statistics.median(values)
        if not 1 <= median <= 10000000:
            raise CalibrationError('每圈有效计数须在固件允许的 1 至 10000000 范围内')
        if max(abs(value / median - 1) for value in values) > 0.01 + 1e-12:
            raise CalibrationError(wheel + " 每圈计数重复或正反差异超过 1%")
        counts_per_rev[wheel] = median
    return {'counts_per_rev': counts_per_rev, 'firmware_build_id': build_id, 'version': version, 'provenance': provenance}


def quantize_firmware_profile(candidate: dict) -> dict:
    """先转换为固件实际 float32，再验证严格升序、同向单调、中位零点和范围。"""
    candidate['center_us'] = word_float(float_word(candidate['center_us']))
    for wheel in ('left', 'right'):
        value = word_float(float_word(candidate['counts_per_rev'][wheel]))
        if not 1 <= value <= 10000000:
            raise CalibrationError('float32 每圈计数超出固件范围')
        candidate['counts_per_rev'][wheel] = value
    points = candidate['points']
    for point in points:
        for field in ('nominal_us', 'left_deg', 'right_deg'):
            point[field] = word_float(float_word(point[field]))
        if not 500 <= point['nominal_us'] <= 2500 or any(abs(point[field]) >= 89 for field in ('left_deg', 'right_deg')):
            raise CalibrationError('float32 脉宽或前轮角超出固件范围')
    if not points[0]['nominal_us'] < candidate['center_us'] < points[-1]['nominal_us']:
        raise CalibrationError('float32 中位不在安全范围内部')
    centers = [point for point in points if abs(point['nominal_us'] - candidate['center_us']) <= 0.01]
    if len(centers) != 1 or any(abs(centers[0][field]) > 0.01 for field in ('left_deg', 'right_deg')):
        raise CalibrationError('float32 中位须是唯一双轮零角节点')
    directions = []
    for low, high in zip(points, points[1:]):
        if high['nominal_us'] <= low['nominal_us']:
            raise CalibrationError('float32 脉宽节点折叠，固件要求严格升序')
        signs = []
        for field in ('left_deg', 'right_deg'):
            delta = high[field] - low[field]
            if delta == 0:
                raise CalibrationError('float32 前轮角节点折叠，固件要求严格单调')
            signs.append(1 if delta > 0 else -1)
        if signs[0] != signs[1]:
            raise CalibrationError('float32 左右前轮角方向不一致')
        directions.append(signs[0])
    if len(set(directions)) != 1:
        raise CalibrationError('float32 前轮角映射非严格单调')
    candidate['pulse_min_us'], candidate['pulse_max_us'] = points[0]['nominal_us'], points[-1]['nominal_us']
    return candidate


def validate_measurements(data: dict, base: Path) -> dict:
    """检查完整实测矩阵、重复性、回差及独立中间点，返回可生成常量的候选配置。"""
    counts = validate_encoder_measurements(data, base)
    provenance = counts['provenance']
    if not isinstance(provenance.get('angle_method'), str) or not provenance['angle_method'].strip():
        raise CalibrationError('实测来源缺少 angle_method')
    build_id, version = counts['firmware_build_id'], counts['version']
    counts_per_rev = counts['counts_per_rev']
    center = finite(data.get('center_us'), 'center_us')
    toe = {wheel: finite(provenance.get('toe_' + wheel + '_deg'), '直行前束') for wheel in ('left', 'right')}
    samples = data.get("servo_samples", [])
    if not isinstance(samples, list) or not samples or any(not isinstance(sample, dict) for sample in samples):
        raise CalibrationError("缺少前轮角实测")
    table = {}
    validation = []
    seen = set()
    for sample in samples:
        pulse = finite(sample.get("nominal_us"), "名义脉宽")
        angles = {wheel: finite(sample.get(wheel + "_deg"), "前轮角") for wheel in ("left", "right")}
        if not 500 < pulse < 2500 or any(abs(angle) >= 90 for angle in angles.values()):
            raise CalibrationError("脉宽或前轮角超出允许数据范围")
        direction = sample.get("direction")
        repeat = sample.get("repeat")
        role = sample.get("role")
        if direction not in ("increasing", "decreasing") or isinstance(repeat, bool) or not isinstance(repeat, int) or repeat not in (1, 2, 3):
            raise CalibrationError("角度记录须注明增/减脉宽和第 1 至 3 次重复")
        exported = evidence_export(sample, base, build_id, KIND_SERVO)
        rows = exported.get('records', [])
        if not isinstance(rows, list) or any(not isinstance(row, dict) for row in rows):
            raise CalibrationError('舵机原始记录必须是列表对象')
        if not any(abs(finite(record.get('servo_nominal_us'), '原始名义脉宽') - pulse) < 1e-4 for record in rows):
            raise CalibrationError("角度脉宽未出现在关联试验的原始记录中")
        key = (pulse, direction, repeat, role)
        if key in seen:
            raise CalibrationError("角度记录重复")
        seen.add(key)
        item = {"nominal_us": pulse, "direction": direction, "repeat": repeat, **{wheel + "_deg": angle for wheel, angle in angles.items()}}
        if role == "table":
            table.setdefault(pulse, []).append(item)
        elif role == "validation":
            validation.append(item)
        else:
            raise CalibrationError("角度记录 role 必须是 table 或 validation")
    if not 3 <= len(table) <= 17:
        raise CalibrationError("有效映射表需要 3 至 17 个实测脉宽节点")
    points = []
    for pulse, items in sorted(table.items()):
        point = {"nominal_us": pulse}
        for direction in ("increasing", "decreasing"):
            if {sample["repeat"] for sample in items if sample["direction"] == direction} != {1, 2, 3}:
                raise CalibrationError("每个表节点需增、减脉宽各三次实测")
        for wheel in ("left", "right"):
            field = wheel + "_deg"
            values = [item[field] for item in items]
            if max(values) - min(values) > 2 + 1e-12:
                raise CalibrationError("表节点的重复波动或双向回差超过 2°")
            point[field] = statistics.mean(values)
        points.append(point)
    trends = []
    for wheel in ("left", "right"):
        differences = [high[wheel + "_deg"] - low[wheel + "_deg"] for low, high in zip(points, points[1:])]
        if not (all(delta > 0 for delta in differences) or all(delta < 0 for delta in differences)):
            raise CalibrationError("前轮角映射须严格单调")
        trends.append(1 if differences[0] > 0 else -1)
        if abs(interpolate(points, center, wheel)) > 2:
            raise CalibrationError("中位相对直行零角偏差超过 2°")
    if trends[0] != trends[1]:
        raise CalibrationError("左右前轮角方向必须一致")
    centers = [point for point in points if abs(point["nominal_us"] - center) <= 0.01]
    if len(centers) != 1:
        raise CalibrationError("直行中位必须是实测表节点")
    if not points[0]['nominal_us'] < centers[0]['nominal_us'] < points[-1]['nominal_us']:
        raise CalibrationError('中位须在两侧安全边界之间')
    if not validation:
        raise CalibrationError("缺少未参与建表的中间验证点")
    for sample in validation:
        if sample["nominal_us"] in table:
            raise CalibrationError("独立验证点不能参与建表")
        for wheel in ("left", "right"):
            if abs(interpolate(points, sample["nominal_us"], wheel) - sample[wheel + "_deg"]) > 2 + 1e-12:
                raise CalibrationError("独立验证点误差超过 2°")
    # 保留原始前束与测量数据；候选表统一减去实测中位，明确相对直行的零点。
    zero = {wheel: centers[0][wheel + "_deg"] for wheel in ("left", "right")}
    for point in points:
        for wheel in ("left", "right"):
            point[wheel + "_deg"] -= zero[wheel]
            if abs(point[wheel + "_deg"]) >= 89:
                raise CalibrationError("相对中位前轮角超出 ±89°")
    candidate = quantize_firmware_profile({"version": version, "center_us": centers[0]["nominal_us"], "points": points, "counts_per_rev": counts_per_rev,
            "firmware_build_id": build_id, "toe_deg": toe, "provenance": provenance,
            "measured_center_offset_deg": zero,
            "acceptance": {"encoder_relative_deviation_max": 0.01, "angle_error_max_deg": 2.0}})
    for sample in validation:
        for wheel in ('left', 'right'):
            actual = sample[wheel + '_deg'] - zero[wheel]
            if abs(interpolate(candidate['points'], sample['nominal_us'], wheel) - actual) > 2 + 1e-12:
                raise CalibrationError('float32 候选表的独立验证点误差超过 2°')
    return candidate


def generate_profile(source: Path, output: Path) -> dict:
    """生成操作者指定的 C 常量文件；不直接覆盖固件基线，也不自动烧录。"""
    source = source.resolve()
    output = output.resolve()
    if output.suffix.lower() != '.c' or output.with_suffix('.json') == source:
        raise CalibrationError('候选必须输出为独立 .c 文件，不能覆盖原始测量 JSON')
    firmware_default = Path(__file__).resolve().parents[1] / "user/src/calibration_profile.c"
    if output == firmware_default.resolve():
        raise CalibrationError("请输出到独立候选文件，复核后再人工应用到固件基线")
    candidate = validate_measurements(load_json(source), source.parent)
    def c_float(value: float) -> str:
        result = format(value, ".9g")
        if "." not in result and "e" not in result:
            result += ".0"
        return result + "f"
    points = candidate["points"]
    lines = ['#include "calibration_profile.h"', '',
             '// 由真实实测记录生成的候选配置；合入前需复核机械安全范围和原始证据。',
             '// 来源文件 SHA256：' + hashlib.sha256(source.read_bytes()).hexdigest(),
             'const float calibration_profile_counts_per_rev_left = ' + c_float(candidate["counts_per_rev"]["left"]) + ';',
             'const float calibration_profile_counts_per_rev_right = ' + c_float(candidate["counts_per_rev"]["right"]) + ';',
             '// 本候选未提供滚动周长，m/s换算保持不可用，后续按实际周长另行补齐。',
             'const float calibration_profile_circumference_left_m = 0.0f;',
             'const float calibration_profile_circumference_right_m = 0.0f;',
             '// 完整双轮候选不附带单轮初步表；所有新增公共常量均显式定义。',
             'const SteeringWheelMap calibration_profile_right_steering_map = {0};',
             'const SteeringMap calibration_profile_steering_map = {',
             f'    {candidate["version"]}u, 1u, {len(points)}u,',
             '    ' + ', '.join(c_float(value) for value in (candidate["center_us"], points[0]["nominal_us"], points[-1]["nominal_us"])) + ',',
             '    {']
    lines += ['        {' + ', '.join(c_float(point[key]) for key in ("nominal_us", "left_deg", "right_deg")) + '},' for point in points]
    lines += ['    }', '};', '']
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text('\n'.join(lines), encoding="utf-8")
    write_json(output.with_suffix('.json'), candidate)
    return candidate


def rpm_reference(exported: dict, reference: dict) -> dict:
    """用实际采集帧率、两次显示同步标记和完整圈数核对同一累计计数区间。"""
    metadata = exported.get('metadata', {})
    if not isinstance(metadata, dict) or metadata.get('valid') is not True or metadata.get('frozen') is not True or metadata.get('kind') != KIND_RPM:
        raise CalibrationError("参考分析需要有效且冻结的试验")
    fps = finite(reference.get("capture_fps"), "实际采集帧率")
    start = finite(reference.get("start_frame"), "起始帧")
    end = finite(reference.get("end_frame"), "结束帧")
    turns = finite(reference.get("complete_turns"), "完整圈数")
    wheel = reference.get("wheel")
    n = finite(reference.get("counts_per_rev"), "每圈计数")
    if fps <= 0 or end <= start or turns <= 0 or not turns.is_integer() or n <= 0 or wheel not in ("left", "right"):
        raise CalibrationError("帧率、区间、完整圈数、轮侧或每圈计数非法")
    frozen_n = finite(metadata.get('counts_per_rev_' + wheel), '本组试验实际使用的每圈计数')
    if frozen_n <= 0 or not math.isclose(n, frozen_n, rel_tol=1e-6):
        raise CalibrationError('参考换算 N 与本组试验的实测参数不一致或无法确认')
    frames = reference.get("marker_frames")
    all_markers = metadata.get('markers', [])
    if not isinstance(all_markers, list) or any(not isinstance(marker, dict) for marker in all_markers):
        raise CalibrationError('标记元数据无效')
    markers = [marker for marker in all_markers if marker.get('index') in (0, 1)]
    markers.sort(key=lambda marker: marker['index'])
    if not isinstance(frames, list) or len(frames) != 2 or len(markers) != 2 or [marker['index'] for marker in markers] != [0, 1]:
        raise CalibrationError("需要两次 TFT 色块的采集帧号和固件绘制起止时间")
    f1, f2 = [finite(frame, "同步帧") for frame in frames]
    if not f1 <= start < end <= f2 or f1 >= f2:
        raise CalibrationError("测速帧必须位于两个同步标记之间")
    times = []
    delay = 0.0
    for marker in markers:
        begin = finite(marker.get("begin_us"), "标记绘制开始")
        finish = finite(marker.get("end_us"), "标记绘制结束")
        if finish < begin:
            raise CalibrationError("显示标记时间逆序")
        times.append((begin + finish) / 2)
        delay += (finish - begin) / 2
    if times[1] <= times[0]:
        raise CalibrationError("固件同步标记时间逆序")
    scale = (times[1] - times[0]) / (f2 - f1)
    if abs(scale * fps / 1e6 - 1) > 0.02:
        raise CalibrationError("采集帧率与同步时间相差超过 2%，证据不足")
    t1 = times[0] + (start - f1) * scale
    t2 = times[0] + (end - f1) * scale
    records = exported.get("records", [])
    if not isinstance(records, list) or len(records) < 2 or any(not isinstance(record, dict) for record in records):
        raise CalibrationError("原始记录不足")
    records = [{'elapsed_us': finite(row.get('elapsed_us'), '原始采样时间'),
                'total_' + wheel: finite(row.get('total_' + wheel), '原始累计计数')} for row in records]
    if any(b['elapsed_us'] <= a['elapsed_us'] for a, b in zip(records, records[1:])):
        raise CalibrationError('原始样本时间不连续递增')
    if not records[0]['elapsed_us'] <= t1 < t2 <= records[-1]['elapsed_us']:
        raise CalibrationError('测速区间未被原始样本完整覆盖')
    selected = [min(records, key=lambda row: abs(row["elapsed_us"] - endpoint)) for endpoint in (t1, t2)]
    a, b = selected
    dt = (b["elapsed_us"] - a["elapsed_us"]) / 1e6
    if dt <= 0:
        raise CalibrationError("采样区间不足")
    delta = b["total_" + wheel] - a["total_" + wheel]
    measured = 60 * abs(delta) / n / dt
    reference_rpm = 60 * turns * fps / (end - start)
    frame_uncertainty = finite(reference.get("endpoint_frame_uncertainty", 1), "视频端点误差")
    if frame_uncertainty < 0:
        raise CalibrationError("视频端点误差不可为负")
    sample_uncertainty = (abs(a["elapsed_us"] - t1) + abs(b["elapsed_us"] - t2)) / 1e6
    seconds_uncertainty = 2 * frame_uncertainty / fps + delay / 1e6 + sample_uncertainty
    tolerance = max(reference_rpm * 0.03, 120 / n / dt, reference_rpm * seconds_uncertainty / ((end - start) / fps))
    return {"wheel": wheel, "reference_rpm": reference_rpm, "encoder_rpm": measured, "tolerance_rpm": tolerance,
            "difference_rpm": abs(measured - reference_rpm), "pass": abs(measured - reference_rpm) <= tolerance,
            "interval_us": [a["elapsed_us"], b["elapsed_us"]], "uncertainty_s": seconds_uncertainty}


if __name__ == '__main__':
    raise SystemExit(main())

