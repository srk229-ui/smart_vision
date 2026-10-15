"""关闭输出诊断版实机协议检查；没有人工圈数，不属于编码器精度标定。"""

from pathlib import Path
from elftools.elf.elffile import ELFFile
import argparse
import json
import sys
import time

ROOT = Path(__file__).resolve().parents[5]
sys.path.insert(0, str(ROOT / 'tools'))
import calibrate_pyocd as cal

DIRECTORY = Path(__file__).resolve().parent
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--hold-seconds', type=float, default=125.0)
parser.add_argument('--output-prefix', default='protocol_repeat')
args = parser.parse_args()
if args.hold_seconds <= 0 or Path(args.output_prefix).name != args.output_prefix:
    parser.error('时长须为正值，输出前缀须为单个文件名')
firmware = DIRECTORY.parent / 'builds/calibration_readonly/firmware.axf'
image = cal.FirmwareImage.from_elf(firmware)
with firmware.open('rb') as stream:
    names = {'wifi_assistant_state', 'wifi_assistant_reconnect_count'}
    objects = {symbol.name: (symbol['st_value'], symbol['st_size'])
               for symbol in ELFFile(stream).get_section_by_name('.symtab').iter_symbols()
               if symbol.name in names}

result = {'purpose': 'on_device_protocol_only', 'real_measurements': False,
          'build_id': image.build_id, 'elf_sha256': image.sha256, 'operations': [], 'passed': False}
base_factory = cal.attach_factory(image, '80941C44-B8F862DC2544', True)


def factory():
    transport, close = base_factory()

    def read_wifi():
        transport.ensure_live_identity()
        return {name: int.from_bytes(bytes(transport.probe.read_memory_block8(address, size)), 'little')
                for name, (address, size) in objects.items()}

    transport.read_wifi = read_wifi
    return transport, close


worker = None
try:
    worker = cal.IOWorker(factory)
    result['preflight'] = worker.call('snapshot')
    assert result['preflight']['command_ready'] and result['preflight']['features'] == 7
    result['wifi_before'] = worker.call('read_wifi')

    def command(opcode, args):
        begin = time.monotonic()
        state = worker.call('command', opcode, args)
        result['operations'].append({'opcode': opcode, 'elapsed_s': time.monotonic() - begin,
                                     'ack_status': state['ack_status'], 'phase': state['phase_name']})
        return state

    for index in range(3):
        prepared = cal.prepare_count_start(worker, discard=index > 0)
        state = command(cal.CMD_COUNT_START, [prepared['prepare_token']])
        assert state['phase'] == cal.KIND_COUNT
        time.sleep(0.15)
        state = command(cal.CMD_COUNT_END, [prepared['prepare_token']])
        assert state['phase'] == 7 and state['end_name'] == 'MANUAL'
        worker.call('stop_and_wait')
    print('连续3组start/end及结束后重开通过，未导出被丢弃组。', flush=True)

    prepared = cal.prepare_count_start(worker, discard=True)
    command(cal.CMD_COUNT_START, [prepared['prepare_token']])
    prepared = cal.prepare_count_start(worker, discard=True)
    state = command(cal.CMD_COUNT_START, [prepared['prepare_token']])
    assert state['phase'] == cal.KIND_COUNT
    print(f'活动组直接STOP/确认/丢弃/重开通过，开始{args.hold_seconds:g}秒验证。', flush=True)
    begin = time.monotonic()
    reported = 0
    while time.monotonic() - begin < args.hold_seconds:
        time.sleep(5)
        state = worker.call('snapshot')
        assert state['phase'] == cal.KIND_COUNT and state['control_fault'] == 0
        assert state['pwm_left'] == state['pwm_right'] == state['servo_duty'] == 0
        assert not state['servo_enabled'] and state['log_count'] == 0
        result['last_active_snapshot'] = state
        wifi = worker.call('read_wifi')
        assert wifi['wifi_assistant_state'] == result['wifi_before']['wifi_assistant_state']
        elapsed = time.monotonic() - begin
        if elapsed >= reported + 30:
            print(f'手转协议保持活动 {elapsed:.1f}s，输出快照为零。', flush=True)
            reported = int(elapsed)
    result['active_host_seconds'] = time.monotonic() - begin
    if args.hold_seconds > 120:
        assert state['active_elapsed_us'] > 120000000
    result['ended'] = command(cal.CMD_COUNT_END, [prepared['prepare_token']])
    worker.call('stop_and_wait')
    exported = worker.call('frozen_export')
    exported['provenance'] = {'real_measurements': False, 'purpose': 'on_device_protocol_only'}
    result['raw_export'] = cal.save_export(exported, DIRECTORY / (args.output_prefix + '_count.json'))
    result['wifi_after'] = worker.call('read_wifi')
    if 'wifi_assistant_reconnect_count' in objects:
        assert result['wifi_before']['wifi_assistant_reconnect_count'] == result['wifi_after']['wifi_assistant_reconnect_count']
    else:
        result['wifi_reconnect_counter_observation'] = '诊断版不引用该计数，链接器移除符号；未假设其读数'
    assert result['wifi_before']['wifi_assistant_state'] == result['wifi_after']['wifi_assistant_state']
    result['passed'] = True
    print('PASS 实机协议：连续操作、无保存重开、end、零输出快照、Wi-Fi状态稳定。', flush=True)
except Exception as exc:
    result['error'] = str(exc)
    raise
finally:
    if worker is not None:
        worker.close()
    with (DIRECTORY / (args.output_prefix + '_result.json')).open('x', encoding='utf-8') as output:
        output.write(json.dumps(result, ensure_ascii=False, indent=2))
