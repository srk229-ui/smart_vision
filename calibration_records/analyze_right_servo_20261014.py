"""保留操作者口述右前轮三点实测，关联导出并估计零角脉宽，不应用固件配置。"""

# R016的一次性历史脚本保留下方原计算；端点已由R019更正，禁止再生成旧映射或改写历史台账。
raise SystemExit('此R016历史脚本已停用：1444/1610角度关联已由R019更正。请运行 python tools/check_measured_profile.py 核对当前映射。')

from pathlib import Path
import hashlib
import json

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / 'calibration_records/servo_01.json'
OUTPUT = ROOT / 'calibration_records/servo_right_three_points_20261014.json'
raw = json.loads(SOURCE.read_text(encoding='utf-8'))
metadata = raw['metadata']
points = [{'pulse_us': 1444.0, 'right_deg': 17.0},
          {'pulse_us': 1526.0, 'right_deg': 5.0},
          {'pulse_us': 1610.0, 'right_deg': -13.0}]
assert metadata['valid'] and metadata['frozen'] and metadata['kind'] == 5 and metadata['log_end_reason'] == 1
assert all(row['pwm_left'] == row['pwm_right'] == 0 for row in raw['records'])
assert all(any(row['servo_nominal_us'] == point['pulse_us'] for row in raw['records']) for point in points)
segments = []
for low, high in zip(points, points[1:]):
    segments.append({'min_us': low['pulse_us'], 'max_us': high['pulse_us'],
                     'slope_deg_per_us': (high['right_deg'] - low['right_deg']) / (high['pulse_us'] - low['pulse_us']),
                     'anchor_us': low['pulse_us'], 'anchor_right_deg': low['right_deg']})
zero_us = 1526.0 + 5.0 * (1610.0 - 1526.0) / (5.0 - (-13.0))
grid_us = raw['snapshot']['nominal_us_per_duty']
nearest_duty = round(zero_us / grid_us)
nearest_us = nearest_duty * grid_us
estimated_angle = 5.0 - 18.0 * (nearest_us - 1526.0) / 84.0
assert abs(zero_us - 1549.3333333333333) < 1e-9 and nearest_us == 1550.0
analysis = {
    'schema_version': 1, 'type': 'partial_right_servo_measurement_analysis',
    'reported_on': '2026-10-14', 'measurement_date_confirmed': None,
    'wheel': 'right', 'angle_reference': '相对车身纵轴；平行为0度，保留操作者给出的正负号',
    'provenance': {'real_measurements': True, 'angle_source': '操作者在对话中提供，并确认右前轮',
                   'instrument_measured_pulse_us_confirmed': None,
                   'angle_in_original_export': bool(raw['manual_measurements']),
                   'angle_to_sample_timestamp_association': None},
    'run_export': 'servo_01.json', 'run_export_sha256': hashlib.sha256(SOURCE.read_bytes()).hexdigest(),
    'firmware_build_id': metadata['build_id'], 'firmware_elf_sha256': metadata['elf_sha256'],
    'raw_run_validation': {'frozen_manual_end': True, 'recording_mode': metadata['recording_mode'],
                           'all_motor_pwm_zero': True, 'all_reported_pulses_present_as_nominal_us': True,
                           'duration_us_modulo_2_32': (metadata['count_end_us'] - metadata['count_start_us']) & 0xffffffff},
    'measured_points': points, 'left_front_wheel_points': None,
    'approach_direction': None, 'repeat_number': None,
    'model': {'kind': 'piecewise_linear', 'segments': segments,
              'right_wheel_parallel_zero_estimate_us': zero_us,
              'zero_is_interpolated_not_measured': True},
    'suggested_next_measurement': {'request_us': nearest_us, 'integer_duty': nearest_duty,
                                   'nominal_grid_us': grid_us, 'predicted_right_deg': estimated_angle,
                                   'measured_right_deg': None},
    'validation_status': '右前轮单次三点初步关系；零角、左轮、增减方向重复、回差和中间点仍待测',
    'applied_to_firmware': False,
}
with OUTPUT.open('x', encoding='utf-8') as output:
    json.dump(analysis, output, ensure_ascii=False, indent=2)
    output.write('\n')


def replace_preserving_format(name, old, new):
    path = ROOT / name
    original = path.read_bytes()
    newline = '\r\n' if b'\r\n' in original else '\n'
    data = original.decode('utf-8').replace('\r\n', '\n')
    assert old in data, name
    path.write_bytes(data.replace(old, new).replace('\n', newline).encode('utf-8'))


replace_preserving_format('docs/TODO.md',
    '默认空表；CALIBRATION_GUIDE.md，不以模拟值关闭',
    '右前轮三点实测已记录：1444/+17、1526/+5、1610/-13；零角/左轮/正反重复仍待测，默认空表；R016')
replace_preserving_format('docs/IMPLEMENTATION_REPORT.md',
    '编译profile保持N=0与空表；没有真实数据，未生成有效候选。',
    '编译profile保持N=0与空表。已有左右轮单组计数及右前轮三点口述实测，尚未满足整组重复/双前轮/零角验收，未生成有效候选；右轮记录见R016与calibration_records/servo_right_three_points_20261014.json。')
replace_preserving_format('docs/CALIBRATION_GUIDE.md',
    '已有左右各一组手转计数的初步结果，完整正反重复与真实转角仍待测',
    '已有左右各一组手转计数及右前轮三点转角初步结果，完整正反重复、实测零角与左前轮仍待测')

record = f'''
## R016 — 2026年10月14日右前轮三点实测记录

操作者提供1526us/+5度、1444us/+17度、1610us/-13度，角度相对车身纵轴；随后明确测的是右前轮。原始servo_01.json未录入angle条目，本次单独保存口述角度及来源，不改原始导出，不补造左轮/方向/重复/仪器读数或采样时戳。

关联原始导出build_id={metadata['build_id']}、run_id={metadata['run_id']}，正常MANUAL结束且冻结、metadata.valid=true，三个名义脉宽均出现在事件日志，两电机记录PWM均零。GPT起终点模差811.65秒，仅是该记录的时间表示，不是独立波形/机械保持/STOP引脚延迟验收。

按脉宽升序为1444/+17、1526/+5、1610/-13，右轮转角随脉宽递减。分段斜率分别-12/82与-18/84度/us；右轮与车身纵轴平行的0度脉宽由1526/+5和1610/-13之间插值得1549.333us，尚非实测零点。当前名义整数网格2us，下一组可实测1550us；预测右角约-0.143度只属插值。整车中位仍须同时核对左轮和前束。

分析及原始导出SHA保存在calibration_records/servo_right_three_points_20261014.json；M3-08保持待实机，缺左轮、实测零角、双方向3重复/回差及独立中间点。默认profile、固件和窗口没有修改或发送执行器命令，M4仍未解锁。
'''
path = ROOT / 'docs/REVIEW_LOG.md'
original = path.read_bytes()
newline = '\r\n' if b'\r\n' in original else '\n'
path.write_bytes(original + record.replace('\n', newline).encode('utf-8'))
print('Saved right wheel measurement analysis; zero estimate:', zero_us, 'us; next nominal point:', nearest_us, 'us')
