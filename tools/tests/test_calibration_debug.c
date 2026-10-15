#include "calibration_debug.h"
#include "control.h"
#include "steering_map.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static ControlParams params;
static ControlSnapshot output;
static uint8_t reserved;
static uint32_t now_us;
static uint16_t raw_left;
static uint16_t raw_right;
static uint32_t sequence;
static uint32_t heartbeat_sequence;
static uint32_t run_calls;
static uint32_t stop_calls;
static uint8_t reject_restore;

// 控制器状态由mock提供，协议和采样/查表均调用真实C模块，不复制邮箱算法。
void control_get_params(ControlParams *out, uint32_t *version)
{ if (out != NULL) *out = params; if (version != NULL) *version = output.params_version; }
void control_get_snapshot(ControlSnapshot *out) { if (out != NULL) *out = output; }
ControlState control_get_state(void) { return output.state; }
void control_set_calibration_reserved(uint8_t value) { reserved = value; }
uint8_t control_get_calibration_reserved(void) { return reserved; }
uint32_t control_stop_generation(void) { return stop_calls; }
uint8_t control_apply_calibration_params(const ControlParams *value, uint32_t version)
{
    if (!reserved || output.state == CONTROL_RUNNING || version != output.params_version) return 0u;
    if (reject_restore && value->target_left == 3.0f && !value->calibration_confirmed) return 0u;
    params = *value; ++output.params_version; return 1u;
}
void control_request_stop(void)
{
    ++stop_calls;
    if (output.state != CONTROL_FAULT) output.state = CONTROL_READY;
    output.pwm_left = output.pwm_right = 0.0f;
    output.servo_enabled = 0u;
    output.servo_pulse_us_requested = output.servo_pulse_us_nominal = 0.0f;
}
void control_report_link_fault(void) { output.state = CONTROL_FAULT; output.fault = CONTROL_FAULT_LINK; }
void control_report_invalid_fault(void) { output.state = CONTROL_FAULT; output.fault = CONTROL_FAULT_INVALID_VALUE; }
uint8_t control_reset_fault(void)
{ if (output.state == CONTROL_RUNNING) return 0u; output.state = CONTROL_READY; output.fault = CONTROL_FAULT_NONE; return 1u; }
uint8_t control_request_calibration_bench(void)
{
    if (!reserved || output.state != CONTROL_READY || !params.calibration_confirmed) return 0u;
    output.state = CONTROL_RUNNING; output.mode = CONTROL_MODE_BENCH; ++run_calls; return 1u;
}
uint8_t control_request_servo_cal(float center, float min, float max, float rate, uint32_t version)
{
    if (!reserved || output.state != CONTROL_READY || version != output.params_version || center < min || center > max || rate <= 0.0f) return 0u;
    output.state = CONTROL_RUNNING; output.mode = CONTROL_MODE_SERVO_CAL;
    output.servo_enabled = 1u; output.servo_pulse_us_requested = center; ++run_calls; return 1u;
}
uint8_t control_set_servo_cal_pulse_us(float pulse)
{ if (output.state != CONTROL_RUNNING || pulse < 1400.0f || pulse > 1600.0f) return 0u; output.servo_pulse_us_requested = pulse; return 1u; }
float control_servo_us_per_duty(void) { return 2.0f; }

static uint32_t bits(float value) { uint32_t result; memcpy(&result, &value, sizeof(result)); return result; }
static void heartbeat(uint32_t session)
{
    calibration_debug_mailbox.heartbeat_session_id = session;
    calibration_debug_mailbox.heartbeat_seq = ++heartbeat_sequence;
}
static void request(uint32_t opcode, const uint32_t args[8], uint32_t session, uint8_t bad_crc)
{
    uint32_t payload[11];
    unsigned i;
    payload[0] = session; payload[1] = ++sequence; payload[2] = opcode;
    for (i = 0u; i < 8u; ++i) payload[i + 3u] = args != NULL ? args[i] : 0u;
    calibration_debug_mailbox.session_id = session;
    calibration_debug_mailbox.opcode = opcode;
    for (i = 0u; i < 8u; ++i) calibration_debug_mailbox.args[i] = payload[i + 3u];
    calibration_debug_mailbox.request_crc = calibration_debug_crc32(payload, sizeof(payload)) ^ bad_crc;
    calibration_debug_mailbox.request_seq = sequence; // 模拟主机完成payload后最后提交。
}
static uint32_t command(uint32_t opcode, const uint32_t args[8])
{
    request(opcode, args, 123u, 0u);
    calibration_debug_poll(now_us);
    assert(calibration_debug_mailbox.ack_seq == sequence);
    return calibration_debug_mailbox.ack_status;
}
static void sample(int16_t left, int16_t right, uint32_t dt, uint8_t keepalive)
{
    EncoderMeasurementSnapshot measurement;
    raw_left = (uint16_t)(raw_left + left);
    raw_right = (uint16_t)(raw_right + right);
    now_us += dt;
    if (keepalive) heartbeat(123u);
    (void)encoder_measurement_sample(raw_left, raw_right, now_us);
    encoder_measurement_get_snapshot(&measurement);
    calibration_debug_before_tick(now_us, &measurement);
    output.sample_id = measurement.sample_id;
    if (output.state == CONTROL_RUNNING && output.mode == CONTROL_MODE_BENCH)
    { output.pwm_left = 100.0f; output.pwm_right = 110.0f; }
    if (output.state != CONTROL_RUNNING)
    { output.pwm_left = output.pwm_right = 0.0f; output.servo_enabled = 0u; output.servo_pulse_us_requested = 0.0f; }
    output.servo_pulse_us_nominal = output.servo_pulse_us_requested;
    calibration_debug_after_tick(now_us, &measurement, &output,
                                 output.servo_enabled ? (uint16_t)(output.servo_pulse_us_requested / 2.0f) : 0u);
}
static void setup(void)
{
    unsigned i;
    memset(&params, 0, sizeof(params)); memset(&output, 0, sizeof(output));
    output.state = CONTROL_READY; output.params_version = 1u;
    params.target_left = 3.0f; params.target_right = 4.0f; params.kp_left = 50.0f;
    now_us = 0u; raw_left = raw_right = 0u; sequence = heartbeat_sequence = run_calls = stop_calls = 0u; reserved = reject_restore = 0u;
    encoder_measurement_init(1, 1); steering_map_init(); calibration_debug_init();
    for (i = 0u; i < 12u; ++i) sample(0, 0, 10000u, 0u);
}
static void prepare_servo(void)
{
    uint32_t args[8] = {0};
    args[0] = bits(1500.0f); args[1] = bits(1400.0f); args[2] = bits(1600.0f);
    args[3] = bits(200.0f); args[4] = output.params_version; args[5] = 1u;
    heartbeat(123u); assert(command(CAL_CMD_PREPARE_SERVO, args) == CAL_ACK_OK);
    assert(output.state == CONTROL_READY && reserved && run_calls == 0u);
    assert(!params.calibration_confirmed);
}
static void start(void)
{
    uint32_t args[8] = {0}; args[0] = calibration_debug_mailbox.prepare_token;
    assert(command(CAL_CMD_START, args) == CAL_ACK_OK);
}

int main(void)
{
    uint32_t args[8] = {0};
    EncoderMeasurementSnapshot measurement;
    ControlSnapshot hardware_output;
    unsigned i;
    assert(sizeof(CalibrationDebugMailbox) == 416u && sizeof(CalibrationDebugRecord) == 32u);
    assert(calibration_debug_crc32("123456789", 9u) == 0xcbf43926u);
    setup();
    assert(calibration_debug_mailbox.magic == CALIBRATION_DEBUG_MAGIC);
    assert(calibration_debug_mailbox.build_id == calibration_debug_crc32(calibration_debug_build_stamp, (uint32_t)strlen(calibration_debug_build_stamp)));
    output.state = CONTROL_FAULT; output.fault = CONTROL_FAULT_LINK;
    assert(command(CAL_CMD_CLEAR_FAULT, NULL) == CAL_ACK_OK && output.state == CONTROL_READY); // 无owner时仍可人工清故障。
    encoder_measurement_get_snapshot(&measurement);
    args[0] = bits(1000.0f); args[1] = bits(1000.0f); args[2] = measurement.calibration_version; args[3] = 1u;
    assert(command(CAL_CMD_APPLY_ENCODER, args) == CAL_ACK_OK); // 新连接可应用已由主机核验的完整实测集合。
    memset(args, 0, sizeof(args));
    request(CAL_CMD_PREPARE_COUNT, args, 123u, 1u); calibration_debug_poll(now_us);
    assert(calibration_debug_mailbox.ack_status == CAL_ACK_BAD_CRC && !reserved);
    prepare_servo(); start();
    assert(run_calls == 1u && calibration_debug_mailbox.phase == CAL_PHASE_ACTIVE_SERVO);
    calibration_debug_poll(now_us); assert(run_calls == 1u); // ACK遗失后重复同seq不能二次START。
    encoder_measurement_get_snapshot(&measurement);
    calibration_debug_after_tick(now_us, &measurement, &output, 750u);
    assert(calibration_debug_mailbox.log_count == 0u); // 起点累计已包含的同样本不能再次记录。
    sample(0, 0, 10000u, 1u);
    assert(calibration_debug_mailbox.log_count == 1u && calibration_debug_log[0].servo_duty == 750u);
    args[0] = bits(1550.0f); assert(command(CAL_CMD_SET_SERVO, args) == CAL_ACK_OK);
    request(CAL_CMD_START, args, 123u, 0u); ++calibration_debug_mailbox.stop_seq;
    sample(0, 0, 10000u, 1u); calibration_debug_poll(now_us);
    assert(output.state == CONTROL_READY && !reserved && run_calls == 1u);
    assert(calibration_debug_mailbox.ack_status == CAL_ACK_STOPPED && calibration_debug_mailbox.log_frozen);
    assert(calibration_debug_mailbox.stop_ack_seq == calibration_debug_mailbox.stop_seq);
    assert(calibration_debug_mailbox.command_ready == 1u);
    assert(calibration_debug_log[1].servo_duty == 0u);

    setup(); prepare_servo(); start();
    for (i = 0u; i < 50u; ++i) sample(0, 0, 10000u, 0u);
    assert(output.state == CONTROL_FAULT && output.fault == CONTROL_FAULT_LINK);
    assert(calibration_debug_mailbox.end_reason == CAL_END_LINK_TIMEOUT && !reserved && calibration_debug_mailbox.log_frozen);
    heartbeat(123u); calibration_debug_poll(now_us); assert(output.state == CONTROL_FAULT && run_calls == 1u);

    setup(); prepare_servo(); start();
    sample(0, 0, 20000u, 1u);
    assert(calibration_debug_mailbox.end_reason == CAL_END_MEASUREMENT && output.state == CONTROL_FAULT);

    setup(); prepare_servo(); start();
    for (i = 0u; i < 13000u; ++i) sample(0, 0, 10000u, 1u);
    assert(calibration_debug_mailbox.phase == CAL_PHASE_ACTIVE_SERVO && output.state == CONTROL_RUNNING);
    assert(calibration_debug_mailbox.log_count == 1u && !calibration_debug_mailbox.log_frozen);
    args[0] = calibration_debug_mailbox.prepare_token + 1u;
    assert(command(CAL_CMD_SERVO_END, args) == CAL_ACK_BAD_STATE);
    args[0] = calibration_debug_mailbox.prepare_token;
    assert(command(CAL_CMD_SERVO_END, args) == CAL_ACK_OK);
    assert(!calibration_debug_mailbox.log_frozen);
    sample(0, 0, 10000u, 1u);
    assert(calibration_debug_mailbox.phase == CAL_PHASE_COMPLETE && calibration_debug_mailbox.log_end_reason == CAL_END_MANUAL);
    assert(calibration_debug_mailbox.log_count == 2u && calibration_debug_log[1].servo_duty == 0u);

    setup(); prepare_servo(); start();
    for (i = 0u; i < CALIBRATION_DEBUG_LOG_CAPACITY - 1u; ++i)
    {
        args[0] = bits(i & 1u ? 1550.0f : 1500.0f);
        assert(command(CAL_CMD_SET_SERVO, args) == CAL_ACK_OK);
        sample(0, 0, 6000u, 1u);
    }
    assert(calibration_debug_mailbox.log_count == CALIBRATION_DEBUG_LOG_CAPACITY - 1u);
    assert(!calibration_debug_mailbox.log_frozen && output.state == CONTROL_RUNNING);
    for (i = 0u; i < 2000u; ++i) sample(0, 0, 6000u, 1u); // 最后停止槽仍空时，稳定保持不因容量而提前结束。
    assert(output.state == CONTROL_RUNNING);
    args[0] = bits(1550.0f); assert(command(CAL_CMD_SET_SERVO, args) == CAL_ACK_OK);
    sample(0, 0, 6000u, 1u);
    assert(calibration_debug_mailbox.end_reason == CAL_END_LOG_FULL && output.state == CONTROL_READY);
    assert(!calibration_debug_mailbox.log_frozen);
    sample(0, 0, 6000u, 1u);
    assert(calibration_debug_mailbox.log_frozen && calibration_debug_mailbox.log_count == CALIBRATION_DEBUG_LOG_CAPACITY);
    assert(calibration_debug_log[CALIBRATION_DEBUG_LOG_CAPACITY - 1u].servo_duty == 0u);

    setup(); prepare_servo(); start();
    calibration_debug_mailbox.log_count = CALIBRATION_DEBUG_LOG_CAPACITY;
    encoder_measurement_get_snapshot(&measurement);
    ++measurement.sample_id;
    output.pwm_left = 100.0f; // 模拟after_tick已收到刚施加的非零硬件命令，不能同次报告冻结。
    hardware_output = output;
    calibration_debug_after_tick(now_us, &measurement, &hardware_output, 750u);
    assert(!calibration_debug_mailbox.log_frozen);
    sample(0, 0, 10000u, 1u);
    assert(calibration_debug_mailbox.log_frozen && calibration_debug_mailbox.end_reason == CAL_END_LOG_FULL);

    setup(); now_us = UINT32_MAX - 200000u; encoder_measurement_init(1, 1);
    for (i = 0u; i < 12u; ++i) sample(0, 0, 10000u, 0u);
    prepare_servo(); start();
    for (i = 0u; i < 50u; ++i) sample(0, 0, 10000u, 0u);
    assert(calibration_debug_mailbox.end_reason == CAL_END_LINK_TIMEOUT); // GPT低32位回绕仍按500ms差值停车。

    setup(); args[0] = 1u;
    heartbeat(123u); assert(command(CAL_CMD_PREPARE_COUNT, args) == CAL_ACK_OK);
    args[0] = calibration_debug_mailbox.prepare_token; assert(command(CAL_CMD_COUNT_START, args) == CAL_ACK_OK);
    sample(100, -50, 10000u, 1u);
    assert(command(CAL_CMD_COUNT_END, args) == CAL_ACK_NOT_STATIONARY);
    for (i = 0u; i < 10u; ++i) sample(0, 0, 10000u, 1u);
    // 手转经过旧120秒门槛仍应活动，不受1024条执行器日志容量影响。
    for (i = 0u; i < 13000u; ++i) sample(0, 0, 10000u, 1u);
    assert(calibration_debug_mailbox.phase == CAL_PHASE_ACTIVE_COUNT && !calibration_debug_mailbox.log_frozen);
    assert(output.pwm_left == 0.0f && output.pwm_right == 0.0f && !output.servo_enabled);
    assert(command(CAL_CMD_COUNT_END, args) == CAL_ACK_OK);
    args[0] = 1u;
    assert(command(CAL_CMD_PREPARE_COUNT, args) == CAL_ACK_BAD_STATE); // 没经过零输出PIT不能覆盖待冻结摘要。
    sample(0, 0, 10000u, 0u);
    assert(calibration_debug_mailbox.log_frozen && calibration_debug_mailbox.log_count == 0u);
    assert(calibration_debug_mailbox.count_end_left_lo == 100u);
    assert(calibration_debug_mailbox.count_end_right_lo == (uint32_t)-50 && calibration_debug_mailbox.count_end_right_hi == UINT32_MAX);
    encoder_measurement_get_snapshot(&measurement);
    args[0] = bits(1000.0f); args[1] = bits(1100.0f); args[2] = measurement.calibration_version; args[3] = 1u;
    assert(command(CAL_CMD_APPLY_ENCODER, args) == CAL_ACK_OK);
    assert(command(CAL_CMD_APPLY_ENCODER, args) == CAL_ACK_STALE_VERSION);
    sample(0, 0, 10000u, 0u);
    assert(calibration_debug_mailbox.measurement_version == measurement.calibration_version + 1u);

    setup(); memset(args, 0, sizeof(args)); args[0] = 1u;
    heartbeat(123u); assert(command(CAL_CMD_PREPARE_COUNT, args) == CAL_ACK_OK);
    args[0] = calibration_debug_mailbox.prepare_token; assert(command(CAL_CMD_COUNT_START, args) == CAL_ACK_OK);
    control_request_stop(); // 模拟V1 STOP，手转组本来就是READY，须用代次而不是RUNNING判断。
    sample(0, 0, 10000u, 1u);
    assert(calibration_debug_mailbox.phase == CAL_PHASE_ABORTED && calibration_debug_mailbox.end_reason == CAL_END_CONTROL);
    assert(!reserved && calibration_debug_mailbox.log_frozen);

    setup(); memset(args, 0, sizeof(args));
    args[0] = bits(5.0f); args[1] = bits(6.0f); args[2] = output.params_version; args[3] = 1u;
    heartbeat(123u); assert(command(CAL_CMD_PREPARE_RPM, args) == CAL_ACK_OK);
    assert(params.calibration_confirmed && params.target_left == 5.0f); start();
    for (i = 0u; i < 200u; ++i) sample(0, 0, 10000u, 1u);
    assert(calibration_debug_take_marker_request() == 0);
    calibration_debug_marker_completed(0u, 0u, 100u);
    assert(calibration_debug_mailbox.snapshot[CAL_SNAP_MARKER_MASK] == 1u); // 时间0必须可表示已完成。
    for (i = 0u; i < 400u; ++i) sample(0, 0, 10000u, 1u);
    assert(calibration_debug_take_marker_request() == 1);
    ++calibration_debug_mailbox.stop_seq; sample(0, 0, 10000u, 1u);
    assert(!reserved && !params.calibration_confirmed && params.target_left == 3.0f && params.target_right == 4.0f);
    assert(calibration_debug_mailbox.log_frozen && output.state == CONTROL_READY);
    assert((calibration_debug_mailbox.snapshot_seq & 1u) == 0u);

    setup(); memset(args, 0, sizeof(args));
    args[0] = bits(5.0f); args[1] = bits(6.0f); args[2] = output.params_version; args[3] = 1u;
    heartbeat(123u); assert(command(CAL_CMD_PREPARE_RPM, args) == CAL_ACK_OK); start();
    for (i = 0u; i < 800u; ++i) sample(0, 0, 10000u, 1u);
    assert(calibration_debug_mailbox.log_end_reason == CAL_END_TIME_LIMIT && calibration_debug_mailbox.log_count == 800u);
    assert(!params.calibration_confirmed && params.target_left == 3.0f);

    setup(); memset(args, 0, sizeof(args));
    args[0] = bits(5.0f); args[1] = bits(6.0f); args[2] = output.params_version; args[3] = 1u;
    heartbeat(123u); assert(command(CAL_CMD_PREPARE_RPM, args) == CAL_ACK_OK); start();
    sample(10, 11, 10000u, 1u);
    ++calibration_debug_mailbox.stop_seq; calibration_debug_poll(now_us); // 主循环先接受STOP，下一采样仍可能有惯性计数。
    assert(!calibration_debug_mailbox.log_frozen);
    sample(7, 9, 10000u, 1u);
    assert(calibration_debug_mailbox.log_frozen && calibration_debug_mailbox.log_count == 2u);
    assert(calibration_debug_mailbox.count_end_left_lo == 17u && calibration_debug_mailbox.count_end_right_lo == 20u);
    assert(calibration_debug_log[1].delta_left == 7 && calibration_debug_log[1].delta_right == 9);

    setup(); memset(args, 0, sizeof(args));
    args[0] = bits(5.0f); args[1] = bits(6.0f); args[2] = output.params_version; args[3] = 1u;
    heartbeat(123u); assert(command(CAL_CMD_PREPARE_RPM, args) == CAL_ACK_OK); start();
    reject_restore = 1u; ++calibration_debug_mailbox.stop_seq; sample(0, 0, 10000u, 1u);
    assert(reserved && calibration_debug_busy() && output.state == CONTROL_FAULT);
    assert((calibration_debug_log[0].flags & 32u) != 0u);
    reject_restore = 0u; assert(command(CAL_CMD_RELEASE, NULL) == CAL_ACK_OK);
    assert(!reserved && !calibration_debug_busy() && !params.calibration_confirmed);
    // RPM准备入口采用同一400上限，超限不占用标定控制权；本轮3m/s目标和边界可准备。
    setup(); memset(args, 0, sizeof(args));
    args[0] = bits(CONTROL_TARGET_COUNTS_MAX + 0.01f); args[1] = bits(CONTROL_TARGET_COUNTS_MAX);
    args[2] = output.params_version; args[3] = 1u;
    heartbeat(123u); assert(command(CAL_CMD_PREPARE_RPM, args) == CAL_ACK_BAD_ARGUMENT);
    assert(!reserved && output.state == CONTROL_READY);
    args[0] = bits(348.146f); args[1] = bits(340.820f);
    assert(command(CAL_CMD_PREPARE_RPM, args) == CAL_ACK_OK && reserved);
    setup(); args[0] = bits(CONTROL_TARGET_COUNTS_MAX); args[1] = bits(CONTROL_TARGET_COUNTS_MAX);
    args[2] = output.params_version; args[3] = 1u;
    heartbeat(123u); assert(command(CAL_CMD_PREPARE_RPM, args) == CAL_ACK_OK);

    puts("PASS calibration mailbox CRC/STOP/session/lease/time/count/restore/marker; hardware pending");
    return 0;
}
