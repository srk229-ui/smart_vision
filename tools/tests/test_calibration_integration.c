#include "control.h"
#include "calibration_debug.h"
#include "encoder_measurement.h"
#include "steering_map.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static uint32_t time_us, sequence, heartbeat_sequence;
static uint16_t left_raw, right_raw;

static uint32_t bits(float value)
{
    uint32_t result;
    memcpy(&result, &value, sizeof(result));
    return result;
}

static void heartbeat(void)
{
    calibration_debug_mailbox.heartbeat_session_id = 42u;
    calibration_debug_mailbox.heartbeat_seq = ++heartbeat_sequence;
}

static void tick(uint32_t dt, int16_t left, int16_t right)
{
    EncoderMeasurementSnapshot measurement;
    ControlSnapshot output;
    uint8_t accepted;
    time_us += dt;
    left_raw = (uint16_t)(left_raw - left); // 模拟左接线符号-1，硬件寄存器自然回绕。
    right_raw = (uint16_t)(right_raw + right);
    accepted = encoder_measurement_sample(left_raw, right_raw, time_us);
    encoder_measurement_get_snapshot(&measurement);
    // 顺序与真实ISR一致，使用真实控制/诊断/测量模块，未复制控制算法。
    calibration_debug_before_tick(time_us, &measurement);
    if (!accepted) control_report_invalid_fault();
    control_tick(measurement.delta_left, measurement.delta_right);
    control_get_snapshot(&output);
    calibration_debug_after_tick(time_us, &measurement, &output, (uint16_t)output.servo_duty_cmd);
}

static uint32_t command(uint32_t opcode, const uint32_t *args, unsigned count)
{
    uint32_t request[11] = {42u, 0u, 0u};
    unsigned i;
    request[1] = ++sequence; request[2] = opcode;
    for (i = 0u; i < count; ++i) request[3u + i] = args[i];
    calibration_debug_mailbox.session_id = request[0];
    calibration_debug_mailbox.opcode = opcode;
    for (i = 0u; i < 8u; ++i) calibration_debug_mailbox.args[i] = request[3u + i];
    calibration_debug_mailbox.request_crc = calibration_debug_crc32(request, sizeof(request));
    calibration_debug_mailbox.request_seq = request[1];
    calibration_debug_poll(time_us);
    assert(calibration_debug_mailbox.ack_seq == sequence);
    return calibration_debug_mailbox.ack_status;
}

static void setup(void)
{
    unsigned i;
    time_us = sequence = heartbeat_sequence = 0u;
    left_raw = right_raw = 0u;
    control_init(92.5f, 85.0f, 100.0f, -1.0f);
    control_configure_servo_pwm(10000u, 50u);
    control_set_output_enabled(0u, 1u);
    encoder_measurement_init(-1, 1);
    steering_map_init();
    calibration_debug_init();
    for (i = 0u; i < 12u; ++i) tick(10000u, 0, 0);
}

int main(void)
{
    uint32_t args[6];
    uint32_t version;
    ControlParams params;
    ControlSnapshot output;
    unsigned i;
    setup();
    control_get_params(&params, &version);
    args[0] = bits(1526.0f); args[1] = bits(1444.0f); args[2] = bits(1610.0f);
    args[3] = bits(500.0f); args[4] = version; args[5] = 1u;
    heartbeat();
    assert(command(CAL_CMD_PREPARE_SERVO, args, 6u) == CAL_ACK_OK);
    assert(!control_request_run(CONTROL_MODE_BENCH));
    // 准备设置STOP屏障；没有下一PIT就START必须由真实控制器拒绝。
    args[0] = calibration_debug_mailbox.prepare_token;
    assert(command(CAL_CMD_START, args, 1u) == CAL_ACK_CONTROL_REJECTED);
    tick(10000u, 0, 0);
    control_get_params(&params, &version);
    args[0] = bits(1526.0f); args[1] = bits(1444.0f); args[2] = bits(1610.0f);
    args[3] = bits(500.0f); args[4] = version; args[5] = 1u;
    heartbeat(); assert(command(CAL_CMD_PREPARE_SERVO, args, 6u) == CAL_ACK_OK);
    tick(10000u, 0, 0);
    args[0] = calibration_debug_mailbox.prepare_token;
    heartbeat(); assert(command(CAL_CMD_START, args, 1u) == CAL_ACK_OK);
    for (i = 0u; i < 15u; ++i)
    {
        heartbeat(); tick(10000u, 1, 2);
        control_get_snapshot(&output);
        assert(output.pwm_left == 0.0f && output.pwm_right == 0.0f && output.servo_enabled);
    }
    // 手动STOP不立即冒充已施加零PWM，必须经过真实控制周期才冻结。
    ++calibration_debug_mailbox.stop_seq;
    calibration_debug_poll(time_us);
    assert(!calibration_debug_mailbox.log_frozen);
    tick(10000u, 0, 0);
    assert(calibration_debug_mailbox.log_frozen && calibration_debug_mailbox.log_end_reason == CAL_END_STOP);
    assert(calibration_debug_mailbox.log_count == 2u);
    assert(calibration_debug_log[1].servo_duty == 0u);
    // 主循环START锚点不得重复记入已经计入起点的编码器样本。
    assert(calibration_debug_log[0].delta_left == 1 && calibration_debug_log[0].delta_right == 2);

    setup(); time_us = UINT32_MAX - 200000u; encoder_measurement_init(-1, 1);
    for (i = 0u; i < 12u; ++i) tick(10000u, 0, 0);
    control_get_params(&params, &version);
    args[0] = bits(1526.0f); args[1] = bits(1444.0f); args[2] = bits(1610.0f);
    args[3] = bits(1.0f); args[4] = version; args[5] = 1u;
    heartbeat(); assert(command(CAL_CMD_PREPARE_SERVO, args, 6u) == CAL_ACK_OK);
    tick(10000u, 0, 0); args[0] = calibration_debug_mailbox.prepare_token;
    heartbeat(); assert(command(CAL_CMD_START, args, 1u) == CAL_ACK_OK);
    args[0] = bits(1600.0f); assert(command(CAL_CMD_SET_SERVO, args, 1u) == CAL_ACK_OK);
    // 真实控制1us/s慢斜坡跨GPT回绕并保持130秒；只有整数duty改变才留事件，不被100Hz float变化填满。
    for (i = 0u; i < 13000u; ++i)
    {
        heartbeat(); tick(10000u, 0, 0);
        control_get_snapshot(&output);
        assert(output.servo_enabled && output.pwm_left == 0.0f && output.pwm_right == 0.0f);
    }
    assert(calibration_debug_mailbox.phase == CAL_PHASE_ACTIVE_SERVO && calibration_debug_mailbox.log_count < 100u);
    assert(output.servo_pulse_us_requested == 1600.0f);
    args[0] = calibration_debug_mailbox.prepare_token;
    assert(command(CAL_CMD_SERVO_END, args, 1u) == CAL_ACK_OK);
    assert(!calibration_debug_mailbox.log_frozen);
    tick(10000u, 0, 0);
    assert(calibration_debug_mailbox.phase == CAL_PHASE_COMPLETE && calibration_debug_mailbox.log_end_reason == CAL_END_MANUAL);
    assert(calibration_debug_log[calibration_debug_mailbox.log_count - 1u].servo_duty == 0u);

    setup(); args[0] = 1u;
    heartbeat(); assert(command(CAL_CMD_PREPARE_COUNT, args, 1u) == CAL_ACK_OK);
    tick(10000u, 0, 0); args[0] = calibration_debug_mailbox.prepare_token;
    heartbeat(); assert(command(CAL_CMD_COUNT_START, args, 1u) == CAL_ACK_OK);
    control_request_stop(); tick(10000u, 0, 0); // 模拟V1的普通STOP，即使COUNT本来READY也应取消。
    assert(calibration_debug_mailbox.phase == CAL_PHASE_ABORTED && calibration_debug_mailbox.log_frozen);

    setup(); time_us = UINT32_MAX - 200000u; encoder_measurement_init(-1, 1);
    for (i = 0u; i < 12u; ++i) tick(10000u, 0, 0);
    args[0] = 1u; heartbeat(); assert(command(CAL_CMD_PREPARE_COUNT, args, 1u) == CAL_ACK_OK);
    tick(10000u, 0, 0); args[0] = calibration_debug_mailbox.prepare_token;
    heartbeat(); assert(command(CAL_CMD_COUNT_START, args, 1u) == CAL_ACK_OK);
    for (i = 0u; i < 13000u; ++i) { heartbeat(); tick(10000u, 0, 0); }
    assert(calibration_debug_mailbox.phase == CAL_PHASE_ACTIVE_COUNT && !calibration_debug_mailbox.log_frozen);
    control_get_snapshot(&output);
    assert(output.state == CONTROL_READY && output.pwm_left == 0.0f && output.pwm_right == 0.0f && !output.servo_enabled);
    assert(command(CAL_CMD_COUNT_END, args, 1u) == CAL_ACK_OK); tick(10000u, 0, 0);
    assert(calibration_debug_mailbox.phase == CAL_PHASE_COMPLETE && calibration_debug_mailbox.log_frozen);

    setup(); args[0] = 1u;
    heartbeat(); assert(command(CAL_CMD_PREPARE_COUNT, args, 1u) == CAL_ACK_OK);
    tick(10000u, 0, 0); args[0] = calibration_debug_mailbox.prepare_token;
    heartbeat(); assert(command(CAL_CMD_COUNT_START, args, 1u) == CAL_ACK_OK);
    tick(20000u, 3, 4);
    assert(calibration_debug_mailbox.log_end_reason == CAL_END_MEASUREMENT);
    control_get_snapshot(&output); assert(output.state == CONTROL_FAULT && output.pwm_left == 0.0f);
    // 真实控制与邮箱共同验证本轮3m/s目标，运行仍需人工START且8秒保护保持。
    setup(); control_set_output_enabled(1u, 1u); control_get_params(&params, &version);
    memset(args, 0, sizeof(args));
    args[0] = bits(348.146f); args[1] = bits(340.820f); args[2] = version; args[3] = 1u;
    heartbeat(); assert(command(CAL_CMD_PREPARE_RPM, args, 4u) == CAL_ACK_OK);
    tick(10000u, 0, 0); args[0] = calibration_debug_mailbox.prepare_token;
    heartbeat(); assert(command(CAL_CMD_START, args, 1u) == CAL_ACK_OK);
    for(i = 0u; i < 700u; ++i) { heartbeat(); tick(10000u, 0, 0); }
    control_get_snapshot(&output);
    assert(output.state == CONTROL_RUNNING && output.fault == CONTROL_FAULT_NONE);
    assert(output.target_left > 348.14f && output.target_right > 340.81f);
    assert(output.pwm_left <= 5000.0f && output.pwm_right <= 5000.0f);
    for(i = 0u; i < 100u; ++i) { heartbeat(); tick(10000u, 0, 0); }
    assert(calibration_debug_mailbox.log_end_reason == CAL_END_TIME_LIMIT);
    control_get_snapshot(&output); assert(output.state == CONTROL_READY && output.pwm_left == 0.0f && output.pwm_right == 0.0f);

    puts("PASS calibration integration: real control + measurement + mailbox, barriers, zero outputs, stop, fault");
    return 0;
}
