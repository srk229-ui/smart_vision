#include "control.h"
#include "pid.h"
#include "calibration_profile.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static ControlParams params;
static uint32_t version;
static uint32_t frame;

static ControlSnapshot snapshot(void)
{
    ControlSnapshot s;
    control_get_snapshot(&s);
    return s;
}

static void setup(void)
{
    control_init(92.5f, 85.0f, 100.0f, -1.0f);
    control_set_output_enabled(1u, 1u);
    control_tick(0, 0);
    frame = 0u;
    control_get_params(&params, &version);
    params.target_left = 10.0f;
    params.target_right = 20.0f;
    params.calibration_confirmed = 1u;
    assert(control_apply_params(&params, version));
}

static void publish(float confidence, float error, uint8_t valid)
{
    VisionResult vision;
    memset(&vision, 0, sizeof(vision));
    vision.frame_id = ++frame;
    vision.capture_time_ms = control_now_ms();
    vision.confidence = confidence;
    vision.error_px = error;
    vision.valid = valid;
    vision.track_lost = !valid;
    control_publish_vision(&vision);
}

static void track_ready(void)
{
    publish(1.0f, 0.0f, 1u);
    control_tick(0, 0);
    publish(1.0f, 0.0f, 1u);
    control_tick(0, 0);
    publish(1.0f, 0.0f, 1u);
}

static void local_setup(void)
{
    setup();
    control_get_params(&params, &version);
    params.calibration_confirmed = 0u;
    assert(control_apply_params(&params, version));
}

static void test_local_track(void)
{
    ControlSnapshot s;
    VisionResult stale;
    unsigned i;

    // 本地按键不依赖远程确认：上电不运行、没有图像拒绝，取得单帧后才可启动。
    control_init(92.5f, 85.0f, 100.0f, -1.0f);
    control_set_output_enabled(1u, 1u);
    publish(0.4f, 0.0f, 1u);
    assert(!control_request_local_track_run()); // 初始化同采样编号仍受写零周期屏障约束。
    control_tick(0, 0);
    assert(control_request_local_track_run());
    control_tick(0, 0);
    assert(snapshot().state == CONTROL_RUNNING && snapshot().pwm_left > 0.0f);
    local_setup();
    assert(!control_request_local_track_run());
    publish(0.4f, 0.0f, 1u);
    assert(!control_request_run(CONTROL_MODE_TRACK));
    assert(!control_request_run(CONTROL_MODE_BENCH));
    assert(control_request_local_track_run());
    control_request_stop();
    assert(!control_request_local_track_run());
    control_tick(0, 0);
    assert(control_request_local_track_run());

    // 最新坏帧在CLEAR后仍是坏帧；重复的旧好图不能重新取得资格，必须再收到真正新好图。
    local_setup(); publish(1.0f, 0.0f, 1u);
    memset(&stale, 0, sizeof(stale));
    stale.frame_id = frame; stale.capture_time_ms = control_now_ms();
    stale.valid = 1u; stale.confidence = 1.0f;
    publish(0.0f, 0.0f, 0u);
    assert(control_reset_fault());
    assert(!control_request_local_track_run());
    control_publish_vision(&stale);
    assert(!control_request_local_track_run());
    publish(1.0f, 0.0f, 1u);
    assert(control_request_local_track_run());

    // 第500ms仍允许运行，第510ms按最后有效采集时间停车；无效帧数量不额外停车。
    for (i = 0; i < 50u; ++i)
    {
        publish(0.0f, 0.0f, 0u);
        control_tick(0, 0);
        assert(snapshot().state == CONTROL_RUNNING);
    }
    assert(snapshot().vision_age_ms == 500u);
    control_tick(0, 0);
    s = snapshot();
    assert(s.state == CONTROL_FAULT && s.fault == CONTROL_FAULT_VISION_TIMEOUT);
    assert(s.pwm_left == 0.0f && s.pwm_right == 0.0f && !s.servo_enabled);
    publish(1.0f, 0.0f, 1u);
    assert(!control_request_local_track_run()); // 故障同采样内仍必须先写零。
    control_tick(0, 0);
    publish(0.0f, 0.0f, 0u);
    assert(!control_request_local_track_run());
    assert(snapshot().fault == CONTROL_FAULT_VISION_TIMEOUT); // 失败启动不能顺带清故障。
    publish(1.0f, 0.0f, 1u);
    assert(snapshot().state == CONTROL_FAULT); // 图像恢复本身不会启动或解除故障。
    assert(control_request_local_track_run());
    assert(snapshot().fault == CONTROL_FAULT_NONE);

    // 启动也按采集年龄500ms统一边界判断；新收到的过期图和未来时间戳均不能续命。
    local_setup(); publish(1.0f, 0.0f, 1u);
    for (i = 0; i < 50u; ++i) control_tick(0, 0);
    assert(control_request_local_track_run());
    local_setup(); publish(1.0f, 0.0f, 1u);
    for (i = 0; i < 51u; ++i) control_tick(0, 0);
    assert(!control_request_local_track_run());
    memset(&stale, 0, sizeof(stale)); stale.frame_id = ++frame;
    stale.capture_time_ms = control_now_ms() - 501u; stale.valid = 1u; stale.confidence = 1.0f;
    control_publish_vision(&stale);
    assert(!control_request_local_track_run());
    stale.frame_id = ++frame; stale.capture_time_ms = control_now_ms() + 10u;
    control_publish_vision(&stale);
    assert(!control_request_local_track_run());

    // 短时坏图沿用最后有效转向，并维持原半速规则；好图恢复后正常更新转向和目标。
    local_setup();
    control_get_params(&params, &version);
    params.accel_counts_per_s = 1000.0f; params.servo_rate_deg_s = 360.0f;
    assert(control_apply_params(&params, version));
    publish(1.0f, 0.0f, 1u); assert(control_request_local_track_run());
    publish(1.0f, 10.0f, 1u);
    control_tick(0, 0); control_tick(0, 0);
    assert(fabsf(snapshot().servo_cmd - 94.7f) < 0.001f);
    for (i = 0; i < 20u; ++i) { publish(0.0f, -100.0f, 0u); control_tick(0, 0); }
    s = snapshot();
    assert(s.state == CONTROL_RUNNING && fabsf(s.servo_cmd - 94.7f) < 0.001f);
    // 命令94.7量化为1552us，右轮映射约8.714286度；扣中位5度后，半速目标约3.758453/12.483094。
    assert(fabsf(s.target_left - 3.758453f) < 0.001f && fabsf(s.target_right - 12.483094f) < 0.001f);
    // 回中后差速比例按4/s归零，不再与基础速度共用慢斜坡；等待8拍覆盖原0.2483比例。
    publish(1.0f, 0.0f, 1u);
    for (i = 0u; i < 8u; ++i) control_tick(0, 0);
    assert(fabsf(snapshot().servo_cmd - 92.5f) < 0.001f);
    assert(snapshot().target_left == 10.0f && snapshot().target_right == 20.0f);

    // 本地入口不能解除初始化/非法计算/诊断失联故障，也不能绕过独占或执行器编译许可。
    local_setup(); publish(1.0f, 0.0f, 1u); control_report_invalid_fault(); control_tick(0, 0);
    assert(!control_request_local_track_run() && snapshot().fault == CONTROL_FAULT_INVALID_VALUE);
    local_setup(); publish(1.0f, 0.0f, 1u); control_report_initialization_fault(); control_tick(0, 0);
    assert(!control_request_local_track_run() && !control_reset_fault());
    local_setup(); publish(1.0f, 0.0f, 1u); assert(control_request_local_track_run());
    control_report_link_fault(); control_tick(0, 0);
    assert(!control_request_local_track_run() && snapshot().fault == CONTROL_FAULT_LINK);
    local_setup(); publish(1.0f, 0.0f, 1u); control_set_calibration_reserved(1u);
    assert(!control_request_local_track_run()); control_set_calibration_reserved(0u);
    control_set_output_enabled(0u, 1u); control_tick(0, 0);
    assert(!control_request_local_track_run());
    control_set_output_enabled(1u, 0u); control_tick(0, 0);
    assert(!control_request_local_track_run());
    local_setup(); publish(1.0f, 0.0f, 1u); control_get_params(&params, &version);
    params.target_left = params.target_right = 0.0f;
    assert(control_apply_params(&params, version));
    assert(!control_request_local_track_run());
}

static void test_default_pd(void)
{
    unsigned i;
    setup(); track_ready(); assert(control_request_local_track_run());
    publish(1.0f,10.0f,1u);
    for(i=0u;i<20u;++i) control_tick(0,0);
    assert(fabsf(snapshot().servo_cmd-94.7f)<0.001f); // 首帧D=0，只含0.22乘10。
    publish(1.0f,20.0f,1u);
    for(i=0u;i<20u;++i) control_tick(0,0);
    assert(fabsf(snapshot().servo_cmd-97.0f)<0.001f); // 200ms差分：92.5+4.4+0.002乘50。
    control_request_stop(); control_tick(0,0);
    assert(!snapshot().pwm_left && !snapshot().servo_enabled);
}

static void test_mapped_differential(void)
{
    ControlSnapshot s;
    unsigned i;
    setup(); control_get_params(&params, &version);
    params.accel_counts_per_s = 1000.0f; params.servo_rate_deg_s = 360.0f;
    params.steer_kp = 1.0f; params.steer_kd = 0.0f; params.curve_speed_factor = 1.0f;
    assert(control_apply_params(&params, version));
    track_ready(); assert(control_request_run(CONTROL_MODE_TRACK));
    // 右转90.3命令度量化为1502us，映射相对角约-5.268293度；两侧斜率不同，不能沿用命令差。
    publish(1.0f, -2.2f, 1u);
    for (i = 0u; i < 28u; ++i) control_tick(0, 0);
    s = snapshot();
    assert(s.state == CONTROL_RUNNING && fabsf(s.servo_cmd - 90.3f) < 0.001f);
    assert(fabsf(s.target_left - 13.526997f) < 0.001f && fabsf(s.target_right - 12.946007f) < 0.001f);
    // 中位映射原始+5度必须被抵消，回中后等待独立差速比例归零。
    publish(1.0f, 0.0f, 1u);
    for (i = 0u; i < 12u; ++i) control_tick(0, 0);
    assert(snapshot().target_left == 10.0f && snapshot().target_right == 20.0f);
    // 两侧实测端点分别相对中位+12/-18度；沿用增益10后都由±70%限幅约束。
    publish(1.0f, 7.5f, 1u);
    for (i = 0u; i < 40u; ++i) control_tick(0, 0);
    assert(fabsf(snapshot().target_left - 3.0f) < 0.001f && fabsf(snapshot().target_right - 34.0f) < 0.001f);
    // +0.7至-0.7按4/s至少需35拍；等待40拍覆盖舵机跨中位和比例渐变。
    publish(1.0f, -7.5f, 1u);
    for (i = 0u; i < 40u; ++i) control_tick(0, 0);
    assert(fabsf(snapshot().target_left - 17.0f) < 0.001f && fabsf(snapshot().target_right - 6.0f) < 0.001f);
    control_request_stop(); control_tick(0, 0);
    // 人工扩大命令到−33至+20deg外推角域以外时，仍须在输出前锁故障。
    control_get_params(&params, &version); params.servo_max = 120.0f;
    assert(control_apply_params(&params, version));
    track_ready(); assert(control_request_run(CONTROL_MODE_TRACK));
    publish(1.0f, 100.0f, 1u);
    for (i = 0u; i < 28u && snapshot().state == CONTROL_RUNNING; ++i)
    {
        control_tick(0, 0); s = snapshot();
        // 每个仍运行的周期都须在−33至+20deg有界外推角域内；不能先发出越界脉宽再停车。
        if (s.state == CONTROL_RUNNING)
            assert(s.servo_pulse_us_nominal >= 1354.0f && s.servo_pulse_us_nominal <= 1630.0f);
    }
    s = snapshot();
    assert(s.state == CONTROL_FAULT && s.fault == CONTROL_FAULT_INVALID_VALUE);
    assert(!s.target_left && !s.target_right && !s.pwm_left && !s.pwm_right && !s.servo_enabled);
    assert(!s.integral_left && !s.integral_right);
}

static void test_extrapolated_limits(void)
{
    ControlSnapshot s;
    unsigned i;
    setup(); control_get_params(&params, &version);
    params.servo_min = CALIBRATION_SERVO_RIGHT_COMMAND; params.servo_max = CALIBRATION_SERVO_LEFT_COMMAND;
    params.accel_counts_per_s = 1000.0f; params.servo_rate_deg_s = 360.0f;
    params.steer_kp = 1.0f; params.steer_kd = 0.0f; params.curve_speed_factor = 1.0f;
    assert(control_apply_params(&params, version));
    track_ready(); assert(control_request_run(CONTROL_MODE_TRACK));
    // 持续刷新有效近带，独立检查左右外推端点与差速上限；这些断言不代表实车机械角验收。
    for (i = 0u; i < 40u; ++i) { publish(1.0f, 100.0f, 1u); control_tick(0, 0); }
    s = snapshot();
    assert(s.state == CONTROL_RUNNING && s.servo_cmd == CALIBRATION_SERVO_LEFT_COMMAND);
    assert(s.servo_pulse_us_nominal == 1630.0f && fabsf(s.target_left - 3.0f) < 0.001f && fabsf(s.target_right - 34.0f) < 0.001f);
    for (i = 0u; i < 40u; ++i) { publish(1.0f, -100.0f, 1u); control_tick(0, 0); }
    s = snapshot();
    assert(s.state == CONTROL_RUNNING && s.servo_cmd == CALIBRATION_SERVO_RIGHT_COMMAND);
    assert(s.servo_pulse_us_nominal == 1354.0f && fabsf(s.target_left - 17.0f) < 0.001f && fabsf(s.target_right - 6.0f) < 0.001f);
    control_request_stop(); control_tick(0, 0);
    assert(!snapshot().servo_enabled && !snapshot().pwm_left && !snapshot().pwm_right);
}

static VisionResult make_preview_frame(int direction)
{
    VisionResult vision;
    unsigned row;
    memset(&vision, 0, sizeof(vision));
    vision.frame_id = ++frame; vision.capture_time_ms = control_now_ms();
    vision.valid = 1u; vision.confidence = 1.0f; vision.error_px = 0.0f;
    // 近带80至100仍居中，50至70已出现弯道；71至79平滑连接，远带不能凭空跳到另一条路。
    for (row = 42u; row < 120u; ++row)
    {
        int offset = row >= 80u ? 0 : (row <= 70u ? 24 : (int)((80u - row) * 24u / 10u));
        unsigned middle = (unsigned)(94 - direction * offset);
        vision.middle[row] = (uint8_t)middle; vision.left[row] = (uint8_t)(middle - 20u);
        vision.right[row] = (uint8_t)(middle + 20u); vision.row_quality[row] = 2u; ++vision.valid_rows;
    }
    return vision;
}

static void preview_setup(void)
{
    setup(); control_get_params(&params, &version);
    // 使用固定1m/s样本的分轮基础目标；50/s基础斜坡保留，前瞻应在首tick驱动两轮不同目标。
    params.target_left = calibration_profile_counts_per_rev_left * 0.01f / calibration_profile_circumference_left_m;
    params.target_right = calibration_profile_counts_per_rev_right * 0.01f / calibration_profile_circumference_right_m;
    assert(control_apply_params(&params, version));
}

static void test_preview_control(void)
{
    VisionResult vision;
    ControlPreviewSnapshot p;
    ControlSnapshot s;
    unsigned i;
    preview_setup(); vision = make_preview_frame(1); control_publish_vision(&vision);
    control_get_preview_snapshot(&p);
    assert(p.valid && p.row_shift == 30u && p.near_error_px == 0.0f);
    assert(fabsf(p.far_error_px - 24.0f) < 0.001f && fabsf(p.effective_error_px - 18.0f) < 0.001f);
    assert(control_request_local_track_run()); control_tick(0, 0);
    s = snapshot(); control_get_preview_snapshot(&p);
    // 最新远带观测在启动后第一拍即进入舵机/后轮，基础各0.5计数；差速比例本拍只变化0.04。
    assert(s.servo_cmd > 92.5f && p.lead_active && p.differential_command > s.servo_cmd);
    assert(fabsf(p.base_target_left - 0.5f) < 0.001f && fabsf(p.base_target_right - 0.5f) < 0.001f);
    assert(s.target_left < 0.5f && s.target_right > 0.5f && fabsf(p.differential_applied - 0.04f) < 0.001f);
    // 无新帧时100ms后远带/预测退场，但500ms视觉保护仍按原采集时间计时。
    for (i = 0u; i < 10u; ++i) { control_publish_vision(&vision); control_tick(0, 0); }
    control_get_preview_snapshot(&p);
    assert(!p.valid && !p.lead_active && snapshot().state == CONTROL_RUNNING && snapshot().vision_age_ms == 110u);
    assert(snapshot().vision_error_px == 0.0f);
    control_request_stop(); control_get_preview_snapshot(&p);
    assert(!p.valid && !p.lead_active && !p.base_target_left && !p.differential_applied);

    preview_setup(); vision = make_preview_frame(-1); control_publish_vision(&vision);
    assert(control_request_local_track_run()); control_tick(0, 0);
    s = snapshot(); control_get_preview_snapshot(&p);
    assert(s.servo_cmd < 92.5f && s.target_left > 0.5f && s.target_right < 0.5f && p.differential_applied < 0.0f);

    // 连接中断5行时，远处道路不得作为提前转弯证据。
    preview_setup(); vision = make_preview_frame(1);
    for (i = 74u; i <= 78u; ++i) vision.row_quality[i] = 0u;
    control_publish_vision(&vision); control_get_preview_snapshot(&p);
    assert(!p.valid && p.blend == 0.0f && snapshot().vision_error_px == 0.0f);
    // 实际近带误差与配置近带不同，说明可能已改用自适应带，此时退回实际误差。
    vision = make_preview_frame(1); vision.error_px = 30.0f; control_publish_vision(&vision);
    control_get_preview_snapshot(&p); assert(!p.valid && snapshot().vision_error_px == 30.0f);

    // STOP后无新帧重启，旧融合误差不得重新进入首拍P，也不能绕开100ms前瞻有效期。
    preview_setup(); vision = make_preview_frame(1); control_publish_vision(&vision);
    assert(control_request_local_track_run()); control_tick(0, 0);
    control_request_stop(); control_tick(0, 0);
    assert(control_request_local_track_run()); control_tick(0, 0); control_get_preview_snapshot(&p);
    assert(!p.valid && !p.lead_active && snapshot().vision_error_px == 0.0f && snapshot().servo_cmd == 92.5f);

    // 先在RUNNING接收同向帧建立D资格，再让远带反向；近带不变时不能制造额外D脉冲。
    preview_setup(); vision = make_preview_frame(1); control_publish_vision(&vision);
    assert(control_request_local_track_run()); control_tick(0, 0);
    vision = make_preview_frame(1); control_publish_vision(&vision); control_tick(0, 0);
    vision = make_preview_frame(-1); control_publish_vision(&vision); control_tick(0, 0);
    control_get_preview_snapshot(&p); assert(p.differential_command == 92.5f); // 前轮尚在左侧时，反向预测只先到中位。
    control_tick(0, 0); control_get_preview_snapshot(&p);
    assert(p.lead_active && fabsf(p.differential_command - 88.54f) < 0.01f);
    publish(0.0f, 0.0f, 0u); control_tick(0, 0); control_get_preview_snapshot(&p);
    assert(!p.valid && !p.lead_active);
}

int main(void)
{
    ControlSnapshot s;
    VisionResult old;
    unsigned i;
    control_init(92.5f, 85.0f, 100.0f, -1.0f);
    control_set_output_enabled(1u, 1u);
    control_tick(0, 0);
    assert(snapshot().state == CONTROL_READY && snapshot().pwm_left == 0.0f);
    assert(!control_request_run(CONTROL_MODE_BENCH));
    control_get_params(&params, &version);
    assert(params.steer_kp==0.22f && params.steer_kd==0.002f && params.servo_rate_deg_s==60.0f);
    assert(params.kp_left == 17.5f && params.kp_right == 17.5f);
    assert(params.ki_left == 480.0f && params.ki_right == 480.0f);
    assert(speed_pid_l.kp == params.kp_left && speed_pid_r.kp == params.kp_right);
    assert(speed_pid_l.ki == params.ki_left && speed_pid_r.ki == params.ki_right);
    // 实际默认参数反换算成m/s，应分轮达到1.2；READY仍无输出，不消耗人工启动资格。
    assert(fabsf(params.target_left * calibration_profile_circumference_left_m /
                 calibration_profile_counts_per_rev_left * 100.0f - 1.2f) < 0.000001f);
    assert(fabsf(params.target_right * calibration_profile_circumference_right_m /
                 calibration_profile_counts_per_rev_right * 100.0f - 1.2f) < 0.000001f);
    assert(params.target_left > params.target_right && !params.calibration_confirmed);
    assert(params.vision_timeout_ms == 500u && params.lost_frame_limit == 3u);
    assert(speed_pid_l.integrator == 0.0f && speed_pid_r.integrator == 0.0f);

    // 人工确认后首周期目标为0.5；P=17.5×0.5=8.75，I=480×0.5×0.010=2.4，整数PWM为11。
    setup();
    assert(control_request_run(CONTROL_MODE_BENCH));
    control_tick(0, 0);
    s = snapshot();
    assert(s.target_left == 0.5f && s.target_right == 0.5f);
    assert(speed_pid_l.out_p == 8.75f && speed_pid_r.out_p == 8.75f);
    assert(fabsf(speed_pid_l.out_i - 2.4f) < 0.0001f && fabsf(speed_pid_r.out_i - 2.4f) < 0.0001f);
    assert(s.pwm_left == 11.0f && s.pwm_right == 11.0f);
    control_request_stop();
    s = snapshot();
    assert(s.pwm_left == 0.0f && s.pwm_right == 0.0f);
    assert(speed_pid_l.integrator == 0.0f && speed_pid_r.integrator == 0.0f);

    // 兼容TRACK接口仍保留人工确认，但单帧即可启动，不再存在三帧或0.6资格门槛。
    setup(); publish(1.0f, 0.0f, 1u);
    assert(control_request_run(CONTROL_MODE_TRACK));
    control_request_stop();
    setup(); publish(0.4f, 0.0f, 1u);
    assert(control_request_run(CONTROL_MODE_TRACK));

    // 最新失效帧必须拒绝；500ms内更早的好帧不能替代最新有效性。
    setup(); track_ready(); publish(0.0f, 0.0f, 0u);
    assert(!control_request_run(CONTROL_MODE_TRACK));
    setup(); track_ready();
    for (i = 0; i < 51u; ++i) control_tick(0, 0);
    assert(!control_request_run(CONTROL_MODE_TRACK));

    setup(); track_ready();
    control_get_params(&params, &version); params.calibration_confirmed = 0u;
    assert(control_apply_params(&params, version));
    assert(!control_request_run(CONTROL_MODE_TRACK));

    setup();
    assert(!control_request_run(CONTROL_MODE_TRACK));
    track_ready();
    assert(control_request_run(CONTROL_MODE_TRACK));
    control_tick(0, 0);
    s = snapshot();
    assert(fabsf(s.target_left - 0.5f) < 0.0001f && s.pwm_left > 0.0f);
    control_get_params(&params, &version);
    assert(!control_apply_params(&params, version));
    assert(!control_request_run(CONTROL_MODE_TRACK));
    while (snapshot().state == CONTROL_RUNNING) control_tick(0, 0);
    s = snapshot();
    assert(s.state == CONTROL_FAULT && s.fault == CONTROL_FAULT_VISION_TIMEOUT);
    assert(s.pwm_left == 0.0f && s.pwm_right == 0.0f && !s.servo_enabled);
    assert(!control_reset_fault());
    control_tick(0, 0);
    assert(control_reset_fault());
    assert(!control_request_run(CONTROL_MODE_TRACK));

    setup(); track_ready(); assert(control_request_run(CONTROL_MODE_TRACK));
    publish(0.0f, 0.0f, 0u); publish(0.0f, 0.0f, 0u); publish(0.0f, 0.0f, 0u);
    control_tick(0, 0);
    assert(snapshot().state == CONTROL_RUNNING && snapshot().fault == CONTROL_FAULT_NONE);
    for (i = 0; i < 49u; ++i) control_tick(0, 0);
    assert(snapshot().state == CONTROL_RUNNING && snapshot().vision_age_ms == 500u);
    control_tick(0, 0);
    assert(snapshot().fault == CONTROL_FAULT_VISION_TIMEOUT);

    setup(); track_ready(); assert(control_request_run(CONTROL_MODE_TRACK));
    memset(&old, 0, sizeof(old)); old.frame_id = frame;
    old.capture_time_ms = control_now_ms(); old.confidence = 1.0f; old.valid = 1u;
    for (i = 0; i < 51u; ++i) { control_tick(0, 0); old.capture_time_ms = control_now_ms(); control_publish_vision(&old); }
    assert(snapshot().fault == CONTROL_FAULT_VISION_TIMEOUT);

    setup(); assert(control_request_run(CONTROL_MODE_BENCH));
    for (i = 0; i < 100u; ++i) control_tick(0, 0);
    s = snapshot(); assert(fabsf(s.target_left - 10.0f) < 0.0001f);
    assert(fabsf(s.target_right - 20.0f) < 0.0001f);
    control_request_stop(); s = snapshot();
    assert(s.pwm_left == 0.0f && s.target_left == 0.0f && speed_pid_l.integrator == 0.0f);
    assert(!control_request_run(CONTROL_MODE_BENCH));
    control_tick(0, 0); assert(control_request_run(CONTROL_MODE_BENCH));
    for (i = 0; i < 800u; ++i) control_tick(0, 0);
    assert(snapshot().state == CONTROL_READY && snapshot().pwm_left == 0.0f);

    setup(); assert(control_request_run(CONTROL_MODE_BENCH));
    control_report_link_fault();
    assert(snapshot().fault == CONTROL_FAULT_LINK && snapshot().pwm_left == 0.0f);
    control_tick(0, 0); assert(control_reset_fault());
    assert(snapshot().state == CONTROL_READY);

    setup(); control_get_params(&params, &version); params.kp_left = NAN;
    assert(!control_apply_params(&params, version));
    control_get_params(&params, &version); assert(!control_apply_params(&params, version - 1u));
    params.look_rows[0] = 120u; assert(!control_apply_params(&params, version));
    control_get_params(&params, &version); params.vision_timeout_ms = 5000u;
    assert(!control_apply_params(&params, version));

    setup(); track_ready(); assert(control_request_run(CONTROL_MODE_TRACK));
    publish(1.0f, 20.0f, 1u); control_tick(0, 0);
    assert(snapshot().servo_cmd > 92.5f && snapshot().servo_cmd <= 93.11f);
    for (i = 0; i < 40u; ++i) { publish(1.0f, 100.0f, 1u); control_tick(0, 0); }
    assert(snapshot().servo_cmd <= 100.0f);

    setup(); control_set_output_enabled(0u, 0u);
    assert(!control_request_run(CONTROL_MODE_BENCH));
    setup(); control_set_output_enabled(1u, 0u); track_ready();
    assert(!control_request_run(CONTROL_MODE_TRACK));
    assert(control_request_run(CONTROL_MODE_BENCH)); control_tick(0, 0);
    assert(!snapshot().servo_enabled);

    setup(); control_get_params(&params, &version);
    params.pwm_max = 10.5f; params.integral_max = 10.0f;
    assert(!control_apply_params(&params, version));
    params.pwm_max = 10.0f; params.kp_left = 0.0f; params.ki_left = 3.0f;
    params.target_left = 1.0f;
    assert(control_apply_params(&params, version)); assert(control_request_run(CONTROL_MODE_BENCH));
    for (i = 0; i < 400u; ++i) control_tick(0, 0);
    assert(snapshot().pwm_left == 10.0f && snapshot().saturated_left);

    setup(); track_ready(); control_get_params(&params, &version);
    params.look_weights[0] = 2.0f;
    assert(control_apply_params(&params, version));
    assert(!control_request_run(CONTROL_MODE_TRACK));
    track_ready(); assert(control_request_run(CONTROL_MODE_TRACK));

    // 当前400目标可提交；任一轮超限拒绝整组，不裁剪或改变已生效的配置。
    setup(); control_get_params(&params, &version);
    params.target_left = params.target_right = CONTROL_TARGET_COUNTS_MAX;
    assert(control_apply_params(&params, version));
    control_get_params(&params, &version);
    params.target_left = CONTROL_TARGET_COUNTS_MAX + 0.01f;
    assert(!control_apply_params(&params, version));
    params.target_left = CONTROL_TARGET_COUNTS_MAX; params.target_right = CONTROL_TARGET_COUNTS_MAX + 0.01f;
    assert(!control_apply_params(&params, version));
    {
        ControlParams unchanged;
        uint32_t unchanged_version;
        control_get_params(&unchanged, &unchanged_version);
        assert(unchanged_version == version);
        assert(unchanged.target_left == CONTROL_TARGET_COUNTS_MAX && unchanged.target_right == CONTROL_TARGET_COUNTS_MAX);
    }
    // 上限针对目标：实测425/426计数照常参与PI，不能把超速反馈截成400。
    assert(control_request_run(CONTROL_MODE_BENCH));
    control_tick(425, 426);
    s = snapshot();
    assert(s.measured_left == 425.0f && s.measured_right == 426.0f);
    assert(s.pwm_left == 0.0f && s.pwm_right == 0.0f);

    test_preview_control();
    test_mapped_differential();
    test_extrapolated_limits();
    test_default_pd();
    test_local_track();
    puts("PASS test_control: local keys, 500ms vision, restart, version, slope, BENCH, stop, limits");
    return 0;
}
