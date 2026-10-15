#include "control.h"
#include "pid.h"
#include "calibration_config.h"
#include <math.h>
#include <stddef.h>
#include <string.h>

static ControlParams config; // 停止态整组应用的基础参数；RUNNING只读，不把降速/斜坡结果反写到这里。
static ControlSnapshot current; // 主循环与PIT共享的实时状态/命令；主循环读取时须通过短临界区复制。
static uint32_t (*critical_enter)(void); // 由硬件层提供的关中断入口，返回进入前的中断状态供恢复。
static void (*critical_leave)(uint32_t); // 与入口成对恢复原中断状态，不能直接无条件开中断。
static volatile uint32_t device_ms; // 每次control_tick累加10的逻辑时钟；volatile保证跨中断读取，不补计漏服务的时间。
static uint32_t run_start_ms; // 本次人工获准启动的逻辑时刻，BENCH用它检查8秒时限。
static uint32_t last_good_ms; // 最近可用视觉帧的采集逻辑时间，超时年龄以它为起点。
static uint32_t last_frame_ms; // 最近接受新帧的采集逻辑时间，供相邻帧PD时间差计算，失效帧也会更新。
static uint32_t stop_generation; // 每次显式STOP或故障递增，供被动手转模式观察V1停止，避免READY状态掩盖请求。
static uint32_t stop_sample; // 显式STOP/故障/输出许可撤销时的采样编号；BENCH自然到期不重新设置此屏障。
static uint8_t motor_output_enabled; // 编译开关传入的电机使用许可；许可不等于已经发车。
static uint8_t servo_output_enabled; // 编译开关传入的舵机脉冲许可；TRACK要求它有效，BENCH可无舵机。
static uint8_t good_frames; // 连续达到0.6可信度的新帧计数，饱和于3，用于人工TRACK启动资格。
static uint8_t lost_frames; // 连续不可用新帧计数，饱和于255；达到3由PIT锁故障。
static uint8_t frame_seen; // 是否已有接受过的帧，避免初始frame_id=0被误判为重复。
static uint8_t good_seen; // 是否已有可用帧，区分有效采集时间0与尚无有效图像。
static uint8_t derivative_ready; // 是否具有可连续求微分的上一帧误差；启动、失效、停机后重置。
static uint8_t initialization_valid; // 平台初始化失败永久禁止本次上电清故障恢复，避免坏时钟/profile被旁路。
static uint8_t calibration_reserved; // 标定准备/手转期间阻止普通启动，STOP和故障仍优先。
static uint32_t servo_pwm_max = 10000u; // PWM占空计数刻度，硬件初始化可显式配置。
static uint32_t servo_pwm_hz = 50u; // 舵机硬件频率Hz，控制周期不改变该值。
#if CALIBRATION_DEBUG_ENABLE
static float servo_cal_target_us; // 本次人工设置的单点目标us，不是限速后的输出。
static float servo_cal_min_us, servo_cal_max_us; // 本次确认的机械安全范围us。
static uint32_t servo_cal_min_duty, servo_cal_max_duty; // 人工范围内实际可输出的整数占空网格，避免截断越过机械下限。
static float servo_cal_rate_us_s; // 单点脉宽变化率us/s，不改变原转向命令度单位。
static float servo_electrical_min_us, servo_electrical_max_us; // 初始化基线允许的量化电气范围，禁止标定扩大。
#endif
static float previous_error; // 上一可用于转向微分的误差，单位像素，不是上一周期编码器误差。
static float steering_target; // 有效新帧P/PD给出的限位目标角；PIT另对实际servo_cmd执行变化率限制。

// 主循环改配置/状态或读取快照时保存中断状态，防止PIT看到只更新了一半的结构。
// 宿主单线程测试没有硬件钩子时返回0；真实板端在Init、启动PIT之前安装钩子。
static uint32_t lock_control(void)
{
    return critical_enter != NULL ? critical_enter() : 0u;
}

static void unlock_control(uint32_t key)
{
    if (critical_leave != NULL) critical_leave(key); // 恢复进入前的PRIMASK，不破坏外层临界区。
}

static float clamp_value(float value, float low, float high)
{
    return value < low ? low : (value > high ? high : value); // 闭区间限幅，边界值原样保留。
}

static float approach(float value, float target, float step)
{
    // 本次最多沿目标方向改变step；目标已很近时直接到达，避免斜坡越过目标。
    return value + clamp_value(target - value, -step, step);
}

static uint8_t in_range(float value, float low, float high)
{
    return (uint8_t)(isfinite(value) && value >= low && value <= high); // 先拒绝NaN/无穷，再检查闭区间。
}

// 原宏使用double中间量并向整数截断；保持TRACK/BENCH原有量化结果。
static uint32_t legacy_servo_duty(float command)
{
    return (uint32_t)((float)servo_pwm_max / (1000.0 / (float)servo_pwm_hz) *
                      (0.5 + (float)command / 90.0));
}

static float duty_to_nominal_us(uint32_t duty)
{
    return (float)((double)duty * 1000000.0 / ((double)servo_pwm_max * servo_pwm_hz));
}

static void publish_servo_pulse(void)
{
    if (!current.servo_enabled)
    {
        current.servo_pulse_us_requested = 0.0f;
        current.servo_duty_cmd = 0u;
        current.servo_pulse_us_nominal = 0.0f;
        return;
    }
#if CALIBRATION_DEBUG_ENABLE
    if (current.mode == CONTROL_MODE_SERVO_CAL)
    {
        uint32_t duty = (uint32_t)((double)current.servo_pulse_us_requested * servo_pwm_max * servo_pwm_hz / 1000000.0);
        // 原始请求可处于网格之间；实际整数脉宽必须仍落在人工确认范围内。
        current.servo_duty_cmd = duty < servo_cal_min_duty ? servo_cal_min_duty :
                                 (duty > servo_cal_max_duty ? servo_cal_max_duty : duty);
    }
    else
#endif
    {
        current.servo_pulse_us_requested = (float)(500.0 + (float)current.servo_cmd * 1000.0 / 90.0);
        current.servo_duty_cmd = legacy_servo_duty(current.servo_cmd);
    }
    current.servo_pulse_us_nominal = duty_to_nominal_us(current.servo_duty_cmd);
}

// 参数是停止态整组提交的候选。每个范围对应ControlParams单位和执行器限制，非法值拒绝而非静默裁剪。
static uint8_t params_valid(const ControlParams *p)
{
    float weight_sum = 0.0f;
    uint8_t i;
    // 1. 空指针先短路；双轮基础目标均只允许前进0～100 counts/10ms。
    if (p == NULL || !in_range(p->target_left, 0.0f, 100.0f) ||
        !in_range(p->target_right, 0.0f, 100.0f) ||
        // 2. 左右增益分别校验；Ki按秒，不兼容旧按循环次数累加的非零值。
        !in_range(p->kp_left, 0.0f, 1000.0f) || !in_range(p->kp_right, 0.0f, 1000.0f) ||
        !in_range(p->ki_left, 0.0f, 1000.0f) || !in_range(p->ki_right, 0.0f, 1000.0f) ||
        // 3. PWM命令最终写uint32，所以上限必须为整数，积分贡献的限幅不能超过它。
        !in_range(p->pwm_max, 0.0f, 5000.0f) || p->pwm_max != floorf(p->pwm_max) ||
        !in_range(p->integral_max, 0.0f, p->pwm_max) ||
        // 4. 斜坡单位(counts/10ms)/s；舵机上下限有序，中位在内，方向仅-1或+1。
        !in_range(p->accel_counts_per_s, 1.0f, 1000.0f) ||
        !in_range(p->servo_min, 0.0f, 180.0f) || !in_range(p->servo_max, 0.0f, 180.0f) ||
        !in_range(p->servo_center, p->servo_min, p->servo_max) || p->servo_min >= p->servo_max ||
        !(p->servo_direction == -1.0f || p->servo_direction == 1.0f) ||
        // 5. 转向P/D、命令度/s变化率和弯道缩放均有限；因子1表示不因弯道再降速。
        !in_range(p->steer_kp, 0.0f, 5.0f) || !in_range(p->steer_kd, 0.0f, 5.0f) ||
        !in_range(p->servo_rate_deg_s, 1.0f, 360.0f) ||
        !in_range(p->curve_speed_factor, 0.1f, 1.0f) ||
        // 6. 当前接口固定100逻辑ms/3新无效帧，不允许通过调参放宽保护；人工确认只接受0/1。
        p->vision_timeout_ms != 100u || p->lost_frame_limit != 3u ||
        p->calibration_confirmed > 1u) return 0u;
    // 7. 五行必须在图内且严格递增，权重逐项为正，避免重复行/零分母和越界取图。
    for (i = 0; i < CONTROL_BAND_COUNT; ++i)
    {
        if (p->look_rows[i] >= CONTROL_IMAGE_HEIGHT ||
            !in_range(p->look_weights[i], 0.001f, 100.0f) ||
            (i > 0u && p->look_rows[i] <= p->look_rows[i - 1u])) return 0u;
        weight_sum += p->look_weights[i]; // 参数层累计原权重；视觉层会归一化并乘行质量。
    }
    return (uint8_t)(weight_sum > 0.0f);
}

// 一并清计算命令和双轮内部历史，停车不等待斜坡；基础config目标与视觉计数不由此函数清除。
// 这里只改变软件状态，真实PWM由ISR随后写入，机械停止及重载延迟另行实测。
static void reset_outputs(void)
{
    Pid_Reset(&speed_pid_l);
    Pid_Reset(&speed_pid_r);
    current.target_left = current.target_right = 0.0f;
    current.pwm_left = current.pwm_right = 0.0f;
    current.raw_pwm_left = current.raw_pwm_right = 0.0f;
    current.integral_left = current.integral_right = 0.0f;
    current.saturated_left = current.saturated_right = 0u;
    current.servo_enabled = 0u;
    publish_servo_pulse(); // 停止同步清脉宽/占空字段，不能把中位软件值当成仍在输出。
    current.servo_cmd = config.servo_center; // 仅记录中位；servo_enabled=0时不发脉冲，不保证物理回中。
    steering_target = config.servo_center;
    derivative_ready = 0u;
    speed_target = speed_pwm = 0.0f; // 同步旧例程的右轮兼容字段，不能把它们当独立控制入口。
}

static void latch_fault(ControlFault fault)
{
    // 故障锁存：新图或重连不能自行恢复；先使输出清零，再记录需要跨越的采样屏障。
    current.state = CONTROL_FAULT;
    ++stop_generation;
    current.fault = fault;
    good_frames = 0u;
    reset_outputs();
    stop_sample = current.sample_id; // 若在PIT内触发，仍须下一tick才能清故障。
}

void control_set_critical_hooks(uint32_t (*enter)(void), void (*leave)(uint32_t))
{
    // 硬件Init在PIT启用之前安装，避免运行时更换成对钩子产生不一致的锁行为。
    critical_enter = enter;
    critical_leave = leave;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     初始化运行状态和参数，上电保持停止
// 参数说明     servo_center/min/max/direction    人工待标定的舵机初始配置
// 返回参数     void
// 备注信息     启动前仍须由操作者确认标定，默认不会输出电机或舵机脉冲
//-------------------------------------------------------------------------------------------------------------------
void control_init(float servo_center, float servo_min, float servo_max, float servo_direction)
{
    static const uint8_t rows[CONTROL_BAND_COUNT] = {80u, 85u, 90u, 95u, 100u};
    static const float weights[CONTROL_BAND_COUNT] = {1.0f, 2.0f, 4.0f, 2.0f, 1.0f};
    uint8_t i;
    // 1. 整体清零使基础目标、Ki、人工确认从0开始，不沿用上一次试验的运行状态。
    memset(&config, 0, sizeof(config));
    memset(&current, 0, sizeof(current));
    config.kp_left = config.kp_right = 50.0f; // 待实测的P起点；初始化Ki仍为0。
    config.pwm_max = config.integral_max = 5000.0f; // PWM计数，不是百分比。
    config.accel_counts_per_s = 50.0f; // 每10ms最多改变0.5个速度单位，停止不走此斜坡。
    // 2. 舵机使用独立中位和配置限位，实际方向/机械范围仍待人工确认。
    config.servo_center = servo_center;
    config.servo_min = servo_min;
    config.servo_max = servo_max;
    config.servo_direction = servo_direction;
    config.steer_kp = 0.5f; // 命令度/像素；Kd初始为0，仅P转向。
    config.servo_rate_deg_s = 60.0f; // 命令度/s，即标称10ms最多0.6度。
    config.curve_speed_factor = 0.5f; // |像素误差|达到30时基础目标乘0.5。
    config.vision_timeout_ms = 100u;
    config.lost_frame_limit = 3u;
    for (i = 0; i < CONTROL_BAND_COUNT; ++i)
    {
        config.look_rows[i] = rows[i];
        config.look_weights[i] = weights[i];
    }
    // 3. 时钟/帧资格/输出许可全复位，UINT32_MAX表示尚无有效帧而非年龄0。
    device_ms = run_start_ms = last_good_ms = last_frame_ms = 0u;
    stop_sample = stop_generation = 0u;
    calibration_reserved = 0u;
    initialization_valid = 1u;
#if CALIBRATION_DEBUG_ENABLE
    servo_electrical_min_us = duty_to_nominal_us(legacy_servo_duty(servo_min));
    servo_electrical_max_us = duty_to_nominal_us(legacy_servo_duty(servo_max));
#endif
    good_frames = lost_frames = frame_seen = good_seen = 0u;
    motor_output_enabled = servo_output_enabled = 0u;
    current.params_version = 1u; // 初始基线版本，成功整组应用才递增。
    current.mode = CONTROL_MODE_TRACK;
    current.vision_age_ms = UINT32_MAX;
    // 4. 两轮独立历史；默认速度D=0。构造结束直接进入READY或初始配置FAULT。
    Pid_Param_Init(&speed_pid_l, 50.0f, 0.0f, 0.0f, 5000.0f, 5000.0f);
    Pid_Param_Init(&speed_pid_r, 50.0f, 0.0f, 0.0f, 5000.0f, 5000.0f);
    current.state = params_valid(&config) ? CONTROL_READY : CONTROL_FAULT;
    current.fault = current.state == CONTROL_READY ? CONTROL_FAULT_NONE : CONTROL_FAULT_INVALID_VALUE;
    reset_outputs();
}

void control_set_output_enabled(uint8_t motor_enabled, uint8_t servo_enabled)
{
    uint32_t key = lock_control();
    // 只归一化使用许可；TRACK/BENCH需要电机许可，SERVO_CAL仅需要舵机许可，撤销所需许可则停止。
    motor_output_enabled = (uint8_t)(motor_enabled != 0u);
    servo_output_enabled = (uint8_t)(servo_enabled != 0u);
    if ((current.mode != CONTROL_MODE_SERVO_CAL && !motor_output_enabled) ||
        ((current.mode == CONTROL_MODE_TRACK || current.mode == CONTROL_MODE_SERVO_CAL) && !servo_output_enabled))
    {
        if (current.state == CONTROL_RUNNING) current.state = CONTROL_READY;
        reset_outputs();
        stop_sample = current.sample_id;
    }
    unlock_control(key);
}

void control_get_params(ControlParams *params, uint32_t *version)
{
    uint32_t key = lock_control();
    if (params != NULL) *params = config; // 参数和版本同锁复制，调用者可只取其中一项。
    if (version != NULL) *version = current.params_version; // 写回时须带此expected_version。
    unlock_control(key);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     停止态原子应用整组参数并发布新版本
// 参数说明     params          参数快照；expected_version 为读取时的版本
// 返回参数     uint8_t         成功为 1，运行中、过期或非法请求为 0
// 备注信息     应用参数后清积分；故障状态不会因此解除
//-------------------------------------------------------------------------------------------------------------------
static uint8_t apply_params(const ControlParams *params, uint32_t expected_version, uint8_t calibration_owner)
{
    uint32_t key;
    // 1. 在锁外校验稳定的候选，缩短屏蔽中断时间；非法候选本身不自动清故障。
    if (!params_valid(params)) return 0u;
    key = lock_control();
    // 2. 锁内重查状态与版本，防止读取后已有别的请求生效，或已开始运行。
    if (current.state == CONTROL_RUNNING || calibration_reserved != calibration_owner ||
        expected_version != current.params_version)
    {
        unlock_control(key);
        return 0u;
    }
    // 3. 控制带变化使旧误差/三帧资格不再对应新配置，要求重新获得可用高可信帧。
    if (memcmp(config.look_rows, params->look_rows, sizeof(config.look_rows)) != 0 ||
        memcmp(config.look_weights, params->look_weights, sizeof(config.look_weights)) != 0)
    {
        good_frames = good_seen = derivative_ready = 0u;
    }
    config = *params; // 一次提交整组，PIT不会观察到左右轮/保护参数只更新一半。
    ++current.params_version;
    current.params_applied_ms = device_ms; // 生效逻辑时刻，供与版本一起追溯；不是硬件PWM生效时刻。
    // 4. 用新增益重建两轮控制器并清历史；旧兼容字段只跟随左增益/右轮遥测。
    Pid_Param_Init(&speed_pid_l, config.kp_left, config.ki_left, 0.0f, config.integral_max, config.pwm_max);
    Pid_Param_Init(&speed_pid_r, config.kp_right, config.ki_right, 0.0f, config.integral_max, config.pwm_max);
    speed_KP = config.kp_left;
    speed_KI = config.ki_left;
    speed_KD = 0.0f;
    speed_IMAX = config.integral_max;
    speed_OUTMAX = config.pwm_max;
    reset_outputs();
    unlock_control(key);
    return 1u;
}

uint8_t control_apply_params(const ControlParams *params, uint32_t expected_version)
{
    return apply_params(params, expected_version, 0u); // 独占标定期间拒绝普通参数写入，避免结束还原覆盖他人修改。
}

uint8_t control_apply_calibration_params(const ControlParams *params, uint32_t expected_version)
{
#if CALIBRATION_DEBUG_ENABLE
    return apply_params(params, expected_version, 1u);
#else
    (void)params; (void)expected_version;
    return 0u;
#endif
}

static uint8_t request_run(ControlMode mode, uint8_t calibration_owner)
{
    uint32_t key = lock_control();
    // 1. 人工启动门槛逐项拒绝：READY、电机许可、人工确认缺一不可；FAULT不能直接发车。
    if (current.state != CONTROL_READY || calibration_reserved != calibration_owner ||
        !motor_output_enabled || !config.calibration_confirmed ||
        // 停止所在sample尚未前进，不能同一tick内停后又启，需让后续PIT有机会写零。
        current.sample_id == stop_sample ||
        // 只接受循迹/架空两种明确模式；至少一轮基础目标大于0才算启动请求。
        !(mode == CONTROL_MODE_TRACK || mode == CONTROL_MODE_BENCH) ||
        (config.target_left <= 0.0f && config.target_right <= 0.0f) ||
        // TRACK另需舵机许可、连续3可信新帧、确有好帧且采集年龄<=100逻辑ms；BENCH不看图。
        (mode == CONTROL_MODE_TRACK && (!servo_output_enabled || good_frames < 3u || !good_seen ||
         (uint32_t)(device_ms - last_good_ms) > config.vision_timeout_ms)))
    {
        unlock_control(key);
        return 0u;
    }
    // 2. 只切换软件状态；电机斜坡与舵机命令限速由随后PIT执行，首个转向新帧不求历史D。
    current.mode = mode;
    current.state = CONTROL_RUNNING;
    run_start_ms = device_ms;
    derivative_ready = 0u;
    unlock_control(key);
    return 1u;
}

uint8_t control_request_run(ControlMode mode)
{
    return request_run(mode, 0u); // 普通启动不得使用标定会话的临时参数或许可。
}

void control_configure_servo_pwm(uint32_t duty_max, uint32_t frequency_hz)
{
    // 只允许停车初始化，避免中途改变占空刻度造成脉宽跳变。
    uint32_t key = lock_control();
    if (current.state == CONTROL_RUNNING || duty_max == 0u || frequency_hz < 50u || frequency_hz > 300u)
        latch_fault(CONTROL_FAULT_INVALID_VALUE);
    else
    {
        servo_pwm_max = duty_max;
        servo_pwm_hz = frequency_hz;
#if CALIBRATION_DEBUG_ENABLE
        servo_electrical_min_us = duty_to_nominal_us(legacy_servo_duty(config.servo_min));
        servo_electrical_max_us = duty_to_nominal_us(legacy_servo_duty(config.servo_max));
#endif
    }
    unlock_control(key);
}

float control_servo_us_per_duty(void)
{
    uint32_t key = lock_control();
    float scale = duty_to_nominal_us(1u);
    unlock_control(key);
    return scale;
}

void control_set_calibration_reserved(uint8_t reserved)
{
    uint32_t key = lock_control();
    calibration_reserved = (uint8_t)(reserved != 0u);
    unlock_control(key);
}

uint32_t control_stop_generation(void)
{
    uint32_t key = lock_control();
    uint32_t generation = stop_generation;
    unlock_control(key);
    return generation;
}

uint8_t control_get_calibration_reserved(void)
{
    uint32_t key = lock_control();
    uint8_t reserved = calibration_reserved;
    unlock_control(key);
    return reserved;
}

uint8_t control_request_calibration_bench(void)
{
#if CALIBRATION_DEBUG_ENABLE
    return request_run(CONTROL_MODE_BENCH, 1u);
#else
    return 0u;
#endif
}

uint8_t control_request_servo_cal(float center_us, float min_us, float max_us,
                                 float rate_us_s, uint32_t expected_params_version)
{
#if CALIBRATION_DEBUG_ENABLE
    uint32_t key = lock_control();
    // 参数版本、停车屏障和编译许可必须一致，机械范围不能扩大初始化电气边界。
    if (current.state != CONTROL_READY || !calibration_reserved || !servo_output_enabled ||
        current.sample_id == stop_sample || expected_params_version != current.params_version ||
        !in_range(min_us, servo_electrical_min_us, servo_electrical_max_us) ||
        !in_range(max_us, min_us, servo_electrical_max_us) || min_us >= max_us ||
        !in_range(center_us, min_us, max_us) ||
        !in_range(rate_us_s, 1.0f, config.servo_rate_deg_s * 1000.0f / 90.0f))
    {
        unlock_control(key);
        return 0u;
    }
    {
        double scale = (double)servo_pwm_max * servo_pwm_hz / 1000000.0;
        uint32_t min_duty = (uint32_t)ceil((double)min_us * scale);
        uint32_t max_duty = (uint32_t)floor((double)max_us * scale);
        if (min_duty > max_duty) { unlock_control(key); return 0u; } // 范围内无可输出整数脉宽，拒绝准备。
        servo_cal_min_duty = min_duty;
        servo_cal_max_duty = max_duty;
    }
    reset_outputs();
    servo_cal_target_us = center_us;
    servo_cal_min_us = min_us;
    servo_cal_max_us = max_us;
    servo_cal_rate_us_s = rate_us_s;
    current.servo_pulse_us_requested = center_us;
    current.servo_cmd = (center_us - 500.0f) * 0.09f; // 仅维持旧命令刻度显示，不是轮角。
    current.mode = CONTROL_MODE_SERVO_CAL;
    current.state = CONTROL_RUNNING;
    current.servo_enabled = 1u;
    publish_servo_pulse();
    unlock_control(key);
    return 1u;
#else
    (void)center_us; (void)min_us; (void)max_us; (void)rate_us_s; (void)expected_params_version;
    return 0u;
#endif
}

uint8_t control_set_servo_cal_pulse_us(float pulse_us)
{
#if CALIBRATION_DEBUG_ENABLE
    uint32_t key = lock_control();
    uint8_t accepted = (uint8_t)(current.state == CONTROL_RUNNING &&
        current.mode == CONTROL_MODE_SERVO_CAL && in_range(pulse_us, servo_cal_min_us, servo_cal_max_us));
    if (accepted) servo_cal_target_us = pulse_us;
    unlock_control(key);
    return accepted;
#else
    (void)pulse_us;
    return 0u;
#endif
}

float control_servo_cal_requested_us(void)
{
#if CALIBRATION_DEBUG_ENABLE
    uint32_t key = lock_control();
    float pulse = current.state == CONTROL_RUNNING && current.mode == CONTROL_MODE_SERVO_CAL ? servo_cal_target_us : 0.0f;
    unlock_control(key);
    return pulse;
#else
    return 0.0f;
#endif
}

void control_request_stop(void)
{
    uint32_t key = lock_control();
    ++stop_generation; // 即使本来READY，标定模块仍能检测这次人工停止。
    if (current.state != CONTROL_FAULT) current.state = CONTROL_READY; // STOP不能顺带解除已锁存故障。
    reset_outputs();
    stop_sample = current.sample_id; // config基础目标与good_frames保留；再次发车仍须人工请求/新鲜度等条件。
    unlock_control(key);
}

uint8_t control_reset_fault(void)
{
    uint32_t key = lock_control();
    // 运行态不清故障；停止屏障没跨过或配置仍非法也拒绝。正常READY调用也仅保持停止。
    if (current.state == CONTROL_RUNNING || current.sample_id == stop_sample || !initialization_valid || !params_valid(&config))
    {
        unlock_control(key);
        return 0u;
    }
    current.state = CONTROL_READY;
    current.fault = CONTROL_FAULT_NONE;
    good_frames = lost_frames = 0u; // 清故障后TRACK须重新积累三帧；人工确认标志不会在这里自动撤销。
    reset_outputs();
    unlock_control(key);
    return 1u;
}

void control_report_link_fault(void)
{
    uint32_t key = lock_control();
    if (current.state == CONTROL_RUNNING) latch_fault(CONTROL_FAULT_LINK); // 停车连接失败不锁行车LINK故障，运行异常则锁存。
    unlock_control(key);
}

void control_report_initialization_fault(void)
{
    uint32_t key = lock_control();
    initialization_valid = 0u; // 只有重新初始化并修复编译配置后才允许恢复，不由普通CLEAR解除。
    latch_fault(CONTROL_FAULT_INVALID_VALUE);
    unlock_control(key);
}

void control_report_invalid_fault(void)
{
    uint32_t key = lock_control();
    // 接口/计算非法即使在READY也锁存，避免继续使用不一致的控制带或非有限命令。
    latch_fault(CONTROL_FAULT_INVALID_VALUE);
    unlock_control(key);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     接收新帧结果，以采集时间而不是处理时间计算新鲜度
// 参数说明     vision          只读视觉结果，重复帧不参与计数和微分
// 返回参数     void
// 备注信息     同一帧仅更新一次舵机目标；实际变化率限制在十毫秒控制任务中执行
//-------------------------------------------------------------------------------------------------------------------
void control_publish_vision(const VisionResult *vision)
{
    uint32_t key;
    uint32_t frame_dt;
    uint8_t usable;
    // 1. 只接受不同且向前的新帧；无符号差值转有符号做半范围比较，兼容常见uint32回绕。
    if (vision == NULL) return;
    key = lock_control();
    if (frame_seen && (int32_t)(vision->frame_id - current.frame_id) <= 0)
    {
        unlock_control(key);
        return;
    }
    // 2. 用采集而非处理时刻算PD间隔；当前采集时间本身按10ms逻辑时基量化。
    frame_dt = vision->capture_time_ms - last_frame_ms;
    last_frame_ms = vision->capture_time_ms;
    frame_seen = 1u;
    current.frame_id = vision->frame_id; // 失效新帧也更新它，不能据此反推上次PWM实际使用的视觉来源。
    current.threshold = vision->threshold;
    // 3. 同时校验模块有效/丢线、可信度、有限像素误差与采集年龄；未来时间戳的无符号差值也会被拒绝。
    usable = (uint8_t)(vision->valid && !vision->track_lost &&
        in_range(vision->confidence, 0.3f, 1.0f) && // 控制API底线0.3，正常视觉输出有效门槛另为0.35。
        in_range(vision->error_px, -(float)CONTROL_IMAGE_WIDTH, (float)CONTROL_IMAGE_WIDTH) &&
        (uint32_t)(device_ms - vision->capture_time_ms) <= config.vision_timeout_ms);
    current.confidence = usable ? vision->confidence : 0.0f; // 失效归0；旧像素误差可保留，必须结合年龄和可信度看。
    if (usable)
    {
        float derivative = 0.0f;
        float command;
        current.vision_error_px = vision->error_px;
        last_good_ms = vision->capture_time_ms; // 可用新帧才刷新保护起点，重复/失效帧不能延长寿命。
        good_seen = 1u;
        lost_frames = 0u;
        // 启动资格比运行可用门槛严格：>=0.6连续计数至3，低于0.6则断开连续性。
        good_frames = vision->confidence >= 0.6f ? (uint8_t)(good_frames < 3u ? good_frames + 1u : 3u) : 0u;
        if (current.state == CONTROL_RUNNING && current.mode == CONTROL_MODE_TRACK)
        {
            // 4. 只有RUNNING/TRACK计算新转向目标；首次、失效后或间隔为0/过长时D保持0。
            if (derivative_ready && frame_dt > 0u && frame_dt <= config.vision_timeout_ms)
                derivative = (vision->error_px - previous_error) * 1000.0f / (float)frame_dt; // 像素差/ms乘1000变为像素/s。
            // 误差正值表示中线偏左；direction决定舵机电气方向，P项为命令度，D项也是命令度。
            command = config.servo_center - config.servo_direction *
                      (config.steer_kp * vision->error_px + config.steer_kd * derivative);
            if (!isfinite(command)) latch_fault(CONTROL_FAULT_INVALID_VALUE);
            else steering_target = clamp_value(command, config.servo_min, config.servo_max); // 先限配置角，再由PIT限变化率。
            previous_error = vision->error_px;
            derivative_ready = 1u;
        }
    }
    else
    {
        good_frames = 0u;
        derivative_ready = 0u;
        if (lost_frames < UINT8_MAX) ++lost_frames; // 新失效结果才加1，饱和避免回绕；不是每tick加1。
    }
    unlock_control(key);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     固定十毫秒执行采样、保护、目标斜坡、双轮 PI 和舵机限速
// 参数说明     measured_left/right    本周期带方向的编码器增量
// 返回参数     void
// 备注信息     不进行网络和显示；调用方必须把快照中的实际输出写入硬件
//-------------------------------------------------------------------------------------------------------------------
void control_tick(int16_t measured_left, int16_t measured_right)
{
    float factor = 1.0f;
    // 1. 标称周期由ms转秒，供Ki积分/速度斜坡/舵机变化率共用；不代表已测得实际间隔。
    float dt_s = (float)CONTROL_PERIOD_MS / 1000.0f;
    device_ms += CONTROL_PERIOD_MS; // 每被服务一次加10；漏中断不会自动补墙钟时间。
    ++current.sample_id; // 采样编号推进，使main中的停止/清故障屏障能判断后续服务已发生。
    current.timestamp_ms = device_ms;
    current.measured_left = measured_left;
    current.measured_right = measured_right;
    current.vision_age_ms = good_seen ? device_ms - last_good_ms : UINT32_MAX;
    // 2. TRACK保护先于任何新PI输出；无好图/age>100优先于连续3新帧丢线，age=100仍允许。
    if (current.state == CONTROL_RUNNING && current.mode == CONTROL_MODE_TRACK)
    {
        if (!good_seen || current.vision_age_ms > config.vision_timeout_ms)
            latch_fault(CONTROL_FAULT_VISION_TIMEOUT);
        else if (lost_frames >= config.lost_frame_limit) latch_fault(CONTROL_FAULT_TRACK_LOST);
    }
    // BENCH只按独立8秒逻辑时限停止，不把图像缺失当故障，也不能用网络恢复重新启动。
    if (current.state == CONTROL_RUNNING && current.mode == CONTROL_MODE_BENCH &&
        (uint32_t)(device_ms - run_start_ms) >= CONTROL_BENCH_MAX_MS)
    {
        current.state = CONTROL_READY;
        reset_outputs();
    }
#if CALIBRATION_DEBUG_ENABLE
    // 舵机独占模式必须先于电机许可分支：即使电机编译关闭，也允许已确认的舵机单点。
    if (current.state == CONTROL_RUNNING && current.mode == CONTROL_MODE_SERVO_CAL)
    {
        if (!servo_output_enabled) { control_request_stop(); return; }
        Pid_Reset(&speed_pid_l); Pid_Reset(&speed_pid_r);
        current.target_left = current.target_right = current.pwm_left = current.pwm_right = 0.0f;
        current.raw_pwm_left = current.raw_pwm_right = current.integral_left = current.integral_right = 0.0f;
        current.saturated_left = current.saturated_right = 0u;
        speed_target = speed_pwm = 0.0f;
        speed_real = (float)measured_right;
        current.servo_pulse_us_requested = approach(current.servo_pulse_us_requested,
                                                   servo_cal_target_us, servo_cal_rate_us_s * dt_s);
        current.servo_cmd = (current.servo_pulse_us_requested - 500.0f) * 0.09f;
        current.servo_enabled = 1u;
        publish_servo_pulse();
        return;
    }
#endif
    // 3. READY/FAULT或无电机许可每tick写零命令，并清控制器历史，不走减速斜坡。
    if (current.state != CONTROL_RUNNING || !motor_output_enabled)
    {
        /* 停机每周期保持零输出，不让旧积分或速度斜坡推迟停止。 */
        Pid_Reset(&speed_pid_l);
        Pid_Reset(&speed_pid_r);
        current.target_left = current.target_right = 0.0f;
        current.pwm_left = current.pwm_right = 0.0f;
        current.raw_pwm_left = current.raw_pwm_right = 0.0f;
        current.integral_left = current.integral_right = 0.0f;
        current.saturated_left = current.saturated_right = 0u;
        current.servo_enabled = 0u;
        publish_servo_pulse();
        speed_target = speed_pwm = 0.0f;
        speed_real = (float)measured_right; // 停车仍保留右轮实测读数，零PWM不等于轮子已静止。
        return;
    }
    if (current.mode == CONTROL_MODE_TRACK)
    {
        // 4. |像素误差|/30归一化弯道程度0～1，因子从1线性下降至curve_speed_factor。
        float bend = clamp_value(fabsf(current.vision_error_px) / 30.0f, 0.0f, 1.0f);
        factor = 1.0f - bend * (1.0f - config.curve_speed_factor);
        if (current.confidence < 0.6f || lost_frames > 0u) factor = fminf(factor, 0.5f); // 低可信/短失线将降速目标至多半基准。
    }
    // 5. 基础目标先乘降速因子，再按变化率×秒数靠近；两轮独立，半速目标不等于实际轮速瞬间减半。
    current.target_left = approach(current.target_left, config.target_left * factor, config.accel_counts_per_s * dt_s);
    current.target_right = approach(current.target_right, config.target_right * factor, config.accel_counts_per_s * dt_s);
    // 6. 速度误差=本轮有效目标-本轮编码器增量；PI内部限幅后量化成硬件整数，再以float存快照。
    current.pwm_left = (float)(uint32_t)PidLocCtrl(&speed_pid_l, current.target_left - measured_left, dt_s);
    current.pwm_right = (float)(uint32_t)PidLocCtrl(&speed_pid_r, current.target_right - measured_right, dt_s);
    // 原始输出/积分用于诊断；实际PWM与ISR写入一致，不能把raw直接写硬件。
    current.raw_pwm_left = speed_pid_l.out_raw;
    current.raw_pwm_right = speed_pid_r.out_raw;
    current.integral_left = speed_pid_l.out_i;
    current.integral_right = speed_pid_r.out_i;
    // 下限只在raw<0时报告；非零上限恰好触顶也置位。pwm_max=0的禁出力配置不按上限触顶标记。
    current.saturated_left = (uint8_t)(current.raw_pwm_left < 0.0f ||
        (config.pwm_max > 0.0f && current.pwm_left >= config.pwm_max));
    current.saturated_right = (uint8_t)(current.raw_pwm_right < 0.0f ||
        (config.pwm_max > 0.0f && current.pwm_right >= config.pwm_max));
    // 7. TRACK向新帧目标靠近，BENCH向中位靠近；60命令度/s在10ms最多走0.6度。
    // 使能只表示允许ISR发脉冲；50Hz脉冲重载点与这次CPU写命令时刻分开。
    current.servo_enabled = servo_output_enabled;
    current.servo_cmd = approach(current.servo_cmd,
        current.mode == CONTROL_MODE_TRACK ? steering_target : config.servo_center,
        config.servo_rate_deg_s * dt_s);
    publish_servo_pulse(); // 快照的整数占空与ISR写入保持一致，脉宽仍是名义换算。
    // 8. 更新旧例程兼容的右轮遥测；真正左右控制状态仍由统一快照及独立PI保存。
    speed_target = current.target_right;
    speed_real = (float)measured_right;
    speed_pwm = current.pwm_right;
}

void control_get_snapshot(ControlSnapshot *snapshot)
{
    uint32_t key = lock_control();
    if (snapshot != NULL) *snapshot = current; // 同锁复制所有字段；视觉字段可能比最近tick输出更晚更新。
    unlock_control(key);
}

ControlState control_get_state(void)
{
    uint32_t key = lock_control();
    ControlState state = current.state; // 在锁内取值、锁外返回，避免把共享变量读多次。
    unlock_control(key);
    return state;
}

uint32_t control_now_ms(void)
{
    return device_ms; // Cortex-M上对齐32位单次读取；返回逻辑ms，不用于测ISR真实耗时。
}
