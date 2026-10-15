#include "control.h"
#include "pid.h"
#include "calibration_config.h"
#include "calibration_profile.h"
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
static uint8_t latest_frame_usable; // 最近接受的新帧是否可用；启动要求为1，短时失效取消远带并保留近带转向、半速。
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
static float previous_error; // 上一可用于转向微分的近带误差，单位像素；前瞻混合只进P项，避免切换时制造D脉冲。
static float steering_target; // 近远融合P和近带D给出的限位目标角；PIT另对实际servo_cmd限速。
static float steering_feedback_target; // 不含远带P前馈的近带PD目标；远带失效/过期时回到该目标。
static ControlPreviewSnapshot preview; // 原控制ABI之外的小型前瞻状态；主循环与PIT写，读取用统一临界区。

// 后轮差速沿用参考归档的阿克曼几何：后轴中心半径R=L/tan(δ)，左右目标乘1∓b/(2R)。
// δ取右前轮实测表插值/有界外推角相对当前中位的增量，仍是单轮代理；几何及扩展机械范围待实测。
#define DIFFERENTIAL_GAIN        (10.0f)   // 无量纲差速增益，放大标定相对角推算的几何比例；沿用10.0，最终限幅±70%。
#define CONTROL_WHEELBASE_M      (0.2f)    // 估计轴距，单位m，沿用参考配置，尚非本车实测值。
#define CONTROL_REAR_TRACK_M     (0.153f)  // 估计后轮距，单位m，用于内外轮速度比例，待实测。
#define CONTROL_DIFFERENTIAL_MAX (0.7f)    // 无量纲有符号差速比例限幅±0.7，使分配倍率处于0.3至1.7。
#define CONTROL_PREVIEW_TOP_ROW (42u) // 前瞻只使用地面ROI第42至119行，方向为图像向上看得更远。
#define CONTROL_PREVIEW_SHIFT_MIN (12.0f) // 远带最小前移12行，按基础目标速度增加，不表示实测前瞻距离。
#define CONTROL_PREVIEW_SHIFT_PER_MPS (18.0f) // 每增加1m/s名义基础目标，远带额外前移18行。
#define CONTROL_PREVIEW_SHIFT_MAX (36.0f) // 远带最多前移36行；越过ROI或缺少连续道路时退回近带。
#define CONTROL_PREVIEW_BLEND_MAX (0.75f) // 远带最多占融合误差75%，再由近远可信度共同衰减。
#define CONTROL_PREVIEW_STEP_PX (8.0f) // 连续中线每相邻图像行允许的最大位移8像素，跨缺行按行距同比放宽。
#define CONTROL_PREVIEW_AGE_MS (100u) // 远带与后轮前馈最长使用100逻辑ms，不延长原500ms视觉停车期限。
#define CONTROL_DIFFERENTIAL_LEAD_S (0.10f) // 后轮最多参考舵机变化率乘100ms的前馈命令，不越过转向目标。
#define CONTROL_DIFFERENTIAL_RATE_S (4.0f) // 无量纲差速比例变化率4每秒，标称10ms最多变化0.04。

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

// 控制路径统一使用原始右前轮角−33至+20deg的有界外推；严格实测区间API仍供标定使用。
static uint8_t control_steering_angle(float pulse_us, float *angle_deg)
{
    return steering_wheel_map_us_to_angle_extrapolate(&calibration_profile_right_steering_map, pulse_us,
        -CALIBRATION_STEERING_RIGHT_LIMIT_DEG, CALIBRATION_STEERING_LEFT_LIMIT_DEG, angle_deg);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     由右前轮实测脉宽映射生成有符号后轮差速比例
// 参数说明     command         已限定到当前目标内的实际或前馈舵机命令度
// 参数说明     differential    输出无量纲比例，成功时限定在±CONTROL_DIFFERENTIAL_MAX
// 返回参数     uint8_t         映射与计算均有效返回1；越界、非法表或角度返回0且不写输出
// 备注信息     以当前直行中位的映射角归零，保留原始+5度记录；不将单轮角当作完整双轮等效角
//-------------------------------------------------------------------------------------------------------------------
static uint8_t mapped_differential(float command, float *differential)
{
    float right_deg, center_deg, relative_deg, value;
    // 使用与舵机输出相同的整数PWM刻度再反算名义us；首次启动也能从命令重建，避免未使能时脉宽为0。
    float pulse_us = duty_to_nominal_us(legacy_servo_duty(command));
    float center_us = duty_to_nominal_us(legacy_servo_duty(config.servo_center));
    // 原始角仅允许−33至+20deg有界外推；不回退到命令角，失败仍由调用者锁故障并清输出。
    if (!control_steering_angle(pulse_us, &right_deg) ||
        !control_steering_angle(center_us, &center_deg))
        return 0u;
    relative_deg = right_deg - center_deg; // 默认中位1526us对应+5度，减去偏置后直行差速为0；不修改标定表。
    // 单轮相对角是差速转向代理；拒绝接近tan奇点的角度，防止未来改表后出现符号翻转或无穷。
    if (!isfinite(relative_deg) || fabsf(relative_deg) >= 89.0f) return 0u;
    value = DIFFERENTIAL_GAIN * CONTROL_REAR_TRACK_M * tanf(relative_deg * 3.14159265f / 180.0f) /
            (2.0f * CONTROL_WHEELBASE_M); // 相对角由度转弧度，轮距/(2轴距)无量纲，增益沿用10.0。
    if (!isfinite(value)) return 0u;
    *differential = clamp_value(value, -CONTROL_DIFFERENTIAL_MAX, CONTROL_DIFFERENTIAL_MAX);
    return 1u;
}

// 函数简介     检查当前视觉行可否作为前瞻的连续道路证据
// 参数说明     vision/row      稳定当前帧结果及0起图像行号
// 返回参数     uint8_t         质量、边界和中点均有效为1，否则0
// 备注信息     双边必须是真实图内边；单边恢复允许边框，但中点仍须在当前白段内
static uint8_t preview_row_valid(const VisionResult *vision, unsigned row)
{
    uint8_t quality;
    if (row < CONTROL_PREVIEW_TOP_ROW || row >= CONTROL_IMAGE_HEIGHT) return 0u;
    quality = vision->row_quality[row];
    if ((quality != 1u && quality != 2u) || vision->left[row] >= vision->right[row] ||
        vision->right[row] >= CONTROL_IMAGE_WIDTH || vision->middle[row] < vision->left[row] ||
        vision->middle[row] > vision->right[row]) return 0u;
    if (quality == 2u && (vision->left[row] == 0u || vision->right[row] == CONTROL_IMAGE_WIDTH - 1u)) return 0u;
    return 1u;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     在锁外从当前近远连续道路生成前瞻像素误差
// 参数说明     vision/params   稳定当前帧和短锁复制的参数；result接收本帧诊断部分
// 返回参数     void            证据不足时保持近带误差、blend=0及valid=0
// 备注信息     不外推缺失道路，不刷新采集时间；配置近带与实际结果不一致时禁用远带
//-------------------------------------------------------------------------------------------------------------------
static void preview_from_vision(const VisionResult *vision, const ControlParams *params, ControlPreviewSnapshot *result)
{
    unsigned i, rows = 0u, first, last, previous_row = 0u, gaps = 0u;
    unsigned far_rows[CONTROL_BAND_COUNT];
    int shift;
    float total = 0.0f, available = 0.0f, weighted = 0.0f, near_weighted = 0.0f, near_available = 0.0f;
    float nominal_speed, previous_middle = 0.0f;
    uint8_t have_previous = 0u;
    memset(result, 0, sizeof(*result));
    result->frame_id = vision->frame_id; result->capture_time_ms = vision->capture_time_ms;
    result->near_error_px = result->far_error_px = result->effective_error_px = vision->error_px;
    if (!vision->valid || vision->track_lost || !in_range(vision->confidence, 0.3f, 1.0f) || !isfinite(vision->error_px) ||
        calibration_profile_counts_per_rev_left <= 0.0f || calibration_profile_counts_per_rev_right <= 0.0f) return;
    // 分轮计数目标换算名义m/s，只决定取更远的图像行；不是当前实测车体速度或标定地面距离。
    nominal_speed = 0.5f * (params->target_left * calibration_profile_circumference_left_m / calibration_profile_counts_per_rev_left +
                           params->target_right * calibration_profile_circumference_right_m / calibration_profile_counts_per_rev_right) /
                    ((float)CONTROL_PERIOD_MS / 1000.0f);
    if (!isfinite(nominal_speed)) return;
    shift = (int)floorf(clamp_value(CONTROL_PREVIEW_SHIFT_MIN + CONTROL_PREVIEW_SHIFT_PER_MPS * nominal_speed,
                                   CONTROL_PREVIEW_SHIFT_MIN, CONTROL_PREVIEW_SHIFT_MAX) + 0.5f);
    result->row_shift = (uint8_t)shift;
    // 1. 只有配置近带仍支持本帧误差时才前移；自适应带无法确认时继续使用原近带结果。
    for (i = 0u; i < CONTROL_BAND_COUNT; ++i)
    {
        unsigned row = params->look_rows[i];
        float weight = params->look_weights[i];
        if (row >= CONTROL_IMAGE_HEIGHT || (int)row - shift < (int)CONTROL_PREVIEW_TOP_ROW ||
            !in_range(weight, 0.001f, 100.0f) || (i > 0u && row <= params->look_rows[i - 1u])) return;
        far_rows[i] = row - (unsigned)shift; total += weight;
        if (preview_row_valid(vision, row))
        {
            float q = vision->row_quality[row] == 2u ? 1.0f : 0.5f;
            near_available += weight * q; near_weighted += weight * q * vision->middle[row]; ++rows;
        }
    }
    if (rows < 3u || near_available / total < 0.35f ||
        fabsf((94.0f - near_weighted / near_available) - vision->error_px) > 4.0f) return;
    // 2. 从远带到近带逐行检查连接，最多允许连续缺2行；不能跨背景、分叉或大跳变借用远处白块。
    first = far_rows[0]; last = params->look_rows[CONTROL_BAND_COUNT - 1u];
    for (i = first; i <= last; ++i)
    {
        if (!preview_row_valid(vision, i)) { if (++gaps > 2u) return; continue; }
        if (have_previous && fabsf((float)vision->middle[i] - previous_middle) > CONTROL_PREVIEW_STEP_PX * (float)(i - previous_row)) return;
        previous_middle = (float)vision->middle[i]; previous_row = i; have_previous = 1u; gaps = 0u;
    }
    // 3. 远带仍需至少3行、质量权重至少35%；重建行半权，混合不改变近带原始结果和有效期限。
    rows = 0u;
    for (i = 0u; i < CONTROL_BAND_COUNT; ++i)
    {
        unsigned row = far_rows[i];
        if (preview_row_valid(vision, row))
        {
            float q = vision->row_quality[row] == 2u ? 1.0f : 0.5f;
            float weight = params->look_weights[i] * q;
            available += weight; weighted += weight * vision->middle[row]; ++rows;
        }
    }
    if (rows < 3u || available / total < 0.35f) return;
    result->far_error_px = 94.0f - weighted / available;
    result->blend = CONTROL_PREVIEW_BLEND_MAX * fminf(vision->confidence, available / total);
    result->effective_error_px = vision->error_px + result->blend * (result->far_error_px - vision->error_px);
    result->valid = 1u;
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
    // 1. 空指针先短路；双轮基础目标允许前进0～统一上限，当前为400 counts/10ms。
    if (p == NULL || !in_range(p->target_left, 0.0f, CONTROL_TARGET_COUNTS_MAX) ||
        !in_range(p->target_right, 0.0f, CONTROL_TARGET_COUNTS_MAX) ||
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
        // 6. 视觉统一使用500逻辑ms；旧丢线帧数字段固定3保留ABI，不再参与停车判断；诊断确认仍只接受0/1。
        p->vision_timeout_ms != 500u || p->lost_frame_limit != 3u ||
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
    steering_target = steering_feedback_target = config.servo_center;
    preview.valid = preview.lead_active = 0u;
    preview.blend = 0.0f; preview.effective_error_px = preview.near_error_px;
    current.vision_error_px = preview.near_error_px; // STOP/故障同步撤销远带误差，无新帧重启只能使用近带。
    preview.base_target_left = preview.base_target_right = 0.0f;
    preview.differential_requested = preview.differential_applied = 0.0f;
    preview.differential_command = config.servo_center;
    derivative_ready = 0u;
    speed_target = speed_pwm = 0.0f; // 同步旧例程的右轮兼容字段，不能把它们当独立控制入口。
}

static void latch_fault(ControlFault fault)
{
    // 故障锁存：新图或重连不能自行恢复；先使输出清零，再记录需要跨越的采样屏障。
    current.state = CONTROL_FAULT;
    ++stop_generation;
    current.fault = fault;
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
// 备注信息     默认目标仅供本地按键启动，上电仍不输出；诊断BENCH仍须人工确认
//-------------------------------------------------------------------------------------------------------------------
void control_init(float servo_center, float servo_min, float servo_max, float servo_direction)
{
    static const uint8_t rows[CONTROL_BAND_COUNT] = {80u, 85u, 90u, 95u, 100u};
    static const float weights[CONTROL_BAND_COUNT] = {1.0f, 2.0f, 4.0f, 2.0f, 1.0f};
    uint8_t i;
    // 1. 清零人工确认和运行状态，再设置本地按键目标及默认候选增益；目标非零不等于已启动。
    memset(&config, 0, sizeof(config));
    memset(&current, 0, sizeof(current));
    memset(&preview, 0, sizeof(preview));
    // 目标计数=默认速度m/s×本轮counts/rev×周期秒/本轮周长m；左右N不同，不能给同一个计数目标。
    // 当前N2333.3/2284.2及名义周长0.20106193m得到1.2m/s下约139.2586/136.3281 counts/10ms；上电仍零输出。
    config.target_left = CONTROL_DEFAULT_SPEED_MPS * calibration_profile_counts_per_rev_left *
                         ((float)CONTROL_PERIOD_MS / 1000.0f) / calibration_profile_circumference_left_m;
    config.target_right = CONTROL_DEFAULT_SPEED_MPS * calibration_profile_counts_per_rev_right *
                          ((float)CONTROL_PERIOD_MS / 1000.0f) / calibration_profile_circumference_right_m;
    config.kp_left = config.kp_right = 17.5f; // 用户授权应用实测计数换算后的合成候选1，单位PWM/(counts/10ms)，待实机验证。
    config.ki_left = config.ki_right = 480.0f; // 新候选1的秒制Ki，单位PWM/[(counts/10ms)·s]；10ms增量为4.8乘误差，上电积分为0。
    config.pwm_max = config.integral_max = 5000.0f; // PWM计数，不是百分比。
    config.accel_counts_per_s = 50.0f; // TRACK基础目标/BENCH有效目标每10ms最多变化0.5；TRACK差速比例另按4每秒渐变。
    // 2. 舵机使用独立中位和配置限位，实际方向/机械范围仍待人工确认。
    config.servo_center = servo_center;
    config.servo_min = servo_min;
    config.servo_max = servo_max;
    config.servo_direction = servo_direction;
    config.steer_kp = 0.22f; // 同步闭环仿真候选：命令度/像素，实际车上效果待首弯验证。
    config.steer_kd = 0.002f; // 命令度/(像素/秒)；按采集帧间隔求微分，不按10ms重复差分。
    config.servo_rate_deg_s = 60.0f; // 命令度/s，即标称10ms最多0.6度。
    config.curve_speed_factor = 0.5f; // |像素误差|达到30时基础目标乘0.5。
    config.vision_timeout_ms = 500u; // 从最后有效图像采集时刻起，超过500逻辑ms统一停车。
    config.lost_frame_limit = 3u; // 兼容旧ControlParams字段布局，控制逻辑不再读取此值。
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
    latest_frame_usable = frame_seen = good_seen = 0u;
    motor_output_enabled = servo_output_enabled = 0u;
    current.params_version = 1u; // 初始基线版本，成功整组应用才递增。
    current.mode = CONTROL_MODE_TRACK;
    current.vision_age_ms = UINT32_MAX;
    // 4. 两轮独立历史；默认速度D=0。构造结束直接进入READY或初始配置FAULT。
    Pid_Param_Init(&speed_pid_l, config.kp_left, config.ki_left, 0.0f, config.integral_max, config.pwm_max);
    Pid_Param_Init(&speed_pid_r, config.kp_right, config.ki_right, 0.0f, config.integral_max, config.pwm_max);
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
    // 3. 控制带变化使旧误差不再对应新配置，要求重新取得一帧可用图像。
    if (memcmp(config.look_rows, params->look_rows, sizeof(config.look_rows)) != 0 ||
        memcmp(config.look_weights, params->look_weights, sizeof(config.look_weights)) != 0)
    {
        latest_frame_usable = good_seen = derivative_ready = 0u;
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

static uint8_t request_run(ControlMode mode, uint8_t calibration_owner, uint8_t local_track)
{
    uint32_t key = lock_control();
    // 1. 本地TRACK按键可在视觉故障后直接重试；其余故障和平台初始化失败仍不能旁路。
    uint8_t recoverable_vision_fault = (uint8_t)(local_track && current.state == CONTROL_FAULT &&
        (current.fault == CONTROL_FAULT_VISION_TIMEOUT || current.fault == CONTROL_FAULT_TRACK_LOST));
    if ((current.state != CONTROL_READY && !recoverable_vision_fault) || !initialization_valid ||
        calibration_reserved != calibration_owner || !motor_output_enabled ||
        (!local_track && !config.calibration_confirmed) ||
        // 停止所在sample尚未前进，不能同一tick内停后又启，需让后续PIT有机会写零。
        current.sample_id == stop_sample ||
        // 只接受循迹/架空两种明确模式；至少一轮基础目标大于0才算启动请求。
        !(mode == CONTROL_MODE_TRACK || mode == CONTROL_MODE_BENCH) ||
        (config.target_left <= 0.0f && config.target_right <= 0.0f) ||
        // TRACK仍需舵机许可、最新帧有效且采集年龄不超过500逻辑ms；BENCH不看图。
        (mode == CONTROL_MODE_TRACK && (!servo_output_enabled || !latest_frame_usable || !good_seen ||
         (uint32_t)(device_ms - last_good_ms) > config.vision_timeout_ms)))
    {
        unlock_control(key);
        return 0u;
    }
    // 2. 只切换软件状态；电机斜坡与舵机命令限速由随后PIT执行，首个转向新帧不求历史D。
    current.mode = mode;
    current.fault = CONTROL_FAULT_NONE; // 所有条件通过后才清可恢复视觉故障；失败保持原故障与停止状态。
    current.state = CONTROL_RUNNING;
    run_start_ms = device_ms;
    derivative_ready = 0u;
    if (mode == CONTROL_MODE_TRACK)
    {
        // 即使仍满足500ms启动资格，也只允许100ms以内且未被STOP撤销的前瞻进入首拍P。
        if (!preview.valid || (uint32_t)(device_ms - preview.capture_time_ms) > CONTROL_PREVIEW_AGE_MS)
        {
            preview.valid = 0u; preview.blend = 0.0f; preview.effective_error_px = preview.near_error_px;
        }
        current.vision_error_px = preview.effective_error_px;
        steering_feedback_target = clamp_value(config.servo_center - config.servo_direction * config.steer_kp * preview.near_error_px,
                                               config.servo_min, config.servo_max);
        steering_target = clamp_value(config.servo_center - config.servo_direction * config.steer_kp * current.vision_error_px,
                                      config.servo_min, config.servo_max); // 首个PIT即可按最新有效观测转向，首帧D仍清零。
    }
    unlock_control(key);
    return 1u;
}

uint8_t control_request_run(ControlMode mode)
{
    return request_run(mode, 0u, 0u); // 兼容人工接口保留确认；Wi-Fi只调参/STOP，不调用该启动接口。
}

// 函数简介     本地KEY1提交一次循迹启动，视觉故障可在本次操作中恢复
// 参数说明     void
// 返回参数     uint8_t         接受为1，图像/输出/停机屏障/独占条件未满足为0
// 备注信息     不依赖上位机确认；故障消失或新图到达本身不会自动调用本接口
uint8_t control_request_local_track_run(void)
{
    return request_run(CONTROL_MODE_TRACK, 0u, 1u);
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
    return request_run(CONTROL_MODE_BENCH, 1u, 0u);
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
    stop_sample = current.sample_id; // 保留基础目标和最近帧事实；再次发车仍须人工请求及最新图像有效。
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
    // 保留最新帧可用性和采集时间事实；清故障不能把最近坏帧或过期图像变成启动资格。
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
    uint32_t key, frame_dt, version, generation;
    uint8_t usable;
    ControlParams params;
    ControlPreviewSnapshot candidate;
    if (vision == NULL) return;
    // 1. 短锁复制配置及停止代次；近远逐行扫描在锁外执行，避免长时间屏蔽PIT。
    key = lock_control();
    if (frame_seen && (int32_t)(vision->frame_id - current.frame_id) <= 0) { unlock_control(key); return; }
    params = config; version = current.params_version; generation = stop_generation;
    unlock_control(key);
    preview_from_vision(vision, &params, &candidate);
    key = lock_control();
    // 计算期间切参、STOP或故障后丢弃在途结果；重新检查帧号，不能把旧前瞻重新提交。
    if (version != current.params_version || generation != stop_generation ||
        (frame_seen && (int32_t)(vision->frame_id - current.frame_id) <= 0)) { unlock_control(key); return; }
    frame_dt = vision->capture_time_ms - last_frame_ms;
    last_frame_ms = vision->capture_time_ms; frame_seen = 1u;
    current.frame_id = vision->frame_id; current.threshold = vision->threshold;
    // 2. 年龄以重新进入短锁后的device_ms计算；前瞻不创造新帧或延长最后有效采集时间。
    usable = (uint8_t)(vision->valid && !vision->track_lost && in_range(vision->confidence, 0.3f, 1.0f) &&
        in_range(vision->error_px, -(float)CONTROL_IMAGE_WIDTH, (float)CONTROL_IMAGE_WIDTH) &&
        (uint32_t)(device_ms - vision->capture_time_ms) <= config.vision_timeout_ms);
    latest_frame_usable = usable;
    current.confidence = usable ? vision->confidence : 0.0f;
    if (usable)
    {
        float derivative = 0.0f, feedback, command;
        if ((uint32_t)(device_ms - vision->capture_time_ms) > CONTROL_PREVIEW_AGE_MS)
        { candidate.valid = 0u; candidate.blend = 0.0f; candidate.effective_error_px = vision->error_px; }
        // 只提交视觉诊断部分，保留PIT正在推进的基础/差速斜坡状态。
        preview.frame_id = candidate.frame_id; preview.capture_time_ms = candidate.capture_time_ms;
        preview.near_error_px = candidate.near_error_px; preview.far_error_px = candidate.far_error_px;
        preview.effective_error_px = candidate.effective_error_px; preview.blend = candidate.blend;
        preview.row_shift = candidate.row_shift; preview.valid = candidate.valid; preview.lead_active = 0u;
        current.vision_error_px = candidate.effective_error_px;
        last_good_ms = vision->capture_time_ms; good_seen = 1u;
        if (current.state == CONTROL_RUNNING && current.mode == CONTROL_MODE_TRACK)
        {
            // D始终对未融合的近带误差求导，远带启停/置信度变化仅作用P项，避免人为微分冲击。
            if (derivative_ready && frame_dt > 0u && frame_dt <= config.vision_timeout_ms)
                derivative = (vision->error_px - previous_error) * 1000.0f / (float)frame_dt;
            feedback = config.servo_center - config.servo_direction *
                       (config.steer_kp * vision->error_px + config.steer_kd * derivative);
            command = feedback - config.servo_direction * config.steer_kp * (candidate.effective_error_px - vision->error_px);
            if (!isfinite(feedback) || !isfinite(command)) latch_fault(CONTROL_FAULT_INVALID_VALUE);
            else
            {
                steering_feedback_target = clamp_value(feedback, config.servo_min, config.servo_max);
                steering_target = clamp_value(command, config.servo_min, config.servo_max);
                previous_error = vision->error_px; derivative_ready = 1u;
            }
        }
    }
    else
    {
        derivative_ready = 0u;
        preview.valid = preview.lead_active = 0u; preview.blend = 0.0f;
        preview.effective_error_px = preview.near_error_px;
        current.vision_error_px = preview.near_error_px;
        steering_target = steering_feedback_target; // 失效帧取消远带前馈，但保留最后近带转向和原500ms保护。
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
    float differential = 0.0f; // 有符号差速比例；仅TRACK计算，BENCH保持0，停止/故障/舵机标定提前返回。
    // 1. 标称周期由ms转秒，供Ki积分/速度斜坡/舵机变化率共用；不代表已测得实际间隔。
    float dt_s = (float)CONTROL_PERIOD_MS / 1000.0f;
    device_ms += CONTROL_PERIOD_MS; // 每被服务一次加10；漏中断不会自动补墙钟时间。
    ++current.sample_id; // 采样编号推进，使main中的停止/清故障屏障能判断后续服务已发生。
    current.timestamp_ms = device_ms;
    current.measured_left = measured_left;
    current.measured_right = measured_right;
    current.vision_age_ms = good_seen ? device_ms - last_good_ms : UINT32_MAX;
    preview.lead_active = 0u;
    if (preview.valid && (!latest_frame_usable || (uint32_t)(device_ms - preview.capture_time_ms) > CONTROL_PREVIEW_AGE_MS))
    {
        preview.valid = 0u; preview.blend = 0.0f; preview.effective_error_px = preview.near_error_px;
        current.vision_error_px = preview.near_error_px;
        steering_target = steering_feedback_target; // 100ms后取消远带与预测，但不延长原最后有效图像的500ms期限。
    }
    // 2. TRACK只按最后有效图像年龄停车；age=500仍运行，超过500才故障，不再计连续无效帧。
    if (current.state == CONTROL_RUNNING && current.mode == CONTROL_MODE_TRACK)
    {
        if (!good_seen || current.vision_age_ms > config.vision_timeout_ms)
            latch_fault(CONTROL_FAULT_VISION_TIMEOUT);
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
        preview.base_target_left = preview.base_target_right = 0.0f;
        preview.differential_requested = preview.differential_applied = 0.0f;
        preview.differential_command = config.servo_center;
        publish_servo_pulse();
        speed_target = speed_pwm = 0.0f;
        speed_real = (float)measured_right; // 停车仍保留右轮实测读数，零PWM不等于轮子已静止。
        return;
    }
    // 4. 本周期先推进舵机命令，再计算后轮目标；仍按原60命令度/s限速，硬件在ISR返回后统一写入。
    current.servo_enabled = servo_output_enabled;
    current.servo_cmd = approach(current.servo_cmd,
        current.mode == CONTROL_MODE_TRACK ? steering_target : config.servo_center,
        config.servo_rate_deg_s * dt_s);
    if (current.mode == CONTROL_MODE_TRACK)
    {
        float right_deg, lead_command = current.servo_cmd;
        float bend = clamp_value(fabsf(current.vision_error_px) / 30.0f, 0.0f, 1.0f);
        factor = 1.0f - bend * (1.0f - config.curve_speed_factor);
        if (current.confidence < 0.6f || !latest_frame_usable) factor = fminf(factor, 0.5f);
        // 实际即将输出的量化脉宽也须满足−33至+20deg外推角域；预测不能替代输出前检查。
        if (!control_steering_angle(
                duty_to_nominal_us(legacy_servo_duty(current.servo_cmd)), &right_deg))
        { latch_fault(CONTROL_FAULT_INVALID_VALUE); speed_real = (float)measured_right; return; }
        if (preview.valid && latest_frame_usable && (uint32_t)(device_ms - preview.capture_time_ms) <= CONTROL_PREVIEW_AGE_MS)
        {
            lead_command = approach(current.servo_cmd, steering_target, config.servo_rate_deg_s * CONTROL_DIFFERENTIAL_LEAD_S);
            // S弯反向时预测最多先到中位，不能在前轮尚未回中时提前施加反方向的后轮差速。
            if ((lead_command - config.servo_center) * (current.servo_cmd - config.servo_center) < 0.0f)
                lead_command = config.servo_center;
            preview.lead_active = 1u;
        }
        if (!mapped_differential(lead_command, &differential))
        { latch_fault(CONTROL_FAULT_INVALID_VALUE); speed_real = (float)measured_right; return; }
        preview.differential_command = lead_command;
        preview.differential_requested = differential;
        // 5. 基础速度保留原50计数单位/s斜坡；差速比例单独以4/s变化，避免把弯道轮速差拖成约1秒过程。
        // 每轮最终目标的变化率包含差速贡献，不再声称仍受50计数单位/s限制；PI/PWM上限保持。
        preview.base_target_left = approach(preview.base_target_left, config.target_left * factor, config.accel_counts_per_s * dt_s);
        preview.base_target_right = approach(preview.base_target_right, config.target_right * factor, config.accel_counts_per_s * dt_s);
        preview.differential_applied = approach(preview.differential_applied, differential, CONTROL_DIFFERENTIAL_RATE_S * dt_s);
        current.target_left = preview.base_target_left * (1.0f - preview.differential_applied);
        current.target_right = preview.base_target_right * (1.0f + preview.differential_applied);
    }
    else
    {
        // BENCH保持原独立轮目标斜坡，不查前瞻/转角表，也不继承上次TRACK的差速状态。
        preview.base_target_left = preview.base_target_right = 0.0f;
        preview.differential_requested = preview.differential_applied = 0.0f;
        preview.differential_command = config.servo_center;
        current.target_left = approach(current.target_left, config.target_left, config.accel_counts_per_s * dt_s);
        current.target_right = approach(current.target_right, config.target_right, config.accel_counts_per_s * dt_s);
    }
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
    // 7. 舵机已在本周期开始限速并检查表域；这里发布与后轮同一次ISR写出的整数PWM命令。
    publish_servo_pulse();
    // 8. 更新旧例程兼容的右轮遥测；真正左右控制状态仍由统一快照及独立PI保存。
    speed_target = current.target_right;
    speed_real = (float)measured_right;
    speed_pwm = current.pwm_right;
}

void control_get_preview_snapshot(ControlPreviewSnapshot *snapshot)
{
    uint32_t key = lock_control();
    if (snapshot != NULL) *snapshot = preview; // 同锁复制近远输入和本次基础/差速状态，诊断不能写回控制。
    unlock_control(key);
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
