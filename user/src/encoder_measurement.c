#include "encoder_measurement.h"
#include <math.h>
#include <float.h>
#include <stddef.h>
#include <string.h>

typedef struct
{
    int16_t left; // 单个有效样本的左轮方向校正计数。
    int16_t right; // 单个有效样本的右轮方向校正计数。
    uint32_t dt_us; // 此样本真实采样时长，单位us。
} EncoderWindowSample;

static EncoderMeasurementSnapshot current; // PIT唯一写入，主循环使用短临界区复制，包含64位累计。
static EncoderWindowSample samples[ENCODER_MEASUREMENT_WINDOW_SAMPLES]; // 最近50个有效样本；异常后全部清窗。
static uint8_t write_index; // 下一次写入环形缓存的位置0至49。
static uint8_t sample_count; // 缓存填充数0至50，未填满时不发布对应RPM。
static int8_t direction_left; // 左轮接线方向，仅允许-1或+1。
static int8_t direction_right; // 右轮独立接线方向。
static uint8_t baseline_ready; // 已保存持续计数和GPT前值为1，首样本不估速。
static uint32_t (*critical_enter)(void); // 返回进入前中断屏蔽状态的回调。
static void (*critical_leave)(uint32_t); // 使用保存值恢复原状态，不能无条件开中断。

static uint32_t lock_measurement(void)
{
    return critical_enter ? critical_enter() : 0u;
}
static void unlock_measurement(uint32_t key)
{
    if (critical_leave) critical_leave(key);
}

// 函数简介     清空估速窗口并撤销所有RPM有效标记
// 参数说明     void
// 返回参数     void
// 备注信息     不清总计数、总时间和参数版本，异常样本不能混入后续有效窗口
static void reset_windows(void)
{
    write_index = sample_count = 0u;
    memset(samples, 0, sizeof(samples));
    current.rpm_valid_100 = current.rpm_valid_500 = 0u;
    current.window_us_100 = current.window_us_500 = 0u;
    current.rpm_left_100 = current.rpm_right_100 = 0.0f;
    current.rpm_left_500 = current.rpm_right_500 = 0.0f;
    current.speed_valid = current.speed_valid_100 = current.speed_valid_500 = 0u;
    current.speed_left_mps = current.speed_right_mps = 0.0f;
    current.speed_left_mps_100 = current.speed_right_mps_100 = 0.0f;
    current.speed_left_mps_500 = current.speed_right_mps_500 = 0.0f;
}

// 函数简介     一致读取指定车轮的计数到线速度比例
// 参数说明     wheel/window_us/scale    轮侧、非零时间us、输出m/s每计数
// 返回参数     uint8_t         双参数已配置且比例有限为1
// 备注信息     只在短临界区内复制N与周长，双精度换算在恢复原中断状态后执行
static uint8_t speed_scale(EncoderWheel wheel, uint32_t window_us, double *scale)
{
    uint32_t key;
    float counts_per_rev;
    float circumference;
    if ((wheel != ENCODER_WHEEL_LEFT && wheel != ENCODER_WHEEL_RIGHT) || window_us == 0u) return 0u;
    key = lock_measurement();
    counts_per_rev = wheel == ENCODER_WHEEL_LEFT ? current.counts_per_rev_left : current.counts_per_rev_right;
    circumference = wheel == ENCODER_WHEEL_LEFT ? current.circumference_left_m : current.circumference_right_m;
    unlock_measurement(key);
    if (counts_per_rev <= 0.0f || circumference <= 0.0f) return 0u; // 未配置时不返回看似有效的零速度。
    *scale = (double)circumference * 1000000.0 / ((double)counts_per_rev * window_us);
    return (uint8_t)(isfinite(*scale) && *scale > 0.0);
}

uint8_t encoder_measurement_counts_to_mps(EncoderWheel wheel, float counts, uint32_t window_us, float *speed_mps)
{
    double scale;
    double result;
    if (!speed_mps || !isfinite(counts) || !speed_scale(wheel, window_us, &scale)) return 0u;
    result = (double)counts * scale; // 净计数保留方向；N已经包含输出车轮与传动机构的总倍率。
    if (!isfinite(result) || fabs(result) > FLT_MAX) return 0u;
    *speed_mps = (float)result;
    return 1u;
}

uint8_t encoder_measurement_mps_to_counts(EncoderWheel wheel, float speed_mps, uint32_t window_us, float *counts)
{
    double scale;
    double result;
    if (!counts || !isfinite(speed_mps) || !speed_scale(wheel, window_us, &scale)) return 0u;
    result = (double)speed_mps / scale; // 返回浮点目标以保留低速分辨率；正负及控制上限另由控制API校验。
    if (!isfinite(result) || fabs(result) > FLT_MAX) return 0u;
    *counts = (float)result;
    return 1u;
}

// 函数简介     汇总最近指定样本数的净计数和真实时长
// 参数说明     count           10或50；输出指针为左右累计及us窗口
// 返回参数     uint8_t         已填满且双轮N有效为1
// 备注信息     RPM=计数×60000000/(每圈计数×us时长)，不假定窗口恰为100/500ms
static uint8_t compute_window(uint8_t count, float *left, float *right, uint32_t *time_us)
{
    int32_t sum_left = 0; // 50×32767仍可用有符号32位表示。
    int32_t sum_right = 0;
    uint32_t elapsed = 0u;
    uint8_t index = write_index;
    uint8_t i;
    double rpm_left;
    double rpm_right;
    if (sample_count < count) return 0u;
    for (i = 0u; i < count; ++i)
    {
        index = index == 0u ? ENCODER_MEASUREMENT_WINDOW_SAMPLES - 1u : index - 1u;
        sum_left += samples[index].left;
        sum_right += samples[index].right;
        elapsed += samples[index].dt_us; // 每样本小于15ms，50槽求和不会溢出。
    }
    *time_us = elapsed;
    if (current.counts_per_rev_left <= 0.0f || current.counts_per_rev_right <= 0.0f || elapsed == 0u) return 0u;
    rpm_left = (double)sum_left * 60000000.0 / ((double)current.counts_per_rev_left * elapsed);
    rpm_right = (double)sum_right * 60000000.0 / ((double)current.counts_per_rev_right * elapsed);
    if (!isfinite(rpm_left) || !isfinite(rpm_right) || fabs(rpm_left) > FLT_MAX || fabs(rpm_right) > FLT_MAX)
    {
        *left = *right = 0.0f; // 极端N不得把浮点溢出后的Inf作为有效物理量发布。
        return 0u;
    }
    *left = (float)rpm_left;
    *right = (float)rpm_right;
    return 1u;
}

void encoder_measurement_set_critical_hooks(uint32_t (*enter)(void), void (*leave)(uint32_t))
{
    if ((enter == NULL) != (leave == NULL)) return; // 拒绝半对钩子，避免进入后无法恢复中断。
    critical_enter = enter;
    critical_leave = leave;
}

void encoder_measurement_init(int8_t sign_left, int8_t sign_right)
{
    memset(&current, 0, sizeof(current));
    direction_left = sign_left;
    direction_right = sign_right;
    baseline_ready = 0u;
    current.calibration_version = 1u;
    reset_windows();
}

uint8_t encoder_measurement_sample(uint16_t raw_left, uint16_t raw_right, uint32_t now_us)
{
    uint16_t bits_left = (uint16_t)(raw_left - current.raw_left); // 无符号模65536差分，可跨FFFF→0000。
    uint16_t bits_right = (uint16_t)(raw_right - current.raw_right);
    uint32_t dt = now_us - current.now_us; // 相邻间隔必须短于GPT一次回绕，差值允许跨32位边界。
    int32_t left;
    int32_t right;
    uint8_t first = (uint8_t)!baseline_ready;
    ++current.sample_id;
    current.now_us = now_us;
    current.raw_left = raw_left;
    current.raw_right = raw_right;
    current.valid = current.counter_valid = current.timing_valid = current.jitter_warning = 0u;
    current.speed_valid = 0u;
    current.speed_left_mps = current.speed_right_mps = 0.0f;
    current.dt_us = first ? 0u : dt;
    current.raw_delta_left = current.raw_delta_right = current.delta_left = current.delta_right = 0;
    baseline_ready = 1u; // 异常也重新建立前值，禁止下次继续使用过时基线。
    if (first && (direction_left == -1 || direction_left == 1) &&
        (direction_right == -1 || direction_right == 1)) return 1u;
    if (!first) current.total_time_us += dt; // 时间保留异常段，不能悄悄删掉超时证据。
    current.timing_valid = (uint8_t)(!first && dt > ENCODER_MEASUREMENT_DT_MIN_US && dt < ENCODER_MEASUREMENT_DT_MAX_US);
    current.jitter_warning = (uint8_t)(!first && (dt < ENCODER_MEASUREMENT_JITTER_MIN_US || dt > ENCODER_MEASUREMENT_JITTER_MAX_US));
    if (!first && bits_left != 0x8000u && bits_right != 0x8000u &&
        (direction_left == -1 || direction_left == 1) && (direction_right == -1 || direction_right == 1))
    {
        // 先在32位解码负差分，排除±32768歧义后再校正符号，避免窄类型反号溢出。
        left = bits_left < 0x8000u ? (int32_t)bits_left : (int32_t)bits_left - 65536;
        right = bits_right < 0x8000u ? (int32_t)bits_right : (int32_t)bits_right - 65536;
        current.raw_delta_left = (int16_t)left;
        current.raw_delta_right = (int16_t)right;
        current.delta_left = (int16_t)(left * direction_left);
        current.delta_right = (int16_t)(right * direction_right);
        current.counter_valid = 1u;
        current.total_left += current.delta_left;
        current.total_right += current.delta_right; // 异常时窗可解码值仅留作证据，不能证明没有隐藏整圈。
    }
    current.valid = (uint8_t)(current.counter_valid && current.timing_valid);
    if (!current.valid)
    {
        if (current.error_count != UINT32_MAX) ++current.error_count;
        current.stationary_us = 0u;
        reset_windows();
        return 0u; // 外层调试模式中止；生产是否继续原控制合同由集成层判断counter_valid。
    }
    if (current.delta_left == 0 && current.delta_right == 0)
        current.stationary_us = UINT32_MAX - current.stationary_us < dt ? UINT32_MAX : current.stationary_us + dt;
    else current.stationary_us = 0u; // 两轮都严格零增量才累计静止证据。
    samples[write_index].left = current.delta_left;
    samples[write_index].right = current.delta_right;
    samples[write_index].dt_us = dt;
    write_index = (uint8_t)((write_index + 1u) % ENCODER_MEASUREMENT_WINDOW_SAMPLES);
    if (sample_count < ENCODER_MEASUREMENT_WINDOW_SAMPLES) ++sample_count;
    current.rpm_valid_100 = compute_window(10u, &current.rpm_left_100, &current.rpm_right_100, &current.window_us_100);
    current.rpm_valid_500 = compute_window(50u, &current.rpm_left_500, &current.rpm_right_500, &current.window_us_500);
    // 单样本与平滑窗口分别按真实时长换算；旧窗口未填满时不能沿用上次速度。
    current.speed_valid = (uint8_t)(encoder_measurement_counts_to_mps(ENCODER_WHEEL_LEFT, current.delta_left, dt, &current.speed_left_mps) &&
        encoder_measurement_counts_to_mps(ENCODER_WHEEL_RIGHT, current.delta_right, dt, &current.speed_right_mps));
    current.speed_valid_100 = (uint8_t)(current.rpm_valid_100 && current.circumference_left_m > 0.0f && current.circumference_right_m > 0.0f);
    current.speed_valid_500 = (uint8_t)(current.rpm_valid_500 && current.circumference_left_m > 0.0f && current.circumference_right_m > 0.0f);
    current.speed_left_mps_100 = current.speed_valid_100 ? current.rpm_left_100 * current.circumference_left_m / 60.0f : 0.0f;
    current.speed_right_mps_100 = current.speed_valid_100 ? current.rpm_right_100 * current.circumference_right_m / 60.0f : 0.0f;
    current.speed_left_mps_500 = current.speed_valid_500 ? current.rpm_left_500 * current.circumference_left_m / 60.0f : 0.0f;
    current.speed_right_mps_500 = current.speed_valid_500 ? current.rpm_right_500 * current.circumference_right_m / 60.0f : 0.0f;
    if (!current.speed_valid) current.speed_left_mps = current.speed_right_mps = 0.0f;
    return 1u;
}

void encoder_measurement_get_snapshot(EncoderMeasurementSnapshot *snapshot)
{
    uint32_t key = lock_measurement();
    if (snapshot) *snapshot = current;
    unlock_measurement(key);
}

uint8_t encoder_measurement_set_counts_per_rev(float left, float right, uint32_t expected_version)
{
    uint32_t key;
    if (!isfinite(left) || !isfinite(right) || left < ENCODER_MEASUREMENT_CPR_MIN || right < ENCODER_MEASUREMENT_CPR_MIN ||
        left > ENCODER_MEASUREMENT_CPR_MAX || right > ENCODER_MEASUREMENT_CPR_MAX) return 0u;
    key = lock_measurement();
    if (expected_version != current.calibration_version)
    {
        unlock_measurement(key);
        return 0u;
    }
    current.counts_per_rev_left = left;
    current.counts_per_rev_right = right;
    ++current.calibration_version;
    reset_windows(); // 新N不能把旧参数下发布的RPM继续标为有效。
    unlock_measurement(key);
    return 1u;
}

uint8_t encoder_measurement_set_wheel_circumference(float left_m, float right_m, uint32_t expected_version)
{
    uint32_t key;
    if (!isfinite(left_m) || !isfinite(right_m) ||
        left_m < ENCODER_MEASUREMENT_CIRCUMFERENCE_MIN_M || right_m < ENCODER_MEASUREMENT_CIRCUMFERENCE_MIN_M ||
        left_m > ENCODER_MEASUREMENT_CIRCUMFERENCE_MAX_M || right_m > ENCODER_MEASUREMENT_CIRCUMFERENCE_MAX_M) return 0u;
    key = lock_measurement();
    if (expected_version != current.calibration_version)
    {
        unlock_measurement(key);
        return 0u;
    }
    current.circumference_left_m = left_m;
    current.circumference_right_m = right_m;
    ++current.calibration_version;
    reset_windows(); // 参数变化清估速窗；持续计数和控制PI单位保持原合同。
    unlock_measurement(key);
    return 1u;
}
