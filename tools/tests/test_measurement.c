#include "encoder_measurement.h"
#include <assert.h>
#include <math.h>
#include <float.h>
#include <stdio.h>

static EncoderMeasurementSnapshot snapshot;
static unsigned enters;
static unsigned leaves;
static uint32_t enter_test(void) { ++enters; return 0x5au; }
static void leave_test(uint32_t key) { assert(key == 0x5au); ++leaves; }
static void read_snapshot(void) { encoder_measurement_get_snapshot(&snapshot); }

int main(void)
{
    uint16_t left = 0xfffeu;
    uint16_t right = 1u;
    uint32_t time = 0xfffff000u;
    unsigned i;
    float converted = 123.0f;
    encoder_measurement_set_critical_hooks(enter_test, leave_test);
    encoder_measurement_init(-1, 1);
    assert(encoder_measurement_sample(left, right, time));
    read_snapshot();
    assert(snapshot.sample_id == 1u && !snapshot.valid && !snapshot.counter_valid && snapshot.dt_us == 0u);
    assert(!encoder_measurement_counts_to_mps(ENCODER_WHEEL_LEFT, 10.0f, 10000u, &converted) && converted == 123.0f);
    assert(!encoder_measurement_set_counts_per_rev(0.0f, 200.0f, 1u));
    assert(!encoder_measurement_set_counts_per_rev(NAN, 200.0f, 1u));
    assert(encoder_measurement_set_counts_per_rev(100.0f, 200.0f, 1u));
    assert(!encoder_measurement_set_counts_per_rev(100.0f, 200.0f, 1u));
    assert(!encoder_measurement_mps_to_counts(ENCODER_WHEEL_LEFT, 1.0f, 10000u, &converted));
    assert(!encoder_measurement_set_wheel_circumference(0.0f, 0.2f, 2u));
    assert(!encoder_measurement_set_wheel_circumference(NAN, 0.2f, 2u));
    assert(!encoder_measurement_set_wheel_circumference(0.1f, 200.0f, 2u));
    assert(encoder_measurement_set_wheel_circumference(0.1f, 0.2f, 2u));
    assert(!encoder_measurement_set_wheel_circumference(0.1f, 0.2f, 2u));
    assert(encoder_measurement_counts_to_mps(ENCODER_WHEEL_LEFT, 10.0f, 10000u, &converted));
    assert(fabsf(converted - 1.0f) < 0.00001f);
    assert(encoder_measurement_mps_to_counts(ENCODER_WHEEL_RIGHT, -2.0f, 10000u, &converted));
    assert(fabsf(converted + 20.0f) < 0.00001f);
    converted = 123.0f;
    assert(!encoder_measurement_counts_to_mps((EncoderWheel)99, 1.0f, 10000u, &converted));
    assert(!encoder_measurement_counts_to_mps(ENCODER_WHEEL_LEFT, 1.0f, 0u, &converted));
    assert(!encoder_measurement_counts_to_mps(ENCODER_WHEEL_LEFT, INFINITY, 10000u, &converted));
    assert(!encoder_measurement_counts_to_mps(ENCODER_WHEEL_LEFT, FLT_MAX, 1u, &converted));
    assert(!encoder_measurement_mps_to_counts(ENCODER_WHEEL_LEFT, FLT_MAX, UINT32_MAX, &converted));
    assert(!encoder_measurement_mps_to_counts(ENCODER_WHEEL_LEFT, NAN, 10000u, &converted));
    assert(!encoder_measurement_mps_to_counts(ENCODER_WHEEL_LEFT, 1.0f, 10000u, NULL));
    assert(converted == 123.0f); // 所有非法/溢出请求保持调用者原值，不把失败当零目标。
    // 连续负向左原始计数和正向右计数，同时覆盖16位及GPT32位回绕。
    for (i = 0u; i < 60u; ++i)
    {
        left = (uint16_t)(left - 10u);
        right = (uint16_t)(right + 20u);
        time += 10000u;
        assert(encoder_measurement_sample(left, right, time));
    }
    read_snapshot();
    assert(snapshot.valid && snapshot.counter_valid && snapshot.timing_valid);
    assert(snapshot.delta_left == 10 && snapshot.delta_right == 20 && snapshot.raw_delta_left == -10);
    assert(snapshot.total_left == 600 && snapshot.total_right == 1200 && snapshot.total_time_us == 600000u);
    assert(snapshot.rpm_valid_100 && snapshot.rpm_valid_500);
    assert(snapshot.window_us_100 == 100000u && snapshot.window_us_500 == 500000u);
    assert(fabsf(snapshot.rpm_left_500 - 600.0f) < 0.001f && fabsf(snapshot.rpm_right_100 - 600.0f) < 0.001f);
    assert(snapshot.speed_valid && snapshot.speed_valid_100 && snapshot.speed_valid_500);
    assert(fabsf(snapshot.speed_left_mps - 1.0f) < 0.00001f);
    assert(fabsf(snapshot.speed_right_mps_100 - 2.0f) < 0.00001f);
    assert(fabsf(snapshot.speed_left_mps_500 - 1.0f) < 0.00001f);
    // 异常时窗保留可解码delta和累计证据，清窗且禁止验收。
    left = (uint16_t)(left - 10u);
    right = (uint16_t)(right + 20u);
    time += 15000u;
    assert(!encoder_measurement_sample(left, right, time));
    read_snapshot();
    assert(!snapshot.valid && snapshot.counter_valid && !snapshot.timing_valid && snapshot.error_count == 1u);
    assert(snapshot.total_left == 610 && snapshot.total_time_us == 615000u && !snapshot.rpm_valid_500);
    assert(!snapshot.speed_valid && !snapshot.speed_valid_100 && !snapshot.speed_valid_500);
    assert(snapshot.speed_left_mps == 0.0f && snapshot.speed_right_mps_500 == 0.0f);
    // 半环差分无法区分+32768与-32768，拒绝并重建前值。
    left = (uint16_t)(left + 0x8000u);
    time += 10000u;
    assert(!encoder_measurement_sample(left, right, time));
    read_snapshot();
    assert(!snapshot.counter_valid && snapshot.delta_left == 0 && snapshot.error_count == 2u);
    for (i = 0u; i < 10u; ++i)
    {
        time += 10000u;
        assert(encoder_measurement_sample(left, right, time));
    }
    read_snapshot();
    assert(snapshot.stationary_us == 100000u && snapshot.rpm_valid_100 && snapshot.rpm_left_100 == 0.0f);
    assert(encoder_measurement_set_counts_per_rev(300.0f, 300.0f, snapshot.calibration_version));
    read_snapshot();
    assert(!snapshot.rpm_valid_100 && snapshot.total_left == 610);
    assert(!snapshot.speed_valid && !snapshot.speed_valid_100 && snapshot.speed_left_mps_100 == 0.0f);
    // 合法但有抖动的真实窗口；换算采用真实9000us而非固定10ms。
    for (i = 0u; i < 50u; ++i)
    {
        left = (uint16_t)(left - 9u);
        right = (uint16_t)(right + 9u);
        time += 9000u;
        assert(encoder_measurement_sample(left, right, time));
    }
    read_snapshot();
    assert(snapshot.jitter_warning && snapshot.window_us_500 == 450000u);
    assert(fabsf(snapshot.rpm_left_500 - 200.0f) < 0.001f);
    assert(snapshot.speed_valid_500 && fabsf(snapshot.speed_left_mps_500 - 1.0f / 3.0f) < 0.00001f);
    assert(fabsf(snapshot.speed_right_mps - 2.0f / 3.0f) < 0.00001f); // 真实9000us换算，不能固定除以10ms。
    assert(enters == leaves);
    // 边界5000us和0us拒绝；非法方向配置连基线也拒绝。
    assert(!encoder_measurement_sample(left, right, time + 5000u));
    assert(!encoder_measurement_sample(left, right, time + 5000u));
    encoder_measurement_init(0, 1);
    assert(!encoder_measurement_sample(0u, 0u, 0u));
    encoder_measurement_init(1, 1);
    assert(encoder_measurement_sample(0xffffu, 0u, 0u));
    assert(encoder_measurement_sample(0u, 0xffffu, 10000u));
    read_snapshot();
    assert(snapshot.delta_left == 1 && snapshot.delta_right == -1);
    // 多次完整计数器回绕累计超过INT32_MAX，证明里程证据使用64位而非窄计数。
    encoder_measurement_init(1, 1);
    left = right = 0u;
    time = 0u;
    assert(encoder_measurement_sample(left, right, time));
    for (i = 0u; i < 70000u; ++i)
    {
        left = (uint16_t)(left + 32767u);
        right = (uint16_t)(right - 32767u);
        time += 10000u;
        assert(encoder_measurement_sample(left, right, time));
    }
    read_snapshot();
    assert(snapshot.total_left == (int64_t)70000 * 32767 && snapshot.total_right == -(int64_t)70000 * 32767);
    assert(!snapshot.rpm_valid_100 && snapshot.error_count == 0u);
    // 正但极小的非法物理参数在参数入口拒绝，最小合法N计算仍须有限。
    assert(!encoder_measurement_set_counts_per_rev(1.0e-40f, 1.0e-40f, snapshot.calibration_version));
    assert(encoder_measurement_set_counts_per_rev(1.0f, 1.0f, snapshot.calibration_version));
    for (i = 0u; i < 10u; ++i)
    {
        left = (uint16_t)(left + 1u);
        right = (uint16_t)(right + 1u);
        time += 10000u;
        assert(encoder_measurement_sample(left, right, time));
    }
    read_snapshot();
    assert(snapshot.rpm_valid_100 && isfinite(snapshot.rpm_left_100) && snapshot.rpm_left_100 == 6000.0f);
    assert(enters == leaves);
    puts("measurement tests passed");
    return 0;
}
