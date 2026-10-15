#include "calibration_profile.h"
#include "calibration_debug.h"
#include "encoder_measurement.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

int main(void)
{
    EncoderMeasurementSnapshot sample;
    float left_target;
    float right_target;
    float left_speed;
    float right_speed;
    float angle;
    float pulse;
    unsigned i;
    encoder_measurement_init(-1, 1);
    assert(encoder_measurement_set_counts_per_rev(calibration_profile_counts_per_rev_left,
        calibration_profile_counts_per_rev_right, 1u));
    encoder_measurement_get_snapshot(&sample);
    assert(encoder_measurement_set_wheel_circumference(calibration_profile_circumference_left_m,
        calibration_profile_circumference_right_m, sample.calibration_version));
    assert(encoder_measurement_mps_to_counts(ENCODER_WHEEL_LEFT, 0.5f, 10000u, &left_target));
    assert(encoder_measurement_mps_to_counts(ENCODER_WHEEL_RIGHT, 0.5f, 10000u, &right_target));
    assert(left_target != right_target); // 分轮N不同，相同物理速度需要不同计数目标。
    assert(encoder_measurement_counts_to_mps(ENCODER_WHEEL_LEFT, left_target, 10000u, &left_speed));
    assert(encoder_measurement_counts_to_mps(ENCODER_WHEEL_RIGHT, right_target, 10000u, &right_speed));
    assert(fabsf(left_speed - 0.5f) < 0.000001f && fabsf(right_speed - 0.5f) < 0.000001f);
    assert(encoder_measurement_sample(0u, 0u, 0xffff0000u));
    for (i = 1u; i <= 50u; ++i)
        assert(encoder_measurement_sample((uint16_t)(0u - 100u * i), (uint16_t)(100u * i), 0xffff0000u + 9000u * i));
    encoder_measurement_get_snapshot(&sample);
    assert(sample.speed_valid && sample.speed_valid_100 && sample.speed_valid_500);
    assert(sample.total_left == 5000 && sample.total_right == 5000 && sample.window_us_500 == 450000u);
    assert(encoder_measurement_counts_to_mps(ENCODER_WHEEL_LEFT, 100.0f, 9000u, &left_speed));
    assert(fabsf(sample.speed_left_mps_500 - left_speed) < 0.000001f);
    assert(sample.speed_right_mps > sample.speed_left_mps); // 相同计数窗口不能被当成相同物理轮速。
    assert(steering_wheel_map_us_to_angle(&calibration_profile_right_steering_map, 1550.0f, &angle));
    assert(steering_wheel_map_angle_to_us(&calibration_profile_right_steering_map, 0.0f, &pulse));
    assert(!steering_map_validate(&calibration_profile_steering_map));
    assert(sizeof(CalibrationDebugMailbox) == 416u && sizeof(CalibrationDebugRecord) == 32u);
    printf("{\"left_cpr\":%.9g,\"right_cpr\":%.9g,\"left_circumference_m\":%.9g,"
        "\"right_circumference_m\":%.9g,\"left_target_counts_050\":%.9g,\"right_target_counts_050\":%.9g,"
        "\"right_deg_1550\":%.9g,\"right_zero_us\":%.9g,\"mailbox_bytes\":416,\"record_bytes\":32}\n",
        (double)calibration_profile_counts_per_rev_left, (double)calibration_profile_counts_per_rev_right,
        (double)calibration_profile_circumference_left_m, (double)calibration_profile_circumference_right_m,
        (double)left_target, (double)right_target, (double)angle, (double)pulse);
    return 0;
}
