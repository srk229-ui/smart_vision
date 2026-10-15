#include "steering_map.h"
#include "calibration_profile.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

static void test_bounded_extrapolation(void)
{
    const SteeringWheelMap *map = &calibration_profile_right_steering_map;
    SteeringWheelMap descending = *map;
    float angle = 123.0f, pulse = 789.0f;
    unsigned i;
    assert(steering_wheel_map_angle_to_us_extrapolate(map, -35.0f, -35.0f, 35.0f, &pulse));
    assert(fabsf(pulse - 1343.777778f) < 0.001f);
    assert(steering_wheel_map_us_to_angle_extrapolate(map, pulse, -35.0f, 35.0f, &angle));
    assert(fabsf(angle + 35.0f) < 0.0001f);
    assert(steering_wheel_map_angle_to_us_extrapolate(map, 35.0f, -35.0f, 35.0f, &pulse) && pulse == 1736.0f);
    assert(steering_wheel_map_us_to_angle_extrapolate(map, 1736.0f, -35.0f, 35.0f, &angle) && angle == 35.0f);
    assert(steering_wheel_map_us_to_angle_extrapolate(map, 1344.0f, -35.0f, 35.0f, &angle));
    assert(fabsf(angle + 34.9512195f) < 0.0001f);
    for (i = 1444u; i <= 1610u; ++i)
    {
        float strict;
        assert(steering_wheel_map_us_to_angle(map, (float)i, &strict));
        assert(steering_wheel_map_us_to_angle_extrapolate(map, (float)i, -35.0f, 35.0f, &angle));
        assert(fabsf(angle - strict) < 0.0001f);
    }
    angle = 123.0f; pulse = 789.0f;
    assert(!steering_wheel_map_us_to_angle_extrapolate(map, 1342.0f, -35.0f, 35.0f, &angle));
    assert(!steering_wheel_map_us_to_angle_extrapolate(map, 1738.0f, -35.0f, 35.0f, &angle));
    assert(!steering_wheel_map_us_to_angle_extrapolate(map, NAN, -35.0f, 35.0f, &angle));
    assert(!steering_wheel_map_us_to_angle_extrapolate(map, 499.0f, -35.0f, 35.0f, &angle));
    assert(!steering_wheel_map_us_to_angle_extrapolate(map, 1526.0f, 35.0f, -35.0f, &angle));
    assert(!steering_wheel_map_angle_to_us_extrapolate(map, -35.01f, -35.0f, 35.0f, &pulse));
    assert(!steering_wheel_map_angle_to_us_extrapolate(map, 35.01f, -35.0f, 35.0f, &pulse));
    assert(!steering_wheel_map_angle_to_us_extrapolate(map, NAN, -35.0f, 35.0f, &pulse));
    assert(angle == 123.0f && pulse == 789.0f);
    for (i = 0u; i < descending.point_count; ++i) descending.points[i].wheel_deg *= -1.0f;
    assert(steering_wheel_map_angle_to_us_extrapolate(&descending, -35.0f, -35.0f, 35.0f, &pulse) && pulse == 1736.0f);
    assert(steering_wheel_map_us_to_angle_extrapolate(&descending, pulse, -35.0f, 35.0f, &angle) && angle == -35.0f);
    descending.points[1].pulse_us = descending.points[0].pulse_us;
    assert(!steering_wheel_map_us_to_angle_extrapolate(&descending, 1500.0f, -35.0f, 35.0f, &angle));
    assert(!steering_wheel_map_angle_to_us_extrapolate(&descending, 0.0f, -35.0f, 35.0f, &pulse));
}

int main(void)
{
    SteeringMap map = {0};
    SteeringMap active;
    float left = 123.0f;
    float right = 456.0f;
    float pulse = 789.0f;
    unsigned i;
    test_bounded_extrapolation();
    map.calibrated = 1u;
    map.point_count = 3u;
    map.center_us = 1500.0f;
    map.pulse_min_us = 1400.0f;
    map.pulse_max_us = 1600.0f;
    // 以下只是宿主算法测试合成值，禁止当成实车profile。
    map.points[0] = (SteeringMapPoint){1400.0f, -20.0f, -15.0f};
    map.points[1] = (SteeringMapPoint){1500.0f, 0.0f, 0.0f};
    map.points[2] = (SteeringMapPoint){1600.0f, 15.0f, 20.0f};
    assert(steering_map_validate(&map));
    assert(!steering_map_validate(&calibration_profile_steering_map));
    assert(fabsf(calibration_profile_counts_per_rev_left - 2333.3f) < 0.001f);
    assert(fabsf(calibration_profile_counts_per_rev_right - 2284.2f) < 0.001f);
    assert(steering_wheel_map_validate(&calibration_profile_right_steering_map));
    assert(steering_wheel_map_us_to_angle(&calibration_profile_right_steering_map, 1526.0f, &right) && right == 5.0f);
    assert(steering_wheel_map_us_to_angle(&calibration_profile_right_steering_map, 1444.0f, &right) && right == -13.0f);
    assert(steering_wheel_map_us_to_angle(&calibration_profile_right_steering_map, 1610.0f, &right) && right == 17.0f);
    assert(steering_wheel_map_us_to_angle(&calibration_profile_right_steering_map, 1485.0f, &right) && right == -4.0f);
    assert(steering_wheel_map_us_to_angle(&calibration_profile_right_steering_map, 1568.0f, &right) && right == 11.0f);
    assert(steering_wheel_map_angle_to_us(&calibration_profile_right_steering_map, 0.0f, &pulse));
    assert(fabsf(pulse - 1503.222222f) < 0.001f);
    for (i = 1444u; i <= 1610u; ++i)
    {
        assert(steering_wheel_map_us_to_angle(&calibration_profile_right_steering_map, (float)i, &right));
        assert(steering_wheel_map_angle_to_us(&calibration_profile_right_steering_map, right, &pulse));
        assert(fabsf(pulse - (float)i) < 0.001f);
    }
    right = 456.0f;
    pulse = 789.0f;
    assert(!steering_wheel_map_us_to_angle(&calibration_profile_right_steering_map, 1443.0f, &right));
    assert(!steering_wheel_map_us_to_angle(&calibration_profile_right_steering_map, 1611.0f, &right));
    assert(!steering_wheel_map_us_to_angle(&calibration_profile_right_steering_map, NAN, &right));
    assert(!steering_wheel_map_angle_to_us(&calibration_profile_right_steering_map, 18.0f, &pulse));
    assert(!steering_wheel_map_angle_to_us(&calibration_profile_right_steering_map, -14.0f, &pulse));
    assert(right == 456.0f && pulse == 789.0f);
    {
        SteeringWheelMap partial = calibration_profile_right_steering_map;
        partial.points[1].pulse_us = partial.points[0].pulse_us;
        assert(!steering_wheel_map_validate(&partial));
        partial = calibration_profile_right_steering_map;
        partial.points[2].wheel_deg = -6.0f;
        assert(!steering_wheel_map_validate(&partial));
        partial = calibration_profile_right_steering_map;
        partial.points[1].wheel_deg = INFINITY;
        assert(!steering_wheel_map_validate(&partial));
        partial = calibration_profile_right_steering_map;
        partial.side = (SteeringSide)99;
        assert(!steering_wheel_map_validate(&partial));
        partial = calibration_profile_right_steering_map;
        for (i = 0u; i < partial.point_count; ++i) partial.points[i].wheel_deg *= -1.0f;
        assert(steering_wheel_map_us_to_angle(&partial, 1485.0f, &right) && right == 4.0f);
        assert(steering_wheel_map_angle_to_us(&partial, 4.0f, &pulse) && pulse == 1485.0f);
    }
    assert(steering_map_us_to_angles(&map, 1450.0f, &left, &right));
    assert(left == -10.0f && right == -7.5f);
    assert(steering_map_angle_to_us(&map, STEERING_SIDE_LEFT, -10.0f, &pulse) && pulse == 1450.0f);
    assert(steering_map_angle_to_us(&map, STEERING_SIDE_RIGHT, -10.0f, &pulse));
    assert(fabsf(pulse - 1433.33333f) < 0.001f);
    for (i = 0u; i <= 200u; ++i)
    {
        float input = 1400.0f + i;
        assert(steering_map_us_to_angles(&map, input, &left, &right));
        assert(steering_map_angle_to_us(&map, STEERING_SIDE_LEFT, left, &pulse));
        assert(fabsf(pulse - input) < 0.001f);
        assert(steering_map_angle_to_us(&map, STEERING_SIDE_RIGHT, right, &pulse));
        assert(fabsf(pulse - input) < 0.001f);
    }
    pulse = 789.0f;
    assert(!steering_map_angle_to_us(&map, STEERING_SIDE_LEFT, 16.0f, &pulse) && pulse == 789.0f);
    assert(!steering_map_angle_to_us(&map, (SteeringSide)99, 0.0f, &pulse));
    assert(!steering_map_us_to_angles(&map, NAN, &left, &right));
    assert(!steering_map_us_to_angles(&map, 1300.0f, &left, &right));
    steering_map_init();
    steering_map_get(&active);
    assert(active.version == 1u && !active.calibrated);
    assert(steering_map_apply(&map, active.version));
    assert(!steering_map_apply(&map, active.version));
    steering_map_get(&active);
    assert(active.version == 2u && active.calibrated);
    map.points[1].pulse_us = 1400.0f;
    assert(!steering_map_validate(&map));
    map.points[1].pulse_us = 1500.0f;
    map.points[1].wheel_left_deg = 2.0f;
    assert(!steering_map_validate(&map));
    map.points[1].wheel_left_deg = 0.0f;
    map.points[2].wheel_right_deg = -5.0f;
    assert(!steering_map_validate(&map));
    map.points[2].wheel_right_deg = 20.0f;
    // 反向接线仍允许合法的降序轮角表，反插值必须正确。
    for (i = 0u; i < 3u; ++i)
    {
        map.points[i].wheel_left_deg *= -1.0f;
        map.points[i].wheel_right_deg *= -1.0f;
    }
    assert(steering_map_validate(&map));
    assert(steering_map_angle_to_us(&map, STEERING_SIDE_LEFT, 10.0f, &pulse) && pulse == 1450.0f);
    map.points[2].wheel_left_deg = INFINITY;
    assert(!steering_map_validate(&map));
    puts("steering map tests passed");
    return 0;
}
