#include "calibration_profile.h"

// 用户确认left_01.json与right_02.json分别手转10圈；只取本轮起终点净计数绝对值。
// 左轮23333/10=2333.3，右轮22842/10=2284.2；单组平均只用于初步物理量换算，重复性仍待测。
const float calibration_profile_counts_per_rev_left = 23333.0f / 10.0f;
const float calibration_profile_counts_per_rev_right = 22842.0f / 10.0f;
// 周长=pi×0.064m，来源为既有vehicle.json中的名义轮径；不是编码器记录直接测得的滚动周长。
const float calibration_profile_circumference_left_m = 0.20106193f;
const float calibration_profile_circumference_right_m = 0.20106193f;

// 来源：servo_right_three_points_20261014.json；脉宽是请求/标称us，角度为右前轮相对车身纵轴的实测deg。
const SteeringWheelMap calibration_profile_right_steering_map = {
    STEERING_SIDE_RIGHT, 3u,
    {{1444.0f, -13.0f}, {1526.0f, 5.0f}, {1610.0f, 17.0f}}
};
// 双轮表仍为空：不能把右轮插值、插值零点或虚构左轮角度当成完整双前轮机械标定。
const SteeringMap calibration_profile_steering_map = {0};
