#ifndef CALIBRATION_PROFILE_H
#define CALIBRATION_PROFILE_H // 可审计编译期实测标定基线防重复包含。
#include "steering_map.h"
#define CALIBRATION_STEERING_RIGHT_LIMIT_DEG (33.0f) // 右转外推估算角幅值上限33deg；右前轮车身纵轴角取负，未经机械实测。
#define CALIBRATION_STEERING_LEFT_LIMIT_DEG (20.0f) // 左转外推估算角幅值上限20deg；原始角取正，独立于右转上限。
#define CALIBRATION_SERVO_RIGHT_COMMAND (76.87f) // 50Hz/10000刻度下截为1354us，估算右转32.7561deg；非实测舵机角。
#define CALIBRATION_SERVO_LEFT_COMMAND (101.71f) // 同刻度下截为1630us，估算左转19.8571deg；留0.01命令度避免浮点落入前一格。
extern const float calibration_profile_counts_per_rev_left; // 左输出车轮单组10圈平均counts/rev；初步实测比例，正反重复性待验收。
extern const float calibration_profile_counts_per_rev_right; // 右输出车轮单组10圈平均counts/rev；两轮分别换算，不再次乘减速比。
extern const float calibration_profile_circumference_left_m; // 左轮换算周长m，当前由用户给定名义轮径64mm推算，滚动周长待测。
extern const float calibration_profile_circumference_right_m; // 右轮换算周长m；此基线不代表已验证车体地速或打滑误差。
extern const SteeringWheelMap calibration_profile_right_steering_map; // 右前轮单次三点插值，保留车身纵轴参考；左轮和实测零点尚缺。
extern const SteeringMap calibration_profile_steering_map; // 离线验证后生成的平均实测表；当前为空表，不构造示例实测值。
#endif
