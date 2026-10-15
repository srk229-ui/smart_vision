#include "zf_common_headfile.h"
#include "img_process.h"
#include "wifi_assistant.h"
#include "control.h"
#include "camera_frame.h"
#include "local_buttons.h"

// 5ms PIT调用本地按键扫描；启动只登记，停止优先且不等待主循环。
void local_drive_scan(void);
#include "pid.h"
#include "calibration_config.h"
#include "calibration_clock.h"
#include "encoder_measurement.h"
#include "calibration_debug.h"
#include "steering_map.h"
#include "calibration_profile.h"

// 图像尺寸必须与视觉接口一致，避免修改相机配置后发生缓冲越界。
#if MT9V03X_W != CONTROL_IMAGE_WIDTH || MT9V03X_H != CONTROL_IMAGE_HEIGHT
#error "Camera dimensions must match the control interface"
#endif

#define CONTROL_LOOP_PERIOD_MS (CONTROL_PERIOD_MS) // PIT与通信tick统一周期，单位ms；当前10，不能单独改此别名

// 编码器引脚信息
// 请确保两轮编码器前进时回传值都为正数，符号相反时修改 ENCODER_1_SIGN 或 ENCODER_2_SIGN，并重新架空核对
// 前进编码器符号，必须通过架空轮或手动滚轮核对。
#define ENCODER_1_SIGN (-1) // 左轮原始计数乘此符号；仅为当前接线配置，前进为正须实测
#define ENCODER_2_SIGN (1) // 右轮方向独立标定，不能因左轮通过而默认右轮正确
// 左轮编码器
#define ENCODER_1 (QTIMER1_ENCODER1)
#define ENCODER_1_A (QTIMER1_ENCODER1_CH1_C0)
#define ENCODER_1_B (QTIMER1_ENCODER1_CH2_C1)
// 右轮编码器
#define ENCODER_2 (QTIMER1_ENCODER2)
#define ENCODER_2_A (QTIMER1_ENCODER2_CH1_C2)
#define ENCODER_2_B (QTIMER1_ENCODER2_CH2_C24)

// 电机参数设置
#ifndef MOTOR_ENABLE
#define MOTOR_ENABLE (1) // 编译期电机输出许可，1仍需人工RUN与全部保护通过；0禁止软件重新开启

#endif
// !!! 请确保dir和pwm脚不要弄反了，否则电机会疯转 !!!
// 左轮电机
#define MOTOR1_PWM (PWM2_MODULE0_CHA_C6)
#define MOTOR1_DIR (C7)
// 左电机前进需要的DIR脚的电平 (GPIO_HIGH or GPIO_LOW) 请自行测试
#define MOTOR1_FORWARD_DIR_LEVEL (GPIO_HIGH)
// 右轮电机
#define MOTOR2_PWM (PWM2_MODULE1_CHA_C8)
#define MOTOR2_DIR (C9)
// 右电机前进需要的DIR脚的电平 (GPIO_HIGH or GPIO_LOW) 请自行测试
#define MOTOR2_FORWARD_DIR_LEVEL (GPIO_HIGH)


// 舵机参数设置
#ifndef SERVO_MOTOR_ENABLE
#define SERVO_MOTOR_ENABLE (1) // 编译期舵机输出许可，停止/故障仍关闭脉冲；0禁止软件重新开启

#endif
#define SERVO_MOTOR_PWM (PWM4_MODULE2_CHA_C30) // 定义主板上舵机对应引脚
#define SERVO_MOTOR_CENTER (92.5f) // 独立直行中位，不能用左右极限平均值代替实车标定
#define SERVO_MOTOR_FREQ (50)                  // 定义主板上舵机频率  请务必注意范围 50-300
#define SERVO_MOTOR_L_MAX (CALIBRATION_SERVO_LEFT_COMMAND) // 左转外推极限命令；对应右前轮相对车身纵轴估算+19.8571deg。
#define SERVO_MOTOR_R_MAX (CALIBRATION_SERVO_RIGHT_COMMAND) // 右转外推极限命令；PWM向范围内取格后估算-32.7561deg。
#define SERVO_DIR (SERVO_MOTOR_L_MAX>SERVO_MOTOR_R_MAX?-1.f:1.f) // 根据左右duty的大小自动决定舵机方向

// 以下宏在初步测试例程时不可更改，后面若需更改舵机算法可以自行更改
// ------------------ 舵机占空比计算方式 ------------------
//
// 舵机对应的 0-180 活动角度对应 控制脉冲的 0.5ms-2.5ms 高电平
//
// 命令角x对应的高电平宽度为0.5+x/90，单位ms；90命令度对应1.5ms。
// 占空比计数=PWM_DUTY_MAX × 脉宽ms / 周期ms，周期ms=1000/SERVO_MOTOR_FREQ。
// 例如50Hz、90命令度：PWM_DUTY_MAX × 1.5/20；100Hz时为PWM_DUTY_MAX × 1.5/10。
// 这里的命令度是脉宽刻度，并非实测前轮转角；机械中位/极限必须单独标定。
//
// ------------------ 舵机占空比计算方式 ------------------
// 此宏只换算，不负责限位；统一控制器先按配置限位约束命令，再由PIT调用；机械安全范围仍待标定。
#define SERVO_MOTOR_DUTY(x) ((float)PWM_DUTY_MAX / (1000.0 / (float)SERVO_MOTOR_FREQ) * (0.5 + (float)(x) / 90.0))

#if (SERVO_MOTOR_FREQ < 50 || SERVO_MOTOR_FREQ > 300)
#error "SERVO_MOTOR_FREQ ERROE!"
#endif
