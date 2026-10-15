#ifndef _PID_H // PID公共接口头文件的重复包含保护。
#define _PID_H // 防止控制器类型和声明被重复展开。

#include <stddef.h>
#include <stdint.h>

typedef struct
{
    // 配置均要求有限非负；速度场景误差单位counts/10ms，输出贡献使用PWM命令单位。
    float                kp;                    // 比例增益，单位PWM命令单位/(counts/10ms)。
    // Ki单位PWM命令单位/[(counts/10ms)·s]，每次积分还须乘周期秒数。
    float                ki;                    // 秒制积分增益，0关闭积分。
    // Kd单位PWM命令单位·s/(counts/10ms)，误差变化须先除周期秒数。
    float                kd;                    // 秒制微分增益，主速度环为0。
    // 位置式限制总I，增量式只限制本次ΔI；均使用PWM命令单位，不是误差上限。
    float                imax;                  // 积分贡献的绝对值上限。

    float                out_p;                 // 位置式P输出贡献；增量式为本次ΔP，单位PWM命令。
    float                out_i;                 // 位置式已提交的总I贡献；增量式为本次ΔI，单位PWM命令。
    float                out_d;                 // 位置式D输出贡献；增量式为本次ΔD，单位PWM命令。
    // out由控制层整数化后写PWM，属于计算命令，不是引脚实测反馈。
    float                out;                   // 0..outmax的正向浮点命令。
    float                out_raw;               // 限幅前总命令，可越界，诊断饱和方向。
    // 主控制层另约束上限不超过5000；本PID接口仅要求有限非负，下限固定0。
    float                outmax;                // 正向命令上限，单位PWM命令。

    // 不存未加权误差；Ki=0清零，增量式计算完成后也清零此位置式专用状态。
    float                integrator;            // 位置式总积分贡献，范围±imax。
    float                last_error;            // 上次成功计算的误差，速度场景为counts/10ms；复位为0。
    float                last_derivative;       // 增量式上次误差差分Δe，与误差同单位，不是每秒导数；复位为0。
    unsigned long        last_t;                // 兼容占位，复位为0；周期用dt_s，不读取它。
}pid_param_t;

extern pid_param_t speed_pid_l;  // 左轮独立控制器及历史，调用方须与控制中断同步访问。
extern pid_param_t speed_pid_r;  // 右轮独立控制器及历史，不共享左轮积分。
// 兼容初始化模板：单位同上述增益/限幅，变量改动不会自动应用到已初始化的控制器。
extern float speed_KP, speed_KI,speed_KD, speed_IMAX, speed_OUTMAX;
extern float speed_target;                      // 兼容右轮有效目标，counts/10ms；正向误差=目标-对应反馈。
extern float speed_real;                        // 兼容右轮本周期编码器反馈，counts/10ms；不是m/s或rpm。
extern float speed_pwm;                         // 兼容右轮整数化PWM命令，浮点存储，不是引脚实测值。


void My_Pid_Init(void); // 用兼容模板分别初始化左右控制器并清历史，不发车。
// 保存增益/限幅并复位；空指针返回，调用方须与控制中断同步。
void Pid_Param_Init(pid_param_t * pid, float kp, float ki, float kd, float imax, float outmax);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     清空控制历史，保留增益和限幅配置
// 参数说明     pid             控制器指针，空指针时直接返回
// 返回参数     void
// 备注信息     停机、故障和控制模式切换时调用，必须与控制中断同步
//-------------------------------------------------------------------------------------------------------------------
void Pid_Reset(pid_param_t *pid);

float constrain_float(float amt, float low, float high); // 浮点边界裁剪，调用方保证有限数及low<=high。
short constrain_short(short amt, short low, short high); // 保留旧整型裁剪接口，调用方保证low<=high。


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     带条件积分的正向位置式 PID，速度环使用本接口
// 参数说明     pid             控制器指针
// 参数说明     error           目标与反馈之差；主速度环单位counts/10ms
// 参数说明     dt_s            采样周期，单位为秒，十毫秒应传入 0.010f
// 返回参数     float           正向浮点命令，控制层整数化写PWM；非法输入返回零并清历史
// 备注信息     Ki 为零时清积分；积分输出限制在正负 imax，实际输出限制在零到 outmax
//-------------------------------------------------------------------------------------------------------------------
float PidLocCtrl(pid_param_t *pid, float error, float dt_s);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     保留兼容的正向增量式 PID
// 参数说明     pid             控制器指针，不能与位置式控制混用同一历史状态
// 参数说明     error           目标与反馈之差；速度场景单位counts/10ms
// 参数说明     dt_s            采样周期，单位为秒
// 返回参数     float           正向浮点命令；非法输入返回零并清空控制历史
// 备注信息     不用于主速度环；切换前调用Pid_Reset；imax限本次ΔI，拒绝饱和积分时整步舍弃
//-------------------------------------------------------------------------------------------------------------------
float PidIncCtrl(pid_param_t *pid, float error, float dt_s);


#endif
