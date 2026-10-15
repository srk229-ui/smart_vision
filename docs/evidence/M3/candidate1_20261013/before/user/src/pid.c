#include "pid.h"
#include <math.h>

// 兼容旧接口的初始化模板，不自动同步到已初始化的控制器；主速度环由control.c独立配置两轮。
// 速度误差为counts/10ms；Kp单位为PWM命令单位/速度单位，Ki再除以秒，Kd再乘以秒。
// Kd默认关闭；这些初始增益和限幅仍须实测确认。
float speed_KP = 50, speed_KI = 0,speed_KD = 0.0, speed_IMAX = 5000.0, speed_OUTMAX = 5000.0;
float speed_target = 0.0; // 兼容右轮有效目标，counts/10ms；上电为0不发车。
float speed_real = 0.0;   // 兼容右轮本周期编码器反馈，counts/10ms。
float speed_pwm = 0.0;    // 兼容右轮整数化PWM命令，浮点存储，不是引脚反馈。

pid_param_t speed_pid_l;  // 左轮独立参数及积分/误差历史。
pid_param_t speed_pid_r;  // 右轮独立历史，不复用左轮积分。

/*******************************************************************************
* 函 数 名         : My_Pid_Init
* 函数功能         : PID初始化	
* 输    入         : 无
* 输    出         : 无
*******************************************************************************/
void My_Pid_Init(void)
{
    // 两轮使用同一初始化模板，但配置和历史分别保存在各自结构体中。
    Pid_Param_Init(&speed_pid_l, speed_KP, speed_KI,speed_KD, speed_IMAX, speed_OUTMAX);
    Pid_Param_Init(&speed_pid_r, speed_KP, speed_KI,speed_KD, speed_IMAX, speed_OUTMAX);
}


/*******************************************************************************
* 函 数 名         : Pid_Param_Init
* 函数功能         : PID参数初始化	
* 输    入         : pid控制器；kp/ki/kd增益；imax积分贡献上限；outmax正向输出上限
* 输    出         : 无
* 备注信息         : 这里只保存配置；计算时检查合法性，调用方须同步控制中断
*******************************************************************************/
void Pid_Param_Init(pid_param_t * pid, float kp, float ki, float kd, float imax, float outmax)
{
    if (pid == NULL) // 空指针不写配置或历史。
    {
        return;
    }
    pid->kp        = kp;     // 比例增益：误差乘增益得到输出贡献。
    pid->ki        = ki;     // 秒制积分增益：计算时还须乘dt_s。
    pid->kd        = kd;     // 秒制微分增益：误差变化须先除dt_s。
    pid->imax      = imax;   // 积分输出贡献的幅值上限，不是未加权误差上限。
    pid->outmax    = outmax; // 正向命令上限，下限固定为0。
    Pid_Reset(pid);          // 保留刚写入的配置，清除旧积分和误差。
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     清空控制历史，保留增益和限幅配置
// 参数说明     pid             控制器指针，空指针时直接返回
// 返回参数     void
// 备注信息     由控制层在停机、故障和控制模式切换时同步调用
//-------------------------------------------------------------------------------------------------------------------
void Pid_Reset(pid_param_t *pid)
{
    if (pid == NULL) // 非法输入路径可能传入空指针，复位须可安全返回。
    {
        return;
    }
    pid->out_p = 0.0f;           // 清比例贡献；增量式对应本次比例增量。
    pid->out_i = 0.0f;           // 清积分贡献；增量式对应本次积分增量。
    pid->out_d = 0.0f;           // 清微分贡献；增量式对应本次微分增量。
    pid->out = 0.0f;             // 清软件命令，实际PWM由调用方稍后写硬件。
    pid->out_raw = 0.0f;         // 清诊断总量，停机后不保留旧饱和值。
    pid->integrator = 0.0f;      // 清位置式累计贡献，禁止停机后释放旧积分。
    pid->last_error = 0.0f;      // 后续微分从清零后的误差历史计算。
    pid->last_derivative = 0.0f; // 清增量式上次误差差分，不是按秒导数。
    pid->last_t = 0;             // 兼容占位；采样周期只用传入的dt_s。
}

// 检查输入、配置及会参与本次计算的历史状态，防止非法数值进入 PWM 输出。
static int pid_input_valid(const pid_param_t *pid, float error, float dt_s)
{
    // 条件须全部成立；指针先判断，短路求值保证后续成员读取安全。
    return pid != NULL &&
           isfinite(error) &&                            // 误差不能为NaN/无穷，防止污染乘积。
           isfinite(dt_s) && dt_s > 0.0f &&               // 有限正秒数；10ms传0.010f，禁止除零。
           isfinite(pid->kp) && pid->kp >= 0.0f &&        // Kp可为0，禁止非数和负增益。
           isfinite(pid->ki) && pid->ki >= 0.0f &&        // Ki可为0，后续主动清位置式积分。
           isfinite(pid->kd) && pid->kd >= 0.0f &&        // Kd可为0，后续跳过微分计算。
           isfinite(pid->imax) && pid->imax >= 0.0f &&    // 积分贡献幅值非负；0表示不保留I贡献。
           isfinite(pid->outmax) && pid->outmax >= 0.0f && // 正向上限非负；0使最终命令为0。
           isfinite(pid->integrator) &&                  // 位置式会读取旧积分，必须为有限数。
           isfinite(pid->last_error) &&                  // 两种算法都会读取上次误差。
           isfinite(pid->last_derivative) &&             // 增量式会读取上次误差差分。
           isfinite(pid->out);                           // 增量式会读取上次实际命令。
}

// 已饱和时只允许积分向解除饱和的方向变化。
static int pid_integral_allowed(float raw_output, float error, float outmax)
{
    // 上越界且误差正，继续正积分会更饱和；下越界且误差负，同理拒绝继续负积分。
    // 其他方向允许积分帮助退出饱和；恰好等于边界不被这里的严格越界条件拒绝。
    return !((raw_output > outmax && error > 0.0f) ||
             (raw_output < 0.0f && error < 0.0f));
}

/*******************************************************************************
* 函 数 名         : PidLocCtrl
* 函数功能	   : 位置式PID控制
* 输    入         : pid, error, dt_s（单位为秒）
* 输    出         : 正向浮点命令0到outmax，调用方整数化后写PWM
* 备注信息         : 主速度环使用；error为counts/10ms，积分状态直接保存PWM输出贡献
*******************************************************************************/
float PidLocCtrl(pid_param_t *pid, float error, float dt_s)
{
    float integral_delta;     // 本次拟增加的积分贡献，单位与PWM命令相同。
    float integral_candidate; // 积分幅值限幅后的候选总贡献，尚未提交历史。
    float raw_candidate;      // 用候选积分形成的P+I+D，用于判断是否继续推动饱和。

    // 1. 输入、配置或旧历史不合法时，清状态并返回0，不能带异常继续输出。
    if (!pid_input_valid(pid, error, dt_s))
    {
        Pid_Reset(pid);
        return 0.0f;
    }

    // 2. 计算各项：积分状态是已乘Ki的输出贡献，不是误差或误差乘秒的累加量。
    // Ki=0直接丢弃旧积分；非零时先限制历史到当前±imax，避免旧限幅范围残留。
    pid->integrator = pid->ki == 0.0f ? 0.0f :
                      constrain_float(pid->integrator, -pid->imax, pid->imax);
    pid->out_p = pid->kp * error; // P=Kp×误差；速度误差为counts/10ms，结果为PWM贡献。
    // D=Kd×(本次误差-上次误差)/周期秒；主速度环Kd=0，分支直接给0而不计算导数。
    pid->out_d = pid->kd == 0.0f ? 0.0f :
                 pid->kd * ((error - pid->last_error) / dt_s);
    integral_delta = pid->ki == 0.0f ? 0.0f : pid->ki * dt_s * error; // ΔI=Ki×秒×误差；10ms乘0.010。
    // 有限数乘加仍可能溢出：P、D、ΔI和未限幅积分和都检查后才能继续。
    if (!isfinite(pid->out_p) || !isfinite(pid->out_d) || !isfinite(integral_delta) ||
        !isfinite(pid->integrator + integral_delta))
    {
        Pid_Reset(pid);
        return 0.0f;
    }

    // 3. 先限候选积分幅值，再试算总输出；尚不提交可能继续推动饱和的积分。
    integral_candidate = constrain_float(pid->integrator + integral_delta, -pid->imax, pid->imax); // I候选∈[-imax,imax]。
    raw_candidate = pid->out_p + integral_candidate + pid->out_d; // 未经0..outmax裁剪的候选总量。
    if (!isfinite(raw_candidate))
    {
        Pid_Reset(pid);
        return 0.0f;
    }
    if (pid_integral_allowed(raw_candidate, error, pid->outmax))
    {
        pid->integrator = integral_candidate; // 未继续推动越界，或积分正在帮助退出饱和，提交候选。
    }
    else
    {
        // 4. 候选越界且误差仍同向：只允许本次积分走到输出边界。
        // 由P+I+D=边界反求I=边界-P-D；上越界取outmax，下越界取固定下限0。
        float integral_at_limit = (raw_candidate > pid->outmax ? pid->outmax : 0.0f) -
                                  pid->out_p - pid->out_d;
        if (!isfinite(integral_at_limit))
        {
            Pid_Reset(pid);
            return 0.0f;
        }
        // 正误差仅在边界积分大于旧积分时前进；负误差仅在小于旧积分时后退。
        // 已在边界外且继续同向时保持旧积分，不为追逐边界而反向改写历史。
        if ((error > 0.0f && integral_at_limit > pid->integrator) ||
            (error < 0.0f && integral_at_limit < pid->integrator))
        {
            pid->integrator = constrain_float(integral_at_limit, -pid->imax, pid->imax); // 只到输出边界，并遵守±imax。
        }
    }

    // 5. 发布实际提交的I与raw，再裁剪命令；raw保留输出饱和方向供诊断。
    pid->out_i = pid->integrator; // 诊断积分项等于已提交的总输出贡献。
    pid->out_raw = pid->out_p + pid->out_i + pid->out_d; // 最终P+I+D，可为负或超出outmax。
    if (!isfinite(pid->out_raw))
    {
        Pid_Reset(pid);
        return 0.0f;
    }
    pid->out = constrain_float(pid->out_raw, 0.0f, pid->outmax); // 正向电机命令下限0，不输出负占空比。
    pid->last_error = error; // 当前计算结束后保存本次误差，供下次位置式微分读取。
    return pid->out;         // 浮点命令，实际硬件写入和整数化由控制层负责。
}


/*******************************************************************************
* 函 数 名         : PidIncCtrl
* 函数功能	   : 增量式PID控制
* 输    入         : pid, error, dt_s（单位为秒）
* 输    出         : float
* 备注信息         : 仅保留兼容，主速度环使用 PidLocCtrl；切换前清空历史
*******************************************************************************/
float PidIncCtrl(pid_param_t *pid, float error, float dt_s)
{
    float error_delta;   // 本次减上次误差；速度场景为counts/10ms，不是每秒导数。
    float raw_candidate; // 旧实际输出加本次P/I/D增量得到的候选总命令。

    // 1. 兼容接口沿用同一合法性门槛；主速度环不调用它，切换前须清历史。
    if (!pid_input_valid(pid, error, dt_s))
    {
        Pid_Reset(pid);
        return 0.0f;
    }
    // 2. 计算输出增量：out_p/out_i/out_d均为本次增量，不是位置式总贡献。
    error_delta = error - pid->last_error; // Δe=e本次-e上次，末尾才更新误差历史。
    pid->out_p = pid->kp * error_delta;    // ΔP=Kp×Δe，单位为PWM命令。
    pid->out_i = pid->ki == 0.0f ? 0.0f : pid->ki * dt_s * error; // ΔI=Ki×周期秒×误差，Ki=0不增加I。
    // ΔD=Kd×(Δe本次-Δe上次)/周期秒；last_derivative存差分本身，尚未除以秒。
    pid->out_d = pid->kd == 0.0f ? 0.0f :
                 pid->kd * ((error_delta - pid->last_derivative) / dt_s);
    // 差分、乘积和除法都可能溢出；任一增量异常就清输出/历史，不提交本周期。
    if (!isfinite(error_delta) || !isfinite(pid->out_p) ||
        !isfinite(pid->out_i) || !isfinite(pid->out_d))
    {
        Pid_Reset(pid);
        return 0.0f;
    }
    // 3. 仅限制本次积分增量幅值；总输出从上次已限幅的实际命令继续累加。
    pid->out_i = constrain_float(pid->out_i, -pid->imax, pid->imax); // 本次ΔI限制在±imax。
    raw_candidate = pid->out + pid->out_p + pid->out_i + pid->out_d; // u候选=u上次+ΔP+ΔI+ΔD。
    if (!isfinite(raw_candidate))
    {
        Pid_Reset(pid);
        return 0.0f;
    }
    if (!pid_integral_allowed(raw_candidate, error, pid->outmax))
    {
        pid->out_i = 0.0f; // 候选越界且误差仍同向，整步拒绝本次ΔI；保留P/D增量。
    }

    // 4. 用实际允许的增量形成raw并限幅；更新差分历史，不保存位置式积分状态。
    pid->out_raw = pid->out + pid->out_p + pid->out_i + pid->out_d;
    if (!isfinite(pid->out_raw))
    {
        Pid_Reset(pid);
        return 0.0f;
    }
    pid->out = constrain_float(pid->out_raw, 0.0f, pid->outmax); // 保留0..outmax内的命令供下次累加。
    pid->integrator = 0.0f;          // 清位置式专用积分字段，不从该字段释放历史。
    pid->last_derivative = error_delta; // 保存本次Δe，供下次计算相邻差分的变化。
    pid->last_error = error;        // 最后更新误差，不提前覆盖当前计算所需旧历史。
    return pid->out;
}

/*******************************************************************************
* 函 数 名         : constrain_float
* 函数功能         : 浮点型数限幅
* 输    入         : amt,low,high
* 输    出         : float
*******************************************************************************/
float constrain_float(float amt, float low, float high)
{
    // 下越界取low，上越界取high，否则保留原值；调用方保证有限数及low<=high。
    return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}

/*******************************************************************************
* 函 数 名         : constrain_short
* 函数功能         : 短整型数限幅
* 输    入         : amt,low,high
* 输    出         : short
*******************************************************************************/
short constrain_short(short amt, short low, short high)
{
    // 相同的边界裁剪规则；保留旧整型接口，主速度PI不使用本函数。
    return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}

