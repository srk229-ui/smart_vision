#ifndef CONTROL_H
#define CONTROL_H

#include "control_types.h"

typedef struct
{
    uint32_t frame_id; // 最近接受的有效视觉帧序号；停止/失效时保留来源供诊断。
    uint32_t capture_time_ms; // 该帧原采集逻辑毫秒；前瞻有效期不以处理时刻续期。
    float near_error_px; // 原视觉近带像素误差，左偏为正；也是转向D项的误差域。
    float far_error_px; // 当前帧连续远带加权像素误差；valid=0时不得视为有效前瞻。
    float effective_error_px; // 近远融合后送转向P和弯道降速的像素误差；前瞻失效时退回近带。
    float blend; // 远带混合系数0至0.75；由近远可信度共同限制，非角度或曲率。
    float base_target_left; // TRACK左轮基础斜坡状态，counts/10ms；尚未乘差速倍率。
    float base_target_right; // TRACK右轮基础斜坡状态，counts/10ms；停机/切模式清零。
    float differential_command; // 后轮差速所参考的名义舵机命令度；可能领先本次实际舵机命令。
    float differential_requested; // 映射计算后且限幅±0.7的有符号差速期望比例。
    float differential_applied; // 按4每秒变化率逼近后的实际分配比例；不是实测轮速差。
    uint8_t row_shift; // 远带相对配置五行向上平移的像素行数，当前12至36；valid=0时为尝试值。
    uint8_t valid; // 最近有效帧具备连续远带且年龄不超过100逻辑ms为1，否则0。
    uint8_t lead_active; // 本控制tick启用100ms受限转向前馈为1；停止/失效时为0。
} ControlPreviewSnapshot;

// 函数简介     一致读取前瞻控制诊断，不改变原控制/冻结记录ABI
// 参数说明     snapshot        输出近远误差、前馈和基础/差速斜坡状态；NULL忽略
// 返回参数     void
// 备注信息     只读软件状态，未冻结到原图传记录；不是实测执行器位置或时延
void control_get_preview_snapshot(ControlPreviewSnapshot *snapshot);

/* 操作返回 1 表示成功、0 表示拒绝；不会静默裁剪非法参数。 */

// 函数简介     安装平台短临界区回调，宿主单线程测试可不安装
// 参数说明     enter/leave     保存和恢复中断状态的配对函数
// 返回参数     void
// 备注信息     上电初始化阶段安装；leave必须恢复进入前的PRIMASK，不能一律开中断
void control_set_critical_hooks(uint32_t (*enter)(void), void (*leave)(uint32_t));

// 函数简介     初始化参数与状态，上电保持停止
// 参数说明     servo_center/min/max/direction    舵机初始配置，实车仍须标定
// 返回参数     void
void control_init(float servo_center, float servo_min, float servo_max, float servo_direction);

// 函数简介     设置编译配置允许的执行器输出，关闭时不能由参数重新开启
// 参数说明     motor_enabled/servo_enabled    非零允许对应输出
// 返回参数     void
// 备注信息     许可不等于起车；按模式撤销所需许可时清软件输出，SERVO_CAL允许禁电机，实际脉冲由后续PIT门控
void control_set_output_enabled(uint8_t motor_enabled, uint8_t servo_enabled);

// 函数简介     一致读取参数及版本
// 参数说明     params/version  输出指针，可为空
// 返回参数     void
void control_get_params(ControlParams *params, uint32_t *version);

// 函数简介     停止态原子应用整组参数并清积分
// 参数说明     params/expected_version    新参数及读取时的版本
// 返回参数     uint8_t         成功 1；非法、过期或运行中为 0
// 备注信息     改控制带后需要重新取得一帧有效图像，应用参数不解除故障
uint8_t control_apply_params(const ControlParams *params, uint32_t expected_version);

// 函数简介     提交人工启动请求，校验状态、标定和图像条件
// 参数说明     mode            TRACK 或最长八秒的架空 BENCH
// 返回参数     uint8_t         接受 1，拒绝 0
// 备注信息     兼容人工/诊断接口仍需确认、输出许可及正目标；TRACK还需最新有效帧≤500ms；Wi-Fi不使用此启动接口
uint8_t control_request_run(ControlMode mode);

// 函数简介     本地KEY1请求循迹启动，允许清可恢复视觉故障后重新运行
// 参数说明     void
// 返回参数     uint8_t         全部启动条件满足为1，否则0且保持停止/故障
// 备注信息     不要求calibration_confirmed；要求最新有效图像≤500ms、正目标、输出许可及停机屏障；标定独占时拒绝
uint8_t control_request_local_track_run(void);

// 函数简介     请求停止并清输出、目标及积分，故障仍保持锁存
// 参数说明     void
// 返回参数     void
// 备注信息     清零运行目标但保留参数中的基础目标；下一个PIT写零，再由PWM外设重载
//              停机屏障至少经过一次tick后解除；本接口不等待机械停止
void control_request_stop(void);

// 函数简介     人工解除故障，不自动启动
// 参数说明     void
// 返回参数     uint8_t         接受 1；运行中或未经过停机周期为 0
// 备注信息     成功后回READY，保留最近帧有效性和采集时间；不会把无效或过期帧变成启动资格，也不自动运行
uint8_t control_reset_fault(void);

// 函数简介     诊断等显式链路异常时锁存故障，普通Wi-Fi收发不调用
// 参数说明     void
// 返回参数     void
void control_report_link_fault(void);

// 函数简介     锁存平台初始化失败，本次上电不能用清故障恢复
// 参数说明     void
// 返回参数     void
// 备注信息     时钟/profile/持续计数配置失败时调用，修复后重新初始化才解锁
void control_report_initialization_fault(void);

// 函数简介     参数或模块接口不一致时锁存故障
// 参数说明     void
// 返回参数     void
void control_report_invalid_fault(void);

// 函数简介     发布新帧，按采集时间判断新鲜度，重复帧不计数
// 参数说明     vision          只读视觉结果
// 返回参数     void
// 备注信息     主循环调用；复制数据不保存传入指针。时间为10ms逻辑量化，非独立硬件时戳
void control_publish_vision(const VisionResult *vision);

// 函数简介     固定十毫秒采样、保护、PI和舵机输出计算
// 参数说明     measured_left/right    本周期编码器增量
// 返回参数     void
// 备注信息     PIT配置标称10ms，每次被服务按0.010秒更新；测量约定counts/10ms
//              调用者把输出快照施加到硬件，不在此处执行网络或显示
void control_tick(int16_t measured_left, int16_t measured_right);

// 函数简介     读取左右轮、真实量化 PWM、舵机和故障的一致快照
// 参数说明     snapshot        输出指针，可为空
// 返回参数     void
// 备注信息     字段复制一致，但主循环可能已发布更晚视觉帧；frame_id不是PWM来源凭证
void control_get_snapshot(ControlSnapshot *snapshot);

// 函数简介     读取运行状态
// 参数说明     void
// 返回参数     ControlState    当前状态
ControlState control_get_state(void);

// 函数简介     读取十毫秒控制时基，支持无符号回绕差值
// 参数说明     void
// 返回参数     uint32_t        设备逻辑时间，单位ms，每次tick加10；漏服务时不追赶物理时间
uint32_t control_now_ms(void);

// 函数简介     配置舵机PWM电气刻度，初始化后、PIT启动前调用
// 参数说明     duty_max/frequency_hz    硬件占空最大计数及舵机频率Hz
// 返回参数     void
// 备注信息     保留原TRACK/BENCH双精度换算与整数截断，不把命令度改为轮角
void control_configure_servo_pwm(uint32_t duty_max, uint32_t frequency_hz);

// 函数简介     查询当前占空计数对应的名义微秒刻度
// 参数说明     void
// 返回参数     float    单个占空计数对应us；不是引脚实测分辨率
// 备注信息     用于离线记录反算，不允许主机硬编码50Hz的2us比例
float control_servo_us_per_duty(void);

// 函数简介     设置本地标定独占准备许可，阻止普通TRACK/BENCH启动
// 参数说明     reserved    1占用，0释放；准备前必须请求停止
// 返回参数     void
// 备注信息     本标志不授予输出许可、不代替人工确认，STOP始终可用
void control_set_calibration_reserved(uint8_t reserved);

// 函数简介     查询显式停止/故障代次，使READY中的被动手转也能响应V1 STOP
// 参数说明     void
// 返回参数     uint32_t    单调无符号代次，允许回绕
// 备注信息     仅比较是否变化，不把它当硬件已停止或机械静止证据
uint32_t control_stop_generation(void);

// 函数简介     查询是否由标定会话独占准备，普通参数和启动在此期间拒绝
// 参数说明     void
// 返回参数     uint8_t    1独占，0未独占
// 备注信息     用于主循环阻止宽度标定等其他准备操作，STOP不受影响
uint8_t control_get_calibration_reserved(void);

// 函数简介     标定独占会话停车时临时应用或还原整组控制参数
// 参数说明     params/version    参数候选及当前版本，单位与原ControlParams一致
// 返回参数     uint8_t    成功1；非独占、运行中、过期或非法为0
// 备注信息     仅诊断版；不永久授予TRACK硬件确认，试验结束须还原准备前参数
uint8_t control_apply_calibration_params(const ControlParams *params, uint32_t version);

// 函数简介     在标定独占会话中请求原有BENCH，仍检查正目标与硬件人工确认
// 参数说明     void
// 返回参数     uint8_t    接受1，拒绝0
// 备注信息     仅诊断版开放，不绕过编译许可、停车屏障或8秒BENCH规则
uint8_t control_request_calibration_bench(void);

// 函数简介     诊断版请求舵机单点运行，两电机始终保持零输出
// 参数说明     center/min/max    人工确认的中位和安全脉宽范围，单位us
// 参数说明     rate_us_s/version    脉宽变化率us/s及准备时参数版本
// 返回参数     uint8_t    接受1，非法/运行中/未独占/许可关闭/版本过期为0
// 备注信息     需要本次人工确认；时限和心跳由标定模块用GPT独立时间保护
uint8_t control_request_servo_cal(float center_us, float min_us, float max_us,
                                 float rate_us_s, uint32_t expected_params_version);

// 函数简介     设置已人工启动的舵机单点目标，不延长试验总时限
// 参数说明     pulse_us    已确认范围内的有限脉宽us
// 返回参数     uint8_t    接受1，越界/非标定运行为0
// 备注信息     输出只在后续PIT统一施加，不直接访问PWM寄存器
uint8_t control_set_servo_cal_pulse_us(float pulse_us);

// 函数简介     查询单点目标脉宽，供诊断记录区分目标与限速后的请求
// 参数说明     void
// 返回参数     float    目标us；非SERVO_CAL运行时为0
// 备注信息     返回控制目标，不是机械角反馈或引脚测量
float control_servo_cal_requested_us(void);

#endif
