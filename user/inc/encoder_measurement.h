#ifndef ENCODER_MEASUREMENT_H
#define ENCODER_MEASUREMENT_H // 编码器物理量测量接口的防重复包含标记。

#include <stdint.h>
#include "calibration_config.h"

#ifndef ENCODER_MEASUREMENT_DT_GUARD_ENABLED
#define ENCODER_MEASUREMENT_DT_GUARD_ENABLED CALIBRATION_DEBUG_ENABLE // 调试集成层的异常时窗停机开关；测量有效性始终检查。
#endif
#define ENCODER_MEASUREMENT_DT_MIN_US (5000u) // 小于等于5ms的窗口无效；标称10ms与异常0ms的中点，非实测门槛。
#define ENCODER_MEASUREMENT_DT_MAX_US (15000u) // 大于等于15ms的窗口无效；标称10ms与疑似20ms漏服务的中点。
#define ENCODER_MEASUREMENT_JITTER_MIN_US (9500u) // 小于9.5ms记录抖动警告，但仍可按真实时间估速。
#define ENCODER_MEASUREMENT_JITTER_MAX_US (10500u) // 大于10.5ms记录抖动警告，工程目标仍须实测。
#define ENCODER_MEASUREMENT_WINDOW_SAMPLES (50u) // 最多缓存50个样本，支持标称100/500ms的10/50样本估速。
#define ENCODER_MEASUREMENT_CPR_MIN (1.0f) // 每输出车轮圈至少1个有效计数，防止极小N造成虚假或溢出RPM。
#define ENCODER_MEASUREMENT_CPR_MAX (10000000.0f) // 实测每轮圈计数上限10000000，超出本标定合同拒绝。
#define ENCODER_MEASUREMENT_CIRCUMFERENCE_MIN_M (0.001f) // 轮周长换算下限1mm，排除近零参数造成反换算溢出。
#define ENCODER_MEASUREMENT_CIRCUMFERENCE_MAX_M (10.0f) // 轮周长换算上限10m；单位m，不能把毫米值直接传入。

typedef enum
{
    ENCODER_WHEEL_LEFT = 0, // 左驱动轮，使用左轮每圈有效计数和周长。
    ENCODER_WHEEL_RIGHT // 右驱动轮；两轮比例分别标定，不能混用左右N。
} EncoderWheel;

typedef struct
{
    uint32_t sample_id; // 每次采样推进的序号，首个基线样本为1，允许无符号回绕。
    uint32_t now_us; // 最近采样读取的GPT1低32位微秒时间，允许约71.58分钟回绕。
    uint16_t raw_left; // 左轮最近读取的持续硬件16位计数，未做方向校正。
    uint16_t raw_right; // 右轮最近读取的持续硬件16位计数，未做方向校正。
    uint32_t dt_us; // 两次采样时间的无符号差值，单位us；首样本为0。
    uint64_t total_time_us; // 建立基线后累加的真实时间，单位us，包含异常间隔以便追溯。
    int16_t raw_delta_left; // 左轮模65536差分，单位counts；歧义差分时置0，须查counter_valid。
    int16_t raw_delta_right; // 右轮模65536差分，单位counts，未校正接线方向。
    int16_t delta_left; // 左轮方向校正后的安全增量，单位counts/当前窗口；前进应为正。
    int16_t delta_right; // 右轮方向校正后的安全增量；10ms控制调用者仍使用原单位合同。
    int64_t total_left; // 左轮已可解码窗口的净计数累计；异常时窗可能有隐藏整圈，须结合error_count。
    int64_t total_right; // 右轮已可解码窗口的净计数累计；不对各增量取绝对值。
    float rpm_left_100; // 左输出车轮约100ms窗口RPM；无效时置0且rpm_valid_100为0。
    float rpm_right_100; // 右输出车轮约100ms窗口RPM，N使用实测每轮有效counts/rev。
    float rpm_left_500; // 左输出车轮约500ms窗口RPM，不是电机轴RPM。
    float rpm_right_500; // 右输出车轮约500ms窗口RPM，反转为负。
    uint32_t window_us_100; // 最近10个有效样本的真实时长，单位us；未填满时为0。
    uint32_t window_us_500; // 最近50个有效样本的真实时长，单位us；未填满时为0。
    uint8_t rpm_valid_100; // 10样本已填满且左右N均有效为1；0时不能把RPM占位0解释为静止。
    uint8_t rpm_valid_500; // 50样本已填满且左右N均有效为1，否则0。
    uint8_t valid; // 本次计数和时窗均有效为1；首样本及异常样本为0。
    uint8_t counter_valid; // 方向配置正确且两轮模差分均可解码为1；首基线为0。
    uint8_t timing_valid; // 真实dt严格位于5至15ms之间为1；首样本为0。
    uint8_t jitter_warning; // dt在9.5至10.5ms之外为1；不等同必然漏周期。
    uint32_t error_count; // 累计异常样本数，饱和而非回绕；非0时整组标定不能直接验收。
    uint32_t stationary_us; // 两轮连续严格零增量且有效的真实时间，单位us，饱和至UINT32_MAX。
    uint32_t calibration_version; // counts/rev参数版本，初始化1，每次成功更新递增。
    float counts_per_rev_left; // 左输出车轮一圈的实测有效计数，0为未标定。
    float counts_per_rev_right; // 右输出车轮一圈的实测有效计数，0为未标定；不再额外乘减速比。
    float circumference_left_m; // 左轮换算周长，单位m；名义轮径推算与实测滚动周长须在来源中区分。
    float circumference_right_m; // 右轮换算周长，单位m；0表示线速度换算尚不可用。
    float speed_left_mps; // 左轮本次有效计数/真实dt换算的轮周速度，单位m/s，前进为正。
    float speed_right_mps; // 右轮本次轮周速度，单位m/s；不等同打滑情况下的车体地速。
    float speed_left_mps_100; // 左轮最近10个有效样本的轮周速度，单位m/s，实际时长见window_us_100。
    float speed_right_mps_100; // 右轮最近10样本速度，单位m/s，未填满时为无效占位0。
    float speed_left_mps_500; // 左轮最近50样本速度，单位m/s，使用真实窗口总时长。
    float speed_right_mps_500; // 右轮最近50样本速度，单位m/s，反转为负。
    uint8_t speed_valid; // 本次计数/时窗及左右N/周长均有效为1；0时速度占位0不表示静止。
    uint8_t speed_valid_100; // 10样本估速和两轮周长均可用为1；不代替原始采样有效标记。
    uint8_t speed_valid_500; // 50样本估速和两轮周长均可用为1；换算参数更新会撤销此标记。
} EncoderMeasurementSnapshot;

// 函数简介     安装快照与参数的短临界区回调
// 参数说明     enter/leave    保存并恢复原中断状态的配对回调，可同时为空
// 返回参数     void
// 备注信息     初始化时调用；leave不能一律开中断，宿主单线程测试可不安装
void encoder_measurement_set_critical_hooks(uint32_t (*enter)(void), void (*leave)(uint32_t));
// 函数简介     初始化持续计数采样器，不读写硬件计数寄存器
// 参数说明     sign_left/right    接线方向，只允许-1或+1
// 返回参数     void
// 备注信息     PIT启动前调用；未知counts/rev置0，不伪造RPM
void encoder_measurement_init(int8_t sign_left, int8_t sign_right);
// 函数简介     唯一PIT采样者提交两轮持续计数及真实时间
// 参数说明     raw_left/right    原始16位寄存器；now_us为同次GPT微秒时间
// 返回参数     uint8_t         基线或有效样本为1；方向、差分或时窗异常为0
// 备注信息     首样本valid为0；异常时重建前值并清RPM窗，生产集成可保留可解码delta的原控制合同
uint8_t encoder_measurement_sample(uint16_t raw_left, uint16_t raw_right, uint32_t now_us);
// 函数简介     一致复制测量快照，避免64位累计在ISR更新中被读裂
// 参数说明     snapshot        输出指针，可为空
// 返回参数     void
// 备注信息     主循环和调试入口读取，不得再次清硬件计数
void encoder_measurement_get_snapshot(EncoderMeasurementSnapshot *snapshot);
// 函数简介     原子更新左右输出车轮实测counts/rev并清旧RPM窗口
// 参数说明     left/right      有限计数，范围1至10000000；expected_version为读取时的版本
// 返回参数     uint8_t         成功1，非法或过期0
// 备注信息     停止和人工确认由调用者保证；不修改控制目标/PI单位，不清累计计数
uint8_t encoder_measurement_set_counts_per_rev(float left, float right, uint32_t expected_version);

// 函数简介     停止态原子更新两轮换算周长，并清除旧物理量窗口
// 参数说明     left_m/right_m    周长m；expected_version为快照中的calibration_version
// 返回参数     uint8_t         成功1，非法或版本过期0
// 备注信息     调用者保证停车；采用名义轮径时结果是轮周速度估计，不是已实测地速
uint8_t encoder_measurement_set_wheel_circumference(float left_m, float right_m, uint32_t expected_version);
// 函数简介     将指定车轮的方向校正净计数换算为轮周线速度
// 参数说明     wheel/counts/window_us/speed_mps    轮侧、净计数、实际窗口us、输出m/s
// 返回参数     uint8_t         成功1；未配置/非法/溢出0，失败不修改输出
// 备注信息     v=counts×周长×1000000/(N×window_us)；采样调用者须先检查计数和时间有效性
uint8_t encoder_measurement_counts_to_mps(EncoderWheel wheel, float counts, uint32_t window_us, float *speed_mps);
// 函数简介     将指定车轮的轮周线速度换算为采样窗口计数
// 参数说明     wheel/speed_mps/window_us/counts    轮侧、速度m/s、窗口us、输出计数
// 返回参数     uint8_t         成功1；未配置/非法/溢出0，失败不修改输出
// 备注信息     控制目标使用标称10000us，测量使用真实dt；此函数不改PI参数、不启动电机、不裁剪目标
uint8_t encoder_measurement_mps_to_counts(EncoderWheel wheel, float speed_mps, uint32_t window_us, float *counts);

#endif
