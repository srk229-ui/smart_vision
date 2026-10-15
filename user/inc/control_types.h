#ifndef CONTROL_TYPES_H
#define CONTROL_TYPES_H // 公共控制类型头文件防重复包含标记。

#include <stdint.h>

#define CONTROL_TARGET_COUNTS_MAX (400.0f) // 双轮基础前进目标共用上限，单位counts/10ms；覆盖本轮0～3m/s换算范围，实际可达性待测。
#define CONTROL_DEFAULT_SPEED_MPS (1.2f) // 本地启动默认基础目标1.2m/s，按左右各自N和现有周长换算；不是已实测车体地速。

#define CONTROL_PERIOD_MS          (10u) // PIT控制任务的标称周期，单位ms；当前固定10 ms，改变时须同步采样单位与计时约定。
#define CONTROL_IMAGE_WIDTH        (188u) // 灰度/二值图每行像素数188，横向列坐标0至187；与相机配置一致。
#define CONTROL_IMAGE_HEIGHT       (120u) // 灰度/二值图行数120，纵向行坐标0至119；行号由图像顶部向底部递增。
#define CONTROL_BAND_COUNT         (5u) // 参与误差计算的控制行数量5；控制行数组与权重数组逐项对应。
#define CONTROL_BENCH_MAX_MS        (8000u) // 人工架空轮试验的最长逻辑运行时间8000 ms；到期停止，不能作为循迹试验时限。

typedef enum
{
    CONTROL_INIT = 0, // 初始化状态标识；当前control_init在返回前直接进入READY或FAULT，不对外等待于此状态。
    CONTROL_READY, // 停止且可接受人工准备/启动请求；电机输出为0，本地TRACK要求有效新帧及停机周期，诊断BENCH另需确认。
    CONTROL_RUNNING, // 已通过人工启动条件，按TRACK或BENCH执行控制；不表示目标速度已达到。
    CONTROL_FAULT // 故障锁存且输出清零；故障消失不会自动恢复；本地KEY1可清视觉超时故障并重试，其余故障须人工处理。
} ControlState;

typedef enum
{
    CONTROL_MODE_TRACK = 0, // 摄像头循迹模式；启动需最新有效帧，运行按最后有效采集图像统一500ms超时停车。
    CONTROL_MODE_BENCH, // 人工架空轮速度试验模式；不依赖视觉，最长8秒，允许的舵机仅保持中位。
    CONTROL_MODE_SERVO_CAL // 诊断版人工舵机单点模式；两电机始终为零，不依赖视觉或正轮速目标。
} ControlMode;

typedef enum
{
    CONTROL_FAULT_NONE = 0, // 没有锁存故障；仍须结合state判断当前能否启动。
    CONTROL_FAULT_VISION_TIMEOUT, // 最近有效图像的采集年龄超过500逻辑ms，或运行中没有有效图像。
    CONTROL_FAULT_TRACK_LOST, // 保留旧故障枚举值兼容外部接口；当前不再按丢线帧数触发。
    CONTROL_FAULT_LINK, // 诊断会话失联等显式链路故障；普通TRACK不因Wi-Fi异常停车。
    CONTROL_FAULT_INVALID_VALUE // 参数、控制带或计算结果非法，控制接口无法保持有效约定。
} ControlFault;

typedef struct
{
    uint32_t frame_id; // CSI完成帧序号，uint32回绕；重复/回退帧不作为新视觉事件。
    uint32_t capture_time_ms; // 本帧采集完成时的设备逻辑时间，单位ms、10 ms量化；由CSI回调记录，不是处理完成时间。
    float error_px; // 控制带质量加权误差，单位像素；94减加权中点，赛道中点在画面左侧为正；仅valid=1可用。
    float confidence; // 控制带有效质量权重占全部权重的比例0至1；双边质量1，单边或骨架路径重建0.5，无效行0。
    uint8_t threshold; // 本帧Otsu灰度阈值0至255，由调用方填写；边线搜索依据二值0/255，不用此值判黑。
    uint8_t valid; // 本帧整体视觉结果可用为1，否则0；需至少3个控制行参与且confidence不低于0.35。
    uint8_t track_lost; // 本帧丢线为1，否则0；表示整体结果失效，不要求整幅图每一行都无边线。
    uint8_t left[CONTROL_IMAGE_HEIGHT]; // 按行索引的左边界列坐标0至187；重建坐标会限幅，row_quality=0时为占位，不能当作实测边。
    uint8_t middle[CONTROL_IMAGE_HEIGHT]; // 按行索引的中线列坐标0至187；无效行默认94，使用前须检查对应row_quality。
    uint8_t right[CONTROL_IMAGE_HEIGHT]; // 按行索引的右边界列坐标0至187；重建坐标会限幅，无效行默认187，不能当作实测边。
    uint8_t row_quality[CONTROL_IMAGE_HEIGHT]; // 每行质量：0无效、1单边或当前图像骨架路径重建、2左右边均实测且中点一致；用于误差与可信度加权。
    uint16_t valid_rows; // 整幅120行中row_quality为1或2的行数0至120；不是五个控制行的有效数量。
    uint16_t reconstructed_rows; // 整幅图中row_quality为1的重建行数；属于valid_rows的子集。
} VisionResult;

typedef struct
{
    float target_left; // 左轮基础前进目标，单位counts/10ms，范围0至CONTROL_TARGET_COUNTS_MAX（当前400）；实际送PI前还会降速、差速分配和执行斜坡。
    float target_right; // 右轮基础前进目标，单位counts/10ms，范围0至CONTROL_TARGET_COUNTS_MAX（当前400）；实际送PI前还会降速、差速分配和执行斜坡。
    float kp_left; // 左轮速度比例增益，单位PWM计数/(counts/10ms)，范围0至1000；乘目标减实测的速度误差。
    float ki_left; // 左轮速度积分增益，单位PWM计数/[(counts/10ms)·s]，范围0至1000；积分按秒，0清历史。
    float kp_right; // 右轮速度比例增益，单位PWM计数/(counts/10ms)，范围0至1000；右轮拥有独立PI状态。
    float ki_right; // 右轮速度积分增益，单位PWM计数/[(counts/10ms)·s]，范围0至1000；右轮独立按秒积分。
    float pwm_max; // 双轮实际PWM计数上限，必须为0至5000的整数值；不是百分比，实际占空比由PWM_DUTY_MAX换算。
    float integral_max; // 每轮积分输出贡献的绝对上限，单位PWM计数，范围0至pwm_max；限制的是Ki积分后的贡献。
    float accel_counts_per_s; // 基础目标斜坡变化率，单位(counts/10ms)/s，范围1至1000；TRACK差速另按4/s渐变，BENCH仍用于每轮有效目标；STOP清零。
    float servo_center; // 人工待标定的舵机直行命令角，单位命令度，须在servo_min至servo_max内；不等于实际前轮转角。
    float servo_min; // 舵机允许的最小命令角，单位命令度、范围0至180且小于servo_max；不固定代表左打方向。
    float servo_max; // 舵机允许的最大命令角，单位命令度、范围0至180；须与机械安全极限一致。
    float servo_direction; // 视觉误差到舵机命令的方向符号，只允许-1或+1；命令为中位减此符号乘P/PD修正量。
    float steer_kp; // 转向比例增益，单位命令度/像素，范围0至5；作用于有效新帧的加权像素误差。
    float steer_kd; // 转向微分增益，单位命令度·s/像素，范围0至5；按新帧采集时间间隔计算误差变化率。
    float servo_rate_deg_s; // 舵机命令变化率上限，单位命令度/s，范围1至360；PIT每次按此值乘周期秒数限速。
    float curve_speed_factor; // 大像素误差时的基础目标缩放因子，范围0.1至1；误差绝对值达30像素时取此值。
    uint8_t look_rows[CONTROL_BAND_COUNT]; // 五个控制行索引0至119，必须严格递增；默认80/85/90/95/100，逐项对应look_weights。
    float look_weights[CONTROL_BAND_COUNT]; // 五个控制行的正权重，每项范围0.001至100；默认1/2/4/2/1，视觉内部再归一化并乘行质量。
    uint16_t vision_timeout_ms; // 有效图像采集年龄上限，单位逻辑ms；当前参数校验仅接受500，超过才停车。
    uint8_t lost_frame_limit; // 旧丢线帧数兼容占位；固定3保持字段布局，当前控制不使用此值停车。
    uint8_t calibration_confirmed; // 诊断/兼容人工启动的确认位，0未确认/1已确认；本地按键TRACK不依赖此字段。
} ControlParams;

typedef struct
{
    uint32_t sample_id; // 已执行control_tick的序号，每次增加1并允许uint32回绕；停机屏障至少要求序号前进一次。
    uint32_t timestamp_ms; // 最近一次控制任务的设备逻辑时间，单位ms，每次累加10；不是独立硬件时钟测得的真实时间。
    uint32_t frame_id; // 控制层最近接受的不同视觉帧序号，可能属于失效帧；有效图像年龄另看vision_age_ms。
    uint32_t params_version; // 已应用参数版本，初始化为1、每次成功整组应用加1；用于拒绝读取旧版本后的写请求。
    uint32_t params_applied_ms; // 最近成功应用参数时的设备逻辑时间，单位ms；初始化为0，不是快照读取时间。
    float target_left; // 本控制周期送入左轮PI的有效目标，单位counts/10ms；TRACK由基础斜坡乘平滑差速倍率生成，停止/故障时为0。
    float target_right; // 本控制周期送入右轮PI的有效目标，单位counts/10ms；与配置中的基础目标区分。
    int16_t measured_left; // 左编码器在本周期的带方向增量，单位counts/10ms；由ENCODER_1_SIGN校正，前进应为正。
    int16_t measured_right; // 右编码器在本周期的带方向增量，单位counts/10ms；由ENCODER_2_SIGN校正，前进应为正。
    float pwm_left; // 左轮本次计算的实际PWM计数，限幅后按uint32量化再存为float；ISR按此值写硬件；STOP立即清软件值，下次PIT向PWM寄存器写零；引脚生效时刻仍须实测。
    float pwm_right; // 右轮本次计算的实际PWM计数，范围0至pwm_max且为整数值；不代表已测得硬件实际波形。
    float raw_pwm_left; // 左轮条件积分计算后的限幅前浮点PI输出，单位PWM计数；可为负或高于上限，仅用于诊断。
    float raw_pwm_right; // 右轮条件积分计算后的限幅前浮点PI输出，单位PWM计数；不能直接作为硬件占空比。
    float integral_left; // 左轮当前积分输出贡献，单位PWM计数、范围±integral_max；停止、故障或应用参数时清零。
    float integral_right; // 右轮当前积分输出贡献，单位PWM计数；与左轮独立，不是累计误差或原始编码器计数。
    float vision_error_px; // 最近可用帧经近远前瞻融合的像素误差；前瞻失效/过期退回近带，符号同VisionResult.error_px；须结合可信度/年龄。
    float confidence; // 最近发布新帧被控制层判定可用时的可信度0至1，否则0；随时间变旧时另由vision_age_ms判定。
    float servo_cmd; // 当前经配置的servo_min/servo_max限位和变化率限制的舵机命令角，单位命令度；停止时可保持中位值但不输出PWM。
    uint32_t vision_age_ms; // 最近可用帧采集完成到当前控制tick的逻辑年龄，单位ms；尚无可用帧时为UINT32_MAX。
    ControlState state; // 当前控制状态INIT/READY/RUNNING/FAULT；真实运行许可和故障恢复以状态机为准。
    ControlMode mode; // 当前或上一次获准启动的TRACK/BENCH/SERVO_CAL模式；READY时此字段不表示已经发车。
    ControlFault fault; // 已锁存故障原因，NONE表示无故障；人工清故障后变NONE，网络恢复不自动清除。
    uint8_t threshold; // 最近接受帧的Otsu阈值0至255，包括失效帧；仅供诊断，不决定速度PI的有效性。
    uint8_t saturated_left; // 左轮输出受边界约束标记：raw小于0，或pwm_max大于0且实际PWM达到上限时为1。
    uint8_t saturated_right; // 右轮同规则的输出边界标记；恰好触顶也置1，不表示已经发生机械堵转。
    float servo_pulse_us_requested; // 本周期请求脉宽，单位us；关闭输出时为0，不代表引脚实测。
    uint32_t servo_duty_cmd; // 本周期写入PWM的整数占空计数；与PWM_DUTY_MAX同一刻度。
    float servo_pulse_us_nominal; // 由整数占空计数反算的名义脉宽us；不是示波器实测值。
    uint8_t servo_enabled; // 本周期是否允许ISR输出舵机PWM，0停脉冲/1按servo_cmd输出；仍须结合RUNNING和编译许可。
} ControlSnapshot;

#endif
