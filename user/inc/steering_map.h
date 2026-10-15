#ifndef STEERING_MAP_H
#define STEERING_MAP_H // 舵机脉宽与实测左右前轮角映射接口防重复包含。
#include <stdint.h>
#define STEERING_MAP_MAX_POINTS (17u) // 每张表最多17个实测点，覆盖中位和两侧安全范围。
typedef enum
{
    STEERING_SIDE_LEFT = 0, // 左前轮，车辆左转定义为正角，单位deg。
    STEERING_SIDE_RIGHT // 右前轮，左右前轮分别测量，不能用舵机命令角代替。
} SteeringSide;
typedef struct
{
    float pulse_us; // 实测对应的舵机高电平脉宽，单位us，范围500至2500。
    float wheel_left_deg; // 左前轮相对直行中位的实测角，单位deg，左转为正。
    float wheel_right_deg; // 右前轮实测角，单位deg，同样左转为正。
} SteeringMapPoint;
typedef struct
{
    uint32_t version; // 运行表版本；初始化为1，成功原子应用递增，0表示未生成的基线常量。
    uint8_t calibrated; // 人工测量并确认合格为1；空表0不能输出物理转角。
    uint8_t point_count; // 有效点数3至17；未标定基线为0。
    float center_us; // 直行中位对应的实测脉宽，单位us，必须是双轮零角的实测节点。
    float pulse_min_us; // 人工确认的最小安全脉宽，必须等于首节点，单位us。
    float pulse_max_us; // 人工确认的最大安全脉宽，必须等于末节点，单位us。
    SteeringMapPoint points[STEERING_MAP_MAX_POINTS]; // 按pulse_us严格升序排列；两轮角分别严格单调且方向一致。
} SteeringMap;

typedef struct
{
    float pulse_us; // 操作者测角时的请求/标称脉宽us；实际引脚脉宽须单独用仪器确认。
    float wheel_deg; // 指定前轮相对车身纵轴的角度deg，保留原测量符号和直行偏置。
} SteeringWheelMapPoint;
typedef struct
{
    SteeringSide side; // 本表实测轮侧；只测右轮时不能据此返回左轮或整车等效转角。
    uint8_t point_count; // 单轮实测节点数2至17，0表示没有单轮映射。
    SteeringWheelMapPoint points[STEERING_MAP_MAX_POINTS]; // 脉宽严格递增、角度严格单调；不要求已实测零角节点。
} SteeringWheelMap;

// 函数简介     验证单轮静态插值表
// 参数说明     map             单轮测点与轮侧
// 返回参数     uint8_t         可作区间内插值为1，否则0
// 备注信息     不代表双轮机械标定验收，不把插值零角冒充实测中位
uint8_t steering_wheel_map_validate(const SteeringWheelMap *map);
// 函数简介     分段线性查询单轮脉宽对应的实际角度估计
// 参数说明     map/pulse_us/angle_deg    单轮表、输入us、输出deg
// 返回参数     uint8_t         成功1，非法或越界0且不写输出
// 备注信息     只在首末实测节点之间插值，不外推，不触发舵机输出
uint8_t steering_wheel_map_us_to_angle(const SteeringWheelMap *map, float pulse_us, float *angle_deg);
// 函数简介     分段反插值查询单轮角度对应的脉宽
// 参数说明     map/angle_deg/pulse_us    单轮表、输入deg、输出us
// 返回参数     uint8_t         成功1，非法或越界0且不写输出
// 备注信息     零角查询是插值估计；不能自动替代双前轮直行中位或机械安全范围
uint8_t steering_wheel_map_angle_to_us(const SteeringWheelMap *map, float angle_deg, float *pulse_us);
// 函数简介     使用端段斜率有界外推单轮脉宽对应的角度估算
// 参数说明     map/pulse_us    原始单轮实测表与输入名义脉宽us
// 参数说明     angle_min_deg/angle_max_deg/angle_deg    原始角允许闭区间deg及输出deg
// 返回参数     uint8_t         合法且在角域/500至2500us内为1，否则0且不写输出
// 备注信息     区间内插值、区间外延续首末段；仅容忍1e-4deg浮点端点误差，不代表机械实测
uint8_t steering_wheel_map_us_to_angle_extrapolate(const SteeringWheelMap *map, float pulse_us,
    float angle_min_deg, float angle_max_deg, float *angle_deg);
// 函数简介     使用同一端段斜率反解有界外推角的名义脉宽
// 参数说明     map/angle_deg   原始单轮表及相对车身纵轴的请求角deg
// 参数说明     angle_min_deg/angle_max_deg/pulse_us    允许角闭区间deg及输出us
// 返回参数     uint8_t         成功1，非法或超角域/500至2500us为0且不写输出
// 备注信息     不修改实测点、不量化PWM；调用控制器仍须检查量化后脉宽的估算角
uint8_t steering_wheel_map_angle_to_us_extrapolate(const SteeringWheelMap *map, float angle_deg,
    float angle_min_deg, float angle_max_deg, float *pulse_us);
// 函数简介     验证实测表的范围、中位和严格单调性
// 参数说明     map             候选表，只读
// 返回参数     uint8_t         合法1，否则0
// 备注信息     只验证软件合同，不代替机械限位及迟滞现场复核
uint8_t steering_map_validate(const SteeringMap *map);
// 函数简介     分段线性计算脉宽对应的两前轮角
// 参数说明     map/pulse_us/left_deg/right_deg    表、输入us及两个输出deg
// 返回参数     uint8_t         成功1，未标定/越界/非法0且不写输出
// 备注信息     输出为表推算角，仍不是实时转角传感器反馈
uint8_t steering_map_us_to_angles(const SteeringMap *map, float pulse_us, float *left_deg, float *right_deg);
// 函数简介     反插值指定前轮角对应的安全舵机脉宽
// 参数说明     map/side/angle_deg/pulse_us    表、LEFT或RIGHT、输入deg、输出us
// 返回参数     uint8_t         成功1，越界或非法0且不写输出
// 备注信息     两轮角分别反解，不夹紧越界请求，不将此函数直接接入当前循迹控制
uint8_t steering_map_angle_to_us(const SteeringMap *map, SteeringSide side, float angle_deg, float *pulse_us);
// 函数简介     安装运行表读取和应用的短临界区回调
// 参数说明     enter/leave    保存和恢复中断状态的配对函数
// 返回参数     void
// 备注信息     初始化时安装，宿主单线程测试可为空
void steering_map_set_critical_hooks(uint32_t (*enter)(void), void (*leave)(uint32_t));
// 函数简介     初始化未标定运行表和版本
// 参数说明     void
// 返回参数     void
// 备注信息     上电初始化，不构造假实测数据
void steering_map_init(void);
// 函数简介     一致读取运行表
// 参数说明     map             输出指针，可为空
// 返回参数     void
// 备注信息     大表只在停止态读取和导出，不在控制ISR反复复制
void steering_map_get(SteeringMap *map);
// 函数简介     原子应用已验证的实测表
// 参数说明     map/expected_version    候选表及读取时的运行版本
// 返回参数     uint8_t         成功1；非法或过期0
// 备注信息     调用者保证停车和人工确认，版本由本模块更新，不接管控制器输出
uint8_t steering_map_apply(const SteeringMap *map, uint32_t expected_version);
#endif
