#ifndef CALIBRATION_CONFIG_H
#define CALIBRATION_CONFIG_H // 标定构建配置防重复包含标记。

#ifndef CALIBRATION_DEBUG_ENABLE
#define CALIBRATION_DEBUG_ENABLE (0) // 正式版默认关闭内存命令和试验记录；诊断目标通过编译定义设为1。
#endif

#if CALIBRATION_DEBUG_ENABLE != 0 && CALIBRATION_DEBUG_ENABLE != 1
#error "CALIBRATION_DEBUG_ENABLE must be 0 or 1"
#endif

#define CALIBRATION_HEARTBEAT_US (100000u) // 电脑端建议每100ms提交新的会话心跳序号。
#define CALIBRATION_LINK_TIMEOUT_US (500000u) // 有输出试验500ms未收到新鲜心跳即停车，使用GPT真实时间。
#define CALIBRATION_RPM_MAX_US (8000000u) // 架空RPM单次最多8秒；手转与舵机由人工START/END界定。
#define CALIBRATION_STATIONARY_US (100000u) // 开始/结束手转计数要求两轮连续100ms严格零增量。

#endif
