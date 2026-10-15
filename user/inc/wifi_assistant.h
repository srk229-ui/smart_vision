#ifndef WIFI_ASSISTANT_H
#define WIFI_ASSISTANT_H

#include "zf_common_typedef.h"
#include "wifi_assistant_config.h"

// 通信状态枚举仅描述模块/热点/socket/传输阶段，不替代车辆控制状态机。
typedef enum
{
    WIFI_ASSISTANT_DISABLED, // 未启用或尚未初始化通信。
    WIFI_ASSISTANT_INITIALIZING, // 正在检查模块并建立连接。
    WIFI_ASSISTANT_MODULE_ERROR, // 模块通信/版本获取失败。
    WIFI_ASSISTANT_LINK_ERROR, // 热点已连但socket连接失败。
    WIFI_ASSISTANT_READY, // V1通信就绪，不代表车辆可以运行。
    WIFI_ASSISTANT_SEND_ERROR, // SPI/解析容量或发送异常，停车后才能恢复连接。
    WIFI_ASSISTANT_WIFI_ERROR // 模块通信正常，但连接热点失败
} wifi_assistant_state_enum;

extern float wifi_assistant_servo_gain; // 兼容显示的舵机P镜像，控制层参数才是实际配置来源。
extern volatile wifi_assistant_state_enum wifi_assistant_state; // 主循环维护，供调试器/显示读取；volatile不提供通用线程互斥。
/* 调试统计：发送失败次数、最近未发送字节数、自动重连次数及完整图像帧数 */
extern volatile uint32 wifi_assistant_send_error_count; // 未完成发送尝试次数。
extern volatile uint32 wifi_assistant_last_unsent_bytes; // 最近未发送尾段字节数。
extern volatile uint32 wifi_assistant_reconnect_count; // 停止态重连尝试次数。
extern volatile uint32 wifi_assistant_frame_count; // 图像与边线全部发送成功的帧数。
/* 调试统计：非法、运行中或控制模块拒绝的人工参数请求次数。 */
extern volatile uint32 wifi_assistant_parameter_reject_count; // 保留通道请求拒绝计数，不是逐包事件数。

// 调用前由主循环保证车辆停止；失败只更新通信状态，不隐式发车。
/* 上电或停止态重连时调用，完成模块及逐飞助手接口初始化 */
void wifi_assistant_init(void);
// elapsed_ms取已服务的控制周期；时间为逻辑ms，不是独立硬件时间戳。
/* 在现有控制定时器中调用，仅更新时间，不执行 SPI 或网络操作 */
void wifi_assistant_tick(uint32 elapsed_ms);
// 一次poll固定运行身份，接收STOP/停车参数并发送快照；不接收远程启动，Wi-Fi异常不改变行车状态。
/* 主循环调用，处理参数接收及虚拟示波器发送 */
void wifi_assistant_poll(void);
/* 二值化处理前保存原始图像，供原始灰度图传使用 */
void wifi_assistant_capture_raw(const uint8 *image);
// 运行态直接拒绝图传；mid_error、servo_angle和threshold保留旧调用接口，不作为当前波形来源。
/* 主循环完成图像处理和舵机控制后，发送图像及三条边线 */
void wifi_assistant_on_frame(const uint8 *image, const uint8 *left,
                             const uint8 *middle, const uint8 *right,
                             float mid_error, float servo_angle, uint8 threshold);

/* 取出通道三的静止直道宽度标定请求，消费后清零。 */
uint8 wifi_assistant_take_width_calibration_request(void);

#endif
