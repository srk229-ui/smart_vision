#ifndef WIFI_ASSISTANT_CONFIG_H
#define WIFI_ASSISTANT_CONFIG_H

/* WIFI-SPI 通信开关及热点信息；电脑端 2.4 GHz 热点设置为相同名称和密码 */
#define WIFI_ASSISTANT_ENABLE             (1) // 1编译启用助手适配；启用不等于允许执行器发车。
#define WIFI_ASSISTANT_SSID               "SEEKFREE" // 热点标识来自人工配置；注释和调参结果不输出凭据。
#define WIFI_ASSISTANT_PASSWORD           "SEEKFREE123" // 热点认证凭据；只供停止态建立热点连接使用。

/* 电脑与模块接入同一热点；目标 IP 为电脑当前 WLAN 的 IPv4 地址 */
#define WIFI_ASSISTANT_TARGET_IP          "10.138.155.234" // 电脑TCP Server/UDP接收端的当前IPv4地址，须与实际网络一致。

/* 通信协议：1-电脑作为 TCP Server  2-UDP；端口在 zf_device_wifi_spi.h 中设置 */
#define WIFI_ASSISTANT_TRANSPORT          (1) // 1为模块连接电脑TCP Server；2使用UDP。

// 这里通道号为V1界面0..7；线上包编号为1..8，协议解析层负责减1。
/* 助手界面通道5：0-仅接收参数  1-图像传输  2-虚拟示波器 */
#define WIFI_ASSISTANT_DEFAULT_STREAM     (1) // 仅决定停车默认显示流；通道5可以在停车修改。
/* 助手界面通道6：0-原始灰度图像  1-处理后的图像（仍以8位灰度格式传输） */
#define WIFI_ASSISTANT_DEFAULT_IMAGE      (0) // 0原始灰度；1处理图，仍按8位图像封装。
/* 助手界面通道7：发送周期，单位 ms；33 ms 对应配置上限约30帧/秒，实际帧率待测 */
#define WIFI_ASSISTANT_DEFAULT_PERIOD_MS  (33) // 停车发送间隔，单位ms；不改变相机采集或控制周期。
#define WIFI_ASSISTANT_DEFAULT_SERVO_GAIN (0.22f) // 兼容显示镜像初值，单位命令角/像素；控制实际用steer_kp。
/* 行车固定为最高十次每秒波形；不发送图像，不由此周期改变控制采样。 */
#define WIFI_ASSISTANT_RUNNING_PERIOD_MS  (100) // 运行发送间隔100ms，最高10Hz；不改变100Hz控制。

/* 停止态每块数据最多尝试次数；只有停止后才允许重建模块及网络连接。 */
#define WIFI_ASSISTANT_SEND_ATTEMPTS      (3) // 一段停车发送的最大尝试次数，含首尝试。
#define WIFI_ASSISTANT_RECONNECT_PERIOD_MS (2000) // 停车连接恢复等待间隔，单位逻辑ms。

// 配置错误在编译阶段拒绝，不在运行中默默裁剪。
#if WIFI_ASSISTANT_SEND_ATTEMPTS < 1 || WIFI_ASSISTANT_SEND_ATTEMPTS > 5
#error "WIFI_ASSISTANT_SEND_ATTEMPTS must be between 1 and 5"
#endif

// 停车恢复间隔限制在1..60秒；不是运行链路保护阈值。
#if WIFI_ASSISTANT_RECONNECT_PERIOD_MS < 1000 || WIFI_ASSISTANT_RECONNECT_PERIOD_MS > 60000
#error "WIFI_ASSISTANT_RECONNECT_PERIOD_MS must be between 1000 and 60000"
#endif

// 当前适配只支持这两个既有逐飞接口，不能把其它值当作TCP。
#if WIFI_ASSISTANT_TRANSPORT != 1 && WIFI_ASSISTANT_TRANSPORT != 2
#error "WIFI_ASSISTANT_TRANSPORT must be 1 (TCP) or 2 (UDP)"
#endif

// 默认流模式必须是0..2，实际运行时仍固定波形策略。
#if WIFI_ASSISTANT_DEFAULT_STREAM < 0 || WIFI_ASSISTANT_DEFAULT_STREAM > 2
#error "WIFI_ASSISTANT_DEFAULT_STREAM must be 0, 1 or 2"
#endif

// 只允许原始/处理图两种来源，图传协议均为8位灰度。
#if WIFI_ASSISTANT_DEFAULT_IMAGE != 0 && WIFI_ASSISTANT_DEFAULT_IMAGE != 1
#error "WIFI_ASSISTANT_DEFAULT_IMAGE must be 0 or 1"
#endif

// 停车发送周期的有效范围20..1000ms；不改PIT或相机硬件周期。
#if WIFI_ASSISTANT_DEFAULT_PERIOD_MS < 20 || WIFI_ASSISTANT_DEFAULT_PERIOD_MS > 1000
#error "WIFI_ASSISTANT_DEFAULT_PERIOD_MS must be between 20 and 1000"
#endif

#endif
