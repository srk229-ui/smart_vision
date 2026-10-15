## 停车显示与图像调试

### TFT显示的实际调用与输入

当前main只在成功处理新帧后刷新运行信息，且必须满足snapshot.state!=RUNNING、距离last_display_ms至少200逻辑ms。main调用tft180_displayimage03x(image,125,100)，该宏展开为tft180_show_gray_image(0,0,image,188,120,125,100,0)：源仍是188×120二值快照，只把显示区域缩放为125×100，不改变视觉算法输入尺寸。

随后tft180_show_int分别显示左右本次编码器读数，tft180_show_float显示视觉可信度，另显示故障枚举。读数和显示基于主循环刚取得的ControlSnapshot；例如有效目标与PWM来自最近PIT，视觉可信度可能来自更晚的发布。这是调试显示，不是一个新的控制器，也不测量真实PWM引脚。

无成功新帧时不会进入这段图像显示分支。停车时camera_frame_take反复失败，画面可能保持旧内容；是否满足新鲜度应看保护与故障，不以“TFT还显示赛道”推断有新采集。通信状态show_state单独只在停车路径刷新，不能和车辆READY/RUNNING/FAULT混作一个状态。

### 原灰度、二值图和边线各自从哪里来

| 图像/信号 | 创建与消费顺序 | 读图含义 |
| --- | --- | --- |
| 原灰度副本 | main在二值化前调用wifi_assistant_capture_raw；符合停车图传条件时复制 | 原图可用于判断曝光/阴影及Otsu问题；未复制时不能拿二值图冒充灰度 |
| 二值算法图 | binaryzation_process原地改写main.image，再vision_process只读 | TFT当前显示的是这个处理图；阈值和纯色处理见视觉章 |
| 左/中/右线 | VisionResult按行给出，wifi_assistant_on_frame复制到独立缓存 | 边线不画回二值算法图；row_quality=0的坐标只是占位，V1几何线条本身不证明有效 |
| 波形目标/反馈/PWM/转向 | wifi_assistant_poll从一致控制快照填8通道 | 运行配置10Hz发送，无法据此复原每个100Hz控制样本 |

行驶关闭TFT图像刷新、原图保存和完整图传，把主循环时间优先留给新帧与有界通信。停车图传的缓存、一次性有效标记和发送故障详见[WIFI功能章](#guide-wifi)；这里的屏幕显示周期不是相机采集周期，不能用刷屏帧率标定相机50FPS。

### 本章代码索引

| 调用入口 | 阅读目的 |
| --- | --- |
| {{src|user/src/main.c|main}} | 看snapshot门控、200ms限频、显示先后和图像缓存使用 |
| {{src|../libraries/zf_device/zf_device_tft180.c|tft180_show_gray_image}} | tft180_displayimage03x宏的实际显示实现；宏见[zf_device_tft180.h](../../libraries/zf_device/zf_device_tft180.h#L146) |
| {{src|../libraries/zf_device/zf_device_tft180.c|tft180_show_int}}、{{src|../libraries/zf_device/zf_device_tft180.c|tft180_show_float}} | 编码器、可信度与故障数值显示 |
| {{src|user/src/wifi_assistant.c|show_state}} | 通信状态的停车显示，独立于车辆状态机 |
| {{src|user/src/wifi_assistant.c|wifi_assistant_capture_raw}}、{{src|user/src/wifi_assistant.c|wifi_assistant_on_frame}}、{{src|user/src/wifi_assistant.c|wifi_assistant_poll}} | 原灰度、二值边线图传及快照波形 |
