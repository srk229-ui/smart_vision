## 总体架构与数据流

当前工程是RT1064裸机程序。完整循迹链为：**相机采集 → 一致灰度快照 → Otsu二值化 → 逐行白段与质量 → 五行加权误差 → 新帧P/PD转向目标 → PIT限速与PWM；双轮编码器 → 独立速度PI → 电机PWM → 轮速反馈。** 视觉观察车辆相对赛道的位置，编码器闭合左右轮速度；软件没有前轮转角传感器，因此servo_cmd是命令刻度，实际轮角和R50能力需机械标定。

### 与参考解析的对应关系

本报告借鉴[参考工程的完整闭环代码解析](../../20th_smart_vision/docs/完整闭环代码解析.md)的阅读层次：先看调用入口和数据流，再看关键函数、公式、反馈、失败分支及验证边界。参考车为三轮全向底盘；当前车为双驱动轮加转向舵机，不能按参考的轮速逆解算解释本车。

| 功能层 | 当前实际实现 | 阅读时应区分 |
| --- | --- | --- |
| 调度 | main、CSI、PIT_CH0三种入口 | 主循环按新帧执行；PIT配置10ms；相机和PWM各自运行 |
| 普通赛道视觉 | 最近白段、逐行中点、质量加权控制带 | 使用188×120图像，未启用八邻域、最长白列或特殊元素状态机 |
| 转向 | control_publish_vision计算P/PD目标；control_tick限制命令变化率 | 图像误差单位像素，输出是舵机命令度，无IMU角速度阻尼 |
| 驱动 | 左右独立基础目标与PI（当前V1共用目标） | 无三轮运动学、无电流PI；左右反馈不能混用 |
| 操作与调试 | V1人工参数、10Hz波形、停车图传/标定 | 当前不是M4的实验协议、电脑桥或完整100Hz日志 |

下图保留总体连接关系，具体调用顺序见下方三条链和各功能章节。

{{ARCHITECTURE_DIAGRAM}}

### 实际调用链：入口与交接点

```text
采集事件：
CSI_IRQHandler()
  → CSI_DriverIRQHandler() → CSI_TransferHandleIRQ()
  → frame_callback() → 原mt9v03x_finished_callback()
  → 完成图指针 + capture_time_ms + frame_sequence

主循环每次迭代：
control_get_params() → 必要时vision_set_control_band()
  → 领取人工宽度请求 → camera_frame_take()
  → [有新快照] capture_raw → histogram → Otsu → binary → vision_process
  → control_publish_vision() → control_get_snapshot()
  → [有人工请求] calibrate_width()
  → [停止且到期] TFT显示 → wifi_assistant_on_frame()
  → [超时] 取消宽度请求 → wifi_assistant_poll()

PIT_CH0每次被服务：
encoder_get_count/clear_count(左、右) → control_tick()
  → 保护/停机 → 降速目标 → 目标斜坡 → 两个PidLocCtrl()
  → 舵机变化率限制 → control_get_snapshot()
  → 前进DIR → 左右pwm_set_duty() → 舵机pwm_set_duty()
  → wifi_assistant_tick(10) → 清PIT事件位
```

### 主要数据由谁写、由谁读

| 数据/状态 | 写入位置 | 消费位置 | 含义及所有权 |
| --- | --- | --- | --- |
| mt9v03x_image | 原相机完成回调 | 包装回调记录地址，main复制 | 当前完成缓冲指针，后续可被采集复用 |
| frame_sequence/captured_ms/completed_image | camera_frame的CSI包装回调 | camera_frame_take短锁领取 | 三项对应同一个发布事件；时间来自设备逻辑时钟 |
| main.image / hist_gram | camera_frame_take、灰度处理函数 | vision_process；停车图传先复制 | main自有快照22560字节，原地灰度→二值，不在DMA原图上计算 |
| VisionResult | vision_process；main补threshold | control_publish_vision、宽度标定、边线图传 | 每行证据和整体置信度；无效占位中点不能驱动转向 |
| config（ControlParams） | control_init / 停止态control_apply_params | control_tick、主循环配置视觉控制带 | 基础目标与保护配置，不被斜坡结果反写 |
| steering_target | 有效RUNNING/TRACK新帧发布 | control_tick | 已按配置限位的转向目标，和限速后servo_cmd不同 |
| speed_pid_l/r | 控制层初始化/复位与各轮PidLocCtrl | 控制快照、下一周期PI | 两轮历史独立，I项保存PWM输出贡献 |
| current（ControlSnapshot） | control_tick及控制操作/视觉发布 | PIT硬件写出、main、WIFI | 原子复制可保证字段读取一致，但不是所有字段都在同一物理时刻产生 |

### 本章代码索引

| 阅读顺序 | 实际入口 | 阅读目的 |
| --- | --- | --- |
| 1 | {{src|user/src/main.c|main}}、{{src|user/src/main.c|Init}} | 先找到初始化和主循环，观察main如何交接各模块 |
| 2 | {{src|user/src/isr.c|CSI_IRQHandler}}、{{src|user/src/isr.c|PIT_IRQHandler}} | 区分采集事件与固定控制入口 |
| 3 | {{src|user/src/camera_frame.c|camera_frame_take}}、{{src|user/src/img_process.c|vision_process}} | 追到图像与视觉结果的生成 |
| 4 | {{src|user/src/control.c|control_publish_vision}}、{{src|user/src/control.c|control_tick}} | 追到视觉外环和双轮速度内环 |
| 5 | {{src|user/src/wifi_assistant.c|wifi_assistant_poll}} | 最后看人工请求和调试数据如何进入/离开车辆 |

接口结构定义见[control_types.h](../user/inc/control_types.h#L1)。每个功能章下面都有就近函数索引；末尾全局查找表汇总这些入口。

## 工程接入与可恢复基线

### 哪些目录决定当前运行代码

| 目录/文件 | 实际职责 | 核对方式 |
| --- | --- | --- |
| [user/src](../user/src)、[user/inc](../user/inc) | 主循环、中断、控制、相机快照、视觉、PI与WIFI应用 | 从main/isr追真实调用；头文件固定单位与硬件宏 |
| [drivers/wifi_spi_v2](../drivers/wifi_spi_v2) | 本地WIFI6B21-SPI V2驱动 | uvprojx唯一登记本地C源，包含目录优先于根库旧头 |
| [mdk/rt1064.uvprojx](../mdk/rt1064.uvprojx) | 文件组、器件、编译与链接目标 | 当前目标nor_sdram_zf_dtcm；不使用已经清理的临时工程 |
| [mdk/scf](../mdk/scf) | Flash/SDRAM/DTCM链接布局 | 最终map检查真实段和资源；不能仅凭目录名推断变量落位 |
| [同级libraries](../../libraries) | 逐飞外设、MT9V03X、组件、NXP SDK及启动 | demo必须与当前同级库配套；参考工程自己的库不参与当前编译 |
| [tools](../tools)与[evidence/M2](evidence/M2) | 宿主验证、图像回放、索引生成和最终证据 | 工具验证的是同一源文件的软件路径，不能替代实机 |

本轮工程恢复登记了control.c、camera_frame.c与本地V2；旧WIFI同名C不重复编译。本地V2应用保持“检查模块 → 单独连热点 → 建Socket”的步骤。用户昨天成功的临时Keil工程已清理，不能将那次实测直接归给当前AXF。

共享库的seekfree_assistant.c/.h本次包含接收复位、合法包observer、剩余容量读取和异常计数。复制demo到另一位置时须同时保留这套同级libraries；只带应用目录会缺失本次通信保护改动。uvoptx与M0用户原件逐字节相同，其他用户配置保留。

### 怎样恢复及核对同一版本

实施前原件保存在[evidence/M0/baseline_20261012](evidence/M0/baseline_20261012)。最终[源清单](evidence/M2/source_manifest.json)登记22项源码/工程哈希；[固件](evidence/M2/firmware.axf)、[map](evidence/M2/firmware.map)、[完整构建日志](evidence/M0/keil_rebuild.log)相互核对。R007/R009前版本分别保存在revisions目录，恢复时应按同一清单取成套源码/公共库与工程配置，再完整重编译，不能混搭旧AXF与新参数说明。

程序调用了flash_init，但当前ControlParams与reference_width仍只在RAM中维护。没有从Flash装载/保存控制参数的业务链，也没有M4的恢复基线网络接口；掉电后需重新人工确认和宽度标定。

### 本章代码索引

| 关键入口 | 作用 |
| --- | --- |
| {{src|user/src/main.c|Init}} | 看注册后的模块是否实际初始化 |
| {{src|user/src/wifi_assistant.c|wifi_assistant_init}}、{{src|drivers/wifi_spi_v2/zf_device_wifi_spi.c|wifi_spi_init}} | 区分应用接入与底层模块配置 |
| {{src|../libraries/zf_components/seekfree_assistant.c|seekfree_assistant_receive_reset}} | 共享库必须随当前版本交付的接收改动 |

## 初始化、主循环与反馈采样

### 上电按什么顺序建立运行基础

main先调用clock_init(SYSTEM_CLOCK_600M)、debug_init，等待300ms后进入Init。这里的600MHz与等待值为代码配置；整车响应与真实周期不能由主频推断。

| Init步骤 | 关键调用与参数 | 建立的状态/硬件条件 |
| --- | --- | --- |
| 1. 先零输出 | 双电机pwm_init(...,17000,0)、方向GPIO；舵机pwm_init(...,50,0) | 硬件PWM先以0命令初始化；舵机此时也不输出中位脉宽 |
| 2. 安装同步 | control_set_critical_hooks(interrupt_global_disable,interrupt_global_enable) | 主循环控制操作能保存/恢复PRIMASK，避免PIT观察到半更新结构 |
| 3. 控制初始化 | control_init(92.5,85,100,SERVO_DIR)，再设置编译许可 | 参数目标为0，配置有效即READY；许可不是RUN请求 |
| 4. 采样与调度 | 两次encoder_quad_init，再pit_ms_init(PIT_CH0,10) | 此时反馈计数器和控制器已经就绪，才开放周期任务 |
| 5. 前台外设 | flash_init、tft180_init、wifi_assistant_init | 初始化/联网等待在main内，不在PIT中执行 |
| 6. 图像入口 | mt9v03x_init失败则等待1000ms重试；成功后camera_frame_install、vision_reset | 安装双缓冲包装回调，清视觉跨帧历史，仍保持停止 |

startup在进入main前已开启全局IRQ；PIT初始化后就可能在WIFI和相机等待期间服务，持续写出零命令。main末尾的interrupt_global_enable(0)确认主循环中断开放，不是首次启用PIT。相机一直初始化失败时main不能进入常规poll，但已建立的控制任务保持停止；WIFI初始化失败会记录错误状态，返回后主循环允许停车重连。

### 一次主循环迭代如何消费数据

1. **安装当前控制带。** control_get_params同时取得整组参数和version。与band_version不同才调用vision_set_control_band；成功才推进band_version，失败走control_report_invalid_fault。参数的控制版本和视觉内部安装版本各有用途。
2. **领取人工请求。** wifi_assistant_take_width_calibration_request复制并消费一次性请求，main只登记pending和起始逻辑时间，不在这里发车。
3. **领取新快照。** camera_frame_take返回1才进入图像链；返回0时main不重新发布旧图，PIT继续独立检查视觉年龄。
4. **灰度到控制目标。** 二值化前保存允许发送的原灰度副本，然后统计直方图、求Otsu、原地二值化、vision_process；main补vision.threshold，再control_publish_vision。转向目标在这里更新，速度PI与真正硬件写出在PIT。
5. **处理停车辅助功能。** 取一致快照后尝试一次宽度标定；仅非RUNNING且显示间隔≥200逻辑ms时刷新TFT。wifi_assistant_on_frame内部检查状态和图传周期，边线另传，不涂改算法输入图。
6. **主循环末尾收发。** 宽度pending持续没有新帧且超过1000逻辑ms则取消；最后wifi_assistant_poll接收人工请求并发送快照。某轮poll产生的宽度请求通常在下一轮领取。

main把视觉放在显示/网络之前，但没有为各工作分配固定的CPU时间片。停车时整图发送、屏幕和重连仍可能拉长一次主循环；运行路径关闭这些工作并使用SPI截止检查。实际帧龄、最坏耗时和中断抖动仍需仪器确认。

### 10ms采样怎样成为轮速反馈

PIT_CH0每次服务分别执行encoder_get_count、乘ENCODER_1_SIGN或ENCODER_2_SIGN，然后encoder_clear_count。左侧配置-1、右侧+1，将接线计数转换成“前进为正”；这只是当前配置，必须架空核对。底层读取QTIMER当前计数寄存器并将其作为int16返回，清零直接写CNTR=0，两个轮子各有独立计数器。

读取不是累计里程：正常10ms服务时，增量12就表示12 counts/10ms；一秒约1200 counts。若一次服务被延迟到20ms仍直接把较长窗口计数交给control_tick，当前没有按实际窗口重新归一化，也没有补漏计时。两轮依次读清还存在很小的采样时刻偏移。改变周期前须按后文现场测试检查量化、丢服务和方向。

control_tick完成后，PIT调用control_get_snapshot，固定DIR为各轮前进电平，将pwm_left/right的非负整数命令转uint32写入pwm_set_duty。速度PI不直接碰DIR寄存器，负误差最多把命令压到0，当前没有反向制动命令。舵机servo_enabled为0则写0；为1则使用SERVO_MOTOR_DUTY换算。PWM载波与脉宽由外设产生，CPU不会等待17kHz周期或舵机高电平结束。

最后wifi_assistant_tick只推进通信逻辑时间，再清PIT事件位。PIT_CH1/2/3没有本周业务任务，ISR仅检查并清标志；SysTick入口也未承担该控制链。

### 本章代码索引

| 调用入口 | 输入/输出或阅读要点 |
| --- | --- |
| {{src|user/src/main.c|main}}、{{src|user/src/main.c|Init}} | 完整初始化顺序与主循环六个步骤 |
| {{src|user/src/control.c|control_set_critical_hooks}}、{{src|user/src/control.c|control_set_output_enabled}} | 同步钩子与编译输出许可 |
| {{src|../libraries/zf_driver/zf_driver_encoder.c|encoder_quad_init}} | 将A/B引脚与QTIMER计数器关联 |
| {{src|../libraries/zf_driver/zf_driver_encoder.c|encoder_get_count}}、{{src|../libraries/zf_driver/zf_driver_encoder.c|encoder_clear_count}} | 当前计数读取与独立清零，不是计算rpm |
| {{src|../libraries/zf_driver/zf_driver_pit.c|pit_init}} | pit_ms_init宏的实际底层实现；宏定义见[zf_driver_pit.h](../../libraries/zf_driver/zf_driver_pit.h#L90) |
| {{src|user/src/isr.c|PIT_IRQHandler}} | 反馈先于控制，控制先于PWM，最后推进通信时间 |
| {{src|../libraries/zf_driver/zf_driver_pwm.c|pwm_set_duty}}、{{src|../libraries/zf_driver/zf_driver_pwm.c|pwm_init}} | 占空命令换寄存器值、LDOK与完整周期重载 |

## 相机快照与帧新鲜度

### 从相机配置到原SDK双缓冲

当前[相机配置](../../libraries/zf_device/zf_device_mt9v03x.h#L77)为188×120灰度、50FPS、自动曝光关闭、曝光配置512、增益32、左右/上下偏移0。512是设备曝光配置值，不能写成已确认的毫秒/微秒。每帧8位像素共22560字节；50FPS换算为20ms配置周期，实际帧率和曝光影响待M3确认。

```text
Init()
  → mt9v03x_init()
      → 先尝试SCCB配置；失败后UART配置/回读
      → csi_init(..., mt9v03x_finished_callback, ...)
      → csi_add_empty_buffer(image1、image2) → csi_start()
  → camera_frame_install() 保存原callback和userData，再换成frame_callback

采集完成IRQ：
CSI_IRQHandler → CSI_DriverIRQHandler → CSI_TransferHandleIRQ → frame_callback
  → 原mt9v03x_finished_callback
      → csi_get_full_buffer成功 → 切换mt9v03x_image
      → 提交另一空缓冲 → mt9v03x_finish_flag=1
  → 包装层发布完成指针、逻辑时间、帧号
```

两个采集缓冲在驱动源码中用AT_DTCM_SECTION_ALIGN(...,64)声明；其实际落位须和最终map配合检查。原双缓冲负责采集交替，包装层不能省略原callback，否则图像地址和空缓冲提交会失去原行为。包装安装使用短临界区，若callback已经是frame_callback则不重复保存自己，避免递归。

### 包装回调怎样保证新旧事件语义

frame_callback先在FrameDone事件清旧finish_flag，再调用保存的原callback并传原userData；只有status确为FrameDone且原回调重新置flag，才发布新事件。先存completed_image和captured_ms，最后frame_sequence加1。其他事件、原回调没有取得完整缓冲的失败路径都不刷新图像时间。

captured_ms通过control_now_ms读取最近PIT推进的逻辑时钟。它对应采集完成回调，不是曝光开始时间，也不是独立高精度时戳；CSI先于某次PIT被服务时可读到上一tick标签。把复制完成时刻填进去会让旧图看起来变新鲜，因此main始终沿用这个采集标签。

### camera_frame_take的五个步骤

| 步骤 | 关键判断/操作 | 成功或失败之后保留什么 |
| --- | --- | --- |
| 1. 验输出空间 | destination、frame_id、capture_time_ms都非NULL | 输出容量需至少22560字节；接口不会推断缓冲长度 |
| 2. 同锁领取 | 保存原PRIMASK，复制sequence/time/pointer；拒绝NULL或已消费编号 | 无新图返回0，不向控制层伪造重复结果 |
| 3. 复制整图 | 清领取时旧flag，再恢复原PRIMASK后memcpy | 正常main开放IRQ时PIT/CSI可以服务；不强开外层本来关闭的IRQ |
| 4. 再验序号 | 重新短锁；若sequence!=frame_sequence则拒绝 | 目标图本次不可用；consumed推进到被拒旧序号，最新事件仍待下一轮消费 |
| 5. 成功发布 | 计入跳过的中间编号，更新consumed，输出帧号与采集时间 | 返回1；main可在自己的快照上二值化/扫描 |

序号复核保护“复制期间又发布新完成事件”的软件竞争，并不锁住物理DMA或证明缓存一致性。当前真实CSI/DMA、缓冲复用与cache行为尚未取得板端证据；不能将模拟PASS写成图像原子采集已实测。

### 丢帧统计和视觉超时是两条不同路径

camera_frame_dropped统计主循环没有消费的发布序号及复制竞争拒绝的旧图。两个独立教学场景均从已消费10开始：场景A成功领取13，计入11/12两帧；场景B复制13时14到来，拒绝13并按13−10=3计入11/12/13，将consumed推进到13，下一轮成功消费14不会再次计旧13。数值推演用于解释公式，非现场采样结果；uint32差值按自然回绕计算。

control_publish_vision只用可用新帧刷新last_good_ms。camera_frame_take返回0、重复帧、旧帧或不可用图像都不能延长最近好图的保护寿命。control_tick报告的vision_age_ms是“最近可用视觉采集时间的年龄”，并非最近任意frame_id的年龄；是否停车由控制章的age>100逻辑ms与连续失效新帧条件决定。

### 本章代码索引

| 层次 | 关键函数 | 阅读目的 |
| --- | --- | --- |
| 相机设备 | {{src|../libraries/zf_device/zf_device_mt9v03x.c|mt9v03x_init}}、{{src|../libraries/zf_device/zf_device_mt9v03x.c|mt9v03x_finished_callback}} | 配置失败/成功返回与原缓冲回调 |
| CSI封装 | {{src|../libraries/zf_driver/zf_driver_csi.c|csi_init}}、{{src|../libraries/zf_driver/zf_driver_csi.c|csi_get_full_buffer}}、{{src|../libraries/zf_driver/zf_driver_csi.c|csi_add_empty_buffer}} | 获取完成缓冲与提交空缓冲 |
| IRQ入口 | {{src|user/src/isr.c|CSI_IRQHandler}}、{{src|../libraries/sdk/drives/fsl_csi.c|CSI_DriverIRQHandler}}、{{src|../libraries/sdk/drives/fsl_csi.c|CSI_TransferHandleIRQ}} | 查SDK何时调用应用callback |
| 包装与领取 | {{src|user/src/camera_frame.c|camera_frame_install}}、{{src|user/src/camera_frame.c|frame_callback}}、{{src|user/src/camera_frame.c|camera_frame_take}} | 元数据同步、恢复原中断状态与复制复核 |
| 统计/保护 | {{src|user/src/camera_frame.c|camera_frame_dropped}}、{{src|user/src/control.c|control_publish_vision}}、{{src|user/src/control.c|control_now_ms}} | 软件丢帧、最近好图标签与逻辑时间 |
