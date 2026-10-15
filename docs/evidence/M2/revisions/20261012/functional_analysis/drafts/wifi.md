## V1人工参数与有界 WIFI

本章解释当前`demo`已经实现的人工调试链路：电脑逐飞助手V1经网络连接WIFI6B21-SPI V2.0模块，模块通过SPI与RT1064交换数据，应用层再把参数交给统一控制接口。模块的V2驱动与电脑的V1图像/波形协议是两个层次，不能用模块型号推断上位机协议版本。

当前实现提供人工设置、人工发车、停车图传和运行最高10 Hz波形。M4的实验协议、电脑桥、心跳及100 Hz完整日志尚未解锁；这些功能不在本章调用链中。以下时间预算和周期来自代码配置，SPI兼容、最坏服务耗时、控制抖动及真实停车延迟仍需M3实测。

### 1. 从初始化到收发回调：先建立通信，控制仍保持停止

调用入口为`Init → wifi_assistant_init`。`Init`先建立零PWM输出、控制状态和10 ms PIT，再初始化显示/WIFI，随后初始化相机。联网过程可能阻塞，但它不会产生人工启动请求；PIT已经可以维持停止输出。`wifi_assistant_init`返回`void`，初始化结果通过`wifi_assistant_state`区分模块、热点和socket失败，主程序不会把函数返回当作行车许可。

| 顺序 | 关键调用与输入 | 结果及下一步 |
| --- | --- | --- |
| 1 | `wifi_spi_service_begin(0)` | 选择停止恢复方式；调用前须保证车辆未运行 |
| 2 | 绑定`seekfree_assistant_parameter_observer = assistant_parameter_event`，调用`seekfree_assistant_receive_reset()` | 清V1接收半包与更新标记，再清本地宽度请求、STOP锁存及原图有效标记 |
| 3 | `seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_WIFI_SPI)` | 将发送回调绑定到`wifi_spi_send_buffer`，接收回调绑定到`wifi_spi_read_buffer`；本函数只绑定接口，不初始化硬件 |
| 4 | 将发送回调覆盖为`assistant_send` | 接收仍走驱动；V1所有发送段增加应用层续传、运行门控及故障处理 |
| 5 | `wifi_spi_init(NULL, NULL)` | 复位模块、初始化SPI/驱动FIFO并获取版本；返回0才表示该步骤成功，版本空字符串也判模块错误 |
| 6 | `wifi_spi_wifi_connect(配置宏中的热点标识, 配置宏中的认证信息)` | 返回0表示连接热点成功；本章不展开凭据 |
| 7 | `wifi_spi_socket_connect(传输类型, 目标地址, 目标端口, 本地端口)` | 当前配置由模块连接电脑TCP Server；失败记`LINK_ERROR`，不能进入通信READY |
| 8 | 配置图像和X方向边线缓存，重置发送时刻，设通信`READY` | V1组件持有独立图传缓存地址；车辆控制状态仍须另行通过人工启动条件 |

电脑V1需匹配TCP Server角色及目标端口，当前驱动目标端口配置为8086；实际目标IPv4仍由配置与现场网络保持一致，不在正文复制地址或热点凭据。这里的“回调”是普通收发函数指针。SPI收发由主循环推进；PIT只调用`wifi_assistant_tick`累加通信逻辑毫秒，不在ISR中联网、图传或解析上位机参数。B17在驱动中用于查询模块是否就绪。

`wifi_spi_init`仅使能DWT周期计数，不清`CYCCNT`，并清版本/MAC/IP及驱动FIFO，避免旧连接信息被当作新连接成功证据。工程使用SPI_1、20 MHz、MODE3及软件片选；`WIFI_SPI_AUTO_CONNECT=0`，热点和socket由应用层显式分阶段建立。

就近入口：{{src|user/src/main.c|Init}}、{{src|user/src/wifi_assistant.c|wifi_assistant_init}}、{{src|../libraries/zf_components/seekfree_assistant_interface.c|seekfree_assistant_interface_init}}、{{src|drivers/wifi_spi_v2/zf_device_wifi_spi.c|wifi_spi_init}}、{{src|drivers/wifi_spi_v2/zf_device_wifi_spi.c|wifi_spi_get_version}}、{{src|drivers/wifi_spi_v2/zf_device_wifi_spi.c|wifi_spi_wifi_connect}}、{{src|drivers/wifi_spi_v2/zf_device_wifi_spi.c|wifi_spi_socket_connect}}、{{src|user/src/isr.c|PIT_IRQHandler}}、{{src|user/src/wifi_assistant.c|wifi_assistant_tick}}。

### 2. 接收链路：两个FIFO各自保留一类未完成数据

主循环接收顺序是`wifi_assistant_poll → seekfree_assistant_data_analysis → seekfree_assistant_receive_callback → wifi_spi_read_buffer`。驱动先交付已有缓存，再按模块应答主动取包；解析器把交付字节写入自己的FIFO，等待完整参数包。

| 缓冲层 | 当前容量 | 保存什么 | 清理入口 |
| --- | --- | --- | --- |
| V2驱动`wifi_spi_fifo` | 1024字节 | 本次调用者容量装不下的模块接收尾部；发送时顺带收到的上行参数数据 | `wifi_spi_init`重新初始化驱动FIFO |
| V1组件`seekfree_assistant_fifo` | 128字节 | 等待拼齐/寻找帧头的参数半包和字节流 | `seekfree_assistant_receive_reset` |

`wifi_spi_read_buffer(buffer, length)`的`length`是调用者容量，返回值是**实际交付字节数**。它先取驱动FIFO已有量与请求容量的较小值，再尝试SPI读取；单个模块接收载荷为32字节。载荷装不下时，多出的尾部继续留在驱动FIFO，不能假定每次调用者都还有32字节空间。运行时，一次`read_buffer`主动读取最多4个模块包，并受本轮DWT截止限制；发送函数附带的尾包读取也受共用截止约束，不能把“4包”扩大解释为整轮所有收发的统一包数上限。

解析器每轮先计算`receive_capacity = 128 - fifo_used(...)`，再只向回调请求这部分容量。FIFO无剩余空间时，先解析已有数据，不继续塞一整块新数据。

**3字节半包与125字节请求的算例。** 旧FIFO已有3字节，新的上游突发有128字节，则首轮只能请求`128−3=125`字节；剩下3字节仍在上游，下一轮续读。若直接请求128字节，原FIFO的整体写入可能因空间不足被拒绝，突发中的STOP也会一起丢失。修复后的回归夹具验证了首读125字节、完整STOP成功消费、剩余3字节继续解析。一般字节布局中，若STOP本身落在未读尾部，需下一轮补齐，不能把这个算例解释成任意布局都能首轮完成STOP。

若回调报告量超过本次请求容量，或FIFO写入返回非成功，组件增加`seekfree_assistant_receive_error_count`并清接收状态。应用层会比对该计数，禁止执行错误轮次中的参数。`receive_reset`只清本组件FIFO和更新标记，不清参数数值；未标更新的旧正目标不会再被应用。清理也不归零累计错误计数，因此异常不会被复位动作掩盖。

就近入口：{{src|drivers/wifi_spi_v2/zf_device_wifi_spi.c|wifi_spi_read_buffer}}、{{src|../libraries/zf_components/seekfree_assistant.c|seekfree_assistant_data_analysis}}、{{src|../libraries/zf_components/seekfree_assistant.c|seekfree_assistant_receive_reset}}。

### 3. V1参数包怎样成为请求：校验、观察事件，再覆盖通道末值

当前参数包包含帧头、功能字、线上通道号、8位和校验及一个`float`值，本工程结构大小为8字节。接收帧头为`0x55`，设置参数功能为`0x20`。这里使用**和校验**，不能称作CRC。

解析器只在FIFO至少有一整包时窥看候选包，保留原校验字节，把局部副本中的校验位置零，再重新求和。它同时验证和校验、设置参数功能、通道下限1及上限8；四项都成立才处理请求。无帧头或非法候选只移除1字节继续寻找帧头；合法包移除整包；不足一包的尾部留待下轮补齐。

合法候选按以下顺序处理：

1. 调用可选`seekfree_assistant_parameter_observer(channel, value)`，保留每个合法包的事件顺序。
2. 将线上通道减1，写入`seekfree_assistant_parameter[0..7]`。
3. 置对应更新标记，供应用层消费。

协议合法仅说明包可被解释，不表示参数值已被控制层接受。同通道多包在数值数组中保留末值；observer提供单独的STOP锁存，补足“末值覆盖”无法表达的停止事件。普通坏和校验包只触发重同步，不增加容量异常计数，也不会直接触发LINK故障。

就近入口：{{src|../libraries/zf_components/seekfree_assistant.c|seekfree_assistant_data_analysis}}、{{src|user/src/wifi_assistant.c|assistant_parameter_event}}。

### 4. STOP优先于参数和启动，门控取接收前的状态

`wifi_assistant_poll`在接收前保存`initially_running`；`apply_parameters(initially_running)`先把8个通道的末值和更新标记复制到局部，再清共享更新标记。这意味着请求即使被拒绝，也不会在下一轮重复执行。

`assistant_parameter_event`观察的是**线上通道1、值精确为0**，对应V1界面通道0停止。只要本轮合法包序列中出现过该事件，`stop_seen`就保持为1；即使随后同通道正目标覆盖了数值数组，应用层仍优先调用`control_request_stop`，不提交其它运动参数，不执行本批启动。合法STOP→START、START→STOP都以停止结束；坏校验包不会制造STOP事件。该观察器只保留通道0零目标停止，不能把其它人工操作也理解成有同等的逐包事件保存机制。

若本轮没有STOP，但`initially_running=1`，其它通道更新全部拒绝，包括图传设置。其间PIT可能因丢线、BENCH到时或人工停止而令当前状态变化，仍不能解锁这一批改参；必须在下一轮停止态重新提交。这是防止“刚停车就顺带套用原运行批次参数”的门控。

`control_request_stop`立即清软件有效目标、PWM、积分及舵机使能，保留配置中的基准目标；已有FAULT不会被STOP解除。后续PIT才能把零命令写进PWM寄存器，随后还有外设重载和机械响应。基础目标非零不会自行恢复发车；新人工启动仍需跨过停机屏障等条件。

就近入口：{{src|user/src/wifi_assistant.c|apply_parameters}}、{{src|user/src/control.c|control_request_stop}}、{{src|user/src/control.c|control_request_run}}。

### 5. 八个输入通道：运动字段、人工操作和停车显示设置

以下通道号为**V1界面0..7**；线上编号统一加1。它们是输入参数通道，与下节8个输出波形通道的含义分别定义，不能因为索引相同就认为输入输出一一对应。

| 界面/线上 | 输入值及限制 | 当前动作 |
| --- | --- | --- |
| 0 / 1 | 0停止；正目标范围`(0,100]`，单位`counts/10ms` | 正目标同时写左右基准目标并产生人工启动请求；尚须控制接口许可 |
| 1 / 2 | Kp，`[0,1000]` | 同时修改左右速度P增益；V1界面共用，内部两轮PI仍独立 |
| 2 / 3 | Ki，`[0,1000]` | 同时修改左右速度I增益；积分按秒计算，速度D固定0 |
| 3 / 4 | 精确整数0..6 | 人工操作，见下表；此通道不再设置速度Kd |
| 4 / 5 | 舵机P，`[0,5]`，命令角/像素 | 修改`ControlParams.steer_kp`，不直接写舵机PWM |
| 5 / 6 | 精确0/1/2 | 停车仅收参数/图传/波形；切换使旧原图失效并重置发送计时 |
| 6 / 7 | 精确0/1 | 原始灰度/处理图；切换使旧原图失效，两者均按8位图像发送 |
| 7 / 8 | 发送周期`[20,1000] ms` | 修改停车发送间隔；转`uint32`舍去小数，运行仍固定100 ms间隔 |

速度Kp把`counts/10ms`误差变为PWM命令，Ki再乘0.010秒的控制周期形成积分贡献；V1输入不能沿用参考三轮工程的cm/s数值或另一采样周期的增益语义。代码中的范围只是接口接受范围，不表示范围内任意值都已实车验证。

| 界面通道3值 | 操作 | 同批正目标是否还能请求启动 |
| --- | --- | --- |
| 0 | 无操作 | 保留原有启动请求 |
| 1 | 只选择本地`TRACK`模式 | 可请求，但须通过TRACK条件 |
| 2 | 只选择本地`BENCH`模式 | 可请求；BENCH为人工确认的架空试验，最长8秒 |
| 3 | 请求`control_reset_fault` | 清故障不自动启动；TRACK须重新取得三帧资格 |
| 4 | 将`calibration_confirmed=1`写入候选运动配置 | 可请求；确认标记不能代替硬件标定证据 |
| 5 | 请求停止、撤销确认，并置`start_cancelled` | 取消本批启动 |
| 6 | 准备一次静止直道宽度请求，并置`start_cancelled` | 取消本批启动，实际标定由主循环另行检查 |

选择模式只改变`selected_mode`，不直接切车辆的运行模式。通道3值必须与上述整数精确相等；把原Kd浮点数发送到这里不会被静默转成一个操作。值5是循环内的停止/撤销操作：它取消启动，但合法候选中的标定确认及其它运动字段仍可提交；它不同于通道0零目标造成的整批提前返回。宽度请求由`wifi_assistant_take_width_calibration_request`取出后清零，主循环再检查停止、轮速、居中、新鲜度和逐行宽度；无新帧超过1秒取消待请求。

就近入口：{{src|user/src/wifi_assistant.c|apply_parameters}}、{{src|user/src/wifi_assistant.c|wifi_assistant_take_width_calibration_request}}、{{src|user/src/main.c|calibrate_width}}、{{src|user/src/main.c|main}}。

### 6. 运动参数为何整组提交，哪些操作不回滚

停止批次先调用`control_get_params(&params, &version)`取得完整配置及版本，随后只修改局部候选。通道0/1/2/4及标定确认字段置`dirty`；任一运动字段非法会令`control_batch_valid=0`，该批不再提交运动配置或启动。候选有效且`dirty=1`时，适配层只调用一次`control_apply_params(&params, version)`。

控制层在短临界区内重新核对运行状态和`expected_version`，一次安装整组，更新版本/生效逻辑时刻并清PI历史，保证PIT看不到左右轮只更新一半。返回1为成功，0表示运行中、版本过期或候选非法；失败后不继续启动。V1包自身没有携带参数版本，这个版本检查由车端读取/提交完成，不能称作电脑已经完成版本同步或收到ACK。

运动配置成功后，适配层才更新兼容舵机P镜像；若`start_requested && !start_cancelled`，最后调用`control_request_run(selected_mode)`。启动还要求READY、输出许可、人工标定确认、正目标及停机屏障；TRACK另需舵机许可和连续三帧高可信新鲜图像。BENCH不采用视觉转向，但有8秒逻辑时限。启动返回0也只增加拒绝计数，不绕过控制接口。

**整组提交的边界仅为运动配置。** 通道5/6/7本地显示设置、模式选择、已经执行的清故障/停止、一次性标定准备，不属于同一回滚事务。循环按通道索引处理这些动作；即使最终某个运动值非法，已执行的本地操作仍保留。例如同批舵机P超范围而图传模式合法，运动组不提交，但停止态显示模式可能已改变。通道3清故障失败会增加拒绝计数，本身不把`control_batch_valid`置零；其它合法运动候选仍可进入控制层检查。文档不能把它写成“八通道全成功才全部生效”。

拒绝计数主要按这一轮保留下来的通道请求计数；同通道末值覆盖、事件锁存和控制接口拒绝不是一份逐包ACK日志。当前人工波形也未发送拒绝原因、参数版本或实验ID。

就近入口：{{src|user/src/wifi_assistant.c|apply_parameters}}、{{src|user/src/control.c|control_get_params}}、{{src|user/src/control.c|control_apply_params}}、{{src|user/src/control.c|control_request_run}}、{{src|user/src/control.c|control_reset_fault}}。

### 7. 一次poll的真实顺序：预算不能被状态切换绕开

`wifi_assistant_poll`在主循环末尾执行，视觉新帧处理与目标发布先于该次通信。它不按每10 ms固定调用，也没有把发送放入PIT。

| 顺序 | 当前操作 | 提前退出条件 |
| --- | --- | --- |
| 1 | 读`initially_running`，`wifi_spi_service_begin(initially_running)` | 运行不调用状态TFT刷新 |
| 2 | 检查通信READY | 运行非READY调用链路故障接口；停止错误状态到重连间隔才重新初始化，随后结束本轮 |
| 3 | 保存SPI/解析错误基线，调用`seekfree_assistant_data_analysis` | 任一累计计数变化，清半包、更新标记、宽度请求和STOP锁存，结束本轮 |
| 4 | `apply_parameters(initially_running)`，`control_get_snapshot` | 接收前运行、处理后已停/故障，立即结束，不转入停车发送/恢复 |
| 5 | 若原先停止、这批人工请求刚启动，重新以`service_begin(1)`启用运行预算 | 这是进入运行方式，不是在原运行轮次中反复重置截止 |
| 6 | 检查READY、流模式及发送到期条件 | 停车只在`stream_mode=2`发波形；运行强制波形并使用100 ms间隔 |
| 7 | 记录发送时刻，填8通道快照，调用波形封装发送 | 无论正常或上述提前退出，都配对`wifi_spi_service_end` |

`elapsed_time_ms`由PIT每次已服务周期累加，是逻辑毫秒。`(uint32)(now-last_send_ms)`用于处理无符号回绕，但不能补偿漏掉的PIT服务；因此“配置100 ms间隔”不能代替真实线上发送间隔测量。发送时刻在本次尝试前更新，避免失败后前台每轮密集重发。

就近入口：{{src|user/src/main.c|main}}、{{src|user/src/wifi_assistant.c|wifi_assistant_poll}}、{{src|drivers/wifi_spi_v2/zf_device_wifi_spi.c|wifi_spi_service_begin}}、{{src|drivers/wifi_spi_v2/zf_device_wifi_spi.c|wifi_spi_service_end}}。

### 8. 八个输出波形通道：展示快照，保留控制与硬件的边界

当前`wave.channel_num=8`，逐飞组件填入V1帧头、功能/通道数、长度和8位和校验，再调用受控发送回调。本工程一个8通道波形包为4字节头部加8个4字节`float`，共36字节，不含模块SPI层头部。它与输入8个参数滑块是两套含义。

| 波形索引 | 来自`ControlSnapshot` | 含义与单位 |
| --- | --- | --- |
| 0 | `target_left` | 左轮有效目标，已包含弯道/可信度降速与速度斜坡；`counts/10ms` |
| 1 | `target_right` | 右轮有效目标，`counts/10ms` |
| 2 | `measured_left` | 最近PIT的左编码器计数，约定为标称10 ms窗口计数，转float显示 |
| 3 | `measured_right` | 最近PIT的右编码器计数，含义同左轮 |
| 4 | `pwm_left` | 左轮实际下发整数PWM命令，以float保存；不是`raw_pwm_left`或硬件反馈 |
| 5 | `pwm_right` | 右轮实际下发整数PWM命令 |
| 6 | `vision_error_px` | 像素误差；无效图时可能保留最后可用误差，不等于当前图仍可信 |
| 7 | `servo_cmd` | 变化率限制后的命令角；不是实测前轮角或已生效脉宽 |

快照复制在同一短临界区内完成，保证字段读取一致，但不同字段仍可能由不同事件更新：主循环可能刚发布了更晚的视觉帧，PWM仍来自最近PIT。`ControlSnapshot.frame_id`是最新发布帧号，不是PWM所用视觉来源的凭证；当前8通道中也没有发送帧号。

运行最高10 Hz波形取的是当时快照，**没有做100 ms平均，也没有收集每个100 Hz控制样本**。它适合人工观察大致跟随、符号与饱和；可能漏掉或混叠快速振荡，不能用它证明100 Hz抖动、完整超调曲线或每周期保护响应。严格原日志与实验评价属于未解锁的M4。

就近入口：{{src|user/src/wifi_assistant.c|wifi_assistant_poll}}、{{src|user/src/control.c|control_get_snapshot}}、{{src|../libraries/zf_components/seekfree_assistant.c|seekfree_assistant_oscilloscope_send}}。

### 9. SPI的两层DWT截止：整轮共享，单次交换也有上限

受检交换`wifi_spi_exchange_checked(write_data, read_data, length)`返回0成功、1错误。写指针为空时发0产生时钟，读指针为空时仍取走RX数据但不保存；外部片选由上层写/命令/数据函数统一释放。每次受检交换固定局部`start`，每个字节的TX/RX等待都复用它，不在等待循环中续期。

| 配置 | 代码值 | 作用范围与限制 |
| --- | --- | --- |
| `WIFI_SPI_RUNTIME_BUDGET_US` | 2000 μs | 从本轮`service_begin(1)`起，RX/TX/命令/数据段共享的运行截止预算 |
| `WIFI_SPI_RUNTIME_TRANSFER_US` | 1000 μs | 每次`exchange_checked`的运行等待预算；头段与载荷段各自还有局部检查，但不能绕过整轮截止 |
| `WIFI_SPI_STOP_TRANSFER_US` | 10000 μs | 停车单次受检交换预算，不包括模块复位、热点/socket等恢复等待 |
| 运行B17等待 | 未就绪立即返回 | 不进行停止路径的10 μs轮询睡眠 |
| 停止B17等待 | 参数单位ms，内部乘100形成10 μs轮询次数 | 设置/查询命令保留原恢复等待；不能误读成参数本来就是100 μs |

判定使用`(uint32)(DWT->CYCCNT-start)`，按`system_clock/1000000`换算为每μs核心周期，再同时检查局部预算与运行整轮预算。短区间的无符号差值可跨32位计数回绕；当前这些区间远短于一次回绕。DWT时间包含中断抢占，不只是SPI移位时间。`runtime_service_enabled`保存轮次身份，控制层在中途停车也不会把本轮变成长重试路径。

受检交换先等模块Busy清除，再暂停LPSPI、清TX/RX FIFO与状态；仅修改连续传输、屏蔽、硬件片选及帧长位，设置`FRAMESZ=7`代表8位。CPOL/CPHA/PRESCALE等其余TCR配置保留，CCR不重写，沿用原SPI_1/20 MHz/MODE3设置。随后逐字节检查TX空间、硬件错误和RX到达，最后再确认Busy结束。

任一等待逾期或硬件错误都会调用`wifi_spi_abort_exchange`：暂停、清FIFO/状态、重新使能，增加`wifi_spi_io_error_count`并返回错误；外层仍拉高软件片选。它不在失败处复位模块或联网。`service_end`记录最近运行轮次及峰值整数μs，随后清轮次标记；这些是软件观测字段，源码存在它们不表示本次已经取得实机数据，也不构成物理2 ms最坏耗时保证。

就近入口：{{src|drivers/wifi_spi_v2/zf_device_wifi_spi.c|wifi_spi_deadline_expired}}、{{src|drivers/wifi_spi_v2/zf_device_wifi_spi.c|wifi_spi_exchange_checked}}、{{src|drivers/wifi_spi_v2/zf_device_wifi_spi.c|wifi_spi_abort_exchange}}、{{src|drivers/wifi_spi_v2/zf_device_wifi_spi.c|wifi_spi_wait_idle}}、{{src|drivers/wifi_spi_v2/zf_device_wifi_spi.c|wifi_spi_service_end}}。

### 10. 从V1发送段到模块数据块：返回的是未发尾部

所有V1发送段经`assistant_send(buffer, length)`进入驱动。它计算`running = 当前控制RUNNING || 本轮service仍为运行`，因此PIT刚停车时也不能触发停车20 ms延时。运行只尝试一次；停车按配置最多尝试3次，尚有尾部及下一次尝试时等待20 ms后续传。

`wifi_spi_send_buffer`返回**剩余未发送字节数**，不是已发送量。应用层以`buffer+(length-remaining)`定位已发前缀后的尾部；驱动只在一个发送块成功交换后扣长度并推进指针。停车单发送块上限4088字节，运行单块上限128字节；运行收尾取包仍共享原DWT截止。Busy软件标记防当前工程主循环调用重入，不是可以任意跨ISR/线程使用的原子锁。

模块的数据交换先发4字节SPI层头部，再发有效数据：长发送块先保存本地32字节接收窗口内的RX，余下TX仍读走RX但不保存；短发送块先补零到32字节，避免发送未初始化字节或读越界。模块报告长度超过本次载荷容量也增加I/O异常计数，将包头设无效、长度清零，消费者不能复用前一次的命令和长度。

`assistant_send`在尾部未发完，或运行轮次新增底层I/O错误时设通信`SEND_ERROR`、使原图失效，并调用控制链路故障入口。V1组件当前没有消费发送回调的返回值；因此后续图像段可能仍调用该回调，但通信状态已不是READY，回调立即返回整段未发，不继续SPI。完整图像计数只在整套组件发送后通信仍READY时增加。

兼容多缓冲接口`wifi_spi_send_buffer_multi`先求全部段总长度，再依次发送；返回值包含当前未发尾部及以后所有段。某段有长度但地址为空，或第一次发送失败，就停止后续段，避免跳过缺段把一个协议帧拼成伪完整数据。

就近入口：{{src|user/src/wifi_assistant.c|assistant_send}}、{{src|drivers/wifi_spi_v2/zf_device_wifi_spi.c|wifi_spi_send_buffer}}、{{src|drivers/wifi_spi_v2/zf_device_wifi_spi.c|wifi_spi_transfer_data}}、{{src|drivers/wifi_spi_v2/zf_device_wifi_spi.c|wifi_spi_transfer_command}}、{{src|drivers/wifi_spi_v2/zf_device_wifi_spi.c|wifi_spi_send_buffer_multi}}。

### 11. 异常如何到达PWM，停止后怎样重连

解析前后SPI错误计数或容量错误计数变化时，`poll`先将通信设`SEND_ERROR`，使原图失效，记录重连等待起点，清V1半包/更新标记、本地宽度请求及STOP锁存，不调用该轮`apply_parameters`。若本轮接收前在运行，会调用`control_report_link_fault`；该接口只在**调用当时控制状态仍RUNNING**时锁LINK。若PIT已经停机或锁了另一故障，不覆盖已有状态/故障原因。

| 异常/事件 | 应用层结果 | 控制与恢复边界 |
| --- | --- | --- |
| 坏帧头、坏和校验、非法功能或通道 | 逐字节重同步，无参数更新 | 不等于容量异常，也不直接锁LINK |
| 回调返回量越界或V1 FIFO写失败 | 增解析异常计数、清接收、本轮请求作废 | 运行时走链路故障接口；停止时等恢复 |
| 受检SPI超时/硬件错误/非法返回长度 | 增I/O计数，回收本次交换 | 上层不使用该轮请求/异常包 |
| 发送尾部未完成 | 通信SEND_ERROR，原图无效 | 运行结束该轮；停车恢复不自动发车 |
| 人工通道0停止 | 本批STOP优先，软件输出清零 | 不解锁FAULT，不能同一sample停后又启 |

主循环锁存故障时，软件输出立即清零；后续PIT调用`control_tick`/读取输出快照，再写左右电机及舵机零命令。若故障在PIT内部检测到，则该次ISR继续写零。之后仍须等待PWM外设重载，机械停住更晚。代码配置电机17 kHz、舵机50 Hz，不能据此直接宣称通信异常后已在某个真实毫秒内停车；M3须分别测“接受/故障→寄存器写零→最后脉冲→机械停止”。

通信状态非READY且车辆停止时，只有模块/热点/socket/发送四类错误状态达到配置2000逻辑ms重连间隔，才再次调用`wifi_assistant_init`。重连清两层接收缓存及未应用请求，重绑接口，重新等待当前完整图；它保留已有控制参数和本地模式/显示设置，但不会生成正目标人工启动请求。清故障也只回READY，TRACK重新积累三帧资格；现场故障本身仍需操作者排除。

当前没有M4的心跳和500 ms失联保护。一次接收返回0可能只是无数据或模块尚未就绪，不能直接证明TCP已断；模块断链如何反映在SPI/发送返回、运行波形是否及时失败，必须实机验证。PIT另行执行图像年龄/丢线保护，不依赖WIFI波形发送成功。

就近入口：{{src|user/src/wifi_assistant.c|wifi_assistant_poll}}、{{src|user/src/wifi_assistant.c|assistant_send}}、{{src|user/src/control.c|control_report_link_fault}}、{{src|user/src/control.c|control_tick}}、{{src|user/src/isr.c|PIT_IRQHandler}}、{{src|user/src/control.c|control_reset_fault}}。

### 12. 原灰度图传的生命周期：二值化前保存，停车后消费

主循环对一个成功领取的新帧按以下顺序处理：`camera_frame_take → wifi_assistant_capture_raw → 直方图/Otsu/原地二值化 → vision_process → control_publish_vision → 停车显示 → wifi_assistant_on_frame`。原图保存必须在原地二值化之前，否则“原始图”已经只有0/255。

1. `capture_raw`先清`raw_frame_valid`。只有停止、通信READY、`stream_mode=1`、`image_mode=0`且图传到期同时成立，才将当前188×120灰度复制到独立`camera_image`，置有效标记。当前22560字节只是独立图传副本，不修改算法图。
2. 算法继续在主循环自己的图上二值化和寻线。边线数组独立传递，不在图上涂线后再交给视觉。
3. `on_frame`运行时、通信异常、未选图传或未到期都直接退出。原图模式需要本次有效副本，消费后清标记；处理图模式在此复制当前处理图，两种模式仍走8位MT9V03X图像封装。
4. 复制当前左/中/右边线到独立缓存，并记录本次发送时刻。配置使用X方向边线，每条120个8位横坐标，纵坐标由图像行确定。
5. 在停车service方式下调用`seekfree_assistant_camera_send`。组件依次发送图像头、图像主体、打点头和三条边线；图像主体22560字节按4088字节发送上限拆为6个发送数据块，额外协议头和模块尾包回读不包含在这个“6”中。
6. 只有全部段走完后通信仍READY，完整图像计数才增加。异常或重连使原图有效标记失效；不会在新连接续接旧半帧。

若原图保存时尚未到期、处理结束时才到期，`on_frame`会因本次没有原图副本而等待下一新帧，而不是发送旧图。图传模式切换、图源切换及连接恢复同样会使旧缓存失效。`on_frame`的`mid_error/servo_angle/threshold`保留旧调用接口，当前波形不使用它们，统一从控制快照取值。

完整图传和状态TFT均只允许停止路径。运行时即使停车设置仍为图像模式，也由`poll`强制小波形；停止后恢复原停车设置。发送频率还受到实际新帧到达、主循环处理和链路占用影响，不能把设定周期写成实测图传帧率。

就近入口：{{src|user/src/main.c|main}}、{{src|user/src/wifi_assistant.c|wifi_assistant_capture_raw}}、{{src|user/src/wifi_assistant.c|wifi_assistant_on_frame}}、{{src|../libraries/zf_components/seekfree_assistant.c|seekfree_assistant_camera_information_config}}、{{src|../libraries/zf_components/seekfree_assistant.c|seekfree_assistant_camera_boundary_config}}、{{src|../libraries/zf_components/seekfree_assistant.c|seekfree_assistant_camera_send}}、{{src|../libraries/zf_components/seekfree_assistant.c|seekfree_assistant_camera_data_send}}。

### 13. 沿调用链阅读和现场核对

读代码可按“初始化绑定 → 两层接收 → 校验与observer → STOP/参数应用 → 快照波形 → SPI截止与续传 → 故障/停车恢复 → 图传缓存”顺序追踪。每一步先区分参数来源、返回的是交付量还是未发量，以及调用处属于主循环还是PIT，避免把调试开销加入控制ISR。

现有通信软件回归覆盖真实解析器/FIFO、同通道STOP顺序、3字节半包加突发、非法返回长度和SPI寄存器/截止模型。它们验证代码路径，不能代替实车模块兼容和停车时序。M3现场应保留当前固件/参数，先停车图波/重连，再架空小目标和人工STOP，随后受控TRACK注入通信/图像异常，分别记录控制时序与PWM实际生效；未通过M3，不将本章扩展成agent实验闭环已实现。
