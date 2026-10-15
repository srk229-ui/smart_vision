## 公共接口与参数一致性

### 三类数据分别处于哪一层

`VisionResult`承接一次新图像的计算结果，`ControlParams`保存人工准备的基础配置，`ControlSnapshot`提供当前控制状态和计算命令。主循环通过接口把视觉和参数送给控制层，PIT通过同一控制层计算输出，再由ISR把命令写入硬件。当前应用的运动参数与启停请求使用这条统一链；兼容全局变量不是正式调参入口，直接修改控制器历史也不具有整组验证、版本和状态保护。

| 数据对象 | 由谁产生、由谁使用 | 关键含义 |
| --- | --- | --- |
| `VisionResult` | CSI提供帧号/采集时间，主循环视觉处理填入边线、质量、误差和可信度；`control_publish_vision`读取 | 帧号表示完成事件；时间来自10ms量化的设备逻辑时基。失效行的边线/中线是占位值，不能当作实测路径 |
| `ControlParams` | 人工操作准备候选；`control_apply_params`整组提交；运行中的PIT只读 | `target_left/right`是基础目标，弯道降速和斜坡不反写它们。标定确认是人工许可，不是自动获得的物理标定证据 |
| `ControlSnapshot` | 控制层更新；主循环和ISR通过`control_get_snapshot`复制 | `target_left/right`是实际送PI的有效目标；`pwm_left/right`是整数化后存为float的命令，不能解释成引脚测量值 |

快照字段属于多个时间域：编码器读数、有效目标和PWM在PIT更新；新视觉发布可以先更新`frame_id`、`threshold`、`confidence`和可用误差。短临界区保证结构复制一致，但不把所有字段变成同一时刻的一次采样。`frame_id`还可能属于失效新帧，不能从读取时的帧号反推此前某次PWM计算使用了哪一帧。

这些结构是内部C接口，尚不是M4的64字节日志或网络ABI。后续协议须显式选字段、定义版本和编码方式，不能直接发送结构内存。

### 单位与方向约定

| 信号或参数 | 当前单位/方向 | 使用方式 |
| --- | --- | --- |
| 基础目标、有效目标、编码器反馈 | `counts/10ms`；前进应为正，基础目标只允许0～100 | ISR分别读取、清零编码器，并乘`ENCODER_1_SIGN/ENCODER_2_SIGN`；有效目标减反馈得到速度误差 |
| 速度Kp | `PWM命令单位/(counts/10ms)` | 乘速度误差产生比例输出贡献 |
| 速度Ki | `PWM命令单位/[(counts/10ms)·s]` | 每次积分乘`dt_s=0.010`；旧按调用次数定义的非零Ki不能原样使用 |
| 速度斜坡 | `(counts/10ms)/s` | 每tick允许变化量为变化率乘周期秒数 |
| 视觉误差 | 像素；`94-控制带加权中点`，中点偏左为正 | 新帧P/PD转向和PIT弯道降速使用该值 |
| 转向Kp、Kd | 命令度/像素、命令度·s/像素 | Kp乘误差，Kd乘新帧误差变化率；二者产生舵机命令修正量 |
| 舵机命令及变化率 | 命令度、命令度/s | 命令度是脉宽刻度，需实测映射到前轮角度和R50能力 |
| `device_ms`、采集时间、视觉年龄 | 逻辑ms | 每次被服务的PIT加10；丢服务不会补计真实经过时间 |

编码器读数没有再除以实测间隔。调度正常时它对应标称10ms窗口；IRQ延迟或漏服务会使物理窗口变长，不能仍把它当作经独立时钟归一化的速度。人工测得每圈有效计数N和轮周长C后，规范速度单位可换算为`v[m/s]=counts10×100×C[m]/N`，这不替代对N、符号和真实采样间隔的标定。

### 运行期间保存的状态

| 持久RAM状态 | 跨调用保留的内容 | 复位/变化条件 |
| --- | --- | --- |
| `config` | 整组基础参数 | 上电初始化；停止态合法整组应用。当前没有由`flash_init`自动恢复这组控制参数的链路 |
| `current` | 状态、模式、故障、版本、测量和命令 | 上电清零，之后按操作、新帧和PIT分别更新 |
| `good_frames/good_seen/last_good_ms` | 高可信连续资格、是否已有可用帧、最近可用帧采集时刻 | 可用新帧更新；无效或低可信帧断开高可信连续计数；改控制带清旧资格，清故障清连续计数 |
| `frame_seen/last_frame_ms` | 已接受帧和相邻采集时间 | 每个向前的新帧更新，失效新帧也更新；重复/回退帧直接返回 |
| `derivative_ready/previous_error/steering_target` | 转向微分历史和限位目标 | 启动、停止或失效清微分资格；RUNNING/TRACK可用新帧计算目标 |
| 双轮`pid_param_t` | 各轮独立参数、积分贡献和误差历史 | 初始化、参数应用、停止/故障及停止态tick清历史，不共享左右积分 |
| `device_ms/run_start_ms/stop_sample` | 逻辑时钟、本次BENCH起点、显式停止等操作的采样屏障 | tick推进时钟/样本；人工启动记录起点；显式STOP、故障和要求全停的输出许可撤销记录屏障 |

`speed_target/speed_real/speed_pwm`只是右轮兼容字段；`speed_KP/KI`等兼容模板跟随部分应用参数，不能自动把变量修改传播到已初始化的两轮控制器。整车控制应使用参数API和统一快照。

### 参数候选怎样验证与提交

`params_valid`先拒绝空指针，对每个浮点范围通过`isfinite`排除NaN/无穷，再按闭区间校验。它不把非法值悄悄裁剪为合法值。

| 参数组 | 全组通过条件 |
| --- | --- |
| 双轮速度 | 两个基础目标均0～100；四个独立Kp/Ki均0～1000 |
| 输出与积分 | `pwm_max`为0～5000的整数，`floorf(pwm_max)==pwm_max`；`integral_max`为0～`pwm_max` |
| 速度斜坡 | `accel_counts_per_s`为1～1000 |
| 舵机配置 | min/max在0～180且min<max；center在二者之内；direction严格为-1或+1 |
| 转向与降速 | 转向Kp/Kd均0～5；变化率1～360命令度/s；弯道因子0.1～1 |
| 控制带 | 五行均0～119且严格递增；对应五权重各0.001～100，权重总和为正 |
| 保护与确认 | 视觉年龄参数只能100，丢线阈值只能3，人工确认只能0或1 |

整组提交按以下顺序完成：

1. `control_get_params`在同一短临界区复制配置与版本，调用者在自己的候选上修改。
2. `control_apply_params`在锁外校验稳定候选，减少IRQ屏蔽时间；非法候选返回0，保留已有参数。
3. 进入锁后再查`state!=RUNNING`及`expected_version==current.params_version`。运行中写入或旧版本写入均拒绝，不部分应用。
4. 若控制行或权重改变，清`good_frames/good_seen/derivative_ready`，避免用旧控制带的图像资格启动。
5. 一次复制整组配置，版本加1，记录逻辑生效时间；即使提交值完全相同，成功提交仍产生新版本。
6. 用新参数重新初始化独立左右PI，并清有效目标、输出和历史；保留原控制状态，FAULT不会因应用参数解除。随后恢复原PRIMASK并返回1。

例如读到版本7，准备目标20和新Kp后，若其他请求已成功提交成版本8，本次带版本7的请求返回0；必须重新读取，再准备候选。`pwm_max=10.5`也会整组拒绝，不能让软件限幅上限与硬件整数命令形成两个约定。这里是接口行为示例，不是一次真实车端调参记录。

参数应用本身不发车。当前V1适配器可把同一批人工正目标请求拆成“先应用候选，再请求启动”；是否进入RUNNING仍由启动API完整检查。改控制带后，main还须把新行/权重安装到视觉模块；setter失败锁`INVALID_VALUE`且不推进视觉已安装版本，不能报参数已经完整进入实际视觉链。

### 中断一致性与章内代码索引

`Init`在启动PIT之前安装保存/恢复PRIMASK的配对钩子。主循环改状态、参数或取快照时用短临界区，防止PIT读取半组数据；离开时恢复进入前状态，不能破坏外层已经关闭的IRQ。PIT内计算不执行网络、显示或整图复制。32位对齐的`device_ms`读取给出逻辑时间，`volatile`不把它变成实测墙钟。

| 阅读目的 | 源码入口 |
| --- | --- |
| 初始化数据与输出许可 | {{src|user/src/control.c|control_init}}；{{src|user/src/control.c|control_set_output_enabled}} |
| 验证候选和整组版本提交 | {{src|user/src/control.c|params_valid}}；{{src|user/src/control.c|control_get_params}}；{{src|user/src/control.c|control_apply_params}} |
| 临界区、状态与快照 | {{src|user/src/control.c|lock_control}}；{{src|user/src/control.c|unlock_control}}；{{src|user/src/control.c|control_get_snapshot}}；{{src|user/src/control.c|control_now_ms}} |
| main安装控制带、V1准备请求 | {{src|user/src/main.c|main}}；{{src|user/src/wifi_assistant.c|apply_parameters}} |

## 启动 停止和故障保护

### 上电初始化与控制服务先后

启动文件在进入main之前开放全局IRQ。main配置时钟/中断分组、初始化调试口并等待300ms，随后调用`Init`。Init先用零占空比初始化双电机与舵机，安装控制临界区钩子，初始化参数/状态和编码器，接着启动标称10ms的PIT；WIFI和相机初始化在其后进行。

因此等待联网或相机重试时，PIT已经有机会采样并推进逻辑时间，而控制仍为READY和零输出。末尾的`interrupt_global_enable(0)`不是PIT首次能执行的时刻。默认参数使control_init直接返回READY；舵机初始配置非法则返回FAULT。`CONTROL_INIT`不是一个持续运行的初始化调度任务。

输出许可由Init按编译宏传入。许可为1只代表执行器可用；状态、人工确认、正目标和模式条件仍全部需要通过。当前参数协议没有把编译许可0重新打开的入口。

### 人工启动的输入、拒绝与结果

`control_request_run(mode)`在短临界区完成检查，只在READY接受请求。

| 检查 | 不通过时的原因 |
| --- | --- |
| READY、电机许可、`calibration_confirmed=1` | FAULT不能直接启动；未确认方向、限位和试验条件不能发车 |
| `sample_id!=stop_sample` | 显式停止等操作后，至少要服务一个后续tick，让ISR有机会写零 |
| mode为TRACK或BENCH | 不接受其他枚举值 |
| 至少一轮基础目标>0 | 两轮均0不是启动请求；合法范围由整组参数校验保证 |
| TRACK额外条件 | 舵机许可有效；连续3个可信度>=0.6的新可用结果；已有可用图，最近好帧采集逻辑年龄<=100ms |

拒绝返回0，不进入RUNNING；通信适配器会记录拒绝计数。成功只切到RUNNING、设置模式与本次运行起点、清首帧微分资格；它不立即写PWM，也不保证轮速已达到目标。后续PIT才从清零目标出发执行斜坡、PI和舵机变化率限制。

三帧资格按不同向前的发布帧计数，不按三个PIT计数。相机配置50FPS时，三帧也不能解释成固定30ms；实际相位、领取成功率和处理耗时需要实测。人工硬件确认与参考宽度标定是两个动作：TRACK启动API不额外要求参考宽度非零，未标定时视觉只能使用双边实测，不能启用单边重建。

### 新帧发布与运行保护怎样衔接

`control_publish_vision`先按帧号半范围比较拒绝重复/回退结果，再计算相邻采集逻辑时间差。所有接受的新帧都更新最新帧号和阈值，可用条件还必须同时满足：`valid!=0`、`track_lost==0`、可信度0.3～1、有限像素误差在±188内、采集年龄<=100逻辑ms。

可用帧更新最近可用误差、`last_good_ms`和`good_seen`，清丢线计数；可信度>=0.6使连续启动计数增长至3，否则清该计数。不可用新帧将快照可信度归0，清启动/微分资格并增加`lost_frames`，但不刷新最后好图采集时间；旧误差可保留用于短暂运行，不能脱离可信度和年龄解释它。

每次`control_tick`先推进逻辑时间与sample，保存本次编码器读数，再执行保护。RUNNING/TRACK优先检查无好图或`vision_age_ms>100`，然后才检查`lost_frames>=3`。两条件同次满足时，锁存原因是`VISION_TIMEOUT`。age恰为100仍允许，常见下一服务逻辑值110才触发；这不是独立仪器测出的110ms物理停车上界。

前两个不可用结果只触发至多半基准的降速目标，仍经过正常斜坡；第三个不同的新无效结果在随后被服务的PIT触发停车。主循环完全没有新图时不会伪造失效结果或增加lost_frames，年龄保护仍在PIT检查。复制竞争拒绝快照也不直接计作一次控制层丢线。

| 故障 | 真实触发入口 | 恢复含义 |
| --- | --- | --- |
| `VISION_TIMEOUT` | TRACK的PIT保护查最近可用采集年龄/是否有好图 | 新图恢复不自动解除锁存 |
| `TRACK_LOST` | TRACK的PIT保护查连续3个不同新帧不可用 | 源码判断的是结果不可用，不要求全图120行都无边线 |
| `LINK` | 运行中SPI/接收解析异常报告 | 停车连接失败不按运行故障锁存；当前尚未实现M4心跳失联监测 |
| `INVALID_VALUE` | 控制带安装/模块约定失败，或转向计算非有限等路径 | READY也可锁存；普通非法参数候选则直接拒绝，不能把两种路径混为一谈 |

`latch_fault`设置FAULT和原因，清高可信启动计数，调用统一清输出，再记录当前sample作为屏障。故障产生在PIT内时，本次ISR就会写零，但清故障仍需再服务一个后续tick。LINK/接口故障来自主循环时，软件先清零，下一次PIT才施加到硬件。

### STOP、清故障与保留基础目标

`control_request_stop`将非FAULT状态转READY；已有FAULT则保留。它立即清有效速度目标、PWM/raw、积分、饱和标记和舵机使能，复位两轮PI以及微分资格，并记录`stop_sample`。配置中的基础目标和已有高可信连续帧资格保留。舵机命令字段记中位，但使能为0，并不驱动前轮回中。

假设显式STOP发生在sample=N，立刻再请求启动或清故障会因为sample仍等于N被拒；后续tick推进到N+1并维持零命令，才跨过屏障。若STOP返回READY、保留的3帧资格仍新鲜，之后的人工TRACK请求可以使用这组资格，不强制重新采三帧。清FAULT则不同：`control_reset_fault`要求不RUNNING、屏障已过、配置合法；成功回READY、清故障和good/lost连续计数，仍需重新积累3帧并再次人工启动。该接口在正常READY调用也会清连续资格并保持停止，不能把它当作无副作用状态查询。

完整V1 STOP包由observer优先锁存；同批后来的正目标末值不能覆盖STOP。同批STOP优先路径不继续应用或启动运动请求。若该轮通信服务起点为RUNNING，中途STOP后仍立即结束本轮，不转入停车长重试。控制API的屏障是另一层保证，二者共同使“同批停后又启”不能绕过一次零输出服务。

BENCH启动使用自己的`run_start_ms`；每次人工获准启动重新计时，到`device_ms-run_start_ms>=8000`自动回READY并清输出，不依赖图像。**BENCH自然到期不重新设置stop_sample**；到期ISR在本次控制任务后续输出段写零。显式STOP、故障及要求全停的许可撤销才建立新的屏障，不能把所有回READY的路径都描述成同一种停机屏障。下一次BENCH仍要人工请求，不自动续接8秒。

撤销电机许可，或当前TRACK撤销舵机许可，会立即清软件输出并记录屏障；BENCH仅撤销舵机许可不停止电机，后续tick把舵机使能更新为0。参数应用、清故障或网络恢复都不自动启动。

### 软件零输出与物理停车

PIT先取统一快照，保持配置的电机前进DIR，再写两轮整数PWM；舵机使能为0时写0。底层`pwm_set_duty`更新缓冲寄存器并设置LDOK，初始化选择完整周期重载。人工STOP的API已接受与下一ISR写零是不同事件；写零与引脚停止脉冲又是不同事件。

电机已配置17kHz，周期换算约58.8µs；舵机已配置50Hz，完整周期20ms。它们的相位与PIT不必同步，真实端到端停车延迟、驱动板滑行/制动模式和机械停距仍属M3实测。停机屏障证明控制服务次数前进，不能证明轮子已静止或前轮已到中位。

| 阅读目的 | 源码入口 |
| --- | --- |
| 上电零输出、PIT早启 | {{src|user/src/main.c|Init}}；{{src|user/src/main.c|main}} |
| 人工启动/停止/清故障 | {{src|user/src/control.c|control_request_run}}；{{src|user/src/control.c|control_request_stop}}；{{src|user/src/control.c|control_reset_fault}} |
| 故障清输出与来源 | {{src|user/src/control.c|reset_outputs}}；{{src|user/src/control.c|latch_fault}}；{{src|user/src/control.c|control_report_link_fault}}；{{src|user/src/control.c|control_report_invalid_fault}} |
| 新帧资格与PIT保护/时限 | {{src|user/src/control.c|control_publish_vision}}；{{src|user/src/control.c|control_tick}} |
| STOP事件优先与硬件施加 | {{src|user/src/wifi_assistant.c|assistant_parameter_event}}；{{src|user/src/wifi_assistant.c|apply_parameters}}；{{src|user/src/isr.c|PIT_IRQHandler}} |

## 双轮速度 PI

### 从本周期编码器反馈到PWM

主速度环实际调用位置式`PidLocCtrl`，由同一次10ms PIT分别计算左右轮。输入不是图像误差，而是本轮经过降速/斜坡的有效目标减本轮带符号编码器增量。主控制层把速度Kd固定为0，因而当前是PI。

```text
PIT分别读编码器并清零
  → control_tick执行保护
  → 基础目标×降速因子，经斜坡得到左右有效目标
  → e_left/right=有效目标-本轮编码器反馈
  → 独立PidLocCtrl，dt_s=0.010秒
  → 0..pwm_max的浮点输出截为uint32，再以float存快照
  → ISR写前进DIR和左右PWM
  → 电机/传动产生新的编码器计数，进入下一次PIT
```

这里没有电流反馈内环、模糊增益调度或速度微分。默认Kp=50、Ki=0、积分/输出上限5000是源码配置起点，不能当作实车闭环已调好；`PWM_DUTY_MAX=10000`时5000对应配置满量程的一半，不是5000%。

### 位置式各项与状态单位

设误差为e、周期秒数为dt、上次积分贡献为I：

```text
P = Kp × e
D = Kd × (e-e_prev) / dt                 （主速度环Kd=0）
ΔI = Ki × e × dt
I_candidate = clamp(I+ΔI, -imax, +imax)
raw_candidate = P+I_candidate+D
```

`integrator`直接保存PWM输出贡献，`out_i`在提交后等于它；它不是累计原始误差，也不是未乘Ki的积分。Kp把counts/10ms误差变为PWM贡献，Ki再按秒积分。10ms必须传0.010秒；若错误传10，同一误差积分增量会放大1000倍。

输入检查要求指针非空、误差和dt有限、dt>0；kp/ki/kd/imax/outmax均有限非负；被读取的积分、上次误差、增量差分和旧输出历史也必须有限。随后还检查P、D、ΔI、未限幅积分和以及最终总和，因为有限输入相乘仍可能溢出。任一异常路径调用`Pid_Reset`并返回0；PID接口本身不把控制状态设置成FAULT，状态层的FAULT有自己的显式触发入口。

Ki=0先清掉位置式旧积分贡献，重新启用Ki不会释放关闭期间残留的误差积分。Kd=0直接给微分项0，跳过除法。`Pid_Reset`清输出、积分和误差历史，保留增益/限幅；控制层在停止、故障、应用参数及停止态每tick执行相应清历史路径。左右结构体互相独立。

### 条件积分为何需要候选与边界两步

不能只把输出裁到上限，同时让积分无限增加。当前实现先形成候选积分和候选raw，再判断“是否越界且继续往越界方向积分”：

| 候选总量和误差 | 积分处理 |
| --- | --- |
| raw>outmax且e>0 | 拒绝继续上推的整步积分；必要时只前进到上边界 |
| raw<0且e<0 | 拒绝继续下推的整步积分；必要时只后退到下边界 |
| 未越界，或误差方向有助于退出当前饱和 | 提交候选积分贡献 |

若候选刚跨越边界，完全拒绝整步会使输出停在边界前。于是反求`I_at_limit=边界-P-D`，上边界取outmax，下边界取0：正误差仅在I_at_limit大于旧I时提交；负误差仅在它小于旧I时提交，且仍限制在±imax。已经在边界外继续同向时保留旧I，不为追逐边界而反向改写积分历史。

最后发布真正提交的`out_i`，重算`out_raw=P+I+D`，将`out`裁到0～outmax并保存本次误差供下次使用。`out_raw`是最终提交后的诊断量，不是可能被拒绝的raw_candidate。控制层以同一个整数命令写快照和硬件，避免遥测浮点值与实际整数写入不一致。

### 上限、下限与恢复的可核算算例

下表全部为教学参数与状态，不是实车测量：Kp=2、Ki=10、Kd=0、dt=0.010s、outmax=100、imax=30。积分贡献和输出单位均为PWM命令，误差单位counts/10ms。

| 场景 | 旧I与本次e | 候选与边界处理 | 最终I / raw / 输出 |
| --- | --- | --- | --- |
| 未饱和 | I=0，e=10 | P=20，ΔI=1，候选I=1，raw=21，直接提交 | 1 / 21 / 21 |
| 一步刚越上限 | I=9.5，e=45 | P=90，ΔI=4.5，候选I=14、raw=104；反求边界I=100-90=10，只走到10 | 10 / 100 / 100 |
| 继续同向推动上限 | I=10，e=45 | 候选I=14.5、raw=104.5；边界I=10不大于旧I，保持积分 | 10 / 100 / 100 |
| 比例需求下降，退出上限 | I=10，e=20 | P=40，ΔI=2，候选I=12、raw=52，可正常提交 | 12 / 52 / 52 |
| 负误差释放积分且总输出仍正 | I=10，e=-1 | P=-2，ΔI=-0.1，候选I=9.9、raw=7.9，允许下降 | 9.9 / 7.9 / 浮点7.9，控制层写7 |
| 一步触下限 | I=10.2，e=-5 | P=-10，ΔI=-0.5，候选I=9.7、raw=-0.3；边界I=10，小于旧I，只退到10 | 10 / 0 / 0 |
| 负需求已在下限外 | I=0，e=-5 | 候选I=-0.5、raw=-10.5；边界I=10不小于旧I，保持旧I | 0 / -10 / 0 |

上述场景分别给定旧状态，不能把表的所有行当作一段连续采样轨迹。Ki改0后，本次旧I直接变0。例如旧I=10、e=10时只剩P=20，输出20；再次开Ki从清零历史起算。

快照饱和标记在`raw<0`时置位，或非零pwm_max且整数命令达到上限时置位。因此“最终raw恰100、输出100”也报上限触顶；恰到下限raw=0不报下限触顶。pwm_max=0不会按上限触顶报饱和，负raw仍可报下限约束。饱和标记不是电机堵转传感器。

### 兼容接口与现场限制

`PidIncCtrl`保留正向增量式兼容：在旧实际输出上累加ΔP/ΔI/ΔD，imax只限本次ΔI，越界同向时整步舍弃该ΔI；它没有位置式“积分只走到边界”的同一状态规则。`last_derivative`保存的是上次误差差分，不是每秒导数。该接口不在当前主速度链中使用，不能与位置式混用同一历史，切换前必须复位。

当前位置式输出下限为0；超速导致负误差时可以减小或关闭正向PWM，但不会反转DIR产生主动反向制动。真实速度跟踪、负载变化、饱和恢复和停距须架空/实车检验。BENCH每次人工试验最多8秒逻辑时间，当前运行不改运动参数；先停车、提交下一组、再人工重启，不能把10Hz波形当作完整100Hz评分日志。

| 阅读目的 | 源码入口 |
| --- | --- |
| 本轮目标、反馈、PI和命令量化 | {{src|user/src/control.c|control_tick}}；{{src|user/src/isr.c|PIT_IRQHandler}} |
| 初始化与清历史 | {{src|user/src/pid.c|Pid_Param_Init}}；{{src|user/src/pid.c|Pid_Reset}} |
| 条件积分主算法和门槛 | {{src|user/src/pid.c|PidLocCtrl}}；{{src|user/src/pid.c|pid_input_valid}}；{{src|user/src/pid.c|pid_integral_allowed}} |
| 兼容增量式与幅值限制 | {{src|user/src/pid.c|PidIncCtrl}}；{{src|user/src/pid.c|constrain_float}} |
| 最终硬件占空与重载 | {{src|../libraries/zf_driver/zf_driver_pwm.c|pwm_set_duty}}；{{src|../libraries/zf_driver/zf_driver_pwm.c|pwm_init}} |

## 舵机控制与降速

### 新帧转向目标与10ms执行命令分开

只有RUNNING/TRACK的可用新帧计算转向目标；READY、FAULT和BENCH都不把视觉误差变成新转向目标。输入是当前新帧误差、上一有效转向误差及相邻采集逻辑时间差，输出是配置限位内的`steering_target`。

```text
主循环发布可用新帧
  → 按帧采集时间差得到误差变化率
  → center-direction×(Kp×像素误差+Kd×变化率)
  → 限制到servo_min..servo_max，保存steering_target
随后每个PIT
  → servo_cmd向steering_target靠近，最多rate×0.010命令度
  → ISR按servo_enabled换算脉宽并写PWM
  → 50Hz硬件在重载点采用命令；前轮机械响应另测
```

启动只清微分资格并进入RUNNING，启动前好图用于资格检查；转向目标在启动后的可用新帧才更新。因此第一轮PIT可能仍向中位保持，不能把“有三帧资格”解释为“启动前已经施加了这三帧的转向命令”。BENCH即使收到视觉结果，也只在PIT让允许输出的舵机向中位靠近。

### P/PD、符号与微分资格

计算公式为：

```text
de_dt = (e_now-e_prev)×1000 / frame_dt_ms
servo_target_raw = servo_center
                 - servo_direction×(steer_kp×e_now+steer_kd×de_dt)
steering_target = clamp(servo_target_raw, servo_min, servo_max)
```

误差正值表示加权中点在画面左侧；到底对应前轮向哪侧转，还取决于人工标定的方向与连杆。默认center=92.5、min=85、max=100、direction=-1、Kp=0.5、Kd=0，因此e=+10时数学目标97.5命令度，e=-10时为87.5，e=+20时原始102.5被限制到100。这些是源码配置下的算术推演，不是实测转角或R50证明。

微分只在`derivative_ready!=0`、相邻采集时间差>0且<=100逻辑ms时使用。首个启动后可用帧、失效之后的首个可用帧，以及量化到同一采集时刻的两个不同帧，都保持D=0。重复帧不会刷新历史；可用转向帧计算结束后才保存本次误差并建立微分资格。

教学PD例子：若Kp=0.5、Kd=0.02、上一误差4、本次误差10、采集差20ms，则变化率=(10-4)/0.020=300像素/s；P项5命令度、D项6命令度。默认direction=-1和center92.5下原始目标103.5，限为100。若这是启动后的首帧，则D按0，目标只有97.5。示例Kd并非当前默认设置；当前采集时间差来自10ms逻辑量化，不能把这300像素/s当成独立真实时基测量结果。

转向目标若非有限数会锁`INVALID_VALUE`；正常合法参数和有限误差先经过限位，再执行后续变化率限制。舵机允许输出字段属于软件门控，不能当作“舵机已经到位”的传感器。

### 命令变化率如何作用

PIT使用`approach(value,target,step)=value+clamp(target-value,-step,+step)`，其中`step=servo_rate_deg_s×dt_s`。当前默认60命令度/s、dt_s=0.010s，每次最多变化0.6命令度，距离目标不足0.6时直接到目标，不越过它。

例如当前命令92.5、新目标100，假设后续新图持续可用、目标保持100且不触发保护，教学推演前两次为93.1、93.7；到第13次才到100，约130ms逻辑控制时间。新图即使立刻给出100，也不会绕过这个限速。50Hz硬件每20ms产生一周期脉冲，两次10ms命令更新可能只体现为一个脉冲采用的最新值；0.6度/tick不能写成0.6度/物理PWM周期，更不能当机械角速度实测值。

停止或故障直接清PWM门控，跳过这条靠近目标的路径；软件servo_cmd记中位不代表停机过程中执行了一次回中动作。

### 弯道与低可信度怎样共同影响速度

TRACK在PIT读最近可用视觉误差，先把绝对误差归一化到0～1，再得基础目标缩放因子：

```text
bend = clamp(abs(error_px)/30, 0, 1)
factor = 1-bend×(1-curve_speed_factor)
若confidence<0.6或lost_frames>0：factor=min(factor,0.5)
desired_left/right = config.target_left/right×factor
有效目标每次向desired靠近，步长=accel_counts_per_s×0.010
```

默认弯道因子0.5时，误差0、15、30像素对应factor=1、0.75、0.5；再大的误差仍为0.5。若配置因子0.2且误差30，则factor=0.2；低可信规则只再取min(0.2,0.5)，不会把已经更低的目标抬回半速。因子1关闭这条按误差连续缩放，但低可信/短失线的半基准约束仍有效。

教学例子：两轮基础目标20，误差15、可信度0.8，最终期望目标均15；若可信度0.5而结果仍可用，期望目标降至10，且该帧不能延续>=0.6的启动资格。如果当前有效目标仍20，默认斜坡每tick最多减0.5：下一轮为19.5，不会立即成为10。在后续新图持续可用、期望目标保持10且不触发保护的假设下，约20次标称tick才靠近10。最后可用误差被保留的短失线也走这个限制，第三个新不可用结果或年龄超时则直接零输出，不经减速斜坡。

基础参数20始终保留，快照有效目标19.5/19.0等才是本次PI输入。左右可有独立基础目标，但当前代码对两轮使用同一视觉factor，没有按轮距、转向角另作几何差速补偿。`abs(error)/30`是图像偏差的降速代理量，不是经过车辆几何标定得到的真实道路曲率。

### 从命令度到舵机PWM及章内索引

`SERVO_MOTOR_DUTY(x)`将命令度换算成高电平宽度`0.5+x/90`ms，再换成PWM占空计数：

```text
pulse_ms = 0.5+x/90
period_ms = 1000/SERVO_MOTOR_FREQ
duty_count = PWM_DUTY_MAX×pulse_ms/period_ms
```

代码已配置50Hz和PWM_DUTY_MAX=10000，90命令度数学对应1.5ms、占空计数750；默认中位92.5对应数学脉宽约1.52778ms、计数约763.89，ISR截为763再交底层16位占空转换。默认85～100命令度对应约1.44444～1.61111ms数学脉宽；图中的1.5ms中位示例不能取代实车92.5配置或实际波形。

宏本身不限位，控制层先按配置范围约束命令；配置范围仍需确认与机械安全极限一致。寄存器写入、50Hz重载采用、舵机轴运动和前轮实际角度是不同事件。M3应核对方向、物理中位、限位、响应和R50转弯半径；当前软件与算例不能提供这些实机结论。

| 阅读目的 | 源码入口 |
| --- | --- |
| 新帧资格、P/PD目标与历史 | {{src|user/src/control.c|control_publish_vision}} |
| 限位、每tick变化率与降速因子 | {{src|user/src/control.c|clamp_value}}；{{src|user/src/control.c|approach}}；{{src|user/src/control.c|control_tick}} |
| 默认参数与停止门控 | {{src|user/src/control.c|control_init}}；{{src|user/src/control.c|reset_outputs}} |
| 命令施加和硬件重载 | {{src|user/src/isr.c|PIT_IRQHandler}}；{{src|../libraries/zf_driver/zf_driver_pwm.c|pwm_set_duty}}；{{src|../libraries/zf_driver/zf_driver_pwm.c|pwm_init}} |
