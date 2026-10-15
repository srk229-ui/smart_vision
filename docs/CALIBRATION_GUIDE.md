# 舵机转角与车轮 RPM 标定操作说明

当前源码（2026年10月14日，R026）：KEY1启动循迹、KEY2停止，本地默认20/20 counts/10ms；上电零输出。普通循迹取消三帧/远程确认和通信异常停车，视觉统一最后有效图像超过500逻辑ms停车，保留低可信/短丢线半速。曝光调整及二值记录增量已回退。本轮未烧录或发车，实机验收仍待完成。证据见[evidence/M3/local_button_control_20261014](evidence/M3/local_button_control_20261014)。

板上版本实读确认（2026年10月14日，R027）：本轮DAP只读Flash逐字节匹配本地按键正式版b12069b70cb851748425d63960d2b67e0c25b1fc3bcab2fac6941df0d267f296，CPU运行；此前“最后仅曝光版有烧录证据”描述是当时记录，现由此次实读更新。本轮未烧录。Wi-Fi助手监听已恢复，但模块B17持续低、初始化就绪超时仍待检查接线/断电复测。见[本轮记录](evidence/M3/wifi_recheck_20261014_01/diagnosis_progress.json)。

本轮软件提供 pyOCD 运行态标定、原始记录导出和候选配置生成。用户已授权将左右各一组10圈平均N和右前轮三点部分表用于初步换算；完整正反重复、实测滚动周长、实测零角与左前轮仍待测，完整双轮表保持为空。下列现场步骤由操作者执行，软件测试不能代替机械、探针、GPT、实际PWM或路线验收。

普通行车只需有效图像后按KEY1；KEY2和已连接上位机零目标可停止。V1正目标仅在停车时设置速度，通道3的1/2不再选择发车模式。架空BENCH/RPM仍通过下述诊断工具明确准备、确认并START。按键操作不会直接启动诊断试验。

## 2026年10月14日源码参数更新

当前主工程默认左右Kp=17.5、秒制Ki=480，控制/V1/RPM目标上限统一为400 counts/10ms，兼容本轮0～3m/s换算范围。3m/s左右目标约348.146/340.820；V1目标仍是counts/10ms且共用左右值，RPM准备允许分轮。本次仅补换算接口，不迁移PI单位或应用右轮部分表到视觉转向，8秒及其他保护保持。本节源码更新当时未烧录；当前正式版烧录状态见R020，尚未发车。

## 已接入的换算接口

`calibration_profile.c`提供左N=2333.3、右N=2284.2及名义64mm轮径对应的周长0.20106193m，上电在PIT启动前装入测量模块。每圈计数来自本轮起终点净计数除以操作者确认的10圈；不能把两轮N合并后用于精确分轮速度。名义周长下输出是轮周速度估计，实际地速仍须独立测量。

```c
float left_counts, right_counts, speed_mps, right_deg;
// 后续调参以m/s描述期望，按标称10ms分别换成现有PI的计数目标；换算本身不发车。
encoder_measurement_mps_to_counts(ENCODER_WHEEL_LEFT, 0.5f, 10000u, &left_counts);
encoder_measurement_mps_to_counts(ENCODER_WHEEL_RIGHT, 0.5f, 10000u, &right_counts);
// 两个目标约58.0244和56.8034；提交前须检查函数返回1及现有控制上限。
// 测量例子：左轮本窗口100计数、实际9000us；有方向的净计数才能用于换算。
encoder_measurement_counts_to_mps(ENCODER_WHEEL_LEFT, 100.0f, 9000u, &speed_mps);
// 仅查询右前轮静态表，1550us估计约+8.428571deg，不设置舵机输出。
steering_wheel_map_us_to_angle(&calibration_profile_right_steering_map, 1550.0f, &right_deg);
```

四个查询均须检查返回值；失败保持输出原值。实际采样优先读取`EncoderMeasurementSnapshot`的`speed_left_mps`/`speed_right_mps`、`speed_left_mps_100`/`speed_right_mps_100`或`speed_left_mps_500`/`speed_right_mps_500`，并检查对应`speed_valid`、`speed_valid_100`、`speed_valid_500`，占位0不能当静止证据。旧RPM字段和邮箱ABI保持兼容，现有pyOCD仍按原字段读RPM和N；新增m/s通过C接口/测量快照读取。

右轮表已按操作者更正为1444us/-13deg、1526us/+5deg、1610us/+17deg，只允许1444～1610us内插值，不输出左轮角。反查0deg约1503.222us是估计值，名义2us网格1504us预测约+0.170732deg，不自动改舵机中位。正角为左转，脉宽增大向左偏，当前SERVO_DIR=-1保留；[俯视方向示意图](visualizations/steering_sign_top_view_20261014.png)已同步更正。可复跑原件核对：`python tools/check_measured_profile.py`；默认写current_measured_profile，保留旧冻结证据。工具使用既有speed_pi_sim隔离环境，不安装依赖。来源、84行名义2us插值表及真实C结果见[更正后的换算审计](evidence/M3/servo_points_correction_20261014/measurement_audit.json)。

本轮更正角度关联的新固件另存[evidence/M3/servo_points_correction_20261014/builds](evidence/M3/servo_points_correction_20261014/builds)。下文旧冻结路径表示历史版本；使用新版时必须同时选新版AXF和相同构建的工具/记录，不能只改源码就认为正在运行的旧固件已更新。现场各阶段验收要求继续有效。

## 1. 选择固件并建立连接

在 `D:\镜头组\demo` 中执行：

```powershell
python tools/run_calibration_checks.py --evidence-dir docs/evidence/M3/count_session_20261013/tests
python tools/build_calibration.py --output-dir docs/evidence/M3/count_session_20261013/builds
python -m pyocd list
```

构建工具只编译，不连接探针或烧录。三个目标独立保存产物：

| Keil 目标 | 用途 | 冻结固件 |
| --- | --- | --- |
| nor_sdram_zf_dtcm | 正式控制；默认不编译标定命令和32KB记录区 | [正式版](evidence/M3/count_session_20261013/builds/nor_sdram_zf_dtcm/firmware.axf) |
| calibration_readonly | 首次运行态验证；编译关闭两电机及舵机输出，可做被动手转计数 | [关闭输出诊断版](evidence/M3/count_session_20261013/builds/calibration_readonly/firmware.axf) |
| calibration_active | 人工确认后允许舵机单点和架空 BENCH；上电仍零输出 | [人工标定版](evidence/M3/servo_session_20261013/builds/calibration_active/firmware.axf) |

先由操作者烧录关闭输出诊断版，保持 CPU Run，退出 Keil 对探针的占用。用实际探针精确序列号替换下例 `PROBE_ID`：

```powershell
python tools/calibrate_pyocd.py status --elf docs/evidence/M3/count_session_20261013/builds/calibration_readonly/firmware.axf --probe PROBE_ID
```

工具固定使用本机已核对的 pyOCD 0.45.1、`mimxrt1064`、`attach`；禁用自动解锁、项目调试脚本、读缓存和断开自动恢复。不下载、不复位、不暂停、不单步。它检查 ELF 全局对象、ABI、构建字符串、DTCM 地址和记录区尾部可读性，并要求 GPT 时间与采样序号继续推进。身份变化或探针异常会结束会话，不自动重连或重放启动。

`status` 与独立 `export` 不发送心跳或执行器命令。诊断邮箱为416字节，记录区为1024×32字节；最终地址以本次 ELF 为准，不能沿用上次 Watch 地址。关闭输出版需完成命令/快照往返、实际零 PWM、半写拒绝、断线和 STOP 验证后，才进入人工活动版实测。

所有正式测量集合使用同一活动版固件。换固件后另建集合，不能把不同构建的手转和角度证据拼成同一配置。关闭输出版的数据可以单独留作连接和计数检查证据。

## 2. 交互入口及停止方式

手转推荐使用计数专用入口：

```powershell
python tools/calibrate_pyocd.py interactive --elf docs/evidence/M3/count_session_20261013/builds/calibration_readonly/firmware.axf --probe PROBE_ID --count-only --skip-save --recovery-dir calibration_records
```

该入口中人工输入 `start` 即准备并开始无输出手转，`end` 在两轮连续静止100ms后结束并显示左右净计数，单组没有固定时长。`restart` 会先提交STOP、确认本次STOP已消费和零输出冻结，再丢弃当前未导出的手转组并重新开始；不要求先export，也不会删除已导出的文件。开启 `--skip-save` 后，下一次 `start` 也可跳过上一组未保存数据。窗口中可用 `set skip-save off` 恢复必须先导出的门槛，`set skip-save on` 再开启。有效测量仍须 `count-note` 和 `export` 才能进入集合；被丢弃的未导出组无法恢复。`quit` 仍自动保存当前未导出组。

诊断固件只在上电检查Wi-Fi，不自动重连缺席模块或进行整图传输，已有V1连接仍以有界收发处理STOP。手转不依赖Wi-Fi连接；交互attach等待主循环就绪再接收测量命令。命令ACK超时与测量时长不同：旧诊断版可能在READY手转时进入停车Wi-Fi重连，阻塞主循环超过主机0.75秒确认窗口；须使用本节配套的新固件与工具，不能只更新电脑端。独立status/export仍为只读，不发送心跳。

下面为舵机专用交互入口；它拒绝电机试验，继续使用独立人工确认。RPM须去掉`--servo-only`进入完整入口：

```powershell
python tools/calibrate_pyocd.py interactive --elf docs/evidence/M3/servo_session_20261013/builds/calibration_active/firmware.axf --probe PROBE_ID --servo-only --recovery-dir calibration_records
```

| 交互命令 | 作用 |
| --- | --- |
| status | 看运行状态、真实dt、累计计数、有效RPM和名义脉宽 |
| prepare count | 准备手转计数；不输出 |
| prepare servo 中位us 最小us 最大us 变化率us每秒 | 准备人工已确认的舵机范围；不输出 |
| prepare rpm 左目标 右目标 | 准备架空轮目标，单位counts/10ms；不启动 |
| start | 完整入口显示准备内容并另输入RUN；计数专用入口直接开始手转 |
| restart | 停止并确认零输出冻结，丢弃当前未保存手转/舵机组后重开；舵机重用已确认范围并另输入RUN，RPM拒绝 |
| set skip-save on或off | 计数专用入口设置下一次start是否跳过未保存组 |
| pulse 脉宽us | 设置已启动舵机单点；新版舵机没有固定时限 |
| end | 舵机正常结束并确认实际零输出周期冻结；手转仍须静止100ms |
| stop | 独立高优先级停止；逐飞助手 STOP 也有效 |
| clear | 人工清除已排除原因的可恢复故障；不启动 |
| release | 取消准备并恢复临时参数；不启动 |
| query-us 脉宽us | 查询标定表的左右轮角估计值 |
| query-angle left或right 角度deg | 指定前轮反查脉宽；不输出 |
| apply-counts 集合.json | 单独验证左右每圈计数证据，人工确认后停车应用候选 N |
| angle 左角 右角 increasing或decreasing 1或2或3 table或validation [仪器脉宽us] | 记录活动单点的人工实测角 |
| count-note left或right forward或reverse 圈数 | 正常手转结束后注明准确圈数 |
| export 路径.json | 停止冻结后保存 JSON 和同名 CSV，不能覆盖旧组 |
| collect 集合.json | 汇总已经导出的人工条目；来源默认未确认 |
| quit | 提交停止并退出；保存尚未导出的原始组和人工条目 |

探针读写由单一工作线程执行。等待人工输入时仍每100ms发送新心跳；固件以GPT时间在500ms失联时停车，RPM组最长8秒，手转和舵机组无固定时限，由start/end控制；STOP、500ms心跳失联及测量异常仍可终止。重新连接、清故障、查询和读取记录不重新启动。

测量过程中保持 CPU Run；暂停 CPU 后软件无法执行超时保护，计数与采样时间也会失配。保留现场独立停止方式。只有修复配置并重新初始化才能解除 GPT/profile/计数模式初始化故障，普通 `clear` 不能旁路。

## 3. 实测左右轮每圈计数

1. 两类输出保持关闭，车轮做唯一位置标记。沿测试方向轻转预紧，再静止。
2. 在计数专用入口输入 `start`；完整入口仍可 `prepare count`、人工确认、`start`、`RUN`。起点要求两轮连续100ms严格零计数。
3. 指定车轮准确转10圈；静止100ms后 `end`。此期间普通 TRACK/BENCH 启动和参数写入被独占准备权拒绝。
4. 输入 `count-note left forward 10` 等实际轮侧与方向，随后 `export`，再测下一组。
5. 左右轮各做前进/反转3组，共12组，分别保存原始起终点累计计数。用 `collect` 汇总并补齐操作者、日期和测量方式。
6. 各轮每组 `N=abs(净计数)/10`，所有组相对中位数偏差不超过1%，方向符号必须正确。用 `apply-counts` 应用该集合，不必等待转角表完成。

N为输出车轮一圈的有效计数，已经包含安装轴与传动机构影响；不额外乘减速比，不默认厂家 PPR×4。当前驱动为 A 上升沿计数、B提供方向。控制继续使用counts/10ms，物理RPM另按真实窗口时间换算：

```text
RPM = 净计数 × 60000000 / (N × 窗口us)
```

窗口分别使用最近10/50个有效样本，不假定真实时长恰为100/500ms。N未标定或窗口未填满时，界面显示null/未标定，不能把占位0解释为静止。

## 4. 实测脉宽与前轮转角

固定车体与供电、关闭电机，量角板或俯拍建立车体纵轴基准。分别记录两前轮，不用舵机轴角代替前轮角；以直行姿态为零、左转为正，直行前束另记。

原配置中位92.5、范围85至100仅是电气刻度；50Hz、10000占空刻度下，原输出名义中位约1526us、边界1444至1610us，均不是机械安全实测值。`prepare servo` 要填写操作者实际确认的中位、安全范围和变化率，范围只能收窄现有基线。直接在确认范围内以约10us步长采点，曲线变化明显处加密。

先`prepare servo`并输入`CONFIRM`确认范围；每次人工 `start` 后另输入`RUN`，用 `pulse` 设置一点，等待机械到位再测左右角并用 `angle` 录入。记录工具自动关联当下样本、请求us、整数duty和名义us；可另外填仪器实测us。增脉宽、减脉宽各重复3次，中位必须有实测节点，另取不参与建表的中间验证点。用`end`正常停止后`export`保存，再`start`重用已确认范围；操作错误可`restart`，无需保存本组，确认本次STOP与实际零输出冻结后丢弃原始组和本组角度笔记，再另输入`RUN`。已导出文件与已汇总条目保留。

新版固件feature位3标识舵机无时限、SERVO_END和事件记录。日志只在首次、显式pulse后的首周期、实际整数duty变化、最终零输出周期留档；稳定保持不占容量，慢速斜坡也不按float每周期变化记满。最多1023个活动事件，另留一槽给停止；确实耗尽事件容量时仍保护停止，可导出或restart后另组测量。JSON标明`recording_mode:servo_events`，事件行累计计数为null，整组净计数仍来自完整起终点；GPT/elapsed仅保留低32位模差，不能当连续PWM波形或跨多次回绕的完整时长。

本地独占SERVO_CAL由pyOCD的100ms心跳/500ms失联保护；缺席Wi-Fi或该链路错误不会停止本地舵机测角，已连接V1的STOP仍有效。Wi-Fi故障不再停止普通TRACK/BENCH/RPM；诊断pyOCD心跳、测量异常及独占规则保持。`angle`会拒绝名义输出尚未到达本次目标的记录；机械是否到位仍由操作者确认。

有效表3至17个节点，脉宽严格递增，两轮角分别严格单调且方向一致。候选生成保留原始角/前束/中位偏置，节点扣除实测直行偏置；双向回差及重复波动超过2°、中间点误差超过2°时拒绝有效单表，不通过平均隐藏明显回差。正反查询使用分段线性插值，非法和越界输入拒绝，不外推。

SERVO_CAL两电机目标/PWM/积分始终为零。实际整数占空只能落在确认范围的ceil(下限)至floor(上限)网格内，避免向下截断越机械下限。普通TRACK/BENCH保留原脉宽公式、限速和量化。查询是实测表估计，没有实时角传感器；关脉冲后不能用中位软件值推断前轮已回中。

## 5. 手机慢动作核对 RPM

应用实测N后架空试验，依次 `prepare rpm 5 5`、`10 10`、`20 20`，每档至少两次，操作者每次单独启动。不适合当前车轮的视频辨圈速度可降低目标，不能延长8秒时限。

手机同时拍轮上唯一标记和TFT下方预留的小色块（坐标112,112，16×16像素）。固件在约2s、6s由主循环翻转颜色，记录绘制调用开始/结束时间；它们是软件时戳，不是精确发光时刻。视频选择同一位置穿越的起末帧，中间为准确完整K圈。采用实际采集fps，不能使用慢放播放fps。

独立参考文件填写：

```json
{
  "capture_fps": null,
  "start_frame": null,
  "end_frame": null,
  "complete_turns": null,
  "wheel": "left",
  "counts_per_rev": null,
  "marker_frames": [null, null],
  "endpoint_frame_uncertainty": 1
}
```

将null替换为真实读数，独立保存原视频，然后执行：

```powershell
python tools/calibrate_pyocd.py rpm-reference --export 本组原始.json --reference 视频读数.json --output 本组分析.json
```

工具按两次同步事件映射同一时间段，核对日志版本/N、连续样本与累计计数，并计入采样端点、视频帧和绘制延迟。误差门槛取参考RPM的3%、两计数量化及端点不确定度的最大值。这是拟定工程验收目标。不能可靠辨圈、同步时间或稳态区间不足时保留证据不足，不据此关闭RPM实测。

## 6. 生成与冻结配置

`collect` 生成的集合包含encoder_runs与servo_samples，每条关联本组原始export。来源默认 `real_measurements:false`，只有操作者核对全部真实记录后才填写true。至少补齐：操作者、日期、编码器/测角方式、两轮直行前束、同版firmware_build_id、实测center_us及版本。无真实测量时保持未标定。

```powershell
python tools/calibrate_pyocd.py profile --measurements 完整测量集合.json --output calibration_records/候选配置.c
```

工具验证12组圈计数、重复性、正反方向、每个角节点双向3重复、独立中间点及原始日志关联，再生成候选C和审计JSON。不能直接覆盖默认profile或输入集合；复核通过后再由操作者/主代理合入 `calibration_profile.c`，重新完整构建、人工烧录和验证。没有Flash写入或自动应用转角控制。

退出时工具尝试停止、等冻结并保存未导出记录。探针失联后无法读回时，失败原因需保留，重新连接不自动恢复试验。已导出的原始失败组不删除、不覆写；默认完整入口准备下一组前仍须导出当前冻结组；舵机显式restart可跳过保存并重用本窗口已确认的范围。计数专用入口开启skip-save或显式restart后，按操作者要求丢弃未导出的手转组再重开。

本轮属于M3现场标定准备，不实现M4的电脑桥、4096条完整实验日志或自动评分。R50、路线、实际PWM停止延迟和机械精度仍需分别验收。
