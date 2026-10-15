### 软件检查实际执行哪些函数

```text
run_software_checks.main()
  → compile_test(test_pid / test_vision / test_control)
      → 编译真实模块C及测试入口 → 执行 → 保存compile.log/result.log
  → 实际WIFI函数/FIFO/解析器/寄存器模型场景
  → main真实标定函数与camera实际源模拟场景
--keil-only分支
  → keil_build() → 隐藏启动Keil全量重编译 → 核对退出码与0错误0警告
```

宿主编译优先GCC，当前受限环境不能启动GCC时回退本机MSVC；测试使用同一控制/视觉/PI C实现，不是另写一套数学模型冒充车端行为。WIFI测试提取真实函数并配SPI寄存器/FIFO模型，证明条件和处理顺序，不能证明模块电气时序或无线收发。每个检查日志明确软件边界。

### 图像回放如何保持算法一致

replay_vision的build_library编译实际img_process.c为动态库，configure安装控制带/可选宽度并复位跨帧状态。main按图像文件顺序或synthetic_frames输入，调用同一get_hist_gram、get_threshold_otsu、binaryzation_process、vision_process，保存阈值、帧号、误差、可信度、控制行质量及可选二值PGM/逐行JSON。输入RAW/PGM无需图像依赖，PNG/JPEG使用可选Pillow。

合成两种局部阴影由check_shadow_case按预期逐项核对；已有14帧回放为5有效9失效，可辨认阴影通过，深阴影输出valid=0/track_lost=1等不可用视觉结果；该回放未调用整车控制/PWM，不能把全部回放都写成真实阴影鲁棒性或停车保护验收。回放时间标签由--period-ms构造，默认20ms，为离线假设；hardware_acceptance=false。实拍文件离线回放也不自动证明现场轮速、PWM或完整路线通过。

### 索引及图形工具的边界

update_report_index对每个章内函数取得真实定义行，更新可见函数名/行号和相对#L链接；全局表继续保留主要入口并自动收录正文引用的补充函数。C源码忽略注释/字符串中的伪定义，Python工具用语法树定位def；函数缺失或存在多个定义时拒绝生成，避免悄悄猜行号。章节正文的算法含义仍由主代理review，不由生成器自动编写。

render_control_timeline的build_model/state_at构造教学事件模型，输出总体/10ms窗口SVG及离线HTML，数值分类和边界仍见时序章。它不连接车、不测CPU耗时；M3独立计时和M4完整日志另有门槛。原六Mermaid源码、三份图形资产与已验收渲染/交互证据保持。

### 本章代码索引

| 工具入口 | 输入/产物与证明范围 |
| --- | --- |
| {{src|tools/run_software_checks.py|main}}、{{src|tools/run_software_checks.py|compile_test}}、{{src|tools/run_software_checks.py|keil_build}} | 同一C模块宿主验证与当前Keil全量构建 |
| {{src|tools/replay_vision.py|build_library}}、{{src|tools/replay_vision.py|configure}}、{{src|tools/replay_vision.py|main}} | 动态库、参数安装、离线逐帧回放 |
| {{src|tools/replay_vision.py|check_shadow_case}} | 两类合成阴影的逐项预期检查 |
| {{src|tools/update_report_index.py|definition_line}}、{{src|tools/update_report_index.py|refresh_chapter_links}}、{{src|tools/update_report_index.py|main}} | 定位真实定义、刷新章内链接与生成总表 |
| {{src|tools/render_control_timeline.py|build_model}}、{{src|tools/render_control_timeline.py|state_at}} | 教学时序模型与查看时刻的帧来源推演 |
