#include "main.h" // 所有引脚信息更改在main.h里改宏
#include "vision_record.h" // 停车冻结取证，不在中断写图像。
#include "vision_track.h" // 主循环普通道路恢复和独立微秒耗时记录，不进入PIT。

// *************************** 例程硬件连接说明 ***************************
/*

            连接好电机、主板、驱动板、摄像头、编码器
            (请确保全部正确连接后在使用此例程，所有外设均会使用)
*/
// *************************** 例程测试说明 ***************************
/*
            本例程包含功能：
            1.初始化摄像头并采集图像进行大津法二值化
            2.初始化屏幕并显示关键信息
            3.加权视觉误差、置信度和有界舵机 P/PD 转向
            4.十毫秒双轮 PI、上电停止、人工启动与故障锁存


            注意！注意！注意！注意！
            注意！注意！注意！注意！
            注意！注意！注意！注意！

            1.请仔细阅读main.h的所有注释，确保所有宏都和自己的硬件连接对应
            2.若屏幕显示二值化后的图像有缺陷，可能不能正常走直线，
            可以自行更改二值化算法或找光环境良好且均匀的环境测试例程（例程不提供更多的二值化算法）

*/

// **************************** 代码区域 ****************************

// 舵机中位采用现有配置作为初始值，实车仍需人工标定。
const float servo_motor_duty_middle = SERVO_MOTOR_CENTER;

static uint8_t image[MT9V03X_H][MT9V03X_W]; // main持有的完整快照，先灰度后原地二值化；不直接在DMA缓冲上计算。
static uint32_t hist_gram[256]; // 256灰度桶，32位统计，由每帧直方图函数完整清零。
static uint8_t vision_clock_ready; // GPT初始化成功才把步骤时间标为有效，不改变初始化故障策略。
static uint8_t display_lines[CONTROL_IMAGE_HEIGHT*3u]; // 只供显示，不能回流控制。
static VisionResult vision; // 本帧边线/质量/误差；数组为static，避免在中断/栈上放120行结果。

// 按键事件由PIT单写，主循环在短临界区领取；STOP代际变化会使旧START失效。
static volatile uint8_t local_start_requested; // 尚未消费的KEY1事件，供联网等待取消钩子只读。
static volatile uint8_t local_input_ready; // 相机包装与视觉初始化完成后才接收启动事件。
static volatile uint32_t local_start_generation; // 按下启动时的STOP代际，拒绝跨停止事件复用。

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     在5ms中断扫描本地按键并优先处理停止
// 参数说明     void
// 返回参数     void
// 备注信息     仅GPIO、消抖和控制状态更新；PWM仍由CH0统一写出
//-------------------------------------------------------------------------------------------------------------------
void local_drive_scan(void)
{
    uint8_t events = local_buttons_scan((uint8_t)!gpio_get_level(C15), (uint8_t)!gpio_get_level(C14));
    if (events & LOCAL_BUTTON_EVENT_STOP)
    {
        local_start_requested = 0u; // 同时按下或先启后停均撤销尚未消费的启动。
        control_request_stop();
    }
    else if ((events & LOCAL_BUTTON_EVENT_START) && local_input_ready &&
             control_get_state() != CONTROL_RUNNING && !control_get_calibration_reserved())
    {
        // 已运行或诊断独占时直接忽略KEY1，不让无效启动打断波形/诊断通信。
        local_start_generation = control_stop_generation();
        local_start_requested = 1u; // 这里只登记；主循环发布最新视觉后才可真正启动。
    }
}

// 函数简介     查询是否需要让联网等待立即让出主循环
// 参数说明     void
// 返回参数     非零表示已有本地启动事件
// 备注信息     驱动钩子只读事件，不在SPI等待或ISR内启动电机
static uint8_t local_drive_network_cancel(void)
{
    return local_start_requested;
}

// 主循环领取与启动共用短临界区，避免检查代际后又插入STOP而被旧START覆盖。
static void local_drive_poll_start(void)
{
    uint32_t key = interrupt_global_disable();
    if (local_start_requested)
    {
        local_start_requested = 0u; // 不论成功与否只消费一次，条件恢复后不得自动发车。
        if (local_start_generation == control_stop_generation())
            (void)control_request_local_track_run();
    }
    interrupt_global_enable(key);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     初始化外设与控制层，上电保持 READY 和零输出
// 参数说明     void
// 返回参数     void
// 备注信息     联网和相机等待不启动电机；本地按键事件由主循环消费
//-------------------------------------------------------------------------------------------------------------------
void Init(void)
{
    EncoderMeasurementSnapshot measurement_profile; // 启动前读取物理量版本，分次配置N和名义周长时拒绝陈旧更新。
    // 1. 先以零占空比启动PWM外设，后面的联网/相机重试不会因此直接发车。
    /* 电机与舵机 */
    pwm_init(MOTOR1_PWM, 17000, 0); // 左电机17kHz载波由硬件生成，最后参数0表示上电零命令。
    gpio_init(MOTOR1_DIR, GPO, MOTOR1_FORWARD_DIR_LEVEL, GPO_PUSH_PULL);
    pwm_init(MOTOR2_PWM, 17000, 0); // 右轮独立通道，前进方向电平须在main.h按实车标定。
    gpio_init(MOTOR2_DIR, GPO, MOTOR2_FORWARD_DIR_LEVEL, GPO_PUSH_PULL);
    pwm_init(SERVO_MOTOR_PWM, SERVO_MOTOR_FREQ, 0); // 50Hz/20ms硬件脉冲，上电为0，不立即输出中位脉宽。

    // 2. 在PIT启动前安装保存/恢复PRIMASK钩子，配置控制和两路编码器，保证首轮也能安全读写共享状态。
    /* 控制与编码器 */
    control_set_critical_hooks(interrupt_global_disable, interrupt_global_enable);
    control_init(servo_motor_duty_middle,
                 MIN(SERVO_MOTOR_L_MAX, SERVO_MOTOR_R_MAX),
                 MAX(SERVO_MOTOR_L_MAX, SERVO_MOTOR_R_MAX), SERVO_DIR);
    control_configure_servo_pwm(PWM_DUTY_MAX, SERVO_MOTOR_FREQ); // 保留原电气换算刻度，PIT统一使用整数占空快照。
    control_set_output_enabled(MOTOR_ENABLE, SERVO_MOTOR_ENABLE); // 编译许可仍需本地启动、正目标和有效视觉条件才能起车。
    encoder_quad_init(ENCODER_1, ENCODER_1_A, ENCODER_1_B);
    encoder_quad_init(ENCODER_2, ENCODER_2_A, ENCODER_2_B);
    // 计数器保持16位自然回绕；只在初始化清零，运行中不再读后清零而丢掉中间脉冲。
    encoder_clear_count(ENCODER_1);
    encoder_clear_count(ENCODER_2);
    encoder_measurement_set_critical_hooks(interrupt_global_disable, interrupt_global_enable);
    encoder_measurement_init(ENCODER_1_SIGN, ENCODER_2_SIGN);
    steering_map_set_critical_hooks(interrupt_global_disable, interrupt_global_enable);
    steering_map_init();
    // 编码器比例采用用户授权的单组10圈平均；右轮部分角表用于查询，双轮控制标定表仍为空。
    if ((calibration_profile_counts_per_rev_left != 0.0f || calibration_profile_counts_per_rev_right != 0.0f) &&
        !encoder_measurement_set_counts_per_rev(calibration_profile_counts_per_rev_left,
                                                calibration_profile_counts_per_rev_right, 1u))
        control_report_initialization_fault();
    encoder_measurement_get_snapshot(&measurement_profile);
    if ((calibration_profile_circumference_left_m != 0.0f || calibration_profile_circumference_right_m != 0.0f) &&
        !encoder_measurement_set_wheel_circumference(calibration_profile_circumference_left_m,
            calibration_profile_circumference_right_m, measurement_profile.calibration_version))
        control_report_initialization_fault(); // 非法换算配置维持零输出；不会绕过人工启动或修改速度PI单位。
    if (calibration_profile_steering_map.calibrated &&
        !steering_map_apply(&calibration_profile_steering_map, 1u))
        control_report_initialization_fault();
    // GPT1提供独立微秒时间；GPT2保留给原延时库，任何初始化失败保持故障零输出。
    vision_clock_ready = calibration_clock_init();
    if (!vision_clock_ready) control_report_initialization_fault();
    calibration_debug_set_critical_hooks(interrupt_global_disable, interrupt_global_enable);
    calibration_debug_init();
    // LENGTH/ONCE为0才能形成模65536连续计数，防止未来驱动改成短周期重载仍被误用。
    if ((TMR1->CHANNEL[0].CTRL & (TMR_CTRL_LENGTH_MASK | TMR_CTRL_ONCE_MASK)) ||
        (TMR1->CHANNEL[2].CTRL & (TMR_CTRL_LENGTH_MASK | TMR_CTRL_ONCE_MASK)))
        control_report_initialization_fault();
    local_buttons_init();
    local_start_requested = local_input_ready = 0u;
    gpio_init(C15, GPI, 1, GPI_PULL_UP); // KEY1低电平启动，KEY2低电平停止；KEY3/4不占用。
    gpio_init(C14, GPI, 1, GPI_PULL_UP);
    wifi_spi_set_cancel_hook(local_drive_network_cancel); // 等待模块/热点/Socket时，启动键可使网络让出主循环。
    pit_ms_init(PIT_CH1, 5u); // 只扫描按键和登记事件，不执行相机设置、图传或速度闭环。
    pit_ms_init(PIT_CH0, CONTROL_LOOP_PERIOD_MS); // 固定10ms；startup已开IRQ，控制可早于后续联网/相机完成运行。

    // 3. 显示/网络初始化在main上下文执行，不能放入速度控制ISR。
    /* 显示与通信 */
    flash_init();
    tft180_init();

    // 4. 相机初始化失败就在停止状态重试；成功后包装原回调，保留SDK双缓冲，再复位视觉时间历史。
    /* 摄像头 */
    while (mt9v03x_init())
    {
        tft180_show_string(0, 16, "CAM RETRY");
        system_delay_ms(1000);
    }
    camera_frame_install();
    vision_reset();
    vision_track_init(vision_clock_ready ? calibration_clock_now_us : NULL); // 初始化估计映射恢复和固定记录区，不自动发车。
    local_input_ready = 1u; // 本地视觉已具备接收条件；联网失败或等待不能阻止KEY1请求。
    tft180_show_string(0, 16, "K1 START K2 STOP");
    wifi_assistant_init(); // 放在相机之后；本地启动事件可打断驱动等待，返回后先处理新帧。

}

// 静止、居中的直道图像才能用于宽度标定，不能从合成默认值推断实车宽度。
static uint8_t calibrate_width(const ControlParams *params, const ControlSnapshot *snapshot)
{
    uint16_t row;
    uint16_t widths[CONTROL_IMAGE_HEIGHT]; // 先收集全组候选，避免控制行失败时已写入一部分参考宽度。
    uint8_t i;
    // 1. 只在不运行且左右读数接近0时标定；零PWM不代表轮子已停，因此另查编码器。
    if (snapshot->state == CONTROL_RUNNING || snapshot->measured_left < -1 ||
        snapshot->measured_left > 1 || snapshot->measured_right < -1 || snapshot->measured_right > 1 ||
        // 图像须可用/高可信，误差在±5像素内，即人工居中直道，不能拿偏斜单边图标定宽度。
        !vision.valid || vision.confidence < 0.9f || vision.error_px < -5.0f || vision.error_px > 5.0f ||
        (uint32_t)(control_now_ms() - vision.capture_time_ms) > 100u) return 0u;
    // 2. 按左右边坐标差记录实测宽度，虚拟重建或太窄白斑清为0；0表示该行未标定。
    for (row = 0; row < CONTROL_IMAGE_HEIGHT; ++row)
    {
        uint16_t width = (uint16_t)(vision.right[row] - vision.left[row]); // 坐标差，不是白像素个数；3白像素只得到宽度2。
        widths[row] = vision.row_quality[row] == 2u && width >= 3u && width < CONTROL_IMAGE_WIDTH ? width : 0u;
    }
    // 3. 五个控制行必须都有合格的双边实测宽度；其他行可为0，不要求全图120行皆有效。
    for (i = 0; i < CONTROL_BAND_COUNT; ++i)
        if (widths[params->look_rows[i]] == 0u) return 0u;
    // 4. 全组校验通过再提交，并核对每次setter的返回；宽度被拒绝时不能显示OK。
    for (row = 0; row < CONTROL_IMAGE_HEIGHT; ++row)
        if (!vision_set_reference_width(row, widths[row])) return 0u;
    vision_reset(); // 保留刚提交的宽度，只清视觉种子/时间历史；不自动发车或代替人工确认。
    return 1u;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     在停车主循环独立刷新带故障标签的状态，不等待新相机帧
// 参数说明     image_pending   新二值图待绘制标记；仅实际绘制后清零，不能据此伪造采集时间
// 返回参数     void
// 备注信息     行驶不绘屏；停车普通刷新间隔200逻辑ms，状态/故障变化优先绘制，不在中断调用
//-------------------------------------------------------------------------------------------------------------------
static void local_display_poll(uint8_t *image_pending)
{
    static uint32_t last_display_ms; // 最近一次停车绘制的逻辑时刻；不代表屏幕实际刷新耗时。
    static ControlState displayed_state = CONTROL_RUNNING; // 初值使上电首次停车状态不必等待200ms。
    static ControlFault displayed_fault = CONTROL_FAULT_NONE; // 上次已绘制的故障枚举，变更可绕过普通限频。
    ControlSnapshot snapshot;
    uint32_t now;
    const char *reason;
    control_get_snapshot(&snapshot); // 每轮重新取控制快照；即使camera_frame_take失败，也能看到PIT新锁存的故障。
    if (snapshot.state == CONTROL_RUNNING)
    {
        displayed_state = CONTROL_RUNNING; // 记住运行状态，使下一次停车立即刷新；行驶不占用绘屏时间。
        return;
    }
    now = control_now_ms();
    if (snapshot.state == displayed_state && snapshot.fault == displayed_fault &&
        (uint32_t)(now - last_display_ms) < 200u) return;
    // 只有新图待绘时才重画125×100图像；无新图仍更新下面的状态，画面保留最后一张二值图。
    if (*image_pending)
    {
        tft180_displayimage03x((uint8_t *)image, 125, 100);
        *image_pending = 0u;
    }
    control_get_snapshot(&snapshot); // 图像绘制期间PIT仍运行；标签使用绘图后最新的故障快照。
    tft180_set_color(RGB565_BLACK, RGB565_WHITE);
    tft180_show_string(0, 0, "LEFT: ");
    tft180_show_int(48, 0, snapshot.measured_left, 3);
    tft180_show_string(0, 30, "RIGHT:");
    tft180_show_int(48, 30, snapshot.measured_right, 3);
    tft180_show_string(0, 50, "CONF: ");
    tft180_show_float(48, 50, snapshot.confidence, 1, 2);
    switch (snapshot.fault)
    {
        case CONTROL_FAULT_NONE: reason = "NONE"; break;
        case CONTROL_FAULT_VISION_TIMEOUT: reason = "VISION TIMEOUT"; break;
        case CONTROL_FAULT_TRACK_LOST: reason = "TRACK LOST"; break; // 保留枚举解释，不新增按丢线帧数停车逻辑。
        case CONTROL_FAULT_LINK: reason = "DIAG LINK"; break;
        case CONTROL_FAULT_INVALID_VALUE: reason = "INVALID VALUE"; break;
        default: reason = "UNKNOWN"; break;
    }
    // 默认8×16字库不含汉字；两条16字符底色覆盖128像素，适配当前竖屏并清掉上一条较长原因。
    if (snapshot.fault != CONTROL_FAULT_NONE) tft180_set_color(RGB565_WHITE, RGB565_RED);
    tft180_show_string(0, 70, "FAULT:          ");
    tft180_show_int(56, 70, (int32)snapshot.fault, 2);
    tft180_show_string(0, 86, "                ");
    tft180_show_string(0, 86, reason);
    tft180_set_color(TFT180_DEFAULT_PENCOLOR, TFT180_DEFAULT_BGCOLOR); // 恢复驱动默认配色，避免后续Wi-Fi状态继承故障底色。
    last_display_ms = now;
    displayed_state = snapshot.state;
    displayed_fault = snapshot.fault;
}

int main(void)
{
    uint32_t track_stop_generation = 0u; // 停止事件变化时清路径历史，避免复用上次实验。
    uint32_t band_version = 0u; // 视觉已安装的参数版本；初始0保证第一轮配置五行控制带。
    uint8_t display_image_pending = 0u; // 仅成功处理新图后置位；无新图仍可独立刷新故障标签。
    uint32_t calibration_request_ms = 0u; // 等新图宽度请求的起点，1秒仍无帧则取消。
    uint8_t calibration_pending = 0u; // 请求只准备一次标定，首个新帧给出OK/REJECTED，不循环自动接受。
    clock_init(SYSTEM_CLOCK_600M); // 设置CPU时钟与中断分组；实际抖动不能据600MHz配置推断。
    debug_init();
    system_delay_ms(300);
    Init();
    interrupt_global_enable(0); // 确认主循环IRQ开放；并非此处才第一次启用Init中已启动的PIT。

    while (1)
    {
        ControlParams params;
        ControlSnapshot snapshot;
        uint32_t version;
        uint32_t frame_id;
        uint32_t capture_ms;
        // 1. 参数/版本同锁复制；控制带只在新版本安装成功后记录，失败则锁接口故障。
        calibration_debug_poll(calibration_clock_now_us()); // 主循环只解析完整命令；STOP/心跳仍由PIT优先保护。
        control_get_snapshot(&snapshot);
        vision_record_observe(&snapshot); // 即使相机不再出帧，也能在停车后冻结上一组记录。
        control_get_params(&params, &version);
        if (version != band_version)
        {
            vision_track_reset(); // 控制带或参数版本变化后重新建立本轮路径历史。
            if (vision_set_control_band(params.look_rows, params.look_weights)) band_version = version; // 不把失败版本记成已生效。
            else control_report_invalid_fault();
        }
        // 人工请求由poll锁存、下一轮领取；仅登记待标定，不触发电机或舵机启动。
        if (wifi_assistant_take_width_calibration_request() && !control_get_calibration_reserved())
        {
            calibration_pending = 1u;
            calibration_request_ms = control_now_ms();
        }
        // 优先处理新帧，显示和网络不得抢在视觉与控制目标发布之前。
        // 2. 只处理成功复制的新帧；没有新图时PIT仍继续查图像年龄，main不伪造新结果。
        if (calibration_pending && control_get_calibration_reserved())
        {
            calibration_pending = 0u; // 标定会话独占期间不把旧宽度请求留给之后另一工况。
            printf("[vision] width calibration BUSY\r\n");
        }
#if CALIBRATION_DEBUG_ENABLE
        {
            int32_t marker = calibration_debug_take_marker_request();
            if (marker >= 0)
            {
                static uint8_t marker_white = 1u; // 每次翻转实际颜色，中止后新试验也有可见边沿。
                static uint16_t pixels[16u * 16u]; // 仅诊断版小色块，不在ISR绘屏或进行整图刷新。
                uint32_t mark_start = calibration_clock_now_us();
                uint16_t i;
                marker_white ^= 1u;
                for (i = 0u; i < 16u * 16u; ++i)
                    pixels[i] = marker_white ? 0xffffu : 0xf800u;
                // 112,112的16像素色块在两种128×160/160×128方向内，且避开125×100停车图像。
                tft180_show_rgb565_image(112u, 112u, pixels, 16u, 16u, 16u, 16u, 0u);
                calibration_debug_marker_completed((uint32_t)marker, mark_start, calibration_clock_now_us());
            }
        }
#endif
        {
            uint32_t copy_started_us; // 仅测稳定快照复制，不把停止代际检查计入复制步骤。
            uint32_t stop_generation = control_stop_generation();
            if (stop_generation != track_stop_generation)
            {
                vision_track_reset();
                track_stop_generation = stop_generation;
            }
            copy_started_us = calibration_clock_now_us(); // 独立微秒，不用10ms逻辑时钟测算法耗时。
            if (camera_frame_take((uint8_t *)image, &frame_id, &capture_ms))
            {
                uint8_t threshold;
                uint32_t stage_us = calibration_clock_now_us();
                vision_timing_begin(frame_id, capture_ms, copy_started_us);
                vision_timing_set(VISION_TIME_COPY, stage_us - copy_started_us);
                {
                    ControlSnapshot capture_snapshot;
                    control_get_snapshot(&capture_snapshot);
                    vision_record_observe(&capture_snapshot); // 复制期间已停车则先冻结，停车新帧不能挤入记录。
                }
                vision_record_capture((const uint8_t *)image,frame_id,capture_ms); // 只采样运行中的稳定灰度。
                wifi_assistant_capture_raw((const uint8 *)image); // 二值化前保存原灰度；应用内部仅在允许停车图传时复制。
                stage_us = calibration_clock_now_us();
                vision_track_histogram((const uint8_t *)image,hist_gram); // 地面ROI计数，排除上部白墙对全局阈值的影响。
                vision_timing_set(VISION_TIME_HISTOGRAM, calibration_clock_now_us() - stage_us);
                stage_us = calibration_clock_now_us();
                threshold = get_threshold_otsu(hist_gram); // 求当前图类间方差最大阈值，纯色/空输入在算法内处理。
                vision_timing_set(VISION_TIME_OTSU, calibration_clock_now_us() - stage_us);
                stage_us = calibration_clock_now_us();
                binaryzation_process((uint8_t *)image, MT9V03X_H, MT9V03X_W, threshold); // 原地转0/255，边线判断不再依赖灰度阈值。
                vision_timing_set(VISION_TIME_BINARY, calibration_clock_now_us() - stage_us);
                // 寻线模块只读二值图，按新帧号/采集时间生成质量与误差；阈值由本调用层补入诊断字段。
                stage_us = calibration_clock_now_us();
                vision_process((uint8_t *)image, frame_id, capture_ms, &vision);
                vision_timing_set(VISION_TIME_BASELINE, calibration_clock_now_us() - stage_us);
                vision_track_process((const uint8_t *)image, &params, &vision); // 当前帧恢复，失败保留原结果及采集时间。
                vision.threshold = threshold;
                stage_us = calibration_clock_now_us();
                control_publish_vision(&vision); // 更新图像资格与P/PD目标；PI和舵机限速/实际PWM写入仍在PIT。
                vision_timing_set(VISION_TIME_PUBLISH, calibration_clock_now_us() - stage_us);
                vision_timing_finish(); // 发布固定环形记录；运行中不打印、不网络传输耗时。
                {
                    ControlSnapshot evidence_snapshot;
                    control_get_snapshot(&evidence_snapshot);
                    vision_record_frame((const uint8_t *)image,&vision,&evidence_snapshot);
                    vision_record_observe(&evidence_snapshot); // 处理期间发生停止时，此采集帧完成后冻结。
                }
                local_drive_poll_start(); // 先发布视觉再领取KEY1，避免图传/显示抢在本地启动之前。
                control_get_snapshot(&snapshot); // 原子取当下状态；最近tick输出与最新视觉字段可能来自不同时间域。
                // 3. 人工宽度请求仅用这次新帧尝试一次；不符合静止/居中条件则显式拒绝。
                if (calibration_pending)
                {
                    uint8_t accepted = calibrate_width(&params, &snapshot);
                    calibration_pending = 0u; // 消费请求，失败不会在后续另一张图上悄悄自动成功。
                    printf("[vision] width calibration %s\r\n", accepted ? "OK" : "REJECTED");
                }
                display_image_pending = 1u; // 新图只登记待绘；状态显示统一在相机分支外处理，避免无帧不显示故障。
                // 图传使用独立边线数组，不在二值图上涂线；接口内部按运行状态禁止行驶发送整图。
                vision_record_display_lines(&vision,display_lines);
                wifi_assistant_on_frame((const uint8 *)image, display_lines, display_lines+120u, display_lines+240u,
                                        vision.error_px, snapshot.servo_cmd, threshold);
            }
        }
        // 5. 始终无新帧时取消待标定，避免旧请求在很久后的错误工况下才被执行。
        if (calibration_pending && (uint32_t)(control_now_ms() - calibration_request_ms) > 1000u)
        {
            calibration_pending = 0u;
            printf("[vision] width calibration NO NEW FRAME\r\n");
        }
        local_drive_poll_start(); // 无新图也消费请求；最新图不可用时拒绝，不把旧请求留到未来。
        local_display_poll(&display_image_pending); // 先消费启动，再按最新状态判断；停车故障刷新不依赖本轮是否收到图像。
        wifi_assistant_poll(); // 主循环最后收发；运行路径有截止预算，重连/整图续传仅在停车路径。
    }
}
