#include "wifi_assistant.h"
#include "zf_common_headfile.h"
#include "seekfree_assistant_interface.h"
#include "control.h"
#include "vision_record.h" // 冻结记录只停车回放，沿用V1图传协议。
#include "calibration_config.h" // 诊断版通过pyOCD工作，联网服务不能阻塞标定邮箱。

float wifi_assistant_servo_gain = WIFI_ASSISTANT_DEFAULT_SERVO_GAIN; // 兼容显示用舵机P镜像；控制实际采用ControlParams.steer_kp。
volatile wifi_assistant_state_enum wifi_assistant_state = WIFI_ASSISTANT_DISABLED; // 通信状态，与车辆READY/RUNNING/FAULT状态相互独立。
volatile uint32 wifi_assistant_send_error_count; // 发送尝试未完成的次数，不是丢图帧数。
volatile uint32 wifi_assistant_last_unsent_bytes; // 最近一段发送尝试剩余字节数。
volatile uint32 wifi_assistant_reconnect_count; // 停止态发起重新初始化的次数。
volatile uint32 wifi_assistant_frame_count; // 图像及边线全部发送成功的完整帧数。
volatile uint32 wifi_assistant_parameter_reject_count; // 保留通道请求被拒绝的计数，非逐包精确统计。

#if WIFI_ASSISTANT_ENABLE
static volatile uint32 elapsed_time_ms; // PIT累计的逻辑时间，单位ms；不在这里执行网络。
static uint32 last_send_ms; // 最近一次发波形或图像的逻辑时刻，单位ms。
static uint32 last_reconnect_ms; // 最近连接失败/发送异常后开始等待的逻辑时刻，单位ms。
static uint32 send_period_ms = WIFI_ASSISTANT_DEFAULT_PERIOD_MS; // 停车发送周期，单位ms；运行使用固定周期。
static uint8 stream_mode = WIFI_ASSISTANT_DEFAULT_STREAM; // 停车0仅收参数、1图传、2波形。
static uint8 image_mode = WIFI_ASSISTANT_DEFAULT_IMAGE; // 0二值化前的原始图、1处理图。
static uint32 replay_generation; // 已识别的冻结代际，新一轮停车自动进入回放。
static uint8 replay_enabled; // 人工选择实时图后为0，不删除冻结证据。
static uint8 replay_quality[CONTROL_IMAGE_HEIGHT]; // 回放行质量，0行不显示默认中线。
static uint8 replay_lines[CONTROL_IMAGE_HEIGHT*3u]; // 回放原边线，复制到图传缓存前按质量屏蔽。
static uint8 raw_frame_valid; // 仅标记当前原始图已完整保存，消费后清零。
static uint8 camera_image[MT9V03X_H][MT9V03X_W]; // 独立图传缓存，尺寸与相机相同。
static uint8 camera_left[MT9V03X_H]; // 每行左边线横坐标，8位像素索引。
static uint8 camera_middle[MT9V03X_H]; // 每行中线横坐标，8位像素索引。
static uint8 camera_right[MT9V03X_H]; // 每行右边线横坐标，8位像素索引。
static uint8 camera_row_y[MT9V03X_H]; // XY点表只打包有效行，无效行不发出默认94列。
static uint8 width_calibration_request; // 主循环消费的一次性宽度标定请求。
static uint8 stop_seen; // 本次解析轮次出现过通道0零目标的锁存。
static wifi_assistant_state_enum displayed_state = WIFI_ASSISTANT_DISABLED; // 已显示的通信状态，避免重复刷TFT。

#if WIFI_SPI_AUTO_CONNECT != 0
#error "Use WIFI_SPI_AUTO_CONNECT = 0; wifi_assistant_init establishes the socket explicitly"
#endif

#if MT9V03X_W > 256
#error "The existing demo uses 8-bit boundary coordinates"
#endif

/* 行车发送只尝试一次；停止态才允许延时续传和自动恢复。 */
// 函数简介     按当前轮次规则发送一段V1数据，必要时续传未发尾段。
// 参数说明     buffer是当前数据段首地址；length为本段字节数。
// 返回参数     仍未发送的字节数，0表示本段送入模块完毕。
static uint32 assistant_send(const uint8 *buffer, uint32 length)
{
    uint8 attempt;
    // 1. 控制状态或本轮SPI预算任一仍为运行，都按运行规则处理。
    // 即使PIT已经停车，service标记也保留本轮身份，不能转入延时重试。
    uint8 running = control_get_state() == CONTROL_RUNNING || wifi_spi_service_is_running();
    uint8 attempts = running ? 1 : WIFI_ASSISTANT_SEND_ATTEMPTS; // 运行只尝试一次；停车按配置允许有限续传。
    uint32 remaining = length; // 尚未发送的尾段字节数。
    uint32 io_errors = wifi_spi_io_error_count; // 本段发送前的底层错误基线。
    // 接口未就绪时不碰SPI，把整段留给上层作为未发送结果。
    if(wifi_assistant_state != WIFI_ASSISTANT_READY)
    {
        return length;
    }
    if (wifi_spi_cancel_requested())
    {
        wifi_assistant_state = WIFI_ASSISTANT_SEND_ERROR;
        raw_frame_valid = 0;
        last_reconnect_ms = elapsed_time_ms;
        return length; // KEY1已登记时不继续发任何图像段，回主循环先处理视觉和本地启动。
    }
    // 2. 只有尾段仍有数据且尚未用完尝试次数，才继续本段发送。
    for(attempt = 0; remaining != 0 && attempt < attempts; ++attempt)
    {
        if (wifi_spi_cancel_requested()) break; // 每次续传前让出，不在新段中等待本地启动。
        // 底层返回剩余字节数；length-remaining是已发前缀，续传指针跳过该前缀。
        remaining = wifi_spi_send_buffer(buffer + (length - remaining), remaining);
        if(remaining != 0)
        {
            ++wifi_assistant_send_error_count;
            wifi_assistant_last_unsent_bytes = remaining; // 保留本次尾段长度用于调试。
            // 仅停止轮次且还有下一次尝试时等待；运行轮次绝不进入20ms延时。
            if(!running && attempt + 1 < attempts)
            {
                uint8 pause_ms;
                // 原停车20ms等待拆为1ms片段；事件到达后不等待剩余片段，也不续传。
                for (pause_ms = 0u; pause_ms < 20u && !wifi_spi_cancel_requested(); ++pause_ms)
                    system_delay_ms(1u);
            }
        }
    }
    // 3. 尾段未发完属于发送失败；运行时即使尾段已发完，新增底层I/O异常仍要处理。
    if(remaining != 0 || wifi_spi_cancel_requested() || (running && wifi_spi_io_error_count != io_errors))
    {
        wifi_assistant_state = WIFI_ASSISTANT_SEND_ERROR;
        raw_frame_valid = 0; // 链路异常后不能再发送旧原始图。
        last_reconnect_ms = elapsed_time_ms; // 停车恢复从此时开始等重连间隔。
        // Wi-Fi发送异常只影响通信；本地行车继续，诊断会话仍由独立pyOCD心跳决定结束。
    }
    return remaining; // 返回剩余字节数；组件未直接使用返回值，整帧成功另由通信状态/计数判断。
}

// 仅在停止路径刷新状态；此函数本身不检查行车，调用者负责门控。
static void show_state(void)
{
    const char *text;
    if(displayed_state == wifi_assistant_state) // 同状态无需重新刷屏或重复打印。
    {
        return;
    }
    displayed_state = wifi_assistant_state;
    switch(displayed_state)
    {
        case WIFI_ASSISTANT_INITIALIZING: text = "WIFI INIT...    "; break;
        case WIFI_ASSISTANT_MODULE_ERROR: text = "WIFI INIT FAIL  "; break;
        case WIFI_ASSISTANT_LINK_ERROR:   text = "WIFI LINK FAIL  "; break;
        case WIFI_ASSISTANT_READY:        text = "WIFI OK         "; break;
        case WIFI_ASSISTANT_SEND_ERROR:   text = "WIFI SEND FAIL  "; break;
        case WIFI_ASSISTANT_WIFI_ERROR:   text = "WIFI JOIN FAIL  "; break;
        default:                         text = "WIFI OFF        "; break;
    }
    tft180_show_string(0, 112, text);
    printf("[wifi_assistant] %s\r\n", text);
}

/* 每个合法线上参数包都观察一次，不能让后续同通道包覆盖停止语义。 */
// 参数使用线上通道编号1..8，V1界面通道0对应线上1。
// 只锁存该通道值0的停止事件；其它同通道请求仍以解析器保留的末值为准。
static void assistant_parameter_event(uint8 channel, float value)
{
    if(channel == 1 && value == 0.0f) { stop_seen = 1; }
}

/* 同一轮先收集请求，停车参数统一提交，避免左右轮出现不同版本。 */
// 函数简介     收集本轮参数，优先停止，再按接收前状态决定能否提交运动参数。
// 参数说明     initially_running为接收开始前的运行标记，不能因本轮已停车而改写。
// 返回参数     void；接受/拒绝结果通过控制状态和拒绝计数体现。
static void apply_parameters(uint8 initially_running)
{
    uint8 channel;
    uint8 updated[SEEKFREE_ASSISTANT_SET_PARAMETR_COUNT]; // 各通道本轮是否有未消费末值。
    float values[SEEKFREE_ASSISTANT_SET_PARAMETR_COUNT]; // 各通道最后一个合法包的数据。
    uint8 dirty = 0; // 只有ControlParams字段变化才置1。
    uint8 control_batch_valid = 1; // 任一运动字段非法时阻止整组提交。
    uint32 version; // 配置读取时的版本，用于提交时拒绝陈旧副本。
    ControlParams params; // 完整运动配置的局部候选副本。
    // 1. 先把各通道末值及更新标记拷到局部，再消费共享标记。
    // 解析与应用都在主循环完成；observer同样在合法包解析过程中调用。
    for(channel = 0; channel < SEEKFREE_ASSISTANT_SET_PARAMETR_COUNT; ++channel)
    {
        updated[channel] = seekfree_assistant_parameter_update_flag[channel];
        values[channel] = seekfree_assistant_parameter[channel];
        seekfree_assistant_parameter_update_flag[channel] = 0; // 即使本批被拒绝，也不在下一轮重复执行旧请求。
    }
    /* 同批只要出现过停止就立即结束，不应用其它运动参数。 */
    // stop_seen能记住被后续正目标覆盖的通道0停止包；更新末值为0也是停止。
    if(stop_seen || (updated[0] && values[0] == 0.0f))
    {
        stop_seen = 0;
        control_request_stop(); // 软件停止立即清输出历史；实际PWM零命令由后续PIT写入。
        // 本批停止后不提交其余运动请求，也不继续发车；拒绝统计按保留的通道请求计。
        for(channel = 0; channel < SEEKFREE_ASSISTANT_SET_PARAMETR_COUNT; ++channel)
        {
            if(updated[channel] && !(channel == 0 && values[channel] == 0.0f))
            {
                ++wifi_assistant_parameter_reject_count;
            }
        }
        return;
    }
    // 2. 使用接收前的运行标记门控整批，其间PIT故障/停车不能解锁本批改参。
    // 包括显示/发送设置在内，本批其它更新均拒绝；下一轮停止态请求才可应用。
    if(initially_running)
    {
        for(channel = 0; channel < SEEKFREE_ASSISTANT_SET_PARAMETR_COUNT; ++channel)
        {
            if(updated[channel] && !(channel == 0 && values[channel] == 0.0f))
            {
                ++wifi_assistant_parameter_reject_count;
            }
        }
        return;
    }
    // 3. 停止态先取完整配置及版本；修改局部副本，最后一次提交左右轮等运动字段。
    control_get_params(&params, &version);
    for(channel = 0; channel < SEEKFREE_ASSISTANT_SET_PARAMETR_COUNT; ++channel)
    {
        float value = values[channel];
        if(!updated[channel])
        {
            continue;
        }
        // 界面通道0..7与线上通道1..8相差1；参数单位由控制接口统一定义。
        switch(channel)
        {
            case 0:
                // 通道0：0已在停止优先分支处理；正目标不超过统一上限（当前400），单位counts/10ms。
                // 正目标仅设置左右基准目标；启动只能由本地KEY1发起，收到参数包不会发车。
                if(value > 0.0f && value <= CONTROL_TARGET_COUNTS_MAX)
                {
                    params.target_left = params.target_right = value; // 左右基准目标同改，单位counts/10ms。
                    dirty = 1;
                }
                else if(value != 0.0f)
                {
                    control_batch_valid = 0;
                    ++wifi_assistant_parameter_reject_count;
                }
                break;
            case 1:
            case 2:
                // 通道1/2：共用左右轮Kp/Ki；这里只做范围检查，整组配置还要通过控制层验证。
                // Kp把速度误差转为PWM命令；Ki按秒积分，不能按一次调用累加一个固定周期。
                if(value >= 0.0f && value <= 1000.0f)
                {
                    if(channel == 1)
                    {
                        params.kp_left = params.kp_right = value; // 左右Kp同时更新，避免分轮版本不同。
                    }
                    else
                    {
                        params.ki_left = params.ki_right = value; // 左右Ki同时更新，积分增量另由秒制周期计算。
                    }
                    dirty = 1;
                }
                else
                {
                    control_batch_valid = 0;
                    ++wifi_assistant_parameter_reject_count;
                }
                break;
            case 3:
                /* 速度微分固定为零，此通道只接收下列人工操作整数。 */
                // 0/1/2作为旧模式操作兼容空动作；本地KEY1始终启动TRACK，BENCH只由诊断入口使用。
                if(value == 0.0f || value == 1.0f || value == 2.0f) { }
                // 3保留旧人工清故障接口；清除后维持停止，只能另按本地KEY1启动。
                else if(value == 3.0f)
                {
                    if(!control_reset_fault()) { ++wifi_assistant_parameter_reject_count; }
                }
                // 4把人工标定确认写入待提交配置；确认标志不等于已有实机标定证据。
                else if(value == 4.0f) { params.calibration_confirmed = 1; dirty = 1; }
                // 5保留旧撤销确认并停止操作；普通TRACK不依赖该确认位。
                else if(value == 5.0f)
                {
                    control_request_stop();
                    params.calibration_confirmed = 0;
                    dirty = 1;
                }
                // 6只准备一次直道宽度标定请求；主循环另行检查静止图像，不触发启动。
                else if(value == 6.0f) { width_calibration_request = 1; }
                else
                {
                    control_batch_valid = 0;
                    ++wifi_assistant_parameter_reject_count;
                }
                break;
            case 4:
                // 通道4为舵机P：像素误差到命令角的增益，允许0..5，D仍由控制配置决定。
                if(value >= 0.0f && value <= 5.0f)
                {
                    params.steer_kp = value;
                    dirty = 1;
                }
                else
                {
                    control_batch_valid = 0;
                    ++wifi_assistant_parameter_reject_count;
                }
                break;
            // 通道5/6/7是本地显示/发送设置，不属于ControlParams运动配置事务。
            case 5:
                // 通道5只接受整数0/1/2：仅收参数/停车图传/停车波形；运行仍强制10Hz波形。
                if(value == 0.0f || value == 1.0f || value == 2.0f)
                {
                    stream_mode = (uint8)value;
                    raw_frame_valid = 0; // 换图/波形模式后旧原始缓存失效。
                    last_send_ms = elapsed_time_ms; // 切换后重新等发送周期，不立刻复用旧帧。
                }
                else { ++wifi_assistant_parameter_reject_count; }
                break;
            case 6:
                // 通道6：0实时灰度、1实时二值、2冻结二值循环、3冻结末帧灰度；不触发运动。
                if(value == 0.0f || value == 1.0f || value == 2.0f || value == 3.0f)
                {
                    image_mode = (uint8)value;
                    replay_enabled=(uint8)(value>=2.0f);
                    replay_generation=vision_stop_log.generation; // 手动实时选择在本冻结代际内保持。
                    raw_frame_valid = 0;
                }
                else { ++wifi_assistant_parameter_reject_count; }
                break;
            case 7:
                // 通道7允许20..1000ms；转为uint32时舍去小数部分，不改变10ms控制周期。
                if(value >= 20.0f && value <= 1000.0f)
                {
                    send_period_ms = (uint32)value; // 单位ms；浮点输入的小数部分在此截断。
                }
                else { ++wifi_assistant_parameter_reject_count; }
                break;
        }
    }
    // 4. 任一运动字段非法，取消整组运动配置；已执行的人工操作/本地设置不回滚。
    if(!control_batch_valid)
    {
        return;
    }
    if(dirty)
    {
        // expected_version使用读取配置时的版本；控制层重新检查停止态、版本及整组字段。
        // 只提交一次，避免左右轮或执行器字段分次落入不同版本。
        if(!control_apply_params(&params, version))
        {
            ++wifi_assistant_parameter_reject_count;
            return;
        }
        wifi_assistant_servo_gain = params.steer_kp; // 只在运动参数提交成功后更新兼容显示镜像。
    }
    // 参数接收在此结束，不调用任何行车启动接口。
}
#endif

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     WIFI-SPI 模块及逐飞助手 V1 接口初始化
// 参数说明     void            无
// 返回参数     void
// 备注信息     只在上电或停止态调用；连接失败时显示状态并返回
//-------------------------------------------------------------------------------------------------------------------
void wifi_assistant_init(void)
{
#if WIFI_ASSISTANT_ENABLE
    // 1. 初始化属于上电/停止恢复路径，不能从运行分支进入。
    wifi_spi_service_begin(0);
    seekfree_assistant_parameter_observer = assistant_parameter_event; // 只观察合法包事件；回调不进行SPI收发或控制计算。
    // 新连接先清接收半包和未应用标记，再清应用层的一次性请求/停止锁存。
    seekfree_assistant_receive_reset();
    width_calibration_request = 0; // 新连接不继承旧宽度准备请求。
    stop_seen = 0; // 新连接不继承上轮STOP事件。
    wifi_assistant_state = WIFI_ASSISTANT_INITIALIZING;
    raw_frame_valid = 0; // 重连后等新图，不续传旧连接半帧。
    last_reconnect_ms = elapsed_time_ms;
    show_state();
    // 2. 组件绑定WIFI-SPI收发；将发送回调改为本模块，以统一重试与故障语义。
    seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_WIFI_SPI);
    seekfree_assistant_transfer_callback = assistant_send; // 所有V1发送段统一经过剩余字节数/故障门控。
    /* V2.0 先检查模块通信，再单独连接热点，便于定位初始化失败阶段 */
    // 模块初始化返回错误或版本字符串仍为空，均不能当作当前模块通信成功。
    if(wifi_spi_init(NULL, NULL) || wifi_spi_version[0] == '\0')
    {
        wifi_assistant_state = WIFI_ASSISTANT_MODULE_ERROR;
        last_reconnect_ms = elapsed_time_ms;
        show_state();
        return;
    }

    printf("[wifi_assistant] firmware=%s, MAC=%s\r\n", wifi_spi_version, wifi_spi_mac_addr);
    // 3. 热点凭据只来自配置宏；模块检查和热点连接分开，便于按状态定位失败阶段。
    if(wifi_spi_wifi_connect(WIFI_ASSISTANT_SSID, WIFI_ASSISTANT_PASSWORD))
    {
        wifi_assistant_state = WIFI_ASSISTANT_WIFI_ERROR;
        last_reconnect_ms = elapsed_time_ms;
        show_state();
        return;
    }

// 4. 按配置建立TCP客户端或UDP链路；电脑端须匹配协议及目标端口。
#if WIFI_ASSISTANT_TRANSPORT == 1
    if(wifi_spi_socket_connect("TCP", WIFI_ASSISTANT_TARGET_IP, WIFI_SPI_TARGET_PORT, WIFI_SPI_LOCAL_PORT))
#else
    if(wifi_spi_socket_connect("UDP", WIFI_ASSISTANT_TARGET_IP, WIFI_SPI_TARGET_PORT, WIFI_SPI_LOCAL_PORT))
#endif
    {
        wifi_assistant_state = WIFI_ASSISTANT_LINK_ERROR;
        last_reconnect_ms = elapsed_time_ms;
        show_state();
        return;
    }

    // 5. V1组件只持有本模块的独立图像/边线缓存地址，不修改算法输入图。
    seekfree_assistant_camera_information_config(SEEKFREE_ASSISTANT_MT9V03X,
                                                  camera_image[0], MT9V03X_W, MT9V03X_H);
    seekfree_assistant_camera_boundary_config(SEEKFREE_ASSISTANT_X_BOUNDARY, MT9V03X_H,
                                               camera_left, camera_middle, camera_right,
                                               NULL, NULL, NULL);
    last_send_ms = elapsed_time_ms; // 重连后发送最新完整帧，避免续接旧连接的半帧
    // READY只表示通信接口可用，不表示车辆已满足人工启动条件。
    wifi_assistant_state = WIFI_ASSISTANT_READY;
    printf("[wifi_assistant] module=%s, PC=%s:%s\r\n",
           wifi_spi_ip_addr_port, WIFI_ASSISTANT_TARGET_IP, WIFI_SPI_TARGET_PORT);
    show_state();
#endif
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     更新上位机通信计时
// 参数说明     elapsed_ms      本次经过的时间，单位 ms
// 返回参数     void
// 备注信息     由现有控制定时器调用，不在中断中执行网络收发
//-------------------------------------------------------------------------------------------------------------------
void wifi_assistant_tick(uint32 elapsed_ms)
{
#if WIFI_ASSISTANT_ENABLE
    // 此计数仅由控制定时器写、主循环读；volatile不是通用互斥锁。
    // 传入的是已服务的逻辑ms，漏服务不能凭此计数证明真实时间准确。
    elapsed_time_ms += elapsed_ms;
#else
    (void)elapsed_ms;
#endif
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     处理逐飞助手参数及示波器数据
// 参数说明     void            无
// 返回参数     void
// 备注信息     主循环调用；行车固定十次每秒波形，停车后通道五选择图像或波形
//-------------------------------------------------------------------------------------------------------------------
void wifi_assistant_poll(void)
{
#if WIFI_ASSISTANT_ENABLE
    uint32 now;
    uint32 io_errors;
    uint32 parser_errors;
    uint8 initially_running = control_get_state() == CONTROL_RUNNING; // 固定接收前的运行身份，同轮STOP不能转入停车改参或延时重试。
    ControlSnapshot snapshot;
    seekfree_assistant_oscilloscope_struct wave;
    // 1. 在接收前固定本轮运行身份，并启动共享RX/TX的截止预算。
    // COUNT虽然保持READY，诊断版仍使用有界收发预算，防止命令确认被停车等待挤占。
    wifi_spi_service_begin(initially_running || CALIBRATION_DEBUG_ENABLE);
    if (wifi_spi_cancel_requested())
    {
        wifi_spi_service_end();
        return; // KEY1在上一次主循环领取后才到达，跳过本轮通信，下一轮先处理新图。
    }
    if(!initially_running) { show_state(); } // 运行不刷新状态TFT。
    // 通信非READY时行车直接返回；故障只影响Wi-Fi，停止态才允许按间隔重连。
    if(wifi_assistant_state != WIFI_ASSISTANT_READY)
    {
        // 四类可恢复错误状态任一成立，且逻辑等待达到重连周期，才尝试重新初始化。
        // 无符号差值处理计数回绕；运行分支不会到此处。
        // 诊断版不自动重连缺席模块；已有连接仍可用有界接收处理V1 STOP。
        if(!initially_running && !CALIBRATION_DEBUG_ENABLE && (wifi_assistant_state == WIFI_ASSISTANT_MODULE_ERROR ||
                 wifi_assistant_state == WIFI_ASSISTANT_WIFI_ERROR ||
                 wifi_assistant_state == WIFI_ASSISTANT_LINK_ERROR ||
                 wifi_assistant_state == WIFI_ASSISTANT_SEND_ERROR) &&
                (uint32)(elapsed_time_ms - last_reconnect_ms) >= WIFI_ASSISTANT_RECONNECT_PERIOD_MS)
        {
            ++wifi_assistant_reconnect_count;
            /* 行车期间绝不进入复位、热点连接和网络重连。 */
            wifi_assistant_init();
        }
        wifi_spi_service_end();
        return;
    }
    // 2. 记录本轮开始时的SPI与解析错误计数；普通坏校验包只重同步，不增加容量错误计数。
    io_errors = wifi_spi_io_error_count;
    parser_errors = seekfree_assistant_receive_error_count;
    seekfree_assistant_data_analysis();
    // 任一计数变化都使本轮请求失效；即使解析器已读出正目标也不能继续执行。
    if(wifi_spi_cancel_requested() || wifi_spi_io_error_count != io_errors || seekfree_assistant_receive_error_count != parser_errors)
    {
        wifi_assistant_state = WIFI_ASSISTANT_SEND_ERROR;
        raw_frame_valid = 0;
        last_reconnect_ms = elapsed_time_ms;
        /* 错误轮次同时清除半包和更新旗标，避免重连后复用陈旧启动命令。 */
        // 同时丢弃半包、更新标记、宽度请求及停止锁存，避免旧请求跨错误轮次重放。
        // 已解析的合法STOP不能被随后SPI/容量错误清掉，COUNT虽为READY也须终止。
        if(stop_seen) { control_request_stop(); }
        seekfree_assistant_receive_reset();
        width_calibration_request = 0; // 错误轮次不保留待标定请求。
        stop_seen = 0; // STOP锁存属于本轮，错误清理后不跨轮复用。
        // 接收错误只作通信清理，已解析STOP仍执行；不会把Wi-Fi异常升级为行车故障。
        wifi_spi_service_end();
        return;
    }
    // 3. 应用接收请求后读取一份一致快照，不分次拼接控制状态与各轮输出。
    apply_parameters(initially_running);
    control_get_snapshot(&snapshot);
    // 接收前在运行、接收后已停止/故障：立即结束本轮，保留运行预算到退出。
    if(initially_running && snapshot.state != CONTROL_RUNNING)
    {
        /* 停止发生的行车轮次立即结束；下一轮停车通信才允许恢复方式。 */
        wifi_spi_service_end();
        return;
    }
    /* 收包期间控制状态若变为运行，发送仍必须启用行车通信总预算。 */
    // 远程参数不会启动；这里保留状态复查，避免其它本地调用改变状态后使用停车发送规则。
    if(!initially_running && snapshot.state == CONTROL_RUNNING)
    {
        wifi_spi_service_begin(1); // 刚启动后发送也只能按运行方式服务。
    }
    now = elapsed_time_ms;
    // 4. 三种情况不发波形：链路非READY；停止态未选波形；发送间隔未到。
    // 运行固定100ms间隔，停车使用人工设置；该发送节奏不是控制采样周期。
    if(wifi_assistant_state != WIFI_ASSISTANT_READY ||
       (snapshot.state != CONTROL_RUNNING && stream_mode != 2) ||
       (uint32)(now - last_send_ms) < (snapshot.state == CONTROL_RUNNING ? WIFI_ASSISTANT_RUNNING_PERIOD_MS : send_period_ms))
    {
        wifi_spi_service_end();
        if(snapshot.state!=CONTROL_RUNNING && stream_mode==1 && vision_stop_log.frozen)
            wifi_assistant_on_frame(NULL,NULL,NULL,NULL,0.0f,0.0f,0u); // 无新相机帧也能回放。
        return;
    }
    last_send_ms = now; // 本轮已占用发送时刻，失败不在主循环里密集重发。
    // 5. 8通道取控制快照：有效目标已包含斜坡/降速，PWM是下发整数命令而非硬件反馈。
    wave.channel_num = 8;
    wave.data[0] = snapshot.target_left; // 左轮有效目标，counts/10ms。
    wave.data[1] = snapshot.target_right; // 右轮有效目标，counts/10ms。
    wave.data[2] = (float)snapshot.measured_left; // 本次左编码器10ms窗口计数，转float供V1显示。
    wave.data[3] = (float)snapshot.measured_right; // 本次右编码器10ms窗口计数，转float供V1显示。
    wave.data[4] = snapshot.pwm_left; // 左轮实际下发PWM整数命令。
    wave.data[5] = snapshot.pwm_right; // 右轮实际下发PWM整数命令。
    wave.data[6] = snapshot.vision_error_px; // 像素误差；失效时可能保留最后有效值，不代表图像仍可信。
    wave.data[7] = snapshot.servo_cmd; // 限速后的舵机命令角，不是实测前轮角。
    seekfree_assistant_oscilloscope_send(&wave);
    wifi_spi_service_end();
#endif
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     保存待发送的原始灰度图像
// 参数说明     *image          当前摄像头图像首地址
// 返回参数     void
// 备注信息     必须在原地二值化之前调用，只在原始图传到期时复制
//-------------------------------------------------------------------------------------------------------------------
void wifi_assistant_capture_raw(const uint8 *image)
{
#if WIFI_ASSISTANT_ENABLE
    // 先取消旧缓存有效标记，只有本次满足条件的新原始图才能用于图传。
    raw_frame_valid = 0;
    // 必须同时满足停止、链路READY、图传模式、原始图模式和到期，才复制灰度图。
    if(!CALIBRATION_DEBUG_ENABLE && control_get_state() != CONTROL_RUNNING && wifi_assistant_state == WIFI_ASSISTANT_READY && stream_mode == 1 && image_mode == 0 &&
       (uint32)(elapsed_time_ms - last_send_ms) >= send_period_ms)
    {
        memcpy(camera_image, image, sizeof(camera_image)); // 复制sizeof(camera_image)字节原始灰度，当前188×120=22560；不覆盖算法图。
        raw_frame_valid = 1; // 仅这次完整副本可在后续on_frame消费。
    }
#else
    (void)image;
#endif
}

// 函数简介     将有效坐标压缩为V1已有的XY点表，不依赖上位机如何处理255。
// 参数说明     无；camera_*是独立显示缓存，绝不修改视觉原结果。
// 返回参数     无；含重建行时仅显示中线，避免把虚拟边线冒充实测。
static void assistant_pack_boundaries(void)
{
    uint16 row,count=0u;
    uint8 double_only=1u;
    for(row=0u;row<MT9V03X_H;++row) {
        if(camera_middle[row]>=MT9V03X_W) continue;
        if(camera_left[row]>=MT9V03X_W || camera_right[row]>=MT9V03X_W) double_only=0u;
        camera_left[count]=camera_left[row]; camera_middle[count]=camera_middle[row];
        camera_right[count]=camera_right[row]; camera_row_y[count]=(uint8)row; ++count;
    }
    seekfree_assistant_camera_boundary_config(SEEKFREE_ASSISTANT_XY_BOUNDARY,count,
        count && double_only ? camera_left : NULL,count ? camera_middle : NULL,
        count && double_only ? camera_right : NULL,
        count && double_only ? camera_row_y : NULL,count ? camera_row_y : NULL,
        count && double_only ? camera_row_y : NULL);
}

// 函数简介     在独立发送图左上角标记冻结序号F1/8，避免把回放当作实时图。
// 参数说明     ordinal为按时间顺序的槽号；gray非零总显示最后一帧。
// 返回参数     无；原始冻结图和算法输入不受绘制影响。
static void assistant_replay_label(uint32 ordinal,uint8 gray)
{
    static const uint8 digits[9][5]={
        {0x3e,0x51,0x49,0x45,0x3e},{0,0x42,0x7f,0x40,0},{0x42,0x61,0x51,0x49,0x46},
        {0x21,0x41,0x45,0x4b,0x31},{0x18,0x14,0x12,0x7f,0x10},{0x27,0x45,0x45,0x45,0x39},
        {0x3c,0x4a,0x49,0x49,0x30},{1,0x71,9,5,3},{0x36,0x49,0x49,0x49,0x36}};
    static const uint8 prefix[5]={0x7f,9,9,9,1},slash[5]={0x20,0x10,8,4,2};
    const uint8 *glyph[4];
    unsigned x,y,i;
    uint32 count=vision_stop_log.count;
    if(!count || count>8u) return;
    glyph[0]=prefix; glyph[1]=digits[gray ? count : ordinal%count+1u]; glyph[2]=slash; glyph[3]=digits[count];
    for(y=0u;y<9u;++y) for(x=0u;x<25u;++x) camera_image[y][x]=0u;
    for(i=0u;i<4u;++i) for(x=0u;x<5u;++x) for(y=0u;y<7u;++y)
        if(glyph[i][x]&(1u<<y)) camera_image[y+1u][i*6u+x+1u]=255u;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     发送当前图像、边线并更新图像调试数据
// 参数说明     *image          处理后图像首地址
// 参数说明     *left / *middle / *right左边线、中线和右边线数组
// 参数说明     mid_error       当前中线偏差
// 参数说明     servo_angle     当前舵机控制角度
// 参数说明     threshold       当前二值化阈值
// 返回参数     void
// 备注信息     主循环完成图像处理后调用；按设定周期发送，避免每帧都占用链路
//-------------------------------------------------------------------------------------------------------------------
void wifi_assistant_on_frame(const uint8 *image, const uint8 *left,
                             const uint8 *middle, const uint8 *right,
                             float mid_error, float servo_angle, uint8 threshold)
{
#if WIFI_ASSISTANT_ENABLE
    // 处理函数传入的旧兼容诊断参数不用于波形；波形统一从控制快照读取。
    uint32 now = elapsed_time_ms;
    (void)mid_error;
    (void)servo_angle;
    (void)threshold;
    // 运行、链路异常、未选图传或发送未到期，任一成立都拒绝图像发送。
    // 诊断版不走停车整图发送，避免大图和延时续传阻塞pyOCD命令。
    if(CALIBRATION_DEBUG_ENABLE || control_get_state() == CONTROL_RUNNING || wifi_assistant_state != WIFI_ASSISTANT_READY || stream_mode != 1 ||
       (uint32)(now - last_send_ms) < send_period_ms)
    {
        return;
    }
    if(vision_stop_log.frozen && replay_generation!=vision_stop_log.generation) {
        replay_generation=vision_stop_log.generation; replay_enabled=1u; // 每次停车默认自动回放。
    }
    // 冻结二值按250ms切换一张，灰度模式显示最后采样帧；不依赖新相机帧到来。
    if(replay_enabled && vision_record_replay((uint32)(now-vision_stop_log.stop_ms)/250u,
       (uint8)(image_mode==3u),(uint8_t *)camera_image,replay_lines,replay_quality)) {
        unsigned row;
        for(row=0u;row<CONTROL_IMAGE_HEIGHT;++row) {
            uint8 q=replay_quality[row];
            camera_left[row]=q==2u ? replay_lines[row] : 255u;
            camera_middle[row]=q ? replay_lines[120u+row] : 255u;
            camera_right[row]=q==2u ? replay_lines[240u+row] : 255u;
        }
        assistant_replay_label((uint32)(now-vision_stop_log.stop_ms)/250u,(uint8)(image_mode==3u));
    } else {
    if(!image || !left || !middle || !right || image_mode>=2u) return;
    // 1. 原始模式必须使用二值化前保存的本次图；没有有效缓存就等下一帧。
    if(image_mode == 0)
    {
        if(!raw_frame_valid)
        {
            return;
        }
        raw_frame_valid = 0; // 原始图只消费一次。
    }
    else
    // 2. 处理图模式复制当前处理结果；边线另存另传，不把画线图送回视觉算法。
    {
        memcpy(camera_image, image, sizeof(camera_image));
    }
    memcpy(camera_left, left, sizeof(camera_left));
    memcpy(camera_middle, middle, sizeof(camera_middle));
    memcpy(camera_right, right, sizeof(camera_right));
    }
    // 保留不足一个周期的余量：33ms在10ms逻辑时钟上交替约30/40ms到期，避免每次清零变成40ms。
    // 主循环较慢时跳过已错过的发送时隙，每次只发当前新图，不连续补发积压帧。
    last_send_ms = now - (uint32)(now - last_send_ms) % send_period_ms; // 余量单位ms，范围0至周期减1；无符号差处理回绕。
    // 3. 这里只可能走停车图传；各段通过assistant_send续传，运行没有这条大图路径。
    wifi_spi_service_begin(0);
    assistant_pack_boundaries(); // 只发送有效XY点，所有无效中线行均不进入协议载荷。
    seekfree_assistant_camera_send();
    wifi_spi_service_end();
    // 组件发送图像及三条边线时都使用受控回调；链路仍READY才把整帧计为成功。
    if(wifi_assistant_state == WIFI_ASSISTANT_READY)
    {
        ++wifi_assistant_frame_count; // 仅统计图像及边线全部发送成功的完整帧
    }
#else
    (void)image;
    (void)left;
    (void)middle;
    (void)right;
    (void)mid_error;
    (void)servo_angle;
    (void)threshold;
#endif
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     取出一次静止直道宽度标定请求
// 参数说明     void            无
// 返回参数     uint8           1-有新请求，0-没有新请求
// 备注信息     主循环消费请求，实际标定与有效性检查由视觉模块完成
//-------------------------------------------------------------------------------------------------------------------
uint8 wifi_assistant_take_width_calibration_request(void)
{
#if WIFI_ASSISTANT_ENABLE
    // 请求只由主循环解析与消费；复制后清零，保证通道3=6被处理一次，不自动发车。
    uint8 requested = width_calibration_request;
    width_calibration_request = 0; // 返回一次性请求后不再重放。
    return requested;
#else
    return 0;
#endif
}
