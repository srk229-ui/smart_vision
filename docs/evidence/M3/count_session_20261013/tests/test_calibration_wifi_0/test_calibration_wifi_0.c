
#include <assert.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
typedef uint8_t uint8;
typedef uint16_t uint16;
typedef uint32_t uint32;
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
#include "control_types.h"
#include "calibration_config.h"
#define WIFI_ASSISTANT_ENABLE 1
#define WIFI_ASSISTANT_RECONNECT_PERIOD_MS 2000
#define WIFI_ASSISTANT_RUNNING_PERIOD_MS 100
#define WIFI_ASSISTANT_SEND_ATTEMPTS 3
#define MT9V03X_H 120
#define MT9V03X_W 188
typedef struct { uint8 channel_num; float data[8]; } seekfree_assistant_oscilloscope_struct;
static wifi_assistant_state_enum wifi_assistant_state;
static uint32 elapsed_time_ms=5000, last_reconnect_ms, last_send_ms, send_period_ms=100;
static uint32 wifi_spi_io_error_count, seekfree_assistant_receive_error_count;
static uint32 wifi_assistant_reconnect_count, wifi_assistant_send_error_count, wifi_assistant_last_unsent_bytes;
static uint32 wifi_assistant_frame_count;
static uint8 width_calibration_request, stop_seen, raw_frame_valid, stream_mode=1, image_mode=1;
static uint8 camera_image[120][188], camera_left[120], camera_middle[120], camera_right[120];
static ControlState state=CONTROL_READY;
static uint32 stop_calls, fault_calls, init_calls, apply_calls, camera_calls, receive_calls;
static uint8 receive_mode, service_running, force_send_error;
ControlState control_get_state(void) { return state; }
void control_request_stop(void) { ++stop_calls; state=CONTROL_READY; }
void control_report_link_fault(void) { ++fault_calls; state=CONTROL_FAULT; }
void control_get_snapshot(ControlSnapshot *out) { memset(out,0,sizeof(*out)); out->state=state; }
void wifi_spi_service_begin(uint8 running) { service_running=running; }
void wifi_spi_service_end(void) { service_running=0; }
uint8 wifi_spi_service_is_running(void) { return service_running; }
uint32 wifi_spi_send_buffer(const uint8 *data,uint32 length) { (void)data; return force_send_error ? length : 0; }
void system_delay_ms(uint32 ms) { (void)ms; }
void show_state(void) {}
void wifi_assistant_init(void) { ++init_calls; }
void seekfree_assistant_receive_reset(void) {}
void seekfree_assistant_data_analysis(void) {
    ++receive_calls;
    assert(service_running == (CALIBRATION_DEBUG_ENABLE || state==CONTROL_RUNNING));
    if(receive_mode) stop_seen=1;
    if(receive_mode==2) ++wifi_spi_io_error_count;
}
void apply_parameters(uint8 running) { (void)running; ++apply_calls; if(stop_seen) control_request_stop(); }
void seekfree_assistant_oscilloscope_send(seekfree_assistant_oscilloscope_struct *wave) { (void)wave; }
void seekfree_assistant_camera_send(void) { ++camera_calls; }
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
    // 2. 只有尾段仍有数据且尚未用完尝试次数，才继续本段发送。
    for(attempt = 0; remaining != 0 && attempt < attempts; ++attempt)
    {
        // 底层返回剩余字节数；length-remaining是已发前缀，续传指针跳过该前缀。
        remaining = wifi_spi_send_buffer(buffer + (length - remaining), remaining);
        if(remaining != 0)
        {
            ++wifi_assistant_send_error_count;
            wifi_assistant_last_unsent_bytes = remaining; // 保留本次尾段长度用于调试。
            // 仅停止轮次且还有下一次尝试时等待；运行轮次绝不进入20ms延时。
            if(!running && attempt + 1 < attempts)
            {
                system_delay_ms(20); // 仅停车时给模块缓冲区留出恢复时间
            }
        }
    }
    // 3. 尾段未发完属于发送失败；运行时即使尾段已发完，新增底层I/O异常仍要处理。
    if(remaining != 0 || (running && wifi_spi_io_error_count != io_errors))
    {
        wifi_assistant_state = WIFI_ASSISTANT_SEND_ERROR;
        raw_frame_valid = 0; // 链路异常后不能再发送旧原始图。
        last_reconnect_ms = elapsed_time_ms; // 停车恢复从此时开始等重连间隔。
        if(running && (!CALIBRATION_DEBUG_ENABLE || control_get_state() == CONTROL_RUNNING))
        {
            control_report_link_fault(); // 锁存控制链路故障；不因随后网络恢复而自行发车。
        }
    }
    return remaining; // 返回剩余字节数；组件未直接使用返回值，整帧成功另由通信状态/计数判断。
}
void wifi_assistant_poll(void)
{
#if WIFI_ASSISTANT_ENABLE
    uint32 now;
    uint32 io_errors;
    uint32 parser_errors;
    uint8 initially_running = control_get_state() == CONTROL_RUNNING; // 接收前捕获，后续STOP或PIT故障不能改写本轮身份。
    ControlSnapshot snapshot;
    seekfree_assistant_oscilloscope_struct wave;
    // 1. 在接收前固定本轮运行身份，并启动共享RX/TX的截止预算。
    // COUNT虽然保持READY，诊断版仍使用有界收发预算，防止命令确认被停车等待挤占。
    wifi_spi_service_begin(initially_running || CALIBRATION_DEBUG_ENABLE);
    if(!initially_running) { show_state(); } // 运行不刷新状态TFT。
    // 通信非READY：运行立即报告链路故障；停止态才可能按重连间隔恢复。
    if(wifi_assistant_state != WIFI_ASSISTANT_READY)
    {
        if(initially_running)
        {
            control_report_link_fault();
        }
        // 四类可恢复错误状态任一成立，且逻辑等待达到重连周期，才尝试重新初始化。
        // 无符号差值处理计数回绕；运行分支不会到此处。
        // 诊断版不自动重连缺席模块；已有连接仍可用有界接收处理V1 STOP。
        else if(!CALIBRATION_DEBUG_ENABLE && (wifi_assistant_state == WIFI_ASSISTANT_MODULE_ERROR ||
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
    if(wifi_spi_io_error_count != io_errors || seekfree_assistant_receive_error_count != parser_errors)
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
        if(initially_running) { control_report_link_fault(); }
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
    /* 停车接收后刚启动的本轮，也必须启用行车通信总预算。 */
    // 接收前停止、参数使车辆刚启动：发送前启用运行预算，不能继续停车发送规则。
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
    last_send_ms = now; // 图像和波形共用发送计时基准。
    // 3. 这里只可能走停车图传；各段通过assistant_send续传，运行没有这条大图路径。
    wifi_spi_service_begin(0);
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
int main(void) {
    uint8 image[120*188]={0}, boundary[120]={0};
    uint8 byte=0;
    wifi_assistant_state=WIFI_ASSISTANT_MODULE_ERROR;
    wifi_assistant_poll();
    assert(init_calls==(CALIBRATION_DEBUG_ENABLE ? 0u : 1u));
    assert(state==CONTROL_READY && fault_calls==0);
    wifi_assistant_state=WIFI_ASSISTANT_READY; receive_mode=1;
    wifi_assistant_poll();
    assert(receive_calls==1 && stop_calls==1 && apply_calls==1);
    receive_mode=2; stop_seen=0;
    wifi_assistant_poll();
    assert(stop_calls==2 && stop_seen==0 && apply_calls==1);
    assert(state==CONTROL_READY && fault_calls==0);
    wifi_assistant_state=WIFI_ASSISTANT_READY; force_send_error=1;
    wifi_spi_service_begin(CALIBRATION_DEBUG_ENABLE);
    assert(assistant_send(&byte,1)==1);
    assert(fault_calls==0);
    wifi_assistant_state=WIFI_ASSISTANT_READY; state=CONTROL_RUNNING;
    wifi_spi_service_begin(1); assistant_send(&byte,1);
    assert(fault_calls==1 && state==CONTROL_FAULT);
    state=CONTROL_READY; force_send_error=0; wifi_assistant_state=WIFI_ASSISTANT_READY;
    wifi_assistant_capture_raw(image);
    wifi_assistant_on_frame(image,boundary,boundary,boundary,0,0,0);
    assert(camera_calls==(CALIBRATION_DEBUG_ENABLE ? 0u : 1u));
    puts("PASS actual Wi-Fi diagnostic/normal functions: offline retry, bounded RX, STOP+RX fault, send fault, image gate");
    return 0;
}
