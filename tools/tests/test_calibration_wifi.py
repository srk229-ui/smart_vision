"""直接编译真实Wi-Fi服务函数，验证离线诊断不会阻塞邮箱且STOP不丢失。"""

from pathlib import Path
import argparse
import importlib.util
import re
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'tools'))
import run_software_checks as checks

spec = importlib.util.spec_from_file_location('wifi_contract', ROOT / 'drivers/wifi_spi_v2/tests/test_m2_wifi.py')
contract = importlib.util.module_from_spec(spec)
spec.loader.exec_module(contract)

STUB = r'''
#include "control_types.h"
#include "vision_record.h"
volatile VisionTimingLog vision_timing_log;
uint8_t vision_track_control_rows[5]={80,85,90,95,100};
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
static uint32 replay_generation;
static uint8 replay_enabled, replay_quality[120], replay_lines[360];
void wifi_assistant_on_frame(const uint8*,const uint8*,const uint8*,const uint8*,float,float,uint8);
static uint8 camera_image[120][188], camera_left[120], camera_middle[120], camera_right[120], camera_row_y[120];
#define SEEKFREE_ASSISTANT_XY_BOUNDARY 2
static uint16 boundary_count;
void seekfree_assistant_camera_boundary_config(int kind,uint16 count,void *x1,void *x2,void *x3,void *y1,void *y2,void *y3) {
    unsigned k; (void)kind;(void)x1;(void)x3;(void)y1;(void)y3; boundary_count=count;
    for(k=0;k<count;++k) { assert(x2 && y2); assert(((uint8*)x2)[k]<188u && ((uint8*)y2)[k]<120u); }
}
static ControlState state=CONTROL_READY;
static ControlMode mode=CONTROL_MODE_TRACK;
static uint8 reserved;
static uint8 last_apply_running;
static uint32 stop_calls, fault_calls, init_calls, apply_calls, camera_calls, receive_calls;
static uint8 receive_mode, service_running, force_send_error;
static uint8 cancel_requested;
static uint32 cancel_after_delay, delay_calls, send_calls, reset_calls;
uint8 wifi_spi_cancel_requested(void) { return cancel_requested; }
ControlState control_get_state(void) { return state; }
void control_request_stop(void) { ++stop_calls; state=CONTROL_READY; reserved=0; }
void control_report_link_fault(void) { ++fault_calls; state=CONTROL_FAULT; }
void control_get_snapshot(ControlSnapshot *out) { memset(out,0,sizeof(*out)); out->state=state; out->mode=mode; }

void wifi_spi_service_begin(uint8 running) { service_running=running; }
void wifi_spi_service_end(void) { service_running=0; }
uint8 wifi_spi_service_is_running(void) { return service_running; }
uint32 wifi_spi_send_buffer(const uint8 *data,uint32 length) { (void)data; ++send_calls; return force_send_error ? length : 0; }
void system_delay_ms(uint32 ms) { assert(ms==1u); ++delay_calls; if(cancel_after_delay && delay_calls>=cancel_after_delay) cancel_requested=1; }
void show_state(void) {}
void wifi_assistant_init(void) { ++init_calls; }
void seekfree_assistant_receive_reset(void) { ++reset_calls; }
void seekfree_assistant_data_analysis(void) {
    ++receive_calls;
    assert(service_running == (CALIBRATION_DEBUG_ENABLE || state==CONTROL_RUNNING));
    if(receive_mode==1 || receive_mode==2) stop_seen=1;
    if(receive_mode==2 || receive_mode==3) ++wifi_spi_io_error_count;
    if(receive_mode==4) ++seekfree_assistant_receive_error_count;
    if(receive_mode==5 || receive_mode==6) cancel_requested=1;
    if(receive_mode==6) stop_seen=1;
}
void apply_parameters(uint8 running) { last_apply_running=running; ++apply_calls; if(stop_seen) control_request_stop(); }
void seekfree_assistant_oscilloscope_send(seekfree_assistant_oscilloscope_struct *wave) { (void)wave; }
void seekfree_assistant_camera_send(void) { ++camera_calls; }
'''

CASES = r'''
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

    // 发送失败只改变通信状态，正式及诊断版均不把Wi-Fi错误变成行车LINK故障。
    wifi_assistant_state=WIFI_ASSISTANT_READY; force_send_error=1;
    wifi_spi_service_begin(CALIBRATION_DEBUG_ENABLE);
    assert(assistant_send(&byte,1)==1);
    assert(fault_calls==0);
    wifi_assistant_state=WIFI_ASSISTANT_READY; state=CONTROL_RUNNING;
    wifi_spi_service_begin(1); assistant_send(&byte,1);
    assert(fault_calls==0 && state==CONTROL_RUNNING && wifi_assistant_state==WIFI_ASSISTANT_SEND_ERROR);
    for(unsigned test=0;test<3;++test) {
        uint32 init_before=init_calls;
        state=CONTROL_RUNNING; reserved=test!=0;
        mode=test==0 ? CONTROL_MODE_SERVO_CAL : test==1 ? CONTROL_MODE_TRACK : CONTROL_MODE_BENCH;
        wifi_assistant_state=WIFI_ASSISTANT_MODULE_ERROR; wifi_assistant_poll();
        assert(fault_calls==0 && state==CONTROL_RUNNING && init_calls==init_before);
    }

    // 底层I/O和解析容量错误都清理坏通信轮次；不带STOP时继续行车，已解析STOP仍然优先执行。
    for(unsigned error=3;error<=4;++error) {
        state=CONTROL_RUNNING; wifi_assistant_state=WIFI_ASSISTANT_READY;
        receive_mode=(uint8)error; stop_seen=0;
        wifi_assistant_poll();
        assert(state==CONTROL_RUNNING && fault_calls==0 && wifi_assistant_state==WIFI_ASSISTANT_SEND_ERROR);
    }
    state=CONTROL_RUNNING; reserved=1; wifi_assistant_state=WIFI_ASSISTANT_READY;
    receive_mode=2; stop_seen=0;
    wifi_assistant_poll();
    assert(state==CONTROL_READY && reserved==0 && fault_calls==0 && stop_seen==0);
    state=CONTROL_RUNNING; wifi_assistant_state=WIFI_ASSISTANT_READY;
    receive_mode=1; stop_seen=0; last_apply_running=0;
    wifi_assistant_poll();
    assert(state==CONTROL_READY && last_apply_running==1 && fault_calls==0);

    state=CONTROL_READY; force_send_error=0; wifi_assistant_state=WIFI_ASSISTANT_READY;
    wifi_assistant_capture_raw(image);
    wifi_assistant_on_frame(image,boundary,boundary,boundary,0,0,0);
    assert(camera_calls==(CALIBRATION_DEBUG_ENABLE ? 0u : 1u));
    // 33ms周期保留余量：10ms时钟的330ms内应有10个到期帧，不能退化为每40ms一帧。
    {
        uint32 before=camera_calls;
        send_period_ms=33; last_send_ms=0;
        for(elapsed_time_ms=10;elapsed_time_ms<=330;elapsed_time_ms+=10)
            wifi_assistant_on_frame(image,boundary,boundary,boundary,0,0,0);
        assert(camera_calls==before+(CALIBRATION_DEBUG_ENABLE ? 0u : 10u));
        // 主循环迟到跳过历史时隙，同一逻辑时刻重复调用不能补发积压图。
        elapsed_time_ms=1000;
        wifi_assistant_on_frame(image,boundary,boundary,boundary,0,0,0);
        before=camera_calls;
        wifi_assistant_on_frame(image,boundary,boundary,boundary,0,0,0);
        assert(camera_calls==before);
        // uint32计时回绕时仍按无符号差分到期，保留7ms余量。
        last_send_ms=UINT32_MAX-19u; elapsed_time_ms=20;
        wifi_assistant_on_frame(image,boundary,boundary,boundary,0,0,0);
        assert(camera_calls==before+(CALIBRATION_DEBUG_ENABLE ? 0u : 1u));
        if(!CALIBRATION_DEBUG_ENABLE) assert((uint32)(elapsed_time_ms-last_send_ms)==7u);
        // 行车图传门控仍拒绝新周期请求。
        before=camera_calls; state=CONTROL_RUNNING; elapsed_time_ms=100;
        wifi_assistant_on_frame(image,boundary,boundary,boundary,0,0,0);
        assert(camera_calls==before); state=CONTROL_READY;
    }
    // 停车发送开始前有按键事件时立即退出；空段也必须使通信失效，后续图像段不再触碰SPI。
    state=CONTROL_READY; force_send_error=0; cancel_requested=1;
    wifi_spi_service_begin(0); wifi_assistant_state=WIFI_ASSISTANT_READY;
    send_calls=delay_calls=0; raw_frame_valid=1;
    assert(assistant_send(&byte,0)==0 && wifi_assistant_state==WIFI_ASSISTANT_SEND_ERROR && !raw_frame_valid);
    assert(assistant_send(&byte,1)==1 && send_calls==0 && delay_calls==0);
    wifi_assistant_state=WIFI_ASSISTANT_READY;
    assert(assistant_send(&byte,1)==1 && send_calls==0 && delay_calls==0);

    // 首次发送失败后的20ms停车等待按1ms切片；第3ms事件到达后不续传，也不发送后续段。
    cancel_requested=0; cancel_after_delay=3; force_send_error=1;
    wifi_assistant_state=WIFI_ASSISTANT_READY; send_calls=delay_calls=0;
    assert(assistant_send(&byte,1)==1);
    assert(send_calls==1 && delay_calls==3 && wifi_assistant_state==WIFI_ASSISTANT_SEND_ERROR);
    assert(assistant_send(&byte,1)==1 && send_calls==1 && delay_calls==3);
    assert(state==CONTROL_READY && fault_calls==0);

    // 运行轮次失败或取消始终不转入停车延时重试，也不抛出行车LINK故障。
    cancel_requested=0; cancel_after_delay=0; state=CONTROL_RUNNING;
    wifi_assistant_state=WIFI_ASSISTANT_READY; send_calls=delay_calls=0;
    wifi_spi_service_begin(1); assert(assistant_send(&byte,1)==1);
    assert(send_calls==1 && delay_calls==0 && state==CONTROL_RUNNING && fault_calls==0);

    // poll开头已挂起按键时不收包、不重连、不应用参数，结束时释放服务预算。
    cancel_requested=1; wifi_assistant_state=WIFI_ASSISTANT_READY;
    {
        uint32 receives_before=receive_calls, applies_before=apply_calls, inits_before=init_calls;
        wifi_assistant_poll();
        assert(receive_calls==receives_before && apply_calls==applies_before && init_calls==inits_before && !service_running);
    }
    // 收包过程中到达取消：清半包/宽度请求且整组参数不生效；已解析的STOP仍然优先。
    for(unsigned with_stop=0;with_stop<2;++with_stop) {
        uint32 applies_before=apply_calls, stops_before=stop_calls, resets_before=reset_calls;
        state=CONTROL_RUNNING; cancel_requested=0; stop_seen=0; width_calibration_request=raw_frame_valid=1;
        receive_mode=(uint8)(5u+with_stop); wifi_assistant_state=WIFI_ASSISTANT_READY;
        wifi_assistant_poll();
        assert(apply_calls==applies_before && reset_calls==resets_before+1u && stop_calls==stops_before+with_stop);
        assert(!width_calibration_request && !raw_frame_valid && !stop_seen && !service_running && fault_calls==0);
        assert(state==(with_stop ? CONTROL_READY : CONTROL_RUNNING));
    }
    // 真实记录模块完成停车冻结，正常目标自动回放；相机停帧时poll也能发送。
    {
        ControlSnapshot record_snapshot; VisionResult record_result;
        uint32 before;
        memset(&record_snapshot,0,sizeof(record_snapshot)); memset(&record_result,0,sizeof(record_result));
        record_snapshot.state=CONTROL_RUNNING; record_snapshot.mode=CONTROL_MODE_TRACK;
        vision_record_observe(&record_snapshot);
        record_result.frame_id=7u;record_result.capture_time_ms=100u;
        record_result.valid=1u;record_result.confidence=0.5f;
        record_result.row_quality[80]=1u;record_result.middle[80]=30u;
        vision_record_capture(image,7u,100u);
        vision_timing_log.next=1u;vision_timing_log.records[0].frame_id=7u;vision_timing_log.records[0].capture_time_ms=100u;
        vision_record_frame(image,&record_result,&record_snapshot);
        record_snapshot.state=CONTROL_FAULT;record_snapshot.fault=CONTROL_FAULT_VISION_TIMEOUT;record_snapshot.timestamp_ms=700u;
        vision_record_observe(&record_snapshot);
        state=CONTROL_FAULT;mode=CONTROL_MODE_TRACK;receive_mode=0;stop_seen=cancel_requested=0;
        wifi_assistant_state=WIFI_ASSISTANT_READY;stream_mode=image_mode=1u;
        replay_generation=0u;replay_enabled=0u;last_send_ms=0u;elapsed_time_ms=1000u;before=camera_calls;
        wifi_assistant_on_frame(NULL,NULL,NULL,NULL,0,0,0);
        assert(camera_calls==before+(CALIBRATION_DEBUG_ENABLE ? 0u : 1u));
        if(!CALIBRATION_DEBUG_ENABLE) assert(replay_enabled && boundary_count==1u && camera_middle[0]==30u && camera_row_y[0]==80u && camera_image[1][1]==255u);
        elapsed_time_ms+=100u;before=camera_calls;wifi_assistant_poll();
        assert(camera_calls==before+(CALIBRATION_DEBUG_ENABLE ? 0u : 1u));
        // 手动切回实时图只改变显示，不清除冻结证据；空新帧不得重复发送旧实时缓存。
        replay_enabled=0u;replay_generation=vision_stop_log.generation;image_mode=1u;
        elapsed_time_ms+=100u;before=camera_calls;wifi_assistant_on_frame(NULL,NULL,NULL,NULL,0,0,0);
        assert(camera_calls==before && vision_stop_log.frozen);
        // 冻结回放也受运行门控，不能趁新一次人工启动传大图。
        replay_enabled=1u;state=CONTROL_RUNNING;elapsed_time_ms+=100u;
        wifi_assistant_on_frame(NULL,NULL,NULL,NULL,0,0,0);assert(camera_calls==before);
        state=CONTROL_READY;
    }
    // XY只携带有效行，避免依赖上位机对255占位的裁剪行为。
    memset(camera_middle,255,sizeof(camera_middle)); memset(camera_left,255,sizeof(camera_left)); memset(camera_right,255,sizeof(camera_right));
    camera_middle[80]=30u; camera_middle[90]=35u;
    assistant_pack_boundaries(); assert(boundary_count==2u && camera_row_y[0]==80u && camera_row_y[1]==90u);
    memset(camera_middle,255,sizeof(camera_middle)); assistant_pack_boundaries(); assert(!boundary_count);
    puts("PASS actual Wi-Fi normal/diagnostic functions: offline run, no link stop, STOP priority, send/poll cancel, image gate");
    return 0;
}
'''


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--evidence-dir', required=True, type=Path)
    args = parser.parse_args()
    checks.EVIDENCE = args.evidence_dir.resolve()
    source = (ROOT / 'user/src/wifi_assistant.c').read_text(encoding='utf-8')
    header = (ROOT / 'user/inc/wifi_assistant.h').read_text(encoding='utf-8')
    enum = re.search(r'typedef enum\s*\{[^}]+\}\s*wifi_assistant_state_enum;', header, re.S).group(0)
    functions = '\n'.join(contract.extract_function(source, name) for name in (
        'assistant_send', 'wifi_assistant_poll', 'wifi_assistant_capture_raw', 'assistant_pack_boundaries', 'assistant_replay_label', 'wifi_assistant_on_frame'))
    for enabled in (0, 1):
        name = 'test_calibration_wifi_' + str(enabled)
        directory = checks.EVIDENCE / name
        directory.mkdir(parents=True, exist_ok=True)
        generated = directory / (name + '.c')
        generated.write_text(contract.COMMON + enum + STUB + functions + CASES, encoding='utf-8')
        checks.compile_test(name, [generated,ROOT / "user/src/vision_record.c"], ['CALIBRATION_DEBUG_ENABLE=' + str(enabled)])


if __name__ == '__main__':
    main()
