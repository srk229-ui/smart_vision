"""执行真实主循环视觉片段，核对步骤记录、发布时间和停止保护。"""
from pathlib import Path
import argparse
import sys

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'tools'))
import run_software_checks as checks


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--evidence-dir',type=Path)
    args=parser.parse_args()
    if args.evidence_dir:checks.EVIDENCE=args.evidence_dir.resolve()
    source=(ROOT/'user/src/main.c').read_text(encoding='utf-8')
    begin=source.index('uint8_t threshold;',source.index('if (camera_frame_take('))
    end=source.index('local_drive_poll_start(); // 先发布视觉',begin)
    body=source[begin:end]
    fixture='''#include "control.h"
#include "img_process.h"
#include "vision_track.h"
#include "vision_record.h"
#include <stdint.h>
#include <assert.h>
#include <string.h>
#include <stdio.h>
typedef uint8_t uint8;
#define MT9V03X_H 120
#define MT9V03X_W 188
static uint8_t image[120][188];
static uint32_t hist_gram[256], fake_clock;
static VisionResult vision;
static ControlParams params;
static uint32_t calibration_clock_now_us(void) { return ++fake_clock; }
static void wifi_assistant_capture_raw(const uint8_t *data) { (void)data; }
static void frame(uint32_t frame_id,uint32_t capture_ms) {
    uint32_t copy_started_us=calibration_clock_now_us();
'''+body+'}\n'
    cases='''int main(void) {
    unsigned row,col,i;ControlSnapshot snapshot;
    control_init(92.5f,85.0f,100.0f,-1.0f);control_set_output_enabled(1u,1u);
    control_get_params(&params,NULL);vision_reset();vision_track_init(calibration_clock_now_us);
    for(row=0;row<120;++row)for(col=40;col<148;++col)image[row][col]=220;
    control_tick(0,0);frame(1u,10u);
    assert(vision_timing_log.frames==1u);
    for(i=VISION_TIME_COPY;i<=VISION_TIME_BASELINE;++i)assert(vision_timing_log.records[0].elapsed_us[i]>0u);
    assert(vision_timing_log.records[0].elapsed_us[VISION_TIME_PUBLISH]>0u);
    assert(vision_timing_log.records[0].elapsed_us[VISION_TIME_TOTAL]>vision_timing_log.records[0].elapsed_us[VISION_TIME_BASELINE]);
    assert(vision_timing_log.records[0].capture_time_ms==10u && vision_timing_log.records[0].mapping_estimated);
    assert(control_request_local_track_run());control_tick(0,0);
    control_get_snapshot(&snapshot);assert(snapshot.state==CONTROL_RUNNING);
    memset(image,0,sizeof(image));frame(2u,20u);
    assert(!vision.valid && vision.capture_time_ms==20u);
    assert(vision_timing_log.records[1].recovery_status==VISION_TRACK_NO_PATH);
    for(i=0;i<52;++i)control_tick(0,0);
    control_get_snapshot(&snapshot);assert(snapshot.state==CONTROL_FAULT && snapshot.fault==CONTROL_FAULT_VISION_TIMEOUT);
    assert(!snapshot.pwm_left && !snapshot.pwm_right && !snapshot.servo_enabled);
    // 主循环领图前后若已经停车，新图只能更新预览，不能挤掉停车前的取证帧。
    for(row=0;row<120;++row)for(col=0;col<188;++col)image[row][col]=220;
    frame(3u,control_now_ms());
    assert(vision_stop_log.frozen && vision_stop_log.gray_frame_id==2u && vision_stop_log.last_capture_ms==20u);
    assert(vision_stop_log.gray[0]==0u && vision_stop_log.fault==CONTROL_FAULT_VISION_TIMEOUT);
    puts("PASS actual main visual pipeline: measured steps, original capture time, no synthetic fresh frame, 500ms stop");
    return 0;
}
'''
    folder=checks.EVIDENCE/'test_vision_pipeline';folder.mkdir(parents=True,exist_ok=True)
    generated=folder/'test_vision_pipeline.c';generated.write_text(fixture+cases,encoding='utf-8')
    checks.compile_test('test_vision_pipeline',[generated,*[ROOT/'user/src'/f'{n}.c' for n in ('vision_record','vision_track','img_process','control','pid')]])


if __name__=='__main__':main()
