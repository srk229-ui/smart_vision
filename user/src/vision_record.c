#include "vision_record.h"
#include <string.h>

VisionStopLog vision_stop_log; // 固定内存取证，禁止在PIT、网络回调或DMA缓冲上写入。
static uint8_t armed, pending; // 是否已观察到TRACK运行，是否已复制一个等待完成的采样帧。
static uint32_t pending_frame, pending_capture; // 待完成的灰度标签，防止灰度/二值跨帧拼接。

void vision_record_observe(const ControlSnapshot *snapshot)
{
    if(snapshot==NULL) return;
    if(snapshot->state==CONTROL_RUNNING && snapshot->mode==CONTROL_MODE_TRACK) {
        if(!armed) {
            uint32_t sequence=vision_stop_log.sequence+1u;
            uint32_t generation=vision_stop_log.generation+1u;
            vision_stop_log.sequence=sequence; // 整组初始化期间保持奇数，不关中断。
            memset(vision_stop_log.frames,0,sizeof(vision_stop_log.frames));
            vision_stop_log.magic=VISION_STOP_MAGIC; vision_stop_log.abi=1u;
            vision_stop_log.bytes=sizeof(vision_stop_log); vision_stop_log.generation=generation;
            vision_stop_log.frozen=vision_stop_log.count=vision_stop_log.next=0u;
            vision_stop_log.stop_ms=vision_stop_log.fault=vision_stop_log.stop_state=0u;
            vision_stop_log.gray_frame_id=vision_stop_log.last_capture_ms=0u;
            vision_stop_log.frame_bytes=sizeof(VisionStopFrame); vision_stop_log.packed_bytes=VISION_STOP_PACKED;
            vision_stop_log.reserved=0u; pending=0u; armed=1u;
            vision_stop_log.sequence=sequence+1u;
        }
    } else if(armed) {
        ++vision_stop_log.sequence;
        vision_stop_log.stop_ms=snapshot->timestamp_ms;
        vision_stop_log.fault=(uint32_t)snapshot->fault;
        vision_stop_log.stop_state=(uint32_t)snapshot->state;
        memcpy(&vision_stop_log.timing,(const void *)&vision_timing_log,sizeof(vision_stop_log.timing));
        vision_stop_log.frozen=1u; armed=0u; pending=0u;
        ++vision_stop_log.sequence;
    }
}

void vision_record_capture(const uint8_t *gray, uint32_t frame_id, uint32_t capture_ms)
{
    if(!armed || vision_stop_log.frozen || gray==NULL || pending) return;
    if(vision_stop_log.count && (uint32_t)(capture_ms-vision_stop_log.last_capture_ms)<VISION_STOP_INTERVAL_MS) return;
    ++vision_stop_log.sequence;
    memcpy(vision_stop_log.gray,gray,sizeof(vision_stop_log.gray));
    pending_frame=frame_id; pending_capture=capture_ms; pending=1u;
    ++vision_stop_log.sequence;
}

void vision_record_frame(const uint8_t *binary, const VisionResult *result, const ControlSnapshot *snapshot)
{
    VisionStopFrame *frame;
    unsigned i;
    uint32_t slot;
    if(!armed || !pending || vision_stop_log.frozen || !binary || !result || !snapshot ||
       result->frame_id!=pending_frame || result->capture_time_ms!=pending_capture) return;
    ++vision_stop_log.sequence;
    slot=vision_stop_log.next; frame=&vision_stop_log.frames[slot];
    frame->timing=vision_timing_log.records[(vision_timing_log.next+VISION_TIMING_CAPACITY-1u)%VISION_TIMING_CAPACITY];
    frame->error_px=result->error_px; frame->confidence=result->confidence; frame->servo_cmd=snapshot->servo_cmd;
    frame->pwm_left=snapshot->pwm_left; frame->pwm_right=snapshot->pwm_right;
    frame->target_left=snapshot->target_left; frame->target_right=snapshot->target_right;
    frame->measured_left=(float)snapshot->measured_left; frame->measured_right=(float)snapshot->measured_right;
    frame->valid=result->valid; frame->track_lost=result->track_lost;
    frame->valid_rows=result->valid_rows; frame->reconstructed_rows=result->reconstructed_rows;
    frame->threshold=result->threshold;
    memcpy(frame->control_rows,vision_track_control_rows,CONTROL_BAND_COUNT);
    memset(frame->reserved,0,sizeof(frame->reserved));
    memcpy(frame->left,result->left,120u); memcpy(frame->middle,result->middle,120u);
    memcpy(frame->right,result->right,120u); memcpy(frame->quality,result->row_quality,120u);
    memset(frame->binary,0,sizeof(frame->binary));
    for(i=0u;i<VISION_STOP_PIXELS;++i) if(binary[i]>127u) frame->binary[i>>3]|=(uint8_t)(1u<<(i&7u));
    vision_stop_log.gray_frame_id=pending_frame; vision_stop_log.last_capture_ms=pending_capture;
    vision_stop_log.next=(slot+1u)%VISION_STOP_CAPACITY;
    if(vision_stop_log.count<VISION_STOP_CAPACITY) ++vision_stop_log.count;
    pending=0u;
    ++vision_stop_log.sequence;
}

uint8_t vision_record_replay(uint32_t ordinal, uint8_t gray, uint8_t *image, uint8_t *lines, uint8_t *quality)
{
    const VisionStopFrame *frame;
    uint32_t slot;
    unsigned i;
    if(!vision_stop_log.frozen || !vision_stop_log.count || !image || !lines || !quality) return 0u;
    slot=(vision_stop_log.next+VISION_STOP_CAPACITY-vision_stop_log.count+
          (gray ? vision_stop_log.count-1u : ordinal%vision_stop_log.count))%VISION_STOP_CAPACITY;
    frame=&vision_stop_log.frames[slot];
    if(gray) memcpy(image,vision_stop_log.gray,VISION_STOP_PIXELS);
    else for(i=0u;i<VISION_STOP_PIXELS;++i) image[i]=(frame->binary[i>>3]&(1u<<(i&7u))) ? 255u : 0u;
    memcpy(lines,frame->left,120u); memcpy(lines+120u,frame->middle,120u); memcpy(lines+240u,frame->right,120u);
    memcpy(quality,frame->quality,120u);
    return 1u;
}

void vision_record_display_lines(const VisionResult *result, uint8_t *lines)
{
    unsigned row;
    if(!result || !lines) return;
    for(row=0u;row<120u;++row) {
        uint8_t q=result->row_quality[row];
        lines[row]=q==2u ? result->left[row] : 255u;
        lines[120u+row]=q ? result->middle[row] : 255u;
        lines[240u+row]=q==2u ? result->right[row] : 255u;
    }
}
