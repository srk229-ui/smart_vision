#include "vision_record.h"
#include "img_process.h"
#include "control.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static uint8_t binary[VISION_STOP_PIXELS],gray[VISION_STOP_PIXELS],replay[VISION_STOP_PIXELS];
static uint8_t lines[360],quality[120]; // 独立显示副本，不能进入视觉控制。
static ControlParams params;
static VisionResult result;
static ControlSnapshot snapshot;

static void road(unsigned left,unsigned right)
{
    unsigned y,x;
    memset(binary,0,sizeof(binary));
    for(y=42;y<120;++y) for(x=left;x<=right;++x) binary[y*188u+x]=255u;
}

static uint8_t frame(uint32_t id,uint32_t ms)
{
    vision_process(binary,id,ms,&result);
    vision_timing_begin(id,ms,0u);
    return vision_track_process(binary,&params,&result);
}

int main(void)
{
    unsigned i,y,x;
    uint8_t status;
    control_init(92.5f,85.0f,100.0f,-1.0f); control_get_params(&params,NULL);
    vision_track_init(NULL); vision_reset();
    road(40u,140u); assert(frame(1u,10u)==VISION_TRACK_BASELINE && result.valid);
    // 上部白墙不参与阈值统计，地面灰度分布保持一致。
    {
        uint32_t histogram[256],sum=0u;
        memset(gray,80,sizeof(gray)); memset(gray,255,42u*188u);
        vision_track_histogram(gray,histogram);
        for(i=0u;i<256u;++i) sum+=histogram[i];
        assert(sum==78u*188u && histogram[80]==sum && !histogram[255]);
    }
    // 近带原结果高可信时，远带也必须来自整幅连续道路筛选，不能保留原结果中的伪造远点。
    vision_track_reset(); vision_reset(); road(40u,140u);
    vision_process(binary,1u,10u,&result);
    result.middle[70]=20u; result.left[70]=5u; result.right[70]=35u; result.row_quality[70]=2u;
    (void)vision_track_process(binary,&params,&result);
    assert(result.valid && result.row_quality[70]==2u && result.middle[70]==90u);
    vision_track_reset(); vision_reset(); road(40u,140u); (void)frame(1u,10u);
    // 当前可见单边按地面法向恢复，持续超过三帧，不再依赖最近像素宽度。
    road(0u,80u);
    for(i=2u;i<=12u;++i) {
        status=frame(i,i*30u);
        assert(status==VISION_TRACK_SINGLE && result.valid && result.error_px>50.0f);
        assert(result.confidence==0.5f && result.capture_time_ms==i*30u);
    }
    // 对称右弯单边不能误改为左转；同一帧重复调用保留原采集标签。
    vision_track_reset(); vision_reset(); road(40u,140u); (void)frame(30u,400u);
    road(108u,187u);
    for(i=31u;i<40u;++i) { status=frame(i,400u+(i-30u)*30u); assert(status==VISION_TRACK_SINGLE && result.error_px<-50.0f); }
    result.capture_time_ms=9999u;
    assert(vision_track_process(binary,&params,&result)==VISION_TRACK_DUPLICATE && result.capture_time_ms==670u);
    // 即使近期像素宽度已经过期，仍可用配置路宽和当前真实边线恢复。
    vision_track_reset(); vision_reset(); road(40u,140u); (void)frame(1u,10u); road(0u,80u);
    status=frame(13u,2000u);
    assert(result.valid && status==VISION_TRACK_SINGLE);
    vision_track_reset(); vision_reset();
    status=frame(14u,2010u); assert(result.valid && status==VISION_TRACK_SINGLE);

    // 中间地面小白斑、图像上部墙面和另一侧白块不应拉走近端连续赛道。
    vision_track_reset(); vision_reset(); road(40u,140u); (void)frame(20u,2100u);
    road(0u,80u);
    for(y=0u;y<35u;++y) for(x=0u;x<188u;++x) binary[y*188u+x]=255u;
    for(y=42u;y<120u;++y) {
        for(x=91u;x<=100u;++x) binary[y*188u+x]=255u;
        for(x=150u;x<188u;++x) binary[y*188u+x]=255u;
    }
    (void)frame(21u,2130u);
    assert(result.valid && result.error_px>50.0f && result.row_quality[20]==0u);
    // 即使原算法把背景标成高可信，仍要经过当前道路检查，不能直接绕过恢复。
    vision_process(binary,25u,2140u,&result);
    result.valid=1u;result.track_lost=0u;result.confidence=1.0f;result.error_px=0.0f;
    for(i=0u;i<5u;++i) result.row_quality[params.look_rows[i]]=2u;
    (void)vision_track_process(binary,&params,&result);
    assert(result.valid && result.error_px>50.0f);
    // 近端完全没有道路时，远处白墙即使带两条边也不能自动恢复循迹。
    memset(binary,0,sizeof(binary));
    for(y=20u;y<75u;++y) for(x=35u;x<150u;++x) binary[y*188u+x]=255u;
    (void)frame(26u,2160u); assert(!result.valid);
    // 原先必须前移控制带的连续单边，现在允许由地面法向直接恢复。
    vision_track_reset(); vision_reset();
    for(y=0u;y<120u;++y) {
        memset(binary+y*188u,0,188u);
        if(y>=42u) {
            unsigned left=y>=84u ? 0u : (84u-y)*2u;
            for(x=left;x<=110u;++x) binary[y*188u+x]=255u;
        }
    }
    status=frame(23u,2200u);
    assert((status==VISION_TRACK_ADAPTIVE || status==VISION_TRACK_SINGLE) && result.valid && result.confidence<=0.6f);
    // 质量0中线绝不再作为94列发送；质量1只发当前重建中点，不伪造双边。
    memset(&result,0,sizeof(result)); memset(result.middle,94,sizeof(result.middle));
    result.row_quality[80]=1u; result.middle[80]=30u;
    vision_record_display_lines(&result,lines);
    assert(lines[120u+90u]==255u && lines[80]==255u && lines[240u+80u]==255u && lines[200]==30u);

    // 无相机新帧时也能从运行转停止冻结；人工STOP与视觉故障都不丢证据。
    memset(&snapshot,0,sizeof(snapshot)); snapshot.state=CONTROL_RUNNING; snapshot.mode=CONTROL_MODE_TRACK;
    vision_record_observe(&snapshot);
    for(i=0u;i<12u;++i) {
        memset(gray,(int)(i+1u),sizeof(gray)); road(20u,120u);
        result.frame_id=100u+i; result.capture_time_ms=3000u+i*80u;
        vision_record_capture(gray,result.frame_id,result.capture_time_ms);
        vision_timing_begin(result.frame_id,result.capture_time_ms,0u); vision_timing_finish();
        vision_record_frame(binary,&result,&snapshot);
    }
    assert(vision_stop_log.count==8u && vision_stop_log.next==4u && sizeof(VisionStopFrame)==3428u);
    snapshot.state=CONTROL_FAULT; snapshot.fault=CONTROL_FAULT_VISION_TIMEOUT; snapshot.timestamp_ms=4500u;
    vision_record_observe(&snapshot);
    assert(vision_stop_log.frozen && vision_stop_log.fault==CONTROL_FAULT_VISION_TIMEOUT);
    assert(vision_record_replay(0u,0u,replay,lines,quality));
    assert(!memcmp(replay,binary,sizeof(binary)) && vision_stop_log.frames[4].timing.frame_id==104u);
    assert(vision_record_replay(0u,1u,replay,lines,quality) && !memcmp(replay,gray,sizeof(gray)));
    {
        uint32_t seq=vision_stop_log.sequence;
        memset(gray,0,sizeof(gray)); vision_record_capture(gray,999u,9999u);
        vision_record_frame(binary,&result,&snapshot); vision_record_observe(&snapshot);
        assert(vision_stop_log.sequence==seq && vision_stop_log.gray[0]==12u);
    }
    // 导出真实C布局样本供离线解码器回归，不连接任何硬件。
    {
        FILE *file=NULL;
#ifdef _MSC_VER
        assert(fopen_s(&file,"vision_stop_fixture.bin","wb")==0 && file);
#else
        file=fopen("vision_stop_fixture.bin","wb"); assert(file);
#endif
        assert(fwrite(&vision_stop_log,sizeof(vision_stop_log),1u,file)==1u); fclose(file);
    }
    snapshot.state=CONTROL_RUNNING; snapshot.fault=CONTROL_FAULT_NONE; vision_record_observe(&snapshot);
    assert(!vision_stop_log.frozen && !vision_stop_log.count && vision_stop_log.generation==2u);
    snapshot.state=CONTROL_READY; vision_record_observe(&snapshot);
    assert(vision_stop_log.frozen && !vision_stop_log.count && !vision_record_replay(0u,0u,replay,lines,quality));
    puts("PASS first curve: prolonged single edge, distractors, no near road, adaptive band, invalid display, fault/STOP freeze and replay");
    return 0;
}
