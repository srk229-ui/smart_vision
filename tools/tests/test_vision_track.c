#include "vision_track.h"
#include "img_process.h"
#include "control.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static uint8_t binary[CONTROL_IMAGE_WIDTH*CONTROL_IMAGE_HEIGHT]; // 稳定宿主图像，0为背景，255为白路。
static uint32_t fake_us, clock_step; // 宿主注入时钟，用于测量回绕和预算异常分支。

static uint32_t test_clock(void)
{
    fake_us += clock_step;
    return fake_us;
}

int main(void)
{
    ControlParams params;
    VisionResult result, original;
    unsigned row, col, i;
    uint8_t status;
    control_init(92.5f, 85.0f, 100.0f, -1.0f);
    control_get_params(&params, NULL);
    vision_track_init(test_clock);
    vision_reset();
    for (row = 0u; row < CONTROL_IMAGE_HEIGHT; ++row)
        for (col = 40u; col < 148u; ++col) binary[row*CONTROL_IMAGE_WIDTH+col] = 255u;
    vision_process(binary, 1u, 10u, &result); original = result;
    vision_timing_begin(1u, 10u, 0u);
    assert(vision_track_process(binary, &params, &result) == VISION_TRACK_BASELINE);
    assert(memcmp(&result, &original, sizeof(result)) == 0);
    vision_timing_finish();
    assert(vision_timing_log.frames == 1u && !(vision_timing_log.sequence & 1u));
    assert(vision_timing_log.records[0].elapsed_us[VISION_TIME_BEV] == 0u);

    memset(binary, 0, sizeof(binary));
    vision_process(binary, 2u, 20u, &result); original = result;
    assert(vision_track_process(binary, &params, &result) == VISION_TRACK_NO_PATH);
    assert(!result.valid && result.capture_time_ms == 20u);
    assert(memcmp(&result, &original, sizeof(result)) == 0);
    result.capture_time_ms = 999u;
    assert(vision_track_process(binary, &params, &result) == VISION_TRACK_DUPLICATE);
    assert(result.capture_time_ms == 20u); // 重复帧保持第一次的采集标签。
    assert(vision_track_process(NULL, &params, &result) == VISION_TRACK_INVALID);
    params.look_rows[0] = 120u;
    assert(vision_track_process(binary, &params, &result) == VISION_TRACK_INVALID);
    control_get_params(&params, NULL);
    params.look_weights[0] = NAN;
    assert(vision_track_process(binary, &params, &result) == VISION_TRACK_INVALID);
    control_get_params(&params, NULL);

    // 用全白当前帧验证宽白区不能无证据伪造路径，原结果保持失效。
    vision_track_reset(); vision_reset(); memset(binary, 255, sizeof(binary));
    vision_process(binary, 3u, 30u, &result);
    assert(vision_track_process(binary, &params, &result) == VISION_TRACK_NO_PATH);
    assert(!result.valid);

    // 构造需要恢复的图像，注入大时钟步长，超预算必须保持原结果。
    memset(binary, 0, sizeof(binary));
    for (row = 0u; row < CONTROL_IMAGE_HEIGHT; ++row)
        for (col = 25u; col < 140u; ++col) binary[row*CONTROL_IMAGE_WIDTH+col] = 255u;
    vision_process(binary, 4u, 40u, &result);
    result.valid = 0u; result.track_lost = 1u; result.confidence = 0.0f; original = result;
    clock_step = 20000u;
    status = vision_track_process(binary, &params, &result);
    assert(status == VISION_TRACK_BUDGET);
    assert(memcmp(&result, &original, sizeof(result)) == 0);

    // 单边法向恢复接入原11项耗时，保持采集标签；宿主时钟仅检查记账，不代表ARM耗时。
    vision_track_init(test_clock); clock_step=1u; fake_us=0u;
    vision_reset(); memset(binary,0,sizeof(binary));
    for(row=42u;row<120u;++row) for(col=0u;col<=80u;++col) binary[row*188u+col]=255u;
    vision_process(binary,5u,50u,&result); vision_timing_begin(5u,50u,0u);
    assert(vision_track_process(binary,&params,&result)==VISION_TRACK_SINGLE && result.valid);
    vision_timing_finish();
    assert(vision_timing_log.records[0].elapsed_us[VISION_TIME_PROJECT]>0u);
    assert(vision_timing_log.records[0].elapsed_us[VISION_TIME_BASELINE]>0u);
    assert(vision_timing_log.records[0].capture_time_ms==50u && vision_timing_log.records[0].clock_valid);

    // 微秒回绕、64槽覆盖和累计最大值均通过无符号间隔验证。
    vision_track_init(test_clock); clock_step = 0u; fake_us = UINT32_MAX-5u;
    vision_timing_begin(10u, 100u, fake_us);
    fake_us = 4u; vision_timing_set(VISION_TIME_BASELINE, 10u); vision_timing_finish();
    assert(vision_timing_log.records[0].elapsed_us[VISION_TIME_TOTAL] == 10u);
    for (i = 0u; i < 70u; ++i)
    {
        vision_timing_begin(11u+i, 110u+i, fake_us);
        vision_timing_set(VISION_TIME_BASELINE, i); ++fake_us; vision_timing_finish();
    }
    assert(vision_timing_log.frames == 71u && vision_timing_log.next == 7u);
    assert(vision_timing_log.max_us[VISION_TIME_BASELINE] == 69u);
    assert(vision_timing_log.sum_us[VISION_TIME_BASELINE] == 10u+69u*70u/2u);
    vision_track_init(NULL); vision_timing_begin(1u, 0u, 100u);
    vision_timing_set(VISION_TIME_THIN, 123u); vision_timing_finish();
    assert(!vision_timing_log.records[0].clock_valid && !vision_timing_log.records[0].elapsed_us[VISION_TIME_THIN]);
    puts("PASS vision recovery guards, original-result parity, metadata, budget, timing wrap and ring");
    return 0;
}
