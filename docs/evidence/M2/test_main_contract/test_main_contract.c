
#include "control_types.h"
#include "img_process.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>
static VisionResult vision;
static uint32_t now = 100;
uint32_t control_now_ms(void) { return now; }
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

int main(void) {
    ControlParams params;
    ControlSnapshot state;
    unsigned row;
    const uint8_t band[5] = {80,85,90,95,100};
    memset(&params,0,sizeof(params)); memcpy(params.look_rows,band,5);
    memset(&state,0,sizeof(state)); state.state=CONTROL_READY;
    memset(&vision,0,sizeof(vision)); vision.valid=1; vision.confidence=1; vision.capture_time_ms=now;
    for(row=0;row<120;++row) { vision.left[row]=93; vision.right[row]=95; vision.row_quality[row]=2; }
    assert(calibrate_width(&params,&state)==0);
    for(row=0;row<120;++row) { vision.left[row]=40; vision.right[row]=148; }
    assert(calibrate_width(&params,&state)==1);
    state.state=CONTROL_RUNNING; assert(calibrate_width(&params,&state)==0);
    state.state=CONTROL_READY; state.measured_left=2; assert(calibrate_width(&params,&state)==0);
    state.measured_left=0; now+=110; assert(calibrate_width(&params,&state)==0);
    vision.capture_time_ms=now; vision.row_quality[90]=0; assert(calibrate_width(&params,&state)==0);
    vision.row_quality[90]=2; vision.error_px=10; assert(calibrate_width(&params,&state)==0);
    puts("PASS main width calibration contract: 7 scenarios (real function)");
    return 0;
}
