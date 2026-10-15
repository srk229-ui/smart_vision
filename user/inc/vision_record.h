#ifndef VISION_RECORD_H
#define VISION_RECORD_H // 停车视觉取证接口重复包含保护。

#include "vision_track.h"

#define VISION_STOP_CAPACITY (8u) // 最近8个采样帧；约80ms一帧，可覆盖500ms视觉保护前后。
#define VISION_STOP_INTERVAL_MS (80u) // 二值帧最小留存间隔，逻辑毫秒；不是相机帧率。
#define VISION_STOP_PIXELS (CONTROL_IMAGE_WIDTH*CONTROL_IMAGE_HEIGHT) // 一幅188×120图像的像素数。
#define VISION_STOP_PACKED (VISION_STOP_PIXELS/8u) // 逐像素最低位优先的二值打包字节数2820。
#define VISION_STOP_MAGIC (0x31525356u) // 小端文件头VSR1，离线解码需同时校验ABI和长度。

typedef struct
{
    VisionTimingRecord timing; // 同一采集帧的步骤、恢复状态和原始时间标签。
    float error_px; // 当前视觉误差；valid为0时不可用于控制。
    float confidence; // 当前帧可信度0到1，不用历史有效帧填充。
    float servo_cmd; // 记录时限速后的舵机命令刻度，非实测前轮角。
    float pwm_left; // 记录时左轮实际PWM命令计数。
    float pwm_right; // 记录时右轮实际PWM命令计数。
    float target_left; // 左轮有效目标counts/10ms。
    float target_right; // 右轮有效目标counts/10ms。
    float measured_left; // 最近控制周期左轮增量counts/10ms。
    float measured_right; // 最近控制周期右轮增量counts/10ms。
    uint32_t valid; // 当前帧整体有效标记，0或1。
    uint32_t track_lost; // 当前帧整体丢线标记，0或1。
    uint32_t valid_rows; // 当前有效图像行数量0到120。
    uint32_t reconstructed_rows; // 其中重建行数量，不得超过valid_rows。
    uint32_t threshold; // 本帧Otsu阈值0到255，用于复核墙面排除与曝光影响。
    uint8_t control_rows[CONTROL_BAND_COUNT]; // 本帧实际采用的五个图像行，valid为0时只是尝试值。
    uint8_t reserved[3]; // 固定为0，保持小端布局四字节对齐。
    uint8_t left[CONTROL_IMAGE_HEIGHT]; // 原左边线坐标，质量0时为无效占位。
    uint8_t middle[CONTROL_IMAGE_HEIGHT]; // 原中线坐标，须结合quality读取。
    uint8_t right[CONTROL_IMAGE_HEIGHT]; // 原右边线坐标，须结合quality读取。
    uint8_t quality[CONTROL_IMAGE_HEIGHT]; // 0无效、1重建、2当前双边。
    uint8_t binary[VISION_STOP_PACKED]; // 当前二值原图，未叠加任何边线或状态字。
} VisionStopFrame;

typedef struct
{
    uint32_t magic; // 固定VISION_STOP_MAGIC，外部工具据此识别格式。
    volatile uint32_t sequence; // 主循环写前奇数、写后偶数；外部需前后相同偶数。
    uint32_t abi; // 当前固定1，字段布局改变时必须递增。
    uint32_t bytes; // 整个结构体字节数，由固件sizeof给出。
    uint32_t generation; // 每次观察到新的TRACK运行递增，旧记录在新运行时清除。
    uint32_t frozen; // 停止后为1，禁止后续停车图像覆盖。
    uint32_t count; // 已保存的有效槽数，范围0到8。
    uint32_t next; // 下一次写入的槽号，范围0到7。
    uint32_t stop_ms; // 主循环首次观察到停车时的控制逻辑毫秒。
    uint32_t fault; // 停车快照故障码；人工STOP通常为0。
    uint32_t stop_state; // 停车时的ControlState，不能据此推断物理静止。
    uint32_t gray_frame_id; // 灰度图与最近保存二值帧对应的采集编号。
    uint32_t frame_bytes; // VisionStopFrame字节数，离线工具严格核对。
    uint32_t packed_bytes; // 每幅打包二值图字节数2820。
    uint32_t last_capture_ms; // 最后一个保存帧的原采集逻辑时间。
    uint32_t reserved; // 保留为0，固定64字节头。
    VisionStopFrame frames[VISION_STOP_CAPACITY]; // 物理槽序存储，离线按next/count还原时间顺序。
    uint8_t gray[VISION_STOP_PIXELS]; // 最后一个已保存帧的灰度图，未二值化、未叠线。
    VisionTimingLog timing; // 停车时最近64帧耗时快照，之后不再覆盖。
} VisionStopLog;

extern VisionStopLog vision_stop_log; // 正式和诊断目标均可只读导出；不在中断写入。

// 函数简介     主循环观察运行到停止的变化，冻结记录；再次人工启动时开始新一组。
// 参数说明     snapshot必须由控制API一致复制，诊断BENCH不触发本记录。
// 返回参数     无；未运行过时不会把停车预览冻结。
void vision_record_observe(const ControlSnapshot *snapshot);

// 函数简介     二值化之前按80ms间隔保存当前灰度图，等待同帧处理结果。
// 参数说明     gray是稳定快照；frame_id/capture_ms沿用采集标签。
// 返回参数     无；停车和已冻结时不复制。
void vision_record_capture(const uint8_t *gray, uint32_t frame_id, uint32_t capture_ms);

// 函数简介     完成视觉和耗时发布后保存对应二值帧、质量及控制快照。
// 参数说明     binary、result须与capture对应；snapshot用于记录实际控制状态。
// 返回参数     无；重复或不匹配帧不写入。
void vision_record_frame(const uint8_t *binary, const VisionResult *result, const ControlSnapshot *snapshot);

// 函数简介     为停车图传恢复冻结图和质量，输出按时间顺序选择的帧。
// 参数说明     gray非零取最后灰度图，否则ordinal选择二值帧；lines为三组120行坐标。
// 返回参数     有冻结记录为1；所有写入在主循环，调用者仍须检查车辆未运行。
uint8_t vision_record_replay(uint32_t ordinal, uint8_t gray, uint8_t *image, uint8_t *lines, uint8_t *quality);

// 函数简介     将无效显示坐标设为255供图传筛选器剔除，重建行不伪装为实测双边。
// 参数说明     result是原结果；lines是三组120字节的独立显示缓存。
// 返回参数     无；不改视觉输入和控制结果。
void vision_record_display_lines(const VisionResult *result, uint8_t *lines);

#endif
