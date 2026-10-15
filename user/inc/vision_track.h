#ifndef VISION_TRACK_H
#define VISION_TRACK_H // 普通道路鸟瞰恢复及耗时接口的重复包含保护。

#include "control_types.h"

#define VISION_TRACK_ROWS (116u) // 鸟瞰横向格数，Y从-1.15米到+1.15米。
#define VISION_TRACK_COLS (90u) // 鸟瞰前向格数，X从0.06米到1.84米。
#define VISION_TRACK_CELLS (VISION_TRACK_ROWS * VISION_TRACK_COLS) // 固定网格总格数10440。
#define VISION_TRACK_ITERATIONS (45u) // 最多45轮双子步骤细化；未收敛不发布恢复路径。
#define VISION_TRACK_BEAMS (8u) // 普通道路最多保留8个路径候选，内存和搜索工作均有界。
#define VISION_TRACK_POINTS (80u) // 每条候选最多80格，不在堆上分配路径。
#define VISION_TRACK_BUDGET_US (10000u) // 恢复处理的软件预算10毫秒，尚未实测；超时不发布未完成验证的有效结果。
#define VISION_GROUND_TOP_ROW (42u) // 地面证据的最上图像行，估计ROI；不处理更上部墙面。
#define VISION_NORMAL_WINDOW_M (0.08f) // 可见边线切向窗口的单侧目标弧长，米；以多点抑制像素锯齿。
#define VISION_NORMAL_MIN_SPAN_M (0.04f) // 切向两端至少相隔4厘米；更短的边线不作法向恢复。
#define VISION_NORMAL_MAX_STEP_M (0.15f) // 相邻边线或恢复点最大跳变，米；拒绝跨断裂连线。
#define VISION_TIMING_CAPACITY (64u) // 最近64个新帧的分步骤记录，覆盖后保留累积次数和最大值。

typedef enum
{
    VISION_TIME_COPY = 0, // 主循环领取稳定图像快照的实际微秒间隔。
    VISION_TIME_HISTOGRAM, // 当前灰度帧完整直方图统计的微秒间隔。
    VISION_TIME_OTSU, // Otsu阈值计算的微秒间隔。
    VISION_TIME_BINARY, // 灰度原地二值化的微秒间隔。
    VISION_TIME_BASELINE, // 原逐行寻线及可信度计算的微秒间隔。
    VISION_TIME_BEV, // 鸟瞰索引表采样的微秒间隔；未触发恢复时为0。
    VISION_TIME_THIN, // 双子步骤骨架细化的微秒间隔。
    VISION_TIME_PATH, // 有界候选路径搜索的微秒间隔。
    VISION_TIME_PROJECT, // 单边双向变换/法向恢复及骨架投影的微秒间隔，两者执行时累加。
    VISION_TIME_PUBLISH, // 发布视觉结果给控制器的微秒间隔。
    VISION_TIME_TOTAL, // 从复制开始到视觉发布结束的整帧墙钟微秒间隔，包含抢占和未单列开销。
    VISION_TIME_COUNT // 耗时字段总数11，用作数组容量，不能作为步骤索引。
} VisionTimingStage;

typedef enum
{
    VISION_TRACK_BASELINE = 0, // 原视觉可信，保持原结果且不执行恢复。
    VISION_TRACK_RECOVERED, // 当前帧鸟瞰路径成功；重建行质量为1，不能冒充双边实测。
    VISION_TRACK_NO_PATH, // 当前白路证据不足或路径不能覆盖至少三个控制行，沿用原结果。
    VISION_TRACK_LIMIT, // 细化未在45轮内收敛，沿用原结果，不把粗白路当骨架。
    VISION_TRACK_BUDGET, // 独立时基测到超过恢复软件预算，沿用原结果。
    VISION_TRACK_DUPLICATE, // 重复或回退帧未再次推进历史，不刷新采集时间。
    VISION_TRACK_INVALID, // 输入、控制带或映射无效，不执行恢复。
    VISION_TRACK_GUARDED, // 原结果被近端连续道路的几何筛选修正。
    VISION_TRACK_SINGLE, // 当前可见边线在地面法向偏移半个配置路宽恢复，非历史帧续命。
    VISION_TRACK_ADAPTIVE // 当前连续路径改用更远控制带，原控制参数保持不变。
} VisionTrackStatus;

typedef struct
{
    uint32_t frame_id; // 相机新帧序号，支持uint32回绕。
    uint32_t capture_time_ms; // 原采集逻辑毫秒，耗时统计不会把它改成处理完成时间。
    uint32_t finish_us; // GPT记录的视觉发布完成时刻，微秒，uint32自然回绕。
    uint32_t elapsed_us[VISION_TIME_COUNT]; // 本帧各步骤微秒间隔；包含中断抢占，未执行步骤为0。
    uint16_t thin_iterations; // 实际细化轮数，范围0到45。
    uint16_t path_points; // 选中路径节点数，范围0到80。
    uint8_t recovery_status; // VisionTrackStatus值，明确恢复或回退原因。
    uint8_t mapping_estimated; // 映射来源为估计时为1；不代表相机已实测标定。
    uint8_t clock_valid; // 已安装独立微秒时基时为1；0时耗时不是有效测量。
    uint8_t reserved; // 固定为0，保持结构四字节对齐。
} VisionTimingRecord;

typedef struct
{
    uint32_t sequence; // 主循环写前置奇数、写后置偶数；外部只读必须前后相同且为偶数。
    uint32_t frames; // 累积记录帧数，uint32自然回绕。
    uint32_t next; // 下一次覆盖的环形槽，范围0到63。
    uint32_t max_us[VISION_TIME_COUNT]; // 各步骤自初始化以来的最大微秒间隔，非实测通过门槛。
    uint64_t sum_us[VISION_TIME_COUNT]; // 各步骤累计微秒，可与frames计算平均值。
    VisionTimingRecord records[VISION_TIMING_CAPACITY]; // 最近64帧记录，可离线计算分位数和查看恢复失败。
} VisionTimingLog;

extern uint8_t vision_track_control_rows[CONTROL_BAND_COUNT]; // 最近当前帧实际使用的控制行；有效标记为0时不表示已用于转向。
extern volatile VisionTimingLog vision_timing_log; // 正式与诊断目标均保留的只读记录，不写日志到网络或中断。

// 函数简介     初始化恢复状态、八邻域删除表及分步骤记录。
// 参数说明     clock_us为独立微秒读取函数；NULL表示宿主无有效测量，不启用时间预算。
// 返回参数     void
// 备注信息     主循环初始化时调用；不修改速度环、启动资格或执行器参数。
void vision_track_init(uint32_t (*clock_us)(void));

// 函数简介     清除路径及重复帧历史，不清耗时累计记录。
// 参数说明     无。
// 返回参数     void
// 备注信息     停止代际或参数版本变化后由主循环调用，不在PIT中调用。
void vision_track_reset(void);

// 函数简介     只统计配置地面ROI的灰度直方图，降低上部墙面亮度对Otsu的影响。
// 参数说明     gray为稳定188×120灰度；histogram含256个32位桶。
// 返回参数     无；完整清零桶，原灰度图不改写。
void vision_track_histogram(const uint8_t *gray, uint32_t *histogram);

// 函数简介     原结果不可靠时，用当前二值图的普通道路骨架尝试恢复。
// 参数说明     binary为稳定188×120快照；params提供五行及权重；result先含原视觉结果。
// 返回参数     VisionTrackStatus值；先发布几何筛选结果，恢复失败不重新使用被拒绝的原中线。
// 备注信息     当前帧必须有可见白路；不读取地图、真实姿态、路段编号或环岛状态。
uint8_t vision_track_process(const uint8_t *binary, const ControlParams *params, VisionResult *result);

// 函数简介     开始一帧耗时记录并清零本帧步骤。
// 参数说明     frame_id及capture_ms为采集元数据；started_us为复制开始的独立微秒。
// 返回参数     void
// 备注信息     仅主循环单写；同一帧不得反复作为新帧记录。
void vision_timing_begin(uint32_t frame_id, uint32_t capture_ms, uint32_t started_us);

// 函数简介     保存一个步骤的实际微秒间隔。
// 参数说明     stage为有效步骤枚举；elapsed_us为无符号结束减开始，兼容单次计时回绕。
// 返回参数     void
// 备注信息     不使用control_now_ms计算性能；没有独立时基时写0。
void vision_timing_set(VisionTimingStage stage, uint32_t elapsed_us);

// 函数简介     发布本帧记录到固定环形区并更新最大值和累计值。
// 参数说明     无。
// 返回参数     void
// 备注信息     记录区单写，主循环保持中断开放，不在这里打印或发送图像。
void vision_timing_finish(void);

#endif
