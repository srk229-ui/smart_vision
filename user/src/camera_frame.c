#include "zf_common_headfile.h"
#include "control.h"
#include "camera_frame.h"

static csi_transfer_callback_t original_callback; // 安装前SDK回调，负责原双缓冲提交/完成处理，不能被包装层省略。
static void *original_user_data; // 与原回调成对保存的上下文，转发使用安装时的原值。
static volatile uint32_t frame_sequence; // CSI每成功发布一帧加1；volatile用于中断/main共享，uint32允许回绕。
static volatile uint32_t captured_ms; // 成功完成事件的10ms量化逻辑时间，不是复制完成时刻。
static uint8_t * volatile completed_image; // 原回调确认的完成缓冲地址；后续DMA可复用内容，复制后须再检查序号。
static uint32_t consumed_sequence; // main已经消费/拒绝的旧序号，避免同一失败重复计数。
static uint32_t dropped_frames; // 已发布但未消费或复制被拒的帧数，不代表全部硬件漏帧。

// 回调只发布元数据，不复制整帧/计算视觉/发网络；原SDK仍承担双缓冲职责。
static void frame_callback(CSI_Type *base, csi_handle_t *handle, status_t status, void *user_data)
{
    (void)user_data; // 使用保存的原userData，不把包装入口上下文传错给原回调。
    // 1. 先清旧完成标志，原回调取得完整缓冲才重新置位，避免旧图被标成新图。
    if (status == kStatus_CSI_FrameDone) mt9v03x_finish_flag = 0u;
    if (original_callback != NULL) original_callback(base, handle, status, original_user_data); // 原回调先执行，保留缓冲切换。
    // 2. 状态和原驱动完成flag须同时成立；失败/其他事件不刷新采集年龄。
    if (status == kStatus_CSI_FrameDone && mt9v03x_finish_flag)
    {
        completed_image = (uint8_t *)mt9v03x_image; // 先发布地址/时间，最后增加序号。
        captured_ms = control_now_ms(); // 若PIT还未服务，可读到上一tick逻辑时刻；真实时序仍待测。
        ++frame_sequence;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     相机初始化后包装原 CSI 回调，保持逐飞双缓冲处理
// 参数说明     void
// 返回参数     void
// 备注信息     只在应用层安装，不修改共享驱动；不得重复包装同一回调
//-------------------------------------------------------------------------------------------------------------------
void camera_frame_install(void)
{
    // 安装时短暂屏蔽IRQ，避免回调保存/替换中途被CSI调用。
    uint32_t key = interrupt_global_disable();
    if (csi_handle.callback != frame_callback) // 不重复包装自身，防止original_callback指回本函数形成递归。
    {
        original_callback = csi_handle.callback;
        original_user_data = csi_handle.userData;
        frame_sequence = consumed_sequence = dropped_frames = 0u; // 统计从包装后的发布事件重新开始。
        completed_image = NULL; // 还没有包装层确认的新图，领取时先拒绝旧缓冲。
        csi_handle.callback = frame_callback;
    }
    interrupt_global_enable(key); // 恢复原PRIMASK，不破坏调用者已有的关中断状态。
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     领取并复制新帧，复制期间采集发生变化时丢弃该快照
// 参数说明     destination     至少容纳 188×120 字节的图像空间
// 参数说明     frame_id/capture_time_ms    对应采集完成事件的元数据
// 返回参数     uint8_t         成功为 1，没有新帧或快照失效为 0
// 备注信息     不额外屏蔽中断复制整帧；结束后不清新事件标志
//-------------------------------------------------------------------------------------------------------------------
uint8_t camera_frame_take(uint8_t *destination, uint32_t *frame_id, uint32_t *capture_time_ms)
{
    uint8_t *source; // 当前原缓冲地址，仅在本次复制时使用，不让视觉长期持有。
    uint32_t sequence; // 与指针成对领取的事件编号。
    uint32_t timestamp; // 与事件编号对应的采集逻辑时刻。
    uint32_t key;
    // 1. 图像与两项元数据均须有输出空间，先校验再进入临界区。
    if (destination == NULL || frame_id == NULL || capture_time_ms == NULL) return 0u;
    // 2. 编号/时间/指针同锁领取，防止main读到跨两个采集事件的混合组合。
    key = interrupt_global_disable();
    sequence = frame_sequence;
    timestamp = captured_ms;
    source = completed_image;
    if (source == NULL || sequence == consumed_sequence) // 尚无完成图，或该编号已消费。
    {
        interrupt_global_enable(key);
        return 0u;
    }
    mt9v03x_finish_flag = 0u; // 只清领取时旧事件，不在算法末尾误清新事件。
    interrupt_global_enable(key);
    // 3. 恢复原中断状态后复制22560字节；正常main开放IRQ时PIT/CSI可继续服务，故结束再查序号。
    memcpy(destination, source, CONTROL_IMAGE_WIDTH * CONTROL_IMAGE_HEIGHT);
    key = interrupt_global_disable();
    // 4. 复制期间有新完成事件，拒绝本快照；最新事件留待下一轮领取。
    if (sequence != frame_sequence)
    {
        dropped_frames += sequence - consumed_sequence; // 跳过的旧编号及本次拒绝帧都计入，无符号差值支持回绕。
        consumed_sequence = sequence; // 推进至被拒旧编号，同一失败不反复增加统计。
        interrupt_global_enable(key);
        return 0u;
    }
    // 5. 成功帧前还有未消费的中间编号时计丢帧，本次成功帧自身不计入。
    if ((uint32_t)(sequence - consumed_sequence) > 1u)
        dropped_frames += sequence - consumed_sequence - 1u;
    consumed_sequence = sequence; // 每次成功发布编号只消费一次。
    *frame_id = sequence;
    *capture_time_ms = timestamp; // 传采集时刻而非现在处理时刻，旧图不能因此重新变新鲜。
    interrupt_global_enable(key);
    return 1u;
}

uint32_t camera_frame_dropped(void)
{
    return dropped_frames; // main消费路径的32位统计；真实DMA/缓存行为仍需M3验证。
}
