#ifndef CAMERA_FRAME_H
#define CAMERA_FRAME_H

#include <stdint.h>

// 函数简介     相机初始化成功后包装原 CSI 回调，保留原双缓冲行为
// 参数说明     void
// 返回参数     void
// 备注信息     相机驱动已建立句柄后在主循环调用，防止把包装回调保存成自己的原回调
void camera_frame_install(void);

// 函数简介     领取一致的新帧快照，复制期间换帧时拒绝该快照
// 参数说明     destination     至少 188×120 字节；frame_id/capture_time_ms 为输出元数据
// 返回参数     uint8_t         成功 1，无新帧或快照失效 0
// 备注信息     三个指针均必填；复制失败时目标缓冲不可用于本帧处理
//              仅元数据领取/复核用短临界区，整图复制前恢复原中断状态，不强制开启外层已关闭的IRQ
//              时间来自完成回调读取的10ms逻辑时基；DMA/cache一致性仍需实机验证
uint8_t camera_frame_take(uint8_t *destination, uint32_t *frame_id, uint32_t *capture_time_ms);

// 函数简介     读取消费前覆盖或复制竞争造成的丢帧计数
// 参数说明     void
// 返回参数     uint32_t        软件观察到的丢帧数，不统计正常重复读取
// 备注信息     含未消费覆盖与复制时换帧，不代表相机所有物理采集失败数
uint32_t camera_frame_dropped(void);

#endif
