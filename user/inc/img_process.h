#ifndef _IMG_PROCESS_H_
#define _IMG_PROCESS_H_ // 视觉接口头文件的重复包含保护。

#include <stdint.h>
#include "control_types.h"

// 函数简介     获取完整清零的32位灰度直方图
// 参数说明     _img为8位灰度图；_rows/_cols为尺寸；_hist_gram为256个uint32_t空间
// 返回参数     void
// 备注信息     输出有效时先清零全部256桶；空图像得到零计数，不保留旧帧统计
void get_hist_gram(unsigned char *_img, unsigned short _rows, unsigned short _cols,
                   uint32_t *_hist_gram);

// 函数简介     以最大类间差异计算Otsu阈值，处理空直方图和纯色输入
// 参数说明     _hist_gram为已统计的256级灰度计数，不修改输入
// 返回参数     unsigned char阈值，范围0～255；空输入为0，纯色为该灰度值
// 备注信息     低灰度类为≤阈值，高灰度类为>阈值；配合二值化后纯色图全部为黑
unsigned char get_threshold_otsu(const uint32_t *_hist_gram);

// 函数简介     将灰度图原地转换为0/255二值图
// 参数说明     _img容量至少为_rows×_cols字节；_threshold_value为灰度阈值
// 返回参数     void
// 备注信息     像素严格大于阈值才为255，等于阈值为0；调用后原灰度内容被覆盖
void binaryzation_process(unsigned char *_img, unsigned short _rows, unsigned short _cols,
                          unsigned int _threshold_value);

// 函数简介     保留兼容的逐行扫描入口，新控制链使用vision_process
// 参数说明     二值图、尺寸及边线输出；_threshold_val只保留签名兼容，不参与判白
// 返回参数     void
// 备注信息     仅输出双边实测；无效行中点为显示占位，没有有效标志，不用于当前驾驶
void auxiliary_process(uint8_t *_src_pixel_mat, uint8_t _src_rows, uint8_t _src_cols,
                        unsigned char _threshold_val, uint8_t *_left_line,
                        uint8_t *_mid_line, uint8_t *_right_line);

// 函数简介：清除跨帧种子、连续重建计数和帧缓存，保留人工标定及控制带配置。
// 参数说明：无。
// 返回参数：无。
// 备注信息：主循环串行调用；新试验前清时间历史，不发车、不清除已设置标定。
//           上电参考宽度全为0，未经人工标定不得假定单边重建已经可用。
void vision_reset(void);

// 函数简介：处理固定188×120二值图，生成边界、置信度和加权图像偏差。
// 参数说明：输入为连续0/255稳定快照；frame_id由相机计数，可按uint32_t自然回绕；
//           相邻比较间隔须小于2^31。capture_time_ms由采集回调记录，单位ms，
//           当前是10ms量化设备逻辑时间；result提供VisionResult输出空间。
// 返回参数：整体有效为1，否则为0；result->track_lost同时标明整体失效。
// 备注信息：至少3个控制行且置信度≥0.35才有效；实测质量系数1，重建0.5。
//           偏差=94-质量加权中点，单位像素，赛道在图像左侧为正。
//           仅已标定单边可重建，整图出现单边候选的连续新帧最多允许3帧。
//           重复帧返回缓存，不刷新采集时间或重建期限；不修改输入图像。
//           调用后须另填result->threshold；无效行默认中点和零偏差仅供显示。
//           本模块状态由主循环串行访问，不在PIT内并发处理图像或设置配置。
uint8_t vision_process(const uint8_t *binary_after_otsu, uint32_t frame_id,
                       uint32_t capture_time_ms, VisionResult *result);

// 函数简介：设置人工居中直道标定宽度，定义为左右白段端点的坐标差。
// 参数说明：row为0～119图像行号；width单位像素，3～187；0清除该行标定。
// 返回参数：接受为1；非法行号/宽度为0，保留原值。
// 备注信息：调用者仅在停止态串行修改，函数本身不检测控制状态。
//           宽度不是白像素个数或真实车道厘米数；虚拟边界可在画面外。
//           输出显示边界会限幅，但中点先以未限幅虚拟边界计算。
uint8_t vision_set_reference_width(uint16_t row, uint16_t width);

// 函数简介     只读查询已提交的人工直道宽度，供恢复触发和重建显示使用。
// 参数说明     row为0到119图像行号。
// 返回参数     像素宽度；未标定或非法行号返回0。
// 备注信息     不修改原寻线历史，不把估计路宽写回人工标定数组。
uint16_t vision_get_reference_width(uint16_t row);

// 函数简介：完整校验并归一化后提交五行前瞻控制带及相对权重。
// 参数说明：rows严格递增且均小于120；weights必须为有限正数，无量纲。
// 返回参数：接受为1；任何一项非法为0，保留原配置。
// 备注信息：调用者仅在停止态串行修改；默认行80/85/90/95/100及权重1/2/4/2/1。
//           设置后权重存为总和为1的比例；质量加权时会按可用证据再次归一化中点。
//           完整校验不代表跨中断原子发布；本模块依赖主循环无并发访问的调用约束。
uint8_t vision_set_control_band(const uint8_t rows[CONTROL_BAND_COUNT],
                                const float weights[CONTROL_BAND_COUNT]);

#endif
