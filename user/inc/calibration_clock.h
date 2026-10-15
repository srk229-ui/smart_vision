#ifndef CALIBRATION_CLOCK_H
#define CALIBRATION_CLOCK_H // GPT1独立标定时基接口防重复包含。
#include <stdint.h>
// 函数简介     初始化空闲GPT1为1MHz自由运行计数
// 参数说明     void
// 返回参数     uint8_t         初始化配置合法为1，否则0
// 备注信息     PIT前调用，不修改共享PERCLK根分频，不重置WIFI使用的DWT
uint8_t calibration_clock_init(void);
// 函数简介     读取独立硬件微秒时戳
// 参数说明     void
// 返回参数     uint32_t        GPT1低32位微秒计数，约71.58分钟回绕
// 备注信息     相邻读取用无符号差值，不能跨越多次回绕；未初始化返回0
uint32_t calibration_clock_now_us(void);
#endif
