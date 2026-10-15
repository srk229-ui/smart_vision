#include "calibration_clock.h"
#include "fsl_clock.h"
#include "fsl_gpt.h"

static uint8_t clock_ready; // 仅在GPT1配置通过后置1；初始化阶段使用，不参与速度PI逻辑时间。

uint8_t calibration_clock_init(void)
{
    gpt_config_t config;
    uint32_t frequency = CLOCK_GetFreq(kCLOCK_PerClk); // 回读实际PERCLK，禁止假设始终等于板级配置75MHz。
    uint32_t divider;
    if (clock_ready) return 1u; // 重复初始化不清正在运行的计时器，避免破坏已有试验和相邻差值。
    clock_ready = 0u;
    if (frequency < 1000000u || frequency % 1000000u != 0u) return 0u; // 必须能精确分出1MHz。
    divider = frequency / 1000000u;
    if (divider == 0u || divider > 4096u) return 0u; // GPT预分频寄存器范围1至4096。
    /* 独立GPT1时基 */
    CLOCK_EnableClock(kCLOCK_Gpt1);
    CLOCK_EnableClock(kCLOCK_Gpt1S); // 板级初始化关闭过串行时钟门控，显式恢复本计时器所需门控。
    GPT_GetDefaultConfig(&config);
    config.clockSource = kGPT_ClockSource_Periph;
    config.divider = divider;
    config.enableFreeRun = true; // 不使用默认比较后重启模式，保持连续32位计数。
    config.enableRunInDbg = true; // 暂停后的长间隔可在恢复时发现；运动测量仍禁止打断点。
    config.enableMode = true; // 只在本次上电启用时清零，后续试验禁止复位GPT。
    GPT_Init(GPT1, &config);
    GPT_StartTimer(GPT1);
    clock_ready = 1u;
    return 1u;
}

uint32_t calibration_clock_now_us(void)
{
    return clock_ready ? GPT_GetCurrentTimerCount(GPT1) : 0u; // 不修改PIT周期、控制逻辑ms或WIFI的DWT。
}
