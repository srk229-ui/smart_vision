/*********************************************************************************************************************
* RT1064DVL6A Opensourec Library 即（RT1064DVL6A 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
* 
* 本文件是 RT1064DVL6A 开源库的一部分
* 
* RT1064DVL6A 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
* 
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
* 
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
* 
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
* 
* 文件名称          isr
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          IAR 8.32.4 or MDK 5.33
* 适用平台          RT1064DVL6A
* 店铺链接          https://seekfree.taobao.com/
* 
* 修改记录
* 日期              作者                备注
* 2022-09-21        SeekFree            first version
********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "isr.h"
#include "main.h"

int16 encoder_data_1 = 0; // 左轮最近一次10ms窗口的有符号增量，已按前进方向修正
int16 encoder_data_2 = 0; // 右轮最近一次10ms窗口的有符号增量，供统一速度闭环使用
int16 encoder_data_3 = 0;
int16 encoder_data_4 = 0;



// CSI只推进SDK采集/换缓冲，并通过包装回调发布元数据；图像计算在主循环执行。
void CSI_IRQHandler(void)
{
    CSI_DriverIRQHandler();     // 调用SDK自带的中断函数 这个函数最后会调用我们设置的回调函数
    __DSB();                    // 数据同步隔离
}

void SysTick_Handler(void)
{

}

// CH0已配置10ms：读取编码器→保护/闭环→硬件写出；图像、显示及SPI等待不在此执行。
// 这是配置周期，真实入口间隔、最坏耗时和漏服务必须用M3独立仪器确认。
void PIT_IRQHandler(void)
{
    // CH1先处理，CH0同时到期时本轮闭环即可看到STOP并写零。
    if (pit_flag_get(PIT_CH1))
    {
        local_drive_scan();
        pit_flag_clear(PIT_CH1);
    }
    if (pit_flag_get(PIT_CH0))
    {
        // 1. 本ISR是计数器唯一采样者：持续硬件计数求模差分，不再周期清零。
        // raw保留16位位模式，测量模块先用宽整数校正方向，保证送PI的int16增量可表示。
        uint32_t now_us = calibration_clock_now_us(); // 独立真实时刻，不代替原控制逻辑10ms时间。
        EncoderMeasurementSnapshot measurement;
        uint8_t accepted = encoder_measurement_sample((uint16_t)encoder_get_count(ENCODER_1),
                                                       (uint16_t)encoder_get_count(ENCODER_2), now_us);
        encoder_measurement_get_snapshot(&measurement);
        encoder_data_1 = measurement.delta_left;
        encoder_data_2 = measurement.delta_right;
        // 诊断版遇时窗异常作废并停车；正式版保留可解码计数的原PI单位，歧义差分仍锁故障。
        calibration_debug_before_tick(now_us, &measurement); // STOP优先，再记录本次测量异常原因，均先于输出。
        if (!accepted && (CALIBRATION_DEBUG_ENABLE || !measurement.counter_valid))
            control_report_invalid_fault();

        // 2. 控制器先处理停止/故障，再计算斜坡、PI与舵机；使用短临界区取得一致输出。
        ControlSnapshot output; // 含本轮整数PWM、舵机命令与输出门控，不读取旧全局PID输出
        control_tick(encoder_data_1, encoder_data_2);
        control_get_snapshot(&output);
        // 3. 电机仅提供前进命令；负的PI需求已在控制器按0限幅，不在这里反转DIR。
        // STOP/FAULT跳过斜坡并给出零PWM；寄存器写入之后仍需等待外设重载，机械停住更晚。
        gpio_set_level(MOTOR1_DIR, MOTOR1_FORWARD_DIR_LEVEL);
        gpio_set_level(MOTOR2_DIR, MOTOR2_FORWARD_DIR_LEVEL);
        pwm_set_duty(MOTOR1_PWM, (uint32)output.pwm_left);
        pwm_set_duty(MOTOR2_PWM, (uint32)output.pwm_right);
        // 舵机50Hz脉冲由PWM外设独立产生，不占用10ms任务等待脉宽的时间。
        // 命令角按脉宽公式换算；门控为0时关闭脉冲，不代表前轮已回到中位。
        pwm_set_duty(SERVO_MOTOR_PWM, output.servo_enabled ? output.servo_duty_cmd : 0u);
        // 记录的是寄存器命令与同次采样；真实引脚重载和机械到位仍需仪器/视频测量。
        calibration_debug_after_tick(now_us, &measurement, &output,
                                     (uint16_t)(output.servo_enabled ? output.servo_duty_cmd : 0u));

        // 4. 通信这里只累加逻辑毫秒计数；主循环poll才执行有界SPI收发。
        wifi_assistant_tick(CONTROL_LOOP_PERIOD_MS);
        // 当前在末尾清事件位；PIT位不能累计多次到期，超长屏蔽/执行可能合并事件，需实测排查。
        pit_flag_clear(PIT_CH0);
    }

    if(pit_flag_get(PIT_CH2))
    {
        pit_flag_clear(PIT_CH2);
    }
    
    if(pit_flag_get(PIT_CH3))
    {
        pit_flag_clear(PIT_CH3);
    }

    __DSB();
}

void LPUART1_IRQHandler(void)
{
    if(kLPUART_RxDataRegFullFlag & LPUART_GetStatusFlags(LPUART1))
    {
        // 接收中断
    #if DEBUG_UART_USE_INTERRUPT                        // 如果开启 debug 串口中断
        debug_interrupr_handler();                      // 调用 debug 串口接收处理函数 数据会被 debug 环形缓冲区读取
    #endif                                              // 如果修改了 DEBUG_UART_INDEX 那这段代码需要放到对应的串口中断去
        
    }
        
    LPUART_ClearStatusFlags(LPUART1, kLPUART_RxOverrunFlag);    // 不允许删除
}

void LPUART2_IRQHandler(void)
{
    if(kLPUART_RxDataRegFullFlag & LPUART_GetStatusFlags(LPUART2))
    {
        // 接收中断
        
    }
        
    LPUART_ClearStatusFlags(LPUART2, kLPUART_RxOverrunFlag);    // 不允许删除
}

void LPUART3_IRQHandler(void)
{
    if(kLPUART_RxDataRegFullFlag & LPUART_GetStatusFlags(LPUART3))
    {
        // 接收中断
        
    }
        
    LPUART_ClearStatusFlags(LPUART3, kLPUART_RxOverrunFlag);    // 不允许删除
}

void LPUART4_IRQHandler(void)
{
    if(kLPUART_RxDataRegFullFlag & LPUART_GetStatusFlags(LPUART4))
    {
        // 接收中断 
        if(NULL != flexio_camera_uart_handler)
        {
            flexio_camera_uart_handler();
        }
    }
        
    LPUART_ClearStatusFlags(LPUART4, kLPUART_RxOverrunFlag);    // 不允许删除
}

void LPUART5_IRQHandler(void)
{
    if(kLPUART_RxDataRegFullFlag & LPUART_GetStatusFlags(LPUART5))
    {
        // 接收中断
        if(NULL != camera_uart_handler)
        {
            camera_uart_handler();
        }
    }
        
    LPUART_ClearStatusFlags(LPUART5, kLPUART_RxOverrunFlag);    // 不允许删除
}

void LPUART6_IRQHandler(void)
{
    if(kLPUART_RxDataRegFullFlag & LPUART_GetStatusFlags(LPUART6))
    {
        // 接收中断
        
    }
        
    LPUART_ClearStatusFlags(LPUART6, kLPUART_RxOverrunFlag);    // 不允许删除
}


void LPUART8_IRQHandler(void)
{
    if(kLPUART_RxDataRegFullFlag & LPUART_GetStatusFlags(LPUART8))
    {
        // 接收中断
        if(NULL != wireless_module_uart_handler)
        {
            wireless_module_uart_handler();
        }
        
    }
        
    LPUART_ClearStatusFlags(LPUART8, kLPUART_RxOverrunFlag);    // 不允许删除
}


void GPIO1_Combined_0_15_IRQHandler(void)
{
    if(exti_flag_get(B0))
    {
        exti_flag_clear(B0);// 清除中断标志位
    }
    
}


void GPIO1_Combined_16_31_IRQHandler(void)
{
    // WIFI6B21-SPI V2.0 通过轮询 B17 就绪电平通信，不注册 GPIO 中断回调
    wireless_module_spi_handler();
    if(exti_flag_get(B17))
    {
        exti_flag_clear(B17); // 清除可能残留的中断标志位
    }
    if(exti_flag_get(B16))
    {
        exti_flag_clear(B16); // 清除中断标志位
    }

    
}

void GPIO2_Combined_0_15_IRQHandler(void)
{
    // 勿删除此IF语句
    if(NULL != flexio_camera_vsync_handler)
    {
        flexio_camera_vsync_handler();
    }
    
    
    if(exti_flag_get(C0))
    {
        exti_flag_clear(C0);// 清除中断标志位
    }

}


void GPIO2_Combined_16_31_IRQHandler(void)
{
    // -----------------* ToF INT 更新中断 预置中断处理函数 *-----------------
    //tof_module_exti_handler();
    // -----------------* ToF INT 更新中断 预置中断处理函数 *-----------------
    
    if(exti_flag_get(C16))
    {
        exti_flag_clear(C16); // 清除中断标志位
    }
    
    
}




void GPIO3_Combined_0_15_IRQHandler(void)
{

    if(exti_flag_get(D4))
    {
        exti_flag_clear(D4);// 清除中断标志位
    }
}









/*
中断函数名称，用于设置对应功能的中断函数
Sample usage:当前启用了周期定时器中断
void PIT_IRQHandler(void)
{
    //务必清除标志位
    __DSB();
}
记得进入中断后清除标志位
CTI0_ERROR_IRQHandler
CTI1_ERROR_IRQHandler
CORE_IRQHandler
FLEXRAM_IRQHandler
KPP_IRQHandler
TSC_DIG_IRQHandler
GPR_IRQ_IRQHandler
LCDIF_IRQHandler
CSI_IRQHandler
PXP_IRQHandler
WDOG2_IRQHandler
SNVS_HP_WRAPPER_IRQHandler
SNVS_HP_WRAPPER_TZ_IRQHandler
SNVS_LP_WRAPPER_IRQHandler
CSU_IRQHandler
DCP_IRQHandler
DCP_VMI_IRQHandler
Reserved68_IRQHandler
TRNG_IRQHandler
SJC_IRQHandler
BEE_IRQHandler
PMU_EVENT_IRQHandler
Reserved78_IRQHandler
TEMP_LOW_HIGH_IRQHandler
TEMP_PANIC_IRQHandler
USB_PHY1_IRQHandler
USB_PHY2_IRQHandler
ADC1_IRQHandler
ADC2_IRQHandler
DCDC_IRQHandler
Reserved86_IRQHandler
Reserved87_IRQHandler
GPIO1_INT0_IRQHandler
GPIO1_INT1_IRQHandler
GPIO1_INT2_IRQHandler
GPIO1_INT3_IRQHandler
GPIO1_INT4_IRQHandler
GPIO1_INT5_IRQHandler
GPIO1_INT6_IRQHandler
GPIO1_INT7_IRQHandler
GPIO1_Combined_0_15_IRQHandler
GPIO1_Combined_16_31_IRQHandler
GPIO2_Combined_0_15_IRQHandler
GPIO2_Combined_16_31_IRQHandler
GPIO3_Combined_0_15_IRQHandler
GPIO3_Combined_16_31_IRQHandler
GPIO4_Combined_0_15_IRQHandler
GPIO4_Combined_16_31_IRQHandler
GPIO5_Combined_0_15_IRQHandler
GPIO5_Combined_16_31_IRQHandler
WDOG1_IRQHandler
RTWDOG_IRQHandler
EWM_IRQHandler
CCM_1_IRQHandler
CCM_2_IRQHandler
GPC_IRQHandler
SRC_IRQHandler
Reserved115_IRQHandler
GPT1_IRQHandler
GPT2_IRQHandler
PWM1_0_IRQHandler
PWM1_1_IRQHandler
PWM1_2_IRQHandler
PWM1_3_IRQHandler
PWM1_FAULT_IRQHandler
SEMC_IRQHandler
USB_OTG2_IRQHandler
USB_OTG1_IRQHandler
XBAR1_IRQ_0_1_IRQHandler
XBAR1_IRQ_2_3_IRQHandler
ADC_ETC_IRQ0_IRQHandler
ADC_ETC_IRQ1_IRQHandler
ADC_ETC_IRQ2_IRQHandler
ADC_ETC_ERROR_IRQ_IRQHandler
PIT_IRQHandler
ACMP1_IRQHandler
ACMP2_IRQHandler
ACMP3_IRQHandler
ACMP4_IRQHandler
Reserved143_IRQHandler
Reserved144_IRQHandler
ENC1_IRQHandler
ENC2_IRQHandler
ENC3_IRQHandler
ENC4_IRQHandler
TMR1_IRQHandler
TMR2_IRQHandler
TMR3_IRQHandler
TMR4_IRQHandler
PWM2_0_IRQHandler
PWM2_1_IRQHandler
PWM2_2_IRQHandler
PWM2_3_IRQHandler
PWM2_FAULT_IRQHandler
PWM3_0_IRQHandler
PWM3_1_IRQHandler
PWM3_2_IRQHandler
PWM3_3_IRQHandler
PWM3_FAULT_IRQHandler
PWM4_0_IRQHandler
PWM4_1_IRQHandler
PWM4_2_IRQHandler
PWM4_3_IRQHandler
PWM4_FAULT_IRQHandler
Reserved171_IRQHandler
GPIO6_7_8_9_IRQHandler*/



