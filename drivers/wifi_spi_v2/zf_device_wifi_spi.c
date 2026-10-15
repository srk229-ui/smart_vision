/*********************************************************************************************************************
* RT1064DVL6A Opensource Library 即（RT1064DVL6A 开源库）是一个基于官方 SDK 接口的第三方开源库
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
* 文件名称          zf_device_wifi_spi
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
/*********************************************************************************************************************
* 接线定义：
*                   ------------------------------------
*                   模块管脚            单片机管脚
*                   RST                 查看 zf_device_wifi_spi.h 中 WIFI_SPI_RST_PIN 宏定义
*                   INT                 查看 zf_device_wifi_spi.h 中 WIFI_SPI_INT_PIN 宏定义
*                   CS                  查看 zf_device_wifi_spi.h 中 WIFI_SPI_CS_PIN 宏定义
*                   MISO                查看 zf_device_wifi_spi.h 中 WIFI_SPI_MISO_PIN 宏定义
*                   SCK                 查看 zf_device_wifi_spi.h 中 WIFI_SPI_SCK_PIN 宏定义
*                   MOSI                查看 zf_device_wifi_spi.h 中 WIFI_SPI_MOSI_PIN 宏定义
*                   5V                  5V 电源
*                   GND                 电源地
*                   其余引脚悬空
*                   ------------------------------------
*********************************************************************************************************************/

#include "stdio.h"
#include "zf_common_clock.h"
#include "zf_common_debug.h"
#include "zf_common_fifo.h"
#include "zf_driver_delay.h"
#include "zf_driver_gpio.h"
#include "zf_driver_spi.h"
#include "zf_device_type.h"

#include "zf_device_wifi_spi.h"
#include "fsl_lpspi.h"

#define WIFI_CONNECT_TIME_OUT       10000       // 单位毫秒
#define SOCKET_CONNECT_TIME_OUT     50000       // 单位毫秒
#define OTHER_TIME_OUT              1000        // 单位毫秒

#if ((WIFI_SPI_RECVIVE_SIZE < 32) || (WIFI_SPI_RECVIVE_SIZE > 4088))
    #error "WIFI_SPI_RECVIVE_SIZE must be >= 32 or <= 4088"
#endif

#if (WIFI_SPI_RECVIVE_SIZE >= WIFI_SPI_RECVIVE_FIFO_SIZE)
    #error "WIFI_SPI_RECVIVE_FIFO_SIZE must be > WIFI_SPI_RECVIVE_SIZE"
#endif


#if (WIFI_SPI_TRANSFER_SIZE != 4088)
    #error "WIFI_SPI_TRANSFER_SIZE must be == 4088"
#endif



char wifi_spi_version[12];                      // 保存模块固件版本信息
char wifi_spi_mac_addr[20];                     // 保存模块MAC地址信息
char wifi_spi_ip_addr_port[25];                 // 保存模块IP地址与端口信息

static fifo_struct  wifi_spi_fifo; // 驱动层接收FIFO，保留上层本次容量装不下的字节。
static uint8        wifi_spi_buffer[WIFI_SPI_RECVIVE_FIFO_SIZE]; // 驱动FIFO底层存储，容量由头文件宏定义。
static volatile     wifi_spi_state_enum wifi_spi_mutex; // 重入忙标记；本工程主循环收发，非通用原子互斥锁。
/* 本地驱动使用带截止时间的 SPI 交换，不调用共享库的无限等待接口。 */
static LPSPI_Type *const wifi_spi_bases[] = LPSPI_BASE_PTRS; // SDK基址表，用工程SPI序号选取LPSPI寄存器。
static uint8 service_cancelled; // 本轮一旦取消即锁存，按键事件被消费后也不恢复旧半包；新服务轮次才清除。
static uint8 (*wifi_spi_cancel_hook)(void); // 主循环安装的轻量按键查询；钩子只读事件，不控制车辆或收发网络。
static uint8 runtime_service_enabled; // 本轮身份锁存，不能因轮中控制停车而解锁长等待。
static uint32 runtime_service_start; // 本轮开始时的CYCCNT，单位核心周期。
volatile uint32 wifi_spi_io_error_count; // 底层超时/硬件错误/非法返回长度累计次数。
volatile uint32 wifi_spi_runtime_last_us; // 最近一次运行轮次耗时，整数us，含中断抢占。
volatile uint32 wifi_spi_runtime_max_us; // 运行轮次耗时峰值，整数us；不是实机保证值。
// 失败现场只记录寄存器和协议头，不保存热点密码或通信载荷；供DAP只读排查。
volatile uint32 wifi_spi_failure_trace[8]; // 顺序：累计次数、SR、FSR、TCR、CR、失败阶段、字节下标、本段长度。
volatile uint32 wifi_spi_parameter_trace[4][3]; // 行：版本/MAC/IP/入网；列：阶段及失败位、回复命令、回复长度。

// 将当前核心时钟Hz换算为每微秒DWT计数；截止检查以当前system_clock为基准。
static uint32 wifi_spi_cycles_per_us(void)
{
    return system_clock / 1000000u; // system_clock单位Hz，返回核心周期/us。
}

// 同一次受检交换使用固定start；无符号差值允许短区间跨CYCCNT回绕。
static uint8 wifi_spi_deadline_expired(uint32 start, uint32 budget_us)
{
    uint32 now = DWT->CYCCNT;
    // 先检查单次SPI交换预算：运行1ms、停止10ms，单位都从us换算为核心周期。
    if((uint32)(now - start) >= budget_us * wifi_spi_cycles_per_us())
    {
        return 1;
    }
    // 再检查本轮运行RX/TX共用2ms截止；后续包/命令/数据段不能重新获得完整预算。
    // 这是一轮服务的设计截止检查，包含PIT等中断抢占，物理最坏耗时仍待实测。
    return runtime_service_enabled &&
           (uint32)(now - runtime_service_start) >= WIFI_SPI_RUNTIME_BUDGET_US * wifi_spi_cycles_per_us();
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     开始主循环的一轮通信预算
// 参数说明     running         1-行车两毫秒总预算，0-停止态恢复方式
// 返回参数     void
// 备注信息     截止时间包含控制中断抢占；调用后必须配对结束，不能在收发内部重置
//-------------------------------------------------------------------------------------------------------------------
// 调用者固定一轮身份；传1后即使控制层中途停车，该轮仍按运行方式退出。
void wifi_spi_service_begin(uint8 running)
{
    service_cancelled = 0; // 显式开始新轮次才允许重新联网，service_end保留取消结果供上层查询。
    runtime_service_enabled = running != 0; // 调用者指定身份，非每包重新读取控制状态。
    runtime_service_start = DWT->CYCCNT; // RX/TX整轮共享起点，只在轮次边界建立。
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     查询本轮是否仍使用行车通信预算
// 参数说明     void            无
// 返回参数     uint8           1-行车预算，0-停止态
// 备注信息     控制中断刚停车时，本轮仍不得切入延时重试
//-------------------------------------------------------------------------------------------------------------------
uint8 wifi_spi_service_is_running(void)
{
    return runtime_service_enabled;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     结束通信轮次并保留行车耗时峰值
// 参数说明     void            无
// 返回参数     void
// 备注信息     峰值包含中断抢占，仅为软件观测，仍需实机检查完整控制时序
//-------------------------------------------------------------------------------------------------------------------
// 仅运行轮次更新耗时统计；计算DWT差值再除每微秒周期数，结果为整数us。
void wifi_spi_service_end(void)
{
    if(runtime_service_enabled)
    {
        wifi_spi_runtime_last_us = (uint32)(DWT->CYCCNT - runtime_service_start) / wifi_spi_cycles_per_us(); // 短区间无符号差值处理32位计数回绕。
        if(wifi_spi_runtime_last_us > wifi_spi_runtime_max_us)
        {
            wifi_spi_runtime_max_us = wifi_spi_runtime_last_us; // 只提高峰值，不在停止轮次重写运行测量。
        }
    }
    runtime_service_enabled = 0; // 结束后下一轮才可以使用停止恢复规则。
}

// NXP ERRATA051472：接收后关开MEN可能误置REF；旧SDK 2.2.1未包含后续驱动的处理。
// 仅在重新使能且尚未提交新TCR/数据时清REF；交换期间真实TEF/REF仍按原失败路径处理。
static void wifi_spi_enable_clean(LPSPI_Type *base, uint8 enable)
{
    LPSPI_Enable(base, enable);
    if(enable)
    {
        LPSPI_ClearStatusFlags(base, kLPSPI_ReceiveErrorFlag);
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     安装主循环联网等待取消查询
// 参数说明     hook            返回非零表示已有待处理按键；NULL关闭取消查询
// 返回参数     void
// 备注信息     钩子只查询ISR锁存事件，不执行控制、网络或延时；不在受检SPI交换内部调用。
//-------------------------------------------------------------------------------------------------------------------
void wifi_spi_set_cancel_hook(uint8 (*hook)(void))
{
    wifi_spi_cancel_hook = hook;
    service_cancelled = 0; // 安装或关闭钩子时同时重置历史服务结果。
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     查询是否应退出当前联网服务
// 参数说明     void            无
// 返回参数     uint8           非零表示有待处理按键
// 备注信息     仅查询，不消费按键；供上层把发送重试延时也拆为可取消小段。
//-------------------------------------------------------------------------------------------------------------------
uint8 wifi_spi_cancel_requested(void)
{
    if(!service_cancelled && wifi_spi_cancel_hook != NULL && wifi_spi_cancel_hook())
    {
        service_cancelled = 1; // 锁存瞬态事件，避免KEY2已被ISR消费后旧读取又被误认为成功。
    }
    return service_cancelled;
}

// 仅在CS包边界或等待/复位阶段退出：释放引脚和忙标记，清除旧连接半包，不计为SPI硬件故障。
static uint8 wifi_spi_cancel_pending(void)
{
    LPSPI_Type *base;
    if(!wifi_spi_cancel_requested()) { return 0; }
    gpio_high(WIFI_SPI_CS_PIN);
    gpio_high(WIFI_SPI_RST_PIN); // 复位低电平等待被打断时也必须恢复释放，后续停车重试重新初始化。
    base = wifi_spi_bases[WIFI_SPI_INDEX];
    wifi_spi_enable_clean(base, false);
    LPSPI_FlushFifo(base, true, true);
    LPSPI_ClearStatusFlags(base, kLPSPI_AllStatusFlag);
    wifi_spi_enable_clean(base, true);
    fifo_init(&wifi_spi_fifo, FIFO_DATA_8BIT, wifi_spi_buffer, WIFI_SPI_RECVIVE_FIFO_SIZE);
    wifi_spi_mutex = WIFI_SPI_IDLE;
    return 1;
}

// 上电复位和扫描延时每1ms检查按键，保持原总延时；取消后由主循环先处理按键再决定重试。
static uint8 wifi_spi_delay_cancelable(uint32 delay_ms)
{
    while(delay_ms != 0)
    {
        if(wifi_spi_cancel_pending()) { return 1; }
        system_delay_ms(1);
        --delay_ms;
    }
    return wifi_spi_cancel_pending();
}

// 1. 交换异常统一回收硬件FIFO/状态，并用错误计数通知上层。
/* 保留时钟和模式配置，只终止当前交换并清理 FIFO，允许停车后重新初始化。 */
static uint8 wifi_spi_abort_exchange(LPSPI_Type *base, uint32 stage, uint32 index, uint32 length)
{
    // 必须在清FIFO/标志前留存现场，阶段1/2/3/4/5分别表示前忙/TX满/写前/RX空/尾忙。
    wifi_spi_failure_trace[1] = LPSPI_GetStatusFlags(base);
    wifi_spi_failure_trace[2] = base->FSR;
    wifi_spi_failure_trace[3] = base->TCR;
    wifi_spi_failure_trace[4] = base->CR;
    wifi_spi_failure_trace[5] = stage;
    wifi_spi_failure_trace[6] = index;
    wifi_spi_failure_trace[7] = length;
    ++wifi_spi_failure_trace[0]; // 最后更新次数，便于调试器判断快照是否改变。
    wifi_spi_enable_clean(base, false); // 暂停当前SPI事务，不重新设置波特率或模式。
    LPSPI_FlushFifo(base, true, true); // TX/RX都清，避免超时半包留在硬件FIFO。
    LPSPI_ClearStatusFlags(base, kLPSPI_AllStatusFlag); // 清本次残留错误/完成标记。
    wifi_spi_enable_clean(base, true); // 允许后续停车恢复重新使用接口。
    ++wifi_spi_io_error_count; // 上层poll比对该计数后丢弃本轮参数并报告故障。
    return 1;
}

/* 外部片选由上层保持；每个八位字节等待均检查同一个截止时间。 */
// 函数简介     逐字节有截止检查的8位SPI交换，绕过共享库无限忙等待。
// 参数说明     write_data为空时发0；read_data为空时读出后丢弃；length为字节数。
// 返回参数     0成功，1超时或硬件异常；外部片选由调用层统一释放。
static uint8 wifi_spi_exchange_checked(const uint8 *write_data, uint8 *read_data, uint16 length)
{
    // 1. 选择工程配置的SPI基址，固定本次交换起点与本次等待预算。
    LPSPI_Type *base = wifi_spi_bases[WIFI_SPI_INDEX];
    uint32 start = DWT->CYCCNT;
    uint32 budget_us = runtime_service_enabled ? WIFI_SPI_RUNTIME_TRANSFER_US : WIFI_SPI_STOP_TRANSFER_US; // 运行单交换1000us，停止单交换10000us。
    uint16 index;
    // 新交换前先等上一硬件事务Busy结束；每次等待都检查局部和整轮截止。
    while(LPSPI_GetStatusFlags(base) & kLPSPI_ModuleBusyFlag)
    {
        if(wifi_spi_deadline_expired(start, budget_us)) { return wifi_spi_abort_exchange(base, 1, 0, length); }
    }
    // Busy已经空闲也可能耗完预算，配置寄存器前再次拒绝逾期交换。
    if(wifi_spi_deadline_expired(start, budget_us)) { return wifi_spi_abort_exchange(base, 1, 0, length); }
    // 2. 暂停模块并清旧FIFO/错误，防止上次残留数据被当作本次应答。
    wifi_spi_enable_clean(base, false);
    LPSPI_FlushFifo(base, true, true);
    LPSPI_ClearStatusFlags(base, kLPSPI_AllStatusFlag);
    // NOSTALL清零，FIFO满/空时允许硬件停顿，不在软件未准备好时继续移位。
    base->CFGR1 &= ~LPSPI_CFGR1_NOSTALL_MASK;
    wifi_spi_enable_clean(base, true); // 先启用外设，再把TCR命令提交到TX FIFO。
    // 仅清连续传输、屏蔽、硬件片选和帧长位，再设FRAMESZ=7表示8位。
    // 保留CPOL/CPHA、PRESCALE等TCR其余位；不写CCR，沿用SPI_1/20MHz/MODE3初始化。
    base->TCR = (base->TCR & ~(LPSPI_TCR_CONT_MASK | LPSPI_TCR_CONTC_MASK |
                  LPSPI_TCR_RXMSK_MASK | LPSPI_TCR_TXMSK_MASK | LPSPI_TCR_PCS_MASK |
                  LPSPI_TCR_FRAMESZ_MASK)) | LPSPI_TCR_FRAMESZ(7);
    // 3. 每个字节共用本次start，不能在TX/RX等待内部把超时时间重新归零。
    for(index = 0; index < length; ++index)
    {
        uint8 value;
        // TX FIFO未留槽位就等待；等待期间若任一截止到期，回收并立即返回。
        while(LPSPI_GetTxFifoCount(base) >= LPSPI_GetTxFifoSize(base))
        {
            if(wifi_spi_deadline_expired(start, budget_us)) { return wifi_spi_abort_exchange(base, 2, index, length); }
        }
        // 发送/接收错误任一置位，或时间预算耗尽，都走同一异常回收路径。
        if((LPSPI_GetStatusFlags(base) & (kLPSPI_TransmitErrorFlag | kLPSPI_ReceiveErrorFlag)) ||
           wifi_spi_deadline_expired(start, budget_us)) { return wifi_spi_abort_exchange(base, 3, index, length); }
        // 没有写缓冲时用0产生读取时钟；每次提交一个8位帧。
        LPSPI_WriteData(base, write_data == NULL ? 0 : write_data[index]);
        // 等RX FIFO出现本字节；同时检测硬件错误与共用截止，不只等接收标志。
        while(LPSPI_GetRxFifoCount(base) == 0)
        {
            if((LPSPI_GetStatusFlags(base) & (kLPSPI_TransmitErrorFlag | kLPSPI_ReceiveErrorFlag)) ||
               wifi_spi_deadline_expired(start, budget_us)) { return wifi_spi_abort_exchange(base, 4, index, length); }
        }
        // 即使上层不接收，也必须取走RX数据，避免下一字节混入旧应答。
        value = (uint8)LPSPI_ReadData(base);
        if(read_data != NULL) { read_data[index] = value; }
    }
    // 4. 最后一字节已从RX取出，仍需确认模块Busy清除后才允许上层结束片选。
    while(LPSPI_GetStatusFlags(base) & kLPSPI_ModuleBusyFlag)
    {
        if(wifi_spi_deadline_expired(start, budget_us)) { return wifi_spi_abort_exchange(base, 5, length, length); }
    }
    return 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     等待WIFI SPI就绪
// 参数说明     wait_time       最大等待时间 单位毫秒
// 返回参数     uint8           状态 0-成功 1-错误
// 使用示例     内部使用，用户无需关心
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
// 运行与停止等待策略不同；这里的wait_time只用于停止恢复路径。
static uint8 wifi_spi_wait_idle (uint32 wait_time)
{
    uint32 time = 0;
    if(wifi_spi_cancel_pending()) { return 1; }
    
    if(runtime_service_enabled)
    {
        /* 行车模块未就绪立即让出主循环，不做毫秒级等待。 */
        // 运行要求B17已就绪且本轮还有预算；任一不满足就立即让出主循环，不做10us睡眠。
        return !gpio_get_level(WIFI_SPI_INT_PIN) ||
               wifi_spi_deadline_expired(runtime_service_start, WIFI_SPI_RUNTIME_BUDGET_US);
    }
    // 停止态将ms乘100换为10us轮询次数，保留原恢复等待方式。
    wait_time = wait_time*100;
    while(0 == gpio_get_level(WIFI_SPI_INT_PIN))
    {
        if(wifi_spi_cancel_pending()) { return 1; } // 每10us轮询检查，不必等最长50s建链超时。
        system_delay_us(10);
        time++;
        if(wait_time <= time)
        {
            break;
        }
    }
    if(wifi_spi_cancel_pending()) { return 1; }
    return (wait_time <= time);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     写入数据到WIFI SPI
// 参数说明     *buffer1        第一组需要发送的数据缓冲区地址
// 参数说明     length1         第一组数据长度
// 参数说明     *buffer2        第二组需要发送的数据缓冲区地址
// 参数说明     length2         第二组数据长度
// 返回参数     uint8           0-成功，1-超时或非法返回长度
// 使用示例     内部使用，用户无需关心
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
// 一份外部片选覆盖头段/数据段；前段失败即跳过后段，但必定拉高片选。
static uint8 wifi_spi_write (const uint8 *buffer1, uint16 length1, const uint8 *buffer2, uint16 length2)
{
    uint8 failed = 0;
    if(wifi_spi_cancel_pending()) { return 1; }
    gpio_low(WIFI_SPI_CS_PIN); // 软件片选开始，两个数据段属于同一次模块事务。
    if(NULL != buffer1) { failed = wifi_spi_exchange_checked(buffer1, NULL, length1); }
    if(!failed && NULL != buffer2) { failed = wifi_spi_exchange_checked(buffer2, NULL, length2); }
    gpio_high(WIFI_SPI_CS_PIN); // 即使任一受检交换失败，也释放片选。
    if(wifi_spi_cancel_pending()) { return 1; } // 完成当前有界CS包后取消，不发布本包接收内容。
    return failed;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     WIFI SPI 发送与接收同时进行（命令收发）
// 参数说明     *packets        发送与接收的地址
// 参数说明     length          需要接收的长度
// 返回参数     uint8           0-成功，1-超时或非法返回长度
// 使用示例     内部使用，用户无需关心
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
// 头部与载荷都原地全双工交换；本次每个交换仍受同一运行轮次截止约束。
static uint8 wifi_spi_transfer_command (wifi_spi_packets_struct *packets, uint16 length)
{
    uint8 failed;
    if(wifi_spi_cancel_pending())
    {
        packets->head.command = WIFI_SPI_INVALID1;
        packets->head.length = 0;
        return 1;
    }
    gpio_low(WIFI_SPI_CS_PIN);
    failed = wifi_spi_exchange_checked((const uint8 *)&packets->head, (uint8 *)&packets->head, sizeof(wifi_spi_head_struct));
    // 头部成功且有载荷长度时才交换载荷，避免异常头部后继续产生时钟。
    if(!failed && length) { failed = wifi_spi_exchange_checked(packets->buffer, packets->buffer, length); }
    gpio_high(WIFI_SPI_CS_PIN); // 包头/载荷结束后统一释放片选。
    if(wifi_spi_cancel_pending()) { failed = 1; } // 完成当前有界CS包后取消，不发布本包接收内容。
    // length为0时仅交换4字节应答头；设置命令使用独立head对象，不读取载荷。
    // 头内声明的应答长度不代表本次写入该对象的字节数；有载荷窗口时才校验容量。
    if(!failed && length != 0 && packets->head.length > length)
    {
        ++wifi_spi_io_error_count; // 长度异常也通过统一错误计数向上层传播。
        failed = 1;
    }
    // 失败后把包头置无效且长度清零，消费者不能使用上一包残留的命令/长度。
    if(failed)
    {
        packets->head.command = WIFI_SPI_INVALID1;
        packets->head.length = 0; // 异常包没有可交付的有效载荷。
    }
    return failed;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     WIFI SPI 发送与接收同时进行(数据收发)
// 参数说明     *write_data     发送的数据缓冲区地址
// 参数说明     *read_data      接收到的数据的存储地址
// 参数说明     length          需要接收的长度
// 返回参数     uint8           0-成功，1-超时或非法返回长度
// 使用示例     内部使用，用户无需关心
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
// 1. 在同一片选内先交换模块头，再交换数据，头部指定本次发送长度。
static uint8 wifi_spi_transfer_data (const uint8 *write_data, wifi_spi_packets_struct *read_data, uint16 length)
{
    uint8 failed;
    if(wifi_spi_cancel_pending())
    {
        read_data->head.command = WIFI_SPI_INVALID1;
        read_data->head.length = 0;
        return 1;
    }
    gpio_low(WIFI_SPI_CS_PIN);
    read_data->head.command = WIFI_SPI_DATA; // 请求透传数据命令，头和载荷共用片选。
    read_data->head.length = length; // 本次发送有效字节数，不是本地接收容量。
    failed = wifi_spi_exchange_checked((const uint8 *)&read_data->head, (uint8 *)&read_data->head, sizeof(wifi_spi_head_struct));
    // 2. 长发送包先全双工交换接收容量32字节，其余发送数据只发不保存RX载荷。
    if(!failed && WIFI_SPI_RECVIVE_SIZE < length)
    {
        failed = wifi_spi_exchange_checked(write_data, read_data->buffer, WIFI_SPI_RECVIVE_SIZE); // 最多保存本地载荷容量内的接收数据。
        if(!failed)
        {
            failed = wifi_spi_exchange_checked(write_data + WIFI_SPI_RECVIVE_SIZE, NULL, length - WIFI_SPI_RECVIVE_SIZE); // 发送剩余尾段，RX仍由底层读走但不保存。
        }
    }
    // 3. 短包仍按模块32字节交换窗口运行；先补零再拷有效字节，避免越界读。
    else if(!failed)
    {
        /* 补零到模块要求的收发长度，避免短包越界读和发送未初始化字节。 */
        memset(read_data->buffer, 0, WIFI_SPI_RECVIVE_SIZE); // 每次重建补零区，不能发送未初始化尾字节。
        if(length) { memcpy(read_data->buffer, write_data, length); }
        failed = wifi_spi_exchange_checked(read_data->buffer, read_data->buffer, WIFI_SPI_RECVIVE_SIZE); // 发同一缓冲区，逐字节读回覆盖已发位置。
    }
    gpio_high(WIFI_SPI_CS_PIN); // 无论结果如何都结束本次片选。
    if(wifi_spi_cancel_pending()) { failed = 1; } // 完成当前有界CS包后取消，不发布本包接收内容。
    // 模块返回的接收长度必须不超过本地32字节载荷；超范围统一计错/清包头。
    if(!failed && read_data->head.length > WIFI_SPI_RECVIVE_SIZE)
    {
        ++wifi_spi_io_error_count;
        failed = 1;
    }
    if(failed)
    {
        read_data->head.command = WIFI_SPI_INVALID1;
        read_data->head.length = 0; // 异常接收包不暴露旧载荷长度。
    }
    return failed;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     WIFI SPI 参数设置
// 参数说明     command         命令类型
// 参数说明     *buffer         参数地址
// 参数说明     length          参数长度
// 参数说明     wait_time       停止态最大等待时间，单位ms；运行路径不调用此恢复流程
// 返回参数     uint8           状态 0-成功 1-错误
// 使用示例     内部使用，用户无需关心
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
static uint8 wifi_spi_set_parameter (wifi_spi_packets_command_enum command, uint8 *buffer, uint16 length, uint32 wait_time)
{
    uint8 return_state;
    uint32 trace_row = 3; // 仅记录命令头，不读取载荷。
    uint32 trace_stage = 1; // 1等就绪、2写命令、3等应答、4读应答、5读完。
    wifi_spi_head_struct head = {0};
    return_state = 1;
    do
    {
        head.command = command;
        head.length  = length;
        
        // 等待从机准备就绪
        if(wifi_spi_wait_idle(wait_time))
        {
            break;
        }

        // 受检头/载荷写失败就终止本次设置命令，不继续等待一个无效应答。
        trace_stage = 2;
        if(wifi_spi_write(&head.command, sizeof(wifi_spi_head_struct), buffer, length)) { break; }
        trace_stage = 3;
        if(wifi_spi_wait_idle(wait_time))
        {
            break;
        }
        // 接收应答信号

        head.command = WIFI_SPI_DATA;
        head.length = 0;
        // 读应答失败同样退出；本demo运行路径不调用此类带恢复延时的设置流程。
        trace_stage = 4;
        if(wifi_spi_transfer_command((wifi_spi_packets_struct *)&head, head.length)) { break; }
        trace_stage = 5;
        system_delay_us(20);
        if(wifi_spi_cancel_pending()) { break; }
        if(WIFI_SPI_REPLY_OK == head.command)
        {
            return_state = 0;
        }
    }while(0);
    
    if(command == WIFI_SPI_SET_WIFI_INFORMATION)
    {
        wifi_spi_parameter_trace[trace_row][1] = head.command;
        wifi_spi_parameter_trace[trace_row][2] = head.length;
        wifi_spi_parameter_trace[trace_row][0] = trace_stage | ((uint32)return_state << 8); // 位8为失败标记。
    }
    return return_state;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     WIFI SPI 模块信息获取
// 参数说明     command         命令类型
// 参数说明     *buffer         保存接收到的参数地址
// 参数说明     wait_time       停止态最大等待时间，单位ms；运行路径不调用此恢复流程
// 返回参数     uint8           状态 0-成功 1-错误
// 使用示例     内部使用，用户无需关心
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
static uint8 wifi_spi_get_parameter (wifi_spi_packets_command_enum command, wifi_spi_packets_struct *read_data, uint32 wait_time)
{
    uint8 return_state;
    uint32 trace_row = command == WIFI_SPI_GET_VERSION ? 0 : (command == WIFI_SPI_GET_MAC_ADDR ? 1 : (command == WIFI_SPI_GET_IP_ADDR ? 2 : 3)); // 仅记录命令头，不读取载荷。
    uint32 trace_stage = 1; // 1等就绪、2写命令、3等应答、4读应答、5读完。

    return_state = 1;
    do
    {
        // 等待从机准备就绪
        if(wifi_spi_wait_idle(wait_time))
        {
            break;
        }
        read_data->head.command = command;
        // 版本等查询写命令失败就退出，避免把旧缓存当作新查询返回。
        trace_stage = 2;
        if(wifi_spi_write(&(read_data->head.command), WIFI_SPI_RECVIVE_SIZE, NULL, 0)) { break; }

        trace_stage = 3;
        if(wifi_spi_wait_idle(wait_time))
        {
            break;
        }
        read_data->head.command = WIFI_SPI_DATA;
        read_data->head.length = 0;
        // 读取应答前已经清请求长度；受检交换失败则保留错误返回。
        trace_stage = 4;
        if(wifi_spi_transfer_command(read_data, WIFI_SPI_RECVIVE_SIZE)) { break; }
        trace_stage = 5;
        return_state = 0;
    }while(0);
    if(trace_row < 3)
    {
        wifi_spi_parameter_trace[trace_row][1] = read_data->head.command;
        wifi_spi_parameter_trace[trace_row][2] = read_data->head.length;
        wifi_spi_parameter_trace[trace_row][0] = trace_stage | ((uint32)return_state << 8); // 位8为失败标记。
    }
    return return_state;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     WIFI SPI 固件版本获取
// 参数说明     void            端口号
// 返回参数     uint8           状态 0-成功 1-错误
// 使用示例     
// 备注信息     调用函数之后，固件版本信息以字符串形式保存在wifi_spi_version数组中
//-------------------------------------------------------------------------------------------------------------------
static uint8 wifi_spi_get_version (void)
{
    uint8 return_state;
    wifi_spi_packets_struct temp_packets = {0};

    return_state = wifi_spi_get_parameter(WIFI_SPI_GET_VERSION, &temp_packets, OTHER_TIME_OUT);
    // 成功状态、匹配版本应答、长度非0且小于目标数组，四项全部成立才复制。
    // 先整数组清零，为复制后的版本保留字符串结束符；重连不能复用旧版本。
    if((0 == return_state) && (WIFI_SPI_REPLY_VERSION == temp_packets.head.command) &&
       temp_packets.head.length > 0 && temp_packets.head.length < sizeof(wifi_spi_version))
    {
        memset(wifi_spi_version, 0, sizeof(wifi_spi_version));
        memcpy(wifi_spi_version, temp_packets.buffer, temp_packets.head.length);
    }
    else
    {
        return_state = 1;
    }
    return return_state;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     WIFI SPI MAC地址获取
// 参数说明     void            端口号
// 返回参数     uint8           状态 0-成功 1-错误
// 使用示例     
// 备注信息     调用函数之后，MAC地址信息以字符串形式保存在wifi_spi_mac_addr数组中
//-------------------------------------------------------------------------------------------------------------------
static uint8 wifi_spi_get_mac_addr (void)
{
    uint8 return_state;
    wifi_spi_packets_struct temp_packets = {0};

    return_state = wifi_spi_get_parameter(WIFI_SPI_GET_MAC_ADDR, &temp_packets, OTHER_TIME_OUT);
    // MAC应答必须类型匹配且长度留出结束符空间，才更新这次模块信息。
    if((0 == return_state) && (WIFI_SPI_REPLY_MAC_ADDR == temp_packets.head.command) &&
       temp_packets.head.length > 0 && temp_packets.head.length < sizeof(wifi_spi_mac_addr))
    {
        memset(wifi_spi_mac_addr, 0, sizeof(wifi_spi_mac_addr));
        memcpy(wifi_spi_mac_addr, temp_packets.buffer, temp_packets.head.length);
    }
    else
    {
        return_state = 1;
    }
    return return_state;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     WIFI SPI IP地址与端口号获取
// 参数说明     void            端口号
// 返回参数     uint8           状态 0-成功 1-错误
// 使用示例     
// 备注信息     调用函数之后，IP地址与端口号信息以字符串形式保存在wifi_spi_ip_addr_port数组中
//              需要在连接Socket之后调用此函数才能正常获取信息
//-------------------------------------------------------------------------------------------------------------------
static uint8 wifi_spi_get_ip_addr_port (void)
{
    uint8 return_state;
    wifi_spi_packets_struct temp_packets = {0};

    return_state = wifi_spi_get_parameter(WIFI_SPI_GET_IP_ADDR, &temp_packets, OTHER_TIME_OUT);
    // IP/端口应答必须类型匹配且非空、长度不越界；否则返回失败而不复制。
    if((0 == return_state) && (WIFI_SPI_REPLY_IP_ADDR == temp_packets.head.command) &&
       temp_packets.head.length > 0 && temp_packets.head.length < sizeof(wifi_spi_ip_addr_port))
    {
        memset(wifi_spi_ip_addr_port, 0, sizeof(wifi_spi_ip_addr_port));
        memcpy(wifi_spi_ip_addr_port, temp_packets.buffer, temp_packets.head.length);
    }
    else
    {
        return_state = 1;
    }
    return return_state;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     WIFI SPI 获取系统时间
// 参数说明     time_format     时间格式
// 返回参数     *buffer         保存时间字符串地址 缓冲区大小至少需要30个字节
// 返回参数     buffer_size     缓冲区大小 
// 返回参数     uint8           状态 0-成功 1-错误
// 使用示例     
// 备注信息     需要连接有网络的热点之后才能获取到正确的时间，并且在调用这个函数之前需要确保WIFI模块中需要接收的数据已经全部接收完毕
// 备注信息     仅WIFI SPI 外接天线版本支持
// 备注信息     输出的信息为字符串信息，可直接printf到串口助手查看
//-------------------------------------------------------------------------------------------------------------------
uint8 wifi_spi_get_time (wifi_spi_time_enum time_format, char *buffer, uint8 buffer_size)
{
    uint8 return_state = 1;
    wifi_spi_packets_struct temp_packets = {0};
    uint8 send_cmd, receive_cmd;
    
    // 接收时间的数组必须大于30个字节
    // 并且模块固件必须是V2版本
    if((30 <= buffer_size) && (!strncmp(wifi_spi_version, "V2", 2)))
    {
        send_cmd = WIFI_SPI_GET_TIME1 + time_format - WIFI_SPI_UTC_0;
        receive_cmd = WIFI_SPI_REPLY_TIME1 + time_format - WIFI_SPI_UTC_0;

        return_state = wifi_spi_get_parameter(send_cmd, &temp_packets, OTHER_TIME_OUT);
        if((0 == return_state) && (receive_cmd == temp_packets.head.command))
        {
            return_state = strncmp((const char *)temp_packets.buffer, "OK", 2);
            if(0 == return_state)
            {
                memcpy(buffer, &temp_packets.buffer[3], temp_packets.head.length - 3);
                buffer[temp_packets.head.length - 3] = 0;
            }
        }
    }
    return return_state;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     WIFI SPI 扫描热点
// 参数说明     *buffer         保存扫描到的热点信息 包含名称与信号强度，每个信号强度后面会跟一个换行符
// 参数说明     buffer_size     缓冲区的长度
// 返回参数     uint8           状态 0-成功 1-错误
// 使用示例     
// 备注信息     仅WIFI SPI 外接天线版本支持
// 备注信息     输出的信息为字符串信息，可直接printf到串口助手查看
// 备注信息     每一行包含一个wifi名称与密码 
//-------------------------------------------------------------------------------------------------------------------
uint8 wifi_spi_wifi_scan (char *buffer, uint16 buffer_size)
{
    uint8 return_state = 1;
    
    // 模块必须是V2版本的固件才支持此功能
    if(!strncmp(wifi_spi_version, "V2", 2))
    {
        return_state = wifi_spi_set_parameter(WIFI_SPI_SET_WIFI_SCAN, NULL, 0, WIFI_CONNECT_TIME_OUT);
        
        if(0 == return_state)
        {
        #if (0 == WIFI_SPI_READ_TRANSFER)
            wifi_spi_send_buffer(NULL, 0);
        #endif
            while(0 == wifi_spi_read_buffer((uint8 *)buffer, buffer_size))
            {
                if(wifi_spi_delay_cancelable(10)) { return_state = 1; break; }
            }
        }
    }
    
    return return_state;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     WIFI SPI 设置连接的WiFi信息并尝试连接WiFi
// 参数说明     *wifi_ssid      WIFI名称
// 参数说明     *pass_word      WIFI密码
// 返回参数     uint8           状态 0-成功 1-错误
// 使用示例     wifi_spi_wifi_connect("SEEKFREE", "SEEKFREE123");
// 备注信息     wifi_spi_wifi_connect("SEEKFREE", NULL); // 连接没有密码的WIFI热点
//-------------------------------------------------------------------------------------------------------------------
uint8 wifi_spi_wifi_connect (char *wifi_ssid, char *pass_word)
{
    uint8 return_state;
    uint8 temp_buffer[64];
    uint16 length;
    
    if(NULL != pass_word)
    {
        // WIFI热点有密码发送热点名称与密码
        length = sprintf((char *)temp_buffer, "%s\r\n%s\r\n", wifi_ssid, pass_word);
    }
    else
    {
        // WIFI热点没有密码只需要发送热点名称
        length = sprintf((char *)temp_buffer, "%s\r\n", wifi_ssid);
    }
    
    return_state = wifi_spi_set_parameter(WIFI_SPI_SET_WIFI_INFORMATION, temp_buffer, length, WIFI_CONNECT_TIME_OUT);
    
    // 本机IP地址与端口号信息以字符串形式保存在wifi_spi_ip_addr_port数组中
    if(!return_state && !wifi_spi_cancel_pending()) { wifi_spi_get_ip_addr_port(); }
    if(wifi_spi_cancel_pending()) { return_state = 1; }
    
    return return_state;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     WIFI SPI 设置连接的Socket信息并尝试连接Socket
// 参数说明     *transport_type 传输类型
// 参数说明     *ip_addr        IP地址
// 参数说明     *port           目标端口号
// 参数说明     *local_port     本机端口号
// 返回参数     uint8           状态 0-成功 1-错误
// 使用示例     wifi_spi_socket_connect("TCP", "192.168.2.5", "8080", "6060");
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
uint8 wifi_spi_socket_connect (char *transport_type, char *ip_addr, char *port, char *local_port)
{
    uint8 return_state;
    uint8 temp_buffer[41];
    uint16 length;
    
    length = sprintf((char *)temp_buffer, "%s\r\n%s\r\n%s\r\n%s\r\n", transport_type, ip_addr, port, local_port);
        
    return_state = wifi_spi_set_parameter(WIFI_SPI_SET_SOCKET_INFORMATION, temp_buffer, length, SOCKET_CONNECT_TIME_OUT);
    
    // 本机IP地址与端口号信息以字符串形式保存在wifi_spi_ip_addr_port数组中
    if(!return_state && !wifi_spi_cancel_pending()) { wifi_spi_get_ip_addr_port(); }
    if(wifi_spi_cancel_pending()) { return_state = 1; }
    
    return return_state;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     WIFI SPI 断开Socket连接
// 参数说明     void            
// 返回参数     uint8           状态 0-成功 1-错误
// 使用示例     wifi_spi_socket_disconnect();
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
uint8 wifi_spi_socket_disconnect (void)
{
    wifi_spi_packets_struct temp_packets = {0};

    return wifi_spi_get_parameter(WIFI_SPI_CLOSE_SOCKET, &temp_packets, OTHER_TIME_OUT);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     WIFI SPI 软复位
// 参数说明     void            
// 返回参数     uint8           状态 0-成功 1-错误
// 使用示例     
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
uint8 wifi_spi_reset (void)
{
    uint8 return_state;
    wifi_spi_head_struct head = {0};
    return_state = 1;
    do
    {
        head.command = WIFI_SPI_RESET;
        head.length  = 0xA5A5;
        return_state = wifi_spi_wait_idle(OTHER_TIME_OUT);
        if(return_state)
        {
            break;
        }
        return_state = wifi_spi_write(&head.command, sizeof(wifi_spi_head_struct), NULL, 0);
    }while(0);
    
    return return_state;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     WIFI SPI UDP模式时立即发送函数
// 参数说明     void
// 返回参数     uint8           状态 0-成功 1-错误
// 使用示例     
// 备注信息     在UDP模式下模块收到数据后会等待2毫秒，2毫秒后未收到数据则将数据通过socket发送到网络，如果希望立即发送则在数据传输完毕后调用此函数
//-------------------------------------------------------------------------------------------------------------------
uint8 wifi_spi_udp_send_now (void)
{
    uint8 return_state = 1;
    wifi_spi_packets_struct temp_packets = {0};
    
    if(WIFI_SPI_IDLE == wifi_spi_mutex)
    {
        // 将通讯状态设置为忙
        wifi_spi_mutex = WIFI_SPI_BUSY;
        do
        {
            if(wifi_spi_wait_idle(OTHER_TIME_OUT))
            {
                break;
            }

            // 立即开始socket发送
            temp_packets.head.command = WIFI_SPI_UDP_SEND;
            temp_packets.head.length = 0;
            if(wifi_spi_transfer_command(&temp_packets, WIFI_SPI_RECVIVE_SIZE)) { break; }
            
            // 检查收到的包中是否有数据
            if((WIFI_SPI_REPLY_DATA_START == temp_packets.head.command) || (WIFI_SPI_REPLY_DATA_END == temp_packets.head.command))
            {
                // 保存接收到的数据
                if(temp_packets.head.length)
                {
                    fifo_write_buffer(&wifi_spi_fifo, temp_packets.buffer, temp_packets.head.length);
                }
            }
            
            // 等待应答信号
            if(wifi_spi_wait_idle(OTHER_TIME_OUT))
            {
                break;
            }
            
            // 接收应答信号
            temp_packets.head.command = WIFI_SPI_DATA;
            temp_packets.head.length = 0;
            if(wifi_spi_transfer_command(&temp_packets, temp_packets.head.length)) { break; }
            
            if(WIFI_SPI_REPLY_OK == temp_packets.head.command)
            {
                return_state = 0;
            }
            
        }while(0);
        
        // 将通讯状态设置为空闲
        wifi_spi_mutex = WIFI_SPI_IDLE;
    } 
    
    return return_state;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     WIFI SPI 数据块发送函数并同步接收数据
// 参数说明     *buff           需要发送的数据地址
// 参数说明     length          发送长度
// 返回参数     uint32          剩余未发送的长度
// 使用示例     wifi_spi_send_buffer(buffer, 100);
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
// 1. 剩余长度作为返回值，只有本轮实际送入模块的字节才从它扣除。
uint32 wifi_spi_send_buffer (const uint8 *buffer, uint32 length)
{
    uint16 send_length; // 协议单包发送量，单位字节。
    wifi_spi_packets_struct temp_packets = {0}; // 等待超时退出时，包状态保持为无效值
    
    // 检查WIFI SPI状态，如果在其他中断或者线程中已经发起了通讯，则本次不能发送数据
    // 此volatile忙标记防止已有调用重入，不是原子RTOS锁；本demo只在主循环收发。
    if(wifi_spi_cancel_pending()) { return length; }
    if(WIFI_SPI_IDLE == wifi_spi_mutex)
    {
        // 将通讯状态设置为忙
        wifi_spi_mutex = WIFI_SPI_BUSY;
        
        while(length)
        {
            // 停车单包不超过协议4088字节；运行还要限制每个发送包为128字节。
            send_length = (uint16)(length > WIFI_SPI_TRANSFER_SIZE ? WIFI_SPI_TRANSFER_SIZE : length);
            if(runtime_service_enabled && send_length > WIFI_SPI_RUNTIME_TX_BYTES) // 运行小包上限仅限制单包，整轮仍按同一DWT截止。
            {
                send_length = WIFI_SPI_RUNTIME_TX_BYTES;
            }
            
            // 运行下模块未就绪或共用预算耗完就立即返回尾段；停车才使用恢复等待。
            if(wifi_spi_wait_idle(OTHER_TIME_OUT))
            {
                break;
            }
            
            // 任何受检SPI错误都保留本包为未发，不推进数据指针。
            if(wifi_spi_transfer_data(buffer, &temp_packets, send_length)) { break; }
            
            // 检查收到的包中是否有数据
            if((WIFI_SPI_REPLY_DATA_START == temp_packets.head.command) || (WIFI_SPI_REPLY_DATA_END == temp_packets.head.command))
            {
                // 保存接收到的数据
                if(temp_packets.head.length)
                {
                    fifo_write_buffer(&wifi_spi_fifo, temp_packets.buffer, temp_packets.head.length);
                }
            }
            
            // 本包全部交换成功后才扣掉发送字节数并移动到后续数据。
            length -= send_length;
            buffer += send_length; // 只跳过已经成功发出的前缀。
        }
        
        // 检查最后一次的接收是否将所有的数据都接收完毕
        // 2. 若应答说还有接收数据，继续取模块尾包；运行仍受原2ms共用截止限制。
        while(WIFI_SPI_REPLY_DATA_START == temp_packets.head.command)
        {
            if(wifi_spi_wait_idle(OTHER_TIME_OUT))
            {
                break;
            }
            
            // 继续读取模块剩余数据
            temp_packets.head.command = WIFI_SPI_DATA;
            temp_packets.head.length  = 0;
            if(wifi_spi_transfer_command(&temp_packets, WIFI_SPI_RECVIVE_SIZE)) { break; }
            // 检查收到的包中是否有数据
            if((WIFI_SPI_REPLY_DATA_START == temp_packets.head.command) || (WIFI_SPI_REPLY_DATA_END == temp_packets.head.command))
            {
                // 保存接收到的数据
                if(temp_packets.head.length)
                {
                    fifo_write_buffer(&wifi_spi_fifo, temp_packets.buffer, temp_packets.head.length);
                }
            }
        }
        // 无论正常完成还是break退出，都释放软件忙标记，让下一轮可以服务。
        wifi_spi_mutex = WIFI_SPI_IDLE;
    }
    return length; // 返回剩余字节，应用层据此续传或故障处理。
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     WIFI SPI 字符串发送函数并同步接收数据
// 参数说明     *string           需要发送的字符串
// 返回参数     void          
// 使用示例     wifi_spi_send_string("123");
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
void wifi_spi_send_string(const char *string)
{
    wifi_spi_send_buffer((uint8*)string, strlen(string));
}
    
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     WIFI SPI 读取缓冲区
// 参数说明     *buff           接收缓冲区
// 参数说明     length          读取数据长度
// 返回参数     uint32          实际读取数据长度
// 使用示例     wifi_spi_read_buffer(buffer, 100);
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
// 1. 返回实际交给调用者的字节数，最多是调用者请求length；未交付尾部保留在驱动FIFO。
uint32 wifi_spi_read_buffer (uint8 *buffer, uint32 length)
{
    zf_assert(NULL != buffer);
    uint32 fifo_read_length;
    uint32 write_length = 0; // 累计本次已经复制给上层的字节数。
    if(wifi_spi_cancel_pending()) { return 0; } // 取消优先于领取旧FIFO字节。
    uint8 transfers = 0; // 本次主动从模块取包的次数。
    wifi_spi_packets_struct temp_packets = {0}; // 等待超时退出时，包状态保持为无效值

    // 先消费驱动已有FIFO数据，再尝试从模块继续读取；共享V1解析器另有128字节FIFO。
    // 首先判断FIFO中是否有数据，如果有则先从FIFO读取
    if(fifo_used(&wifi_spi_fifo))
    {
        fifo_read_length = fifo_used(&wifi_spi_fifo);
        // 本次最多取调用者容量与FIFO已有量的较小值，不能覆盖上层缓冲区。
        fifo_read_length = length < fifo_read_length ? length : fifo_read_length;
        fifo_read_buffer(&wifi_spi_fifo, buffer, &fifo_read_length, FIFO_READ_AND_CLEAN);
        
        buffer += fifo_read_length; // 后续数据接到调用者已填前缀之后。
        length -= fifo_read_length; // 调用者缓冲剩余容量，单位字节。
        write_length += fifo_read_length; // 实际交付字节数，含驱动FIFO已缓存数据。
    }
    
// 2. 打开主动读取配置时，空发送也可查询模块待收数据；只在软件忙标记空闲时执行。
#if(1 == WIFI_SPI_READ_TRANSFER)
    if(WIFI_SPI_IDLE == wifi_spi_mutex)
    {
        wifi_spi_mutex = WIFI_SPI_BUSY;                         // 将通讯状态设置为忙
        
        do
        {
            // 驱动FIFO空闲容量不足一个32字节包，或模块未就绪/截止到期，任一成立就退出。
            if( (WIFI_SPI_RECVIVE_SIZE > wifi_spi_fifo.size) || // 如果缓冲区空间不够则不再读取
                (wifi_spi_wait_idle(OTHER_TIME_OUT))            // 超时退出
              )
            {
                break;
            }
            
            temp_packets.head.command = WIFI_SPI_DATA;
            temp_packets.head.length  = 0;
            if(wifi_spi_transfer_command(&temp_packets, WIFI_SPI_RECVIVE_SIZE)) { break; }
            // 检查收到的包中是否有数据
            if( ((WIFI_SPI_REPLY_DATA_START == temp_packets.head.command) || (WIFI_SPI_REPLY_DATA_END == temp_packets.head.command)) &&
                (temp_packets.head.length)
              )
            {
                if(length)
                {
                    // 当前调用者缓冲最多装本包有效字节；复制量取剩余容量与本包长度的较小值。
                    fifo_read_length = length >= temp_packets.head.length ? temp_packets.head.length : length;
                    memcpy(buffer, temp_packets.buffer, fifo_read_length); // 只复制fifo_read_length字节，不假定上层一定有32字节余量。
                    buffer += fifo_read_length;
                    length -= fifo_read_length; // 扣除已交付量后仍有空间，才继续向调用者缓冲写。
                    write_length += fifo_read_length;
                    // 调用者剩余空间装不下的本包尾部写回驱动FIFO，下一次read_buffer继续消费。
                    if(fifo_read_length < temp_packets.head.length) // 外部缓冲不足多余部分写入FIFO
                    {
                        fifo_write_buffer(&wifi_spi_fifo, temp_packets.buffer + fifo_read_length, temp_packets.head.length - fifo_read_length);
                    }
                }
                else                                                // 全部写入FIFO
                {
                    fifo_write_buffer(&wifi_spi_fifo, temp_packets.buffer, temp_packets.head.length);
                }
            }
            // 运行最多主动读取4包，并受整轮时间截止双重限制；停车按模块还有数据继续读。
            ++transfers;
        }while(WIFI_SPI_REPLY_DATA_START == temp_packets.head.command &&
               (!runtime_service_enabled || transfers < WIFI_SPI_RUNTIME_RX_PACKETS));
        wifi_spi_mutex = WIFI_SPI_IDLE; // 主动读取结束/异常退出都释放忙标记。
    }
#endif
    
    if(wifi_spi_cancel_pending()) { return 0; } // 取消发生于读取过程中也不返回已领取的半包。
    return write_length;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     WiFi 模块初始化
// 参数说明     *wifi_ssid      目标连接的 WiFi 的名称 字符串形式
// 参数说明     *pass_word      目标连接的 WiFi 的密码 字符串形式
// 返回参数     uint8           模块初始化状态 0-成功 1-错误
// 使用示例     wifi_spi_init("SEEKFREE", "SEEKFREE123");
// 备注信息     wifi_spi_init("SEEKFREE", NULL); // 连接没有密码的WIFI热点
//-------------------------------------------------------------------------------------------------------------------
uint8 wifi_spi_init (char *wifi_ssid, char *pass_word)
{
    uint8 return_state = 0;
    
    // 初始化属于上电/停止恢复，复位与长等待不允许从行车通信路径进入。
    /* 只启用计数器，不清零，避免干扰其它模块的计时。 */
    // 启用DWT的调试追踪/周期计数功能，仅置使能位，不清CYCCNT以免影响其它计时。
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    // 新连接先清版本/MAC/IP与驱动FIFO，防止上次连接的数据被当作本次成功证据。
    /* 重连先清信息，不能用上一连接的版本字符串判定新连接成功。 */
    memset(wifi_spi_version, 0, sizeof(wifi_spi_version));
    memset(wifi_spi_mac_addr, 0, sizeof(wifi_spi_mac_addr));
    memset(wifi_spi_ip_addr_port, 0, sizeof(wifi_spi_ip_addr_port));
    fifo_init(&wifi_spi_fifo, FIFO_DATA_8BIT, wifi_spi_buffer, WIFI_SPI_RECVIVE_FIFO_SIZE); // 重建驱动FIFO，旧连接尾部不跨重连复用。
    // SPI_1/20MHz/MODE3沿用工程宏；外部片选GPIO由本驱动控制，硬件CS不接管。
    spi_init(WIFI_SPI_INDEX, SPI_MODE3, WIFI_SPI_SPEED, WIFI_SPI_SCK_PIN, WIFI_SPI_MOSI_PIN, WIFI_SPI_MISO_PIN, SPI_CS_NULL);//硬件SPI初始化
    gpio_init(WIFI_SPI_CS_PIN,  GPO, 1, GPO_PUSH_PULL);
    gpio_init(WIFI_SPI_RST_PIN, GPO, 1, GPO_PUSH_PULL);
    gpio_init(WIFI_SPI_INT_PIN, GPI, 0, GPI_PULL_DOWN);
    
    // 复位
    gpio_set_level(WIFI_SPI_RST_PIN, 0);
    if(wifi_spi_delay_cancelable(10)) { return 1; }
    gpio_set_level(WIFI_SPI_RST_PIN, 1);
    
    // 等待模块初始化
    if(wifi_spi_delay_cancelable(100)) { return 1; }
    wifi_spi_mutex = WIFI_SPI_IDLE;

    do
    {
        // 固件版本信息以字符串形式保存在wifi_spi_version数组中
        return_state = wifi_spi_get_version();
        if(return_state)
        {
            break;
        }
        
        // MAC地址信息以字符串形式保存在wifi_spi_mac_addr数组中
        wifi_spi_get_mac_addr();
        if(wifi_spi_cancel_pending()) { return_state = 1; break; }

        // 传NULL表示这里只检查模块；应用层随后单独连接热点，便于区分失败阶段。
        if(NULL == wifi_ssid)
        {
            // 初始化的时候不需要连接WIFI
            break;
        }
        return_state = wifi_spi_wifi_connect(wifi_ssid, pass_word);
        if(return_state)
        {
            break;
        }
        
    #if(1 == WIFI_SPI_AUTO_CONNECT)
        return_state = wifi_spi_socket_connect("TCP", WIFI_SPI_TARGET_IP, WIFI_SPI_TARGET_PORT, WIFI_SPI_LOCAL_PORT);
        if(return_state)
        {
            break;
        }
    #endif
        
    #if(2 == WIFI_SPI_AUTO_CONNECT)
        return_state = wifi_spi_socket_connect("UDP", WIFI_SPI_TARGET_IP, WIFI_SPI_TARGET_PORT, WIFI_SPI_LOCAL_PORT);
        if(return_state)
        {
            break;
        }
    #endif
    }while(0);

    return return_state;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     兼容旧组件的多缓冲区发送接口
// 参数说明     multi_buffer    各段数据地址及长度
// 返回参数     uint32          剩余未发送字节数
// 备注信息     使用 V2.0 发送接口；任一段发送失败后停止后续数据段
//-------------------------------------------------------------------------------------------------------------------
// 按数组顺序发送多个缓冲段；遇到第一处失败即停止，不能跳过缺段继续拼接后段。
uint32 wifi_spi_send_buffer_multi(wifi_spi_send_multi_struct *multi_buffer)
{
    uint8 index; // 多段数组索引，范围由WIFI_SPI_MAX_MULTI限制。
    uint32 remaining = 0; // 当前及以后所有段的剩余字节总数。
    uint32 unsent; // 单段发送返回的未发送字节数。

    zf_assert(NULL != multi_buffer);
    // 先求全部未发字节数，随后只扣已成功发送的前缀；返回值包含当前尾段及后续段。
    for(index = 0; index < WIFI_SPI_MAX_MULTI; ++index)
    {
        remaining += multi_buffer->length[index];
    }
    for(index = 0; index < WIFI_SPI_MAX_MULTI; ++index)
    {
        if(multi_buffer->length[index] == 0)
        {
            continue;
        }
        // 有长度却无数据地址不能发送，保留当前与后续段的全部剩余长度。
        if(multi_buffer->source[index] == NULL)
        {
            return remaining;
        }
        unsent = wifi_spi_send_buffer(multi_buffer->source[index], multi_buffer->length[index]);
        // 本段长度减unsent是本段已发字节，不能把unsent当已发量扣除。
        remaining -= multi_buffer->length[index] - unsent;
        if(unsent != 0)
        {
            return remaining;
        }
    }
    return remaining;
}
