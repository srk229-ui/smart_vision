"""直接编译现有函数，验证人工请求规则及硬件等待截止时间；不代替实机时序验收。"""

from pathlib import Path
import re
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[4]

COMMON = r"""
#include <assert.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
typedef uint8_t uint8;
typedef uint16_t uint16;
typedef uint32_t uint32;
"""

CANCEL_STUB = r"""
// 此旧专项覆盖受检交换/协议边界；按键取消的GPIO/FIFO收尾由独立驱动测试覆盖。
uint8 wifi_spi_cancel_pending(void) { return 0; }
uint8 wifi_spi_cancel_requested(void) { return 0; }
"""

PARAM_STUB = r"""
#include "control_types.h"
#define SEEKFREE_ASSISTANT_SET_PARAMETR_COUNT 8
uint8 seekfree_assistant_parameter_update_flag[8];
float seekfree_assistant_parameter[8];
uint32 wifi_assistant_parameter_reject_count;
float wifi_assistant_servo_gain;
static uint8 width_calibration_request, stream_mode, image_mode, stop_seen;
static uint8 replay_enabled;
static uint32 replay_generation;
static struct { uint32 generation; } vision_stop_log;
static uint32 last_send_ms, elapsed_time_ms, send_period_ms;
static uint8 raw_frame_valid;
static ControlParams actual;
static uint32 actual_version;
static uint8 allow_apply, allow_run;
static uint32 apply_calls, run_calls, stop_calls;
static ControlMode requested_mode;
void control_get_params(ControlParams *p, uint32 *version) { *p = actual; *version = actual_version; }
uint8 control_apply_params(const ControlParams *p, uint32 version) {
    ++apply_calls;
    if(!allow_apply || version != actual_version) { return 0; }
    actual = *p; ++actual_version; return 1;
}
uint8 control_request_run(ControlMode mode) { ++run_calls; requested_mode = mode; return allow_run; }
void control_request_stop(void) { ++stop_calls; }
void camera_exposure_request_record(void) {}
uint8 control_reset_fault(void) { return 1; }
static void reset_case(void) {
    memset(seekfree_assistant_parameter_update_flag, 0, sizeof(seekfree_assistant_parameter_update_flag));
    memset(seekfree_assistant_parameter, 0, sizeof(seekfree_assistant_parameter));
    memset(&actual, 0, sizeof(actual)); actual.kp_left = actual.kp_right = 50;
    actual_version = 10; allow_apply = allow_run = 1;
    apply_calls = run_calls = stop_calls = wifi_assistant_parameter_reject_count = 0;
    width_calibration_request = stop_seen = 0;
}
void request(uint8 channel, float value) {
    seekfree_assistant_parameter_update_flag[channel] = 1;
    seekfree_assistant_parameter[channel] = value;
}
"""

PARAM_CASES = r"""
int main(void) {
    reset_case(); request(0, 0); request(1, 70); apply_parameters(1);
    assert(stop_calls == 1 && apply_calls == 0 && run_calls == 0);
    assert(actual.kp_left == 50 && wifi_assistant_parameter_reject_count == 1);
    reset_case(); request(0, 20); request(4, 2); apply_parameters(1);
    assert(stop_calls == 0 && apply_calls == 0 && run_calls == 0 && wifi_assistant_parameter_reject_count == 2);
    reset_case(); request(0, 20); request(1, 60); request(2, 3); request(3, 4); apply_parameters(0);
    assert(apply_calls == 1 && actual_version == 11 && run_calls == 0);
    assert(actual.target_left == 20 && actual.target_right == 20 && actual.kp_left == 60 && actual.kp_right == 60);
    assert(actual.ki_left == 3 && actual.ki_right == 3 && actual.calibration_confirmed == 1);
    assert(stop_calls == 0); // 收到正目标与确认仍保持停止，不调用远程启动。
    reset_case(); request(3, 2); apply_parameters(0);
    assert(run_calls == 0 && apply_calls == 0);
    request(0, 15); apply_parameters(0); assert(run_calls == 0 && apply_calls == 1 && actual.target_left == 15);
    reset_case(); request(0, 20); request(3, 0.25f); apply_parameters(0);
    assert(apply_calls == 0 && run_calls == 0 && wifi_assistant_parameter_reject_count == 1);
    reset_case(); request(0, 20); request(1, NAN); apply_parameters(0);
    assert(apply_calls == 0 && run_calls == 0 && wifi_assistant_parameter_reject_count == 1);
    reset_case(); allow_apply = 0; request(0, 20); apply_parameters(0);
    assert(apply_calls == 1 && run_calls == 0 && wifi_assistant_parameter_reject_count == 1);
    reset_case(); request(0, 20); request(3, 5); apply_parameters(0);
    assert(stop_calls == 1 && apply_calls == 1 && run_calls == 0 && actual.calibration_confirmed == 0);
    reset_case(); request(0, 20); request(3, 6); apply_parameters(0);
    assert(width_calibration_request == 1 && run_calls == 0);
    reset_case(); allow_run = 0; request(0, 20); apply_parameters(0);
    assert(apply_calls == 1 && run_calls == 0 && wifi_assistant_parameter_reject_count == 0);
    // 高于旧100上限的本轮目标、400边界可提交；超限拒绝且不触发启动。
    reset_case(); request(0, 348.146f); apply_parameters(0);
    assert(apply_calls == 1 && run_calls == 0 && actual.target_left == 348.146f && actual.target_right == 348.146f);
    reset_case(); request(0, CONTROL_TARGET_COUNTS_MAX); apply_parameters(0);
    assert(apply_calls == 1 && run_calls == 0 && actual.target_left == CONTROL_TARGET_COUNTS_MAX);
    reset_case(); request(0, CONTROL_TARGET_COUNTS_MAX + 0.01f); apply_parameters(0);
    assert(apply_calls == 0 && run_calls == 0 && wifi_assistant_parameter_reject_count == 1);
    // 已解析的STOP即使被同通道后续正目标覆盖，也优先停止且不应用整组参数。
    reset_case(); stop_seen = 1; request(0, 20); request(1, 70); apply_parameters(0);
    assert(stop_calls == 1 && apply_calls == 0 && run_calls == 0 && actual.kp_left == 50);
    reset_case(); request(3, 1); request(0, 25); apply_parameters(0);
    assert(run_calls == 0 && apply_calls == 1 && actual.target_left == 25);
    puts("PASS V1 local-start contract: stopped tuning, no remote START, remote STOP priority, limits");
    return 0;
}
"""

SPI_STUB = r"""
typedef struct { uint32 CR, CFGR1, TCR, CCR, FSR; } LPSPI_Type;
typedef struct { volatile uint32 CYCCNT; } CycleCounter;
static CycleCounter cycle;
#define DWT (&cycle)
static LPSPI_Type peripheral;
#define LPSPI_BASE_PTRS { 0, &peripheral }
#define WIFI_SPI_INDEX 1
#define LPSPI_CFGR1_NOSTALL_MASK 1u
#define LPSPI_TCR_CONT_MASK 0x200000u
#define LPSPI_TCR_CONTC_MASK 0x100000u
#define LPSPI_TCR_RXMSK_MASK 0x80000u
#define LPSPI_TCR_TXMSK_MASK 0x40000u
#define LPSPI_TCR_PCS_MASK 0x3000000u
#define LPSPI_TCR_FRAMESZ_MASK 0xFFFu
#define LPSPI_TCR_FRAMESZ(x) (x)
#define kLPSPI_ModuleBusyFlag 1u
#define kLPSPI_TransmitErrorFlag 2u
#define kLPSPI_ReceiveErrorFlag 4u
#define kLPSPI_AllStatusFlag 255u
uint32 system_clock = 1000000u;
static uint8 fault_mode, pending, last_byte;
static uint32 status_flags; // 模拟重新使能产生的伪REF，以及发送后真实REF。
uint32 LPSPI_GetStatusFlags(LPSPI_Type *base) {
    (void)base; ++cycle.CYCCNT;
    if(fault_mode == 1) { return kLPSPI_ModuleBusyFlag; }
    if(fault_mode == 4) { return kLPSPI_TransmitErrorFlag; }
    return status_flags;
}
uint32 LPSPI_GetTxFifoCount(LPSPI_Type *base) {
    (void)base; ++cycle.CYCCNT; return fault_mode == 2 ? 4 : 0;
}
uint32 LPSPI_GetTxFifoSize(LPSPI_Type *base) { (void)base; return 4; }
uint32 LPSPI_GetRxFifoCount(LPSPI_Type *base) {
    (void)base; ++cycle.CYCCNT; return fault_mode == 3 ? 0 : pending;
}
void LPSPI_Enable(LPSPI_Type *base, uint8 enabled) { (void)base; if(enabled) { status_flags |= kLPSPI_ReceiveErrorFlag; } }
void LPSPI_FlushFifo(LPSPI_Type *base, uint8 tx, uint8 rx) { (void)base; (void)tx; (void)rx; pending = 0; }
void LPSPI_ClearStatusFlags(LPSPI_Type *base, uint32 flags) { (void)base; status_flags &= ~flags; }
void LPSPI_WriteData(LPSPI_Type *base, uint32 value) { (void)base; last_byte = (uint8)value; pending = 1; if(fault_mode == 5) { status_flags |= kLPSPI_ReceiveErrorFlag; pending = 0; } }
uint32 LPSPI_ReadData(LPSPI_Type *base) { (void)base; pending = 0; return last_byte ^ 0x5Au; }
"""

SPI_CASES = r"""
static void reset_case(uint8 mode) {
    status_flags = 0; fault_mode = mode; cycle.CYCCNT = 0; pending = 0; wifi_spi_io_error_count = 0;
    wifi_spi_runtime_last_us = wifi_spi_runtime_max_us = 0; runtime_service_enabled = 0;
}
int main(void) {
    uint8 tx_large[80] = {0}; uint8 rx_large[80];
    uint8 tx[4] = {1, 2, 3, 4}; uint8 rx[4]; uint8 alias[4] = {1, 2, 3, 4}; uint8 mode;
    reset_case(0); peripheral.TCR = 0xC8000007u; peripheral.CCR = 0x12345678u; wifi_spi_service_begin(1);
    assert(wifi_spi_exchange_checked(tx, rx, 4) == 0);
    assert((peripheral.TCR & 0xF8000000u) == 0xC8000000u && (peripheral.TCR & 0xFFFu) == 7);
    assert(peripheral.CCR == 0x12345678u);
    assert(rx[0] == (1 ^ 0x5A) && rx[3] == (4 ^ 0x5A)); wifi_spi_service_end();
    assert(wifi_spi_runtime_last_us > 0 && wifi_spi_runtime_last_us < 2000);
    reset_case(0); assert(wifi_spi_exchange_checked(alias, alias, 4) == 0);
    assert(alias[0] == (1 ^ 0x5A) && alias[3] == (4 ^ 0x5A));
    for(mode = 1; mode <= 5; ++mode) {
        reset_case(mode); wifi_spi_service_begin(1);
        assert(wifi_spi_exchange_checked(tx, rx, 4) == 1);
        assert(wifi_spi_io_error_count == 1 && cycle.CYCCNT <= 1003);
        if(mode == 5) { assert(cycle.CYCCNT < 100 && wifi_spi_failure_trace[5] == 4 && (wifi_spi_failure_trace[1] & kLPSPI_ReceiveErrorFlag)); }
    }
    reset_case(0); wifi_spi_service_begin(1);
    assert(wifi_spi_service_is_running() == 1);
    assert(wifi_spi_exchange_checked(tx, rx, 4) == 0);
    cycle.CYCCNT += 1800;
    assert(wifi_spi_exchange_checked(tx_large, rx_large, 80) == 1 && cycle.CYCCNT <= 2003);
    wifi_spi_service_end(); assert(wifi_spi_service_is_running() == 0);
    reset_case(1); wifi_spi_service_begin(1); cycle.CYCCNT = 1999;
    assert(wifi_spi_exchange_checked(tx, rx, 4) == 1 && cycle.CYCCNT <= 2003);
    wifi_spi_service_end(); assert(wifi_spi_runtime_max_us >= 2000 && wifi_spi_runtime_max_us <= 2003);
    reset_case(1); cycle.CYCCNT = 0xFFFFFF00u; wifi_spi_service_begin(1);
    assert(wifi_spi_exchange_checked(tx, rx, 4) == 1);
    assert((uint32)(cycle.CYCCNT - 0xFFFFFF00u) <= 1003);
    reset_case(1); assert(wifi_spi_exchange_checked(tx, rx, 4) == 1 && cycle.CYCCNT <= 10003);
    puts("PASS checked SPI normal, alias, busy, TX, RX, error, shared receive/send deadline and wrap, enable false REF, genuine transfer REF (11 scenarios)");
    return 0;
}
"""


COMMAND_STUB = r"""
#define WIFI_SPI_CS_PIN 1
#define WIFI_SPI_INVALID1 0
#define WIFI_SPI_RECVIVE_SIZE 32
typedef struct { uint8 command, reserve; uint16 length; } wifi_spi_head_struct;
typedef struct { wifi_spi_head_struct head; uint8 buffer[32]; } wifi_spi_packets_struct;
static uint16 reply_length;
static uint8 reply_command, fail_phase, phases, cs_low_count, cs_high_count;
volatile uint32 wifi_spi_io_error_count;
void gpio_low(uint8 pin) { (void)pin; ++cs_low_count; }
void gpio_high(uint8 pin) { (void)pin; ++cs_high_count; }
uint8 wifi_spi_exchange_checked(const uint8 *tx, uint8 *rx, uint16 length) {
    ++phases; assert(tx == rx);
    if(phases == fail_phase) { return 1; }
    if(phases == 1) {
        wifi_spi_head_struct reply = {0};
        assert(length == sizeof(reply)); reply.command = reply_command; reply.length = reply_length;
        memcpy(rx, &reply, sizeof(reply));
    } else { memset(rx, 0x5A, length); }
    return 0;
}
static void reset_command(uint16 length, uint8 command, uint8 failure) {
    reply_length = length; reply_command = command; fail_phase = failure;
    phases = cs_low_count = cs_high_count = 0; wifi_spi_io_error_count = 0;
}
"""

COMMAND_CASES = r"""
int main(void) {
    struct { wifi_spi_head_struct head; uint32 guard; } ack = {{0}, 0xA55AC33Cu};
    wifi_spi_packets_struct packet = {{0}, {0}};
    // 设置命令只用4B头对象；应答声明有载荷也不允许读取或改写头后的哨兵。
    reset_command(4, 0x80, 0);
    assert(wifi_spi_transfer_command((wifi_spi_packets_struct *)&ack.head, 0) == 0);
    assert(ack.head.command == 0x80 && ack.head.length == 4 && ack.guard == 0xA55AC33Cu);
    assert(phases == 1 && cs_low_count == 1 && cs_high_count == 1 && wifi_spi_io_error_count == 0);
    reset_command(65535, 0x81, 0);
    assert(wifi_spi_transfer_command((wifi_spi_packets_struct *)&ack.head, 0) == 0);
    assert(ack.head.command == 0x81 && ack.guard == 0xA55AC33Cu && phases == 1);
    // 有实际载荷窗口时，边界通过，超界仍必须失效包头并报告错误。
    reset_command(32, 0xA0, 0);
    assert(wifi_spi_transfer_command(&packet, 32) == 0 && phases == 2 && packet.buffer[31] == 0x5A);
    reset_command(33, 0xA0, 0);
    assert(wifi_spi_transfer_command(&packet, 32) == 1 && wifi_spi_io_error_count == 1);
    assert(packet.head.command == 0 && packet.head.length == 0 && cs_high_count == 1);
    reset_command(4, 0x80, 1);
    assert(wifi_spi_transfer_command((wifi_spi_packets_struct *)&ack.head, 0) == 1);
    assert(phases == 1 && cs_high_count == 1 && ack.head.command == 0 && ack.guard == 0xA55AC33Cu);
    reset_command(32, 0xA0, 2);
    assert(wifi_spi_transfer_command(&packet, 32) == 1 && phases == 2 && cs_high_count == 1);
    puts("PASS actual command header-only ACK, no payload overwrite, capacity boundary/overflow, transport errors (6 scenarios)");
    return 0;
}
"""


PARSER_STUB = r"""
#define SEEKFREE_ASSISTANT_BUFFER_SIZE 128
#define SEEKFREE_ASSISTANT_RECEIVE_HEAD 0x55
#define SEEKFREE_ASSISTANT_RECEIVE_SET_PARAMETER 0x20
#define zf_assert assert
static uint8 assistant_bytes[128];
static fifo_struct seekfree_assistant_fifo;
static uint8 incoming[128];
static uint32 incoming_length;
static uint32 last_receive_capacity;
static uint8 oversized_receive;
volatile uint32 seekfree_assistant_receive_error_count;
void seekfree_assistant_receive_reset(void);
void (*seekfree_assistant_parameter_observer)(uint8 channel, float value);
static uint32 read_transport(uint8 *destination, uint32 capacity) {
    uint32 count = incoming_length < capacity ? incoming_length : capacity;
    last_receive_capacity = capacity;
    memcpy(destination, incoming, count);
    memmove(incoming, incoming + count, incoming_length - count);
    incoming_length -= count;
    return oversized_receive ? capacity + 1 : count;
}
uint32 (*seekfree_assistant_receive_callback)(uint8 *buffer, uint32 length) = read_transport;
"""

PARSER_CASES = r"""
static void reset_parser(void) {
    reset_case(); incoming_length = 0; oversized_receive = 0;
    seekfree_assistant_receive_error_count = last_receive_capacity = 0;
    seekfree_assistant_fifo.buffer = assistant_bytes;
    seekfree_assistant_fifo.max = seekfree_assistant_fifo.size = sizeof(assistant_bytes);
    seekfree_assistant_fifo.type = FIFO_DATA_8BIT;
    seekfree_assistant_fifo.execution = FIFO_IDLE;
    seekfree_assistant_parameter_observer = assistant_parameter_event;
    seekfree_assistant_receive_reset();
}
static void queue_parameter(float value, uint8 corrupt) {
    seekfree_assistant_parameter_struct packet = {0};
    packet.head = SEEKFREE_ASSISTANT_RECEIVE_HEAD;
    packet.function = SEEKFREE_ASSISTANT_RECEIVE_SET_PARAMETER;
    packet.channel = 1; packet.data = value;
    packet.check_sum = seekfree_assistant_sum((uint8 *)&packet, sizeof(packet));
    if(corrupt) { packet.check_sum ^= 1; }
    assert(incoming_length + sizeof(packet) <= sizeof(incoming));
    memcpy(incoming + incoming_length, &packet, sizeof(packet));
    incoming_length += sizeof(packet);
}
int main(void) {
    uint8 trailing;
    reset_parser(); queue_parameter(0, 0); queue_parameter(20, 0);
    seekfree_assistant_data_analysis(); apply_parameters(0);
    assert(stop_calls == 1 && apply_calls == 0 && run_calls == 0 && stop_seen == 0);
    reset_parser(); queue_parameter(20, 0); queue_parameter(0, 0);
    seekfree_assistant_data_analysis(); apply_parameters(1);
    assert(stop_calls == 1 && apply_calls == 0 && run_calls == 0);
    reset_parser(); queue_parameter(0, 1); queue_parameter(20, 0);
    seekfree_assistant_data_analysis(); apply_parameters(0);
    assert(stop_calls == 0 && apply_calls == 1 && run_calls == 0);
    reset_parser(); queue_parameter(20, 1); queue_parameter(0, 0);
    seekfree_assistant_data_analysis(); apply_parameters(0);
    assert(stop_calls == 1 && apply_calls == 0 && run_calls == 0);
    reset_parser(); queue_parameter(20, 0);
    trailing = incoming[7]; incoming_length = 7;
    seekfree_assistant_data_analysis(); assert(fifo_used(&seekfree_assistant_fifo) == 7);
    seekfree_assistant_receive_reset();
    assert(fifo_used(&seekfree_assistant_fifo) == 0 && seekfree_assistant_parameter_update_flag[0] == 0);
    incoming[0] = trailing; incoming_length = 1;
    seekfree_assistant_data_analysis(); apply_parameters(0);
    assert(apply_calls == 0 && run_calls == 0);
    queue_parameter(20, 0); seekfree_assistant_data_analysis(); apply_parameters(0);
    assert(apply_calls == 1 && run_calls == 0);
    reset_parser(); queue_parameter(20, 0); incoming_length = 3;
    seekfree_assistant_data_analysis(); assert(fifo_used(&seekfree_assistant_fifo) == 3);
    memset(incoming, 0xCC, sizeof(incoming)); incoming_length = 0;
    queue_parameter(0, 0); incoming_length = sizeof(incoming);
    seekfree_assistant_data_analysis();
    assert(last_receive_capacity == 125 && incoming_length == 3);
    assert(seekfree_assistant_receive_error_count == 0);
    apply_parameters(1);
    assert(stop_calls == 1 && apply_calls == 0 && run_calls == 0);
    seekfree_assistant_data_analysis(); assert(incoming_length == 0);
    queue_parameter(0, 0); seekfree_assistant_data_analysis(); apply_parameters(0);
    assert(stop_calls == 2 && apply_calls == 0 && run_calls == 0);
    reset_parser(); queue_parameter(20, 0); oversized_receive = 1;
    seekfree_assistant_data_analysis(); apply_parameters(0);
    assert(seekfree_assistant_receive_error_count == 1 && fifo_used(&seekfree_assistant_fifo) == 0);
    assert(seekfree_assistant_parameter_update_flag[0] == 0 && run_calls == 0 && apply_calls == 0);
    puts("PASS actual FIFO + V1 parser + observer + apply: STOP orders, bad checksum, reconnect, 3B+128B burst, receive error (7 scenarios)");
    return 0;
}
"""


def extract_function(source, name):
    pattern = r"^(?:static\s+)?(?:void|uint8|uint32|fifo_state_enum)\s+" + re.escape(name) + r"\s*\("
    match = re.search(pattern, source, re.M)
    if not match:
        raise ValueError(name)
    opening = source.index("{", match.start())
    depth = 1
    cursor = opening + 1
    while depth:
        if source[cursor] == "{":
            depth += 1
        elif source[cursor] == "}":
            depth -= 1
        cursor += 1
    return source[match.start():cursor]


def run_case(directory, name, code):
    source = directory / (name + ".c")
    executable = directory / (name + ".exe")
    source.write_text(code, encoding="utf-8")
    # 优先使用可启动的 GCC；部分受限 Windows 环境需要改用已安装的 MSVC。
    try:
        subprocess.run(["gcc", "--version"], check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        command = ["gcc", "-std=c99", "-Wall", "-Wextra", "-Werror", "-I", str(ROOT / "demo/user/inc"),
                   str(source), "-o", str(executable)]
    except (OSError, subprocess.CalledProcessError):
        versions = sorted(Path("C:/Program Files/Microsoft Visual Studio/2022/Community/VC/Tools/MSVC").glob("*"))
        kits = sorted(Path("C:/Program Files (x86)/Windows Kits/10/Include").glob("*"))
        if not versions or not kits:
            raise RuntimeError("没有可用的宿主 C 编译器")
        compiler = versions[-1]
        kit = kits[-1]
        command = [str(compiler / "bin/Hostx64/x64/cl.exe"), "/nologo", "/std:c11", "/utf-8", "/W4", "/WX", "/MD",
                   "/I" + str(ROOT / "demo/user/inc"), "/I" + str(compiler / "include"),
                   "/I" + str(kit / "ucrt"), "/I" + str(kit / "shared"), "/I" + str(kit / "um"),
                   "/Fo" + str(directory / (name + ".obj")), "/Fe" + str(executable), str(source), "/link",
                   "/LIBPATH:" + str(compiler / "lib/x64"),
                   "/LIBPATH:" + str(kit.parents[1] / "Lib" / kit.name / "ucrt/x64"),
                   "/LIBPATH:" + str(kit.parents[1] / "Lib" / kit.name / "um/x64")]
    subprocess.run(command, check=True, cwd=directory)
    subprocess.run([str(executable)], check=True, timeout=5)


def main():
    assistant = (ROOT / "demo/user/src/wifi_assistant.c").read_text(encoding="utf-8")
    start = assistant.index("static void apply_parameters(")
    end = assistant.index("\n#endif", start)
    parameters = assistant[start:end]
    observer = extract_function(assistant, "assistant_parameter_event")
    driver = (ROOT / "demo/drivers/wifi_spi_v2/zf_device_wifi_spi.c").read_bytes().decode("gb18030")
    start = driver.index("static LPSPI_Type *const wifi_spi_bases")
    end = driver.index("static uint8 wifi_spi_wait_idle", start)
    checked = driver[start:end]
    # 旧受检交换夹具不模拟模块FIFO与复位脚；取消函数使用COMMON中的恒0桩。
    for name in ('wifi_spi_set_cancel_hook', 'wifi_spi_cancel_requested', 'wifi_spi_cancel_pending', 'wifi_spi_delay_cancelable'):
        if re.search(r'\b' + name + r'\s*\(', checked):
            checked = checked.replace(extract_function(checked, name), '')
    checked = re.sub(r'^static uint8 \(\*wifi_spi_cancel_hook\)\(void\);[^\n]*\n', '', checked, flags=re.M)
    header = (ROOT / "demo/drivers/wifi_spi_v2/zf_device_wifi_spi.h").read_bytes().decode("gb18030")
    defines = "\n".join(re.findall(r"^#define WIFI_SPI_(?:RUNTIME|STOP)_[^\r\n]+", header, re.M))
    build_parent = (ROOT / "demo/drivers/wifi_spi_v2/tests/_build").resolve()
    assert build_parent.is_relative_to(ROOT.resolve())
    build_parent.mkdir(parents=True, exist_ok=True)
    shared = (ROOT / "libraries/zf_components/seekfree_assistant.c").read_bytes().decode("gb18030")
    fifo_header = (ROOT / "libraries/zf_common/zf_common_fifo.h").read_bytes().decode("gb18030")
    fifo_header = fifo_header[fifo_header.index("typedef enum"):fifo_header.rindex("#endif")]
    fifo_source = (ROOT / "libraries/zf_common/zf_common_fifo.c").read_bytes().decode("gb18030")
    fifo_functions = "\n".join(extract_function(fifo_source, name) for name in
                                 ["fifo_head_offset", "fifo_end_offset", "fifo_clear", "fifo_used", "fifo_write_buffer", "fifo_read_buffer"])
    protocol_header = (ROOT / "libraries/zf_components/seekfree_assistant.h").read_bytes().decode("gb18030")
    parameter_type = re.search(r"typedef struct\s*\{[^}]+}seekfree_assistant_parameter_struct;", protocol_header, re.S).group(0)
    parser_functions = "\n".join(extract_function(shared, name) for name in
                                   ["seekfree_assistant_sum", "seekfree_assistant_data_analysis", "seekfree_assistant_receive_reset"])
    parser_case = COMMON + fifo_header + PARAM_STUB + parameter_type + PARSER_STUB + fifo_functions + observer + parameters + parser_functions + PARSER_CASES
    with tempfile.TemporaryDirectory(prefix="m2_wifi_", dir=build_parent) as temporary:
        directory = Path(temporary).resolve()
        assert directory.is_relative_to(build_parent)
        run_case(directory, "parameters", COMMON + PARAM_STUB + parameters + PARAM_CASES)
        run_case(directory, "spi", COMMON + "\n" + defines + "\n" + CANCEL_STUB + SPI_STUB + checked + SPI_CASES)
        run_case(directory, "parser", parser_case)
        run_case(directory, "command", COMMON + CANCEL_STUB + COMMAND_STUB + extract_function(driver, "wifi_spi_transfer_command") + COMMAND_CASES)
    print("Software-only evidence: no physical module, PWM or timing validation.")


if __name__ == "__main__":
    main()
