"""编译真实联网等待、复位和事务函数，验证按键取消后的清理及重新初始化。"""

import argparse
from pathlib import Path
import re
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))
import run_software_checks as checks


def function(source, name):
    pattern = r"(?:static\s+)?(?:uint8|uint32|void)\s+" + re.escape(name) + r"\s*\([^;]*?\)\s*\{"
    match = re.search(pattern, source)
    if match is None:
        raise ValueError("未找到真实函数：" + name)
    depth = 1
    end = match.end()
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[match.start():end]


FIXTURE = r"""
#include <assert.h>
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
typedef uint8_t uint8;
typedef uint16_t uint16;
typedef uint32_t uint32;
#define SPI_1 0
#define SPI1_SCK_D12 0
#define SPI1_MOSI_D14 0
#define SPI1_MISO_D15 0
#define D13 0
#define B17 1
#define B16 2
#define GPO 0
#define GPI 1
#define GPO_PUSH_PULL 0
#define GPI_PULL_DOWN 0
#define FIFO_DATA_8BIT 0
#define FIFO_READ_AND_CLEAN 0
#define SPI_MODE3 0
#define SPI_CS_NULL 0
#define OTHER_TIME_OUT 1000
#define WIFI_CONNECT_TIME_OUT 10000
#define CoreDebug_DEMCR_TRCENA_Msk 1u
#define DWT_CTRL_CYCCNTENA_Msk 1u
#define zf_assert assert
#define spi_init(...) ((void)0)
typedef struct { uint32 DEMCR; } CoreDebugType;
typedef struct { uint32 CTRL; uint32 CYCCNT; } DwtType;
static CoreDebugType core_debug;
static DwtType dwt;
#define CoreDebug (&core_debug)
#define DWT (&dwt)
typedef struct { uint32 flags; } LPSPI_Type;
static LPSPI_Type peripheral_base;
#define LPSPI_BASE_PTRS { &peripheral_base }
#define kLPSPI_AllStatusFlag 0xffu
#define kLPSPI_ReceiveErrorFlag 0x02u
typedef struct { uint32 size, used; } fifo_struct;
static uint8 pins[3];
static uint32 now_us, cancel_at_us, max_delay_us, checks_count, flushes, clears;
static uint32 exchange_calls, cancel_exchange, query_calls;
static uint8 cancel_flag, module_ready, fail_exchange, pulse_mode;
static void LPSPI_Enable(LPSPI_Type *p, bool enable) { (void)p; (void)enable; }
static void LPSPI_ClearStatusFlags(LPSPI_Type *p, uint32 flags) { p->flags &= ~flags; ++clears; }
static void LPSPI_FlushFifo(LPSPI_Type *p, bool tx, bool rx) { (void)p; assert(tx && rx); ++flushes; }
static void gpio_high(uint32 pin) { pins[pin] = 1; }
static void gpio_low(uint32 pin) { pins[pin] = 0; }
static void gpio_set_level(uint32 pin, uint32 value) { pins[pin] = (uint8)value; }
static void gpio_init(uint32 pin, uint32 dir, uint32 value, uint32 config) {
    (void)dir; (void)config; pins[pin] = (uint8)value;
}
static uint8 gpio_get_level(uint32 pin) { return pin == B17 ? module_ready : pins[pin]; }
static void system_delay_us(uint32 us) {
    now_us += us; if(us > max_delay_us) max_delay_us = us;
    if(cancel_at_us && now_us >= cancel_at_us) cancel_flag = 1;
}
static void system_delay_ms(uint32 ms) { system_delay_us(ms * 1000u); }
static void fifo_init(fifo_struct *fifo, uint32 type, uint8 *storage, uint32 size) {
    (void)type; (void)storage; fifo->size = size; fifo->used = 0;
}
static uint32 fifo_used(fifo_struct *fifo) { return fifo->used; }
static void fifo_write_buffer(fifo_struct *fifo, const uint8 *data, uint32 length) {
    (void)data; fifo->used += length; fifo->size -= length;
}
static void fifo_read_buffer(fifo_struct *fifo, uint8 *data, uint32 *length, uint32 mode) {
    (void)mode; memset(data, 0x5a, *length); fifo->used -= *length; fifo->size += *length;
}
static uint32 wifi_spi_cycles_per_us(void) { return 1; }
static uint8 wifi_spi_deadline_expired(uint32 start, uint32 budget) { (void)start; (void)budget; return 0; }
static uint8 wifi_spi_get_version(void) { ++query_calls; return 0; }
static uint8 wifi_spi_get_mac_addr(void) { ++query_calls; return 0; }
uint8 wifi_spi_wifi_connect(char *ssid, char *password) { (void)ssid; (void)password; ++query_calls; return 0; }
uint8 wifi_spi_socket_connect(char *type, char *ip, char *port, char *local) {
    (void)type; (void)ip; (void)port; (void)local; ++query_calls; return 0;
}
"""

EXCHANGE = r"""
static uint8 wifi_spi_exchange_checked(const uint8 *out, uint8 *in, uint16 length) {
    assert(pins[WIFI_SPI_CS_PIN] == 0);
    ++exchange_calls;
    if(cancel_exchange && exchange_calls == cancel_exchange) cancel_flag = 1;
    if(fail_exchange) return 1;
    if(in && length == sizeof(wifi_spi_head_struct)) {
        wifi_spi_head_struct result = {WIFI_SPI_REPLY_OK, 0, 0};
        memcpy(in, &result, sizeof(result));
    } else if(in) {
        memset(in, 0, length);
    }
    (void)out;
    return 0;
}
"""

CASES = r"""
static uint8 cancel_hook(void) {
    assert(pins[WIFI_SPI_CS_PIN] == 1); /* 查询只能发生在CS事务外。 */
    ++checks_count;
    if(pulse_mode && cancel_flag) { cancel_flag = 0; return 1; }
    return cancel_flag;
}
static void reset_case(void) {
    now_us = cancel_at_us = max_delay_us = checks_count = flushes = clears = 0;
    exchange_calls = cancel_exchange = query_calls = 0;
    cancel_flag = fail_exchange = runtime_service_enabled = pulse_mode = 0;
    module_ready = 1;
    pins[WIFI_SPI_CS_PIN] = pins[WIFI_SPI_RST_PIN] = 1;
    wifi_spi_mutex = WIFI_SPI_BUSY;
    fifo_init(&wifi_spi_fifo, FIFO_DATA_8BIT, wifi_spi_buffer, WIFI_SPI_RECVIVE_FIFO_SIZE);
    fifo_write_buffer(&wifi_spi_fifo, (const uint8 *)"old", 3);
    wifi_spi_set_cancel_hook(cancel_hook);
}
static void assert_clean(void) {
    assert(pins[WIFI_SPI_CS_PIN] == 1 && pins[WIFI_SPI_RST_PIN] == 1);
    assert(wifi_spi_mutex == WIFI_SPI_IDLE && fifo_used(&wifi_spi_fifo) == 0);
    assert(wifi_spi_fifo.size == WIFI_SPI_RECVIVE_FIFO_SIZE);
    assert(flushes && clears);
}
int main(void) {
    uint8 bytes[8] = {1,2,3}, before[8];
    wifi_spi_packets_struct packet = {0};
    uint32 before_errors;
    reset_case();
    module_ready = 0; cancel_at_us = 730;
    before_errors = wifi_spi_io_error_count;
    assert(wifi_spi_wait_idle(50000) == 1);
    assert(now_us >= 730 && now_us <= 740 && max_delay_us <= 1000);
    assert(wifi_spi_io_error_count == before_errors);
    assert_clean();
    cancel_flag = 0; module_ready = 1; wifi_spi_service_begin(0);
    assert(wifi_spi_wait_idle(50000) == 0);

    reset_case(); cancel_at_us = 3000;
    assert(wifi_spi_init(NULL, NULL) == 1); /* 10ms复位低电平期间取消。 */
    assert(now_us == 3000 && max_delay_us <= 1000 && query_calls == 0);
    assert_clean();
    cancel_flag = 0; cancel_at_us = 0; now_us = 0; wifi_spi_service_begin(0);
    assert(wifi_spi_init(NULL, NULL) == 0); /* 取消后重新完整初始化。 */
    assert(now_us == 110000 && query_calls == 2);
    assert(pins[WIFI_SPI_RST_PIN] && pins[WIFI_SPI_CS_PIN]);

    reset_case(); cancel_at_us = 27000;
    assert(wifi_spi_init(NULL, NULL) == 1); /* 100ms启动等待期间取消。 */
    assert(now_us == 27000 && max_delay_us <= 1000 && query_calls == 0);
    assert_clean();

    reset_case(); cancel_flag = 1;
    assert(wifi_spi_write(bytes, 4, bytes, 3) == 1 && exchange_calls == 0);
    assert_clean();
    reset_case(); cancel_exchange = 1;
    assert(wifi_spi_write(bytes, 4, bytes, 3) == 1);
    assert(exchange_calls == 2); /* 已拉低CS后完整发送头与载荷，再取消。 */
    assert_clean();
    reset_case(); cancel_exchange = 1;
    assert(wifi_spi_transfer_command(&packet, 32) == 1 && exchange_calls == 2);
    assert(packet.head.command == WIFI_SPI_INVALID1 && packet.head.length == 0);
    assert_clean();
    reset_case(); fail_exchange = 1;
    assert(wifi_spi_transfer_data(bytes, &packet, 3) == 1);
    assert(exchange_calls == 1 && pins[WIFI_SPI_CS_PIN] == 1);

    reset_case(); cancel_exchange = 1;
    assert(wifi_spi_set_parameter(WIFI_SPI_SET_WIFI_INFORMATION, bytes, 3, 10000) == 1);
    assert(exchange_calls == 2); /* 取消后不再发ACK查询。 */
    assert_clean();
    reset_case(); cancel_flag = 1;
    assert(wifi_spi_get_parameter(WIFI_SPI_GET_VERSION, &packet, 1000) == 1);
    assert(exchange_calls == 0);
    assert_clean();

    reset_case(); cancel_flag = 1; memcpy(before, bytes, sizeof(bytes));
    assert(wifi_spi_read_buffer(bytes, sizeof(bytes)) == 0);
    assert(memcmp(before, bytes, sizeof(bytes)) == 0); /* 旧FIFO半包不交给上层。 */
    assert(wifi_spi_send_buffer(bytes, sizeof(bytes)) == sizeof(bytes));
    assert_clean();

    reset_case(); cancel_exchange = 1; wifi_spi_mutex = WIFI_SPI_IDLE;
    assert(wifi_spi_read_buffer(bytes, sizeof(bytes)) == 0); /* 已领取FIFO后才取消，本轮字节同样作废。 */
    assert_clean();

    reset_case(); pulse_mode = 1; cancel_exchange = 1; wifi_spi_mutex = WIFI_SPI_IDLE;
    assert(wifi_spi_read_buffer(bytes, sizeof(bytes)) == 0); /* 瞬态按键在返回末端之前已清除。 */
    assert(cancel_hook() == 0 && wifi_spi_cancel_requested() == 1);
    assert_clean();
    before_errors = exchange_calls;
    assert(wifi_spi_send_buffer(bytes, sizeof(bytes)) == sizeof(bytes));
    assert(exchange_calls == before_errors); /* 本轮已取消，禁止继续发送。 */
    wifi_spi_service_end();
    assert(wifi_spi_cancel_requested() == 1); /* 上层在end后仍能识别本轮取消。 */
    wifi_spi_service_begin(0);
    cancel_exchange = 0;
    assert(wifi_spi_cancel_requested() == 0);
    assert(wifi_spi_send_buffer(bytes, sizeof(bytes)) == 0); /* 新服务轮次恢复正常事务。 */

    reset_case(); memcpy(wifi_spi_version, "V2.0", 5); cancel_flag = 1;
    assert(wifi_spi_wifi_scan((char *)bytes, sizeof(bytes)) == 1);
    assert(exchange_calls == 0);
    assert_clean();
    wifi_spi_set_cancel_hook(NULL);
    assert(wifi_spi_cancel_requested() == 0);
    puts("PASS actual Wi-Fi cancellation: wait/reset/CS packet/ACK/FIFO/mutex/retry/scan");
    return 0;
}
"""


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--evidence-dir", type=Path,
                        default=ROOT / "docs/evidence/M3/button_start_20261014/wifi_cancel")
    args = parser.parse_args()
    checks.EVIDENCE = args.evidence_dir.resolve()
    source = (ROOT / "drivers/wifi_spi_v2/zf_device_wifi_spi.c").read_bytes().decode("gb18030")
    header = (ROOT / "drivers/wifi_spi_v2/zf_device_wifi_spi.h").read_bytes().decode("gb18030")
    header = "\n".join(line for line in header.splitlines() if not line.startswith("#include"))
    globals_start = source.index("char wifi_spi_version[12];")
    globals_end = source.index("static uint32 wifi_spi_cycles_per_us(", globals_start)
    globals_text = source[globals_start:globals_end]
    names = ["wifi_spi_service_begin", "wifi_spi_service_end", "wifi_spi_enable_clean", "wifi_spi_set_cancel_hook", "wifi_spi_cancel_requested",
             "wifi_spi_cancel_pending", "wifi_spi_delay_cancelable", "wifi_spi_wait_idle",
             "wifi_spi_write", "wifi_spi_transfer_command", "wifi_spi_transfer_data",
             "wifi_spi_set_parameter", "wifi_spi_get_parameter", "wifi_spi_send_buffer",
             "wifi_spi_read_buffer", "wifi_spi_wifi_scan", "wifi_spi_init"]
    generated = checks.EVIDENCE / "test_wifi_cancel.c"
    generated.parent.mkdir(parents=True, exist_ok=True)
    generated.write_text(FIXTURE + header + globals_text + EXCHANGE +
                         "\n\n".join(function(source, name) for name in names) + CASES,
                         encoding="utf-8")
    checks.compile_test("test_wifi_cancel", [generated])


if __name__ == "__main__":
    main()
