"""通过调试探针模拟模块忙，验证有限重试及自动重连；测试前确认 PWM 为零。"""

import json
import time
from datetime import datetime, timedelta, timezone
from pathlib import Path

from elftools.elf.elffile import ELFFile
from pyocd.core.helpers import ConnectHelper


ROOT = Path(__file__).resolve().parents[1]
with (ROOT / "mdk/Objects/rt1064.axf").open("rb") as elf_file:
    elf = ELFFile(elf_file)
    SYMBOLS = {
        symbol.name: (int(symbol["st_value"]), int(symbol["st_size"]))
        for symbol in elf.get_section_by_name(".symtab").iter_symbols()
    }

session = ConnectHelper.session_with_chosen_probe(
    unique_id="80941C44-B8F862DC2544",
    blocking=False,
    options={
        "target_override": "mimxrt1064",
        "connect_mode": "attach",
        "frequency": 1000000,
        "auto_unlock": False,
        "no_config": True,
        "resume_on_disconnect": False,
    },
)
report = {}

with session:
    target = session.target

    def read_value(name):
        address, size = SYMBOLS[name]
        return int.from_bytes(bytes(target.read_memory_block8(address, size)), "little")

    def snapshot():
        values = {name: read_value(name) for name in (
            "wifi_assistant_state", "elapsed_time_ms", "last_send_ms",
            "wifi_assistant_frame_count", "wifi_assistant_send_error_count",
            "wifi_assistant_last_unsent_bytes", "wifi_assistant_reconnect_count",
            "wifi_spi_mutex",
        )}
        values["cpu_state"] = target.get_state().name
        values["pwm_val3"] = [target.read16(address) for address in (
            0x403E0016, 0x403E0076, 0x403E80D6,
        )]
        return values

    def emit(label):
        values = snapshot()
        report[label] = values
        print(label + "=" + json.dumps(values), flush=True)
        return values

    def wait_for(predicate, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return
            time.sleep(0.002)
        raise AssertionError("等待测试条件超时")

    def inject_busy():
        # 仅在模块没有正在进行的 SPI 操作时注入，暂停后始终恢复 CPU。
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            target.halt()
            try:
                if read_value("wifi_assistant_state") == 4 and read_value("wifi_spi_mutex") == 0:
                    address, size = SYMBOLS["wifi_spi_mutex"]
                    target.write_memory_block8(address, (1).to_bytes(size, "little"))
                    return
            finally:
                target.resume()
            time.sleep(0.002)
        raise AssertionError("没有找到可安全注入的空闲状态")

    wait_for(lambda: read_value("wifi_assistant_state") == 4, 15)
    baseline = emit("baseline")
    assert baseline["cpu_state"] == "RUNNING"
    assert baseline["pwm_val3"] == [0, 0, 0]
    try:
        wait_for(lambda: read_value("wifi_assistant_frame_count") >= baseline["wifi_assistant_frame_count"] + 3, 5)
        stable = emit("sending_before_injection")
        assert stable["wifi_assistant_state"] == 4

        inject_busy()
        wait_for(lambda: read_value("wifi_assistant_state") == 5, 3)
        stopped = emit("persistent_failure")
        assert stopped["wifi_assistant_send_error_count"] >= stable["wifi_assistant_send_error_count"] + 3
        # 保留忙状态，由程序自行复位模块，禁止测试脚本代替程序解除故障。
        wait_for(lambda: read_value("wifi_assistant_state") == 4 and
                 read_value("wifi_assistant_frame_count") >= stopped["wifi_assistant_frame_count"] + 5, 20)
        recovered = emit("auto_reconnected")
        assert recovered["wifi_assistant_reconnect_count"] > stopped["wifi_assistant_reconnect_count"]
        assert recovered["pwm_val3"] == [0, 0, 0]
        report["success"] = True
    except Exception as error:
        report["success"] = False
        report["error"] = str(error)
        # 测试异常时撤销注入状态，避免留下人工制造的模块忙标记。
        address, size = SYMBOLS["wifi_spi_mutex"]
        target.write_memory_block8(address, bytes(size))
        raise
    finally:
        if target.get_state().name == "HALTED":
            target.resume()
        stamp = datetime.now(timezone(timedelta(hours=8))).strftime("%Y%m%d_%H%M%S")
        result = ROOT / "mdk" / ("wifi_recovery_test_" + stamp + ".json")
        result.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
        print("REPORT=" + str(result), flush=True)
