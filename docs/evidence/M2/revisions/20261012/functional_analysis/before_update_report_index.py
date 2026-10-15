"""从实际源码定义更新功能代码索引及构建证据，拒绝缺失入口。"""

from pathlib import Path
import hashlib
import json
import os
import re
from urllib.parse import quote

ROOT = Path(__file__).resolve().parents[1]
REPORT = ROOT / "docs/IMPLEMENTATION_REPORT.md"
ENTRIES = [
    ("系统初始化", "user/src/main.c", ["Init", "main"]),
    ("硬件输出", "user/src/isr.c", ["PIT_IRQHandler", "CSI_IRQHandler"]),
    ("参数与单位", "user/src/control.c", ["params_valid", "control_apply_params"]),
    ("运行状态", "user/src/control.c", ["control_init", "control_request_run", "control_request_stop", "control_reset_fault"]),
    ("故障与快照", "user/src/control.c", ["latch_fault", "control_publish_vision", "control_tick", "control_get_snapshot"]),
    ("相机快照", "user/src/camera_frame.c", ["frame_callback", "camera_frame_install", "camera_frame_take", "camera_frame_dropped"]),
    ("灰度处理", "user/src/img_process.c", ["get_hist_gram", "get_threshold_otsu", "binaryzation_process"]),
    ("边线与误差", "user/src/img_process.c", ["find_white_run", "vision_process"]),
    ("视觉配置", "user/src/img_process.c", ["vision_reset", "vision_set_reference_width", "vision_set_control_band"]),
    ("速度PI", "user/src/pid.c", ["Pid_Param_Init", "Pid_Reset", "PidLocCtrl"]),
    ("人工标定", "user/src/main.c", ["calibrate_width"]),
    ("人工参数", "user/src/wifi_assistant.c", ["assistant_parameter_event", "apply_parameters", "wifi_assistant_take_width_calibration_request"]),
    ("通信调度", "user/src/wifi_assistant.c", ["assistant_send", "wifi_assistant_init", "wifi_assistant_poll", "wifi_assistant_on_frame"]),
    ("SPI截止时间", "drivers/wifi_spi_v2/zf_device_wifi_spi.c", ["wifi_spi_deadline_expired", "wifi_spi_exchange_checked", "wifi_spi_service_begin", "wifi_spi_service_end"]),
    ("共享V1解析", "../libraries/zf_components/seekfree_assistant.c", ["seekfree_assistant_data_analysis", "seekfree_assistant_receive_reset"]),
]


def read_text(path):
    data = path.read_bytes()
    try:
        return data.decode("utf-8-sig")
    except UnicodeDecodeError:
        return data.decode("gb18030")


def replace_region(text, name, content):
    start = "<!-- " + name + "_START -->"
    end = "<!-- " + name + "_END -->"
    first = text.index(start) + len(start)
    last = text.index(end, first)
    return text[:first] + "\n" + content + "\n" + text[last:]


def definition_line(source, symbol, relative):
    """只计入带函数体的源码定义；声明、注释和字符串中的示例不作入口。"""
    ignored = r'/\*[\s\S]*?\*/|//[^\r\n]*|"(?:\\[\s\S]|[^"\\])*"|\'(?:\\[\s\S]|[^\'\\])*\''
    code = re.sub(ignored, lambda match: re.sub(r"[^\r\n]", " ", match.group()), source)
    pattern = (r"^(?:static\s+)?(?:void|int|float|unsigned char|uint8_t|uint32_t|uint8|uint32|ControlState)\s+"
               + re.escape(symbol) + r"\s*\([^;{}]*\)\s*\{")
    matches = list(re.finditer(pattern, code, re.M))
    if len(matches) != 1:
        raise ValueError("真实定义数量异常: " + relative + " " + symbol + " " + str(len(matches)))
    return code[:matches[0].start()].count("\n") + 1


def report_relative_path(path, report):
    """从报告所在目录计算源码路径，避免盘符被Markdown解释为URL协议。"""
    return Path(os.path.relpath(path.resolve(), report.resolve().parent)).as_posix()


def source_link(path, report, symbol, line):
    """编码路径中的空格、中文和特殊字符；行号同时保留为可见文字。"""
    if line < 1:
        raise ValueError("源码行号必须从1开始")
    target = quote(report_relative_path(path, report), safe="/.") + "#L" + str(line)
    return "[" + symbol + "（第 " + str(line) + " 行）](" + target + ")"


def build_code_index(root=ROOT, report=REPORT, entries=ENTRIES):
    """只读生成代码索引和定义清单，便于复核而不改报告或构建证据。"""
    rows = ["| 功能 | 源码路径（相对本报告） | 关键入口及行号 |", "| --- | --- | --- |"]
    checks = []
    for category, relative, symbols in entries:
        path = (root / relative).resolve()
        source = read_text(path)
        links = []
        for symbol in symbols:
            line = definition_line(source, symbol, relative)
            links.append(source_link(path, report, symbol, line))
            checks.append({"path": relative, "symbol": symbol, "line": line})
        visible_path = report_relative_path(path, report).replace("|", "\\|")
        rows.append("| " + category + " | " + visible_path + " | " + "、".join(links) + " |")
    return "\n".join(rows), checks


def main():
    index, checks = build_code_index()
    text = REPORT.read_text(encoding="utf-8")
    text = replace_region(text, "CODE_INDEX", index)
    log = ROOT / "docs/evidence/M0/keil_rebuild.log"
    if log.exists():
        data = log.read_bytes()
        build = data.decode("utf-16" if data.startswith((b"\xff\xfe", b"\xfe\xff")) else "gb18030", errors="replace")
        size = re.search(r"Program Size: Code=(\d+) RO-data=(\d+) RW-data=(\d+) ZI-data=(\d+)", build)
        if "0 Error(s), 0 Warning(s)" not in build or size is None:
            raise ValueError("构建日志没有通过或缺资源摘要")
        axf = ROOT / "mdk/Objects/rt1064.axf"
        digest = hashlib.sha256(axf.read_bytes()).hexdigest()
        summary = "ARM Compiler 6.15 完整重构建：**0错误、0警告**。代码 {}字节，RO {}字节，RW {}字节，ZI {}字节。\n\n固件AXF SHA256：`{}`。构建日志位于 `evidence/M0/keil_rebuild.log`；副本与最终源清单位于 `evidence/M2`。".format(*size.groups(), digest)
        text = replace_region(text, "BUILD_SUMMARY", summary)
    REPORT.write_text(text, encoding="utf-8", newline="\n")
    output = ROOT / "docs/evidence/M2/code_index.json"
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(checks, ensure_ascii=False, indent=2), encoding="utf-8")
    print("Verified", len(checks), "real function definitions and report links")


if __name__ == "__main__":
    main()
