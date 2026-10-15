"""从实际源码定义更新功能代码索引及构建证据，拒绝缺失入口。"""

from pathlib import Path
import ast
import argparse
import hashlib
import json
import os
import re
from urllib.parse import quote, unquote

ROOT = Path(__file__).resolve().parents[1]
REPORT = ROOT / "docs/IMPLEMENTATION_REPORT.md"
ENTRIES = [
    ("M3真实时基", "user/src/calibration_clock.c", ["calibration_clock_init", "calibration_clock_now_us"]),
    ("M3编码器物理量", "user/src/encoder_measurement.c", ["encoder_measurement_sample", "encoder_measurement_get_snapshot", "encoder_measurement_set_counts_per_rev", "encoder_measurement_set_wheel_circumference", "encoder_measurement_counts_to_mps", "encoder_measurement_mps_to_counts"]),
    ("M3实测转角表", "user/src/steering_map.c", ["steering_map_validate", "steering_map_us_to_angles", "steering_map_angle_to_us", "steering_map_apply", "steering_wheel_map_validate", "steering_wheel_map_us_to_angle", "steering_wheel_map_angle_to_us"]),
    ("M3内存标定", "user/src/calibration_debug.c", ["calibration_debug_init", "calibration_debug_poll", "calibration_debug_before_tick", "calibration_debug_after_tick"]),
    ("M3输出仲裁", "user/src/control.c", ["control_request_servo_cal", "control_request_calibration_bench", "control_apply_calibration_params", "control_stop_generation"]),
    ("M3电脑工具", "tools/calibrate_pyocd.py", ["main", "attach_factory", "validate_encoder_measurements", "validate_measurements", "generate_profile", "rpm_reference"]),
    ("本地按键", "user/src/local_buttons.c", ["local_buttons_init", "local_buttons_scan"]),
    ("本地事件桥", "user/src/main.c", ["local_drive_scan", "local_drive_poll_start", "local_drive_network_cancel"]),
    ("可取消联网", "drivers/wifi_spi_v2/zf_device_wifi_spi.c", ["wifi_spi_set_cancel_hook", "wifi_spi_cancel_requested"]),
    ("系统初始化", "user/src/main.c", ["Init", "main"]),
    ("硬件输出", "user/src/isr.c", ["PIT_IRQHandler", "CSI_IRQHandler"]),
    ("参数与单位", "user/src/control.c", ["params_valid", "control_apply_params"]),
    ("运行状态", "user/src/control.c", ["control_init", "control_request_local_track_run", "control_request_run", "control_request_stop", "control_reset_fault"]),
    ("故障与快照", "user/src/control.c", ["latch_fault", "control_publish_vision", "control_tick", "control_get_snapshot"]),
    ("相机快照", "user/src/camera_frame.c", ["frame_callback", "camera_frame_install", "camera_frame_take", "camera_frame_dropped"]),
    ("灰度处理", "user/src/img_process.c", ["get_hist_gram", "get_threshold_otsu", "binaryzation_process"]),
    ("边线与误差", "user/src/img_process.c", ["find_white_run", "vision_process"]),
    ("视觉配置", "user/src/img_process.c", ["vision_reset", "vision_set_reference_width", "vision_set_control_band"]),
    ("普通道路鸟瞰恢复", "user/src/vision_track.c", ["vision_track_init", "vision_track_reset", "vision_track_process", "thin_road", "find_path", "project_path"]),
    ("首弯道路约束", "user/src/vision_track.c", ["vision_track_histogram", "guard_road", "summarize_guard"]),
    ("鸟瞰法向单边", "user/src/vision_track.c", ["distort_point", "image_ground_point", "ground_image_point", "edge_tangent", "normal_intersections", "recover_normal"]),
    ("停车视觉取证", "user/src/vision_record.c", ["vision_record_observe", "vision_record_capture", "vision_record_frame", "vision_record_replay", "vision_record_display_lines"]),
    ("冻结图传点表", "user/src/wifi_assistant.c", ["assistant_pack_boundaries", "assistant_replay_label"]),
    ("停车记录导出", "tools/export_vision_stop.py", ["decode", "read_stable", "read_probe", "save", "main"]),
    ("视觉步骤耗时", "user/src/vision_track.c", ["vision_timing_begin", "vision_timing_set", "vision_timing_finish"]),
    ("人工宽度只读", "user/src/img_process.c", ["vision_get_reference_width"]),
    ("离线鸟瞰映射", "tools/generate_vision_map.py", ["generate", "geometry", "render_header", "main"]),
    ("耗时离线解码", "tools/decode_vision_timing.py", ["decode", "main"]),
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


def select_calibration_branch(code):
    """索引明确选择诊断实现，排除CALIBRATION_DEBUG_ENABLE互斥的关闭版桩；保留原行数。"""
    depth = 0
    selected = True
    result = []
    for line in code.splitlines(keepends=True):
        stripped = line.strip()
        if depth == 0 and stripped == "#if CALIBRATION_DEBUG_ENABLE":
            depth = 1
            selected = True
        elif depth:
            if re.match(r"#\s*(if|ifdef|ifndef)\b", stripped):
                depth += 1
            elif re.match(r"#\s*endif\b", stripped):
                depth -= 1
                if depth == 0:
                    selected = True
            elif depth == 1 and re.match(r"#\s*else\b", stripped):
                selected = False
        result.append(line if selected else re.sub(r"[^\r\n]", " ", line))
    return "".join(result)


def definition_line(source, symbol, relative):
    """只计入带函数体的源码定义；声明、注释和字符串中的示例不作入口。"""
    if Path(relative).suffix == ".py":
        definitions = [node for node in ast.walk(ast.parse(source))
                       if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)) and node.name == symbol]
        if len(definitions) != 1:
            raise ValueError("Python真实定义数量异常: " + relative + " " + symbol)
        return definitions[0].lineno
    ignored = r'/\*[\s\S]*?\*/|//[^\r\n]*|"(?:\\[\s\S]|[^"\\])*"|\'(?:\\[\s\S]|[^\'\\])*\''
    code = re.sub(ignored, lambda match: re.sub(r"[^\r\n]", " ", match.group()), source)
    if Path(relative).name == "calibration_debug.c":
        code = select_calibration_branch(code)
    pattern = (r"^(?:static\s+)?(?:void|int|float|unsigned char|uint8_t|uint16_t|uint32_t|int16_t|"
               r"uint8|uint16|uint32|int16|int32_t|uint64_t|ControlState|status_t|fifo_state_enum)\s+"
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


FUNCTION_LINK = re.compile(
    r"\[([A-Za-z_][A-Za-z_0-9]*)（第 (\d+) 行）\]\(([^)]+)#L\d+\)")
SOURCE_REFERENCE = re.compile(r"\{\{src\|([^|{}]+)\|([A-Za-z_][A-Za-z_0-9]*)\}\}")


def refresh_chapter_links(text, root=ROOT, report=REPORT):
    """将草稿标记转成实际定义链接，并更新章内已有函数链接的行号。"""
    def replace_reference(match):
        relative, symbol = match.groups()
        path = (root / relative).resolve()
        line = definition_line(read_text(path), symbol, relative)
        return source_link(path, report, symbol, line)

    text = SOURCE_REFERENCE.sub(replace_reference, text)
    checks = []

    def replace_link(match):
        symbol, _, target = match.groups()
        path = (report.parent / unquote(target)).resolve()
        relative = Path(os.path.relpath(path, root)).as_posix()
        line = definition_line(read_text(path), symbol, relative)
        checks.append({"path": relative, "symbol": symbol, "line": line})
        return source_link(path, report, symbol, line)

    return FUNCTION_LINK.sub(replace_link, text), checks


def chapter_index_entries(checks):
    """全局查找表保留原入口，并收录章节解释涉及的额外真实函数。"""
    entries = [(category, relative, list(symbols)) for category, relative, symbols in ENTRIES]
    known = {(relative, symbol) for _, relative, symbols in entries for symbol in symbols}
    supplemental = {}
    for item in checks:
        key = (item["path"], item["symbol"])
        if key not in known:
            supplemental.setdefault(item["path"], []).append(item["symbol"])
            known.add(key)
    entries.extend(("章节补充", relative, symbols) for relative, symbols in supplemental.items())
    return entries


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--evidence-dir", type=Path, default=ROOT / "docs/evidence/current_index")
    parser.add_argument("--build-dir", type=Path,
                        help="显式指定本轮冻结构建目录；不指定时仅刷新索引，不改历史构建摘要")
    args = parser.parse_args()
    args.evidence_dir.mkdir(parents=True, exist_ok=True)
    text = REPORT.read_text(encoding="utf-8")
    # 先排除旧全局表，防止已从正文移除的补充函数因旧表而永久保留。
    text = replace_region(text, "CODE_INDEX", "")
    text, chapter_checks = refresh_chapter_links(text)
    index, checks = build_code_index(entries=chapter_index_entries(chapter_checks))
    text = replace_region(text, "CODE_INDEX", index)
    log = args.build_dir / "keil_rebuild.log" if args.build_dir is not None else None
    if log is not None:
        if not log.exists():
            raise ValueError("指定构建目录缺少冻结日志: " + str(log))
        data = log.read_bytes()
        build = data.decode("utf-16" if data.startswith((b"\xff\xfe", b"\xfe\xff")) else "gb18030", errors="replace")
        size = re.search(r"Program Size: Code=(\d+) RO-data=(\d+) RW-data=(\d+) ZI-data=(\d+)", build)
        if "0 Error(s), 0 Warning(s)" not in build or size is None:
            raise ValueError("构建日志没有通过或缺资源摘要")
        axf = args.build_dir / "firmware.axf"
        digest = hashlib.sha256(axf.read_bytes()).hexdigest()
        build_path = report_relative_path(args.build_dir, REPORT)
        source_path = report_relative_path(args.evidence_dir, REPORT)
        summary = "ARM Compiler 6.15 完整重构建：**0错误、0警告**。代码 {}字节，RO {}字节，RW {}字节，ZI {}字节。\n\n固件AXF SHA256：`{}`。构建日志及本轮固件副本位于 `{}`；本轮源清单位于 `{}`。M2及M3标定历史证据保留原位。".format(*size.groups(), digest, build_path, source_path)
        text = replace_region(text, "BUILD_SUMMARY", summary)
    REPORT.write_text(text, encoding="utf-8", newline="\n")
    output = args.evidence_dir / "code_index.json"
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(checks, ensure_ascii=False, indent=2), encoding="utf-8")
    chapters = args.evidence_dir / "chapter_code_index.json"
    chapters.write_text(json.dumps(chapter_checks, ensure_ascii=False, indent=2), encoding="utf-8")
    print("Verified", len(checks), "unique function definitions and", len(chapter_checks), "chapter links")


if __name__ == "__main__":
    main()
