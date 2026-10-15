"""编译并回放真实 C 视觉算法；合成样本只用于软件验证，不证明实车通过。"""

from __future__ import annotations

import argparse
import csv
import ctypes
import hashlib
import json
import math
import re
import shutil
import statistics
import subprocess
import sys
import tempfile
from contextlib import ExitStack
from pathlib import Path

WIDTH, HEIGHT = 188, 120
DEFAULT_ROWS = [80, 85, 90, 95, 100]
EXPORTED_APIS = [
    "get_hist_gram", "get_threshold_otsu", "binaryzation_process",
    "vision_process", "vision_set_reference_width",
    "vision_set_control_band", "vision_reset",
]
SHADOW_EXPECTATIONS = {
    "shadow_readable_synthetic": {
        "valid": 1, "threshold": 45, "error_px": 0.0, "confidence": 1.0,
        "valid_rows": 120, "reconstructed_rows": 0,
        "control_row_quality": [2, 2, 2, 2, 2],
        "description": "第72至105行局部阴影：背景25、白赛道140；其余背景45、白赛道205。默认控制带预期仍可辨认。",
    },
    "shadow_deep_synthetic": {
        "valid": 0, "threshold": 45, "error_px": 0.0, "confidence": 0.0,
        "valid_rows": 86, "reconstructed_rows": 0,
        "control_row_quality": [0, 0, 0, 0, 0],
        "description": "第72至105行深阴影：背景15、白赛道35。全局Otsu预期丢失控制带；无效结果的零误差不能解释成居中。",
    },
}


class VisionResult(ctypes.Structure):
    _fields_ = [
        ("frame_id", ctypes.c_uint32),
        ("capture_time_ms", ctypes.c_uint32),
        ("error_px", ctypes.c_float),
        ("confidence", ctypes.c_float),
        ("threshold", ctypes.c_uint8),
        ("valid", ctypes.c_uint8),
        ("track_lost", ctypes.c_uint8),
        ("left", ctypes.c_uint8 * HEIGHT),
        ("middle", ctypes.c_uint8 * HEIGHT),
        ("right", ctypes.c_uint8 * HEIGHT),
        ("row_quality", ctypes.c_uint8 * HEIGHT),
        ("valid_rows", ctypes.c_uint16),
        ("reconstructed_rows", ctypes.c_uint16),
    ]


def msvc_command(directory: Path, demo: Path, compiler: str | None):
    tools_root = Path("C:/Program Files/Microsoft Visual Studio/2022/Community/VC/Tools/MSVC")
    kits_root = Path("C:/Program Files (x86)/Windows Kits/10/Include")
    def version_key(path):
        return tuple(int(part) for part in path.name.split(".") if part.isdigit())
    tool_versions = sorted(tools_root.glob("*"), key=version_key)
    kit_versions = sorted(kits_root.glob("*"), key=version_key)
    if not tool_versions or not kit_versions:
        raise ValueError("未找到 VS2022 Community 和 Windows Kits，无法使用 MSVC 回退")
    tools, kit = tool_versions[-1], kit_versions[-1]
    executable = compiler if compiler and compiler.lower() != "msvc" else str(tools / "bin/Hostx64/x64/cl.exe")
    output = directory / "vision.dll"
    # 对象、导入库和链接产物全部留在临时目录；明确导出 ctypes 使用的真实 C 接口。
    return [
        executable, "/nologo", "/LD", "/MD", "/std:c11", "/utf-8", "/O2", "/W4", "/WX",
        "/I" + str(demo / "user/inc"), "/I" + str(tools / "include"),
        "/I" + str(kit / "ucrt"), "/I" + str(kit / "shared"), "/I" + str(kit / "um"),
        "/Fo" + str(directory / "vision.obj"), "/Fe" + str(output),
        str(demo / "user/src/img_process.c"), "/link",
        "/OUT:" + str(output), "/IMPLIB:" + str(directory / "vision.lib"),
        "/PDB:" + str(directory / "vision.pdb"),
        "/LIBPATH:" + str(tools / "lib/x64"),
        "/LIBPATH:" + str(kit.parents[1] / "Lib" / kit.name / "ucrt/x64"),
        "/LIBPATH:" + str(kit.parents[1] / "Lib" / kit.name / "um/x64"),
        *("/EXPORT:" + name for name in EXPORTED_APIS),
    ]


def build_library(directory: Path, compiler: str | None):
    demo = Path(__file__).resolve().parents[1]
    output = directory / ("vision.dll" if sys.platform == "win32" else "vision.so")
    selected = compiler or shutil.which("gcc") or "gcc"
    is_msvc = selected.lower() == "msvc" or Path(selected).name.lower() in ("cl", "cl.exe")
    fallback_reason = None
    if is_msvc:
        if sys.platform != "win32":
            raise ValueError("MSVC 编译器选项只适用于 Windows")
        command = msvc_command(directory, demo, selected)
        subprocess.run(command, cwd=directory, check=True, capture_output=True, text=True, errors="replace")
        library = ctypes.CDLL(str(output))
        compiler_kind = "msvc"
    else:
        command = [
            selected, "-std=c99", "-O2", "-Wall", "-Wextra", "-Werror", "-shared",
            "-I", str(demo / "user" / "inc"), str(demo / "user" / "src" / "img_process.c"),
            "-o", str(output), "-lm",
        ]
        command.append("-static-libgcc" if sys.platform == "win32" else "-fPIC")
        try:
            subprocess.run(command, cwd=directory, check=True, capture_output=True, text=True, errors="replace")
            library = ctypes.CDLL(str(output))
            compiler_kind = "gcc"
        except (OSError, subprocess.CalledProcessError) as error:
            # 显式指定编译器时尊重用户选择，缺省才回退本机已安装的 MSVC。
            if compiler is not None or sys.platform != "win32":
                raise
            fallback_reason = str(error)
            command = msvc_command(directory, demo, None)
            subprocess.run(command, cwd=directory, check=True, capture_output=True, text=True, errors="replace")
            library = ctypes.CDLL(str(output))
            compiler_kind = "msvc"
    library._vision_build_metadata = {
        "compiler": compiler_kind, "compiler_path": command[0],
        "fallback_reason": fallback_reason, "exports": EXPORTED_APIS,
        "source": str(demo / "user/src/img_process.c"),
        "source_sha256": hashlib.sha256((demo / "user/src/img_process.c").read_bytes()).hexdigest(),
    }
    library.get_hist_gram.argtypes = [
        ctypes.POINTER(ctypes.c_uint8), ctypes.c_ushort, ctypes.c_ushort,
        ctypes.POINTER(ctypes.c_uint32),
    ]
    library.get_hist_gram.restype = None
    library.get_threshold_otsu.argtypes = [ctypes.POINTER(ctypes.c_uint32)]
    library.get_threshold_otsu.restype = ctypes.c_uint8
    library.binaryzation_process.argtypes = [
        ctypes.POINTER(ctypes.c_uint8), ctypes.c_ushort, ctypes.c_ushort, ctypes.c_uint,
    ]
    library.binaryzation_process.restype = None
    library.vision_process.argtypes = [
        ctypes.POINTER(ctypes.c_uint8), ctypes.c_uint32, ctypes.c_uint32,
        ctypes.POINTER(VisionResult),
    ]
    library.vision_process.restype = ctypes.c_uint8
    library.vision_set_reference_width.argtypes = [ctypes.c_uint16, ctypes.c_uint16]
    library.vision_set_reference_width.restype = ctypes.c_uint8
    library.vision_set_control_band.argtypes = [
        ctypes.POINTER(ctypes.c_uint8), ctypes.POINTER(ctypes.c_float),
    ]
    library.vision_set_control_band.restype = ctypes.c_uint8
    library.vision_reset.argtypes = []
    library.vision_reset.restype = None
    return library


def pgm_token(data: bytes, position: int):
    while position < len(data):
        if data[position] in b" \t\r\n":
            position += 1
        elif data[position] == ord("#"):
            newline = data.find(b"\n", position)
            position = len(data) if newline < 0 else newline + 1
        else:
            break
    start = position
    while position < len(data) and data[position] not in b" \t\r\n#":
        position += 1
    if start == position:
        raise ValueError("PGM 文件头不完整")
    return data[start:position], position


def read_frame(path: Path) -> bytes:
    data = path.read_bytes()
    if path.suffix.lower() in (".raw", ".bin"):
        pixels = data
    elif path.suffix.lower() == ".pgm":
        position = 0
        tokens = []
        for _ in range(4):
            token, position = pgm_token(data, position)
            tokens.append(token)
        magic, width, height, maximum = tokens
        if (int(width), int(height), int(maximum)) != (WIDTH, HEIGHT, 255):
            raise ValueError(f"{path.name}: PGM 必须为 188×120、最大灰度 255")
        if magic == b"P5":
            # 只消耗头部的分隔符，避免把首像素的空白字节错误跳过。
            if position >= len(data) or data[position] not in b" \t\r\n":
                raise ValueError("PGM 二进制头缺少分隔符")
            position += 2 if data[position:position + 2] == b"\r\n" else 1
            pixels = data[position:]
        elif magic == b"P2":
            values = re.sub(rb"#[^\n]*", b"", data[position:]).split()
            pixels = bytes(int(value) for value in values)
        else:
            raise ValueError("只支持 P5/P2 PGM")
    else:
        try:
            from PIL import Image
        except ImportError as error:
            raise ValueError("PNG/JPEG 需要 Pillow；也可使用无依赖的 RAW/PGM") from error
        with Image.open(path) as image:
            if image.size != (WIDTH, HEIGHT):
                raise ValueError(f"{path.name}: 必须为 188×120，工具不会自动缩放")
            pixels = image.convert("L").tobytes()
    if len(pixels) != WIDTH * HEIGHT:
        raise ValueError(f"{path.name}: 像素字节数 {len(pixels)}，应为 {WIDTH * HEIGHT}")
    return pixels


def synthetic_frames():
    def lane(offset):
        image = bytearray([45] * (WIDTH * HEIGHT))
        for y in range(HEIGHT):
            center = 94 + offset(y)
            for x in range(max(0, center - 30), min(WIDTH, center + 31)):
                image[y * WIDTH + x] = 205
        return bytes(image)

    yield "straight_synthetic", lane(lambda y: 0)
    yield "left_bend_synthetic", lane(lambda y: -(HEIGHT - 1 - y) // 4)
    yield "right_bend_synthetic", lane(lambda y: (HEIGHT - 1 - y) // 4)
    yield "s_bend_synthetic", lane(lambda y: int(15 * math.sin((HEIGHT - 1 - y) / 20)))
    # 灰度夹具交给同一 C Otsu 处理，不预先制作二值结果或伪造视觉输出。
    for name, background, road in [
        ("shadow_readable_synthetic", 25, 140),
        ("shadow_deep_synthetic", 15, 35),
    ]:
        shaded = bytearray(lane(lambda y: 0))
        for y in range(72, 106):
            for x in range(WIDTH):
                shaded[y * WIDTH + x] = road if 64 <= x <= 124 else background
        yield name, bytes(shaded)
    yield "all_black_synthetic", bytes(WIDTH * HEIGHT)
    yield "all_white_synthetic", bytes([255] * (WIDTH * HEIGHT))
    yield "uniform_gray_synthetic", bytes([127] * (WIDTH * HEIGHT))
    one_edge = bytearray(WIDTH * HEIGHT)
    for y in range(HEIGHT):
        one_edge[y * WIDTH:y * WIDTH + 101] = bytes([205] * 101)
    for index in range(1, 6):
        yield f"one_edge_{index}_synthetic", bytes(one_edge)


def configure(library, args):
    args._applied_control_rows = list(DEFAULT_ROWS)
    if args.reference_width:
        profile = json.loads(args.reference_width.read_text(encoding="utf-8-sig"))
        widths = profile.get("reference_width") if isinstance(profile, dict) else profile
        if not isinstance(widths, list) or len(widths) != HEIGHT:
            raise ValueError("参考宽度文件必须包含 120 项 reference_width 数组")
        for row, width in enumerate(widths):
            if isinstance(width, bool) or not isinstance(width, int) or not 0 <= width <= 187:
                raise ValueError(f"第 {row} 行参考宽度不是有效整数")
            if not library.vision_set_reference_width(row, width):
                raise ValueError(f"第 {row} 行参考宽度非法：{width}")
    if args.control_band:
        parameters = json.loads(args.control_band.read_text(encoding="utf-8-sig"))
        rows, weights = parameters["look_rows"], parameters["look_weights"]
        if len(rows) != 5 or len(weights) != 5:
            raise ValueError("控制带必须有 5 行和 5 个权重")
        if any(isinstance(row, bool) or not isinstance(row, int) or not 0 <= row < HEIGHT for row in rows):
            raise ValueError("控制行必须为 0～119 的整数")
        if any(isinstance(weight, bool) or not isinstance(weight, (int, float)) or
               not math.isfinite(weight) or weight <= 0 for weight in weights):
            raise ValueError("控制权重必须为有限正数")
        if not library.vision_set_control_band(
            (ctypes.c_uint8 * 5)(*rows), (ctypes.c_float * 5)(*weights)
        ):
            raise ValueError("控制带必须严格递增，权重必须为有限正数")
        args._applied_control_rows = rows
    library.vision_reset()


def check_shadow_case(name, pixels, result, args):
    expected = SHADOW_EXPECTATIONS[name]
    actual = {
        "valid": int(result.valid), "threshold": int(result.threshold),
        "error_px": float(result.error_px), "confidence": float(result.confidence),
        "valid_rows": int(result.valid_rows), "reconstructed_rows": int(result.reconstructed_rows),
        "control_row_quality": [int(result.row_quality[row]) for row in args._applied_control_rows],
    }
    # 固定预期属于缺省控制带和未标定配置，自定义配置只记录实际结果。
    checked = args.control_band is None and args.reference_width is None
    passed = all(
        math.isclose(actual[key], expected[key], rel_tol=0.0, abs_tol=0.0001)
        if key in ("error_px", "confidence") else actual[key] == expected[key]
        for key in actual
    ) if checked else None
    evidence = {
        "name": name, "evidence_type": "synthetic",
        "input_gray_levels": sorted(set(pixels)),
        "input_sha256": hashlib.sha256(pixels).hexdigest(),
        "control_rows": args._applied_control_rows, "actual": actual,
        "expected_default_configuration": expected,
        "expectation_checked": checked, "expectation_passed": passed,
        "hardware_acceptance": False,
    }
    if args.save_shadow_dir:
        stem = args.save_shadow_dir / name
        raw_path = stem.with_suffix(".raw.pgm")
        binary_path = stem.with_suffix(".binary.pgm")
        raw_path.write_bytes(b"P5\n188 120\n255\n" + pixels)
        binary_path.write_bytes(b"P5\n188 120\n255\n" + args._current_binary_pixels)
        evidence["raw_image"] = str(raw_path.resolve())
        evidence["binary_image"] = str(binary_path.resolve())
        stem.with_suffix(".json").write_text(json.dumps(evidence, ensure_ascii=False, indent=2), encoding="utf-8")
    return evidence


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--input", type=Path, help="188×120 原始图像文件或图像文件夹")
    source.add_argument("--synthetic", action="store_true", help="使用合成场景，不能替代实机验收")
    parser.add_argument("--compiler", help="显式指定 GCC 或 cl.exe；缺省自动使用 GCC/MSVC")
    parser.add_argument("--output", type=Path, help="CSV 输出；默认写入标准输出")
    parser.add_argument("--summary-output", type=Path, help="保存 JSON 软件回放摘要及编译器证据")
    parser.add_argument("--period-ms", type=int, default=20, help="回放时间轴间隔，默认 20ms")
    parser.add_argument("--reference-width", type=Path, help="人工标定参考宽度 JSON")
    parser.add_argument("--control-band", type=Path, help="含 look_rows/look_weights 的参数 JSON")
    parser.add_argument("--save-binary-dir", type=Path, help="保存二值 PGM 和逐行结果 JSON")
    parser.add_argument("--save-shadow-dir", type=Path, help="仅保存两种合成阴影的原灰度、C二值图及预期检查")
    parser.add_argument("--export-width-profile", type=Path, help="从人工指定的居中直道样本导出宽度")
    parser.add_argument("--confirmed-centered-straight", action="store_true", help="操作者确认输入是实际居中直道")
    args = parser.parse_args()
    if not 1 <= args.period_ms <= 1000:
        parser.error("回放间隔必须为 1～1000ms")
    if args.export_width_profile and (args.synthetic or not args.confirmed_centered_straight):
        parser.error("宽度标定必须指定实际图像，并由操作者确认居中直道")
    if args.synthetic:
        frames = synthetic_frames()
        evidence_type = "synthetic"
    else:
        if args.input.is_dir():
            suffixes = {".raw", ".bin", ".pgm", ".png", ".jpg", ".jpeg"}
            def key(path):
                return [int(part) if part.isdigit() else part for part in re.split(r"(\d+)", path.name)]
            paths = sorted((path for path in args.input.iterdir() if path.suffix.lower() in suffixes), key=key)
        else:
            paths = [args.input]
        if not paths:
            raise ValueError("输入目录没有受支持的图像")
        frames = ((path.name, read_frame(path)) for path in paths)
        evidence_type = "file_replay"
    if args.save_binary_dir:
        args.save_binary_dir.mkdir(parents=True, exist_ok=True)
    if args.save_shadow_dir:
        args.save_shadow_dir.mkdir(parents=True, exist_ok=True)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
    output = args.output.open("w", encoding="utf-8", newline="") if args.output else sys.stdout
    widths_by_row = [[] for _ in range(HEIGHT)]
    fields = ["name", "evidence_type", "frame_id", "replay_time_ms", "threshold",
              "valid", "track_lost", "error_px", "confidence", "valid_rows", "reconstructed_rows",
              "control_row_quality", "expected_valid", "expectation_passed"]
    count = valid_count = 0
    shadow_checks = []
    try:
        temporary_root = Path(__file__).resolve().parent
        with tempfile.TemporaryDirectory(prefix="smartcar_vision_", dir=temporary_root) as temporary, ExitStack() as cleanup:
            # 清理只允许指向工具目录内本次创建的临时目录。
            if Path(temporary).resolve().parent != temporary_root:
                raise ValueError("临时目录超出工作区工具目录")
            library = build_library(Path(temporary), args.compiler)
            build_metadata = library._vision_build_metadata
            if sys.platform == "win32":
                # Windows 加载中的 DLL 不能删除，先卸载再清理临时目录。
                from _ctypes import FreeLibrary
                cleanup.callback(FreeLibrary, library._handle)
            configure(library, args)
            writer = csv.DictWriter(output, fieldnames=fields, lineterminator="\n")
            writer.writeheader()
            for count, (name, pixels) in enumerate(frames, start=1):
                image = (ctypes.c_uint8 * (WIDTH * HEIGHT)).from_buffer_copy(pixels)
                histogram = (ctypes.c_uint32 * 256)()
                library.get_hist_gram(image, HEIGHT, WIDTH, histogram)
                threshold = library.get_threshold_otsu(histogram)
                library.binaryzation_process(image, HEIGHT, WIDTH, threshold)
                result = VisionResult()
                library.vision_process(image, count, count * args.period_ms, ctypes.byref(result))
                result.threshold = threshold
                valid_count += result.valid
                shadow_check = None
                if args.synthetic and name in SHADOW_EXPECTATIONS:
                    args._current_binary_pixels = bytes(image)
                    shadow_check = check_shadow_case(name, pixels, result, args)
                    shadow_checks.append(shadow_check)
                writer.writerow({
                    "name": name, "evidence_type": evidence_type,
                    "frame_id": result.frame_id, "replay_time_ms": result.capture_time_ms,
                    "threshold": threshold, "valid": result.valid, "track_lost": result.track_lost,
                    "error_px": f"{result.error_px:.3f}", "confidence": f"{result.confidence:.3f}",
                    "valid_rows": result.valid_rows, "reconstructed_rows": result.reconstructed_rows,
                    "control_row_quality": ";".join(str(result.row_quality[row]) for row in args._applied_control_rows),
                    "expected_valid": SHADOW_EXPECTATIONS[name]["valid"] if shadow_check else "",
                    "expectation_passed": shadow_check["expectation_passed"] if shadow_check else "",
                })
                for row in range(HEIGHT):
                    if result.row_quality[row] == 2:
                        widths_by_row[row].append(int(result.right[row]) - int(result.left[row]))
                if args.save_binary_dir:
                    stem = f"{count:06d}"
                    (args.save_binary_dir / f"{stem}.pgm").write_bytes(b"P5\n188 120\n255\n" + bytes(image))
                    detail = {
                        "name": name, "evidence_type": evidence_type,
                        "frame_id": result.frame_id, "threshold": threshold,
                        "valid": result.valid, "error_px": result.error_px,
                        "confidence": result.confidence,
                        "left": list(result.left), "middle": list(result.middle),
                        "right": list(result.right), "row_quality": list(result.row_quality),
                    }
                    (args.save_binary_dir / f"{stem}.json").write_text(
                        json.dumps(detail, ensure_ascii=False, indent=2), encoding="utf-8"
                    )
        if args.export_width_profile:
            widths = [int(statistics.median(values)) if values else 0 for values in widths_by_row]
            if any(widths[row] < 3 for row in DEFAULT_ROWS):
                raise ValueError("默认控制带缺少完整实测边界，不能导出合格标定")
            args.export_width_profile.parent.mkdir(parents=True, exist_ok=True)
            args.export_width_profile.write_text(json.dumps({
                "source": str(args.input.resolve()),
                "operator_confirmed_centered_straight": True,
                "reference_width": widths,
            }, ensure_ascii=False, indent=2), encoding="utf-8")
    finally:
        if args.output:
            output.close()
    summary = {"frames": count, "valid_frames": valid_count, "evidence_type": evidence_type,
               "time_source": "assumed_replay_period", "hardware_acceptance": False,
               "build": build_metadata, "shadow_checks": shadow_checks}
    if args.summary_output:
        args.summary_output.parent.mkdir(parents=True, exist_ok=True)
        args.summary_output.write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(summary), file=sys.stderr)
    return 1 if any(check["expectation_passed"] is False for check in shadow_checks) else 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, ValueError, KeyError, subprocess.CalledProcessError) as error:
        print(f"回放失败：{error}", file=sys.stderr)
        if isinstance(error, subprocess.CalledProcessError):
            print(error.stderr, file=sys.stderr)
        raise SystemExit(1)
