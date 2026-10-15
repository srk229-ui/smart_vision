"""编译并执行固件的宿主测试，保存软件证据；不代替实机验收。"""

from pathlib import Path
import argparse
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
EVIDENCE = ROOT / "docs/evidence/M2"


def compile_test(name, sources, defines=()):
    # 控制器读取分轮标定和实测舵角插值，宿主夹具链接相同实现；按需补齐依赖，避免重复编译。
    sources = list(sources)
    if ROOT / "user/src/control.c" in sources:
        for module in ("calibration_profile", "steering_map"):
            dependency = ROOT / "user/src" / (module + ".c")
            if dependency not in sources:
                sources.append(dependency)
    directory = EVIDENCE / name
    directory.mkdir(parents=True, exist_ok=True)
    executable = directory / (name + ".exe")
    try:
        subprocess.run(["gcc", "--version"], check=True, capture_output=True)
        command = ["gcc", "-std=c99", "-Wall", "-Wextra", "-Werror", "-pedantic", "-O2",
                   "-I", str(ROOT / "user/inc"), *("-D" + value for value in defines), *map(str, sources), "-lm", "-o", str(executable)]
    except (OSError, subprocess.CalledProcessError):
        # 受限 Windows 环境中 GCC 不可启动时，使用已安装的 MSVC。
        compiler = sorted(Path("C:/Program Files/Microsoft Visual Studio/2022/Community/VC/Tools/MSVC").glob("*"))[-1]
        kit = sorted(Path("C:/Program Files (x86)/Windows Kits/10/Include").glob("*"))[-1]
        # 固件编译开关参与条件表达式是合法用法；仅关闭MSVC的常量条件提示，其余警告仍视为错误。
        command = [str(compiler / "bin/Hostx64/x64/cl.exe"), "/nologo", "/std:c11", "/utf-8", "/W4", "/WX", "/wd4127", "/MD",
                   "/I" + str(ROOT / "user/inc"), "/I" + str(compiler / "include"),
                   "/I" + str(kit / "ucrt"), "/I" + str(kit / "shared"), "/I" + str(kit / "um"),
                   "/Fe" + str(executable), *("/D" + value for value in defines), *map(str, sources), "/link",
                   "/LIBPATH:" + str(compiler / "lib/x64"),
                   "/LIBPATH:" + str(kit.parents[1] / "Lib" / kit.name / "ucrt/x64"),
                   "/LIBPATH:" + str(kit.parents[1] / "Lib" / kit.name / "um/x64")]
    result = subprocess.run(command, cwd=directory, capture_output=True)
    (directory / "compile.log").write_bytes(result.stdout + result.stderr)
    if result.returncode:
        raise RuntimeError((result.stdout + result.stderr).decode("utf-8", errors="replace"))
    result = subprocess.run([str(executable)], cwd=directory, capture_output=True, timeout=15)
    (directory / "result.log").write_bytes(result.stdout + result.stderr)
    print(result.stdout.decode("utf-8", errors="replace").strip())
    if result.returncode:
        raise RuntimeError(name + " 验证失败，请查看结果日志")


def keil_build():
    path = ROOT / "docs/evidence/M0/keil_rebuild.log"
    path.parent.mkdir(parents=True, exist_ok=True)
    startup = subprocess.STARTUPINFO()
    startup.dwFlags |= subprocess.STARTF_USESHOWWINDOW
    startup.wShowWindow = 0
    command = ["C:/Keil_v5/UV4/UV4.exe", "-r", str(ROOT / "mdk/rt1064.uvprojx"),
               "-t", "nor_sdram_zf_dtcm", "-j0", "-o", str(path)]
    result = subprocess.run(command, cwd=ROOT / "mdk", startupinfo=startup, timeout=180)
    print("Keil rebuild exit:", result.returncode)
    if not path.exists():
        raise RuntimeError("Keil 未生成构建日志")
    data = path.read_bytes()
    text = data.decode("utf-16" if data.startswith((b"\xff\xfe", b"\xfe\xff")) else "gb18030", errors="replace")
    print("\n".join(text.splitlines()[-12:]))
    if result.returncode != 0 or "0 Error(s), 0 Warning(s)" not in text:
        raise RuntimeError("Keil 完整构建未通过")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--keil-only", action="store_true")
    parser.add_argument("--evidence-dir", type=Path, help="另存本轮软件证据，避免覆盖历史M2")
    args = parser.parse_args()
    global EVIDENCE
    if args.evidence_dir is not None:
        EVIDENCE = args.evidence_dir.resolve()
    if args.keil_only:
        keil_build()
        return
    for name, modules in [("test_pid", ["pid"]), ("test_vision", ["img_process"]),
                          ("test_control", ["control", "pid"]),
                          ("test_local_buttons", ["local_buttons"]),
                          ("test_first_curve", ["vision_record", "vision_track", "img_process", "control", "pid"])]:
        sources = [ROOT / "tools/tests" / (name + ".c")]
        sources += [ROOT / "user/src" / (module + ".c") for module in modules]
        compile_test(name, sources)
    result = subprocess.run([sys.executable, str(ROOT / "drivers/wifi_spi_v2/tests/test_m2_wifi.py")],
                            cwd=ROOT, capture_output=True)
    EVIDENCE.mkdir(parents=True, exist_ok=True)
    (EVIDENCE / "wifi_checks.log").write_bytes(result.stdout + result.stderr)
    print(result.stdout.decode("utf-8", errors="replace").strip())
    if result.returncode:
        raise RuntimeError(result.stderr.decode("utf-8", errors="replace"))
    for name in ["test_main_contract", "test_camera_contract", "test_local_drive", "test_wifi_cancel", "test_vision_pipeline"]:
        result = subprocess.run([sys.executable, str(ROOT / "tools/tests" / (name + ".py")),
                                 "--evidence-dir", str(EVIDENCE)], cwd=ROOT, capture_output=True)
        (EVIDENCE / (name + "_checks.log")).write_bytes(result.stdout + result.stderr)
        print(result.stdout.decode("utf-8", errors="replace").strip())
        if result.returncode:
            raise RuntimeError(result.stderr.decode("utf-8", errors="replace"))
    print("PASS software checks; hardware, track and physical timing remain pending.")


if __name__ == "__main__":
    main()
