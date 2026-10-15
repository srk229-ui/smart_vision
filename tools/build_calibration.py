"""构建M3三种固件并核对标定内存；不下载、不连接探针。"""

from pathlib import Path
import argparse
import hashlib
import json
import os
import re
import shutil
import subprocess
from elftools.elf.elffile import ELFFile

ROOT = Path(__file__).resolve().parents[1]
DESTINATION = ROOT / "docs/evidence/M3/calibration/builds"
TARGETS = {
    "nor_sdram_zf_dtcm": ("mdk/Objects/rt1064.axf", "mdk/Listings/rt1064.map"),
    "calibration_readonly": ("mdk/Objects/calibration_readonly/calibration_readonly.axf",
                             "mdk/Listings/calibration_readonly/calibration_readonly.map"),
    "calibration_active": ("mdk/Objects/calibration_active/calibration_active.axf",
                           "mdk/Listings/calibration_active/calibration_active.map"),
}


def inspect_elf(path, diagnostic):
    """校验最终ELF对象大小和完整DTCM地址区间，正式版不得残留命令/记录对象。"""
    result = {}
    with path.open("rb") as stream:
        elf = ELFFile(stream)
        if not elf.little_endian or elf["e_machine"] != "EM_ARM":
            raise RuntimeError("固件必须是小端ARM ELF")
        symbols = elf.get_section_by_name(".symtab")
        for name, expected in (("calibration_debug_mailbox", 416), ("calibration_debug_log", 32768)):
            items = symbols.get_symbol_by_name(name) or []
            if not diagnostic:
                if items:
                    raise RuntimeError("正式版仍包含标定命令或记录区: " + name)
                continue
            if len(items) != 1:
                raise RuntimeError("诊断符号缺失或重复: " + name)
            symbol = items[0]
            start, size = int(symbol["st_value"]), int(symbol["st_size"])
            if size != expected or start % 32 or not 0x20000000 <= start < start + size <= 0x2006DC00:
                raise RuntimeError("标定区对象大小/对齐或DTCM范围错误: " + name)
            result[name] = {"address": hex(start), "bytes": size, "last_address": hex(start + size - 1)}
    return result


def build(target):
    """按独立目标完整构建，先检查返回码和日志，再冻结固件副本。"""
    destination = DESTINATION / target
    destination.mkdir(parents=True, exist_ok=True)
    log = destination / "keil_rebuild.log"
    if log.exists():
        log.unlink()  # 只删除明确命名的本目标旧日志，避免误用上一次PASS。
    startup = subprocess.STARTUPINFO()
    startup.dwFlags |= subprocess.STARTF_USESHOWWINDOW
    startup.wShowWindow = 0
    temporary = destination / "_temp"
    temporary.mkdir(parents=True, exist_ok=True)
    environment = os.environ.copy()
    environment["TEMP"] = environment["TMP"] = str(temporary)  # 编译器临时文件放在可写工作区，避免沙箱Temp重命名失败。
    process = subprocess.run(
        ["C:/Keil_v5/UV4/UV4.exe", "-r", str(ROOT / "mdk/rt1064.uvprojx"),
         "-t", target, "-j0", "-o", str(log)],
        cwd=ROOT / "mdk", startupinfo=startup, timeout=240, env=environment,
    )
    data = log.read_bytes() if log.exists() else b""
    encoding = "utf-16" if data.startswith((b"\xff\xfe", b"\xfe\xff")) else "gb18030"
    output = data.decode(encoding, errors="replace")
    print(target, "\n".join(output.splitlines()[-7:]), flush=True)
    sizes = re.search(r"Program Size: Code=(\d+) RO-data=(\d+) RW-data=(\d+) ZI-data=(\d+)", output)
    if process.returncode or "0 Error(s), 0 Warning(s)" not in output or sizes is None:
        raise RuntimeError("完整构建失败: " + target)
    axf, map_path = (ROOT / value for value in TARGETS[target])
    # 部分Keil设置把map放在对象目录，兼容当前实际生成位置。
    if not map_path.exists():
        map_path = axf.with_suffix(".map")
    if not axf.exists() or not map_path.exists():
        raise RuntimeError("完整构建没有生成AXF/map")
    memory = inspect_elf(axf, target != "nor_sdram_zf_dtcm")
    shutil.copy2(axf, destination / "firmware.axf")
    shutil.copy2(map_path, destination / "firmware.map")
    result = {
        "target": target, "errors": 0, "warnings": 0,
        "size": dict(zip(("code", "ro", "rw", "zi"), map(int, sizes.groups()))),
        "axf_sha256": hashlib.sha256(axf.read_bytes()).hexdigest(),
        "memory": memory, "hardware_tested": False,
    }
    (destination / "build.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
    return result


def main():
    global DESTINATION
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--target", choices=tuple(TARGETS), help="不指定时完整构建三个目标")
    parser.add_argument('--output-dir', type=Path, help='将本轮固件另存到新目录，保留历史冻结版本')
    args = parser.parse_args()
    if args.output_dir is not None:
        DESTINATION = args.output_dir.resolve()
    results = [build(target) for target in ([args.target] if args.target else TARGETS)]
    (DESTINATION / "summary.json").write_text(json.dumps(results, indent=2), encoding="utf-8")


if __name__ == "__main__":
    main()
