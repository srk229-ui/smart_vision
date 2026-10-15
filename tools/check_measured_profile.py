"""核对实测原件与真实C换算接口，保存单组来源及插值表；不连接硬件。"""

import argparse
import importlib.util
import json
import math
from pathlib import Path
import sys
import subprocess

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "speed_pi_sim"))
import run_software_checks as checks


def main():
    sys.stdout.reconfigure(encoding="utf-8")
    # 复用项目已有的实测原件校验；该工具的公共数据模块使用numpy，沿用既有隔离环境。
    if importlib.util.find_spec("numpy") is None:
        python = ROOT / "speed_pi_sim/.venv/Scripts/python.exe"
        if python.exists() and Path(sys.executable).resolve() != python.resolve():
            return subprocess.run([str(python), str(Path(__file__).resolve()), *sys.argv[1:]], cwd=ROOT).returncode
        raise RuntimeError("请使用speed_pi_sim的既有虚拟环境运行本工具")
    from pi_sim.calibration import load_calibration
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--evidence-dir", type=Path, default=ROOT / "docs/evidence/current_measured_profile")
    args = parser.parse_args()
    evidence = args.evidence_dir.resolve()
    vehicle = json.loads((ROOT / "speed_pi_sim/config/vehicle.json").read_text(encoding="utf-8-sig"))
    audit = load_calibration(ROOT / "calibration_records", vehicle)
    circumference = math.pi * vehicle["wheel_diameter_m"]
    checks.EVIDENCE = evidence / "tests"
    checks.compile_test("test_measured_profile", [ROOT / "tools/tests/test_measured_profile.c",
        *[ROOT / "user/src" / (name + ".c") for name in ("encoder_measurement", "steering_map", "calibration_profile")]])
    actual = json.loads((checks.EVIDENCE / "test_measured_profile/result.log").read_text(encoding="utf-8"))
    for row in audit["counts"]:
        wheel = row["wheel"]
        if abs(actual[wheel + "_cpr"] - row["counts_per_rev"]) > 0.0002:
            raise RuntimeError("固件分轮N与原始起终点不一致：" + wheel)
        if abs(actual[wheel + "_circumference_m"] - circumference) > 1e-7:
            raise RuntimeError("固件周长与已提供的名义轮径不一致：" + wheel)
        expected_counts = 0.5 * row["counts_per_rev"] * 0.01 / circumference
        if abs(actual[wheel + "_target_counts_050"] - expected_counts) > 1e-5:
            raise RuntimeError("真实C速度目标反换算错误：" + wheel)
    if abs(actual["right_deg_1550"] - audit["servo"]["predicted_right_deg_at_1550"]) > 1e-5:
        raise RuntimeError("右轮正插值与实测分段不一致")
    if abs(actual["right_zero_us"] - audit["servo"]["right_zero_estimate_us"]) > 0.001:
        raise RuntimeError("右轮反插值错误")
    audit.update(c_interface=actual, circumference_m=circumference,
        circumference_source=vehicle["wheel_diameter_source"], applied_to_firmware_source=True,
        pi_speed_unit="counts/10ms", hardware_flashed=False, hardware_acceptance=False)
    audit["servo"]["applied_to_firmware_source"] = True
    evidence.mkdir(parents=True, exist_ok=True)
    (evidence / "measurement_audit.json").write_text(json.dumps(audit, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print("PASS raw count anchors, separate-wheel conversion, nominal circumference, right-wheel interpolation and ABI")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
