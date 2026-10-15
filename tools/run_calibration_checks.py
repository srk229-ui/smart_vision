"""运行M3标定软件与原有回归，保存独立证据；不连接或驱动实机。"""

from pathlib import Path
import argparse
import json
import os
import subprocess
import sys
import run_software_checks as checks

ROOT = Path(__file__).resolve().parents[1]
EVIDENCE = ROOT / "docs/evidence/M3/calibration/tests"


def main():
    global EVIDENCE
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--evidence-dir', type=Path, help='另存本轮验证证据，保留历史结果')
    args = parser.parse_args()
    if args.evidence_dir is not None:
        EVIDENCE = args.evidence_dir.resolve()
    sys.stdout.reconfigure(encoding="utf-8")
    checks.EVIDENCE = EVIDENCE
    cases = [
        ("test_measurement", ["encoder_measurement"], []),
        ("test_steering_map", ["steering_map", "calibration_profile"], []),
        ("test_measured_profile", ["encoder_measurement", "steering_map", "calibration_profile"], []),
        ("test_servo_cal_control", ["control", "pid"], []),
        ("test_servo_cal_control_enabled", ["control", "pid"], ["CALIBRATION_DEBUG_ENABLE=1"]),
        ("test_calibration_debug", ["calibration_debug", "encoder_measurement", "steering_map"],
         ["CALIBRATION_DEBUG_ENABLE=1", "CALIBRATION_DEBUG_HOST=1"]),
        ("test_calibration_integration", ["control", "pid", "calibration_debug", "encoder_measurement", "steering_map"],
         ["CALIBRATION_DEBUG_ENABLE=1", "CALIBRATION_DEBUG_HOST=1"]),
    ]
    for name, modules, defines in cases:
        source_name = "test_servo_cal_control" if name.endswith("_enabled") else name
        sources = [ROOT / "tools/tests" / (source_name + ".c")]
        sources += [ROOT / "user/src" / (module + ".c") for module in modules]
        checks.compile_test(name, sources, defines)
    subprocess.run([sys.executable, str(ROOT / 'tools/tests/test_calibration_wifi.py'),
                    '--evidence-dir', str(EVIDENCE)], cwd=ROOT, check=True,
                   env={**os.environ, 'PYTHONIOENCODING': 'utf-8'})
    result = subprocess.run([sys.executable, "-m", "unittest", "discover", "-s", "tools/tests",
                             "-p", "test_calibrate_pyocd.py", "-v"], cwd=ROOT, capture_output=True, env={**os.environ, "PYTHONIOENCODING": "utf-8"})
    EVIDENCE.mkdir(parents=True, exist_ok=True)
    (EVIDENCE / "test_calibrate_pyocd.log").write_bytes(result.stdout + result.stderr)
    print((result.stdout + result.stderr).decode("utf-8", errors="replace"))
    if result.returncode:
        raise RuntimeError("pyOCD软件测试失败")
    subprocess.run([sys.executable, str(ROOT / "tools/run_software_checks.py"), "--evidence-dir",
                    str(EVIDENCE.parent / "regression")], cwd=ROOT, check=True)
    (EVIDENCE / "summary.json").write_text(json.dumps({"c_cases": [case[0] for case in cases],
        "wifi_service_variants": 2,
        "python_tests_passed": True, "legacy_regression_passed": True, "hardware_tested": False}, indent=2), encoding="utf-8")
    print("PASS M3 calibration software; probe, GPT, PWM, mechanics and RPM field acceptance remain pending.")


if __name__ == "__main__":
    main()
