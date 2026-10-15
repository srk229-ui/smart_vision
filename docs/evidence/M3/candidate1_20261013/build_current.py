from pathlib import Path
import json
import sys

# 复用工程构建入口，仅另存本次候选参数的三目标证据，不连接或下载实机。
root = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(root / 'tools'))
import build_calibration as builds
builds.DESTINATION = Path(__file__).resolve().parent / 'builds'
results = [builds.build(target) for target in builds.TARGETS]
(builds.DESTINATION / 'summary.json').write_text(json.dumps(results, indent=2), encoding='utf-8')
