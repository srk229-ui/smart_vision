"""编译主循环真实标定函数，回归错误报告成功的问题。"""

from pathlib import Path
import argparse
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))
from run_software_checks import compile_test
import run_software_checks as checks


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--evidence-dir", type=Path)
    args = parser.parse_args()
    if args.evidence_dir is not None:
        checks.EVIDENCE = args.evidence_dir.resolve()
    source = (ROOT / "user/src/main.c").read_text(encoding="utf-8")
    start = source.index("static uint8_t calibrate_width(")
    # 只截取标定函数自身，避免新增的停车显示函数被带入宿主夹具。
    end = source.index("\n}\n", start) + len("\n}\n")
    function = source[start:end]
    fixture = r'''
#include "control_types.h"
#include "img_process.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>
static VisionResult vision;
static uint32_t now = 100;
uint32_t control_now_ms(void) { return now; }
'''
    cases = r'''
int main(void) {
    ControlParams params;
    ControlSnapshot state;
    unsigned row;
    const uint8_t band[5] = {80,85,90,95,100};
    memset(&params,0,sizeof(params)); memcpy(params.look_rows,band,5);
    memset(&state,0,sizeof(state)); state.state=CONTROL_READY;
    memset(&vision,0,sizeof(vision)); vision.valid=1; vision.confidence=1; vision.capture_time_ms=now;
    for(row=0;row<120;++row) { vision.left[row]=93; vision.right[row]=95; vision.row_quality[row]=2; }
    assert(calibrate_width(&params,&state)==0);
    for(row=0;row<120;++row) { vision.left[row]=40; vision.right[row]=148; }
    assert(calibrate_width(&params,&state)==1);
    state.state=CONTROL_RUNNING; assert(calibrate_width(&params,&state)==0);
    state.state=CONTROL_READY; state.measured_left=2; assert(calibrate_width(&params,&state)==0);
    state.measured_left=0; now+=110; assert(calibrate_width(&params,&state)==0);
    vision.capture_time_ms=now; vision.row_quality[90]=0; assert(calibrate_width(&params,&state)==0);
    vision.row_quality[90]=2; vision.error_px=10; assert(calibrate_width(&params,&state)==0);
    puts("PASS main width calibration contract: 7 scenarios (real function)");
    return 0;
}
'''
    destination = checks.EVIDENCE / "test_main_contract"
    destination.mkdir(parents=True, exist_ok=True)
    generated = destination / "test_main_contract.c"
    generated.write_text(fixture + function + cases, encoding="utf-8")
    compile_test("test_main_contract", [generated, ROOT / "user/src/img_process.c"])


if __name__ == "__main__":
    main()
