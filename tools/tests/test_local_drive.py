"""编译实际主循环按键事件桥，验证本地启停与停止代际竞争。"""
from pathlib import Path
import argparse
import sys
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))
import run_software_checks as checks

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--evidence-dir", type=Path)
    args = parser.parse_args()
    if args.evidence_dir is not None:
        checks.EVIDENCE = args.evidence_dir.resolve()
    source = (ROOT / "user/src/main.c").read_text(encoding="utf-8")
    begin = source.index("static volatile uint8_t local_start_requested;")
    end = source.index("// 函数简介     初始化外设与控制层", begin)
    body = source[begin:end]
    fixture = r'''
#include "control.h"
#include "local_buttons.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
enum { C15 = 15, C14 = 14 };
static uint8_t start_down, stop_down;
static uint32_t mask;
static uint8_t gpio_get_level(int pin) { return (uint8_t)!(pin == C15 ? start_down : stop_down); }
static uint32_t interrupt_global_disable(void) { uint32_t old=mask; mask=1; return old; }
static void interrupt_global_enable(uint32_t old) { mask=old; }
'''
    cases = r'''
static void scan(uint8_t start, uint8_t stop) {
    start_down=start; stop_down=stop;
    local_drive_scan(); local_drive_scan();
}
static void good(uint32_t frame) {
    VisionResult v; memset(&v,0,sizeof(v));
    v.frame_id=frame; v.capture_time_ms=control_now_ms(); v.valid=1; v.confidence=1;
    control_publish_vision(&v);
}
static void setup(void) {
    control_set_critical_hooks(interrupt_global_disable,interrupt_global_enable);
    control_init(90,80,100,-1); control_set_output_enabled(1,1); control_tick(0,0);
    local_buttons_init(); local_start_requested=0; local_input_ready=1;
    scan(0,0);
}
int main(void) {
    unsigned i;
    setup(); good(1); scan(1,0);
    assert(local_drive_network_cancel()); local_drive_poll_start();
    assert(control_get_state()==CONTROL_RUNNING && !mask);
    for(i=0;i<100;i++) local_drive_scan();
    assert(!local_drive_network_cancel());
    scan(0,0); scan(1,0); assert(!local_drive_network_cancel());
    assert(control_get_state()==CONTROL_RUNNING);
    scan(1,1); assert(control_get_state()==CONTROL_READY && !local_start_requested);
    control_tick(0,0); scan(0,1); scan(1,1); local_drive_poll_start();
    assert(control_get_state()==CONTROL_READY);
    scan(1,0); local_drive_poll_start(); assert(control_get_state()==CONTROL_READY);
    scan(0,0); scan(1,0); local_drive_poll_start(); assert(control_get_state()==CONTROL_RUNNING);

    // 已登记启动与主循环领取之间插入远程STOP，代际不一致使其作废。
    setup(); good(1); scan(1,0); control_request_stop(); control_tick(0,0);
    local_drive_poll_start(); assert(control_get_state()==CONTROL_READY && !local_start_requested);
    good(2); local_drive_poll_start(); assert(control_get_state()==CONTROL_READY);

    // 无图启动失败即消费；后来有图不会复用旧请求。
    setup(); scan(1,0); local_drive_poll_start(); assert(control_get_state()==CONTROL_READY);
    good(1); local_drive_poll_start(); assert(control_get_state()==CONTROL_READY);
    scan(0,0); scan(1,0); local_drive_poll_start(); assert(control_get_state()==CONTROL_RUNNING);

    // 视觉超时恢复仍需新KEY1动作，普通非法故障不能被按键清除。
    for(i=0;i<51;i++) control_tick(0,0);
    assert(control_get_state()==CONTROL_FAULT);
    control_tick(0,0); good(2); local_drive_poll_start(); assert(control_get_state()==CONTROL_FAULT);
    scan(0,0); scan(1,0); local_drive_poll_start(); assert(control_get_state()==CONTROL_RUNNING);
    control_report_initialization_fault(); control_tick(0,0); good(3);
    scan(0,0); scan(1,0); local_drive_poll_start(); assert(control_get_state()==CONTROL_FAULT);

    // 诊断独占时KEY1不能登记取消事件，保持会话通信。
    setup(); good(1); control_set_calibration_reserved(1u); scan(1,0);
    assert(!local_drive_network_cancel()); control_set_calibration_reserved(0u);
    local_drive_poll_start(); assert(control_get_state()==CONTROL_READY);

    // 相机尚未就绪时的按下事件不能跨初始化重放。
    setup(); local_input_ready=0; scan(1,0); local_input_ready=1; good(1);
    local_drive_poll_start(); assert(control_get_state()==CONTROL_READY);
    scan(0,0); scan(1,0); local_drive_poll_start(); assert(control_get_state()==CONTROL_RUNNING);
    puts("PASS actual main local-drive bridge: offline start, STOP priority, generation, no replay, recovery");
    return 0;
}
'''
    directory = checks.EVIDENCE / "test_local_drive"
    directory.mkdir(parents=True, exist_ok=True)
    generated = directory / "test_local_drive.c"
    generated.write_text(fixture + body + cases, encoding="utf-8")
    checks.compile_test("test_local_drive", [generated, ROOT/"user/src/local_buttons.c",
                        ROOT/"user/src/control.c", ROOT/"user/src/pid.c"])

if __name__ == "__main__":
    main()
