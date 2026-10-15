"""用模拟 CSI 事件编译实际快照代码，验证复制竞争和旧完成标志。"""

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
    source = (ROOT / "user/src/camera_frame.c").read_text(encoding="utf-8")
    body = "\n".join(line for line in source.splitlines() if not line.startswith("#include"))
    fixture = r'''
#include "control_types.h"
#include "camera_frame.h"
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <assert.h>
#include <stdio.h>
typedef uint32_t uint32;
typedef int status_t;
typedef struct { int unused; } CSI_Type;
typedef struct csi_handle csi_handle_t;
typedef void (*csi_transfer_callback_t)(CSI_Type *,csi_handle_t *,status_t,void *);
struct csi_handle { csi_transfer_callback_t callback; void *userData; };
enum { kStatus_CSI_FrameDone=1 };
static csi_handle_t csi_handle;
static CSI_Type peripheral;
static uint8_t pixels_a[188*120],pixels_b[188*120];
static uint8_t *mt9v03x_image,*pending_image;
static volatile uint8_t mt9v03x_finish_flag;
static uint32_t fake_ms,irq_mask;
static uint8_t inject,original_success=1;
uint32_t control_now_ms(void) { return fake_ms; }
uint32_t interrupt_global_disable(void) { uint32_t old=irq_mask; irq_mask=1; return old; }
void interrupt_global_enable(uint32_t key) { irq_mask=key; }
static void board_callback(CSI_Type *base,csi_handle_t *handle,status_t status,void *data) {
    (void)base;(void)handle;(void)status; assert(data==(void *)7);
    if(original_success) { mt9v03x_image=pending_image; mt9v03x_finish_flag=1; }
}
static void *checked_copy(void *dest,const void *src,size_t length) {
    void *result; assert(!irq_mask); result=memcpy(dest,src,length);
    if(inject) { inject=0; pending_image=pixels_a;fake_ms=60;
        csi_handle.callback(&peripheral,&csi_handle,kStatus_CSI_FrameDone,csi_handle.userData); }
    return result;
}
#define memcpy checked_copy
'''
    cases = r'''
int main(void) {
    uint8_t output[188*120]; uint32_t id,time;
    memset(pixels_a,17,sizeof(pixels_a));memset(pixels_b,29,sizeof(pixels_b));
    csi_handle.callback=board_callback;csi_handle.userData=(void *)7;
    camera_frame_install();camera_frame_install();
    assert(!camera_frame_take(output,&id,&time));
    pending_image=pixels_a;fake_ms=20;
    csi_handle.callback(&peripheral,&csi_handle,kStatus_CSI_FrameDone,csi_handle.userData);
    assert(camera_frame_take(output,&id,&time)&&id==1&&time==20&&output[0]==17);
    assert(!camera_frame_take(output,&id,&time));
    pending_image=pixels_b;fake_ms=40;
    csi_handle.callback(&peripheral,&csi_handle,kStatus_CSI_FrameDone,csi_handle.userData);
    inject=1;assert(!camera_frame_take(output,&id,&time));
    assert(camera_frame_dropped()==1);
    assert(camera_frame_take(output,&id,&time)&&id==3&&time==60&&output[0]==17);
    assert(camera_frame_dropped()==1);
    original_success=0;mt9v03x_finish_flag=1;fake_ms=80;
    csi_handle.callback(&peripheral,&csi_handle,kStatus_CSI_FrameDone,csi_handle.userData);
    assert(!camera_frame_take(output,&id,&time));
    assert(!camera_frame_take(NULL,&id,&time)&&!irq_mask);
    puts("PASS camera actual-source contract: callback, time, copy race, drop count, stale flag");
    return 0;
}
'''
    destination = checks.EVIDENCE / "test_camera_contract"
    destination.mkdir(parents=True, exist_ok=True)
    generated = destination / "test_camera_contract.c"
    generated.write_text(fixture + body + cases, encoding="utf-8")
    compile_test("test_camera_contract", [generated])


if __name__ == "__main__":
    main()
