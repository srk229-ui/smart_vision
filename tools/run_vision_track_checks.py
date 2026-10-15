"""验证普通道路C移植、骨架对照及闭环场景；不连接探针、不烧录。"""
from pathlib import Path
import argparse
import ctypes as C
import hashlib
import json
import os
import subprocess
import sys
import time
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'tools'))
sys.path.insert(0, str(ROOT/'full_track_sim'))
import run_software_checks as checks
from generate_vision_map import render_header
from track_sim.config import load, merge, feasible_preset
from track_sim.elements import thin
from track_sim.native import Controller, U8P
import track_sim.native as native
import track_sim.simulation as simulation


def compile_shared(folder, camera=None):
    """同一宿主桥接真实当前C核心；替代映射只放证据目录，不修改固件常量。"""
    folder.mkdir(parents=True, exist_ok=True)
    include_define = []
    if camera is not None:
        header = folder/'test_map.h'
        header.write_text(render_header(camera),encoding='utf-8')
        include_define = ['/DVISION_TRACK_MAP_HEADER="'+header.as_posix()+'"']
    # 原桥接提供原控制/PWM/编码器ABI，以下包装只把视觉扩展替换成真实C移植。
    code = '''#define sim_reset original_sim_reset
#define sim_vision original_sim_vision
#include "__BRIDGE__"
#undef sim_reset
#undef sim_vision
#include "__TRACK__"
SIM_API int sim_reset(const float *values,const float *scale) {
    float p[30]; int ok;
    memcpy(p,values,sizeof(p)); p[27]=500.0f;
    ok=original_sim_reset(p,scale); vision_track_init(NULL); return ok;
}
SIM_API int sim_vision(const uint8_t *gray,uint32_t id,uint32_t capture,double *meta,uint8_t *lines,uint8_t *binary) {
    ControlParams p;
    uint32_t histogram[256]; uint8_t threshold;
    memcpy(binary,gray,CONTROL_IMAGE_WIDTH*CONTROL_IMAGE_HEIGHT);
    vision_track_histogram(gray,histogram); threshold=get_threshold_otsu(histogram);
    binaryzation_process(binary,CONTROL_IMAGE_HEIGHT,CONTROL_IMAGE_WIDTH,threshold);
    vision_process(binary,id,capture,&last_vision); last_vision.threshold=threshold;
    meta[4]=threshold;
    control_get_params(&p,NULL);vision_timing_begin(id,capture,0u);
    vision_track_process(binary,&p,&last_vision);vision_timing_finish();
    meta[0]=last_vision.error_px;meta[1]=last_vision.confidence;meta[2]=last_vision.valid;meta[3]=last_vision.track_lost;
    meta[5]=last_vision.valid_rows;meta[6]=last_vision.reconstructed_rows;
    memcpy(lines,last_vision.left,120u);memcpy(lines+120,last_vision.middle,120u);
    memcpy(lines+240,last_vision.right,120u);memcpy(lines+360,last_vision.row_quality,120u);
    return last_vision.valid;
}
SIM_API int test_thin(const uint8_t *input,uint8_t *output) {
    uint8_t status;vision_track_init(NULL);vision_timing_begin(0u,0u,0u);memcpy(road_mask,input,sizeof(road_mask));
    status=thin_road(0u);memcpy(output,skeleton,sizeof(skeleton));return status;
}
SIM_API int test_status(void) { return timing_record.recovery_status; }
SIM_API int test_frames(void) { return (int)vision_timing_log.frames; }
SIM_API void test_normal_rows(uint8_t *out) {
    memcpy(out,guard_kind,120u);memcpy(out+120u,edge_valid[0],120u);
    memcpy(out+240u,edge_valid[1],120u);memcpy(out+360u,normal_hits,120u);
}
SIM_API int test_band(const uint8_t *quality,const uint8_t *rows,uint8_t *selected) {
    ControlParams p;VisionResult r;unsigned i;
    control_init(92.5f,85.0f,100.0f,-1.0f);control_get_params(&p,NULL);memset(&r,0,sizeof(r));
    memcpy(p.look_rows,rows,5u);memcpy(r.row_quality,quality,120u);
    for(i=0u;i<120u;++i)r.middle[i]=70u;
    if(!summarize_guard(&p,&r))return 0;
    memcpy(selected,vision_track_control_rows,5u);return guard_adaptive ? 2 : 1;
}
SIM_API int test_ground(float u,float v,float *x,float *y) { return image_ground_point(u,v,x,y); }
SIM_API int test_image(float x,float y,float *u,float *v) { return ground_image_point(x,y,u,v); }
SIM_API int test_guard(const uint8_t *binary,uint32_t id,uint32_t capture,float *meta,uint8_t *lines) {
    ControlParams p; VisionResult r; int status;
    control_get_params(&p,NULL); vision_process(binary,id,capture,&r); vision_timing_begin(id,capture,0u);
    status=vision_track_process(binary,&p,&r);
    meta[0]=r.error_px;meta[1]=r.confidence;meta[2]=(float)r.valid;meta[3]=(float)r.reconstructed_rows;
    memcpy(lines,r.left,120u);memcpy(lines+120u,r.middle,120u);memcpy(lines+240u,r.right,120u);memcpy(lines+360u,r.row_quality,120u);
    return status;
}
SIM_API void test_reset(void) { control_init(92.5f,85.0f,100.0f,-1.0f); vision_reset(); vision_track_init(NULL); }
'''.replace('__BRIDGE__',(ROOT/'full_track_sim/native/bridge.c').as_posix()).replace('__TRACK__',(ROOT/'user/src/vision_track.c').as_posix())
    fixture=folder/'bridge.c';fixture.write_text(code,encoding='utf-8')
    vc=sorted(Path('C:/Program Files/Microsoft Visual Studio/2022/Community/VC/Tools/MSVC').glob('*'))[-1]
    kit=sorted(Path('C:/Program Files (x86)/Windows Kits/10/Include').glob('*'))[-1]
    sources=[fixture]+[ROOT/'user/src'/f'{name}.c' for name in ('control','pid','img_process','encoder_measurement','steering_map','calibration_profile')]
    target=folder/'controller.dll'
    command=[str(vc/'bin/Hostx64/x64/cl.exe'),'/nologo','/std:c11','/utf-8','/O2','/W4','/WX','/wd4127','/MD','/LD',
             '/I'+str(ROOT/'user/inc'),'/I'+str(vc/'include'),*['/I'+str(kit/n) for n in ('ucrt','shared','um')],
             *include_define,*map(str,sources),'/link','/OUT:'+str(target),'/LIBPATH:'+str(vc/'lib/x64'),
             *['/LIBPATH:'+str(kit.parents[1]/'Lib'/kit.name/n/'x64') for n in ('ucrt','um')]]
    result=subprocess.run(command,cwd=folder,capture_output=True,timeout=120)
    (folder/'build.log').write_bytes(result.stdout+result.stderr)
    if result.returncode:raise RuntimeError((result.stdout+result.stderr).decode('utf-8',errors='replace'))
    return target


def skeleton_parity(dll):
    """对照原Zhang-Suen及多个连接形状，确认两个子步骤同步删除。"""
    lib=C.CDLL(str(dll));lib.test_thin.argtypes=[U8P,U8P];lib.test_thin.restype=C.c_int
    cases=[]
    mask=np.zeros((116,90),np.uint8);mask[8:108,35:55]=1;cases.append(('straight',mask.copy()))
    mask[50:70,8:82]=1;cases.append(('cross',mask.copy()))
    yy,xx=np.indices((116,90));mask=(np.abs(xx-(45+14*np.sin(yy/21)))<9).astype(np.uint8);cases.append(('s_curve',mask))
    mask=(((xx-45)**2+(yy-58)**2<30**2)&((xx-45)**2+(yy-58)**2>17**2)).astype(np.uint8);cases.append(('ring_connectivity_only',mask))
    mask=np.zeros((116,90),np.uint8);mask[:45,:20]=1;cases.append(('border',mask))
    rng=np.random.default_rng(71);cases.append(('noise',(rng.random((116,90))>.78).astype(np.uint8)))
    cases.append(('iteration_limit',np.ones((116,90),np.uint8)))
    outcomes=[]
    for name,mask in cases:
        out=np.zeros_like(mask);status=lib.test_thin(mask.ctypes.data_as(U8P),out.ctypes.data_as(U8P))
        np.testing.assert_array_equal(out,thin(mask).astype(np.uint8))
        if name=='iteration_limit' and status!=3:raise AssertionError('未收敛骨架没有报告迭代上限')
        outcomes.append({'name':name,'status':status,'pixels':int(out.sum()),'same_as_python':True})
    return outcomes


def run_case(dll,config,name,output):
    """控制路径使用当前C视觉扩展，仿真真实地图只进入渲染和评分。"""
    old_build=native.build
    native.build=lambda source_root=None:dll
    try:
        sim=simulation.Simulation(config)
        start=time.perf_counter();result=sim.run();wall=time.perf_counter()-start
        # 原仿真界面仍用冻结版100ms文字；这里依据当前C故障及实际年龄生成准确说明。
        if int(sim.snapshot['fault'])==1:
            result['reason']='当前C：最后有效图像采集年龄超过500逻辑ms'
        outcome={'name':name,'result':result,'config':config,'host_wall_s':wall,'hardware_tested':False,
                 'fault_snapshot':sim.snapshot,'host_dll_sha256':hashlib.sha256(dll.read_bytes()).hexdigest(),
                 'note':'宿主耗时不是ARM性能；保留虚拟直道标定及三帧启动包装，当前C保护为500ms；elements未执行'}
        directory=output/name;directory.mkdir(parents=True,exist_ok=True)
        (directory/'result.json').write_text(json.dumps(outcome,ensure_ascii=False,indent=2),encoding='utf-8')
        print(name,result['status'],round(result['time_s'],2),round(result['progress_pct'],2),flush=True)
        return outcome
    finally:
        native.build=old_build


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--evidence-dir',type=Path,default=ROOT/'docs/evidence/M3/bev_minimal_20261014')
    parser.add_argument('--skip-scenes',action='store_true')
    args=parser.parse_args();output=args.evidence_dir.resolve();output.mkdir(parents=True,exist_ok=True)
    sys.stdout.reconfigure(encoding='utf-8')
    checks.EVIDENCE=output/'tests'
    checks.compile_test('test_vision_track',[ROOT/'tools/tests/test_vision_track.c',*[ROOT/'user/src'/f'{n}.c' for n in ('vision_track','img_process','control','pid')]])
    for name,command in [('pipeline',[sys.executable,str(ROOT/'tools/tests/test_vision_pipeline.py'),'--evidence-dir',str(checks.EVIDENCE)]),
                         ('normal',[sys.executable,str(ROOT/'tools/tests/test_vision_normal.py'),'--evidence-dir',str(output/'normal')]),
                         ('map',[sys.executable,'-m','unittest','discover','-s',str(ROOT/'tools/tests'),'-p','test_vision_map.py','-v']),
                         ('timing_decode',[sys.executable,'-m','unittest','discover','-s',str(ROOT/'tools/tests'),'-p','test_vision_timing_decode.py','-v'])]:
        check=subprocess.run(command,cwd=ROOT,capture_output=True,env={**os.environ,'PYTHONIOENCODING':'utf-8'})
        (output/(name+'_checks.log')).write_bytes(check.stdout+check.stderr)
        if check.returncode:raise RuntimeError((check.stdout+check.stderr).decode('utf-8',errors='replace'))
    default_dll=compile_shared(output/'host_default')
    parity=skeleton_parity(default_dll)
    (output/'skeleton_parity.json').write_text(json.dumps(parity,indent=2),encoding='utf-8')
    print('PASS seven skeleton parity cases',flush=True)
    results=[]
    if not args.skip_scenes:
        assumed=feasible_preset();camera=dict(assumed['camera'],road_width_m=.45)
        feasible_dll=compile_shared(output/'host_hypothetical',camera)
        for scenario,directions in [('straight',['forward']),('left_curve',['forward']),('right_curve',['forward']),('s_curve',['forward']),('midterm',['forward','reverse'])]:
            for direction in directions:
                cfg=merge(assumed,{'experiment':{'scenario':scenario,'algorithm':'baseline','direction':direction}})
                results.append(run_case(feasible_dll,cfg,scenario+'_'+direction,output/'scenarios'))
        cfg=load({'controller':{'steer_kp':.22,'steer_kd':.002},'experiment':{'scenario':'midterm','algorithm':'baseline','speed_mps':.5}})
        # 当前配置保持估计相机、部分实测角表、原中位/变化率；基础0.5m/s和PD与本轮固件一致。
        results.append(run_case(default_dll,cfg,'estimated_current_midterm',output/'scenarios'))
        for patch,name in [({'blackout_start_s':.4,'blackout_duration_s':1.0},'camera_timeout'),({'noise_sigma':8},'midterm_noise')]:
            cfg=merge(assumed,{'camera':patch,'experiment':{'scenario':'midterm','algorithm':'baseline'}})
            results.append(run_case(feasible_dll,cfg,name,output/'scenarios'))
        success_names={r['name'] for r in results if r['result']['status']=='success'}
        required={'straight_forward','left_curve_forward','right_curve_forward','s_curve_forward','midterm_forward','midterm_reverse','midterm_noise'}
        if not required<=success_names:
            raise RuntimeError('普通道路闭环尚未通过：'+','.join(sorted(required-success_names)))
        timeout=next(r for r in results if r['name']=='camera_timeout')
        if timeout['result']['status']!='failed' or timeout['result']['time_s']>.95:
            raise RuntimeError('当前500ms控制保护未在预期模拟窗口触发')
    summary={'c_guards':True,'skeleton_parity_cases':len(parity),'scenarios':[{'name':r['name'],**r['result']} for r in results],
             'hardware_tested':False,'firmware_mapping_source':'estimated_current_camera_not_measured',
             'firmware_source_sha256':{p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in (ROOT/'user/src').glob('*.c')}}
    (output/'vision_summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding='utf-8')
    print('PASS C port checks; ARM timing and real A→B remain pending')


if __name__=='__main__':
    main()
