"""解码停车冻结记录；仅显式提供精确探针ID时才只读导出，不复位、不烧录、不启动。"""
from pathlib import Path
import argparse
import csv
import hashlib
import json
import math
import struct

from decode_vision_timing import decode as decode_timing, RECORD, STAGES

HEADER=struct.Struct('<16I')
VALUES=struct.Struct('<9f4I')
CAPACITY=8
PACKED=2820
FRAME_BYTES=3428
TOTAL_BYTES=54288
TIMING_BYTES=4240


def decode(raw):
    """只接受对应ABI的完整、偶数代际、已冻结取证，不把实时内存当停车证据。"""
    if len(raw)!=TOTAL_BYTES:
        raise ValueError('冻结记录长度错误，请核对对应AXF的vision_stop_log大小')
    header=dict(zip(('magic','sequence','abi','bytes','generation','frozen','count','next','stop_ms',
                     'fault','stop_state','gray_frame_id','frame_bytes','packed_bytes','last_capture_ms','reserved'),HEADER.unpack_from(raw)))
    if (header['magic']!=0x31525356 or header['sequence']&1 or header['abi']!=1 or header['bytes']!=TOTAL_BYTES
        or header['frozen']!=1 or header['count']>8 or header['next']>=8 or header['frame_bytes']!=FRAME_BYTES
        or header['packed_bytes']!=PACKED or header['reserved']):
        raise ValueError('不是稳定、已冻结且格式匹配的记录')
    frames=[]
    first=(header['next']-header['count'])%CAPACITY
    for index in range(header['count']):
        offset=HEADER.size+((first+index)%CAPACITY)*FRAME_BYTES
        values=RECORD.unpack_from(raw,offset)
        timing=dict(zip(('frame_id','capture_time_ms','finish_us'),values[:3]))
        timing.update({name+'_us':value for name,value in zip(STAGES,values[3:14])})
        timing.update(dict(zip(('thin_iterations','path_points','recovery_status','mapping_estimated','clock_valid','reserved'),values[14:])))
        if timing['recovery_status']>9 or timing['thin_iterations']>45 or timing['path_points']>80 or timing['reserved']:
            raise ValueError('帧耗时元数据非法')
        scalar=VALUES.unpack_from(raw,offset+64)
        names=('error_px','confidence','servo_cmd','pwm_left','pwm_right','target_left','target_right','measured_left','measured_right',
               'valid','track_lost','valid_rows','reconstructed_rows')
        record=dict(zip(names,scalar))
        if (not all(math.isfinite(v) for v in scalar[:9]) or not 0<=record['confidence']<=1 or
            record['valid']>1 or record['track_lost']>1 or not 0<=record['reconstructed_rows']<=record['valid_rows']<=120):
            raise ValueError('帧视觉状态非法')
        record['threshold']=struct.unpack_from('<I',raw,offset+116)[0]
        record['control_rows']=list(raw[offset+120:offset+125])
        if record['threshold']>255 or any(r>=120 for r in record['control_rows']) or any(raw[offset+125:offset+128]):
            raise ValueError('阈值或实际控制带元数据非法')
        start=offset+128
        lines=[list(raw[start+120*i:start+120*(i+1)]) for i in range(4)]
        if any(q>2 for q in lines[3]) or any(v>=188 for line in lines[:3] for v in line):
            raise ValueError('原始边线坐标或质量非法')
        packed=raw[start+480:start+480+PACKED]
        pixels=bytes(255 if packed[i>>3] & (1<<(i&7)) else 0 for i in range(22560))
        frames.append(dict(record,timing=timing,left=lines[0],middle=lines[1],right=lines[2],quality=lines[3],pixels=pixels))
    gray_offset=HEADER.size+CAPACITY*FRAME_BYTES
    gray=raw[gray_offset:gray_offset+22560]
    if frames and (frames[-1]['timing']['frame_id']!=header['gray_frame_id'] or frames[-1]['timing']['capture_time_ms']!=header['last_capture_ms']):
        raise ValueError('灰度/二值采集标签不一致')
    timing_rows,timing_summary=decode_timing(raw[gray_offset+22560:])
    return header,frames,gray,timing_rows,timing_summary


def read_stable(read_memory,address,size):
    """读前后完整头一致、sequence为偶数才接收；函数没有任何目标写入接口。"""
    first=bytes(read_memory(address,HEADER.size))
    if len(first)!=64 or HEADER.unpack(first)[1]&1 or HEADER.unpack(first)[5]!=1:
        raise ValueError('请先停车并等待主循环冻结；本工具不发送STOP命令')
    raw=bytes(read_memory(address,size))
    after=bytes(read_memory(address,HEADER.size))
    if first!=after or raw[:64]!=first:
        raise ValueError('导出期间记录变化，请在停车后重新读取')
    decode(raw)
    return raw


def read_probe(elf_path,probe_id):
    """只读连接采用工程已核对的attach选项，先比对Flash，再读取冻结对象。"""
    from elftools.elf.elffile import ELFFile
    import pyocd
    from pyocd.core.helpers import ConnectHelper
    from pyocd.core.session import Session
    from pyocd.core.target import Target
    from calibrate_pyocd import ATTACH_OPTIONS
    if pyocd.__version__!='0.45.1':
        raise ValueError('只读连接仅核对过pyOCD 0.45.1')
    with elf_path.open('rb') as stream:
        elf=ELFFile(stream)
        if elf['e_machine']!='EM_ARM' or not elf.little_endian:
            raise ValueError('需要对应的小端ARM AXF')
        symbols=elf.get_section_by_name('.symtab').get_symbol_by_name('vision_stop_log') or []
        if len(symbols)!=1 or symbols[0]['st_size']!=TOTAL_BYTES:
            raise ValueError('AXF缺少匹配的冻结记录对象')
        address=int(symbols[0]['st_value'])
        if not 0x20000000<=address<address+TOTAL_BYTES<=0x2006dc00:
            raise ValueError('记录对象不在预期DTCM范围')
        segments=[(int(s['p_paddr']),s.data()) for s in elf.iter_segments()
                  if s['p_type']=='PT_LOAD' and s['p_filesz'] and 0x70000000<=s['p_paddr']<0x70400000]
        if not segments or any(a+len(b)>0x70400000 for a,b in segments):
            raise ValueError('AXF缺少可核对的Flash加载区')
    class ReadOnlySession(Session):
        def _load_user_script(self):
            return None
    probes=[p for p in ConnectHelper.get_all_connected_probes(blocking=False) if p.unique_id==probe_id]
    if len(probes)!=1:
        raise ValueError('未找到唯一精确匹配的探针ID')
    session=ReadOnlySession(probes[0],auto_open=False,options=dict(ATTACH_OPTIONS))
    try:
        session.open()
        target=session.target
        if target.get_state()!=Target.State.RUNNING:
            raise ValueError('CPU未运行；不会自动恢复或复位')
        for base,data in segments:
            for offset in range(0,len(data),4096):
                block=data[offset:offset+4096]
                if bytes(target.read_memory_block8(base+offset,len(block)))!=block:
                    raise ValueError('板上Flash与所选AXF不一致，拒绝按错误地址读取')
        return read_stable(target.read_memory_block8,address,TOTAL_BYTES)
    finally:
        session.close()


def save(raw,output,source=None):
    """写入新目录中的原始记录、无叠线图、质量叠加预览、CSV及JSON。"""
    from PIL import Image
    header,frames,gray,timings,stats=decode(raw)
    output.mkdir(parents=True,exist_ok=False)
    (output/'vision_stop.bin').write_bytes(raw)
    Image.frombytes('L',(188,120),gray).save(output/'last_gray.png')
    records=[]
    for index,frame in enumerate(frames):
        image=Image.frombytes('L',(188,120),frame['pixels'])
        image.save(output/f'{index:02d}_binary.png')
        overlay=image.convert('RGB')
        for row,q in enumerate(frame['quality']):
            if not q: continue
            overlay.putpixel((frame['middle'][row],row),(255,128,0) if q==1 else (255,0,0))
            if q==2:
                overlay.putpixel((frame['left'][row],row),(0,255,0)); overlay.putpixel((frame['right'][row],row),(255,255,0))
        overlay.resize((752,480),Image.Resampling.NEAREST).save(output/f'{index:02d}_overlay.png')
        records.append({k:v for k,v in frame.items() if k!='pixels'})
    (output/'summary.json').write_text(json.dumps(dict(header=header,frames=records,timing_summary=stats,source=source,
        sha256=hashlib.sha256(raw).hexdigest(),note='冻结为软件观察到停车；不是物理静止或首弯通过证明'),ensure_ascii=False,indent=2),encoding='utf-8')
    if timings:
        with (output/'timing.csv').open('w',encoding='utf-8-sig',newline='') as stream:
            writer=csv.DictWriter(stream,fieldnames=list(timings[0])); writer.writeheader(); writer.writerows(timings)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    group=parser.add_mutually_exclusive_group(required=True)
    group.add_argument('--input',type=Path,help='解码调试器已导出的54288字节原始记录，不连接硬件')
    group.add_argument('--elf',type=Path,help='只读连接前先用对应AXF比对板上Flash，必须同时提供精确探针ID')
    parser.add_argument('--probe-id',help='只读导出时精确匹配，工具不自动选择探针')
    parser.add_argument('--output-dir',required=True,type=Path)
    args=parser.parse_args()
    if args.output_dir.exists(): raise FileExistsError('输出目录已存在，拒绝覆盖')
    if args.elf and not args.probe_id: parser.error('--elf需要--probe-id；不会自动连接')
    raw=args.input.read_bytes() if args.input else read_probe(args.elf,args.probe_id)
    save(raw,args.output_dir,str(args.input or args.elf))
    print('已导出冻结图像、逐帧状态及耗时；未发送任何目标写入或运动命令')


if __name__=='__main__':
    main()
