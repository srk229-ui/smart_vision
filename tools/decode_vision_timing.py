"""离线解码调试器导出的vision_timing_log二进制，不连接或写入车模。"""
from pathlib import Path
import argparse
import csv
import json
import math
import struct

STAGES=('copy','histogram','otsu','binary','baseline','bev','thin','path','project','publish','total')
HEADER=struct.Struct('<III11I11Q')
RECORD=struct.Struct('<III11IHH4B')
CAPACITY=64


def decode(raw):
    """固定小端4240字节布局；奇数代际或槽号越界的快照必须拒绝。"""
    if len(raw)!=HEADER.size+CAPACITY*RECORD.size:
        raise ValueError('快照必须恰好4240字节；请先核对对应ELF的符号大小')
    h=HEADER.unpack_from(raw)
    sequence,frames,next_slot=h[:3]
    if sequence & 1 or next_slot>=CAPACITY:
        raise ValueError('记录正在更新或槽号非法，请重新导出稳定快照')
    count=min(frames,CAPACITY)
    first=(next_slot-count)%CAPACITY
    rows=[]
    for i in range(count):
        values=RECORD.unpack_from(raw,HEADER.size+((first+i)%CAPACITY)*RECORD.size)
        row=dict(zip(('frame_id','capture_time_ms','finish_us'),values[:3]))
        row.update({name+'_us':value for name,value in zip(STAGES,values[3:14])})
        row.update(dict(zip(('thin_iterations','path_points','recovery_status','mapping_estimated','clock_valid','reserved'),values[14:])))
        if row['thin_iterations']>45 or row['path_points']>80 or row['recovery_status']>9 or row['mapping_estimated']>1 or row['clock_valid']>1 or row['reserved']:
            raise ValueError('记录字段非法，可能不是对应版本的稳定快照')
        rows.append(row)
    stats={}
    for i,name in enumerate(STAGES):
        valid=sorted(r[name+'_us'] for r in rows if r['clock_valid'])
        stats[name]={'lifetime_max_us':h[3+i],
                     'window_valid_frames':len(valid),
                     'window_p99_us':valid[max(0,math.ceil(.99*len(valid))-1)] if valid else None,
                     'lifetime_mean_us':h[14+i]/frames if frames and rows and all(r['clock_valid'] for r in rows) else None}
    return rows,{'sequence':sequence,'frames':frames,'window_records':count,'stages':stats,
                 'note':'最近64帧p99；累计均值在计数回绕或时钟未就绪混入时不可作为有效性能结论；需采集端前后相同偶数sequence'}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input',type=Path)
    parser.add_argument('--output',type=Path,required=True,help='写入CSV，旁边另存JSON统计；已有文件拒绝覆盖')
    args=parser.parse_args()
    rows,stats=decode(args.input.read_bytes())
    summary=args.output.with_suffix('.json')
    if args.output.exists() or summary.exists():raise FileExistsError('输出已存在，请选择新的文件名')
    args.output.parent.mkdir(parents=True,exist_ok=True)
    fields=('frame_id','capture_time_ms','finish_us')+tuple(n+'_us' for n in STAGES)+('thin_iterations','path_points','recovery_status','mapping_estimated','clock_valid','reserved')
    with args.output.open('w',newline='',encoding='utf-8-sig') as f:
        writer=csv.DictWriter(f,fieldnames=fields);writer.writeheader();writer.writerows(rows)
    summary.write_text(json.dumps(stats,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps({'records':len(rows),'csv':str(args.output),'summary':str(summary)},ensure_ascii=True))


if __name__=='__main__':main()
