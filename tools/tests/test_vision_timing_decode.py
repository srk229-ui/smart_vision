"""验证环形耗时记录的离线布局及无效快照拒绝。"""
from pathlib import Path
import sys
import unittest

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'tools'))
from decode_vision_timing import decode,HEADER,RECORD


class DecodeTests(unittest.TestCase):
    def snapshot(self,sequence=2,frames=70,next_slot=6):
        raw=bytearray(4240)
        HEADER.pack_into(raw,0,sequence,frames,next_slot,*([100]*11),*([7000]*11))
        for slot in range(64):
            frame=slot+64 if slot<6 else slot
            RECORD.pack_into(raw,144+slot*64,frame,frame*10,frame*20,*([frame]*11),10,20,1,1,1,0)
        return raw

    def test_ring_order_and_statistics(self):
        rows,stats=decode(self.snapshot())
        self.assertEqual([r['frame_id'] for r in rows],list(range(6,70)))
        self.assertEqual(stats['stages']['total']['window_p99_us'],69)
        self.assertEqual(stats['stages']['total']['lifetime_mean_us'],100)

    def test_empty_and_partial(self):
        self.assertEqual(decode(bytes(4240))[0],[])
        rows,_=decode(self.snapshot(frames=3,next_slot=3));self.assertEqual(len(rows),3)
        raw=self.snapshot(frames=1,next_slot=1);raw[144+62]=0
        _,stats=decode(raw)
        self.assertIsNone(stats['stages']['total']['window_p99_us'])
        self.assertIsNone(stats['stages']['total']['lifetime_mean_us'])

    def test_invalid_snapshots(self):
        for raw in [b'',self.snapshot(sequence=3),self.snapshot(next_slot=64)]:
            with self.assertRaises(ValueError):decode(raw)
        raw=self.snapshot();raw[144+6*64+62]=2
        with self.assertRaises(ValueError):decode(raw)


if __name__=='__main__':unittest.main()
