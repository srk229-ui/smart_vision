"""用真实C布局样本及模拟只读传输验证停车记录导出，不连接探针。"""
from pathlib import Path
import struct
import sys
import tempfile
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import export_vision_stop as export


def fixture():
    raw=bytearray(export.TOTAL_BYTES)
    export.HEADER.pack_into(raw,0,0x31525356,2,1,len(raw),3,1,1,1,1500,1,3,9,3428,2820,1000,0)
    export.RECORD.pack_into(raw,64,9,1000,0,*([0]*11),0,0,8,1,0,0)
    export.VALUES.pack_into(raw,128,20.0,.5,92.5,0,0,0,0,0,0,1,0,120,120)
    raw[64+128:64+128+120]=bytes([20]*120)
    raw[64+248:64+248+120]=bytes([70]*120)
    raw[64+368:64+368+120]=bytes([120]*120)
    raw[64+488:64+488+120]=bytes([1]*120)
    raw[64+608:64+608+2820]=bytes([0x55]*2820)
    # 耗时快照为空，但头部仍是合法偶数代际。
    return bytes(raw)


class StopExportTests(unittest.TestCase):
    def test_layout_and_images(self):
        header,frames,gray,rows,stats=export.decode(fixture())
        self.assertEqual(header['count'],1)
        self.assertEqual(frames[0]['timing']['frame_id'],9)
        self.assertEqual(frames[0]['pixels'][:4],bytes([255,0,255,0]))
        self.assertEqual(len(gray),22560)

    def test_unstable_or_wrong_layout(self):
        for offset,value in [(4,3),(8,2),(20,0),(24,9),(28,8),(48,100),(52,1)]:
            raw=bytearray(fixture());struct.pack_into('<I',raw,offset,value)
            with self.assertRaises(ValueError): export.decode(raw)
        with self.assertRaises(ValueError): export.decode(fixture()[:-1])

    def test_read_only_stable_capture(self):
        raw=fixture();calls=[]
        def reader(address,size):
            calls.append((address,size));return raw[:size]
        self.assertEqual(export.read_stable(reader,0x20001000,len(raw)),raw)
        self.assertEqual(len(calls),3)

    def test_capture_changes_rejected(self):
        raw=fixture();calls=0
        def reader(address,size):
            nonlocal calls
            calls+=1
            result=bytearray(raw[:size])
            if calls==3:struct.pack_into('<I',result,4,4)
            return result
        with self.assertRaises(ValueError):export.read_stable(reader,0,len(raw))

    def test_artifacts_and_no_overwrite(self):
        base=Path(__file__).resolve().parents[2]/'docs/evidence/M3/first_curve_20261015/export_tests'
        base.mkdir(parents=True,exist_ok=True)
        with tempfile.TemporaryDirectory(dir=base) as folder:
            output=Path(folder)/'export'
            export.save(fixture(),output,'synthetic fixture')
            self.assertTrue((output/'00_overlay.png').is_file())
            self.assertEqual((output/'vision_stop.bin').read_bytes(),fixture())
            with self.assertRaises(FileExistsError):export.save(fixture(),output)


if __name__=='__main__':unittest.main()
