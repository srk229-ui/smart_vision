"""对照仿真投影验证离线索引，不以宿主结果冒充相机实测。"""
from pathlib import Path
import sys
import unittest
import copy
import numpy as np

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'tools'))
sys.path.insert(0,str(ROOT/'full_track_sim'))
from generate_vision_map import generate
from track_sim.camera import Camera
from track_sim.config import load,feasible_preset


class MapTests(unittest.TestCase):
    def compare(self,config):
        camera=Camera(config)
        indices,widths=generate(dict(config['camera'],road_width_m=.45))
        valid=camera.bev_valid.ravel()
        expected=np.full(116*90,65535,dtype=np.int64)
        u=np.floor(camera.bev_u.ravel()[valid]+.5).astype(int)
        v=np.floor(camera.bev_v.ravel()[valid]+.5).astype(int)
        expected[valid]=v*188+u
        np.testing.assert_array_equal(indices,expected)
        self.assertTrue(all(3<=w<=187 for w in widths))

    def test_estimated_and_hypothetical(self):
        self.compare(load());self.compare(feasible_preset())

    def test_distortion_and_rotation(self):
        cfg=load();cfg['camera'].update(distortion=[.03,-.01,.002,-.003,.001],yaw_deg=3,roll_deg=2)
        self.compare(cfg)

    def test_intrinsics(self):
        cfg=load();cfg['camera']['intrinsics']={'fx':115,'fy':110,'cx':93,'cy':61}
        self.compare(cfg)

    def test_homography(self):
        cfg=load();cam=Camera(cfg)
        k=np.array([[cam.fx,0,cam.cx],[0,cam.fy,cam.cy],[0,0,1]])
        rotation=cam.basis.T
        inverse=k@np.column_stack((rotation[:,0],rotation[:,1],-rotation@cam.origin))
        cfg['camera']['homography_image_to_ground']=np.linalg.inv(inverse).tolist()
        self.compare(cfg)

    def test_invalid_inputs(self):
        base=dict(load()['camera'],road_width_m=.45)
        for patch in [{'height_m':0},{'pitch_deg':float('nan')},{'source':''},
                      {'homography_image_to_ground':[[0]*3]*3},{'distortion':[1,2]},
                      {'intrinsics':{'fx':0,'fy':110,'cx':94,'cy':60}}]:
            cfg=copy.deepcopy(base);cfg.update(patch)
            with self.assertRaises(ValueError):generate(cfg)


if __name__=='__main__':unittest.main()
