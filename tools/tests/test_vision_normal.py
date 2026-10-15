"""真实C法向恢复的合成几何验证；不连接实机，不以合成误差替代相机标定。"""
from pathlib import Path
import argparse
import copy
import ctypes as C
import json
import math
import sys
import unittest
import numpy as np
from PIL import Image

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'tools'))
sys.path.insert(0, str(ROOT/'full_track_sim'))
from run_vision_track_checks import compile_shared
from generate_vision_map import geometry, generate
from track_sim.camera import Camera
from track_sim.config import load

EVIDENCE = ROOT/'docs/evidence/M3/bev_normal_20261015/normal'
U8P = C.POINTER(C.c_uint8)
FP = C.POINTER(C.c_float)


class NormalTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.config = load()
        cls.camera = Camera(cls.config)
        cls.lib = cls.open_library(compile_shared(EVIDENCE/'host'))
        cls.metrics = []

    @staticmethod
    def open_library(path):
        lib = C.CDLL(str(path))
        lib.test_guard.argtypes = [U8P, C.c_uint32, C.c_uint32, FP, U8P]
        lib.test_ground.argtypes = [C.c_float, C.c_float, FP, FP]
        lib.test_image.argtypes = [C.c_float, C.c_float, FP, FP]
        return lib

    @classmethod
    def tearDownClass(cls):
        (EVIDENCE/'geometry_metrics.json').write_text(json.dumps({
            'synthetic_only': True, 'road_width_m': .45, 'hardware_tested': False,
            'cases': cls.metrics}, ensure_ascii=False, indent=2), encoding='utf-8')

    def guard(self, image, lib=None, reset=True, frame=1, ms=10):
        lib = lib or self.lib
        if reset:
            lib.test_reset()
        image = np.ascontiguousarray(image, np.uint8)
        meta, lines = np.zeros(4, np.float32), np.zeros((4, 120), np.uint8)
        status = lib.test_guard(image.ctypes.data_as(U8P), frame, ms, meta.ctypes.data_as(FP), lines.ctypes.data_as(U8P))
        return status, meta, lines

    def render(self, distance):
        image = np.where((np.abs(distance) <= .225) & self.camera.visible, 255, 0).astype(np.uint8)
        image[:42] = 0
        return image

    def save_overlay(self, name, image, lines):
        rgb = np.repeat(image[..., None], 3, axis=2)
        for row in np.flatnonzero(lines[3]):
            rgb[row, lines[1, row]] = (255, 128, 0) if lines[3, row] == 1 else (255, 0, 0)
        Image.fromarray(rgb).resize((752, 480), Image.Resampling.NEAREST).save(EVIDENCE/(name+'.png'))

    def test_01_sloped_left_right_vs_pixel_width(self):
        """0到65度的左右局部道路：恢复点到真实中心线的法向距离应小于2厘米。"""
        gx, gy = self.camera.ground[..., 0], self.camera.ground[..., 1]
        _, widths = generate(dict(self.config['camera'], road_width_m=.45))
        for angle, intercept in [(0, .15), (25, 0), (40, -.15), (55, -.25), (65, -.6)]:
            for mirror in (-1, 1):
                theta, d = math.radians(angle*mirror), intercept*mirror
                image = self.render(-math.sin(theta)*gx+math.cos(theta)*gy-d)
                status, meta, lines = self.guard(image)
                rows = np.flatnonzero(lines[3] == 1)
                self.assertTrue(meta[2], (angle, mirror, status, meta))
                self.assertGreaterEqual(len(rows), 15)
                errors = np.abs(-math.sin(theta)*gx[rows, lines[1, rows]]+math.cos(theta)*gy[rows, lines[1, rows]]-d)
                self.assertLess(float(errors.max()), .02)
                self.assertIn(status, (8, 9))
                # 教学对照使用当前估计直道像素宽度，不参与C控制或真实标定。
                old = np.where(lines[0, rows] == 0, lines[2, rows].astype(int)-np.array(widths)[rows]//2,
                               lines[0, rows].astype(int)+np.array(widths)[rows]//2)
                visible = (old >= 0) & (old < 188)
                old_error = np.abs(-math.sin(theta)*gx[rows[visible], old[visible]]+math.cos(theta)*gy[rows[visible], old[visible]]-d)
                record = {'type': 'line', 'angle_deg': angle*mirror, 'intercept_m': d, 'rows': len(rows),
                          'status': status, 'median_error_m': float(np.median(errors)), 'max_error_m': float(errors.max()),
                          'old_visible_rows': int(visible.sum()), 'old_median_error_m': float(np.median(old_error)) if len(old_error) else None}
                self.metrics.append(record)
                if angle >= 40:
                    self.assertGreater(len(old_error), 0)
                    self.assertLess(float(np.median(errors)), float(np.median(old_error))*.4)
                self.save_overlay(f'line_{angle}_{mirror}', image, lines)

    def test_02_r50_circles(self):
        """R50左右圆弯只向C提供图像，地图真值仅用于独立评分。"""
        gx, gy = self.camera.ground[..., 0], self.camera.ground[..., 1]
        for sign in (-1, 1):
            for cx in (.05, .15, .25):
                cy = sign*.5
                image = self.render(np.hypot(gx-cx, gy-cy)-.5)
                status, meta, lines = self.guard(image)
                rows = np.flatnonzero(lines[3] == 1)
                self.assertTrue(meta[2])
                self.assertGreaterEqual(len(rows), 25)
                errors = np.abs(np.hypot(gx[rows, lines[1, rows]]-cx, gy[rows, lines[1, rows]]-cy)-.5)
                self.assertLess(float(errors.max()), .02)
                self.metrics.append({'type': 'circle', 'radius_m': .5, 'center': [cx, cy], 'rows': len(rows),
                                     'status': status, 'median_error_m': float(np.median(errors)), 'max_error_m': float(errors.max())})
                self.save_overlay(f'r50_{sign}_{cx}', image, lines)

    def test_03_no_history_expiry_and_duplicate(self):
        image = np.zeros((120, 188), np.uint8)
        image[42:, :81] = 255
        first = self.guard(image)
        later = self.guard(image, reset=False, frame=2, ms=10000)
        self.assertEqual(first[0], 8)
        self.assertEqual(later[0], 8)
        np.testing.assert_array_equal(first[2], later[2])
        repeated = self.guard(np.zeros_like(image), reset=False, frame=2, ms=99999)
        self.assertEqual(repeated[0], 5)
        np.testing.assert_array_equal(later[2], repeated[2])

    def test_04_reject_unknown_border_short_edge_and_outside_center(self):
        for image in [np.full((120, 188), 255, np.uint8), np.zeros((120, 188), np.uint8)]:
            _, meta, lines = self.guard(image)
            self.assertFalse(meta[2])
            self.assertFalse(np.any(lines[3]))
        image = np.zeros((120, 188), np.uint8)
        image[100:103, :81] = 255  # 仅三个像素行，地面跨度不足4厘米。
        _, meta, lines = self.guard(image)
        self.assertFalse(meta[2])
        self.assertFalse(np.any(lines[3] == 1))
        image[:] = 0
        image[42:, :5] = 255  # 半宽偏移后中心在画面外，骨架也不得对裁剪道路补假中线。
        _, meta, lines = self.guard(image)
        self.assertFalse(meta[2])
        self.assertFalse(np.any(lines[3] == 1))

    def test_05_discontinuity_not_joined(self):
        image = np.zeros((120, 188), np.uint8)
        image[90:, :81] = 255
        image[42:84, 100:] = 255  # 六行断口后出现另一侧白块，不能接到近端道路。
        _, _, lines = self.guard(image)
        self.assertFalse(np.any(lines[3, :84]))

    def test_06_bidirectional_mapping_intrinsics_rotation_distortion_h(self):
        variants = [('default', copy.deepcopy(self.config['camera']))]
        rotated = copy.deepcopy(self.config['camera'])
        rotated.update(yaw_deg=3, roll_deg=2, distortion=[.03, -.01, .002, -.003, .001],
                       intrinsics={'fx': 115, 'fy': 110, 'cx': 93, 'cy': 61})
        variants.append(('rotated', rotated))
        hcamera = copy.deepcopy(self.config['camera'])
        hcamera['homography_image_to_ground'] = geometry(dict(hcamera, road_width_m=.45))[0].tolist()
        variants.append(('homography', hcamera))
        for name, camera in variants:
            cfg = copy.deepcopy(self.config)
            cfg['camera'] = camera
            reference = Camera(cfg)
            lib = self.lib if name == 'default' else self.open_library(compile_shared(EVIDENCE/name, dict(camera, road_width_m=.45)))
            for v in (50, 70, 90, 110):
                for u in (30, 70, 110, 150):
                    x, y = C.c_float(), C.c_float()
                    valid = lib.test_ground(u, v, C.byref(x), C.byref(y))
                    expected = reference.ground[v, u]
                    if not valid:
                        self.assertTrue(not (.06 <= expected[0] <= 1.84 and abs(expected[1]) <= 1.15))
                        continue
                    np.testing.assert_allclose([x.value, y.value], expected, atol=5e-4)
                    pu, pv = C.c_float(), C.c_float()
                    self.assertTrue(lib.test_image(x.value, y.value, C.byref(pu), C.byref(pv)))
                    np.testing.assert_allclose([pu.value, pv.value], [u, v], atol=.06)
            self.assertFalse(lib.test_ground(float('nan'), 90, C.byref(x), C.byref(y)))
            self.assertFalse(lib.test_image(0.5, float('nan'), C.byref(x), C.byref(y)))

    def test_07_configurable_metric_width(self):
        image = np.zeros((120, 188), np.uint8)
        image[42:, :111] = 255
        wider = dict(self.config['camera'], road_width_m=.5)
        lib = self.open_library(compile_shared(EVIDENCE/'width_050_fixture_only', wider))
        _, _, nominal = self.guard(image)
        _, _, changed = self.guard(image, lib=lib)
        self.assertEqual(nominal[3, 90], 1)
        self.assertEqual(changed[3, 90], 1)
        self.assertLess(changed[1, 90], nominal[1, 90])

    def test_08_adaptive_band_in_both_directions(self):
        self.lib.test_band.argtypes = [U8P, U8P, U8P]
        for fixed, valid, expected in [([60, 65, 70, 75, 80], (72, 101), 78),
                                       ([80, 85, 90, 95, 100], (55, 79), 72)]:
            quality = np.zeros(120, np.uint8)
            quality[slice(*valid)] = 1
            rows = np.array(fixed, np.uint8)
            selected = np.zeros(5, np.uint8)
            self.assertEqual(self.lib.test_band(quality.ctypes.data_as(U8P), rows.ctypes.data_as(U8P), selected.ctypes.data_as(U8P)), 2)
            self.assertEqual(int(selected[2]), expected)
            self.assertTrue(np.all(quality[selected]))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--evidence-dir', type=Path)
    args, remaining = parser.parse_known_args()
    if args.evidence_dir:
        EVIDENCE = args.evidence_dir.resolve()
    EVIDENCE.mkdir(parents=True, exist_ok=True)
    unittest.main(argv=[sys.argv[0], *remaining], verbosity=2)
