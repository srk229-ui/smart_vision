"""离线生成鸟瞰像素索引；支持内参、畸变和H，不连接实机。"""
from pathlib import Path
import argparse
import hashlib
import json
import math
import numpy as np

ROOT = Path(__file__).resolve().parents[1]


def generate(camera):
    """格点先投影到原图，再保存最近邻像素索引及逐行显示宽度。"""
    if not isinstance(camera.get('source'), str) or not camera['source']:
        raise ValueError('必须说明映射来源')
    for key in ('height_m', 'forward_m', 'lateral_m', 'pitch_deg', 'yaw_deg', 'roll_deg', 'hfov_deg', 'road_width_m'):
        if isinstance(camera[key], bool) or not math.isfinite(camera[key]):
            raise ValueError('相机参数必须有限：' + key)
    if not .03 <= camera['height_m'] <= 2 or not 1 <= camera['pitch_deg'] <= 80 or not 25 <= camera['hfov_deg'] <= 140 or not .15 <= camera['road_width_m'] <= 1:
        raise ValueError('相机高度、俯角、视场或路宽越界')
    k = camera.get('intrinsics')
    fx = 94 / math.tan(math.radians(camera['hfov_deg'] / 2))
    fy, cx, cy = fx, 93.5, 59.5
    if k:
        fx, fy, cx, cy = (float(k[key]) for key in ('fx', 'fy', 'cx', 'cy'))
        if not all(math.isfinite(v) for v in (fx, fy, cx, cy)) or fx <= 0 or fy <= 0:
            raise ValueError('内参必须有限且焦距大于0')
    distortion = camera['distortion']
    if len(distortion) != 5 or not all(math.isfinite(v) and abs(v) <= 2 for v in distortion):
        raise ValueError('畸变需要五个有限系数')
    pitch, yaw, roll = map(math.radians, (camera['pitch_deg'], camera['yaw_deg'], camera['roll_deg']))
    right = np.array([0, -1, 0.])
    down = np.array([-math.sin(pitch), 0, -math.cos(pitch)])
    optical = np.array([math.cos(pitch), 0, -math.sin(pitch)])
    basis = np.column_stack((right*math.cos(roll)+down*math.sin(roll), -right*math.sin(roll)+down*math.cos(roll), optical))
    rz = np.array([[math.cos(yaw), -math.sin(yaw), 0], [math.sin(yaw), math.cos(yaw), 0], [0, 0, 1]])
    rotation = rz @ basis
    origin = np.array([camera['forward_m'], camera['lateral_m'], camera['height_m']])
    h = camera.get('homography_image_to_ground')
    if h is not None:
        h = np.asarray(h, float)
        if h.shape != (3, 3) or not np.isfinite(h).all() or abs(np.linalg.det(h)) < 1e-12 or any(distortion):
            raise ValueError('H需要有限可逆3×3矩阵，并使用已校正图像')
        inverse = np.linalg.inv(h)
    else:
        inverse = None

    def project(x, y):
        if inverse is not None:
            q = inverse @ np.array([x, y, 1.])
            if abs(q[2]) < 1e-8:
                return None
            u, v = q[:2] / q[2]
        else:
            q = rotation.T @ (np.array([x, y, 0.]) - origin)
            if q[2] <= 0:
                return None
            xx, yy = q[:2] / q[2]
            rr = xx*xx+yy*yy
            k1, k2, p1, p2, k3 = distortion
            radial = 1+k1*rr+k2*rr*rr+k3*rr*rr*rr
            u = fx*(xx*radial+2*p1*xx*yy+p2*(rr+2*xx*xx))+cx
            v = fy*(yy*radial+p1*(rr+2*yy*yy)+2*p2*xx*yy)+cy
        if not math.isfinite(u) or not math.isfinite(v) or not 0 <= u <= 187 or not 0 <= v <= 119:
            return None
        return int(math.floor(u+.5)), int(math.floor(v+.5))

    indices = []
    for row in range(116):
        for col in range(90):
            point = project(.06+.02*col, -1.15+.02*row)
            indices.append(65535 if point is None else point[1]*188+point[0])
    if sum(i != 65535 for i in indices) < 20:
        raise ValueError('映射没有足够可见地面格点，拒绝生成空表')
    # 显示宽度只用作重建边界，不冒充双边实测；H模式用中心附近横向点作尺度估算。
    widths = []
    for image_row in range(120):
        best = None
        for col in range(90):
            x = .06+.02*col
            mid, left, right_pixel = project(x, 0), project(x, camera['road_width_m']/2), project(x, -camera['road_width_m']/2)
            if mid and left and right_pixel:
                item = (abs(mid[1]-image_row), abs(right_pixel[0]-left[0]))
                if best is None or item[0] < best[0]:
                    best = item
        widths.append(min(187, max(3, best[1])) if best is not None else 80)
    return indices, widths


def geometry(camera):
    """生成无畸变像素与地面的双向矩阵；只在离线求逆，固件按点变换。"""
    fx = 94 / math.tan(math.radians(camera['hfov_deg'] / 2))
    fy, cx, cy = fx, 93.5, 59.5
    if camera.get('intrinsics'):
        fx, fy, cx, cy = (float(camera['intrinsics'][key]) for key in ('fx', 'fy', 'cx', 'cy'))
    intrinsics = [fx, fy, cx, cy]
    if camera.get('homography_image_to_ground') is not None:
        image_to_ground = np.asarray(camera['homography_image_to_ground'], float)
        ground_to_image = np.linalg.inv(image_to_ground)
    else:
        pitch, yaw, roll = map(math.radians, (camera['pitch_deg'], camera['yaw_deg'], camera['roll_deg']))
        right = np.array([0, -1, 0.])
        down = np.array([-math.sin(pitch), 0, -math.cos(pitch)])
        optical = np.array([math.cos(pitch), 0, -math.sin(pitch)])
        basis = np.column_stack((right*math.cos(roll)+down*math.sin(roll), -right*math.sin(roll)+down*math.cos(roll), optical))
        rz = np.array([[math.cos(yaw), -math.sin(yaw), 0], [math.sin(yaw), math.cos(yaw), 0], [0, 0, 1]])
        rotation = (rz @ basis).T
        origin = np.array([camera['forward_m'], camera['lateral_m'], camera['height_m']])
        k = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])
        ground_to_image = k @ np.column_stack((rotation[:, 0], rotation[:, 1], -rotation @ origin))
        image_to_ground = np.linalg.inv(ground_to_image)
    # 归一化只改齐次比例，不改坐标；避免绝对分母门槛依赖用户H的整体倍数。
    return image_to_ground / np.max(np.abs(image_to_ground)), ground_to_image / np.max(np.abs(ground_to_image)), intrinsics


def render_header(camera, source_hash='host_fixture'):
    """实机和宿主使用同一生成入口，路宽、采样表和双向几何保持一致。"""
    indices, widths = generate(camera)
    image_to_ground, ground_to_image, intrinsics = geometry(camera)
    estimated = int(not camera['source'].startswith('measured_'))
    def floats(values):
        return ','.join(format(float(v), '.9e')+'f' for v in np.asarray(values).ravel())
    lines = ['/* 离线生成的鸟瞰采样及双向地面变换；修改相机或路宽后重新生成。 */',
             '/* 来源：'+camera['source']+'；原配置SHA256：'+source_hash+'。 */',
             '#ifndef VISION_TRACK_MAP_H', '#define VISION_TRACK_MAP_H // 鸟瞰常量表重复包含保护。',
             '#define VISION_MAP_ESTIMATED ('+str(estimated)+'u) // 来源为估计时为1，不代表相机完成实测。',
             '#define VISION_MAP_INVALID (65535u) // uint16索引无效占位；原图最大索引22559。',
             '#define VISION_MAP_ROAD_WIDTH_M ('+floats([camera['road_width_m']])+') // 赛道法向宽度，米；单边向道路内偏移一半。',
             'static const float vision_map_image_to_ground[9] = {'+floats(image_to_ground)+'}; // 去畸变像素到地面X前/Y左的齐次矩阵。',
             'static const float vision_map_ground_to_image[9] = {'+floats(ground_to_image)+'}; // 地面到去畸变像素的齐次矩阵。',
             'static const float vision_map_intrinsics[4] = {'+floats(intrinsics)+'}; // fx/fy/cx/cy，像素；畸变换算使用。',
             'static const float vision_map_distortion[5] = {'+floats(camera['distortion'])+'}; // Brown系数k1/k2/p1/p2/k3；H模式必须全零。',
             'static const uint16_t vision_map_indices[VISION_TRACK_CELLS] = { // 原图行×188加列的最近邻索引。']
    lines += ['    '+','.join(map(str, indices[i:i+30]))+',' for i in range(0, len(indices), 30)]
    lines += ['};', 'static const uint8_t vision_map_widths[CONTROL_IMAGE_HEIGHT] = { // 估计直道像素宽度，只作双边白段筛选及显示。',
              '    '+','.join(map(str, widths)), '};', '#endif', '']
    return '\n'.join(lines)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--camera', type=Path, default=ROOT/'vision_config/estimated_camera.json')
    parser.add_argument('--output', type=Path, default=ROOT/'user/inc/vision_track_map.h')
    args = parser.parse_args()
    raw = args.camera.read_bytes()
    camera = json.loads(raw)
    args.output.write_text(render_header(camera, hashlib.sha256(raw).hexdigest()), encoding='utf-8', newline='\r\n')
    print(json.dumps({'road_width_m':camera['road_width_m'], 'mapping_source':camera['source'], 'output':str(args.output)}))


if __name__ == '__main__':
    main()
