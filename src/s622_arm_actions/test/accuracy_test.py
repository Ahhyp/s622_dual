#!/usr/bin/env python3
"""自动测试 YOLO 反投影精度：在 4 个位置间移动方块，每位置采集 N 次。
用法: python3 accuracy_test.py [--samples 30]
"""
import sys
import time
import subprocess
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from yolov8_obb_msgs.msg import Yolov8Inference
from tf2_ros import Buffer, TransformListener
import tf_transformations as tft

TABLE_Z = 0.0
CUBE_TOP_Z = TABLE_Z + 0.04
N_SAMPLES = 30

# 4 个测试位置 (x, y)，z 固定 0.02（落地后中心高度）
TEST_POSITIONS = [
    (0.36, 0.02),   # 0.968m  known good
    (0.38, 0.10),   # 0.940m  conf~0.30
    (0.38, 0.00),   # 0.985m  conf~0.60
    (0.38, -0.02),  # 0.995m  conf~0.65
    (0.38, -0.08),  # 1.027m  conf~0.85
]

CAMERA_HFOV_DEG = 82.4
CAMERA_VFOV_DEG = 52.5


def move_cube(x, y, z=0.02):
    """用 ign service 移动方块到指定位置"""
    req = f"name: 'target_box', position: {{x: {x}, y: {y}, z: {z}}}"
    subprocess.run(
        ['ign', 'service', '-s', '/world/empty/set_pose',
         '--reqtype', 'ignition.msgs.Pose',
         '--reptype', 'ignition.msgs.Boolean',
         '--req', req, '--timeout', '2000'],
        capture_output=True, timeout=5)


class AccuracyTester(Node):
    def __init__(self, n_samples):
        super().__init__('accuracy_tester')
        self.n_samples = n_samples
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        self.K_img = None
        self.depth_img = None
        self.latest_det = None

        self.create_subscription(CameraInfo, '/camera/color/camera_info',
                                 self._on_info, 10)
        self.create_subscription(Image, '/camera/depth/image_raw',
                                 self._on_depth, 10)
        self.create_subscription(Yolov8Inference, '/yolov8/obb_detections',
                                 self._on_yolo, 10)

    def _on_info(self, msg):
        if self.K_img is None:
            self.K_img = np.array(msg.k).reshape(3, 3)

    def _on_depth(self, msg):
        if msg.encoding == '32FC1':
            self.depth_img = np.frombuffer(msg.data, dtype=np.float32).reshape(
                msg.height, msg.width)

    def _on_yolo(self, msg):
        if self.K_img is None or self.depth_img is None:
            return
        for r in msg.results:
            if r.class_name == 'cube' and r.confidence > 0.1:
                self.latest_det = (r.center_x, r.center_y, r.angle, r.confidence)
                break

    def _sample_depth(self, u, v, k=5):
        h, w = self.depth_img.shape[0], self.depth_img.shape[1]
        half = k // 2
        vals = []
        for dy in range(-half, half + 1):
            for dx in range(-half, half + 1):
                x, y = int(u) + dx, int(v) + dy
                if 0 <= x < w and 0 <= y < h:
                    z = self.depth_img[y, x]
                    if np.isfinite(z) and 0.01 < z < 5.0:
                        vals.append(z)
        return float(np.median(vals)) if vals else float('nan')

    def _cam_tf(self, camera_frame):
        t = self.tf.lookup_transform(
            'base_link', camera_frame, rclpy.time.Time(),
            rclpy.duration.Duration(seconds=0.3))
        trans = t.transform.translation
        rot = t.transform.rotation
        T = tft.quaternion_matrix([rot.x, rot.y, rot.z, rot.w])
        T[:3, 3] = [trans.x, trans.y, trans.z]
        return T

    def estimate_depth(self, u, v, fx, fy, cx, cy, camera_frame):
        z = self._sample_depth(u, v)
        if not np.isfinite(z):
            return None
        pc = np.array([(u - cx) * z / fx, (v - cy) * z / fy, z, 1.0])
        try:
            T = self._cam_tf(camera_frame)
        except Exception:
            return None
        pb = T @ pc
        return (pb[0], pb[1]), float(z)

    def estimate_plane(self, u, v, fx, fy, cx, cy, camera_frame):
        try:
            T = self._cam_tf(camera_frame)
        except Exception:
            return None
        cam_xyz = T[:3, 3]
        R = T[:3, :3]
        d_cam = np.array([(u - cx) / fx, (v - cy) / fy, 1.0])
        d_base = R @ d_cam
        if abs(d_base[2]) < 1e-6:
            return None
        t_param = (CUBE_TOP_Z - cam_xyz[2]) / d_base[2]
        if t_param <= 0:
            return None
        return (cam_xyz[0] + t_param * d_base[0],
                cam_xyz[1] + t_param * d_base[1]), float(t_param)

    def test_position(self, gt_x, gt_y):
        """采集一个位置的误差数据"""
        gt = np.array([gt_x, gt_y])
        depth_errs = []
        plane_errs = []
        depth_z_vals = []

        # 等方块稳定 + YOLO 看到它
        self.latest_det = None
        waited = 0
        while self.latest_det is None and waited < 30:
            rclpy.spin_once(self, timeout_sec=0.5)
            waited += 1
        if self.latest_det is None:
            self.get_logger().error(f'cube not detected at ({gt_x},{gt_y})')
            return None

        while len(depth_errs) < self.n_samples or len(plane_errs) < self.n_samples:
            rclpy.spin_once(self, timeout_sec=0.3)
            if self.latest_det is None:
                continue
            u, v, yaw, conf = self.latest_det
            fx, fy = self.K_img[0, 0], self.K_img[1, 1]
            cx, cy = self.K_img[0, 2], self.K_img[1, 2]
            cf = 'camera_color_optical_frame'

            if len(depth_errs) < self.n_samples:
                r = self.estimate_depth(u, v, fx, fy, cx, cy, cf)
                if r is not None:
                    (px, py), zc = r
                    depth_errs.append((px - gt[0], py - gt[1]))
                    depth_z_vals.append(zc)

            if len(plane_errs) < self.n_samples:
                r = self.estimate_plane(u, v, fx, fy, cx, cy, cf)
                if r is not None:
                    (px, py), _ = r
                    plane_errs.append((px - gt[0], py - gt[1]))

            self.latest_det = None

        return {'gt': (gt_x, gt_y),
                'depth': np.array(depth_errs),
                'plane': np.array(plane_errs),
                'z_cam_mean': float(np.mean(depth_z_vals))}


def print_position_result(r):
    gt = r['gt']
    print(f'\n位置 ({gt[0]:.2f}, {gt[1]:.2f}):')

    for name, key in [('深度传感器', 'depth'), ('桌面平面', 'plane')]:
        a = r[key]  # (N, 2)
        d = np.hypot(a[:, 0], a[:, 1])
        ex_m, ey_m = np.mean(a, axis=0)
        print(f'  {name}: 误差 {np.mean(d)*1000:.1f}±{np.std(d)*1000:.1f}mm  '
              f'ΔX={ex_m*1000:+.1f}mm ΔY={ey_m*1000:+.1f}mm')

    if 'z_cam_mean' in r:
        print(f'  深度 z_cam 均值: {r["z_cam_mean"]:.3f}m')


def main():
    ns = N_SAMPLES
    if '--samples' in sys.argv:
        ns = int(sys.argv[sys.argv.index('--samples') + 1])

    rclpy.init()
    node = AccuracyTester(ns)

    # 打印相机 FOV 估算
    print(f'相机 FOV: {CAMERA_HFOV_DEG}° × {CAMERA_VFOV_DEG}°')
    print(f'分辨率: 960×540')

    results = []
    for i, (x, y) in enumerate(TEST_POSITIONS):
        print(f'\n[{i+1}/{len(TEST_POSITIONS)}] 移动方块到 ({x:.2f}, {y:.2f})...')
        move_cube(x, y)
        time.sleep(1.0)
        r = node.test_position(x, y)
        if r:
            results.append(r)
            print_position_result(r)

    # 汇总
    print(f'\n{"="*55}')
    print('汇总分析')
    print(f'{"="*55}')
    for name, key in [('深度传感器', 'depth'), ('桌面平面', 'plane')]:
        all_errs = np.vstack([r[key] for r in results])  # (M, 2)
        d = np.hypot(all_errs[:, 0], all_errs[:, 1])
        ex_all, ey_all = all_errs[:, 0], all_errs[:, 1]
        print(f'\n{name} ({len(all_errs)} 样本, {len(results)} 位置):')
        print(f'  总体误差: {np.mean(d)*1000:.1f}±{np.std(d)*1000:.1f} mm')
        print(f'  X 偏差:   {np.mean(ex_all)*1000:+.1f}±{np.std(ex_all)*1000:.1f} mm')
        print(f'  Y 偏差:   {np.mean(ey_all)*1000:+.1f}±{np.std(ey_all)*1000:.1f} mm')

        # 判断误差模式
        per_pos = [np.hypot(r[key][:, 0], r[key][:, 1]) for r in results]
        means = [np.mean(p) for p in per_pos]
        if len(means) >= 2:
            gt_dists = [np.hypot(r['gt'][0], r['gt'][1]) for r in results]
            if max(means) - min(means) < 0.003:  # < 3mm 波动
                print(f'  模式: 恒定偏移（位置间均值波动 < 3mm）')
            else:
                print(f'  模式: 随位置变化（波动 {max(means)-min(means):.3f}m）')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()