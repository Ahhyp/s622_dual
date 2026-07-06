#!/usr/bin/env python3
"""把"像素 + 深度图 + 相机内参"反投影到相机系 3D 点。
跟 yolov8_grasping/pose_estimator.py 几乎一样，
独立放一份是为了 visual_servo 包不依赖 yolov8_grasping。
"""
import numpy as np


class ErrorEstimator:
    def __init__(self):
        self.fx = self.fy = self.cx = self.cy = None

    def set_intrinsics(self, fx, fy, cx, cy):
        self.fx, self.fy, self.cx, self.cy = fx, fy, cx, cy

    def has_intrinsics(self) -> bool:
        return self.fx is not None

    def pixel_to_camera(self, u, v, depth_img, window=5):
        """像素 (u, v) → 相机光学系 (X, Y, Z) 米。失败返回 None。

        depth_img: numpy 数组，单位 mm（uint16）或 m（float32 自行换算）。
                   这里按 RealSense / Gazebo rgbd_camera 的常见 mm 处理。
        window: 取 (u,v) 周围 window × window 的中位数，抗深度空洞。
        """
        if self.fx is None:
            return None
        h, w = depth_img.shape
        u, v = int(round(u)), int(round(v))
        if not (0 <= u < w and 0 <= v < h):
            return None

        k = window // 2
        patch = depth_img[max(0, v - k):v + k + 1,
                          max(0, u - k):u + k + 1]
        valid = patch[patch > 0]
        if valid.size == 0:
            return None
        z_mm = float(np.median(valid))
        # Z = z_mm / 1000.0
        Z = z_mm # 这个 z_mm 的单位可能本来就是 m 而不是 mm

        X = (u - self.cx) * Z / self.fx
        Y = (v - self.cy) * Z / self.fy
        return np.array([X, Y, Z], dtype=np.float64)