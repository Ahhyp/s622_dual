#!/usr/bin/env python3
"""
标定数据采集脚本：采集 5+ 个位置的坐标转换数据。

每行 CSV 包含:
  - spawn 位置 (x,y,z)
  - GT 实际中心 (actual_x, actual_y, actual_z)
  - YOLO 检测像素、深度、cam/base 坐标
  - 相机 TF (tx,ty,tz,qx,qy,qz,qw) — 用于事后检查相机位姿是否变化
  - 各 sensor 的时间戳 — 用于检查同步
  - E2b: ray-box 理论表面点 vs depth 表面点 (当场同步验证)

用法：
  1. 启动仿真: ros2 launch gz_launch s622_gazebo.launch.py
  2. 等 YOLO 和 visual_servo_node 就绪
  3. 新终端: python3 collect_calib_data.py

输出：
  /home/yep/my_S622/debug_data/calib_data_current_camera.csv
"""

import sys
import time
import math
import subprocess
from typing import Any, Optional, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs
from cv_bridge import CvBridge
from yolov8_obb_msgs.msg import Yolov8Inference


# ============================================================
# 测试位置 — 臂可达范围内的 7 个位置
# 方块 yaw=0 (轴对齐), 验证阶段先不用旋转
# ============================================================
TEST_POSITIONS = [
    # (x, y, z_spawn, label)
    (0.35, 0.00, 0.05, "中心"),
    (0.30, 0.15, 0.05, "近臂左"),
    (0.42, 0.20, 0.05, "远臂右"),
    (0.30, -0.20, 0.05, "近臂右"),
    (0.40, -0.15, 0.05, "远臂左"),
    (0.38, 0.08, 0.05, "中左"),
    (0.38, -0.08, 0.05, "中右"),
]

DEPTH_WINDOW = 5  # 调试阶段用 5×5，避免混入边缘/背景

# ─── 方块参数（与 verify_coordinate_chain 一致）───
BLOCK_HALF_X = 0.020  # 半边长 (4cm 立方体)
BLOCK_HALF_Y = 0.020
BLOCK_HALF_Z = 0.020

# ─── E2b 容差 ───
E2B_Z_TOL_MM = 10.0    # depth Z 与理论 Z 的容差 (mm)
E2B_SURF_TOL_MM = 10.0 # 表面点 3D 距离容差 (mm)


def quat_to_rot(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    """quaternion → 3×3 rotation matrix"""
    return np.array([
        [1 - 2*qy*qy - 2*qz*qz,     2*qx*qy - 2*qz*qw,     2*qx*qz + 2*qy*qw],
        [2*qx*qy + 2*qz*qw,     1 - 2*qx*qx - 2*qz*qz,     2*qy*qz - 2*qx*qw],
        [2*qx*qz - 2*qy*qw,     2*qy*qz + 2*qx*qw,     1 - 2*qx*qx - 2*qy*qy],
    ], dtype=float)


class CalibDataCollector(Node):
    def __init__(self):
        super().__init__("calib_data_collector")

        self.base_frame = "base_link"
        self.camera_frame = "camera_color_optical_frame"

        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 内参
        self.fx: Optional[float] = None
        self.fy: Optional[float] = None
        self.cx: Optional[float] = None
        self.cy: Optional[float] = None

        # 数据缓存
        self.depth_img: Optional[np.ndarray] = None
        self.depth_header_stamp: Any = None  # 原始 Header.stamp, 供 transform 查询
        self.latest_det: Optional[dict] = None
        self.det_cb_count = 0  # 诊断: 回调被调用次数

        # 时间戳缓存 (记录最新收到的各 sensor 时间戳)
        self.rgb_stamp: Optional[float] = None       # sec
        self.depth_stamp: Optional[float] = None     # sec
        self.detection_stamp: Optional[float] = None  # sec

        # 相机 TF 缓存: camera_color_optical_frame → base_link
        # 即 T_base_camera, translation 是 camera 原点在 base_link 系的位置
        self.cam_tx: Optional[float] = None
        self.cam_ty: Optional[float] = None
        self.cam_tz: Optional[float] = None
        self.cam_qx: Optional[float] = None
        self.cam_qy: Optional[float] = None
        self.cam_qz: Optional[float] = None
        self.cam_qw: Optional[float] = None

        # 矩阵形式（供 E2b 投影/反投影/ray-box 使用）
        self.cam_R: Optional[np.ndarray] = None   # 3×3, camera → base
        self.cam_t: Optional[np.ndarray] = None    # 3, camera 原点在 base 系

        # ROS 接口
        self.create_subscription(CameraInfo, "/camera/color/camera_info",
                                 self._cb_info, 10)
        self.create_subscription(Image, "/camera/color/image_raw",
                                 self._cb_rgb, qos_profile_sensor_data)
        self.create_subscription(Image, "/camera/depth/image_raw",
                                 self._cb_depth, qos_profile_sensor_data)
        self.create_subscription(Yolov8Inference, "/yolov8/obb_detections",
                                 self._cb_det, 10)

        self.get_logger().info("数据采集节点就绪，等待 YOLO 检测...")

    def _cb_info(self, msg: CameraInfo):
        self.fx = float(msg.k[0])
        self.fy = float(msg.k[4])
        self.cx = float(msg.k[2])
        self.cy = float(msg.k[5])

    def _cb_rgb(self, msg: Image):
        self.rgb_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def _cb_depth(self, msg: Image):
        self.depth_img = self.bridge.imgmsg_to_cv2(msg, "passthrough")
        self.depth_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.depth_header_stamp = msg.header.stamp

    def _cb_det(self, msg: Yolov8Inference):
        self.det_cb_count += 1
        self.detection_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if len(msg.results) == 0:
            self.latest_det = None
            return
        # 取置信度最高的（多目标场景比 results[0] 更稳）
        r = max(msg.results, key=lambda x: x.confidence)
        self.latest_det = {
            "u": float(r.center_x),
            "v": float(r.center_y),
            "conf": float(r.confidence),
            "class": str(r.class_name),
            "angle": float(r.angle),
        }

    def _lookup_camera_tf(self) -> bool:
        """查询并缓存 camera_frame → base_link 的 TF（分量 + 矩阵）"""
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame, self.camera_frame,
                rclpy.time.Time(), timeout=Duration(seconds=0.5))
        except Exception:
            return False
        t = tf.transform.translation
        q = tf.transform.rotation
        self.cam_tx = float(t.x)
        self.cam_ty = float(t.y)
        self.cam_tz = float(t.z)
        self.cam_qx = float(q.x)
        self.cam_qy = float(q.y)
        self.cam_qz = float(q.z)
        self.cam_qw = float(q.w)
        # 矩阵形式
        self.cam_R = quat_to_rot(q.x, q.y, q.z, q.w)
        self.cam_t = np.array([t.x, t.y, t.z], dtype=float)
        return True

    # ----------------------------------------------------------------
    def pixel_to_camera(self, u: float, v: float) -> Optional[np.ndarray]:
        if self.fx is None or self.depth_img is None:
            return None
        h, w = self.depth_img.shape
        ui, vi = int(round(u)), int(round(v))
        if not (0 <= ui < w and 0 <= vi < h):
            return None
        k = DEPTH_WINDOW // 2
        patch = self.depth_img[max(0, vi - k):vi + k + 1,
                                max(0, ui - k):ui + k + 1]
        valid = patch[np.isfinite(patch) & (patch > 0)]
        if valid.size == 0:
            return None
        z = float(np.median(valid))
        x = (u - self.cx) * z / self.fx
        y = (v - self.cy) * z / self.fy
        return np.array([x, y, z], dtype=float)

    def transform_to_base(self, xyz_cam: np.ndarray) -> Optional[np.ndarray]:
        pt = PointStamped()
        pt.header.frame_id = self.camera_frame
        # 用 depth 图时间戳做 TF 查询，而非 rclpy.time.Time()（静态相机下影响不大但更规范）
        if self.depth_header_stamp is not None:
            pt.header.stamp = self.depth_header_stamp
        else:
            pt.header.stamp = rclpy.time.Time().to_msg()
        pt.point.x = float(xyz_cam[0])
        pt.point.y = float(xyz_cam[1])
        pt.point.z = float(xyz_cam[2])
        try:
            pt_base = self.tf_buffer.transform(
                pt, self.base_frame, timeout=Duration(seconds=0.5))
        except Exception as e:
            return None
        return np.array([pt_base.point.x, pt_base.point.y, pt_base.point.z])

    # ─── E2b 投影/反投影/ray-box 工具 ─────────────────────
    def camera_to_base_mat(self, P_cam: np.ndarray) -> np.ndarray:
        """camera → base (矩阵版): P_base = R @ P_cam + t"""
        return self.cam_R @ P_cam + self.cam_t

    def base_to_camera(self, P_base: np.ndarray) -> np.ndarray:
        """base → camera (矩阵逆): P_cam = R^T @ (P_base - t)"""
        return self.cam_R.T @ (P_base - self.cam_t)

    def camera_to_pixel(self, P_cam: np.ndarray) -> Optional[np.ndarray]:
        """camera_optical_frame 3D → 像素 (u,v)"""
        x, y, z = P_cam[0], P_cam[1], P_cam[2]
        if z <= 1e-6:
            return None
        u = self.fx * x / z + self.cx
        v = self.fy * y / z + self.cy
        return np.array([u, v], dtype=float)

    def pixel_depth_to_camera_static(self, u: float, v: float,
                                      depth: float) -> np.ndarray:
        """像素 (u,v) + 给定深度 → camera_optical_frame 3D（不读 depth 图）"""
        X = (u - self.cx) * depth / self.fx
        Y = (v - self.cy) * depth / self.fy
        return np.array([X, Y, depth], dtype=float)

    def ray_box_intersection_depth_z(
        self,
        u: float,
        v: float,
        box_center: np.ndarray,
        half_extents: np.ndarray,
        eps: float = 1e-9,
    ) -> Optional[Tuple[float, np.ndarray]]:
        """
        像素射线与 base 系下轴对齐方块 AABB 求交。

        参数化: P_cam(Z) = Z * [(u-cx)/fx, (v-cy)/fy, 1]
                P_base(Z) = cam_t + cam_R @ P_cam(Z)

        返回 (Z_entry, P_entry_base) 或 None。
        """
        r_cam = np.array([
            (u - self.cx) / self.fx,
            (v - self.cy) / self.fy,
            1.0,
        ], dtype=float)

        O = self.cam_t
        d = self.cam_R @ r_cam

        box_min = box_center - half_extents
        box_max = box_center + half_extents

        z_min = -np.inf
        z_max = np.inf

        for i in range(3):
            if abs(d[i]) < eps:
                if O[i] < box_min[i] or O[i] > box_max[i]:
                    return None
            else:
                z1 = (box_min[i] - O[i]) / d[i]
                z2 = (box_max[i] - O[i]) / d[i]
                z_near = min(z1, z2)
                z_far = max(z1, z2)
                z_min = max(z_min, z_near)
                z_max = min(z_max, z_far)
                if z_min > z_max:
                    return None

        if z_max <= 0:
            return None

        Z_entry = z_min if z_min > 0 else z_max
        P_entry_base = O + Z_entry * d
        return (Z_entry, P_entry_base)

    def compute_e2b(self, P_gt: np.ndarray) -> dict:
        """
        在当前 live depth 图上对 GT 中心像素执行 E2b 验证。

        返回 dict 包含所有 E2b 字段，或 {"error": ...}。
        """
        if self.cam_R is None or self.fx is None or self.depth_img is None:
            return {"error": "not ready"}

        # GT → pixel
        P_cam_gt = self.base_to_camera(P_gt)
        uv_gt = self.camera_to_pixel(P_cam_gt)
        if uv_gt is None:
            return {"error": "GT behind camera"}

        u_gt, v_gt = uv_gt[0], uv_gt[1]
        h, w = self.depth_img.shape
        ui, vi = int(round(u_gt)), int(round(v_gt))
        if not (0 <= ui < w and 0 <= vi < h):
            return {"error": "pixel out of bounds"}

        # depth at GT pixel
        k = DEPTH_WINDOW // 2
        patch = self.depth_img[max(0, vi - k):vi + k + 1,
                                max(0, ui - k):ui + k + 1]
        valid = patch[np.isfinite(patch) & (patch > 0)]
        if valid.size == 0:
            return {"error": "no valid depth"}

        Z_depth = float(np.median(valid))

        # ── 诊断: 5×5 patch 内 depth 分布 ──
        Z_center = float(self.depth_img[vi, ui])
        Z_min = float(np.min(valid))
        Z_p10 = float(np.percentile(valid, 10))
        Z_p90 = float(np.percentile(valid, 90))

        print(f"  [E2b-patch] ({ui},{vi}) "
              f"center={Z_center:.4f} min={Z_min:.4f} "
              f"p10={Z_p10:.4f} median={Z_depth:.4f} p90={Z_p90:.4f} "
              f"n_valid={valid.size}/{patch.size}")

        P_cam_surface = self.pixel_depth_to_camera_static(
            float(ui), float(vi), Z_depth)
        P_surface = self.camera_to_base_mat(P_cam_surface)

        # ray-box expected
        half_extents = np.array([BLOCK_HALF_X, BLOCK_HALF_Y, BLOCK_HALF_Z])
        ray_hit = self.ray_box_intersection_depth_z(
            u_gt, v_gt, P_gt, half_extents)

        if ray_hit is None:
            print(f"  [E2b] GT→pixel=({u_gt:.0f},{v_gt:.0f}) "
                  f"ray MISS box")
            return {
                "error": None,
                "gt_u": u_gt, "gt_v": v_gt,
                "z_depth_at_gt": Z_depth,
                "z_expected": float("nan"),
                "depth_minus_expected_mm": float("nan"),
                "surface_err_mm": float("nan"),
                "surface_inside_box": False,
                "ray_hit_box": False,
                "z_p10": Z_p10,
                "z_center": Z_center,
                "z_min": Z_min,
                "n_valid": valid.size,
            }

        Z_expected, P_expected = ray_hit
        depth_err_mm = (Z_depth - Z_expected) * 1000.0
        surface_err_mm = float(np.linalg.norm(P_surface - P_expected)) * 1000.0

        # inside block check (with tolerance)
        diff = P_surface - P_gt
        tol = 0.003
        inside = (abs(diff[0]) <= BLOCK_HALF_X + tol and
                  abs(diff[1]) <= BLOCK_HALF_Y + tol and
                  abs(diff[2]) <= BLOCK_HALF_Z + tol)

        Z_OK = abs(depth_err_mm) < E2B_Z_TOL_MM
        surf_OK = surface_err_mm < E2B_SURF_TOL_MM

        p10_err_mm = (Z_p10 - Z_expected) * 1000.0
        p10_OK = abs(p10_err_mm) < E2B_Z_TOL_MM

        print(f"  [E2b] GT→pixel=({u_gt:.0f},{v_gt:.0f}) "
              f"Z_expected={Z_expected:.4f}m "
              f"ΔZ_median={depth_err_mm:+.1f}mm {'✅' if Z_OK else '❌'} "
              f"ΔZ_p10={p10_err_mm:+.1f}mm {'✅' if p10_OK else '❌'} "
              f"surf_err={surface_err_mm:.1f}mm {'✅' if surf_OK else '❌'} "
              f"inside={inside}")

        return {
            "error": None,
            "gt_u": u_gt, "gt_v": v_gt,
            "z_depth_at_gt": Z_depth,
            "z_expected": Z_expected,
            "depth_minus_expected_mm": depth_err_mm,
            "surface_err_mm": surface_err_mm,
            "surface_inside_box": inside,
            "ray_hit_box": True,
            "z_p10": Z_p10,
            "z_center": Z_center,
            "z_min": Z_min,
            "n_valid": valid.size,
        }

    # ----------------------------------------------------------------
    def wait_for_detection(self, timeout: float = 30.0) -> Optional[dict]:
        """等待稳定检测，返回 (u, v, conf)"""
        self.get_logger().info(f"  等待 YOLO 检测 (超时={timeout:.0f}s)...")
        self.latest_det = None
        start = time.time()
        stable_count = 0
        last_det = None
        last_progress = 0

        while time.time() - start < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)

            det = self.latest_det
            elapsed = time.time() - start

            # 每秒输出一次进度
            if int(elapsed) > last_progress:
                self.get_logger().info(
                    f"  [{int(elapsed)}s] det_cb={self.det_cb_count} "
                    f"det={'YES' if det else 'NO'}",
                    throttle_duration_sec=0.9,
                )
                last_progress = int(elapsed)

            if det is None:
                stable_count = 0
                last_det = None
                continue

            if last_det is None:
                last_det = det
                stable_count = 1
            else:
                dist = math.hypot(det["u"] - last_det["u"],
                                  det["v"] - last_det["v"])
                if dist < 30 and det["conf"] > 0.10:
                    stable_count += 1
                    last_det = det
                else:
                    stable_count = 0
                    last_det = det

            if stable_count >= 3:
                self.get_logger().info(
                    f"  ✅ 检测稳定: uv=({det['u']:.1f},{det['v']:.1f}) "
                    f"conf={det['conf']:.3f}")
                return det

        self.get_logger().warning(f"  ❌ 检测超时 ({timeout:.0f}s)!")
        return None


import os as _os
import re as _re
_MODEL_SDF = _os.path.expanduser(
    "~/my_S622/src/gz_launch/models/target_box/model.sdf")


def _get_gazebo_model_pose(name: str = "target_box") -> Optional[dict]:
    """
    查询 Gazebo 中模型的实际 6-DoF pose。

    调用 ign model -m <name> --pose 并解析输出。
    Gazebo Fortress 输出类似:
      Model: [42]
        ...
        Pose [XYZ (m), RPY (rad)]:
          x: 0.350 y: 0.000 z: 0.050
          roll: 0.000 pitch: 0.000 yaw: 0.000

    返回 {"x":..., "y":..., "z":..., "roll":..., "pitch":..., "yaw":...} 或 None。
    """
    r = subprocess.run(
        ["ign", "model", "-m", name, "--pose"],
        timeout=10, capture_output=True, text=True)
    out = r.stdout
    # Gazebo Fortress 格式:
    #   Pose [ XYZ (m) ] [ RPY (rad) ]:
    #     [0.350000 0.000000 0.050000]
    #     [0.000000 -0.000000 0.000000]
    m = _re.search(
        r'\[\s*([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)\s*\]\s*'
        r'\[\s*([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)\s*\]', out)
    if not m:
        return None
    return {
        "x": float(m.group(1)),
        "y": float(m.group(2)),
        "z": float(m.group(3)),
        "roll": float(m.group(4)),
        "pitch": float(m.group(5)),
        "yaw": float(m.group(6)),
    }


def _get_entity_id(name: str = "target_box") -> int:
    """查询 entity ID，Gazebo Fortress 的 remove 必须用 ID 不能用 name"""
    r = subprocess.run(
        ["ign", "model", "-m", name, "--pose"],
        timeout=10, capture_output=True, text=True)
    m = _re.search(r'Model:\s*\[(\d+)\]', r.stdout)
    if m:
        return int(m.group(1))
    return -1


def spawn_box(x: float, y: float, z: float, name: str = "target_box"):
    """Spawn target_box at given position."""
    cmd = [
        "ros2", "run", "ros_gz_sim", "create",
        "-world", "empty",
        "-file", _MODEL_SDF,
        "-name", name,
        "-x", str(x), "-y", str(y), "-z", str(z),
        "-R", "0", "-P", "0", "-Y", "0",
    ]
    try:
        r = subprocess.run(cmd, timeout=15, capture_output=True, text=True)
        merged = (r.stdout + r.stderr)
        if r.returncode != 0:
            print(f"  spawn 失败 (rc={r.returncode}): {merged.strip()[:200]}")
            return False
        # ROS 2 logging goes to stderr
        if "OK creation" not in merged:
            print(f"  spawn 返回非预期: {merged.strip()[:200]}")
            return False
        return True
    except Exception as e:
        print(f"  spawn 异常: {e}")
        return False


def remove_box(name: str = "target_box"):
    """Remove entity from Gazebo by entity ID (name-based removal is buggy in Fortress 6.x)"""
    eid = _get_entity_id(name)
    if eid < 0:
        return  # 不存在，跳过
    subprocess.run(
        ["ign", "service", "-s", "/world/empty/remove",
         "--reqtype", "ignition.msgs.Entity",
         "--reptype", "ignition.msgs.Boolean",
         "--timeout", "5000",
         "--req", f"id: {eid}"],
        timeout=10, capture_output=True, text=True)


def compute_actual(spawn_x, spawn_y, spawn_z):
    """4cm 立方体落地后中心: z = 0 (地面) + 0.02 (半高)"""
    return (spawn_x, spawn_y, 0.020)


def main():
    rclpy.init()
    collector = CalibDataCollector()

    # 等待内参和深度图就绪
    print("=" * 70)
    print("标定数据采集")
    print("=" * 70)
    print("等待相机数据就绪...")
    start = time.time()
    while collector.fx is None or collector.depth_img is None:
        rclpy.spin_once(collector, timeout_sec=0.1)
        if time.time() - start > 15:
            print("错误: 等待相机数据超时。请确认仿真已启动。")
            return
    print(f"内参: fx={collector.fx:.1f}, fy={collector.fy:.1f}, "
          f"cx={collector.cx:.1f}, cy={collector.cy:.1f}")
    print(f"深度图: shape={collector.depth_img.shape}")

    # 输出 CSV 头
    csv_path = "/home/yep/my_S622/debug_data/calib_data_current_camera.csv"
    with open(csv_path, "w") as f:
        f.write("label,spawn_x,spawn_y,spawn_z,"
                "actual_x,actual_y,actual_z,"
                "pixel_u,pixel_v,conf,"
                "depth_m,"
                "cam_x,cam_y,cam_z,"
                "base_x,base_y,base_z,"
                "err_x_mm,err_y_mm,err_z_mm,err_norm_mm,"
                "camera_tx,camera_ty,camera_tz,"
                "camera_qx,camera_qy,camera_qz,camera_qw,"
                "rgb_stamp,depth_stamp,detection_stamp,"
                "gt_u,gt_v,"
                "z_depth_at_gt,z_expected,"
                "depth_minus_expected_mm,surface_err_mm,"
                "surface_inside_box,ray_hit_box\n")

    results = []

    last_pixel = None  # 跟踪上一个位置的检测像素，避免读到旧数据

    for i, (sx, sy, sz, label) in enumerate(TEST_POSITIONS):
        print(f"\n--- [{i+1}/{len(TEST_POSITIONS)}] {label}: "
              f"spawn=({sx:.2f}, {sy:.2f}, {sz:.2f}) ---")

        # 清理旧方块
        remove_box()
        time.sleep(1.5)

        # Spawn 新方块
        if not spawn_box(sx, sy, sz):
            print("  跳过")
            continue

        # 等方块落地 + 相机拍到新画面 + YOLO 推理
        print("  等待方块渲染 + YOLO 识别 (8s)...")
        time.sleep(8.0)

        # 查询 Gazebo 内部模型实际 pose
        gz_pose = _get_gazebo_model_pose()
        if gz_pose:
            gx, gy, gz = gz_pose["x"], gz_pose["y"], gz_pose["z"]
            print(f"  Gazebo pose: ({gx:.4f}, {gy:.4f}, {gz:.4f})"
                  f"  yaw={gz_pose.get('yaw', 0):.4f}")
            if (abs(gx - sx) > 0.001 or abs(gy - sy) > 0.001
                    or abs(gz - sz) > 0.001):
                print(f"  ⚠️ Gazebo pose ≠ spawn ({sx},{sy},{sz})! "
                      f"Δ=({(gx-sx)*1000:+.1f},{(gy-sy)*1000:+.1f},{(gz-sz)*1000:+.1f})mm")
        else:
            print(f"  ⚠️ 无法查询 Gazebo model pose")
            gz_pose = None

        # 等 YOLO 检测稳定（确认像素和上次不同，避免读到旧检测）
        det = collector.wait_for_detection(timeout=25.0)
        if det is None:
            print(f"  无检测，跳过 {label}")
            continue

        # 确认检测的不是旧位置（同一物体不应出现在完全相同像素位置）
        if last_pixel is not None:
            px_dist = math.hypot(det["u"] - last_pixel[0], det["v"] - last_pixel[1])
            if px_dist < 5:
                print(f"  警告: 检测像素与上一位置几乎相同 (dist={px_dist:.1f}px), 可能 spawn 未生效")
        last_pixel = (det["u"], det["v"])

        u, v, conf = det["u"], det["v"], det["conf"]

        # 记录当前相机 TF（即使静态也每位置记一次，方便事后校验）
        if not collector._lookup_camera_tf():
            print(f"  ⚠️ 相机 TF 查询失败")

        # Pixel → Camera
        xyz_cam = collector.pixel_to_camera(u, v)
        if xyz_cam is None:
            print(f"  pixel_to_camera 失败")
            continue

        # Camera → Base
        xyz_base = collector.transform_to_base(xyz_cam)
        if xyz_base is None:
            print(f"  transform_to_base 失败")
            continue

        # Ground truth — 优先用 Gazebo 实际 pose，fallback 到计算值
        if gz_pose:
            ax, ay = gz_pose["x"], gz_pose["y"]
            az = gz_pose["z"] + 0.02  # link center = model origin + link_pose.z
        else:
            ax, ay, az = compute_actual(sx, sy, sz)

        # 误差
        err = xyz_base - np.array([ax, ay, az])
        err_norm = float(np.linalg.norm(err))

        print(f"  pixel=({u:.1f},{v:.1f}) conf={conf:.3f} depth={xyz_cam[2]:.4f}m")
        print(f"  cam=({xyz_cam[0]:.4f},{xyz_cam[1]:.4f},{xyz_cam[2]:.4f})")
        print(f"  base=({xyz_base[0]:.4f},{xyz_base[1]:.4f},{xyz_base[2]:.4f})")
        print(f"  actual=({ax:.4f},{ay:.4f},{az:.4f})")
        print(f"  Δ=({err[0]*1000:+.1f},{err[1]*1000:+.1f},{err[2]*1000:+.1f})mm "
              f"|Δ|={err_norm*1000:.1f}mm")

        # ── E2b: 当场在 live depth 上验证 ray-box 表面点 ──
        P_gt = np.array([ax, ay, az])
        e2b = collector.compute_e2b(P_gt)

        def _f(v, fmt=""):
            """安全格式化 CSV 值，nan/None → 空字符串"""
            if v is None:
                return ""
            if isinstance(v, float) and math.isnan(v):
                return ""
            if isinstance(v, bool):
                return str(v)
            if fmt:
                return f"{v:{fmt}}"
            return str(v)

        # 写 CSV
        with open(csv_path, "a") as f:
            f.write(f"{label},{sx},{sy},{sz},"
                    f"{ax},{ay},{az},"
                    f"{u:.2f},{v:.2f},{conf:.3f},"
                    f"{xyz_cam[2]:.4f},"
                    f"{xyz_cam[0]:.4f},{xyz_cam[1]:.4f},{xyz_cam[2]:.4f},"
                    f"{xyz_base[0]:.4f},{xyz_base[1]:.4f},{xyz_base[2]:.4f},"
                    f"{err[0]*1000:.1f},{err[1]*1000:.1f},{err[2]*1000:.1f},"
                    f"{err_norm*1000:.1f},"
                    f"{collector.cam_tx:.4f},{collector.cam_ty:.4f},{collector.cam_tz:.4f},"
                    f"{collector.cam_qx:.6f},{collector.cam_qy:.6f},{collector.cam_qz:.6f},{collector.cam_qw:.6f},"
                    f"{collector.rgb_stamp or 0:.6f},"
                    f"{collector.depth_stamp or 0:.6f},"
                    f"{collector.detection_stamp or 0:.6f},"
                    f"{_f(e2b.get('gt_u'), '.1f')},{_f(e2b.get('gt_v'), '.1f')},"
                    f"{_f(e2b.get('z_depth_at_gt'), '.4f')},{_f(e2b.get('z_expected'), '.4f')},"
                    f"{_f(e2b.get('depth_minus_expected_mm'), '.1f')},{_f(e2b.get('surface_err_mm'), '.1f')},"
                    f"{_f(e2b.get('surface_inside_box'))},{_f(e2b.get('ray_hit_box'))}\n")

        results.append({
            "label": label,
            "spawn": (sx, sy, sz),
            "actual": (ax, ay, az),
            "pixel": (u, v),
            "conf": conf,
            "depth": xyz_cam[2],
            "cam": tuple(xyz_cam),
            "base": tuple(xyz_base),
            "err": tuple(err),
            "err_norm": err_norm,
            "e2b": e2b,
        })

        time.sleep(0.5)

    # 清理
    remove_box()

    # ─── 打印摘要 ───
    print("\n" + "=" * 70)
    print("采集完成，摘要：")
    print("=" * 70)
    if results:
        err_x = [abs(r["err"][0]) * 1000 for r in results]
        err_y = [abs(r["err"][1]) * 1000 for r in results]
        err_z = [abs(r["err"][2]) * 1000 for r in results]
        err_n = [r["err_norm"] * 1000 for r in results]
        print(f"  Δx: mean={np.mean(err_x):.1f}mm, max={np.max(err_x):.1f}mm")
        print(f"  Δy: mean={np.mean(err_y):.1f}mm, max={np.max(err_y):.1f}mm")
        print(f"  Δz: mean={np.mean(err_z):.1f}mm, max={np.max(err_z):.1f}mm")
        print(f"  |Δ|: mean={np.mean(err_n):.1f}mm, max={np.max(err_n):.1f}mm")

    # ── E2b 汇总 ──
    if results:
        e2b_ok = [r for r in results if "e2b" in r and r["e2b"].get("ray_hit_box")]
        if e2b_ok:
            dz = [abs(r["e2b"]["depth_minus_expected_mm"]) for r in e2b_ok]
            se = [r["e2b"]["surface_err_mm"] for r in e2b_ok]
            z_ok = sum(1 for r in e2b_ok
                       if abs(r["e2b"]["depth_minus_expected_mm"]) < E2B_Z_TOL_MM)
            s_ok = sum(1 for r in e2b_ok
                       if r["e2b"]["surface_err_mm"] < E2B_SURF_TOL_MM)
            print(f"\n  E2b (ray-box vs depth):")
            print(f"    ΔZ: mean={np.mean(dz):.1f}mm, max={np.max(dz):.1f}mm")
            print(f"    surface_err: mean={np.mean(se):.1f}mm, max={np.max(se):.1f}mm")
            print(f"    Z_OK (<{E2B_Z_TOL_MM:.0f}mm): {z_ok}/{len(e2b_ok)}  "
                  f"surf_OK (<{E2B_SURF_TOL_MM:.0f}mm): {s_ok}/{len(e2b_ok)}")

    print(f"\n原始数据已保存到: {csv_path}")

    collector.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
