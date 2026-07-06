#!/usr/bin/env python3
"""
坐标转换链验证：6 个诊断实验

实验 A: 反投影闭环 — pixel+depth → P_base → 投回 pixel, 检查是否一致
实验 B: 正向投影 — ground truth P_base → 投影到像素, 与 YOLO 检测对比
实验 C: 方块 8 角点投影 — 验证投影 bbox 与 YOLO bbox 的覆盖关系
实验 D: 计算点是否在方块内 — 检查反投影结果是否落在物体体积内
实验 E1: 理论深度闭环 — 用 GT 正向投影的 Z_gt 反投影, 验证投影/反投影数学链
实验 E2: 可见表面点 — 用 GT 投影像素 + 真实深度图, 检查表面点是否在方块内
实验 E2b: ray-box 理论表面点 — 像素射线 vs 方块 AABB 求交, 对比 depth 图表面点

用法:
  python3 verify_coordinate_chain.py

前提: 仿真运行中, 有 CameraInfo + TF + depth
       calib_data_current_camera.csv 中有实测数据
"""

import math
import csv
import time
from typing import Optional, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from sensor_msgs.msg import CameraInfo, Image
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs


# ─── 方块参数 ───
BLOCK_HALF_X = 0.020   # 半边长 (4cm 立方体)
BLOCK_HALF_Y = 0.020
BLOCK_HALF_Z = 0.020
CENTER_Z = 0.020        # 盒子落地后中心 z（地面 z=0 + 半高 0.02）


def quat_to_rot(qx, qy, qz, qw) -> np.ndarray:
    """quaternion → 3×3 rotation matrix"""
    return np.array([
        [1-2*qy*qy-2*qz*qz,   2*qx*qy-2*qz*qw,   2*qx*qz+2*qy*qw],
        [2*qx*qy+2*qz*qw,   1-2*qx*qx-2*qz*qz,   2*qy*qz-2*qx*qw],
        [2*qx*qz-2*qy*qw,   2*qy*qz+2*qx*qw,   1-2*qx*qx-2*qy*qy],
    ], dtype=float)


class CoordinateChainVerifier(Node):
    def __init__(self):
        super().__init__("verify_coordinate_chain")

        self.base_frame = "base_link"
        self.camera_frame = "camera_color_optical_frame"

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.fx: Optional[float] = None
        self.fy: Optional[float] = None
        self.cx: Optional[float] = None
        self.cy: Optional[float] = None

        self.create_subscription(CameraInfo, "/camera/color/camera_info",
                                 self._cb_info, 10)
        self.create_subscription(Image, "/camera/depth/image_raw",
                                 self._cb_depth, 10)

        self.depth_img: Optional[np.ndarray] = None
        self.bridge = CvBridge()

        self.cam_R: Optional[np.ndarray] = None
        self.cam_t: Optional[np.ndarray] = None

    def _cb_info(self, msg: CameraInfo):
        self.fx = float(msg.k[0])
        self.fy = float(msg.k[4])
        self.cx = float(msg.k[2])
        self.cy = float(msg.k[5])

    def _cb_depth(self, msg: Image):
        self.depth_img = self.bridge.imgmsg_to_cv2(msg, "passthrough")

    def wait_ready(self, timeout=10.0):
        start = time.time()
        while self.fx is None or self.depth_img is None:
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - start > timeout:
                return False
        return True

    # ─── TF 工具 ─────────────────────────────────────────
    def _lookup_camera_tf(self) -> bool:
        """刷新 camera→base 的 R,t 缓存（只查一次，双向派生）

        lookup_transform(target=base, source=camera) 返回 R,t 满足:
          P_base = R @ P_cam + t

        其中 R 是从 camera 到 base 的旋转矩阵，
        t  是 camera 原点在 base 系下的坐标。
        """
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame, self.camera_frame,
                rclpy.time.Time(), timeout=Duration(seconds=0.5))
        except Exception:
            return False

        q = tf.transform.rotation
        t = tf.transform.translation
        self.cam_R = quat_to_rot(q.x, q.y, q.z, q.w)   # camera → base
        self.cam_t = np.array([t.x, t.y, t.z], dtype=float)  # camera 在 base 系的位置
        return True

    # ─── 投影函数 ───────────────────────────────────────
    def camera_to_base(self, P_cam: np.ndarray) -> np.ndarray:
        """camera_optical_frame → base_link

        P_base = R @ P_cam + t
        直接使用 lookup_transform(base, camera) 返回的 R,t
        """
        return self.cam_R @ P_cam + self.cam_t

    def base_to_camera(self, P_base: np.ndarray) -> np.ndarray:
        """base_link → camera_optical_frame

        P_cam = R^T @ (P_base - t)
        对 camera_to_base 取数学逆
        """
        return self.cam_R.T @ (P_base - self.cam_t)

    def camera_to_pixel(self, P_cam: np.ndarray) -> Optional[np.ndarray]:
        """camera_optical_frame 3D → 像素 (u,v)"""
        x, y, z = P_cam[0], P_cam[1], P_cam[2]
        if z <= 1e-6:
            return None
        u = self.fx * x / z + self.cx
        v = self.fy * y / z + self.cy
        return np.array([u, v], dtype=float)

    def pixel_depth_to_camera(self, u: float, v: float,
                               depth: float) -> np.ndarray:
        """像素 (u,v) + 深度值 → camera_optical_frame 3D"""
        Z = depth
        X = (u - self.cx) * Z / self.fx
        Y = (v - self.cy) * Z / self.fy
        return np.array([X, Y, Z], dtype=float)

    # ─── 实验 A ──────────────────────────────────────────
    def experiment_A(self, u: float, v: float, depth: float,
                     label: str) -> Tuple[float, float]:
        """
        反投影闭环验证:
          pixel(u,v) + depth → P_cam → P_base → P_cam' → pixel(u',v')
        返回 |u'-u| 和 |v'-v|
        """
        # 正向: pixel+depth → camera
        P_cam = self.pixel_depth_to_camera(u, v, depth)

        # camera → base
        P_base = self.camera_to_base(P_cam)

        # base → camera (反向)
        P_cam_back = self.base_to_camera(P_base)

        # camera → pixel (重投影)
        uv_back = self.camera_to_pixel(P_cam_back)
        if uv_back is None:
            return (999, 999)

        du = abs(uv_back[0] - u)
        dv = abs(uv_back[1] - v)

        self.get_logger().info(
            f"  [A] {label}: pixel({u:.1f},{v:.1f}) → "
            f"P_cam=({P_cam[0]:.4f},{P_cam[1]:.4f},{P_cam[2]:.4f}) → "
            f"P_base=({P_base[0]:.4f},{P_base[1]:.4f},{P_base[2]:.4f}) → "
            f"pixel'=({uv_back[0]:.1f},{uv_back[1]:.1f}) "
            f"Δ=({du:.2f},{dv:.2f})px"
        )
        return (du, dv)

    # ─── 实验 B ─────────────────────────────────────────
    def experiment_B(self, P_base_gt: np.ndarray,
                     u_yolo: float, v_yolo: float,
                     label: str) -> Tuple[float, float, float, float]:
        """
        正向投影验证:
          ground truth P_base → P_cam → pixel(u_pred, v_pred)
        与 YOLO 检测像素对比
        """
        P_cam = self.base_to_camera(P_base_gt)
        uv_pred = self.camera_to_pixel(P_cam)
        if uv_pred is None:
            return (999, 999, 0, 0)

        du = uv_pred[0] - u_yolo
        dv = uv_pred[1] - v_yolo

        self.get_logger().info(
            f"  [B] {label}: GT=({P_base_gt[0]:.3f},{P_base_gt[1]:.3f},{P_base_gt[2]:.3f}) "
            f"→ pixel_pred=({uv_pred[0]:.1f},{uv_pred[1]:.1f})  "
            f"YOLO=({u_yolo:.1f},{v_yolo:.1f})  "
            f"Δ=({du:+.1f},{dv:+.1f})px"
        )
        return (uv_pred[0], uv_pred[1], du, dv)

    # ─── 实验 C ─────────────────────────────────────────
    def experiment_C(self, P_center: np.ndarray, u_yolo: float, v_yolo: float,
                     label: str) -> dict:
        """
        8 角点投影:
          P_center ± (BLOCK_HALF_X, BLOCK_HALF_Y, BLOCK_HALF_Z)
        投影所有角点, 检查覆盖 YOLO bbox
        """
        corners_base = []
        for dx in [-BLOCK_HALF_X, BLOCK_HALF_X]:
            for dy in [-BLOCK_HALF_Y, BLOCK_HALF_Y]:
                for dz in [-BLOCK_HALF_Z, BLOCK_HALF_Z]:
                    corners_base.append(
                        P_center + np.array([dx, dy, dz]))

        corners_uv = []
        for c in corners_base:
            P_cam = self.base_to_camera(c)
            uv = self.camera_to_pixel(P_cam)
            if uv is not None:
                corners_uv.append(uv)

        corners_uv = np.array(corners_uv)
        if len(corners_uv) == 0:
            return {"error": "所有角点都在相机后方"}

        u_min, v_min = corners_uv[:, 0].min(), corners_uv[:, 1].min()
        u_max, v_max = corners_uv[:, 0].max(), corners_uv[:, 1].max()
        u_center = (u_min + u_max) / 2.0
        v_center = (v_min + v_max) / 2.0

        # YOLO 是否在投影 bbox 内
        yolo_inside = (u_min <= u_yolo <= u_max and
                       v_min <= v_yolo <= v_max)

        self.get_logger().info(
            f"  [C] {label}: 投影 bbox=({u_min:.0f},{v_min:.0f})~"
            f"({u_max:.0f},{v_max:.0f}), "
            f"中心=({u_center:.0f},{v_center:.0f}), "
            f"YOLO=({u_yolo:.0f},{v_yolo:.0f}) "
            f"{'✅ YOLO在bbox内' if yolo_inside else '❌ YOLO在bbox外'}"
        )

        return {
            "u_min": u_min, "v_min": v_min,
            "u_max": u_max, "v_max": v_max,
            "u_center": u_center, "v_center": v_center,
            "yolo_u": u_yolo, "yolo_v": v_yolo,
            "yolo_inside": yolo_inside,
        }

    # ─── 实验 D ─────────────────────────────────────────
    def experiment_D(self, P_calc: np.ndarray, P_center: np.ndarray,
                     label: str) -> dict:
        """检查计算点是否在方块体积内"""
        diff = P_calc - P_center
        inside_x = abs(diff[0]) <= BLOCK_HALF_X
        inside_y = abs(diff[1]) <= BLOCK_HALF_Y
        inside_z = abs(diff[2]) <= BLOCK_HALF_Z
        inside_all = inside_x and inside_y and inside_z

        self.get_logger().info(
            f"  [D] {label}: P=({P_calc[0]:.4f},{P_calc[1]:.4f},{P_calc[2]:.4f}) "
            f"center=({P_center[0]:.3f},{P_center[1]:.3f},{P_center[2]:.3f}) "
            f"diff=({diff[0]*1000:+.0f},{diff[1]*1000:+.0f},{diff[2]*1000:+.0f})mm "
            f"inside: x={inside_x} y={inside_y} z={inside_z} "
            f"{'✅ 在方块内' if inside_all else '❌ 不在方块内'}"
        )

        return {
            "inside_x": inside_x, "inside_y": inside_y, "inside_z": inside_z,
            "inside_all": inside_all,
        }

    # ─── 实验 E1 ─────────────────────────────────────────
    def experiment_E1(self, P_base_gt: np.ndarray, label: str) -> dict:
        """
        用 GT 正向投影的理论深度 Z_gt 做反投影，验证数学链。

        流程:
          1. GT → P_cam_gt = base_to_camera(P_base_gt)
          2. 取理论深度 Z_gt = P_cam_gt[2]
          3. GT 投影像素 (u_gt, v_gt) + 理论深度 Z_gt → 反投影 → P_calc
          4. 比较 P_calc 与 P_base_gt

        这个实验不读深度图，Z 是精确的理论值。
        误差应 ≈ 0，验证的是 内参 + TF + 投影/反投影公式 是否正确。
        """
        P_cam_gt = self.base_to_camera(P_base_gt)
        uv_gt = self.camera_to_pixel(P_cam_gt)
        if uv_gt is None:
            self.get_logger().error(f"  [E1] {label}: GT 在相机后方")
            return {"error": "GT behind camera"}

        u_gt, v_gt = uv_gt[0], uv_gt[1]
        Z_gt = float(P_cam_gt[2])  # 理论深度，不读 depth 图

        # 用理论深度反投影
        P_cam_calc = self.pixel_depth_to_camera(u_gt, v_gt, Z_gt)
        P_base_calc = self.camera_to_base(P_cam_calc)

        diff = P_base_calc - P_base_gt
        err_norm = float(np.linalg.norm(diff))

        self.get_logger().info(
            f"  [E1] {label}: GT=({P_base_gt[0]:.4f},{P_base_gt[1]:.4f},{P_base_gt[2]:.4f}) "
            f"→ pixel=({u_gt:.1f},{v_gt:.1f}) Z_gt={Z_gt:.4f}m → "
            f"P_calc=({P_base_calc[0]:.4f},{P_base_calc[1]:.4f},{P_base_calc[2]:.4f}) "
            f"Δ=({diff[0]*1e6:+.1f},{diff[1]*1e6:+.1f},{diff[2]*1e6:+.1f})μm"
        )

        return {
            "err_norm_mm": err_norm * 1000.0,
            "error": None,
        }

    # ─── 实验 E2 ─────────────────────────────────────────
    def experiment_E2(self, P_base_gt: np.ndarray, label: str) -> dict:
        """
        GT 投影像素 + 真实深度图 → 可见表面点，检查是否在方块体积内。

        深度图返回的不是物体中心深度，而是光线碰到的第一个可见表面。
        对于 10×6×4cm 方块，表面点偏离中心 20~30mm 是正常的。
        本实验不要求表面点等于中心——只检查是否在方块体积内。
        """
        P_cam_gt = self.base_to_camera(P_base_gt)
        uv_gt = self.camera_to_pixel(P_cam_gt)
        if uv_gt is None:
            self.get_logger().error(f"  [E2] {label}: GT 在相机后方")
            return {"error": "GT behind camera"}

        u_gt, v_gt = uv_gt[0], uv_gt[1]

        if self.depth_img is None:
            self.get_logger().error(f"  [E2] {label}: 无深度图")
            return {"error": "no depth image"}

        h, w = self.depth_img.shape
        ui, vi = int(round(u_gt)), int(round(v_gt))
        if not (0 <= ui < w and 0 <= vi < h):
            self.get_logger().error(
                f"  [E2] {label}: GT 投影像素 ({u_gt:.0f},{v_gt:.0f}) 超出图像")
            return {"error": "pixel out of bounds"}

        k = 2
        patch = self.depth_img[max(0, vi - k):vi + k + 1,
                                max(0, ui - k):ui + k + 1]
        valid = patch[np.isfinite(patch) & (patch > 0)]
        if valid.size == 0:
            self.get_logger().error(f"  [E2] {label}: 无有效深度")
            return {"error": "no valid depth"}

        Z_depth = float(np.median(valid))
        # 一致: 深度来自 (ui,vi), 反投影也用 (ui,vi) — 调试阶段用整数像素更可解释
        P_cam_surface = self.pixel_depth_to_camera(float(ui), float(vi), Z_depth)
        P_surface = self.camera_to_base(P_cam_surface)

        # 不跟中心比——只检查是否在方块体积内
        # 加 3mm 容差: 深度图离散像素 + 5×5 patch 边缘混合 + Gazebo 采样误差
        tol = 0.003
        diff = P_surface - P_base_gt
        inside_x = abs(diff[0]) <= BLOCK_HALF_X + tol
        inside_y = abs(diff[1]) <= BLOCK_HALF_Y + tol
        inside_z = abs(diff[2]) <= BLOCK_HALF_Z + tol
        inside_all = inside_x and inside_y and inside_z

        self.get_logger().info(
            f"  [E2] {label}: GT=({P_base_gt[0]:.3f},{P_base_gt[1]:.3f},{P_base_gt[2]:.3f}) "
            f"→ pixel=({ui},{vi}) depth={Z_depth:.4f}m → "
            f"surface=({P_surface[0]:.4f},{P_surface[1]:.4f},{P_surface[2]:.4f}) "
            f"Δ=({diff[0]*1000:+.0f},{diff[1]*1000:+.0f},{diff[2]*1000:+.0f})mm"
            f" inside: x={inside_x} y={inside_y} z={inside_z} "
            f"{'✅ 在方块内' if inside_all else '❌ 在方块外'}"
        )

        return {
            "inside_all": inside_all,
            "diff_mm": diff * 1000.0,
            "error": None,
        }

    # ─── 工具: ray-box AABB 求交 ─────────────────────────
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

        Z 是 camera optical frame 的 Z 坐标（与 depth 图一致），
        不是归一化射线长度。

        返回:
          (Z_entry, P_entry_base)  — 第一个入射点的 optical Z 和 base 坐标
          None                      — 无交点
        """
        # 不要 normalize — depth 图的 Z 就是 optical Z
        r_cam = np.array([
            (u - self.cx) / self.fx,
            (v - self.cy) / self.fy,
            1.0,
        ], dtype=float)

        # P_base(Z) = O + Z * d_base
        O = self.cam_t                       # 相机原点在 base 系
        d = self.cam_R @ r_cam               # 射线方向在 base 系 (scaled by Z)

        box_min = box_center - half_extents
        box_max = box_center + half_extents

        z_min = -np.inf
        z_max = np.inf

        for i in range(3):
            if abs(d[i]) < eps:
                # 射线与该轴平行 — 若相机原点不在 slab 内则无交点
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

    # ─── 实验 E2b ────────────────────────────────────────
    def experiment_E2b(self, P_base_gt: np.ndarray, label: str) -> dict:
        """
        Ray-box 理论表面点 vs depth 图表面点。

        对 GT center 的像素射线，计算它和方块 AABB 的理论入射交点，
        然后拿 depth 图返回的表面点去对比。

        这能直接回答: depth 图返回的是不是理论上方块应该出现的表面点。
        """
        P_cam_gt = self.base_to_camera(P_base_gt)
        uv_gt = self.camera_to_pixel(P_cam_gt)
        if uv_gt is None:
            self.get_logger().error(f"  [E2b] {label}: GT 在相机后方")
            return {"error": "GT behind camera"}

        u_gt, v_gt = uv_gt[0], uv_gt[1]

        if self.depth_img is None:
            self.get_logger().error(f"  [E2b] {label}: 无深度图")
            return {"error": "no depth image"}

        h, w = self.depth_img.shape
        ui, vi = int(round(u_gt)), int(round(v_gt))
        if not (0 <= ui < w and 0 <= vi < h):
            self.get_logger().error(
                f"  [E2b] {label}: GT 投影像素 ({u_gt:.0f},{v_gt:.0f}) 超出图像")
            return {"error": "pixel out of bounds"}

        # 1. 取 depth 图表面点（用整数像素保证深度采样点与反投影像素一致）
        k = 2
        patch = self.depth_img[max(0, vi - k):vi + k + 1,
                                max(0, ui - k):ui + k + 1]
        valid = patch[np.isfinite(patch) & (patch > 0)]
        if valid.size == 0:
            self.get_logger().error(f"  [E2b] {label}: 无有效深度")
            return {"error": "no valid depth"}

        Z_depth = float(np.median(valid))
        # 一致: 深度来自 (ui,vi), 反投影也用 (ui,vi)
        P_cam_surface = self.pixel_depth_to_camera(float(ui), float(vi), Z_depth)
        P_surface = self.camera_to_base(P_cam_surface)

        # 2. Ray-box 理论交点
        half_extents = np.array([BLOCK_HALF_X, BLOCK_HALF_Y, BLOCK_HALF_Z])
        ray_hit = self.ray_box_intersection_depth_z(
            u_gt, v_gt, P_base_gt, half_extents)

        if ray_hit is None:
            self.get_logger().warning(
                f"  [E2b] {label}: GT center ray 没有和方块 AABB 相交!")
            return {"error": "ray miss box", "Z_depth": Z_depth}

        Z_expected, P_expected = ray_hit

        depth_err_mm = (Z_depth - Z_expected) * 1000.0
        surface_err_mm = float(np.linalg.norm(P_surface - P_expected)) * 1000.0

        Z_OK = abs(depth_err_mm) < 10.0        # 10mm 容差
        surf_OK = surface_err_mm < 10.0

        self.get_logger().info(
            f"  [E2b] {label}: "
            f"Z_depth={Z_depth:.4f}m Z_expected={Z_expected:.4f}m "
            f"ΔZ={depth_err_mm:+.1f}mm {'✅' if Z_OK else '❌'} | "
            f"P_expected=({P_expected[0]:.4f},{P_expected[1]:.4f},{P_expected[2]:.4f}) "
            f"surface_err={surface_err_mm:.1f}mm {'✅' if surf_OK else '❌'}"
        )

        return {
            "Z_depth": Z_depth,
            "Z_expected": Z_expected,
            "depth_err_mm": depth_err_mm,
            "surface_err_mm": surface_err_mm,
            "Z_OK": Z_OK,
            "surf_OK": surf_OK,
            "error": None,
        }


# ================================================================
# 主流程
# ================================================================
def main():
    rclpy.init()
    node = CoordinateChainVerifier()

    if not node.wait_ready():
        print("错误: 等待相机内参/深度图超时")
        return

    if not node._lookup_camera_tf():
        print("错误: 无法查 base→camera TF")
        return
    print(f"\n内参: fx={node.fx:.1f} fy={node.fy:.1f} "
          f"cx={node.cx:.1f} cy={node.cy:.1f}")
    print(f"camera_R:\n{node.cam_R}")
    print(f"camera_t: {node.cam_t}\n")

    # ─── 读取实测数据 ───
    csv_path = "/home/yep/my_S622/debug_data/calib_data_current_camera.csv"
    rows = []
    try:
        with open(csv_path, "r") as f:
            reader = csv.DictReader(f)
            for row in reader:
                rows.append(row)
    except FileNotFoundError:
        print(f"CSV 未找到: {csv_path}")
        print("请先运行 collect_calib_data.py 采集数据")
        return

    if not rows:
        print("CSV 为空")
        return

    print("=" * 70)
    print("  实验 A: 反投影闭环 (pixel+depth → base → pixel)")
    print("=" * 70)
    a_errors = []
    for row in rows:
        u = float(row["pixel_u"])
        v = float(row["pixel_v"])
        depth = float(row["depth_m"])
        label = row["label"]
        du, dv = node.experiment_A(u, v, depth, label)
        a_errors.append((du, dv))
    if a_errors:
        mean_du = np.mean([e[0] for e in a_errors])
        mean_dv = np.mean([e[1] for e in a_errors])
        print(f"  实验 A 结论: 平均 Δu={mean_du:.3f}px, Δv={mean_dv:.3f}px "
              f"{'✅ 闭环' if mean_du < 0.5 and mean_dv < 0.5 else '❌ 存在漂移'}")

    print("\n" + "=" * 70)
    print("  实验 B: 正向投影 (ground truth → pixel vs YOLO)")
    print("=" * 70)
    b_errors = []
    for row in rows:
        sx = float(row["spawn_x"])
        sy = float(row["spawn_y"])
        P_gt = np.array([sx, sy, CENTER_Z])
        u_yolo = float(row["pixel_u"])
        v_yolo = float(row["pixel_v"])
        label = row["label"]
        up, vp, du, dv = node.experiment_B(P_gt, u_yolo, v_yolo, label)
        b_errors.append((du, dv))
    if b_errors:
        mean_du = np.mean([e[0] for e in b_errors])
        mean_dv = np.mean([e[1] for e in b_errors])
        std_du = np.std([e[0] for e in b_errors])
        std_dv = np.std([e[1] for e in b_errors])
        print(f"  实验 B 结论: Δ(u_pred-YOLO) = ({mean_du:.1f}±{std_du:.0f}, "
              f"{mean_dv:.1f}±{std_dv:.0f})px")

    print("\n" + "=" * 70)
    print("  实验 C: 方块 8 角点投影")
    print("=" * 70)
    c_inside = 0
    for row in rows:
        sx = float(row["spawn_x"])
        sy = float(row["spawn_y"])
        P_center = np.array([sx, sy, CENTER_Z])
        u_yolo = float(row["pixel_u"])
        v_yolo = float(row["pixel_v"])
        label = row["label"]
        result = node.experiment_C(P_center, u_yolo, v_yolo, label)
        if result.get("yolo_inside"):
            c_inside += 1
    print(f"  实验 C 结论: YOLO 在投影 bbox 内的比率 = {c_inside}/{len(rows)} "
          f"{'✅' if c_inside == len(rows) else '❌'}")

    print("\n" + "=" * 70)
    print("  实验 D: 计算点是否在方块体积内")
    print("=" * 70)
    d_inside = 0
    for row in rows:
        sx = float(row["spawn_x"])
        sy = float(row["spawn_y"])
        P_center = np.array([sx, sy, CENTER_Z])
        bx = float(row["base_x"])
        by = float(row["base_y"])
        bz = float(row["base_z"])
        P_calc = np.array([bx, by, bz])
        label = row["label"]
        result = node.experiment_D(P_calc, P_center, label)
        if result["inside_all"]:
            d_inside += 1
    print(f"  实验 D 结论: 计算点在方块内的比率 = {d_inside}/{len(rows)} "
          f"{'✅' if d_inside == len(rows) else '❌ 反投影点不在目标物体上'}")

    print("\n" + "=" * 70)
    print("  实验 E1: 理论深度闭环 (GT Z_gt, 不读深度图)")
    print("=" * 70)
    e1_errors = []
    for row in rows:
        sx = float(row["spawn_x"])
        sy = float(row["spawn_y"])
        P_gt = np.array([sx, sy, CENTER_Z])
        label = row["label"]
        result = node.experiment_E1(P_gt, label)
        if result.get("error") is None:
            e1_errors.append(result["err_norm_mm"])
    if e1_errors:
        e1_mean = np.mean(e1_errors)
        e1_max = np.max(e1_errors)
        print(f"  实验 E1 结论: 误差 mean={e1_mean:.3f}mm (={e1_mean*1000:.0f}μm), "
              f"max={e1_max:.3f}mm (={e1_max*1000:.0f}μm)")
        e1_pass = e1_mean < 0.01  # <10μm
        print(f"    {'✅ 通过 (数学链正确)' if e1_pass else '❌ 内参/TF/投影公式有问题'}")
    else:
        print(f"  实验 E1 结论: ❌ 所有位置实验失败")
        e1_pass = False

    print("\n" + "=" * 70)
    print("  实验 E2: GT 投影像素 + 深度图 → 可见表面点")
    print("=" * 70)
    e2_inside = 0
    for row in rows:
        sx = float(row["spawn_x"])
        sy = float(row["spawn_y"])
        P_gt = np.array([sx, sy, CENTER_Z])
        label = row["label"]
        # 每次实验前 spin 一下确保深度图最新
        rclpy.spin_once(node, timeout_sec=0.1)
        result = node.experiment_E2(P_gt, label)
        if result.get("error") is None:
            if result["inside_all"]:
                e2_inside += 1
    if rows:
        print(f"  实验 E2 结论: 表面点在方块内的比率 = {e2_inside}/{len(rows)} "
              f"{'✅ 全部在方块内' if e2_inside == len(rows) else '❌ 部分在方块外'}")
        e2_pass = e2_inside == len(rows)
    else:
        print(f"  实验 E2 结论: ❌ 无有效数据")
        e2_pass = False

    print("\n" + "=" * 70)
    print("  实验 E2b: Ray-box 理论表面点 vs depth 表面点")
    print("=" * 70)
    e2b_Z_errs = []
    e2b_surf_errs = []
    e2b_Z_ok = 0
    e2b_surf_ok = 0
    e2b_total = 0
    for row in rows:
        sx = float(row["spawn_x"])
        sy = float(row["spawn_y"])
        P_gt = np.array([sx, sy, CENTER_Z])
        label = row["label"]
        rclpy.spin_once(node, timeout_sec=0.1)
        result = node.experiment_E2b(P_gt, label)
        if result.get("error") is None:
            e2b_total += 1
            e2b_Z_errs.append(abs(result["depth_err_mm"]))
            e2b_surf_errs.append(result["surface_err_mm"])
            if result["Z_OK"]:
                e2b_Z_ok += 1
            if result["surf_OK"]:
                e2b_surf_ok += 1
        elif result.get("error") == "ray miss box":
            e2b_total += 1  # 算入总数但不加分
    if e2b_Z_errs:
        print(f"  实验 E2b 结论:")
        print(f"    ΔZ (depth vs expected): mean={np.mean(e2b_Z_errs):.1f}mm, "
              f"max={np.max(e2b_Z_errs):.1f}mm")
        print(f"    surface_err: mean={np.mean(e2b_surf_errs):.1f}mm, "
              f"max={np.max(e2b_surf_errs):.1f}mm")
        print(f"    Z_OK (<10mm): {e2b_Z_ok}/{e2b_total}  surf_OK (<10mm): {e2b_surf_ok}/{e2b_total}")
        e2b_pass = e2b_Z_ok >= e2b_total * 0.7 and e2b_surf_ok >= e2b_total * 0.7
        print(f"    {'✅ 通过 (≥70%)' if e2b_pass else '❌ depth 图与理论表面不一致'}")
    else:
        print(f"  实验 E2b 结论: ❌ 无有效数据")
        e2b_pass = False

    print("\n" + "=" * 70)
    print("  综合诊断")
    print("=" * 70)
    # 判断
    all_pass = True
    if a_errors:
        mdu = np.mean([e[0] for e in a_errors])
        mdv = np.mean([e[1] for e in a_errors])
        if mdu > 0.5 or mdv > 0.5:
            print("  ❌ 实验 A 失败: 内参/TF/frame 使用不一致")
            all_pass = False
        else:
            print("  ✅ 实验 A 通过: 内参/TF/frame 一致")
    if b_errors:
        mdu = np.mean([e[0] for e in b_errors])
        mdv = np.mean([e[1] for e in b_errors])
        if abs(mdu) > 20 or abs(mdv) > 20:
            print(f"  ❌ 实验 B 失败: YOLO 中心 vs GT 投影像素偏差 "
                  f"({mdu:.0f},{mdv:.0f})px > 20px")
            all_pass = False
        else:
            print(f"  ✅ 实验 B 通过: GT 投影与 YOLO 偏差 < 20px")
    if c_inside < len(rows):
        print(f"  ❌ 实验 C 失败: {len(rows)-c_inside} 个位置 YOLO 不在 8 角点投影内")
        all_pass = False
    else:
        print("  ✅ 实验 C 通过: 所有 YOLO 检测点在 8 角点投影内")
    if d_inside < len(rows):
        print(f"  ❌ 实验 D 失败: {len(rows)-d_inside} 个位置反投影点不在方块体积内")
        print("     → 当前 pixel+depth 采样点不在目标物体表面, 在背景/桌面上")
        all_pass = False
    else:
        print("  ✅ 实验 D 通过")
    if not e1_pass:
        print(f"  ❌ 实验 E1 失败: 理论深度闭环存在误差, 内参/TF/投影公式需要排查")
        all_pass = False
    else:
        print("  ✅ 实验 E1 通过: 内参 + TF + 投影/反投影 数学链正确 (误差 <10μm)")
    if not e2_pass:
        print(f"  ❌ 实验 E2 失败: 深度图反投影的表面点不在方块体积内")
        print("     → 可能原因: 深度图噪声大, 或 GT 投影像素处深度采样到了背景")
        all_pass = False
    else:
        print("  ✅ 实验 E2 通过: 深度图反投影的表面点在方块体积内")
    if not e2b_pass:
        print(f"  ❌ 实验 E2b 失败: depth 图表面点与 ray-box 理论表面点偏差大")
        if e1_pass:
            print("     → 点在方块体积内, 但不是理论上该射线应命中的表面")
            print("     → 可能原因: patch 混合/边缘深度/几何尺寸不匹配/图像同步")
        all_pass = False
    else:
        print("  ✅ 实验 E2b 通过: depth 图表面点与理论 ray-box 入射点一致 (<10mm)")
        if e1_pass and e2_pass:
            print("     → 坐标链 + depth 图几何完全可信")

    print(f"\n  {'✅ 全部通过' if all_pass else '❌ 存在问题, 需排查上述实验'}")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
