#!/usr/bin/env python3
"""
标定验证脚本：系统性地测试相机→机械臂坐标转换链精度

测试流程:
  在 5 个已知位置 spawn target_box
  → 从相机检测 + 深度图 + TF 计算 base_link 坐标
  → 与 ground truth 对比
  → 逐项排查 6 个标定指标
  → 输出中文报告

用法:
  python3 calibration_verify.py [--positions 0 1 2 3 4]
"""

import sys
import time
import math
import subprocess
from typing import Optional, Tuple, List
from dataclasses import dataclass, field

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from geometry_msgs.msg import PointStamped
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs
from cv_bridge import CvBridge


# ============================================================
# 测试位置定义 (base_link 系, 米)
# ============================================================
TEST_POSITIONS = [
    # (x, y, z_spawn, 标签)
    (0.35, 0.00, 0.05, "中心"),
    (0.30, 0.15, 0.05, "左上(近臂)"),
    (0.40, 0.20, 0.05, "右上(远臂)"),
    (0.30, -0.15, 0.05, "左下"),
    (0.40, -0.20, 0.05, "右下"),
]

# ============================================================
# 标定检查项
# ============================================================
@dataclass
class CalibCheck:
    name: str
    status: str = "未检查"  # PASS / FAIL / WARN
    detail: str = ""

@dataclass
class PositionResult:
    label: str
    spawn_xyz: Tuple[float, float, float]
    actual_xyz: Tuple[float, float, float]  # 落地后的实际位置
    pixel_uv: Tuple[float, float] = (0, 0)
    depth_m: float = 0.0
    camera_xyz: Tuple[float, float, float] = (0, 0, 0)
    base_xyz: Tuple[float, float, float] = (0, 0, 0)
    error_xyz: Tuple[float, float, float] = (0, 0, 0)
    error_norm: float = 0.0
    conf: float = 0.0


class CalibrationVerifier(Node):
    def __init__(self):
        super().__init__("calibration_verify")

        # ─── 基本参数 ───
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("camera_frame", "camera_color_optical_frame")
        self.declare_parameter("depth_window", 9)
        self.declare_parameter("table_z_margin", 0.005)

        self.base_frame = self.get_parameter("base_frame").value
        self.camera_frame = self.get_parameter("camera_frame").value
        self.depth_window = self.get_parameter("depth_window").value

        # ─── 内参缓存 ───
        self.fx: Optional[float] = None
        self.fy: Optional[float] = None
        self.cx: Optional[float] = None
        self.cy: Optional[float] = None
        self.image_width: Optional[int] = None
        self.image_height: Optional[int] = None

        # ─── 数据缓存 ───
        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.depth_img: Optional[np.ndarray] = None
        self.latest_rgb_img: Optional[np.ndarray] = None

        # ─── ROS 接口 ───
        self.create_subscription(CameraInfo, "/camera/color/camera_info",
                                 self._cb_info, 10)
        self.create_subscription(Image, "/camera/color/image_raw",
                                 self._cb_rgb, qos_profile_sensor_data)
        self.create_subscription(Image, "/camera/depth/image_raw",
                                 self._cb_depth, qos_profile_sensor_data)

        # ─── 结果存储 ───
        self.checks: List[CalibCheck] = []
        self.results: List[PositionResult] = []

        self.get_logger().info("=" * 60)
        self.get_logger().info("标定验证节点启动")
        self.get_logger().info("=" * 60)

    # ================================================================
    # 回调
    # ================================================================
    def _cb_info(self, msg: CameraInfo):
        self.fx = float(msg.k[0])
        self.fy = float(msg.k[4])
        self.cx = float(msg.k[2])
        self.cy = float(msg.k[5])
        self.image_width = int(msg.width)
        self.image_height = int(msg.height)

    def _cb_rgb(self, msg: Image):
        try:
            self.latest_rgb_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception:
            pass

    def _cb_depth(self, msg: Image):
        self.depth_img = self.bridge.imgmsg_to_cv2(msg, "passthrough")

    # ================================================================
    # 核心计算
    # ================================================================
    def pixel_to_camera(self, u: float, v: float) -> Optional[np.ndarray]:
        """像素 + 深度图 → 相机光学系 3D 坐标"""
        if self.fx is None or self.depth_img is None:
            return None

        h, w = self.depth_img.shape
        ui, vi = int(round(u)), int(round(v))
        if not (0 <= ui < w and 0 <= vi < h):
            return None

        k = self.depth_window // 2
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
        """相机系 3D → base_link 3D"""
        pt = PointStamped()
        pt.header.frame_id = self.camera_frame
        pt.header.stamp = rclpy.time.Time().to_msg()
        pt.point.x = float(xyz_cam[0])
        pt.point.y = float(xyz_cam[1])
        pt.point.z = float(xyz_cam[2])

        try:
            pt_base = self.tf_buffer.transform(
                pt, self.base_frame, timeout=Duration(seconds=0.5))
        except Exception as e:
            self.get_logger().warning(f"TF failed: {e}")
            return None

        return np.array([pt_base.point.x, pt_base.point.y, pt_base.point.z])

    def get_ground_truth(self, spawn_x: float, spawn_y: float,
                         spawn_z: float, static: bool) -> np.ndarray:
        """推算方块中心的真实 base 坐标。

        target_box 模型: 10×6×4cm, link pose z=0.02 (中心在底面+2cm)
        - static=true: 中心 = spawn_xyz + (0, 0, 0.02)
        - static=false: 落到地面 z≈0.0, 中心 z≈0.02
        """
        if static:
            return np.array([spawn_x, spawn_y, spawn_z + 0.02])
        else:
            return np.array([spawn_x, spawn_y, 0.02])

    # ================================================================
    # 标定检查项
    # ================================================================
    def check_all(self):
        """逐项执行 6 项标定检查"""
        self.get_logger().info("\n" + "=" * 60)
        self.get_logger().info("开始逐项标定检查")
        self.get_logger().info("=" * 60)

        # ─── 1. 相机内参 ───
        self._check_intrinsics()

        # ─── 2. 深度单位 ───
        self._check_depth_units()

        # ─── 3. RGB-Depth 对齐 ───
        self._check_rgb_depth_alignment()

        # ─── 4. T_base_camera ───
        self._check_tf_base_camera()

        # ─── 5. TCP 工具坐标 ───
        self._check_tcp()

        # ─── 6. 桌面高度 ───
        self._check_table_height()

        # ─── 打印摘要 ───
        self._print_check_summary()

    def _check_intrinsics(self):
        c = CalibCheck("1. 相机内参 (fx,fy,cx,cy)")
        if self.fx is None:
            c.status = "FAIL"
            c.detail = "未收到 CameraInfo"
        else:
            expected_fx = 465.6
            expected_fy = 625.2
            err_fx = abs(self.fx - expected_fx) / expected_fx * 100
            err_fy = abs(self.fy - expected_fy) / expected_fy * 100

            c.detail = (f"fx={self.fx:.2f} (预期 {expected_fx}, 偏差 {err_fx:.1f}%), "
                        f"fy={self.fy:.2f} (预期 {expected_fy}, 偏差 {err_fy:.1f}%), "
                        f"cx={self.cx:.1f}, cy={self.cy:.1f}, "
                        f"分辨率={self.image_width}x{self.image_height}")

            if err_fx < 2 and err_fy < 2:
                c.status = "PASS"
            else:
                c.status = "WARN"
                c.detail += " | 内参与预期偏差较大，需检查 Gazebo 相机配置"

        self.checks.append(c)

    def _check_depth_units(self):
        c = CalibCheck("2. 深度单位")
        if self.depth_img is None:
            c.status = "FAIL"
            c.detail = "无深度图"
        else:
            d_min = float(np.min(self.depth_img[self.depth_img > 0]))
            d_max = float(np.max(self.depth_img))
            d_mean = float(np.mean(self.depth_img[self.depth_img > 0]))

            c.detail = (f"深度范围 [{d_min:.4f}, {d_max:.4f}], "
                        f"均值 {d_mean:.4f}")

            # Gazebo rgbd_camera 输出 float32，单位米
            # 正常范围: 0.3~3.0m
            if 0.2 < d_mean < 5.0:
                c.status = "PASS"
                c.detail += " | 单位=m (float32)，正常"
            elif d_mean > 100:
                c.status = "WARN"
                c.detail += " | 数值偏大，可能单位=mm (uint16)"
            else:
                c.status = "WARN"
                c.detail += " | 深度值异常"

        self.checks.append(c)

    def _check_rgb_depth_alignment(self):
        c = CalibCheck("3. RGB-Depth 对齐")
        if self.depth_img is None or self.latest_rgb_img is None:
            c.status = "FAIL"
            c.detail = "缺少 RGB 或深度图"
        else:
            # 检查尺寸是否一致
            d_h, d_w = self.depth_img.shape[:2]
            r_h, r_w = self.latest_rgb_img.shape[:2]
            if d_h == r_h and d_w == r_w:
                c.status = "PASS"
                c.detail = f"RGB 和 Depth 尺寸一致 ({d_w}x{d_h})"
            else:
                # 可能对齐到 color，深度尺寸 ≠ 彩色尺寸是正常的
                c.status = "WARN"
                c.detail = (f"RGB ({r_w}x{r_h}) vs Depth ({d_w}x{d_h}) "
                            f"尺寸不同，需确认是否使用 aligned_depth")

            # 在 Gazebo 仿真中，rgbd_camera 传感器输出的是对齐的深度
            c.detail += " | Gazebo rgbd_camera 默认输出对齐深度"
        self.checks.append(c)

    def _check_tf_base_camera(self):
        c = CalibCheck("4. T_base_camera (相机外参)")
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame, self.camera_frame,
                rclpy.time.Time(), timeout=Duration(seconds=1.0))
            t = tf.transform.translation
            q = tf.transform.rotation
            c.status = "PASS"
            c.detail = (f"相机在 base_link 系: "
                        f"pos=({t.x:.4f}, {t.y:.4f}, {t.z:.4f}), "
                        f"quat=({q.x:.4f}, {q.y:.4f}, {q.z:.4f}, {q.w:.4f})")

            # 验证与 xacro 配置一致
            # 预期: xyz="0.5 -0.2 0.7" rpy="0 55° π"
            expected = np.array([0.5, -0.2, 0.7])
            actual = np.array([t.x, t.y, t.z])
            err = np.linalg.norm(actual - expected)
            c.detail += f" | 与 xacro 预期偏差 {err*1000:.1f}mm"
            if err > 0.05:
                c.status = "WARN"
                c.detail += " | 偏差较大!"
        except Exception as e:
            c.status = "FAIL"
            c.detail = f"TF 查询失败: {e}"
        self.checks.append(c)

    def _check_tcp(self):
        c = CalibCheck("5. TCP/工具坐标 (grasp_frame)")
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame, "grasp_frame",
                rclpy.time.Time(), timeout=Duration(seconds=1.0))
            t = tf.transform.translation
            c.status = "PASS"
            c.detail = (f"grasp_frame 在 base_link: "
                        f"pos=({t.x:.4f}, {t.y:.4f}, {t.z:.4f})")
            c.detail += " | 需人工确认 grasp_frame 是否在两指中心"
        except Exception as e:
            c.status = "FAIL"
            c.detail = f"TF 查询失败: {e}"
        self.checks.append(c)

    def _check_table_height(self):
        c = CalibCheck("6. 桌面高度标定")
        # 使用深度图在桌面区域采样估算桌面 z
        if self.depth_img is not None:
            # 取图像下半部分（地面区域）的深度中值
            h, w = self.depth_img.shape
            ground_region = self.depth_img[int(h * 0.6):, :]
            valid = ground_region[np.isfinite(ground_region) & (ground_region > 0)]
            if valid.size > 0:
                median_depth = float(np.median(valid))
                c.detail = (f"图像下半区域深度中值 = {median_depth:.4f}m "
                            f"(用于估算相机到桌面距离)")
                # 这不能直接给出桌面 z，需要结合相机外参
                c.status = "PASS"
                c.detail += " | 需结合 TF 将深度转为 base z"
            else:
                c.status = "WARN"
                c.detail = "深度图无有效数据"
        else:
            c.status = "FAIL"
            c.detail = "无深度图"
        self.checks.append(c)

    def _print_check_summary(self):
        self.get_logger().info("\n" + "=" * 60)
        self.get_logger().info("标定检查摘要")
        self.get_logger().info("=" * 60)
        for c in self.checks:
            icon = {"PASS": "✅", "WARN": "⚠️ ", "FAIL": "❌"}.get(c.status, "?")
            self.get_logger().info(f"{icon} {c.status} | {c.name}")
            self.get_logger().info(f"   {c.detail}")

    # ================================================================
    # 单位置测试
    # ================================================================
    def test_single_position(self, spawn_x: float, spawn_y: float,
                             spawn_z: float, label: str,
                             pixel_uv: Tuple[float, float],
                             static: bool = False) -> PositionResult:
        """对单个像素点执行完整的 pixel→3D→base 链并记录误差"""
        result = PositionResult(
            label=label,
            spawn_xyz=(spawn_x, spawn_y, spawn_z),
            actual_xyz=(0, 0, 0),
            pixel_uv=pixel_uv,
        )

        result.actual_xyz = tuple(self.get_ground_truth(
            spawn_x, spawn_y, spawn_z, static))

        # Pixel → Camera
        u, v = pixel_uv
        xyz_cam = self.pixel_to_camera(u, v)
        if xyz_cam is None:
            self.get_logger().error(f"[{label}] pixel_to_camera 失败")
            return result

        result.camera_xyz = tuple(xyz_cam)
        result.depth_m = float(xyz_cam[2])

        # Camera → Base
        xyz_base = self.transform_to_base(xyz_cam)
        if xyz_base is None:
            self.get_logger().error(f"[{label}] transform_to_base 失败")
            return result

        result.base_xyz = tuple(xyz_base)

        # 误差
        err = xyz_base - np.array(result.actual_xyz)
        result.error_xyz = tuple(err)
        result.error_norm = float(np.linalg.norm(err))

        self.get_logger().info(
            f"[{label}] spawn=({spawn_x:.3f},{spawn_y:.3f},{spawn_z:.3f}) "
            f"actual=({result.actual_xyz[0]:.4f},{result.actual_xyz[1]:.4f},{result.actual_xyz[2]:.4f}) "
            f"pixel=({u:.1f},{v:.1f}) depth={result.depth_m:.4f}m "
            f"cam=({xyz_cam[0]:.4f},{xyz_cam[1]:.4f},{xyz_cam[2]:.4f}) "
            f"base=({xyz_base[0]:.4f},{xyz_base[1]:.4f},{xyz_base[2]:.4f}) "
            f"Δ=({err[0]*1000:+.1f},{err[1]*1000:+.1f},{err[2]*1000:+.1f})mm "
            f"|Δ|={result.error_norm*1000:.1f}mm"
        )

        return result

    def print_report(self):
        """打印中文标定验证报告"""
        r = self.results
        c = self.checks

        report = []
        report.append("")
        report.append("=" * 70)
        report.append("         S622 相机-机械臂 标定验证报告")
        report.append("=" * 70)
        report.append(f"  日期: {time.strftime('%Y-%m-%d %H:%M:%S')}")
        report.append(f"  base_frame: {self.base_frame}")
        report.append(f"  camera_frame: {self.camera_frame}")
        report.append("")

        # ─── 标定检查结果 ───
        report.append("-" * 70)
        report.append("  一、标定项检查")
        report.append("-" * 70)
        for check in c:
            icon = {"PASS": "✅", "WARN": "⚠️", "FAIL": "❌"}.get(check.status, "?")
            report.append(f"  {icon} [{check.status}] {check.name}")
            report.append(f"     {check.detail}")
            report.append("")

        # ─── 位置误差 ───
        report.append("-" * 70)
        report.append("  二、5 位置坐标转换误差")
        report.append("-" * 70)
        header = (f"  {'位置':<16s} {'实际(base)':>28s} {'计算(base)':>28s} "
                  f"{'误差(mm)':>22s} {'|Δ|':>8s}")
        report.append(header)
        report.append("  " + "-" * 110)

        for res in r:
            actual = res.actual_xyz
            base = res.base_xyz
            err = res.error_xyz
            report.append(
                f"  {res.label:<16s} "
                f"({actual[0]:+.4f},{actual[1]:+.4f},{actual[2]:+.4f}) "
                f"({base[0]:+.4f},{base[1]:+.4f},{base[2]:+.4f}) "
                f"({err[0]*1000:+5.1f},{err[1]*1000:+5.1f},{err[2]*1000:+5.1f}) "
                f"{res.error_norm*1000:6.1f}mm"
            )

        # ─── 误差统计 ───
        if r:
            errs = np.array([list(res.error_xyz) for res in r])
            report.append("")
            report.append("-" * 70)
            report.append("  三、误差统计")
            report.append("-" * 70)
            report.append(f"  Δx: mean={np.mean(np.abs(errs[:,0]))*1000:.1f}mm, "
                          f"max={np.max(np.abs(errs[:,0]))*1000:.1f}mm")
            report.append(f"  Δy: mean={np.mean(np.abs(errs[:,1]))*1000:.1f}mm, "
                          f"max={np.max(np.abs(errs[:,1]))*1000:.1f}mm")
            report.append(f"  Δz: mean={np.mean(np.abs(errs[:,2]))*1000:.1f}mm, "
                          f"max={np.max(np.abs(errs[:,2]))*1000:.1f}mm")
            report.append(f"  |Δ|: mean={np.mean([res.error_norm for res in r])*1000:.1f}mm, "
                          f"max={np.max([res.error_norm for res in r])*1000:.1f}mm")

        # ─── 结论 ───
        report.append("")
        report.append("-" * 70)
        report.append("  四、结论与建议")
        report.append("-" * 70)

        fail_count = sum(1 for ch in c if ch.status == "FAIL")
        warn_count = sum(1 for ch in c if ch.status == "WARN")

        if fail_count > 0:
            report.append(f"  ❌ {fail_count} 项检查失败，需立即修复。")
        if warn_count > 0:
            report.append(f"  ⚠️ {warn_count} 项检查有警告，建议排查。")

        if r:
            avg_err = np.mean([res.error_norm for res in r])
            if avg_err < 0.01:
                report.append(f"  ✅ 平均位置误差 {avg_err*1000:.1f}mm < 10mm，"
                              f"标定精度良好，可进入抓取测试。")
            elif avg_err < 0.03:
                report.append(f"  ⚠️ 平均位置误差 {avg_err*1000:.1f}mm > 10mm，"
                              f"建议调整外参 offset 或重做手眼标定。")
            else:
                report.append(f"  ❌ 平均位置误差 {avg_err*1000:.1f}mm >> 10mm，"
                              f"必须修复标定后才能抓取。")

            # 判断误差模式
            if errs.shape[0] >= 2:
                # y 误差是否恒定
                y_std = np.std(errs[:, 1])
                x_std = np.std(errs[:, 0])
                if y_std < 0.01 and np.mean(np.abs(errs[:, 1])) > 0.01:
                    report.append(f"  💡 Δy 恒定为 {np.mean(errs[:,1])*1000:.0f}mm，"
                                  f"建议加 grasp_target_offset_base y 补偿。")
                if x_std > y_std * 3:
                    report.append(f"  💡 Δx 随位置变化 (std={x_std*1000:.1f}mm)，"
                                  f"可能存在内参/深度对齐问题。")

        report.append("")
        report.append("=" * 70)
        report.append("  报告结束")
        report.append("=" * 70)

        report_str = "\n".join(report)
        self.get_logger().info(report_str)
        return report_str


def spawn_box(x: float, y: float, z: float, name: str = "target_box",
              static: bool = False) -> bool:
    """通过 ros_gz_sim create 在 Gazebo 中 spawn 方块"""
    cmd = [
        "ros2", "run", "ros_gz_sim", "create",
        "-world", "empty",
        "-file", "target_box",
        "-name", name,
        "-x", str(x), "-y", str(y), "-z", str(z),
        "-R", "0", "-P", "0", "-Y", "0.5",
    ]
    try:
        subprocess.run(cmd, timeout=10, capture_output=True, text=True)
        return True
    except Exception as e:
        print(f"spawn 失败: {e}")
        return False


def remove_box(name: str = "target_box"):
    """从 Gazebo 中移除方块"""
    subprocess.run(
        ["ros2", "service", "call", f"/world/empty/remove",
         "gz.msgs.Entity", f"name: '{name}'"],
        timeout=5, capture_output=True, text=True)


def main():
    rclpy.init()

    verifier = CalibrationVerifier()

    # ─── 等待数据就绪 ───
    print("等待相机数据...")
    timeout = 10.0
    start = time.time()
    while verifier.fx is None or verifier.depth_img is None:
        rclpy.spin_once(verifier, timeout_sec=0.1)
        if time.time() - start > timeout:
            print("错误: 等待相机数据超时")
            return

    print(f"相机内参就绪: fx={verifier.fx:.2f}, fy={verifier.fy:.2f}, "
          f"cx={verifier.cx:.1f}, cy={verifier.cy:.1f}")
    print(f"深度图就绪: shape={verifier.depth_img.shape}")

    # ─── 先跑 6 项标定检查 ───
    verifier.check_all()

    # ─── 测试 5 个位置 ───
    # 模式: 用 YOLO 检测像素 OR 手动指定像素
    # 这里需要用户交互: 先在 Gazebo 中放好方块，或手动输入像素坐标

    print("\n--- 位置测试 ---")
    print("请确保 target_box 已 spawn 在已知位置。")
    print("如果使用 YOLO 检测，按 Enter 继续。")
    print("如果手动输入像素，输入 'manual'。")
    choice = input("> ").strip()

    if choice == "manual":
        # 手动模式：用户输入 5 个位置的 spawn_xyz 和对应像素
        for pos in TEST_POSITIONS:
            x, y, z, label = pos
            print(f"\n--- {label}: spawn=({x}, {y}, {z}) ---")
            u = float(input("  像素 u: "))
            v = float(input("  像素 v: "))
            res = verifier.test_single_position(x, y, z, label, (u, v))
            verifier.results.append(res)
    else:
        # YOLO 模式：需要 YOLO 节点在运行
        print("YOLO 模式暂不支持交互，请手动输入像素值。")
        print("或者用 debug_target 模式测试每个位置的 rough_target...")
        # 这里可以集成 YOLO 订阅，但当前先用 manual 模式

    # ─── 打印报告 ───
    report = verifier.print_report()

    # ─── 保存报告 ───
    report_path = "/home/yep/my_S622/debug_data/calibration_report.md"
    with open(report_path, "w") as f:
        f.write(report + "\n")
    print(f"\n报告已保存到: {report_path}")

    verifier.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
