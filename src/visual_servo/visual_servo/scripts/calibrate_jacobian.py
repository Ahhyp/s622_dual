#!/usr/bin/env python3
"""
j_img_to_base 标定脚本

通过实际移动机械臂末端，观测投影像素的变化，直接测量图像→base 的 Jacobian。

原理:
  J_uv_xy = [∂u/∂x, ∂u/∂y; ∂v/∂x, ∂v/∂y]   (px/m)
  在当前位置附近，分别沿 base_x 和 base_y 微动 Δd
  → 收集 (Δx,Δy)→(Δu,Δv) 样本
  → 最小二乘回归 UV = P @ J_uv_xy.T
  → j_img_to_base = J_uv_xy^{-1}             (m/px)

用法:
  1. 仿真运行中，机械臂已就位 (MoveIt 规划后)
  2. 确保 visual_servo_node 不在运行 (会抢 /servo_node/delta_twist_cmds)
  3. ros2 run visual_servo calibrate_jacobian
"""

import sys
import time
import math
import subprocess as _sp
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TwistStamped, PointStamped
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs
from cv_bridge import CvBridge


STEP_DIST = 0.015       # 每步移动距离 (m)，15mm 避免非线性
MOVE_DURATION = 0.6     # 每次移动的持续秒数
SETTLE_TIME = 1.5       # 移动后等待稳定的秒数
SAMPLES = 16            # 总采样轮数 (正反各 8 轮)


class JacobianCalibrator(Node):
    def __init__(self):
        super().__init__("calibrate_jacobian")

        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("camera_frame", "camera_color_optical_frame")
        self.declare_parameter("ee_frame", "grasp_frame")
        self.declare_parameter("step_dist", STEP_DIST)
        self.declare_parameter("move_duration", MOVE_DURATION)
        self.declare_parameter("settle_time", SETTLE_TIME)
        self.declare_parameter("samples", SAMPLES)
        self.declare_parameter("debug_mode", True)

        self.base_frame = self.get_parameter("base_frame").value
        self.camera_frame = self.get_parameter("camera_frame").value
        self.ee_frame = self.get_parameter("ee_frame").value
        self.step_dist = float(self.get_parameter("step_dist").value)
        self.move_duration = float(self.get_parameter("move_duration").value)
        self.settle_time = float(self.get_parameter("settle_time").value)
        self.n_samples = int(self.get_parameter("samples").value)
        self.debug = bool(self.get_parameter("debug_mode").value)

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 内参
        self.fx: Optional[float] = None
        self.fy: Optional[float] = None
        self.cx: Optional[float] = None
        self.cy: Optional[float] = None

        # ROS 接口
        self.create_subscription(CameraInfo, "/camera/color/camera_info",
                                 self._cb_info, 10)

        self.bridge = CvBridge()
        self.twist_pub = self.create_publisher(
            TwistStamped, "/servo_node/delta_twist_cmds", 10)

        self.start_servo_cli = self.create_client(Trigger, "/servo_node/start_servo")
        self.stop_servo_cli = self.create_client(Trigger, "/servo_node/stop_servo")

    # ─── 回调 ───────────────────────────────────────────────
    def _cb_info(self, msg: CameraInfo):
        self.fx = float(msg.k[0])
        self.fy = float(msg.k[4])
        self.cx = float(msg.k[2])
        self.cy = float(msg.k[5])

    # ─── TF 工具 ───────────────────────────────────────────
    def _lookup_ee_in_base(self) -> Optional[np.ndarray]:
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame, self.ee_frame,
                rclpy.time.Time(), timeout=Duration(seconds=0.5))
        except Exception as e:
            self.get_logger().warning(f"TF lookup failed: {e}")
            return None
        t = tf.transform.translation
        return np.array([t.x, t.y, t.z], dtype=float)

    def _project_to_pixel(self, pos_base: np.ndarray) -> Optional[np.ndarray]:
        if self.fx is None:
            return None
        pt = PointStamped()
        pt.header.frame_id = self.base_frame
        pt.header.stamp = rclpy.time.Time().to_msg()
        pt.point.x = float(pos_base[0])
        pt.point.y = float(pos_base[1])
        pt.point.z = float(pos_base[2])
        try:
            pt_cam = self.tf_buffer.transform(
                pt, self.camera_frame, timeout=Duration(seconds=0.5))
        except Exception:
            return None
        x, y, z = pt_cam.point.x, pt_cam.point.y, pt_cam.point.z
        if z <= 1e-6:
            return None
        u = self.fx * x / z + self.cx
        v = self.fy * y / z + self.cy
        return np.array([u, v], dtype=float)

    # ─── Servo ──────────────────────────────────────────────
    def _send_twist(self, vx: float, vy: float, vz: float):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.base_frame
        msg.twist.linear.x = vx
        msg.twist.linear.y = vy
        msg.twist.linear.z = vz
        self.twist_pub.publish(msg)

    def _spin_and_sleep(self, duration: float):
        """spin + sleep，保证 TF buffer 持续更新"""
        deadline = time.time() + duration
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

    def _move_for_duration(self, vx: float, vy: float):
        """持续发 twist 一段时间，带 spin"""
        deadline = time.time() + self.move_duration
        while time.time() < deadline:
            self._send_twist(vx, vy, 0.0)
            rclpy.spin_once(self, timeout_sec=0.02)
        self._send_twist(0.0, 0.0, 0.0)

    def _move_and_measure(self, vx: float, vy: float) -> Optional[dict]:
        """沿 (vx,vy) 方向移动并测量 EE 位移 + 投影像素变化。

        返回 {"dx","dy","du","dv"} 或 None。
        修复点:
          - 正向移动和反向回退都用 tight spin loop
          - 所有 sleep 都用 _spin_and_sleep
          - 存储每轴实际位移 (dx,dy) 供最小二乘回归
        """
        rclpy.spin_once(self, timeout_sec=0.1)

        ee_before = self._lookup_ee_in_base()
        if ee_before is None:
            return None
        uv_before = self._project_to_pixel(ee_before)
        if uv_before is None:
            return None

        # ── 正向移动 ──
        self._move_for_duration(vx, vy)
        self._spin_and_sleep(self.settle_time)
        rclpy.spin_once(self, timeout_sec=0.2)

        ee_after = self._lookup_ee_in_base()
        if ee_after is None:
            return None
        uv_after = self._project_to_pixel(ee_after)
        if uv_after is None:
            return None

        dx = ee_after[0] - ee_before[0]
        dy = ee_after[1] - ee_before[1]
        du = uv_after[0] - uv_before[0]
        dv = uv_after[1] - uv_before[1]

        actual_d = math.hypot(dx, dy)
        if actual_d < 0.002:
            self.get_logger().warning(f"  实际位移 {actual_d*1000:.1f}mm < 2mm, 臂可能没动")
            return None

        if self.debug:
            self.get_logger().info(
                f"  ee: ({ee_before[0]:.4f},{ee_before[1]:.4f}) → "
                f"({ee_after[0]:.4f},{ee_after[1]:.4f}) "
                f"Δ=({dx*1000:+.1f},{dy*1000:+.1f})mm | "
                f"uv: ({uv_before[0]:.1f},{uv_before[1]:.1f}) → "
                f"({uv_after[0]:.1f},{uv_after[1]:.1f}) "
                f"Δu={du:+.1f} Δv={dv:+.1f}"
            )

        # ── 反向回退 (tight loop, 修复硬 bug) ──
        self._move_for_duration(-vx, -vy)
        self._spin_and_sleep(self.settle_time)

        return {"dx": dx, "dy": dy, "du": du, "dv": dv}

    # ─── 主流程 ─────────────────────────────────────────────
    def run(self):
        self.get_logger().info("=" * 60)
        self.get_logger().info("j_img_to_base 标定")
        self.get_logger().info("=" * 60)

        self.get_logger().info("等待相机内参...")
        start = time.time()
        while self.fx is None:
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - start > 10:
                self.get_logger().error("超时：未收到 CameraInfo")
                return

        self.get_logger().info(
            f"内参就绪: fx={self.fx:.1f} fy={self.fy:.1f} "
            f"cx={self.cx:.1f} cy={self.cy:.1f}")

        # 2. 停掉 visual_servo_node (避免抢 twist 话题)
        self.get_logger().info("正在关闭 visual_servo_node...")
        _sp.run(["pkill", "-f", "visual_servo_node"], timeout=5)
        time.sleep(1.0)

        # 3. 启动 servo
        if not self.start_servo_cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("servo 服务不可用, 退出")
            return
        future = self.start_servo_cli.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        if not (future.result() and future.result().success):
            self.get_logger().error("servo 启动失败, 退出")
            return
        self.get_logger().info("servo 已启动")

        # 4. 测量 (正负方向交替, 供最小二乘回归抵消偏置)
        self._measurements = []
        vx_pos = self.step_dist / self.move_duration
        vy_pos = self.step_dist / self.move_duration

        self.get_logger().info(
            f"开始测量: step={self.step_dist*1000:.0f}mm, "
            f"duration={self.move_duration}s, "
            f"settle={self.settle_time}s, "
            f"samples={self.n_samples}")

        for i in range(self.n_samples):
            # 正负交替
            sgn = 1 if i % 2 == 0 else -1
            self.get_logger().info(
                f"--- 采样 {i+1}/{self.n_samples} (sign={sgn:+d}) ---")

            m_x = self._move_and_measure(vx_pos * sgn, 0.0)
            if m_x is not None:
                self._measurements.append(m_x)

            m_y = self._move_and_measure(0.0, vy_pos * sgn)
            if m_y is not None:
                self._measurements.append(m_y)

        # 5. 停 servo
        future = self.stop_servo_cli.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)

        # 6. 最小二乘回归
        if len(self._measurements) < 4:
            self.get_logger().error(f"有效测量不足 ({len(self._measurements)} < 4)!")
            return

        P = np.array([[m["dx"], m["dy"]] for m in self._measurements])  # N×2
        UV = np.array([[m["du"], m["dv"]] for m in self._measurements])  # N×2

        # UV = P @ J_uv_xy.T  →  J_uv_xy.T = pinv(P) @ UV
        J_uv_xy = np.linalg.lstsq(P, UV, rcond=None)[0].T  # 2×2

        self.get_logger().info(
            f"\nJ_uv_xy (px/m, 最小二乘):\n"
            f"  [[{J_uv_xy[0,0]:.1f}, {J_uv_xy[0,1]:.1f}]\n"
            f"   [{J_uv_xy[1,0]:.1f}, {J_uv_xy[1,1]:.1f}]]")

        try:
            J_inv = np.linalg.inv(J_uv_xy)
        except np.linalg.LinAlgError:
            self.get_logger().error("Jacobian 不可逆!")
            return

        self.get_logger().info(
            f"\n✅ j_img_to_base (m/px):\n"
            f"  [[{J_inv[0,0]:.6f}, {J_inv[0,1]:.6f}]\n"
            f"   [{J_inv[1,0]:.6f}, {J_inv[1,1]:.6f}]]")

        j_flat = [J_inv[0,0], J_inv[0,1], J_inv[1,0], J_inv[1,1]]

        print("\n" + "=" * 60)
        print(" 标定结果 — 复制到启动参数:")
        print("=" * 60)
        print(f"  -p j_img_to_base:=\"[{j_flat[0]:.6f}, {j_flat[1]:.6f}, "
              f"{j_flat[2]:.6f}, {j_flat[3]:.6f}]\"")
        print("")
        print(f"  ros2 run visual_servo visual_servo_node --ros-args \\")
        print(f"    -p j_img_to_base:=\"[{j_flat[0]:.6f}, {j_flat[1]:.6f}, "
              f"{j_flat[2]:.6f}, {j_flat[3]:.6f}]\"")
        print("=" * 60)

        # 诊断
        self._diagnose(P, UV, J_uv_xy, J_inv)

    def _diagnose(self, P, UV, J_uv_xy, J_inv):
        self.get_logger().info("\n--- 诊断 ---")
        # 残差
        UV_pred = P @ J_uv_xy.T
        residuals = np.linalg.norm(UV - UV_pred, axis=1)
        rmse = np.sqrt(np.mean(residuals**2))
        self.get_logger().info(
            f"  回归 RMSE: {rmse:.1f}px (N={len(P)}) "
            f"{'✅' if rmse < 5 else '⚠️ 噪声较大'}")

        j_norm = np.linalg.norm(J_inv)
        if j_norm < 1e-6:
            self.get_logger().error("j_img_to_base 接近零矩阵! 标定可能失败。")
        elif j_norm > 0.1:
            self.get_logger().warning(
                "j_img_to_base 值异常大，建议降低 gain 或增大 max_step")
        else:
            self.get_logger().info(
                f"  ||J_inv|| = {j_norm:.4f} — 值在正常范围")


def main():
    rclpy.init()
    node = JacobianCalibrator()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
