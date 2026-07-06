#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""calibrate_grasp_offset.py

半自动标定 grasp_projection_offset_px (单次或迭代式).

工作流程:
  1. 脚本启动后,设置 visual_servo_node 进入 calibration_mode
  2. 临时把 grasp_projection_offset_px 置零 (或保留当前值用于增量标定)
  3. 用户放好物体,按回车
  4. 脚本发布 /servo_trigger=true, 等待机械臂跑完伺服 + 盲降并稳定
  5. 用户用尺子测量 "EE 指尖落点 - target 真实中心" 的水平偏差
     (Δx, Δy 单位 mm, 在 base_link 系下)
  6. 脚本读 TF 和 camera_info, 计算等效像素 offset
  7. 输出建议值 (累加到当前 offset 上, 实现迭代收敛)
  8. 用户选择: 应用新 offset 重测 / 写入 YAML / 退出

用法:
  ros2 run visual_servo calibrate_grasp_offset
  ros2 run visual_servo calibrate_grasp_offset --yaml /path/to/visual_servo.yaml

正负号约定 (重要!):
  Δx, Δy 是 "EE 落点位置 - target 真实位置" 的 base 系分量.
  例如 EE 落在 target 的 +x 方向 8mm, +y 方向 -3mm,
  则输入 Δx = 8, Δy = -3.

  指尖位置可以这样判断: 关闭夹爪 (脚本会提示是否手动闭爪) 后,
  以两指中心点为基准, 量到 target 几何中心的距离.
"""

import sys
import time
import math
import argparse
from typing import Optional, Tuple, List

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Bool
from yolov8_obb_msgs.msg import Yolov8Inference

from tf2_ros import Buffer, TransformListener

from rcl_interfaces.srv import SetParameters, GetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType

try:
    import yaml
    HAVE_YAML = True
except ImportError:
    HAVE_YAML = False


VS_NODE = "/visual_servo_node"


class GraspOffsetCalibrator(Node):
    def __init__(self):
        super().__init__("calibrate_grasp_offset")

        self.cb_group = ReentrantCallbackGroup()

        # ---- subscribers ----
        self.fx: Optional[float] = None
        self.fy: Optional[float] = None
        self.cx: Optional[float] = None
        self.cy: Optional[float] = None
        self.image_width: Optional[int] = None
        self.image_height: Optional[int] = None

        self.target_uv_smooth: Optional[np.ndarray] = None

        self.create_subscription(
            CameraInfo, "/camera/color/camera_info",
            self.cb_info, 10, callback_group=self.cb_group)
        self.create_subscription(
            Yolov8Inference, "/yolov8/obb_detections",
            self.cb_det, 10, callback_group=self.cb_group)

        # ---- publishers ----
        self.trigger_pub = self.create_publisher(Bool, "/servo_trigger", 10)

        # ---- TF ----
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ---- service clients to visual_servo_node ----
        self.set_param_cli = self.create_client(
            SetParameters, f"{VS_NODE}/set_parameters",
            callback_group=self.cb_group)
        self.get_param_cli = self.create_client(
            GetParameters, f"{VS_NODE}/get_parameters",
            callback_group=self.cb_group)

        # ---- frames (and try reading from visual_servo_node) ----
        self.base_frame = "base_link"
        self.camera_frame = "camera_color_optical_frame"
        self.ee_frame = "grasp_frame"

        self.get_logger().info("calibrate_grasp_offset node ready")

    def cb_info(self, msg: CameraInfo):
        self.fx = float(msg.k[0])
        self.fy = float(msg.k[4])
        self.cx = float(msg.k[2])
        self.cy = float(msg.k[5])
        self.image_width = msg.width
        self.image_height = msg.height

    def cb_det(self, msg: Yolov8Inference):
        if not msg.results:
            return
        best = max(msg.results, key=lambda r: r.confidence)
        uv = np.array([best.center_x, best.center_y], dtype=float)
        if self.target_uv_smooth is None:
            self.target_uv_smooth = uv.copy()
        else:
            alpha = 0.2
            self.target_uv_smooth = (1 - alpha) * self.target_uv_smooth + alpha * uv

    # ----------------------------------------------------------------------
    # Param helpers
    # ----------------------------------------------------------------------
    def set_param(self, name: str, value):
        if not self.set_param_cli.wait_for_service(timeout_sec=3.0):
            raise RuntimeError(f"{VS_NODE}/set_parameters service unavailable")

        param = Parameter()
        param.name = name
        pv = ParameterValue()
        if isinstance(value, bool):
            pv.type = ParameterType.PARAMETER_BOOL
            pv.bool_value = value
        elif isinstance(value, int):
            pv.type = ParameterType.PARAMETER_INTEGER
            pv.integer_value = value
        elif isinstance(value, float):
            pv.type = ParameterType.PARAMETER_DOUBLE
            pv.double_value = value
        elif isinstance(value, str):
            pv.type = ParameterType.PARAMETER_STRING
            pv.string_value = value
        elif isinstance(value, list):
            pv.type = ParameterType.PARAMETER_DOUBLE_ARRAY
            pv.double_array_value = [float(x) for x in value]
        else:
            raise ValueError(f"unsupported type for {name}: {type(value)}")
        param.value = pv

        req = SetParameters.Request()
        req.parameters = [param]
        future = self.set_param_cli.call_async(req)
        self._wait_future(future, 3.0)
        result = future.result()
        if not result or not result.results or not result.results[0].successful:
            reason = (result.results[0].reason
                      if result and result.results else "unknown")
            raise RuntimeError(f"set_param {name}={value} failed: {reason}")
        self.get_logger().info(f"set {name} = {value}")

    def get_param_double_array(self, name: str) -> Optional[List[float]]:
        if not self.get_param_cli.wait_for_service(timeout_sec=2.0):
            return None
        req = GetParameters.Request()
        req.names = [name]
        future = self.get_param_cli.call_async(req)
        self._wait_future(future, 3.0)
        result = future.result()
        if not result or not result.values:
            return None
        v = result.values[0]
        if v.type == ParameterType.PARAMETER_DOUBLE_ARRAY:
            return list(v.double_array_value)
        return None

    def _wait_future(self, future, timeout_sec: float):
        deadline = time.time() + timeout_sec
        while rclpy.ok() and not future.done():
            if time.time() > deadline:
                raise RuntimeError("future timeout")
            time.sleep(0.02)

    # ----------------------------------------------------------------------
    # TF / Jacobian
    # ----------------------------------------------------------------------
    def lookup_ee_pos(self) -> Optional[np.ndarray]:
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame, self.ee_frame,
                rclpy.time.Time(), timeout=Duration(seconds=0.3))
        except Exception:
            return None
        t = tf.transform.translation
        return np.array([t.x, t.y, t.z], dtype=float)

    def lookup_camera_in_base(self) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame, self.camera_frame,
                rclpy.time.Time(), timeout=Duration(seconds=0.3))
        except Exception:
            return None
        q = tf.transform.rotation
        R = np.array([
            [1-2*q.y*q.y-2*q.z*q.z, 2*q.x*q.y-2*q.z*q.w, 2*q.x*q.z+2*q.y*q.w],
            [2*q.x*q.y+2*q.z*q.w, 1-2*q.x*q.x-2*q.z*q.z, 2*q.y*q.z-2*q.x*q.w],
            [2*q.x*q.z-2*q.y*q.w, 2*q.y*q.z+2*q.x*q.w, 1-2*q.x*q.x-2*q.y*q.y],
        ], dtype=float)
        t = np.array([tf.transform.translation.x,
                      tf.transform.translation.y,
                      tf.transform.translation.z], dtype=float)
        return R, t

    def compute_pixel_jacobian_at(self, point_base: np.ndarray) -> Optional[np.ndarray]:
        """J: 2x2,  Δuv = J @ Δ(base_xy).
        point_base: 投影点的 base 系位置 (x, y, z).
        """
        if self.fx is None:
            return None
        cam = self.lookup_camera_in_base()
        if cam is None:
            return None
        R, t_cam = cam
        p_cam = R.T @ (point_base - t_cam)
        x, y, z = p_cam[0], p_cam[1], p_cam[2]
        if z <= 1e-6:
            return None
        du_dpc = np.array([self.fx/z, 0.0, -self.fx*x/(z*z)])
        dv_dpc = np.array([0.0, self.fy/z, -self.fy*y/(z*z)])
        # dp_cam/dp_base[xy] = R.T 的 0/1 列 = R 的 0/1 行
        J = np.array([
            [np.dot(du_dpc, R[0, :]), np.dot(du_dpc, R[1, :])],
            [np.dot(dv_dpc, R[0, :]), np.dot(dv_dpc, R[1, :])],
        ])
        return J

    # ----------------------------------------------------------------------
    # Stability detection
    # ----------------------------------------------------------------------
    def wait_for_ee_stable(self, stable_dur: float = 1.5,
                           pos_eps: float = 0.002,
                           max_wait: float = 60.0) -> Optional[np.ndarray]:
        """等待 EE 在 pos_eps (m) 内连续 stable_dur 秒内位置不变, 返回稳定后的 EE 位置."""
        self.get_logger().info(
            f"waiting for EE to stabilize "
            f"(eps={pos_eps*1000:.1f}mm, dur={stable_dur:.1f}s, max={max_wait:.0f}s)..."
        )
        start = time.time()
        last_pos = None
        last_move_time = time.time()

        while rclpy.ok() and time.time() - start < max_wait:
            time.sleep(0.1)
            pos = self.lookup_ee_pos()
            if pos is None:
                continue
            if last_pos is None:
                last_pos = pos
                last_move_time = time.time()
                continue
            if np.linalg.norm(pos - last_pos) > pos_eps:
                last_pos = pos
                last_move_time = time.time()
            else:
                if time.time() - last_move_time >= stable_dur:
                    return pos
        return None

    def wait_for_intrinsics(self, timeout_sec: float = 10.0) -> bool:
        start = time.time()
        while rclpy.ok() and self.fx is None:
            if time.time() - start > timeout_sec:
                return False
            time.sleep(0.1)
        return True

    def wait_for_target_uv(self, timeout_sec: float = 5.0) -> Optional[np.ndarray]:
        start = time.time()
        while rclpy.ok() and self.target_uv_smooth is None:
            if time.time() - start > timeout_sec:
                return None
            time.sleep(0.1)
        return self.target_uv_smooth.copy()


def prompt_float(prompt: str, default: Optional[float] = None) -> float:
    while True:
        s = input(prompt).strip()
        if not s and default is not None:
            return default
        try:
            return float(s)
        except ValueError:
            print("  invalid number, try again")


def prompt_yes_no(prompt: str, default: bool = True) -> bool:
    dh = "Y/n" if default else "y/N"
    while True:
        s = input(f"{prompt} [{dh}]: ").strip().lower()
        if not s:
            return default
        if s in ("y", "yes"):
            return True
        if s in ("n", "no"):
            return False


def write_yaml_offset(yaml_path: str, new_offset: List[float]):
    """把 grasp_projection_offset_px 写到 yaml_path.
    
    支持两种结构:
      - 顶层 key:  grasp_projection_offset_px: [...]
      - ros2 风格: visual_servo_node: { ros__parameters: { ... } }
    """
    if not HAVE_YAML:
        raise RuntimeError("PyYAML not installed; cannot write YAML")

    with open(yaml_path, "r") as f:
        data = yaml.safe_load(f) or {}

    target_key = "grasp_projection_offset_px"

    # 尝试 ros2 嵌套结构
    placed = False
    for node_key, node_val in data.items():
        if isinstance(node_val, dict) and "ros__parameters" in node_val:
            params = node_val["ros__parameters"]
            if isinstance(params, dict):
                params[target_key] = [float(x) for x in new_offset]
                placed = True
                break

    if not placed:
        # 直接顶层
        data[target_key] = [float(x) for x in new_offset]

    with open(yaml_path, "w") as f:
        yaml.safe_dump(data, f, default_flow_style=False, sort_keys=False)


def calibration_session(node: GraspOffsetCalibrator, yaml_path: Optional[str]):
    print()
    print("=" * 70)
    print("  Grasp Projection Offset Calibration")
    print("=" * 70)
    print()
    print("Pre-flight checks:")
    print(f"  - visual_servo_node alive:           checking...")

    # 等待依赖
    if not node.wait_for_intrinsics(timeout_sec=10.0):
        print("  FAIL: camera_info not received within 10s")
        return

    print(f"  - camera intrinsics: fx={node.fx:.1f} fy={node.fy:.1f} "
          f"cx={node.cx:.1f} cy={node.cy:.1f} ({node.image_width}x{node.image_height})")

    # 读当前 offset (用于增量标定)
    current_offset = node.get_param_double_array("grasp_projection_offset_px")
    if current_offset is None or len(current_offset) != 2:
        current_offset = [0.0, 0.0]
    print(f"  - current grasp_projection_offset_px: "
          f"[{current_offset[0]:+.2f}, {current_offset[1]:+.2f}]")

    if not prompt_yes_no("Reset offset to [0,0] before calibration?", default=False):
        print(f"keeping current offset {current_offset} as starting point")
        starting_offset = list(current_offset)
    else:
        starting_offset = [0.0, 0.0]
        node.set_param("grasp_projection_offset_px", starting_offset)

    print()
    print("Enabling calibration mode (gripper will NOT close after descent)...")
    node.set_param("calibration_mode", True)
    node.set_param("enable_motion", True)

    accumulated_offset = np.array(starting_offset, dtype=float)
    iteration = 0

    while True:
        iteration += 1
        print()
        print("-" * 70)
        print(f"  Iteration #{iteration}")
        print("-" * 70)
        print()
        print("1) Place the target object on the workspace")
        print("2) Make sure YOLO is detecting it (check /yolov8/obb_detections)")
        input("3) Press Enter to trigger grasp...")

        # 触发抓取
        node.trigger_pub.publish(Bool(data=True))
        print("trigger sent, waiting for descent to complete...")

        # 等待 EE 稳定
        ee_pos = node.wait_for_ee_stable(stable_dur=1.5, pos_eps=0.002, max_wait=60.0)
        if ee_pos is None:
            print("ERROR: EE did not stabilize within 60s, aborting iteration")
            if not prompt_yes_no("Retry?", default=True):
                break
            continue

        # 拿目标像素 (用于诊断)
        target_uv = node.wait_for_target_uv(timeout_sec=2.0)

        print()
        print(f"EE position (base): "
              f"x={ee_pos[0]:+.4f} y={ee_pos[1]:+.4f} z={ee_pos[2]:+.4f}")
        if target_uv is not None:
            print(f"YOLO target_uv: ({target_uv[0]:.1f}, {target_uv[1]:.1f})")
        else:
            print("YOLO target_uv: unavailable (object may be occluded by gripper)")

        # 算雅可比
        J = node.compute_pixel_jacobian_at(ee_pos)
        if J is None:
            print("ERROR: cannot compute pixel jacobian (TF or intrinsics missing)")
            break
        print(f"pixel jacobian J (Δuv = J @ Δxy):\n{J}")
        print(f"   |  meaning: 1mm in base x -> {J[0,0]*1e-3:+.2f}px in u, "
              f"{J[1,0]*1e-3:+.2f}px in v")
        print(f"   |           1mm in base y -> {J[0,1]*1e-3:+.2f}px in u, "
              f"{J[1,1]*1e-3:+.2f}px in v")

        print()
        print("== Measure the landing error ==")
        print("Look at the gripper tip and the target object. Measure the offset:")
        print("  Δx = (EE tip x) - (target center x)  in base_link's +x direction (mm)")
        print("  Δy = (EE tip y) - (target center y)  in base_link's +y direction (mm)")
        print("Example: if EE landed 8mm to base +x and 3mm to base -y of target,")
        print("  enter Δx=8, Δy=-3")
        print()

        dx_mm = prompt_float("  Δx (mm)? ")
        dy_mm = prompt_float("  Δy (mm)? ")

        delta_xy = np.array([dx_mm * 1e-3, dy_mm * 1e-3], dtype=float)
        delta_uv = J @ delta_xy

        print()
        print(f"computed pixel correction: Δoffset_uv = ({delta_uv[0]:+.2f}, "
              f"{delta_uv[1]:+.2f}) px")

        new_offset = accumulated_offset + delta_uv
        print(f"current offset:    [{accumulated_offset[0]:+.2f}, "
              f"{accumulated_offset[1]:+.2f}]")
        print(f"new offset to use: [{new_offset[0]:+.2f}, {new_offset[1]:+.2f}]")
        print()

        # 退出标定模式以释放机械臂(回到 IDLE)
        node.trigger_pub.publish(Bool(data=False))
        time.sleep(0.5)

        # 决策
        print("Options:")
        print("  [a] Apply new offset and run another iteration to verify")
        print("  [s] Save to YAML and exit")
        print("  [p] Print and exit (no save)")
        print("  [r] Retry iteration with same offset")
        print("  [q] Quit without applying")
        choice = input("> ").strip().lower()

        if choice == "a":
            node.set_param("grasp_projection_offset_px", new_offset.tolist())
            accumulated_offset = new_offset
            continue
        elif choice == "s":
            node.set_param("grasp_projection_offset_px", new_offset.tolist())
            accumulated_offset = new_offset
            if yaml_path:
                try:
                    write_yaml_offset(yaml_path, new_offset.tolist())
                    print(f"wrote to {yaml_path}")
                except Exception as e:
                    print(f"failed to write YAML: {e}")
                    print(f"manual entry: grasp_projection_offset_px: "
                          f"[{new_offset[0]:+.2f}, {new_offset[1]:+.2f}]")
            else:
                print("no --yaml provided. Add to your config manually:")
                print(f"  grasp_projection_offset_px: "
                      f"[{new_offset[0]:+.2f}, {new_offset[1]:+.2f}]")
            break
        elif choice == "p":
            print("Add to your config manually:")
            print(f"  grasp_projection_offset_px: "
                  f"[{new_offset[0]:+.2f}, {new_offset[1]:+.2f}]")
            break
        elif choice == "r":
            continue
        elif choice == "q":
            print("quitting, no changes applied")
            break
        else:
            print("unknown choice, treating as quit")
            break

    print()
    print("Disabling calibration mode...")
    try:
        node.set_param("calibration_mode", False)
    except Exception as e:
        print(f"warning: failed to disable calibration_mode: {e}")

    print()
    print("Calibration session done.")


def main(argv=None):
    parser = argparse.ArgumentParser(description="Calibrate grasp_projection_offset_px")
    parser.add_argument("--yaml", type=str, default=None,
                        help="YAML config file to update (optional)")
    args = parser.parse_args(argv)

    rclpy.init()
    node = GraspOffsetCalibrator()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)

    import threading
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        calibration_session(node, args.yaml)
    except KeyboardInterrupt:
        print("\ninterrupted")
        try:
            node.set_param("calibration_mode", False)
            node.trigger_pub.publish(Bool(data=False))
        except Exception:
            pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
