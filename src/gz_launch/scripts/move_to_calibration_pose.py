#!/usr/bin/env python3
"""[M2.7] 把右臂 grasp_frame 送到标定位姿，并打印到达后的关节角。

用法（仿真运行时）：
  ros2 run gz_launch move_to_calibration_pose.py \
      --position 0.0 0.35 0.62 --quat 0 0.7071 0 0.7071

position/quat 是 **right_base_link 系**下 right_grasp_frame 的目标位姿
（MoveIt planning frame；右臂 base 在 world (-0.35,0,0) yaw=0，故 right_base 坐标 =
world 坐标 + (0.35, 0, 0)）。板在 grasp_frame 上方 0.08m。
到达后打印 6 个右臂关节角（right_j1..j6，弧度），用于写进 dual_arm_config.yaml 的
calibration_board_visible named pose。

先查相机 TF 确定精确目标：
  ros2 run tf2_ros tf2_echo world camera_color_optical_frame
"""

import argparse
import math
import sys

import numpy as np
import rclpy
from geometry_msgs.msg import Point, Quaternion
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState


def _clamp_quat(q):
    norm = math.sqrt(sum(v * v for v in q))
    return tuple(v / norm for v in q)


class MoveToCalibrationPose(Node):
    def __init__(self, position, quat, tol_ori=0.5):
        super().__init__("move_to_calibration_pose")
        from pymoveit2 import MoveIt2

        self._joint_lock = __import__("threading").Lock()
        self._joint_history = []
        self._tol_ori = float(tol_ori)
        # joint_state_broadcaster 默认 RELIABLE；BEST_EFFORT 订阅收不到 → 用 RELIABLE
        self.create_subscription(
            JointState, "/joint_states", self._on_joint_state,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE),
        )

        self._arm = MoveIt2(
            node=self,
            joint_names=["right_j1", "right_j2", "right_j3", "right_j4", "right_j5", "right_j6"],
            base_link_name="right_base_link",
            end_effector_name="right_grasp_frame",
            group_name="right_arm",
            ignore_new_calls_while_executing=False,
            callback_group=MutuallyExclusiveCallbackGroup(),
            move_group_namespace="/move_group_fairino",
        )
        self._arm.max_velocity = 0.2
        self._arm.max_acceleration = 0.2
        self._arm.allowed_planning_time = 10.0
        self._position = position
        self._quat = quat

    def _on_joint_state(self, msg):
        values = dict(zip(msg.name, msg.position))
        try:
            joints = tuple(float(values[n]) for n in
                           ["right_j1", "right_j2", "right_j3", "right_j4", "right_j5", "right_j6"])
        except (KeyError, TypeError):
            return
        if all(math.isfinite(v) for v in joints):
            with self._joint_lock:
                self._joint_history.append(joints)

    def _wait_joints(self, timeout=10.0):
        import time
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and rclpy.ok():
            # executor 线程负责 spin；这里只轮询
            with self._joint_lock:
                if self._joint_history:
                    return self._joint_history[-1]
            time.sleep(0.02)
        return None

    def _wait_moveit_ready(self, timeout=30.0):
        """等待 move_group 的 plan/execute 服务就绪（rclpy 服务发现有延迟）。"""
        import time
        deadline = time.monotonic() + timeout
        plan = getattr(self._arm, "_plan_kinematic_path_service", None) \
            or getattr(self._arm, "_plan_kinematic_path_client", None)
        execute = getattr(self._arm, "_execute_trajectory_action_client", None)
        while time.monotonic() < deadline and rclpy.ok():
            # executor 线程负责 spin；这里只轮询服务状态
            if plan is not None and execute is not None \
                    and plan.service_is_ready() and execute.server_is_ready():
                return True
            time.sleep(0.1)
        return False

    def run(self):
        if not self._wait_moveit_ready():
            self.get_logger().error("move_group plan/execute services not ready")
            return False
        joints_before = self._wait_joints()
        if joints_before is None:
            self.get_logger().error("no /joint_states")
            return False
        self.get_logger().info(f"before joints (rad): {[round(v, 4) for v in joints_before]}")

        self.get_logger().info(
            f"planning/executing to position={self._position} quat={self._quat} "
            f"tol_ori={self._tol_ori} rad ..."
        )
        self._arm.move_to_pose(
            position=Point(x=self._position[0], y=self._position[1], z=self._position[2]),
            quat_xyzw=Quaternion(x=self._quat[0], y=self._quat[1], z=self._quat[2], w=self._quat[3]),
            # frame_id 用右臂 base（MoveIt planning frame；Gazebo 里 "world" 根 frame 不存在）
            frame_id="right_base_link",
            tolerance_orientation=self._tol_ori,
        )
        self._arm.wait_until_executed()

        import time
        time.sleep(1.0)
        joints_after = self._wait_joints()
        if joints_after is None:
            self.get_logger().error("no /joint_states after move")
            return False
        self.get_logger().info(f"after  joints (rad): {[round(v, 4) for v in joints_after]}")
        self.get_logger().info(
            "JOINT_RECORD = [" + ", ".join(f"{v:.6f}" for v in joints_after) + "]"
        )
        return True


def main():
    import threading
    from rclpy.executors import MultiThreadedExecutor

    parser = argparse.ArgumentParser()
    parser.add_argument("--position", nargs=3, type=float, required=True,
                        help="world 系目标位置 x y z (m)")
    parser.add_argument("--quat", nargs=4, type=float, required=True,
                        help="world 系目标四元数 x y z w")
    parser.add_argument("--tol-ori", type=float, default=0.5,
                        help="姿态容差 (rad)，放宽以让 MoveIt 在目标位置自由选姿态（默认 0.5）")
    args = parser.parse_args()

    rclpy.init()
    node = MoveToCalibrationPose(tuple(args.position), _clamp_quat(tuple(args.quat)),
                                 tol_ori=args.tol_ori)
    # pymoveit2 的 future/action 需要节点被 spin：后台 executor 线程
    executor = MultiThreadedExecutor(2)
    executor.add_node(node)
    executor_thread = threading.Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()
    try:
        ok = node.run()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
