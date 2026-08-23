#!/usr/bin/env python3
"""MoveIt 粗对齐规划 + servo 开关协调。

职责:
  1. 规划执行到指定位姿（位置 + 朝下姿态 + yaw）
  2. 规划前 stop servo、规划后 start servo，避免抢控制器
不依赖 visual_servo_node 的状态机，纯执行模块。
"""
import math
from typing import Optional

import numpy as np
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import Pose
from std_srvs.srv import Trigger
from tf_transformations import quaternion_from_euler

from pymoveit2 import MoveIt2


class MoveItPlanner:
    def __init__(
        self,
        node: Node,
        joint_names,
        base_link: str,
        end_effector: str,
        group_name: str,
        callback_group=None,
        max_vel: float = 1.0,
        max_acc: float = 1.0,
        move_group_namespace: str = "/move_group_fairino",
    ):
        self.node = node
        cb = callback_group or ReentrantCallbackGroup()

        self.moveit2 = MoveIt2(
            node=node,
            joint_names=joint_names,
            base_link_name=base_link,
            end_effector_name=end_effector,
            group_name=group_name,
            callback_group=cb,
            move_group_namespace=move_group_namespace,
        )
        self.moveit2.max_velocity = max_vel
        self.moveit2.max_acceleration = max_acc

        # servo 开关服务客户端
        self.start_servo_cli = node.create_client(
            Trigger, "/servo_node/start_servo", callback_group=cb)
        self.stop_servo_cli = node.create_client(
            Trigger, "/servo_node/stop_servo", callback_group=cb)

    # ---------------- servo 开关 ----------------
    def _call_servo_switch(self, client, name: str) -> bool:
        if not client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().warning(f"{name} service unavailable")
            return False
        future = client.call_async(Trigger.Request())

        # 同样换成轮询
        import time
        deadline = time.time() + 2.0
        while time.time() < deadline:
            if future.done():
                break
            time.sleep(0.01)

        ok = future.result() is not None and future.result().success
        self.node.get_logger().info(f"{name}: {'ok' if ok else 'failed'}")
        return ok

    def stop_servo(self) -> bool:
        return self._call_servo_switch(self.stop_servo_cli, "stop_servo")

    def start_servo(self) -> bool:
        return self._call_servo_switch(self.start_servo_cli, "start_servo")

    # ---------------- 粗对齐规划 ----------------
    def plan_to_pregrasp(
        self,
        position: np.ndarray,
        yaw: float = 0.0,
    ) -> bool:
        """规划执行到「指定位置 + 夹爪朝下 + 指定 yaw」的位姿。

        Args:
            position: base_link 系 (x,y,z)，已含 hover 偏移。
            yaw: 绕 base z 轴偏航（对齐 OBB），弧度。
        Returns:
            规划+执行是否成功。
        过程: stop servo → MoveIt 规划执行 → start servo
        """
        # 朝下姿态：roll=π 让夹爪 z 轴朝下，叠加 yaw
        quat = quaternion_from_euler(math.pi, 0.0, yaw)

        pose = Pose()
        pose.position.x = float(position[0])
        pose.position.y = float(position[1])
        pose.position.z = float(position[2])
        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]

        # 1. 让出控制权
        self.stop_servo()

        # 2. MoveIt 规划执行（全局规划，自动避奇异/碰撞）
        self.node.get_logger().info(
            f"planning to pregrasp ({pose.position.x:.3f}, "
            f"{pose.position.y:.3f}, {pose.position.z:.3f}), yaw={yaw:.2f}")
        self.moveit2.move_to_pose(
            position=[pose.position.x, pose.position.y, pose.position.z],
            quat_xyzw=[quat[0], quat[1], quat[2], quat[3]],
            cartesian=False,   # 关节空间规划，避奇异能力强
        )
        success = self.moveit2.wait_until_executed()

        # 3. 收回控制权给 servo（无论成功失败都要 start，
        #    否则后续伺服阶段拿不到控制权）
        self.start_servo()

        if not success:
            self.node.get_logger().error("pregrasp planning failed")
        return success