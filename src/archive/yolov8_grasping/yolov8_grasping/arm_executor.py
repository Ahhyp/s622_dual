#!/usr/bin/env python3
"""MoveIt2 + 夹爪封装"""
import time
from typing import List, Sequence

from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import Pose
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from pymoveit2 import MoveIt2


class ArmExecutor:
    def __init__(self,
        node: Node,
        # 机械臂相关
        joint_names: List[str],
        base_link: str,
        end_effector: str,
        group_name: str,
        # 夹爪相关
        gripper_topic: str = "/hand_controller/joint_trajectory",
        gripper_joint_names: Sequence[str] = ("finger1_joint", "finger2_joint"),
        gripper_open: Sequence[float] = (0.025, -0.025),
        gripper_close: Sequence[float] = (0.0, 0.0),
        gripper_time_sec: float = 1.0,
        # 运动学限速
        max_vel: float = 0.3,
        max_acc: float = 0.3,
    ):
        self.node = node

        # 夹爪参数（list/tuple 都接受，统一存成 list 方便后续赋值）
        self.gripper_joint_names = list(gripper_joint_names)
        self.gripper_open_val = list(gripper_open)
        self.gripper_close_val = list(gripper_close)
        self.gripper_time_sec = float(gripper_time_sec)

        # ---- MoveIt2 接口 ----
        # ReentrantCallbackGroup 让 action 回调和主循环可以并发，
        # 否则 wait_until_executed() 会和回调互锁。
        cb_group = ReentrantCallbackGroup()
        self.moveit2 = MoveIt2(
            node=node,
            joint_names=joint_names,
            base_link_name=base_link,
            end_effector_name=end_effector,
            group_name=group_name,
            callback_group=cb_group,
        )
        self.moveit2.max_velocity = max_vel
        self.moveit2.max_acceleration = max_acc

        # ---- 夹爪 publisher ----
        # JointTrajectory 是控制器接口，不要换成 Float64。
        self.gripper_pub = node.create_publisher(
            JointTrajectory, gripper_topic, 10
        )

    # ------------------------------------------------------------------
    # 夹爪控制
    # ------------------------------------------------------------------
    def _send_gripper(self, positions: Sequence[float], wait: float):
        """构造一个单点 JointTrajectory 发出去，然后阻塞等到位。

        硬件没有反馈话题时只能用 sleep 简化处理；后面想做精确控制，
        把这里换成订阅 /joint_states 等到位即可。
        """
        msg = JointTrajectory()
        msg.joint_names = self.gripper_joint_names

        point = JointTrajectoryPoint()
        point.positions = list(positions)
        # time_from_start 不能为 0，否则部分控制器会拒收
        point.time_from_start.sec = int(self.gripper_time_sec)
        point.time_from_start.nanosec = int(
            (self.gripper_time_sec - int(self.gripper_time_sec)) * 1e9
        )
        msg.points.append(point)

        self.gripper_pub.publish(msg)
        time.sleep(wait)
    
    def open_gripper(self, wait: float = None):
        """张开夹爪。wait 默认等夹爪走完轨迹的时间。"""
        wait = self.gripper_time_sec if wait is None else wait
        self._send_gripper(self.gripper_open_val, wait)

    def close_gripper(self, wait: float = None):
        """闭合夹爪。wait 默认等夹爪走完轨迹的时间。"""
        wait = self.gripper_time_sec if wait is None else wait
        self._send_gripper(self.gripper_close_val, wait)
    
    # ------------------------------------------------------------------
    # 机械臂运动
    # ------------------------------------------------------------------
    def move_to_pose(self, pose: Pose, cartesian: bool = False) -> bool:
        """把末端运动到指定 Pose。

        Args:
            pose: 目标位姿，frame 由 MoveIt2 内部按 base_link 处理。
            cartesian: True 走笛卡尔直线（用于下降/抬升），
                       False 走关节空间规划（用于飞向 pregrasp）。

        Returns:
            执行是否成功。失败原因可能是规划失败或执行被打断。
        """
        position = [pose.position.x, pose.position.y, pose.position.z]
        quat = [
            pose.orientation.x, pose.orientation.y,
            pose.orientation.z, pose.orientation.w,
        ]

        self.moveit2.move_to_pose(
            position=position,
            quat_xyzw=quat,
            cartesian=cartesian,
        )
        return self.moveit2.wait_until_executed()