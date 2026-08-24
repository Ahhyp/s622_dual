"""MoveIt 规划执行封装（不再管 servo，servo 由 ServoLifecycleManager 管）。 保留一个旧的"""
import random
import time
from typing import List, Optional

import numpy as np
from rclpy.node import Node
from rclpy.callback_groups import CallbackGroup, ReentrantCallbackGroup
from rclpy.duration import Duration

from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import RobotState

from pymoveit2 import MoveIt2



class MoveItPlanner:
    def __init__(
        self,
        node: Node,
        joint_names: List[str],
        base_link: str,
        end_effector: str,
        group_name: str,
        callback_group: Optional[CallbackGroup] = None,
        max_vel: float = 0.2,
        max_acc: float = 0.2,
    ):
        self.node = node
        self.joint_names = joint_names
        self.base_link = base_link
        self.end_effector = end_effector
        self.group_name = group_name
        cb = callback_group or ReentrantCallbackGroup()
        self._cb = cb

        self.moveit2 = MoveIt2(
            node=node,
            joint_names=joint_names,
            base_link_name=base_link,
            end_effector_name=end_effector,
            group_name=group_name,
            callback_group=cb,
        )
        self.moveit2.max_velocity = max_vel
        self.moveit2.max_acceleration = max_acc
        self.moveit2.num_planning_attempts = 10
        self.moveit2.allowed_planning_time = 5.0

        # ---- smart IK 相关 ----
        # 关节 safety limits (比 URDF 收紧,避开 servo halt 区)
        # S622: j4 官方 ±1.47, servo halt 在 ±1.37, 这里再留 0.07
        self.joint_safety_limits = [
            (-3.05, 3.05),   # j1
            (-1.55, 1.55),   # j2
            (-1.55, 1.55),   # j3
            (-1.30, 1.30),   # j4 ← 关键
            (-1.55, 1.55),   # j5
            (-3.05, 3.05),   # j6
        ]
        self._latest_joint_state: Optional[JointState] = None
        self._joint_sub = node.create_subscription(
            JointState, '/joint_states', self._on_joint_state, 10,
            callback_group=cb)
        # self._ik_client = node.create_client(
        #     GetPositionIK, '/compute_ik', callback_group=cb)
        self._all_ik_client = node.create_client(
            GetAllIK, '/fairino/get_all_ik',
        callback_group=cb)

    # ============ joint state cache ============
    def _on_joint_state(self, msg: JointState):
        self._latest_joint_state = msg

    def get_current_joint_positions(self) -> Optional[List[float]]:
        if self._latest_joint_state is None:
            return None
        name_to_pos = dict(zip(self._latest_joint_state.name,
                               self._latest_joint_state.position))
        try:
            return [name_to_pos[n] for n in self.joint_names]
        except KeyError as e:
            self.node.get_logger().warn(f'joint not in /joint_states: {e}')
            return None


    def _score_ik(self, joints: List[float],
                  current: List[float]) -> float:
        """Lower is better. inf = invalid."""
        limit_penalty = 0.0
        for j, (lo, hi) in zip(joints, self.joint_safety_limits):
            if j < lo or j > hi:
                return float('inf')
            margin = min(j - lo, hi - j)
            if margin < 0.25:
                limit_penalty += (0.25 - margin) ** 2 * 100.0
        motion_cost = sum(abs(j - c) for j, c in zip(joints, current))
        return limit_penalty + motion_cost

    # ============ public planning APIs ============
    def plan_to_pose(self, position, quat_xyzw,
                     cartesian: bool = False) -> bool:
        """裸 pymoveit2 路径,保留作 fallback。"""
        self.node.get_logger().info(
            f'plan_to_pose: pos=({position[0]:.3f},{position[1]:.3f},{position[2]:.3f}) '
            f'quat=({quat_xyzw[0]:.3f},{quat_xyzw[1]:.3f},{quat_xyzw[2]:.3f},{quat_xyzw[3]:.3f})')
        self.moveit2.move_to_pose(
            position=list(position), quat_xyzw=list(quat_xyzw),
            cartesian=cartesian)
        return self.moveit2.wait_until_executed()

    def plan_to_pose_smart(self, position, quat_xyzw,
                           planner_id: str = "AITstar") -> bool:
        current = self.get_current_joint_positions()
        if current is None:
            return self.plan_to_pose(position, quat_xyzw)

        # 1. 一次拿到所有解析解
        solutions = self._call_all_ik(position, quat_xyzw,
                                       seed=current,
                                       check_collision=False)
        if not solutions:
            self.node.get_logger().error('no analytical IK')
            return self.plan_to_pose(position, quat_xyzw)
        
        # 2. 评分排序（软惩罚版）
        scored = []
        for sol in solutions:
            score = self._score_ik_soft(sol, current)
            scored.append((score, sol))
        scored.sort(key=lambda x: x[0])
        
        self.node.get_logger().info(
            f'got {len(scored)} IK solutions, scores: '
            f'{[f"{s:.2f}" for s, _ in scored[:5]]}')

        # 3. 切到 AITstar，按顺序尝试
        self.moveit2.planner_id = planner_id
        for score, joints in scored:
            self.node.get_logger().info(
                f'try IK score={score:.3f} j={[f"{v:+.2f}" for v in joints]}')
            if self.plan_to_joint_positions(joints):
                return True
        
        self.node.get_logger().warn('all IK candidates failed planning')
        return False

    def _call_all_ik(self, position, quat_xyzw, seed, check_collision=False,
                     timeout_s: float = 1.0):
        if not self._all_ik_client.service_is_ready():
            if not self._all_ik_client.wait_for_service(timeout_sec=2.0):
                return []

        req = GetAllIK.Request()
        req.pose.header.frame_id = self.base_link
        req.pose.pose.position.x = float(position[0])
        req.pose.pose.position.y = float(position[1])
        req.pose.pose.position.z = float(position[2])
        req.pose.pose.orientation.x = float(quat_xyzw[0])
        req.pose.pose.orientation.y = float(quat_xyzw[1])
        req.pose.pose.orientation.z = float(quat_xyzw[2])
        req.pose.pose.orientation.w = float(quat_xyzw[3])
        req.group_name = self.group_name
        req.seed_state = list(seed)
        req.check_collision = check_collision

        future = self._all_ik_client.call_async(req)
        # 用 Event 等，不卡死 executor
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        if not done.wait(timeout=timeout_s):
            future.cancel()
            return []
        
        res = future.result()
        if res is None or res.error_code != 0:
            return []
        
        return [list(js.position) for js in res.solutions]

    def _score_ik_soft(self, joints, current) -> float:
        """软惩罚：贴近 safety 限位扣分但不拒绝。"""
        limit_penalty = 0.0
        for j, (lo, hi) in zip(joints, self.joint_safety_limits):
            margin = min(j - lo, hi - j)
            if margin < 0:
                limit_penalty += 1000.0 * margin * margin
            elif margin < 0.25:
                limit_penalty += 100.0 * (0.25 - margin) ** 2
        
        # 关节空间最短路径距离（考虑 ±π 缠绕）
        motion_cost = sum(
            min(abs(j - c), 2 * 3.14159 - abs(j - c))
            for j, c in zip(joints, current)
        )
        
        # 可选：可操作性（如果 service 返回了 manipulability 字段）
        # manip_penalty = ...
        
        return limit_penalty + motion_cost


    def plan_to_joint_positions(self, positions: List[float]) -> bool:
        if len(positions) != len(self.joint_names):
            self.node.get_logger().error(
                f'joint length mismatch: {len(positions)} vs {len(self.joint_names)}')
            return False
        self.node.get_logger().info(f'plan_to_joint_positions: {positions}')
        self.moveit2.move_to_configuration(positions)
        return self.moveit2.wait_until_executed()
