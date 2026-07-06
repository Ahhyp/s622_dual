"""MoveIt 规划执行封装。
v2: 接入 fairino_ik 多解 service,删除随机 KDL 循环。
"""
import math
import threading
import time
from typing import List, Optional

from rclpy.node import Node
from rclpy.callback_groups import CallbackGroup, ReentrantCallbackGroup
from rclpy.duration import Duration

from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import RobotState

from fairino_msgs.srv import GetAllIK

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
        planner_id: str = "RRTConnect",
        pipeline_id: str = "fairino",
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
        self.moveit2.pipeline_id = pipeline_id
        self.moveit2.planner_id = planner_id

        # ---- 关节安全限位(比 URDF 收紧,避开 servo halt 区) ----
        self.joint_safety_limits = [
            (-3.05, 3.05),   # j1
            (-4.62, 1.48),   # j2
            (-2.82, 2.82),   # j3
            (-4.62, 1.48),   # j4
            (-3.05, 3.05),   # j5
            (-3.05, 3.05),   # j6
        ]

        # ---- joint state cache ----
        self._latest_joint_state: Optional[JointState] = None
        self._js_lock = threading.Lock()
        self._joint_sub = node.create_subscription(
            JointState, '/joint_states', self._on_joint_state, 10,
            callback_group=cb)

        # ---- IK service clients ----
        self._all_ik_client = node.create_client(
            GetAllIK, '/fairino/get_all_ik', callback_group=cb)
        # 保留 /compute_ik 作为 fallback
        self._ik_client = node.create_client(
            GetPositionIK, '/compute_ik', callback_group=cb)

    # ============ joint state cache ============
    def _on_joint_state(self, msg: JointState):
        with self._js_lock:
            self._latest_joint_state = msg

    def get_current_joint_positions(self) -> Optional[List[float]]:
        with self._js_lock:
            msg = self._latest_joint_state
        if msg is None:
            return None
        name_to_pos = dict(zip(msg.name, msg.position))
        try:
            return [name_to_pos[n] for n in self.joint_names]
        except KeyError as e:
            self.node.get_logger().warn(f'joint not in /joint_states: {e}')
            return None

    # ============ 通用 future 等待 ============
    @staticmethod
    def _wait_future(future, timeout_s: float):
        """用 Event 等 future,不卡 executor。"""
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        if not done.wait(timeout=timeout_s):
            future.cancel()
            return None
        return future.result()

    # ============ 多解 IK ============
    def _call_all_ik(self, position, quat_xyzw,
                     timeout_s: float = 1.0) -> List[List[float]]:
        """调 fairino_ik service 拿全部解析解。"""
        if not self._all_ik_client.service_is_ready():
            if not self._all_ik_client.wait_for_service(timeout_sec=2.0):
                self.node.get_logger().error('/fairino/get_all_ik not available')
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

        future = self._all_ik_client.call_async(req)
        res = self._wait_future(future, timeout_s)
        if res is None:
            self.node.get_logger().error('GetAllIK timeout')
            return []
        if res.error_code != 0:
            self.node.get_logger().warn(
                f'GetAllIK failed: {res.error_message}')
            return []
        return [list(js.position) for js in res.solutions]

    # ============ KDL IK fallback (备用) ============
    def _compute_ik_kdl(self, position, quat_xyzw, seed_joints,
                       timeout_s: float = 0.05) -> Optional[List[float]]:
        """MoveIt 默认 IK,作为 fairino_ik 不可用时的 fallback。"""
        if not self._ik_client.service_is_ready():
            if not self._ik_client.wait_for_service(timeout_sec=1.0):
                return None

        req = GetPositionIK.Request()
        req.ik_request.group_name = self.group_name
        req.ik_request.ik_link_name = self.end_effector
        req.ik_request.pose_stamped.header.frame_id = self.base_link
        req.ik_request.pose_stamped.pose.position.x = float(position[0])
        req.ik_request.pose_stamped.pose.position.y = float(position[1])
        req.ik_request.pose_stamped.pose.position.z = float(position[2])
        req.ik_request.pose_stamped.pose.orientation.x = float(quat_xyzw[0])
        req.ik_request.pose_stamped.pose.orientation.y = float(quat_xyzw[1])
        req.ik_request.pose_stamped.pose.orientation.z = float(quat_xyzw[2])
        req.ik_request.pose_stamped.pose.orientation.w = float(quat_xyzw[3])
        req.ik_request.timeout = Duration(seconds=timeout_s).to_msg()
        req.ik_request.avoid_collisions = True

        rs = RobotState()
        rs.joint_state.name = list(self.joint_names)
        rs.joint_state.position = list(seed_joints)
        req.ik_request.robot_state = rs

        future = self._ik_client.call_async(req)
        res = self._wait_future(future, timeout_s + 0.2)
        if res is None or res.error_code.val != 1:
            return None
        name_to_pos = dict(zip(res.solution.joint_state.name,
                               res.solution.joint_state.position))
        try:
            return [name_to_pos[n] for n in self.joint_names]
        except KeyError:
            return None

    # ============ 评分(软惩罚版) ============
    def _score_ik(self, joints: List[float],
                  current: List[float]) -> float:
        """Lower is better. 软惩罚版,不直接拒绝越限解。"""
        limit_penalty = 0.0
        for j, (lo, hi) in zip(joints, self.joint_safety_limits):
            margin = min(j - lo, hi - j)
            if margin < 0:
                # 越过 safety 区:重惩罚但不拒绝
                limit_penalty += 1000.0 * margin * margin
            elif margin < 0.25:
                # 接近 safety 边界:中等惩罚
                limit_penalty += 100.0 * (0.25 - margin) ** 2

        # 关节最短路径距离(考虑 ±π 缠绕)
        motion_cost = sum(
            min(abs(j - c), 2 * math.pi - abs(j - c))
            for j, c in zip(joints, current)
        )

        return limit_penalty + motion_cost

    # ============ public planning APIs ============
    def set_speed(self, vel: float, acc: float):
        self.moveit2.max_velocity = float(vel)
        self.moveit2.max_acceleration = float(acc)

    def set_planner(self, planner_id: str):
        """运行时切 planner,例如 'RRTConnect'/'LBTRRT'/'RRTstar'。"""
        self.moveit2.planner_id = planner_id

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
                           planner_id: Optional[str] = None,
                           max_candidates: int = 8, cartesian: bool = False) -> bool:
        """多解 IK + 评分选解 + 失败重试。"""
        # 笛卡尔模式:绕开多解逻辑,直接走原 API
        if cartesian:
            return self.plan_to_pose(position, quat_xyzw, cartesian=True)
        
        current = self.get_current_joint_positions()
        if current is None:
            self.node.get_logger().warn(
                'smart: no joint state, fallback to plan_to_pose')
            return self.plan_to_pose(position, quat_xyzw)

        # ---- 1. 一次拿到所有解析解 ----
        t0 = time.time()
        solutions = self._call_all_ik(position, quat_xyzw)
        t_ik = time.time() - t0

        if not solutions:
            self.node.get_logger().error(
                f'smart: no analytical IK ({t_ik*1000:.1f}ms), '
                f'fallback to plan_to_pose')
            return self.plan_to_pose(position, quat_xyzw)

        # ---- 2. 评分排序 ----
        scored = sorted(
            ((self._score_ik(s, current), s) for s in solutions),
            key=lambda x: x[0]
        )
        scored = scored[:max_candidates]

        self.node.get_logger().info(
            f'smart: got {len(solutions)} IK solutions in {t_ik*1000:.1f}ms, '
            f'top scores: {[f"{s:.2f}" for s, _ in scored[:3]]}'
        )

        # ---- 3. 切 planner,按顺序尝试规划 ----
        if planner_id is not None:
            self.set_planner(planner_id)

        for i, (score, joints) in enumerate(scored):
            self.node.get_logger().info(
                f'smart: try IK #{i}/{len(scored)} score={score:.3f} '
                f'j=[{",".join(f"{v:+.2f}" for v in joints)}]'
            )
            if self.plan_to_joint_positions(joints):
                self.node.get_logger().info(
                    f'smart: SUCCESS with IK #{i}')
                return True

        self.node.get_logger().error(
            f'smart: all {len(scored)} IK candidates failed planning')
        return False

    def plan_to_joint_positions(self, positions: List[float]) -> bool:
        if len(positions) != len(self.joint_names):
            self.node.get_logger().error(
                f'joint length mismatch: {len(positions)} vs {len(self.joint_names)}')
            return False
        self.moveit2.move_to_configuration(positions)
        return self.moveit2.wait_until_executed()
