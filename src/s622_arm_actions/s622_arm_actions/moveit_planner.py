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
from moveit_msgs.srv import GetPositionIK, GetMotionPlan

from fairino_msgs.srv import GetAllIK
from moveit_msgs.msg import (
    RobotState, MotionPlanRequest, Constraints, JointConstraint,
    RobotTrajectory,
)

from pymoveit2 import MoveIt2
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory

from control_msgs.msg import JointTolerance
from builtin_interfaces.msg import Duration as DurationMsg


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
        arm_controller_action: str = "",
    ):
        self.node = node
        self.joint_names = joint_names
        self.base_link = base_link
        self.end_effector = end_effector
        self.group_name = group_name
        self.arm_controller_action = arm_controller_action
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

        # ----  plan-only service + controller action client ----
        self._plan_client = node.create_client(GetMotionPlan, '/plan_kinematic_path', callback_group=cb)
        if arm_controller_action:
            self._exec_client = ActionClient(
                node, FollowJointTrajectory, arm_controller_action,
                callback_group=cb)
        else:
            self._exec_client = None
            node.get_logger().warn(
                'arm_controller_action not set; will fallback to pymoveit2 execute')
        
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

        # 无 controller action: fallback 到 pymoveit2 (单臂兼容路径)
        if self._exec_client is None:
            self.moveit2.move_to_configuration(positions)
            return self.moveit2.wait_until_executed()

        current = self.get_current_joint_positions()
        if current is None:
            self.node.get_logger().warn('no joint state; cannot plan')
            return False

        t0 = time.time()
        traj = self._plan_joint_target(positions, current)
        t_plan = time.time() - t0
        if traj is None:
            return False
        n_pts = len(traj.joint_trajectory.points)
        dur = traj.joint_trajectory.points[-1].time_from_start
        dur_s = dur.sec + dur.nanosec / 1e9
        self.node.get_logger().info(
            f'plan OK: {n_pts} pts, {dur_s:.2f}s traj, {t_plan*1000:.0f}ms plan')

        return self._execute_via_controller(traj, timeout_s=dur_s * 2 + 5.0)
    
    # ============ 方案 A: plan-only + controller-execute ============
    def _plan_joint_target(self, positions: List[float],
                            current: List[float],
                            timeout_s: float = 8.0
                           ) -> Optional[RobotTrajectory]:
        """走 /plan_kinematic_path service, 只规划返回 trajectory."""
        if not self._plan_client.service_is_ready():
            if not self._plan_client.wait_for_service(timeout_sec=2.0):
                self.node.get_logger().error(
                    '/plan_kinematic_path not available')
                return None

        req = GetMotionPlan.Request()
        mpr = req.motion_plan_request
        mpr.group_name = self.group_name
        mpr.planner_id = self.moveit2.planner_id
        mpr.pipeline_id = self.moveit2.pipeline_id
        mpr.num_planning_attempts = 10
        mpr.allowed_planning_time = 5.0
        mpr.max_velocity_scaling_factor = float(self.moveit2.max_velocity)
        mpr.max_acceleration_scaling_factor = float(self.moveit2.max_acceleration)

        rs = RobotState()
        rs.joint_state.name = list(self.joint_names)
        rs.joint_state.position = list(current)
        rs.is_diff = False   # 明确不是增量, 免得 planning_scene 状态污染
        mpr.start_state = rs

        goal = Constraints()
        for name, pos in zip(self.joint_names, positions):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = float(pos)
            jc.tolerance_above = 0.001
            jc.tolerance_below = 0.001
            jc.weight = 1.0
            goal.joint_constraints.append(jc)
        mpr.goal_constraints.append(goal)

        future = self._plan_client.call_async(req)
        res = self._wait_future(future, timeout_s)
        if res is None:
            self.node.get_logger().error('plan_kinematic_path timeout')
            return None
        code = res.motion_plan_response.error_code.val
        if code != 1:   # SUCCESS = 1
            self.node.get_logger().warn(
                f'plan_kinematic_path failed: error_code={code}')
            return None
        return res.motion_plan_response.trajectory

    def _execute_via_controller(self, trajectory: RobotTrajectory,
                             timeout_s: float = 30.0) -> bool:
        if self._exec_client is None:
            self.node.get_logger().error(
                'exec_client not initialized (arm_controller_action empty)')
            return False
        if not self._exec_client.wait_for_server(timeout_sec=3.0):
            self.node.get_logger().error(
                f'controller action unavailable: {self.arm_controller_action}')
            return False

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory.joint_trajectory
        goal.trajectory.header.stamp.sec = 0
        goal.trajectory.header.stamp.nanosec = 0

        for name in self.joint_names:
            tol = JointTolerance()
            tol.name = name
            tol.position = 0.05
            tol.velocity = 0.1
            goal.goal_tolerance.append(tol)
        goal.goal_time_tolerance = DurationMsg(sec=5, nanosec=0)   # 从 2 调到 5, 给 sim 收尾余量

        goal_future = self._exec_client.send_goal_async(goal)
        goal_handle = self._wait_future(goal_future, timeout_s=3.0)   # 握手用 wall time
        if goal_handle is None:
            self.node.get_logger().error('send_goal timeout')
            return False
        if not goal_handle.accepted:
            self.node.get_logger().error('goal rejected by controller')
            return False

        result_future = goal_handle.get_result_async()
        # 关键: 用 sim time 等 result
        wrapped = self._wait_future_sim(result_future, timeout_sim_s=timeout_s)
        if wrapped is None:
            self.node.get_logger().error(
                f'execute timeout ({timeout_s:.1f}s sim time). '
                f'arm may still be moving; check sim RTF.')
            goal_handle.cancel_goal_async()
            return False

        err = wrapped.result.error_code
        if err != 0:
            self.node.get_logger().error(
                f'controller execution failed: error_code={err}, '
                f'msg="{wrapped.result.error_string}"')
            return False
        return True


    def _wait_future_sim(self, future, timeout_sim_s: float):
        """按 sim time 轮询 future 完成. 避免 wall-time 抢跑."""
        import rclpy.duration
        start = self.node.get_clock().now()
        timeout = rclpy.duration.Duration(seconds=timeout_sim_s)
        while rclpy.ok():
            if future.done():
                return future.result()
            if self.node.get_clock().now() - start > timeout:
                future.cancel()
                return None
            time.sleep(0.02)   # wall-time poll interval, 不影响 sim
        return None
