"""MoveIt 规划执行封装。
v3: IK 走 move_group 的 FairinoIKPlugin（内部 IKSelector 四维评分选解），
    /fairino/get_all_ik service 已退役。
"""
import threading
import time
from typing import List, Optional

from rclpy.node import Node
from rclpy.callback_groups import CallbackGroup, ReentrantCallbackGroup

from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetMotionPlan

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
        max_vel: float = 1.0,
        max_acc: float = 1.0,
        planner_id: str = "RRTConnect",
        pipeline_id: str = "fairino",
        arm_controller_action: str = "",
        move_group_namespace: str = "/move_group_fairino",
    ):
        self.node = node
        self.joint_names = joint_names
        self.base_link = base_link
        self.end_effector = end_effector
        self.group_name = group_name
        self.arm_controller_action = arm_controller_action
        # 2026-08-23 架构迁移：move_group 服务保持 namespaced（/move_group_fairino/*），
        # 客户端显式连接（对齐 robotarm）。默认连 fairino 实例。
        self.move_group_namespace = move_group_namespace
        cb = callback_group or ReentrantCallbackGroup()
        self._cb = cb

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

        # ----  plan-only service + controller action client ----
        # v3: 不再创建 IK client——IK 由 move_group 的 FairinoIKPlugin 内部完成
        #（解析法全解 + IKSelector 四维评分选解）
        self._plan_client = node.create_client(
            GetMotionPlan,
            f'{self.move_group_namespace}/plan_kinematic_path',
            callback_group=cb)
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
        """规划到位姿。

        v3: IK 由 move_group 的 FairinoIKPlugin 内部完成——
            解析法枚举全解 → IKSelector 四维评分（连续性 S1/可操作度 S2/姿态 S3/关节安全 S4）
            选最优解 → OMPL 规划 → 执行；规划失败 MoveIt 自动重试 IK。
            客户端不再自己调 IK service 和评分（/fairino/get_all_ik 已退役）。
        保留 max_candidates 参数仅为兼容调用方；robotarm 的"多条路径评分选优"
        （select_best_path，腕部运动量最小）留待 manipulation_common 并入时引入。
        """
        if planner_id is not None:
            self.set_planner(planner_id)
        return self.plan_to_pose(position, quat_xyzw, cartesian=cartesian)

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
