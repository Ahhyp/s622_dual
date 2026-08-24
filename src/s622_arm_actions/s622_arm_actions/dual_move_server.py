"""Dual-arm 联合规划执行 server (方案 D).

规划: /move_group_fairino/plan_kinematic_path service (S4: namespaced + fairino 管线),
      group='dual_arm', 12 joint 约束
执行: 拆分 12-DOF 轨迹, 分别发 left/right_arm_controller 的 follow_joint_trajectory

用于 handover 等需要双臂时间同步的场景.
"""
import threading
import time
from typing import Dict, List, Optional

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.duration import Duration

from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetMotionPlan
from moveit_msgs.msg import (
    RobotState, Constraints, JointConstraint,
)
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from s622_bt_manager.action import DualMoveToJointState

from std_srvs.srv import Trigger
from s622_arm_actions.servo_lifecycle import ServoLifecycleManager



LEFT_JOINTS = [f'left_j{i}' for i in range(1, 7)]
RIGHT_JOINTS = [f'right_j{i}' for i in range(1, 7)]
ALL_JOINTS = LEFT_JOINTS + RIGHT_JOINTS


class DualMoveServer(Node):
    def __init__(self):
        super().__init__('dual_move_server')

        # ---- 参数 ----
        self.declare_parameter('group_name', 'dual_arm')
        self.declare_parameter('planner_id', 'RRTConnect')
        # 2026-08-25 S5 实测：fairino_planning_core NUM_JOINTS=6 硬编码，
        # FairinoPlannerManager 只支持单臂 6-DOF 组；dual_arm 12-DOF 联合规划
        # 必须用 ompl 管线（M2.8 已验证 RRTConnect 可行）。
        self.declare_parameter('pipeline_id', 'ompl')
        self.declare_parameter('move_group_namespace', '/move_group_fairino')
        self.declare_parameter('default_velocity_scale', 0.2)
        self.declare_parameter('default_acceleration_scale', 0.2)
        self.declare_parameter('allowed_planning_time', 8.0)
        self.declare_parameter('num_planning_attempts', 10)

        # named poses: 每个 name 12 个数, 顺序 [left_j1..6, right_j1..6]
        self.declare_parameter('named_pose_names', ['dual_home'])
        # 各 pose 的 joint values 从各自 parameter 读, 参照 move_to_pose_server 的做法
        self._group_name = self.get_parameter('group_name').value
        self._planner_id = self.get_parameter('planner_id').value
        self._pipeline_id = self.get_parameter('pipeline_id').value
        self._mg_ns = self.get_parameter('move_group_namespace').value
        self._default_v = self.get_parameter('default_velocity_scale').value
        self._default_a = self.get_parameter('default_acceleration_scale').value
        self._planning_time = self.get_parameter('allowed_planning_time').value
        self._planning_attempts = self.get_parameter('num_planning_attempts').value

        pose_names = list(self.get_parameter('named_pose_names').value)
        self._named_poses: Dict[str, List[float]] = {}
        for name in pose_names:
            param_name = f'{name}_joint_positions'
            try:
                self.declare_parameter(param_name, [0.0] * 12)
            except Exception:
                pass
            raw = self.get_parameter(param_name).value
            if len(raw) != 12:
                self.get_logger().error(
                    f'{param_name} must have 12 values (got {len(raw)}), skipping')
                continue
            self._named_poses[name] = [float(x) for x in raw]
        self.get_logger().info(
            f'loaded dual named poses: {list(self._named_poses.keys())}')

        # ---- 状态 ----
        self._latest_js: Optional[JointState] = None
        self._js_lock = threading.Lock()

        # ---- callback group + clients ----
        cb = ReentrantCallbackGroup()

        self._js_sub = self.create_subscription(
            JointState, '/joint_states', self._on_joint_state, 10,
            callback_group=cb)

        self._plan_cli = self.create_client(
            GetMotionPlan, f'{self._mg_ns}/plan_kinematic_path', callback_group=cb)
        self._left_ctrl_cli = ActionClient(
            self, FollowJointTrajectory, '/left_arm_controller/follow_joint_trajectory',
            callback_group=cb)
        self._right_ctrl_cli = ActionClient(
            self, FollowJointTrajectory, '/right_arm_controller/follow_joint_trajectory',
            callback_group=cb)
        self._left_servo = ServoLifecycleManager(
            self, callback_group=cb, servo_ns='left')
        self._right_servo = ServoLifecycleManager(
            self, callback_group=cb, servo_ns='right')

        # ---- action server ----
        self._action_server = ActionServer(
            self, DualMoveToJointState, 'move_to_joint_state',
            execute_callback=self._execute,
            goal_callback=lambda g: GoalResponse.ACCEPT,
            cancel_callback=lambda gh: CancelResponse.ACCEPT,
            callback_group=cb,
        )

        self.get_logger().info(
            f'dual_move_server ready: group={self._group_name}, '
            f'planner={self._planner_id}, pipeline={self._pipeline_id}, '
            f'mg_ns={self._mg_ns}')

    # ==================== callbacks ====================
    def _on_joint_state(self, msg: JointState):
        with self._js_lock:
            self._latest_js = msg

    def _get_current_12(self) -> Optional[List[float]]:
        with self._js_lock:
            msg = self._latest_js
        if msg is None:
            return None
        name_to_pos = dict(zip(msg.name, msg.position))
        try:
            return [name_to_pos[n] for n in ALL_JOINTS]
        except KeyError as e:
            self.get_logger().warn(f'joint not in /joint_states: {e}')
            return None

    # ==================== future waiters ====================
    @staticmethod
    def _wait_future_wall(future, timeout_s: float):
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        if not done.wait(timeout=timeout_s):
            future.cancel()
            return None
        return future.result()

    def _wait_future_sim(self, future, timeout_sim_s: float):
        start = self.get_clock().now()
        timeout = Duration(seconds=timeout_sim_s)
        while rclpy.ok():
            if future.done():
                return future.result()
            if self.get_clock().now() - start > timeout:
                future.cancel()
                return None
            time.sleep(0.02)
        return None

    # ==================== planning ====================
    def _plan_dual(self, target_12: List[float], current_12: List[float],
                    velocity_scale: float, acceleration_scale: float):
        if not self._plan_cli.service_is_ready():
            if not self._plan_cli.wait_for_service(timeout_sec=2.0):
                self.get_logger().error(f'{self._mg_ns}/plan_kinematic_path unavailable')
                return None

        req = GetMotionPlan.Request()
        mpr = req.motion_plan_request
        mpr.group_name = self._group_name
        mpr.planner_id = self._planner_id
        mpr.pipeline_id = self._pipeline_id
        mpr.num_planning_attempts = int(self._planning_attempts)
        mpr.allowed_planning_time = float(self._planning_time)
        mpr.max_velocity_scaling_factor = float(velocity_scale)
        mpr.max_acceleration_scaling_factor = float(acceleration_scale)

        rs = RobotState()
        rs.joint_state.name = list(ALL_JOINTS)
        rs.joint_state.position = list(current_12)
        rs.is_diff = False
        mpr.start_state = rs

        goal = Constraints()
        for name, pos in zip(ALL_JOINTS, target_12):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = float(pos)
            jc.tolerance_above = 0.001
            jc.tolerance_below = 0.001
            jc.weight = 1.0
            goal.joint_constraints.append(jc)
        mpr.goal_constraints.append(goal)

        future = self._plan_cli.call_async(req)
        res = self._wait_future_wall(future, timeout_s=self._planning_time + 3.0)
        if res is None:
            self.get_logger().error(f'{self._mg_ns}/plan_kinematic_path timeout')
            return None
        code = res.motion_plan_response.error_code.val
        if code != 1:
            self.get_logger().warn(
                f'dual planning failed: error_code={code}')
            return None
        return res.motion_plan_response.trajectory

    # ==================== trajectory splitting ====================
    @staticmethod
    def _split_trajectory(traj: JointTrajectory, joint_names: List[str]) -> JointTrajectory:
        """Extract a sub-trajectory containing only the named joints."""
        name_to_idx = {n: i for i, n in enumerate(traj.joint_names)}
        idxs = [name_to_idx[n] for n in joint_names]

        sub = JointTrajectory()
        sub.joint_names = list(joint_names)
        sub.header = traj.header
        for pt in traj.points:
            sub_pt = JointTrajectoryPoint()
            sub_pt.positions = [pt.positions[i] for i in idxs]
            if pt.velocities:
                sub_pt.velocities = [pt.velocities[i] for i in idxs]
            sub_pt.time_from_start = pt.time_from_start
            sub.points.append(sub_pt)
        return sub

    # ==================== execution (direct to controllers) ====================
    def _execute_dual(self, trajectory, timeout_sim_s: float) -> bool:
        # Split 12-DOF trajectory into left/right 6-DOF
        left_traj = self._split_trajectory(
            trajectory.joint_trajectory, LEFT_JOINTS)
        right_traj = self._split_trajectory(
            trajectory.joint_trajectory, RIGHT_JOINTS)

        # Wait for both controllers
        for cli, name in [(self._left_ctrl_cli, 'left'),
                           (self._right_ctrl_cli, 'right')]:
            if not cli.wait_for_server(timeout_sec=3.0):
                self.get_logger().error(f'{name}_arm_controller unavailable')
                return False

        # Send goals to both controllers in parallel
        results = {}

        def send_and_wait(cli, traj, label):
            goal = FollowJointTrajectory.Goal()
            goal.trajectory = traj
            goal.trajectory.header.stamp.sec = 0
            goal.trajectory.header.stamp.nanosec = 0

            gf = cli.send_goal_async(goal)
            gh = self._wait_future_wall(gf, timeout_s=3.0)
            if gh is None or not gh.accepted:
                self.get_logger().error(f'{label} send_goal failed')
                results[label] = False
                return
            rf = gh.get_result_async()
            wr = self._wait_future_sim(rf, timeout_sim_s=timeout_sim_s)
            if wr is None:
                self.get_logger().error(f'{label} execute timeout')
                gh.cancel_goal_async()
                results[label] = False
                return
            code = wr.result.error_code
            ok = (code == 0 or code == wr.result.SUCCESSFUL)
            if ok:
                results[label] = True
            else:
                self.get_logger().error(
                    f'{label} failed: error_code={code} '
                    f'error_string={wr.result.error_string}')
                results[label] = False

        t_left = threading.Thread(
            target=send_and_wait,
            args=(self._left_ctrl_cli, left_traj, 'left'))
        t_right = threading.Thread(
            target=send_and_wait,
            args=(self._right_ctrl_cli, right_traj, 'right'))
        t_left.start()
        t_right.start()
        t_left.join()
        t_right.join()

        return results.get('left', False) and results.get('right', False)

    # ==================== main execute ====================
    def _execute(self, goal_handle):
        req = goal_handle.request
        result = DualMoveToJointState.Result()

        # ---- 组装 target_12 ----
        target_12: Optional[List[float]] = None
        if len(req.left_positions) == 6 and len(req.right_positions) == 6:
            target_12 = list(req.left_positions) + list(req.right_positions)
            self.get_logger().info(f'dual_move: explicit 12 joint target')
        elif req.named_pose:
            if req.named_pose not in self._named_poses:
                msg = f'unknown named_pose: {req.named_pose}'
                self.get_logger().error(msg)
                goal_handle.abort()
                result.success = False
                result.error_code = -1
                result.error_msg = msg
                return result
            target_12 = self._named_poses[req.named_pose]
            self.get_logger().info(
                f'dual_move: named_pose={req.named_pose}')
        else:
            msg = 'neither left/right_positions nor named_pose provided'
            self.get_logger().error(msg)
            goal_handle.abort()
            result.success = False
            result.error_code = -2
            result.error_msg = msg
            return result

        # ---- 起点 ----
        # 等待 /joint_states (servo_node / arm_actions 可能有延迟)
        deadline = time.time() + 10.0
        while self._latest_js is None and time.time() < deadline:
            time.sleep(0.1)
        current_12 = self._get_current_12()
        if current_12 is None:
            msg = 'no /joint_states available yet'
            self.get_logger().error(msg)
            goal_handle.abort()
            result.success = False
            result.error_code = -3
            result.error_msg = msg
            return result

        v = float(req.velocity_scale) if req.velocity_scale > 0 else self._default_v
        a = float(req.acceleration_scale) if req.acceleration_scale > 0 else self._default_a

        # 无条件停两边 servo —— 轨迹执行和伺服不能共存
        self._left_servo.force_stop()
        self._right_servo.force_stop()

        # ---- plan ----
        t0 = time.time()
        traj = self._plan_dual(target_12, current_12, v, a)

        # ---- debug: verify left arm motion in trajectory ----
        # if traj is not None:
        #     jt = traj.joint_trajectory
        #     n_pts = len(jt.points)
        #     if n_pts > 0:
        #         left_idxs = [i for i, n in enumerate(jt.joint_names) if n.startswith('left')]
        #         first_left = [f'{jt.points[0].positions[i]:.4f}' for i in left_idxs]
        #         last_left = [f'{jt.points[-1].positions[i]:.4f}' for i in left_idxs]
        #         left_deltas = [abs(jt.points[-1].positions[i] - jt.points[0].positions[i]) for i in left_idxs]
        #         self.get_logger().info(
        #             f'dual_move debug: left joints in traj ({n_pts} pts): '
        #             f'first=[{", ".join(first_left)}] '
        #             f'last=[{", ".join(last_left)}] '
        #             f'deltas=[{", ".join(f"{d:.4f}" for d in left_deltas)}]')
        t_plan = time.time() - t0
        if traj is None:
            result.success = False
            result.error_code = -10
            result.error_msg = 'planning failed'
            goal_handle.abort()
            return result

        n_pts = len(traj.joint_trajectory.points)
        dur = traj.joint_trajectory.points[-1].time_from_start
        dur_s = dur.sec + dur.nanosec / 1e9
        self.get_logger().info(
            f'dual plan OK: {n_pts} pts, {dur_s:.2f}s traj, {t_plan*1000:.0f}ms plan')

        # ---- execute ----
        exec_timeout = max(float(req.timeout_sec) if req.timeout_sec > 0
                            else dur_s * 2 + 5.0, dur_s * 2 + 5.0)
        ok = self._execute_dual(traj, timeout_sim_s=exec_timeout)

        if ok:
            result.success = True
            result.error_code = 0
            result.error_msg = ''
            goal_handle.succeed()
        else:
            result.success = False
            result.error_code = -20
            result.error_msg = 'execution failed'
            goal_handle.abort()
        return result


def main():
    rclpy.init()
    node = DualMoveServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
