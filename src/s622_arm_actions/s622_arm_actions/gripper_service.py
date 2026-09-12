#!/usr/bin/env python3
"""SetGripper Service：通过 JointTrajectoryController 控制夹爪开合。"""
import time

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionClient

from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

from s622_bt_manager.srv import SetGripper

# C2（2026-08-24）：夹爪控制改用 manipulation_common.MoveItMotion.control_gripper
#（plan + execute，走 move_group → hand_controller），不再直接发 JointTrajectory。
# 接口（SetGripper.srv）不变，BT 层无感知。
from manipulation_common.planning.motion_executor import MoveItMotion
from manipulation_common.task.abort_manager import AbortManager
from pymoveit2 import MoveIt2


class GripperService(Node):
    def __init__(self):
        super().__init__('gripper_service')

        self.declare_parameter('gripper_topic',
                               '/hand_controller/joint_trajectory')
        self.declare_parameter('finger_joint_names',
                               ['finger1_joint', 'finger2_joint'])
        self.declare_parameter('open_positions', [0.025, -0.025])
        self.declare_parameter('close_positions', [0.0, 0.0])
        # [M4 §7.21] 仿真侧"机械限位(拧螺丝)"等效参数。
        #   >0 时 close 的目标改为 ±close_stop_gap/2 —— 指板停在该间隙、不再继续压向 0，
        #   与真机"螺丝决定最小闭合间隙、气压决定夹持力"的语义一致。
        #   默认 0.0 = 保持原行为（close → close_positions），
        #   因为真机 fairino_hardware 把 position 命令当开关量
        #   （opening > GRIPPER_OPEN_THRESHOLD(0.010) → 张开），
        #   真机路径绝不能收到"中间位置"的命令值。仿真启动时用 launch 参数显式打开。
        self.declare_parameter('close_stop_gap', 0.0)
        # [M4_2 / A''] 仿真专用：close 时【绕过 MoveIt】直发 FollowJointTrajectory 到 hand controller。
        #   背景：A'' 用"关节限位=螺丝 + 命令越界(0/0)"产生持续夹持力；而 MoveIt 会拒绝越界目标，
        #   故仿真侧 close 必须直发。默认 False = 真机原路径（MoveIt 计划→JTC→驱动开阀阈值）。
        self.declare_parameter('finger_direct_close', False)
        self.declare_parameter('command_duration_sec', 1.0)
        self.declare_parameter('settle_sec', 1.2)
        self.declare_parameter('feedback_joint', 'finger1_joint')
        self.declare_parameter('base_link', 'base_link')
        self.declare_parameter('end_effector', 'grasp_frame')
        # S2（2026-08-25）：双臂引入 MoveItMotion —— namespace / group / controller 参数化
        self.declare_parameter('move_group_namespace', '/move_group_fairino')
        self.declare_parameter('arm_joint_names', ['j1', 'j2', 'j3', 'j4', 'j5', 'j6'])
        self.declare_parameter('arm_group_name', 'robot_arm')
        self.declare_parameter('gripper_group_name', 'hand')
        self.declare_parameter('gripper_controller_action',
                               '/hand_controller/follow_joint_trajectory')

        self._topic = self.get_parameter('gripper_topic').value   # 保留（兼容 launch 参数）
        self._joint_names = list(self.get_parameter('finger_joint_names').value)
        self._open = [float(x) for x in self.get_parameter('open_positions').value]
        self._close = [float(x) for x in self.get_parameter('close_positions').value]
        # [M4 §7.21] 有效 close 目标：stop_gap>0 → ±gap/2（仿真限位）；否则原值（真机行为）
        self._stop_gap = float(self.get_parameter('close_stop_gap').value)
        self._close_eff = list(self._close)
        if self._stop_gap > 0.0:
            half = 0.5 * self._stop_gap
            self._close_eff = [(half if o >= 0.0 else -half) for o in self._open]
        self._settle = float(self.get_parameter('settle_sec').value)
        self._fb_joint = self.get_parameter('feedback_joint').value
        base_link = self.get_parameter('base_link').value
        end_effector = self.get_parameter('end_effector').value
        move_group_namespace = self.get_parameter('move_group_namespace').value
        arm_joint_names = list(self.get_parameter('arm_joint_names').value)
        arm_group_name = self.get_parameter('arm_group_name').value
        gripper_group_name = self.get_parameter('gripper_group_name').value
        gripper_controller_action = self.get_parameter('gripper_controller_action').value
        self._direct_close = bool(self.get_parameter('finger_direct_close').value)
        self._gripper_action = gripper_controller_action

        cb = ReentrantCallbackGroup()
        self.js_sub = self.create_subscription(
            JointState, '/joint_states',
            self._on_joint_states, 10, callback_group=cb)

        self._latest_js = None

        # C2：MoveItMotion + 夹爪客户端（hand group）
        # S2：namespace / group / controller 参数化（双臂按臂传入）
        self.moveit2_arm = MoveIt2(
            node=self,
            joint_names=arm_joint_names,
            base_link_name=base_link,
            end_effector_name=end_effector,
            group_name=arm_group_name,
            callback_group=cb,
            move_group_namespace=move_group_namespace,
        )
        self.moveit2_gripper = MoveIt2(
            node=self,
            joint_names=list(self._joint_names),
            base_link_name=base_link,
            end_effector_name=end_effector,
            group_name=gripper_group_name,
            callback_group=cb,
            move_group_namespace=move_group_namespace,
            follow_joint_trajectory_action_name=gripper_controller_action,
        )
        # 2026-08-25 回归修复：夹爪组（2 关节）必须用 ompl 管线。
        # fairino 管线（BiRRT* 等）是 6-DOF 机械臂算法，对 hand 组不可用
        # （start configuration in collision）。对齐 robotarm：gripper 客户端
        # pipeline_id='ompl'（fairino_pose_control_server.py:119 同款）。
        self.moveit2_gripper.pipeline_id = 'ompl'
        # 2026-08-25 回归修复：传入 AbortManager，否则 _wait 只 sleep 0.5s 提前返回
        self.abort = AbortManager(self, arm=self.moveit2_arm, gripper=self.moveit2_gripper)
        self.motion = MoveItMotion(
            self,
            arm_clients={'fairino': self.moveit2_arm},
            gripper=self.moveit2_gripper,
            abort=self.abort,
            open_positions=tuple(self._open),
            close_positions=tuple(self._close_eff),
            action_delay=0.0,
        )

        self.srv = self.create_service(
            SetGripper, 'set_gripper', self._on_set_gripper, callback_group=cb)

        self.get_logger().info(
            f'gripper_service ready (MoveItMotion): open={self._open}, close={self._close_eff}, '
            f'mg_ns={move_group_namespace}, arm_group={arm_group_name}, '
            f'gripper_group={gripper_group_name}')
        if self._stop_gap > 0.0:
            self.get_logger().warn(
                f'[M4] close_stop_gap={self._stop_gap:.4f}m 生效: close 目标 = {self._close_eff} '
                f'(仿真限位, 等效真机螺丝; 真机请勿设置该参数)')
        if self._direct_close:
            self.get_logger().warn(
                f'[M4/A\'\'] finger_direct_close=True 生效: close 将绕过 MoveIt, 直发 '
                f'{self._gripper_action} 目标={self._close} '
                f'(仿真专用: 配合关节限位产生持续夹持力; 真机请勿设置)')

    def _direct_gripper_traj(self, targets) -> bool:
        """[M4_2 / A''] 绕过 MoveIt, 直发单点轨迹给 hand controller。
        越界目标（如限位下的 0/0）MoveIt 会拒绝, 而 A'' 正需要"命令越界 + 机械限位挡位"来产生
        持续夹持力。回调内不 spin（节点由 MultiThreadedExecutor 驱动），沿用项目既有轮询等待写法。"""
        ac = ActionClient(self, FollowJointTrajectory, self._gripper_action)
        if not ac.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(f'[M4] 直发失败: action {self._gripper_action} 不可用')
            return False
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(self._joint_names)
        pt = JointTrajectoryPoint()
        pt.positions = [float(v) for v in targets]
        pt.velocities = [0.0] * len(targets)
        pt.time_from_start = Duration(sec=2, nanosec=0)
        goal.trajectory.points = [pt]
        send_fut = ac.send_goal_async(goal)
        t0 = time.monotonic()
        while not send_fut.done() and time.monotonic() - t0 < 10.0:
            time.sleep(0.02)
        handle = send_fut.result() if send_fut.done() else None
        if handle is None or not handle.accepted:
            self.get_logger().error('[M4] 直发轨迹被拒绝')
            return False
        res_fut = handle.get_result_async()
        t0 = time.monotonic()
        while not res_fut.done() and time.monotonic() - t0 < 30.0:
            time.sleep(0.02)
        if not res_fut.done():
            self.get_logger().warn('[M4] 直发轨迹等待结果超时（可能因限位挡住未报 SUCCEEDED）')
            return False
        code = res_fut.result().result.error_code
        # 被机械限位挡住时 JTC 可能返回 GOAL_TOLERANCE_VIOLATED(-5)/PATH_TOLERANCE(-4) 之类:
        # 对 A'' 而言"被挡住"= 正是我们要的持续夹持力, 故这些码不算失败。
        ok = code in (0, -4, -5)
        self.get_logger().info(f'[M4] 直发轨迹 error_code={code} -> {"ok" if ok else "FAILED"}')
        return ok

    def _on_joint_states(self, msg: JointState):
        self._latest_js = msg

    def _read_finger_position(self) -> float:
        if self._latest_js is None:
            return float('nan')
        try:
            idx = list(self._latest_js.name).index(self._fb_joint)
            return float(self._latest_js.position[idx])
        except ValueError:
            return float('nan')

    def _on_set_gripper(self, request, response):
        if request.command == 'open':
            ok = self.motion.control_gripper(
                open_gripper=True, action_name='SetGripper open')
        elif request.command == 'close':
            if self._direct_close:
                # [M4_2 / A''] 仿真：绕过 MoveIt 直发（目标=close_positions=[0,0]，被关节限位挡住）
                ok = self._direct_gripper_traj(self._close)
            else:
                ok = self.motion.control_gripper(
                    open_gripper=False, action_name='SetGripper close')
        else:
            response.success = False
            response.error_msg = f'unknown command: {request.command}'
            return response

        self.get_logger().info(f'set_gripper: {request.command} -> {"ok" if ok else "FAILED"}')
        time.sleep(self._settle)

        fb = self._read_finger_position()
        response.success = ok
        response.finger_position = fb
        response.error_msg = '' if ok else 'control_gripper failed'
        self.get_logger().info(f'set_gripper done: finger={fb:.4f}')
        return response


def main():
    rclpy.init()
    node = GripperService()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()