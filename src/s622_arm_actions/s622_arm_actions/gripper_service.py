#!/usr/bin/env python3
"""SetGripper Service：通过 JointTrajectoryController 控制夹爪开合。"""
import time

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from sensor_msgs.msg import JointState

from s622_bt_manager.srv import SetGripper

# C2（2026-08-24）：夹爪控制改用 manipulation_common.MoveItMotion.control_gripper
#（plan + execute，走 move_group → hand_controller），不再直接发 JointTrajectory。
# 接口（SetGripper.srv）不变，BT 层无感知。
from manipulation_common.planning.motion_executor import MoveItMotion
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
        self._settle = float(self.get_parameter('settle_sec').value)
        self._fb_joint = self.get_parameter('feedback_joint').value
        base_link = self.get_parameter('base_link').value
        end_effector = self.get_parameter('end_effector').value
        move_group_namespace = self.get_parameter('move_group_namespace').value
        arm_joint_names = list(self.get_parameter('arm_joint_names').value)
        arm_group_name = self.get_parameter('arm_group_name').value
        gripper_group_name = self.get_parameter('gripper_group_name').value
        gripper_controller_action = self.get_parameter('gripper_controller_action').value

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
        self.motion = MoveItMotion(
            self,
            arm_clients={'fairino': self.moveit2_arm},
            gripper=self.moveit2_gripper,
            open_positions=tuple(self._open),
            close_positions=tuple(self._close),
            action_delay=0.0,
        )

        self.srv = self.create_service(
            SetGripper, 'set_gripper', self._on_set_gripper, callback_group=cb)

        self.get_logger().info(
            f'gripper_service ready (MoveItMotion): open={self._open}, close={self._close}, '
            f'mg_ns={move_group_namespace}, arm_group={arm_group_name}, '
            f'gripper_group={gripper_group_name}')

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