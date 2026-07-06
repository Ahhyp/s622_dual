#!/usr/bin/env python3
"""SetGripper Service：通过 JointTrajectoryController 控制夹爪开合。"""
import time

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

from s622_bt_manager.srv import SetGripper


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

        self._topic = self.get_parameter('gripper_topic').value
        self._joint_names = list(self.get_parameter('finger_joint_names').value)
        self._open = [float(x) for x in self.get_parameter('open_positions').value]
        self._close = [float(x) for x in self.get_parameter('close_positions').value]
        self._duration = float(self.get_parameter('command_duration_sec').value)
        self._settle = float(self.get_parameter('settle_sec').value)
        self._fb_joint = self.get_parameter('feedback_joint').value

        cb = ReentrantCallbackGroup()
        self.cmd_pub = self.create_publisher(JointTrajectory, self._topic, 10)
        self.js_sub = self.create_subscription(
            JointState, '/joint_states',
            self._on_joint_states, 10, callback_group=cb)

        self._latest_js = None

        self.srv = self.create_service(
            SetGripper, 'set_gripper', self._on_set_gripper, callback_group=cb)

        self.get_logger().info(
            f'gripper_service ready: topic={self._topic}, '
            f'open={self._open}, close={self._close}')

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

    def _send_traj(self, positions):
        msg = JointTrajectory()
        msg.joint_names = self._joint_names
        pt = JointTrajectoryPoint()
        pt.positions = list(positions)
        pt.time_from_start = Duration(sec=int(self._duration),
                                       nanosec=int((self._duration % 1) * 1e9))
        msg.points = [pt]
        self.cmd_pub.publish(msg)

    def _on_set_gripper(self, request, response):
        if request.command == 'open':
            target = self._open
        elif request.command == 'close':
            target = self._close
        else:
            response.success = False
            response.error_msg = f'unknown command: {request.command}'
            return response

        self.get_logger().info(f'set_gripper: {request.command} -> {target}')
        self._send_traj(target)
        time.sleep(self._settle)

        fb = self._read_finger_position()
        response.success = True
        response.finger_position = fb
        response.error_msg = ''
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