#!/usr/bin/env python3
"""冒烟测试：给 /robot_arm_controller 发一条小轨迹，确认 Gazebo 机械臂真的动。

用法（容器内，已 source 工作空间）：
    python3 scripts/env/smoke_test_motion.py            # 默认 j1 +0.2 rad
    python3 scripts/env/smoke_test_motion.py 1 0.3      # 指定关节序号与增量

退出码 0 = 运动成功且反馈跟随；非 0 = 失败。
"""
import sys
import time

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

JOINTS = ["j1", "j2", "j3", "j4", "j5", "j6"]


class Smoke(Node):
    def __init__(self):
        super().__init__("s622_motion_smoke_test")
        self.js = None
        self.create_subscription(JointState, "/joint_states", self._cb, 10)
        self.client = ActionClient(
            self, FollowJointTrajectory, "/robot_arm_controller/follow_joint_trajectory"
        )

    def _cb(self, msg):
        self.js = msg

    def wait_joint_states(self, timeout=15.0):
        t0 = time.time()
        while self.js is None and time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.2)
        return self.js

    def current(self):
        pos = dict(zip(self.js.name, self.js.position))
        return [pos[j] for j in JOINTS]


def main():
    idx = int(sys.argv[1]) if len(sys.argv) > 1 else 0
    delta = float(sys.argv[2]) if len(sys.argv) > 2 else 0.2

    rclpy.init()
    node = Smoke()
    try:
        if node.wait_joint_states() is None:
            print("FAIL: 15s 内没收到 /joint_states")
            return 1
        start = node.current()
        print("起始关节角:", [round(v, 4) for v in start])

        goal_pos = list(start)
        goal_pos[idx] += delta

        if not node.client.wait_for_server(timeout_sec=15.0):
            print("FAIL: 15s 内 action server 不可用")
            return 1

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINTS
        p0 = JointTrajectoryPoint()
        p0.positions = start
        p0.velocities = [0.0] * 6
        p0.time_from_start.sec = 0
        p1 = JointTrajectoryPoint()
        p1.positions = goal_pos
        p1.velocities = [0.0] * 6
        p1.time_from_start.sec = 3
        goal.trajectory.points = [p0, p1]

        send_future = node.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, send_future, timeout_sec=15.0)
        handle = send_future.result()
        if handle is None or not handle.accepted:
            print("FAIL: goal 被拒绝")
            return 1

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(node, result_future, timeout_sec=40.0)
        result = result_future.result()
        if result is None:
            print("FAIL: 40s 内没有收到 action 结果")
            return 1
        print("action 返回 error_code:", result.result.error_code)

        for _ in range(30):
            rclpy.spin_once(node, timeout_sec=0.1)
        end = node.current()
        print("结束关节角:", [round(v, 4) for v in end])

        moved = abs(end[idx] - start[idx])
        print(f"关节 J{idx + 1} 位移: {moved:.4f} rad (目标 {abs(delta):.4f})")
        if moved > abs(delta) * 0.5:
            print("PASS: 机械臂已跟随轨迹运动")
            return 0
        print("FAIL: 关节几乎没有动")
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
