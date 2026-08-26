#!/usr/bin/env python3
"""NaN 取证 v2：区分 position/velocity 字段的 NaN，并记录每帧完整数据。

用法：
  ros2 run s622_arm_actions nan_diagnose --ros-args \
      -p duration_sec:=240.0 -p log_all:=false

输出：pos_nan / vel_nan 分别统计；NaN 帧记录该关节的 pos/vel 值与时间戳。
"""
import rclpy
import time
from rclpy.node import Node
from sensor_msgs.msg import JointState


class NanDiagnose(Node):
    def __init__(self):
        super().__init__('nan_diagnose')
        self.declare_parameter('duration_sec', 120.0)
        self.declare_parameter('log_all', False)
        self.duration = float(self.get_parameter('duration_sec').value)
        self.log_all = bool(self.get_parameter('log_all').value)

        self.pos_nan = {}   # joint -> count
        self.vel_nan = {}   # joint -> count
        self.pos_nan_frames = []
        self.vel_nan_frames = []

        self.sub = self.create_subscription(
            JointState, '/joint_states', self._on_js, 100)
        self.get_logger().info(
            f'nan_diagnose v2 ready: {self.duration:.0f}s (log_all={self.log_all})')

    def _on_js(self, msg):
        t = self.get_clock().now().to_msg()
        for name, pos, vel in zip(msg.name, msg.position, msg.velocity):
            if pos is not None and not _finite(pos):
                self.pos_nan[name] = self.pos_nan.get(name, 0) + 1
                self.pos_nan_frames.append((t, name, pos))
                if self.log_all:
                    self.get_logger().error(
                        f'POS-NaN t={t.sec}.{t.nanosec:09d} {name} pos={pos}')
            if vel is not None and not _finite(vel):
                self.vel_nan[name] = self.vel_nan.get(name, 0) + 1
                self.vel_nan_frames.append((t, name, vel))
                if self.log_all:
                    self.get_logger().error(
                        f'VEL-NaN t={t.sec}.{t.nanosec:09d} {name} vel={vel}')

    def report(self):
        self.get_logger().info('=== NaN summary v2 ===')
        if not self.pos_nan and not self.vel_nan:
            self.get_logger().info('No NaN in /joint_states.')
            return
        self.get_logger().info(f'POS-NaN joints: {self.pos_nan}')
        self.get_logger().info(f'VEL-NaN joints: {self.vel_nan}')
        if self.pos_nan_frames:
            self.get_logger().info(
                f'POS-NaN first={self.pos_nan_frames[0]} last={self.pos_nan_frames[-1]}')
        if self.vel_nan_frames:
            self.get_logger().info(
                f'VEL-NaN first={self.vel_nan_frames[0]} last={self.vel_nan_frames[-1]}')


def _finite(v):
    try:
        return v == v and abs(v) < 1e300
    except Exception:
        return False


def main():
    rclpy.init()
    node = NanDiagnose()
    try:
        end = time.monotonic() + node.duration
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.2)
        node.report()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
