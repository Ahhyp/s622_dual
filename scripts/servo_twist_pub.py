#!/usr/bin/env python3
"""命令行手动测 servo 用的 twist 发布器. 关键在填 header.stamp."""
import sys
import argparse
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped


class TwistPub(Node):
    def __init__(self, topic, frame_id, lin, ang, rate_hz):
        super().__init__('servo_twist_pub')
        self.pub = self.create_publisher(TwistStamped, topic, 10)
        self.frame_id = frame_id
        self.lin = lin
        self.ang = ang
        self.create_timer(1.0 / rate_hz, self._tick)

    def _tick(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = self.lin
        msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z = self.ang
        self.pub.publish(msg)


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--topic', required=True)
    p.add_argument('--frame', required=True)
    p.add_argument('--lin', nargs=3, type=float, default=[0.0, 0.0, 0.0], metavar=('X', 'Y', 'Z'))
    p.add_argument('--ang', nargs=3, type=float, default=[0.0, 0.0, 0.0], metavar=('X', 'Y', 'Z'))
    p.add_argument('--rate', type=float, default=30.0)
    p.add_argument('--use-sim-time', action='store_true', default=True)
    args = p.parse_args()

    rclpy.init()
    node = TwistPub(args.topic, args.frame, args.lin, args.ang, args.rate)
    # 关键: use_sim_time 让 self.get_clock() 返回 sim time
    node.set_parameters([rclpy.parameter.Parameter(
        'use_sim_time', rclpy.parameter.Parameter.Type.BOOL, args.use_sim_time
    )])
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
