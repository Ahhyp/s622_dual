#!/usr/bin/env python3
"""S4-T16 test injection: relay /c43/weld_pose with the sample raised 5 mm.

This is a ROS-boundary injection, not a physical lift.  The executor sees a
sample pose 5 mm above the frozen grasp; the arms stay at q_grasp12, so the start
gate and the JTC path stay real.  Usage:
    python3 pose_bias_relay.py <src_topic> <dst_topic> <bias_m>
"""
import sys
import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage

class Relay(Node):
    def __init__(self, src, dst, bias):
        super().__init__("s4t16_pose_bias_relay")
        self.bias = bias
        self.pub = self.create_publisher(TFMessage, dst, 10)
        self.create_subscription(TFMessage, src, self.cb, 10)
        self.n = 0
    def cb(self, m):
        out = TFMessage()
        for t in m.transforms:
            if t.child_frame_id == "c43_sample":
                t.transform.translation.z += self.bias
            out.transforms.append(t)
        self.pub.publish(out)
        self.n += 1
        if self.n == 1 or self.n % 200 == 0:
            self.get_logger().info("relayed %d poses, sample z bias %.4f m" % (self.n, self.bias))

def main():
    src = sys.argv[1] if len(sys.argv) > 1 else "/c43/weld_pose"
    dst = sys.argv[2] if len(sys.argv) > 2 else "/c43/test_weld_pose"
    bias = float(sys.argv[3]) if len(sys.argv) > 3 else 0.005
    rclpy.init()
    n = Relay(src, dst, bias)
    rclpy.spin(n)

if __name__ == "__main__":
    main()
