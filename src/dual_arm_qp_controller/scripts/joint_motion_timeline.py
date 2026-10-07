#!/usr/bin/env python3
"""Print per-window max |dq| of selected joints, to observe freeze/hold events.

Usage:
  python3 joint_motion_timeline.py --joint left_j2,right_j2 --duration 8 --window 0.25
"""
import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class Timeline(Node):
    def __init__(self, joints, duration, window):
        super().__init__("joint_motion_timeline")
        self.joints = joints
        self.duration = duration
        self.window = window
        self.start = time.time()
        self.win_start = self.start
        self.last = {}
        self.win_max = {j: 0.0 for j in joints}
        self.done = False
        self.create_subscription(JointState, "/joint_states", self.cb, 50)

    def cb(self, msg):
        now = time.time()
        idx = {n: i for i, n in enumerate(msg.name)}
        for j in self.joints:
            if j not in idx:
                continue
            v = msg.position[idx[j]]
            if j in self.last:
                self.win_max[j] = max(self.win_max[j], abs(v - self.last[j]))
            self.last[j] = v
        if now - self.win_start >= self.window:
            t = now - self.start
            print(f"wall={now:.3f} t={t:6.2f}s  " + "  ".join(
                f"{j}={self.win_max[j]:.5f}" for j in self.joints), flush=True)
            self.win_max = {j: 0.0 for j in self.joints}
            self.win_start = now
        if now - self.start >= self.duration:
            self.done = True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--joint", default="left_j2,right_j2")
    ap.add_argument("--duration", type=float, default=8.0)
    ap.add_argument("--window", type=float, default=0.25)
    args, _ = ap.parse_known_args()

    rclpy.init()
    node = Timeline([j.strip() for j in args.joint.split(",") if j.strip()],
                    args.duration, args.window)
    while rclpy.ok() and not node.done:
        rclpy.spin_once(node, timeout_sec=0.05)
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
