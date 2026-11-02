#!/usr/bin/env python3
"""Experiment 2.2: 100 Hz trace of dual_home -> q_grasp12 homing.

No attach, no detach, no transport QP.  The arm JTCs run the homing trajectory;
the QP stays inactive.  Timestamps are the Gazebo /world/.../clock values.
"""

import csv
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from control_msgs.msg import JointTrajectoryControllerState
from tf2_msgs.msg import TFMessage
from rosgraph_msgs.msg import Clock
import tf2_ros

OUT = sys.argv[1] if len(sys.argv) > 1 else "homing_trace.csv"
DURATION = float(sys.argv[2]) if len(sys.argv) > 2 else 30.0
LJ = ["left_j1", "left_j2", "left_j3", "left_j4", "left_j5", "left_j6"]
RJ = ["right_j1", "right_j2", "right_j3", "right_j4", "right_j5", "right_j6"]
FINGERS = ["left_finger1_joint", "left_finger2_joint",
           "right_finger1_joint", "right_finger2_joint"]


def qmul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw*bx + ax*bw + ay*bz - az*by,
            aw*by - ax*bz + ay*bw + az*bx,
            aw*bz + ax*by - ay*bx + az*bw,
            aw*bw - ax*bx - ay*by - az*bz)


def qinv(q):
    return (-q[0], -q[1], -q[2], q[3])


class Trace(Node):
    def __init__(self):
        super().__init__("c43_homing_trace")
        self.sim = None
        self.weld = "?"
        self.sample = None
        self.joints = {}
        self.targets = {}
        self.rows = []
        self.last = None
        self.start = time.monotonic()
        self.create_subscription(Clock, "/world/c43_s4_world/clock", self.on_clock, 100)
        self.create_subscription(String, "/c43/weld_state", self.on_weld, 10)
        self.create_subscription(TFMessage, "/c43/weld_pose", self.on_pose, 100)
        self.create_subscription(JointState, "/joint_states", self.on_joints, 100)
        self.create_subscription(
            JointTrajectoryControllerState,
            "/left_arm_controller/controller_state", self.on_left_target, 100)
        self.create_subscription(
            JointTrajectoryControllerState,
            "/right_arm_controller/controller_state", self.on_right_target, 100)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.01, self.tick)
        self.get_logger().info(f"homing trace -> {OUT}, {DURATION:.1f} s wall")

    def on_clock(self, msg):
        self.sim = msg.clock.sec + msg.clock.nanosec * 1e-9

    def on_weld(self, msg):
        self.weld = msg.data.strip()

    def on_pose(self, msg):
        for t in msg.transforms:
            if t.child_frame_id.startswith("c43_sample"):
                p = t.transform.translation
                q = t.transform.rotation
                self.sample = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)

    def on_joints(self, msg):
        for n, p in zip(msg.name, msg.position):
            self.joints[n] = p

    def _targets(self, msg):
        if not msg.joint_names:
            return
        for n, p in zip(msg.joint_names, msg.reference.positions):
            self.targets[n] = p

    def on_left_target(self, msg):
        self._targets(msg)

    def on_right_target(self, msg):
        self._targets(msg)

    def tcp(self, frame):
        try:
            tr = self.tf_buffer.lookup_transform(
                "world", frame, rclpy.time.Time(), Duration(seconds=0.0))
            p = tr.transform.translation
            q = tr.transform.rotation
            return (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
        except Exception:
            return None

    def f(self, x):
        if x is None or x == "":
            return ""
        return f"{x:.9f}"

    def tick(self):
        if time.monotonic() - self.start >= DURATION:
            self.finish()
            return
        sim = self.sim
        if sim is None:
            sim = time.monotonic()
        s = self.sample
        vx = vy = vz = wx = wy = wz = None
        if s is not None and self.last is not None:
            t0, s0 = self.last
            dt = sim - t0
            if dt > 1e-6:
                vx, vy, vz = ((s[i] - s0[i]) / dt for i in range(3))
                qd = qmul((s[3], s[4], s[5], s[6]), qinv((s0[3], s0[4], s0[5], s0[6])))
                if qd[3] < 0.0:
                    qd = tuple(-x for x in qd)
                n = math.sqrt(sum(x * x for x in qd[:3]))
                if n > 1e-9:
                    ang = 2.0 * math.atan2(n, qd[3])
                    wx, wy, wz = (ang * qd[i] / n / dt for i in range(3))
        if s is not None:
            self.last = (sim, s)
        lt = self.tcp("left_grasp_frame")
        rt = self.tcp("right_grasp_frame")
        row = {
            "sim_time": self.f(sim),
            "weld_state": self.weld,
            "sample_x": self.f(s[0]) if s else "", "sample_y": self.f(s[1]) if s else "",
            "sample_z": self.f(s[2]) if s else "",
            "sample_qx": self.f(s[3]) if s else "", "sample_qy": self.f(s[4]) if s else "",
            "sample_qz": self.f(s[5]) if s else "", "sample_qw": self.f(s[6]) if s else "",
            "vx": self.f(vx), "vy": self.f(vy), "vz": self.f(vz),
            "wx": self.f(wx), "wy": self.f(wy), "wz": self.f(wz),
            "support_x": "0.000000000", "support_y": "0.000000000", "support_z": "0.000000000",
            "left_tcp_x": self.f(lt[0]) if lt else "", "left_tcp_y": self.f(lt[1]) if lt else "",
            "left_tcp_z": self.f(lt[2]) if lt else "",
            "right_tcp_x": self.f(rt[0]) if rt else "", "right_tcp_y": self.f(rt[1]) if rt else "",
            "right_tcp_z": self.f(rt[2]) if rt else "",
            "sample_support_contact": "NOT_OBSERVED",
            "sample_gripper_contact": "NOT_OBSERVED",
        }
        for n in LJ + RJ:
            row[n + "_actual"] = self.f(self.joints.get(n))
            row[n + "_target"] = self.f(self.targets.get(n))
        for n in FINGERS:
            row[n] = self.f(self.joints.get(n))
        self.rows.append(row)

    def finish(self):
        with open(OUT, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(self.rows[0].keys()))
            w.writeheader()
            w.writerows(self.rows)
        self.get_logger().info(f"wrote {OUT}: {len(self.rows)} rows")
        self.timer.cancel()
        rclpy.shutdown()


def main():
    rclpy.init()
    n = Trace()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
