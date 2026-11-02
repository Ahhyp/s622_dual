#!/usr/bin/env python3
"""Experiment 2: stationary weld -> detach, 50 Hz recorder.

No transport is started.  The arm is held at q_grasp12 by the QP after homing;
the sample sits at the support centre.  Attach at t=1.0 s, detach at t=2.0 s,
record until t=4.5 s.  Velocities are finite differences of the pose samples
(the Gazebo pose topic carries no velocity); contact flags are NOT_OBSERVED.
"""

import csv
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from std_msgs.msg import Empty, String
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage
import tf2_ros

OUT = sys.argv[1] if len(sys.argv) > 1 else "static_weld.csv"
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


class Recorder(Node):
    def __init__(self):
        super().__init__("c43_static_weld_probe")
        self.attach_pub = self.create_publisher(Empty, "/c43/attach", 10)
        self.detach_pub = self.create_publisher(Empty, "/c43/detach", 10)
        self.create_subscription(String, "/c43/weld_state", self.on_weld, 10)
        self.create_subscription(TFMessage, "/c43/weld_pose", self.on_pose, 10)
        self.create_subscription(JointState, "/joint_states", self.on_joints, 10)
        self.create_subscription(Clock, "/world/c43_s4_world/clock", self.on_clock, 50)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.weld = "?"
        self.sim_time = None
        self.sample = None       # (x,y,z,qx,qy,qz,qw)
        self.support = None
        self.joints = {}
        self.start_wall = None
        self.start = self.get_clock().now()
        self.attach_t = None
        self.detach_t = None
        self.detach_confirm_t = None
        self.rows = []
        self.last = None
        self.event = ""
        self.timer = self.create_timer(1.0 / 50.0, self.tick)
        self.get_logger().info("static weld probe: attach at 1.0 s, detach at 2.0 s")

    def on_weld(self, msg):
        prev = self.weld
        self.weld = msg.data.strip()
        if prev != "detached" and self.weld == "detached" and self.detach_confirm_t is None:
            self.detach_confirm_t = self.get_clock().now()

    def on_pose(self, msg):
        for t in msg.transforms:
            p = t.transform.translation
            q = t.transform.rotation
            v = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
            if t.child_frame_id.startswith("c43_sample"):
                self.sample = v
            elif t.child_frame_id in ("c43_support", "c43_support::base_link"):
                self.support = v

    def on_clock(self, msg):
        self.sim_time = msg.clock.sec + msg.clock.nanosec * 1e-9

    def on_joints(self, msg):
        for n, p in zip(msg.name, msg.position):
            self.joints[n] = p

    def lookup_tcp(self, frame):
        try:
            tr = self.tf_buffer.lookup_transform(
                "world", frame, rclpy.time.Time(), Duration(seconds=0.05))
            p = tr.transform.translation
            q = tr.transform.rotation
            return (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
        except Exception:
            return None

    def tick(self):
        now = self.get_clock().now()
        sim = self.sim_time if self.sim_time is not None else now.nanoseconds * 1e-9
        if self.start_wall is None:
            self.start_wall = time.monotonic()
        elapsed = time.monotonic() - self.start_wall
        # Record first; command/finish decisions must never drop the row.
        s = self.sample
        vx = vy = vz = wx = wy = wz = ""
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

        sup = self.support or (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)
        rel = ""
        if s is not None:
            rel = (s[0] - sup[0], s[1] - sup[1], s[2] - sup[2])
        lt = self.lookup_tcp("left_grasp_frame")
        rt = self.lookup_tcp("right_grasp_frame")
        event = self.event
        self.event = ""
        def f(x):
            return "" if x == "" else f"{x:.9f}"
        row = {
            "sim_time": f"{sim:.9f}",
            "event": event,
            "weld_state": self.weld,
            "sample_x": f(s[0]) if s else "", "sample_y": f(s[1]) if s else "",
            "sample_z": f(s[2]) if s else "",
            "sample_qx": f(s[3]) if s else "", "sample_qy": f(s[4]) if s else "",
            "sample_qz": f(s[5]) if s else "", "sample_qw": f(s[6]) if s else "",
            "vx": f(vx), "vy": f(vy), "vz": f(vz),
            "wx": f(wx), "wy": f(wy), "wz": f(wz),
            "support_x": f(sup[0]), "support_y": f(sup[1]), "support_z": f(sup[2]),
            "rel_x": f(rel[0]) if rel != "" else "",
            "rel_y": f(rel[1]) if rel != "" else "",
            "rel_z": f(rel[2]) if rel != "" else "",
            "left_tcp_x": f(lt[0]) if lt else "", "left_tcp_y": f(lt[1]) if lt else "",
            "left_tcp_z": f(lt[2]) if lt else "",
            "right_tcp_x": f(rt[0]) if rt else "", "right_tcp_y": f(rt[1]) if rt else "",
            "right_tcp_z": f(rt[2]) if rt else "",
            "sample_support_contact": "NOT_OBSERVED",
            "sample_gripper_contact": "NOT_OBSERVED",
            "detach_cmd_time": "" if self.detach_t is None else f(
                self.detach_t.nanoseconds * 1e-9),
            "detach_confirmed_time": "" if self.detach_confirm_t is None else f(
                self.detach_confirm_t.nanoseconds * 1e-9),
        }
        for n in FINGERS:
            row[n] = f(self.joints.get(n, ""))
        self.rows.append(row)

        if self.attach_t is None and elapsed >= 1.0:
            self.attach_pub.publish(Empty())
            self.attach_t = now
            self.event = "attach_cmd"
            self.get_logger().info("attach command")
        if self.detach_t is None and elapsed >= 2.0:
            self.detach_pub.publish(Empty())
            self.detach_t = now
            self.event = "detach_cmd"
            self.get_logger().info("detach command")
        if self.detach_t is not None and elapsed >= 2.0 + 2.5:
            self.finish()

    def finish(self):
        if not self.rows:
            self.get_logger().error("no rows recorded")
        else:
            with open(OUT, "w", newline="") as f:
                w = csv.DictWriter(f, fieldnames=list(self.rows[0].keys()))
                w.writeheader()
                w.writerows(self.rows)
            self.get_logger().info(f"wrote {OUT}: {len(self.rows)} rows")
        self.timer.cancel()
        rclpy.shutdown()


def main():
    rclpy.init()
    n = Recorder()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
