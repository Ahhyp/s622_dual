#!/usr/bin/env python3
"""2.5-D4c lift trace: one continuous 100 Hz record of sample pose, finger/arm
joints and finger<->sample contact events, for a slow vertical lift test.

Usage: C43_LT_TAG=base C43_LT_DUR=16 C43_LT_CONTACT=<topic> python3 c43_lift_trace.py
Writes <OUTDIR>/lift_<TAG>.csv and lift_<TAG>_contacts.csv, then a summary.
"""
import csv
import json
import os
import re
import subprocess
import threading
import time

import rclpy
from rclpy.node import Node
from control_msgs.msg import DynamicJointState
from sensor_msgs.msg import JointState
from rosgraph_msgs.msg import Clock
from tf2_msgs.msg import TFMessage

OUTDIR = os.environ.get(
    "C43_CC_OUTDIR",
    "/workspace/docs/双臂协同/双臂协同运动控制与优化/T1_QP满速定位_20261029")
TAG = os.environ.get("C43_LT_TAG", "lift")
DUR = float(os.environ.get("C43_LT_DUR", "16"))
TOPIC = os.environ.get(
    "C43_LT_CONTACT",
    "/world/c43_s4_world/model/c43_sample_grip/link/base_link/sensor/sample_contact/contact")
ARM = [f"left_j{i}" for i in range(1, 7)] + [f"right_j{i}" for i in range(1, 7)]
FING = ["left_finger1_joint", "left_finger2_joint",
        "right_finger1_joint", "right_finger2_joint"]


class Trace(Node):
    def __init__(self):
        super().__init__("c43_lift_trace")
        self.mono0 = time.monotonic()
        self.sim = None
        self.sample = None
        self.js = {}
        self.eff = {}
        self.rows = []
        self.contacts = []
        self.lock = threading.Lock()
        self.create_subscription(JointState, "/joint_states", self.on_js, 50)
        self.create_subscription(DynamicJointState, "/dynamic_joint_states",
                                 self.on_eff, 50)
        self.create_subscription(Clock, "/world/c43_s4_world/clock", self.on_clock, 20)
        self.create_subscription(TFMessage, "/c43/weld_pose", self.on_tf, 20)
        threading.Thread(target=self.read_contacts, daemon=True).start()

    def now(self):
        return time.monotonic() - self.mono0

    def on_clock(self, msg):
        self.sim = msg.clock.sec + msg.clock.nanosec * 1e-9

    def on_tf(self, msg):
        for t in msg.transforms:
            if t.child_frame_id.startswith("c43_sample"):
                p, q = t.transform.translation, t.transform.rotation
                self.sample = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)

    def on_eff(self, msg):
        for name, iv in zip(msg.joint_names, msg.interface_values):
            if "effort" in iv.interface_names:
                self.eff[name] = iv.values[iv.interface_names.index("effort")]

    def on_js(self, msg):
        d = dict(zip(msg.name, msg.position))
        if not all(j in d for j in ARM + FING):
            return
        self.js = d
        with self.lock:
            self.rows.append({"mono": self.now(), "sim": self.sim,
                              **{j: d[j] for j in ARM + FING},
                              **{j + "_eff": self.eff.get(j) for j in FING},
                              "sample": self.sample})

    def read_contacts(self):
        try:
            proc = subprocess.Popen(["ign", "topic", "-e", "-t", TOPIC],
                                    stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                                    text=True, bufsize=1)
        except Exception:
            return
        stamp = (0, 0)
        in_contact = False
        depth = 0
        block = []
        for line in proc.stdout:
            m = re.search(r"(?<!n)sec:\s*(\d+)", line)
            if m:
                stamp = (int(m.group(1)), stamp[1])
            m = re.search(r"nsec:\s*(\d+)", line)
            if m:
                stamp = (stamp[0], int(m.group(1)))
            if not in_contact:
                if line.strip() == "contact {":
                    in_contact = True
                    depth = 1
                    block = []
                continue
            block.append(line)
            depth += line.count("{") - line.count("}")
            if depth <= 0:
                in_contact = False
                text = "".join(block)
                names = re.findall(r'name:\s*"([^"]+)"', text)
                fingers = [n for n in names if "finger" in n]
                if any("c43_sample" in n for n in names):
                    with self.lock:
                        for f in fingers:
                            self.contacts.append({
                                "mono": self.now(),
                                "sim": stamp[0] + stamp[1] * 1e-9,
                                "finger": f,
                                "side": "left" if "left_finger" in f else "right",
                            })

    def summary(self):
        with self.lock:
            rows = list(self.rows)
            contacts = list(self.contacts)
        z = [(r["mono"], r["sample"][2]) for r in rows if r["sample"]]
        out = {"tag": TAG, "duration_s": DUR, "rows": len(rows), "contacts": len(contacts)}
        if z:
            z0 = z[0][1]
            zmax = max(z, key=lambda t: t[1])
            out.update({
                "sample_z_start": round(z0, 6),
                "sample_z_max": round(zmax[1], 6),
                "lift_mm": round((zmax[1] - z0) * 1000, 3),
                "t_lift_max_s": round(zmax[0], 3),
                "sample_z_end": round(z[-1][1], 6),
                "end_vs_max_mm": round((z[-1][1] - zmax[1]) * 1000, 3),
            })
        return out


def main():
    rclpy.init()
    n = Trace()
    t0 = time.monotonic()
    while time.monotonic() - t0 < DUR:
        rclpy.spin_once(n, timeout_sec=0.01)
    os.makedirs(OUTDIR, exist_ok=True)
    with n.lock:
        rows, contacts = list(n.rows), list(n.contacts)
    with open(os.path.join(OUTDIR, f"lift_{TAG}.csv"), "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["mono", "sim"] + ARM + FING +
                   [j + "_eff" for j in FING] + ["sample_x", "sample_y", "sample_z",
                    "sample_qx", "sample_qy", "sample_qz", "sample_qw"])
        for r in rows:
            s = r["sample"] or (None,) * 7
            w.writerow([r["mono"], r["sim"]] + [r[j] for j in ARM + FING] +
                       [r.get(j + "_eff") for j in FING] + list(s))
    print(json.dumps(n.summary(), indent=2))
    with open(os.path.join(OUTDIR, f"lift_{TAG}_contacts.csv"), "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["mono", "sim", "finger", "side"])
        for c in contacts:
            w.writerow([c["mono"], c["sim"], c["finger"], c["side"]])
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
