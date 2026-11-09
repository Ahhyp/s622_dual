#!/usr/bin/env python3
"""2.5-D full-contact C2 capture.

Independent of the transport executor and of the old first-pair-only probe.
- Parse EVERY collision pair in EVERY sample contact message; normalize both
  collision orders and record left_finger1/left_finger2 separately.
- First valid finger-sample contact requests JTC cancel; capture continues.
- Record sim clock + monotonic wall clock, all 12 arm joints + fingers,
  full sample 6D pose, contact points, derived face/normal, cancel/action events.
- Confirm physical stop, then observe ~1 s; no attach/weld.
"""
import csv, json, math, os, re, subprocess, threading, time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from rosgraph_msgs.msg import Clock
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf2_msgs.msg import TFMessage

OUTDIR = "/workspace/docs/双臂协同/双臂协同运动控制与优化/T1_QP满速定位_20261029"
TAG = os.environ.get("C43_CC_TAG", "c2_full")
TARGET = float(os.environ.get("C43_CC_TARGET", "0.0249"))
CLOSE_S = float(os.environ.get("C43_CC_CLOSE_S", "8.0"))
TOPIC = os.environ.get(
    "C43_CC_CONTACT_TOPIC",
    "/world/c43_s4_world/model/c43_sample_contact/link/base_link/sensor/sample_contact/contact")
POST_STOP_S = float(os.environ.get("C43_CC_POST_STOP_S", "1.0"))
JOINTS = ["left_finger1_joint", "left_finger2_joint"]
ARM = [f"left_j{i}" for i in range(1, 7)] + [f"right_j{i}" for i in range(1, 7)]
FINGERS = ["left_finger1_joint", "left_finger2_joint",
           "right_finger1_joint", "right_finger2_joint"]
HALF = (0.06, 0.025, 0.025)

class Capture(Node):
    def __init__(self):
        super().__init__("c43_contact_capture")
        self.create_subscription(JointState, "/joint_states", self.on_js, 50)
        self.create_subscription(Clock, "/world/c43_s4_world/clock", self.on_clock, 20)
        self.create_subscription(TFMessage, "/c43/weld_pose", self.on_tf, 20)
        self.ac = ActionClient(self, FollowJointTrajectory,
                               "/left_hand_controller/follow_joint_trajectory")
        self.mono0 = time.monotonic()
        self.js = {}
        self.sim = None
        self.clock_rx_mono = None   # monotonic time of the last /clock message
        self.sample = None
        self.rows = []
        self.contacts = []       # every finger-sample contact pair
        self.blocks = []         # every message with finger contact
        self.lock = threading.Lock()
        self.first_contact = None
        self.first_by_finger = {}
        self.cancel_send = None
        self.cancel_confirm = None
        self.result = None
        self.result_future = None
        self.result_ready = None
        self.stop = None
        self.post_stop_end = None
        self.goal_handle = None
        self.closed_done = False
        self.thread = threading.Thread(target=self.read_contacts, daemon=True)
        self.thread.start()

    def now(self):
        return time.monotonic() - self.mono0

    def on_clock(self, msg):
        self.sim = msg.clock.sec + msg.clock.nanosec * 1e-9
        self.clock_rx_mono = self.now()

    def sim_now(self):
        """(sim, clock_age).  sim is the last /clock received, NOT the exact
        instant of the event; clock_age is how stale it was, in seconds."""
        age = None if self.clock_rx_mono is None else self.now() - self.clock_rx_mono
        return self.sim, age

    def on_js(self, msg):
        d = dict(zip(msg.name, msg.position))
        if all(j in d for j in ARM + FINGERS):
            self.js = d
            if self.sample is not None:
                self.rows.append({
                    "mono": self.now(), "sim": self.sim,
                    **{j: d[j] for j in ARM + FINGERS},
                    "sample": self.sample,
                })

    def on_tf(self, msg):
        for t in msg.transforms:
            if t.child_frame_id.startswith("c43_sample"):
                p = t.transform.translation
                q = t.transform.rotation
                self.sample = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)

    def read_contacts(self):
        try:
            proc = subprocess.Popen(["ign", "topic", "-e", "-t", TOPIC],
                                    stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                                    text=True, bufsize=1)
        except Exception as e:
            self.get_logger().error(f"contact subprocess failed: {e}")
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
                self.process_block(block, stamp)
                in_contact = False

    def process_block(self, block, stamp):
        text = "".join(block)
        names = re.findall(r'name:\s*"([^"]+)"', text)
        sample_names = [n for n in names if "c43_sample" in n]
        fingers = [n for n in names if ("left_finger" in n or "right_finger" in n)]
        if not sample_names or not fingers:
            return
        pts = []
        for m in re.finditer(
                r"position\s*\{\s*x:\s*([-+0-9.eE]+)\s*y:\s*([-+0-9.eE]+)\s*z:\s*([-+0-9.eE]+)",
                text, re.S):
            pts.append((float(m.group(1)), float(m.group(2)), float(m.group(3))))
        normals = []
        for m in re.finditer(
                r"normal\s*\{\s*x:\s*([-+0-9.eE]+)\s*y:\s*([-+0-9.eE]+)\s*z:\s*([-+0-9.eE]+)",
                text, re.S):
            normals.append((float(m.group(1)), float(m.group(2)), float(m.group(3))))
        recv = self.now()
        with self.lock:
            self.blocks.append({"sim_sec": stamp[0], "sim_nsec": stamp[1],
                                "recv_mono": recv, "names": names,
                                "points": pts, "normals": normals})
            raw1 = names[0] if names else None
            raw2 = names[1] if len(names) > 1 else None
            for finger in fingers:
                side = "left" if "left_finger" in finger else "right"
                rec = {
                    "sim_sec": stamp[0], "sim_nsec": stamp[1],
                    "recv_mono": recv,
                    "raw_collision1": raw1, "raw_collision2": raw2,
                    "normalized_collision1": sample_names[0],
                    "normalized_collision2": finger,
                    "direction": ("sample_first" if raw1 in sample_names else "finger_first"),
                    "sample_name": sample_names[0],
                    "finger": finger, "side": side,
                    "points_world": pts, "normals_world": normals,
                    "sample_pose": self.sample,
                    "joints": {j: self.js.get(j) for j in ARM + FINGERS},
                }
                self.contacts.append(rec)
                if finger not in self.first_by_finger:
                    self.first_by_finger[finger] = rec
                if self.first_contact is None:
                    self.first_contact = rec
                    self.get_logger().info(
                        f"FIRST FINGER CONTACT {finger} sim={stamp[0]}.{stamp[1]:09d}")

    def classify_contact(self, rec):
        if not rec["points_world"] or not rec["sample_pose"]:
            return None
        p = rec["points_world"][0]
        qx, qy, qz, qw = rec["sample_pose"][3:]
        # inverse quaternion rotate (p - t)
        v = (p[0] - rec["sample_pose"][0],
             p[1] - rec["sample_pose"][1],
             p[2] - rec["sample_pose"][2])
        # q^-1 = (-x,-y,-z,w)
        q = (-qx, -qy, -qz, qw)
        vx, vy, vz = v
        dot = q[0]*vx + q[1]*vy + q[2]*vz
        cross = (q[1]*vz - q[2]*vy, q[2]*vx - q[0]*vz, q[0]*vy - q[1]*vx)
        local = (vx + 2*(q[3]*cross[0] + q[1]*cross[2] - q[2]*cross[1]),
                 vy + 2*(q[3]*cross[1] + q[2]*cross[0] - q[0]*cross[2]),
                 vz + 2*(q[3]*cross[2] + q[0]*cross[1] - q[1]*cross[0]))
        faces = []
        for i, axis in enumerate("xyz"):
            if abs(abs(local[i]) - HALF[i]) < 0.0015:
                faces.append(axis + ("+" if local[i] >= 0 else "-"))
        normals = []
        for f in faces:
            axis = "xyz".index(f[0])
            sign = 1 if f[1] == "+" else -1
            normals.append(tuple(sign if k == axis else 0.0 for k in range(3)))
        normal = normals[0] if len(normals) == 1 else None
        return {"sample_local": local, "faces": faces,
                "face_normals_sample": normals, "normal_sample": normal}

    def cancel(self):
        self.cancel_send = self.now()
        self.cancel_sim, self.cancel_clock_age = self.sim_now()
        fut = self.goal_handle.cancel_goal_async()
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        r = fut.result()
        self.cancel_confirm = self.now()
        self.get_logger().info(
            f"CANCEL sent={self.cancel_send:.6f} confirmed={self.cancel_confirm:.6f} resp={r}")

    def run(self):
        if not self.ac.wait_for_server(timeout_sec=30.0):
            raise RuntimeError("left_hand_controller action server not available")
        for _ in range(200):
            rclpy.spin_once(self, timeout_sec=0.01)
        self.get_logger().info(f"initial fingers={self.js} sample={self.sample} target={TARGET}")
        # Open first.
        g = FollowJointTrajectory.Goal()
        tr = JointTrajectory(); tr.joint_names = JOINTS
        p = JointTrajectoryPoint(); p.positions = [0.0305, -0.0305]
        p.time_from_start.sec = 1
        tr.points.append(p); g.trajectory = tr
        f = self.ac.send_goal_async(g)
        rclpy.spin_until_future_complete(self, f, timeout_sec=15)
        h = f.result()
        rf = h.get_result_async()
        rclpy.spin_until_future_complete(self, rf, timeout_sec=10)
        self.get_logger().info("open done")
        for _ in range(100):
            rclpy.spin_once(self, timeout_sec=0.01)

        initial_sample = self.sample
        # Close, but do not wait for the action: first contact must cancel it.
        g = FollowJointTrajectory.Goal()
        tr = JointTrajectory(); tr.joint_names = JOINTS
        p = JointTrajectoryPoint(); p.positions = [TARGET, -TARGET]
        p.time_from_start.sec = int(CLOSE_S)
        p.time_from_start.nanosec = int((CLOSE_S - int(CLOSE_S)) * 1e9)
        tr.points.append(p); g.trajectory = tr
        fut = self.ac.send_goal_async(g)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=15)
        self.goal_handle = fut.result()
        if self.goal_handle is None or not self.goal_handle.accepted:
            raise RuntimeError("close goal not accepted")
        self.result_future = self.goal_handle.get_result_async()
        close_start = self.now()
        self.get_logger().info(f"close target={TARGET} duration={CLOSE_S} accepted")

        while self.now() - close_start < 20.0:
            rclpy.spin_once(self, timeout_sec=0.005)
            if self.result_future is not None and self.result is None and self.result_future.done():
                self.result = self.result_future.result()
                self.result_ready = self.now()
                self.result_sim, self.result_clock_age = self.sim_now()
            if self.first_contact is not None and self.cancel_send is None:
                self.cancel()
            if self.cancel_send is not None:
                if self.stop is None and len(self.rows) > 40:
                    win = [r for r in self.rows if r["mono"] >= self.cancel_send]
                    if len(win) >= 20:
                        recent = win[-20:]
                        spans = []
                        for j in JOINTS:
                            vals = [r[j] for r in recent]
                            spans.append(max(vals) - min(vals))
                        if max(spans) < 0.0001:
                            stop_mono = self.now()
                            stop_sim, stop_clock_age = self.sim_now()
                            self.stop = {"mono": stop_mono, "fingers": {
                                j: self.js.get(j) for j in JOINTS},
                                "spans": spans,
                                "sim": stop_sim,
                                "clock_age_s": stop_clock_age}
                            self.get_logger().info(f"STOP_CONFIRMED fingers={self.stop['fingers']}")
                if self.stop is not None and self.now() - self.stop["mono"] >= POST_STOP_S:
                    self.post_stop_end = self.now()
                    self.post_stop_sim, self.post_stop_clock_age = self.sim_now()
                    break
            if self.first_contact is None and self.result is not None:
                # No contact: allow a short settle then finish.
                if self.now() - close_start > CLOSE_S + 1.0:
                    break

        if self.result is not None:
            result_status = getattr(self.result, "status", None)
            rr = self.result.result
            error_code = rr.error_code; error_string = rr.error_string
        else:
            result_status = None; error_code = None; error_string = ""

        # Final samples and analysis.
        for _ in range(50):
            rclpy.spin_once(self, timeout_sec=0.01)
        final_sample = self.sample
        first = self.first_contact
        cls = self.classify_contact(first) if first else None
        events = []
        # sim for message-derived events is exact (message stamp).  For everything
        # else it is the last /clock we received; clock_age_s says how stale it was.
        if first:
            events.append({"event": "first_contact", "sim": first["sim_sec"] + first["sim_nsec"]*1e-9,
                           "sim_is_approx": False, "clock_age_s": None,
                           "mono": first["recv_mono"], "detail": first["finger"]})
            for finger, rec in self.first_by_finger.items():
                events.append({"event": "finger_first", "sim": rec["sim_sec"] + rec["sim_nsec"]*1e-9,
                               "sim_is_approx": False, "clock_age_s": None,
                               "mono": rec["recv_mono"], "detail": finger})
        if self.cancel_send is not None:
            events.append({"event": "cancel_send", "sim": self.cancel_sim, "sim_is_approx": True,
                           "clock_age_s": self.cancel_clock_age,
                           "mono": self.cancel_send, "detail": ""})
            events.append({"event": "cancel_confirm", "sim": self.cancel_sim, "sim_is_approx": True,
                           "clock_age_s": self.cancel_clock_age,
                           "mono": self.cancel_confirm, "detail": ""})
        if self.result_ready is not None:
            events.append({"event": "action_result", "sim": self.result_sim, "sim_is_approx": True,
                           "clock_age_s": self.result_clock_age, "mono": self.result_ready,
                           "detail": f"status={result_status} error={error_code} {error_string}"})
        if self.stop:
            events.append({"event": "stop_confirmed", "sim": self.stop.get("sim"),
                           "sim_is_approx": True, "clock_age_s": self.stop.get("clock_age_s"),
                           "mono": self.stop["mono"], "detail": ""})
        if self.post_stop_end:
            events.append({"event": "post_stop_end", "sim": self.post_stop_sim,
                           "sim_is_approx": True, "clock_age_s": self.post_stop_clock_age,
                           "mono": self.post_stop_end, "detail": ""})

        max_disp = 0.0; max_row = None
        if initial_sample and self.rows:
            for r in self.rows:
                s = r["sample"]
                if not s: continue
                d = math.sqrt(sum((s[i] - initial_sample[i])**2 for i in range(3)))
                if d > max_disp:
                    max_disp = d; max_row = r
        post_contacts = [c for c in self.contacts if self.stop and c["recv_mono"] > self.stop["mono"] + 0.2]
        finger_counts = {}
        for c in self.contacts:
            finger_counts[c["finger"]] = finger_counts.get(c["finger"], 0) + 1

        # Three SEPARATE facts.  Do not collapse them into one boolean:
        #   ever_contacted  : the finger produced at least one contact pair
        #   same_message    : one contact message carried both left fingers
        #   sustained       : both left fingers still contact 0.2 s after stop
        def _has(keys, needle):
            return sorted(k for k in keys if needle in k)
        ever_1 = _has(self.first_by_finger, "left_finger1")
        ever_2 = _has(self.first_by_finger, "left_finger2")
        same_msg_count = 0
        for b in self.blocks:
            joined = " ".join(n for n in b["names"] if "left_finger" in n)
            if "left_finger1" in joined and "left_finger2" in joined:
                same_msg_count += 1
        post_fingers = sorted({c["finger"] for c in post_contacts})
        sustained_both = (any("left_finger1" in f for f in post_fingers) and
                          any("left_finger2" in f for f in post_fingers))

        rec = {
            "tag": TAG, "target": TARGET, "contact_sensor_topic": TOPIC,
            "first_contact": first, "first_contact_classification": cls,
            "fingers_first": self.first_by_finger,
            "left_finger1_ever_contacted": bool(ever_1),
            "left_finger2_ever_contacted": bool(ever_2),
            "both_left_fingers_ever_contacted": bool(ever_1) and bool(ever_2),
            "same_message": same_msg_count > 0,
            "same_message_count": same_msg_count,
            "both_left_fingers": bool(ever_1) and bool(ever_2),  # kept: same as *_ever_contacted
            "sustained_both_after_stop": sustained_both,
            "contact_pair_count": len(self.contacts),
            "finger_contact_counts": finger_counts,
            "contact_message_blocks": len(self.blocks),
            "cancel_send_mono": self.cancel_send,
            "cancel_confirm_mono": self.cancel_confirm,
            "action_result": {"status": result_status, "error_code": error_code,
                              "error_string": error_string, "mono": self.result_ready},
            "stop": self.stop, "post_stop_end_mono": self.post_stop_end,
            "post_stop_contacts_after_0.2s": len(post_contacts),
            "post_stop_contact_fingers": sorted({c["finger"] for c in post_contacts}),
            "post_stop_contact_times": [c["recv_mono"] for c in post_contacts[-10:]],
            "initial_sample": initial_sample, "final_sample": final_sample,
            "max_sample_displacement_m": max_disp,
            "max_sample_displacement_row": max_row,
            "events": events,
        }
        os.makedirs(OUTDIR, exist_ok=True)
        with open(os.path.join(OUTDIR, f"contact_capture_{TAG}.json"), "w") as fh:
            json.dump(rec, fh, indent=2)
        with open(os.path.join(OUTDIR, f"contact_capture_{TAG}_raw_contacts.json"), "w") as fh:
            json.dump({"contacts": self.contacts, "blocks": self.blocks}, fh, indent=1)
        with open(os.path.join(OUTDIR, f"contact_capture_{TAG}_events.csv"), "w", newline="") as fh:
            w = csv.writer(fh)
            w.writerow(["event", "mono", "sim", "sim_is_approx", "clock_age_s", "detail"])
            for e in events:
                w.writerow([e["event"], e["mono"], e.get("sim"), e.get("sim_is_approx"),
                            e.get("clock_age_s"), e["detail"]])
        with open(os.path.join(OUTDIR, f"contact_capture_{TAG}_joints.csv"), "w", newline="") as fh:
            w = csv.writer(fh)
            w.writerow(["mono", "sim"] + ARM + FINGERS +
                       ["sample_x", "sample_y", "sample_z", "sample_qx", "sample_qy", "sample_qz", "sample_qw"])
            for r in self.rows:
                s = r["sample"] or (None,)*7
                w.writerow([r["mono"], r["sim"]] + [r[j] for j in ARM + FINGERS] + list(s))
        print(json.dumps(rec, indent=2))
        self.get_logger().info(f"WROTE contact_capture_{TAG}.*")

def main():
    rclpy.init(); n = Capture()
    try:
        n.run()
    finally:
        n.destroy_node(); rclpy.shutdown()

if __name__ == "__main__":
    main()
