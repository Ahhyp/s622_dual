#!/usr/bin/env python3
"""Experiment 2.5-C: contact -> cancel -> stop diagnostic.

Independent of the transport executor.  Direct left-hand JTC only.
- C0 (no sample): cancel after a fixed delay, verify CANCELED + physical stop.
- C1/C2 (sample): subscribe the sample contact sensor via `ign topic -e`;
  on the FIRST left/right finger contact, cancel the JTC, then measure the
  physical stop, extra closing travel and sample displacement.  No attach.
"""
import csv, json, os, re, subprocess, threading, time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf2_msgs.msg import TFMessage

OUTDIR = "/workspace/docs/双臂协同/双臂协同运动控制与优化/T1_QP满速定位_20261029"
JOINTS = ["left_finger1_joint", "left_finger2_joint"]
TARGET = float(os.environ.get("C43_CS_TARGET", "0.0249"))
TOPIC = os.environ.get("C43_CS_CONTACT_TOPIC",
    "/world/c43_s4_world/model/c43_sample_contact/link/base_link/sensor/sample_contact/contact")
CANCEL_AFTER = float(os.environ.get("C43_CS_CANCEL_AFTER_S", "0"))
TAG = os.environ.get("C43_CS_TAG", "c2")

class Probe(Node):
    def __init__(self):
        super().__init__("c43_contact_stop_probe")
        self.create_subscription(JointState, "/joint_states", self.on_js, 50)
        self.create_subscription(TFMessage, "/c43/weld_pose", self.on_tf, 20)
        self.ac = ActionClient(self, FollowJointTrajectory,
                               "/left_hand_controller/follow_joint_trajectory")
        self.f = None; self.sample = None; self.rows = []
        self.t0 = time.time()
        self.goal_handle = None; self.result_future = None; self.result = None
        self.contact = None; self.contact_lock = threading.Lock()
        self.stop = None; self.cancel_send = None; self.cancel_confirm = None
        self.contact_samples = []
        self.create_timer(0.01, self.tick)
        if TOPIC:
            threading.Thread(target=self.read_contacts, daemon=True).start()

    def now(self): return time.time() - self.t0

    def on_js(self, msg):
        d = dict(zip(msg.name, msg.position))
        if all(j in d for j in JOINTS):
            self.f = (d[JOINTS[0]], d[JOINTS[1]])
            self.rows.append([self.now(), self.f[0], self.f[1]])
            if self.contact:
                self.contact_samples.append(self.f)

    def on_tf(self, msg):
        for t in msg.transforms:
            if t.child_frame_id.startswith("c43_sample"):
                p = t.transform.translation
                self.sample = (p.x, p.y, p.z)

    def read_contacts(self):
        try:
            proc = subprocess.Popen(["ign", "topic", "-e", "-t", TOPIC],
                                    stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                                    text=True, bufsize=1)
        except Exception as e:
            self.get_logger().error(f"contact subprocess failed: {e}")
            return
        stamps = []
        c1 = c2 = None
        sec = nsec = None
        for line in proc.stdout:
            m = re.search(r"(?<!n)sec:\s*(\d+)", line)
            if m: sec = int(m.group(1))
            m = re.search(r"nsec:\s*(\d+)", line)
            if m: nsec = int(m.group(1))
            m = re.search(r'name:\s*"([^"]+)"', line)
            if not m: continue
            name = m.group(1)
            if "c43_sample" in name:
                # collision1 is the sample; keep looking for collision2
                c1 = name; c2 = None; continue
            if c1 and ("left_finger" in name or "right_finger" in name):
                c2 = name
                with self.contact_lock:
                    if self.contact is None:
                        self.contact = {
                            "sim_sec": sec, "sim_nsec": nsec,
                            "wall_rel": self.now(), "collision1": c1,
                            "collision2": c2, "sides": sorted(
                                {"left" if "left_finger" in c2 else "right"}),
                            "sample": self.sample, "fingers": self.f,
                        }
                        self.get_logger().info(
                            f"FIRST FINGER CONTACT sim={sec}.{nsec:09d} {c2} "
                            f"sample={self.sample} fingers={self.f}")
                c1 = c2 = None

    def tick(self):
        if self.result_future is not None and self.result is None:
            if self.result_future.done():
                self.result = self.result_future.result()
                self.result_wall = self.now()
        # stop detection after cancel: 0.3 s window, < 0.0001 m total movement
        if self.cancel_send is not None and self.stop is None and len(self.rows) > 40:
            win = self.rows[-30:]
            span = max(abs(a[1]-b[1])+abs(a[2]-b[2]) for a in win for b in win[-1:])
            if span < 0.0001:
                self.stop = {"wall_rel": self.now(), "fingers": self.f}
                self.get_logger().info(f"STOP_CONFIRMED fingers={self.f} at t={self.now():.3f}")

    def send_close(self, target, duration):
        goal = FollowJointTrajectory.Goal()
        tr = JointTrajectory(); tr.joint_names = JOINTS
        pt = JointTrajectoryPoint(); pt.positions = [target, -target]
        pt.time_from_start.sec = int(duration)
        pt.time_from_start.nanosec = int((duration-int(duration))*1e9)
        tr.points.append(pt); goal.trajectory = tr
        fut = self.ac.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=15.0)
        self.goal_handle = fut.result()
        if self.goal_handle is None or not self.goal_handle.accepted:
            raise RuntimeError("close goal not accepted")
        self.result_future = self.goal_handle.get_result_async()
        self.get_logger().info(f"close target={target} duration={duration} accepted")

    def cancel(self):
        self.cancel_send = self.now()
        fut = self.goal_handle.cancel_goal_async()
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        r = fut.result()
        self.cancel_confirm = self.now()
        self.get_logger().info(f"CANCEL sent={self.cancel_send:.3f} confirmed={self.cancel_confirm:.3f} resp={r}")

    def run(self):
        if not self.ac.wait_for_server(timeout_sec=30.0):
            raise RuntimeError("left_hand_controller action server not available")
        for _ in range(200): rclpy.spin_once(self, timeout_sec=0.01)
        self.get_logger().info(f"initial fingers={self.f} sample={self.sample} target={TARGET}")
        # Open first.
        g = FollowJointTrajectory.Goal(); tr = JointTrajectory(); tr.joint_names = JOINTS
        p = JointTrajectoryPoint(); p.positions = [0.0305, -0.0305]; p.time_from_start.sec = 1
        tr.points.append(p); g.trajectory = tr
        f = self.ac.send_goal_async(g); rclpy.spin_until_future_complete(self, f, timeout_sec=15)
        h = f.result(); rf = h.get_result_async(); rclpy.spin_until_future_complete(self, rf, timeout_sec=10)
        self.get_logger().info("open done")
        for _ in range(50): rclpy.spin_once(self, timeout_sec=0.01)
        initial_sample = self.sample
        self.contact = None; self.cancel_send = None; self.stop = None
        self.send_close(TARGET, 8.0)
        close_start = self.now()
        # Main monitor loop.
        while self.now() - close_start < 20.0:
            rclpy.spin_once(self, timeout_sec=0.01)
            c = self.contact
            if c and self.cancel_send is None:
                self.cancel()
            if CANCEL_AFTER > 0 and self.cancel_send is None and self.now() - close_start >= CANCEL_AFTER:
                self.cancel()
            if self.cancel_send is not None and self.stop is not None:
                break
        # Give the result/cancel a moment.
        for _ in range(100): rclpy.spin_once(self, timeout_sec=0.01)
        final_sample = self.sample
        result_status = None; error_code = None; error_string = ""
        if self.result is not None:
            result_status = getattr(self.result, "status", None)
            rr = self.result.result
            error_code = rr.error_code; error_string = rr.error_string
        rec = {
            "tag": TAG, "target": TARGET, "contact_sensor_topic": TOPIC,
            "contact": self.contact, "cancel_send_rel": self.cancel_send,
            "cancel_confirm_rel": self.cancel_confirm,
            "stop": self.stop, "result_status": result_status,
            "error_code": error_code, "error_string": error_string,
            "result_ready_rel": getattr(self, "result_wall", None),
            "initial_sample": initial_sample, "final_sample": final_sample,
            "fingers_final": self.f,
            "contact_fingers": self.contact_samples[0] if self.contact_samples else None,
            "extra_close_f1": (abs(self.stop["fingers"][0] - self.contact_samples[0][0])
                               if self.stop and self.contact_samples else None),
            "extra_close_f2": (abs(self.stop["fingers"][1] - self.contact_samples[0][1])
                               if self.stop and self.contact_samples else None),
        }
        os.makedirs(OUTDIR, exist_ok=True)
        with open(os.path.join(OUTDIR, f"contact_stop_{TAG}.json"), "w") as fh:
            json.dump(rec, fh, indent=2)
        with open(os.path.join(OUTDIR, f"contact_stop_{TAG}.csv"), "w", newline="") as fh:
            w = csv.writer(fh); w.writerow(["t_rel", "f1", "f2"]); w.writerows(self.rows)
        print(json.dumps(rec, indent=2))
        self.get_logger().info(f"WROTE contact_stop_{TAG}.json")

def main():
    rclpy.init(); n = Probe()
    try: n.run()
    finally: n.destroy_node(); rclpy.shutdown()

if __name__ == "__main__":
    main()
