#!/usr/bin/env python3
"""Experiment 2.5-A: direct left-hand JTC open/close, no sample."""
import csv, os, time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

OUT = os.environ.get("C43_GRIPPER_PROBE_OUT", "/workspace/docs/双臂协同/双臂协同运动控制与优化/T1_QP满速定位_20261029/gripper_2_5a.csv")
JOINTS = ["left_finger1_joint", "left_finger2_joint"]

class Probe(Node):
    def __init__(self):
        super().__init__("c43_gripper_probe")
        self.create_subscription(JointState, "/joint_states", self.on_js, 50)
        self.ac = ActionClient(self, FollowJointTrajectory,
                               "/left_hand_controller/follow_joint_trajectory")
        self.rows = []
        self.t0 = time.time()
        self.timer = self.create_timer(0.01, self.tick)

    def on_js(self, msg):
        d = dict(zip(msg.name, msg.position))
        if all(j in d for j in JOINTS):
            self.latest = (d[JOINTS[0]], d[JOINTS[1]])
        else:
            self.latest = None

    def tick(self):
        if not hasattr(self, "latest") or self.latest is None:
            return
        self.rows.append([time.time() - self.t0, self.latest[0], self.latest[1]])

    def send(self, pos, dur, label):
        goal = FollowJointTrajectory.Goal()
        tr = JointTrajectory(); tr.joint_names = JOINTS
        pt = JointTrajectoryPoint(); pt.positions = pos
        pt.time_from_start.sec = int(dur)
        pt.time_from_start.nanosec = int((dur - int(dur)) * 1e9)
        tr.points.append(pt); goal.trajectory = tr
        self.get_logger().info(f"send {label} pos={pos} dur={dur}")
        fut = self.ac.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=15.0)
        h = fut.result()
        if h is None or not h.accepted:
            self.get_logger().error(f"{label}: goal NOT accepted")
            return None
        rf = h.get_result_async()
        rclpy.spin_until_future_complete(self, rf, timeout_sec=dur + 15.0)
        r = rf.result()
        if r is None:
            self.get_logger().error(f"{label}: no result")
            return None
        res = r.result
        self.get_logger().info(
            f"{label}: status={r.status} error_code={res.error_code} "
            f"error_string='{res.error_string}'")
        for _ in range(int(1.5 / 0.01)):
            rclpy.spin_once(self, timeout_sec=0.01)
        return r

    def run(self):
        self.get_logger().info("waiting for left_hand_controller...")
        if not self.ac.wait_for_server(timeout_sec=30.0):
            raise RuntimeError("left_hand_controller action server not available")
        for _ in range(200):
            rclpy.spin_once(self, timeout_sec=0.01)
        self.get_logger().info(f"initial fingers={getattr(self,'latest',None)}")
        self.send([0.0305, -0.0305], 1.5, "OPEN_0.0305")
        if os.environ.get("C43_GRASP_PROBE_SLOW", "0") == "1":
            # 2.5-B2: slow close only, no 0.020 push, no attach.
            self.send([0.0251, -0.0251], 5.0, "SLOW_CLOSE_0.0251")
            for _ in range(200):
                rclpy.spin_once(self, timeout_sec=0.01)
        else:
            self.send([0.0251, -0.0251], 2.0, "CLOSE_0.0251")
            self.send([0.0200, -0.0200], 2.0, "CLOSE_0.0200")
            self.send([0.0305, -0.0305], 1.5, "REOPEN_0.0305")
        with open(OUT, "w", newline="") as f:
            w = csv.writer(f); w.writerow(["t_rel", "left_finger1", "left_finger2"])
            w.writerows(self.rows)
        self.get_logger().info(f"wrote {OUT}: {len(self.rows)} rows")

def main():
    rclpy.init(); n = Probe()
    try:
        n.run()
    finally:
        n.destroy_node(); rclpy.shutdown()

if __name__ == "__main__":
    main()
