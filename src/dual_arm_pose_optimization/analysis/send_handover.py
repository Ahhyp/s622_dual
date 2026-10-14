#!/usr/bin/env python3
"""C4.2 -> simulation bridge (step 2 of 2).

Ships the joint target produced by `c42_handover_target` to the two arm
JointTrajectoryControllers in Gazebo, waits, then measures what the simulator
actually achieved and compares it against the C4.2 prediction.

Run under the JTC profile (the QP profile has no arm JTCs):

    ros2 launch gz_launch s622_dual_arm.launch.py gui:=false

    /usr/bin/python3 analysis/send_handover.py --target /tmp/handover_target.json

Reported:
  * per-joint commanded vs achieved (settling error)
  * achieved object pose vs the C4.2-optimised pose
  * achieved arm-arm clearance vs C4.2's predicted value
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class HandoverSender(Node):
    def __init__(self, target: dict, move_s: float, settle_s: float):
        super().__init__("c42_handover_sender")
        self.target = target
        self.move_s = move_s
        self.settle_s = settle_s
        self.latest: dict[str, float] = {}
        self.create_subscription(JointState, "/joint_states", self._on_js, 50)
        self.arm_clients = {
            "left": ActionClient(self, FollowJointTrajectory,
                                 "/left_arm_controller/follow_joint_trajectory"),
            "right": ActionClient(self, FollowJointTrajectory,
                                  "/right_arm_controller/follow_joint_trajectory"),
        }

    def _on_js(self, msg: JointState) -> None:
        for n, p in zip(msg.name, msg.position):
            self.latest[n] = p

    def spin_for(self, seconds: float) -> None:
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def send(self, side: str, joints: list[str], goal_pos: list[float]) -> bool:
        cli = self.arm_clients[side]
        if not cli.wait_for_server(timeout_sec=20.0):
            self.get_logger().error(f"{side}: action server not available")
            return False
        traj = JointTrajectory()
        traj.joint_names = joints
        pt = JointTrajectoryPoint()
        pt.positions = goal_pos
        secs = int(self.move_s)
        pt.time_from_start = Duration(sec=secs,
                                      nanosec=int((self.move_s - secs) * 1e9))
        traj.points.append(pt)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
        fut = cli.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=20.0)
        handle = fut.result()
        if handle is None or not handle.accepted:
            self.get_logger().error(f"{side}: goal rejected")
            return False
        res_fut = handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_fut, timeout_sec=self.move_s + 60.0)
        res = res_fut.result()
        code = res.result.error_code if res is not None else -999
        self.get_logger().info(f"{side}: trajectory finished, error_code={code}")
        return code == FollowJointTrajectory.Result.SUCCESSFUL

    def settle(self) -> None:
        self.spin_for(self.settle_s)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--target", required=True)
    ap.add_argument("--move-s", type=float, default=6.0)
    ap.add_argument("--settle-s", type=float, default=3.0)
    args = ap.parse_args()

    tgt = json.load(open(args.target))
    ql, qr = tgt["q_left"], tgt["q_right"]
    p_O = tgt["p_O"]

    rclpy.init()
    node = HandoverSender(tgt, args.move_s, args.settle_s)
    node.get_logger().info("waiting for /joint_states ...")
    node.spin_for(3.0)
    if not node.latest:
        node.get_logger().error("no /joint_states; is the sim up with the JTC profile?")
        return 1

    L = [f"left_j{i}" for i in range(1, 7)]
    R = [f"right_j{i}" for i in range(1, 7)]
    print("\n--- commanded (C4.2) vs current ---")
    print(f"{'joint':10s} {'current':>10s} {'target':>10s} {'delta':>10s}")
    for j, t in list(zip(L, ql)) + list(zip(R, qr)):
        cur = node.latest.get(j, float("nan"))
        print(f"{j:10s} {cur:10.5f} {t:10.5f} {t - cur:10.5f}")

    ok_l = node.send("left", L, ql)
    ok_r = node.send("right", R, qr)
    node.settle()

    print("\n--- achieved vs commanded ---")
    print(f"{'joint':10s} {'achieved':>10s} {'cmd':>10s} {'err(rad)':>10s} {'err(deg)':>10s}")
    max_err = 0.0
    for j, t in list(zip(L, ql)) + list(zip(R, qr)):
        a = node.latest.get(j, float("nan"))
        e = a - t
        max_err = max(max_err, abs(e))
        print(f"{j:10s} {a:10.5f} {t:10.5f} {e:10.5f} {math.degrees(e):10.4f}")

    print(f"\nmax |joint error| = {max_err:.5f} rad = {math.degrees(max_err):.4f} deg")
    print(f"trajectory action: left={ok_l} right={ok_r}")

    out = {
        "target_p_O": p_O,
        "commanded_q": list(ql) + list(qr),
        "achieved_q": [node.latest.get(j, float("nan"))
                       for j in L + R],
        "max_joint_err_rad": max_err,
        "left_action_ok": bool(ok_l),
        "right_action_ok": bool(ok_r),
        "c42_predicted": tgt["metrics"],
        "c42_verification": tgt["verification"],
    }
    json.dump(out, open("/tmp/handover_achieved.json", "w"), indent=2)
    print("wrote /tmp/handover_achieved.json")
    node.destroy_node()
    rclpy.shutdown()
    return 0 if (ok_l and ok_r and max_err < 0.02) else 1


if __name__ == "__main__":
    sys.exit(main())
