#!/usr/bin/env python3
"""C4.3 S4 W3-R1a T-H13: drive ONE 638-cycle transport round and dump timing.

usage: run_round.py <label> <params_file> <out_dir>

Drives the same scenario as config/c43_s4_transport.yaml (settle 0.30 +
3 x 1.50 + settle 0.30 = 5.10 s = 638 cycles @ 125 Hz) against
launch/c43_s4_mock.launch.py, publishes the ~/heartbeat at 50 Hz (E8 is on in
the bench profile), then calls ~/timing_dump and records the reply verbatim.

Writes <out_dir>/round.json with: the dump message, the final status snapshot
(state, q_cmd, t_traj, failure_code) and the wall clock of the run.
"""
import json
import os
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import Empty
from std_srvs.srv import Trigger

from dual_arm_qp_controller.msg import TransportStatus
from dual_arm_qp_controller.srv import StartTransport

PHASES = [
    ("settle", 0.30, 0.0, 0.0, 0.0),
    ("lift", 1.50, 0.0, 0.0, 0.050),
    ("translate", 1.50, -0.060, 0.0, 0.0),
    ("lower", 1.50, 0.0, 0.0, -0.050),
    ("settle2", 0.30, 0.0, 0.0, 0.0),
]
DT = 0.008


class Driver(Node):
    def __init__(self):
        super().__init__("th13_driver")
        self.status = None
        self.status_count = 0
        self.create_subscription(
            TransportStatus, "/dual_arm_qp_controller/transport_status", self.on_status, 10)
        self.hb = self.create_publisher(Empty, "/dual_arm_qp_controller/heartbeat", 10)
        self.create_timer(0.02, lambda: self.hb.publish(Empty()))
        self.start_cli = self.create_client(
            StartTransport, "/dual_arm_qp_controller/start_transport")
        self.dump_cli = self.create_client(Trigger, "/dual_arm_qp_controller/timing_dump")
        self.resume_cli = self.create_client(Trigger, "/dual_arm_qp_controller/resume")

    def on_status(self, msg):
        self.status = msg
        self.status_count += 1


def spin_until(node, exec_, pred, timeout_s):
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout_s:
        exec_.spin_once(timeout_sec=0.005)
        if pred():
            return True
    exec_.spin_once(timeout_sec=0.005)
    return pred()


def call(node, exec_, cli, req, timeout_s):
    if not cli.wait_for_service(timeout_sec=timeout_s):
        return None
    fut = cli.call_async(req)
    if not spin_until(node, exec_, fut.done, timeout_s):
        return None
    return fut.result()


def main():
    label, params_file, out_dir = sys.argv[1], sys.argv[2], sys.argv[3]
    os.makedirs(out_dir, exist_ok=True)
    rclpy.init()
    node = Driver()
    exec_ = SingleThreadedExecutor()
    exec_.add_node(node)

    rec = {"label": label, "params_file": params_file, "started_wall": time.strftime(
        "%Y-%m-%dT%H:%M:%S%z")}
    ok = spin_until(node, exec_, lambda: node.status is not None, 120.0)
    if not ok:
        rec["error"] = "controller never published transport_status"
        json.dump(rec, open(os.path.join(out_dir, "round.json"), "w"), indent=1)
        print(json.dumps(rec))
        return 1
    rec["controller_up_wall"] = time.strftime("%Y-%m-%dT%H:%M:%S%z")
    rec["state_at_start"] = node.status.state

    req = StartTransport.Request()
    req.dt = DT
    req.time_scaling = "smoothstep"
    req.yaw_sweep_deg = [0.0]
    for name, dur, dx, dy, dz in PHASES:
        req.phase_name.append(name)
        req.phase_duration.append(dur)
        req.phase_dx.append(dx)
        req.phase_dy.append(dy)
        req.phase_dz.append(dz)
    sres = call(node, exec_, node.start_cli, req, 30.0)
    if sres is None:
        rec["error"] = "start_transport timed out"
        json.dump(rec, open(os.path.join(out_dir, "round.json"), "w"), indent=1)
        print(json.dumps(rec))
        return 1
    rec["start_accepted"] = bool(sres.accepted)
    rec["start_message"] = sres.message
    if not sres.accepted:
        json.dump(rec, open(os.path.join(out_dir, "round.json"), "w"), indent=1)
        print(json.dumps(rec))
        return 1

    holds = 0
    t0 = time.monotonic()
    done = False
    while time.monotonic() - t0 < 90.0:
        exec_.spin_once(timeout_sec=0.005)
        st = node.status
        if st is None:
            continue
        if st.state in ("COMPLETE", "ABORT", "FAULT"):
            done = True
            break
        if st.state == "HOLD":
            holds += 1
            call(node, exec_, node.resume_cli, Trigger.Request(), 5.0)
    rec["holds"] = holds
    rec["finished_wall"] = time.strftime("%Y-%m-%dT%H:%M:%S%z")
    final = node.status
    rec["final_state"] = final.state if final else None
    rec["final_failure_code"] = final.failure_code if final else None
    rec["final_t_traj"] = final.t_traj if final else None
    rec["final_t_traj_duration"] = final.t_traj_duration if final else None
    rec["final_q_cmd"] = list(final.q_cmd) if final else None
    rec["final_q_state"] = list(final.q_state) if final else None
    rec["final_solve_fail_streak"] = int(final.solve_fail_streak) if final else None
    rec["status_msgs"] = node.status_count
    rec["completed"] = bool(done and final is not None and final.state == "COMPLETE")

    dres = call(node, exec_, node.dump_cli, Trigger.Request(), 30.0)
    rec["dump_success"] = bool(dres.success) if dres else None
    rec["dump_message"] = dres.message if dres else "dump call failed"
    rec["dump_wall"] = time.strftime("%Y-%m-%dT%H:%M:%S%z")

    json.dump(rec, open(os.path.join(out_dir, "round.json"), "w"), indent=1)
    print(json.dumps({k: rec[k] for k in
                      ("label", "completed", "final_state", "holds", "dump_success",
                       "dump_message")}, indent=1))
    exec_.shutdown()
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
