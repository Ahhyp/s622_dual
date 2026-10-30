#!/usr/bin/env python3
"""T2 driver: run the executor-driven GRASP->LIFT and record the three criteria.

Records:
  * controller transport_status (state/phase/t_traj/limits/e_rel/e_obj)
  * /c43/weld_state (attached / detached)
  * sample z from /c43/weld_pose (TFMessage, child_frame_id == c43_sample::base_link)

Usage: probe_t2.py OUT.json [PHASES.json]
"""
import json, sys, time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage
from dual_arm_qp_controller.msg import TransportStatus
from dual_arm_qp_controller.srv import StartTransport

DEFAULT_PHASES = [
    ("approach", 3.0, 0.0, 0.0, 0.0, 0.0),  # hold at q_grasp12; original +0.02 lifted the pads off the sample
    ("grasp",    2.0, 0.0, 0.0, 0.0, 0.0),
    ("lift",     3.0, 0.0, 0.0, 0.02, 0.0),
    ("transport",3.0, -0.02, 0.0, 0.0, 0.0),
    ("lower",    3.0, 0.0, 0.0, -0.02, 0.0),
    ("release",  3.5, 0.0, 0.0, 0.0, 0.0),  # stationary hold: detach + 100 ms + open
    ("retreat",  2.0, 0.0, 0.0, 0.02, 0.0),  # back away after the fingers open
]

class N(Node):
    def __init__(self):
        super().__init__("t2_driver")
        self.st = None
        self.weld = None
        self.sample_z = None
        self.create_subscription(TransportStatus, "/dual_arm_qp_controller/transport_status", self.cb_st, 10)
        self.create_subscription(String, "/c43/weld_state", self.cb_weld, 10)
        self.create_subscription(TFMessage, "/c43/weld_pose", self.cb_pose, 10)

    def cb_st(self, m): self.st = m
    def cb_weld(self, m): self.weld = m.data
    def cb_pose(self, m):
        for t in m.transforms:
            if t.child_frame_id == "c43_sample":
                self.sample_z = t.transform.translation.z

def call(node, name):
    c = node.create_client(Trigger, name)
    if not c.wait_for_service(timeout_sec=5):
        return None, "no service %s" % name
    f = c.call_async(Trigger.Request())
    rclpy.spin_until_future_complete(node, f, timeout_sec=10)
    r = f.result()
    return r.success, r.message

def main(out_path, phases):
    rclpy.init()
    n = N()
    t0 = time.time()
    while time.time() - t0 < 25 and n.st is None:
        rclpy.spin_once(n, timeout_sec=0.1)
    if n.st is None:
        print("no controller status"); return 2

    if n.st.state != "IDLE":
        print("controller is", n.st.state, "-- abort+reset")
        print(" abort:", call(n, "/dual_arm_qp_controller/abort_transport"))
        print(" reset:", call(n, "/dual_arm_qp_controller/reset_transport"))
        time.sleep(0.5)

    cli = n.create_client(StartTransport, "/dual_arm_qp_controller/start_transport")
    ex = n.create_client(Trigger, "/c43_s4_transport_executor/start")
    cli.wait_for_service(timeout_sec=10)
    ex.wait_for_service(timeout_sec=10)

    req = StartTransport.Request()
    req.phase_name = [p[0] for p in phases]
    req.phase_duration = [p[1] for p in phases]
    req.phase_dx = [p[2] for p in phases]
    req.phase_dy = [p[3] for p in phases]
    req.phase_dz = [p[4] for p in phases]
    req.yaw_sweep_deg = [0.0]  # 1 entry = no yaw excitation (matches run_flow.py)
    req.dt = 0.008
    req.time_scaling = "linear"
    # Order matters: the executor's start precheck rejects an already-active
    # transport, so it must be armed while the controller is still IDLE; the
    # controller then starts within one 8 ms cycle.
    ok, msg = call(n, "/c43_s4_transport_executor/start")
    print("executor start:", ok, "|", msg)
    if not ok:
        with open(out_path, "w") as fh:
            json.dump({"phases": phases, "executor_start": [ok, msg], "rows": []}, fh, indent=1)
        return 4
    f = cli.call_async(req)
    rclpy.spin_until_future_complete(n, f, timeout_sec=10)
    print("start_transport:", f.result().accepted, "|", f.result().message)
    if not f.result().accepted:
        call(n, "/c43_s4_transport_executor/abort")
        with open(out_path, "w") as fh:
            json.dump({"phases": phases, "start_message": f.result().message, "rows": []}, fh, indent=1)
        return 4

    rows = []
    t1 = time.time()
    last = None
    while time.time() - t1 < 50.0:
        rclpy.spin_once(n, timeout_sec=0.05)
        m = n.st
        if m is None:
            continue
        row = {
            "t": round(time.time() - t1, 3), "state": m.state, "phase": int(m.phase_index),
            "phase_name": m.phase_name, "t_traj": round(m.t_traj, 3),
            "active_limit_count": int(m.active_limit_count),
            "q_state": [round(x, 5) for x in m.q_state],
            "e_obj_pos_m": m.e_obj_pos_m, "e_obj_rot_rad": m.e_obj_rot_rad,
            "weld": n.weld, "sample_z": None if n.sample_z is None else round(n.sample_z, 6),
            "failure_code": m.failure_code,
        }
        key = (row["state"], row["phase"], row["active_limit_count"], row["weld"],
               row["sample_z"], row["failure_code"])
        if key != last:
            rows.append(row)
            print("t=%5.2f %-9s phase=%-9s t_traj=%5.2f lim=%2d weld=%-12s z=%s fail=%s" % (
                row["t"], row["state"], row["phase_name"], row["t_traj"],
                row["active_limit_count"], str(row["weld"]),
                "?" if row["sample_z"] is None else "%.4f" % row["sample_z"],
                row["failure_code"] or "-"))
            last = key
        if row["state"] in ("HOLD", "FAULT", "ABORT") and time.time() - t1 > 5.0:
            break

    out = {"phases": phases, "rows": rows, "start_message": f.result().message,
           "executor_start": [ok, msg]}
    with open(out_path, "w") as fh:
        json.dump(out, fh, indent=1)
    print("wrote", out_path, "transitions", len(rows))
    return 0

if __name__ == "__main__":
    out = sys.argv[1] if len(sys.argv) > 1 else "/tmp/t2.json"
    phases = DEFAULT_PHASES
    if len(sys.argv) > 2:
        phases = [tuple(p) for p in json.load(open(sys.argv[2]))]
    sys.exit(main(out, phases))
