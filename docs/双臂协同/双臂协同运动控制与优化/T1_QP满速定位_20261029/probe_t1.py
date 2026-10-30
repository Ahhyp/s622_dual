#!/usr/bin/env python3
"""T1 probe: capture q_state / e_rel / dq at IDLE and during the first approach cycles.

Read-only with respect to the controller: it only calls start_transport and
subscribes to transport_status.  No thresholds are changed here.
"""
import json, sys, time
import rclpy
from rclpy.node import Node
from dual_arm_qp_controller.msg import TransportStatus
from dual_arm_qp_controller.srv import StartTransport

PHASES = [
    ("approach", 3.0, 0.0, 0.0, 0.02, 0.0),
    ("grasp",    2.0, 0.0, 0.0, 0.0, 0.0),
    ("lift",     3.0, 0.0, 0.0, 0.02, 0.0),
    ("transport",3.0, -0.02, 0.0, 0.0, 0.0),
    ("lower",    3.0, 0.0, 0.0, -0.02, 0.0),
]

class N(Node):
    def __init__(self):
        super().__init__("t1_probe")
        self.st = None
        self.create_subscription(TransportStatus, "/dual_arm_qp_controller/transport_status", self.cb, 10)
    def cb(self, m):
        self.st = m

def snap(m, t):
    return {
        "t": round(t, 3), "state": m.state, "active": bool(m.active),
        "phase": int(m.phase_index), "phase_name": m.phase_name,
        "t_traj": round(m.t_traj, 3),
        "active_limit_count": int(m.active_limit_count),
        "min_limit_margin_rad": m.min_limit_margin_rad,
        "e_rel_pos_m": m.e_rel_pos_m, "e_rel_rot_rad": m.e_rel_rot_rad,
        "e_obj_pos_m": m.e_obj_pos_m, "e_obj_rot_rad": m.e_obj_rot_rad,
        "q_state": [round(x, 6) for x in m.q_state],
        "q_cmd": [round(x, 6) for x in m.q_cmd],
        "dq": [round(x, 6) for x in m.dq],
        "max_abs_dq": max(abs(x) for x in m.dq) if len(m.dq) == 12 else None,
    }

def main(out_path):
    rclpy.init()
    n = N()
    t0 = time.time()
    while time.time() - t0 < 20 and n.st is None:
        rclpy.spin_once(n, timeout_sec=0.1)
    if n.st is None:
        print("no transport_status within 20 s"); return 2
    idle = snap(n.st, time.time() - t0)
    print("IDLE state=%s lim=%d e_rel=%.6f m / %.6f rad e_obj=%.3e / %.3e" % (
        idle["state"], idle["active_limit_count"], idle["e_rel_pos_m"], idle["e_rel_rot_rad"],
        idle["e_obj_pos_m"], idle["e_obj_rot_rad"]))
    print("q_state:", idle["q_state"])

    cli = n.create_client(StartTransport, "/dual_arm_qp_controller/start_transport")
    if not cli.wait_for_service(timeout_sec=15):
        print("start_transport service not available"); return 3
    req = StartTransport.Request()
    req.phase_name = [p[0] for p in PHASES]
    req.phase_duration = [p[1] for p in PHASES]
    req.phase_dx = [p[2] for p in PHASES]
    req.phase_dy = [p[3] for p in PHASES]
    req.phase_dz = [p[4] for p in PHASES]
    req.yaw_sweep_deg = [0.0]
    req.dt = 0.008
    req.time_scaling = "linear"
    fut = cli.call_async(req)
    rclpy.spin_until_future_complete(n, fut, timeout_sec=10)
    res = fut.result()
    print("start_transport accepted:", res.accepted, "|", res.message)

    rows = []
    t1 = time.time()
    while time.time() - t1 < 3.0:
        rclpy.spin_once(n, timeout_sec=0.05)
        if n.st is None:
            continue
        s = snap(n.st, time.time() - t1)
        s["start_message"] = res.message
        rows.append(s)
        if len(rows) <= 12 or len(rows) % 5 == 0:
            print("t=%5.2f %-8s t_traj=%5.2f lim=%2d e_rel=%.4f/%.4f e_obj=%.3e max|dq|=%.5f" % (
                s["t"], s["state"], s["t_traj"], s["active_limit_count"],
                s["e_rel_pos_m"], s["e_rel_rot_rad"], s["e_obj_pos_m"],
                s["max_abs_dq"] if s["max_abs_dq"] is not None else -1))
    out = {"idle": idle, "start_accepted": bool(res.accepted), "start_message": res.message,
           "rows": rows}
    with open(out_path, "w") as f:
        json.dump(out, f, indent=1)
    print("wrote", out_path, "rows", len(rows))
    return 0

if __name__ == "__main__":
    sys.exit(main(sys.argv[1] if len(sys.argv) > 1 else "/tmp/t1_probe.json"))
