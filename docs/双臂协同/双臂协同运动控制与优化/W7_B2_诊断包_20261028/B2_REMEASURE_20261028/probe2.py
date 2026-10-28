"""Joint-motion probe: max displacement over the window, not end-minus-end.

The previous probe computed `js_end[k] - n.js[k]` where `js_end` was a copy of the
same dict, so the "delta" was identically zero whatever the robot did.  This one
keeps every sample and measures min/max per joint across the window, and it also
compares the controller's own q_state/q_cmd with /joint_states.
"""
import json, sys, time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from dual_arm_qp_controller.srv import StartTransport
from sensor_msgs.msg import JointState
from dual_arm_qp_controller.msg import TransportStatus

NAMES = [f"{s}_{j}" for s in ("left", "right") for j in ("j1", "j2", "j3", "j4", "j5", "j6")]

def window_stats(series):
    """series: list of (t, {joint: value}); returns per-joint {min,max,span,final}."""
    out = {n: {"min": None, "max": None, "span": 0.0, "first": None, "final": None} for n in NAMES}
    for _, s in series:
        for n in NAMES:
            v = s.get(n)
            if v is None:
                continue
            d = out[n]
            if d["first"] is None:
                d["first"] = v
            d["final"] = v
            d["min"] = v if d["min"] is None else min(d["min"], v)
            d["max"] = v if d["max"] is None else max(d["max"], v)
            d["span"] = d["max"] - d["min"]
    return out

class D(Node):
    def __init__(self):
        super().__init__("w7_b2_probe")
        self.js_series = []
        self.st_series = []
        self.st = None
        self.create_subscription(TransportStatus, "/dual_arm_qp_controller/transport_status",
                                 self.on_st, 10)
        self.create_subscription(JointState, "/joint_states", self.on_js,
                                 QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.start = self.create_client(StartTransport, "/dual_arm_qp_controller/start_transport")

    def on_st(self, m):
        self.st = m
        q_state = {n: float(m.q_state[i]) for i, n in enumerate(NAMES)}
        q_cmd = {n: float(m.q_cmd[i]) for i, n in enumerate(NAMES)}
        self.st_series.append((time.time(), q_state, q_cmd))

    def on_js(self, m):
        d = {n: float(p) for n, p in zip(m.name, m.position) if n in NAMES}
        if d:
            self.js_series.append((time.time(), d))

def main():
    rclpy.init()
    n = D()
    t0 = time.time()
    while time.time() - t0 < 40 and (n.st is None or not n.js_series):
        rclpy.spin_once(n, timeout_sec=0.1)
    if n.st is None or not n.js_series:
        print("NO_DATA", flush=True); return 2
    while not n.start.wait_for_service(timeout_sec=1.0):
        pass
    # window starts exactly at the request
    js_mark, st_mark = len(n.js_series), len(n.st_series)
    req = StartTransport.Request()
    req.phase_name = ["approach", "lift", "transport", "lower"]
    req.phase_duration = [0.5, 0.5, 0.5, 0.5]
    req.phase_dx = [0.0, 0.0, 0.06, 0.0]
    req.phase_dy = [0.0, 0.0, 0.0, 0.0]
    req.phase_dz = [0.02, 0.05, 0.0, -0.05]
    req.yaw_sweep_deg = [0.0, 0.0, 0.0, 0.0, 0.0]
    req.dt = 0.008
    fut = n.start.call_async(req)
    rclpy.spin_until_future_complete(n, fut, timeout_sec=5.0)
    print("start_accepted", fut.result().accepted, "|", fut.result().message, flush=True)
    t1 = time.time()
    while time.time() - t1 < 4.0:
        rclpy.spin_once(n, timeout_sec=0.02)

    js = window_stats(n.js_series[js_mark:])
    qs = window_stats([(t, s) for t, s, _ in n.st_series[st_mark:]])
    qc = window_stats([(t, c) for t, _, c in n.st_series[st_mark:]])
    print("samples js=%d status=%d" % (len(n.js_series[js_mark:]), len(n.st_series[st_mark:])), flush=True)
    print("JS_SPAN_MAX %.6f" % max(v["span"] for v in js.values()), flush=True)
    print("QSTATE_SPAN_MAX %.6f" % max(v["span"] for v in qs.values()), flush=True)
    print("QCMD_SPAN_MAX %.6f" % max(v["span"] for v in qc.values()), flush=True)
    print("JS_TRACKS_QSTATE_MAX_ERR %.6f" % max(
        abs(js[k]["final"] - qs[k]["final"]) for k in NAMES), flush=True)
    print("js_spans", json.dumps({k: round(v["span"], 6) for k, v in js.items()}), flush=True)
    print("qstate_spans", json.dumps({k: round(v["span"], 6) for k, v in qs.items()}), flush=True)
    print("qcmd_spans", json.dumps({k: round(max(v["span"], -v["span"]), 6) for k, v in qc.items()}), flush=True)
    print("qcmd_final_minus_qstate", json.dumps(
        {k: round(qc[k]["final"] - qs[k]["final"], 6) for k in NAMES}), flush=True)
    print("status_final", n.st.state, "t_traj", round(n.st.t_traj, 3), flush=True)
    rclpy.shutdown()
    return 0

if __name__ == "__main__":
    sys.exit(main())
