import json, sys, time
import rclpy
from rclpy.node import Node
from dual_arm_qp_controller.srv import StartTransport
from dual_arm_qp_controller.msg import TransportStatus
from std_srvs.srv import Trigger

PHASES = [
    ("approach", 3.0, 0.0, 0.0, 0.02, 0.0),
    ("grasp",    2.0, 0.0, 0.0, 0.0, 0.0),
    ("lift",     3.0, 0.0, 0.0, 0.02, 0.0),
    ("transport",3.0, -0.02, 0.0, 0.0, 0.0),
    ("lower",    3.0, 0.0, 0.0, -0.02, 0.0),
]

class N(Node):
    def __init__(self):
        super().__init__("w7b2_flow")
        self.st = None
        self.trail = []
        self.create_subscription(TransportStatus, "/dual_arm_qp_controller/transport_status", self.cb, 10)
    def cb(self, m):
        self.st = m

def main():
    rclpy.init()
    n = N()
    t0 = time.time()
    while time.time() - t0 < 30 and n.st is None:
        rclpy.spin_once(n, timeout_sec=0.1)
    if n.st is None:
        print("no controller status"); return 2
    print("controller:", n.st.state, "active:", n.st.active, "q12 ok:", len(n.st.q_state))
    cli = n.create_client(StartTransport, "/dual_arm_qp_controller/start_transport")
    ex = n.create_client(Trigger, "/c43_s4_transport_executor/start")
    cli.wait_for_service(timeout_sec=15)
    print("executor service visible:", ex.wait_for_service(timeout_sec=15))
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
    print("start_transport accepted:", fut.result().accepted, "|", fut.result().message)
    f2 = ex.call_async(Trigger.Request())
    rclpy.spin_until_future_complete(n, f2, timeout_sec=10)
    print("executor start:", f2.result().success, "|", f2.result().message)
    t0 = time.time()
    while time.time() - t0 < 26:
        rclpy.spin_once(n, timeout_sec=0.1)
        m = n.st
        if m is None: continue
        row = (round(time.time()-t0, 2), m.active, int(m.phase_index), m.phase_name,
               round(m.t_traj, 2), m.state, f"lim={m.active_limit_count} margin={m.min_limit_margin_rad:.4f}")
        if not n.trail or n.trail[-1][1:] != row[1:]:
            n.trail.append(row)
    for r in n.trail:
        print("t=%5.2f active=%s idx=%2d name=%-9s t_traj=%5.2f state=%-8s %s" % r)
    json.dump({"trail": n.trail}, open("/tmp/w7b3/flow_trail.json","w"), indent=1)
    return 0

sys.exit(main())
