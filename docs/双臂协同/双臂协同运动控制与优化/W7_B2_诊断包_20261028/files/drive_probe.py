import json, time, sys
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rcl_interfaces.srv import GetParameters
from dual_arm_qp_controller.srv import StartTransport
from sensor_msgs.msg import JointState
from dual_arm_qp_controller.msg import TransportStatus

NAMES = [f"{s}_{j}" for s in ("left", "right") for j in ("j1","j2","j3","j4","j5","j6")]

class D(Node):
    def __init__(self):
        super().__init__("w7fix_driver")
        self.st = None
        self.js = None
        self.samples = []
        self.create_subscription(TransportStatus, "/dual_arm_qp_controller/transport_status", self.on_st, 10)
        self.create_subscription(JointState, "/joint_states", self.on_js, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.start = self.create_client(StartTransport, "/dual_arm_qp_controller/start_transport")
    def on_st(self, m):
        self.st = m
        self.samples.append((time.time(), list(m.q_cmd)))
    def on_js(self, m):
        d = {}
        for n, p in zip(m.name, m.position):
            d[n] = p
        self.js = d

def main():
    rclpy.init()
    n = D()
    t0 = time.time()
    while time.time() - t0 < 30 and (n.st is None or n.js is None):
        rclpy.spin_once(n, timeout_sec=0.1)
    print("have_status", n.st is not None, "have_js", n.js is not None, flush=True)
    if n.js:
        print("js_start", json.dumps({k: round(v, 6) for k, v in n.js.items() if k in NAMES}), flush=True)
    if n.st is None:
        return 2
    while not n.start.wait_for_service(timeout_sec=1.0):
        pass
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
    # sample joint states + q_cmd while the trajectory runs
    mark = len(n.samples)
    t1 = time.time()
    while time.time() - t1 < 4.0:
        rclpy.spin_once(n, timeout_sec=0.02)
    js_end = {k: v for k, v in (n.js or {}).items() if k in NAMES}
    print("js_end", json.dumps({k: round(v, 6) for k, v in js_end.items()}), flush=True)
    if n.js:
        d = {k: js_end[k] - n.js[k] for k in NAMES if k in js_end and k in n.js}
        print("js_delta", json.dumps({k: round(v, 6) for k, v in d.items()}), flush=True)
        print("js_maxabs_delta", max(abs(v) for v in d.values()) if d else None, flush=True)
    # q_cmd trail
    if n.samples:
        first = n.samples[0][1]
        last = n.samples[-1][1]
        dq = [ (None if (a is None or b is None) else b - a) for a, b in zip(first, last) ]
        print("qcmd_delta", json.dumps([None if v is None else round(v, 6) for v in dq]), flush=True)
        print("qcmd_maxabs_delta", max((abs(v) for v in dq if v is not None), default=None), flush=True)
    print("status_final", n.st.state, n.st.failure_code, "t_traj", round(n.st.t_traj, 3), flush=True)
    rclpy.shutdown()
    return 0

sys.exit(main())
