import time, json, sys
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from dual_arm_qp_controller.msg import TransportStatus

class N(Node):
    def __init__(self):
        super().__init__("w7b3_resume")
        self.st = None
        self.create_subscription(TransportStatus, "/dual_arm_qp_controller/transport_status", self.cb, 10)
    def cb(self, m): self.st = m

rclpy.init(); n = N()
t0 = time.time()
while time.time()-t0 < 10 and n.st is None: rclpy.spin_once(n, timeout_sec=0.1)
print("before resume:", n.st.state, "t_traj", round(n.st.t_traj,2), "lim", n.st.active_limit_count)
for name in ("/dual_arm_qp_controller/resume_transport", "/c43_s4_transport_executor/resume"):
    c = n.create_client(Trigger, name)
    if not c.wait_for_service(timeout_sec=5):
        print("no service", name); continue
    f = c.call_async(Trigger.Request()); rclpy.spin_until_future_complete(n, f, timeout_sec=10)
    print(name, "->", f.result().success, f.result().message)
rows = []
t0 = time.time()
while time.time()-t0 < 22:
    rclpy.spin_once(n, timeout_sec=0.1)
    m = n.st
    if m is None: continue
    row = (round(time.time()-t0,2), m.active, int(m.phase_index), m.phase_name, round(m.t_traj,2), m.state,
           f"lim={m.active_limit_count}")
    if not rows or rows[-1][1:] != row[1:]:
        rows.append(row); print("t=%5.2f active=%s idx=%2d %-9s t=%5.2f %-8s %s" % row)
json.dump(rows, open("/tmp/w7b3/resume_trail.json","w"), indent=1)
