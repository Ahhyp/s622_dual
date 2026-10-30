import time, json
import rclpy
from rclpy.node import Node
from dual_arm_qp_controller.msg import TransportStatus
class N(Node):
    def __init__(self):
        super().__init__("lim_watch"); self.st=None
        self.create_subscription(TransportStatus, "/dual_arm_qp_controller/transport_status", self.cb, 10)
    def cb(self,m): self.st=m
rclpy.init(); n=N(); t0=time.time(); out=[]
while time.time()-t0 < 45:
    rclpy.spin_once(n, timeout_sec=0.1)
    m=n.st
    if m is None: continue
    out.append((round(time.time()-t0,2), int(m.phase_index), round(m.t_traj,2), m.state, m.active_limit_count,
                round(m.min_limit_margin_rad,3), round(m.e_obj_pos_m,6)))
json.dump(out, open("/tmp/w7b3/lim_series.json","w"))
import statistics
print("samples", len(out))
for r in out[::max(1,len(out)//14)]:
    print("t=%5.1f idx=%d t_traj=%5.2f %-8s lim=%2d margin=%.3f e_obj=%.2e" % r)
