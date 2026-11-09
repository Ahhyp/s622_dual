#!/usr/bin/env python3
"""C4.3 S4 flow glue: start the executor, then latch the controller trajectory.

Why this exists
---------------
The executor's state machine is armed by its own `~/start` service and then
advances off the controller's phase stream.  Two separate `ros2 service call`
CLIs spend 1.5-2.5 s each just starting a node, which eats the executor's 5 s
APPROACH budget: the controller reaches t_traj == 2.4 s while the executor is
already at 5.0 s, and APPROACH latches FAULT(grasp_check_failed) even though the
grasp check is disabled.

Both calls therefore have to come from ONE node, back to back.

    ros2 run gz_launch c43_flow_start.py        # or: python3 c43_flow_start.py

Optional env:
    C43_FLOW_DX        horizontal transfer [m] (default 0.10)
    C43_FLOW_DZ_LIFT   lift height [m]         (default 0.05)
"""
import os
import sys

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from dual_arm_qp_controller.srv import StartTransport

EXECUTOR_SRV = "/c43_s4_transport_executor/start"
CONTROLLER_SRV = "/dual_arm_qp_controller/start_transport"

PHASES = ["approach", "grasp", "lift", "transport", "lower", "release", "retreat"]
DURATIONS = [3.0, 2.0, 3.0, 3.0, 3.0, 3.5, 2.0]


def main():
    dx = float(os.environ.get("C43_FLOW_DX", "0.10"))
    dz = float(os.environ.get("C43_FLOW_DZ_LIFT", "0.05"))
    dxs = [0.0, 0.0, 0.0, dx, 0.0, 0.0, 0.0]
    dzs = [0.0, 0.0, dz, 0.0, -dz, 0.0, 0.0]

    rclpy.init()
    node = Node("c43_flow_start")
    ex_cli = node.create_client(Trigger, EXECUTOR_SRV)
    qp_cli = node.create_client(StartTransport, CONTROLLER_SRV)

    for cli, name in ((ex_cli, EXECUTOR_SRV), (qp_cli, CONTROLLER_SRV)):
        if not cli.wait_for_service(timeout_sec=20.0):
            node.get_logger().error(f"{name} not available")
            return 2

    # 1) arm the executor state machine (IDLE -> APPROACH)
    fut = ex_cli.call_async(Trigger.Request())
    rclpy.spin_until_future_complete(node, fut, timeout_sec=10.0)
    res = fut.result()
    if res is None or not res.success:
        node.get_logger().error(
            f"executor start refused: {None if res is None else res.message}")
        return 3
    node.get_logger().info(f"executor armed: {res.message}")

    # 2) latch the trajectory immediately -- no CLI startup in between
    req = StartTransport.Request()
    req.phase_name = PHASES
    req.phase_duration = DURATIONS
    req.phase_dx = dxs
    req.phase_dy = [0.0] * len(PHASES)
    req.phase_dz = dzs
    req.start_world_t_object = []
    req.yaw_sweep_deg = []
    req.dt = 0.0
    req.time_scaling = "smoothstep"
    fut = qp_cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=10.0)
    res = fut.result()
    if res is None or not res.accepted:
        node.get_logger().error(
            f"start_transport refused: {None if res is None else res.message}")
        return 4
    node.get_logger().info(
        f"trajectory latched: dx={dx} dz={dz} -> {res.message}")
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
