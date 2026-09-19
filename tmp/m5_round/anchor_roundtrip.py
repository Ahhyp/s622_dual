#!/usr/bin/env python3
"""M5 Anchor 冻结验证（规划 §7）：Joint-space Round Trip × N，统计规划成功率与重复性。

机制与 M5 运行时一致（照 bt 的 PlanToJoints）：
  规划 /move_group_fairino/plan_kinematic_path   (ompl / RRTConnect, group right_arm)
  执行 /right_arm_controller/follow_joint_trajectory

对每个 Anchor：
  参考位 → Anchor → 参考位   算一次 round trip
  记录：规划是否成功、执行是否成功、到达关节与指令的误差、多次到达的散布、
        以及 FK 出的 grasp_frame 位姿散布（这才是对任务有意义的重复性）

用法: anchor_roundtrip.py <n_rounds> [anchor 名, 逗号分隔]
"""
import sys
import time

import numpy as np
import rclpy
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.msg import Constraints, JointConstraint
from moveit_msgs.srv import GetMotionPlan, GetPositionFK
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory

R6 = [f"right_j{i}" for i in range(1, 7)]
REF = [0.2856, -1.2598, -1.7115, -3.3161, -1.2915, -1.5708]   # M4 R_SOLVED（参考位）
ANCHORS = {
    "place":       [1.9919, -1.0456, 1.8560, -0.9782, 0.4309, -1.3972],
    "place_pre":   [1.9919, -1.1663, 1.7845, -0.7864, 0.4310, -1.3967],
    "regrasp":     [-0.1815, -1.9266, -2.1485, -2.1371, -1.7473, -1.5373],
    "regrasp_pre": [-0.2375, -1.7870, -2.3669, -2.0575, -1.8031, -1.5333],
}
V_SCALE = 0.15
PLAN_TIME = 12.0
ATTEMPTS = 20

N = int(sys.argv[1]) if len(sys.argv) > 1 else 1
WHICH = sys.argv[2].split(",") if len(sys.argv) > 2 else list(ANCHORS)

rclpy.init()
n = Node("m5_roundtrip")
plan_cli = n.create_client(GetMotionPlan, "/move_group_fairino/plan_kinematic_path")
fk_cli = n.create_client(GetPositionFK, "/move_group_fairino/compute_fk")
arm_ac = ActionClient(n, FollowJointTrajectory, "/right_arm_controller/follow_joint_trajectory")
for c, nm in ((plan_cli, "plan_kinematic_path"), (fk_cli, "compute_fk")):
    if not c.wait_for_service(timeout_sec=20):
        print(f"缺服务 {nm}")
        sys.exit(1)
if not arm_ac.wait_for_server(timeout_sec=20):
    print("缺 /right_arm_controller/follow_joint_trajectory")
    sys.exit(1)

_js = {"q": None}


def _on_js(m):
    m2 = dict(zip(m.name, m.position))
    if all(k in m2 for k in R6):
        _js["q"] = [m2[k] for k in R6]


n.create_subscription(JointState, "/joint_states", _on_js, 10)
t0 = time.time()
while _js["q"] is None and time.time() - t0 < 20:
    rclpy.spin_once(n, timeout_sec=0.2)
if _js["q"] is None:
    print("/joint_states 无右臂关节")
    sys.exit(1)


def fk(q):
    r = GetPositionFK.Request()
    r.header.frame_id = "world"
    r.header.stamp = n.get_clock().now().to_msg()
    r.fk_link_names = ["right_grasp_frame"]
    r.robot_state.joint_state.name = list(R6)
    r.robot_state.joint_state.position = [float(x) for x in q]
    f = fk_cli.call_async(r)
    rclpy.spin_until_future_complete(n, f, timeout_sec=20)
    res = f.result()
    if res is None or res.error_code.val != 1:
        return None
    p = res.pose_stamped[0].pose
    return np.array([p.position.x, p.position.y, p.position.z])


def goto(q_target, tag=""):
    """规划+执行到 q_target；返回 (规划OK, 执行OK, 到达关节, 耗时s)"""
    t = time.time()
    # 等 joint_states 新一帧，确保 start_state 是当前真实状态
    for _ in range(5):
        rclpy.spin_once(n, timeout_sec=0.1)
    q_now = list(_js["q"])

    req = GetMotionPlan.Request()
    mpr = req.motion_plan_request
    mpr.group_name = "right_arm"
    mpr.planner_id = "RRTConnect"
    mpr.pipeline_id = "ompl"
    mpr.num_planning_attempts = ATTEMPTS
    mpr.allowed_planning_time = PLAN_TIME
    mpr.max_velocity_scaling_factor = V_SCALE
    mpr.max_acceleration_scaling_factor = V_SCALE
    mpr.start_state.joint_state.name = list(R6)
    mpr.start_state.joint_state.position = [float(x) for x in q_now]
    mpr.start_state.is_diff = False
    cs = []
    for nm_, v in zip(R6, q_target):
        jc = JointConstraint()
        jc.joint_name = nm_
        jc.position = float(v)
        jc.tolerance_above = 1e-4
        jc.tolerance_below = 1e-4
        jc.weight = 1.0
        cs.append(jc)
    cc = Constraints()
    cc.joint_constraints = cs
    mpr.goal_constraints.append(cc)

    f = plan_cli.call_async(req)
    rclpy.spin_until_future_complete(n, f, timeout_sec=60)
    res = f.result()
    if res is None or res.motion_plan_response.error_code.val != 1:
        code = None if res is None else res.motion_plan_response.error_code.val
        return False, False, None, time.time() - t, code

    traj = res.motion_plan_response.trajectory.joint_trajectory
    if not traj.points:
        return True, False, None, time.time() - t, "empty_traj"

    goal = FollowJointTrajectory.Goal()
    goal.trajectory = traj
    gh = arm_ac.send_goal_async(goal)
    rclpy.spin_until_future_complete(n, gh, timeout_sec=30)
    if not gh.done() or gh.result() is None or not gh.result().accepted:
        return True, False, None, time.time() - t, "rejected"
    rf = gh.result().get_result_async()
    # 执行超时按墙钟给足（sim RTF≈0.2，仿真时长 ×5）
    rclpy.spin_until_future_complete(n, rf, timeout_sec=180)
    if not rf.done() or rf.result() is None:
        return True, False, None, time.time() - t, "exec_timeout"
    ok = rf.result().result.error_code == 0
    # 停止后读实际关节
    time.sleep(1.0)
    for _ in range(10):
        rclpy.spin_once(n, timeout_sec=0.1)
    return True, ok, list(_js["q"]), time.time() - t, None


print(f"Round Trip × {N}   参考位 = R_SOLVED   速度比例 = {V_SCALE}")
print(f"Anchor: {WHICH}\n")
for aname in WHICH:
    q_t = ANCHORS[aname]
    tgt_pose = fk(q_t)
    print(f"===== {aname} =====")
    print(f"  指令关节 = [{', '.join(f'{v:+.4f}' for v in q_t)}]")
    print(f"  指令 grasp 位姿 = {np.round(tgt_pose, 4)}")
    np_ok = ex_ok = 0
    arrived = []
    poses = []
    tcost = []
    fails = []
    for i in range(N):
        p1, e1, qa, dt1, err1 = goto(q_t, f"{aname}[{i}]")
        if p1 and e1 and qa:
            np_ok += 1
            ex_ok += 1
            arrived.append(qa)
            fp = fk(qa)
            if fp is not None:
                poses.append(fp)
            tcost.append(dt1)
        else:
            fails.append(f"->anchor: plan={p1} exec={e1} err={err1}")
            np_ok += 1 if p1 else 0
        # 回参考位
        p2, e2, _, dt2, err2 = goto(REF, f"{aname}[{i}]ret")
        if p2 and e2:
            tcost.append(dt2)
        else:
            fails.append(f"->ref: plan={p2} exec={e2} err={err2}")
    print(f"  规划成功 {np_ok}/{N}   执行成功 {ex_ok}/{N}", end="")
    if tcost:
        print(f"   单程 {np.mean(tcost):.1f}s (max {max(tcost):.1f}s)")
    else:
        print()
    if arrived:
        A = np.array(arrived)
        cmd = np.array(q_t)
        err = np.abs(A - cmd).max(axis=1) * 1000
        print(f"  到达关节 vs 指令: 最大 {err.max():.2f} mrad, 均值 {err.mean():.2f} mrad")
        spread = (A.max(axis=0) - A.min(axis=0)) * 1000
        print(f"  多次到达散布(峰峰): max {spread.max():.2f} mrad"
              f"  [{', '.join(f'{v:.1f}' for v in spread)}]")
        if poses:
            P = np.array(poses)
            dp = (P.max(axis=0) - P.min(axis=0)) * 1000
            print(f"  grasp 位姿散布(峰峰): {np.round(dp, 3)} mm"
                  f"   (对指令 {np.round(np.abs(P - tgt_pose).max(axis=0) * 1000, 3)} mm)")
    if fails:
        print(f"  失败 {len(fails)} 次: {fails[:5]}")
    print()

n.destroy_node()
rclpy.shutdown()
