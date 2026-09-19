#!/usr/bin/env python3
"""M5 夹具可达性验证：右臂能否把 grasp_frame 送到"杆放在夹具上"的抓取位姿。

目标位姿由 M4 已验证的抓取关系推出（不重新标定）：
  grasp_origin = rod_center + receive_offset
  M4 m4_handover.yaml: receive_offset = (-0.0086, -0.054, 0.0030)（world，杆心 -54mm = 交接抓取端）
  姿态沿用 M4 receive_quat（右爪接物唯一可用姿态）

夹具上杆的标称位姿（fixture 局部系原点在世界 (0.10, -0.20, 0)，杆轴 = 世界 Y、零滚转）：
  rod_center = (0.10, -0.20, 0.120)

用法: fixture_reach_check.py [rodx] [rody] [rodz]
"""
import sys

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from rclpy.node import Node
from tf_transformations import quaternion_matrix

ROD = np.array([float(a) for a in sys.argv[1:4]]) if len(sys.argv) > 3 \
    else np.array([0.10, -0.20, 0.120])

# M4 已验证量
RECEIVE_OFFSET = np.array([-0.0086, -0.054, 0.0030])   # grasp - rod（交接端, -54mm）
RECEIVE_QUAT = [0.0087, 0.6819, 0.0060, 0.7314]        # 世界系, 右爪接物姿态
R6 = [f"right_j{i}" for i in range(1, 7)]


def grasp_target(long_offset_mm, dz=0.0, dx=0.0):
    """long_offset_mm: 沿杆轴的抓取位置（相对杆心，mm；交接端 = -54）"""
    off = RECEIVE_OFFSET.copy()
    off[1] = long_offset_mm / 1000.0
    p = ROD + off + np.array([dx, 0.0, dz])
    return p


CASES = [
    ("place      杆在夹具,持-54端", grasp_target(-54)),
    ("place_pre  同上抬50mm", grasp_target(-54, dz=0.050)),
    ("regrasp    杆在夹具,抓+54端", grasp_target(+54)),
    ("regrasp_pre  沿-x退60mm", grasp_target(+54, dx=-0.060)),
]

rclpy.init()
n = Node("m5_reach")
ik_cli = n.create_client(GetPositionIK, "/move_group_kdl/compute_ik")
fk_cli = n.create_client(GetPositionFK, "/move_group_fairino/compute_fk")
for c, nm in ((ik_cli, "compute_ik"), (fk_cli, "compute_fk")):
    if not c.wait_for_service(timeout_sec=15):
        print(f"找不到服务 {nm}")
        sys.exit(1)


def fk(q, link="right_grasp_frame"):
    req = GetPositionFK.Request()
    req.header.frame_id = "world"
    req.header.stamp = n.get_clock().now().to_msg()
    req.fk_link_names = [link]
    req.robot_state.joint_state.name = list(R6)
    req.robot_state.joint_state.position = [float(x) for x in q]
    f = fk_cli.call_async(req)
    rclpy.spin_until_future_complete(n, f, timeout_sec=20)
    r = f.result()
    if r is None or r.error_code.val != 1:
        return None
    p = r.pose_stamped[0].pose
    return np.array([p.position.x, p.position.y, p.position.z]), \
        np.array([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])


def extract_arm(js):
    """按**关节名**取右臂 6 关节。

    ⚠️ 2026-09-18 踩坑（曾因此误判"IK 不可用"）：compute_ik 返回的
    solution.joint_state 含**全部 16 个关节**（左臂 0~7、右臂 8~13、左右指 14~15）。
    直接取 position[:6] 拿到的是**左臂**的值，回代 FK 会差几百 mm。
    """
    m = dict(zip(js.name, js.position))
    need = [f"right_j{i}" for i in range(1, 7)]
    if not all(k in m for k in need):
        return None
    return [m[k] for k in need]


def ik(p):
    """用 KDL 的 IK。

    实测（2026-09-18）：/move_group_fairino/compute_ik 对 R_SOLVED 与夹具四个目标
    均返回 -31（world / right_base_link 两种 frame 都试过），本工作区不可用；
    /move_group_kdl/compute_ik 在世界系下正确求解（回代残差 ≤0.004mm）。
    fairino 在其它位姿会返回 code=1 但偏差 ~350mm(= 基座偏置量级) 的解，不可信。
    """
    req = GetPositionIK.Request()
    req.ik_request.group_name = "right_arm"
    req.ik_request.ik_link_name = "right_grasp_frame"
    req.ik_request.pose_stamped = PoseStamped()
    req.ik_request.pose_stamped.header.frame_id = "world"
    req.ik_request.pose_stamped.header.stamp = n.get_clock().now().to_msg()
    req.ik_request.pose_stamped.pose.position.x = float(p[0])
    req.ik_request.pose_stamped.pose.position.y = float(p[1])
    req.ik_request.pose_stamped.pose.position.z = float(p[2])
    (req.ik_request.pose_stamped.pose.orientation.x,
     req.ik_request.pose_stamped.pose.orientation.y,
     req.ik_request.pose_stamped.pose.orientation.z,
     req.ik_request.pose_stamped.pose.orientation.w) = [float(v) for v in RECEIVE_QUAT]
    req.ik_request.avoid_collisions = False   # 只问可达性；碰撞由 clearance sweep 查
    req.ik_request.timeout.sec = 5
    f = ik_cli.call_async(req)
    rclpy.spin_until_future_complete(n, f, timeout_sec=30)
    res = f.result()
    if res is None:
        return None, "TIMEOUT", None
    if res.error_code.val != 1:
        return None, f"code={res.error_code.val}", None
    return extract_arm(res.solution.joint_state), "OK", res


print(f"夹具上杆心标称位姿 = {ROD}")
print(f"图参考姿态 receive_quat = {RECEIVE_QUAT}\n")
print(f"{'用例':<30} {'目标 grasp 位置':<34} {'IK':<10} {'FK 残差(mm/deg)'}")
print("-" * 100)

results = {}
for name, tgt in CASES:
    q, txt, _ = ik(tgt)
    if q is None:
        print(f"{name:<30} {np.round(tgt, 4)!s:<34} {txt:<10}")
        continue
    got = fk(q)
    if got is None:
        print(f"{name:<30} {np.round(tgt, 4)!s:<34} {txt:<10} (FK 读取失败)")
        continue
    gp, gq = got
    dp = np.linalg.norm(gp - tgt) * 1000
    R1 = quaternion_matrix(gq)[:3, :3]
    R2 = quaternion_matrix(RECEIVE_QUAT)[:3, :3]
    da = np.degrees(np.arccos(np.clip((np.trace(R1.T @ R2) - 1) / 2, -1, 1)))
    results[name] = (q, dp, da)
    print(f"{name:<30} {np.round(tgt, 4)!s:<34} {txt:<10} {dp:6.2f} / {da:6.2f}")

print("\n=== 结论 ===")
if len(results) == len(CASES) and all(dp < 1.0 and da < 0.5 for _, dp, da in results.values()):
    print("  ✓ 四个目标位姿均可解 IK（KDL）—— 夹具在当前位姿下右臂可达")
    print("\n  IK 解（可作 Joint Anchor 初值，需再经 20 次关节往返验证）：")
    for name, (q, dp, da) in results.items():
        print(f"    {name.split()[0]:<14} = [{', '.join(f'{v:.4f}' for v in q)}]"
              f"   ({dp:.3f} mm / {da:.3f}°)")
else:
    print(f"  ✗ 仅 {len(results)}/{len(CASES)} 可解（或残差过大）—— 需调整夹具世界位姿")
    print("    提示：优先在 fixture 的 x/y 上平移（夹具原点即 spawn 的 -x/-y）")

n.destroy_node()
rclpy.shutdown()
