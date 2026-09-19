#!/usr/bin/env python3
"""M5 夹具碰撞 clearance sweep（§24：不能只看 TCP 能不能到）。

由 M4 的 r_clear_check.py 改写：把"杆盒"换成"夹具的一组盒体"，
并对 4 个已验证的 Joint Anchor 配置逐一检查右臂全部带网格连杆。

夹具盒体（局部系 = 世界系，yaw=0，原点 (0.10,-0.20,0)）—— 与 regrasp_fixture/model.sdf 一致。

期望（§27）：
  允许  rod ↔ 支撑台顶面（夹具的 design contact）
  禁止  任何右臂连杆 ↔ 夹具（finger/gripper/wrist/forearm）
"""
import struct
import sys

import numpy as np
import rclpy
from moveit_msgs.srv import GetPositionFK
from rclpy.node import Node
from tf_transformations import quaternion_matrix

MESH = "/home/yep/my_S622/src/s622_moveit_descriptions/meshes"
R6 = [f"right_j{i}" for i in range(1, 7)]
LINKS = ["right_shoulder_link", "right_upperarm_link", "right_forearm_link",
         "right_wrist1_link", "right_wrist2_link", "right_wrist3_link",
         "right_finger1", "right_finger2"]
MESH_OF = {
    "right_shoulder_link": "shoulder_link.STL", "right_upperarm_link": "upperarm_link.STL",
    "right_forearm_link": "forearm_link.STL", "right_wrist1_link": "wrist1_link.STL",
    "right_wrist2_link": "wrist2_link.STL", "right_wrist3_link": "wrist3_link.STL",
    "right_finger1": "finger1.STL", "right_finger2": "finger2.STL",
}

FIXTURE_ORIGIN = np.array([0.10, -0.20, 0.0])
FIXTURE_YAW = 0.0


def boxes():
    """夹具盒体: (name, center_local, half_size)"""
    cy, sy = np.cos(FIXTURE_YAW), np.sin(FIXTURE_YAW)
    R = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1.0]])
    raw = [
        ("base", (0.0, 0.0, 0.010), (0.060, 0.070, 0.010)),
        ("pedestal_m", (0.0, -0.020, 0.0625), (0.023, 0.006, 0.0425)),
        ("pedestal_p", (0.0, 0.020, 0.0625), (0.023, 0.006, 0.0425)),
        ("lip_m_neg", (-0.021, -0.020, 0.070), (0.003, 0.006, 0.050)),
        ("lip_m_pos", (0.021, -0.020, 0.070), (0.003, 0.006, 0.050)),
        ("lip_p_neg", (-0.021, 0.020, 0.070), (0.003, 0.006, 0.050)),
        ("lip_p_pos", (0.021, 0.020, 0.070), (0.003, 0.006, 0.050)),
    ]
    out = []
    for name, c, h in raw:
        cw = FIXTURE_ORIGIN + R @ np.array(c)
        out.append((name, cw, np.array(h)))
    return out


BOXES = boxes()


def read_stl(path):
    with open(path, "rb") as f:
        data = f.read()
    n = struct.unpack("<I", data[80:84])[0]
    v = []
    for i in range(n):
        off = 84 + i * 50 + 12
        for k in range(3):
            v.append(struct.unpack("<3f", data[off + 12 * k: off + 12 * k + 12]))
    return np.array(v)


def dist_to_box(pts, c, h):
    """点到 AABB 的有符号距离：>0 间隙, <0 穿模深度, 返回 (min_dist_mm, max_pen_mm)"""
    d = np.abs(pts - c) - h
    outside = np.maximum(d, 0.0)
    dist = float(np.linalg.norm(outside, axis=1).min())
    inside = np.all(d < 0, axis=1)
    pen = float(np.min(-d[inside], axis=1).max()) if inside.any() else 0.0
    return dist * 1000, pen * 1000


ANCHORS = {
    "place":       [1.9919, -1.0456, 1.8560, -0.9782, 0.4309, -1.3972],
    "place_pre":   [1.9919, -1.1663, 1.7845, -0.7864, 0.4310, -1.3967],
    "regrasp":     [-0.1815, -1.9266, -2.1485, -2.1371, -1.7473, -1.5373],
    "regrasp_pre": [-0.2375, -1.7870, -2.3669, -2.0575, -1.8031, -1.5333],
}
# 2026-09-18: 由 fixture_solve.py 在**修正后的真实限位**下重解（旧解属另一分支，
# 不可与本次 clearance 结论混用）。限位来源: urdf.xacro ${prefix}j1..j6。
# 夹持 30mm 杆时指关节量（gap = q1 - q2 - 0.18mm）
FINGERS = [17.0, -17.0]

rclpy.init()
n = Node("m5_fix_clear")
fk_cli = n.create_client(GetPositionFK, "/move_group_fairino/compute_fk")
fk_cli.wait_for_service(timeout_sec=15)


def fk_links(q):
    req = GetPositionFK.Request()
    req.header.frame_id = "world"
    req.header.stamp = n.get_clock().now().to_msg()
    req.fk_link_names = LINKS
    req.robot_state.joint_state.name = list(R6) + ["right_finger1_joint", "right_finger2_joint"]
    req.robot_state.joint_state.position = [float(x) for x in q] + \
        [FINGERS[0] / 1000.0, FINGERS[1] / 1000.0]
    f = fk_cli.call_async(req)
    rclpy.spin_until_future_complete(n, f, timeout_sec=25)
    r = f.result()
    if r is None or r.error_code.val != 1:
        return None
    return r.pose_stamped


def check_config(q, verbose=False):
    """返回 (夹具最小间隙mm, 地面穿模mm)；同时给出最近的 link↔box。"""
    ps = fk_links(q)
    if ps is None:
        return None
    min_fix = (1e9, None, None)
    ground_pen = 0.0
    ground_who = None
    for name, P in zip(LINKS, ps):
        T = quaternion_matrix([P.pose.orientation.x, P.pose.orientation.y,
                               P.pose.orientation.z, P.pose.orientation.w])
        T[:3, 3] = [P.pose.position.x, P.pose.position.y, P.pose.position.z]
        v = read_stl(f"{MESH}/{MESH_OF[name]}")
        vw = (T[:3, :3] @ v.T).T + T[:3, 3]
        # 地面 z=0：顶点低于 0 即穿模
        zmin = float(vw[:, 2].min())
        if -zmin > ground_pen:
            ground_pen, ground_who = -zmin, name
        for bname, c, h in BOXES:
            d, pen = dist_to_box(vw, c, h)
            if pen > 0 and verbose:
                print(f"    ⚠ {name} ↔ {bname} 穿模 {pen:.2f} mm")
            if d < min_fix[0]:
                min_fix = (d, name, bname)
    return min_fix, ground_pen * 1000, ground_who


M4_RECEIVE = [0.2856, -1.2598, -1.7115, -3.3161, -1.2915, -1.5708]

print(f"夹具原点 = {FIXTURE_ORIGIN}   指关节 = {FINGERS} mm   "
      f"(gap={FINGERS[0] - FINGERS[1] - 0.18:.2f}mm)\n")
overall = True
print(f"{'配置 / 路径':<34} {'夹具最近(mm)':>12} {'最近 link↔box':<26} {'地面穿模(mm)':>12}")
print("-" * 92)

for aname, q in ANCHORS.items():
    res = check_config(q)
    if res is None:
        print(f"{aname:<34} {'FK 失败':>12}")
        overall = False
        continue
    (d, ln, bn), gp, gw = res
    flag = "" if (d >= 3.0 and gp < 0.1) else "   ⚠"
    if d < 3.0 or gp >= 0.1:
        overall = False
    print(f"{aname:<34} {d:12.2f} {f'{ln} ↔ {bn}':<26} {gp:12.2f}{flag}")
    if gp >= 0.1:
        print(f"{'':<34} {'':>12} {'(扫地: ' + str(gw) + ')':<26}")

# ---- 关键转场路径插值：M4 接物位 → fixture place_pre ----
PATHS = [
    ("M4接物位 → place_pre", M4_RECEIVE, ANCHORS["place_pre"]),
    ("place_pre → place", ANCHORS["place_pre"], ANCHORS["place"]),
    ("regrasp_pre → regrasp", ANCHORS["regrasp_pre"], ANCHORS["regrasp"]),
]
print()
for pname, qa, qb in PATHS:
    worst_fix, worst_gp = 1e9, 0.0
    worst_who = ("", "", "")
    gp_who = ""
    for i in range(21):
        t = i / 20.0
        q = [(1 - t) * a + t * b for a, b in zip(qa, qb)]
        res = check_config(q)
        if res is None:
            continue
        (d, ln, bn), gp, gw = res
        if d < worst_fix:
            worst_fix, worst_who = d, (ln, bn, f"t={t:.2f}")
        if gp > worst_gp:
            worst_gp, gp_who = gp, f"{gw} @t={t:.2f}"
    bad = worst_fix < 3.0 or worst_gp >= 0.1
    if bad:
        overall = False
    print(f"{pname:<34} {worst_fix:12.2f} {f'{worst_who[0]}↔{worst_who[1]} {worst_who[2]}':<26} "
          f"{worst_gp:12.2f}{'   ⚠' if bad else ''}")
    if worst_gp >= 0.1:
        print(f"{'':<34} {'':>12} {'(扫地: ' + gp_who + ')':<26}")

print("\n=== 结论 ===")
if overall:
    print("  ✓ Anchor 与关键转场路径：夹具间隙 ≥3mm、无扫地")
    print("  注：路径为关节空间线性插值；实际执行走规划器，仍需执行期 collision 检查兜底")
else:
    print("  ⚠ 存在过近/穿模/扫地 —— 需改夹具几何、Anchor 或路径")
n.destroy_node()
rclpy.shutdown()
