#!/usr/bin/env python3
"""M5 夹具可达性：FK + 阻尼最小二乘（6-DOF），不依赖本项目不可靠的 IK 服务。

背景（2026-09-18 实测）：
  /move_group_fairino/compute_ik 在 M4 **已知可用**的接物位姿上就返回 -31；
  /move_group_kdl/compute_ik 对四个不同目标返回**完全相同**的伪解（残差 684mm/71°）。
  ⇒ 两者都不能作可达性判据。M4 的 solve_receive.py 用的就是 FK 雅可比下降，这里推广到 6-DOF。

方法：数值雅可比(6x6, 中心差分) + 阻尼最小二乘 + 线搜索，多起点。

用法: fixture_solve.py [rodx rody rodz]
"""
import sys

import numpy as np
import rclpy
from moveit_msgs.srv import GetPositionFK
from rclpy.node import Node
from tf_transformations import quaternion_matrix

ROD = np.array([float(a) for a in sys.argv[1:4]]) if len(sys.argv) > 3 \
    else np.array([0.10, -0.20, 0.120])
RECEIVE_OFFSET = np.array([-0.0086, -0.054, 0.0030])
RECEIVE_QUAT = np.array([0.0087, 0.6819, 0.0060, 0.7314])
R6 = [f"right_j{i}" for i in range(1, 7)]
LIM = np.array([[-3.0543, 3.0543],    # j1
                [-4.6251, 1.4835],    # j2  (2026-09-18 修正：原写 ±2.094 是错的)
                [-2.8274, 2.8274],    # j3
                [-4.6251, 1.4835],    # j4  (同上修正)
                [-3.0543, 3.0543],    # j5
                [-3.0543, 3.0543]])   # j6
# 来源: s622_moveit_descriptions.urdf.xacro 中 ${prefix}j1..j6 的 <limit>，
# prefix="right_" 与 "left_" 共用同一组限位。
SEEDS = [
    [0.2856, -1.2598, -1.7115, -3.3161, -1.2915, -1.5708],   # M4 R_SOLVED
    [0.0, -1.0, 1.0, -1.0, -0.8, 0.0],
    [0.5, -1.57, 1.0, -1.57, -1.0, 0.0],
    [-0.5, -1.57, 1.2, -1.57, -1.2, 0.3],
    [1.0, -1.9, 1.8, -2.2, -1.0, -0.5],
    [0.3, -2.0, 2.2, 0.0, -1.2, -1.57],
    [0.0, -0.8, 1.5, -2.5, 1.2, -1.57],
    [-0.3, -1.2, 2.0, 1.5, -1.0, 1.57],
    [0.6, -1.0, 0.5, -3.0, -0.5, 1.57],
    [1.5, -0.6, 0.0, -1.0, -1.2, -3.0],
]


def limit_margin(q):
    """离最近关节限位的距离（rad）；越小越危险。"""
    return float(np.min(np.minimum(np.asarray(q) - LIM[:, 0], LIM[:, 1] - np.asarray(q))))

rclpy.init()
n = Node("m5_solve")
fk_cli = n.create_client(GetPositionFK, "/move_group_fairino/compute_fk")
fk_cli.wait_for_service(timeout_sec=15)


def fk(q):
    req = GetPositionFK.Request()
    req.header.frame_id = "world"
    req.header.stamp = n.get_clock().now().to_msg()
    req.fk_link_names = ["right_grasp_frame"]
    req.robot_state.joint_state.name = list(R6)
    req.robot_state.joint_state.position = [float(x) for x in q]
    f = fk_cli.call_async(req)
    rclpy.spin_until_future_complete(n, f, timeout_sec=20)
    r = f.result()
    if r is None or r.error_code.val != 1:
        return None
    p = r.pose_stamped[0].pose
    return (np.array([p.position.x, p.position.y, p.position.z]),
            np.array([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]))


def pose_err(cur, tgt_p, tgt_q):
    """6 维误差：位置(m) + 旋转向量(rad)。"""
    p, q = cur
    ep = tgt_p - p
    Rc = quaternion_matrix(q)[:3, :3]
    Rt = quaternion_matrix(tgt_q)[:3, :3]
    Re = Rt @ Rc.T
    w = np.array([Re[2, 1] - Re[1, 2], Re[0, 2] - Re[2, 0], Re[1, 0] - Re[0, 1]]) / 2
    s = np.linalg.norm(w)
    er = w * (np.arcsin(min(1.0, s)) / s) if s > 1e-9 else w
    return np.concatenate([ep, er]), np.linalg.norm(ep) * 1000, np.degrees(np.linalg.norm(er))


def solve(tgt_p, tgt_q, q0, iters=120):
    q = np.array(q0, dtype=float)
    best = None
    lam = 1e-3
    for _ in range(iters):
        cur = fk(q)
        if cur is None:
            return None
        e, dp, da = pose_err(cur, tgt_p, tgt_q)
        if best is None or dp < best[1]:
            best = (q.copy(), dp, da)
        if dp < 0.5 and da < 0.3:
            break
        J = np.zeros((6, 6))
        h = 1e-4
        ok = True
        for i in range(6):
            qp, qm = q.copy(), q.copy()
            qp[i] += h
            qm[i] -= h
            cp, cm = fk(qp), fk(qm)
            if cp is None or cm is None:
                ok = False
                break
            ep_, _, _ = pose_err(cp, tgt_p, tgt_q)
            em_, _, _ = pose_err(cm, tgt_p, tgt_q)
            J[:, i] = (em_ - ep_) / (2 * h)
        if not ok:
            break
        # 阻尼最小二乘：解 J dq = e（注意 e = 目标-当前，故用 e 而非 -e）
        dq = np.linalg.solve(J.T @ J + lam * np.eye(6), J.T @ e)
        dq = np.clip(dq, -0.25, 0.25)
        q_new = np.clip(q + dq, LIM[:, 0], LIM[:, 1])
        c_new = fk(q_new)
        if c_new is None:
            break
        _, dpn, dan = pose_err(c_new, tgt_p, tgt_q)
        if dpn < dp or dan < da:
            q = q_new
            lam = max(lam * 0.7, 1e-6)
        else:
            lam = min(lam * 3.0, 1.0)
    return best


def target(long_mm, dz=0.0, dx=0.0):
    off = RECEIVE_OFFSET.copy()
    off[1] = long_mm / 1000.0
    return ROD + off + np.array([dx, 0.0, dz])


# ---- 先自检 FK 可信度：R_SOLVED 的 FK 应 ≈ M4 实测接物位姿 ----
print("=== FK 自检 ===")
known = fk(SEEDS[0])
print(f"  R_SOLVED FK = {np.round(known[0], 4)}")
print(f"  M4 接物应是 = {np.round(np.array([0.042, 0, 0.340]) + RECEIVE_OFFSET, 4)}")
print(f"  差 = {np.linalg.norm(known[0] - (np.array([0.042, 0, 0.340]) + RECEIVE_OFFSET)) * 1000:.1f} mm"
      "   (M4 的 hold 位是实测值，允许几 mm 差异)\n")

CASES = [
    ("place        杆在夹具, 持 -54mm 端", target(-54)),
    ("place_pre    同上抬 50mm", target(-54, dz=0.050)),
    ("regrasp      杆在夹具, 抓 +54mm 端", target(+54)),
    ("regrasp_pre  同上沿 -x 退 60mm", target(+54, dx=-0.060)),
]

print(f"夹具上杆心 = {ROD}   姿态 = M4 receive_quat\n")
print(f"{'用例':<34} {'目标 grasp 位置':<30} {'全部收敛解(按限位裕度排序)'}")
print("-" * 112)
sols = {}
for name, tgt in CASES:
    cands = []
    best_any = None
    for s in SEEDS:
        r = solve(tgt, RECEIVE_QUAT, s)
        if r is None:
            continue
        if best_any is None or (r[1], r[2]) < (best_any[1], best_any[2]):
            best_any = r
        if r[1] < 1.0 and r[2] < 0.5:
            cands.append(r)
    if not cands:
        print(f"{name:<34} {np.round(tgt, 4)!s:<30} 无干净解 "
              f"(最好 {best_any[1]:.2f}mm/{best_any[2]:.2f}°)" if best_any else "求解失败")
        continue
    # 按限位裕度降序
    cands.sort(key=lambda r: -limit_margin(r[0]))
    lines = []
    seen = set()
    for q, dp, da in cands:
        key = np.round(np.asarray(q) / 0.3).astype(int).tobytes()   # 去重：粗聚类
        if key in seen:
            continue
        seen.add(key)
        lines.append(f"margin={np.degrees(limit_margin(q)):5.1f}°  "
                     f"[{', '.join(f'{v:6.3f}' for v in q)}]")
    sols[name.split()[0]] = cands[0]
    head = lines[0] if lines else ""
    print(f"{name:<34} {np.round(tgt, 4)!s:<30} {len(cands)} 个收敛解")
    for ln in lines[:3]:
        print(f"{'':<34} {'':<30}   {ln}")

print("\n=== 结论 ===")
allclean = all(limit_margin(v[0]) > np.radians(15) for v in sols.values()) and len(sols) == len(CASES)
if len(sols) == len(CASES):
    print(f"  4/4 位姿可达（残差 ≤0.5mm/0.3°）")
    if allclean:
        print("  ✓ 且均存在离限位 >15° 的解 —— 夹具标称位姿可用")
    else:
        print("  ⚠ 存在贴限位解（<15°）—— 优先选 margin 大的分支；若所有分支都贴限位，需平移夹具")
    print("\n  Joint Anchor 候选（限位裕度最大的分支，待 20 次关节往返 + 碰撞验证）：")
    for k, (q, dp, da) in sols.items():
        print(f"    {k:<14} = [{', '.join(f'{v:.4f}' for v in q)}]"
              f"   ({dp:.2f} mm / {da:.2f}°, margin {np.degrees(limit_margin(q)):.1f}°)")
else:
    print(f"  ✗ 仅 {len(sols)}/{len(CASES)} 可解 —— 夹具世界位姿需平移")
    print("    夹具原点 = spawn 的 -x/-y，优先在这两个方向调")

n.destroy_node()
rclpy.shutdown()
