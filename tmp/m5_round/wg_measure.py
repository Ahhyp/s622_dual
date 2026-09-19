#!/usr/bin/env python3
"""量 w_g：右爪指板沿杆轴方向( grasp_frame y )的占用包络。

数据来源（全部取自 URDF，不需仿真）：
  finger1_joint  origin xyz="-0.0099996 -8.7262E-05 0.1658" rpy="1.5708 0 3.1416"
                 axis  xyz="0.99996 0 -0.0087267"   parent wrist3_link
  finger2_joint  origin xyz=" 0.0099996  8.7271E-05 0.1658" rpy="1.5708 0 3.1416"
  wrist3_to_grasp_frame  origin xyz="0 0 0.2168"
  网格 finger1.STL / finger2.STL（二进制）

用法: wg_measure.py [finger1.STL 所在目录]
"""
import struct
import sys
from pathlib import Path

import numpy as np

MESH_DIR = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(
    "/home/yep/my_S622/src/s622_moveit_descriptions/meshes")


def load_binary_stl(path):
    """返回 (N,3,3) 三角面顶点数组。"""
    data = path.read_bytes()
    n = struct.unpack_from("<I", data, 80)[0]
    tris = np.empty((n, 3, 3), dtype=float)
    off = 84
    for i in range(n):
        # 跳过 normal(3f) + attr(2B)，读 9 个 float
        vals = struct.unpack_from("<9f", data, off + 12)
        tris[i] = np.array(vals, dtype=float).reshape(3, 3)
        off += 50
    return tris


def rpy_to_matrix(r, p, y):
    """URDF 约定: R = Rz(yaw) @ Ry(pitch) @ Rx(roll)。"""
    cr, sr = np.cos(r), np.sin(r)
    cp, sp = np.cos(p), np.sin(p)
    cy, sy = np.cos(y), np.sin(y)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


# ---- wrist3_link -> grasp_frame ----
T_W3_GF = np.array([0.0, 0.0, 0.2168])

# ---- 两个指关节（wrist3_link 系下的安装位姿）----
JOINTS = {
    "finger1": dict(t=np.array([-0.0099996, -8.7262e-05, 0.1658]),
                    rpy=(1.5708, 0.0, 3.1416),
                    axis=np.array([0.99996, 0.0, -0.0087267])),
    "finger2": dict(t=np.array([0.0099996, 8.7271e-05, 0.1658]),
                    rpy=(1.5708, 0.0, 3.1416),
                    axis=np.array([0.99996, 0.0, -0.0087267])),
}

print("=" * 72)
print("w_g 测量：指板沿杆轴的占用包络（grasp_frame 系）")
print("=" * 72)

all_min = np.full(3, np.inf)
all_max = np.full(3, -np.inf)

for name, j in JOINTS.items():
    stl = MESH_DIR / f"{name}.STL"
    tris = load_binary_stl(stl)
    verts = tris.reshape(-1, 3)
    print(f"\n--- {name}  ({stl.name}, {len(tris)} tris, {len(verts)} verts) ---")

    # 网格本体系下的包围盒
    bb_min, bb_max = verts.min(0), verts.max(0)
    print(f"  网格本体系 bbox: min={np.round(bb_min, 5)}  max={np.round(bb_max, 5)}")
    print(f"  网格本体系 尺寸: {np.round(bb_max - bb_min, 5)}  (m)")

    # 本体系 -> wrist3_link（q=0；关节轴几乎无 y 分量，位移不改变 y 向包络）
    R = rpy_to_matrix(*j["rpy"])
    p_w3 = (R @ verts.T).T + j["t"]

    # wrist3_link -> grasp_frame（纯平移）
    p_gf = p_w3 - T_W3_GF

    lo, hi = p_gf.min(0), p_gf.max(0)
    print(f"  grasp_frame 系 bbox: min={np.round(lo * 1000, 2)} mm")
    print(f"                       max={np.round(hi * 1000, 2)} mm")
    print(f"  grasp_frame 系 尺寸: {np.round((hi - lo) * 1000, 2)} mm  (x,y,z)")

    all_min = np.minimum(all_min, lo)
    all_max = np.maximum(all_max, hi)

print("\n" + "=" * 72)
print("两指合计占用包络（grasp_frame 系，mm）")
print("=" * 72)
print(f"  min = {np.round(all_min * 1000, 2)}")
print(f"  max = {np.round(all_max * 1000, 2)}")
print(f"  尺寸 = {np.round((all_max - all_min) * 1000, 2)}  (x, y, z)")

wg_y = (all_max - all_min)[1] * 1000
print(f"\n>>> w_g（沿杆轴 = grasp_frame y 的占用包络） = {wg_y:.2f} mm")
print(f">>> 杆长 150mm；扣除两指包络后，单一抓取区之外可用长度 = "
      f"{150 - wg_y:.2f} mm")
