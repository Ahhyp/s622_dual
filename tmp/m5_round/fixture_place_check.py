#!/usr/bin/env python3
"""M5 落夹具后验证：杆是否停在夹具支撑面上（§10 用 GT 判，不让视觉自证）。

判据：
  z   ∈ 0.1200 ± 3mm      （支撑面 0.105 + 杆半高 0.015；两次独立放置实测均 0.1200）
  xy  ∈ 夹具有效区半径     （默认 50mm，对应 m5_task.yaml 的 fixture_valid_region_xy）
  长轴仍沿世界 Y（quat 判据）

用法: fixture_place_check.py <rod_name> [fx] [fy]
"""
import re
import subprocess
import sys

ROD = sys.argv[1] if len(sys.argv) > 1 else "handover_bar"
FX = float(sys.argv[2]) if len(sys.argv) > 2 else 0.10
FY = float(sys.argv[3]) if len(sys.argv) > 3 else -0.20
EXP_Z = 0.1200
TOL_Z = 0.003
TOL_XY = 0.050


def sh(cmd, t=25):
    return subprocess.run(["bash", "-c", f"timeout {t} {cmd}"],
                          capture_output=True, text=True).stdout


def model(name, txt):
    for b in txt.split('name: "')[1:]:
        if not b.startswith(name + '"'):
            continue
        m = re.search(r"position \{\s*x: ([\d.eE+-]+)\s*y: ([\d.eE+-]+)\s*z: ([\d.eE+-]+)", b)
        q = re.search(r"orientation \{\s*x: ([\d.eE+-]+)\s*y: ([\d.eE+-]+)\s*z: ([\d.eE+-]+)\s*w: ([\d.eE+-]+)", b)
        if m and q:
            return (tuple(float(m.group(k)) for k in (1, 2, 3)),
                    tuple(float(q.group(k)) for k in (1, 2, 3, 4)))
        return (None, None)
    return None


txt = sh("ign topic -e -t /world/dual_arm_world/pose/info -n 1 2>/dev/null")
rp = model(ROD, txt)
print(f"=== 落夹具验证: rod='{ROD}'  夹具=( {FX}, {FY} )  期望杆心 z={EXP_Z} ===")
if rp is None:
    print(f"  ✗ 场景里找不到模型 '{ROD}'（可能名字不同，或已被清理）")
    print(f"    场景模型: {[b.split(chr(34))[0] for b in txt.split('name: ')[1:]][:12]}")
    sys.exit(1)
if rp[0] is None:
    print("  ✗ 该模型是 static、pose/info 无位姿")
    sys.exit(1)
p, q = rp
dz = (p[2] - EXP_Z) * 1000
dxy = ((p[0] - FX) ** 2 + (p[1] - FY) ** 2) ** 0.5 * 1000
long_y = abs(q[2]) > 0.6 and abs(q[3]) > 0.6 and abs(q[0]) < 0.3 and abs(q[1]) < 0.3

print(f"  杆心 = ({p[0]:+.4f}, {p[1]:+.4f}, {p[2]:+.4f})   quat = "
      f"({q[0]:+.3f}, {q[1]:+.3f}, {q[2]:+.3f}, {q[3]:+.3f})")
print(f"  z  偏差 = {dz:+.2f} mm      (容差 ±{TOL_Z * 1000:.0f}mm)")
print(f"  xy 偏置 = {dxy:.2f} mm      (有效区半径 {TOL_XY * 1000:.0f}mm)")
print(f"  长轴沿世界Y = {'✓' if long_y else '✗'}")
ok = abs(dz) <= TOL_Z * 1000 and dxy <= TOL_XY * 1000 and long_y
print(f"\n  {'✓ 杆已稳定落在夹具支撑面上 (Planned Release 成功)' if ok else '✗ 未达判据'}")
sys.exit(0 if ok else 2)
