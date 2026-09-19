#!/usr/bin/env python3
"""M5 夹具落座验证：spawn 夹具 + 在其正上方放一根杆，看落座高度/姿态是否符合设计。

设计预期（regrasp_fixture/model.sdf 注释）：
  杆心应停在 V 尖 + 21.21mm = 0.0988 + 0.0212 = 0.1200 m
  杆长轴应保持世界 Y

用法: fixture_seat_check.py [fx] [fy]     默认 0.10 -0.20
"""
import re
import subprocess
import sys
import time

FX = float(sys.argv[1]) if len(sys.argv) > 1 else 0.10
FY = float(sys.argv[2]) if len(sys.argv) > 2 else -0.20
WORLD = "dual_arm_world"
FIXTURE_SDF = "/home/yep/my_S622/src/gz_launch/models/regrasp_fixture/model.sdf"
ROD_SDF = "/home/yep/my_S622/tmp/m4_round/models/rod/model.sdf"


def sh(cmd, t=30):
    return subprocess.run(["bash", "-c", f"timeout {t} {cmd}"],
                          capture_output=True, text=True).stdout


def spawn(sdf, name, x, y, z, yaw=0.0):
    """与 M4 bt_round.py 一致：ros_gz_sim create（ign model 只是查询工具）。"""
    return sh(f"ros2 run ros_gz_sim create -world {WORLD} -file {sdf} -name {name} "
              f"-x {x} -y {y} -z {z} -Y {yaw} 2>&1 | tail -1", t=40).strip()


def rm(name):
    """与 M4 bt_round.py 一致：ign service .../remove。"""
    sh(f"ign service -s /world/{WORLD}/remove --reqtype ignition.msgs.Entity "
       f"--reptype ignition.msgs.Boolean --timeout 3000 "
       f"--req 'name: \"{name}\", type: MODEL' 2>&1 | tail -1", t=40)


def pose_info():
    return sh("ign topic -e -t /world/dual_arm_world/pose/info -n 1 2>/dev/null", 25)


def model(name, txt=None):
    """按 'name: "..."' 切块解析，避免窗口滑到下一个模型。

    注意：static 模型在 pose/info 里通常**没有** position/orientation 块，
    此时返回 (None, None) 表示"存在但无位姿"，而非误报别人的位姿。
    """
    txt = txt if txt is not None else pose_info()
    blocks = txt.split('name: "')
    for b in blocks[1:]:
        if not b.startswith(name + '"'):
            continue
        m = re.search(r"position \{\s*x: ([\d.eE+-]+)\s*y: ([\d.eE+-]+)\s*z: ([\d.eE+-]+)", b)
        q = re.search(r"orientation \{\s*x: ([\d.eE+-]+)\s*y: ([\d.eE+-]+)\s*z: ([\d.eE+-]+)\s*w: ([\d.eE+-]+)", b)
        if m and q:
            return (tuple(float(m.group(k)) for k in (1, 2, 3)),
                    tuple(float(q.group(k)) for k in (1, 2, 3, 4)))
        return (None, None)   # static 模型：存在但无位姿
    return None


print(f"=== 夹具落座验证 @ ({FX}, {FY}) ===")
rm("m5fixture")
rm("m5seatrod")
time.sleep(1)

print("spawn fixture:", spawn(FIXTURE_SDF, "m5fixture", FX, FY, 0.0) or "ok")
time.sleep(2)

# 杆从夹具正上方 0.20m 释放（长轴沿 Y = yaw 90°）
print("spawn rod    :", spawn(ROD_SDF, "m5seatrod", FX, FY, 0.20, yaw=1.5708) or "ok")

# RTF≈0.2: 等 25s 墙钟 ≈ 5s 仿真，足够下落 80mm 并稳定
for i in range(5):
    time.sleep(5)
    rp = model("m5seatrod")
    if rp:
        p, q = rp
        print(f"  t+{(i + 1) * 5: >2}s  rod pos=({p[0]:.4f}, {p[1]:.4f}, {p[2]:.4f})  "
              f"quat=({q[0]:.3f}, {q[1]:.3f}, {q[2]:.3f}, {q[3]:.3f})")

rp = model("m5seatrod")
fx_ = model("m5fixture")
print("\n=== 判定 ===")
if not rp:
    print("  ✗ 读不到 rod 位姿")
    sys.exit(1)
p, q = rp
if fx_:
    if fx_[0] is None:
        print("  fixture 位姿: (static 模型，pose/info 不发布位姿 —— 属正常)")
    else:
        print(f"  fixture 位姿: pos=({fx_[0][0]:.4f}, {fx_[0][1]:.4f}, {fx_[0][2]:.4f})")

exp_z = 0.1200
dz = (p[2] - exp_z) * 1000
print(f"  杆心 z = {p[2]:.4f} m   期望 {exp_z:.4f} m   偏差 {dz:+.2f} mm")
# 长轴是否仍沿 Y：yaw=90° 时 quat ≈ (0,0,0.707,0.707)
long_y = abs(q[2]) > 0.6 and abs(q[3]) > 0.6 and abs(q[0]) < 0.3 and abs(q[1]) < 0.3
print(f"  长轴沿世界 Y: {'✓' if long_y else '✗'}  (quat 判据)")
ok = abs(dz) < 3.0 and long_y
print(f"\n  结论: {'✓ 落座符合设计' if ok else '✗ 与设计不符，需查 V 几何'}")
