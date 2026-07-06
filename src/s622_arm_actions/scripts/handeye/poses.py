# scripts/handeye/poses.py
import numpy as np

CENTER_XYZ = np.array([0.30, 0.15, 0.35])
CENTER_RPY = np.array([-2.13, 0.0, 0.0])  # marker 正对相机

def generate_poses():
    poses = []
    xy_offsets = [
        (0.0, 0.0),
        (0.06, 0.0), (-0.06, 0.0),
        (0.0, 0.05), (0.0, -0.05),
    ]
    # 扰动范围收敛在 marker 仍能被稳定检测的锥体内
    rpy_offsets = [
        (0.0, 0.0, 0.0),
        (np.deg2rad( 15), 0.0, 0.0),   # 绕 X: 前后仰
        (np.deg2rad(-15), 0.0, 0.0),
        (0.0, np.deg2rad( 20), 0.0),   # 绕 Y: 左右倾
        (0.0, np.deg2rad(-20), 0.0),
    ]
    yaw_offsets = [
        (np.deg2rad( 10), np.deg2rad( 10), np.deg2rad( 25)),
        (np.deg2rad(-10), np.deg2rad(-10), np.deg2rad(-25)),
        (np.deg2rad( 15), 0.0,              np.deg2rad( 30)),
        (np.deg2rad(-15), 0.0,              np.deg2rad(-30)),
    ]
    for dx, dy in xy_offsets:
        for drx, dry, drz in rpy_offsets:
            poses.append((
                CENTER_XYZ + np.array([dx, dy, 0.0]),
                CENTER_RPY + np.array([drx, dry, drz]),
            ))
    for drx, dry, drz in yaw_offsets:
        poses.append((
            CENTER_XYZ.copy(),
            CENTER_RPY + np.array([drx, dry, drz]),
        ))
    return poses  # 5*5 + 4 = 29 个