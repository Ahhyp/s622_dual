#!/usr/bin/env python3
"""手眼标定求解 + 验证"""
import os
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from tf2_ros import Buffer, TransformListener
import tf_transformations as tft


def rot_angle_deg(R):
    c = np.clip((np.trace(R) - 1) / 2, -1.0, 1.0)
    return np.rad2deg(np.arccos(c))


def pose_diff(T1, T2):
    dT = np.linalg.inv(T1) @ T2
    return np.linalg.norm(dT[:3, 3]), rot_angle_deg(dT[:3, :3])


def calibrate(T_ee_base_list, T_marker_cam_list):
    R_b2e, t_b2e = [], []
    for T in T_ee_base_list:
        Ti = np.linalg.inv(T)
        R_b2e.append(Ti[:3, :3])
        t_b2e.append(Ti[:3, 3])
    R_m2c = [T[:3, :3] for T in T_marker_cam_list]
    t_m2c = [T[:3, 3]  for T in T_marker_cam_list]

    methods = {
        'TSAI':       cv2.CALIB_HAND_EYE_TSAI,
        'PARK':       cv2.CALIB_HAND_EYE_PARK,
        'HORAUD':     cv2.CALIB_HAND_EYE_HORAUD,
        'ANDREFF':    cv2.CALIB_HAND_EYE_ANDREFF,
        'DANIILIDIS': cv2.CALIB_HAND_EYE_DANIILIDIS,
    }
    out = {}
    for name, m in methods.items():
        R, t = cv2.calibrateHandEye(R_b2e, t_b2e, R_m2c, t_m2c, method=m)
        T = np.eye(4); T[:3,:3] = R; T[:3,3] = t.flatten()
        out[name] = T
    return out


def internal_consistency(T_cam_base, T_ee_base_list, T_marker_cam_list):
    T_marker_ee_list = []
    for T_eb, T_mc in zip(T_ee_base_list, T_marker_cam_list):
        T_me = np.linalg.inv(T_eb) @ T_cam_base @ T_mc
        T_marker_ee_list.append(T_me)
    pos = np.array([T[:3, 3] for T in T_marker_ee_list])
    pos_std = pos.std(axis=0) * 1000
    ref_R = T_marker_ee_list[0][:3, :3]
    rot_diffs = [rot_angle_deg(np.linalg.inv(ref_R) @ T[:3, :3])
                 for T in T_marker_ee_list]
    return pos_std, float(np.std(rot_diffs)), pos.mean(axis=0)


def get_gt():
    rclpy.init()
    node = Node('gt_lookup')
    buf = Buffer(); TransformListener(buf, node)
    for _ in range(30):
        rclpy.spin_once(node, timeout_sec=0.1)
    m = buf.lookup_transform('base_link', 'camera_color_optical_frame',
                             rclpy.time.Time(), timeout=Duration(seconds=2.0))
    t = m.transform.translation; q = m.transform.rotation
    M = tft.quaternion_matrix([q.x, q.y, q.z, q.w])
    M[:3, 3] = [t.x, t.y, t.z]
    node.destroy_node(); rclpy.shutdown()
    return M


def main():
    data = np.load(os.path.join(os.path.dirname(__file__), 'data/handeye.npz'))
    T_eb = data['T_ee_base']; T_mc = data['T_marker_cam']
    print(f'loaded {len(T_eb)} samples')

    results = calibrate(T_eb, T_mc)
    T_gt = get_gt()
    print('\n=== T_cam^base ground truth (from TF) ===')
    print(T_gt.round(4))

    print(f'\n{"method":<12}{"pos[mm]":>10}{"rot[°]":>9}'
          f'{"me_std[mm]":>26}{"me_rot_std[°]":>18}')
    print('-' * 75)
    for name, T_est in results.items():
        pe, re = pose_diff(T_gt, T_est)
        ps, rs, _ = internal_consistency(T_est, T_eb, T_mc)
        print(f'{name:<12}{pe*1000:>10.2f}{re:>9.3f}'
              f'  {np.round(ps,2)!s:>22}{rs:>18.3f}')

    # 保存 DANIILIDIS 结果
    out = os.path.join(os.path.dirname(__file__), 'data/T_cam_base_estimated.npy')
    np.save(out, results['DANIILIDIS'])
    print(f'\nsaved -> {out}')


if __name__ == '__main__':
    main()