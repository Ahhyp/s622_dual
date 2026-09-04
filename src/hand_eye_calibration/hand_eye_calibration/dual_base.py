"""M2.7-H 双臂组合纯数学（可测试，derive 脚本与回归测试共用）。

只含 4x4 齐次矩阵运算，不依赖 ROS/求解器：

- ``t_lr_cam``            : T_LR_cam = T_LC @ inv(T_RC)（方法 B 组合方向，禁止写反）
- ``translation_error_mm``: 组合平移相对 URDF GT (0.7, 0, 0) 的误差
- ``rotation_delta_deg``  : 两矩阵旋转差（度）
- ``decompose_tlr_error`` : GPT P1 误差分解（full / rot_only / trans_only /
                           left_only / right_only，单位 mm）——用于回答
                           "组合 5-7mm 来自平移残差还是旋转×~1m 杠杆臂"

GT 约定（见 M2_规划.md §M2.8 / M2.7_全局相机EyeOnBase仿真.md §1）：
world 中 left_base @ (0.35,0,0) yaw=π、right_base @ (-0.35,0,0)
→ left_base_T_right_base GT = Rz(180°) + t=(0.7,0,0)。
"""
from __future__ import annotations

from typing import Tuple

import numpy as np

#: T_LR GT 平移（left base 系），米
TLR_GT_TRANSLATION: Tuple[float, float, float] = (0.7, 0.0, 0.0)


def t_lr_cam(left_T_cam: np.ndarray, right_T_cam: np.ndarray) -> np.ndarray:
    """{}^{B_L}T_{B_R}^{camera} = T_LC · inv(T_RC)。

    方向保护：这是 M2.7 验收⑤ 的唯一组合公式；若写成 right @ inv(left) 或
    left @ right 即方向写反（有回归测试 test_tlr_direction 守护）。
    """
    left_T_cam = np.asarray(left_T_cam, dtype=float)
    right_T_cam = np.asarray(right_T_cam, dtype=float)
    return left_T_cam @ np.linalg.inv(right_T_cam)


def translation_error_mm(
    transform: np.ndarray,
    gt_translation: Tuple[float, float, float] = TLR_GT_TRANSLATION,
) -> float:
    """组合平移相对 GT 的欧氏误差（mm）。"""
    t = np.asarray(transform, dtype=float)[:3, 3]
    return float(np.linalg.norm(t - np.asarray(gt_translation, dtype=float)) * 1000.0)


def rotation_delta_deg(a: np.ndarray, b: np.ndarray) -> float:
    """两 4x4 矩阵旋转部分的相对转角（度）。"""
    from scipy.spatial.transform import Rotation

    return float(
        np.degrees(
            (Rotation.from_matrix(np.asarray(a)[:3, :3]).inv()
             * Rotation.from_matrix(np.asarray(b)[:3, :3])).magnitude()
        )
    )


def decompose_tlr_error(
    left_est: np.ndarray,
    right_est: np.ndarray,
    gt_left: np.ndarray,
    gt_right: np.ndarray,
    gt_translation: Tuple[float, float, float] = TLR_GT_TRANSLATION,
) -> dict:
    """GPT P1 误差分解：组合平移误差分别来自旋转残差还是平移残差。

    对 (T_LC, T_RC) 的估计与 GT 构造五种组合并求 T_LR 平移误差（mm）：

    - full        : 完整估计（两臂估计平移 + 旋转）
    - rot_only    : 保留估计旋转、两臂平移换 GT → 隔离"旋转残差 × ~1m 杠杆臂"
    - trans_only  : 保留估计平移、两臂旋转换 GT → 隔离平移残差
    - left_only   : 左臂估计 + 右臂 GT
    - right_only  : 左臂 GT + 右臂估计

    M2.7 实测（16 对）：rot_only 5.76 ≥ full 5.52，trans_only 2.37 < 3mm
    → 组合 5-7mm 由每臂 ~0.3-0.5° 旋转残差经 ~1m 相机杠杆放大主导。
    """
    left_est = np.asarray(left_est, dtype=float)
    right_est = np.asarray(right_est, dtype=float)
    gt_left = np.asarray(gt_left, dtype=float)
    gt_right = np.asarray(gt_right, dtype=float)

    def _err(L, Rc):
        return translation_error_mm(t_lr_cam(L, Rc), gt_translation)

    # rot_only: 保留估计旋转，平移换 GT
    L_rot = left_est.copy()
    L_rot[:3, 3] = gt_left[:3, 3]
    R_rot = right_est.copy()
    R_rot[:3, 3] = gt_right[:3, 3]
    # trans_only: 保留估计平移，旋转换 GT
    L_t = left_est.copy()
    L_t[:3, :3] = gt_left[:3, :3]
    R_t = right_est.copy()
    R_t[:3, :3] = gt_right[:3, :3]

    return {
        "full": _err(left_est, right_est),
        "rot_only": _err(L_rot, R_rot),
        "trans_only": _err(L_t, R_t),
        "left_only": _err(left_est, gt_right),
        "right_only": _err(gt_left, right_est),
    }
