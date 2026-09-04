#!/usr/bin/env python3
"""[M2.7-H] 从左右全局相机标定结果推导双基座关系（方法 B）。

T_L_R_cam = {}^{B_L}T_{C_g} · ({}^{B_R}T_{C_g})^{-1}

仿真中与 URDF GT（dual_base_ground_truth：Rz(180°)+t=(0.7,0,0)）比较，
输出平移差/旋转差 + JSON。M2.8 将用它与机械法 touch 结果交叉验证。

用法：
  python3 scripts/derive_dual_base_from_camera.py \
      --left  calib/global_eye_on_base/left/robot_calibration_XXX.calib \
      --right calib/global_eye_on_base/right/robot_calibration_XXX.calib \
      [--json /tmp/dual_base_camera.json]
"""

import argparse
import json
import sys
from pathlib import Path

import numpy as np

try:
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from hand_eye_calibration import evaluator
    from hand_eye_calibration.dual_base import rotation_delta_deg, t_lr_cam
except ImportError as exc:  # pragma: no cover
    print(f"cannot import hand_eye_calibration: {exc}", file=sys.stderr)
    sys.exit(2)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--left", required=True, help="left-arm .calib (B_L_T_Cg)")
    parser.add_argument("--right", required=True, help="right-arm .calib (B_R_T_Cg)")
    parser.add_argument("--json", default=None, help="write JSON report")
    args = parser.parse_args(argv)

    left = evaluator.load_calibration_yaml(args.left)
    right = evaluator.load_calibration_yaml(args.right)
    if left.calibration_type is not right.calibration_type:
        print("error: left/right calibration types differ", file=sys.stderr)
        return 2

    T_L_C = left.transform.matrix()
    T_R_C = right.transform.matrix()
    # [M2.7 回归保护] 组合方向由 hand_eye_calibration.dual_base.t_lr_cam 统一提供
    # （= T_LC @ inv(T_RC)），有 test_m27_regression::test_tlr_direction 守护，禁止写反。
    T_L_R_cam = t_lr_cam(T_L_C, T_R_C)

    # GT from URDF（dual_arm_calibration.monte_carlo 提供）
    from dual_arm_calibration.monte_carlo import dual_base_ground_truth
    T_L_R_gt = dual_base_ground_truth()

    delta = np.linalg.inv(T_L_R_cam) @ T_L_R_gt
    trans_diff_m = float(np.linalg.norm(delta[:3, 3]))
    rot_diff_deg = rotation_delta_deg(T_L_R_cam[:3, :3], T_L_R_gt[:3, :3])

    print("=" * 62)
    print("  M2.7-H dual base from global camera (method B)")
    print(f"  T_L_R_cam = T_L_C · inv(T_R_C)")
    print(f"    translation = ({T_L_R_cam[0,3]:.4f}, {T_L_R_cam[1,3]:.4f}, {T_L_R_cam[2,3]:.4f})")
    print(f"  vs URDF GT  translation = (0.7000, 0.0000, 0.0000)")
    print(f"  translation diff = {trans_diff_m*1000:.2f} mm")
    print(f"  rotation   diff = {rot_diff_deg:.3f} deg")
    print("  M2.7 acceptance: translation <= 3 mm, rotation <= 1 deg",
          "-> PASS" if trans_diff_m <= 0.003 and rot_diff_deg <= 1.0 else "-> FAIL")
    print("=" * 62)

    if args.json:
        Path(args.json).write_text(json.dumps({
            "T_L_R_cam_translation": [float(v) for v in T_L_R_cam[:3, 3]],
            "T_L_R_cam_rotation_matrix": [[float(v) for v in row] for row in T_L_R_cam[:3, :3]],
            "translation_diff_m": trans_diff_m,
            "rotation_diff_deg": rot_diff_deg,
            "passed": bool(trans_diff_m <= 0.003 and rot_diff_deg <= 1.0),
            "left_calib": args.left,
            "right_calib": args.right,
        }, indent=2))
        print(f"report written: {args.json}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
