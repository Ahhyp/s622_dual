#!/usr/bin/env python3
"""
Independent constant-frame hand-eye calibration evaluation node (M2.2).

Replaces the old ``evaluate_calibration.py`` whose eye-in-hand branch built
residuals as zero (``T_base_mrk_pred = T_base_mrk_obs``).  The new node uses
the constant-frame hold-out evaluator in ``hand_eye_calibration.evaluator``:

  * eye-in-hand:  {}^{B}T_{M,i} = {}^{B}T_{E,i} · {}^{E}T_{C} · {}^{C}T_{M,i}
                  must be constant → marker position RMS/P95/MAX + rotation RMS/P95/MAX
  * eye-on-base:  {}^{E}T_{M,i} = {}^{E}T_{B,i} · {}^{B}T_{C} · {}^{C}T_{M,i}
                  must be constant → same metrics

Workflow:
  1) Wait for the TF tree (base, effector, camera, marker frames).
  2) Interactively move the robot to N different poses; at each pose record
     the robot FK and the ArUco marker pose in the camera (from TF).
  3) Evaluation:
       - if ``calibration_file`` is set: score the freshly collected samples
         against that saved hand-eye (fresh-data constant-frame
         self-consistency — NOT a strict hold-out score, because the reference
         frame is derived from these very samples);
       - otherwise: split the collected samples into solve / hold-out subsets,
         solve the hand-eye on the solve subset only (full production gates),
         and score the hold-out subset against the solve-set reference frame.
  4) Report metrics + solver gate + M2.3-B hold-out acceptance gates.

Run (real hardware / simulation with TF):
  ros2 run hand_eye_calibration evaluate_calibration.py --ros-args \
    -p calibration_type:=eye_in_hand \
    -p sample_count:=20 -p solve_count:=15
  # or evaluate against a saved calibration:
  ros2 run hand_eye_calibration evaluate_calibration.py --ros-args \
    -p calibration_file:=$HOME/my_S622/src/hand_eye_calibration/calib/sim/robot_calibration_XXXX.calib
"""
import sys
from pathlib import Path
from typing import Optional

import numpy as np
import rclpy
import tf2_ros
from rclpy.node import Node
from rclpy.time import Time
from scipy.spatial.transform import Rotation as R

try:
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from hand_eye_calibration import evaluator
    from hand_eye_calibration.config import CalibrationType, normalize_calibration_type
    from hand_eye_calibration.solver import CalibrationSample, TransformMatrix
except ImportError as exc:  # pragma: no cover
    print(f"cannot import hand_eye_calibration: {exc}", file=sys.stderr)
    raise


def _matrix_from_tf(transform) -> np.ndarray:
    """Convert geometry_msgs/Transform (or TransformStamped.transform) to 4x4 matrix.

    Quaternion order: geometry_msgs/Quaternion and SciPy ``R.from_quat`` are
    both (x, y, z, w) — see ``evaluator.quat_xyzw_to_matrix``.
    """
    matrix = evaluator.quat_xyzw_to_matrix(
        transform.rotation.x,
        transform.rotation.y,
        transform.rotation.z,
        transform.rotation.w,
    )
    matrix[0, 3] = transform.translation.x
    matrix[1, 3] = transform.translation.y
    matrix[2, 3] = transform.translation.z
    return matrix


def _transform_from_matrix(matrix: np.ndarray) -> TransformMatrix:
    return TransformMatrix(R.from_matrix(matrix[:3, :3]), tuple(float(v) for v in matrix[:3, 3]))


class EvaluateCalibration(Node):
    def __init__(self):
        super().__init__("evaluate_calibration")

        self.calibration_type = normalize_calibration_type(
            self.declare_parameter("calibration_type", "eye_in_hand")
            .get_parameter_value().string_value
        )
        self.robot_base_frame = (
            self.declare_parameter("robot_base_frame", "base_link")
            .get_parameter_value().string_value
        )
        self.robot_effector_frame = (
            self.declare_parameter("robot_effector_frame", "tool0")
            .get_parameter_value().string_value
        )
        self.tracking_base_frame = (
            self.declare_parameter("tracking_base_frame", "camera_color_optical_frame")
            .get_parameter_value().string_value
        )
        self.tracking_marker_frame = (
            self.declare_parameter("tracking_marker_frame", "calibration_aruco")
            .get_parameter_value().string_value
        )
        self.calibration_file = (
            self.declare_parameter("calibration_file", "")
            .get_parameter_value().string_value
        )
        self.sample_count = (
            self.declare_parameter("sample_count", 20)
            .get_parameter_value().integer_value
        )
        self.solve_count = (
            self.declare_parameter("solve_count", 15)
            .get_parameter_value().integer_value
        )
        if self.sample_count < 4:
            raise RuntimeError("sample_count must be at least 4")
        if self.solve_count < 3:
            raise RuntimeError("solve_count must be at least 3")
        if self.solve_count >= self.sample_count:
            raise RuntimeError(
                f"solve_count ({self.solve_count}) must be < sample_count ({self.sample_count}) "
                "so at least one hold-out sample remains"
            )

        self.saved_handeye: Optional[TransformMatrix] = None
        if self.calibration_file:
            path = Path(self.calibration_file)
            if not path.exists():
                raise RuntimeError(f"calibration_file not found: {path}")
            calib = evaluator.load_calibration_yaml(path)
            if calib.calibration_type is not self.calibration_type:
                raise RuntimeError(
                    f"calibration_file type {calib.calibration_type.value} does not match "
                    f"calibration_type {self.calibration_type.value}"
                )
            self.saved_handeye = calib.transform
            self.get_logger().info(
                f"Loaded saved calibration from {path} "
                "(fresh-data self-consistency check, not strict hold-out)"
            )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self._check_tf_ready()

        self.samples: list[CalibrationSample] = []
        self._sample_idx = 0

        if self.saved_handeye is None:
            self.get_logger().info(
                "=" * 60 + "\n"
                "  独立标定评估（constant-frame hold-out）\n"
                f"  模式: {self.calibration_type.value}\n"
                f"  采样点数: {self.sample_count}  solve_count: {self.solve_count}\n"
                "  split strategy: deterministic_spread（按采集序列均匀散布 hold-out）\n"
                "  操作: 移动机械臂到不同位姿 → 终端输入 's' 采集 → 输入 'q' 退出\n" +
                "=" * 60
            )
        else:
            self.get_logger().info(
                "=" * 60 + "\n"
                "  独立标定评估（fresh-data constant-frame self-consistency）\n"
                f"  模式: {self.calibration_type.value}  对已存标定打分\n"
                f"  采样点数: {self.sample_count}\n"
                "  操作: 移动机械臂到不同位姿 → 终端输入 's' 采集 → 输入 'q' 退出\n" +
                "=" * 60
            )

        self.create_timer(0.5, self._prompt_loop)

    def _check_tf_ready(self):
        required = {
            self.robot_base_frame,
            self.robot_effector_frame,
            self.tracking_base_frame,
            self.tracking_marker_frame,
        }
        self.get_logger().info(f"Waiting for TF frames: {required}")
        while rclpy.ok():
            missing = set()
            for frame in required:
                try:
                    self.tf_buffer.lookup_transform(self.robot_base_frame, frame, Time())
                except Exception:
                    missing.add(frame)
            if not missing:
                break
            self.get_logger().info(
                f"  waiting... missing={missing}", throttle_duration_sec=2.0)
            rclpy.spin_once(self, timeout_sec=0.5)
        self.get_logger().info("All TF frames ready.")

    def _prompt_loop(self):
        if self._sample_idx >= self.sample_count:
            self._print_summary()
            raise SystemExit

        with open("/dev/tty", "r") as tty:
            sys.stderr.write(
                f"\n[{self._sample_idx + 1}/{self.sample_count}] "
                "移动机械臂到新位姿后输入 s 采集，或 q 退出: "
            )
            sys.stderr.flush()
            ch = tty.readline().strip().lower()
        if ch == "q":
            self._print_summary()
            raise SystemExit
        if ch != "s":
            return

        if self._collect_sample():
            self._sample_idx += 1

    def _collect_sample(self) -> bool:
        """Record one sample.  Returns True only if the sample was recorded."""
        try:
            eff_tf = self.tf_buffer.lookup_transform(
                self.robot_base_frame, self.robot_effector_frame, Time())
        except Exception as exc:
            self.get_logger().error(f"Failed to get robot FK: {exc}")
            return False

        try:
            mrk_tf = self.tf_buffer.lookup_transform(
                self.tracking_base_frame, self.tracking_marker_frame, Time())
        except Exception as exc:
            self.get_logger().error(f"Failed to get marker TF: {exc}")
            return False

        base_T_ee = _matrix_from_tf(eff_tf.transform)
        cam_T_mrk = _matrix_from_tf(mrk_tf.transform)

        if self.calibration_type is CalibrationType.EYE_IN_HAND:
            robot_pose = _transform_from_matrix(base_T_ee)
        else:  # eye_on_base: store ee_T_base
            robot_pose = _transform_from_matrix(np.linalg.inv(base_T_ee))

        sample = CalibrationSample(
            waypoint_index=self._sample_idx + 1,
            target_joints_deg=(0.0,) * 6,
            robot_pose=robot_pose,
            tracking_pose=_transform_from_matrix(cam_T_mrk),
        )
        self.samples.append(sample)
        self.get_logger().info(
            f"  样本 {self._sample_idx + 1}: "
            f"ee=({base_T_ee[0, 3]:.3f}, {base_T_ee[1, 3]:.3f}, {base_T_ee[2, 3]:.3f}) "
            f"marker_in_cam=({cam_T_mrk[0, 3]:.3f}, {cam_T_mrk[1, 3]:.3f}, {cam_T_mrk[2, 3]:.3f})"
        )
        return True

    def _print_summary(self):
        if not self.samples:
            self.get_logger().warn("无样本数据，退出")
            return

        if self.saved_handeye is not None:
            metrics = evaluator.constant_frame_metrics(self.samples, self.saved_handeye)
            self.get_logger().info(
                "\n" + "=" * 60 + "\n"
                "  对已存标定的评估（fresh-data constant-frame self-consistency）\n"
                f"  样本数: {len(self.samples)}\n"
                f"  位置 RMS/P95/MAX = {metrics['position_rms_m']*1000:.2f} / "
                f"{metrics['position_p95_m']*1000:.2f} / {metrics['position_max_m']*1000:.2f} mm\n"
                f"  旋转 RMS/P95/MAX = {metrics['rotation_rms_deg']:.3f} / "
                f"{metrics['rotation_p95_deg']:.3f} / {metrics['rotation_max_deg']:.3f} deg\n"
                "  注：参考帧由本次采集样本自身导出，属自洽性检查（非严格 hold-out）\n" +
                "=" * 60
            )
            return

        result = evaluator.evaluate_samples(
            self.samples,
            self.calibration_type,
            solve_count=self.solve_count,
        )
        self.get_logger().info("\n" + result.format_report())

        for index, position in enumerate(result.holdout_metrics["per_sample_position_m"]):
            self.get_logger().info(
                f"  hold-out[{index}] pos={position*1000:5.1f}mm "
                f"rot={result.holdout_metrics['per_sample_rotation_deg'][index]:6.4f}°"
            )


def main(args=None):
    rclpy.init(args=args)
    node = EvaluateCalibration()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
