from pathlib import Path
import tempfile
import unittest

import numpy as np
import yaml
from scipy.spatial.transform import Rotation as R

from hand_eye_calibration import evaluator
from hand_eye_calibration.config import CalibrationType
from hand_eye_calibration.solver import CalibrationSample, TransformMatrix


def _synthetic_eih_samples(count, handeye, marker_pose, *, noise_m=0.0, noise_deg=0.0, seed=7):
    """Eye-in-hand synthetic samples: marker fixed in base, camera on effector."""
    rng = np.random.default_rng(seed)
    samples = []
    for index in range(count):
        # diverse robot poses
        base_T_ee = TransformMatrix(
            R.from_euler("xyz", (index * 5.0, -index * 3.0, index * 7.0), degrees=True),
            (0.30 + 0.01 * (index % 5), -0.10 + 0.008 * (index % 4), 0.45 + 0.006 * (index % 3)),
        )
        cam_T_mrk = np.linalg.inv(base_T_ee.matrix() @ handeye.matrix()) @ marker_pose.matrix()
        if noise_m > 0:
            cam_T_mrk[:3, 3] += rng.normal(0.0, noise_m, 3)
        if noise_deg > 0:
            rot = R.from_matrix(cam_T_mrk[:3, :3])
            cam_T_mrk[:3, :3] = (rot * R.from_rotvec(
                rng.normal(0.0, np.deg2rad(noise_deg), 3))).as_matrix()
        samples.append(CalibrationSample(
            index + 1, (0.0,) * 6, base_T_ee, TransformMatrix(
                R.from_matrix(cam_T_mrk[:3, :3]), tuple(float(v) for v in cam_T_mrk[:3, 3])),
        ))
    return samples


def _synthetic_eob_samples(count, handeye, ee_T_marker, *, noise_m=0.0, noise_deg=0.0, seed=7):
    """Eye-on-base synthetic samples: marker fixed on effector, camera fixed in base."""
    rng = np.random.default_rng(seed)
    samples = []
    for index in range(count):
        base_T_ee = TransformMatrix(
            R.from_euler("xyz", (index * 5.0, -index * 3.0, index * 7.0), degrees=True),
            (0.30 + 0.01 * (index % 5), -0.10 + 0.008 * (index % 4), 0.45 + 0.006 * (index % 3)),
        )
        cam_T_mrk = np.linalg.inv(handeye.matrix()) @ base_T_ee.matrix() @ ee_T_marker.matrix()
        if noise_m > 0:
            cam_T_mrk[:3, 3] += rng.normal(0.0, noise_m, 3)
        if noise_deg > 0:
            rot = R.from_matrix(cam_T_mrk[:3, :3])
            cam_T_mrk[:3, :3] = (rot * R.from_rotvec(
                rng.normal(0.0, np.deg2rad(noise_deg), 3))).as_matrix()
        ee_T_base = np.linalg.inv(base_T_ee.matrix())
        samples.append(CalibrationSample(
            index + 1, (0.0,) * 6, TransformMatrix(
                R.from_matrix(ee_T_base[:3, :3]), tuple(float(v) for v in ee_T_base[:3, 3])),
            TransformMatrix(R.from_matrix(cam_T_mrk[:3, :3]), tuple(float(v) for v in cam_T_mrk[:3, 3])),
        ))
    return samples


class SplitTests(unittest.TestCase):
    def test_split_is_disjoint_and_exhaustive(self):
        samples = [CalibrationSample(i, (0,) * 6, None, None) for i in range(20)]
        solve, holdout = evaluator.split_solve_holdout(samples, 15, 5)
        self.assertEqual(len(solve), 15)
        self.assertEqual(len(holdout), 5)
        indices = {s.waypoint_index for s in solve} | {s.waypoint_index for s in holdout}
        self.assertEqual(indices, set(range(20)))

    def test_split_holds_cover_span(self):
        samples = [CalibrationSample(i, (0,) * 6, None, None) for i in range(20)]
        _, holdout = evaluator.split_solve_holdout(samples, 15, 5)
        waypoints = sorted(s.waypoint_index for s in holdout)
        self.assertEqual(waypoints[0], 0)  # first sample is held out
        self.assertGreater(waypoints[-1], 15)  # and a later one too
        self.assertEqual(len(waypoints), 5)

    def test_split_default_holdout_count(self):
        samples = [CalibrationSample(i, (0,) * 6, None, None) for i in range(19)]
        solve, holdout = evaluator.split_solve_holdout(samples, 15)
        self.assertEqual(len(solve), 15)
        self.assertEqual(len(holdout), 4)

    def test_split_validates_counts(self):
        samples = [CalibrationSample(i, (0,) * 6, None, None) for i in range(10)]
        with self.assertRaises(ValueError):
            evaluator.split_solve_holdout(samples, 2)  # < 3 solve samples
        with self.assertRaises(ValueError):
            evaluator.split_solve_holdout(samples, 15)  # holdout would be negative
        with self.assertRaises(ValueError):
            evaluator.split_solve_holdout(samples, 10, 0)  # zero hold-out samples

    def test_split_random_is_deterministic_with_seed(self):
        samples = [CalibrationSample(i, (0,) * 6, None, None) for i in range(20)]
        a = evaluator.split_solve_holdout(samples, 15, 5, rng=np.random.default_rng(42))
        b = evaluator.split_solve_holdout(samples, 15, 5, rng=np.random.default_rng(42))
        self.assertEqual(
            [s.waypoint_index for s in a[1]], [s.waypoint_index for s in b[1]])


class ConstantFrameMetricsTests(unittest.TestCase):
    def _handeye(self):
        return TransformMatrix(R.from_euler("xyz", (10.0, -20.0, 30.0), degrees=True), (0.0325, -0.0375, -0.0794))

    def test_noiseless_eih_is_constant(self):
        handeye = self._handeye()
        marker = TransformMatrix(R.from_euler("xyz", (15.0, -10.0, 25.0), degrees=True), (0.31, 0.27, 0.02))
        samples = _synthetic_eih_samples(12, handeye, marker)
        metrics = evaluator.constant_frame_metrics(samples, handeye)
        self.assertLess(metrics["position_rms_m"], 1e-9)
        self.assertLess(metrics["rotation_rms_deg"], 1e-6)

    def test_noiseless_eob_is_constant(self):
        handeye = TransformMatrix(R.from_euler("xyz", (10.0, -20.0, 30.0), degrees=True), (0.4, 0.2, 0.9))
        ee_T_marker = TransformMatrix(R.from_euler("xyz", (5.0, 10.0, -15.0), degrees=True), (0.03, -0.02, 0.12))
        samples = _synthetic_eob_samples(12, handeye, ee_T_marker)
        metrics = evaluator.constant_frame_metrics(samples, handeye)
        self.assertLess(metrics["position_rms_m"], 1e-9)
        self.assertLess(metrics["rotation_rms_deg"], 1e-6)

    def test_noise_shows_up_in_metrics(self):
        handeye = self._handeye()
        marker = TransformMatrix(R.from_euler("xyz", (15.0, -10.0, 25.0), degrees=True), (0.31, 0.27, 0.02))
        noisy = _synthetic_eih_samples(12, handeye, marker, noise_m=0.001, noise_deg=0.1)
        clean = _synthetic_eih_samples(12, handeye, marker)
        noisy_metrics = evaluator.constant_frame_metrics(noisy, handeye)
        clean_metrics = evaluator.constant_frame_metrics(clean, handeye)
        self.assertGreater(noisy_metrics["position_rms_m"], clean_metrics["position_rms_m"] + 1e-4)
        self.assertGreater(noisy_metrics["rotation_rms_deg"], clean_metrics["rotation_rms_deg"] + 1e-3)

    def test_wrong_handeye_direction_degrades_metrics(self):
        """A hand-eye direction error (ee_T_cam vs cam_T_ee) must be caught."""
        handeye = self._handeye()
        marker = TransformMatrix(R.from_euler("xyz", (15.0, -10.0, 25.0), degrees=True), (0.31, 0.27, 0.02))
        samples = _synthetic_eih_samples(12, handeye, marker)
        good = evaluator.constant_frame_metrics(samples, handeye)
        wrong = evaluator.constant_frame_metrics(
            samples, TransformMatrix(handeye.rotation.inv(), tuple(-v for v in handeye.translation)))
        self.assertGreater(wrong["position_rms_m"], good["position_rms_m"] * 10 + 1e-3)
        self.assertGreater(wrong["rotation_rms_deg"], 1.0)

    def test_empty_inputs_raise(self):
        with self.assertRaises(ValueError):
            evaluator.reference_pose([])
        handeye = self._handeye()
        with self.assertRaises(ValueError):
            evaluator.constant_frame_metrics([], handeye)


class EvaluateSamplesTests(unittest.TestCase):
    def test_holdout_metrics_are_small_on_clean_data(self):
        handeye = TransformMatrix(R.from_euler("xyz", (10.0, -20.0, 30.0), degrees=True), (0.0325, -0.0375, -0.0794))
        marker = TransformMatrix(R.from_euler("xyz", (15.0, -10.0, 25.0), degrees=True), (0.31, 0.27, 0.02))
        samples = _synthetic_eih_samples(19, handeye, marker)
        result = evaluator.evaluate_samples(samples, CalibrationType.EYE_IN_HAND, solve_count=15)
        self.assertEqual(result.solve_count, 15)
        self.assertEqual(result.holdout_count, 4)
        self.assertLess(result.holdout_metrics["position_rms_m"], 1e-6)
        self.assertLess(result.holdout_metrics["rotation_rms_deg"], 1e-3)
        self.assertTrue(result.solver_valid)
        self.assertTrue(result.passed_gates())

    def test_holdout_degrades_with_noise(self):
        handeye = TransformMatrix(R.from_euler("xyz", (10.0, -20.0, 30.0), degrees=True), (0.0325, -0.0375, -0.0794))
        marker = TransformMatrix(R.from_euler("xyz", (15.0, -10.0, 25.0), degrees=True), (0.31, 0.27, 0.02))
        clean = evaluator.evaluate_samples(
            _synthetic_eih_samples(19, handeye, marker),
            CalibrationType.EYE_IN_HAND, solve_count=15)
        noisy = evaluator.evaluate_samples(
            _synthetic_eih_samples(19, handeye, marker, noise_m=0.002, noise_deg=0.2),
            CalibrationType.EYE_IN_HAND, solve_count=15)
        self.assertGreater(
            noisy.holdout_metrics["position_rms_m"],
            clean.holdout_metrics["position_rms_m"] + 1e-4)
        self.assertGreater(
            noisy.holdout_metrics["rotation_rms_deg"],
            clean.holdout_metrics["rotation_rms_deg"] + 1e-3)

    def test_reference_leakage_is_detected(self):
        """Reference frame must come from the solve subset alone.

        Build the hold-out set with a marker shifted +10 mm in base X; the
        correct solve-set reference keeps hold-out RMS ~10 mm, while a leaky
        reference taken from the hold-out set itself collapses it to ~0.
        """
        handeye = TransformMatrix(R.from_euler("xyz", (10.0, -20.0, 30.0), degrees=True), (0.0325, -0.0375, -0.0794))
        marker = TransformMatrix(R.from_euler("xyz", (15.0, -10.0, 25.0), degrees=True), (0.31, 0.27, 0.02))
        shifted_marker = TransformMatrix(marker.rotation, (marker.translation[0] + 0.010, marker.translation[1], marker.translation[2]))
        solve = _synthetic_eih_samples(15, handeye, marker)
        holdout = _synthetic_eih_samples(6, handeye, shifted_marker, seed=99)
        solver_valid, est, _, _ = evaluator.solve_handeye(solve, CalibrationType.EYE_IN_HAND)
        self.assertTrue(solver_valid)
        reference = evaluator.reference_pose(
            [evaluator.constant_frame_pose(s, est) for s in solve])
        strict = evaluator.constant_frame_metrics(holdout, est, reference=reference)
        self.assertGreater(strict["position_rms_m"], 0.008)  # ~10 mm, not ~0
        leaky = evaluator.constant_frame_metrics(holdout, est)  # reference from holdout itself
        self.assertLess(leaky["position_rms_m"], 0.002)

    def test_eob_holdout_recovers_base_to_camera(self):
        handeye = TransformMatrix(R.from_euler("xyz", (10.0, -20.0, 30.0), degrees=True), (0.4, 0.2, 0.9))
        ee_T_marker = TransformMatrix(R.from_euler("xyz", (5.0, 10.0, -15.0), degrees=True), (0.03, -0.02, 0.12))
        samples = _synthetic_eob_samples(16, handeye, ee_T_marker)
        result = evaluator.evaluate_samples(samples, CalibrationType.EYE_ON_BASE, solve_count=12)
        np.testing.assert_allclose(
            result.handeye.translation, handeye.translation, atol=1e-8)
        self.assertLess(
            evaluator.rotation_delta_deg(result.handeye.rotation, handeye.rotation), 1e-6)
        self.assertLess(result.holdout_metrics["position_rms_m"], 1e-8)


class QuaternionTests(unittest.TestCase):
    def test_known_z90_quaternion(self):
        """(0, 0, sqrt(0.5), sqrt(0.5)) must be a +90° rotation about Z."""
        s = np.sqrt(0.5)
        matrix = evaluator.quat_xyzw_to_matrix(0.0, 0.0, s, s)
        expected = np.array([
            [0, -1, 0, 0],
            [1, 0, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1],
        ], dtype=float)
        np.testing.assert_allclose(matrix, expected, atol=1e-12)

    def test_identity_quaternion(self):
        matrix = evaluator.quat_xyzw_to_matrix(0.0, 0.0, 0.0, 1.0)
        np.testing.assert_allclose(matrix, np.eye(4), atol=1e-12)

    def test_x_y_z_w_roundtrip(self):
        """(x,y,z,w) order round-trips through scipy with no scalar-first flag."""
        rotation = R.from_euler("xyz", (15.0, -30.0, 45.0), degrees=True)
        x, y, z, w = rotation.as_quat()
        np.testing.assert_allclose(
            evaluator.quat_xyzw_to_matrix(x, y, z, w)[:3, :3], rotation.as_matrix(), atol=1e-12)


class YAMLTests(unittest.TestCase):
    def test_samples_yaml_roundtrip(self):
        handeye = TransformMatrix(R.from_euler("xyz", (10.0, -20.0, 30.0), degrees=True), (0.0325, -0.0375, -0.0794))
        marker = TransformMatrix(R.from_euler("xyz", (15.0, -10.0, 25.0), degrees=True), (0.31, 0.27, 0.02))
        samples = _synthetic_eih_samples(5, handeye, marker)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "dataset.samples"
            evaluator.save_samples_yaml(path, samples, CalibrationType.EYE_IN_HAND, status="test")
            kind, loaded = evaluator.load_samples_yaml(path)
        self.assertEqual(kind, CalibrationType.EYE_IN_HAND)
        self.assertEqual(len(loaded), 5)
        for original, restored in zip(samples, loaded):
            np.testing.assert_allclose(
                original.robot_pose.matrix(), restored.robot_pose.matrix(), atol=1e-12)
            np.testing.assert_allclose(
                original.tracking_pose.matrix(), restored.tracking_pose.matrix(), atol=1e-12)

    def test_calibration_yaml_roundtrip(self):
        handeye = TransformMatrix(R.from_euler("xyz", (10.0, -20.0, 30.0), degrees=True), (0.0325, -0.0375, -0.0794))
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "robot.calib"
            evaluator.save_calibration_yaml(path, handeye, CalibrationType.EYE_IN_HAND, name="test")
            loaded = evaluator.load_calibration_yaml(path)
        self.assertEqual(loaded.calibration_type, CalibrationType.EYE_IN_HAND)
        np.testing.assert_allclose(loaded.transform.matrix(), handeye.matrix(), atol=1e-12)

    def test_calibration_yaml_preserves_type(self):
        """save/load must carry the type (previously hard-coded to eye_in_hand)."""
        handeye = TransformMatrix(R.from_euler("xyz", (10.0, -20.0, 30.0), degrees=True), (0.4, 0.2, 0.9))
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "robot_eob.calib"
            evaluator.save_calibration_yaml(path, handeye, CalibrationType.EYE_ON_BASE, name="test")
            loaded = evaluator.load_calibration_yaml(path)
        self.assertEqual(loaded.calibration_type, CalibrationType.EYE_ON_BASE)

    def test_hardcoded_quaternion_fixture(self):
        """A hand-written YAML rotation block (x,y,z,w = 0,0,0.7071,0.7071) must be Z+90°."""
        s = 2.0 ** -0.5  # plain float, yaml-representable
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "fixture.calib"
            path.write_text(yaml.safe_dump({
                "parameters": {"calibration_type": "eye_in_hand"},
                "transform": {
                    "translation": {"x": 0.0, "y": 0.0, "z": 0.5},
                    "rotation": {"x": 0.0, "y": 0.0, "z": s, "w": s},
                },
            }), encoding="utf-8")
            loaded = evaluator.load_calibration_yaml(path)
        np.testing.assert_allclose(loaded.transform.translation, (0.0, 0.0, 0.5), atol=1e-12)
        expected = R.from_euler("z", 90.0, degrees=True).as_matrix()
        np.testing.assert_allclose(loaded.transform.rotation.as_matrix(), expected, atol=1e-9)


if __name__ == "__main__":
    unittest.main()
