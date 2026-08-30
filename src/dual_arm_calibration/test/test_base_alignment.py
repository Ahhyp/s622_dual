import tempfile
import unittest
from pathlib import Path

import numpy as np

from dual_arm_calibration import base_alignment as ba
from dual_arm_calibration import monte_carlo as mc


def _random_transform(rng):
    from scipy.spatial.transform import Rotation as R
    matrix = np.eye(4, dtype=float)
    matrix[:3, :3] = R.random(random_state=rng).as_matrix()
    matrix[:3, 3] = rng.uniform(-0.5, 0.5, size=3)
    return matrix


class KabschTests(unittest.TestCase):
    def test_exact_recovery_no_noise(self):
        rng = np.random.default_rng(0)
        truth = mc.dual_base_ground_truth()
        left = mc.fixture_points_wide(12, rng=rng)
        right = ba.apply_transform(left, np.linalg.inv(truth))
        estimate = ba.kabsch_align(left, right)
        t_err, r_err = ba.transform_error(estimate, truth)
        self.assertLess(t_err, 1e-9)
        self.assertLess(r_err, 1e-9)

    def test_known_gt_values(self):
        truth = mc.dual_base_ground_truth()
        np.testing.assert_allclose(truth[:3, 3], [0.7, 0.0, 0.0], atol=1e-12)
        np.testing.assert_allclose(truth[:3, :3], np.diag([-1.0, -1.0, 1.0]), atol=1e-12)

    def test_hardcoded_direction_gt(self):
        """Hard-coded points (not generated via apply_transform) catch T_LR vs T_RL."""
        right = np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, 1]], dtype=float)
        # Under Rz(180°)+t=(0.7,0,0): p_L = diag(-1,-1,1)·p_R + (0.7,0,0)
        left = np.array([[0.7, 0.0, 0.0], [-0.3, 0.0, 0.0], [0.7, -1.0, 0.0], [0.7, 0.0, 1.0]], dtype=float)
        estimate = ba.kabsch_align(left, right)
        t_err, r_err = ba.transform_error(estimate, mc.dual_base_ground_truth())
        self.assertLess(t_err, 1e-9)
        self.assertLess(r_err, 1e-9)

    def test_rigid_is_enforced(self):
        rng = np.random.default_rng(3)
        truth = _random_transform(rng)
        left = rng.uniform(-0.4, 0.4, size=(10, 3))
        right = ba.apply_transform(left, np.linalg.inv(truth))
        estimate = ba.kabsch_align(left, right)
        self.assertGreater(np.linalg.det(estimate[:3, :3]), 0.0)
        np.testing.assert_allclose(estimate, truth, atol=1e-9)

    def test_reflection_is_corrected_to_se3(self):
        """A mirrored point set must NOT yield a reflection: det=+1 enforced."""
        rng = np.random.default_rng(7)
        base = rng.uniform(0.05, 0.4, size=(8, 3))
        mirrored = base.copy()
        mirrored[:, 0] *= -1.0  # reflection through the YZ plane
        estimate = ba.kabsch_align(mirrored, base)
        self.assertGreater(np.linalg.det(estimate[:3, :3]), 0.0)

    def test_collinear_points_rejected(self):
        collinear = np.array([[0, 0, 0], [1, 0, 0], [2, 0, 0], [3, 0, 0]], dtype=float)
        with self.assertRaises(ValueError):
            ba.kabsch_align(collinear, collinear)

    def test_near_collinear_rejected_by_condition(self):
        near = np.array([[0, 0, 0], [1, 0, 1e-9], [2, 0, 0], [3, 0, 0]], dtype=float)
        with self.assertRaises(ValueError):
            ba.kabsch_align(near, near)

    def test_coplanar_but_not_collinear_is_solvable(self):
        # rank-2 set (all z=0) is solvable for a proper rigid transform
        rng = np.random.default_rng(11)
        truth = _random_transform(rng)
        left = np.column_stack([rng.uniform(-0.3, 0.3, 10), rng.uniform(-0.3, 0.3, 10), np.zeros(10)])
        right = ba.apply_transform(left, np.linalg.inv(truth))
        estimate = ba.kabsch_align(left, right)
        t_err, r_err = ba.transform_error(estimate, truth)
        self.assertLess(t_err, 1e-8)
        self.assertLess(r_err, 1e-4)  # coplanar rank-2: slight numerical slack

    def test_geometry_condition_reports_ratios(self):
        rng = np.random.default_rng(2)
        points = rng.uniform(-0.3, 0.3, size=(10, 3))
        cond = ba.geometry_condition(points)
        self.assertIn("sigma2_over_sigma1", cond)
        self.assertIn("sigma3_over_sigma1", cond)
        self.assertGreater(cond["sigma2_over_sigma1"], 0.01)

    def test_noise_degrades_estimate(self):
        rng = np.random.default_rng(5)
        truth = mc.dual_base_ground_truth()
        left = mc.fixture_points_wide(20, rng=rng)
        right = ba.apply_transform(left, np.linalg.inv(truth))
        clean = ba.kabsch_align(left, right)
        noisy_left = left + rng.normal(0, 0.002, size=left.shape)
        noisy = ba.kabsch_align(noisy_left, right)
        clean_err = ba.transform_error(clean, truth)[0]
        noisy_err = ba.transform_error(noisy, truth)[0]
        self.assertGreater(noisy_err, clean_err)

    def test_requires_three_points(self):
        with self.assertRaises(ValueError):
            ba.kabsch_align(np.zeros((2, 3)), np.zeros((2, 3)))

    def test_rejects_nonfinite(self):
        bad = np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0], [np.nan, 0, 0]], dtype=float)
        with self.assertRaises(ValueError):
            ba.kabsch_align(bad, bad)

    def test_alignment_error_metrics(self):
        rng = np.random.default_rng(1)
        left = rng.uniform(-0.3, 0.3, size=(8, 3))
        right = left.copy()
        metrics = ba.alignment_error(left, right, np.eye(4))
        self.assertLess(metrics["rms_m"], 1e-12)
        self.assertEqual(metrics["p95_m"], metrics["max_m"])

    def test_fit_holdout_split(self):
        rng = np.random.default_rng(2)
        truth = mc.dual_base_ground_truth()
        left = mc.fixture_points_wide(20, rng=rng)
        right = ba.apply_transform(left, np.linalg.inv(truth))
        result = ba.fit_holdout_split(left, right, 15, rng=rng)
        self.assertEqual(result["fit_count"], 15)
        self.assertEqual(result["holdout_count"], 5)
        self.assertLess(result["fit_rms_mm"], 1e-6)
        self.assertLess(result["holdout_rms_mm"], 1e-6)

    def test_fit_holdout_requires_three(self):
        rng = np.random.default_rng(2)
        left = np.zeros((5, 3))
        with self.assertRaises(ValueError):
            ba.fit_holdout_split(left, left, 2, rng=rng)


class MonteCarloTests(unittest.TestCase):
    def test_gt_transform_recovered_with_zero_noise(self):
        noise = mc.NoiseModel(touch_axis_sigma_m=0.0, tcp_axis_sigma_m=0.0, fk_axis_sigma_m=0.0)
        rng = np.random.default_rng(0)
        truth = mc.dual_base_ground_truth()
        left = mc.fixture_points_wide(20, rng=rng)
        right = ba.apply_transform(left, np.linalg.inv(truth))
        results = mc.run_trials(left, right, noise, trials=20, rng=rng)
        summary = mc.summarize(results)
        self.assertLess(summary["translation_p95_m"], 1e-9)
        self.assertLess(summary["rotation_p95_deg"], 1e-4)  # SVD numerical noise

    def test_tcp_is_systematic_within_trial(self):
        """TCP bias must be identical for every point of the same arm in one trial."""
        noise = mc.NoiseModel(touch_axis_sigma_m=0.0, tcp_axis_sigma_m=0.001, fk_axis_sigma_m=0.0)
        rng = np.random.default_rng(0)
        left_true = rng.uniform(-0.3, 0.3, size=(10, 3))
        right_true = rng.uniform(-0.3, 0.3, size=(10, 3))
        left_meas, right_meas = noise.measure(left_true, right_true, rng)
        left_offsets = left_meas - left_true
        right_offsets = right_meas - right_true
        # every row of the same arm must be the identical offset vector
        self.assertTrue(np.allclose(left_offsets, left_offsets[0][None, :], atol=1e-12))
        self.assertTrue(np.allclose(right_offsets, right_offsets[0][None, :], atol=1e-12))
        self.assertGreater(np.linalg.norm(left_offsets[0]), 0.0)

    def test_systematic_tcp_error_does_not_average_out(self):
        noise = mc.NoiseModel(touch_axis_sigma_m=0.0, tcp_axis_sigma_m=0.001, fk_axis_sigma_m=0.0)
        rng = np.random.default_rng(0)
        truth = mc.dual_base_ground_truth()
        left = mc.fixture_points_wide(100, rng=rng)
        right = ba.apply_transform(left, np.linalg.inv(truth))
        results = mc.run_trials(left, right, noise, trials=200, rng=rng)
        summary = mc.summarize(results)
        self.assertGreater(summary["translation_p95_m"], 0.0005)

    def test_low_residual_but_high_gt_error(self):
        """Internal consistency does NOT detect systematic TCP bias (adversarial)."""
        noise = mc.NoiseModel(touch_axis_sigma_m=0.0, tcp_axis_sigma_m=0.002, fk_axis_sigma_m=0.0)
        rng = np.random.default_rng(4)
        truth = mc.dual_base_ground_truth()
        left = mc.fixture_points_wide(20, rng=rng)
        right = ba.apply_transform(left, np.linalg.inv(truth))
        left_meas, right_meas = noise.measure(left, right, rng)
        estimate = ba.kabsch_align(left_meas, right_meas)
        internal = ba.alignment_error(left_meas, right_meas, estimate)
        gt_t_err, _ = ba.transform_error(estimate, truth)
        self.assertLess(internal["rms_m"], 0.0005)     # internally consistent
        self.assertGreater(gt_t_err, 0.0005)           # but absolutely wrong

    def test_nested_scan_is_paired_and_monotonic(self):
        noise = mc.NoiseModel(tcp_axis_sigma_m=0.0)  # only touch/FK → more pts helps
        fixture = mc.design_wide_fixture(20, seed=0)
        result = mc.nested_scan(fixture, (4, 10, 20), trials=400, noise=noise, seed=1)
        rows = result["rows"]
        self.assertEqual([r["sample_count"] for r in rows], [4, 10, 20])
        # with random-only noise, adding points should not make P95 worse by much
        p95_4 = rows[0]["translation_p95_m"]
        p95_20 = rows[2]["translation_p95_m"]
        self.assertLessEqual(p95_20 * 1.05, p95_4 * 2.0)

    def test_compare_coverage_wide_beats_small(self):
        noise = mc.NoiseModel(tcp_axis_sigma_m=0.0)
        comp = mc.compare_coverage((4, 10), trials=300, noise=noise, seed=3)
        wide_p95 = comp["wide"]["rows"][0]["translation_p95_m"]
        small_p95 = comp["small"]["rows"][0]["translation_p95_m"]
        self.assertLess(wide_p95, small_p95)

    def test_handover_point_error_amplifies_rotation(self):
        """A 0.5° rotation at ~0.5 m distance → several mm of point error."""
        truth = mc.dual_base_ground_truth()
        perturbed = truth.copy()
        perturbed[:3, :3] = (np.eye(3) + np.array([[0, -0.00873, 0], [0.00873, 0, 0], [0, 0, 0]])) @ truth[:3, :3]
        mapped = ba.apply_transform(mc.HANDOVER_POINTS, perturbed)
        expected = ba.apply_transform(mc.HANDOVER_POINTS, truth)
        errs = np.linalg.norm(mapped - expected, axis=1)
        self.assertGreater(np.max(errs), 0.003)  # > 3 mm induced at ~0.5 m

    def test_gt_consistency_with_urdf(self):
        truth = mc.dual_base_ground_truth()
        origin_right = np.array([0.0, 0.0, 0.0])
        origin_left = ba.apply_transform(np.array([origin_right]), truth)[0]
        np.testing.assert_allclose(origin_left, [0.7, 0.0, 0.0], atol=1e-12)

    def test_fixture_design_has_good_conditioning(self):
        fixture = mc.design_wide_fixture(20, seed=0)
        cond = ba.geometry_condition(fixture)
        self.assertGreater(cond["sigma2_over_sigma1"], 0.1)
        self.assertGreater(cond["sigma3_over_sigma1"], 0.01)


if __name__ == "__main__":
    unittest.main()
