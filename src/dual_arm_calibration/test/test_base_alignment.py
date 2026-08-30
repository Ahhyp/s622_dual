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

    def test_rigid_is_enforced(self):
        # random transform recovery with reflection correction
        rng = np.random.default_rng(3)
        truth = _random_transform(rng)
        left = rng.uniform(-0.4, 0.4, size=(10, 3))
        right = ba.apply_transform(left, np.linalg.inv(truth))
        estimate = ba.kabsch_align(left, right)
        self.assertGreater(np.linalg.det(estimate[:3, :3]), 0.0)
        np.testing.assert_allclose(estimate, truth, atol=1e-9)

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

    def test_alignment_error_metrics(self):
        rng = np.random.default_rng(1)
        left = rng.uniform(-0.3, 0.3, size=(8, 3))
        right = left.copy()
        metrics = ba.alignment_error(left, right, np.eye(4))
        self.assertLess(metrics["rms_m"], 1e-12)
        self.assertEqual(metrics["p95_m"], metrics["max_m"])  # all residuals ~0

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


class MonteCarloTests(unittest.TestCase):
    def test_gt_transform_recovered_with_zero_noise(self):
        noise = mc.NoiseModel(touch_sigma_m=0.0, tcp_error_sigma_m=0.0, fk_error_sigma_m=0.0)
        rng = np.random.default_rng(0)
        truth = mc.dual_base_ground_truth()
        left = mc.fixture_points_wide(20, rng=rng)
        right = ba.apply_transform(left, np.linalg.inv(truth))
        results = mc.run_trials(left, right, noise, trials=20, rng=rng)
        summary = mc.summarize(results)
        self.assertLess(summary["translation_p95_m"], 1e-9)
        self.assertLess(summary["rotation_p95_deg"], 1e-4)  # SVD numerical noise

    def test_systematic_tcp_error_does_not_average_out(self):
        """TCP error is per-arm systematic → more points must NOT reduce P95 to 0."""
        noise = mc.NoiseModel(touch_sigma_m=0.0, tcp_error_sigma_m=0.001, fk_error_sigma_m=0.0)
        rng = np.random.default_rng(0)
        truth = mc.dual_base_ground_truth()
        left = mc.fixture_points_wide(100, rng=rng)
        right = ba.apply_transform(left, np.linalg.inv(truth))
        results = mc.run_trials(left, right, noise, trials=200, rng=rng)
        summary = mc.summarize(results)
        # 1mm systematic per-arm TCP cannot vanish — expect P95 well above 1e-3 m
        self.assertGreater(summary["translation_p95_m"], 0.0005)

    def test_scan_returns_rows(self):
        noise = mc.NoiseModel()
        result = mc.scan_sample_counts((4, 10, 20), trials=30, noise=noise, seed=1, wide=True)
        self.assertEqual(len(result["rows"]), 3)
        self.assertEqual([r["sample_count"] for r in result["rows"]], [4, 10, 20])
        for row in result["rows"]:
            self.assertIn("translation_p95_m", row)
            self.assertIn("rotation_p95_deg", row)

    def test_wide_helps_at_low_count(self):
        """With touch noise only, wide coverage should beat the small cluster."""
        noise = mc.NoiseModel(tcp_error_sigma_m=0.0, fk_error_sigma_m=0.0)
        comp = mc.compare_coverage((4,), trials=300, noise=noise, seed=3)
        wide_p95 = comp["wide"]["rows"][0]["translation_p95_m"]
        small_p95 = comp["small"]["rows"][0]["translation_p95_m"]
        self.assertLess(wide_p95, small_p95)

    def test_gt_consistency_with_urdf(self):
        # sanity: dual base separation is 0.7 m along x in left-base frame
        truth = mc.dual_base_ground_truth()
        origin_right = np.array([0.0, 0.0, 0.0])
        origin_left = ba.apply_transform(np.array([origin_right]), truth)[0]
        np.testing.assert_allclose(origin_left, [0.7, 0.0, 0.0], atol=1e-12)


if __name__ == "__main__":
    unittest.main()
