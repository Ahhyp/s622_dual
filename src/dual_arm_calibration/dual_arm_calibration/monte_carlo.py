"""M2.6 dual-base Monte Carlo simulation (v2, after GPT review).

Simulates the M2.5 real-hardware protocol: both arms touch shared physical
points on a 3D fixture; each arm reports the point in its own base frame
(probe TCP + joint FK + touch repeatability), and Kabsch aligns the two point
sets into {}^{B_L}T_{B_R}.

v2 changes (GPT review 2026-08-31):
  1. Noise sigmas are explicitly **per-axis** Gaussian std (each of x/y/z),
     so a 1 mm/axis TCP produces a 3D RMS magnitude of sqrt(3) ≈ 1.73 mm.
     Parameters renamed ``*_axis_sigma_m`` to prevent misreading.
  2. **Nested/paired design**: one fixed WIDE fixture (20 pts) is designed
     once; every N uses a farthest-point-sampling *subset* of that same
     fixture, and all N share the **same noise realizations per trial**.
     This separates the "more points" effect from "this random layout was
     lucky", fixing the N=4/N=6 wiggle of v1.
  3. Reports translation/rotation GT error P95 **and** handover-workspace
     induced point error P95 (rotation error amplified by working distance —
     the physically meaningful metric).
  4. Systematic TCP is modelled as a per-arm **tool-frame** bias rotated by
     the probe orientation at each touch, matching reality when the probe is
     tilted differently per point; ``fixed_probe_orientation=True`` falls back
     to the v1 constant base-frame offset (valid when all touches share one
     orientation).

Statistical caveat (from review): fit/hold-out residuals measure *internal
consistency* only; a systematic per-arm TCP bias is absorbed into the fitted
transform and does not show up there.  Absolute accuracy must be validated by
an independent modality (M2.7/M2.8 global camera cross-check).
"""

from __future__ import annotations

from dataclasses import dataclass, asdict
from typing import List, Optional, Sequence, Tuple

import numpy as np

from .base_alignment import apply_transform, kabsch_align, transform_error

# Hand-over region representative points (in left-base frame), used to report
# the *induced* position error of the estimated transform at working distance.
HANDOVER_POINTS = np.array([
    [0.00, 0.30, 0.10],
    [0.15, 0.35, 0.15],
    [-0.15, 0.25, 0.20],
    [0.05, 0.45, 0.05],
    [-0.10, 0.40, 0.25],
], dtype=float)


# --------------------------------------------------------------------------
# GT and fixture geometry
# --------------------------------------------------------------------------

def dual_base_ground_truth() -> np.ndarray:
    """{}^{B_L}T_{B_R}^{GT} from the dual-arm URDF.

    world_T_left_base  = (0.35, 0, 0), yaw=π
    world_T_right_base = (-0.35, 0, 0), yaw=0
    → left_base_T_right_base = Rz(180°) with t = (0.7, 0, 0).
    """
    t = np.eye(4, dtype=float)
    t[:3, :3] = np.array([[-1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, 1.0]])
    t[:3, 3] = np.array([0.7, 0.0, 0.0])
    return t


def fixture_points_wide(n: int, *, rng: np.random.Generator) -> np.ndarray:
    """Generate a WIDE fixture covering the whole hand-over volume (~0.5 m)."""
    x = rng.uniform(-0.30, 0.30, size=n)
    y = rng.uniform(0.05, 0.55, size=n)
    z = rng.uniform(0.02, 0.40, size=n)
    return np.stack([x, y, z], axis=1)


def design_wide_fixture(n: int = 20, *, seed: int = 0) -> np.ndarray:
    """Design ONE fixed WIDE fixture (the layout you would actually build).

    Uses low-discrepancy (stratified) sampling so the 20 physical points are
    well spread over the hand-over volume.
    """
    rng = np.random.default_rng(seed)
    xs = np.linspace(-0.28, 0.28, 5) + rng.uniform(-0.02, 0.02, 5)
    ys = np.linspace(0.10, 0.50, 4) + rng.uniform(-0.02, 0.02, 4)
    zs = np.array([0.05, 0.20, 0.35]) + rng.uniform(-0.02, 0.02, 3)
    grid = np.array([[x, y, z] for x in xs for y in ys for z in zs])  # 60 candidates
    # greedy farthest-point sampling down to n
    indices = [0]
    while len(indices) < n:
        dists = np.min(np.linalg.norm(grid - grid[np.asarray(indices)][:, None, :], axis=2), axis=0)
        dists[np.asarray(indices)] = -1.0
        indices.append(int(np.argmax(dists)))
    return grid[np.asarray(indices)]


def subset_farthest_points(points: np.ndarray, count: int) -> np.ndarray:
    """Farthest-point-sampling subset (nested: k ⊂ k+1)."""
    if count > len(points):
        raise ValueError("count exceeds fixture size")
    indices = [0]
    while len(indices) < count:
        dists = np.min(np.linalg.norm(points - points[np.asarray(indices)][:, None, :], axis=2), axis=0)
        dists[np.asarray(indices)] = -1.0
        indices.append(int(np.argmax(dists)))
    return points[np.asarray(indices)]


# --------------------------------------------------------------------------
# Measurement noise
# --------------------------------------------------------------------------

@dataclass(frozen=True)
class NoiseModel:
    touch_axis_sigma_m: float = 0.0005     # per-axis contact repeatability
    tcp_axis_sigma_m: float = 0.000577     # per-axis TCP residual (3D RMS = 1 mm)
    fk_axis_sigma_m: float = 0.0005        # per-axis joint-FK isotropic error
    fixed_probe_orientation: bool = True   # True: TCP bias constant in base frame

    def measure(
        self,
        left_points: np.ndarray,      # true points in B_L
        right_points: np.ndarray,     # true points in B_R
        rng: np.random.Generator,
        *,
        probe_rotations_left: Optional[np.ndarray] = None,
        probe_rotations_right: Optional[np.ndarray] = None,
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Return noisy measurements of the same physical points in each base.

        TCP error is a per-arm systematic bias, sampled once per call:
          - fixed_probe_orientation=True  → constant 3D offset in base frame
            (valid when every touch uses the same probe orientation);
          - fixed_probe_orientation=False → bias lives in the tool frame and
            is rotated by each touch's probe orientation R_B_tool, so its
            base-frame direction varies per point.
        Touch + FK are independent per point.
        """
        n = len(left_points)
        if probe_rotations_left is None:
            probe_rotations_left = np.repeat(np.eye(3)[None, :, :], n, axis=0)
        if probe_rotations_right is None:
            probe_rotations_right = np.repeat(np.eye(3)[None, :, :], n, axis=0)

        left_tcp_tool = rng.normal(0.0, self.tcp_axis_sigma_m, size=3)
        right_tcp_tool = rng.normal(0.0, self.tcp_axis_sigma_m, size=3)
        if self.fixed_probe_orientation:
            left_tcp = np.tile(left_tcp_tool, (n, 1))
            right_tcp = np.tile(right_tcp_tool, (n, 1))
        else:
            left_tcp = probe_rotations_left @ left_tcp_tool
            right_tcp = probe_rotations_right @ right_tcp_tool

        left_noise = rng.normal(0.0, self.touch_axis_sigma_m, size=left_points.shape)
        right_noise = rng.normal(0.0, self.touch_axis_sigma_m, size=right_points.shape)
        left_fk = rng.normal(0.0, self.fk_axis_sigma_m, size=left_points.shape)
        right_fk = rng.normal(0.0, self.fk_axis_sigma_m, size=right_points.shape)

        left_measured = left_points + left_tcp + left_noise + left_fk
        right_measured = right_points + right_tcp + right_noise + right_fk
        return left_measured, right_measured


# --------------------------------------------------------------------------
# Monte Carlo drivers
# --------------------------------------------------------------------------

@dataclass
class TrialResult:
    translation_error_m: float
    rotation_error_deg: float
    handover_point_error_m: float


def run_trials(
    left_true: np.ndarray,
    right_true: np.ndarray,
    noise: NoiseModel,
    *,
    trials: int,
    rng: np.random.Generator,
) -> List[TrialResult]:
    """Run ``trials`` Monte Carlo repetitions for a fixed fixture point set."""
    if trials <= 0:
        raise ValueError("trials must be positive")
    results: List[TrialResult] = []
    truth = dual_base_ground_truth()
    for _ in range(trials):
        left_meas, right_meas = noise.measure(left_true, right_true, rng)
        estimate = kabsch_align(left_meas, right_meas)
        t_err, r_err = transform_error(estimate, truth)
        # induced point error at the hand-over workspace
        mapped = apply_transform(HANDOVER_POINTS, estimate)
        expected = apply_transform(HANDOVER_POINTS, truth)
        handover_err = float(np.max(np.linalg.norm(mapped - expected, axis=1)))
        results.append(TrialResult(t_err, r_err, handover_err))
    return results


def summarize(results: Sequence[TrialResult]) -> dict:
    """P95 / P50 / MAX over trials."""
    if len(results) == 0:
        raise ValueError("no trials to summarize")
    translations = np.asarray([r.translation_error_m for r in results])
    rotations = np.asarray([r.rotation_error_deg for r in results])
    handover = np.asarray([r.handover_point_error_m for r in results])
    return {
        "translation_p95_m": float(np.percentile(translations, 95)),
        "translation_p50_m": float(np.percentile(translations, 50)),
        "translation_max_m": float(np.max(translations)),
        "rotation_p95_deg": float(np.percentile(rotations, 95)),
        "rotation_p50_deg": float(np.percentile(rotations, 50)),
        "rotation_max_deg": float(np.max(rotations)),
        "handover_point_p95_m": float(np.percentile(handover, 95)),
        "handover_point_p50_m": float(np.percentile(handover, 50)),
        "handover_point_max_m": float(np.max(handover)),
    }


def _paired_noise(
    fixture: np.ndarray,
    noise: NoiseModel,
    trials: int,
    rng: np.random.Generator,
    truth: np.ndarray,
) -> Tuple[np.ndarray, np.ndarray]:
    """Pre-generate per-point measurement noise so every N can reuse it.

    Returns (left_meas_all, right_meas_all) with shape (trials, n_fixture, 3).

    TCP bias per trial:
      - fixed_probe_orientation=True  → ONE constant 3D offset in base frame
        shared by all points (valid when every touch uses the same probe
        orientation);
      - fixed_probe_orientation=False → the bias lives in the TOOL frame and is
        rotated into base by each touch's probe orientation R_B_tool,i, so its
        base-frame direction varies per point (realistic when the probe is
        tilted differently per point).
    """
    n = len(fixture)
    left_true_all = np.repeat(fixture[None, :, :], trials, axis=0)
    right_true_all = apply_transform(fixture, np.linalg.inv(truth))
    right_true_all = np.repeat(right_true_all[None, :, :], trials, axis=0)

    left_tcp_tool = rng.normal(0.0, noise.tcp_axis_sigma_m, size=(trials, 3))
    right_tcp_tool = rng.normal(0.0, noise.tcp_axis_sigma_m, size=(trials, 3))
    if noise.fixed_probe_orientation:
        left_tcp = left_tcp_tool[:, None, :]
        right_tcp = right_tcp_tool[:, None, :]
    else:
        # per-point random probe orientations (same across trials for pairing,
        # but different per fixture point so the base-frame TCP direction varies)
        left_rots = _random_probe_rotations(n, rng)
        right_rots = _random_probe_rotations(n, rng)
        left_tcp = np.einsum("pij,tj->tpi", left_rots, left_tcp_tool)
        right_tcp = np.einsum("pij,tj->tpi", right_rots, right_tcp_tool)

    left_rand = rng.normal(0.0, noise.touch_axis_sigma_m, size=(trials, n, 3))
    right_rand = rng.normal(0.0, noise.touch_axis_sigma_m, size=(trials, n, 3))
    left_fk = rng.normal(0.0, noise.fk_axis_sigma_m, size=(trials, n, 3))
    right_fk = rng.normal(0.0, noise.fk_axis_sigma_m, size=(trials, n, 3))

    left_meas_all = left_true_all + left_tcp + left_rand + left_fk
    right_meas_all = right_true_all + right_tcp + right_rand + right_fk
    return left_meas_all, right_meas_all


def _random_probe_rotations(n: int, rng: np.random.Generator) -> np.ndarray:
    """Random (n, 3, 3) rotation matrices simulating varied probe orientations.

    Scipy's ``Rotation.random`` is imported lazily to keep the module
    dependency-light.
    """
    from scipy.spatial.transform import Rotation as R

    return R.random(n, random_state=rng).as_matrix()


def _summarize_matrix(
    estimates: np.ndarray,
    truth: np.ndarray,
) -> dict:
    """Summarize transform + handover errors for a (trials, 4, 4) estimate set."""
    translations = np.empty(len(estimates), dtype=float)
    rotations = np.empty(len(estimates), dtype=float)
    handover = np.empty(len(estimates), dtype=float)
    for index, estimate in enumerate(estimates):
        t_err, r_err = transform_error(estimate, truth)
        translations[index] = t_err
        rotations[index] = r_err
        mapped = apply_transform(HANDOVER_POINTS, estimate)
        expected = apply_transform(HANDOVER_POINTS, truth)
        handover[index] = float(np.max(np.linalg.norm(mapped - expected, axis=1)))
    return {
        "translation_p95_m": float(np.percentile(translations, 95)),
        "translation_p50_m": float(np.percentile(translations, 50)),
        "translation_max_m": float(np.max(translations)),
        "rotation_p95_deg": float(np.percentile(rotations, 95)),
        "rotation_p50_deg": float(np.percentile(rotations, 50)),
        "rotation_max_deg": float(np.max(rotations)),
        "handover_point_p95_m": float(np.percentile(handover, 95)),
        "handover_point_p50_m": float(np.percentile(handover, 50)),
        "handover_point_max_m": float(np.max(handover)),
    }


def nested_scan(
    fixture: np.ndarray,
    counts: Sequence[int] = (4, 6, 10, 15, 20),
    *,
    trials: int = 2000,
    noise: NoiseModel = NoiseModel(),
    seed: int = 0,
) -> dict:
    """Paired nested scan: every N is a farthest-point subset of the SAME
    fixture, and all N reuse the SAME pre-generated noise realizations.

    This isolates the "more points" effect from layout luck and makes the
    N=15 vs N=20 comparison a genuine paired one.
    """
    if len(fixture) < max(counts):
        raise ValueError("fixture has fewer points than the largest requested count")
    rng = np.random.default_rng(seed)
    truth = dual_base_ground_truth()
    left_meas_all, right_meas_all = _paired_noise(fixture, noise, trials, rng, truth)

    rows = []
    for count in counts:
        # farthest-point subset indices
        indices = [0]
        while len(indices) < count:
            dists = np.min(np.linalg.norm(
                fixture - fixture[np.asarray(indices)][:, None, :], axis=2), axis=0)
            dists[np.asarray(indices)] = -1.0
            indices.append(int(np.argmax(dists)))
        idx = np.asarray(indices)
        estimates = np.empty((trials, 4, 4), dtype=float)
        for trial in range(trials):
            estimates[trial] = kabsch_align(
                left_meas_all[trial][idx], right_meas_all[trial][idx])
        rows.append({"sample_count": int(count), **_summarize_matrix(estimates, truth)})
    return {"strategy": "nested_wide_fixture", "noise": asdict(noise), "rows": rows}


def compare_coverage(
    counts: Sequence[int] = (4, 6, 10, 15, 20),
    *,
    trials: int = 2000,
    noise: NoiseModel = NoiseModel(),
    seed: int = 0,
) -> dict:
    """WIDE (nested fixture) vs SMALL (independent small cluster per N).

    The SMALL arm deliberately keeps v1's per-N independent layout so we can
    quantify how much worse a cramped layout is; use it only to illustrate the
    coverage effect, not for the 15-vs-20 decision.
    """
    fixture = design_wide_fixture(20, seed=seed)
    wide = nested_scan(fixture, counts, trials=trials, noise=noise, seed=seed)
    rng = np.random.default_rng(seed + 100)
    small_rows = []
    for count in counts:
        left_true = fixture_points_wide(count, rng=rng)
        right_true = apply_transform(left_true, np.linalg.inv(dual_base_ground_truth()))
        results = run_trials(left_true, right_true, noise, trials=trials, rng=rng)
        small_rows.append({"sample_count": int(count), **summarize(results)})
    return {
        "wide": wide,
        "small": {"strategy": "small_cluster", "noise": asdict(noise), "rows": small_rows},
    }
