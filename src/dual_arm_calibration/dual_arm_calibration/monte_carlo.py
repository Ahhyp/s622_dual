"""M2.6 dual-base Monte Carlo simulation.

Simulates the M2.5 real-hardware protocol: both arms touch shared physical
points on a 3D fixture; each arm reports the point in its own base frame
(probe TCP + joint FK + touch repeatability), and Kabsch aligns the two point
sets into {}^{B_L}T_{B_R}.

Noise model (all values in meters / radians unless stated):
  - ``touch_sigma``: per-contact repeatability (Gaussian, per point, per arm)
  - ``tcp_error_sigma``: per-arm probe TCP calibration residual — a *systematic*
    offset that is identical across that arm's points within one trial
    (sampled fresh each trial)
  - ``fk_error_sigma``: per-arm joint-FK error, modelled as an additional
    per-point Gaussian (worst-case isotropic)

Because the TCP error is systematic per arm, it does NOT average out with more
points — the key insight the Monte Carlo is meant to demonstrate: spatial
coverage matters more than sheer sample count, and systematic TCP error caps
the achievable accuracy regardless of N.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Sequence, Tuple

import numpy as np

from .base_alignment import apply_transform, kabsch_align, transform_error


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


def fixture_points_small(n: int, *, rng: np.random.Generator) -> np.ndarray:
    """Small cluster: all points confined to a compact box (~0.10 m) around
    the hand-over region — poor spatial coverage."""
    # hand-over region is around x≈0, y≈0.3 in left-base coordinates
    origin = np.array([0.0, 0.30, 0.10])
    return origin + rng.uniform(-0.05, 0.05, size=(n, 3))


def fixture_points_wide(n: int, *, rng: np.random.Generator) -> np.ndarray:
    """Wide coverage: spreads over the whole hand-over volume (~0.5 m) with
    real Z variation — the recommended acquisition strategy."""
    x = rng.uniform(-0.30, 0.30, size=n)
    y = rng.uniform(0.05, 0.55, size=n)
    z = rng.uniform(0.02, 0.40, size=n)
    return np.stack([x, y, z], axis=1)


# --------------------------------------------------------------------------
# Measurement noise
# --------------------------------------------------------------------------

@dataclass(frozen=True)
class NoiseModel:
    touch_sigma_m: float = 0.0005    # 0.5 mm contact repeatability
    tcp_error_sigma_m: float = 0.001  # 1 mm per-arm TCP residual (systematic)
    fk_error_sigma_m: float = 0.0005  # 0.5 mm joint-FK isotropic error

    def measure(
        self,
        left_points: np.ndarray,      # true points in B_L
        right_points: np.ndarray,     # true points in B_R
        rng: np.random.Generator,
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Return noisy measurements of the same physical points in each base.

        TCP error is per-arm systematic: a single random offset vector is
        added to every point of that arm for this trial.
        """
        left_tcp = rng.normal(0.0, self.tcp_error_sigma_m, size=3)
        right_tcp = rng.normal(0.0, self.tcp_error_sigma_m, size=3)
        left_noise = rng.normal(0.0, self.touch_sigma_m, size=left_points.shape)
        right_noise = rng.normal(0.0, self.touch_sigma_m, size=right_points.shape)
        left_fk = rng.normal(0.0, self.fk_error_sigma_m, size=left_points.shape)
        right_fk = rng.normal(0.0, self.fk_error_sigma_m, size=right_points.shape)
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


def run_trials(
    left_true: np.ndarray,
    right_true: np.ndarray,
    noise: NoiseModel,
    *,
    trials: int,
    rng: np.random.Generator,
) -> List[TrialResult]:
    """Run ``trials`` Monte Carlo repetitions for a fixed fixture point set."""
    results: List[TrialResult] = []
    truth = dual_base_ground_truth()
    for _ in range(trials):
        left_meas, right_meas = noise.measure(left_true, right_true, rng)
        estimate = kabsch_align(left_meas, right_meas)
        t_err, r_err = transform_error(estimate, truth)
        results.append(TrialResult(t_err, r_err))
    return results


def summarize(results: Sequence[TrialResult]) -> dict:
    """P95 / P50 / MAX of translation and rotation errors over trials."""
    translations = np.asarray([r.translation_error_m for r in results])
    rotations = np.asarray([r.rotation_error_deg for r in results])
    return {
        "translation_p95_m": float(np.percentile(translations, 95)),
        "translation_p50_m": float(np.percentile(translations, 50)),
        "translation_max_m": float(np.max(translations)),
        "rotation_p95_deg": float(np.percentile(rotations, 95)),
        "rotation_p50_deg": float(np.percentile(rotations, 50)),
        "rotation_max_deg": float(np.max(rotations)),
    }


def scan_sample_counts(
    counts: Sequence[int] = (4, 6, 10, 15, 20, 30),
    *,
    trials: int = 500,
    noise: NoiseModel = NoiseModel(),
    seed: int = 0,
    wide: bool = True,
) -> dict:
    """Sample-count → P95 curves for one fixture strategy.

    For each N, generate a fresh fixture point set (so the curve reflects the
    protocol as run on the robot, not one fixed arrangement).
    """
    rng = np.random.default_rng(seed)
    fixture = fixture_points_wide if wide else fixture_points_small
    truth = dual_base_ground_truth()
    rows = []
    for count in counts:
        left_true = fixture(count, rng=rng)
        right_true = apply_transform(left_true, np.linalg.inv(truth))
        results = run_trials(left_true, right_true, noise, trials=trials, rng=rng)
        rows.append({"sample_count": int(count), **summarize(results)})
    return {"strategy": "wide" if wide else "small", "noise": noise, "rows": rows}


def compare_coverage(
    counts: Sequence[int] = (4, 6, 10, 15, 20, 30),
    *,
    trials: int = 500,
    noise: NoiseModel = NoiseModel(),
    seed: int = 0,
) -> dict:
    """Run both fixture strategies and return side-by-side P95 curves."""
    wide = scan_sample_counts(counts, trials=trials, noise=noise, seed=seed, wide=True)
    small = scan_sample_counts(counts, trials=trials, noise=noise, seed=seed + 1, wide=False)
    return {"wide": wide, "small": small}
