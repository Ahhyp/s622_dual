"""Kabsch/SVD rigid alignment for dual-base calibration (M2.5/M2.6).

Solves {}^{B_L}T_{B_R} from corresponding contact points measured by both
arms:  p_i^{B_L} and p_i^{B_R}  (i = 1..N).  Convention:

    p^{B_L} = {}^{B_L}T_{B_R} · p^{B_R}

i.e. the transform maps right-base coordinates into left-base coordinates.

Notes
-----
- Uses the Umeyama/Kabsch closed-form (centroid + SVD), guaranteeing an
  optimal rigid (rotation + translation) least-squares fit.
- Production code is **SE(3) only** (det R = +1); a reflection is rejected by
  geometry conditioning rather than being silently allowed, because a
  physical base-to-base transform must be a proper rotation.
- Mirrors what M2.5 will run on the real robot; this module is pure numpy so
  the same code drives both the simulator (M2.6 Monte Carlo) and, later, the
  real-hardware offline solver.
"""

from __future__ import annotations

import math
from typing import Tuple

import numpy as np

# Below this ratio σ2/σ1 the point set is (near-)collinear and the rotation
# about the line is unobservable → refuse to solve.
_MIN_CONDITION_RATIO = 1e-6


def _validate_points(points: np.ndarray, name: str, *, min_count: int = 3) -> np.ndarray:
    array = np.asarray(points, dtype=float)
    if array.ndim != 2 or array.shape[1] != 3:
        raise ValueError(f"{name} must be an (N, 3) array, got shape {array.shape}")
    if len(array) < min_count:
        raise ValueError(f"{name} needs at least {min_count} correspondences")
    if not np.all(np.isfinite(array)):
        raise ValueError(f"{name} contains non-finite values")
    return array


def geometry_condition(points: np.ndarray) -> dict:
    """Singular-value conditioning of a point set.

    Ratios σ2/σ1 and σ3/σ1 quantify how far the points are from collinear /
    coplanar.  Collinear → σ2/σ1 ≈ 0 (rotation about the line unobservable).
    Coplanar-but-not-collinear still has rank 2 and is solvable; the third
    ratio only reflects how much out-of-plane spread exists.
    """
    points = _validate_points(points, "points")
    centered = points - points.mean(axis=0)
    singular = np.linalg.svd(centered, compute_uv=False)
    s1 = max(singular[0], 1e-30)
    return {
        "singular_values": tuple(float(s) for s in singular),
        "sigma2_over_sigma1": float(singular[1] / s1),
        "sigma3_over_sigma1": float(singular[2] / s1),
    }


def kabsch_align(
    left_points: np.ndarray,
    right_points: np.ndarray,
    *,
    min_condition_ratio: float = _MIN_CONDITION_RATIO,
) -> np.ndarray:
    """Align right-base points onto left-base points.

    Parameters
    ----------
    left_points : (N, 3) array — p_i^{B_L}
    right_points : (N, 3) array — p_i^{B_R}
    min_condition_ratio : reject point sets whose σ2/σ1 is below this value
        (near-collinear).  Default 1e-6.

    Returns
    -------
    T : (4, 4) array — {}^{B_L}T_{B_R}, a proper rigid transform (det R = +1).
    """
    left = _validate_points(left_points, "left_points")
    right = _validate_points(right_points, "right_points")
    if left.shape != right.shape:
        raise ValueError(f"left/right shapes differ: {left.shape} vs {right.shape}")

    condition = geometry_condition(left)
    if condition["sigma2_over_sigma1"] < min_condition_ratio:
        raise ValueError(
            "point set is (near-)collinear: σ2/σ1 = "
            f"{condition['sigma2_over_sigma1']:.3e} — the rotation about the "
            "line is unobservable; spread the fixture in a second dimension"
        )

    left_centroid = left.mean(axis=0)
    right_centroid = right.mean(axis=0)
    left_centered = left - left_centroid
    right_centered = right - right_centroid

    covariance = left_centered.T @ right_centered
    u, _singular, vt = np.linalg.svd(covariance)
    rotation = u @ vt

    # Enforce a proper rotation (SE(3)).  A reflection is never a valid
    # physical base transform; flip the last column of U (Umeyama) so det=+1.
    if np.linalg.det(rotation) < 0.0:
        u_flipped = u.copy()
        u_flipped[:, -1] *= -1.0
        rotation = u_flipped @ vt

    translation = left_centroid - rotation @ right_centroid

    transform = np.eye(4, dtype=float)
    transform[:3, :3] = rotation
    transform[:3, 3] = translation
    return transform


def apply_transform(points: np.ndarray, transform: np.ndarray) -> np.ndarray:
    """Apply a 4x4 transform to (N,3) points (N >= 1)."""
    points = _validate_points(points, "points", min_count=1)
    transform = np.asarray(transform, dtype=float)
    if transform.shape != (4, 4) or not np.all(np.isfinite(transform)):
        raise ValueError("transform must be a finite 4x4 matrix")
    return (transform[:3, :3] @ points.T).T + transform[:3, 3]


def alignment_error(
    left_points: np.ndarray,
    right_points: np.ndarray,
    transform: np.ndarray,
) -> dict:
    """Per-correspondence residuals after applying the estimated transform.

    residual_i = || p_i^{B_L} - T · p_i^{B_R} ||  (m)
    """
    left = _validate_points(left_points, "left_points")
    right = _validate_points(right_points, "right_points")
    if left.shape != right.shape:
        raise ValueError(f"left/right shapes differ: {left.shape} vs {right.shape}")
    predicted = apply_transform(right, transform)
    residuals = np.linalg.norm(left - predicted, axis=1)
    return {
        "residuals_m": residuals,
        "rms_m": float(np.sqrt(np.mean(residuals**2))),
        "p95_m": float(np.percentile(residuals, 95)),
        "max_m": float(np.max(residuals)),
    }


def transform_error(estimate: np.ndarray, truth: np.ndarray) -> Tuple[float, float]:
    """Translation error (m) and rotation error (deg) between two 4x4 transforms.

    delta = estimate^{-1} · truth  →  translation = |delta_t|,
    rotation = angle of delta_R.  Both are proper rotations here, so the
    trace formula applies.
    """
    estimate = np.asarray(estimate, dtype=float)
    truth = np.asarray(truth, dtype=float)
    if estimate.shape != (4, 4) or truth.shape != (4, 4):
        raise ValueError("estimate and truth must be 4x4")
    delta = np.linalg.inv(estimate) @ truth
    translation_error = float(np.linalg.norm(delta[:3, 3]))
    cos_angle = np.clip((np.trace(delta[:3, :3]) - 1.0) / 2.0, -1.0, 1.0)
    rotation_error = math.degrees(math.acos(cos_angle))
    return translation_error, rotation_error


def fit_holdout_split(
    left_points: np.ndarray,
    right_points: np.ndarray,
    fit_count: int,
    *,
    rng: np.random.Generator,
) -> dict:
    """Fit {}^{B_L}T_{B_R} on ``fit_count`` correspondences, score on the rest.

    This mirrors M2.5's 15-fit + 5-hold-out protocol on the real robot.

    NOTE: this only measures *internal consistency* (how well one rigid
    transform explains the correspondences).  A systematic per-arm TCP bias
    is absorbed into the fitted transform (the common offset cancels under
    centering), so these residuals do NOT detect absolute translation error —
    that requires an independent modality (e.g. global-camera cross-check).
    """
    left = _validate_points(left_points, "left_points")
    right = _validate_points(right_points, "right_points")
    if left.shape != right.shape:
        raise ValueError(f"left/right shapes differ: {left.shape} vs {right.shape}")
    if fit_count < 3:
        raise ValueError("fit_count must be at least 3")
    total = len(left)
    if fit_count >= total:
        raise ValueError("fit_count must leave at least one hold-out point")
    indices = rng.permutation(total)
    fit_idx, hold_idx = indices[:fit_count], indices[fit_count:]
    transform = kabsch_align(left[fit_idx], right[fit_idx])
    fit = alignment_error(left[fit_idx], right[fit_idx], transform)
    hold = alignment_error(left[hold_idx], right[hold_idx], transform)
    return {
        "fit_count": int(fit_count),
        "holdout_count": int(total - fit_count),
        "fit_rms_mm": fit["rms_m"] * 1000.0,
        "fit_max_mm": fit["max_m"] * 1000.0,
        "holdout_rms_mm": hold["rms_m"] * 1000.0,
        "holdout_max_mm": hold["max_m"] * 1000.0,
    }
