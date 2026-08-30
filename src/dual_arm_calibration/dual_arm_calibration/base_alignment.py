"""Kabsch/SVD rigid alignment for dual-base calibration (M2.5/M2.6).

Solves {}^{B_L}T_{B_R} from corresponding contact points measured by both
arms:  p_i^{B_L} and p_i^{B_R}  (i = 1..N).  Convention:

    p^{B_L} = {}^{B_L}T_{B_R} · p^{B_R}

i.e. the transform maps right-base coordinates into left-base coordinates.

Notes
-----
- Uses the Umeyama/Kabsch closed-form (centroid + SVD), guaranteeing an
  optimal rigid (rotation + translation) least-squares fit.
- Mirrors what M2.5 will run on the real robot; this module is pure numpy so
  the same code drives both the simulator (M2.6 Monte Carlo) and, later, the
  real-hardware offline solver.
"""

from __future__ import annotations

import math
from typing import Tuple

import numpy as np


def kabsch_align(
    left_points: np.ndarray,
    right_points: np.ndarray,
    *,
    allow_reflection: bool = False,
) -> np.ndarray:
    """Align right-base points onto left-base points.

    Parameters
    ----------
    left_points : (N, 3) array — p_i^{B_L}
    right_points : (N, 3) array — p_i^{B_R}
    allow_reflection : if False (default) enforce det(R) = +1 so the result
        is a proper rigid transform (a physical base-to-base must be rigid).

    Returns
    -------
    T : (4, 4) array — {}^{B_L}T_{B_R}
    """
    left = np.asarray(left_points, dtype=float)
    right = np.asarray(right_points, dtype=float)
    if left.shape != right.shape or left.ndim != 2 or left.shape[1] != 3:
        raise ValueError(f"points must be (N,3) arrays, got {left.shape} vs {right.shape}")
    if len(left) < 3:
        raise ValueError("at least three non-collinear correspondences are required")

    left_centroid = left.mean(axis=0)
    right_centroid = right.mean(axis=0)
    left_centered = left - left_centroid
    right_centered = right - right_centroid

    covariance = left_centered.T @ right_centered
    u, _singular, vt = np.linalg.svd(covariance)
    rotation = u @ vt

    if not allow_reflection and np.linalg.det(rotation) < 0.0:
        # Umeyama correction: flip the last column of u to force det=+1.
        u_flipped = u.copy()
        u_flipped[:, -1] *= -1.0
        rotation = u_flipped @ vt

    translation = left_centroid - rotation @ right_centroid

    transform = np.eye(4, dtype=float)
    transform[:3, :3] = rotation
    transform[:3, 3] = translation
    return transform


def apply_transform(points: np.ndarray, transform: np.ndarray) -> np.ndarray:
    """Apply a 4x4 transform to (N,3) points."""
    points = np.asarray(points, dtype=float)
    return (transform[:3, :3] @ points.T).T + transform[:3, 3]


def alignment_error(
    left_points: np.ndarray,
    right_points: np.ndarray,
    transform: np.ndarray,
) -> dict:
    """Per-correspondence residuals after applying the estimated transform.

    residual_i = || p_i^{B_L} - T · p_i^{B_R} ||  (mm, converted from m)
    """
    left = np.asarray(left_points, dtype=float)
    right = np.asarray(right_points, dtype=float)
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
    rotation = angle of delta_R.
    """
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
    Returns fit/hold-out RMS + MAX in mm.
    """
    total = len(left_points)
    if fit_count >= total:
        raise ValueError("fit_count must leave at least one hold-out point")
    indices = rng.permutation(total)
    fit_idx, hold_idx = indices[:fit_count], indices[fit_count:]
    transform = kabsch_align(left_points[fit_idx], right_points[fit_idx])
    fit = alignment_error(left_points[fit_idx], right_points[fit_idx], transform)
    hold = alignment_error(left_points[hold_idx], right_points[hold_idx], transform)
    return {
        "fit_count": int(fit_count),
        "holdout_count": int(total - fit_count),
        "fit_rms_mm": fit["rms_m"] * 1000.0,
        "fit_max_mm": fit["max_m"] * 1000.0,
        "holdout_rms_mm": hold["rms_m"] * 1000.0,
        "holdout_max_mm": hold["max_m"] * 1000.0,
    }
