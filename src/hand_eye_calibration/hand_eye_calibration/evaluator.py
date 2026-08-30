"""Independent hold-out calibration evaluator (M2.2).

Replaces the placeholder in ``scripts/evaluate_calibration.py`` where the
eye-in-hand branch constructed residuals as zero (``T_base_mrk_pred =
T_base_mrk_obs``) and therefore could never fail.

Constant-frame evaluator — no assumption that the marker frame coincides with
the tool0 frame:

- **Eye-in-hand** (board fixed in the environment): for every sample pose
  ``{}^{B}T_{M,i} = {}^{B}T_{E,i} · {}^{E}T_{C} · {}^{C}T_{M,i}``.  The marker
  pose in the base frame is theoretically constant across poses, so we report
  marker position RMS/P95/MAX and rotation RMS/P95/MAX around a robust
  reference pose.

- **Eye-on-base** (marker fixed on the end-effector):
  ``{}^{E}T_{M,i} = {}^{E}T_{B,i} · {}^{B}T_{C} · {}^{C}T_{M,i}`` — the marker
  pose in the effector frame is constant instead.  The same formula is used
  because ``CalibrationSample.robot_pose`` already carries the type-correct
  robot pose (``base_T_ee`` for eye-in-hand, ``ee_T_base`` for eye-on-base).

Critical requirement: **solve samples and validation (hold-out) samples must
be disjoint**.  Samples used to estimate the hand-eye must never be used to
score it.  ``evaluate_samples`` splits the dataset, solves on the solve subset
and reports hold-out metrics against the reference frame established by the
solve subset alone.

Note on ``constant_frame_metrics`` without an explicit ``reference``: it is a
*fresh-data self-consistency* check (the reference is derived from the very
samples being scored).  Only ``evaluate_samples`` provides a strict hold-out
score (reference from the solve subset).  Scripts using the saved-calibration
mode must not describe it as strict hold-out.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
from types import SimpleNamespace
from typing import List, Optional, Sequence, Tuple

import numpy as np
import yaml
from scipy.spatial.transform import Rotation as R

from .config import CalibrationType, normalize_calibration_type
from .solver import (
    CalibrationSample,
    TransformMatrix,
    rotation_delta_deg,
    solve_handeye_dataset,
)


def quat_xyzw_to_matrix(x, y, z, w) -> np.ndarray:
    """Build a 4x4 rotation matrix from an (x, y, z, w) quaternion.

    SciPy's ``R.from_quat`` default order is (x, y, z, w) — matching both
    ``geometry_msgs/Quaternion`` and this package's YAML rotation blocks.
    """
    matrix = np.eye(4, dtype=float)
    matrix[:3, :3] = R.from_quat([float(x), float(y), float(z), float(w)]).as_matrix()
    return matrix


def _transform_from_yaml(doc: dict) -> TransformMatrix:
    """Read a {translation: {x,y,z}, rotation: {x,y,z,w}} YAML block."""
    t = doc["translation"]
    q = doc["rotation"]
    return TransformMatrix(
        R.from_quat([q["x"], q["y"], q["z"], q["w"]]),
        (float(t["x"]), float(t["y"]), float(t["z"])),
    )


def load_samples_yaml(path) -> Tuple[CalibrationType, List[CalibrationSample]]:
    """Load the collector's ``.samples`` YAML file."""
    data = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    kind = normalize_calibration_type(data["calibration_type"])
    robot_key = "base_T_ee" if kind is CalibrationType.EYE_IN_HAND else "ee_T_base"
    samples: List[CalibrationSample] = []
    for entry in data["samples"]:
        samples.append(CalibrationSample(
            waypoint_index=int(entry["waypoint_index"]),
            target_joints_deg=tuple(float(value) for value in entry["target_joints_deg"]),
            robot_pose=_transform_from_yaml(entry[robot_key]),
            tracking_pose=_transform_from_yaml(entry["camera_T_marker"]),
        ))
    return kind, samples


@dataclass(frozen=True)
class CalibrationFile:
    """Parsed ``.calib`` file: type + transform (+ raw parameters)."""

    calibration_type: CalibrationType
    transform: TransformMatrix
    parameters: dict


def load_calibration_yaml(path) -> CalibrationFile:
    """Load the collector's ``.calib`` YAML file (type + ``transform`` block)."""
    data = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    parameters = data.get("parameters", {}) or {}
    kind = normalize_calibration_type(parameters.get("calibration_type", "eye_in_hand"))
    return CalibrationFile(
        calibration_type=kind,
        transform=_transform_from_yaml(data["transform"]),
        parameters=parameters,
    )


def _transform_to_yaml(transform: TransformMatrix) -> dict:
    quaternion = transform.rotation.as_quat()
    return {
        "translation": {"x": float(transform.translation[0]), "y": float(transform.translation[1]), "z": float(transform.translation[2])},
        "rotation": {"x": float(quaternion[0]), "y": float(quaternion[1]), "z": float(quaternion[2]), "w": float(quaternion[3])},
    }


def save_samples_yaml(path, samples, calibration_type, *, status: str = "saved") -> Path:
    """Write samples in the collector ``.samples`` format (for round-trip tests / archiving)."""
    kind = normalize_calibration_type(calibration_type)
    robot_key = "base_T_ee" if kind is CalibrationType.EYE_IN_HAND else "ee_T_base"
    data = {
        "calibration_type": kind.value,
        "status": status,
        "samples": [
            {
                "waypoint_index": int(sample.waypoint_index),
                "target_joints_deg": [float(value) for value in sample.target_joints_deg],
                robot_key: _transform_to_yaml(sample.robot_pose),
                "camera_T_marker": _transform_to_yaml(sample.tracking_pose),
            }
            for sample in samples
        ],
    }
    path = Path(path)
    path.write_text(yaml.safe_dump(data, sort_keys=False), encoding="utf-8")
    return path


def save_calibration_yaml(
    path,
    transform: TransformMatrix,
    calibration_type=CalibrationType.EYE_IN_HAND,
    *,
    name: str = "calibration",
) -> Path:
    """Write a hand-eye in the collector ``.calib`` format (for round-trip tests / archiving)."""
    data = {
        "parameters": {
            "name": name,
            "calibration_type": normalize_calibration_type(calibration_type).value,
        },
        "transform": _transform_to_yaml(transform),
    }
    path = Path(path)
    path.write_text(yaml.safe_dump(data, sort_keys=False), encoding="utf-8")
    return path


def solver_config_from_yaml() -> SimpleNamespace:
    """Build the solver parameters the solve pipeline needs, from the shared YAML."""
    from .config import _load_yaml_defaults

    d = _load_yaml_defaults()
    get = d.get
    return SimpleNamespace(
        algorithm_names=tuple(get("algorithm_names", ["OpenCV/Park", "OpenCV/Horaud"])),
        maximum_camera_translation_norm_m=float(get("maximum_camera_translation_norm_m", 0.30)),
        maximum_eye_on_base_camera_translation_norm_m=float(get("maximum_eye_on_base_camera_translation_norm_m", 2.0)),
        maximum_algorithm_translation_delta_m=float(get("maximum_algorithm_translation_delta_m", 0.003)),
        maximum_algorithm_rotation_delta_deg=float(get("maximum_algorithm_rotation_delta_deg", 1.0)),
        fixed_marker_refinement_translation_sigma_m=float(get("fixed_marker_refinement_translation_sigma_m", 0.0005)),
        fixed_marker_refinement_rotation_sigma_deg=float(get("fixed_marker_refinement_rotation_sigma_deg", 0.30)),
        fixed_marker_refinement_max_iterations=int(get("fixed_marker_refinement_max_iterations", 25)),
        maximum_marker_position_rms_m=float(get("maximum_marker_position_rms_m", 0.002)),
        maximum_marker_rotation_rms_deg=float(get("maximum_marker_rotation_rms_deg", 1.50)),
    )


def constant_frame_pose(sample: CalibrationSample, handeye: TransformMatrix) -> np.ndarray:
    """Pose of the frame that is expected constant, under the given hand-eye.

    ``robot_pose @ handeye @ tracking_pose``:
      - eye-in-hand → base_T_marker (marker fixed in base),
      - eye-on-base → ee_T_marker  (marker fixed on the effector).
    """
    return sample.robot_pose.matrix() @ handeye.matrix() @ sample.tracking_pose.matrix()


def reference_pose(poses: Sequence[np.ndarray]) -> np.ndarray:
    """Robust reference (median translation + medoid rotation) of implied fixed poses."""
    if len(poses) == 0:
        raise ValueError("reference_pose requires at least one pose")
    translations = np.asarray([pose[:3, 3] for pose in poses])
    reference_translation = np.median(translations, axis=0)
    rotations = [R.from_matrix(pose[:3, :3]) for pose in poses]
    reference_rotation = min(
        rotations,
        key=lambda candidate: sum(rotation_delta_deg(candidate, other) for other in rotations),
    )
    matrix = np.eye(4, dtype=float)
    matrix[:3, :3] = reference_rotation.as_matrix()
    matrix[:3, 3] = reference_translation
    return matrix


def constant_frame_metrics(
    samples: Sequence[CalibrationSample],
    handeye: TransformMatrix,
    *,
    reference: Optional[np.ndarray] = None,
) -> dict:
    """Deviation of each sample's constant frame from a reference pose.

    ``reference`` should be computed from the *solve* set (see
    ``evaluate_samples``) so hold-out samples are never used to define the
    frame they are scored against.  When ``None`` the reference is derived
    from the samples themselves — a *self-consistency* metric, only meaningful
    as a fresh-data consistency check (not a strict hold-out score).
    """
    if len(samples) == 0:
        raise ValueError("constant_frame_metrics requires at least one sample")
    poses = [constant_frame_pose(sample, handeye) for sample in samples]
    ref = reference_pose(poses) if reference is None else np.asarray(reference, dtype=float)
    positions = [float(np.linalg.norm(pose[:3, 3] - ref[:3, 3])) for pose in poses]
    rotations = [
        rotation_delta_deg(R.from_matrix(ref[:3, :3]), R.from_matrix(pose[:3, :3])) for pose in poses
    ]
    return {
        "position_rms_m": float(math.sqrt(np.mean(np.square(positions)))),
        "position_p95_m": float(np.percentile(positions, 95)),
        "position_max_m": float(np.max(positions)),
        "rotation_rms_deg": float(math.sqrt(np.mean(np.square(rotations)))),
        "rotation_p95_deg": float(np.percentile(rotations, 95)),
        "rotation_max_deg": float(np.max(rotations)),
        "per_sample_position_m": positions,
        "per_sample_rotation_deg": rotations,
        "count": len(samples),
    }


def split_solve_holdout(
    samples: Sequence[CalibrationSample],
    solve_count: int,
    holdout_count: Optional[int] = None,
    *,
    rng: Optional[np.random.Generator] = None,
) -> Tuple[List[CalibrationSample], List[CalibrationSample]]:
    """Split samples into disjoint solve / hold-out subsets.

    Deterministic by default: hold-out indices are spread evenly across the
    acquisition sequence (a heuristic that usually improves spatial diversity;
    it is not a strict SE(3) stratification).  Pass an RNG for a random split.
    """
    total = len(samples)
    if solve_count <= 0:
        raise ValueError("solve_count must be positive")
    if holdout_count is None:
        holdout_count = total - solve_count
    if solve_count + holdout_count != total:
        raise ValueError(f"solve_count + holdout_count ({solve_count}+{holdout_count}) != total ({total})")
    if solve_count < 3 or holdout_count < 1:
        raise ValueError("need at least 3 solve samples and 1 hold-out sample")

    if rng is None:
        indices = np.unique(np.round(np.linspace(0, total - 1, holdout_count)).astype(int))
        if len(indices) != holdout_count:  # degenerate spacing fallback
            indices = np.arange(holdout_count, dtype=int)
    else:
        indices = np.sort(rng.choice(total, size=holdout_count, replace=False))
    holdout_indices = set(int(value) for value in indices)
    solve = [sample for index, sample in enumerate(samples) if index not in holdout_indices]
    holdout = [samples[index] for index in sorted(holdout_indices)]
    return solve, holdout


def solve_handeye(
    solve_samples: Sequence[CalibrationSample],
    calibration_type=CalibrationType.EYE_IN_HAND,
    config: Optional[SimpleNamespace] = None,
    *,
    allow_pruning: bool = False,
):
    """Solve the hand-eye on the given samples via the full production pipeline.

    Delegates to ``solver.solve_handeye_dataset`` so the same quality gates as
    the collector apply (installation norm, Park/Horaud agreement, finite
    check, internal marker RMS).  ``allow_pruning`` may only remove samples
    from this solve subset — never from hold-out data.

    Returns ``(valid, handeye, algorithm, details)``; ``valid`` is the
    production solver gate (not the hold-out gate).
    """
    if config is None:
        config = solver_config_from_yaml()
    if len(solve_samples) < 3:
        raise ValueError("at least three solve samples are required")
    valid, refined, algorithm, spread_t, spread_r, metrics, details, retained = solve_handeye_dataset(
        solve_samples, config,
        calibration_type=calibration_type,
        allow_pruning=allow_pruning,
    )
    details = {
        **details,
        "algorithm": algorithm,
        "spread_translation_m": spread_t,
        "spread_rotation_deg": spread_r,
        "marker_position_rms_m": metrics["position_rms_m"],
        "marker_rotation_rms_deg": metrics["rotation_rms_deg"],
        "retained_samples": retained,
    }
    return bool(valid), refined, algorithm, details


@dataclass
class EvaluationResult:
    """Full hold-out evaluation of a dataset against a hand-eye estimate."""

    calibration_type: CalibrationType
    solve_count: int
    holdout_count: int
    algorithm: str
    handeye: TransformMatrix
    internal_metrics: dict
    holdout_metrics: dict
    solve_details: dict
    solver_valid: bool

    def passed_gates(self) -> bool:
        """Overall acceptance = production solver gate AND M2.3-B hold-out gate:
        position RMS <= 3 mm, position MAX <= 5 mm, rotation RMS <= 1 deg."""
        m = self.holdout_metrics
        holdout_ok = bool(
            m["position_rms_m"] <= 0.003
            and m["position_max_m"] <= 0.005
            and m["rotation_rms_deg"] <= 1.0
        )
        return bool(self.solver_valid and holdout_ok)

    def format_report(self) -> str:
        lines = [
            "=" * 62,
            f"  Constant-frame hold-out evaluation ({self.calibration_type.value})",
            f"  solve={self.solve_count}  hold-out={self.holdout_count}  algorithm={self.algorithm}",
            "-" * 62,
            "  SOLVE-SET (internal constant-frame consistency)",
            "    position RMS/P95/MAX = "
            f"{self.internal_metrics['position_rms_m'] * 1000:.2f} / "
            f"{self.internal_metrics['position_p95_m'] * 1000:.2f} / "
            f"{self.internal_metrics['position_max_m'] * 1000:.2f} mm",
            "    rotation RMS/P95/MAX = "
            f"{self.internal_metrics['rotation_rms_deg']:.3f} / "
            f"{self.internal_metrics['rotation_p95_deg']:.3f} / "
            f"{self.internal_metrics['rotation_max_deg']:.3f} deg",
            "  HOLD-OUT (scored vs solve-set reference frame)",
            "    position RMS/P95/MAX = "
            f"{self.holdout_metrics['position_rms_m'] * 1000:.2f} / "
            f"{self.holdout_metrics['position_p95_m'] * 1000:.2f} / "
            f"{self.holdout_metrics['position_max_m'] * 1000:.2f} mm",
            "    rotation RMS/P95/MAX = "
            f"{self.holdout_metrics['rotation_rms_deg']:.3f} / "
            f"{self.holdout_metrics['rotation_p95_deg']:.3f} / "
            f"{self.holdout_metrics['rotation_max_deg']:.3f} deg",
            "-" * 62,
            f"  solver quality gate: {'PASS' if self.solver_valid else 'FAIL'}",
            "  M2.3-B hold-out gates: position RMS <= 3 mm, position MAX <= 5 mm,"
            " rotation RMS <= 1 deg",
            f"    OVERALL PASS = {self.passed_gates()!s}",
            "=" * 62,
        ]
        return "\n".join(lines)


def evaluate_samples(
    samples: Sequence[CalibrationSample],
    calibration_type: CalibrationType,
    *,
    solve_count: int = 15,
    holdout_count: Optional[int] = None,
    config: Optional[SimpleNamespace] = None,
    rng: Optional[np.random.Generator] = None,
    allow_pruning: bool = False,
) -> EvaluationResult:
    """Complete hold-out evaluation:

    1. split the dataset into solve / hold-out subsets (disjoint);
    2. solve the hand-eye on the solve subset only (full production gates);
    3. internal metrics: constant-frame consistency of the solve subset;
    4. hold-out metrics: deviation of hold-out samples from the reference
       frame established by the solve subset alone.
    """
    solve, holdout = split_solve_holdout(samples, solve_count, holdout_count, rng=rng)
    solver_valid, handeye, algorithm, details = solve_handeye(
        solve, calibration_type, config, allow_pruning=allow_pruning,
    )
    reference = reference_pose([constant_frame_pose(sample, handeye) for sample in solve])
    internal_metrics = constant_frame_metrics(solve, handeye, reference=reference)
    holdout_metrics = constant_frame_metrics(holdout, handeye, reference=reference)
    return EvaluationResult(
        calibration_type=normalize_calibration_type(calibration_type),
        solve_count=len(solve),
        holdout_count=len(holdout),
        algorithm=algorithm,
        handeye=handeye,
        internal_metrics=internal_metrics,
        holdout_metrics=holdout_metrics,
        solve_details=details,
        solver_valid=solver_valid,
    )
