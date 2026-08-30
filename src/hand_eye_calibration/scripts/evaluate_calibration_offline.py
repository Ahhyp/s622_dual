#!/usr/bin/env python3
"""Offline constant-frame hold-out calibration evaluation (M2.2).

Pure-Python CLI (no ROS node, no TF): evaluates a collector ``.samples`` file
with hold-out separation and constant-frame metrics.

Modes:
  * re-solve hold-out (default): split the dataset into solve / hold-out
    subsets, solve the hand-eye on the solve subset only (full production
    quality gates), score the hold-out subset against the solve-set reference
    frame.  This is the honest independent metric — samples used for solving
    never score themselves, and the reference frame comes from the solve
    subset alone.
  * saved-calibration check (``--calib`` without ``--solve-count``): report
    constant-frame self-consistency of the samples against an existing
    ``.calib`` hand-eye.  Only meaningful when the samples were NOT used to
    compute that calibration (e.g. a fresh run on the real robot).  The
    reference frame is derived from the very samples being scored, so this is
    a fresh-data consistency check — NOT a strict hold-out score.

Usage:
  python3 scripts/evaluate_calibration_offline.py \
      --samples calib/sim/robot_calibration_XXXX_eye_in_hand.samples \
      [--solve-count 15] [--holdout-count 5] [--seed 0]
  python3 scripts/evaluate_calibration_offline.py \
      --samples calib/sim/robot_calibration_XXXX_eye_in_hand.samples \
      --calib calib/sim/robot_calibration_XXXX_eye_in_hand.calib
"""

import argparse
import json
import sys
from pathlib import Path

import numpy as np

try:
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from hand_eye_calibration import evaluator
    from hand_eye_calibration.solver import rotation_delta_deg
except ImportError as exc:  # pragma: no cover
    print(f"cannot import hand_eye_calibration: {exc}", file=sys.stderr)
    sys.exit(2)


def _main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--samples", required=True, help="collector .samples YAML file")
    parser.add_argument("--calib", default=None, help="saved .calib hand-eye to check (optional)")
    parser.add_argument("--solve-count", type=int, default=None, help="samples used for solving (default 15; omit with --calib for saved-calib check)")
    parser.add_argument("--holdout-count", type=int, default=None, help="hold-out sample count")
    parser.add_argument("--seed", type=int, default=None, help="random split seed (default: deterministic spread)")
    parser.add_argument("--json", default=None, help="write machine-readable report to this path")
    args = parser.parse_args(argv)

    samples_path = Path(args.samples)
    if not samples_path.exists():
        print(f"error: samples file not found: {samples_path}", file=sys.stderr)
        return 2

    kind, samples = evaluator.load_samples_yaml(samples_path)
    print(f"samples: {samples_path}")
    print(f"calibration_type: {kind.value}  total samples: {len(samples)}")

    rng = None if args.seed is None else np.random.default_rng(args.seed)

    if args.solve_count is None:
        if args.calib:
            # saved-calibration check mode: score all samples against the calib.
            calib = evaluator.load_calibration_yaml(args.calib)
            if calib.calibration_type is not kind:
                print(
                    f"error: .calib type {calib.calibration_type.value} does not match "
                    f".samples type {kind.value}",
                    file=sys.stderr,
                )
                return 2
            metrics = evaluator.constant_frame_metrics(samples, calib.transform)
            print("=" * 62)
            print(f"  Saved-calibration check (fresh-data self-consistency): {args.calib}")
            print(f"  position RMS/P95/MAX = {metrics['position_rms_m'] * 1000:.2f} / "
                  f"{metrics['position_p95_m'] * 1000:.2f} / {metrics['position_max_m'] * 1000:.2f} mm")
            print(f"  rotation RMS/P95/MAX = {metrics['rotation_rms_deg']:.3f} / "
                  f"{metrics['rotation_p95_deg']:.3f} / {metrics['rotation_max_deg']:.3f} deg")
            print("  注：参考帧由本次样本自身导出，属自洽性检查（非严格 hold-out）")
            print("=" * 62)
            if args.json:
                Path(args.json).write_text(json.dumps({
                    "mode": "saved_calibration_check",
                    "calibration_type": kind.value,
                    "samples": str(samples_path),
                    "calib": args.calib,
                    "metrics": {k: v for k, v in metrics.items() if k != "per_sample_position_m" and k != "per_sample_rotation_deg"},
                }, indent=2))
            return 0
        args.solve_count = 15  # default re-solve hold-out mode

    result = evaluator.evaluate_samples(
        samples, kind,
        solve_count=args.solve_count,
        holdout_count=args.holdout_count,
        rng=rng,
    )
    print(result.format_report())

    if args.calib:
        saved = evaluator.load_calibration_yaml(args.calib)
        if saved.calibration_type is not kind:
            print(
                f"error: .calib type {saved.calibration_type.value} does not match "
                f".samples type {kind.value}",
                file=sys.stderr,
            )
            return 2
        delta_t = np.asarray(saved.transform.translation) - np.asarray(result.handeye.translation)
        print(f"  Re-solved vs saved {args.calib}:")
        print(f"    rotation delta = {rotation_delta_deg(saved.transform.rotation, result.handeye.rotation):.3f} deg "
              f"| t delta = {float(np.linalg.norm(delta_t)) * 1000:.2f} mm")

    if args.json:
        Path(args.json).write_text(json.dumps({
            "mode": "holdout_resolve",
            "calibration_type": kind.value,
            "samples": str(samples_path),
            "solve_count": result.solve_count,
            "holdout_count": result.holdout_count,
            "algorithm": result.algorithm,
            "solver_valid": result.solver_valid,
            "internal": {k: v for k, v in result.internal_metrics.items() if k not in ("per_sample_position_m", "per_sample_rotation_deg")},
            "holdout": {k: v for k, v in result.holdout_metrics.items() if k not in ("per_sample_position_m", "per_sample_rotation_deg")},
            "passed_gates": bool(result.passed_gates()),
            "handeye": {
                "translation": list(result.handeye.translation),
                "quaternion": list(result.handeye.rotation.as_quat()),
            },
        }, indent=2))
        print(f"report written: {args.json}")
    return 0


if __name__ == "__main__":
    sys.exit(_main())
