#!/usr/bin/env python3
"""M2.6 dual-base Monte Carlo CLI (v2, nested/paired design).

Runs the sample-count → P95 simulation on ONE fixed WIDE fixture with nested
subsets and shared noise realizations, plus a SMALL-cluster comparison, and
prints tables / saves plots.

Important (GPT review): the simulation reports *absolute* transform accuracy
(translation/rotation error vs GT, and the induced hand-over point error).
The real-robot fit/hold-out residuals are a DIFFERENT metric — internal
consistency only, they cannot detect systematic per-arm TCP bias.  Absolute
accuracy on the robot must be validated by an independent modality (M2.7/M2.8
global-camera cross-check).

Usage:
  python3 scripts/mc_base_alignment.py \
      --trials 2000 --outdir /tmp/m2_6 \
      [--counts 4,6,10,15,20] [--seed 0] [--plot]
"""

import argparse
import json
import sys
from pathlib import Path

import numpy as np

try:
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from dual_arm_calibration import monte_carlo as mc
except ImportError as exc:  # pragma: no cover
    print(f"cannot import dual_arm_calibration: {exc}", file=sys.stderr)
    sys.exit(2)


def _format_row(row: dict) -> str:
    return (
        f"  N={row['sample_count']:>2}  "
        f"GT trans P95={row['translation_p95_m'] * 1000:7.2f} mm  "
        f"(P50={row['translation_p50_m'] * 1000:6.2f}, MAX={row['translation_max_m'] * 1000:7.2f})  "
        f"GT rot P95={row['rotation_p95_deg']:6.3f} deg  "
        f"(P50={row['rotation_p50_deg']:5.3f}, MAX={row['rotation_max_deg']:6.3f})  "
        f"handover-pt P95={row['handover_point_p95_m'] * 1000:6.2f} mm"
    )


def _main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--trials", type=int, default=2000, help="Monte Carlo trials per N (default 2000)")
    parser.add_argument("--counts", default="4,6,10,15,20", help="comma-separated sample counts (nested subsets)")
    parser.add_argument("--seed", type=int, default=0, help="RNG seed")
    parser.add_argument("--outdir", default="/tmp/m2_6", help="output directory for plots/JSON")
    parser.add_argument("--plot", action="store_true", help="save matplotlib PNG plots")
    parser.add_argument("--tool-frame-tcp", action="store_true",
                        help="model TCP bias in the tool frame (rotated per touch pose) "
                             "instead of a constant base-frame offset")
    args = parser.parse_args(argv)

    counts = tuple(int(v) for v in args.counts.split(",") if v.strip())
    if args.trials <= 0:
        parser.error("--trials must be positive")
    if any(c < 3 for c in counts):
        parser.error("all sample counts must be >= 3")

    outdir = Path(args.outdir)
    outdir.mkdir(parents=True, exist_ok=True)

    noise = mc.NoiseModel(fixed_probe_orientation=not args.tool_frame_tcp)
    print("=" * 88)
    print(f"M2.6 dual-base Monte Carlo (v2 nested/paired) — trials={args.trials} seed={args.seed}")
    tcp_mode = "tool-frame (rotated per touch pose)" if args.tool_frame_tcp else "base-frame constant"
    print(f"  TCP model: {tcp_mode}")
    print(f"  noise (per-axis σ, 3D RMS = σ·√3):")
    print(f"    touch={noise.touch_axis_sigma_m*1000:.2f} mm/axis "
          f"tcp={noise.tcp_axis_sigma_m*1000:.2f} mm/axis "
          f"(3D RMS {noise.tcp_axis_sigma_m*np.sqrt(3)*1000:.2f} mm) "
          f"fk={noise.fk_axis_sigma_m*1000:.2f} mm/axis")
    print(f"  GT ᵃᴮᴸT ᴮᴿ: Rz(180°) + t=(0.7, 0, 0) (from dual-arm URDF)")
    print(f"  hand-over workspace points: max induced error over "
          f"{len(mc.HANDOVER_POINTS)} points")
    print("=" * 88)

    comparison = mc.compare_coverage(counts, trials=args.trials, noise=noise, seed=args.seed)

    print(f"\n--- WIDE (fixed 20-pt fixture, nested farthest-point subsets) ---")
    for row in comparison["wide"]["rows"]:
        print(_format_row(row))
    print(f"\n--- SMALL cluster (independent per-N layout; coverage contrast only) ---")
    for row in comparison["small"]["rows"]:
        print(_format_row(row))

    print("\n" + "=" * 88)
    print("  How to read:")
    print("  - WIDE curve isolates the 'more points' effect (paired design).")
    print("  - SMALL curve shows the coverage penalty; do NOT use it for 15-vs-20.")
    print("  - GT errors are ABSOLUTE accuracy. The real-robot fit/hold-out")
    print("    residuals are INTERNAL consistency only and cannot detect")
    print("    systematic per-arm TCP bias — validate absolutely via M2.7/M2.8")
    print("    global-camera cross-check.")
    print("=" * 88)

    payload = {
        "trials": args.trials,
        "seed": args.seed,
        "noise": {**vars(noise), "tcp_3d_rms_m": float(noise.tcp_axis_sigma_m * np.sqrt(3))},
        "wide": comparison["wide"]["rows"],
        "small": comparison["small"]["rows"],
    }
    json_path = outdir / "m2_6_report_v2.json"
    json_path.write_text(json.dumps(payload, indent=2, default=float), encoding="utf-8")
    print(f"report written: {json_path}")

    if args.plot:
        try:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
        except ImportError:
            print("matplotlib not available — skipping plots", file=sys.stderr)
            return 0
        fig, axes = plt.subplots(1, 3, figsize=(16, 4.5))
        for block, color, marker in ((comparison["wide"], "#1f77b4", "o"), (comparison["small"], "#d62728", "s")):
            rows = block["rows"]
            counts_arr = [r["sample_count"] for r in rows]
            axes[0].plot(counts_arr, [r["translation_p95_m"] * 1000 for r in rows],
                         marker=marker, color=color, label="wide" if block is comparison["wide"] else "small")
            axes[1].plot(counts_arr, [r["rotation_p95_deg"] for r in rows],
                         marker=marker, color=color, label="wide" if block is comparison["wide"] else "small")
            axes[2].plot(counts_arr, [r["handover_point_p95_m"] * 1000 for r in rows],
                         marker=marker, color=color, label="wide" if block is comparison["wide"] else "small")
        axes[0].set(xlabel="sample count N", ylabel="GT translation P95 (mm)", title="Transform translation error")
        axes[1].set(xlabel="sample count N", ylabel="GT rotation P95 (deg)", title="Transform rotation error")
        axes[2].set(xlabel="sample count N", ylabel="hand-over point P95 (mm)", title="Induced point error @ workspace")
        for ax in axes:
            ax.grid(True, alpha=0.3)
            ax.legend()
        fig.tight_layout()
        plot_path = outdir / "m2_6_p95_curves_v2.png"
        fig.savefig(plot_path, dpi=150)
        print(f"plot written: {plot_path}")
    return 0


if __name__ == "__main__":
    sys.exit(_main())
