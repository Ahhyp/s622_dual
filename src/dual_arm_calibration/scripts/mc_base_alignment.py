#!/usr/bin/env python3
"""M2.6 dual-base Monte Carlo CLI.

Runs the sample-count → P95 simulation for both fixture strategies (wide
hand-over coverage vs small cluster) and prints tables / saves plots.

Usage:
  python3 scripts/mc_base_alignment.py \
      --trials 500 --outdir /tmp/m2_6 \
      [--counts 4,6,10,15,20,30] [--seed 0]
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
        f"translation P95={row['translation_p95_m'] * 1000:7.2f} mm  "
        f"(P50={row['translation_p50_m'] * 1000:6.2f}, MAX={row['translation_max_m'] * 1000:7.2f})  "
        f"rotation P95={row['rotation_p95_deg']:6.3f} deg  "
        f"(P50={row['rotation_p50_deg']:5.3f}, MAX={row['rotation_max_deg']:6.3f})"
    )


def _main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--trials", type=int, default=500, help="Monte Carlo trials per N (default 500)")
    parser.add_argument("--counts", default="4,6,10,15,20,30", help="comma-separated sample counts")
    parser.add_argument("--seed", type=int, default=0, help="RNG seed")
    parser.add_argument("--outdir", default="/tmp/m2_6", help="output directory for plots/JSON")
    parser.add_argument("--plot", action="store_true", help="save matplotlib PNG plots")
    args = parser.parse_args(argv)

    counts = tuple(int(v) for v in args.counts.split(",") if v.strip())
    outdir = Path(args.outdir)
    outdir.mkdir(parents=True, exist_ok=True)

    noise = mc.NoiseModel()
    print("=" * 78)
    print(f"M2.6 dual-base Monte Carlo — trials={args.trials} seed={args.seed}")
    print(f"  noise: touch={noise.touch_sigma_m*1000:.2f}mm  "
          f"tcp(systematic)={noise.tcp_error_sigma_m*1000:.2f}mm  "
          f"fk={noise.fk_error_sigma_m*1000:.2f}mm")
    print(f"  GT {chr(0x1D43)}BL T BR: Rz(180°) + t=(0.7, 0, 0) (from dual-arm URDF)")
    print("=" * 78)

    comparison = mc.compare_coverage(counts, trials=args.trials, noise=noise, seed=args.seed)

    for label, block in (("WIDE hand-over coverage", comparison["wide"]), ("SMALL cluster (poor coverage)", comparison["small"])):
        print(f"\n--- {label} ---")
        for row in block["rows"]:
            print(_format_row(row))

    # Acceptance reference (M2.5): fit RMS ≤ 1.5~2 mm → translate to P95 bound
    print("\n" + "=" * 78)
    print("  M2.5 real-hardware acceptance reference: base residual ≤ 2 mm")
    print("  → pick the smallest N whose translation P95 < 2 mm (wide strategy)")
    print("=" * 78)

    payload = {
        "trials": args.trials,
        "seed": args.seed,
        "noise": {
            "touch_sigma_m": noise.touch_sigma_m,
            "tcp_error_sigma_m": noise.tcp_error_sigma_m,
            "fk_error_sigma_m": noise.fk_error_sigma_m,
        },
        "wide": comparison["wide"]["rows"],
        "small": comparison["small"]["rows"],
    }
    json_path = outdir / "m2_6_report.json"
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
        fig, axes = plt.subplots(1, 2, figsize=(12, 4.5))
        for block, color, marker in ((comparison["wide"], "#1f77b4", "o"), (comparison["small"], "#d62728", "s")):
            rows = block["rows"]
            counts_arr = [r["sample_count"] for r in rows]
            axes[0].plot(counts_arr, [r["translation_p95_m"] * 1000 for r in rows],
                         marker=marker, color=color, label="wide" if block is comparison["wide"] else "small")
            axes[1].plot(counts_arr, [r["rotation_p95_deg"] for r in rows],
                         marker=marker, color=color, label="wide" if block is comparison["wide"] else "small")
        axes[0].axhline(2.0, color="gray", linestyle="--", linewidth=0.8)
        axes[0].text(4, 2.05, "M2.5 target 2 mm", fontsize=8, color="gray")
        axes[0].set(xlabel="sample count N", ylabel="translation P95 (mm)", title="Translation error")
        axes[1].set(xlabel="sample count N", ylabel="rotation P95 (deg)", title="Rotation error")
        for ax in axes:
            ax.grid(True, alpha=0.3)
            ax.legend()
        fig.tight_layout()
        plot_path = outdir / "m2_6_p95_curves.png"
        fig.savefig(plot_path, dpi=150)
        print(f"plot written: {plot_path}")
    return 0


if __name__ == "__main__":
    sys.exit(_main())
