#!/usr/bin/env python3
"""C4.2 / S4: normalised soft objectives, single-objective poles, weighted
selection and the Pareto front.

Reads the S3 candidate table (c42_s3_candidates.csv, produced by c42_reachability)
and writes

    figs/c42_s4_objectives.json   scales, poles, weight sets, Pareto, recommendation
    figs/fig5_pareto.png          Pareto front + single-objective poles

Design notes
------------
* Every objective is normalised to [0,1] against a DECLARED scale before any
  weighting; no raw quantities are ever added together (plan §3.4).
* J4 uses the all-pairs cross-arm clearance -- the true safety figure -- but the
  S3 sweep showed it is geometrically locked (std 1.9 mm) because a rigid grasp
  fixes T_rel and therefore every distal-distal distance.  J8 is added on top:
  the same clearance restricted to pairs with at least one PROXIMAL link, which
  is where the 12-DOF redundancy can actually act (std 50.6 mm, 26x the spread).
  J4 stays in as a floor/veto term, J8 is the optimisable one.
* The recommendation is a REPRODUCIBLE RULE, not a hand pick: among the Pareto
  front, take the candidate with the best equal-weight score.
"""

from __future__ import annotations

import argparse
import csv
import json
import os
import sys
from collections import Counter

import numpy as np

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

# --------------------------------------------------------------------------
# Objectives:  id -> (csv column, direction, scale rule, label)
#   direction +1 = bigger is better, -1 = smaller is better
# --------------------------------------------------------------------------
OBJECTIVES = [
    ("J1_sigma_min_left",  "smin_left",      +1, "p95", "sigma_min(J_L)"),
    ("J2_sigma_min_right", "smin_right",     +1, "p95", "sigma_min(J_R)"),
    ("J3_w_rel",           "w_rel",          +1, "p95", "w_rel (BodyR)"),
    ("J4_arm_arm",         "mesh_arm_arm",   +1, "fixed:0.05", "cross-arm clearance (all pairs)"),
    ("J5_joint_margin",    "joint_margin",   +1, "fixed:0.5", "joint-limit margin [rad]"),
    ("J6_closeness",       "deviation",      -1, "p95", "proximity to Baseline B"),
    ("J7_continuity",      "continuity",     +1, "fixed:1.0", "IK continuity score"),
    ("J8_arm_proximal",    "arm_proximal",   +1, "fixed:0.05", "cross-arm clearance (proximal pairs)"),
]

#: Weight sets.  Documented up front; S6 does the full sensitivity sweep.
WEIGHT_SETS = {
    "equal": {k: 1.0 for k, *_ in OBJECTIVES},
    "precision_first": {
        "J1_sigma_min_left": 3.0, "J2_sigma_min_right": 3.0, "J3_w_rel": 3.0,
        "J4_arm_arm": 0.5, "J5_joint_margin": 1.0, "J6_closeness": 1.0,
        "J7_continuity": 1.0, "J8_arm_proximal": 0.5,
    },
    "safety_first": {
        "J1_sigma_min_left": 1.0, "J2_sigma_min_right": 1.0, "J3_w_rel": 1.0,
        "J4_arm_arm": 4.0, "J5_joint_margin": 3.0, "J6_closeness": 0.5,
        "J7_continuity": 0.5, "J8_arm_proximal": 4.0,
    },
}


def load_rows(path):
    rows = []
    with open(path, newline="") as fh:
        for r in csv.DictReader(fh):
            if r["s4"] != "1":
                continue
            rows.append(r)
    if not rows:
        raise SystemExit("no S4 candidates in " + path)
    return rows


def scales_for(rows):
    scales = {}
    for key, col, _dir, rule, _label in OBJECTIVES:
        vals = np.array([float(r[col]) for r in rows])
        vals = vals[np.isfinite(vals)]
        if rule == "p95":
            s = float(np.percentile(vals, 95))
            if s <= 0.0:
                s = float(vals.max()) if vals.size else 1.0
        else:
            s = float(rule.split(":")[1])
        scales[key] = s if s > 0 else 1.0
    return scales


def raw_matrix(rows):
    cols = np.zeros((len(rows), len(OBJECTIVES)), dtype=float)
    for j, (_k, col, _d, _r, _l) in enumerate(OBJECTIVES):
        cols[:, j] = [float(r[col]) for r in rows]
    return cols


def normalise(raw, scales):
    """-> [0,1] with bigger == better."""
    out = np.zeros_like(raw)
    for j, (key, _col, direction, _rule, _l) in enumerate(OBJECTIVES):
        s = scales[key]
        v = raw[:, j] / s
        if direction < 0:
            v = 1.0 - v
        out[:, j] = np.clip(v, 0.0, 1.0)
    return out


def weighted(norm, weights):
    w = np.array([weights[k] for k, *_ in OBJECTIVES], dtype=float)
    w = w / w.sum()
    return norm @ w


def pareto_mask(norm, seed=20261014, max_points=6000):
    """Non-dominated set for a maximisation problem.

    High-dimensional Pareto fronts are both expensive (O(n * |front|)) and, at
    m = 8, not very selective -- see the reported front fraction.  The full-set
    front is therefore computed on a SEEDED SUBSAMPLE, and the caller also
    computes a low-dimensional (3-objective) front over everything, which is the
    interpretable one.
    """
    n = norm.shape[0]
    idx_all = np.arange(n)
    if n > max_points:
        rng = np.random.default_rng(seed)
        idx_all = np.sort(rng.choice(n, size=max_points, replace=False))
    sub = norm[idx_all]
    m = sub.shape[0]

    order = np.argsort(-sub[:, 0], kind="stable")
    ordered = sub[order]
    front = []
    for i in range(m):
        p = ordered[i]
        if front:
            q = ordered[np.asarray(front, dtype=int)]
            dominated = np.any(np.all(q >= p, axis=1))
            if dominated:
                continue
            keep = ~np.all(p >= q, axis=1)
            front = [f for f, k in zip(front, keep) if k]
        front.append(i)

    out = np.zeros(n, dtype=bool)
    out[idx_all[np.asarray(front, dtype=int)]] = True
    return out


def short(r):
    return {
        "pool": r["pool"],
        "p_O": [round(float(r["px"]), 4), round(float(r["py"]), 4), round(float(r["pz"]), 4)],
        "smin_left": round(float(r["smin_left"]), 5),
        "smin_right": round(float(r["smin_right"]), 5),
        "w_rel": round(float(r["w_rel"]), 5),
        "arm_arm": round(float(r["mesh_arm_arm"]), 5),
        "arm_proximal": round(float(r["arm_proximal"]), 5),
        "joint_margin": round(float(r["joint_margin"]), 5),
        "deviation": round(float(r["deviation"]), 5),
        "continuity": round(float(r["continuity"]), 5),
    }


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--csv", required=True, help="c42_s3_candidates.csv")
    ap.add_argument("--out-dir", required=True)
    ap.add_argument("--baseline-pO", default="0.054908,0.000295,0.335103",
                    help="Baseline B object centre (for the delta report)")
    args = ap.parse_args()
    os.makedirs(args.out_dir, exist_ok=True)

    rows = load_rows(args.csv)
    print(f"[s4] {len(rows):,} S4 (feasible) candidates")
    print("[s4] pool mix:", dict(Counter(r["pool"] for r in rows)))

    scales = scales_for(rows)
    raw = raw_matrix(rows)
    norm = normalise(raw, scales)

    # ----------------------------------------------------------- poles
    poles = {}
    for j, (key, col, direction, _rule, label) in enumerate(OBJECTIVES):
        best = int(np.argmax(norm[:, j]))
        poles[key] = {"label": label, "value_raw": round(raw[best, j], 6),
                      "value_norm": round(float(norm[best, j]), 6),
                      "candidate": short(rows[best])}

    # ------------------------------------------------------- weighting
    for name, w in WEIGHT_SETS.items():
        pass
    scores = {name: weighted(norm, w) for name, w in WEIGHT_SETS.items()}
    best_by_w = {name: int(np.argmax(s)) for name, s in scores.items()}

    # ---------------------------------------------------------- Pareto
    # (a) 8-objective front on a seeded subsample -- report the FRACTION, which
    #     is itself a finding: at m = 8 a Pareto front is barely selective.
    pm = pareto_mask(norm)
    n_pareto = int(pm.sum())
    n_sub = int(min(len(rows), 6000))
    print(f"[s4] 8-objective Pareto front: {n_pareto} of a {n_sub} subsample "
          f"({100.0 * n_pareto / n_sub:.1f}%)")

    # (b) 3-objective KEY front over ALL candidates: the interpretable one.
    key_cols = [0, 1, 7, 5]  # sigmaL, sigmaR, arm_proximal, closeness
    key = norm[:, key_cols].copy()
    key[:, 3] = norm[:, 5]  # closeness
    key = np.column_stack([np.minimum(norm[:, 0], norm[:, 1]), norm[:, 7], norm[:, 5]])
    keym = pareto_mask(key, max_points=len(rows))
    n_key = int(keym.sum())
    print(f"[s4] key 3-objective front (sigma_min_min, arm_proximal, closeness): "
          f"{n_key} of {len(rows)} ({100.0 * n_key / len(rows):.1f}%)")

    # reproducible recommendation: best equal-weight score ON the KEY front
    eq = scores["equal"]
    key_idx = np.where(keym)[0]
    rec = int(key_idx[int(np.argmax(eq[key_idx]))])

    # ------------------------------------------------- figure
    smin_min = np.minimum(raw[:, 0], raw[:, 1])
    prox = raw[:, 7]
    fig, axes = plt.subplots(1, 2, figsize=(16, 6.5))

    ax = axes[0]
    sc = ax.scatter(smin_min, prox, c=eq, s=8, cmap="viridis", alpha=0.6)
    ax.scatter(smin_min[keym], prox[keym], facecolors="none", edgecolors="crimson",
               s=18, linewidths=0.7, label=f"key Pareto front (n={n_key})")
    for j, (key, _c, _d, _r, label) in enumerate(OBJECTIVES):
        b = int(np.argmax(norm[:, j]))
        ax.scatter(smin_min[b], prox[b], marker="*", s=170, edgecolors="k",
                   facecolors="gold", zorder=5)
        ax.annotate(key.split("_")[0], (smin_min[b], prox[b]), fontsize=8,
                    xytext=(4, 4), textcoords="offset points")
    ax.scatter(smin_min[rec], prox[rec], marker="P", s=150, facecolors="none",
               edgecolors="lime", linewidths=2.0, zorder=6, label="recommended")
    ax.set_xlabel("min(sigma_min(J_L), sigma_min(J_R))")
    ax.set_ylabel("proximal cross-arm clearance [m]")
    ax.set_title("S4 candidates: manipulability vs optimisable clearance\n"
                 "colour = equal-weight score, stars = single-objective poles")
    ax.legend(fontsize=8, loc="upper left")
    ax.grid(alpha=0.25)
    fig.colorbar(sc, ax=ax, label="equal-weight score")

    ax = axes[1]
    labels = [k.split("_", 1)[1] for k, *_ in OBJECTIVES]
    xs = np.arange(len(OBJECTIVES))
    for i in np.where(keym)[0]:
        ax.plot(xs, norm[i], color="crimson", alpha=0.04, lw=0.7)
    ax.plot(xs, norm[rec], color="lime", lw=2.5, marker="o", label="recommended")
    for j, (key, *_rest) in enumerate(OBJECTIVES):
        b = int(np.argmax(norm[:, j]))
        ax.plot(xs, norm[b], lw=1.0, ls="--", marker=".", label=f"pole {key.split('_')[0]}")
    ax.set_xticks(xs)
    ax.set_xticklabels(labels, rotation=35, ha="right", fontsize=8)
    ax.set_ylabel("normalised value (bigger = better)")
    ax.set_ylim(-0.02, 1.05)
    ax.set_title("Pareto front in objective space (parallel coordinates)")
    ax.legend(fontsize=7, ncol=2)
    ax.grid(alpha=0.25)

    fig.tight_layout()
    fig_path = os.path.join(args.out_dir, "fig5_pareto.png")
    fig.savefig(fig_path, dpi=130)
    plt.close(fig)

    # ----------------------------------------------------------- JSON
    out = {
        "n_s4_candidates": len(rows),
        "pool_mix": dict(Counter(r["pool"] for r in rows)),
        "objectives": [
            {"id": k, "column": c, "direction": ("max" if d > 0 else "min"),
             "scale_rule": r, "scale_value": round(scales[k], 6), "label": l}
            for k, c, d, r, l in OBJECTIVES
        ],
        "notes": {
            "normalisation": "every objective is scaled to [0,1] with the declared "
                             "scale before weighting; raw quantities are never added",
            "J4_vs_J8": "J4 (all cross-arm pairs) is geometrically locked by the "
                        "rigid grasp (std 1.9 mm); J8 (pairs with >=1 proximal link) "
                        "is where the redundancy acts (std 50.6 mm)",
            "recommendation_rule": "best equal-weight score among the Pareto front "
                                   "(reproducible; no hand picking)",
        },
        "poles": poles,
        "weight_sets": {k: {kk: round(vv, 4) for kk, vv in v.items()}
                        for k, v in WEIGHT_SETS.items()},
        "best_by_weight_set": {
            k: {"score": round(float(scores[k][i]), 6), "candidate": short(rows[i])}
            for k, i in best_by_w.items()
        },
        "pareto": {
            "eight_objective": {
                "subsample": n_sub,
                "front_size": n_pareto,
                "fraction": round(n_pareto / max(n_sub, 1), 4),
                "note": "computed on a seeded subsample; at m=8 the front is barely "
                        "selective, so it is reported as a finding rather than used "
                        "for selection",
            },
            "key_three_objective": {
                "objectives": ["min(smin_L, smin_R)", "arm_proximal", "closeness"],
                "front_size": n_key,
                "fraction": round(n_key / max(len(rows), 1), 4),
                "members": [short(rows[i]) for i in np.where(keym)[0][:200]],
                "members_truncated_to": 200,
            },
        },
        "recommended": {
            "rule": "argmax equal-weight score on the Pareto front",
            "score": round(float(eq[rec]), 6),
            "normalised": {k: round(float(norm[rec, j]), 5)
                           for j, (k, *_r) in enumerate(OBJECTIVES)},
            "candidate": short(rows[rec]),
        },
        "figures": [os.path.basename(fig_path)],
    }
    jpath = os.path.join(args.out_dir, "c42_s4_objectives.json")
    with open(jpath, "w") as fh:
        json.dump(out, fh, indent=2, ensure_ascii=False)
    print("[s4] wrote", jpath)
    print("[s4] wrote", fig_path)

    print("\n=== single-objective poles ===")
    for k, v in poles.items():
        print(f"  {k:22s} {v['value_raw']:>10.5f}   p_O={v['candidate']['p_O']}")
    print("\n=== best by weight set ===")
    for k, v in out["best_by_weight_set"].items():
        print(f"  {k:16s} score={v['score']:.4f}  p_O={v['candidate']['p_O']}")
    base_pO = np.array([float(x) for x in args.baseline_pO.split(",")])
    rec_pO = np.array(out["recommended"]["candidate"]["p_O"])
    out["recommended"]["delta_vs_baseline_B"] = {
        "dp_O": [round(float(v), 4) for v in (rec_pO - base_pO)],
        "distance_m": round(float(np.linalg.norm(rec_pO - base_pO)), 4),
    }

    print("\n=== recommended ===")
    r = out["recommended"]
    print(f"  score={r['score']:.4f}  p_O={r['candidate']['p_O']}")
    print(f"  delta vs Baseline B: {r['delta_vs_baseline_B']['dp_O']} m "
          f"(|d|={r['delta_vs_baseline_B']['distance_m']} m)")
    print(f"  {json.dumps(r['candidate'], ensure_ascii=False)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
