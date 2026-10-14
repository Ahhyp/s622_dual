#!/usr/bin/env python3
"""C4.2 / S6: baseline comparison (plan §7.1), ablation, weight sensitivity and
multi-seed statistics.

Writes
    figs/c42_s6_report.json
    figs/fig7_sensitivity.png
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

OBJ = ["J1_smin_L", "J2_smin_R", "J3_w_rel", "J4_arm_arm",
       "J5_margin", "J6_closeness", "J7_continuity", "J8_arm_prox"]
COLS = ["smin_left", "smin_right", "w_rel", "mesh_arm_arm",
        "joint_margin", "deviation", "continuity", "arm_proximal"]
DIRECTION = [+1, +1, +1, +1, +1, -1, +1, +1]
#: metrics where "improvement" is a relative change; the rest use the special rules
CONTINUOUS = {0, 1, 2, 3, 4, 7}


def clamp01(v):
    return float(min(1.0, max(0.0, v)))


def normalise(raw, scales):
    """Vectorised: every objective to [0,1], bigger == better."""
    j = raw / np.asarray(scales)[None, :]
    j[:, 5] = 1.0 - j[:, 5]          # closeness is "smaller deviation is better"
    return np.clip(j, 0.0, 1.0)


def load_s4(path):
    rows = [r for r in csv.DictReader(open(path)) if r["s4"] == "1"]
    raw = np.array([[float(r[c]) for c in COLS] for r in rows])
    P = np.array([[float(r["px"]), float(r["py"]), float(r["pz"])] for r in rows])
    return rows, raw, P


def pareto_key(norm):
    key = np.column_stack([np.minimum(norm[:, 0], norm[:, 1]), norm[:, 7], norm[:, 5]])
    order = np.argsort(-key[:, 0], kind="stable")
    o = key[order]
    front = []
    for i in range(o.shape[0]):
        p = o[i]
        if front:
            q = o[np.asarray(front, dtype=int)]
            if np.any(np.all(q >= p, axis=1)):
                continue
            front = [f for f, k in zip(front, ~np.all(p >= q, axis=1)) if k]
        front.append(i)
    m = np.zeros(o.shape[0], dtype=bool)
    m[np.asarray(front, dtype=int)] = True
    out = np.zeros(norm.shape[0], dtype=bool)
    out[order[m]] = True
    return out


def topk_idx(score, k=5):
    return set(np.argsort(-score)[:k].tolist())


def jaccard(a, b):
    return len(a & b) / max(len(a | b), 1)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--csv", required=True)
    ap.add_argument("--s4-json", required=True)
    ap.add_argument("--s5-json", required=True)
    ap.add_argument("--scale-variant-json", required=True)
    ap.add_argument("--out-dir", required=True)
    args = ap.parse_args()
    os.makedirs(args.out_dir, exist_ok=True)

    s4 = json.load(open(args.s4_json))
    scales_fixed = np.array([o["scale_value"] for o in s4["objectives"]])
    s5 = json.load(open(args.s5_json))
    s5v = json.load(open(args.scale_variant_json))
    scales_p95 = np.array(s5v["scales"])

    rows, raw, P = load_s4(args.csv)
    print(f"[s6] {len(rows):,} S4 candidates")

    # ---------------------------------------------------------- saturation
    sat = {}
    for i, name in enumerate(OBJ):
        for tag, sc in (("fixed", scales_fixed), ("p95", scales_p95)):
            v = raw[:, i] / sc[i]
            if DIRECTION[i] < 0:
                v = 1.0 - v
            sat.setdefault(name, {})[tag] = round(float(np.mean(v >= 1.0)), 4)
    per_cand = Counter()
    for r_i in range(raw.shape[0]):
        v = raw[r_i] / scales_fixed
        v[5] = 1.0 - v[5]
        per_cand[int(np.sum(v >= 1.0))] += 1
    print("[s6] saturated objectives per candidate (fixed scales):", dict(sorted(per_cand.items())))

    # -------------------------------------------- plan §7.1 comparison table
    def gain(kind, m_opt, m_base):
        if m_base == 0:
            return None
        if kind == "relative":
            return (m_opt - m_base) / m_base
        if kind == "relative_down":
            return (m_base - m_opt) / m_base
        return None

    # the C++ JSON names the all-pairs clearance "arm_arm"
    JSON_KEY = {"mesh_arm_arm": "arm_arm"}

    def result_metrics(d):
        return {c: float(d["metrics"][JSON_KEY.get(c, c)]) for c in COLS}

    b = result_metrics(s5["baseline_eval_same_pipeline"])
    comparison = {}
    for tag, d in (("V0", s5["V0"]), ("V1", s5["V1"])):
        m = result_metrics(d)
        entry = {}
        for i, c in enumerate(COLS):
            kind = "relative" if i in CONTINUOUS else "relative"
            entry[c] = {
                "baseline": round(b[c], 6),
                "optimized": round(m[c], 6),
                "abs_change": round(m[c] - b[c], 6),
                "rel_change_pct": round(100.0 * gain(kind, m[c], b[c]), 2) if b[c] else None,
            }
        comparison[tag] = entry
    # weighted score is a ratio-like quantity -> report the absolute delta too
    comparison["score"] = {
        "baseline": round(float(s5["baseline_eval_same_pipeline"]["score"]), 6),
        "V0": round(float(s5["V0"]["score"]), 6),
        "V1": round(float(s5["V1"]["score"]), 6),
        "V1_rel_change_pct": round(
            100.0 * (s5["V1"]["score"] - s5["baseline_eval_same_pipeline"]["score"])
            / s5["baseline_eval_same_pipeline"]["score"], 2),
    }

    # ------------------------------------------------ scale-variant comparison
    bv = result_metrics(s5v["baseline_eval_same_pipeline"])
    mv = result_metrics(s5v["V1"])
    scale_variant = {}
    for i, c in enumerate(COLS):
        scale_variant[c] = {
            "baseline": round(bv[c], 6), "optimized_p95": round(mv[c], 6),
            "rel_change_pct": round(100.0 * (mv[c] - bv[c]) / bv[c], 2) if bv[c] else None,
        }
    d_opt = float(np.linalg.norm(np.array(s5["V1"]["p_O"]) - np.array(s5v["V1"]["p_O"])))
    print(f"[s6] V1 optimum moves {d_opt:.4f} m between the two scale variants")

    # ---------------------------------------------------------------- ablation
    norm_fixed = normalise(raw, scales_fixed)
    front = pareto_key(norm_fixed)
    fi = np.where(front)[0]
    w0 = np.ones(8)
    nominal = np.zeros(8)
    nominal[:] = w0 / w0.sum()
    base_score = norm_fixed @ nominal
    base_pick = int(fi[int(np.argmax(base_score[fi]))])

    ablation = {}
    for i, name in enumerate(OBJ):
        w = w0.copy()
        w[i] = 0.0
        s = norm_fixed @ (w / w.sum())
        pick = int(fi[int(np.argmax(s[fi]))])
        ablation[name] = {
            "dropped": name,
            "picked_p_O": [round(float(v), 4) for v in P[pick]],
            "moved_m": round(float(np.linalg.norm(P[pick] - P[base_pick])), 4),
            "score_of_new_pick_under_full_weights": round(float(base_score[pick]), 5),
        }
    ablation["_nominal"] = {"picked_p_O": [round(float(v), 4) for v in P[base_pick]],
                            "score": round(float(base_score[base_pick]), 5)}

    # ------------------------------------------------- weight sensitivity (+-50%)
    base_top5 = topk_idx(base_score, 5)
    wsens = {}
    for i, name in enumerate(OBJ):
        for mult, tag in ((1.5, "plus50"), (0.5, "minus50")):
            w = w0.copy()
            w[i] *= mult
            s = norm_fixed @ (w / w.sum())
            wsens.setdefault(name, {})[tag] = round(jaccard(base_top5, topk_idx(s, 5)), 4)

    # ------------------------------------------------------------- multi-seed
    ms = s5.get("multi_seed")
    multi = None
    if ms:
        arr = np.array(ms["scores"])
        best = float(arr.max())
        multi = {
            "runs": int(ms["runs"]),
            "feasible": int(ms["feasible"]),
            "p_opt_1pct_of_best": round(float(np.mean(arr >= 0.99 * best)), 4),
            "mean": round(float(arr.mean()), 6),
            "std": round(float(arr.std()), 6),
            "min": round(float(arr.min()), 6),
            "max": best,
            "scores": [round(float(v), 6) for v in arr],
        }
        print(f"[s6] multi-seed: p_opt(1% of best) = {multi['p_opt_1pct_of_best']}, "
              f"std {multi['std']}")

    # ---------------------------------------------------------------- figure
    fig, axes = plt.subplots(2, 2, figsize=(16, 12))

    ax = axes[0][0]
    if multi:
        ax.hist(multi["scores"], bins=12, color="steelblue", edgecolor="k")
        ax.axvline(multi["max"], color="crimson", ls="--",
                   label=f"best {multi['max']:.5f}")
        ax.axvline(0.99 * multi["max"], color="orange", ls=":",
                   label="99% of best")
        ax.set_xlabel("final weighted score")
        ax.set_ylabel("runs")
        ax.set_title(f"multi-seed: {multi['runs']} runs, "
                     f"p_opt(>=99% of best) = {multi['p_opt_1pct_of_best']}")
        ax.legend(fontsize=8)
    ax.grid(alpha=0.25)

    ax = axes[0][1]
    names = [o for o in OBJ]
    moved = [ablation[o]["moved_m"] for o in names]
    ax.barh(names, moved, color="indianred")
    ax.set_xlabel("distance the pick moves when the objective is dropped [m]")
    ax.set_title("ablation: influence of each objective on the selection")
    ax.grid(alpha=0.25, axis="x")

    ax = axes[1][0]
    x = np.arange(len(OBJ))
    wp = [wsens[o]["plus50"] for o in OBJ]
    wm = [wsens[o]["minus50"] for o in OBJ]
    ax.bar(x - 0.2, wp, 0.4, label="weight x1.5", color="seagreen")
    ax.bar(x + 0.2, wm, 0.4, label="weight x0.5", color="goldenrod")
    ax.set_xticks(x)
    ax.set_xticklabels(OBJ, rotation=30, ha="right", fontsize=8)
    ax.set_ylabel("Top-5 Jaccard vs nominal")
    ax.set_ylim(0, 1.05)
    ax.axhline(1.0, color="k", lw=0.5)
    ax.set_title("weight sensitivity (+-50%, one objective at a time)")
    ax.legend(fontsize=8)
    ax.grid(alpha=0.25, axis="y")

    ax = axes[1][1]
    ax.bar(x - 0.2, [sat[o]["fixed"] for o in OBJ], 0.4, label="fixed scales",
           color="crimson")
    ax.bar(x + 0.2, [sat[o]["p95"] for o in OBJ], 0.4, label="P95 scales",
           color="seagreen")
    ax.set_xticks(x)
    ax.set_xticklabels(OBJ, rotation=30, ha="right", fontsize=8)
    ax.set_ylabel("fraction of S4 candidates CLIPPED at 1.0")
    ax.set_title("objective saturation: fixed vs P95 scales")
    ax.legend(fontsize=8)
    ax.grid(alpha=0.25, axis="y")

    fig.suptitle("C4.2 / S6 — baseline comparison, ablation, weight sensitivity, multi-seed")
    fig.tight_layout()
    fig_path = os.path.join(args.out_dir, "fig7_sensitivity.png")
    fig.savefig(fig_path, dpi=130)
    plt.close(fig)

    out = {
        "comparison_vs_baseline_B": comparison,
        "scale_variant": {
            "scales_fixed": [round(float(v), 6) for v in scales_fixed],
            "scales_p95": [round(float(v), 6) for v in scales_p95],
            "V1_optimum_shift_m": round(d_opt, 4),
            "metrics": scale_variant,
        },
        "saturation": sat,
        "saturated_objectives_per_candidate_fixed": {str(k): int(v)
                                                     for k, v in sorted(per_cand.items())},
        "ablation": ablation,
        "weight_sensitivity_jaccard_top5": wsens,
        "multi_seed": multi,
        "notes": {
            "gain_convention": "C4.2 plan §7.1: continuous quantities use relative "
                               "change; rates would use percentage points; times use "
                               "ratios. No rates/times are reported at this stage.",
            "saturation_issue": "with the original FIXED scales, J8 was clipped for "
                                "94.8% and J5 for 58.7% of candidates, which silently "
                                "removed them from the optimisation. The P95 variant "
                                "fixes that and changes the recommended pose.",
        },
        "figures": [os.path.basename(fig_path)],
    }
    jp = os.path.join(args.out_dir, "c42_s6_report.json")
    json.dump(out, open(jp, "w"), indent=2, ensure_ascii=False)
    print("[s6] wrote", jp, "and", fig_path)
    return 0


if __name__ == "__main__":
    sys.exit(main())
