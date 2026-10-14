#!/usr/bin/env python3
"""C4.2 / S7: one overview figure and one summary JSON for the whole stage.

Reads the JSON/CSV products of S1..S6 and writes
    figs/fig8_overview.png
    figs/c42_summary.json
"""
from __future__ import annotations

import argparse
import csv
import json
import os
import sys

import numpy as np

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

OBJ = ["J1_smin_L", "J2_smin_R", "J3_w_rel", "J4_arm_arm",
       "J5_margin", "J6_closeness", "J7_continuity", "J8_arm_prox"]
COLS = ["smin_left", "smin_right", "w_rel", "mesh_arm_arm",
        "joint_margin", "deviation", "continuity", "arm_proximal"]


def connectivity_sensitivity(P, voxels=(0.01, 0.015, 0.02, 0.03, 0.04), conns=(26, 6)):
    """The "one connected component" claim is resolution dependent -- quantify it."""
    from collections import deque, Counter

    def label(V, conn):
        lo = P.min(0) - V
        hi = P.max(0) + V
        sh = tuple(int(np.floor((hi[k] - lo[k]) / V)) + 1 for k in range(3))
        m = np.zeros(sh, bool)
        ii = np.floor((P - lo) / V).astype(int)
        m[ii[:, 0], ii[:, 1], ii[:, 2]] = True
        if conn == 26:
            off = [(a, b, c) for a in (-1, 0, 1) for b in (-1, 0, 1) for c in (-1, 0, 1)
                   if (a, b, c) != (0, 0, 0)]
        else:
            off = [(1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1)]
        lab = np.zeros(sh, np.int32)
        cur = 0
        for st in np.argwhere(m):
            if lab[tuple(st)]:
                continue
            cur += 1
            q = deque([tuple(st)])
            lab[tuple(st)] = cur
            while q:
                x, y, z = q.popleft()
                for dx, dy, dz in off:
                    a, b, c = x + dx, y + dy, z + dz
                    if (0 <= a < sh[0] and 0 <= b < sh[1] and 0 <= c < sh[2]
                            and m[a, b, c] and lab[a, b, c] == 0):
                        lab[a, b, c] = cur
                        q.append((a, b, c))
        sizes = sorted(Counter(lab[m].tolist()).values(), reverse=True)
        return int(m.sum()), cur, (sizes[0] / max(sizes and sum(sizes) or 1, 1))

    out = []
    for V in voxels:
        for cn in conns:
            nv, nc, frac = label(V, cn)
            out.append({"voxel_m": V, "connectivity": cn, "voxels": nv,
                        "components": nc, "largest_fraction": round(float(frac), 4)})
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--figs", required=True)
    ap.add_argument("--s6", required=True)
    ap.add_argument("--out-dir", required=True)
    args = ap.parse_args()
    os.makedirs(args.out_dir, exist_ok=True)
    F = args.figs

    def J(name):
        return json.load(open(os.path.join(F, name)))

    s3 = J("c42_s3_reachability.json")
    s4 = J("c42_s4_objectives.json")
    s5 = J("c42_s5_optimization.json")
    s5z = J("c42_s5_zones.json")
    s5v = J("c42_s6_scale_variant.json")
    s6 = json.load(open(args.s6))

    a = s3["funnels"]["ALL"]
    fig, axes = plt.subplots(2, 3, figsize=(21, 13))

    # ---- (a) the funnel -------------------------------------------------
    ax = axes[0][0]
    labels = ["candidates", "S0 point", "S1_L", "S1_R", "S2 dual", "S3 grasp", "S4 free"]
    vals = [a["candidates"], a["s0_point_reachable"], a["s1_left"], a["s1_right"],
            a["s2_dual"], a["s3_grasp"], a["s4_collision_free"]]
    colors = ["#888", "#9ecae1", "#6baed6", "#6baed6", "#3182bd", "#e6550d", "#31a354"]
    ax.barh(range(len(vals))[::-1], vals, color=colors)
    for i, v in enumerate(vals):
        ax.text(v * 1.01, len(vals) - 1 - i, f"{v:,}", va="center", fontsize=9)
    ax.set_yticks(range(len(vals))[::-1])
    ax.set_yticklabels(labels)
    ax.set_xlim(0, max(vals) * 1.22)
    ax.set_title(f"(a) S0->S4 funnel ({a['candidates']:,} candidates)\n"
                 f"|S2|/|S0| = {100*a['ratios']['S2_over_S0']:.1f}%, "
                 f"|S4|/|S3| = {100*a['ratios']['S4_over_S3']:.1f}%")
    ax.grid(alpha=0.25, axis="x")

    # ---- (b) saturation -------------------------------------------------
    ax = axes[0][1]
    x = np.arange(8)
    ax.bar(x - 0.2, [s6["saturation"][o]["fixed"] for o in OBJ], 0.4,
           label="fixed scales", color="crimson")
    ax.bar(x + 0.2, [s6["saturation"][o]["p95"] for o in OBJ], 0.4,
           label="P95 scales (fixed)", color="seagreen")
    ax.set_xticks(x)
    ax.set_xticklabels(OBJ, rotation=30, ha="right", fontsize=8)
    ax.set_ylabel("fraction clipped at 1.0")
    ax.set_title("(b) objective saturation (the S6 finding)\n"
                 "J8 was dead for 94.8% of candidates")
    ax.legend(fontsize=8)
    ax.grid(alpha=0.25, axis="y")

    # ---- (c) ablation ---------------------------------------------------
    ax = axes[0][2]
    moved = [s6["ablation"][o]["moved_m"] for o in OBJ]
    cols = ["crimson" if m == 0 else "steelblue" for m in moved]
    ax.barh(OBJ, moved, color=cols)
    for i, m in enumerate(moved):
        ax.text(max(m, 0) + 0.004, i, f"{m:.3f}", va="center", fontsize=8)
    ax.set_xlabel("pick movement when the objective is dropped [m]")
    ax.set_title("(c) ablation: only J3/J5/J6 matter\n"
                 "red = zero influence")
    ax.set_xlim(0, max(moved) * 1.25)
    ax.grid(alpha=0.25, axis="x")

    # ---- (d) baseline vs V0 vs V1 (P95 scales) --------------------------
    ax = axes[1][0]
    JK = {"mesh_arm_arm": "arm_arm"}   # the C++ JSON names all-pairs clearance "arm_arm"

    def prof(d):
        return np.array([float(d["metrics"][JK.get(c, c)]) for c in COLS])
    b = prof(s5v["baseline_eval_same_pipeline"])
    v1 = prof(s5v["V1"])
    rel = 100.0 * (v1 - b) / np.where(b == 0, np.nan, b)
    rel = np.nan_to_num(rel, nan=0.0)
    cc = ["seagreen" if r >= 0 else "crimson" for r in rel]
    ax.barh(COLS, rel, color=cc)
    for i, r in enumerate(rel):
        ax.text(r + (1.5 if r >= 0 else -1.5), i, f"{r:+.1f}%",
                va="center", ha="left" if r >= 0 else "right", fontsize=8)
    ax.axvline(0, color="k", lw=0.8)
    ax.set_xlabel("V1 vs Baseline B [%]  (P95 scales, recommended)")
    ax.set_title("(d) improvement over the M4 baseline")
    ax.grid(alpha=0.25, axis="x")
    ax.set_xlim(min(rel) * 1.5, max(rel) * 1.35)

    # ---- (e) cooperative zone ------------------------------------------
    ax = axes[1][1]
    rows = [r for r in csv.DictReader(open(os.path.join(F, "c42_s3_candidates.csv")))
            if r["s4"] == "1"]
    P = np.array([[float(r["px"]), float(r["py"]), float(r["pz"])] for r in rows])
    S = np.array([float(r["smin_left"]) for r in rows])
    ax.scatter(P[:, 0], P[:, 1], c=S, s=4, cmap="viridis", alpha=0.7)
    bB = np.array(s5["baseline_B"]["p_O"])
    bV0 = np.array(s5["V0"]["p_O"])
    bV1 = np.array(s5v["V1"]["p_O"])
    for p, c, lab in ((bB, "red", "Baseline B"), (bV0, "lime", "V0"),
                      (bV1, "cyan", "V1 (P95, recommended)")):
        ax.plot(p[0], p[1], marker="*", ms=20, color=c, markeredgecolor="k", ls="none",
                label=lab)
    ax.set_aspect("equal")
    ax.set_xlabel("world x [m]")
    ax.set_ylabel("world y [m]")
    ax.set_title(f"(e) cooperative zone: {s5z['cooperative_zone']['voxels']} voxels, "
                 f"{s5z['cooperative_zone']['components_total']} connected component")
    ax.legend(fontsize=8, loc="upper right")
    ax.grid(alpha=0.25)

    # ---- (f) multi-seed -------------------------------------------------
    ax = axes[1][2]
    ms = s6.get("multi_seed")
    if ms:
        ax.hist(ms["scores"], bins=12, color="steelblue", edgecolor="k")
        ax.axvline(ms["max"], color="crimson", ls="--", label=f"best {ms['max']:.5f}")
        ax.axvline(0.99 * ms["max"], color="orange", ls=":", label="99% of best")
        ax.set_xlabel("final weighted score")
        ax.set_ylabel("runs")
        ax.set_title(f"(f) multi-seed: {ms['runs']} runs, all feasible\n"
                     f"p_opt(>=99% of best) = {ms['p_opt_1pct_of_best']}, "
                     f"std = {ms['std']:.1e}")
        ax.legend(fontsize=8)
    ax.grid(alpha=0.25)

    fig.suptitle("C4.2 — dual-arm handover pose optimisation: overview of S1..S6",
                 fontsize=15)
    fig.tight_layout(rect=(0, 0, 1, 0.985))
    fig_path = os.path.join(args.out_dir, "fig8_overview.png")
    fig.savefig(fig_path, dpi=120)
    plt.close(fig)

    conn_tab = connectivity_sensitivity(P)
    print("[s7] connectivity sensitivity (voxel, conn, components, largest frac):")
    for r in conn_tab:
        print(f"    {r['voxel_m']*100:4.1f}cm {r['connectivity']:3d}-conn  "
              f"{r['components']:6d} components  largest {100*r['largest_fraction']:.1f}%")

    summary = {
        "stage": "C4.2",
        "connectivity_sensitivity": conn_tab,
        "c41_index": s3["c41_index"],
        "funnel_ALL": s3["funnels"]["ALL"],
        "nesting_violations": s3["funnels"]["ALL"].get("nesting_violations"),
        "s4": {"n_s4": s4["n_s4_candidates"],
               "pareto_8obj_fraction": s4["pareto"]["eight_objective"]["fraction"],
               "pareto_key3_size": s4["pareto"]["key_three_objective"]["front_size"],
               "poles": {k: v["value_raw"] for k, v in s4["poles"].items()}},
        "s5": {"V0_score": s5["V0"]["score"], "V0_pO": s5["V0"]["p_O"],
               "V1_score": s5["V1"]["score"], "V1_pO": s5["V1"]["p_O"],
               "theta_sensitivity": s5["theta_sensitivity"],
               "segment_rate": s5["segment_metric"]["rate"],
               "cooperative_zone_voxels": s5z["cooperative_zone"]["voxels"],
               "cooperative_zone_litres": s5z["cooperative_zone"]["volume_l"],
               "cooperative_zone_components": s5z["cooperative_zone"]["components_total"]},
        "s6": {"scale_variant_V1_pO": s5v["V1"]["p_O"],
               "scale_variant_shift_m": s6["scale_variant"]["V1_optimum_shift_m"],
               "effective_objectives": [o for o in OBJ
                                        if s6["ablation"][o]["moved_m"] > 0],
               "saturated_fixed": {o: s6["saturation"][o]["fixed"] for o in OBJ}},
        "recommended": {
            "pose": s5v["V1"]["p_O"],
            "scale_set": "P95",
            "reason": "the fixed scales clipped J8 for 94.8% of candidates; the P95 "
                      "variant keeps joint margin and clearance positive while also "
                      "improving sigma_min",
        },
        "limits": [
            "S4 is static feasibility, not reachability",
            "the segment metric is NOT a motion planner",
            "the effective objective set is {J3, J5, J6}",
            "S3 is a structural identity under the rigid-grasp model",
            "the cooperative zone is ONE DOMINANT component, not literally one: at "
            "2 cm/26-connectivity it is 1 component, but at 1 cm it is 524 (largest "
            "holding 94% of the voxels)",
            "Q1-Q4 of m4_handover.yaml are unconfirmed; the rod is a parametric box",
            "Baseline B is provisional_rebuilt",
        ],
        "figures": ["fig5_pareto.png", "fig6_cooperative_zone.png",
                    "fig7_sensitivity.png", "fig8_overview.png"],
    }
    jp = os.path.join(args.out_dir, "c42_summary.json")
    json.dump(summary, open(jp, "w"), indent=2, ensure_ascii=False)
    print("[s7] wrote", fig_path)
    print("[s7] wrote", jp)
    print("[s7] effective objectives:", summary["s6"]["effective_objectives"])
    return 0


if __name__ == "__main__":
    sys.exit(main())
