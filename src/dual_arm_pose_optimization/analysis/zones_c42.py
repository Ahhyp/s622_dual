#!/usr/bin/env python3
"""C4.2 / S5: the cooperative zone (voxelised object-centre set where S4 holds)
plus the overlay figure against C4.1's common reachable workspace.

Writes
    figs/c42_s5_zones.json
    figs/fig6_cooperative_zone.png

Definitional note that the plan's acceptance wording got slightly wrong: CRW is a
set of TCP *points*, while the cooperative zone is a set of *object centres*.
A literal `zone subset of CRW` is therefore not well defined.  What IS well
defined, and what is asserted here, is that every cooperative-zone voxel comes
from a candidate whose TWO INDUCED TCPs are both point-reachable (that is exactly
the S0 layer).  The overlay is still drawn because it is the informative picture.
"""

from __future__ import annotations

import argparse
import csv
import json
import os
import sys
from collections import Counter, deque

import numpy as np

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

C41_FIELDS = 37
C41_P_LEFT, C41_P_RIGHT = 12, 15
# objective scales from the S4 run (declared there; reused here for colouring)
SCALES = [0.153739, 0.143295, 1.151829, 0.05, 0.5, 1.053581, 1.0, 0.05]
WEIGHTS = [1.0] * 8


def load_s4(csv_path):
    rows = []
    with open(csv_path, newline="") as fh:
        for r in csv.DictReader(fh):
            if r["s4"] == "1":
                rows.append(r)
    return rows


def score(r):
    def c01(v):
        return min(1.0, max(0.0, v))
    j = [
        c01(float(r["smin_left"]) / SCALES[0]),
        c01(float(r["smin_right"]) / SCALES[1]),
        c01(float(r["w_rel"]) / SCALES[2]),
        c01(float(r["mesh_arm_arm"]) / SCALES[3]),
        c01(float(r["joint_margin"]) / SCALES[4]),
        c01(1.0 - float(r["deviation"]) / SCALES[5]),
        c01(float(r["continuity"]) / SCALES[6]),
        c01(float(r["arm_proximal"]) / SCALES[7]),
    ]
    return float(np.dot(WEIGHTS, j) / np.sum(WEIGHTS))


def c41_masks(path, voxel=0.02):
    """A_L / A_R occupancy from C4.1's records (chunked memmap)."""
    data = np.memmap(path, dtype=np.float32, mode="r")
    n = data.size // C41_FIELDS
    data = data.reshape(n, C41_FIELDS)
    sub = np.asarray(data[::max(1, n // 400_000)])
    pl = sub[:, C41_P_LEFT:C41_P_LEFT + 3]
    pr = sub[:, C41_P_RIGHT:C41_P_RIGHT + 3]
    pts = np.concatenate([pl, pr], axis=0)
    lo, hi = pts.min(axis=0), pts.max(axis=0)
    edges = [np.arange(lo[k], hi[k] + voxel, voxel) for k in range(3)]
    shape = tuple(len(e) - 1 for e in edges)

    def fill(idx):
        m = np.zeros(shape, dtype=bool)
        for s in range(0, n, 500_000):
            blk = np.asarray(data[s:s + 500_000])
            if blk.size == 0:
                break
            p = blk[:, idx:idx + 3]
            ii = np.floor((p - lo) / voxel).astype(np.int64)
            ok = np.all((ii >= 0) & (ii < np.array(shape)), axis=1)
            ii = ii[ok]
            m[ii[:, 0], ii[:, 1], ii[:, 2]] = True
        return m

    left = fill(C41_P_LEFT)
    right = fill(C41_P_RIGHT)
    return dict(left=left, right=right, crw=left & right, edges=edges, lo=lo, hi=hi,
                voxel=voxel)


def components(mask, structure=26):
    """Label 26-connected components of a boolean voxel mask."""
    if structure == 26:
        offsets = [(dx, dy, dz) for dx in (-1, 0, 1) for dy in (-1, 0, 1)
                   for dz in (-1, 0, 1) if (dx, dy, dz) != (0, 0, 0)]
    else:
        offsets = [(1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1)]
    lab = np.zeros(mask.shape, dtype=np.int32)
    cur = 0
    idxs = np.argwhere(mask)
    for start in idxs:
        if lab[tuple(start)] != 0:
            continue
        cur += 1
        q = deque([tuple(start)])
        lab[tuple(start)] = cur
        while q:
            x, y, z = q.popleft()
            for dx, dy, dz in offsets:
                a, b, c = x + dx, y + dy, z + dz
                if 0 <= a < mask.shape[0] and 0 <= b < mask.shape[1] and 0 <= c < mask.shape[2]:
                    if mask[a, b, c] and lab[a, b, c] == 0:
                        lab[a, b, c] = cur
                        q.append((a, b, c))
    return lab, cur


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--csv", required=True)
    ap.add_argument("--c41-bin", required=True)
    ap.add_argument("--s5-json", required=True)
    ap.add_argument("--out-dir", required=True)
    ap.add_argument("--voxel", type=float, default=0.02)
    ap.add_argument("--min-component-voxels", type=int, default=20)
    args = ap.parse_args()
    os.makedirs(args.out_dir, exist_ok=True)

    rows = load_s4(args.csv)
    print(f"[s5] {len(rows):,} S4 candidates")
    P = np.array([[float(r["px"]), float(r["py"]), float(r["pz"])] for r in rows])
    sc = np.array([score(r) for r in rows])

    # ------------------------------------------------ cooperative zone voxels
    V = args.voxel
    lo = P.min(axis=0) - V
    hi = P.max(axis=0) + V
    edges = [np.arange(lo[k], hi[k] + V, V) for k in range(3)]
    shape = tuple(len(e) - 1 for e in edges)
    best = np.full(shape, -np.inf)
    cnt = np.zeros(shape, dtype=np.int64)
    ii = np.floor((P - lo) / V).astype(np.int64)
    for k in range(len(rows)):
        a, b, c = ii[k]
        cnt[a, b, c] += 1
        if sc[k] > best[a, b, c]:
            best[a, b, c] = sc[k]
    zone = cnt > 0
    zone_vol = float(zone.sum()) * V ** 3 * 1000.0  # litres
    print(f"[s5] cooperative zone: {int(zone.sum())} voxels "
          f"({zone_vol:.1f} L at {V*100:.0f} cm)")

    lab, ncomp = components(zone)
    sizes = Counter(lab[zone].tolist())
    big = [(k, v) for k, v in sizes.items() if v >= args.min_component_voxels]
    big.sort(key=lambda kv: -kv[1])
    print(f"[s5] connected components: {ncomp} total, {len(big)} with "
          f">= {args.min_component_voxels} voxels")

    # best subregion: among the big components, the one with the highest mean score
    sub = []
    for k, v in big:
        m = lab == k
        vals = best[m]
        vals = vals[np.isfinite(vals)]
        sub.append({"component": int(k), "voxels": int(v),
                    "volume_l": round(v * V ** 3 * 1000.0, 3),
                    "mean_score": round(float(vals.mean()), 4),
                    "max_score": round(float(vals.max()), 4)})
    sub.sort(key=lambda d: -d["mean_score"])
    best_sub = sub[0] if sub else None
    print("[s5] best subregion:", best_sub)

    # ------------------------------------------------ CRW overlay + containment
    m = c41_masks(args.c41_bin, args.voxel)
    print(f"[s5] C4.1 masks: left={int(m['left'].sum())} right={int(m['right'].sum())} "
          f"CRW={int(m['crw'].sum())} voxels")

    # containment: every zone voxel's candidate is an S0 candidate by construction
    s0_all = all(r["s0"] == "1" for r in rows)
    print(f"[s5] S0 containment of all S4 candidates: {s0_all}")

    # in-CRW test on the OBJECT CENTRE (informative, not the asserted claim)
    e = m["edges"]
    in_crw = 0
    for k in range(len(P)):
        q = P[k]
        a = np.searchsorted(e[0], q[0]) - 1
        b = np.searchsorted(e[1], q[1]) - 1
        c = np.searchsorted(e[2], q[2]) - 1
        if 0 <= a < m["crw"].shape[0] and 0 <= b < m["crw"].shape[1] and 0 <= c < m["crw"].shape[2]:
            if m["crw"][a, b, c]:
                in_crw += 1
    print(f"[s5] object centres that fall inside the TCP-space CRW: "
          f"{in_crw}/{len(P)} ({100.0*in_crw/len(P):.1f}%)")

    # ------------------------------------------------------------- figure
    s5 = json.load(open(args.s5_json))
    b_B = s5["baseline_B"]["p_O"]
    b_V0 = s5["V0"]["p_O"]
    b_V1 = s5["V1"]["p_O"]

    fig, axes = plt.subplots(2, 2, figsize=(17, 13))

    def slice_plot(ax, key, ylabel):
        # CRW occupancy in the corresponding plane
        if key == "xy":
            occ = m["crw"].any(axis=2)
            ax.contour(e[0][:-1], e[1][:-1], occ.T.astype(float), levels=[0.5],
                       colors="k", linewidths=1.2)
            zg = zone.any(axis=2)
            g = np.where(zg, best.max(axis=2), np.nan)
            pcm = ax.pcolormesh(edges[0], edges[1], np.ma.masked_invalid(g).T,
                                cmap="viridis", shading="auto")
            for p, c, lab_ in ((b_B, "red", "Baseline B"), (b_V0, "lime", "V0"),
                               (b_V1, "cyan", "V1")):
                ax.plot(p[0], p[1], marker="*", ms=17, color=c, markeredgecolor="k",
                        ls="none", label=lab_)
        else:
            occ = m["crw"].any(axis=1)
            ax.contour(e[0][:-1], e[2][:-1], occ.T.astype(float), levels=[0.5],
                       colors="k", linewidths=1.2)
            zg = zone.any(axis=1)
            g = np.where(zg, best.max(axis=1), np.nan)
            pcm = ax.pcolormesh(edges[0], edges[2], np.ma.masked_invalid(g).T,
                                cmap="viridis", shading="auto")
            for p, c, lab_ in ((b_B, "red", "Baseline B"), (b_V0, "lime", "V0"),
                               (b_V1, "cyan", "V1")):
                ax.plot(p[0], p[2], marker="*", ms=17, color=c, markeredgecolor="k",
                        ls="none", label=lab_)
        ax.set_aspect("equal")
        ax.set_xlabel("world x [m]")
        ax.set_ylabel(ylabel)
        ax.legend(fontsize=8, loc="upper left")
        return pcm

    pcm = slice_plot(axes[0][0], "xy", "world y [m]")
    axes[0][0].set_title("cooperative zone (object centre) vs C4.1 CRW (black outline)\n"
                         "x-y projection")
    fig.colorbar(pcm, ax=axes[0][0], label="best equal-weight score in voxel")
    pcm = slice_plot(axes[0][1], "xz", "world z [m]")
    axes[0][1].set_title("x-z projection")
    fig.colorbar(pcm, ax=axes[0][1], label="best equal-weight score in voxel")

    ax = axes[1][0]
    ks = sorted(sizes.items(), key=lambda kv: -kv[1])[:12]
    ax.bar([str(k) for k, _ in ks], [v for _, v in ks], color="steelblue")
    ax.axhline(args.min_component_voxels, color="crimson", ls="--",
               label=f"min size filter ({args.min_component_voxels})")
    ax.set_xlabel("component id")
    ax.set_ylabel("voxels")
    ax.set_title(f"connected components (26-conn): {ncomp} total, "
                 f"{len(big)} pass the size filter")
    ax.legend(fontsize=8)
    ax.grid(alpha=0.25, axis="y")

    ax = axes[1][1]
    d = np.linalg.norm(P - np.array(b_B), axis=1)
    ax.scatter(d, sc, c=sc, cmap="viridis", s=5, alpha=0.5)
    ax.axvline(np.linalg.norm(np.array(b_V1) - np.array(b_B)), color="cyan", ls="--",
               label=f"V1 ({np.linalg.norm(np.array(b_V1)-np.array(b_B)):.3f} m)")
    ax.axvline(np.linalg.norm(np.array(b_V0) - np.array(b_B)), color="lime", ls="--",
               label=f"V0 ({np.linalg.norm(np.array(b_V0)-np.array(b_B)):.3f} m)")
    ax.set_xlabel("distance from Baseline B object centre [m]")
    ax.set_ylabel("equal-weight score")
    ax.set_title("score vs proximity to the M4 baseline")
    ax.legend(fontsize=8)
    ax.grid(alpha=0.25)

    fig.suptitle("C4.2 / S5 — cooperative zone, V0/V1 optima and the M4 baseline")
    fig.tight_layout()
    fig_path = os.path.join(args.out_dir, "fig6_cooperative_zone.png")
    fig.savefig(fig_path, dpi=130)
    plt.close(fig)

    out = {
        "n_s4_candidates": len(rows),
        "voxel_m": V,
        "cooperative_zone": {
            "voxels": int(zone.sum()),
            "volume_l": round(zone_vol, 3),
            "bbox": [[round(float(lo[k]), 4), round(float(hi[k]), 4)] for k in range(3)],
            "components_total": int(ncomp),
            "components_passing_size_filter": len(big),
            "min_component_voxels": args.min_component_voxels,
            "components": sub[:10],
            "best_subregion": best_sub,
        },
        "containment": {
            "asserted": "every S4 candidate is an S0 candidate (both induced TCPs "
                        "point-reachable) -> cooperative zone is inside the "
                        "point-reachable set",
            "s0_holds_for_all_s4": bool(s0_all),
            "object_centres_inside_tcp_space_crw": in_crw,
            "object_centres_total": len(P),
            "definitional_note": "CRW is a set of TCP POINTS while the cooperative "
                                 "zone is a set of OBJECT CENTRES; a literal "
                                 "subset relation between them is not well defined, "
                                 "so the asserted containment is the S0 layer.",
        },
        "c41_crw_voxels": int(m["crw"].sum()),
        "s5": s5,
        "figures": [os.path.basename(fig_path)],
    }
    jp = os.path.join(args.out_dir, "c42_s5_zones.json")
    with open(jp, "w") as fh:
        json.dump(out, fh, indent=2, ensure_ascii=False)
    print("[s5] wrote", jp)
    print("[s5] wrote", fig_path)
    return 0


if __name__ == "__main__":
    sys.exit(main())
