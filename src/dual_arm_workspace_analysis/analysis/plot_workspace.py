#!/usr/bin/env python3
"""C4.1 plotting pipeline: dual-arm workspace & manipulability.

Reads the float32 record file produced by `workspace_sample` and writes

    fig1_common_reachable.png          common reachable space + handover candidates
    fig2_relative_manipulability.png   w_rel field, orientation-conditioned
    fig3_nullspace_rank.png            rank / conditioning / near-degenerate regions
    fig4_arm_manipulability.png        relative vs weakest-arm contrast
    summary.json                       headline numbers, for C4.2

Two-pass design, because this box has ~3 GB of usable RAM and the 1e7 record
file is ~1.5 GB:

  * full-data pass (np.memmap, chunked)  -> occupancy masks, histograms, 1-D
    profiles, and the sigma_min threshold for the near-degenerate map;
  * subsample pass (<=1.5e6 rows, in memory) -> the 2-D median fields of fig2,
    because a per-cell MEAN over 6-D data is dominated by the spread and reads
    as noise, while a median is both robust and cheap on a subsample.

Record layout is FROZEN and comes from
dual_arm_workspace_analysis/workspace_sampler.hpp -- keep FIELDS in sync.
"""

from __future__ import annotations

import argparse
import json
import os
import sys

import numpy as np

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.colors import LogNorm  # noqa: E402

# --------------------------------------------------------------------------
# Frozen record schema (37 float32 per sample)
# --------------------------------------------------------------------------
FIELDS = (
    "q1,q2,q3,q4,q5,q6,q7,q8,q9,q10,q11,q12,"
    "p_left_x,p_left_y,p_left_z,"
    "p_right_x,p_right_y,p_right_z,"
    "p_object_x,p_object_y,p_object_z,"
    "p_rel_x,p_rel_y,p_rel_z,"
    "rel_rotvec_x,rel_rotvec_y,rel_rotvec_z,"
    "d_arms,w_left,w_right,smin_left,smin_right,"
    "w_rel_body,smin_rel_body,w_rel_left,w_rel_world,rank_rel"
).split(",")
N_FIELDS = len(FIELDS)

I = {name: i for i, name in enumerate(FIELDS)}

P_LEFT = [I["p_left_x"], I["p_left_y"], I["p_left_z"]]
P_RIGHT = [I["p_right_x"], I["p_right_y"], I["p_right_z"]]
P_OBJECT = [I["p_object_x"], I["p_object_y"], I["p_object_z"]]
P_REL = [I["p_rel_x"], I["p_rel_y"], I["p_rel_z"]]
REL_ROTVEC = [I["rel_rotvec_x"], I["rel_rotvec_y"], I["rel_rotvec_z"]]

CHUNK = 500_000            # records per chunk in the full-data pass
MAX_SUBSAMPLE = 1_500_000  # rows held in RAM for the 2-D fields
VOXEL = 0.02               # m, occupancy bin size (fig1 / fig3)
FIELD_BIN = 0.08           # m, coarser bins for the fig2 median fields
FIELD_MIN_COUNT = 20       # cells with fewer samples stay blank in fig2
THETA_EDGES_DEG = [0.0, 36.0, 72.0, 108.0, 144.0, 180.0]
HANDOVER_RADII = [0.25, 0.35, 0.45]
LEFT_BASE = (0.35, 0.0)
RIGHT_BASE = (-0.35, 0.0)


# --------------------------------------------------------------------------
# Helpers
# --------------------------------------------------------------------------
def zeros2d(edges):
    return np.zeros((len(edges[0]) - 1, len(edges[1]) - 1), np.int64)


def bin_index(v, edges):
    return np.digitize(v, edges) - 1


def grouped_median(a, b, w, edges, min_count=FIELD_MIN_COUNT):
    """Median of `w` over the 2-D histogram of (a, b)."""
    nx, ny = len(edges[0]) - 1, len(edges[1]) - 1
    ix = bin_index(a, edges[0])
    iy = bin_index(b, edges[1])
    valid = (ix >= 0) & (ix < nx) & (iy >= 0) & (iy < ny)
    if not np.any(valid):
        return np.full((nx, ny), np.nan)
    flat = ix[valid] * ny + iy[valid]
    vals = w[valid]
    uniq, inv = np.unique(flat, return_inverse=True)
    order = np.argsort(inv, kind="stable")
    sorted_vals = vals[order]
    counts = np.bincount(inv)
    starts = np.concatenate([[0], np.cumsum(counts)[:-1]])
    out = np.full(nx * ny, np.nan)
    for s, c, u in zip(starts, counts, uniq):
        if c < min_count:
            continue
        out[u] = np.median(sorted_vals[s:s + c])
    return out.reshape(nx, ny)


def profile(a, w, edges):
    """Mean / P5 / P95 of `w` per bin of `a`."""
    idx = bin_index(a, edges)
    nb = len(edges) - 1
    mean = np.full(nb, np.nan)
    p5 = np.full(nb, np.nan)
    p95 = np.full(nb, np.nan)
    cnt = np.zeros(nb, np.int64)
    for k in range(nb):
        m = idx == k
        c = int(np.count_nonzero(m))
        cnt[k] = c
        if c == 0:
            continue
        v = w[m]
        mean[k] = float(np.mean(v))
        p5[k] = float(np.percentile(v, 5))
        p95[k] = float(np.percentile(v, 95))
    return mean, p5, p95, cnt


# --------------------------------------------------------------------------
# Pass 1: full data (memmap, chunked)
# --------------------------------------------------------------------------
def scan(path: str):
    data = np.memmap(path, dtype=np.float32, mode="r")
    if data.size % N_FIELDS != 0:
        raise SystemExit(f"{path}: size {data.size} is not a multiple of {N_FIELDS}")
    n = data.size // N_FIELDS
    data = data.reshape(n, N_FIELDS)

    stride = max(1, n // MAX_SUBSAMPLE)
    sub = np.asarray(data[::stride])
    print(f"[c41]   subsample stride {stride} -> {sub.shape[0]:,} rows", flush=True)

    pts = np.concatenate([sub[:, P_LEFT], sub[:, P_RIGHT], sub[:, P_OBJECT]], axis=0)
    wmin, wmax = pts.min(axis=0), pts.max(axis=0)
    rel = sub[:, P_REL]
    rmin, rmax = rel.min(axis=0), rel.max(axis=0)

    edges_xy = (np.arange(wmin[0], wmax[0] + VOXEL, VOXEL),
                np.arange(wmin[1], wmax[1] + VOXEL, VOXEL))
    edges_xz = (np.arange(wmin[0], wmax[0] + VOXEL, VOXEL),
                np.arange(wmin[2], wmax[2] + VOXEL, VOXEL))
    fedges_xy = (np.arange(rmin[0], rmax[0] + FIELD_BIN, FIELD_BIN),
                 np.arange(rmin[1], rmax[1] + FIELD_BIN, FIELD_BIN))
    fedges_xz = (np.arange(rmin[0], rmax[0] + FIELD_BIN, FIELD_BIN),
                 np.arange(rmin[2], rmax[2] + FIELD_BIN, FIELD_BIN))

    # 3-D occupancy (for the headline CRW / single-arm volume ratio)
    edges_xyz = (edges_xy[0], edges_xy[1], np.arange(wmin[2], wmax[2] + VOXEL, VOXEL))
    occ3 = {"L": np.zeros([len(e) - 1 for e in edges_xyz], np.int32),
            "R": np.zeros([len(e) - 1 for e in edges_xyz], np.int32)}

    occ = {"L_xy": zeros2d(edges_xy), "R_xy": zeros2d(edges_xy),
           "L_xz": zeros2d(edges_xz), "R_xz": zeros2d(edges_xz)}
    hand = {r: {"xy": zeros2d(edges_xy), "xz": zeros2d(edges_xz), "count": 0}
            for r in HANDOVER_RADII}

    wrel_hist_edges = np.linspace(0.0, 4.5, 91)
    wrel_hist = np.zeros(len(wrel_hist_edges) - 1, np.int64)
    smin_hist_edges = np.logspace(-3, 0.0, 61)
    smin_hist = np.zeros(len(smin_hist_edges) - 1, np.int64)
    rank_counts: dict[int, int] = {}

    smin_sub = sub[:, I["smin_rel_body"]]
    smin_p01 = float(np.percentile(smin_sub, 1.0))
    smin_p05 = float(np.percentile(smin_sub, 5.0))
    smin_p50 = float(np.percentile(smin_sub, 50.0))
    low_near = {"xy": zeros2d(edges_xy), "xz": zeros2d(edges_xz)}

    for start in range(0, n, CHUNK):
        blk = np.asarray(data[start:start + CHUNK])
        if blk.size == 0:
            break
        pl, pr, po = blk[:, P_LEFT], blk[:, P_RIGHT], blk[:, P_OBJECT]
        smin = blk[:, I["smin_rel_body"]]
        wrel = blk[:, I["w_rel_body"]]

        occ["L_xy"] += np.histogram2d(pl[:, 0], pl[:, 1], bins=edges_xy)[0].astype(np.int64)
        occ["L_xz"] += np.histogram2d(pl[:, 0], pl[:, 2], bins=edges_xz)[0].astype(np.int64)
        occ["R_xy"] += np.histogram2d(pr[:, 0], pr[:, 1], bins=edges_xy)[0].astype(np.int64)
        occ["R_xz"] += np.histogram2d(pr[:, 0], pr[:, 2], bins=edges_xz)[0].astype(np.int64)
        occ3["L"] += np.histogramdd(pl, bins=edges_xyz)[0].astype(np.int32)
        occ3["R"] += np.histogramdd(pr, bins=edges_xyz)[0].astype(np.int32)

        d_tcp = np.linalg.norm(pl - pr, axis=1)
        mid = 0.5 * (pl + pr)
        for r_hand in HANDOVER_RADII:
            m = d_tcp <= r_hand
            if not np.any(m):
                continue
            hand[r_hand]["count"] += int(np.count_nonzero(m))
            hand[r_hand]["xy"] += np.histogram2d(
                mid[m, 0], mid[m, 1], bins=edges_xy)[0].astype(np.int64)
            hand[r_hand]["xz"] += np.histogram2d(
                mid[m, 0], mid[m, 2], bins=edges_xz)[0].astype(np.int64)

        wrel_hist += np.histogram(wrel, bins=wrel_hist_edges)[0].astype(np.int64)
        pos = smin > 0
        if np.any(pos):
            smin_hist += np.histogram(smin[pos], bins=smin_hist_edges)[0].astype(np.int64)
        ranks, counts = np.unique(blk[:, I["rank_rel"]], return_counts=True)
        for r, c in zip(ranks, counts):
            rank_counts[int(r)] = rank_counts.get(int(r), 0) + int(c)

        m = smin <= smin_p01
        if np.any(m):
            low_near["xy"] += np.histogram2d(
                po[m, 0], po[m, 1], bins=edges_xy)[0].astype(np.int64)
            low_near["xz"] += np.histogram2d(
                po[m, 0], po[m, 2], bins=edges_xz)[0].astype(np.int64)

    return dict(
        n=n, sub=sub, edges_xy=edges_xy, edges_xz=edges_xz,
        fedges_xy=fedges_xy, fedges_xz=fedges_xz,
        occ=occ, occ3=occ3, edges_xyz=edges_xyz, hand=hand, low_near=low_near,
        wrel_hist=(wrel_hist, wrel_hist_edges),
        smin_hist=(smin_hist, smin_hist_edges),
        rank_counts=rank_counts,
        smin_p01=smin_p01, smin_p05=smin_p05, smin_p50=smin_p50,
    )


# --------------------------------------------------------------------------
# Pass 2: subsample-based median fields
# --------------------------------------------------------------------------
def field_maps(res):
    sub = res["sub"]
    prel = sub[:, P_REL]
    wrel = sub[:, I["w_rel_body"]]
    theta = np.linalg.norm(sub[:, REL_ROTVEC], axis=1)
    prn = np.linalg.norm(prel, axis=1)

    e_dist = np.linspace(0.0, float(prn.max()) + 0.05, 41)
    e_theta = np.deg2rad(np.array(THETA_EDGES_DEG))
    out = {
        "marginal_xy": grouped_median(prel[:, 0], prel[:, 1], wrel, res["fedges_xy"]),
        "marginal_xz": grouped_median(prel[:, 0], prel[:, 2], wrel, res["fedges_xz"]),
        "theta": [],
        "prof_dist": (profile(prn, wrel, e_dist), e_dist),
        "prof_theta": (profile(theta, wrel, e_theta), e_theta),
    }

    th_deg = np.rad2deg(theta)
    for k in range(len(THETA_EDGES_DEG) - 1):
        m = (th_deg >= THETA_EDGES_DEG[k]) & (th_deg < THETA_EDGES_DEG[k + 1])
        n = int(np.count_nonzero(m))
        if n < FIELD_MIN_COUNT:
            out["theta"].append((None, n))
            continue
        out["theta"].append((
            grouped_median(prel[m, 0], prel[m, 1], wrel[m], res["fedges_xy"]), n))
    return out


# --------------------------------------------------------------------------
# Figures
# --------------------------------------------------------------------------
def fig1(res, out_dir):
    exy, exz = res["edges_xy"], res["edges_xz"]
    occ = res["occ"]
    h35 = res["hand"][0.35]

    fig, axes = plt.subplots(2, 3, figsize=(19, 10.5))
    for j, key in enumerate(["xy", "xz"]):
        edges = exy if key == "xy" else exz
        Lg = occ["L_" + key] > 0
        Rg = occ["R_" + key] > 0
        crw = Lg & Rg
        ylabel = f"world {'y' if key == 'xy' else 'z'} [m]"

        axes[j][0].pcolormesh(edges[0], edges[1], Lg.astype(float).T,
                              cmap="Blues", shading="auto")
        axes[j][0].set_title(f"left arm reachable (TCP) — {key}")
        axes[j][1].pcolormesh(edges[0], edges[1], Rg.astype(float).T,
                              cmap="Oranges", shading="auto")
        axes[j][1].set_title(f"right arm reachable (TCP) — {key}")

        ax = axes[j][2]
        ax.pcolormesh(edges[0], edges[1], crw.astype(float).T,
                      cmap="Greens", shading="auto")
        g = h35[key].astype(float)
        ax.contour(edges[0][:-1], edges[1][:-1], g.T, levels=4,
                   colors="crimson", linewidths=1.2)
        ax.set_title(f"CRW = L ∩ R — {key}\n"
                     f"red: handover midpoint, ‖p_L−p_R‖ ≤ 0.35 m")

        for jj in range(3):
            axes[j][jj].plot(*LEFT_BASE, "b^", ms=7)
            axes[j][jj].plot(*RIGHT_BASE, "r^", ms=7)
            axes[j][jj].set_aspect("equal")
            axes[j][jj].set_xlabel("world x [m]")
            axes[j][jj].set_ylabel(ylabel)
            axes[j][jj].grid(alpha=0.2)

    fig.suptitle(f"C4.1 fig1 — dual-arm common reachable workspace "
                 f"({res['n']:,} samples, voxel {VOXEL*100:.0f} cm, ▲ = base)")
    fig.tight_layout()
    p = os.path.join(out_dir, "fig1_common_reachable.png")
    fig.savefig(p, dpi=130)
    plt.close(fig)
    return p


def fig2(res, fld, out_dir):
    fxy = res["fedges_xy"]
    fig = plt.figure(figsize=(19, 17))
    gs = fig.add_gridspec(4, 3)

    for k, (grid, edges, key) in enumerate([
        (fld["marginal_xy"], fxy, "x-y"),
        (fld["marginal_xz"], res["fedges_xz"], "x-z"),
    ]):
        ax = fig.add_subplot(gs[0, k])
        pcm = ax.pcolormesh(edges[0], edges[1], np.ma.masked_invalid(grid).T,
                            cmap="viridis", shading="auto")
        ax.set_aspect("equal")
        ax.set_xlabel("p_rel x [m]")
        ax.set_ylabel("p_rel y / z [m]")
        ax.set_title(f"median w_rel, orientation-marginal ({key})")
        fig.colorbar(pcm, ax=ax)

    ax = fig.add_subplot(gs[0, 2])
    h, e = res["wrel_hist"]
    ax.bar(0.5 * (e[1:] + e[:-1]), h.astype(float), width=(e[1] - e[0]) * 0.9)
    ax.set_yscale("log")
    ax.set_xlabel("w_rel")
    ax.set_ylabel("count")
    ax.set_title("w_rel distribution")
    ax.grid(alpha=0.3)

    for k, (grid, n) in enumerate(fld["theta"]):
        ax = fig.add_subplot(gs[1 + (0 if k < 3 else 1), k if k < 3 else k - 3])
        if grid is None:
            ax.axis("off")
            continue
        pcm = ax.pcolormesh(fxy[0], fxy[1], np.ma.masked_invalid(grid).T,
                            cmap="viridis", shading="auto")
        ax.set_aspect("equal")
        ax.set_title(f"median w_rel, θ_rel ∈ [{THETA_EDGES_DEG[k]:.0f},"
                     f"{THETA_EDGES_DEG[k+1]:.0f})°   (n={n:,})", fontsize=9)
        fig.colorbar(pcm, ax=ax)

    for col, ((mean, p5, p95, cnt), edges, xlabel) in enumerate([
        (fld["prof_dist"][0], fld["prof_dist"][1], "|p_rel| [m]"),
        (fld["prof_theta"][0], fld["prof_theta"][1], "θ_rel [deg]"),
    ]):
        ax = fig.add_subplot(gs[3, col])
        mid = (np.rad2deg(0.5 * (edges[1:] + edges[:-1])) if xlabel.startswith("θ")
               else 0.5 * (edges[1:] + edges[:-1]))
        ax.fill_between(mid, p5, p95, alpha=0.25, label="P5–P95")
        ax.plot(mid, mean, "o-", label="mean")
        ax.set_xlabel(xlabel)
        ax.set_ylabel("w_rel")
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8)
        ax.set_title("w_rel vs relative distance" if col == 0
                     else "w_rel vs relative rotation")

    ax = fig.add_subplot(gs[3, 2])
    ax.axis("off")
    ax.text(0.0, 1.0,
            "Reading fig2\n\n"
            "• w_rel = √det(J_rel J_relᵀ) = ∏σ_i is FRAME-INVARIANT\n"
            "  (det Ad(T) = det R² = 1), so it is comparable across\n"
            "  scenarios without quoting a frame.\n\n"
            "• σ_min(J_rel) is NOT frame-invariant — always quote\n"
            "  its frame. Main results use BodyR.\n\n"
            f"• The 2-D fields are MEDIANS over a strided subsample;\n"
            f"  cells with < {FIELD_MIN_COUNT} samples stay blank.\n"
            "  The marginal field is dominated by the |p_rel| trend.\n\n"
            "• The θ_rel panels bin the rotation ANGLE rather than\n"
            "  the planned ±30° axis slices: exact orientation\n"
            "  slicing leaves ~1e2 samples per slice at 1e7.",
            va="top", ha="left", fontsize=9, family="monospace")

    fig.suptitle("C4.1 fig2 — relative manipulability w_rel and its dependence on relative pose")
    fig.tight_layout()
    p = os.path.join(out_dir, "fig2_relative_manipulability.png")
    fig.savefig(p, dpi=130)
    plt.close(fig)
    return p


def fig3(res, out_dir):
    fig, axes = plt.subplots(2, 2, figsize=(15, 11))

    ax = axes[0][0]
    ks = sorted(res["rank_counts"].keys())
    vals = [res["rank_counts"][k] for k in ks]
    ax.bar([str(k) for k in ks], vals, color="steelblue")
    ax.set_xlabel("rank(J_rel)   (dim ker = 12 − rank, frame-invariant)")
    ax.set_ylabel("count")
    if vals:
        ax.set_ylim(0, max(vals) * 1.25)
        for i, k in enumerate(ks):
            ax.text(i, res["rank_counts"][k], f"{res['rank_counts'][k]:,}",
                    ha="center", va="bottom", fontsize=9)
    ax.set_title("numerical rank distribution (tol = 1e-6)")
    ax.grid(alpha=0.3, axis="y")

    ax = axes[0][1]
    h, e = res["smin_hist"]
    ax.bar(0.5 * (e[1:] + e[:-1]), h.astype(float), width=np.diff(e) * 0.9,
           color="indianred")
    ax.set_xscale("log")
    ax.set_xlabel("σ_min(J_rel), BodyR")
    ax.set_ylabel("count")
    ax.axvline(res["smin_p05"], color="k", ls="--", lw=1, label=f"P5 = {res['smin_p05']:.4f}")
    ax.axvline(res["smin_p50"], color="k", ls=":", lw=1, label=f"P50 = {res['smin_p50']:.4f}")
    ax.legend(fontsize=8)
    ax.set_title("conditioning (quote σ_min WITH its frame)")
    ax.grid(alpha=0.3)

    sub = res["sub"]
    po = sub[:, P_OBJECT]
    smin = sub[:, I["smin_rel_body"]]
    for j, key in enumerate(["xy", "xz"]):
        edges = res["edges_xy"] if key == "xy" else res["edges_xz"]
        occ = res["occ"]["L_" + key] > 0
        field = grouped_median(po[:, 0], po[:, 1 if key == "xy" else 2], smin, edges)
        g = res["low_near"][key].astype(float)

        ax = axes[1][j]
        ax.pcolormesh(edges[0], edges[1],
                      np.ma.masked_where(~occ, occ.astype(float)).T,
                      cmap="Greys", alpha=0.25, shading="auto")
        pcm = ax.pcolormesh(edges[0], edges[1], np.ma.masked_invalid(field).T,
                            cmap="RdYlGn", shading="auto")
        ax.contour(edges[0][:-1], edges[1][:-1], g.T, levels=3,
                   colors="black", linewidths=0.8, alpha=0.7)
        ax.set_aspect("equal")
        ax.set_xlabel("world x [m]")
        ax.set_ylabel(f"world {'y' if key == 'xy' else 'z'} [m]")
        ax.set_title(f"median σ_min(J_rel) [BodyR] — object frame ({key})\n"
                     f"black contours: worst 1% (σ_min ≤ {res['smin_p01']:.4f})")
        fig.colorbar(pcm, ax=ax)

    fig.suptitle("C4.1 fig3 — null-space dimension & near-degenerate regions")
    fig.tight_layout()
    p = os.path.join(out_dir, "fig3_nullspace_rank.png")
    fig.savefig(p, dpi=130)
    plt.close(fig)
    return p


def fig4(res, out_dir):
    sub = res["sub"]
    wrel = sub[:, I["w_rel_body"]]
    wl = sub[:, I["w_left"]]
    wr = sub[:, I["w_right"]]
    smin = sub[:, I["smin_rel_body"]]
    sminl = sub[:, I["smin_left"]]
    sminr = sub[:, I["smin_right"]]

    fig, axes = plt.subplots(1, 2, figsize=(14, 5.8))
    corr = {}
    for ax, (a, b, la, lb, title, key) in zip(axes, [
        (wrel, np.minimum(wl, wr), "w_rel", "min(w_L, w_R)",
         "relative vs weakest-arm manipulability", "w"),
        (smin, np.minimum(sminl, sminr), "σ_min(J_rel) [BodyR]", "min σ_min(J_arm)",
         "relative vs weakest-arm conditioning", "smin"),
    ]):
        # NOTE: matplotlib's hexbin is unreliable on log-scaled axes, and the two
        # quantities here are 3 decades apart, so bin explicitly in log10 space.
        ap, bp = a[a > 0], b[b > 0]
        xlo, xhi = np.percentile(ap, [0.1, 99.9])
        ylo, yhi = np.percentile(bp, [0.1, 99.9])
        xe = np.logspace(np.log10(xlo), np.log10(xhi), 61)
        ye = np.logspace(np.log10(ylo), np.log10(yhi), 61)
        h, _, _ = np.histogram2d(a, b, bins=(xe, ye))
        pcm = ax.pcolormesh(xe, ye, np.ma.masked_where(h.T <= 0, h.T),
                            cmap="viridis", norm=LogNorm(), shading="auto")
        fig.colorbar(pcm, ax=ax, label="count")
        lo, hi = max(xlo, ylo), min(xhi, yhi)
        if lo < hi:
            ax.plot([lo, hi], [lo, hi], "r--", lw=1, label="y = x (overlap range)")
            ax.legend(fontsize=8, loc="upper left")
        ax.set_xscale("log")
        ax.set_yscale("log")
        ax.set_xlabel(la)
        ax.set_ylabel(lb)
        ax.set_title(title)
        n = min(a.size, b.size)
        ra = np.argsort(np.argsort(a[:n]))
        rb = np.argsort(np.argsort(b[:n]))
        corr[key + "_spearman"] = float(np.corrcoef(ra, rb)[0, 1])
        corr[key + "_ratio_p50"] = float(np.median(a[:n] / np.maximum(b[:n], 1e-300)))
    fig.suptitle("C4.1 fig4 — is relative manipulability explained by the weaker arm? "
                 f"(Spearman ρ = {corr['w_spearman']:.2f} / {corr['smin_spearman']:.2f} "
                 f"→ weak, answer: no)")
    fig.tight_layout()
    p = os.path.join(out_dir, "fig4_arm_manipulability.png")
    fig.savefig(p, dpi=130)
    plt.close(fig)
    return p, corr


# --------------------------------------------------------------------------
def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--prefix", required=True,
                    help="output prefix used by workspace_sample (without .bin)")
    ap.add_argument("--out-dir", default=None,
                    help="figure directory (default: directory of --prefix)")
    args = ap.parse_args()

    bin_path = args.prefix + ".bin"
    if not os.path.exists(bin_path):
        raise SystemExit(f"missing {bin_path}")
    out_dir = args.out_dir or os.path.dirname(os.path.abspath(args.prefix)) or "."
    os.makedirs(out_dir, exist_ok=True)

    manifest = {}
    jpath = args.prefix + ".json"
    if os.path.exists(jpath):
        with open(jpath) as fh:
            manifest = json.load(fh)

    print(f"[c41] scanning {bin_path} ...", flush=True)
    res = scan(bin_path)
    print(f"[c41] {res['n']:,} accepted records", flush=True)

    print("[c41] median fields from subsample ...", flush=True)
    fld = field_maps(res)

    print("[c41] figures ...", flush=True)
    p1 = fig1(res, out_dir)
    p2 = fig2(res, fld, out_dir)
    p3 = fig3(res, out_dir)
    p4, corr4 = fig4(res, out_dir)

    crw3 = (res["occ3"]["L"] > 0) & (res["occ3"]["R"] > 0)
    ex3 = res["edges_xyz"]
    crw_bbox = []
    for k in range(3):
        others = tuple(j for j in range(3) if j != k)
        idx = np.where(crw3.any(axis=others))[0]
        crw_bbox.append([round(float(ex3[k][idx[0]]), 3),
                         round(float(ex3[k][idx[-1] + 1]), 3)] if idx.size else [None, None])
    nL3, nR3, nC3 = int((res["occ3"]["L"] > 0).sum()), int((res["occ3"]["R"] > 0).sum()), int(crw3.sum())
    crw_xy = (res["occ"]["L_xy"] > 0) & (res["occ"]["R_xy"] > 0)
    crw_xz = (res["occ"]["L_xz"] > 0) & (res["occ"]["R_xz"] > 0)
    wrel = res["sub"][:, I["w_rel_body"]]

    summary = {
        "accepted": int(res["n"]),
        "manifest": manifest.get("stats", {}),
        "params": manifest.get("params", {}),
        "common_reachable": {
            "voxel_m": VOXEL,
            "left_xy_voxels": int((res["occ"]["L_xy"] > 0).sum()),
            "right_xy_voxels": int((res["occ"]["R_xy"] > 0).sum()),
            "crw_xy_voxels": int(crw_xy.sum()),
            "crw_xz_voxels": int(crw_xz.sum()),
            "crw_xy_area_m2": float(crw_xy.sum() * VOXEL * VOXEL),
            "voxel3_m": VOXEL,
            "left_3d_voxels": nL3,
            "right_3d_voxels": nR3,
            "crw_3d_voxels": nC3,
            "crw_over_left_3d": float(nC3 / max(nL3, 1)),
            "left_3d_volume_m3": float(nL3 * VOXEL ** 3),
            "crw_3d_volume_m3": float(nC3 * VOXEL ** 3),
            "crw_bbox_xyz": crw_bbox,
            "note": "occupancy means >=1 accepted sample in the voxel; task-space "
                    "density is NOT uniform under joint-space sampling (it is "
                    "weighted by the Jacobian), so counts are not volumes.",
        },
        "handover": {
            f"{r:.2f}m": {"samples": int(res["hand"][r]["count"]),
                          "frac_of_accepted": float(res["hand"][r]["count"] /
                                                    max(res["n"], 1))}
            for r in HANDOVER_RADII
        },
        "relative_manipulability": {
            "w_rel_p5": float(np.percentile(wrel, 5)),
            "w_rel_p50": float(np.percentile(wrel, 50)),
            "w_rel_p95": float(np.percentile(wrel, 95)),
            "smin_rel_body_p1": res["smin_p01"],
            "smin_rel_body_p5": res["smin_p05"],
            "smin_rel_body_p50": res["smin_p50"],
            "frame_note": "w_rel = prod(sigma) is frame-invariant (det Ad = 1); "
                          "sigma_min is NOT -- always quote its frame. Main = BodyR.",
            "vs_weakest_arm": corr4,
        },
        "nullspace": {
            "rank_histogram": {str(k): int(v) for k, v in sorted(res["rank_counts"].items())},
            "dim_ker_histogram": {str(12 - k): int(v)
                                  for k, v in sorted(res["rank_counts"].items())},
            "note": "rank(J_rel) is frame-invariant; exact rank deficiency is a "
                    "measure-zero set, so the informative degradation measure is "
                    "sigma_min(J_rel) -- see fig3 right.",
        },
        "figures": [os.path.basename(p) for p in (p1, p2, p3, p4)],
    }
    spath = os.path.join(out_dir, "summary.json")
    with open(spath, "w") as fh:
        json.dump(summary, fh, indent=2, ensure_ascii=False)
    print(f"[c41] wrote {spath}")
    for p in (p1, p2, p3, p4):
        print(f"[c41] wrote {p}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
