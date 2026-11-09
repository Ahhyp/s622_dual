#!/usr/bin/env python3
"""Re-analysis of a 2.5-D C2 contact capture.

Separates three facts that must not be collapsed:
  1. each finger ever produced a contact pair
  2. one contact message carried both left fingers
  3. both left fingers still in contact after the physical stop

Also reports SIGNED sample displacement / rotation at the finger2 detach and a
planar pad-gap reconstruction.  The gap model `g = 2000 q - 0.18 mm` comes from
the 2.5-C mesh measurement; it is a model, not a measurement, and is marked so.

Usage: python3 c43_c2_analyze.py <tag> [--json]
"""
import csv
import json
import math
import os
import sys

OUTDIR = os.environ.get(
    "C43_CC_OUTDIR",
    "/workspace/docs/双臂协同/双臂协同运动控制与优化/T1_QP满速定位_20261029")
HALF = (0.06, 0.025, 0.025)
GAP_OFFSET = 0.00018   # metres; g_inner = 2000*q - GAP_OFFSET (2.5-C mesh fit)


def gap_of(q):
    """Inner pad gap in metres from a finger joint position in metres.

    2.5-C mesh fit: g_inner[mm] = 2000*q[m] - 0.18  ->  g = 2*q - 0.00018 m
    """
    return 2.0 * q - GAP_OFFSET


def quat_to_rotvec_deg(q):
    x, y, z, w = q
    n = math.sqrt(x * x + y * y + z * z)
    if n < 1e-12:
        return 0.0, (0.0, 0.0, 0.0)
    ang = 2.0 * math.atan2(n, w)
    return math.degrees(ang), (x / n, y / n, z / n)


def to_local(p, pose):
    x, y, z, qx, qy, qz, qw = pose
    v = (p[0] - x, p[1] - y, p[2] - z)
    q = (-qx, -qy, -qz, qw)
    cross = (q[1] * v[2] - q[2] * v[1],
             q[2] * v[0] - q[0] * v[2],
             q[0] * v[1] - q[1] * v[0])
    return (v[0] + 2 * (q[3] * cross[0] + q[1] * cross[2] - q[2] * cross[1]),
            v[1] + 2 * (q[3] * cross[1] + q[2] * cross[0] - q[0] * cross[2]),
            v[2] + 2 * (q[3] * cross[2] + q[0] * cross[1] - q[1] * cross[0]))


def faces_of(rec):
    if not rec.get("points_world") or not rec.get("sample_pose"):
        return []
    loc = to_local(rec["points_world"][0], rec["sample_pose"])
    return [(("xyz"[i]) + ("+" if loc[i] >= 0 else "-"), loc[i])
            for i in range(3) if abs(abs(loc[i]) - HALF[i]) < 0.0015]


def load(tag):
    cap = json.load(open(os.path.join(OUTDIR, f"contact_capture_{tag}.json")))
    raw = json.load(open(os.path.join(OUTDIR, f"contact_capture_{tag}_raw_contacts.json")))
    rows = list(csv.DictReader(open(os.path.join(OUTDIR, f"contact_capture_{tag}_joints.csv"))))
    return cap, raw, rows


def series(rows):
    out = []
    for r in rows:
        try:
            out.append({
                "sim": float(r["sim"]), "mono": float(r["mono"]),
                "q1": float(r["left_finger1_joint"]), "q2": float(r["left_finger2_joint"]),
                "p": (float(r["sample_x"]), float(r["sample_y"]), float(r["sample_z"])),
                "q": (float(r["sample_qx"]), float(r["sample_qy"]),
                      float(r["sample_qz"]), float(r["sample_qw"])),
            })
        except (TypeError, ValueError):
            continue
    return out


def at(ss, sim):
    return min(ss, key=lambda s: abs(s["sim"] - sim))


def main():
    tag = sys.argv[1] if len(sys.argv) > 1 else "c2_full"
    cap, raw, rows = load(tag)
    ss = series(rows)
    contacts = raw["contacts"]
    blocks = raw["blocks"]

    by = {}
    for c in contacts:
        by.setdefault(c["finger"], []).append(c)

    def rel(f):
        return [k for k in by if f in k]

    f1, f2 = rel("left_finger1"), rel("left_finger2")
    same_msg = 0
    for b in blocks:
        joined = " ".join(n for n in b["names"] if "left_finger" in n)
        if "left_finger1" in joined and "left_finger2" in joined:
            same_msg += 1
    stop = cap.get("stop") or {}
    post = [c for c in contacts if stop and c["recv_mono"] > stop["mono"] + 0.2]
    post_f = sorted({c["finger"] for c in post})
    sustained = (any("left_finger1" in f for f in post_f) and
                 any("left_finger2" in f for f in post_f))

    report = {
        "tag": tag,
        "target": cap.get("target"),
        "facts": {
            "left_finger1_ever_contacted": bool(f1),
            "left_finger2_ever_contacted": bool(f2),
            "both_ever_contacted": bool(f1) and bool(f2),
            "same_message_count": same_msg,
            "same_message": same_msg > 0,
            "sustained_both_after_stop": sustained,
            "post_stop_fingers": post_f,
        },
    }

    def times(f):
        recs = by.get(f, [])
        return [r["sim_sec"] + r["sim_nsec"] * 1e-9 for r in recs]

    for f in (f1[0] if f1 else None, f2[0] if f2 else None):
        if not f:
            continue
        ts = times(f)
        report.setdefault("finger_windows", {})[f] = {
            "n": len(ts), "first_sim": min(ts), "last_sim": max(ts),
            "duration_s": max(ts) - min(ts),
            "faces": [x[0] for x in faces_of(by[f][0])],
            "first_local_mm": [round(v * 1000, 2) for v in
                               to_local(by[f][0]["points_world"][0], by[f][0]["sample_pose"])],
        }

    # Signed motion at the detach moment (finger2 last contact).
    if f2 and ss:
        t_det = max(times(f2[0]))
        base = at(ss, t_det - 0.06)
        det = at(ss, t_det)
        after = at(ss, t_det + 0.02)
        ref = ss[0]
        rot_det, axis_det = quat_to_rotvec_deg(det["q"])
        report["motion"] = {
            "detach_sim": t_det,
            "sample_start_z_mm": round(ref["p"][2] * 1000, 4),
            "signed_mm_vs_start": {
                "at_first_contact": [round((v - ref["p"][i]) * 1000, 4)
                                     for i, v in enumerate(at(ss, min(times(f1[0])) if f1 else 0)["p"])],
                "at_finger2_first": [round((v - ref["p"][i]) * 1000, 4)
                                     for i, v in enumerate(at(ss, min(times(f2[0])))["p"])],
                "at_finger2_last": [round((v - ref["p"][i]) * 1000, 4)
                                    for i, v in enumerate(det["p"])],
                "at_end": [round((v - ref["p"][i]) * 1000, 4) for i, v in enumerate(ss[-1]["p"])],
            },
            "step_around_detach_mm": [round((det["p"][i] - base["p"][i]) * 1000, 4)
                                      for i in range(3)],
            "rotation_deg_at_detach": round(rot_det, 4),
            "rotation_axis": [round(a, 4) for a in axis_det],
            "rotation_deg_max": round(max(quat_to_rotvec_deg(s["q"])[0] for s in ss), 4),
        }

        # Planar pad-gap reconstruction (model, assumes pads parallel to the faces).
        t0 = min(times(f1[0])) if f1 else t_det
        s0 = at(ss, t0)
        pad1_y = s0["p"][1] - HALF[1]          # finger1 touches the sample's -y face
        pad2_y = pad1_y + gap_of(s0["q1"])
        report["gap_model"] = {
            "assumption": "pads parallel to sample faces; g_inner = 2000*q - 0.18 mm",
            "q_at_first_contact": round(s0["q1"], 7),
            "inner_gap_mm": round(gap_of(s0["q1"]) * 1000, 4),
            "pad1_y_mm": round(pad1_y * 1000, 4),
            "pad2_y_mm": round(pad2_y * 1000, 4),
            "samples": [],
        }
        for s in ss:
            if not (t0 - 0.01 <= s["sim"] <= t_det + 0.06):
                continue
            g2 = pad2_y - (s["p"][1] + HALF[1])   # + = open, - = interference
            g1 = (s["p"][1] - HALF[1]) - pad1_y
            report["gap_model"]["samples"].append({
                "sim": round(s["sim"], 3),
                "dy_mm": round((s["p"][1] - ss[0]["p"][1]) * 1000, 4),
                "dz_mm": round((s["p"][2] - ss[0]["p"][2]) * 1000, 4),
                "gap_finger2_mm": round(g2 * 1000, 4),
                "gap_finger1_mm": round(g1 * 1000, 4),
            })

    print(json.dumps(report, indent=2, ensure_ascii=False))
    if "--json" in sys.argv:
        path = os.path.join(OUTDIR, f"contact_capture_{tag}_analysis.json")
        with open(path, "w") as fh:
            json.dump(report, fh, indent=2, ensure_ascii=False)
        print("WROTE", path)


if __name__ == "__main__":
    main()
