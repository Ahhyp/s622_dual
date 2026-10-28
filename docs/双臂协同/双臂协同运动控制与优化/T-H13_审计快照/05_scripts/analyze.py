#!/usr/bin/env python3
"""C4.3 S4 W3-R1a T-H13: analyse the A/B/C rounds (C1..C5)."""
import json
import os
import statistics
import sys

BASE = "/tmp/th13"
PHASES = ["p1_read_us", "p2_dispatch_us", "p3_safety_us", "p4_task_solve_us",
          "p5_write_us", "p6_publish_us"]
COLS = ["k", "wall_ns", "cycle_us", "solve_us", "interval_us", "branch", "exit_code"] + PHASES


def pct(vals, p):
    if not vals:
        return 0.0
    s = sorted(vals)
    idx = p * (len(s) - 1)
    lo, hi = int(idx), min(int(idx) + 1, len(s) - 1)
    frac = idx - lo
    return s[lo] * (1 - frac) + s[hi] * frac


def read_csv(path):
    rows = []
    with open(path) as f:
        head = f.readline().strip().split(",")
        assert head == COLS, f"unexpected header in {path}: {head}"
        for line in f:
            parts = line.rstrip("\n").split(",")
            if len(parts) != len(COLS):
                continue
            rows.append({c: (float(v) if c not in ("k", "wall_ns", "branch", "exit_code")
                             else int(float(v)))
                         for c, v in zip(COLS, parts)})
    return rows


def longest_transport_block(rows):
    best, cur = [], []
    for r in rows:
        if r["branch"] == 1:
            cur.append(r)
            if len(cur) > len(best):
                best = list(cur)
        else:
            cur = []
    return best


def main():
    out = {}
    for label in ("a", "b", "c", "b2", "c2"):
        rj = os.path.join(BASE, label, "round.json")
        csv = os.path.join(BASE, label, "timing.csv")
        rec = json.load(open(rj)) if os.path.exists(rj) else {}
        info = {
            "params": rec.get("params_file"),
            "dump_success": rec.get("dump_success"),
            "dump_message": rec.get("dump_message"),
            "completed": rec.get("completed"),
            "final_state": rec.get("final_state"),
            "holds": rec.get("holds"),
            "final_t_traj": rec.get("final_t_traj"),
            "csv": csv if os.path.exists(csv) else None,
            "final_q_cmd": rec.get("final_q_cmd"),
        }
        if os.path.exists(csv):
            rows = read_csv(csv)
            info["rows"] = len(rows)
            blk = longest_transport_block(rows)
            info["transport_block"] = len(blk)
            info["branch_census"] = {
                "idle": sum(1 for r in rows if r["branch"] == 0),
                "transport": sum(1 for r in rows if r["branch"] == 1),
                "early_exit": sum(1 for r in rows if r["branch"] == 2),
            }
            win = blk[:638]
            info["window"] = len(win)
            for name in ("cycle_us", "solve_us", "interval_us"):
                v = [r[name] for r in win]
                info[name] = {"mean": statistics.fmean(v) if v else 0.0,
                              "p50": pct(v, .50), "p95": pct(v, .95), "p99": pct(v, .99),
                              "max": max(v) if v else 0.0}
            # C3 on every row of the file (each sample must not self-overlap).
            viol = []
            for r in rows:
                s = sum(r[p] for p in PHASES)
                if s > r["cycle_us"] + 1e-6:
                    viol.append((r["k"], s, r["cycle_us"]))
            info["c3_violations_all_rows"] = len(viol)
            info["c3_worst_examples"] = viol[:5]
            s_win = [sum(r[p] for p in PHASES) for r in win]
            info["c3_max_sum_window"] = max(s_win) if s_win else 0.0
            info["c3_max_ratio_window"] = max(
                (s / r["cycle_us"] for s, r in zip(s_win, win) if r["cycle_us"] > 0),
                default=0.0)
            # phase stats over the window
            phases = {}
            for p in PHASES:
                v = [r[p] for r in win]
                phases[p] = {"recorded": sum(1 for x in v if x > 0),
                             "mean": statistics.fmean(v) if v else 0.0,
                             "p50": pct(v, .50), "p95": pct(v, .95), "p99": pct(v, .99),
                             "max": max(v) if v else 0.0}
            info["phases_window"] = phases
            # C4: P6 == 0 samples and their branch
            z = [r for r in rows if r["p6_publish_us"] == 0.0]
            info["c4_p6_zero_rows"] = len(z)
            info["c4_p6_zero_by_branch"] = {
                str(b): sum(1 for r in z if r["branch"] == b) for b in (0, 1, 2)}
            info["c4_p6_zero_transport_bad"] = sum(1 for r in z if r["branch"] == 1)
            p6t = [r["p6_publish_us"] for r in rows if r["branch"] == 1]
            info["p6_transport_min"] = min(p6t) if p6t else None
        out[label] = info

    json.dump(out, open(os.path.join(BASE, "analysis.json"), "w"), indent=1)

    for label, i in out.items():
        print(f"### {label}")
        print(f"  params={i['params']}")
        print(f"  completed={i['completed']} state={i['final_state']} holds={i['holds']} "
              f"t_traj={i['final_t_traj']}")
        print(f"  dump_success={i['dump_success']}")
        if i.get("rows"):
            print(f"  rows={i['rows']} transport_block={i['transport_block']} "
                  f"census={i['branch_census']}")
            for name in ("cycle_us", "solve_us"):
                d = i[name]
                print(f"  {name}: mean={d['mean']:.1f} p50={d['p50']:.1f} "
                      f"p95={d['p95']:.1f} p99={d['p99']:.1f} max={d['max']:.1f}")
            print(f"  C3 violations(full file)={i['c3_violations_all_rows']} "
                  f"max_sum_window={i['c3_max_sum_window']:.2f} "
                  f"worst_ratio={i['c3_max_ratio_window']:.4f}")
            for p in PHASES:
                d = i["phases_window"][p]
                print(f"  {p}: rec={d['recorded']} mean={d['mean']:.2f} "
                      f"p50={d['p50']:.2f} p99={d['p99']:.2f} max={d['max']:.2f}")
            print(f"  C4 p6==0 rows={i['c4_p6_zero_rows']} by_branch="
                  f"{i['c4_p6_zero_by_branch']} transport_p6_zero="
                  f"{i['c4_p6_zero_transport_bad']} p6_transport_min={i['p6_transport_min']}")
        print()

    # C2: p99(C) - p99(B)
    for cb, cc in (("b", "c"), ("b2", "c2")):
        if out.get(cb, {}).get("cycle_us") and out.get(cc, {}).get("cycle_us"):
            d = out[cc]["cycle_us"]["p99"] - out[cb]["cycle_us"]["p99"]
            print(f"C2 {cc}-{cb}: p99 {out[cb]['cycle_us']['p99']:.1f} -> "
                  f"{out[cc]['cycle_us']['p99']:.1f} = {d:+.1f} us "
                  f"({'PASS' if d <= 100.0 else 'FAIL'} vs <= +100)")
    # bit-identity of the final q_cmd across rounds
    ref = out.get("a", {}).get("final_q_cmd")
    if ref:
        for label in ("b", "c", "b2", "c2"):
            q = out.get(label, {}).get("final_q_cmd")
            if q:
                same = all(abs(x - y) == 0.0 for x, y in zip(ref, q))
                print(f"final q_cmd identical a vs {label}: {same}")
                if not same:
                    md = max(abs(x - y) for x, y in zip(ref, q))
                    print(f"   max |dq| = {md:.3e}")


if __name__ == "__main__":
    main()
