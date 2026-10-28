#!/usr/bin/env python3
"""C4.3 S4 W3-R1a T-H13: analyse the E8 early-exit probe (branch 2 samples)."""
import json
import statistics

BASE = "/tmp/th13"
PHASES = ["p1_read_us", "p2_dispatch_us", "p3_safety_us", "p4_task_solve_us",
          "p5_write_us", "p6_publish_us"]
COLS = ["k", "wall_ns", "cycle_us", "solve_us", "interval_us", "branch", "exit_code"] + PHASES


def pct(v, p):
    if not v:
        return 0.0
    s = sorted(v)
    idx = p * (len(s) - 1)
    lo, hi = int(idx), min(int(idx) + 1, len(s) - 1)
    return s[lo] * (1 - (idx - lo)) + s[hi] * (idx - lo)


def main():
    path = f"{BASE}/c_ee/timing.csv"
    rows = []
    with open(path) as f:
        head = f.readline().strip().split(",")
        assert head == COLS, head
        for line in f:
            p = line.rstrip("\n").split(",")
            if len(p) != len(COLS):
                continue
            rows.append({c: (float(v) if c not in ("k", "wall_ns", "branch", "exit_code")
                             else int(float(v))) for c, v in zip(COLS, p)})
    rec = json.load(open(f"{BASE}/c_ee/round.json"))
    print(f"rows={len(rows)} dump_success={rec['dump_success']}")
    print(f"dump: {rec['dump_message']}")
    census = {}
    for r in rows:
        census[(r["branch"], r["exit_code"])] = census.get((r["branch"], r["exit_code"]), 0) + 1
    print(f"census (branch,exit_code): {dict(sorted(census.items()))}")
    viol = [r for r in rows
            if sum(r[p] for p in PHASES) > r["cycle_us"] + 1e-6]
    print(f"C3 violations (all rows) = {len(viol)}")
    for br in (0, 1, 2):
        sub = [r for r in rows if r["branch"] == br]
        if not sub:
            continue
        print(f"branch {br}: n={len(sub)}")
        for p in PHASES:
            v = [r[p] for r in sub]
            print(f"   {p}: nz={sum(1 for x in v if x > 0)} p50={pct(v,.5):.2f} "
                  f"p99={pct(v,.99):.2f} max={max(v):.2f}")
        sums = [sum(r[p] for p in PHASES) for r in sub]
        print(f"   sum(P1..P6): max={max(sums):.2f} worst_ratio="
              f"{max((s / r['cycle_us'] for s, r in zip(sums, sub) if r['cycle_us'] > 0), default=0):.4f}")
        cyc = [r["cycle_us"] for r in sub]
        print(f"   cycle_us: p50={pct(cyc,.5):.1f} p99={pct(cyc,.99):.1f} max={max(cyc):.1f}")
        z6 = sum(1 for r in sub if r["p6_publish_us"] == 0.0)
        z4 = sum(1 for r in sub if r["p4_task_solve_us"] == 0.0)
        print(f"   P6==0: {z6}   P4==0: {z4}")


if __name__ == "__main__":
    main()
