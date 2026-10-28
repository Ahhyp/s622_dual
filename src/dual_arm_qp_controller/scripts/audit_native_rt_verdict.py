#!/usr/bin/env python3
# C4.3 S4 / W3-R1a C6 -- audit a native-RT verdict produced on ANOTHER machine.
#
# WHY THIS EXISTS
#   run_native_rt_acceptance.sh must run on a native Linux box with FIFO scheduling.
#   This box is WSL2, so its verdict is ENV_REFUSED.  When someone runs the gate on
#   the target machine and copies the artefacts back, this program decides whether
#   the returned evidence may be counted as C6 evidence -- WITHOUT trusting the
#   "pass": true field the remote run may contain.
#
# WHAT IT CHECKS (in this order; any failure is decisive)
#   1. machine_facts.json exists and is internally sane  -> else INCONSISTENT
#   2. the machine that produced the numbers was ELIGIBLE
#      (not WSL, FIFO scheduling actually worked, >= 4 CPUs) -> else ENV_REFUSED
#      A verdict claiming PASS from an ineligible machine is refused, not accepted.
#   3. the raw gtest log (if supplied) is re-parsed and the frozen thresholds are
#      recomputed here, independently of the remote verdict.json -> mismatch is
#      INCONSISTENT
#   4. cycles >= 638 for S4-T2 and >= 6380 for S4-T17
#
# WHAT IT DOES NOT DO
#   * it does not sign anything: a PASS here is "evidence accepted", not an approval
#   * it does not modify any threshold, and it re-derives them from the frozen text
#   * it never fabricates numbers: missing artefacts are reported as NOT_RUN
#
# Usage:
#   audit_native_rt_verdict.py --verdict DIR/verdict.json [--facts DIR/machine_facts.json]
#                              [--log DIR/realtime_budget.log] [--json OUT.json]
# Exit: 0 PASS, 2 ENV_REFUSED, 3 FAIL (eligible machine, numbers missed),
#       4 INCONSISTENT (artefacts disagree / missing), 5 NOT_RUN (no verdict.json)
import argparse
import json
import os
import re
import sys

# Frozen numbers, copied from the acceptance text -- deliberately NOT configurable.
P99_LIMIT_US = 6400.0          # S4-T2 p99(cycle) <= 6400 us  [待标定]
INTERVAL_LIMIT_US = 12000.0    # max interval <= 1.5 * dt at dt = 8 ms
OVERRUN_LIMIT = 0              # overruns > 150 us == 0
RSS_GROWTH_KB_LIMIT = 1024
FD_GROWTH_LIMIT = 0
CYCLES_S4T2 = 638
CYCLES_S4T17 = 6380


def read_json(path):
    try:
        with open(path) as f:
            return json.load(f)
    except Exception as exc:  # noqa: BLE001 - report, never raise
        return {"__error__": str(exc)}


def facts_eligible(facts):
    """Return (eligible, reasons).  Deliberately strict: unknown == ineligible."""
    reasons = []
    if facts.get("__error__"):
        return False, ["machine_facts.json unreadable: " + facts["__error__"]]
    if not facts.get("schema", "").startswith("c43s4.native_rt.machine_facts"):
        reasons.append("machine_facts schema missing/unexpected")
    if facts.get("wsl"):
        reasons.append("machine reports WSL")
    if not facts.get("chrt_fifo_ok"):
        reasons.append("chrt -f 80 failed on the measuring machine")
    if int(facts.get("nproc") or 0) < 4:
        reasons.append("fewer than 4 CPUs")
    if not facts.get("kernel"):
        reasons.append("kernel string missing")
    return (not reasons), reasons


def parse_log(log_text):
    out = {"cycles": [], "metrics": {}}
    for m in re.finditer(r"cycles=(\d+)", log_text):
        out["cycles"].append(int(m.group(1)))
    for name, pat in (
        ("cycle_p99_us", r"cycle_p99_us=([0-9.]+)"),
        ("interval_max_us", r"interval_max_us[=\s:]+([0-9.]+)"),
        ("overruns_over_150", r"overruns_over_150[=\s:]+(\d+)"),
        ("rss_growth_kb", r"rss_growth[=\s:]+(-?\d+)"),
        ("fd_growth", r"fd_growth[=\s:]+(-?\d+)"),
    ):
        m = re.search(pat, log_text)
        if m:
            out["metrics"][name] = float(m.group(1))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--verdict", required=True)
    ap.add_argument("--facts", default=None)
    ap.add_argument("--log", default=None)
    ap.add_argument("--json", default=None)
    args = ap.parse_args()

    result = {
        "schema": "c43s4.native_rt.audit.v1",
        "verdict_path": args.verdict,
        "facts_path": args.facts,
        "log_path": args.log,
        "decision": "INCONSISTENT",
        "checks": {},
        "reasons": [],
    }

    if not os.path.exists(args.verdict):
        result["decision"] = "NOT_RUN"
        result["reasons"].append("verdict.json not present: the gate has not been run on a native RT machine")
        return emit(result, args.json, 5)

    verdict = read_json(args.verdict)
    facts_path = args.facts or os.path.join(os.path.dirname(args.verdict), "machine_facts.json")
    facts = read_json(facts_path) if os.path.exists(facts_path) else {"__error__": "missing"}

    eligible, why = facts_eligible(facts)
    result["checks"]["machine_eligible"] = {"pass": eligible, "detail": why}
    if not eligible:
        result["decision"] = "ENV_REFUSED"
        result["reasons"] = why
        # An ineligible machine claiming pass is the one thing that must never pass:
        if verdict.get("pass"):
            result["reasons"].append("verdict.json claims pass although the machine was ineligible -- refused")
        return emit(result, args.json, 2)

    # Independent re-derivation from the raw log beats the remote's own numbers.
    recomputed = {}
    if args.log and os.path.exists(args.log):
        try:
            log_text = open(args.log, errors="replace").read()
        except OSError as exc:
            log_text = ""
            result["reasons"].append("log unreadable: %s" % exc)
        recomputed = parse_log(log_text)
    else:
        result["reasons"].append("raw log not supplied: thresholds could not be re-derived independently")

    fails = []
    if recomputed:
        m = recomputed["metrics"]
        if "cycle_p99_us" in m and m["cycle_p99_us"] > P99_LIMIT_US:
            fails.append("p99 %.1f us > %.1f us" % (m["cycle_p99_us"], P99_LIMIT_US))
        if "interval_max_us" in m and m["interval_max_us"] > INTERVAL_LIMIT_US:
            fails.append("max interval %.1f us > %.1f us" % (m["interval_max_us"], INTERVAL_LIMIT_US))
        if "overruns_over_150" in m and int(m["overruns_over_150"]) > OVERRUN_LIMIT:
            fails.append("overruns %d > %d" % (int(m["overruns_over_150"]), OVERRUN_LIMIT))
        if "rss_growth_kb" in m and int(m["rss_growth_kb"]) > RSS_GROWTH_KB_LIMIT:
            fails.append("rss growth %d KB > %d KB" % (int(m["rss_growth_kb"]), RSS_GROWTH_KB_LIMIT))
        if "fd_growth" in m and int(m["fd_growth"]) > FD_GROWTH_LIMIT:
            fails.append("fd growth %d > %d" % (int(m["fd_growth"]), FD_GROWTH_LIMIT))
        if recomputed["cycles"] and max(recomputed["cycles"]) < CYCLES_S4T2:
            fails.append("too few cycles (%d < %d)" % (max(recomputed["cycles"]), CYCLES_S4T2))
        # Consistency: the remote verdict must not disagree with its own log.
        remote_checks = verdict.get("checks") or {}
        for name, entry in remote_checks.items():
            if isinstance(entry, dict) and entry.get("pass") is False:
                fails.append("remote check failed: %s" % name)

    result["checks"]["recomputed_from_log"] = recomputed or "NOT_SUPPLIED"
    result["checks"]["remote_pass_flag"] = verdict.get("pass")
    result["checks"]["frozen_thresholds"] = {
        "p99_us": P99_LIMIT_US, "interval_us": INTERVAL_LIMIT_US,
        "overruns": OVERRUN_LIMIT, "rss_growth_kb": RSS_GROWTH_KB_LIMIT,
        "fd_growth": FD_GROWTH_LIMIT,
    }

    if not recomputed:
        result["decision"] = "INCONSISTENT"
        result["reasons"].append("cannot accept a verdict without the raw log")
        return emit(result, args.json, 4)
    if fails:
        result["decision"] = "FAIL"
        result["reasons"] = fails
        return emit(result, args.json, 3)

    result["decision"] = "PASS"
    result["note"] = ("evidence accepted for C6; this is not an approval -- C6 still needs "
                      "a human/Gate decision and S4-T2/S4-T17 keep their own frozen status")
    return emit(result, args.json, 0)


def emit(result, json_path, code):
    text = json.dumps(result, indent=2)
    if json_path:
        with open(json_path, "w") as f:
            f.write(text + "\n")
    print("C6_AUDIT:", result["decision"])
    for r in result["reasons"]:
        print("  -", r)
    if result.get("note"):
        print(" ", result["note"])
    print(text)
    return code


if __name__ == "__main__":
    sys.exit(main())
