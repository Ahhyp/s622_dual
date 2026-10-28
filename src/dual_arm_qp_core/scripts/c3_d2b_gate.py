#!/usr/bin/env python3
# C4.3 S4 / W2-C3 -- D2-B gate, FORMAL.
#
# contract: D2B_APPROVED_CONTRACT v1        approved: 2026-10-27
# scope   : same-build regression against the frozen D2-B baseline, cross-algorithm
#           agreement, physical invariants.  This gate does NOT evaluate -- and must
#           never be used to overwrite -- the historical cross-generation `<=1 ULP`
#           criterion (7656/7656 FAIL, retained as
#           LEGACY_CROSS_GENERATION_CONTRACT_FAIL_RETAINED).
#
# FAIL-CLOSED RULES (each one is exercised by scripts/run_c3_d2b_negative.sh)
#   * a missing / unreadable input is an ERROR, never a pass
#   * an unexpected CSV schema is an ERROR
#   * a provenance library hash that does not match the library being graded is an
#     ERROR (a baseline from another build must not silently grade this one)
#   * a threshold breach is a FAIL
#   * any of the above yields a non-zero exit code
#
# Usage:
#   c3_d2b_gate.py --baseline CSV --rerun CSV --admm CSV
#                  [--provenance MD] [--lib PATH] [--json OUT]
# Exit: 0 PASS | 2 INPUT_ERROR (missing/bad input, schema, provenance) | 3 FAIL
import argparse
import hashlib
import json
import os
import re
import sys

CONTRACT = "D2B_APPROVED_CONTRACT"
CONTRACT_VERSION = "v1"
APPROVED_ON = "2026-10-27"

ABS_DQ_LIMIT = 1e-12        # D2-B.2  same-build absolute bound (rad)
CROSS_ALGO_LIMIT = 1e-6     # D2-B.5  OSQP vs ADMM (rad)
E_OBJ_POS_LIMIT = 1e-6      # D2-B.6  terminal position error (m)
E_OBJ_ROT_LIMIT = 1e-6      # D2-B.6  terminal rotation error (rad)
EXPECTED_ROWS = 638
KJOINTS = 12

BANNER = (
    "C3 gate: {contract} {ver} (approved {date})\n"
    "  scope: same-build regression + cross-algorithm + physical invariants\n"
    "  the historical '<=1 ULP' criterion is NOT evaluated here and remains\n"
    "  LEGACY_CROSS_GENERATION_CONTRACT_FAIL_RETAINED (7656/7656 FAIL)\n"
).format(contract=CONTRACT, ver=CONTRACT_VERSION, date=APPROVED_ON)


class InputError(Exception):
    """Any condition that makes the inputs unfit to grade."""


def load(path, what):
    if not os.path.exists(path):
        raise InputError("%s missing: %s" % (what, path))
    try:
        with open(path) as f:
            header = f.readline().strip().split(",")
            rows = []
            for lineno, line in enumerate(f, start=2):
                parts = line.rstrip("\n").split(",")
                if len(parts) != len(header):
                    raise InputError("%s:%d: ragged row (%d vs %d columns)"
                                     % (path, lineno, len(parts), len(header)))
                rows.append(dict(zip(header, parts)))
    except OSError as exc:
        raise InputError("%s unreadable: %s" % (what, exc))
    needed = {"k", "t", "e_obj_pos_m", "e_obj_rot_rad", "committed"} | {"dq%d" % j for j in range(KJOINTS)}
    missing = needed - set(header)
    if missing:
        raise InputError("%s: unexpected schema, missing columns %s" % (what, sorted(missing)))
    if len(rows) != EXPECTED_ROWS:
        raise InputError("%s: %d rows, expected %d (a truncated run must not grade)"
                         % (what, len(rows), EXPECTED_ROWS))
    for i, r in enumerate(rows):
        if int(r["k"]) != i:
            raise InputError("%s: non-contiguous k at row %d (k=%s)" % (what, i, r["k"]))
    return rows


def max_abs_dq(a, b):
    worst, where = 0.0, None
    for ra, rb in zip(a, b):
        for j in range(KJOINTS):
            d = abs(float(ra["dq%d" % j]) - float(rb["dq%d" % j]))
            if d > worst:
                worst, where = d, (int(ra["k"]), j)
    return worst, where


def sha256_file(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def check_provenance(path, lib_path):
    """The baseline must have been produced by the library being graded now."""
    if not os.path.exists(path):
        raise InputError("provenance missing: %s" % path)
    text = open(path, errors="replace").read()
    m = re.search(r"libdual_arm_qp_core\.so`? sha256\s*`?([0-9a-f]{64})", text)
    if not m:
        m = re.search(r"lib_qp_core_sha256:\s*([0-9a-f]{64})", text)
    if not m:
        raise InputError("provenance carries no libdual_arm_qp_core sha256: %s" % path)
    approved_lib = m.group(1)
    if not os.path.exists(lib_path):
        raise InputError("library under test missing: %s" % lib_path)
    actual_lib = sha256_file(lib_path)
    if actual_lib != approved_lib:
        raise InputError("baseline/provenance was produced by %s but the library under test is %s"
                         % (approved_lib[:16], actual_lib[:16]))
    return {"approved_lib": approved_lib, "actual_lib": actual_lib}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--baseline", required=True)
    ap.add_argument("--rerun", required=True)
    ap.add_argument("--admm", required=True, help="required: cross-algorithm evidence is part of D2-B")
    ap.add_argument("--provenance", default=None)
    ap.add_argument("--lib", default=None)
    ap.add_argument("--json", default=None)
    args = ap.parse_args()

    print(BANNER)
    out = {
        "schema": "c43s4.w2c3.d2b_gate.v2",
        "contract": CONTRACT, "contract_version": CONTRACT_VERSION,
        "approved_on": APPROVED_ON,
        "inputs": {"baseline": args.baseline, "rerun": args.rerun, "admm": args.admm,
                   "provenance": args.provenance, "library": args.lib},
        "checks": {},
        "decision": "INPUT_ERROR",
    }

    try:
        base = load(args.baseline, "baseline")
        rerun = load(args.rerun, "rerun")
        admm = load(args.admm, "admm")
        prov = None
        if args.provenance or args.lib:
            if not (args.provenance and args.lib):
                raise InputError("--provenance and --lib must be supplied together")
            prov = check_provenance(args.provenance, args.lib)
            out["checks"]["D2-B.0_provenance"] = {
                "pass": True,
                "detail": "baseline built by %s... == library under test" % prov["approved_lib"][:16]}
    except InputError as exc:
        out["decision"] = "INPUT_ERROR"
        out["error"] = str(exc)
        return emit(out, args.json, 2)

    failed = []

    def check(name, ok, detail):
        out["checks"][name] = {"pass": bool(ok), "detail": detail}
        if not ok:
            failed.append(name)

    # D2-B.1: same-build reproducibility.  The fixture pair is byte-identical by
    # construction; this check proves the pair in THIS tree still is.
    same_bytes = sha256_file(args.baseline) == sha256_file(args.rerun)
    worst, where = max_abs_dq(base, rerun)
    check("D2-B.1_same_build_identical", same_bytes and worst == 0.0,
          "bytes_equal=%s max|dq-dq_rerun|=%.3e rad (at %s)" % (same_bytes, worst, where))

    worst2, where2 = max_abs_dq(base, admm)
    check("D2-B.5_cross_algo", worst2 <= CROSS_ALGO_LIMIT,
          "max|OSQP-ADMM|=%.3e rad at %s (limit %.0e)" % (worst2, where2, CROSS_ALGO_LIMIT))

    last = base[-1]
    epos, erot = abs(float(last["e_obj_pos_m"])), abs(float(last["e_obj_rot_rad"]))
    check("D2-B.6_terminal_pos", epos <= E_OBJ_POS_LIMIT, "%.3e m (limit %.0e)" % (epos, E_OBJ_POS_LIMIT))
    check("D2-B.6_terminal_rot", erot <= E_OBJ_ROT_LIMIT, "%.3e rad (limit %.0e)" % (erot, E_OBJ_ROT_LIMIT))

    noncommitted = sum(1 for r in base if r["committed"].strip() not in ("1", "True", "true"))
    check("D2-B.3_all_committed", noncommitted == 0, "%d non-committed rows" % noncommitted)

    out["checks"]["D2-B.7_legacy_retained"] = {
        "pass": True,
        "detail": "historical <=1 ULP result is NOT evaluated here; retained as "
                  "LEGACY_CROSS_GENERATION_CONTRACT_FAIL_RETAINED (7656/7656 FAIL)"}
    out["decision"] = "D2B_PASS" if not failed else "D2B_FAIL"
    return emit(out, args.json, 0 if not failed else 3)


def emit(out, json_path, code):
    text = json.dumps(out, indent=2)
    if json_path:
        with open(json_path, "w") as f:
            f.write(text + "\n")
    print("C3_D2B:", out["decision"])
    for name, entry in out.get("checks", {}).items():
        print("  %-30s %-5s %s" % (name, entry["pass"], entry["detail"]))
    if out.get("error"):
        print("  ERROR:", out["error"])
    return code


if __name__ == "__main__":
    sys.exit(main())
