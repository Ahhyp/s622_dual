#!/usr/bin/env python3
"""W3-R1 artifact schema checker (task book §3 W3-R1-06 / R1-T11).

Fails when a required file is missing or malformed.  Deliberately does not
"skip" empty directories: a checker that passes on nothing is worse than none.
"""
import json
import sys
from pathlib import Path

REQUIRED = ["manifest.json", "events.csv", "cycles.csv", "summary.json",
            "test_results.xml", "commands.txt"]

MANIFEST_KEYS = ["schema", "run_id", "git_head", "package_version", "build_type",
                 "mode", "env_class", "input_config_sha256", "started_utc"]
SUMMARY_KEYS = ["schema", "run_id", "result", "cycles", "events",
                "protection_events", "advances", "rejections", "final_state"]


def check_dir(run: Path) -> list[str]:
    errs = []
    for name in REQUIRED:
        p = run / name
        if not p.is_file():
            errs.append(f"{run.name}: missing {name}")
        elif p.stat().st_size == 0:
            errs.append(f"{run.name}: empty {name}")
    if errs:
        return errs

    try:
        man = json.loads((run / "manifest.json").read_text())
    except Exception as e:  # noqa: BLE001
        return [f"{run.name}: manifest.json is not valid JSON: {e}"]
    for k in MANIFEST_KEYS:
        if k not in man:
            errs.append(f"{run.name}: manifest missing key {k}")

    try:
        summ = json.loads((run / "summary.json").read_text())
    except Exception as e:  # noqa: BLE001
        errs.append(f"{run.name}: summary.json is not valid JSON: {e}")
        summ = {}
    for k in SUMMARY_KEYS:
        if k not in summ:
            errs.append(f"{run.name}: summary missing key {k}")

    head = (run / "cycles.csv").read_text().splitlines()[0]
    if head != "k,t,state,failure_code,committed,t_traj,q_cmd":
        errs.append(f"{run.name}: cycles.csv header is '{head}'")

    head = (run / "events.csv").read_text().splitlines()[0]
    if head != "t,kind,from,to,code,detail":
        errs.append(f"{run.name}: events.csv header is '{head}'")

    xml = (run / "test_results.xml").read_text(errors="replace")
    if "<testsuite" not in xml:
        errs.append(f"{run.name}: test_results.xml has no <testsuite> element")
    return errs


def main() -> int:
    root = Path(sys.argv[1]) if len(sys.argv) > 1 else Path("artifacts/w3_r1")
    if not root.is_dir():
        print(f"FAIL: artifact root does not exist: {root}", file=sys.stderr)
        return 1
    runs = [d for d in sorted(root.iterdir()) if d.is_dir()]
    if not runs:
        print(f"FAIL: no run directories under {root}", file=sys.stderr)
        return 1
    all_errs = []
    for run in runs:
        errs = check_dir(run)
        all_errs.extend(errs)
        print(f"{'FAIL' if errs else 'PASS'}  {run.name}")
    if all_errs:
        for e in all_errs:
            print("  - " + e, file=sys.stderr)
        return 1
    print(f"OK: {len(runs)} run directory(ies) satisfy the W3-R1 artifact schema")
    return 0


if __name__ == "__main__":
    sys.exit(main())
