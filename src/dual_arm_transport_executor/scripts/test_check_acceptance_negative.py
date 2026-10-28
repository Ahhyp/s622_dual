#!/usr/bin/env python3
"""Batch B negative tests for check_acceptance_yaml.py.

Each case mutates a copy of the real ledger in a temporary directory and
asserts (a) the exit code and (b) that the expected error code appears in the
output.  Nothing outside the temporary directory is written.

Run:
    python3 src/dual_arm_transport_executor/scripts/test_check_acceptance_negative.py
"""

from __future__ import annotations

import copy
import subprocess
import sys
import tempfile
from pathlib import Path

import yaml

HERE = Path(__file__).resolve().parent
CHECKER = HERE / "check_acceptance_yaml.py"
LEDGER = HERE.parent / "config/c43_s4_acceptance.yaml"

PASS = "PASS"
FAIL = "FAIL"


def run_checker(ledger: Path, *extra: str, plan: Path | None = None) -> subprocess.CompletedProcess:
    cmd = [sys.executable, str(CHECKER), "--ledger", str(ledger), *extra]
    if plan is not None:
        cmd += ["--plan", str(plan)]
    return subprocess.run(cmd, capture_output=True, text=True, timeout=120)


def entry(data, test_id):
    for e in data["entries"]:
        if e["test_id"] == test_id:
            return e
    raise KeyError(test_id)


def case(name, mutate, want_exit, want_code, extra=(), plan=None):
    """mutate(data) edits a deep copy; returns (name, ok, detail)."""
    data = yaml.safe_load(LEDGER.read_text(encoding="utf-8"))
    data = copy.deepcopy(data)
    mutate(data)
    with tempfile.TemporaryDirectory() as td:
        path = Path(td) / "ledger.yaml"
        path.write_text(yaml.safe_dump(data, allow_unicode=True, sort_keys=False), encoding="utf-8")
        res = run_checker(path, *extra, plan=plan)
    out = res.stdout + res.stderr
    ok = res.returncode == want_exit and (want_code is None or want_code in out)
    detail = "" if ok else f"exit={res.returncode} (want {want_exit}), code {want_code!r} absent"
    return (name, ok, detail, out)


CASES = []


def add(name, want_exit, want_code, mutate, extra=(), plan=None):
    CASES.append((name, want_exit, want_code, mutate, extra, plan))


# 1. 合法完整台账 ------------------------------------------------------------
def case_valid():
    res = run_checker(LEDGER)
    out = res.stdout + res.stderr
    ok = (res.returncode == 0 and "VERDICT: LEDGER_OK" in out
          and "ACCEPTANCE_PASS_CLAIMED: false" in out and "GATE_STATUS_CHANGED: false" in out)
    return ("valid ledger -> LEDGER_OK and explicitly no acceptance claim", ok,
            "" if ok else f"exit={res.returncode}", out)

# 2. 删除一个必需测试 --------------------------------------------------------
def drop_test(d):
    d["entries"] = [e for e in d["entries"] if e["test_id"] != "S4-T22"]
    d["required_test_ids"] = [t for t in d["required_test_ids"] if t != "S4-T22"]


add("delete required test S4-T22 -> rule error naming it", 3, "REQUIRED_TEST_MISSING", drop_test)

# 3. 删除一个必需指标 --------------------------------------------------------
def drop_metric(d):
    e = entry(d, "S4-T17")
    e["metrics"] = [m for m in e["metrics"] if m["metric"] != "long_run_max_interval"]


add("delete required metric S4-T17/long_run_max_interval", 3, "REQUIRED_METRIC_MISSING", drop_metric)

# 4. 重复 test_id ------------------------------------------------------------
def dup_test(d):
    clone = copy.deepcopy(entry(d, "S4-T3"))
    d["entries"].append(clone)


add("duplicate test_id -> conflict", 3, "DUPLICATE_TEST_ID", dup_test)

# 5. 同一测试内重复指标键 ----------------------------------------------------
def dup_metric(d):
    e = entry(d, "S4-T3")
    e["metrics"].append(copy.deepcopy(e["metrics"][0]))


add("duplicate metric key within a test", 3, "DUPLICATE_METRIC", dup_metric)

# 6/7. 非法 YAML / 结构非法 --------------------------------------------------
def raw_broken(_d):
    pass


def case_broken_yaml():
    with tempfile.TemporaryDirectory() as td:
        path = Path(td) / "broken.yaml"
        path.write_text("schema: [unclosed\nentries: {oops\n", encoding="utf-8")
        res = run_checker(path)
    ok = res.returncode == 2 and "YAML_UNPARSEABLE" in (res.stdout + res.stderr)
    return ("invalid YAML -> input error", ok, "" if ok else f"exit={res.returncode}", res.stdout + res.stderr)


def case_entries_wrong_type():
    data = copy.deepcopy(yaml.safe_load(LEDGER.read_text(encoding="utf-8")))
    data["entries"] = {"not": "a list"}
    with tempfile.TemporaryDirectory() as td:
        path = Path(td) / "ledger.yaml"
        path.write_text(yaml.safe_dump(data, allow_unicode=True, sort_keys=False), encoding="utf-8")
        res = run_checker(path)
    ok = res.returncode == 2 and "ENTRIES_NOT_LIST" in (res.stdout + res.stderr)
    return ("entries has wrong type -> input error", ok, "" if ok else f"exit={res.returncode}", res.stdout + res.stderr)


# 8. 已批准阈值缺来源 --------------------------------------------------------
def approved_without_source(d):
    m = entry(d, "S4-T5")["metrics"][0]
    m["source"] = ""


add("APPROVED threshold without source", 3, "APPROVED_WITHOUT_SOURCE", approved_without_source)

# 9. [待标定] 与状态不一致 ---------------------------------------------------
def pending_wrong_status(d):
    m = entry(d, "S4-T7")["metrics"][0]
    m["threshold_status"] = "APPROVED"


add("[待标定] but status APPROVED", 3, "PENDING_STATUS_MISMATCH", pending_wrong_status)

# 10. 外部阻断条目谎报 PASS --------------------------------------------------
def pass_on_blocked(d):
    e = entry(d, "S4-T2")
    e["gate_state"] = "BLOCKED_EXTERNAL_ENV"
    e["metrics"][0]["evidence_state"] = "PASS"
    e["metrics"][0]["evidence_ref"] = "build/somewhere"


add("PASS on externally blocked entry", 3, "PASS_ON_EXTERNALLY_BLOCKED", pass_on_blocked)

# 11/12. 历史 FAIL 被删除 / 被改写 ------------------------------------------
def drop_history(d):
    d["supplementary"].pop("historical_failures", None)


def rewrite_history(d):
    d["supplementary"]["historical_failures"][0]["evidence_state"] = "PASS"


add("historical FAIL record removed", 3, "HISTORICAL_FAILURE_DROPPED", drop_history)
add("historical FAIL rewritten to PASS", 3, "HISTORICAL_FAILURE_REWRITTEN", rewrite_history)

# 13. required_test_ids 漂移 -------------------------------------------------
def ids_drift(d):
    d["required_test_ids"] = [t for t in d["required_test_ids"] if t != "S4-T20"]


add("required_test_ids drifts from authoritative set", 3, "REQUIRED_IDS_DRIFT", ids_drift)

# 14. 台账自称验收通过 ------------------------------------------------------
def claim_pass(d):
    d["ledger_state"]["acceptance_pass_claimed"] = True


add("ledger claims acceptance PASS", 3, "ACCEPTANCE_CLAIMED", claim_pass)

# 15. 冻结规划不可用 --------------------------------------------------------
add("frozen plan missing -> input error", 2, "PLAN_MISSING", lambda d: None,
    (), Path("/nonexistent/plan.md"))

# 16. --list-pending 枚举完整 ------------------------------------------------
def case_list_pending():
    res = run_checker(LEDGER, "--list-pending")
    lines = [ln for ln in res.stdout.splitlines() if ln.startswith("S4-T")]
    ok = res.returncode == 0 and len(lines) == 12 and "# pending=12" in res.stdout
    return ("--list-pending enumerates all 12 items", ok,
            "" if ok else f"exit={res.returncode}, lines={len(lines)}", res.stdout)


def main() -> int:
    results = []
    for name, want_exit, want_code, mutate, extra, plan in CASES:
        _, ok, detail, out = case(name, mutate, want_exit, want_code, extra, plan)
        results.append((name, PASS if ok else FAIL, detail))
    for fn in (case_valid, case_broken_yaml, case_entries_wrong_type, case_list_pending):
        name, ok, detail, _out = fn()
        results.append((name, PASS if ok else FAIL, detail))

    width = max(len(r[0]) for r in results)
    print("=" * 78)
    print("W4 batch B — negative tests for check_acceptance_yaml.py")
    print("=" * 78)
    for name, status, detail in results:
        print(f"{status}  {name:<{width}}  {detail}")
    bad = [r for r in results if r[1] == FAIL]
    print("-" * 78)
    print(f"cases={len(results)} passed={len(results) - len(bad)} failed={len(bad)}")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
