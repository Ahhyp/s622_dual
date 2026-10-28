#!/usr/bin/env python3
"""C4.3 S4 — W4 验收台账结构检查器（批次 B）.

用途
----
读取 `config/c43_s4_acceptance.yaml`（W4 阈值台账），检查它是否
**结构合规**、**覆盖了冻结基线要求的全部验收条目与指标**，并完整列出
`[待标定]` 项。

它**不是**验收执行器：本脚本的任何输出都不构成 S4.1 / S4 Gate 的 PASS。

关键设计：必需项目集合**不来自被检查的 YAML 本身**
------------------------------------------------
若从 YAML 推导必需项，删掉某个条目后检查器仍会认为"完整"。因此：

* 权威必需集合写在 `REQUIRED_METRICS`（逐条对应冻结规划 §7 测试矩阵的
  用例编号与"量化指标"列，2026-10-27 冻结版本）；
* 运行时会解析冻结规划 §7 表格，确认 `REQUIRED_METRICS` 的每个编号在
  规划中确实存在（防止检查器自身漂移）；
* YAML 顶层 `required_test_ids` 只作为**被检查字段**：与权威集合不一致即
  报错，而不是用来决定检查范围。

退出码（稳定定义）
------------------
0  LEDGER_OK         结构合规、覆盖完整（仍不代表任何验收 PASS）
2  INPUT_ERROR       YAML 无法解析 / 路径不存在 / 关键字段类型非法
3  LEDGER_RULE_ERROR 覆盖缺失、重复冲突、批准来源缺失、状态越权等

用法
----
    python3 check_acceptance_yaml.py                     # 检查默认台账
    python3 check_acceptance_yaml.py --ledger X.yaml     # 检查指定台账
    python3 check_acceptance_yaml.py --json              # 机器可读输出
    python3 check_acceptance_yaml.py --list-pending      # 只列 [待标定]
    python3 check_acceptance_yaml.py --print-requirements
"""

from __future__ import annotations

import argparse
import json
import re
import sys
from pathlib import Path
from typing import Any, Dict, List, Tuple

try:
    import yaml
except ImportError:  # pragma: no cover - pyyaml is a project dependency
    sys.stderr.write("INPUT_ERROR: PyYAML is required (python3 -c 'import yaml')\n")
    raise SystemExit(2)

EXIT_OK = 0
EXIT_INPUT = 2
EXIT_RULE = 3

def _find_repo_root(start: Path) -> Path:
    """Locate the workspace root even when this script is run from install/."""
    for candidate in [start, *start.parents]:
        if (candidate / "docs/双臂协同").is_dir() and (candidate / "src").is_dir():
            return candidate
    return start.parents[3]


REPO_ROOT = _find_repo_root(Path(__file__).resolve())
DEFAULT_LEDGER = REPO_ROOT / "src/dual_arm_transport_executor/config/c43_s4_acceptance.yaml"
FROZEN_PLAN = (
    REPO_ROOT
    / "docs/双臂协同/双臂协同运动控制与优化/C4.3_S4_执行规划.md"
)

# ---------------------------------------------------------------------------
# 权威必需集合：冻结规划 §7「测试矩阵与量化验收标准」的用例 + 量化指标。
# 仅当冻结规划正式修订时才允许改动本表；改动必须同步更新文档与阴性测试。
# ---------------------------------------------------------------------------
REQUIRED_METRICS: Dict[str, Tuple[str, ...]] = {
    # S4-T1 既有功能回归 → 用例数（按包的下限）与失败数
    "S4-T1": ("package_test_counts",),
    # S4-T2 实时性预算 → P50/P99/P99.9 预算、超周期数、无 malloc
    "S4-T2": ("update_cost_p99", "overrun_cycles", "malloc_in_update"),
    # S4-T3 行装配与 S3 逐位一致
    "S4-T3": ("row_assembly_bitwise_delta",),
    # S4-T4 符号与前馈
    "S4-T4": ("error_descent_ratio",),
    # S4-T5 pi-0.05 拒绝边界
    "S4-T5": ("reject_boundary_rad",),
    # S4-T6 有无前馈对照
    "S4-T6": ("feedforward_error_ratio",),
    # S4-T7 轨迹与相对位姿误差
    "S4-T7": ("terminal_object_error_pos", "peak_relative_pose_error"),
    # S4-T8 故障注入（7 类）
    "S4-T8": ("injection_failure_code_and_state",),
    # S4-T9 限位饱和与归因
    "S4-T9": ("limit_margin_rad",),
    # S4-T10 物体—环境碰撞反例
    "S4-T10": ("collision_trigger_cycle",),
    # S4-T11 解析判据 vs FCL 采样交叉验证
    "S4-T11": ("cross_check_mismatches",),
    # S4-T12 单/双附着对照
    "S4-T12": ("penetration_single_attach",),
    # S4-T13 完整 Gazebo 流程
    "S4-T13": ("dry_run_artifact_completeness",),
    # S4-T14 HOLD 与恢复
    "S4-T14": ("hold_frozen_qcmd_delta",),
    # S4-T15 目标/状态陈旧
    "S4-T15": ("stale_state_trigger_cycles",),
    # S4-T16 抓持检查
    "S4-T16": ("grasp_check_reject",),
    # S4-T17 长时间实时性
    "S4-T17": ("long_run_p99", "long_run_max_interval"),
    # S4-T18 插件实测
    "S4-T18": ("plugin_probe",),
    # S4-T19 轨迹时钟与 HOLD/RESUME
    "S4-T19": ("t_traj_increment",),
    # S4-T20 有限几何
    "S4-T20": ("off_table_false_reject",),
    # S4-T21 执行器死亡 → 控制器本地 HOLD
    "S4-T21": ("local_hold_latency",),
    # S4-T22 故障锁存与恢复权限
    "S4-T22": ("latch_recovery_permission",),
}

THRESHOLD_STATUS = ("APPROVED", "PENDING_CALIBRATION", "UNAPPROVED_SOURCE_UNCONFIRMED")
SOFTWARE_STATE = ("IMPLEMENTED", "PARTIAL", "NOT_IMPLEMENTED", "OUT_OF_SCOPE")
EVIDENCE_STATE = ("PASS", "PARTIAL", "FAIL", "HOLD", "UNVERIFIED", "NOT_RUN", "MISSING_EVIDENCE")
GATE_STATE = ("CLOSED", "OPEN", "HOLD", "BLOCKED_EXTERNAL_ENV", "UNAPPROVED")

REQUIRED_ENTRY_FIELDS = ("test_id", "title", "layer", "planned_step", "source",
                         "software_state", "gate_state", "metrics")
REQUIRED_METRIC_FIELDS = ("metric", "unit", "criterion", "threshold", "threshold_status",
                          "source", "evidence_state", "evidence_ref", "scope")

PENDING_LITERAL = "[待标定]"


class Findings:
    """Accumulates errors/warnings and the report material."""

    def __init__(self) -> None:
        self.errors: List[Tuple[str, str]] = []   # (code, message)
        self.warnings: List[Tuple[str, str]] = []
        self.pending: List[Dict[str, Any]] = []
        self.held: List[Dict[str, Any]] = []
        self.failed: List[Dict[str, Any]] = []
        self.entries_seen = 0
        self.metrics_seen = 0

    def error(self, code: str, msg: str) -> None:
        self.errors.append((code, msg))

    def warn(self, code: str, msg: str) -> None:
        self.warnings.append((code, msg))


def load_ledger(path: Path, f: Findings) -> Dict[str, Any] | None:
    if not path.is_file():
        f.error("LEDGER_MISSING", f"ledger file not found: {path}")
        return None
    try:
        with path.open(encoding="utf-8") as fh:
            data = yaml.safe_load(fh)
    except yaml.YAMLError as exc:
        f.error("YAML_UNPARSEABLE", f"{path.name}: {exc}")
        return None
    if not isinstance(data, dict):
        f.error("YAML_ROOT_NOT_MAPPING", f"{path.name}: top level must be a mapping")
        return None
    return data


def plan_test_ids(plan_path: Path, f: Findings) -> List[str] | None:
    """Extract the row ids of the frozen §7 matrix (independent of the YAML)."""
    if not plan_path.is_file():
        f.error("PLAN_MISSING", f"frozen plan not found: {plan_path}")
        return None
    text = plan_path.read_text(encoding="utf-8")
    # Rows look like:  | **S4-T1** | 既有功能回归 | ...
    ids = re.findall(r"^\|\s*\*\*(S4-T\d{1,2})\*\*\s*\|", text, flags=re.MULTILINE)
    ids = list(dict.fromkeys(ids))
    if len(ids) < 20:
        f.error("PLAN_UNPARSEABLE",
                f"expected >=20 S4-Tn rows in {plan_path.name}, found {len(ids)}")
        return None
    return ids


def check_vocab(name: str, value: Any, allowed: Tuple[str, ...], where: str, f: Findings) -> None:
    if value not in allowed:
        f.error("VOCAB", f"{where}: {name}={value!r} not in {allowed}")


def check_ledger(data: Dict[str, Any], plan_ids: List[str], f: Findings) -> None:
    # ---- coverage against the authoritative mapping (not against the YAML) ----
    entries = data.get("entries")
    if not isinstance(entries, list) or not entries:
        f.error("ENTRIES_NOT_LIST", "entries: must be a non-empty list")
        return
    f.entries_seen = len(entries)

    seen_ids: Dict[str, int] = {}
    by_id: Dict[str, Dict[str, Any]] = {}
    for idx, entry in enumerate(entries):
        if not isinstance(entry, dict):
            f.error("ENTRY_NOT_MAPPING", f"entries[{idx}]: must be a mapping")
            continue
        for field in REQUIRED_ENTRY_FIELDS:
            if field not in entry:
                f.error("ENTRY_FIELD_MISSING", f"entries[{idx}]: missing {field!r}")
        tid = entry.get("test_id")
        if not isinstance(tid, str) or not tid.strip():
            f.error("TEST_ID_INVALID", f"entries[{idx}]: test_id must be a non-empty string")
            continue
        seen_ids[tid] = seen_ids.get(tid, 0) + 1
        by_id.setdefault(tid, entry)
        check_vocab("software_state", entry.get("software_state"), SOFTWARE_STATE, tid, f)
        check_vocab("gate_state", entry.get("gate_state"), GATE_STATE, tid, f)

        metrics = entry.get("metrics")
        if not isinstance(metrics, list) or not metrics:
            f.error("METRICS_NOT_LIST", f"{tid}: metrics must be a non-empty list")
            continue
        metric_keys: Dict[str, int] = {}
        for midx, metric in enumerate(metrics):
            f.metrics_seen += 1
            where = f"{tid}/metrics[{midx}]"
            if not isinstance(metric, dict):
                f.error("METRIC_NOT_MAPPING", f"{where}: must be a mapping")
                continue
            for field in REQUIRED_METRIC_FIELDS:
                if field not in metric:
                    f.error("METRIC_FIELD_MISSING", f"{where}: missing {field!r}")
            key = metric.get("metric")
            if not isinstance(key, str) or not key.strip():
                f.error("METRIC_KEY_INVALID", f"{where}: metric must be a non-empty string")
                continue
            metric_keys[key] = metric_keys.get(key, 0) + 1
            check_vocab("threshold_status", metric.get("threshold_status"),
                        THRESHOLD_STATUS, f"{tid}/{key}", f)
            check_vocab("evidence_state", metric.get("evidence_state"),
                        EVIDENCE_STATE, f"{tid}/{key}", f)

            threshold = metric.get("threshold")
            status = metric.get("threshold_status")
            if threshold == PENDING_LITERAL and status != "PENDING_CALIBRATION":
                f.error("PENDING_STATUS_MISMATCH",
                        f"{tid}/{key}: threshold {PENDING_LITERAL} requires PENDING_CALIBRATION "
                        f"(got {status!r})")
            if status == "APPROVED":
                if not str(metric.get("source") or "").strip():
                    f.error("APPROVED_WITHOUT_SOURCE",
                            f"{tid}/{key}: APPROVED threshold requires a non-empty source")

            if metric.get("evidence_state") == "PASS":
                ref = str(metric.get("evidence_ref") or "").strip()
                if not ref:
                    f.error("PASS_WITHOUT_EVIDENCE",
                            f"{tid}/{key}: evidence_state=PASS requires evidence_ref")
                if entry.get("gate_state") in ("HOLD", "BLOCKED_EXTERNAL_ENV"):
                    f.error("PASS_ON_EXTERNALLY_BLOCKED",
                            f"{tid}/{key}: PASS is not allowed while gate_state="
                            f"{entry.get('gate_state')}")

            if threshold == PENDING_LITERAL:
                f.pending.append({"test_id": tid, "metric": key,
                                  "suggested_value": metric.get("suggested_value")})
            if metric.get("evidence_state") in ("HOLD", "UNVERIFIED", "MISSING_EVIDENCE", "PARTIAL"):
                f.held.append({"test_id": tid, "metric": key,
                               "evidence_state": metric.get("evidence_state")})

        for key, count in metric_keys.items():
            if count > 1:
                f.error("DUPLICATE_METRIC",
                        f"{tid}: metric {key!r} appears {count} times (one test may map to "
                        f"many metrics, but a metric key must be unique within its test)")

    for tid, count in seen_ids.items():
        if count > 1:
            f.error("DUPLICATE_TEST_ID", f"test_id {tid} appears {count} times")

    # ---- required coverage ----
    for tid, keys in REQUIRED_METRICS.items():
        if tid not in by_id:
            f.error("REQUIRED_TEST_MISSING", f"required test {tid} is absent from the ledger")
            continue
        metrics = by_id[tid].get("metrics")
        present = set()
        if isinstance(metrics, list):
            present = {m.get("metric") for m in metrics if isinstance(m, dict)}
        for key in keys:
            if key not in present:
                f.error("REQUIRED_METRIC_MISSING",
                        f"required metric {tid}/{key} is absent from the ledger")

    # ---- the ledger's own claim about the required set must not drift ----
    declared = data.get("required_test_ids")
    if not isinstance(declared, list) or not declared:
        f.error("REQUIRED_IDS_MISSING", "required_test_ids: must be a non-empty list")
    else:
        if len(set(declared)) != len(declared):
            f.error("REQUIRED_IDS_DUPLICATE", "required_test_ids contains duplicates")
        missing = [t for t in REQUIRED_METRICS if t not in declared]
        extra = [t for t in declared if t not in REQUIRED_METRICS]
        if missing:
            f.error("REQUIRED_IDS_DRIFT",
                    f"required_test_ids omits authoritative ids: {missing}")
        if extra:
            f.error("REQUIRED_IDS_DRIFT",
                    f"required_test_ids contains ids outside the frozen matrix: {extra}")

    # ---- historical failures must survive ----
    supp = data.get("supplementary") or {}
    hist = supp.get("historical_failures") if isinstance(supp, dict) else None
    if not isinstance(hist, list) or not hist:
        f.error("HISTORICAL_FAILURE_DROPPED",
                "supplementary.historical_failures: must keep the W2-C3 <=1 ULP FAIL record")
    else:
        for rec in hist:
            if isinstance(rec, dict):
                f.failed.append({"id": rec.get("id"), "evidence_state": rec.get("evidence_state")})
        if not any(isinstance(r, dict) and r.get("evidence_state") == "FAIL" for r in hist):
            f.error("HISTORICAL_FAILURE_REWRITTEN",
                    "supplementary.historical_failures: the retained record must stay FAIL")

    # ---- ledger must not claim acceptance ----
    state = data.get("ledger_state") or {}
    if isinstance(state, dict) and state.get("acceptance_pass_claimed") is True:
        f.error("ACCEPTANCE_CLAIMED",
                "ledger_state.acceptance_pass_claimed must not be true: a ledger cannot "
                "declare acceptance")


def check_mapping_vs_plan(plan_ids: List[str], f: Findings) -> None:
    for tid in REQUIRED_METRICS:
        if tid not in plan_ids:
            f.error("REQUIREMENTS_DRIFT",
                    f"checker requirement {tid} is not present in the frozen plan §7 matrix")
    for tid in plan_ids:
        if tid not in REQUIRED_METRICS:
            f.error("REQUIREMENTS_DRIFT",
                    f"frozen plan §7 lists {tid} but the checker has no requirement for it")


def emit_text(path: Path, f: Findings, data: Dict[str, Any] | None) -> None:
    print("=" * 78)
    print("C4.3 S4 — W4 acceptance ledger check (structure only; NOT an acceptance PASS)")
    print("=" * 78)
    print(f"ledger        : {path}")
    if data is not None:
        print(f"schema        : {data.get('schema')}")
    print(f"entries       : {f.entries_seen}")
    print(f"metrics       : {f.metrics_seen}")
    print()
    print(f"[待标定] items ({len(f.pending)}) — these are NOT calibrated thresholds:")
    for item in f.pending:
        sug = item.get("suggested_value")
        suffix = f"  (suggested, not a threshold: {sug})" if sug is not None else ""
        print(f"  - {item['test_id']}/{item['metric']}{suffix}")
    if not f.pending:
        print("  (none)")
    print()
    print(f"non-PASS evidence states ({len(f.held)}):")
    for item in f.held:
        print(f"  - {item['test_id']}/{item['metric']}: {item['evidence_state']}")
    if not f.held:
        print("  (none)")
    print()
    print(f"retained historical failures ({len(f.failed)}):")
    for item in f.failed:
        print(f"  - {item['id']}: {item['evidence_state']}")
    print()
    for code, msg in f.warnings:
        print(f"WARN [{code}] {msg}")
    for code, msg in f.errors:
        print(f"ERROR [{code}] {msg}")
    print()
    verdict = "LEDGER_OK" if not f.errors else "LEDGER_RULE_ERROR"
    if f.errors and all(c in ("YAML_UNPARSEABLE", "LEDGER_MISSING", "YAML_ROOT_NOT_MAPPING",
                              "PLAN_MISSING", "PLAN_UNPARSEABLE", "ENTRIES_NOT_LIST") for c, _ in f.errors):
        verdict = "INPUT_ERROR"
    print(f"VERDICT: {verdict} — structure/coverage only; "
          f"this is NOT an S4.1 acceptance PASS")
    print("ACCEPTANCE_PASS_CLAIMED: false")
    print("GATE_STATUS_CHANGED: false")


def main(argv: List[str]) -> int:
    ap = argparse.ArgumentParser(description="Check the C4.3 S4 W4 acceptance ledger (structure only)")
    ap.add_argument("--ledger", type=Path, default=DEFAULT_LEDGER)
    ap.add_argument("--plan", type=Path, default=FROZEN_PLAN)
    ap.add_argument("--json", action="store_true", help="machine readable output")
    ap.add_argument("--list-pending", action="store_true", help="print [待标定] items only")
    ap.add_argument("--print-requirements", action="store_true",
                    help="print the authoritative requirement mapping and exit")
    args = ap.parse_args(argv)

    if args.print_requirements:
        for tid, keys in REQUIRED_METRICS.items():
            print(f"{tid}: {', '.join(keys)}")
        return EXIT_OK

    f = Findings()
    data = load_ledger(args.ledger, f)
    plan_ids = plan_test_ids(args.plan, f)
    if data is not None and plan_ids is not None:
        check_mapping_vs_plan(plan_ids, f)
        check_ledger(data, plan_ids, f)

    if args.list_pending:
        for item in f.pending:
            print(f"{item['test_id']}\t{item['metric']}")
        print(f"# pending={len(f.pending)}")
        return EXIT_RULE if f.errors else EXIT_OK

    code = EXIT_OK
    if f.errors:
        hard = {c for c, _ in f.errors}
        code = EXIT_INPUT if hard <= {"YAML_UNPARSEABLE", "LEDGER_MISSING", "YAML_ROOT_NOT_MAPPING",
                                      "PLAN_MISSING", "PLAN_UNPARSEABLE", "ENTRIES_NOT_LIST",
                                      "ENTRY_NOT_MAPPING", "METRICS_NOT_LIST"} else EXIT_RULE

    verdict_by_code = {EXIT_OK: "LEDGER_OK", EXIT_INPUT: "INPUT_ERROR", EXIT_RULE: "LEDGER_RULE_ERROR"}
    if args.json:
        print(json.dumps({
            "schema": "c43s4.w4a.ledger_check.v1",
            "ledger": str(args.ledger),
            "entries": f.entries_seen,
            "metrics": f.metrics_seen,
            "pending": f.pending,
            "non_pass": f.held,
            "historical_failures": f.failed,
            "errors": [{"code": c, "message": m} for c, m in f.errors],
            "warnings": [{"code": c, "message": m} for c, m in f.warnings],
            "verdict": verdict_by_code[code],
            "acceptance_pass_claimed": False,
        }, ensure_ascii=False, indent=2))
    else:
        emit_text(args.ledger, f, data)

    return code


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
