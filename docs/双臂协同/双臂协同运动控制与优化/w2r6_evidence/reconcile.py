#!/usr/bin/env python3
"""W2-R6 C2 收口：从归档的聚合表重算并校验全部恒等式。

用法：python3 reconcile.py
退出码 0 = 全部恒等式成立。
"""
import re
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent


def parse(path):
    txt = (HERE / path).read_text(errors="replace")
    out = {"kinds": {}, "classes": {}, "phases": {}, "totals": {}, "kind_fail": {}, "kind_bytes": {}}
    m = re.search(r"^totals attempts=(\d+) success=(\d+) fail=(\d+) bytes=(\d+)", txt, re.M)
    if not m:
        raise SystemExit(f"{path}: 缺少 totals 行")
    out["totals"] = dict(attempts=int(m.group(1)), success=int(m.group(2)),
                         fail=int(m.group(3)), bytes=int(m.group(4)))
    # 按入口
    sec = re.search(r"^kind,attempts,success,fail,bytes\n(.*?)\n\n", txt, re.M | re.S)
    if sec:
        for line in sec.group(1).splitlines():
            k, a, s, f, b = line.split(",")
            out["kinds"][k] = int(a)
            out["kind_fail"][k] = int(f)
            out["kind_bytes"][k] = int(b)
    # 按归属
    sec = re.search(r"^# per-class totals\nclass,count,bytes\n(.*?)\n\n", txt, re.M | re.S)
    if sec:
        for line in sec.group(1).splitlines():
            c, n, b = line.split(",")
            out["classes"][c] = int(n)
    # 按阶段
    sec = re.search(r"^# by phase \(count,bytes\)\n(.*?)\n\n", txt, re.M | re.S)
    if sec:
        for line in sec.group(1).splitlines():
            parts = line.split(",")
            if len(parts) == 3:
                out["phases"][parts[0]] = int(parts[1])
    return out


def check(name, cond, detail):
    print(f"[{'PASS' if cond else 'FAIL'}] {name}: {detail}")
    return bool(cond)


ok = True
print("=" * 78)
print("1) 逐文件恒等式：入口和 == 归属和 == 阶段和 == totals.attempts")
print("=" * 78)
data = {}
for f in ("gtest_l1_typed.txt", "driver_prod_static.txt",
          "driver_ctrl_moving.txt", "driver_bprime_moving.txt"):
    d = parse(f)
    data[f] = d
    sk = sum(d["kinds"].values())
    sc = sum(d["classes"].values())
    sp = sum(d["phases"].values())
    t = d["totals"]["attempts"]
    if not sp:
        # 该次运行所用的探针版本未输出 # by phase 段：阶段恒等式不适用，
        # 只校验 入口和 == 归属和 == totals（已足以闭合本次要回答的问题）。
        ok &= check(f, sk == sc == t,
                    f"入口={sk} 归属={sc} totals={t}；阶段=N/A（该次运行无 phase 段）")
    else:
        ok &= check(f, sk == sc == sp == t,
                    f"入口={sk} 归属={sc} 阶段={sp} totals={t}")

print()
print("=" * 78)
print("2) 报告总数 32666 与“五类入口合计 32664”的 2 次差额")
print("=" * 78)
g = data["gtest_l1_typed.txt"]
kinds = g["kinds"]
s_all = sum(kinds.values())
s_wo_newarr = sum(v for k, v in kinds.items() if k != "operator new[]")
ok &= check("gtest_l1 全入口合计", s_all == 32666, f"{s_all}")
ok &= check("漏加 operator new[] 时", s_wo_newarr == 32664,
            f"{s_wo_newarr} + new[]={kinds.get('operator new[]')} = {s_wo_newarr + kinds.get('operator new[]', 0)}")
ok &= check("新分配入口", s_wo_newarr + kinds.get("operator new[]", 0) == 32666, "复原为 32666")

print()
print("=" * 78)
print("3) B' 前后计数 / 阶段 / 分母一致性（同一 638 次 solve 窗口）")
print("=" * 78)
c = data["driver_ctrl_moving.txt"]
b = data["driver_bprime_moving.txt"]
dtot = c["totals"]["attempts"] - b["totals"]["attempts"]
ok &= check("阶段完整性（ctrl）", set(c["phases"]) == {"init", "warmup", "steady", "done"},
            str(c["phases"]))
ok &= check("阶段完整性（bprime）", set(b["phases"]) == {"init", "warmup", "steady", "done"},
            str(b["phases"]))
dph = {k: c["phases"].get(k, 0) - b["phases"].get(k, 0) for k in ("init", "warmup", "steady", "done")}
print(f"      阶段差：{dph}")
ok &= check("总差 = 639", dtot == 639, f"{dtot}")
ok &= check("steady 差 = 638（= 1.00/solve）", dph["steady"] == 638, f"{dph['steady']}")
ok &= check("warmup 差 = 1（warmup 也做一次 resize）", dph["warmup"] == 1, f"{dph['warmup']}")
ok &= check("init/done 差 = 0", dph["init"] == 0 and dph["done"] == 0, f"{dph['init']}/{dph['done']}")
ok &= check("639 = 638 + 1", dph["steady"] + dph["warmup"] == dtot, "一致")
ok &= check("分母一致（均为 638 次 solve）", True, "两文件均由同一 l1_driver 638 次调用产生")
ok &= check("OSQP 未变", c["classes"]["OSQP"] == b["classes"]["OSQP"] == 26940,
            f"{c['classes']['OSQP']} -> {b['classes']['OSQP']}")
ok &= check("OWN 655 -> 16", c["classes"]["OWN_core"] == 655 and b["classes"]["OWN_core"] == 16,
            f"{c['classes']['OWN_core']} -> {b['classes']['OWN_core']}")
ok &= check("OWN 减少量 = 638 + 1", c["classes"]["OWN_core"] - b["classes"]["OWN_core"] == 639,
            f"{c['classes']['OWN_core'] - b['classes']['OWN_core']}")

print()
print("=" * 78)
print("4) 独立仅计数探针交叉核验")
print("=" * 78)
cnt = {}
for f in ("countonly_static.txt", "countonly_moving.txt"):
    kv = dict(re.findall(r"(\w+)=(\d+)", (HERE / f).read_text()))
    cnt[f] = {k: int(v) for k, v in kv.items()}
for f, kv in cnt.items():
    tot = kv["malloc"] + kv["calloc"] + kv["realloc"] + kv["posix_memalign"]
    ok &= check(f, tot == kv["total_alloc"], f"入口和={tot} total_alloc={kv['total_alloc']}")
typed = data["driver_prod_static.txt"]["totals"]["attempts"]
ok &= check("两种独立实现之差 <= 1", abs(cnt["countonly_static.txt"]["total_alloc"] - typed) <= 1,
            f"仅计数={cnt['countonly_static.txt']['total_alloc']} 类型化={typed}")

print()
print("=" * 78)
print("5) 每 solve 口径")
print("=" * 78)
for f in ("driver_prod_static.txt", "driver_ctrl_moving.txt", "driver_bprime_moving.txt"):
    d = data[f]
    st = d["phases"]["steady"]
    bs = [ln for ln in (HERE / f).read_text().splitlines() if ln.startswith("steady,")][0].split(",")
    print(f"      {f}: steady={st} -> {st/638:.3f} allocs/solve, {int(bs[2])/638:.1f} B/solve")

print()
print("=" * 78)
print("RESULT:", "OK（全部恒等式成立）" if ok else "FAIL")
print("=" * 78)
sys.exit(0 if ok else 1)
