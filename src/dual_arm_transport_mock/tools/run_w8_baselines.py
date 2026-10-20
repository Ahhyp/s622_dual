#!/usr/bin/env python3
"""C4.3 S3 -- W8: run the baseline scenario group and summarise it.

Each scenario gets its own artifact directory (one directory == one run, see W7),
and this script adds the things that only make sense across scenarios:

    <out>/<scenario>/...   the full §8 artifact set, written by the CLI
    <out>/runs_all.csv     every scenario's runs.csv, one header
    <out>/compare.md       the trade-off table (this is what §6 W8 asks for)
    <out>/manifest.json    what was run, by which binary, at which commit

The scenario matrix is the one §6 W8 specifies: the control arm (object task off),
a w_obj sweep, and feed-forward on/off.  It is data, not code, so adding a point
does not require touching the loop.

Usage:
    run_w8_baselines.py --out runs/c43_s3/w8
    run_w8_baselines.py --out /tmp/x --sweep-samples 4 --jobs 3 --only control,ff_w1
"""

from __future__ import annotations

import argparse
import concurrent.futures
import csv
import json
import os
import subprocess
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path


ARTIFACTS = ("cycles.csv", "cycles.jsonl", "sweeps.csv", "clearance.csv", "runs.csv",
             "summary.json")


@dataclass
class Scenario:
    name: str
    args: list[str] = field(default_factory=list)
    note: str = ""


def baseline_matrix(w_obj: float) -> list[Scenario]:
    """§6 W8: the control arm, a w_obj sweep, and v_ff on/off.

    `w_obj=0` is deliberately absent: with the object task off the object-term is
    skipped inside the QP (S3-T5b proves the two are bit-identical), so it would be
    a duplicate of `control` with more machinery.
    """
    out = [Scenario("control", ["--no-object-task"],
                    "relative task only; the object never leaves its start pose")]
    for w in w_obj:
        for ff in (True, False):
            tag = f"ff_{_tag(w)}" if ff else f"noff_{_tag(w)}"
            args = ["--w-obj", repr(w)]
            if not ff:
                args += ["--no-ff"]
            out.append(Scenario(tag, args, f"w_obj={w:g}, feed-forward {'on' if ff else 'off'}"))
    return out


def _tag(w: float) -> str:
    return f"{w:g}".replace(".", "p")


def find_cli(explicit: str | None) -> Path:
    if explicit:
        p = Path(explicit)
        if not p.is_file():
            sys.exit(f"error: --cli does not exist: {p}")
        return p.resolve()
    # Prefer the install prefix (what a user would run), then the raw build tree.
    candidates = [
        Path("install/dual_arm_transport_mock/lib/dual_arm_transport_mock/c43_s3_mock_main"),
        Path("build/dual_arm_transport_mock/c43_s3_mock_main"),
    ]
    for c in candidates:
        if c.is_file():
            return c.resolve()
    sys.exit("error: cannot find c43_s3_mock_main; pass --cli")


def run_one(cli: Path, scen: Scenario, out: Path, common: list[str], env: dict,
            timeout: float) -> tuple[Scenario, int, float, str]:
    out_dir = out / scen.name
    out_dir.mkdir(parents=True, exist_ok=True)
    cmd = [str(cli), "--out", str(out_dir), "--scenario", scen.name] + common + scen.args
    log = out / f"{scen.name}.log"
    t0 = time.time()
    with log.open("w") as fh:
        fh.write(" ".join(cmd) + "\n\n")
        fh.flush()
        try:
            rc = subprocess.call(cmd, stdout=fh, stderr=subprocess.STDOUT, env=env,
                                 timeout=timeout)
        except subprocess.TimeoutExpired:
            return scen, 124, time.time() - t0, f"timeout after {timeout:.0f} s"
    return scen, rc, time.time() - t0, log.read_text()


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", required=True)
    ap.add_argument("--cli")
    ap.add_argument("--jobs", type=int, default=1)
    ap.add_argument("--sweep-samples", type=int, default=8)
    ap.add_argument("--w-obj", type=float, nargs="+", default=[0.1, 1.0, 10.0])
    ap.add_argument("--min-clearance", type=float, default=None)
    ap.add_argument("--trajectory", default=None)
    ap.add_argument("--only", default=None, help="comma-separated scenario names")
    ap.add_argument("--timeout", type=float, default=1800.0)
    args = ap.parse_args()

    cli = find_cli(args.cli)
    out = Path(args.out).resolve()
    out.mkdir(parents=True, exist_ok=True)

    scenarios = baseline_matrix(args.w_obj)
    if args.only:
        wanted = {s.strip() for s in args.only.split(",") if s.strip()}
        unknown = wanted - {s.name for s in scenarios}
        if unknown:
            sys.exit(f"error: unknown scenario(s): {', '.join(sorted(unknown))}")
        scenarios = [s for s in scenarios if s.name in wanted]

    common = ["--sweep-samples", str(args.sweep_samples)]
    if args.min_clearance is not None:
        common += ["--min-clearance", repr(args.min_clearance)]
    if args.trajectory:
        common += ["--trajectory", args.trajectory]

    env = dict(os.environ)
    lib = cli.parent
    for extra in (Path("install/lib"), Path("install/dual_arm_transport_mock/lib")):
        if extra.is_dir():
            env["LD_LIBRARY_PATH"] = f"{extra.resolve()}:{env.get('LD_LIBRARY_PATH', '')}"
    env.setdefault("LD_LIBRARY_PATH", str(lib))

    print(f"cli      : {cli}")
    print(f"out      : {out}")
    print(f"scenarios: {len(scenarios)} (jobs={args.jobs}, sweep_samples={args.sweep_samples})")

    results: list[tuple[Scenario, int, float, str]] = []
    if args.jobs > 1:
        with concurrent.futures.ThreadPoolExecutor(max_workers=args.jobs) as pool:
            futures = [pool.submit(run_one, cli, s, out, common, env, args.timeout)
                       for s in scenarios]
            for fut in concurrent.futures.as_completed(futures):
                r = fut.result()
                results.append(r)
                print(f"  [{r[0].name}] rc={r[1]} {r[2]:.0f} s")
    else:
        for s in scenarios:
            r = run_one(cli, s, out, common, env, args.timeout)
            results.append(r)
            print(f"  [{r[0].name}] rc={r[1]} {r[2]:.0f} s")

    results.sort(key=lambda r: [s.name for s in scenarios].index(r[0].name))

    # --- artifact completeness: a missing file is a broken run, not a warning ---
    bad = []
    for scen, rc, _, log in results:
        missing = [a for a in ARTIFACTS if not (out / scen.name / a).is_file()]
        if missing:
            bad.append(f"{scen.name}: missing {', '.join(missing)}")
        elif (out / scen.name / "runs.csv").stat().st_size < 2:
            bad.append(f"{scen.name}: empty runs.csv")
        if rc not in (0, 1):
            bad.append(f"{scen.name}: exited {rc}\n{log[-2000:]}")

    # --- runs_all.csv ------------------------------------------------------
    header = None
    merged: list[list[str]] = []
    for scen, _, _, _ in results:
        p = out / scen.name / "runs.csv"
        if not p.is_file():
            continue
        with p.open(newline="") as fh:
            rows = list(csv.reader(fh))
        if not rows:
            continue
        header = header or rows[0]
        merged.extend(rows[1:])
    if header:
        with (out / "runs_all.csv").open("w", newline="") as fh:
            w = csv.writer(fh)
            w.writerow(header)
            w.writerows(merged)

    # --- manifest.json -----------------------------------------------------
    commit = subprocess.run(["git", "rev-parse", "HEAD"], capture_output=True, text=True)
    manifest = {
        "commit": commit.stdout.strip() or "unknown",
        "cli": str(cli),
        "sweep_samples": args.sweep_samples,
        "scenarios": [{"name": s.name, "args": s.args, "note": s.note,
                       "exit_code": next(r[1] for r in results if r[0] is s),
                       "wall_seconds": round(next(r[2] for r in results if r[0] is s), 1)}
                      for s in scenarios],
    }
    (out / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")

    # --- compare.md --------------------------------------------------------
    def nums(rec: dict, *keys, default=float("nan")):
        for k in keys:
            if k in rec and rec[k] not in ("", None):
                try:
                    return float(rec[k])
                except ValueError:
                    return default
        return default

    lines = ["# C4.3 S3 — W8 baseline group", "",
             f"commit `{manifest['commit'][:12]}`, sweep_samples={args.sweep_samples}, "
             f"dt from the frozen trajectory, {len(scenarios)} scenarios", "",
             "| scenario | ok | failure | terminal pos [m] | terminal rot [rad] | "
             "max e_rel | max e_obj | max s_obj | committed | vetoed | note |",
             "|---|---|---|---|---|---|---|---|---|---|---|"]
    control_terminal = None
    for scen, _, _, _ in results:
        p = out / scen.name / "summary.json"
        rec = {}
        if p.is_file():
            sj = json.loads(p.read_text())
            runs = sj.get("runs") or []
            if runs:
                rec = runs[0].get("summary", runs[0])
        tp = nums(rec, "terminal_pos_err_m")
        if scen.name == "control":
            control_terminal = tp
        lines.append(
            f"| `{scen.name}` | {'✓' if rec.get('success') else '✗'} | "
            f"`{rec.get('failure_code', '?')}` | {tp:.3e} | "
            f"{nums(rec, 'terminal_rot_err_rad'):.3e} | "
            f"{nums(rec, 'max_e_rel_norm'):.3e} | "
            f"{nums(rec, 'max_e_obj_norm'):.3e} | "
            f"{nums(rec, 'max_s_obj_norm'):.3e} | "
            f"{int(nums(rec, 'committed_cycles', default=0))} | "
            f"{int(nums(rec, 'vetoed_cycles', default=0))} | {scen.note} |")
    if control_terminal is not None and control_terminal == control_terminal:
        lines += ["",
                  f"The control arm ends {control_terminal:.3e} m from the target — "
                  "that is the distance the object was asked to travel. Relative-only "
                  "control holds the grasp but transports nothing, which is exactly why "
                  "S3 needs the object task."]
    (out / "compare.md").write_text("\n".join(lines) + "\n")

    print()
    print("\n".join(lines[4:]))
    if bad:
        print()
        for b in bad:
            print(f"ERROR {b}", file=sys.stderr)
        return 2
    failed = [s.name for s, rc, _, _ in results if rc != 0]
    if failed:
        print(f"\nnote: scenarios that did not reach 'success': {', '.join(failed)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
