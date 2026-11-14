#!/usr/bin/env python3
"""Lateral load calibration for the left_finger1 joint force/torque sensor.

Uses the native Gazebo topic (no ROS bridge), the world-level persistent wrench
topic (ApplyLinkWrench only applies for one step on /wrench), and /wrench/clear
between load levels because persistent wrenches accumulate.

    python3 ft_probe.py            # baseline -> 2 N -> clear -> 5 N -> clear

Everything goes through subprocess argument lists: no shell quoting.
"""
import math
import re
import subprocess
import sys
import time
from statistics import mean

FT_TOPIC = "/left_finger1_joint_ft"
WRENCH_TOPIC = "/world/c43_s4_world/wrench/persistent"
CLEAR_TOPIC = "/world/c43_s4_world/wrench/clear"
LINK = "s622_dual_arm::left_finger1"
ENTITY = f'name: "{LINK}" type: LINK'

FORCE_RE = re.compile(r"\bforce\s*\{([^{}]*)\}")
AXIS_RE = re.compile(r"\b([xyz]):\s*(\S+)")


def sample(n=4):
    result = subprocess.run(
        ["ign", "topic", "-e", "-t", FT_TOPIC, "-n", str(n)],
        capture_output=True, text=True, timeout=8, check=True,
    )
    rows = []
    for block in FORCE_RE.findall(result.stdout):
        components = dict(AXIS_RE.findall(block))
        # Protobuf text may omit components whose value is zero.
        row = tuple(float(components.get(axis, "0")) for axis in "xyz")
        if not all(math.isfinite(value) for value in row):
            raise ValueError(f"non-finite force from {FT_TOPIC}: {row}")
        rows.append(row)
    if len(rows) != n:
        raise RuntimeError(f"expected {n} force samples from {FT_TOPIC}, got {len(rows)}")
    return rows


def apply(force_z):
    msg = f"entity {{ {ENTITY} }} wrench {{ force {{ z: {force_z} }} }}"
    subprocess.run(["ign", "topic", "-t", WRENCH_TOPIC, "-m",
                    "ignition.msgs.EntityWrench", "-p", msg], timeout=10, check=True)


def clear():
    subprocess.run(["ign", "topic", "-t", CLEAR_TOPIC, "-m",
                    "ignition.msgs.Entity", "-p", ENTITY], timeout=10, check=True)


def report(label, rows, baseline=None):
    print(f"--- {label} (n={len(rows)}) ---")
    for r in rows:
        print(f"    fx={r[0]:+.6f}  fy={r[1]:+.6f}  fz={r[2]:+.6f}")
    averages = tuple(mean(axis) for axis in zip(*rows))
    if baseline is not None:
        delta = tuple(value - zero for value, zero in zip(averages, baseline))
        print(f"    delta=({delta[0]:+.6f}, {delta[1]:+.6f}, {delta[2]:+.6f}) N"
              f"  |delta|={math.hypot(*delta):.6f} N")
    return averages


def main():
    clear()
    time.sleep(0.5)
    baseline = report("baseline", sample())

    for load in (2, 5):
        try:
            apply(-load)
            time.sleep(1.0)
            report(f"persistent {load} N (world -z)", sample(), baseline)
        finally:
            clear()
        time.sleep(1.0)
        report(f"cleared after {load} N", sample(), baseline)


if __name__ == "__main__":
    try:
        main()
    except (OSError, RuntimeError, ValueError, subprocess.SubprocessError) as exc:
        print(f"FT probe failed: {exc}", file=sys.stderr)
        if getattr(exc, "stderr", None):
            print(exc.stderr, file=sys.stderr)
        raise SystemExit(1)
