#!/usr/bin/env bash
# C4.2 / S6 one-command reproduction:
#   baseline-B comparison (plan §7.1), objective saturation, ablation,
#   weight sensitivity (+-50%), multi-seed statistics, scale variant.
#
#   bash src/dual_arm_pose_optimization/analysis/run_c42_s6.sh
set -uo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$(cd "$HERE/../../.." && pwd)"
PKG="dual_arm_pose_optimization"
OUT="$HERE/figs"
CSV="$OUT/c42_s3_candidates.csv"
C41="${C41_BIN:-$WS/src/dual_arm_workspace_analysis/analysis/data/workspace_dual_arm.bin}"
PY="${PY:-$HOME/miniconda3/envs/yolov8/bin/python}"

echo "== C4.2 / S6 ====================================================="

set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
set -u
if [ "${SKIP_BUILD:-0}" != "1" ]; then
  colcon build --merge-install --symlink-install \
    --cmake-args "-DPython3_EXECUTABLE=$(which python3)" \
    --packages-select "$PKG" || exit 1
fi
set +u
# shellcheck disable=SC1091
source "$WS/install/setup.bash"
set -u

rc=0
for f in "$CSV" "$OUT/c42_s4_objectives.json"; do
  if [ ! -f "$f" ]; then
    echo "missing $f -> run run_c42_s4.sh first"; exit 1
  fi
done

SCALES="$("$PY" - "$OUT/c42_s4_objectives.json" <<'PYEOF'
import json, sys
d = json.load(open(sys.argv[1]))
print(",".join(str(o["scale_value"]) for o in d["objectives"]))
PYEOF
)"
# P95 scales for the objectives whose "fixed" reference was arbitrary
SCALES_P95="$("$PY" - "$CSV" <<'PYEOF'
import csv, sys, numpy as np
rows = [r for r in csv.DictReader(open(sys.argv[1])) if r["s4"] == "1"]
cols = ["smin_left","smin_right","w_rel","mesh_arm_arm","joint_margin",
        "deviation","continuity","arm_proximal"]
M = np.array([[float(r[c]) for c in cols] for r in rows])
s = [float(np.percentile(M[:, i], 95)) for i in range(8)]
s[6] = 1.0
print(",".join("%.6f" % v for v in s))
PYEOF
)"

echo
echo "### 1/3 S5+S6 run with the declared (fixed) scales -------------------"
"$WS/build/$PKG/c42_optimize_pose" --urdf "$WS/src/dual_arm_kinematics/test/fixtures/s622_dual_arm.urdf" \
  --srdf "$WS/src/s622_moveit_config/config/s622_dual_arm.srdf" \
  --c41-bin "$C41" --csv "$CSV" --weights equal --scales "$SCALES" \
  --standby-n 6000 --multi-seed 20 \
  --json "$OUT/c42_s5_optimization.json" || rc=1

echo
echo "### 2/3 scale-variant run (P95 scales) ------------------------------"
echo "    P95 scales: $SCALES_P95"
"$WS/build/$PKG/c42_optimize_pose" --urdf "$WS/src/dual_arm_kinematics/test/fixtures/s622_dual_arm.urdf" \
  --srdf "$WS/src/s622_moveit_config/config/s622_dual_arm.srdf" \
  --c41-bin "$C41" --csv "$CSV" --weights equal --scales "$SCALES_P95" \
  --standby-n 2000 --multi-seed 10 \
  --json "$OUT/c42_s6_scale_variant.json" || rc=1

echo
echo "### 3/3 S6 report (ablation / sensitivity / multi-seed) -------------"
"$PY" "$HERE/report_c42_s6.py" --csv "$CSV" \
  --s4-json "$OUT/c42_s4_objectives.json" \
  --s5-json "$OUT/c42_s5_optimization.json" \
  --scale-variant-json "$OUT/c42_s6_scale_variant.json" \
  --out-dir "$OUT" || rc=1

echo
if [ "$rc" -eq 0 ]; then echo "=== S6 OK ==="; else echo "=== S6 FAILED (rc=$rc) ==="; fi
