#!/usr/bin/env bash
# C4.2 — reproduce the WHOLE stage from scratch, S1 through S7.
#
#   bash src/dual_arm_pose_optimization/analysis/run_c42_all.sh
#
# Order matters: S1 (scene) -> S2 (IK + Baseline B) -> S3 (candidate table)
# -> S4 (objectives) -> S5 (refinement/zones) -> S6 (comparison/ablation)
# -> S7 (overview).  Each stage is also runnable on its own.
#
# Environment overrides:
#   SKIP_BUILD=1     do not rebuild
#   SKIP_HEAVY=1     skip the S3 sweep if the candidate table already exists
set -uo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$(cd "$HERE/../../.." && pwd)"
PKG="dual_arm_pose_optimization"
OUT="$HERE/figs"
C41="${C41_BIN:-$WS/src/dual_arm_workspace_analysis/analysis/data/workspace_dual_arm.bin}"
PY="${PY:-$HOME/miniconda3/envs/yolov8/bin/python}"

echo "================================================================"
echo " C4.2 full reproduction  (S1 -> S7)"
echo " workspace : $WS"
echo " c41 bin   : $C41"
echo "================================================================"

if [ ! -f "$C41" ]; then
  echo "ERROR: C4.1 workspace records not found."
  echo "       run: bash src/dual_arm_workspace_analysis/analysis/run_c41.sh"
  exit 1
fi

set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
if [ "${SKIP_BUILD:-0}" != "1" ]; then
  colcon build --merge-install --symlink-install \
    --cmake-args "-DPython3_EXECUTABLE=$(which python3)" \
    --packages-select "$PKG" || exit 1
fi
# shellcheck disable=SC1091
source "$WS/install/setup.bash"
set -u

rc=0
step() { echo; echo "######## $* ########"; }

step "S1  scene / ACM / distance scopes"
SKIP_BUILD=1 bash "$HERE/run_c42_s1.sh" >/tmp/c42all_s1.log 2>&1 \
  && grep -E "^  ---> " /tmp/c42all_s1.log | sed 's/^/  /' || { echo "S1 FAILED"; rc=1; }

step "S2  analytic IK + Baseline B rebuild"
SKIP_BUILD=1 bash "$HERE/run_c42_s2.sh" >/tmp/c42all_s2.log 2>&1 \
  && grep -E "^  ---> " /tmp/c42all_s2.log | sed 's/^/  /' || { echo "S2 FAILED"; rc=1; }

step "S3  layered reachability S0 -> S4  (the expensive sweep, ~2 min)"
if [ "${SKIP_HEAVY:-0}" = "1" ] && [ -f "$OUT/c42_s3_candidates.csv" ]; then
  echo "  skipped (SKIP_HEAVY=1 and candidate table present)"
else
  SKIP_BUILD=1 bash "$HERE/run_c42_s3.sh" >/tmp/c42all_s3.log 2>&1 \
    && grep -E "PASS|FAIL|---> " /tmp/c42all_s3.log | sed 's/^/  /' || { echo "S3 FAILED"; rc=1; }
fi

step "S4  normalised objectives / poles / Pareto"
"$PY" "$HERE/optimize_c42.py" --csv "$OUT/c42_s3_candidates.csv" --out-dir "$OUT" \
  > /tmp/c42all_s4.log 2>&1 \
  && grep -E "^\[s4\]" /tmp/c42all_s4.log | sed 's/^/  /' || { echo "S4 FAILED"; rc=1; }

step "S5  V0/V1 refinement + standby"
SCALES="$("$PY" - "$OUT/c42_s4_objectives.json" <<'PYEOF'
import json, sys
d = json.load(open(sys.argv[1]))
print(",".join(str(o["scale_value"]) for o in d["objectives"]))
PYEOF
)"
"$WS/build/$PKG/c42_optimize_pose" --urdf "$WS/src/dual_arm_kinematics/test/fixtures/s622_dual_arm.urdf" \
  --srdf "$WS/src/s622_moveit_config/config/s622_dual_arm.srdf" \
  --c41-bin "$C41" --csv "$OUT/c42_s3_candidates.csv" --weights equal --scales "$SCALES" \
  --standby-n 6000 --multi-seed 20 --json "$OUT/c42_s5_optimization.json" \
  > /tmp/c42all_s5.log 2>&1 \
  && grep -E "PASS|FAIL|---> " /tmp/c42all_s5.log | sed 's/^/  /' || { echo "S5 FAILED"; rc=1; }
"$PY" "$HERE/zones_c42.py" --csv "$OUT/c42_s3_candidates.csv" --c41-bin "$C41" \
  --s5-json "$OUT/c42_s5_optimization.json" --out-dir "$OUT" \
  > /tmp/c42all_zones.log 2>&1 \
  && grep -E "^\[s5\]" /tmp/c42all_zones.log | sed 's/^/  /' || { echo "zones FAILED"; rc=1; }

step "S6  baseline comparison / ablation / sensitivity / multi-seed"
bash "$HERE/run_c42_s6.sh" >/tmp/c42all_s6.log 2>&1 \
  && grep -E "^\[s6\]" /tmp/c42all_s6.log | sed 's/^/  /' || { echo "S6 FAILED"; rc=1; }

step "S7  overview figure + summary"
"$PY" "$HERE/overview_c42.py" --figs "$OUT" --s6 "$OUT/c42_s6_report.json" \
  --out-dir "$OUT" || { echo "S7 FAILED"; rc=1; }

step "gtest (whole package)"
colcon test --merge-install --packages-select "$PKG" --event-handlers console_direct+ \
  >/tmp/c42all_gtest.log 2>&1
colcon test-result --test-result-base "$WS/build/$PKG" | tail -2
[ $? -eq 0 ] || rc=1

echo
if [ "$rc" -eq 0 ]; then
  echo "================ C4.2 FULL REPRODUCTION OK ================"
else
  echo "================ C4.2 FAILED (rc=$rc) ================"
fi
echo "figures: $OUT/fig5_pareto.png fig6_cooperative_zone.png fig7_sensitivity.png fig8_overview.png"
echo "summary: $OUT/c42_summary.json"
exit "$rc"
