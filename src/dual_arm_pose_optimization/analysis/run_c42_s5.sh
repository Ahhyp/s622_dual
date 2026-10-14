#!/usr/bin/env bash
# C4.2 / S5 one-command reproduction:
#   V0 / V1 pose refinement + theta_max sensitivity + standby search + zones/plot.
#
#   bash src/dual_arm_pose_optimization/analysis/run_c42_s5.sh
#
# Environment overrides:
#   SKIP_BUILD=1   skip colcon build
#   WEIGHTS=equal  equal | precision_first | safety_first
#   STANDBY_N=6000 standby pool size
set -uo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$(cd "$HERE/../../.." && pwd)"
PKG="dual_arm_pose_optimization"
OUT_DIR="$HERE/figs"
CSV="$OUT_DIR/c42_s3_candidates.csv"
C41="${C41_BIN:-$WS/src/dual_arm_workspace_analysis/analysis/data/workspace_dual_arm.bin}"
PY="${PY:-$HOME/miniconda3/envs/yolov8/bin/python}"
WEIGHTS="${WEIGHTS:-equal}"
STANDBY_N="${STANDBY_N:-6000}"

echo "== C4.2 / S5 ====================================================="

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
if [ ! -f "$CSV" ]; then
  echo "### S3 candidate table missing -> regenerating"
  SKIP_BUILD=1 bash "$HERE/run_c42_s3.sh" || exit 1
fi
if [ ! -f "$OUT_DIR/c42_s4_objectives.json" ]; then
  echo "### S4 objectives missing -> regenerating"
  "$PY" "$HERE/optimize_c42.py" --csv "$CSV" --out-dir "$OUT_DIR" || exit 1
fi

# the S4 analysis owns the normalisation scales; pass them straight through
SCALES="$("$PY" - "$OUT_DIR/c42_s4_objectives.json" <<'PYEOF'
import json, sys
d = json.load(open(sys.argv[1]))
print(",".join(str(o["scale_value"]) for o in d["objectives"]))
PYEOF
)"
echo "scales: $SCALES"

echo
echo "### 1/3 S5 refinement (V0 / V1 / theta sensitivity / standby) --------"
"$WS/build/$PKG/c42_optimize_pose" --urdf "$WS/src/dual_arm_kinematics/test/fixtures/s622_dual_arm.urdf" \
  --srdf "$WS/src/s622_moveit_config/config/s622_dual_arm.srdf" \
  --c41-bin "$C41" --csv "$CSV" --weights "$WEIGHTS" --scales "$SCALES" \
  --standby-n "$STANDBY_N" \
  --json "$OUT_DIR/c42_s5_optimization.json"
[ $? -eq 0 ] || rc=1

echo
echo "### 2/3 cooperative zone + overlay figure ----------------------------"
"$PY" "$HERE/zones_c42.py" --csv "$CSV" --c41-bin "$C41" \
  --s5-json "$OUT_DIR/c42_s5_optimization.json" --out-dir "$OUT_DIR"
[ $? -eq 0 ] || rc=1

echo
echo "### 3/3 gtest --------------------------------------------------------"
colcon test --merge-install --packages-select "$PKG" --event-handlers console_direct+ \
  >/tmp/c42_s5_gtest.log 2>&1
TEST_RC=$?
colcon test-result --test-result-base "$WS/build/$PKG" | tail -2
[ "$TEST_RC" -eq 0 ] || rc=1

echo
if [ "$rc" -eq 0 ]; then echo "=== S5 OK ==="; else echo "=== S5 FAILED (rc=$rc) ==="; fi
