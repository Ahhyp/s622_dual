#!/usr/bin/env bash
# C4.2 / S3 one-command reproduction: the layered reachability sweep S0 -> S4,
# plus the S1/S2 regressions.
#
#   bash src/dual_arm_pose_optimization/analysis/run_c42_s3.sh
#
# Environment overrides:
#   SKIP_BUILD=1        skip colcon build
#   P1=60000            handover-pool size (drives BOTH P1a and P1b)
#   GRID_STEP=0.03      P3 grid spacing [m]
#   C41_BIN=...         C4.1 record file
set -uo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$(cd "$HERE/../../.." && pwd)"
PKG="dual_arm_pose_optimization"
OUT_DIR="$HERE/figs"
mkdir -p "$OUT_DIR"

URDF="${URDF:-$WS/src/dual_arm_kinematics/test/fixtures/s622_dual_arm.urdf}"
SRDF="${SRDF:-$WS/src/s622_moveit_config/config/s622_dual_arm.srdf}"
C41_BIN="${C41_BIN:-$WS/src/dual_arm_workspace_analysis/analysis/data/workspace_dual_arm.bin}"
P1="${P1:-60000}"
GRID_STEP="${GRID_STEP:-0.03}"

echo "== C4.2 / S3 ====================================================="
echo " c41 bin   : $C41_BIN"
echo " P1 size   : $P1    grid step: $GRID_STEP m"
echo "=================================================================="

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

if [ ! -f "$C41_BIN" ]; then
  echo "missing C4.1 records: $C41_BIN"
  echo "run: bash src/dual_arm_workspace_analysis/analysis/run_c41.sh"
  exit 1
fi

rc=0

echo
echo "### 1/4 S3 reachability sweep ---------------------------------------"
"$WS/build/$PKG/c42_reachability" --urdf "$URDF" --srdf "$SRDF" --c41-bin "$C41_BIN" \
  --p1 "$P1" --grid-step "$GRID_STEP" \
  --json "$OUT_DIR/c42_s3_reachability.json" \
  --csv "$OUT_DIR/c42_s3_candidates.csv"
[ $? -eq 0 ] || rc=1

echo
echo "### 2/4 S2 regression (Baseline B) ----------------------------------"
"$WS/build/$PKG/c42_baseline_probe" --urdf "$URDF" --srdf "$SRDF" >/tmp/c42_s3_s2.log 2>&1
S2_RC=$?
grep -E "^  ---> " /tmp/c42_s3_s2.log | sed 's/^/  /'
[ "$S2_RC" -eq 0 ] || { echo "  S2 regression FAILED"; rc=1; }

echo
echo "### 3/4 S1 regression (scene / ACM / distances) ---------------------"
"$WS/build/$PKG/c42_scene_probe" --urdf "$URDF" --srdf "$SRDF" >/tmp/c42_s3_s1.log 2>&1
S1_RC=$?
grep -E "^  ---> " /tmp/c42_s3_s1.log | sed 's/^/  /'
[ "$S1_RC" -eq 0 ] || { echo "  S1 regression FAILED"; rc=1; }

echo
echo "### 4/4 gtest --------------------------------------------------------"
export C42_C41_BIN="$C41_BIN"
colcon test --merge-install --packages-select "$PKG" --event-handlers console_direct+ \
  >/tmp/c42_s3_gtest.log 2>&1
TEST_RC=$?
colcon test-result --test-result-base "$WS/build/$PKG" | tail -2
[ "$TEST_RC" -eq 0 ] || rc=1

echo
if [ "$rc" -eq 0 ]; then echo "=== S3 OK ==="; else echo "=== S3 FAILED (rc=$rc) ==="; fi
echo "json : $OUT_DIR/c42_s3_reachability.json"
echo "csv  : $OUT_DIR/c42_s3_candidates.csv"
exit "$rc"
