#!/usr/bin/env bash
# C4.2 / S2 one-command reproduction: analytic multi-solution IK + the official
# Baseline B rebuild, plus the S1 regression.
#
#   bash src/dual_arm_pose_optimization/analysis/run_c42_s2.sh
#
# Environment overrides:
#   SKIP_BUILD=1   skip colcon build
set -uo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$(cd "$HERE/../../.." && pwd)"
PKG="dual_arm_pose_optimization"
OUT_DIR="$HERE/figs"
mkdir -p "$OUT_DIR"

URDF="${URDF:-$WS/src/dual_arm_kinematics/test/fixtures/s622_dual_arm.urdf}"
SRDF="${SRDF:-$WS/src/s622_moveit_config/config/s622_dual_arm.srdf}"
PROBE="$WS/build/$PKG/c42_baseline_probe"

echo "== C4.2 / S2 ====================================================="
echo " workspace : $WS"
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

rc=0

echo
echo "### 1/3 S2 acceptance probe (Baseline B rebuild) ---------------------"
"$PROBE" --urdf "$URDF" --srdf "$SRDF" --json "$OUT_DIR/c42_s2_evidence.json"
[ $? -eq 0 ] || rc=1

echo
echo "### 2/3 S1 regression probe -----------------------------------------"
"$WS/build/$PKG/c42_scene_probe" --urdf "$URDF" --srdf "$SRDF" >/tmp/c42_s2_s1.log 2>&1
S1_RC=$?
grep -E "^  ---> " /tmp/c42_s2_s1.log | sed 's/^/  /'
[ "$S1_RC" -eq 0 ] || { echo "  S1 regression FAILED (rc=$S1_RC)"; rc=1; }

echo
echo "### 3/3 gtest --------------------------------------------------------"
colcon test --merge-install --packages-select "$PKG" --event-handlers console_direct+ \
  >/tmp/c42_s2_gtest.log 2>&1
TEST_RC=$?
colcon test-result --test-result-base "$WS/build/$PKG" | tail -2
for t in test_scene_loader test_collision_backend test_ik_layer test_m4_baseline; do
  printf "  %-24s " "$t"
  "$WS/build/$PKG/$t" 2>/dev/null | grep -E "^\[  PASSED|^\[  FAILED" | tr '\n' ' '
  echo
done
[ "$TEST_RC" -eq 0 ] || rc=1

echo
if [ "$rc" -eq 0 ]; then
  echo "=== S2 OK ==="
else
  echo "=== S2 FAILED (rc=$rc) ==="
fi
echo "evidence json : $OUT_DIR/c42_s2_evidence.json"
exit "$rc"
