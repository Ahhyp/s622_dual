#!/usr/bin/env bash
# C4.2 / S1 one-command reproduction.
#
#   bash src/dual_arm_pose_optimization/analysis/run_c42_s1.sh
#
# Builds the package, runs the gtests, runs the acceptance probe and writes the
# evidence JSON to analysis/figs/c42_s1_evidence.json.
#
# Environment overrides:
#   SKIP_BUILD=1   skip colcon build
#   URDF=...       override the URDF fixture
#   SRDF=...       override the SRDF
set -uo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$(cd "$HERE/../../.." && pwd)"
PKG="dual_arm_pose_optimization"
OUT_DIR="$HERE/figs"
mkdir -p "$OUT_DIR"

URDF="${URDF:-$WS/src/dual_arm_kinematics/test/fixtures/s622_dual_arm.urdf}"
SRDF="${SRDF:-$WS/src/s622_moveit_config/config/s622_dual_arm.srdf}"
JSON="$OUT_DIR/c42_s1_evidence.json"

echo "== C4.2 / S1 ====================================================="
echo " workspace : $WS"
echo " urdf      : $URDF"
echo " srdf      : $SRDF"
echo "=================================================================="

# ROS/colcon setup scripts read unguarded env vars, which trips `set -u`.
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
set -u

if [ "${SKIP_BUILD:-0}" != "1" ]; then
  colcon build --merge-install --symlink-install \
    --cmake-args "-DPython3_EXECUTABLE=$(which python3)" \
    --packages-select "$PKG" || exit 1
fi

# CRITICAL: the ament index must contain s622_moveit_descriptions, otherwise the
# mesh pre-flight fails (which is the point of the negative test, but not of the
# normal run).
set +u
# shellcheck disable=SC1091
source "$WS/install/setup.bash"
set -u

rc=0

echo
echo "### 1/3 negative resource-path test (must FAIL loudly) ---------------"
if [ -n "${AMENT_PREFIX_PATH:-}" ]; then
  SAVED_AMENT="$AMENT_PREFIX_PATH"
  # Drop the workspace from the ament index -> meshes become unresolvable.
  export AMENT_PREFIX_PATH="/opt/ros/humble"
  "$WS/build/$PKG/c42_scene_probe" --urdf "$URDF" --srdf "$SRDF" >/tmp/c42_s1_neg.log 2>&1
  NEG_RC=$?
  export AMENT_PREFIX_PATH="$SAVED_AMENT"
  if [ "$NEG_RC" -ne 0 ] && grep -q "mesh resource(s) could not be resolved" /tmp/c42_s1_neg.log; then
    echo "  PASS  probe refused to build a scene with unresolved meshes"
    sed -n '1,3p' /tmp/c42_s1_neg.log | sed 's/^/        /'
  else
    echo "  FAIL  probe did NOT refuse (rc=$NEG_RC) -- see /tmp/c42_s1_neg.log"
    rc=1
  fi
else
  echo "  SKIP  AMENT_PREFIX_PATH was not set"
fi

echo
echo "### 2/3 acceptance probe --------------------------------------------"
"$WS/build/$PKG/c42_scene_probe" --urdf "$URDF" --srdf "$SRDF" --json "$JSON"
PROBE_RC=$?
echo "  probe rc=$PROBE_RC"
[ "$PROBE_RC" -eq 0 ] || rc=1

echo
echo "### 3/3 gtest --------------------------------------------------------"
colcon test --merge-install --packages-select "$PKG" --event-handlers console_direct+ \
  >/tmp/c42_s1_gtest.log 2>&1
TEST_RC=$?
colcon test-result --test-result-base "$WS/build/$PKG" | tail -3
tail -4 /tmp/c42_s1_gtest.log | sed 's/^/  /'
[ "$TEST_RC" -eq 0 ] || rc=1

echo
if [ "$rc" -eq 0 ]; then
  echo "=== S1 OK ==="
else
  echo "=== S1 FAILED (rc=$rc) ==="
fi
echo "evidence json : $JSON"
echo "gtest log     : /tmp/c42_s1_gtest.log"
exit "$rc"
