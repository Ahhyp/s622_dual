#!/usr/bin/env bash
# C4.2 / S4 one-command reproduction: normalised soft objectives, single-objective
# poles, weighted selection and the Pareto front.
#
#   bash src/dual_arm_pose_optimization/analysis/run_c42_s4.sh
#
# Depends on the S3 candidate table; regenerates it if missing.
# Environment overrides:
#   SKIP_BUILD=1   skip colcon build
#   SKIP_S3=1      do not regenerate the S3 table even if missing
set -uo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$(cd "$HERE/../../.." && pwd)"
PKG="dual_arm_pose_optimization"
OUT_DIR="$HERE/figs"
CSV="$OUT_DIR/c42_s3_candidates.csv"
PY="${PY:-$HOME/miniconda3/envs/yolov8/bin/python}"
mkdir -p "$OUT_DIR"

echo "== C4.2 / S4 ====================================================="

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
  if [ "${SKIP_S3:-0}" = "1" ]; then
    echo "missing $CSV and SKIP_S3=1"; exit 1
  fi
  echo "### 0/3 S3 candidate table missing -> regenerating (~2 min)"
  SKIP_BUILD=1 bash "$HERE/run_c42_s3.sh" || exit 1
fi

echo
echo "### 1/3 S4 optimizer (Python) ---------------------------------------"
"$PY" "$HERE/optimize_c42.py" --csv "$CSV" --out-dir "$OUT_DIR"
[ $? -eq 0 ] || rc=1

echo
echo "### 2/3 invariant tests (proximal scope subset / gradient) ----------"
"$WS/build/$PKG/test_collision_backend" 2>/dev/null | grep -E "^\\[  PASSED|^\\[  FAILED" \
  | sed 's/^/  /'
[ $? -eq 0 ] || rc=1

echo
echo "### 3/3 gtest --------------------------------------------------------"
colcon test --merge-install --packages-select "$PKG" --event-handlers console_direct+ \
  >/tmp/c42_s4_gtest.log 2>&1
TEST_RC=$?
colcon test-result --test-result-base "$WS/build/$PKG" | tail -2
[ "$TEST_RC" -eq 0 ] || rc=1

echo
if [ "$rc" -eq 0 ]; then echo "=== S4 OK ==="; else echo "=== S4 FAILED (rc=$rc) ==="; fi
echo "objectives json : $OUT_DIR/c42_s4_objectives.json"
echo "pareto figure   : $OUT_DIR/fig5_pareto.png"
exit "$rc"
