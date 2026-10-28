#!/usr/bin/env bash
# C4.3 S4 W3-R6 -- build and run the SOFTWARE-ONLY dry-run loop.
#
# No Gazebo, no controller process, no actuator.  The gtest itself uses
# test/fake_controller.hpp.  This script only builds the package, runs that
# gtest with the workspace's shared libraries, and prints the artifact root.
set -eo pipefail

PKG_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS_DIR="$(cd "$PKG_DIR/../.." && pwd)"
cd "$WS_DIR"

# Environment required by this workspace (CLAUDE.md).  ROS/conda setup scripts
# read optional unset variables, so enable `set -u` only after they have run.
source /opt/ros/humble/setup.bash
eval "$(conda shell.bash hook)"
conda activate yolov8
set -u

PYTHON="$(command -v python3)"
colcon build --merge-install --symlink-install \
  --packages-select dual_arm_transport_executor \
  --cmake-args "-DPython3_EXECUTABLE=$PYTHON"

TEST_BIN="$WS_DIR/build/dual_arm_transport_executor/test_dry_run_loop"
if [[ ! -x "$TEST_BIN" ]]; then
  echo "ERROR: test binary not found: $TEST_BIN" >&2
  exit 1
fi

ARTIFACT_ROOT="${C43_W3R6_ARTIFACT_DIR:-${TMPDIR:-/tmp}/c43_s4_w3r6_dry_run}"
export C43_W3R6_ARTIFACT_DIR="$ARTIFACT_ROOT"
export LD_LIBRARY_PATH="$WS_DIR/install/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"

"$TEST_BIN"

echo
echo "W3-R6 dry-run artifacts: $ARTIFACT_ROOT"
find "$ARTIFACT_ROOT" -maxdepth 2 -name summary.json -print | sort
