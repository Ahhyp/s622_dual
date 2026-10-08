#!/usr/bin/env bash
# Regenerate the KDL oracle fixture from the dual-arm xacro.
#
# Requires: a built workspace (the dual-arm xacro does $(find s622_moveit_config)),
# i.e. run from the repo root after:
#   source /opt/ros/humble/setup.bash
#   source install/setup.bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
OUT="${REPO_ROOT}/src/dual_arm_kinematics/test/fixtures/s622_dual_arm.urdf"

xacro "${REPO_ROOT}/src/gz_launch/config/s622_dual_arm_gazebo.urdf.xacro" > "${OUT}"
echo "wrote ${OUT} ($(wc -c < "${OUT}") bytes)"
