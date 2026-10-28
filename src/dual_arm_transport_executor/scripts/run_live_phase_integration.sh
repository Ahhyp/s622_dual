#!/usr/bin/env bash
# C4.3 S4.1 W3-R2: run the live phase-producer / phase-consumer integration test.
#
#   src/dual_arm_transport_executor/scripts/run_live_phase_integration.sh [args]
#
# It sources ROS 2 Humble + this workspace, activates the conda `yolov8`
# interpreter (rclpy 3.10) and executes the Python orchestrator, which forks
# `ros2 launch dual_arm_qp_controller c43_s4_mock.launch.py` itself on
# ROS_DOMAIN_ID=77 / ROS_LOCALHOST_ONLY=1.  Any extra args are forwarded, e.g.
#   .../run_live_phase_integration.sh --out-dir /tmp/w3r2_live
set -eo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
cd "$REPO_ROOT"

# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
if [[ -f "$REPO_ROOT/install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "$REPO_ROOT/install/setup.bash"
fi
export LD_LIBRARY_PATH="$REPO_ROOT/install/lib:${LD_LIBRARY_PATH:-}"
export RCUTILS_LOGGING_BUFFERED_STREAM=0
export PYTHONUNBUFFERED=1

eval "$(conda shell.bash hook)"
conda activate yolov8

echo "[run] repo=$REPO_ROOT python=$(command -v python3) ROS_DOMAIN_ID=77"
exec python3 "$REPO_ROOT/src/dual_arm_transport_executor/scripts/test_live_phase_integration.py" "$@"
