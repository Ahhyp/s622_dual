#!/usr/bin/env bash
# C2.9b: assert the QP profile is exclusive -- no other controller may claim the
# same arm joints, and the velocity interfaces of the JTC profile do not exist.
set -u
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
YAML="${1:-$HERE/../config/dual_arm_qp_controllers_exclusive_test.yaml}"
YAML="$(readlink -f "$YAML")"
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/ros_log}"
export ROS_HOME="${ROS_HOME:-/tmp/ros_home}"
LOG=/tmp/excl_profile.log
SPAWN_TIMEOUT=20
rc=0

PYTHONUNBUFFERED=1 ros2 launch dual_arm_qp_controller dual_arm_qp_mock.launch.py \
  params_file:="$YAML" > "$LOG" 2>&1 &
LAUNCH_PID=$!
trap 'kill $LAUNCH_PID 2>/dev/null; pkill -f "ros2_control_node" 2>/dev/null' EXIT

# readiness: poll the controller_manager service (launch stdout is buffered)
ready=0
for i in $(seq 1 90); do
  if ros2 service call /controller_manager/list_controllers \
       controller_manager_msgs/srv/ListControllers "{}" 2>/dev/null \
       | grep -q "name='dual_arm_qp_controller', state='active'"; then
    ready=1; break
  fi
  sleep 1
done
if [ "$ready" != "1" ]; then
  echo "FAIL: dual_arm_qp_controller did not become active"; exit 1
fi
echo "OK: dual_arm_qp_controller active"

if ros2 run controller_manager spawner left_arm_jtc_probe -c /controller_manager \
     --controller-manager-timeout "$SPAWN_TIMEOUT" > /tmp/excl_p1.log 2>&1; then
  echo "FAIL: position JTC activated alongside the QP (interfaces not exclusive!)"; rc=1
else
  echo "OK: position JTC refused (arm position interfaces already claimed by the QP)"
fi

if ros2 run controller_manager spawner left_arm_jtc_vel_probe -c /controller_manager \
     --controller-manager-timeout "$SPAWN_TIMEOUT" > /tmp/excl_p2.log 2>&1; then
  echo "FAIL: velocity JTC activated (velocity interface should not exist)"; rc=1
else
  echo "OK: velocity JTC refused (no velocity command interface in the QP profile)"
fi

exit $rc
