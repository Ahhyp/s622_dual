#!/usr/bin/env bash
# C4.3 S4 W3-R1a T-H13: run one round of the 638-cycle transport bench.
# usage: run_rounds.sh <label> [<label> ...]

cd /home/yep/my_S622 || exit 1
source /opt/ros/humble/setup.bash
eval "$(conda shell.bash hook)" && conda activate yolov8
source install/setup.bash
export ROS_DOMAIN_ID=73
export ROS_LOCALHOST_ONLY=1

for label in "$@"; do
  dir=/tmp/th13/$label
  mkdir -p "$dir"
  export ROS_HOME="$dir/ros_home"
  rm -rf "$dir/ros_home" "$dir/timing.csv"
  mkdir -p "$dir/ros_home"
  echo "=== round $label $(date -Is)"
  setsid ros2 launch dual_arm_qp_controller c43_s4_mock.launch.py \
      params_file:="$dir/params.yaml" >"$dir/launch.log" 2>&1 &
  lp=$!
  python3 /tmp/th13/run_round.py "$label" "$dir/params.yaml" "$dir"
  rc=$?
  # stop the whole launch process group (controller_manager + spawners)
  kill -- -"$lp" 2>/dev/null
  sleep 1
  kill -9 -- -"$lp" 2>/dev/null
  wait "$lp" 2>/dev/null
  sleep 3
  echo "=== round $label rc=$rc csv=$(ls -l "$dir/timing.csv" 2>/dev/null | awk '{print $5" bytes"}')"
done
