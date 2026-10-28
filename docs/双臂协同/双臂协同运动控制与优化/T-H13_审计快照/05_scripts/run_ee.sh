#!/usr/bin/env bash
cd /home/yep/my_S622 || exit 1
source /opt/ros/humble/setup.bash
eval "$(conda shell.bash hook)" && conda activate yolov8
source install/setup.bash
export ROS_DOMAIN_ID=73
export ROS_LOCALHOST_ONLY=1
dir=/tmp/th13/c_ee
export ROS_HOME="$dir/ros_home"
rm -rf "$dir/ros_home" "$dir/timing.csv"; mkdir -p "$dir/ros_home"
setsid ros2 launch dual_arm_qp_controller c43_s4_mock.launch.py \
    params_file:="$dir/params.yaml" >"$dir/launch.log" 2>&1 &
lp=$!
python3 /tmp/th13/run_early_exit.py c_ee "$dir/params.yaml" "$dir" 2.0 3.0
kill -- -"$lp" 2>/dev/null; sleep 1; kill -9 -- -"$lp" 2>/dev/null
wait "$lp" 2>/dev/null
ls -l "$dir/timing.csv" 2>/dev/null
