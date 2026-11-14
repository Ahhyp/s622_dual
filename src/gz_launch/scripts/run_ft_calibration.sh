#!/usr/bin/env bash
# Run inside the ROS 2 Humble / Gazebo Fortress container.
set -eo pipefail

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
cd "$script_dir/../../.."
source /opt/ros/humble/setup.bash
source install/setup.bash

mkdir -p log/ft_calibration
run_dir=$(mktemp -d "$PWD/log/ft_calibration/run.XXXXXX")
echo "Calibration output: $run_dir"

export ROS_DOMAIN_ID=${C43_FT_ROS_DOMAIN_ID:-77}
export IGN_PARTITION="c43-ft-$(basename "$run_dir")"
export C43_DISABLE_HOMING=1 C43_GRIPPER_FORCE_PLUGINS=0
export C43_FT_SENSOR=left_finger1_joint_ft
export C43_S4_WORLD_FILE="$run_dir/world.sdf"
export C43_ROBOT_SDF="$run_dir/robot.sdf"

gz_share=$(ros2 pkg prefix --share gz_launch)
descriptions=$(ros2 pkg prefix --share s622_moveit_descriptions)
qp_share=$(ros2 pkg prefix --share dual_arm_qp_controller)
export QP_CONTROLLERS_YAML="$qp_share/config/dual_arm_qp_gazebo_controllers.yaml"
xacro "$gz_share/config/c43_s4_dual_arm_gazebo.urdf.xacro" \
    arm_command_mode:=position arm_position_gain:="${QP_ARM_POSITION_GAIN:-20.0}" \
    include_wrist_camera:=false controllers_yaml:="$QP_CONTROLLERS_YAML" \
    ft_sensor_name_left:="$C43_FT_SENSOR" > "$run_dir/robot.xacro.urdf"

# Remove the weld plugin and sample from the temporary calibration inputs.
python3 - "$run_dir" "$descriptions" "$script_dir/../worlds/c43_s4_world_ftest.sdf" <<'PY'
import sys
from pathlib import Path
import xml.etree.ElementTree as ET

run_dir = Path(sys.argv[1])
urdf = (run_dir / "robot.xacro.urdf").read_text()
robot = ET.fromstring(urdf.replace("package://s622_moveit_descriptions", sys.argv[2]))
for gazebo in robot.findall("gazebo"):
    for plugin in gazebo.findall("plugin"):
        if plugin.get("name", "").endswith("::DetachableJoint"):
            gazebo.remove(plugin)
ET.ElementTree(robot).write(run_dir / "robot.urdf", encoding="utf-8")

world_file = ET.parse(sys.argv[3])
world = world_file.getroot().find("world")
for include in world.findall("include"):
    if include.findtext("name") == "c43_sample":
        world.remove(include)
world_file.write(run_dir / "world.sdf", encoding="utf-8")
PY

ign sdf -p "$run_dir/robot.urdf" > "$run_dir/robot.converted.sdf"
python3 "$script_dir/inject_joint_ft_sensor.py" "$run_dir/robot.converted.sdf" "$C43_ROBOT_SDF"

ros2 launch gz_launch c43_s4_full_flow.launch.py gui:=false executor:=false \
    > "$run_dir/launch.log" 2>&1 &
launch_pid=$!
trap 'kill -TERM "$launch_pid" 2>/dev/null || true; wait "$launch_pid" 2>/dev/null || true' EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

deadline=$((SECONDS + 90))
while kill -0 "$launch_pid" 2>/dev/null && (( SECONDS < deadline )); do
    controllers=$(timeout 5 ros2 control list_controllers 2>/dev/null || true)
    controllers=$(sed -E $'s/\033\\[[0-9;]*m//g' <<< "$controllers")
    if grep -Eq '^left_hand_controller([[:space:]]|\[).*[[:space:]]active[[:space:]]*$' <<< "$controllers" &&
       grep -Eq '^left_arm_controller([[:space:]]|\[).*[[:space:]]active[[:space:]]*$' <<< "$controllers"; then
        sleep 2
        python3 -u "$script_dir/ft_probe.py" 2>&1 | tee "$run_dir/probe.log"
        exit 0
    fi
    sleep 1
done

echo "Calibration controllers did not become active. See $run_dir/launch.log" >&2
tail -30 "$run_dir/launch.log" >&2
exit 1
