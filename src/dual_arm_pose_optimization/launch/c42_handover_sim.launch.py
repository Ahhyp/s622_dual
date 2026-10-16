# c42_handover_sim.launch.py
# C4.2 -> simulation bridge: Gazebo + POSITION-interface arm JTCs, so the joint
# target produced by `c42_handover_target` can actually be executed.
#
#   ros2 launch dual_arm_pose_optimization c42_handover_sim.launch.py
#   ros2 launch dual_arm_pose_optimization c42_handover_sim.launch.py gui:=true
#
# Deliberately self-contained: it reuses the gz_launch world/xacro and the
# s622_moveit_descriptions meshes, but ships its own controllers yaml
# (config/c42_handover_sim_controllers.yaml) because the stock JTC profile cannot
# activate against the interfaces this robot exposes.  See that file's header.
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            SetEnvironmentVariable, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_param_builder import load_xacro


def generate_launch_description():
    gz_share = get_package_share_directory("gz_launch")
    desc_share = get_package_share_directory("s622_moveit_descriptions")
    bridge_share = get_package_share_directory("dual_arm_pose_optimization")

    gui_arg = DeclareLaunchArgument("gui", default_value="false",
                                    description="gz GUI (false = server only)")

    set_model_path = SetEnvironmentVariable(
        "IGN_GAZEBO_RESOURCE_PATH",
        os.path.join(gz_share, "models") + ":" +
        os.environ.get("IGN_GAZEBO_RESOURCE_PATH", ""),
    )

    world_file = os.path.join(gz_share, "worlds", "dual_arm_world.sdf")
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ros_gz_sim"), "launch",
                         "gz_sim.launch.py")
        ),
        launch_arguments=[("gz_args", [
            world_file + " -r",
            PythonExpression(["' -s' if '", LaunchConfiguration("gui"),
                              "' == 'false' else ''"]),
        ])],
    )

    controllers_yaml = os.path.join(bridge_share, "config",
                                    "c42_handover_sim_controllers.yaml")
    urdf = load_xacro(
        Path(os.path.join(gz_share, "config", "s622_dual_arm_gazebo.urdf.xacro")),
        mappings={
            "instantiate": "false",
            "include_camera_visual": "false",
            "include_global_camera": "false",
            "include_wrist_camera": "false",
            "calibration_arm": "none",
            "finger_stop_gap": "0.0",
            "gripper_stop_block": "false",
            "gripper_force_plugins": "false",
            # POSITION command interfaces: this is what the bridge's JTCs claim.
            "arm_command_mode": "position",
            "arm_position_gain": os.environ.get("C42_ARM_POSITION_GAIN", "20.0"),
            "controllers_yaml": controllers_yaml,
        },
    ).replace("package://s622_moveit_descriptions", desc_share)

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": urdf, "use_sim_time": True}],
        output="screen",
    )

    spawn_robot = TimerAction(
        period=5.0,
        actions=[Node(package="ros_gz_sim", executable="create",
                      arguments=["-string", urdf, "-x", "0.0", "-y", "0.0", "-z", "0.0",
                                 "-R", "0", "-P", "0", "-Y", "0",
                                 "-name", "s622_dual_arm"],
                      output="screen")],
    )

    def spawner(names):
        return Node(package="controller_manager", executable="spawner",
                    arguments=names + ["-c", "/controller_manager",
                                       "--controller-manager-timeout", "90",
                                       "--switch-timeout", "60"],
                    output="screen")

    spawn_controllers = TimerAction(
        period=12.0,
        actions=[
            spawner(["joint_state_broadcaster"]),
            TimerAction(period=3.0,
                        actions=[spawner(["left_arm_controller", "right_arm_controller",
                                          "left_hand_controller", "right_hand_controller"])]),
        ],
    )

    return LaunchDescription([
        gui_arg, set_model_path, gazebo, robot_state_publisher, spawn_robot,
        spawn_controllers,
    ])
