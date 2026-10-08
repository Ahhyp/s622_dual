# s622_dual_arm_qp.launch.py
# C2.9a: Gazebo with the 125 Hz POSITION profile driving the QP controller.
#
#   ros2 launch dual_arm_qp_controller s622_dual_arm_qp.launch.py            # headless
#   ros2 launch dual_arm_qp_controller s622_dual_arm_qp.launch.py gui:=true
#
# This is deliberately SEPARATE from s622_dual_arm.launch.py (velocity/JTC
# profile): the two profiles claim mutually exclusive command interfaces.
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable,
                            TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_param_builder import load_xacro


def generate_launch_description():
    gz_share = get_package_share_directory("gz_launch")
    desc_share = get_package_share_directory("s622_moveit_descriptions")
    qp_share = get_package_share_directory("dual_arm_qp_controller")

    gui_arg = DeclareLaunchArgument("gui", default_value="false",
                                    description="gz GUI (false = server only)")

    set_model_path = SetEnvironmentVariable(
        "IGN_GAZEBO_RESOURCE_PATH",
        os.path.join(gz_share, "models") + ":" + os.environ.get("IGN_GAZEBO_RESOURCE_PATH", ""),
    )

    world_file = os.path.join(gz_share, "worlds", "dual_arm_world.sdf")
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")
        ),
        launch_arguments=[("gz_args", [
            world_file + " -r",
            PythonExpression(["' -s' if '", LaunchConfiguration("gui"), "' == 'false' else ''"]),
        ])],
    )

    # NOTE: xacro mappings must be plain strings at description time, so the
    # profile is selected by an environment variable (same style as gz_launch).
    controllers_yaml = os.environ.get(
        "QP_CONTROLLERS_YAML",
        os.path.join(qp_share, "config", "dual_arm_qp_gazebo_controllers.yaml"))
    urdf = load_xacro(
        Path(os.path.join(gz_share, "config", "s622_dual_arm_gazebo.urdf.xacro")),
        mappings={
            "instantiate": "false",
            "include_camera_visual": "false",
            "include_global_camera": "false",
            "include_wrist_camera": "false",
            "finger_stop_gap": "0.0",
            "gripper_stop_block": "false",
            "gripper_force_plugins": "false",
            "arm_command_mode": "position",
            "arm_position_gain": os.environ.get("QP_ARM_POSITION_GAIN", "20.0"),
            "controllers_yaml": controllers_yaml,
        },
    ).replace("package://s622_moveit_descriptions", desc_share)

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": urdf}],
        output="screen",
    )

    spawn_robot = TimerAction(
        period=5.0,
        actions=[Node(package="ros_gz_sim", executable="create",
                      arguments=["-string", urdf, "-x", "0.0", "-y", "0.0", "-z", "0.0",
                                 "-R", "0", "-P", "0", "-Y", "0", "-name", "s622_dual_arm"],
                      output="screen")],
    )

    def spawner(names):
        return Node(package="controller_manager", executable="spawner",
                    arguments=names + ["-c", "/controller_manager",
                                       "--controller-manager-timeout", "90"],
                    output="screen")

    spawn_controllers = TimerAction(
        period=12.0,
        actions=[
            spawner(["joint_state_broadcaster"]),
            spawner(["left_hand_controller", "right_hand_controller"]),
            TimerAction(period=3.0, actions=[spawner(["dual_arm_qp_controller"])]),
        ],
    )

    return LaunchDescription([
        gui_arg, set_model_path, gazebo, robot_state_publisher, spawn_robot, spawn_controllers,
    ])
