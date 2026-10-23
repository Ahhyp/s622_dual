# launch/c43_s4_mock.launch.py
# C4.3 S4.1 W2: headless mock plant + the QP controller with the S4 profile.
#
#   ros2 launch dual_arm_qp_controller c43_s4_mock.launch.py
#
# Differences from dual_arm_qp_mock.launch.py:
#   * default params file is config/c43_s4_mock_params.yaml (w_center 1e-4 and
#     require_executor_heartbeat: true -- the E8 interlock is on, so the driver
#     must publish ~/heartbeat or the controller holds);
#   * nothing else.  No Gazebo, no MoveIt, no grippers: this bench measures the
#     12-DOF position-command QP loop only.
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def launch_setup(context, *args, **kwargs):
    share = get_package_share_directory("dual_arm_qp_controller")
    # C4.3 S4.1 W2: bench plant that starts at the frozen S2 q_grasp12, so the
    # transport is exercised from the grasp configuration it is defined for.
    xacro_file = os.path.join(share, "config", "c43_s4_mock_dual_arm.urdf.xacro")
    params_file = LaunchConfiguration("params_file")

    robot_description = {
        "robot_description": ParameterValue(
            Command([FindExecutable(name="xacro"), " ", xacro_file]), value_type=str
        )
    }

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, params_file],
        output="both",
    )
    jsb_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager",
                   "--controller-manager-timeout", "30"],
        output="both",
    )
    qp_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["dual_arm_qp_controller", "-c", "/controller_manager",
                   "--controller-manager-timeout", "30"],
        output="both",
    )
    return [control_node, jsb_spawner, qp_spawner]


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory("dual_arm_qp_controller"),
        "config",
        "c43_s4_mock_params.yaml",
    )
    return LaunchDescription([
        DeclareLaunchArgument("params_file", default_value=default_params,
                              description="C4.3 S4.1 bench controllers yaml"),
        OpaqueFunction(function=launch_setup),
    ])
