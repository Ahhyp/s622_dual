# launch/dual_arm_qp_mock.launch.py
# Headless dual-arm mock plant + the QP controller (no Gazebo, no MoveIt).
#
#   ros2 launch dual_arm_qp_controller dual_arm_qp_mock.launch.py
#   ros2 launch dual_arm_qp_controller dual_arm_qp_mock.launch.py \
#       params_file:=$(ros2 pkg prefix ...)/share/dual_arm_qp_controller/config/dual_arm_qp_controllers_d7_test.yaml
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def launch_setup(context, *args, **kwargs):
    share = get_package_share_directory("dual_arm_qp_controller")
    xacro_file = os.path.join(share, "config", "s622_mock_dual_arm.urdf.xacro")
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
        "dual_arm_qp_controllers.yaml",
    )
    return LaunchDescription([
        DeclareLaunchArgument("params_file", default_value=default_params,
                              description="controllers yaml (position/QP profile)"),
        OpaqueFunction(function=launch_setup),
    ])
