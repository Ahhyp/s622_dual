# src/s622_moveit_config/launch/dual_arm_demo.launch.py
"""
M2.3 独立验收 launch: 仅 URDF+SRDF+kinematics 层面的验证.
不起 controller_manager, 用 joint_state_publisher 发 zeros.
Plan 可行, Execute 会失败(M2.4 再补).
"""
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("s622_dual_arm", package_name="s622_moveit_config")
        .robot_description(
            file_path="config/s622_dual_arm.urdf.xacro",
            mappings={"instantiate": "false"},  # 抑制主 URDF 顶层自动实例化
        )
        .robot_description_semantic(file_path="config/s622_dual_arm.srdf")
        .robot_description_kinematics(file_path="config/dual_arm_kinematics.yaml")
        .trajectory_execution(file_path="config/dual_arm_moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
        )
        .to_moveit_configs()
    )

    # robot_state_publisher
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[moveit_config.robot_description],
    )

    # joint_state_publisher(发 zeros)
    jsp = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        output="screen",
        parameters=[{
            "rate": 30,
            "use_gui": False,
        }],
    )

    # move_group
    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    # rviz(用 MoveIt 默认 rviz config, 也可以先跳过)
    rviz_config = PathJoinSubstitution([
        FindPackageShare("s622_moveit_config"),
        "config", "dual_arm.rviz",
    ])
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
    )

    return LaunchDescription([rsp, jsp, move_group, rviz])