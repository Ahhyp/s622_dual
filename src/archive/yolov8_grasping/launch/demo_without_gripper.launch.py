#!/usr/bin/env python3
"""
demo_without_gripper.launch.py — 无夹爪 Demo 启动文件

与 demo.launch.py 对应，去掉了：
  - 夹爪控制相关（hand_controller 仍会加载但不使用）

保留：
  - MoveIt 启动（fairino3_v6_moveit2_config）
  - Fake 控制器加载
  - demo_node_without_gripper（纯机械臂运动演示）

用法：
  ros2 launch yolov8_grasping demo_without_gripper.launch.py
"""
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # ===== MoveIt 启动（fairino3_v6_moveit2_config）=====
    moveit_demo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("fairino3_v6_moveit2_config"),
                "launch",
                "demo.launch.py",
            ])
        ])
    )

    # ===== Fake 控制器加载 =====
    spawn_controllers_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("fairino3_v6_moveit2_config"),
                "launch",
                "spawn_controllers.launch.py",
            ])
        ])
    )

    # ===== Demo 节点（无夹爪，延迟启动）=====
    demo_node = TimerAction(
        period=3.0,
        actions=[
            Node(
                package="yolov8_grasping",
                executable="demo_node_without_gripper",
                name="demo_node_without_gripper",
                output="screen",
                parameters=[
                    os.path.join(
                        get_package_share_directory("yolov8_grasping"),
                        "config",
                        "demo_node_without_gripper.yaml",
                    ),
                ],
            )
        ],
    )

    return LaunchDescription([
        moveit_demo_launch,
        spawn_controllers_launch,
        demo_node,
    ])
