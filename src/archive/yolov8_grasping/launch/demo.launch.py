#!/usr/bin/env python3
"""
demo.launch.py — 无相机 Demo 启动文件

与 elongated_object_box_system.launch.py 对应，去掉了：
  - 相机启动（realsense）
  - YOLO 检测节点
  - 手眼标定发布器
  - 抓取状态机节点

保留：
  - MoveIt 启动（fairino3_v6_moveit2_config）
  - Fake 控制器加载
  - demo_node（复用 manipulation_common 的简单运动演示）

用法：
  ros2 launch yolov8_grasping demo.launch.py
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

    # ===== Demo 节点（延迟启动）=====
    demo_node = TimerAction(
        period=3.0,
        actions=[
            Node(
                package="yolov8_grasping",
                executable="demo_node",
                name="demo_node",
                output="screen",
                parameters=[
                    os.path.join(
                        get_package_share_directory("yolov8_grasping"),
                        "config",
                        "demo_node.yaml",
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
