#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import xacro


def generate_launch_description():
    # 这里指向你 MoveIt 配置里的 xacro
    xacro_file = os.path.join(
        get_package_share_directory("s622_moveit_config"),
        "config",
        "s622_moveit_descriptions.urdf.xacro",
    )

    if not os.path.exists(xacro_file):
        raise RuntimeError(f"Xacro file not found: {xacro_file}")

    doc = xacro.process_file(xacro_file)
    robot_description = {"robot_description": doc.toxml()}

    retime_server = Node(
        package="trajectory_retime_server",
        executable="retime_server",
        name="trajectory_retime_server",
        output="screen",
        parameters=[robot_description],
    )

    return LaunchDescription([retime_server])
