#!/usr/bin/env python3
import os
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import xacro


def _build_node(context):
    # 2026-08-23：传齐 robot_description / robot_description_semantic /
    # robot_description_kinematics 三个参数——retime_server.cpp 优先用本节点参数
    #（has_usable_parameter），不再走硬编码 /move_group 的拉取路径
    # （我们的 move_group 在 /move_group_fairino namespace 下）。
    # URDF 用 gz_launch 主 launch 同款 robot_gazebo.urdf.xacro（可解析、含 gazebo
    # 段但 RobotModelLoader 容忍；与 move_group 实际使用的 URDF 一致）。
    # ⚠️ 不要用 s622_moveit_config/config/s622_moveit_descriptions.urdf.xacro——
    # 它 include 悬空的 .urdf（源树只有 .xacro），launch 解析直接抛异常。
    xacro_file = os.path.join(
        get_package_share_directory("gz_launch"),
        "config",
        "robot_gazebo.urdf.xacro",
    )
    srdf_file = os.path.join(
        get_package_share_directory("s622_moveit_config"),
        "config",
        "s622_moveit_descriptions.srdf",
    )
    kin_file = os.path.join(
        get_package_share_directory("s622_moveit_config"),
        "config",
        "kinematics_fairino.yaml",
    )

    robot_description_xml = LaunchConfiguration("robot_description").perform(context)
    robot_semantic_xml = LaunchConfiguration("robot_description_semantic").perform(context)
    robot_kinematics_str = LaunchConfiguration("robot_description_kinematics").perform(context)

    if not robot_description_xml:
        if not os.path.exists(xacro_file):
            raise RuntimeError(f"Xacro file not found: {xacro_file}")
        robot_description_xml = xacro.process_file(xacro_file).toxml()

    if not robot_semantic_xml:
        if not os.path.exists(srdf_file):
            raise RuntimeError(f"SRDF file not found: {srdf_file}")
        with open(srdf_file, "r", encoding="utf-8") as f:
            robot_semantic_xml = f.read()

    robot_kinematics = None
    if robot_kinematics_str:
        robot_kinematics = yaml.safe_load(robot_kinematics_str)
    if not isinstance(robot_kinematics, dict):
        if not os.path.exists(kin_file):
            raise RuntimeError(f"Kinematics file not found: {kin_file}")
        with open(kin_file, "r", encoding="utf-8") as f:
            robot_kinematics = yaml.safe_load(f)
    if not isinstance(robot_kinematics, dict):
        robot_kinematics = {}

    retime_server = Node(
        package="trajectory_retime_server",
        executable="retime_server",
        name="trajectory_retime_server",
        output="screen",
        parameters=[
            {"service_name": LaunchConfiguration("service_name")},
            {"robot_description": robot_description_xml},
            {"robot_description_semantic": robot_semantic_xml},
            {"robot_description_kinematics": robot_kinematics},
            # 2026-08-24：use_sim_time 由外层 launch 传入（launch_arguments），
            # 不硬编码——否则节点内 get_clock().now() 用墙钟时间，与 RViz
            #（use_sim_time=True）的时间基准不一致，曾导致 RViz TF jump back 连锁
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
    )
    return [retime_server]


def generate_launch_description():
    xacro_file = os.path.join(
        get_package_share_directory("gz_launch"),
        "config",
        "robot_gazebo.urdf.xacro",
    )
    srdf_file = os.path.join(
        get_package_share_directory("s622_moveit_config"),
        "config",
        "s622_moveit_descriptions.srdf",
    )
    kin_file = os.path.join(
        get_package_share_directory("s622_moveit_config"),
        "config",
        "kinematics_fairino.yaml",
    )

    robot_description_default = ""
    if os.path.exists(xacro_file):
        robot_description_default = xacro.process_file(xacro_file).toxml()

    robot_semantic_default = ""
    if os.path.exists(srdf_file):
        with open(srdf_file, "r", encoding="utf-8") as f:
            robot_semantic_default = f.read()

    robot_kinematics_default = ""
    if os.path.exists(kin_file):
        with open(kin_file, "r", encoding="utf-8") as f:
            robot_kinematics_default = yaml.safe_dump(yaml.safe_load(f))

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "service_name",
                default_value="/retime_trajectory",
                description="Service name for trajectory_retime_server",
            ),
            DeclareLaunchArgument(
                "robot_description",
                default_value=robot_description_default,
                description="URDF XML string for trajectory_retime_server",
            ),
            DeclareLaunchArgument(
                "robot_description_semantic",
                default_value=robot_semantic_default,
                description="SRDF XML string for trajectory_retime_server",
            ),
            DeclareLaunchArgument(
                "robot_description_kinematics",
                default_value=robot_kinematics_default,
                description="YAML string for robot_description_kinematics parameter",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Whether to use simulation time (pass 'true' when inside a Gazebo launch)",
            ),
            OpaqueFunction(function=_build_node),
        ]
    )
