#!/usr/bin/env python3
# =============================================================================
# [M2 标定] 双臂仿真标定环境（Eye-in-Hand，右腕相机）
#
# 组件：
#   1. 双臂仿真（s622_dual_arm.launch.py：world+双臂+world相机+右腕相机桥接+move_group）
#   2. ArUco 标定板 spawn（ros_gz_sim create，右臂前方固定位姿，板面朝右臂）
#   3. ros2_aruco 检测（订阅 /wrist_camera/color/image_raw + camera_info）
#   4. aruco_marker_pose_publisher → /aruco_marker/pose
#   5. calibration_aruco_publisher → 发布 calibration_aruco TF
#
# 用法：
#   终端 1：ros2 launch gz_launch s622_calibration_sim.launch.py
#   终端 2：ros2 run hand_eye_calibration auto_calibration_collector.py \
#             --ros-args -p use_sim_time:=true -p auto_start:=true \
#             -p calibration_output_directory:=$HOME/my_S622/src/hand_eye_calibration/calib/sim
#   （collector 参数默认已配双臂右臂版：right_grasp_frame / wrist_camera / right_arm）
#
# 标定板位姿（右臂 base 在 (-0.35,0,0) yaw=0 朝 +x）：
#   board_x/y/z 默认放在右臂前方桌面上方，板面法线朝右臂（-x 方向）。
#   跑起来后用 rqt_image_view 看 /wrist_camera/color/image_raw 确认板在视野内再微调。
# =============================================================================

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    gz_pkg = get_package_share_directory("gz_launch")
    handeye_pkg = get_package_share_directory("hand_eye_calibration")
    aruco_pkg = get_package_share_directory("ros2_aruco")

    # ---- 标定板位姿（右臂前方桌面，平放 marker 朝上）----
    # 板模型：9cm×9cm×2mm，板面法线沿 y（marker 格子 y=±0.0016）。
    # 平放朝上 = roll=-90°（绕 x 转 -90°：y→z），z≈0.01（板厚 2mm，贴桌面）。
    # 右臂 base 在 (-0.35,0,0) 朝 +x → 板放右臂前方：x=-0.1（前方 0.25m）、y=0.2 偏一侧。
    board_x = DeclareLaunchArgument("board_x", default_value="-0.10",
                                    description="标定板 X（world，右臂前方）")
    board_y = DeclareLaunchArgument("board_y", default_value="0.20",
                                    description="标定板 Y（偏一侧避开机械臂正投影）")
    board_z = DeclareLaunchArgument("board_z", default_value="0.01",
                                    description="标定板 Z（桌面顶面 z=0，板厚 2mm → 0.01）")
    board_roll = DeclareLaunchArgument("board_roll", default_value="-1.5708",
                                       description="绕 X 转 -90°：板面法线 y→z，marker 朝上")
    board_pitch = DeclareLaunchArgument("board_pitch", default_value="0.0")
    board_yaw = DeclareLaunchArgument("board_yaw", default_value="0.0")

    # ---- 1. 双臂仿真（含右腕相机桥接）----
    dual_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gz_pkg, "launch", "s622_dual_arm.launch.py"),
        ),
    )

    # ---- 2. ArUco 标定板 spawn（等仿真就绪后）----
    board_model = os.path.join(
        gz_pkg, "worlds", "models", "aruco_5x5_250_id1", "model.sdf"
    )
    spawn_board = TimerAction(
        period=6.0,
        actions=[
            Node(
                package="ros_gz_sim",
                executable="create",
                output="screen",
                arguments=[
                    "-file", board_model,
                    "-name", "calibration_board",
                    "-x", LaunchConfiguration("board_x"),
                    "-y", LaunchConfiguration("board_y"),
                    "-z", LaunchConfiguration("board_z"),
                    "-R", LaunchConfiguration("board_roll"),
                    "-P", LaunchConfiguration("board_pitch"),
                    "-Y", LaunchConfiguration("board_yaw"),
                    "-allow_renaming", "false",
                ],
            ),
        ],
    )

    aruco_params = os.path.join(handeye_pkg, "config", "aruco_parameters.yaml")

    # ---- 3. ros2_aruco 检测（右腕相机）----
    aruco_node = Node(
        package="ros2_aruco",
        executable="aruco_node",
        output="screen",
        parameters=[
            {"image_topic": "/wrist_camera/color/image_raw"},
            {"camera_info_topic": "/wrist_camera/color/camera_info"},
            aruco_params,
        ],
    )

    # ---- 4. ArUco marker pose publisher（/aruco_markers → /aruco_marker/pose）----
    marker_pose_pub = Node(
        package="hand_eye_calibration",
        executable="aruco_marker_pose_publisher.py",
        name="aruco_marker_pose_publisher",
        output="screen",
        parameters=[
            {"marker_id": 1},
            {"aruco_topic": "/aruco_markers"},
            {"output_topic": "/aruco_marker/pose"},
        ],
    )

    # ---- 5. calibration_aruco TF publisher（wrist_camera_color_optical_frame → calibration_aruco）----
    calib_aruco_pub = Node(
        package="hand_eye_calibration",
        executable="calibration_aruco_publisher.py",
        name="calibration_aruco_publisher",
        output="screen",
        parameters=[
            {"tracking_base_frame": "wrist_camera_color_optical_frame"},
            {"tracking_marker_frame": "calibration_aruco"},
            {"marker_pose_topic": "/aruco_marker/pose"},
        ],
    )

    return LaunchDescription([
        board_x, board_y, board_z, board_roll, board_pitch, board_yaw,
        dual_sim,
        spawn_board,
        aruco_node,
        marker_pose_pub,
        calib_aruco_pub,
    ])
