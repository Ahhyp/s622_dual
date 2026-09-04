#!/usr/bin/env python3
# =============================================================================
# [M2.7] 全局相机 Eye-on-Base 标定仿真环境
#
# 与 M2.3（EIH）相反：全局相机固定 world，大号 ArUco 标定板固定在指定臂末端。
#
# 组件：
#   1. 双臂仿真（s622_dual_arm.launch.py）+ 参数：
#        include_global_camera:=true   （world 全局俯视相机，RGB-only）
#        include_wrist_camera:=false   （关右腕相机 → 省渲染提帧率）
#        calibration_arm:=<arm>        （大号 ArUco 板固定到该臂 grasp_frame）
#   2. ros2_aruco 检测（订阅 /camera/color/image_raw —— 全局相机）
#   3. aruco_marker_pose_publisher → /aruco_marker/pose
#   4. calibration_aruco_publisher → 发布 calibration_aruco TF（camera_color_optical_frame 下）
#
# 用法：
#   ros2 launch gz_launch s622_global_handeye_sim.launch.py arm:=right
#   ros2 launch gz_launch s622_global_handeye_sim.launch.py arm:=left
#
# 标定（手动助手）：
#   ros2 run hand_eye_calibration manual_calibration_assistant.py --ros-args \
#     -p use_sim_time:=true \
#     -p calibration_output_directory:=$HOME/my_S622/src/hand_eye_calibration/calib/global_eye_on_base/right
#   （或直接改 global_eye_on_base_left/right.yaml 的 use_sim_time/output 后用 launch 加载）
#
# GT 验证：仿真中 base_frame → camera_color_optical_frame 的 TF 即真值
#   （freeze_simulation_truth 会查 base_frame → tracking_base_frame），
#   求解结果与 GT 比较（ground_truth_check_enabled=true）。
# =============================================================================

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

# [M2.7] 标定板挂哪只臂：优先 `arm:=` launch 参数（默认取 M2_ARM 环境变量，再默认 right）。
# [2026-09-03 修复] 原实现只在模块导入期从 M2_ARM 环境变量设置 M2_MODE，`arm:=` 参数
# 声明后从未被消费 → `arm:=left` 启动时 calibration_arm 实际仍是 right（相机一直看到
# 右臂上的板，左臂标定数据全废）。现改为 OpaqueFunction：解析 arm 参数后再写环境变量，
# s622_dual_arm.launch.py 构造期（含于函数内 evaluate）即可读到正确值。
DEFAULT_ARM = os.environ.get("M2_ARM", "right")


def generate_launch_description():
    gz_pkg = get_package_share_directory("gz_launch")
    handeye_pkg = get_package_share_directory("hand_eye_calibration")

    arm_arg = DeclareLaunchArgument(
        "arm", default_value=DEFAULT_ARM, choices=["left", "right"],
        description="标定板固定到哪只臂（可用 M2_ARM 环境变量给定默认值）")

    aruco_params = os.path.join(handeye_pkg, "config", "aruco_parameters.yaml")

    # ---- 2. ros2_aruco 检测（全局相机；大号标定板 marker 0.20m（240mm 板，纹理 marker 占 5/6 区域））----
    # 注意：parameters 列表后者覆盖前者 → aruco_parameters.yaml 在前，
    # 后面的显式 image/camera_info/marker_size 覆盖掉腕部默认值
    aruco_node = Node(
        package="ros2_aruco",
        executable="aruco_node",
        output="screen",
        parameters=[
            aruco_params,
            {"image_topic": "/camera/color/image_raw"},
            {"camera_info_topic": "/camera/color/camera_info"},
            {"marker_size": 0.20},
        ],
    )

    # ---- 3. ArUco marker pose publisher（/aruco_markers → /aruco_marker/pose）----
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

    # ---- 4. calibration_aruco TF publisher（camera_color_optical_frame → calibration_aruco）----
    calib_aruco_pub = Node(
        package="hand_eye_calibration",
        executable="calibration_aruco_publisher.py",
        name="calibration_aruco_publisher",
        output="screen",
        parameters=[
            {"tracking_base_frame": "camera_color_optical_frame"},
            {"tracking_marker_frame": "calibration_aruco"},
            {"marker_pose_topic": "/aruco_marker/pose"},
        ],
    )

    def _launch_setup(context, *_args, **_kwargs):
        # [2026-09-03 修复] 在 include 求值前按 arm 参数设置环境变量
        arm = str(context.launch_configurations.get("arm", DEFAULT_ARM))
        _apply_arm_env(arm)
        # ---- 1. 双臂仿真（其构造期读 M2_MODE）----
        dual_sim = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gz_pkg, "launch", "s622_dual_arm.launch.py"),
            ),
        )
        return [dual_sim, aruco_node, marker_pose_pub, calib_aruco_pub]

    return LaunchDescription([
        arm_arg,
        OpaqueFunction(function=_launch_setup),
    ])


def _apply_arm_env(arm: str) -> str:
    """[M2.7 回归保护] 把 `arm:=` 解析结果写入 M2_MODE/M2_ARM 环境变量。

    2026-09-03 曾因只读 M2_ARM env、从不消费 `arm:=` launch 参数导致左臂标定板
    挂错臂（相机看到"冻结"的板，数据全废）。该函数被 _launch_setup 调用，
    有 test_m27_regression::test_m2_arm_env 守护。返回规范化后的 arm。
    """
    arm = str(arm) if arm else DEFAULT_ARM
    os.environ["M2_MODE"] = arm
    os.environ["M2_ARM"] = arm
    return arm
