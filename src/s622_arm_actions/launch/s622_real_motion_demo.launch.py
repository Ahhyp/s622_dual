# s622_real_motion_demo.launch.py
# 真机最小运动验证（无夹爪微动，2026-08-27）
#
# 复用 robotarm yolov8_grasping 的 demo_node_without_gripper：
#   读取当前末端 TF 位姿 → 沿 base_link 的 -Z 方向微动 move_distance（默认 5mm）
#   -> 可选返回原位
#
# 组件:
#   - s622_real_arm.launch.py   （真机控制栈：RSP + CM(fairino_hardware) + 3 controllers
#                                 + move_group_fairino + RViz + retime_server）
#   - demo_node_without_gripper （微动验证节点，默认 Plan Only + 手动 start）
#
# 安全默认（与 robotarm 一致）:
#   - start_demo=false       不自动运动，手动调 ~/start 服务触发
#   - execute_motion=false   Plan Only（只规划不执行）
#   - move_distance=0.005    仅 5 mm
#   - return_to_origin=false 只下降不返回
#   - demo_max_velocity=0.05 极慢（代码默认）
#
# 用法:
#   ros2 launch s622_arm_actions s622_real_motion_demo.launch.py ip:=192.168.58.3
#   # Plan Only 触发（机器人不会动）:
#   ros2 service call /demo_node_without_gripper/start std_srvs/srv/Trigger "{}"
#   # 真实执行（确认安全后）:
#   ros2 launch s622_arm_actions s622_real_motion_demo.launch.py ip:=192.168.58.3 \
#       execute_motion:=true move_distance:=0.005
#   ros2 service call /demo_node_without_gripper/start std_srvs/srv/Trigger "{}"
import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg = get_package_share_directory("s622_arm_actions")

    declare_ip = DeclareLaunchArgument("ip", default_value="192.168.58.3",
                                       description="机械臂控制器 IP")
    declare_prefix = DeclareLaunchArgument("prefix", default_value="",
                                           description="joint 名前缀（双臂用）")
    declare_start_demo = DeclareLaunchArgument(
        "start_demo", default_value="false",
        description="true=节点 ready 后自动跑；false=等 ~/start 服务（推荐）")
    declare_execute = DeclareLaunchArgument(
        "execute_motion", default_value="false",
        description="true=真实执行；false=Plan Only（只规划不执行，推荐先跑这个）")
    declare_move_distance = DeclareLaunchArgument(
        "move_distance", default_value="0.005",
        description="Z 轴微动距离（米），默认 5mm，上限受 max_execute_distance=0.020 保护")
    declare_return = DeclareLaunchArgument(
        "return_to_origin", default_value="false",
        description="true=下降后返回原位；false=只下降（首次推荐）")
    declare_max_execute = DeclareLaunchArgument(
        "max_execute_distance", default_value="0.020",
        description="执行保护上限（米）。move_distance 超过它会被安全机制拒绝执行。"
                    "默认 0.020（20mm）；要跑 0.05 需同时传 max_execute_distance:=0.06")
    # 2026-08-28：demo 限速参数化——SDK 1s 阻塞下，运动速度越高阻塞后位移越大，
    # 越容易触发上位机"速度超限"（ServoJ 14）。慢速（~0.002）可避免。
    declare_demo_speed = DeclareLaunchArgument(
        "demo_max_velocity", default_value="0.05",
        description="MoveIt velocity scaling（0-1）。真机 1s 阻塞下建议 ≤0.002 避免 14")

    # ===== 真机控制栈（RSP + CM + move_group + RViz + retime） =====
    real_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("s622_moveit_config"),
                "launch", "s622_real_arm.launch.py",
            )
        ),
        launch_arguments={
            "ip": LaunchConfiguration("ip"),
            "prefix": LaunchConfiguration("prefix"),
        }.items(),
    )

    # ===== 微动 Demo 节点（延迟 5s 等栈就绪） =====
    demo_node = TimerAction(
        period=5.0,
        actions=[
            Node(
                package="s622_arm_actions",
                executable="demo_node_without_gripper",
                name="demo_node_without_gripper",
                output="screen",
                parameters=[
                    os.path.join(pkg, "config", "demo_node_without_gripper.yaml"),
                    {
                        "start_demo": LaunchConfiguration("start_demo"),
                        "execute_motion": LaunchConfiguration("execute_motion"),
                        "move_distance": LaunchConfiguration("move_distance"),
                        "return_to_origin": LaunchConfiguration("return_to_origin"),
                        "max_execute_distance": LaunchConfiguration("max_execute_distance"),
                        "demo_max_velocity": LaunchConfiguration("demo_max_velocity"),
                    },
                ],
            )
        ],
    )

    return LaunchDescription([
        declare_ip, declare_prefix,
        declare_start_demo, declare_execute,
        declare_move_distance, declare_return, declare_max_execute, declare_demo_speed,
        real_stack,
        demo_node,
    ])
