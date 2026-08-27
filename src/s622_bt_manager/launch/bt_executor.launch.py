from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pick_place_config = os.path.join(
        get_package_share_directory('s622_bt_manager'),
        'config', 'pick_place_poses.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('tree_file', default_value=''), # 
        DeclareLaunchArgument('tree_id', default_value=''), #
        DeclareLaunchArgument('auto_start', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        # 2026-08-27：加 arm 参数（默认 left）——此前缺失导致单独启动时
        # arm=''（single-arm compat mode），BT 连无前缀 server（/set_gripper 等）
        # 而双臂 server 在 /left/* /right/* 下 → service unavailable → FAILURE。
        DeclareLaunchArgument('arm', default_value='left',
                              description='left / right / dual'),

        Node(
            package='s622_bt_manager',
            executable='bt_executor_node',
            name='bt_executor',
            output='screen',
            parameters=[
                pick_place_config,
                {
                'tree_file': LaunchConfiguration('tree_file'),
                'tree_id': LaunchConfiguration('tree_id'),
                'auto_start': LaunchConfiguration('auto_start'),
                'tick_rate_hz': 10,
                'groot2_port': 1667,
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'arm': LaunchConfiguration('arm'),
                }
            ]
        ),
    ])