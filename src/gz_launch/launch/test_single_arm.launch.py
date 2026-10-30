'''
单臂一键启动入口（等价于 s622_gazebo.launch.py）。

历史：本文件原本只有一行注释、没有任何实现，容易被误认为「单臂 launch」。
单臂真正的启动逻辑在 `s622_gazebo.launch.py`（Gazebo + ros2_control + MoveIt2
+ RViz + MoveIt Servo + TOTG 重定时 + YOLOv8 OBB 检测）。

这里保留为显式入口，等价于：
    ros2 launch gz_launch s622_gazebo.launch.py

需要传参数（start_yolo / yolo_model_path / yolo_python / spawn_controllers）时，
直接启动 s622_gazebo.launch.py 即可。
'''
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("gz_launch"),
                "launch",
                "s622_gazebo.launch.py",
            )
        )
    )
    return LaunchDescription([gazebo_launch])
