#!/usr/bin/env bash
# 06: 容器内 —— 启动单臂仿真（Gazebo + MoveIt2 + RViz + YOLO + Servo）
#   等价于: ros2 launch gz_launch s622_gazebo.launch.py
#
#   常用参数:
#     start_yolo:=false           只跑仿真/规划，不启 YOLO（没装 ultralytics 时用）
#     yolo_model_path:=<abs>.pt   指定权重（默认自动探测 src/yolov8_obb/models）
#     yolo_python:=<python>       指定解释器（默认 /opt/ros-yolo-venv/bin/python3）
# 注意：ROS 的 setup.bash 不是 set -u 安全的（引用未定义变量会直接退出）
set -eo pipefail
set +u
source /opt/ros/humble/setup.bash
source /workspace/install/setup.bash
cd /workspace
exec ros2 launch gz_launch s622_gazebo.launch.py "$@"
