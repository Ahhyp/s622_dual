#!/usr/bin/env bash
# 07: 容器内 —— 冒烟测试：确认 Gazebo 机械臂能跟随轨迹运动
#   用法: bash scripts/env/07_smoke_test.sh [关节序号 0-5] [增量rad]
# 注意：ROS 的 setup.bash 不是 set -u 安全的（引用未定义变量会直接退出）
set -eo pipefail
set +u
source /opt/ros/humble/setup.bash
source /workspace/install/setup.bash
cd /workspace
exec python3 scripts/env/smoke_test_motion.py "$@"
