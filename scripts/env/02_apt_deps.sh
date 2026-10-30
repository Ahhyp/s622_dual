#!/usr/bin/env bash
# 02: 容器内安装 s622_dual 的全部 apt 依赖（镜像构建时执行一次）
#     基础镜像 ros:humble（Ubuntu 22.04 + ROS 2 Humble）
#     用法（容器内，root）: bash /workspace/scripts/env/02_apt_deps.sh
set -o pipefail
export DEBIAN_FRONTEND=noninteractive

step() { printf '\n===== [%s] %s =====\n' "$(date +%H:%M:%S)" "$*"; }

step "apt 源（Ubuntu 26.04 宿主机没有 ros-humble-*，容器内用 22.04 源）"
sed -i 's|http://archive.ubuntu.com/ubuntu|http://mirrors.aliyun.com/ubuntu|g; s|http://security.ubuntu.com/ubuntu|http://mirrors.aliyun.com/ubuntu|g' /etc/apt/sources.list
for f in /etc/apt/sources.list.d/*.list; do
  [ -f "$f" ] && sed -i 's|http://packages.ros.org/ros2/ubuntu|https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu|g' "$f"
done
apt-get update -qq

step "编译工具与系统库"
apt-get install -y -qq \
  build-essential cmake git curl wget pkg-config \
  python3-colcon-common-extensions python3-rosdep python3-pip \
  python3-venv python3-numpy python3-scipy \
  libeigen3-dev libyaml-cpp-dev libfcl-dev libboost-all-dev libfmt-dev \
  libzmq3-dev libconsole-bridge-dev libopencv-dev

step "ROS 2 Humble 功能包"
apt-get install -y -qq \
  ros-humble-moveit \
  ros-humble-moveit-servo \
  ros-humble-ros2-control ros-humble-ros2-controllers \
  ros-humble-gz-ros2-control \
  ros-humble-ros-gz \
  ros-humble-realsense2-description \
  ros-humble-xacro ros-humble-robot-state-publisher ros-humble-joint-state-publisher \
  ros-humble-cv-bridge ros-humble-image-transport \
  ros-humble-tf2-ros ros-humble-tf2-geometry-msgs ros-humble-tf2-eigen \
  ros-humble-realtime-tools ros-humble-generate-parameter-library \
  ros-humble-urdf ros-humble-srdfdom

step "GUI 相关（Gazebo / RViz 需要，容器内走 llvmpipe 软件渲染）"
apt-get install -y -qq mesa-utils x11-apps xauth libgl1-mesa-dri

step "验证"
echo "--- ros-humble 包数量: $(dpkg -l | grep -c '^ii  ros-humble')"
for p in moveit_servo gz_ros2_control ros_gz_sim realsense2_description; do
  [ -d "/opt/ros/humble/share/$p" ] && echo "OK   $p" || echo "MISS $p"
done
echo "===== DONE ====="
