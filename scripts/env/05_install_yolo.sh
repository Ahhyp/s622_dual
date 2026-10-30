#!/usr/bin/env bash
# 05: 为 yolov8_obb 安装 Python 运行时（容器内执行）
#   venv 使用 --system-site-packages，从而可以 import rclpy / cv_bridge / sensor_msgs
#   但 torch / ultralytics / 新版 numpy / opencv 装在 venv 里，不覆盖 apt 的系统包
set -o pipefail
VENV=/opt/ros-yolo-venv

step() { printf '\n===== [%s] %s =====\n' "$(date +%H:%M:%S)" "$*"; }

step "创建 venv（继承系统 site-packages）"
[ -d "$VENV" ] || python3 -m venv --system-site-packages "$VENV" || exit 1
"$VENV/bin/python3" -m pip install -q -U pip wheel setuptools
"$VENV/bin/pip" config set global.index-url https://pypi.tuna.tsinghua.edu.cn/simple
"$VENV/bin/pip" config set global.trusted-host pypi.tuna.tsinghua.edu.cn

step "torch / torchvision（CUDA 版，cu126）"
# 2026-10-29：改成 CUDA 版。YOLO 推理原来跑 CPU，是相机帧率的新瓶颈（宿主 load 打满 16 核）。
# 驱动 595 兼容 cu126；GPU 不可用时自动回退 CPU 版，保证换机器也能装上。
TORCH_INDEX=${TORCH_INDEX:-https://download.pytorch.org/whl/cu126}
TORCH_SPEC=${TORCH_SPEC:-torch==2.14.1+cu126 torchvision==0.29.1+cu126}
if ! "$VENV/bin/pip" install $TORCH_SPEC --index-url "$TORCH_INDEX"; then
  echo "[warn] CUDA 轮子安装失败，回退 CPU 版" >&2
  "$VENV/bin/pip" install torch torchvision \
    --index-url https://download.pytorch.org/whl/cpu
fi
echo "TORCH-RC=$?"

step "ultralytics + numpy<2"
"$VENV/bin/pip" install "numpy<2" "opencv-python>=4.6" ultralytics
echo "ULTRA-RC=$?"

step "验证（ROS 消息 + 推理库都能导入）"
source /opt/ros/humble/setup.bash
"$VENV/bin/python3" - <<'PY'
import sys
print("python:", sys.version.split()[0])
import numpy, cv2, torch
print("numpy:", numpy.__version__, "| cv2:", cv2.__version__, "| torch:", torch.__version__)
print("torch cuda build:", torch.version.cuda, "| cuda available:", torch.cuda.is_available())
import ultralytics
print("ultralytics:", ultralytics.__version__)
import rclpy, cv_bridge
print("rclpy: OK | cv_bridge: OK")
PY
echo "===== DONE ====="
