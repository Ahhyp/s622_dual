#!/usr/bin/env bash
# 01: 宿主机侧 —— 从 ros:humble 构建本项目镜像（只需一次）
#
#   镜像内容 = ros:humble (Ubuntu 22.04)
#            + apt 依赖（02_apt_deps.sh）
#            + BehaviorTree.CPP 4.6.2 源码安装（03_btcpp.sh）
#            + torch/ultralytics venv（05_install_yolo.sh）
#
#   用法: ./01_image_build.sh [--force]
#   镜像: s622:humble-ros-yolo
set -euo pipefail

# 当前登录会话可能还没拿到 docker 组权限（kz 已在 docker 组，但组要新会话生效）
if ! docker info >/dev/null 2>&1; then
  echo "[info] 当前 shell 无 docker 权限，改用 sg docker 重新执行"
  # 用 printf %q 逐参数转义，避免 sg 内层 shell 二次展开 $(...) / 通配符
  exec sg docker -c "$(printf '%q ' "$0" "$@")"
fi

WS=$(cd "$(dirname "$0")/../.." && pwd)
IMAGE=s622:humble-ros-yolo
TMP=s622_imgbuild

if docker image inspect "$IMAGE" >/dev/null 2>&1 && [ "${1:-}" != "--force" ]; then
  echo "[ok] 镜像 $IMAGE 已存在；如需重建加 --force"
  exit 0
fi

echo "[info] 工作区: $WS"
docker rm -f "$TMP" >/dev/null 2>&1 || true
docker run -d --name "$TMP" --hostname s622-imgbuild \
  -v "$WS":/workspace "ros:humble" sleep infinity >/dev/null

trap 'docker rm -f "$TMP" >/dev/null 2>&1 || true' EXIT

docker exec "$TMP" bash /workspace/scripts/env/02_apt_deps.sh
docker exec "$TMP" bash /workspace/scripts/env/03_btcpp.sh
docker exec "$TMP" bash /workspace/scripts/env/05_install_yolo.sh

docker commit "$TMP" "$IMAGE" >/dev/null
echo "[ok] 镜像已生成: $IMAGE"
