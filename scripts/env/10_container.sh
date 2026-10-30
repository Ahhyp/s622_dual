#!/usr/bin/env bash
# 10: 宿主机侧 —— s622 运行容器管理（带 X11 + host 网络 + 工作区挂载）
#
#   ./10_container.sh ensure    # 没有就创建，有就启动（默认）
#   ./10_container.sh recreate  # 强制重建（改过 --gpus 等创建期参数时用）
#   ./10_container.sh shell     # 进容器（已 source ROS + 工作空间）
#   ./10_container.sh run <cmd> # 在容器里跑一条命令（透传 DISPLAY）
#   ./10_container.sh stop      # 停止
#   ./10_container.sh status    # 状态（含渲染模式）
#
# 渲染: RENDER=auto（默认，宿主机有 NVIDIA + nvidia runtime 就用 GPU）
#       RENDER=gpu 强制；RENDER=software 强制 llvmpipe
set -euo pipefail

if ! docker info >/dev/null 2>&1; then
  # 用 printf %q 逐参数转义，避免 sg 内层 shell 二次展开 $(...) / 通配符
  exec sg docker -c "$(printf '%q ' "$0" "$@")"
fi

IMAGE=s622:humble-ros-yolo
NAME=s622_run
WS=$(cd "$(dirname "$0")/../.." && pwd)

# 交互终端才加 -t；脚本/非交互调用时 docker exec -t 会报 "the input device is not a TTY"
TTY_FLAGS=(-i)
[ -t 0 ] && TTY_FLAGS+=(-t)

# 渲染模式：RENDER=auto（默认，宿主机有 NVIDIA 驱动 + nvidia runtime 就用 GPU）
#           RENDER=gpu 强制 GPU；RENDER=software 强制 llvmpipe
render_mode() {
  case "${RENDER:-auto}" in
    gpu|software) echo "$RENDER" ;;
    *)
      # 没有 nvidia runtime 一定用不了 GPU
      if ! docker info 2>/dev/null | grep -q "nvidia"; then
        echo software
        return
      fi
      # 已经是 GPU 的容器不要自动降级：agent 沙箱里看不到 /dev/nvidia*，
      # nvidia-smi 必然失败，不能据此把它重建回 llvmpipe。降级请显式 RENDER=software。
      if [ "$(container_render)" = gpu ]; then
        echo gpu
        return
      fi
      if command -v nvidia-smi >/dev/null 2>&1 && nvidia-smi -L >/dev/null 2>&1; then
        echo gpu
        return
      fi
      # 沙箱里看不到 /dev/nvidia*，宿主 nvidia-smi 必然失败 —— 改用 docker 自己探测
      # （container toolkit 装好时 --gpus all 能跑通；~2s，只在需要新建容器时调用）
      if docker run --rm --gpus all --entrypoint /usr/bin/nvidia-smi \
           "$IMAGE" -L >/dev/null 2>&1; then
        echo gpu
      else
        echo software
      fi ;;
  esac
}

container_render() {
  docker inspect -f '{{ index .Config.Labels "s622.render" }}' "$NAME" 2>/dev/null || true
}

create() {
  local mode="$1"
  local -a gpu_flags=()
  local -a env_flags=(-e DISPLAY="${DISPLAY:-:0}" -e QT_X11_NO_MITSHM=1)
  if [ "$mode" = gpu ]; then
    gpu_flags=(--gpus all)
    # 能力要包含 graphics，否则容器里没有 libGLX_nvidia（只剩 mesa → llvmpipe）
    # 镜像里烘焙了 ENV LIBGL_ALWAYS_SOFTWARE=1（旧的 docker commit 遗留），显式压掉
    env_flags+=(-e NVIDIA_DRIVER_CAPABILITIES=all -e LIBGL_ALWAYS_SOFTWARE=0)
  else
    env_flags+=(-e LIBGL_ALWAYS_SOFTWARE=1)
  fi
  if ! docker run -d --name "$NAME" --hostname s622-run \
      --net=host --ipc=host \
      "${gpu_flags[@]}" \
      -v "$WS":/workspace \
      -v "$HOME/ros_build":/build_scripts \
      -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
      "${env_flags[@]}" \
      --label "s622.render=$mode" \
      --restart unless-stopped \
      "$IMAGE" sleep infinity >/dev/null; then
    if [ "$mode" = gpu ]; then
      echo "[warn] GPU 容器创建失败，回退软件渲染（检查 nvidia-container-toolkit / --gpus all）" >&2
      docker rm -f "$NAME" >/dev/null 2>&1 || true
      create software
    else
      return 1
    fi
  fi
}

ensure() {
  # Xwayland 默认只允许本用户连，容器里是 root
  xhost +SI:localuser:root >/dev/null 2>&1 || true
  local want have
  want=$(render_mode)
  if docker ps -a --format '{{.Names}}' | grep -qx "$NAME"; then
    have=$(container_render)
    if [ "$have" != "$want" ]; then
      # --gpus / -e 只能在创建时生效，模式变了必须重建
      echo "[info] 容器渲染模式 ${have:-未知} → $want，重建 $NAME" >&2
      docker rm -f "$NAME" >/dev/null
      create "$want"
    else
      docker start "$NAME" >/dev/null
    fi
  else
    create "$want"
  fi
  echo "[ok] 容器 $NAME 就绪（DISPLAY=${DISPLAY:-:0}，渲染=$(container_render)）"
}

case "${1:-ensure}" in
  ensure|start) ensure ;;
  status)
    # docker ps 的 .Labels 是 "k=v,k=v" 字符串，不能当 map 索引；渲染模式用 inspect 取
    docker ps -a --filter "name=$NAME" --format '{{.Names}}\t{{.Status}}\t{{.Image}}'
    docker inspect -f '  render={{ index .Config.Labels "s622.render" }}' "$NAME" 2>/dev/null || true ;;
  recreate)
    # 改了 --gpus / 环境变量后重建容器（工作区是挂载的，不会丢代码）
    docker rm -f "$NAME" >/dev/null 2>&1 || true
    ensure ;;
  shell)
    ensure
    docker exec "${TTY_FLAGS[@]}" -e DISPLAY="${DISPLAY:-:0}" "$NAME" bash -lc \
      'source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && cd /workspace && exec bash' ;;
  run)
    shift
    ensure
    docker exec "${TTY_FLAGS[@]}" -e DISPLAY="${DISPLAY:-:0}" "$NAME" bash -lc \
      "source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && cd /workspace && $*" ;;
  stop)
    docker stop "$NAME" >/dev/null && echo "[ok] 已停止 $NAME" ;;
  *)
    echo "用法: $0 {ensure|recreate|shell|run <cmd>|stop|status}" >&2
    echo "  渲染: RENDER=auto|gpu|software（默认 auto，有 NVIDIA 就用 GPU）" >&2
    exit 1 ;;
esac
