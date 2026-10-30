#!/usr/bin/env bash
# 08: 容器内 —— 停止单臂仿真留下的所有进程（不动容器的 PID 1）
#   用法: bash scripts/env/08_stop_sim.sh [--hard]
#     --hard  先 SIGTERM，2 秒后再 SIGKILL 兜底
#
#   注意：脚本以文件形式执行，避免 pkill 的模式串出现在自己的命令行里误杀自己。
set -o pipefail

PATTERNS=(
  "ros2 launch"
  "ign gazebo"
  "ign-gazebo"
  "rviz2"
  "moveit_ros_move_group/move_group"
  "servo_node_main"
  "parameter_bridge"
  "yolov8_obb_node"
  "retime_server"
  "controller_manager/spawner"
)

echo "== 停止仿真进程 =="
for p in "${PATTERNS[@]}"; do
  if pkill -f "$p" 2>/dev/null; then
    echo "  SIGNAL  $p"
  fi
done

echo "== 停止 ros2 守护进程 =="
ros2 daemon stop >/dev/null 2>&1 || true

sleep 2

if [ "${1:-}" = "--hard" ]; then
  echo "== SIGKILL 兜底 =="
  for p in "${PATTERNS[@]}"; do
    pkill -9 -f "$p" 2>/dev/null && echo "  KILL    $p"
  done
  sleep 1
fi

echo "== 复查 =="
# 注意：僵尸进程（stat 以 Z 开头）也会被 pgrep 匹配，这里排除，只看真正还活着的
left=$(ps -eo stat=,cmd= | grep -v '^Z' | grep -v grep \
  | grep -cE "ign gazebo|rviz2|moveit_ros_move_group|servo_node_main|yolov8_obb_node")
echo "  仍在运行的仿真进程: $left"
echo "  僵尸进程: $(ps -eo stat= | grep -c '^Z')  （僵尸只有重启容器才能回收：docker restart s622_run）"
