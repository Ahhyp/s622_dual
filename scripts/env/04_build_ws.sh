#!/usr/bin/env bash
# 04: 容器内 colcon build（与 CLAUDE.md 的策略一致：merge-install + symlink-install）
#     用法: bash /workspace/scripts/env/04_build_ws.sh [--clean]
set -o pipefail
source /opt/ros/humble/setup.bash
cd /workspace || exit 1

if [ "${1:-}" = "--clean" ]; then
  echo "清理 build/ install/ log/"
  rm -rf build install log
fi

echo "===== [$(date +%H:%M:%S)] colcon build 开始 ====="
colcon build --merge-install --symlink-install \
  --parallel-workers "$(nproc)" --continue-on-error \
  --event-handlers console_cohesion+ \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 -DCMAKE_BUILD_TYPE=Release
rc=$?
echo "===== [$(date +%H:%M:%S)] colcon build RC=$rc ====="
exit $rc
