#!/usr/bin/env bash
# 03: 容器内源码编译 BehaviorTree.CPP 4.6.2 到 /usr/local
#     Humble apt 源里只有 BT.CPP v3，而 behavioortree_ros2 / s622_bt_manager 需要 >= 4.6。
#     关键：Groot2 接口和 SQLite 日志必须 ON，否则 btcpp_ros2_samples / s622_bt_manager
#     链接时报 undefined reference to BT::Groot2Publisher::Groot2Publisher(...)
set -o pipefail
VER=4.6.2
SRC=/opt/BehaviorTree.CPP-${VER}

step() { printf '\n===== [%s] %s =====\n' "$(date +%H:%M:%S)" "$*"; }

if [ ! -d "$SRC" ]; then
  step "下载 BT.CPP ${VER}"
  curl -fL --retry 3 -o /tmp/btcpp.tar.gz \
    "https://gh-proxy.com/https://github.com/BehaviorTree/BehaviorTree.CPP/archive/refs/tags/${VER}.tar.gz" || exit 1
  tar -xzf /tmp/btcpp.tar.gz -C /opt || exit 1
fi

step "编译并安装"
cd "$SRC" || exit 1
rm -rf build
cmake -B build -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=ON \
  -DBTCPP_EXAMPLES=OFF -DBTCPP_UNIT_TESTS=OFF \
  -DBTCPP_GROOT_INTERFACE=ON -DBTCPP_SQLITE_LOGGING=ON || exit 1
cmake --build build -j"$(nproc)" || exit 1
cmake --install build || exit 1
ldconfig

step "验证"
ls /usr/local/lib/libbehaviortree_cpp*
echo "Groot2Publisher 符号数: $(nm -DC /usr/local/lib/libbehaviortree_cpp.so | grep -c Groot2Publisher)"
echo "===== DONE ====="
