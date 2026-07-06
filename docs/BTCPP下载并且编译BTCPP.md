# 下载
到  https://github.com/BehaviorTree/BehaviorTree.ROS2 下载整个包， 解压到 src 目录之下
然后
```bash
# 1. 完全退出 conda（base 也要退）
conda deactivate
conda deactivate    # 多打一次确保退干净

# 检查：提示符前不应该有 (base) 或 (yolov8)
which python3
# 应该返回 /usr/bin/python3，不是 miniconda 下的

# 2. source ROS（顺序很重要：先退 conda 再 source ROS）
source /opt/ros/humble/setup.bash

# 3. 清掉这次失败留下的 build 缓存
rm -rf build/btcpp_ros2_interfaces

# 4. 重新编
colcon build --merge-install --symlink-install \
  --packages-select behaviortree_ros2 btcpp_ros2_interfaces btcpp_ros2_samples

```




#  conda 干扰 ROS2 的典型表现
看错误关键行：
```
execute_process(/home/yep/miniconda3/bin/python3 -m rosidl_adapter ...
ModuleNotFoundError: No module named 'em'
```

CMake 调用了 **conda 的 Python**（`/home/yep/miniconda3/bin/python3`），而不是系统的 `/usr/bin/python3`。conda 那个 Python 里没装 ROS2 需要的 `em`（empy）模块，所以挂了。

另外注意你的提示符现在是 `(base)`，说明刚才 `conda deactivate` 之后又被自动激活了，或者你新开了终端。conda 默认会自动激活 base，这就是根源。

## 一、立即修复（这次能编过）

```bash
# 1. 完全退出 conda（base 也要退）
conda deactivate
conda deactivate    # 多打一次确保退干净

# 检查：提示符前不应该有 (base) 或 (yolov8)
which python3
# 应该返回 /usr/bin/python3，不是 miniconda 下的

# 2. source ROS（顺序很重要：先退 conda 再 source ROS）
source /opt/ros/humble/setup.bash

# 3. 清掉这次失败留下的 build 缓存
rm -rf build/btcpp_ros2_interfaces

# 4. 重新编
colcon build --merge-install --symlink-install \
  --packages-select behaviortree_ros2 btcpp_ros2_interfaces btcpp_ros2_samples
```

## 二、永久解决：关掉 conda 自动激活 base

每次开终端都自动进 base 是 conda 的默认行为，对 ROS2 开发非常不友好。关掉它：

```bash
conda config --set auto_activate_base false
```

之后新开终端就是干净的系统环境，需要用 conda 时手动 `conda activate yolov8` 即可。

## 三、推荐的工作习惯

**两个环境严格分离**：

| 任务                                        | 环境                      |
| ----------------------------------------- | ----------------------- |
| `colcon build`、跑 ROS2 节点（C++/MoveIt/BT 等） | 系统 Python，**不开 conda**  |
| 跑 YOLO 推理（用 ultralytics、torch）            | `conda activate yolov8` |

YOLO 推理节点可以做成独立的 Python ROS2 node，启动时 conda 激活；其他节点用系统 Python 或 C++，互不干扰。

## 四、为什么 conda 会破坏 ROS2 编译

ROS2 Humble 的 `rosidl_adapter`、`ament_cmake` 等工具是 **针对系统 Python 3.10 + 系统 apt 装的依赖** 设计的。conda 环境里：

* Python 版本可能不一致（你的 yolov8 是 conda 装的，但 base 可能是 3.11/3.12）
* 没有 `python3-empy`、`python3-lark`、`python3-numpy` 等 ament 工具依赖
* `CMake` 通过 `find_package(Python3)` 找到的是 conda 的 Python，路径就跑偏了

所以编译 ROS2 时**必须**用干净环境。

## 五、如果还想偷懒的一招

在 `.bashrc` 里加一个函数，明确切换：

```bash
# 加到 ~/.bashrc 末尾
ros_env() {
    conda deactivate 2>/dev/null
    conda deactivate 2>/dev/null
    source /opt/ros/humble/setup.bash
    [ -f ~/my_S622/install/setup.bash ] && source ~/my_S622/install/setup.bash
    echo "ROS2 Humble env ready (conda off)"
}
```

以后每次要编/跑 ROS2，先敲 `ros_env`。

---

按上面修一下，重新跑 colcon，应该就能过了。出问题贴日志，我接着看。



# 验证 接下来：跑通官方 sample，理解 BT.CPP 的工作方式

编译过了只是有了工具，还得先**亲手让一个行为树跑起来**，建立直观感觉。然后再开始搭你自己的 mock 骨架。

## 一、先跑官方 sample

先确认环境真的 OK：

```bash
# 新开终端（保持干净）
conda deactivate 2>/dev/null
source /opt/ros/humble/setup.bash
source ~/my_S622/install/setup.bash

# 看看 sample 包提供了哪些可执行文件
ros2 pkg executables btcpp_ros2_samples
```

应该能看到几个例子，比如：

```
btcpp_ros2_samples sample_bt_executor
btcpp_ros2_samples sleep_server
...
```

按官方 README 的最小例子，需要**两个终端**：

**终端 1（启动 sleep server，相当于一个 mock action server）：**

```bash
source /opt/ros/humble/setup.bash
source ~/my_S622/install/setup.bash
ros2 run btcpp_ros2_samples sleep_server
```

**终端 2（启动 BT executor，跑行为树）：**

```bash
source /opt/ros/humble/setup.bash
source ~/my_S622/install/setup.bash
ros2 launch btcpp_ros2_samples sample_bt_executor.launch.xml
```

你会看到行为树开始 tick，控制台打印每个节点的状态。

**重点观察**：

* 行为树 XML 在哪里（`BehaviorTree.ROS2/btcpp_ros2_samples/bt_xml/`）
* 节点是怎么在 C++ 里实现的（`btcpp_ros2_samples/src/`）
* executor 是怎么加载 XML 和注册节点的（`sample_bt_executor.cpp`）

花 15 分钟读一遍这三个文件，比看 10 篇博客都管用。

## 二、然后用 Groot2 连上看（可选但强烈推荐）

如果你装了 Groot2：

1. 打开 Groot2
2. 选 `Monitor` 模式
3. 服务端地址填 `localhost:1667`（sample 默认就开了这个 publisher 端口）
4. 连上后能实时看到行为树高亮当前 tick 的节点

这个可视化效果会让你立刻明白行为树是怎么"流动"的。

## 三、跑通后，开始搭你自己的项目骨架

下面是我建议的工作包结构。你已经有 `~/my_S622/src/` 工作空间，直接在里面建：

```
my_S622/src/
├── bt_pick_place_msgs/        # 接口定义
│   ├── srv/DetectObjects.srv
│   └── action/MoveToPose.action
├── bt_pick_place/             # BT 节点 + XML + executor
│   ├── src/
│   │   ├── bt_executor.cpp
│   │   └── nodes/
│   ├── bt_xml/
│   │   └── pick_and_place.xml
│   └── launch/
└── (mock 节点先放在 bt_pick_place 里，后续再独立)
```

### Step 1：建接口包

```bash
cd ~/my_S622/src
ros2 pkg create --build-type ament_cmake bt_pick_place_msgs \
    --dependencies geometry_msgs std_msgs action_msgs
```

然后定义接口（先定义最少的两个）：

**`bt_pick_place_msgs/srv/DetectObjects.srv`**

```
# Request
bool trigger
---
# Response
bool success
geometry_msgs/PoseStamped[] poses
string[] labels
```

**`bt_pick_place_msgs/action/MoveToPose.action`**

```
# Goal
geometry_msgs/PoseStamped target_pose
float32 velocity_scale 0.3
---
# Result
bool success
string message
---
# Feedback
float32 progress
```

修改 `CMakeLists.txt`：

```cmake
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(action_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/DetectObjects.srv"
  "action/MoveToPose.action"
  DEPENDENCIES geometry_msgs action_msgs
)

ament_export_dependencies(rosidl_default_runtime)
```

修改 `package.xml`：

```xml
<buildtool_depend>rosidl_default_generators</buildtool_depend>
<depend>geometry_msgs</depend>
<depend>action_msgs</depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```

### Step 2：建 BT 包

```bash
cd ~/my_S622/src
ros2 pkg create --build-type ament_cmake bt_pick_place \
    --dependencies rclcpp behaviortree_cpp behaviortree_ros2 \
                   geometry_msgs bt_pick_place_msgs
```

### Step 3：编译两个包

```bash
cd ~/my_S622
colcon build --merge-install --symlink-install \
  --packages-select bt_pick_place_msgs bt_pick_place
```

## 四、做完上面这些再继续

跑完 sample、建好两个包之后，告诉我：

1. sample 是否正常跑起来了，控制台看到了什么
2. 你的两个新包是否编译通过

然后我给你写**第一版 mock executor**：加载一个最简的 XML，包含一个 mock 的 `MoveToPose` BT 节点（连到一个 sleep 几秒就返回成功的 mock action server），让整条链路先跑起来。

这一步打通后，后面把每个 mock 替换成真实的 MoveIt2 / YOLO / 夹爪，就是局部替换，不会动整体架构。

## 五、如果遇到卡点

常见小坑提前提醒：

* `sleep_server` 没起的话，BT 节点连不上 action server，会一直等
* 同时 source 多个 setup.bash 没关系，但顺序是先 `/opt/ros` 再 `~/my_S622/install`
* BT XML 文件路径在 launch 文件里写的是绝对路径或者通过 `ament_index` 找，自己改 XML 时要注意装到 install 目录里
* 用 `--symlink-install`，改 XML 不用重编（但 C++ 改了要重编）

开始吧，跑通 sample 后回来。
