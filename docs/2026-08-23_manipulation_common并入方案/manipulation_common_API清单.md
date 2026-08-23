# manipulation_common 模块 API 清单

- **日期**：2026-08-23
- **来源**：robotarm 项目（`~/fairino_robotarm-main/src/manipulation_common`）
- **状态**：✅ 已纳入 my_S622（构建通过 + 37 单测全绿 + git 已跟踪）
- **代码量**：2804 行，8 个模块 + 1 个节点

---

## 1. 模块总览

| 模块 | 文件 | 职责 |
|---|---|---|
| `MoveItMotion` | `planning/motion_executor.py` | 核心运动执行器（多 move_group / IK 切换 / 规划+执行+重定时） |
| `TrajectoryScore` 等 | `planning/trajectory_scoring.py` | 轨迹评分选优（路径长度 / 腕部运动量） |
| `KeepoutManager` | `planning/keepout_manager.py` | 禁入区碰撞对象管理 |
| `DetectionCache` | `perception/detection_cache.py` | 目标位姿缓存（按目标类型分主题订阅） |
| `TargetSelector` | `perception/target_selector.py` | 目标选择（显式 / 缓存双模式） |
| `AbortManager` | `task/abort_manager.py` | 急停 / 恢复 / 重置的状态机 |
| `PoseTools` | `utils/pose_tools.py` | 位姿构造工具 |
| `TfTools` | `utils/tf_tools.py` | TF 变换工具 |
| `yaml_loader` | `launch_utils/yaml_loader.py` | 参数文件加载（launch 用） |
| `MotionControlNode` | `nodes/motion_control_node.py` | 键盘/话题命令 → MoveIt stop/reset 事件（demo 节点） |
| `params` | `utils/params.py` | 节点参数读取辅助 |

---

## 2. 详细 API

### 2.1 MoveItMotion（`planning/motion_executor.py`）

```python
class MoveItMotion:
    def __init__(self, node, group_name, move_group_namespace="", ...)
    def wait_client_ready(self, planning_client=None, timeout_sec=3.0) -> bool
    def arm(self)  # 当前选中机械臂
    def _select_arm(self, planning_client: Optional[str])  # 按 planning_client 切换 IK（fairino/kdl）
    def _planning_client_key(self, planning_client, arm) -> str
    def set_ik(self, ...)  # IK 切换
    # 核心：plan + execute + retime（依赖 robotarm 版 pymoveit2 的 move_group_namespace）
```

依赖：robotarm 定制版 pymoveit2（`move_group_namespace` 参数）—— **阶段 A 已完成的前提**。

### 2.2 轨迹评分（`planning/trajectory_scoring.py`）

```python
class TrajectoryScore          # 评分结果 dataclass
class TrajectoryScoreConfig    # 权重配置
def path_length(trajectory) -> float
def joint_subset_path_length(trajectory, joint_indices) -> float
def score_trajectory(...) -> TrajectoryScore
def rank_paths(...) -> List[TrajectoryScore]
def select_best_path(...) -> Optional[JointTrajectory]
```

### 2.3 KeepoutManager（`planning/keepout_manager.py`）

```python
class KeepoutConfig           # 禁入区配置
class KeepoutManager:
    def enable(self, z_min: float)   # 添加盒状碰撞对象（禁入区）
    def disable(self)                # 移除
    def _make_collision_object(self, z_min: float) -> CollisionObject
```

### 2.4 DetectionCache（`perception/detection_cache.py`）

```python
class DetectionCache:
    def reset(self)
    def get_position(self, target) -> Optional[PointStamped]
    def get_rpy(self, target) -> Optional[Dict[str, float]]
    # 回调：on_elongated_object_pos / on_cube_pos / on_box_pos / on_stone_pos
    #       on_elongated_object_rpy / on_cube_rpy / on_stone_rpy
```

按目标类型订阅对应 topic（elongated_object / cube / box / stone），缓存最新位姿。

### 2.5 TargetSelector（`perception/target_selector.py`）

```python
class TargetSelector:
    def set_preference(self, preferred_target: str)
    def set_timeout(self, detection_timeout: float)
    def msg_age_sec(self, stamp) -> float
    def pair_valid(self, obj_pos: PointStamped, obj_rpy: dict) -> bool
    def select_target(self, TargetType, *args, **kwargs)  # 显式 / 缓存双模式
    def _select_from_explicit(self, TargetType, ...)
    def _select_from_cache(self, TargetType, cache)
```

### 2.6 AbortManager（`task/abort_manager.py`）

```python
class AbortManager:
    def is_set(self) -> bool
    def is_blocked(self) -> bool
    def clear(self)
    def recovery_active(self) -> bool
    def recovery_message(self) -> str
    def recovery_released(self) -> bool
    def is_reset_requested(self) -> bool
    def is_stop_requested(self) -> bool
    def set_recovery_hooks(self, **hooks)
    def set_command_hook(self, hook)
    # 状态机：STOP / RESET / RESUME；恢复钩子（open/home）链式执行
```

### 2.7 PoseTools / TfTools / params（utils/）

```python
class PoseTools:
    def make_pose(self, x, y, z, roll_deg, pitch_deg, yaw_deg) -> Pose
    def to_pose_stamped(self, pose: Pose, frame_id: str | None = None) -> PoseStamped

class TfTools:
    def _check_tf_ready(self)
    def transform_point(self, point_stamped, target_frame: str, timeout_sec=0.2)
    def camera_point_to_base(self, point_stamped, timeout_sec=0.2)

# params.py
def param(node, name, default)
def param_f(node, name, default: float) -> float
def param_b(node, name, default: bool) -> bool
```

### 2.8 yaml_loader（launch_utils/yaml_loader.py）

```python
def load_yaml(package_name: str, relative_path: str) -> Dict[str, Any]
def load_ros_parameters_yaml(...)
def package_file(package_name: str, relative_path: str) -> str
def wrap_yaml_as_ros_params_file(...)
```

### 2.9 MotionControlNode（nodes/motion_control_node.py）

```python
class MotionControlNode(Node):
    def _configure_keyboard(self)
    def _publish_command(self, command: str)
    def _relay_command(self, msg)
    def tick(self)
def trajectory_event_for_command(command: str) -> str
def main()
# console_script: motion_control
# launch: launch/motion_control.launch.py
```

---

## 3. 验证状态

| 项 | 结果 |
|---|---|
| `colcon build --packages-select manipulation_common` | ✅ 通过（仅 setuptools tests_require 弃用警告，无害） |
| 4 个测试文件（37 用例） | ✅ **37 passed**（需 `export ROS_LOG_DIR=/tmp/ros_log`，否则沙箱拦截 ~/.ros/log 导致 rclpy 初始化 error） |
| git 跟踪 | ✅ 27 个文件已 `git add` |

> **⚠️ 测试注意事项**：本机沙箱禁止写 `~/.ros/log`，跑含 rclpy 的测试前必须 `export ROS_LOG_DIR=/tmp/ros_log`，否则报 `Context.init() must only be called once`（实为日志文件打开失败）。这是环境问题，非代码问题。

---

## 4. 与 my_S622 的衔接点

- **namespace**：`MoveItMotion(move_group_namespace="/move_group_fairino")` 连 namespaced move_group（阶段 A 已就绪）
- **retime**：`_retime_trajectory_if_needed` 需要 `trajectory_retime_server` 启动（阶段 D 加 launch 节点）
- **测试命令**（以后复用）：

```bash
source /opt/ros/humble/setup.bash
source ~/my_S622/install/setup.bash
eval "$(conda shell.bash hook)" && conda activate yolov8
export ROS_LOG_DIR=/tmp/ros_log
cd ~/my_S622/src/manipulation_common && python3 -m pytest test/
```
