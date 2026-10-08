# 真机部署：hardware 包 IP 参数化（规划）

日期：2026-08-26
状态：✅ 实施完成，真机 58.3 单臂 20mm 微动验证通过（2026-08-27）
关联执行记录：[执行记录.md](执行记录.md)（含 ServoJ 14 / 速度超限 / 轨迹过期 排查全过程）
关联包：`fairino_hardware`、`s622_moveit_config`、`gz_launch`

## 1. 背景

- 真机双臂：左臂控制器 IP `192.168.58.2`，右臂控制器 IP `192.168.58.3`
- 当前 `fairino_hardware` 包 IP 硬编码为 `192.168.58.2`（两个宏、三处使用点）
- 目标：把 IP 参数化，先跑通 **58.3 单臂**，再扩展双臂
- 说明：robotarm 项目已在 58.2 上反复验证过，58.2 留作对照；my_S622 用参数化版本在 58.3 验证

## 2. IP 硬编码全景（全工作区）

唯一连接机械臂控制器的代码在 `fairino_hardware` 包内：

| #   | 位置                                                                                                  | 用途                                                           | 连接方式 |
| --- | ----------------------------------------------------------------------------------------------------- | -------------------------------------------------------------- | -------- |
| 1   | `include/fairino_hardware/fairino_hardware_interface.hpp:15` → `CONTROLLER_IP_ADDRESS "192.168.58.2"` | ros2_control 插件默认 IP（hpp:81 `_controller_ip` 用它初始化） | XML-RPC  |
| 2   | `src/fairino_hardware_interface.cpp:259`                                                              | `on_activate()` 里 `_ptr_robot->RPC(_controller_ip)`           | XML-RPC  |
| 3   | `include/fairino_hardware/data_type_def.h:17` → `CONTROLLER_IP "192.168.58.2"`                        | command_server 默认 IP                                         | —        |
| 4   | `src/command_server.cpp:208`                                                                          | `robot_command_thread` 构造 `RPC(_controller_ip)`              | XML-RPC  |
| 5   | `src/command_server.cpp:2565`                                                                         | `robot_recv_thread` 构造（8081 TCP 状态端口）                  | TCP      |

`libfairino/include/robot.h:32` 只是注释。其余包（planning_ros / visual_servo / BT）不直接连控制器。

## 3. 结论：必须参数化，且不止 IP 一处

双臂真机下盘点出 4 个点：

1. **IP 参数化**（必改）
   ros2_control 标准做法：`<hardware><param name="ip">` → 插件 `on_init()` 读
   `info_.hardware_parameters["ip"]`，缺省回退宏 `CONTROLLER_IP_ADDRESS`。
   双臂 = 两块 `<ros2_control>` 各带各的 IP。

2. **joint 名前缀不认**（必改，🚨 关键）
   `export_state_interfaces()` / `export_command_interfaces()` 硬匹配 `"j1".."j6"` / `"finger1_joint"`，
   双臂 URDF 是 `left_j1` / `right_j1`，不认前缀会直接 Fatal "Unknown joint name"。
   需加 `prefix` 参数，按 `${prefix}j1..6`、`${prefix}finger1/2` 匹配。

3. **命令接口类型冲突**（必改）
   仿真 ros2_control 块声明 **velocity** 命令接口（gz_ros2_control 插件）；
   真机插件 `on_init` **强制 position** 命令接口。
   → 真机需要独立的 ros2_control 块 + 独立 controllers yaml（position 版），仿真文件不动。

4. **command_server 双臂化**（决策：本轮不做）
   当前主链路（MoveIt + BT + 插件 ServoJ）不依赖 command_server；
   `fr_command_server` / `fr_state_brodcast` 节点名与 `nonrt_state_data` topic 双臂会冲突。
   → 保持单臂调试工具（58.2 默认），后续需要时再参数化 + 双实例。

## 4. 改动清单

### Phase A — fairino_hardware 插件参数化（核心）

文件：`src/fairino_hardware/src/fairino_hardware_interface.cpp` + `include/fairino_hardware/fairino_hardware_interface.hpp`

- `on_init()` 增加参数读取：
  - `ip`：`info_.hardware_parameters["ip"]`，缺省用宏 `CONTROLLER_IP_ADDRESS`（hpp:81 默认值保留）
  - `prefix`：`info_.hardware_parameters["prefix"]`，缺省空字符串（单臂兼容）
- `export_state_interfaces()` / `export_command_interfaces()`：匹配 `${prefix}j1..6`、`${prefix}finger1_joint`、`${prefix}finger2_joint`
- 日志（"机械臂SDK连接成功/失败"等）加前缀或硬件块名区分左右
- 单臂（无前缀）行为完全不变 —— 兼容现有用法

### Phase B-1 — 真机 URDF / 控制器配置（新增文件，仿真不动）

- `s622_moveit_config/config/s622_real_ros2_control.xacro`（新）：fairino_hardware 插件 + `ip`/`prefix` 参数 + **position** 命令接口块（仿 `s622_gazebo_ros2_control.xacro` 的 prefix 结构）
- 单臂真机 URDF（新）：基于 `s622_moveit_descriptions.urdf.xacro` 单臂本体 + 真机 ros2_control 块，ip 参数由 launch 传入
- `s622_moveit_config/config/real_controllers.yaml`（新）：position 版（arm/hand controller + JSB）
- 真机 launch（新）：CM + controller + JSB + move_group + RViz，`ip` 作为 launch 参数（默认 192.168.58.3）

> 说明：先做**单臂真机**（58.3）跑通；双臂真机文件（复用 `s622_dual_arm_gazebo.urdf.xacro` 本体、双 ros2_control 块带 left/right IP）在单臂验证通过后再加。

### Phase B-2 — 真机保守速度配置（2026-08-26 追加）

仿真提速配置在真机必须降回保守值（对照 robotarm 58.2 真机基准），**独立文件，不污染仿真**：

| 配置 | 位置 | 仿真当前值 | 真机建议值 |
|------|------|-----------|-----------|
| BT 运动 velocity_scale | BT XML（真机版） | 0.8（2026-08-25 提速） | **0.3**（对齐 robotarm） |
| MoveIt 全速 scaling | `joint_limits.yaml`（真机版） | 1.0 | **0.1 ~ 0.3** |
| 视觉伺服 descend_speed | `visual_servo_node.py`（真机版参数） | 0.12 | **0.02 ~ 0.04** |
| 视觉伺服 max_linear_vel / max_angular_vel | 同上 | 0.3 / 1.5 | **0.08 / 0.45** |
| 视觉伺服 visual_align_max_step | 同上 | 0.004 | **0.0015** |
| servo.yaml 奇异阈值 / joint_limit_margin | `servo.yaml`（真机版） | 200/500、0.02 | 恢复保守（60/120、0.1） |

不受影响（无需改）：加速度限制（has_acceleration_limits: false → TOTG 默认 1 rad/s²，比 robotarm 的 2.0 更保守）、Gazebo 物理步长（真机无关）、关节 max_velocity（与 URDF 物理上限一致）。

### ⚠️ 真机控制接口能力边界（2026-08-27 记录，重要）

**机械臂（真机 FairinoHardwareInterface）只有位置控制接口能用。**

背景：2026-08-25 修复仿真 gazebo NaN 报错时，给 finger 关节补了
`<state_interface name="velocity" />`（见 `docs/2026-08-25_place位姿标定/执行记录.md`）。
**那是 gz_ros2_control 插件的专属修复（仿真 read() 无判空读 JointVelocity 组件），不代表真机能力。**

真机插件（`fairino_hardware`）实际能力：

| 接口 | 真机状态 | 说明 |
|------|---------|------|
| command: position | ✅ 唯一可用 | `on_init()` **强制**每关节 command interface 必须是 position（否则 Fatal）；`write()` 用 ServoJ 下发位置 |
| state: position | ✅ 可用 | `read()` 读 `GetActualJointPosDegree`（真实关节角） |
| state: velocity | ❌ 不存在 | 插件**不导出** velocity state；`/joint_states` 的 velocity 恒 NaN（真机实测确认） |
| command: velocity | ❌ 不可用 | 声明即 Fatal "position expected" |
| command/state: effort | ❌ 不可用 | 同样不支持 |

推论与约束：
1. 真机 ros2_control 块（`s622_real_ros2_control.xacro`）与控制器配置（`real_controllers.yaml`）
   **必须是 position 命令接口** —— 已按此实现（与仿真 gz 的 velocity 命令接口完全不同，互不干扰）
2. `/joint_states` velocity/effort 为 NaN 是**正常现象**，不是故障（真机验证已确认）
3. 任何依赖 velocity/effort 反馈的方案（速度模式控制、力矩控制、导纳/力控、
   速度前馈 PID 等）在真机上**不可直接实现**，需要额外手段（如位置差分估速）
4. 仿真中手臂/手指的 velocity state interface、velocity 命令接口都是 gz 插件机制，
   **不能作为真机能力的参照**

### 不动的文件

- 仿真全套：`gz_launch` 的 gz 插件 ros2_control 块、`dual_arm_controllers.yaml`（velocity 版）、现有 launch
- `command_server` 相关（IP 宏保持 58.2 默认）

## 5. 验证计划

1. **构建**：`colcon build` fairino_hardware + s622_moveit_config，确认无编译错误
2. **单臂 58.3**：
   - 检查网络连通（ping 58.3）
   - 起真机 launch（ip=192.168.58.3），确认插件 `RPC` 连接成功、`GetActualJointPosDegree` 读到真实关节角
   - 确认 `/joint_states` 正常、controller 可下发
   - 低速 ServoJ 单关节点动验证（**先确认急停/低速**）
3. **双臂**（单臂通过后）：
   - 双 ros2_control 块（left=58.2 / right=58.3）同时激活
   - 左右 `/joint_states` 各自真实、move_group ×2 可规划

## 6. 决策记录

| 决策点                | 结论                                               |
| --------------------- | -------------------------------------------------- |
| command_server 双臂化 | 本轮不做，保持单臂调试工具                         |
| 真机文件组织          | 新增独立真机文件，仿真不动                         |
| 验证起点              | 58.3 单臂先跑通（58.2 留给已验证的 robotarm 项目） |
