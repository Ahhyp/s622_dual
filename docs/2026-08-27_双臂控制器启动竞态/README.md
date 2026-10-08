# 双臂仿真问题集：spawner 竞态 / dual_arm 管线 / BT arm 参数（2026-08-27）

日期：2026-08-27
状态：✅ 双臂仿真（pick_place_dual + 双臂规划执行）恢复正常
关联：`docs/2026-08-27_仿真回归验证/`（残留 RSP、FastDDS SHM 两个环境陷阱）

---

## 0. 速查表（以后遇到直接对号入座）

| 现象 | 根因 | 修复 | 涉及文件 |
|------|------|------|----------|
| `A controller named 'left_arm_controller' was already loaded` + spawner exit 1 | spawner `--service-call-timeout` 默认 10s 太短，启动风暴下 CM 响应超时 → 误判重试 | spawner 加超时参数 60s（S1） | `s622_dual_arm.launch.py` |
| RViz 规划 `BiRRT*: start configuration in collision`（0 迭代、planning_time=0） | **dual_arm 组（12-DOF）走了 fairino 管线**（fairino 只支持 6-DOF 单臂） | RViz 用单臂组；或 dual_arm 组切 ompl 管线 | 客户端用法（代码已按此约束） |
| BT `set_gripper / move_to_pose service unavailable` | 单独 `bt_executor.launch.py` 启动缺 arm 参数 → arm='' 连无前缀 server | launch 加 `arm` 参数（默认 left） | `bt_executor.launch.py` |
| `class fairino_hardware/... does not exist`（仿真启动） | 残留真机 RSP 孤儿进程同名服务冲突，gz_ros2_control 拿到真机 URDF | kill 残留 RSP | 环境（见仿真回归执行记录） |
| `Failed init_port fastrtps_port7431: open_and_lock_file failed` | WSL2 FastDDS SHM 端口并发竞争 | 清理 /dev/shm + 重启（偶发）；严重时可禁 SHM | 环境（见仿真回归执行记录） |

---

## 1. 问题一：spawner 启动竞态（S1，已修复并保留）

### 1.1 现象
双臂仿真偶发：spawner 死亡 exit 1，`left_arm_controller` 从未 activate →
`left_arm_controller/follow_joint_trajectory` action 不存在 → RViz/BT 执行全失败。
用户还复现 `ros2 control list_controllers` 报 `Failed getting a result ... in 10.0`。

### 1.2 根因
`spawner.py`（Humble）`--service-call-timeout` 默认 **10.0s**。双臂 launch 启动风暴
（t=0 起 move_group×2 加载 fairino+ompl 双管线、YOLO CUDA、retime；controller t=8~9s 才 spawn）
把 controller_manager 的 service 响应拖到 10s 以上：
- spawner 第一次 load_controller 实际**成功**，但响应超时 → spawner 误判失败 → 重试
- CM 报 `A controller named 'left_arm_controller' was already loaded` → spawner exit 1
- 后续 3 个 controller 不再 spawn

证据：成功那次第一个 controller（left_arm_controller）从设置参数到 `Configured and activated`
约 7.65s（赶在 10s 前）；后三个各 0.3~0.4s。失败那次刚好超过 10s。
官方文档：spawner 的 service-call-timeout 就是为"启动期 CPU load 高、service 不能及时返回"加入的。

### 1.3 修复代码（S1，保留生效）
`src/gz_launch/launch/s622_dual_arm.launch.py` 两个 spawner 都加：

```python
# jsb_spawner（joint_state_broadcaster）和 arm_hand_spawner（4 controllers）同样处理
Node(
    package="controller_manager",
    executable="spawner",
    arguments=[
        "-p", dual_arm_controllers_yaml,
        "joint_state_broadcaster",   # 或 4 个 controller 名
        # S1：启动风暴下 CM service 响应可能 >10s，默认 --service-call-timeout=10.0
        # 导致 spawner 误判失败重试（already loaded）→ 后续 controller 不启动。拉长防御。
        "--service-call-timeout", "60.0",
        "--controller-manager-timeout", "60.0",
        "--switch-timeout", "60.0",
    ],
    parameters=[{"use_sim_time": True}],
    output="screen",
)
```

### 1.4 验证
S1 后（21:21 起）5 controller 全 active、两个 spawner 均 `finished cleanly`。

### 1.5 曾试过的 S2（时序调整）——已实施后回退
S2 内容：spawner 提前（robot 3s / controller 5s / arm 6s），move_group×2 / YOLO / retime
延后到 14s，rviz/planning_scene/servo 15s、arm_actions 16s、bt_executor 18s。
**结论**：S1 单独足够（21:21 验证），S2 只是让启动变慢 ~10s，无必要 → **已完整回退**，
launch 恢复 8/25 的 robotarm 对齐时序（robot 5s / controller 8s / arm 9s，move_group 立即启动）。
回退后 `git diff` 相对 HEAD 仅剩 S1 的 18 行新增。

---

## 2. 问题二：dual_arm 组 + fairino 管线 → start collision 误报（结论已定）

### 2.1 现象
RViz 里选 `dual_arm` 组（12-DOF 联合）plan & execute：
```
group_name=dual_arm, selected_planner=birrt*, tool_model=FLANGE
Planning obstacles aggregated: obs_count=1 filtered=0
Fairino plan failure: planner=birrt* planning_time=0.000000 path_points=0
  message=BiRRT*: start configuration in collision.
```
0 迭代、planning_time=0 → 不是真碰撞，是 fairino 对 12-DOF 状态不支持。

### 2.2 根因
- fairino 规划管线是 **6-DOF 单臂专用**（DH 模型 `fkine` q 为 6 维、只支持单臂组）
- `dual_arm` 组是 12-DOF 联合组，**必须用 ompl 管线**
- 规划管线统一（2026-08-25）后 move_group 默认管线 = fairino → dual_arm 组误走 fairino

**证据**：`dual_move_server.py` 注释明确："必须用 ompl 管线（M2.8 已验证 RRTConnect 可行）"，
`pipeline_id='ompl'`。M2.8 手递手交接（12-DOF OMPL planning）一直用 ompl。

**为什么之前能成功**：8/25-26 验证（BT）用 left_arm/right_arm 单臂组（6-DOF，fairino 支持）；
RViz 之前规划成功也是单臂组。这次选 dual_arm 组才踩中。

### 2.3 正确用法（无需改代码）
- 单臂规划：group = `left_arm` / `right_arm`，管线 fairino（默认）
- 双臂联合规划：group = `dual_arm`，**RViz 面板 Planning Pipeline 切 `ompl`**（或客户端 pipeline_id='ompl'）

### 2.4 遗留（可选代码加固）
fairino planner 对 >6 关节的 group 应显式拒绝（打清楚错误）而非报 start collision；
或 move_group 配置按组约束管线。当前靠客户端约定（dual_move_server 已强制 ompl）。

---

## 3. 问题三：bt_executor.launch.py 缺 arm 参数（已修复）

### 3.1 现象
单独启动 BT：`ros2 launch s622_bt_manager bt_executor.launch.py tree_file:=pick_place_dual.xml`
→ 日志 `arm parameter: '' (single-arm compat mode)` → `set_gripper / move_to_pose service
unavailable` → BT FAILURE（双臂 server 在 /left/* /right/* 下，BT 却连无前缀名）。

### 3.2 根因
`bt_executor.launch.py` 未声明/传递 `arm` 参数，节点默认 ''（单臂兼容模式）。
双臂环境必须 arm=left / right（s622_dual_arm.launch.py 内嵌的 bt_executor 已传 `"arm": "left"`）。

### 3.3 修复代码
`src/s622_bt_manager/launch/bt_executor.launch.py`：
```python
DeclareLaunchArgument('arm', default_value='left',
                      description='left / right / dual'),
# ...
'arm': LaunchConfiguration('arm'),   # 加入节点 parameters
```
用法：`ros2 launch s622_bt_manager bt_executor.launch.py tree_file:=pick_place_dual.xml arm:=left`

---

## 4. 环境陷阱（两个独立问题，另见仿真回归执行记录）

1. **残留 RSP 孤儿进程**：Ctrl+C 关 launch 后 `robot_state_publisher` 可能残留（父进程已退），
   同名 `/robot_state_publisher` 服务被随机路由 → gz_ros2_control 拿到错误 URDF
   （报 `fairino_hardware/... does not exist`）。排查：`ps -eo pid,cmd | grep robot_state_publisher`，kill 残留。
2. **WSL2 FastDDS SHM**：偶发 `Failed init_port fastrtps_portXXXX: open_and_lock_file failed`
   （多进程并发创建端口文件竞争）→ service 通信丢包。处置：清 `/dev/shm/fastrtps_*` + 重启；
   反复出现可禁 SHM（FASTRTPS_DEFAULT_PROFILES_FILE 指 UDPv4-only profile）。

---

## 5. 当前状态与遗留

- ✅ S1（spawner 超时）：保留生效
- ✅ S2（时序）：实施后回退（S1 足够）
- ✅ bt_executor arm 参数：已修
- ✅ pick_place_dual + 双臂规划执行：用户验证正常
- ⏸ S3：BT `CheckSystemReady` 是真检查（当前 dummy，不查 controller 状态）
- ⏸ S4：retime_server 双臂模型（当前默认加载单臂 model，双臂轨迹 joint 集合不匹配，需双臂 launch 传双臂 URDF）
- ⏸ fairino 对 dual_arm 组显式拒绝（见 2.4）
