# S622 双臂 C4.3 S4 — 实验 1 到 2.5-D 交接说明（2026-11-02）

> 目的：给一个**没有先前对话上下文**的 GPT 一份可独立阅读的交接。
> 本文只描述实际跑过的实验、真实数据和当前边界；没有把仿真 weld 成功写成真实夹持 PASS。
>
> 仓库：`https://github.com/Ahhyp/s622_dual`
> 容器：`s622_run`；工作区 `/home/kz/s622_dual` → 容器 `/workspace`
> 环境：ROS 2 Humble + Gazebo Fortress 6.18
> 本轮分支：`master`（历史另有 `public` 镜像分支）

---

## 0. 一句话结论

- **安全 homing 已通过**：right-first + 50 mm vertical pre_grasp + 原 11 点下降，3/3 次样件接触为 0，样件零漂移；全机器人 FCL 检查无非法碰撞，最小跨臂间隙 **19.36 mm**，最小环境间隙 **5.39 mm**。
- **夹爪 JTC 正常**：无样件时 `±0.0251`、`±0.0200` 都能准确到达，action `SUCCEEDED`。
- **executor 闭合逻辑已修复**：原 `gripper_close_positions` 误设成张开值 `±0.0305`，且 2 mm 容差会提前通过；现在改为 `±0.0251`、0.5 mm 容差 + JTC SUCCEEDED + 0.2 s 稳定窗口。
- **仍未完成**：真实样件闭合到 `±0.0251` 时**没有接触**（实际内距约 50.35 mm）；继续到首次接触时只有 **left_finger1 单侧**接触，右侧未接触；FCL 显示接触点是**样件顶部边缘/角点**，不是侧面中部。
- **W7 整体仍是 `INTEGRATION_HOLD`**；真实摩擦夹持 `NOT_EVALUATED`，搬运仍不能称为真实抓取。

---

## 1. 物理对象与冻结基线

### 1.1 场景

| 对象 | 几何/参数 |
|---|---|
| 样件 | `0.12 × 0.05 × 0.05 m`，质量 0.35 kg，中心目标 `(0,0,0.275)`，底面 0.25 |
| 支撑 | 圆柱 `r=0.04 m`，高 `0.25 m`，顶面 `z=0.25`，中心 `(0,0,0)` |
| 桌面 | 桌面顶面 `z=0`（桌腿在下方） |
| 初始位姿 | `dual_home = [0,-1.5708,0,0,0,0]` 左右相同 |
| 冻结抓取位姿 | `q_grasp12`（左右各 6 关节，见下） |
| 左/右 base | world `x=+0.35 / -0.35`，yaw `π / 0` |
| 夹爪工具 | `grasp_frame`，wrist3→grasp 约 0.2168 m |

### 1.2 q_grasp12

```text
left  = [2.795036910, -1.706950858, 1.279963524, -1.143808993, -1.570796327, -0.346555744]
right = [0.346555744, -1.434641796, -1.279963524, -1.997783660, 1.570796327, -2.795036910]
```

`q_grasp12` 的 TCP world（FK，工具模型 GRIPPER）：

```text
left  TCP ≈ (0.0497, 0.0000, 0.3000)
right TCP ≈ (-0.0497, 0.0000, 0.3000)
```

### 1.3 50 mm pre_grasp（2.4 保留基线）

```text
h = 0.05 m；只抬高 world Z，X/Y/姿态不变
left pre  = [2.795036910, -1.604022235, 0.945042433, -0.911816525, -1.570796327, -0.346555744]
right pre = [0.346555744, -1.537570419, -0.945042433, -2.229776129, 1.570796327, -2.795036910]
```

两段路径：

1. `dual_home → pre_grasp`，**right-first**：右臂先、左臂保持 home；然后左臂、右臂保持 pre；
2. `pre_grasp → q_grasp12`，**原 11 个 IK 路点**，5 mm Z 步长，6 s。

手指全程张开 `+0.0305/-0.0305`。

---

## 2. 实验时间线与结论

| # | 实验 | 关键结果 | 判定 |
|---|---|---|---|
| 1 | T1 / 首次 W7 全流程 | 出生位姿 ≠ 冻结 grasp 链，目标误差 0.279 m；修正后 3.33e-4 m | 软件接线完成；weld-only；非物理抓取 PASS |
| 2 | 静态 weld/detach | 样品已在 `(-17.6,-13.8,286.2) mm`，未掉落不能证明支撑 | 不可用于支撑/抓取结论 |
| 2.1-A | dual_home 张开、无 homing | 216 行稳定 `(0,0.0895,274.9995) mm` | 稳定 |
| 2.1-B | q_grasp12 张开、无 homing | 216 行稳定 `(0.009,-0.005,274.999) mm` | 稳定；终点本身不是碰撞原因 |
| 3 | q_grasp12 张开搬运 | COMPLETE，样品 `(-20,0,275) mm`；手指仍 `±0.0305` | weld-only，不是夹持 |
| 2.2 | 无 weld / 无 QP homing | 24.630 s 首次 >1 mm；三维最大 24.58 mm | homing 路径碰撞 |
| 2.3 | sample contact sensor | 24.510 s 首触 `left_finger1`，z≈0.300 | 首个碰撞 link 确认 |
| 2.4 | 50 mm vertical pre_grasp + right-first | 3/3 样件接触 0、零漂移；FCL 0 非法；跨臂 19.36 mm、环境 5.39 mm | **PASS** |
| 2.5-A | 左夹爪直接 JTC | 0.0251/0.0200 实际到达；action SUCCEEDED | PASS；控制器无问题 |
| 2.5-B1 | executor 闭合（无样件在指间） | 实际 0.02513；JTC SUCCEEDED + 稳定窗口 | PASS |
| 2.5-B2 | 原样件，目标 ±0.0251 | 实际 0.0251749，内距 50.35 mm；无接触 | NO_CONTACT |
| 2.5-C0 | 无样件 cancel | cancel 后 0.23 s 停止；CANCELED | PASS（停止机制） |
| 2.5-C1 | 样件，目标 ±0.0250 | 首次接触单左；额外闭合 0.0065 mm；样件 0.018 mm | 单侧接触停止 PASS |
| 2.5-C2 | 样件，目标 ±0.0249 | 首次接触仍单左；额外闭合 0.0004 mm；样件 0.055 mm | 单侧接触停止 PASS |
| 2.5-D | 只读几何测量 | finger2 距样件 3.6 µm；接触在顶边角点；TCP Δx 49.7 mm、Δz 25 mm | 双指几何审计完成 |

---

## 3. 各实验细节

### 实验 1 —— T1 / 首次 W7 全流程（2026-10-28/29）

- 初始问题：Gazebo 出生位姿和冻结 grasp 链不一致，第一次全流程的目标误差达到 **0.279 m**。
- 修正 spawn/目标后，`t1_launch_after_spawnfix.log` 中 `e_obj=3.33e-4 m`，流程可以 COMPLETE。
- 但这次是 **weld-only** 的流程验证，手指并没有真正夹住样件；W7 仍是 `INTEGRATION_HOLD`。
- 证据：`t1_launch_before_fix.log`、`t1_launch_after_spawnfix.log`；相关提交 `7305178`、`a3c112d`。

### 实验 2 —— 静态 weld/detach

- `static_weld_detach.csv`（227 行）：样品从 `(-17.56,-13.81,286.22) mm` 到 `(-18.53,-15.22,286.00) mm`。
- 样品已经在支撑中心之外、且 z≈0.286，**没有掉落不能证明支撑**；当时夹爪仍卡在样品附近，接触项 `NOT_OBSERVED`。
- 结论：不能把“静态 detach 没掉”当成“抓取/支撑成功”。

### 实验 2.1-A / 2.1-B —— 静置对照

- 2.1-A：`dual_home` 出生、夹爪全开、不执行 homing；216 行稳定：
  `(0, 0.0000895, 0.2749995) m`。
- 2.1-B：直接用 `q_grasp12` 出生、全开、不执行 homing；216 行稳定：
  `(0.0000089, -0.0000053, 0.2749993) m`。
- 结论：**终点 q_grasp12 本身不是碰撞原因**；问题在从 dual_home 到 q_grasp12 的路径。

### 实验 3 —— q_grasp12 张开搬运

- 结果：`COMPLETE`；最终样品 `(-20, 0, 275) mm`，`e_obj≈0.0005 m`。
- 日志明确：`fingers at close targets (0.030500, -0.030498)`，夹爪一直张开。
- 样品只被 **weld** 带着走，不是摩擦夹持。

### 实验 2.2 —— 无 weld、无 QP 的纯 homing

- 设置：`dual_home`、夹爪全开、`C43_HOMING_NO_QP=1`、不 attach/detach、100 Hz trace。
- 首次持续 >1 mm：**sim 24.630 s**，样品 `(-1.117,-2.261,275.803) mm`。
- 最大位移（sim 44.160）：样品 `(-16.385,-15.646,284.384) mm`；
  X/Y/Z = `-16.39/-15.74/+9.39 mm`；水平 **22.72 mm**，三维 **24.58 mm**。
- 结论：只跑 homing 就推样件，问题不在 weld/QP；`target` 列未捕获，后续用显式目标/contact sensor 补。

### 实验 2.3 —— 首次碰撞 link

- 临时 sample contact sensor + 诊断 world。
- 首触：**sim 24.510 s**，`c43_sample::base_link::collision` ↔
  `s622_dual_arm::left_finger1::left_finger1_collision`，接触 z≈0.300。
- 比可见偏移 24.630 s 早 **0.12 s**。
- 结论：首个碰撞 link = **left_finger1**；当时夹爪张开，接触点位于样件顶面附近。

### 实验 2.4 / 2.4-B / 2.4-C —— vertical pre_grasp + right-first

**2.4 目标**：保持 q_grasp12，只把 TCP 抬高 `h=0.05 m`，生成 pre_grasp，再低速下降。

- IK：`fairino_planning_core` FK/IK，TCP X/Y/姿态不变，Z+0.05；pre 与 q_grasp 关节距离约 0.420 rad。
- 2.4 初版：同时运动；FCL 发现 stage1 89/100 附近 `right_finger2 ↔ left_finger1` 双臂碰撞（h=0.05，2 帧；h=0.10 虽无碰撞但间隙仅 0.17 mm）。
- 2.4-C：只改 stage1 时序为 **right-first**：
  1. 右臂 home→pre，左臂 home 不动；
  2. 左臂 home→pre，右臂 pre 不动；
  3. 原 11 点下降。
- FCL（C4.3 xacro URDF、原 SRDF、张开手指、样件/支撑/地面）：
  - 跨臂非法碰撞 **0**；最小跨臂间隙 **19.36 mm**；
  - 环境碰撞 **0**；最小环境间隙 **5.39 mm**；
  - 实际 100 Hz 反馈 FCL：跨臂 0、环境 0，最小跨臂 19.376 mm。
- 891 个 whole-robot self contact 已逐对审计：全部是同一臂相邻/固定结构 + floor/base，**没有 `left_* ↔ right_*` 跨臂对**；没有扩大 SRDF ACM。
- Gazebo 3/3（rf1/rf2/rf3）：样件始终 `(0,0,0.274999773)`，机器人-样件接触 0，两段 JTC 成功。
- 回归：**190 tests / 0 failures**。
- 结论：2.4-C **PASS**（受测几何/仿真条件）。

### 实验 2.5-A —— 左夹爪直接 JTC

- 无样件 world；right-first 到 q_grasp12；直接 `left_hand_controller`。
- OPEN 0.0305 / CLOSE 0.0251 / CLOSE 0.0200 / REOPEN 全部 `status=4 SUCCEEDED, error_code=0`。
- 实际：
  - `0.0251` → `0.0251002/-0.0251001`（误差约 0.0002 mm）；
  - `0.0200` → `0.0201013/-0.0201013`（误差约 0.1013 mm）。
- 结论：**控制器正常**；问题在 executor 配置/判定。

### 实验 2.5-B1 —— 修复后的 executor 闭合

- 修复：
  1. launch `gripper_close_positions` → `[0.0251,-0.0251]`，并加 close==open 的启动拒绝；
  2. executor 完成判定 = JTC SUCCEEDED + 0.5 mm 容差 + 0.2 s 稳定窗口。
- 用样件移开的 world（保留启动 weld 链），executor 驱动 GRASP：
  - `closing left fingers` → `goal accepted` → `goal finished: SUCCEEDED`；
  - `fingers at close targets (0.025130, -0.025132)`；之后才 attach。
- 结论：**B1 PASS**（executor 确实下发闭合并等待完成）。

### 实验 2.5-B2 —— 原样件 ±0.0251

- 原样件、right-first、QP inactive、直接慢速左 JTC（5 s），不 attach。
- `SLOW_CLOSE 0.0251`：SUCCEEDED；实际 `+0.0251749/-0.0251749`；内距 **50.3498 mm**（两关节绝对值和；不是 mesh 真实指面距离）。
- 样件 `(0,0,0.274999773)` 不变；机器人-样件 contact 0。
- 结论：**NO_CONTACT**；不能算夹持成功；没有继续缩到 0.0245/0.0230 硬压。

### 实验 2.5-C0/C1/C2 —— 接触限幅诊断

独立诊断，不改 executor，不 attach。

- **C0（无样件）**：
  - close 1.5 s 后 cancel；cancel sent/confirmed 3.551/3.554 s；
  - JTC `status=5 CANCELED`；物理停止 3.780 s（约 **0.23 s** 后）；
  - final fingers `0.029506/-0.029504`。
  - 结论：cancel→停止链路 PASS。
- **C1（样件，目标 ±0.0250）**：
  - 首触 sim **125.390 s**；`left_finger1` 单侧；
  - 首触 fingers `0.025089/-0.025089`；
  - cancel sent/confirmed 10.010/10.011 s；JTC CANCELED；停止 10.230 s（约 **0.22 s**）；
  - 额外闭合约 **0.0065 mm**；样件位移约 **0.018 mm** 三维。
- **C2（样件，目标 ±0.0249）**：
  - 首触 sim **57.200 s**；仍是 `left_finger1` 单侧；
  - 首触 fingers `0.025082/-0.025082`；
  - cancel/stop 10.346/10.570 s；额外闭合约 **0.0004 mm**；样件位移约 **0.055 mm** 三维。
- 结论：**首次接触停止机制 PASS**，但 **双侧接触 NOT_ESTABLISHED**；右侧没有确认接触。
- 重要：C1/C2 的 contact sim time 与 cancel wall time 不是同一时钟；`cancel→stop` 的 0.22 s 是墙钟有效值，端到端延迟尚未统一时钟验证。

### 实验 2.5-D —— 双指几何只读测量

FCL 用 C4.3 xacro URDF 的手指 collision mesh 对样件盒测量；q_grasp12 + C1/C2 手指位置。

- C1 首触状态：
  - `left_finger1` 相交；
  - **`left_finger2` 距离 = 0.003593 mm（3.6 µm）**；
  - 最近点样件局部 `(0.059911,0.025000,0.025000)` = **+X 端 +Y 侧 +Z 顶的交界/顶角**；
  - right_finger1/2 分别 5.409 / 5.414 mm，未接触。
- C2 首触状态：FCL 显示 `left_finger1` 与 `left_finger2` 都相交；但首事件日志只有 left_finger1，**传感器全对捕获未完成，不能据此宣称双侧**。
- `left_grasp_frame = (0.049692,0.000003,0.300004)`；
  sample center `(0,0,0.275)`；
  TCP delta `(+49.692,+0.003,+25.004) mm`；
  finger-origin midpoint delta `(+49.692,+0.003,+76.004) mm`；
  grasp frame 与 sample identity 夹角 **π rad**。
- 结论：不是 Y 向大偏置（Y 仅 0.003 mm），而是**夹爪在样件端部、从顶部区域闭合，接触顶边/角点**；这才是单侧接触的核心几何原因。

---

## 4. 当前正确配置（不要回退）

| 项 | 当前值 |
|---|---|
| 初始位姿 | `dual_home` |
| homing | `pre_grasp` h=0.05 → 原 11 点下降 |
| stage1 时序 | **right-first**（不是 simultaneous，也不是 left-first） |
| 手指 | 全程张开 `±0.0305` |
| 诊断 homing | `C43_HOMING_NO_QP=1`（QP inactive） |
| executor 闭合目标 | `±0.0251` |
| executor 容差 | `0.0005` + JTC SUCCEEDED + `0.2 s` 稳定 |
| 接触实验 | 不 attach/weld；首次接触立即 cancel |
| 样件/支撑/ q_grasp12 | 冻结，未改 |

**禁止**：

- 回到 `q_grasp12` 作为出生位姿（会穿模，早期“稳定”是卡在物体里）；
- 用 simultaneous/left-first homing（会撞/互碰）；
- 把 `gripper_close_positions` 设回 `±0.0305`；
- 为了得到接触继续把统一目标缩到 0.0245/0.0230 硬压；
- 把 weld 搬运成功当成真实摩擦夹持。

---

## 5. 已验证 / 未验证

### 已验证

- 安全 homing：3/3、样件零漂移、FCL 无非法碰撞。
- 左夹爪 JTC 位置控制。
- executor 闭合目标/完成判定修复；B1 实际 0.02513。
- 首次接触 cancel/stop：C0/C1/C2，0.22–0.23 s 停止，额外闭合 ≤0.0065 mm。
- 接触传感器能识别 `left_finger1`；C1/C2 样件位移 ≤0.055 mm。
- 2.5-D 手指 mesh 几何测量。

### 未验证/未通过

- **双侧手指接触**：C1 只有左侧；C2 FCL 双指相交但首事件只有左侧，需全对捕获。
- **真实摩擦夹持**：没有证明能靠摩擦力抬起 0.35 kg。
- **完整执行器整合**：contact-triggered cancel 还没接入正式 GRASP。
- **统一时钟**的 contact→cancel→stop 端到端延迟；首触时手指速度。
- 夹爪顶部/端部几何是否需要 TCP 垂直修正；q_grasp12 冻结，不能盲改。
- W7 整体：`INTEGRATION_HOLD`。

---

## 6. 下一步建议（按优先级）

1. **D3：垂直/顶边对中诊断**  
   现在证据指向夹爪在样件顶边/端部闭合，而非 Y 向偏移。先在诊断配置里尝试**垂直 grasp-frame 偏移**（不是随机 XY），用 FCL 检查，不改冻结 q_grasp12。
2. **D4：全接触对捕获**  
   在 cancel 前记录同一条 contact 消息里的所有 collision 对；统一 `/world/.../clock` 与单调墙钟；记录首触手指速度。
3. 只有确认双侧接触且稳定，才考虑把 contact-triggered cancel 接入 executor。
4. 再单独验证摩擦夹持/抬升；在此之前不要 auto attach weld。

---

## 7. 证据索引

目录：`docs/双臂协同/双臂协同运动控制与优化/T1_QP满速定位_20261029/`

### 早期

- `t1_launch_before_fix.log` / `t1_launch_after_spawnfix.log`
- `static_weld_detach.csv`
- `static_weld_home_open.csv` / `static_weld_qgrasp_open_gui.csv`
- `qgrasp_open_run1.json`（实验 3）
- `homing_trace.csv`（2.2）
- `homing_trace_contact.csv` / `first_contact_summary.txt` / `contact_first_block.txt`（2.3）

### 2.4

- `EXPERIMENT_2_4_PRE_GRASP.md`
- `pre_grasp_targets.json`、`gen_pre_grasp.cpp`
- `collision_2_4b_gazebo_urdf.txt`
- `collision_2c_simultaneous.txt` / `collision_2c_left_first.txt` / `collision_2c_right_first.txt`
- `collision_2c_rf1_actual.txt`
- `rf1_trace.csv`
- 3 次运行：`/tmp/rf{1,2,3}_world.log`、`/tmp/rf{1,2,3}_contact.txt`（临时文件，若未归档则报告中的数值为准）

### 2.5

- `EXPERIMENT_2_5A_GRIPPER.md`
- `gripper_2_5a.csv`、`gripper_2_5b.csv`
- `EXPERIMENT_2_5C_CONTACT_STOP.md`
- `contact_stop_c0.json` / `c1.json` / `c2.json`
- `contact_stop_c0.csv` / `c1.csv` / `c2.csv`
- `EXPERIMENT_2_5D_GEOMETRY.md`
- `src/dual_arm_pose_optimization/src/c43_geometry_probe.cpp`
- `src/gz_launch/scripts/c43_contact_stop_probe.py`

### 关键代码

- `src/gz_launch/scripts/c43_homing.py`：right-first + pre_grasp + 11 点下降
- `src/gz_launch/launch/c43_s4_full_flow.launch.py`：close 0.0251、close!=open 校验
- `src/dual_arm_transport_executor/src/executor_node.cpp`：JTC SUCCEEDED + 0.0005 容差 + 0.2 s 稳定
- `src/dual_arm_pose_optimization/src/c43_path_check_main.cpp`：全机器人 FCL/实际 trace
- `src/dual_arm_pose_optimization/src/c43_2c_sequence_check.cpp`：时序/碰撞对审计
- `src/dual_arm_pose_optimization/src/c43_geometry_probe.cpp`：2.5-D finger mesh 测量
- 诊断资产：`src/gz_launch/worlds/c43_s4_world_contact.sdf`、
  `c43_s4_world_no_sample.sdf`、`c43_s4_world_sample_away*.sdf`、
  `src/gz_launch/models/c43_sample_contact/`

---

## 8. 复现命令（容器内）

```bash
cd /workspace
source /opt/ros/humble/setup.bash
source install/setup.bash

# 安全 homing（headless，诊断 world，no weld/QP）
C43_S4_WORLD_FILE=/workspace/install/share/gz_launch/worlds/c43_s4_world_contact.sdf \
C43_PRE_GRASP_HOMING=1 C43_HOMING_NO_QP=1 C43_REQUIRE_GRASP_CHECK=0 \
ros2 launch gz_launch c43_s4_full_flow.launch.py gui:=false

# GUI（llvmpipe，避免黑屏）
docker exec -e LIBGL_ALWAYS_SOFTWARE=1 -e __GLX_VENDOR_LIBRARY_NAME=mesa s622_run ...
```

回归：

```bash
colcon test --merge-install --packages-select dual_arm_transport_executor \
  --test-result-base build/dual_arm_transport_executor
colcon test-result --test-result-base build/dual_arm_transport_executor --verbose
# 190 tests, 0 failures
```

---

## 9. 交接注意事项

1. 不要把“实验 2.2 的最大位移 16.4 mm”当成三维位移；完整值是水平 22.72 mm / 三维 24.58 mm。
2. `50.3498 mm` 是两 finger 关节绝对值的和，不应直接当作真实指面 mesh 间距；2.5-D 用 mesh 看到接触点是顶角。
3. `self_collisions=891` 是 whole-robot 相邻/固定结构接触；逐对审计后没有跨臂非法对；不要批量扩大 SRDF 豁免。
4. 2.5-C 的 C1/C2 是**单侧左侧接触**，不能写成双侧夹持成功。
5. 当前最有价值的问题不是“JTC 能不能到位”，而是“夹爪顶边/端部几何如何进入样件侧面中部”。
6. 代码未 commit 前的回退快照在宿主机 `/home/kz/s622_backup_*`；本轮已 push 的内容以 GitHub 为准。
