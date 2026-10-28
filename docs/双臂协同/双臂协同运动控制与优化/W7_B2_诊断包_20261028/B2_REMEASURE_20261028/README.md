# B-2 复测：修正探针后的真实关节位移（2026-10-28 20:00）

## 为什么要有这一轮

`../DIAGNOSTICS.md` 原 §2/§3 的结论是「`/joint_states` 12 个关节位移全部为 0 ⇒ 另有指令压住关节」。
该结论建立在探针的测量错误上：

```python
js_end = {k: v for k, v in (n.js or {}).items() if k in NAMES}   # copy of n.js
d = {k: js_end[k] - n.js[k] for k in NAMES}                       # same dict -> identically 0
```

`js_end` 就是 `n.js` 的拷贝，相减恒为 0 —— 机器人无论怎么动，脚本都报 `js_maxabs_delta = 0.0`。
外部复核（GPT，2026-10-28）独立复现了这一点；本目录是修正后的实测。

## 修正后的探针

`probe2.py`：

* 保存**全部**样本（`/joint_states` 与 `TransportStatus`），按窗口逐关节取 `min/max/span`，
  而不是端点差（端点差会把「动了又回原位」误判成没动）；
* 同时记录 `status.q_state`（控制器读到的状态接口）、`status.q_cmd`（控制器计算的目标）与
  `/joint_states`（Gazebo 侧实际关节），三者对照；
* 窗口起点用本轮测试的 `mark` 索引，不再混入启动前的样本。

自测（`selftest.txt`，对合成序列跑同一个 `window_stats()`）：

| 序列 | 新探针 span | 旧探针（端点差） |
|---|---|---|
| 静止 | 0.000 | 0.000 |
| 单关节单向移动 0.4 rad | 0.400 | 0.400 |
| 往返 0.2 rad（末态回原位） | **0.199** | **0.000** ← 旧探针的盲点 |

## 实测（`probe_output.txt`，`ROS_DOMAIN_ID=93`，headless Fortress 6.16）

一次被接受的 `start_transport`（4 相位 × 0.5 s）：

| 指标 | 值 |
|---|---|
| `/joint_states` 单关节最大位移 `JS_SPAN_MAX` | **2.302684 rad**（`right_j2`） |
| 控制器状态接口 `QSTATE_SPAN_MAX` | 2.296908 rad |
| 控制器目标 `QCMD_SPAN_MAX` | 2.328111 rad |
| `JS` 对 `QSTATE` 的最大偏差 | 0.024000 rad |
| 控制器终态 | `COMPLETE`, `t_traj=2.0` |

十二个关节的 span 全部在 0.94–2.30 rad 之间（见 `probe_output.txt` 的 `js_spans`）。

`probe_output_rejected_start.txt` 是同一探针在**姿态门禁拒绝**（`start pose deviation 2.488524 rad`）
情况下的对照：被拒后关节只残余 0.0026 rad 的沉降，说明探针不会把静止报成运动。

## 结论

* **B-2 的旧表述（「命令写不进硬件」）不成立**：控制链 QP → ros2_control → Gazebo 是通的，
  关节按命令运动了约 2.3 rad。
* 旧证据是**测量错误**，已作废并在此更正；原 `DIAGNOSTICS.md` 保留原文并在顶部标注。
* 这条更正**不**表示 W7 已通过：`status COMPLETE` 只说明轨迹状态机到达终态，
  不等于物理抓取/搬运成功；执行器全流程（GRASP→附着→LIFT→…）仍未跑通。

## 复现

```bash
ros2 launch gz_launch c43_s4_full_flow.launch.py headless:=true   # 需要干净的初始姿态
python3 probe2.py                                                 # 会自行调用 ~/start_transport
```

注意：若机械臂已被上一次运行带走，`start_transport` 会因姿态偏差被拒（`probe_output_rejected_start.txt`），
必须重启仿真让关节回到 `initial_positions`。
