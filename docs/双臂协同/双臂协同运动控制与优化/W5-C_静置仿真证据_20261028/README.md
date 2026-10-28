# W5-C 静置仿真原始证据（2026-10-28）

本目录是 C4.3 S4 / W5-C（标准样件在圆柱支撑上静置 **5 s 仿真时间**）的原始观测归档。
所有结论由本目录文件可直接复核；`/tmp` 中的任何副本都不是证据来源。

## 1. 这是什么 / 不是什么

**是**：在 `src/gz_launch/worlds/c43_s4_world.sdf` 中、无控制、无 attach/detach、重力开启、
物理未暂停的条件下，由 Gazebo Fortress 6.16.0 推进物理得到的 `c43_sample` 位姿序列，
以及按**仿真时钟**判定的测试区间。

**不是**：
- 不是 `≤ 2 mm` 阈值的**正式** Gate PASS。`2 mm` 在冻结规划中仍是 `[待标定]` 建议值，
  未获批准；本报告只给实测值与静置行为描述。
- 不是原生 Linux RT 证据，不能用于 C6 / S4-T2 / S4-T17。
- 不是真机或夹持力证据。

## 2. 复现命令

```bash
cd <repo>
python3 src/gz_launch/scripts/run_w5c_rest_test.py \
    --outdir docs/双臂协同/双臂协同运动控制与优化/W5-C_静置仿真证据_20261028 \
    --duration 5.0
# 退出码 0 = OBSERVED（区间 ≥ 5.0 s 仿真时间）；2 = 环境/加载失败；3 = 观测不足
```

脚本自身完成：启动 `ign gazebo -s -r`（server-only）、等待模型加载、等待 0.5 s 仿真时间接触稳定、
订阅 `/world/c43_s4_world/pose/info`、轮询 `/world/c43_s4_world/stats` 直到仿真时间推进 ≥ 5.0 s、
停止录制与仿真、解析并写出本目录文件。

## 3. 观测方法

| 项目 | 方法 |
|---|---|
| 仿真时间 | `/world/c43_s4_world/stats` 的 `sim_time`（每次轮询 `ign topic -e -n 1`） |
| 位姿 | `/world/c43_s4_world/pose/info` 连续录制，按每条消息 `header.stamp` 打时间戳（≈50 Hz） |
| 区间起点 | 模型全部加载后、等待 0.5 s 仿真时间接触稳定之后的第一个 stats 时刻 |
| 区间终点 | 仿真时间相对起点推进 ≥ 5.0 s 后的最后一个 stats 时刻 |
| 漂移 | 水平位移 `sqrt(x²+y²)` 的最大值与首末位移；Z 误差 `|z − 0.275|` |
| 姿态 | 相对首帧四元数的夹角 `2·acos(|q·q0|)` |
| 速度（估算） | 相邻消息中心位移 / 时间戳差；不含角速度 |

## 4. 实测结果（2026-10-28，WSL2 + Gazebo Fortress 6.16.0）

```
仿真区间      : 0.905 s → 6.080 s   （跨度 5.175 s 仿真时间）
位姿样本      : 256 条（≈49.5 Hz；CSV 内 1.015 s → 6.115 s，跨度 5.100 s）
水平漂移      : max sqrt(x²+y²) = 1.84e-17 m（数值噪声级，无滑动）
首末水平位移  : 1.37e-17 m
Z 范围        : 0.274967073 → 0.274999773 m
Z 误差        : 起点 |Δz| = 3.29e-05 m；终点 |Δz| = 2.27e-07 m；t≈4.0 s 后完全平直（100+ 样本 0 变化）
姿态变化      : 0 rad（四元数逐帧不变）
速度估算      : max 6.46e-05 m/s（仅出现在最初约 3 s 的接触沉降），mean 6.41e-06 m/s
支撑位姿      : 256 帧全部 (0,0,0)，无变化
桌面          : (0.4, 0, -1.015)（顶面平齐 world z=0）
Gazebo server : 退出码 0；结束统计 sim 6.585 s / 1317 迭代 / RTF 0.9973
```

解释：样件在初始零间隙接触（底面 0.25 = 支撑顶面 0.25）后，被接触求解器以 32.9 µm 的量级
推到平衡位置，并在约 3 s 内收敛到 0.2749998 m；此后位姿完全静止，无滑落、无穿透加剧、
无姿态变化、无非物理跳变。

## 5. 未覆盖 / 不可得

- **接触力消息不可得**：该 world 加载了 Contact system，但 `ign topic -l` 未出现
  `/world/c43_s4_world/contacts`，`ign topic -e` 亦无消息；因此"接触存在"由
  「样件静置 5.175 s 不下落、Z 恒定」推断，而非直接读取接触力。
- 未做 GUI 视觉检查（headless）。
- 未做长时间静置（>5 s）、未做扰动/倾斜、未做附着（属 W6）。
- 未在原生 Linux / RT 内核复现（与 W5 无关，属 C6）。

## 6. 文件清单

| 文件 | 内容 |
|---|---|
| `summary.json` | 机读汇总：schema `c43s4.w5c.rest_test.v1`、命令、哈希、区间、统计、状态 |
| `pose_samples.csv` | 窗口内 `c43_sample` 位姿序列（`sim_time_s,x,y,z,qw,qx,qy,qz`，256 行） |
| `pose_info_raw.txt` | `ign topic -e -t /world/c43_s4_world/pose/info` 的原始输出（522 KB） |
| `gazebo_server.log` | 本次 server 的全部 stdout/stderr（39 行，无 error/warn） |
| `environment.txt` | Gazebo 版本、内核、命令、退出码、所有哈希 |
