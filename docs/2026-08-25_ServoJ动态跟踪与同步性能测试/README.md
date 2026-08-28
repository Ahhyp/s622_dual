# 双臂机械臂 ServoJ 动态跟踪与同步性能测试规划

## 1. 测试目标

本测试用于量化双臂机械臂控制链的动态跟踪能力和双臂同步性能，为后续 MoveIt2 轨迹执行、双臂交接、12 轴协同规划和闭链协作提供控制层依据。

测试分为两个核心部分：

1. **单臂正弦跟随测试**

   * 测量不同运动频率下的跟踪误差
   * 测量幅值衰减
   * 测量相位滞后
   * 估算 command → feedback 的等效延迟

2. **双臂同步跟随测试**

   * 测量左右臂实际运动之间的相对延迟
   * 测量相对延迟的随机波动
   * 计算 jitter 的均值、标准差和 3σ
   * 判断后续双臂协同运动能采用多紧的同步策略

测试必须遵循：

**Gazebo 仿真验证 → 单臂真机低风险测试 → 单臂全频段测试 → 双臂真机同步测试。**

---

# 2. 总体测试阶段

## Phase A：Gazebo 测试链路验证

目标不是证明 Gazebo 能模拟真实电机动态，而是确认：

* 正弦轨迹生成正确
* 250 Hz 指令发送正确
* 时间戳记录正确
* `/joint_states` 反馈采集正确
* CSV/rosbag 数据完整
* 相位差、幅值、RMS、互相关分析代码正确
* 左右臂 joint 映射正确
* 测试过程能自动开始、停止并生成报告

Gazebo 阶段全部通过后才允许进入真机。

---

## Phase B：单臂真机低风险验证

只测试：

* 一只机械臂
* 一个关节
* 小振幅
* 0.5 Hz
* 短时间

确认 ServoJ 行为、安全停止和反馈正常。

---

## Phase C：单臂真机频率扫描

测试：

* 0.5 Hz
* 1 Hz
* 2 Hz
* 5 Hz

获取完整频率响应趋势。

---

## Phase D：双臂同步测试

左右臂执行完全相同的轨迹，通过实际反馈计算：

* 平均相对延迟
* jitter
* σ
* 3σ

---

# 3. 测试信号

单个关节使用正弦位置轨迹：

[
q_{cmd}(t)=q_0+A\sin(2\pi ft)
]

其中：

* (q_0)：测试关节中心位置
* (A)：运动振幅
* (f)：正弦运动频率

测试频率：

| Test | Frequency | Period |
| ---- | --------: | -----: |
| F1   |    0.5 Hz |  2.0 s |
| F2   |      1 Hz |  1.0 s |
| F3   |      2 Hz |  0.5 s |
| F4   |      5 Hz |  0.2 s |

注意：

**这里的 0.5/1/2/5 Hz 是机械臂目标位置往复运动的频率，不是 ServoJ 指令发送频率。**

ServoJ 发送周期假设为：

[
f_s=250Hz
]

即：

[
T_s=4ms
]

所以：

| Motion frequency | Samples per cycle @ 250 Hz |
| ---------------- | -------------------------: |
| 0.5 Hz           |                        500 |
| 1 Hz             |                        250 |
| 2 Hz             |                        125 |
| 5 Hz             |                         50 |

---

# 4. Gazebo 阶段

## 4.1 Gazebo 测试目的

Gazebo 阶段主要验证测试工具，而不是追求与真机相同的动态结果。

尤其不要把 Gazebo 得到的：

* 2 ms delay
* 0.01° tracking error
* 0.1 ms jitter

当作真机能力。

Gazebo 的意义是：

> 在零硬件风险条件下把整个实验方法跑通。

---

## 4.2 Gazebo 单臂测试

首先只测试：

```text
left_j1
```

或选择机械臂空间最宽裕的关节。

建议初始：

```text
Amplitude A = 0.05 rad ≈ 2.86°
Center q0 = 当前安全位置
Frequency = 0.5 Hz
Duration = 10 s
```

成功后依次：

```text
0.5 Hz
1 Hz
2 Hz
5 Hz
```

每组建议至少：

```text
10~20 个完整周期
```

因此建议：

| Frequency | Suggested duration |
| --------- | -----------------: |
| 0.5 Hz    |            30–40 s |
| 1 Hz      |               20 s |
| 2 Hz      |               15 s |
| 5 Hz      |               10 s |

数据分析时舍弃前 2~3 个周期，避免启动瞬态影响。

---

# 5. 必须记录的数据

每一个 ServoJ 控制周期至少记录：

```text
steady_clock timestamp
ROS timestamp
command joint position
actual joint position
actual joint velocity（如 SDK 有）
ServoJ call start timestamp
ServoJ call end timestamp
ServoJ return code
```

如果 SDK 可以提供，还要记录：

```text
robot controller timestamp
joint current
joint torque
robot state
communication sequence number
```

建议 CSV 至少包含：

```text
t
cmd_j1
actual_j1
cmd_j2
actual_j2
...
servoj_call_us
feedback_age_ms
```

双臂测试额外记录：

```text
left_feedback_timestamp
right_feedback_timestamp
```

---

# 6. 单臂分析指标

## 6.1 Tracking Error

定义：

[
e(t)=q_{cmd}(t)-q_{actual}(t)
]

至少计算：

### RMSE

[
RMSE=\sqrt{\frac{1}{N}\sum e_i^2}
]

### MAE

[
MAE=\frac{1}{N}\sum |e_i|
]

### Maximum Error

[
e_{max}=\max |e_i|
]

最终每个频率得到：

| Frequency | RMSE | MAE | Max error |
| --------- | ---: | --: | --------: |
| 0.5 Hz    |      |     |           |
| 1 Hz      |      |     |           |
| 2 Hz      |      |     |           |
| 5 Hz      |      |     |           |

---

# 7. 幅值衰减测试

从实际轨迹拟合实际正弦振幅：

[
A_{actual}
]

计算：

[
G=\frac{A_{actual}}{A_{cmd}}
]

例如：

```text
A_cmd    = 5.0°
A_actual = 4.7°
```

则：

[
G=0.94
]

代表振幅下降约：

[
6%
]

最终得到：

| Frequency | Amplitude ratio |
| --------- | --------------: |
| 0.5 Hz    |                 |
| 1 Hz      |                 |
| 2 Hz      |                 |
| 5 Hz      |                 |

推荐画图：

```text
X axis: Frequency (Hz)
Y axis: Amplitude Ratio
```

---

# 8. 相位滞后与等效延迟

真实反馈通常会落后于指令：

```text
Command:   /¯\_/¯\_/¯\_
Actual:      /¯\_/¯\_/¯\_
             ↑
             delay
```

可以通过正弦拟合得到 phase lag：

[
\phi
]

换算成时间：

[
\tau=\frac{\phi}{2\pi f}
]

最终记录：

| Frequency | Phase lag | Equivalent delay |
| --------- | --------: | ---------------: |
| 0.5 Hz    |           |                  |
| 1 Hz      |           |                  |
| 2 Hz      |           |                  |
| 5 Hz      |           |                  |

需要特别注明：

> Equivalent delay 并不是纯网络延迟。

它综合包含：

* ROS 2 调度
* controller_manager
* hardware_interface
* SDK
* Ethernet
* ServoJ
* 机器人内部伺服
* 电机动态
* feedback 通道

因此更准确的名称是：

**command-to-motion equivalent delay**

---

# 9. 真机测试前安全检查

Gazebo 测试全部通过后才进入真机。

真机第一轮不得直接测试：

```text
6 轴
双臂
5 Hz
大幅运动
```

必须逐步升级。

测试前确认：

* [ ] 急停有效
* [ ] 软件 stop 有效
* [ ] ServoJ 停发行为已经测试
* [ ] 测试关节远离机械限位
* [ ] 周围无人员进入机械臂工作区
* [ ] 单关节运动不会导致腕部或夹爪碰桌
* [ ] 软件设置关节位置范围
* [ ] 软件设置最大单周期 position delta
* [ ] 通信异常立即停止
* [ ] ServoJ return error 立即停止
* [ ] feedback 超时立即停止
* [ ] 测试程序运行时间有硬上限

---

# 10. 真机测试升级顺序

## Stage R1

```text
Arm: left
Joint: j1
Amplitude: ±1°
Frequency: 0.5 Hz
Duration: 5–10 s
```

主要验证：

```text
command 对不对
方向对不对
反馈对不对
ServoJ 正常不正常
stop 正常不正常
```

---

## Stage R2

如果完全正常：

```text
Amplitude: ±2~3°
Frequency: 0.5 Hz
Duration: 20–30 s
```

开始正式记录数据。

---

## Stage R3

逐渐增加频率：

```text
0.5 Hz
↓
1 Hz
↓
2 Hz
↓
5 Hz
```

每次提高频率前检查：

```text
peak velocity
peak acceleration
tracking error
robot vibration
SDK error
```

---

# 11. 正弦轨迹本身的速度和加速度限制

这个非常重要。

因为：

[
q=A\sin(2\pi ft)
]

所以最大速度：

[
v_{max}=2\pi fA
]

最大加速度：

[
a_{max}=(2\pi f)^2A
]

因此频率增加以后，加速度增长非常快。

例如相同振幅下：

```text
1 Hz → 2 Hz
```

最大加速度变成：

```text
4 倍
```

而：

```text
1 Hz → 5 Hz
```

最大加速度变成：

```text
25 倍
```

所以绝对不能简单理解成：

> 5 Hz 只是比 1 Hz 快五倍。

从机械负载角度，它激进得多。

因此进入真机前，测试程序必须自动计算：

```text
peak velocity
peak acceleration
```

如果超过机器人限制：

```text
拒绝启动测试
```

---

# 12. 是否测试全部六轴

第一阶段不用。

建议先选择：

```text
j1
j3
j5
```

分别代表不同动力学特征。

如果时间充足，再扩展：

```text
j1~j6
```

但正式结果最好至少覆盖：

* 基座大关节
* 中部主运动关节
* 腕部小关节

因为它们的动态性能可能明显不同。

---

# 13. 双臂同步测试

单臂测试完成后才开始。

先让两个对应关节执行：

[
q_L(t)=q_0+A\sin(2\pi ft)
]

[
q_R(t)=q_0+A\sin(2\pi ft)
]

即：

```text
left_j1
right_j1
```

执行完全相同的目标轨迹。

注意这里真正比较的是：

```text
actual_left
vs
actual_right
```

而不只是：

```text
command_left
vs
command_right
```

---

# 14. 双臂测试顺序

首先：

```text
A = ±1~2°
f = 0.5 Hz
```

然后：

```text
0.5 Hz
1 Hz
2 Hz
```

只有确认安全和系统动态允许以后，再考虑：

```text
5 Hz
```

双臂不需要为了“测试完整”强行做到 5 Hz。

如果单臂测试已经证明 5 Hz 跟踪严重恶化，那么双臂 5 Hz 的工程价值很低。

---

# 15. 双臂相对延迟

记录：

[
q_L(t)
]

和：

[
q_R(t)
]

计算 cross-correlation：

[
R_{LR}(\tau)
]

求：

[
\tau^*=\arg\max R_{LR}(\tau)
]

如果得到：

```text
τ = +2.8 ms
```

规定符号以后，例如：

```text
positive = right arm lags left arm
```

即可解释为：

> 右臂相对于左臂平均晚约 2.8 ms。

---

# 16. Jitter 测量

不要只对整段 60 秒数据做一次互相关。

应该把数据分成多个时间窗口。

例如：

```text
每 2 s / 5 s 一个窗口
```

分别计算：

```text
Δt1
Δt2
Δt3
...
ΔtN
```

然后求：

[
\mu=mean(\Delta t)
]

[
\sigma=std(\Delta t)
]

以及：

[
3\sigma
]

例如：

```text
Mean relative delay = +2.1 ms
σ = 0.55 ms
3σ = 1.65 ms
```

最终报告：

> Right arm lag relative to left arm: +2.1 ms mean, 1.65 ms 3σ jitter.

---

# 17. 建议双臂测试重复方式

每个频率不要只测试一次。

建议：

```text
每个 frequency：
3~5 runs

每次：
30~60 s
```

最终分别计算：

```text
run 内 jitter
run 间 mean delay variation
总体 3σ
```

这样才能判断系统是不是偶尔出现异常 scheduler/DDS/SDK spike。

---

# 18. 双臂同步结果判据

建议将 M1 的决策结果定义为：

| Relative jitter 3σ | Engineering interpretation |
| ------------------ | -------------------------- |
| < 2 ms             | 同步性能很好，可以继续研究更紧耦合协同        |
| 2–10 ms            | 适合互避、交接、松耦合协同              |
| > 10 ms            | 不建议把系统价值建立在高精度同步上          |

这个数字不是机器人的“安全认证指标”。

它是：

**后续架构选择的实验依据。**

---

# 19. 额外建议：同时测 ServoJ 调用耗时

每次：

```cpp
hardware_interface::write()
```

或 ServoJ SDK 调用前后记录：

```text
t_start
t_end
```

得到：

[
T_{call}=t_{end}-t_{start}
]

统计：

```text
mean
median
P95
P99
max
```

这个数字很重要。

例如：

```text
250 Hz
```

意味着控制周期只有：

```text
4 ms
```

如果 ServoJ：

```text
平均 0.5 ms
P99 1.2 ms
```

很好。

如果：

```text
P99 7 ms
```

那你的 250 Hz hardware interface 就存在严重问题。

---

# 20. 250 Hz 周期本身也要测

不要只相信：

```text
controller_manager update_rate: 250
```

实际记录每轮调用间隔：

[
T_i=t_i-t_{i-1}
]

理论值：

[
4ms
]

统计：

```text
mean period
σ
3σ
P95
P99
max
deadline miss count
```

例如最终：

```text
Nominal loop: 250 Hz
Mean period: 4.01 ms
P99: 4.38 ms
Maximum: 6.21 ms
Deadline misses (>4 ms): 0.8%
```

这个对后面的双臂同步分析非常有价值。

---

# 21. 推荐输出图表

每个测试自动生成：

### Plot 1 — Command vs Actual Position

```text
X: Time (s)
Y: Joint Position (rad)
```

显示：

```text
command
actual
```

---

### Plot 2 — Tracking Error

```text
X: Time (s)
Y: Position Error (rad)
```

---

### Plot 3 — Amplitude Ratio vs Frequency

```text
X: Frequency (Hz)
Y: Amplitude Ratio
```

---

### Plot 4 — Equivalent Delay vs Frequency

```text
X: Frequency (Hz)
Y: Equivalent Delay (ms)
```

---

### Plot 5 — RMSE vs Frequency

```text
X: Frequency (Hz)
Y: RMSE (rad)
```

---

### Plot 6 — Left vs Right Actual Position

双臂测试。

---

### Plot 7 — Relative Delay Distribution

```text
X: Relative Delay (ms)
Y: Count
```

---

# 22. 建议实验目录

放到：

```text
dual_arm_experiments/
├── scripts/
│   ├── sine_tracking_test.py
│   ├── dual_sync_test.py
│   └── emergency_stop_test.py
│
├── analysis/
│   ├── analyze_single_arm.py
│   ├── analyze_dual_sync.py
│   └── generate_report.py
│
├── config/
│   └── sine_test.yaml
│
└── results/
    ├── gazebo/
    └── real_robot/
```

配置不要写死进代码。

例如：

```yaml
arm: left
joint: left_j1

command_rate: 250.0

frequencies:
  - 0.5
  - 1.0
  - 2.0
  - 5.0

amplitude_rad: 0.05
duration_sec: 20.0

warmup_cycles: 2
```

---

# 23. Gazebo 退出判据

进入真机以前，以下全部通过：

* [ ] 正弦指令生成频率正确
* [ ] ServoJ/Gazebo command rate 实际接近 250 Hz
* [ ] CSV/rosbag 没有明显丢数据
* [ ] command/actual 可以正确对齐
* [ ] RMSE 计算正确
* [ ] 正弦拟合正确
* [ ] amplitude ratio 正确
* [ ] phase lag / equivalent delay 正确
* [ ] cross-correlation 能恢复人为注入的已知延迟
* [ ] jitter 计算正确
* [ ] 双臂 joint mapping 正确
* [ ] 测试超时会自动退出
* [ ] stop 后不会继续发送轨迹

特别建议做一个分析程序单元验证：

人为构造：

```text
Left  = sin(2πft)
Right = sin(2πf(t - 0.007))
```

人为设置：

```text
delay = 7 ms
```

分析程序必须能算回接近：

```text
7 ms
```

这一步通过以后再相信真机测出来的数字。

---

# 24. 真机退出判据

M1 完成后至少产出：

## 单臂

```text
Tracking RMSE @ 0.5/1/2/5 Hz
Maximum tracking error
Amplitude ratio
Equivalent delay
ServoJ call P95/P99
Control-loop period P95/P99
```

## 双臂

```text
Mean relative delay
Relative-delay σ
Relative-delay 3σ
Worst observed relative delay
```

最终报告建议形成：

| Metric                  | Result |
| ----------------------- | -----: |
| Tracking RMSE @ 0.5 Hz  |        |
| Tracking RMSE @ 1 Hz    |        |
| Tracking RMSE @ 2 Hz    |        |
| Tracking RMSE @ 5 Hz    |        |
| Equivalent delay @ 1 Hz |        |
| ServoJ call P99         |        |
| Control loop P99        |        |
| Left-right mean delay   |        |
| Left-right jitter σ     |        |
| Left-right jitter 3σ    |        |

---

# 25. 最终工程结论

完成这套实验后，不应该只得到一句：

> ServoJ 可以 250 Hz 运行。

而应该得到类似：

> ServoJ control loop operated at 250 Hz with a measured P99 cycle period of X ms. Single-arm sinusoidal tracking achieved Y° RMS error at 1 Hz with Z ms equivalent command-to-motion delay. Dual-arm synchronized tracking showed a mean relative delay of A ms and 3σ timing jitter of B ms.

这样的结果才真正说明：

* 控制频率是多少
* 实际跟踪性能是多少
* 延迟是多少
* 双臂同步能力是多少
* 后续是否值得做高同步要求的双臂协同

---

# 26. 推荐执行顺序

严格按以下顺序执行：

```text
Gazebo 单关节 0.5 Hz
↓
Gazebo 单关节全频率
↓
Gazebo 双臂同步
↓
分析脚本人工延迟验证
↓
真机单臂 ±1° / 0.5 Hz
↓
真机单臂 ±2~3° / 0.5 Hz
↓
真机 1 Hz
↓
真机 2 Hz
↓
根据速度/加速度和跟踪结果决定是否进行 5 Hz
↓
真机双臂 0.5 Hz
↓
真机双臂 1 Hz
↓
真机双臂 2 Hz
↓
得到 mean / σ / 3σ
↓
进入双臂协同架构决策
```

原则是：

**先证明测量工具正确，再测机器人；先证明低频安全，再提高频率；先证明单臂稳定，再测双臂同步。**
