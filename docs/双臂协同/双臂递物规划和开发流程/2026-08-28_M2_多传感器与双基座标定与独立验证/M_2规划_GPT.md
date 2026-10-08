我看过 `robotarm` 这部分代码了。现在可以把 M2 规划得更扎实一些，而且有一个很重要的修正：

> **`robotarm` 已经有一套相当完整的腕部相机手眼标定框架，不需要重写。**
>
> 但严格来说，它是“**RGB-D 相机的 RGB 通道做手眼标定**”：订阅的是彩色 `Image + CameraInfo`，用 ArUco 角点 + IPPE/PnP 求 `camera_color_optical_frame → marker`，没有用深度图参与手眼求解。深度应该另外做 RGB-D 对齐/偏差验证。([GitHub][1])

而且我还发现 `robotarm` 的一个值得你在迁移前修掉的问题：现有 `evaluate_calibration.py` 的 `eye_in_hand` 分支把 `T_base_mrk_pred` 直接赋成了 `T_base_mrk_obs`，所以该分支的误差实际上会被构造成 0，不能作为独立验收指标。相比之下，`solver.py` 内部已有的 fixed-marker consistency `marker_metrics()` 思路是正确的：每组样本反推出固定 marker pose，再计算其位置/旋转 RMS。([GitHub][2])

所以我建议 M2 不再叫简单的“标定”，而是：

# M2 — 多传感器与双基座标定 + 独立验证

目标不是“得到几个 TF”，而是最终拿到一套**有误差指标、可重复、可版本化、仿真有真值验证**的完整 TF 链。

最终应该确定：

```text
world
 ├── left_base
 │     └── left_tool0
 │            └── left_wrist_camera_color_optical
 │
 ├── right_base
 │     └── right_tool0
 │            └── right_wrist_camera_color_optical   # 如果右臂也有
 │
 └── global_camera_color_optical
```

并且最关键的三个未知量是：

[
{}^{B_L}T_{B_R}
]

双机械臂基座关系；

[
{}^{E}T_{C_w}
]

腕部相机 eye-in-hand；

[
{}^{B}T_{C_g}
]

全局相机 eye-to-hand。

---

## M2.0：先冻结坐标系约定和数据格式

这一步别省，否则后面最容易出现 `T_A_B` 到底谁到谁的问题。

整个项目统一定义：

[
{}^A T_B
]

表示：

> 把 B 坐标系中的坐标转换到 A 坐标系。

因此：

[
p_A={}^A T_B p_B
]

例如：

```text
left_base_T_right_base
tool0_T_wrist_camera_color_optical
left_base_T_global_camera_color_optical
```

不要在代码里出现模糊的：

```text
camera_to_tool
handeye_tf
base_transform
```

全部明确父子方向。

每次标定还应该保存：

```text
calibration_id
timestamp
git_commit
URDF hash/version
robot serial/side
camera serial
CameraInfo/intrinsics
dataset
solver
sample count
result transform
RMSE / P95 / MAX
```

这样以后相机重新安装，不会把新旧数据混起来。

**退出判据：**

所有 calibration 模块都只采用这一种 transform convention，并有一个 synthetic composition 单元测试。

---

# M2.1：先把 `robotarm` 手眼标定迁移成公共模块

这个不用重新发明。

`robotarm` 现有实现已经有：

* ArUco 亚像素角点；
* IPPE square PnP；
* 10 帧稳定观测；
* 运动后 settle + 关节静止检测；
* 样本位姿多样性检查；
* 固定 20 个采样位姿槽；
* 至少 15 个有效样本；
* Park / Horaud 求解；
* Tsai-Lenz 作为诊断；
* 多算法一致性检查；
* fixed-marker nonlinear refinement；
* 最差样本剔除；
* Gazebo ground-truth TF 检查；
* `.samples` / `.calib` 保存。([GitHub][3])

现有参数也已经比较成熟，比如：

```text
stable_frames                 10
minimum_samples               15
minimum_solution_samples      14
translation span              >= 40 mm
rotation span                 >= 20 deg

Park/Horaud disagreement:
translation <= 3 mm
rotation    <= 1 deg

fixed-marker RMS:
position <= 2 mm
rotation <= 0.70 deg
```

这些都已经写在 `robotarm` 配置里。([GitHub][1])

所以你的项目应该做的是：

```text
robotarm hand_eye_calibration
        ↓
参数化
        ↓
dual_arm_calibration/handeye
        ↓
left / right / global camera 共用
```

而不是重新写三套求解器。

---

# M2.2：先修“独立标定评估器”

这一项我会放在真机标定之前。

不要继续沿用现有 `evaluate_calibration.py` 那种：

```text
prediction = observation
```

的 eye-in-hand placeholder。([GitHub][2])

应该统一成一个 **constant-frame evaluator**。

### Eye-in-hand

标定结果：

[
{}^E T_C
]

每个**未参与求解的 hold-out pose** 都计算：

[
{}^B T_{M,i}
============

{}^B T_{E,i}
{}^E T_C
{}^C T_{M,i}
]

因为标定板固定在环境中，所以理论上：

[
{}^B T_{M,1}
============

# {}^B T_{M,2}

...
]

因此计算：

```text
marker position RMS
marker position P95
marker rotation RMS
marker rotation P95
```

### Eye-to-hand

固定全局相机：

[
{}^B T_C
]

marker 固定安装在末端，计算：

[
{}^E T_{M,i}
============

{}^E T_{B,i}
{}^B T_C
{}^C T_{M,i}
]

理论上它同样应该保持不变。

这样**不需要假设 marker frame 和 tool0 重合**。

而且一定：

> 求解 samples 和 validation samples 分开。

比如：

```text
15~20 poses：calibration
5~10 poses：hold-out validation
```

不能用参与求解的数据自己给自己打分。

---

# M2.3：腕部 RGB-D eye-in-hand

这一块直接以 `robotarm` 为基线。

当前 repo 的 frame 已经明确配置为：

```text
base_frame          = base_link
ee_frame            = tool0
tracking_base_frame = camera_color_optical_frame
tracking_marker     = calibration_aruco
```

而且它已经支持仿真 ground-truth TF 与估计结果比较。([GitHub][1])

### M2.3-A Gazebo 真值

在 URDF 里人为设：

[
{}^{tool0}T_{camera}=T_{GT}
]

然后跑**完全相同的 collector + solver**。

不要读取 GT 给 solver。

最后才比较：

[
T_{est}^{-1}T_{GT}
]

这里建议记录：

```text
translation error
X/Y/Z error
rotation error
fixed-marker RMS
Park/Horaud spread
```

现有 `robotarm` 仿真配置本身已经设置了 ground-truth 检查，例如总平移 3 mm、单轴 2 mm、旋转 1° 的门槛。([GitHub][1])

不过我们还可以加一个**纯数学 synthetic test**，那一层不经过 Gazebo 图像渲染，误差应该接近浮点精度：

```text
translation < 1e-6 m
rotation    < 1e-4 deg
```

Gazebo rendered-camera 则不用追求 `1e-6`。

---

### M2.3-B 真机腕部相机

用：

```text
15~20 solve poses
+
5~10 holdout poses
```

姿态覆盖不要只是 XYZ 平移。

手眼标定尤其需要 rotation excitation，因此 J4/J5/J6 要有足够变化。

建议采样空间类似：

```text
near/far
left/right
up/down
roll +
roll -
pitch +
pitch -
combined orientations
```

`robotarm` 当前已经有 20 槽 waypoint 表，而且配置中的姿态本身包含 XYZ 和 roll/tilt 组合，可以直接作为初始模板。([GitHub][1])

### 第一阶段验收建议

```text
solver internal:
position RMS <= 2 mm
rotation RMS <= 0.7°

hold-out:
position RMS <= 3 mm
position MAX <= 5 mm
rotation RMS <= 1 deg
```

这里 hold-out 比 solver 内部指标稍微宽一些是合理的。

---

# M2.4：腕部 RGB-D 还要单独做一个 Depth Validation

这点原规划里其实缺了。

因为 `robotarm` 手眼标定虽然用的是 D435 之类 RGB-D 相机，但手眼求解实际只使用：

```text
RGB image
+
CameraInfo
+
ArUco PnP
```

并没有读取实际 depth image。([GitHub][3])

但 M3 后面是：

```text
YOLO mask
↓
depth
↓
point cloud
↓
PCA
```

所以在进入 M3 前必须验证：

> **RGB pixel 对应的 depth 到底准不准。**

做一个平面标定板：

```text
camera distance:
0.25 m
0.35 m
0.50 m
0.70 m
```

每个距离采多帧。

测：

[
z_{depth}-z_{PnP}
]

以及点云拟合平面的：

```text
plane RMS
median depth bias
P95 depth error
invalid depth ratio
```

这个数字对以后视觉抓取非常有意义。

否则以后物体抓偏 8 mm，你不知道是：

```text
hand-eye 3 mm
+
depth bias 5 mm
```

还是 YOLO 错了。

---

# M2.5：双基座标定——建议机械法作为主标定

这一项我不建议依赖视觉作为唯一来源。

因为如果：

```text
left wrist handeye 有误差
+
right wrist handeye 有误差
```

再用两台相机求 base-to-base，就会把相机标定误差一起带进去。

所以：

> **双基座主标定最好独立于相机。**

Claude 原来的“辅助对触”方向是对的，但需要再工程化一点。

---

## 第一步先校准两个测量 TCP

如果你用两个尖头 probe：

```text
left_tool0 → left_probe_tip
right_tool0 → right_probe_tip
```

这两个变换必须先准确。

否则你测到的所谓“基座误差”其实包含：

```text
TCP offset error
```

建议使用固定球心/尖点做多姿态 pivot TCP calibration。

输出：

```text
left_tool0_T_probe
right_tool0_T_probe
TCP repeatability
```

---

## 然后采共同物理点

设计一个简单的 3D calibration fixture：

```text
●     ●

   ●

●        ●
      ●
```

不要所有点都在一条线附近。

最好也不要只在非常小的一块平面区域。

建议：

```text
15~20 correspondences
```

覆盖整个未来**双臂交接区域**，同时尽量有一定 Z 高度变化。

每个物理点 (P_i)，左臂测：

[
p_i^{B_L}
]

右臂测：

[
p_i^{B_R}
]

求：

[
p_i^{B_L}
\approx
R p_i^{B_R}+t
]

用 Kabsch / SVD rigid alignment 求：

[
{}^{B_L}T_{B_R}
]

这个数学问题非常干净。

建议：

```text
15 samples fit
5 samples hold-out
```

不要把 20 个全拿去 fit。

最终报告：

```text
fit RMS
fit P95
fit MAX

holdout RMS
holdout P95
holdout MAX
```

### 初始退出判据

我建议：

```text
fit RMS      <= 1.5~2 mm
holdout RMS  <= 2 mm
holdout MAX  <= 3 mm
```

如果做不到，就先查：

```text
TCP calibration
joint zero
URDF geometry
fixture rigidity
touch repeatability
```

不要直接把误差塞进软件 offset。

---

# M2.6：双基座仿真 + Monte Carlo

这个非常适合先做。

Gazebo 中：

[
{}^{B_L}T_{B_R}^{GT}
]

已知。

模拟：

```text
contact noise
TCP error
joint/FK noise
```

然后恢复：

[
{}^{B_L}T_{B_R}^{est}
]

建议采样数：

```text
N = 4 / 6 / 10 / 15 / 20 / 30
```

每个 N：

```text
500~1000 Monte-Carlo trials
```

输出两张最有价值的曲线：

```text
sample count → translation P95
sample count → rotation P95
```

同时再比较：

```text
小范围点云
vs
覆盖整个交接区
```

你会非常直观看到：

> “姿态/空间覆盖”往往比单纯“多采几个点”更重要。

最终再据此决定真机到底采：

```text
15 还是 20
```

而不是凭经验猜。

---

# M2.7：全局 RGB 相机 eye-to-hand

这里其实可以继续复用同一个 `robotarm` solver。

现有配置已经明确支持：

```text
calibration_type: eye_in_hand
```

同时代码里对 eye-in-hand 和 eye-on-base 使用不同 robot-pose 语义，并且配置里也有 eye-on-base 相机安装距离安全门。([GitHub][4])

对于固定全局相机：

```text
global camera
       ↓
看到安装在 tool 上的 ArUco
```

标定：

[
{}^{B}T_{C_g}
]

非常重要的一点：

## 我建议同一个 global camera 分别对两条臂独立标定一次

得到：

[
{}^{B_L}T_{C_g}
]

以及：

[
{}^{B_R}T_{C_g}
]

那么就可以独立推导：

[
{}^{B_L}T_{B_R}^{camera}
========================

{}^{B_L}T_{C_g}
\left(
{}^{B_R}T_{C_g}
\right)^{-1}
]

这就给了你一个非常漂亮的 **双基座独立交叉验证**。

---

# M2.8：双基座最终必须有“两种方法互证”

最终你会有：

### 方法 A：机械触碰

[
{}^{B_L}T_{B_R}^{touch}
]

### 方法 B：全局相机

[
{}^{B_L}T_{B_R}^{camera}
]

计算：

[
\Delta T
========

\left(
{}^{B_L}T_{B_R}^{touch}
\right)^{-1}
{}^{B_L}T_{B_R}^{camera}
]

记录：

```text
translation disagreement
rotation disagreement
```

如果两个完全独立的方法最后得到：

```text
translation difference < 2~3 mm
rotation difference    < 0.2~0.5 deg
```

我会对这套双臂坐标系统相当有信心。

如果：

```text
touch result: 1 mm RMS
camera result: 和它差 8 mm
```

那就说明问题大概率在：

```text
global hand-eye
camera optical frame
marker geometry
camera intrinsics
```

而不是机械基座。

这个“误差可定位性”非常重要。

---

# M2.9：最终系统级交叉验证

M2 结束前，不要只看矩阵数字。

在真实双臂交接区放 5~10 个 validation targets。

然后分别让：

```text
global camera
left arm
right arm
```

去描述同一个物理点。

最终得到：

[
p_{left}
,\quad
p_{right}
,\quad
p_{vision}
]

统一转换到 `world/left_base` 后比较。

最后 M2 应该产生类似这样的报告：

| 指标                                  |       目标 |
| ------------------------------------- | ---------: |
| Wrist hand-eye internal RMS           |     ≤ 2 mm |
| Wrist hold-out RMS                    |     ≤ 3 mm |
| Dual-base touch hold-out RMS          |     ≤ 2 mm |
| Touch vs global-camera base transform |   ≤ 2–3 mm |
| Global eye-to-hand hold-out RMS       |     ≤ 3 mm |
| RGB-D depth bias @ working range      |   实测给出 |
| RGB-D plane RMS                       |   实测给出 |
| 三链路同一点最终偏差                  | **≤ 3 mm** |

最后那个：

> **三链路同一点偏差 ≤ 3 mm**

我认为才应该是 Claude 原来 G2 的真正核心指标。

---

# 我建议的实际实施顺序

1. **M2.0** 坐标系 convention + 数据格式冻结。
2. **M2.1** 把 `robotarm` hand-eye collector/solver 迁进双臂工程。
3. **M2.2** 修独立 evaluator，增加 hold-out 数据。
4. **M2.3** Gazebo eye-in-hand 真值回归。
5. **M2.4** 真机腕部 RGB 手眼 + RGB-D depth validation。
6. **M2.5** TCP probe calibration。
7. **M2.6** 双基座 touch calibration + Monte Carlo。
8. **M2.7** global camera 分别对 left/right 做 eye-to-hand。
9. **M2.8** touch 与 camera 推导的 base transform 交叉验证。
10. **M2.9** 交接区 5~10 个独立物理点做系统级 validation。

这里我特别建议**不要先写一大堆新 hand-eye 代码**。

`robotarm` 已经有一个不错的 calibration core，你现在真正需要补的是：

> **双基座求解器 + 独立 evaluator + RGB-D depth validation + calibration versioning。**

还有一个值得马上告诉 Claude 的点：

> `robotarm/scripts/evaluate_calibration.py` 的 eye-in-hand 评估分支目前存在 `T_base_mrk_pred = T_base_mrk_obs` 的 placeholder，不能直接拿来作为标定精度证明；M2 应新建基于 hold-out fixed-frame constancy 的 evaluator。([GitHub][2])

我认为这会让整个 M2 从“把几个 TF 标出来”升级成一个很完整的**标定、误差建模与交叉验证子系统**，而且后面 M3 抓取失败时会非常容易定位到底是哪一层出了问题。

[1]: https://github.com/jasonlee0617/fairino_robotarm/blob/main/src/calibration_ws/hand_eye_calibration/config/auto_calibration_collector_params.yaml "fairino_robotarm/src/calibration_ws/hand_eye_calibration/config/auto_calibration_collector_params.yaml at main · jasonlee0617/fairino_robotarm · GitHub"
[2]: https://github.com/jasonlee0617/fairino_robotarm/blob/main/src/calibration_ws/hand_eye_calibration/scripts/evaluate_calibration.py "fairino_robotarm/src/calibration_ws/hand_eye_calibration/scripts/evaluate_calibration.py at main · jasonlee0617/fairino_robotarm · GitHub"
[3]: https://github.com/jasonlee0617/fairino_robotarm/blob/main/src/calibration_ws/hand_eye_calibration/hand_eye_calibration/vision.py "fairino_robotarm/src/calibration_ws/hand_eye_calibration/hand_eye_calibration/vision.py at main · jasonlee0617/fairino_robotarm · GitHub"
[4]: https://github.com/jasonlee0617/fairino_robotarm/blob/main/src/calibration_ws/hand_eye_calibration/hand_eye_calibration/solver.py "fairino_robotarm/src/calibration_ws/hand_eye_calibration/hand_eye_calibration/solver.py at main · jasonlee0617/fairino_robotarm · GitHub"
