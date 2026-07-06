# DH 工具偏移问题的修复

诊断很准确，问题就在这。下面给两个方案，按工作量排序。

---

## 方案 A：直接改 d6（推荐，立即可用）

最简单的修复——把 DH 的 d6 改成"j5 到 grasp_frame"的距离，让 IK 直接为 grasp_frame 求解。

修改 `fairino_planning_core/config/fairino_ik_service.yaml`：

```yaml
fairino_ik_service:
  ros__parameters:
    joint_names: ["j1", "j2", "j3", "j4", "j5", "j6"]
    dh_a:     [ 0.0,   -0.280, -0.240,  0.0,      0.0,      0.0  ]
    dh_alpha: [ 1.570796, 0.0,   0.0,   1.570796, -1.570796, 0.0 ]
    dh_d:     [ 0.140,  0.0,   0.0,   0.102,    0.102,    0.2168]
                                                          # ↑ 0.100 → 0.2168
```

### 为什么这样改就行

`fairino_ik` 的 IK 公式里，d6 出现在两个关键位置：

```cpp
// Step 1: 计算腕心 WCP
const double xw = p.x() - d6 * ax;      // 从目标 pose 减去 d6 沿末端 z 轴
const double yw = p.y() - d6 * ay;
const double zw = p.z() - d6 * az;
```

d6 越大，腕心越往"内"缩。把 d6 从 0.100 改成 0.2168 后：

1. IK 公式认为"目标 pose 是 grasp_frame 应该在的位置"
2. 解算出的 q1~q6 让 j5+d6·z 这条向量末端 = 目标 pose
3. URDF 执行时，grasp_frame 正好落在 j5 + 0.2168·z 处 = 目标 pose ✓

**FK 验证那一步也通过**，因为同一个 DH 模型的 forward 和 inverse 一致。

### 改完后注意

* 工作空间会**略微变化**：因为腕心位置不同，原来可达的某些边缘 pose 可能变得不可达
* IK 解的 q1~q5 值会**和之前不同**（q6 不变，因为它不影响末端位置）
* **可以用之前的 service 调用命令重新验证**：

```bash
ros2 service call /fairino/get_all_ik fairino_msgs/srv/GetAllIK "{
  pose: {
    header: {frame_id: 'base_link'},
    pose: {
      position: {x: 0.421, y: -0.028, z: 0.160},
      orientation: {x: 0.0, y: 0.707, z: 0.0, w: 0.707}
    }
  },
  group_name: 'robot_arm'
}"
```

应能拿到解，且执行后 grasp_frame 实际 Z ≈ 0.160（不再差 0.117）。

---

## 方案 B：DH 保持纯运动学，工具偏移单独管（长远更好）

如果你将来要**换夹爪**或者**支持多个 TCP**，方案 A 就不够灵活——每换一个夹爪都要改 DH yaml，把"机器人本体"和"工具"两件事混在一起。

更干净的做法是：**DH 仍然代表到法兰，service 接收 grasp_frame 目标，内部转换到法兰目标再求 IK**。

### 实现

`fairino_ik_service.yaml` 加一个工具偏移字段：

```yaml
fairino_ik_service:
  ros__parameters:
    joint_names: [...]
    dh_a:     [...]
    dh_alpha: [...]
    dh_d:     [0.140, 0.0, 0.0, 0.102, 0.102, 0.100]   # ← 回到法兰
    
    # Tool transform: from flange (wrist3_link) to grasp_frame
    tool_offset_xyz:  [0.0, 0.0, 0.2168]              # 工具偏移
    tool_offset_rpy:  [0.0, 0.0, 0.0]                  # 工具旋转(如有)
```

Service 里加入工具变换：

```cpp
// 构造函数读取
auto tool_xyz = node_->declare_parameter<std::vector<double>>(
    "tool_offset_xyz", std::vector<double>{0.0, 0.0, 0.0});
auto tool_rpy = node_->declare_parameter<std::vector<double>>(
    "tool_offset_rpy", std::vector<double>{0.0, 0.0, 0.0});

T_flange_to_tool_ = Eigen::Isometry3d::Identity();
T_flange_to_tool_.translation() << tool_xyz[0], tool_xyz[1], tool_xyz[2];
T_flange_to_tool_.linear() = (
    Eigen::AngleAxisd(tool_rpy[2], Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(tool_rpy[1], Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(tool_rpy[0], Eigen::Vector3d::UnitX())
).toRotationMatrix();

// handle() 里
fairino_planning::Pose target_tool = ...;     // 用户给的 grasp_frame 目标
// 转换:目标 grasp_frame 对应的法兰位姿
fairino_planning::Pose target_flange = target_tool * T_flange_to_tool_.inverse();

// 用法兰目标求 IK
std::vector<fairino_planning::JointArray> solutions = ik_->solve(target_flange);
```

### 优劣对比

| 维度                | 方案 A（改 d6） | 方案 B（工具偏移分离）              |
| ----------------- | ---------- | ------------------------- |
| 工作量               | 改一行 yaml   | 改 service C++ 代码 + yaml   |
| 换夹爪               | 改 DH yaml  | 只改工具偏移                    |
| 概念清晰度             | DH 含工具，有点混 | 机器人本体 vs 工具完全分离           |
| 多 TCP 支持（吸盘/夹爪切换） | 难          | service 加个 tool_name 参数即可 |
| 实测是否一致            | ✅          | ✅                         |

---

## 我的建议

**先按方案 A 改 d6 验证整个链路工作**（5 分钟搞定），把基础跑通。

后续如果你要做：

* 多夹爪切换（吸盘/二指/三指共用一套规划）
* 工具长度动态校准（每次换工具后自动重读偏移）
* 工具坐标系不是纯平移（例如夹爪斜装）

再升级到方案 B。

---

## 一个常见陷阱预告

改完 d6 后，可能会发现：

**"之前能规划成功的 pose 现在解不出来了"**

原因：腕心位置变化导致原来勉强可达的边缘 pose 变得不可达。这不是 bug，是工作空间确实变了。验证方法：把目标 Z 降一点（从 0.16 改到 0.12 试试），如果能解，说明仅仅是工作空间边界变了；如果完全解不出，那是 DH 其他参数也有问题。

---

## 一句话总结

立即修复：yaml 里 `dh_d` 的最后一个值从 `0.100` 改成 `0.2168`，重启 service。这样 IK 直接为 grasp_frame 求解，URDF 执行后位置一致。长远建议方案 B（DH 管运动学，工具偏移单独配置），但不急。

改完后跑一次原来那个 (0.421, -0.028, 0.160) 验证，Z 应该正好命中。
