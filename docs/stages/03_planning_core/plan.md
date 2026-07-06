# 阶段 2：核心运动学与规划算法

## 目标

从零手写纯 C++（无 ROS 依赖）的运动学求解器和路径规划算法，脱离 ROS 独立编译和测试。

**本包是项目最核心的算法，必须自己写。**

## 前提

- 阶段 1 完成（URDF/DH 参数可用）
- 依赖：Eigen3（`libeigen3-dev`）、nanoflann（`libnanoflann-dev`）
- 原项目 `fairino_planning_core` 作为最终参考（写完才对照）

## 待创建

| 包名 | 说明 | 方式 |
|------|------|------|
| `fairino_planning_core` | C++17 运动学 + 规划核心库 | **自写** |

## 实现步骤

DH 参数（从阶段 1 URDF 提取）：
```
d     = {0.140, 0,     0,     0.102, 0.102, 0.100}
a     = {0,    -0.280, -0.240, 0,     0,     0}
alpha = {π/2,   0,      0,    π/2,  -π/2,   0}
```

### 第 1 步：types.h — 核心数据类型

定义 JointConfig（6 维关节角）、Transform4d（4x4 变换矩阵）、PlanningRequest/Result。

### 第 2 步：dh_kinematics.h/.cpp — FK 正运动学

DH 变换矩阵级联。单个 DH 变换 = Rot_z(θ) × Trans_z(d) × Trans_x(a) × Rot_x(α)。

### 第 3 步：fairino_ik.h/.cpp — IK 解析逆运动学

腕前点分离法：WCP → 前 3 关节几何解 → 后 3 关节反解 → 最多 8 组解 → 限位过滤。

### 第 4 步：ik_selector.h/.cpp — IK 解选择器

加权评分：运动代价 + 限位惩罚 + 奇异规避。

### 第 5 步：collision_interface.h — 碰撞检测抽象接口

纯虚类 CollisionChecker，方便脱离 ROS mock 测试。

### 第 6 步：rrt_tree.h/.cpp — RRT 树数据结构

KD 树（nanoflann）加速近邻搜索。

### 第 7 步：mixed_sampler.h/.cpp — 混合采样器

80% 均匀 + 10% 目标偏置 + 10% 高斯扰动。

### 第 8 步：rrt_star.h/.cpp — RRT* 单树

采样 → 延伸 → 碰撞检查 → rewire（优化父节点）。

### 第 9 步：bi_rrt_star.h/.cpp — BiRRT* 双向搜索

两棵树交替生长 + tryConnect() + planWithFallback()。

### 第 10 步：path_shortcut.h/.cpp — 路径快捷方式

随机两点直连，删中间冗余点。

### 第 11 步：trajectory_smoother.h/.cpp — 轨迹平滑

B-spline 平滑关节轨迹。

## 验证

1. FK-IK 互逆：`FK(IK(pose)) ≈ pose`
2. 无碰撞场景规划：`planner.plan(q_start, q_goal)` 返回有效路径
3. 写完所有模块后对照原项目，记录差异到 issues.md
