# 阶段 3（流程文档阶段三）：核心运动学与规划算法（自写）

## 包

`fairino_planning_core` — 纯 C++17，无 ROS 依赖。

## 实现内容

| 模块 | 文件 |
|------|------|
| 核心类型 | `types.h`（JointArray, Pose, JointLimits, PlanRequest/Result） |
| FK 正运动学 | `dh_kinematics.h/.cpp`（标准 DH 变换链） |
| IK 解析逆运动学 | `ik/fairino_ik.h/.cpp`（腕前点分离法，0~8 组解） |
| IK 解选择器 | `ik/ik_selector.h/.cpp`（最小版：运动+限位居中代价） |
| RRT 树 | `tree/rrt_tree.h/.cpp`（KD 树近邻，动态扩容，rewire 支持） |
| BiRRT* 规划器 | `algorithms/bi_rrt_star.h/.cpp`（双树交替+连接+rewire） |
| 碰撞检测接口 | `collision/collision_interface.h`（纯虚类） |
| 测试程序 | `test_core.cpp`（FK/IK/IKSelector/BiRRT* 验证） |

## 验证

```bash
colcon build --packages-select fairino_planning_core
./install/lib/fairino_planning_core/test_core
```

6 组测试全部 PASS：FK 零位/非零位、IK FK↔IK 闭环、IK 选择器、BiRRT* 短距离/远距离。

## 遇到的问题

### 问题 1：rrt_tree.h 残留 JointConfig 类型名

**日期：** 2026-05-28

**原因：** 从原项目复制时只改了 `TreeNode::state`，遗漏了方法签名。

**解决：** 头文件 4 处 `JointConfig` → `JointArray`。

### 问题 2：rrt_tree.cpp 残留 Eigen 向量语法

**日期：** 2026-05-28

**原因：** 原项目 `JointConfig = Eigen::Matrix` 支持 `.squaredNorm()` / `.norm()` / `operator-`，改为 `std::array` 后不可用。

**解决：** nearest/nearRadius fallback、propagateCost 共 3 处改为手动循环。

### 问题 3：DHKinematics 无默认构造函数

**日期：** 2026-05-28

**原因：** `FairinoIK` 构造函数中 `fk_` 成员被默认构造，但 `DHKinematics` 无默认构造函数。

**解决：** 用 lambda 在初始化列表中构建 DHParam 数组：
```cpp
: d_(d), a_(a), alpha_(alpha), fk_([](const auto& d, const auto& a, const auto& al) {
    std::array<DHParam, DOF> dh;
    for (int i = 0; i < DOF; ++i) dh[i] = {a[i], al[i], d[i], 0.0};
    return dh;
}(d, a, alpha))
```

### 问题 4：JointLimits 默认值为零导致 IKSelector 全部过滤

**日期：** 2026-05-28

**原因：** `JointLimits` 结构体没有成员默认值，lower/upper 全为零。

**解决：** `types.h` 中 `JointLimits` 加上 S622 默认限位值。

### 问题 5：test_core 未安装到 install 目录

**日期：** 2026-05-28

**原因：** CMakeLists.txt 有 `add_executable` 但缺 `install(TARGETS ...)`。

**解决：** 补 `install(TARGETS test_core DESTINATION lib/${PROJECT_NAME})`。

### 问题 6：ik_selectior.cpp 文件名拼写错误

**日期：** 2026-05-28

**原因：** 文件名 `ik_selectior.cpp` 拼错。

**解决：** 重命名为 `ik_selector.cpp`。
