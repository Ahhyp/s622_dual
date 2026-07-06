# 阶段 3：MoveIt2 配置与规划插件

## 目标

将阶段 2 的规划核心库封装为 MoveIt2 插件，配置完整的运动规划环境，使机械臂能在 RViz 中进行拖拽规划和执行。

## 前提

- 阶段 1（URDF 模型）
- 阶段 2（fairino_planning_core 编译通过）

## 待创建的包

| 序号 | 包名 | 说明 | 来源 |
|------|------|------|------|
| 1 | `fairino3_v6_moveit2_config` | Fairino3 v6 MoveIt2 全套配置 | 从原项目复制 |
| 2 | `s622_moveit_config` | S622 MoveIt2 配置（主力用） | 从原项目复制 |
| 3 | `fairino_planning_ros` | MoveIt2 规划器 + IK 插件 | 从原项目复制 |
| 4 | `pymoveit2` | 外部项目，MoveIt2 Python 接口 | 从原项目复制 |

## 包内容说明

### fairino_planning_ros
- `FairinoPlannerManager` — 规划器管理类
- `FairinoPlanningContext::solve()` — 规划请求 → BiRRT* → 后处理 → 轨迹
- `FairinoIKPlugin` — 解析 IK 注册为 MoveIt IK 求解器
- `MoveItCollisionChecker` — 包装 MoveIt PlanningScene 碰撞检测

### MoveIt2 配置包
- SRDF：规划组、末端执行器、碰撞禁用矩阵、虚拟关节
- `kinematics.yaml` — 指向自定义 IK 插件
- `fairino_planning.yaml` — 规划器参数
- `ros2_controllers.yaml` / `moveit_controllers.yaml`

## 验证

```bash
ros2 launch s622_moveit_config demo.launch.py
```

RViz MotionPlanning 面板中拖拽规划，路径生成正确无碰撞。
