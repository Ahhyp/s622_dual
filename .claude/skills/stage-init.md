---
name: stage-init
description: 初始化新阶段，创建 docs/stages/ 下的 plan.md 和 issues.md
---

# stage-init

项目[代码级项目搭建流程](代码级项目搭建流程.md)是唯一计划来源（v2，8 阶段，含完整代码模板和验收标准）。

## 参数

`<stage_num>` — 阶段号（1~8），对应流程文档的节号：

- `1` = 阶段一（§4：厂商层，复制）
- `2` = 阶段二（§5：Gazebo 仿真，自写）
- `3` = 阶段三（§6：自研 planning_core，自写）
- `4` = 阶段四（§7：自研 planning_ros，自写）
- `5` = 阶段五（§8：感知与标定，部分自写）
- `6` = 阶段六（§9：visual_servo，自写）
- `7` = 阶段七（§10：OctoMap + GraspNet，自写）
- `8` = 阶段八（§11：工具层，部分自写）

## 目录约定

所有阶段记录放在 `docs/stages/` 下：

```
docs/stages/
├── README.md                       ← 阶段索引
├── 01_robot_model/
│   ├── plan.md
│   └── issues.md
├── 02_gz_launch/
│   ├── plan.md
│   └── issues.md
├── 03_planning_core/
│   ├── plan.md
│   └── issues.md
├── 04_planning_ros/
│   ├── plan.md
│   └── issues.md
└── ...
```

编号格式：`<两位数阶段号>_<英文简称>`。

## 执行步骤

### 1. 创建目录

```bash
mkdir -p docs/stages/<NN>_<简称>
```

### 2. 从流程文档提取信息

读取 `代码级项目搭建流程.md` 对应章节，提取：
- 阶段名称和目标
- 涉及包列表（自写 vs 复制）
- 验收标准

### 3. 创建 plan.md

```markdown
# 阶段 N：<名称>（自写/复制）

## 目标
...

## 涉及包
| 包名 | 方式 | 说明 |
|------|------|------|

## 验收标准
- ...

## 后续
→ [阶段 N+1](../<NN>_<简称>/plan.md)
```

### 4. 创建 issues.md

```markdown
# 阶段 N：遇到的问题与解决方案

---

（尚无记录）
```

### 5. 更新 README.md

更新 `docs/stages/README.md` 中的状态列。

### 6. 输出

告知用户本阶段概要、自写/复制比例、验收标准。
