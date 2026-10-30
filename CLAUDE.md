# CLAUDE.md

## 项目概述

本项目是 S622 机械臂智能抓取系统的学习复现项目，按照[代码级项目搭建流程](代码级项目搭建流程.md)从零搭建。

**原则：每一步都建包、写代码、编译、运行、验收、记录。**

## 参考文档

| 文件 | 说明 |
|------|------|
| [代码级项目搭建流程](代码级项目搭建流程.md) | **主文档**，13 阶段搭建流程，每步有代码模板和验收标准 |
| [原项目结构](原项目结构.md) | 原项目 26 个包完整目录树 |
| [学习](学习.md) | 原项目运行指南、源码阅读路径 |
| [项目搭建流程](项目搭建流程.md) | 原项目原始搭建文档 |
| [IK 架构说明](docs/IK架构说明.md) | 两套 IK 系统的关系、调用链、死代码说明 |

## 环境

- **宿主机**：Ubuntu 26.04（没有任何 `ros-humble-*` 二进制包，ROS 不能装在宿主机）
- **ROS 2 Humble + Gazebo Fortress 全部跑在 Docker 容器里**：镜像 `s622:humble-ros-yolo`，运行容器 `s622_run`
- 工作区 `/home/kz/s622_dual` 挂载为容器内 `/workspace`；容器内 Python 3.10（`/usr/bin/python3`）
- **GPU**：宿主机 Quadro RTX 4000 已透进容器（`--gpus all`）。Gazebo **服务器**（相机传感器）用 NVIDIA，Gazebo **GUI 和 RViz 走 mesa/llvmpipe**（NVIDIA GLX 在 Xwayland 双卡下 3D 会黑屏，见经验教训 13）；`RENDER=software ./scripts/env/10_container.sh recreate` 可整体回退 llvmpipe
- YOLO 运行时在容器内独立 venv `/opt/ros-yolo-venv`（**torch 2.14.1+cu130**，YOLO 走 GPU；ultralytics），不污染 ROS 系统 Python
- 源码位于 `src/`，按分层目录组织
- 完整部署说明：[docs/环境部署_Ubuntu26.04_Docker.md](docs/环境部署_Ubuntu26.04_Docker.md)

## 构建

```bash
# 宿主机（脚本内置 sg docker 兜底 docker 组权限）
./scripts/env/10_container.sh ensure
./scripts/env/10_container.sh run "bash /workspace/scripts/env/04_build_ws.sh --clean"
```

容器内等价命令（merge-install + symlink-install，与旧流程一致）：

```bash
source /opt/ros/humble/setup.bash
rm -rf build install log
colcon build --merge-install --symlink-install \
    --parallel-workers $(nproc) --continue-on-error \
    --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 -DCMAKE_BUILD_TYPE=Release
```

## 目录结构

```
src/                          ← 所有包平铺，不再用子目录分类
├── fairino_description/
├── fairino_msgs/
├── fairino_hardware/
├── s622_moveit_descriptions/
├── fairino3_v6_moveit2_config/
├── s622_moveit_config/
├── gz_launch/
├── fairino_planning_core/
├── fairino_planning_ros/
├── visual_servo/              # 视觉伺服抓取
├── s622_task_interfaces/      # 双臂项目：Action/Service 接口（待建）
├── s622_servo_actions/        # 双臂项目：伺服 Action Server（待建）
├── s622_bt_manager/           # 双臂项目：BT.CPP 任务管理器（待建）
├── dual_arm_kinematics/       # 双臂 DH 正逆运动学、雅可比、相对可操作度
├── dual_arm_workspace_analysis/  # C4.1：双臂工作空间与可操作度分析
├── dual_arm_pose_optimization/   # C4.2：交接位姿优化（离线 MoveIt + FCL + 多目标）
└── yolov8_obb/                # YOLOv8 OBB 检测
```

## 进度跟踪

| 阶段 | 内容 | 自写/复制 | 状态 |
|------|------|-----------|------|
| 阶段一 | 6 个厂商包（description/msgs/hardware/moveit_descriptions/moveit_config） | 复制 | ✅ 已完成 |
| 阶段二 | gz_launch | **自写** | ✅ 已完成 |
| 阶段三 | fairino_planning_core | **自写** | ✅ 已完成 |
| 阶段四 | fairino_planning_ros | **自写** | ✅ 已完成 |
| 阶段五 | 感知与标定 | 部分自写 | ✅ 已完成 |
| 阶段六 | visual_servo | **自写** | ✅ 已完成 |
| 阶段七 | OctoMap + GraspNet | **自写** | 搁置 |
| 阶段八 | 工具层 | 部分自写 | 搁置 |
| **阶段九** | **双臂协同** | **自写** | ← 当前 |

### 双臂协同子阶段

> 总计划：[docs/双臂协同/双臂协同总体计划.md](docs/双臂协同/双臂协同总体计划.md)
> 阶段记录：[dual_arm_stages/](dual_arm_stages/)

| 子阶段 | 内容 | 状态 |
|--------|------|------|
| 阶段 1 | BT.CPP 单臂 Pick-Place 闭环重构 | ← 当前 |
| 阶段 2 | 双臂坐标系统一 | 待开始 |
| 阶段 3 | 双臂独立控制 | 待开始 |
| 阶段 4 | 双臂同步运动 | 待开始 |
| 阶段 5 | 双臂递物交接 | 待开始 |
| 阶段 6 | 双臂协同搬运 | 待开始 |
| 阶段 7 | 实验统计与简历包装 | 待开始 |

## 新对话快速上手

新对话开始时先读 `CLAUDE.md`；任务级证据在 `docs/双臂协同/双臂协同运动控制与优化/`。

## 快速启动（双臂协同需要仿真正在运行）

```bash
# 终端 1（宿主机）：仿真 = Gazebo + MoveIt2 + RViz + YOLO + Servo 一起拉起
./scripts/env/10_container.sh run "bash /workspace/scripts/env/06_run_sim.sh"

# 单臂入口也保留了一个别名（等价于上面）
# ./scripts/env/10_container.sh run "ros2 launch gz_launch test_single_arm.launch.py"

# 终端 2（宿主机开容器 shell）：visual_servo
./scripts/env/10_container.sh shell
ros2 run visual_servo visual_servo_node --ros-args \
  -p enable_motion:=true -p z_strategy:="table" \
  -p table_z:=0.0 -p grasp_height_above_table:=0.030 \
  -p pregrasp_height_above_table:=0.16 -p lock_check_pixel_tolerance:=12.0 \
  -p descend_speed:=0.04 -p descend_timeout:=30.0 \
  -p lifting_timeout:=30.0 -p lift_height:=0.10 \
  -p visual_align_timeout:=25.0 -p pregrasp_camera_offset:=0.03

# 触发抓取
ros2 topic pub /servo_trigger std_msgs/msg/Bool "data: true" --once

# 停止仿真（被杀掉的进程会变僵尸，只有重启容器能回收）
./scripts/env/10_container.sh run "bash /workspace/scripts/env/08_stop_sim.sh --hard"
docker restart s622_run
```

> YOLO 检测节点（`obb_node`）已由 launch 自动启动，解释器默认 `/opt/ros-yolo-venv/bin/python3`，
> 权重自动探测；不需要再单独开终端。要单独跑时用同一个 venv：
> `ros2 run yolov8_obb yolov8_obb_node --ros-args -p model_path:=$(ros2 pkg prefix --share yolov8_obb)/...`

## 可用 Skill

| 命令 | 说明 |
|------|------|
| `/record-issue <标题>` | 追加问题记录到 docs/issues.md |

## 经验教训

1. **ROS 环境在容器里**（宿主机 Ubuntu 26.04 没有 Humble 二进制）；宿主机命令走 `scripts/env/10_container.sh`，脚本内部用 `sg docker` 兜底 docker 组权限
2. 容器内天然是 `/usr/bin/python3`（3.10），构建仍保留 `-DPython3_EXECUTABLE=/usr/bin/python3`，避免 CMake 找错解释器
3. 首次编译前必须 `rm -rf build install log` 清理缓存
4. 不擅自修改原项目 CMakeLists.txt，构建失败优先检查系统依赖
5. 厂商包直接整批复制，不要逐包搭建（配置依赖互相耦合）
6. 复制 s622_moveit_descriptions 时删除 camera 引用，等仿真阶段再引入
7. 🚨 **相机内参 bug**：gz-sim Fortress 的 `rgbd_camera` 传感器会导致 `<horizontal_fov>` 和 `<lens><intrinsics>` 不一致（渲染用 hfov，camera_info 读 lens）。解决方案：分离为独立 `camera` + `depth_camera` 传感器，显式提供 `<lens>`，hfov 使用反推的实际渲染值 82.4°（而非 D435 物理 69°）。详记录于 [gazebo_rgbd_intrinsics_bug_resolve.md](docs/dual_arm_stages/stage_1/gazebo_rgbd_intrinsics_bug_resolve.md)。恢复指南在文档第七章。涉及文件：
   - `src/s622_moveit_descriptions/urdf/camera/rgbd_split.gazebo.xacro`（新建）
   - `src/s622_moveit_descriptions/urdf/camera/camera.xacro`（修改，hfov=82.4°, vfov=52.5°）
   - **真机部署时恢复使用 realsense-ros 驱动即可，不需回滚**
8. **IK 架构**：项目有两套 IK。主路径 `plan_to_pose_smart()` 通过 `/fairino/get_all_ik` service 调 DH 解析法拿全部解并评分，MoveIt 只做碰撞检测+轨迹插值。`kinematics.yaml` 配的 KDL 仅作 fallback。`_compute_ik_kdl()` 是死代码。`fairino_planning_ros/FairinoIKPlugin` 存在但未启用。详见 [IK 架构说明](docs/IK架构说明.md)。
9. **编译成功 ≠ 能运行**：2026-10-28 换机后 33 个包一次编译通过，但启动时连缺三个运行期依赖——`gz_ros2_control`（没有它 Gazebo 里根本没有 `/controller_manager`，spawner 一直等待）、`moveit_servo`（`servo_node_main` 起不来）、`realsense2_description`（URDF 里 `$(find ...)` 解析失败，launch 直接退出）。换机验收必须真的 `ros2 launch` 一次，并用 `ros2 control list_controllers` + 一条轨迹确认闭环。
10. **launch 里不要写作者机器的绝对路径**：`s622_gazebo.launch.py` 原来写死 `~/my_S622/.../yolo-obb-gazebo.pt` 和 `/home/yep/miniconda3/envs/yolov8/bin/python3`，换机后 YOLO 节点必死。现已改为自动探测 + launch 参数（`yolo_model_path` / `yolo_python` / `start_yolo`）。
11. **GPU 渲染（2026-10-29 启用）**：宿主机 Quadro RTX 4000 经 `nvidia-container-toolkit` 透进容器，`10_container.sh` 自动加 `--gpus all -e NVIDIA_DRIVER_CAPABILITIES=all -e LIBGL_ALWAYS_SOFTWARE=0`，并按 `s622.render` 标签在模式变化时自动重建。相机 4 fps → 12 fps（带 YOLO）／17.4 fps（`start_yolo:=false`，接近传感器 20 Hz 上限）。
    - 排查要点：容器内 `glxinfo -B` 的 renderer 必须是 NVIDIA；`nvidia-smi` 里要有 `ign gazebo server/gui`（Type=G）；缺 `graphics` 能力时会静默落回 llvmpipe
    - 瓶颈演化：先是渲染（llvmpipe）→ 换 GPU 后变成 YOLO CPU 推理 → 换 CUDA torch 后回到物理仿真 + llvmpipe GUI（见经验教训 14）
    - 细节见 [docs/环境部署_Ubuntu26.04_Docker.md](docs/环境部署_Ubuntu26.04_Docker.md) 第 8 节
12. **docker 免 sudo / 免审批**：`kz` 在 `docker` 组里，但补充组是**进程启动时继承**的，DSH 常驻进程一直没拿到 gid 117，所以每条 docker 命令都 `permission denied`、只能提权并逐条审批。解法是 socket ACL + `docker.socket` drop-in（`scripts/env/10-docker-socket-acl.conf`，见部署文档第 9 节）：`systemctl restart docker.socket` 会自动补 ACL，之后普通沙箱里 `docker ps` / `docker run --gpus all` 直接可用。注意 ACL 等价于 docker 组权限，docker API 等同 root。
    - 没有"按命令白名单放行"这种 DSH 能力（审批只有 `ask` / `never`），只能从 OS 权限层解决；`danger-full-access` 预设是全局关审批，别当默认
13. **GUI 3D 黑屏 → 拆分渲染**：开 GPU 后 Gazebo/RViz 菜单正常但 3D 视口全黑。原因是 GNOME Wayland + Intel/NVIDIA 双卡下，容器里经 Xwayland 用 NVIDIA GLX 渲染的窗口合成不到屏幕。现在 `s622_gazebo.launch.py`：Gazebo **服务器** `-s` 单独跑（相机传感器吃 GPU），Gazebo **GUI** 用 `ign gazebo -g` + `LIBGL_ALWAYS_SOFTWARE=1` + **`__GLX_VENDOR_LIBRARY_NAME=mesa`**，RViz 同样走 mesa。
    - 关键坑：只设 `LIBGL_ALWAYS_SOFTWARE=1` 无效，GLVND 仍选 NVIDIA；必须加 `__GLX_VENDOR_LIBRARY_NAME=mesa`，`glxinfo -B` 才会显示 `llvmpipe`
    - 相机 17.4 fps（`start_yolo:=false`）／14.7~15.9 fps（带 YOLO，YOLO 已切 CUDA 见 14）；GUI 18~31 fps
    - 调试时注意：NVIDIA 渲染下 `xwd` 抓到的 GL 区域恒为黑（DRI3 读不回来），**不能拿截图当"屏幕黑屏"的证据**；软件渲染下才抓得到
    - 细节见 [docs/环境部署_Ubuntu26.04_Docker.md](docs/环境部署_Ubuntu26.04_Docker.md) 第 10 节
14. **YOLO 切 CUDA（2026-10-29）**：`/opt/ros-yolo-venv` 从 `torch 2.14.1+cpu` 换成 `2.14.1+cu130`（PyPI Linux 轮子自带 CUDA 13 运行库）。相机 ~10 → **14.7~15.9 fps**，宿主 load 15 → 10，YOLO 日志 `Using device: 0`。
    - 坑 1：`pip install torch==2.14.1` **不会**替换 `2.14.1+cpu`（PEP 440 本地版本号），必须 `--force-reinstall`
    - 坑 2：`download.pytorch.org` 直连会卡死（下到 17 MB 不动），走 PyPI/清华镜像反而快；拿到的同样是 CUDA 版
    - 已 `docker commit` 进镜像（17.3 GB），`recreate` 不丢；`05_install_yolo.sh` 默认装 CUDA 版并带 CPU 回退
    - 细节见 [docs/环境部署_Ubuntu26.04_Docker.md](docs/环境部署_Ubuntu26.04_Docker.md) 第 11 节
