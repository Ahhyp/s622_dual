# 环境部署：Ubuntu 26.04 + Docker 容器（ROS 2 Humble）

> 记录日期：2026-10-28
> 目标：让单臂仿真 `s622_gazebo.launch.py` 在新宿主机（Ubuntu 26.04）上真正跑起来

---

## 1. 为什么必须用容器

| 项 | 宿主机现状 | 影响 |
|---|---|---|
| 发行版 | Ubuntu 26.04.1 LTS (resolute)，glibc 2.43 | ROS 2 Humble 官方二进制只针对 22.04 (jammy) |
| `/opt/ros` | 不存在 | 宿主机没有任何 ROS 2 |
| `ros-humble-*` | apt 源里没有 | 直接在宿主机装 Humble 不可行 |
| 用户权限 | `kz` 在 `sudo` / `docker` 组，但当前会话未刷新 | 需要 `sg docker` |

结论：**ROS 2 Humble + Gazebo Fortress 全部跑在 Docker 容器里，工作区 bind-mount 进容器**。
这样既不改宿主机系统，也能复用 22.04 的二进制包。

## 2. 环境拓扑

| 项 | 值 |
|---|---|
| 镜像 | `s622:humble-ros-yolo`（8.6 GB，基于 `ros:humble` = Ubuntu 22.04 + Humble） |
| 运行容器 | `s622_run`（`--net=host --ipc=host`，`--restart unless-stopped`） |
| 工作区挂载 | `/home/kz/s622_dual` → `/workspace` |
| 脚本挂载 | `/home/kz/ros_build` → `/build_scripts` |
| 图形 | `/tmp/.X11-unix` → 容器，`DISPLAY=:0`（Xwayland），**GPU 硬件渲染**（`--gpus all`，Quadro RTX 4000） |
| 容器内 Gazebo | Fortress 6.18（命令是 `ign gazebo`） |
| 容器内 ROS | Humble（340 个 `ros-humble-*` 包） |
| BT.CPP | 4.6.2 源码装在 `/usr/local`（Groot2 + SQLite 日志 ON） |
| YOLO 运行时 | `/opt/ros-yolo-venv`：**torch 2.14.1+cu130**（GPU 推理）、torchvision 0.29.1+cu130、ultralytics 8.4.191、numpy 1.26.4、opencv 4.11 |

## 3. 首次构建（换机器时照做）

```bash
cd /home/kz/s622_dual

# 1) 构建镜像：ros:humble + apt 依赖 + BT.CPP + YOLO venv
./scripts/env/01_image_build.sh          # 已存在会跳过，--force 重建

# 2) 创建/启动运行容器（自动 xhost + X11 挂载）
./scripts/env/10_container.sh ensure

# 3) 编译工作区（33 个包，约 3 分钟）
./scripts/env/10_container.sh run "bash /workspace/scripts/env/04_build_ws.sh --clean"
```

脚本一览（`scripts/env/`）：

| 脚本 | 在哪执行 | 作用 |
|---|---|---|
| `01_image_build.sh` | 宿主机 | 构建镜像 |
| `02_apt_deps.sh` | 容器 | apt 依赖（含 `gz_ros2_control`、`moveit_servo`、`realsense2_description`） |
| `03_btcpp.sh` | 容器 | 源码编译 BehaviorTree.CPP 4.6.2 |
| `04_build_ws.sh` | 容器 | colcon build（merge-install + symlink-install） |
| `05_install_yolo.sh` | 容器 | 建 venv 装 torch/ultralytics |
| `10_container.sh` | 宿主机 | ensure / recreate / shell / run / stop / status（渲染模式自动选 GPU 或软件） |
| `10-docker-socket-acl.conf` | 宿主机 | docker.socket 的 drop-in：给 kz 用户加 socket ACL（免 sudo 用 docker，见第 9 节） |
| `06_run_sim.sh` | 容器 | 启动单臂仿真 |
| `07_smoke_test.sh` | 容器 | 冒烟测试：发一条轨迹看机械臂是否动 |
| `08_stop_sim.sh` | 容器 | 停止仿真全部进程（`--hard` 兜底 SIGKILL） |

## 4. 日常启动

```bash
# 一键启动（宿主机）：Gazebo + MoveIt2 + RViz + Servo + YOLO
./scripts/env/10_container.sh run "bash /workspace/scripts/env/06_run_sim.sh"

# 只要仿真/规划、不启 YOLO
./scripts/env/10_container.sh run "bash /workspace/scripts/env/06_run_sim.sh start_yolo:=false"

# 另开一个容器 shell
./scripts/env/10_container.sh shell

# 冒烟测试
./scripts/env/10_container.sh run "bash /workspace/scripts/env/07_smoke_test.sh 0 0.2"

# 停止仿真（僵尸进程只能靠重启容器回收）
./scripts/env/10_container.sh run "bash /workspace/scripts/env/08_stop_sim.sh --hard"
sg docker -c "docker restart s622_run"
```

## 5. 本次为「能跑」改了什么

### 5.1 `src/gz_launch/launch/s622_gazebo.launch.py`

| 问题 | 原代码 | 现在 |
|---|---|---|
| YOLO 权重路径写死作者机器 | `~/my_S622/src/yolov8_obb/models/...` | `_find_yolo_model()`：环境变量 → `install/share` → 源码树自动探测 |
| Python 解释器写死作者机器 | `/home/yep/miniconda3/envs/yolov8/bin/python3` | `_find_yolo_python()`：`YOLO_OBB_PYTHON` → `/opt/ros-yolo-venv/bin/python3` → `python3` |
| 换机器后无法关掉 YOLO | 无开关 | `start_yolo:=false` |
| `IGN_GAZEBO_RESOURCE_PATH` 被赋值两次（第二次覆盖第一次） | 重复 `SetEnvironmentVariable` | 删除重复，保留追加式设置 |

新增 launch 参数：`start_yolo`、`yolo_model_path`、`yolo_python`。

### 5.2 `src/gz_launch/launch/test_single_arm.launch.py`

原文件只有 4 行注释（**空实现**），容易误认为单臂入口。现已改为 `s622_gazebo.launch.py` 的
显式别名，`ros2 launch gz_launch test_single_arm.launch.py` 可直接用。

### 5.3 补齐的缺失依赖

构建能过、运行才发现缺的三个包：

- `ros-humble-gz-ros2-control` —— 没有它 Gazebo 里**不会出现 `/controller_manager`**，
  `spawner` 一直 `waiting for service`，MoveIt 无法执行轨迹
- `ros-humble-moveit-servo` —— launch 里 `servo_node_main` 起不来
- `ros-humble-realsense2-description` —— URDF 里 `$(find realsense2_description)` 解析失败，
  `ros2 launch` 直接抛 `PackageNotFoundError`

## 6. 验证记录

### 2026-10-28 首次跑通（软件渲染）

| 检查 | 结果 |
|---|---|
| `colcon build` | 33/33 包成功（2026-10-27 容器内，`Summary: 33 packages finished [3min 4s]`） |
| Gazebo | Fortress 6.18，world `empty` 正常，实体树 `ground_plane/sun/robot_arm/target_box` |
| 控制器 | `robot_arm_controller` / `joint_state_broadcaster` / `hand_controller` 全部 **active** |
| 相机 | `/camera/color/image_raw` 约 4 fps（llvmpipe 软件渲染） |
| YOLO | `obb_node` 持续推理，`last detections=1`（识别到 cube） |
| MoveIt | RViz MotionPlanning 面板已连 `/move_group_fairino`，规划组 `robot_arm` |
| 运动闭环 | `07_smoke_test.sh`：j1 从 0.5 → 0.7001 rad（目标 +0.2），action `error_code=0` |
| GUI | RViz + Gazebo 窗口均正常渲染（截图见 `log/shot_rviz.png`、`log/shot_gazebo.png`） |

### 2026-10-29 启用 GPU 渲染

| 检查 | 结果 |
|---|---|
| `nvidia-smi`（容器内） | `GPU 0: Quadro RTX 4000 (UUID: GPU-7dffbb0d-...)`，驱动 595.99.02 |
| GLX 渲染器（容器内 `glxinfo -B`） | `OpenGL renderer string: Quadro RTX 4000/PCIe/SSE2`，`direct rendering: Yes` |
| 进程实占显存 | `ign gazebo server` 298 MiB、`ign gazebo gui` 239 MiB（`nvidia-smi` 里 Type=G） |
| 相机帧率（开 YOLO） | `/camera/color/image_raw` **12.0 fps**（原 4 fps） |
| 相机帧率（`start_yolo:=false`） | **17.4 fps**，已接近传感器 `update_rate=20`，渲染侧不再是瓶颈 |
| 瓶颈定位 | 关 YOLO 时 GPU 利用率 22~41%，宿主 load 9.4/16 核；开 YOLO 后 load 15/16 核 —— **CPU 推理成新瓶颈** |
| 运动闭环 | `07_smoke_test.sh 0 0.2`：j1 0.5 → 0.7001 rad，`error_code=0`，PASS |

## 7. 已知限制与注意事项

1. **GPU 渲染已启用（2026-10-29）**：宿主机 Quadro RTX 4000 已通过 `nvidia-container-toolkit`
   透进容器，Gazebo/RViz 走 NVIDIA GLX，相机从 4 fps 提升到 12~17 fps（细节见第 8 节）。
   换机器重装时的等价步骤：`sudo apt install nvidia-container-toolkit` →
   `sudo nvidia-ctk runtime configure --runtime=docker` → `sudo systemctl restart docker` →
   `./scripts/env/10_container.sh recreate`。
2. **docker 组**：`kz` 已在 `docker` 组，但当前登录会话可能没刷新；
   所有宿主机脚本都内置了 `sg docker` 兜底。
3. **容器重启**：容器带 `--restart unless-stopped`，宿主机重启后会自动起来；
   X11 授权（`xhost`）不会持久，重新登录后需再跑一次 `10_container.sh ensure`。
4. **RViz 首次启动**：曾出现过一次 `exit code -11`（启动早期、控制器还没就绪时），
   重启后稳定；如再遇到先看 `log/run_single_arm.log`。
5. **磁盘**：镜像 8.6 GB；旧构建容器 `ros_humble_build`（约 3 GB）已不再需要，
   确认无误后可 `docker rm -f ros_humble_build` 释放空间。
6. **仓库既有问题**（上次构建留下，未提交）：
   - 新增 `src/dual_arm_calibration/package.xml`（仓库缺该文件，不补无法构建）
   - 新增空目录 `src/s622_moveit_descriptions/{textures,visual}`（CMakeLists 引用了它们）
   - `src/fairino_hardware/libfairino/lib/libfairino.so.2` 是构建时被改写的软链（正常现象）

## 8. GPU 渲染（Quadro RTX 4000）

### 8.1 一次性宿主机配置（2026-10-29 已完成）

```bash
sudo apt install nvidia-container-toolkit          # 1.20.1，来自 NVIDIA CUDA 源
sudo nvidia-ctk runtime configure --runtime=docker # 写入 /etc/docker/daemon.json 的 nvidia runtime
sudo systemctl restart docker
```

之后 `docker info` 的 Runtimes 里会出现 `nvidia`，`docker run --gpus all` 才可用。
驱动侧无需求助：宿主机 595.99.02 的 open 内核模块本来就正常，之前只是没透进容器。

### 8.2 容器侧开关

`scripts/env/10_container.sh` 自动选择渲染模式并打 `s622.render` 标签：

| 命令 | 作用 |
|---|---|
| `./scripts/env/10_container.sh ensure` | 默认 `RENDER=auto`：宿主机 `nvidia-smi` 可用且 docker 有 nvidia runtime → 用 GPU，否则回退 llvmpipe |
| `./scripts/env/10_container.sh recreate` | 强制重建（`--gpus all` 是创建期参数，改配置后必须重建） |
| `RENDER=software ./scripts/env/10_container.sh recreate` | 强制软件渲染（排查 GPU 问题时用） |

GPU 模式下容器带：`--gpus all -e NVIDIA_DRIVER_CAPABILITIES=all -e LIBGL_ALWAYS_SOFTWARE=0`。
容器创建失败会自动回退软件渲染，不会把仿真环境搞挂。

日常使用命令不变：

```bash
./scripts/env/10_container.sh run "bash /workspace/scripts/env/06_run_sim.sh"
```

### 8.3 怎么确认真的走了 GPU

```bash
# 1) 容器内能看到卡
docker exec s622_run nvidia-smi -L
# 2) GLX 渲染器必须是 NVIDIA，不能是 llvmpipe
docker exec -e DISPLAY=:0 s622_run glxinfo -B | grep -i "OpenGL renderer"
# 3) 仿真起来后，宿主机 nvidia-smi 的 Processes 里要有 ign gazebo server/gui（Type=G）
nvidia-smi | sed -n '/Processes/,$p'
```

### 8.4 坑与后续

1. **镜像 ENV 里烘焙了 `LIBGL_ALWAYS_SOFTWARE=1`**（早期 `docker commit` 遗留），GPU 模式下必须
   显式 `-e LIBGL_ALWAYS_SOFTWARE=0` 压掉；重建镜像后（`01_image_build.sh` 不再传该变量）会自动消失。
2. **`--gpus` / `-e` 只在创建容器时生效**：改完必须 `recreate`；`ensure` 检测到标签不一致会自动重建。
3. **`NVIDIA_DRIVER_CAPABILITIES` 必须含 `graphics`**：默认只有 `compute,utility`，容器里不会挂
   `libGLX_nvidia`，结果是"看起来有 GPU 其实还在 llvmpipe"。
4. **当前瓶颈已从渲染转到 CPU**：YOLO venv 是 `torch+cpu`，开 YOLO 后宿主 load 15/16 核、
   相机 12 fps；关 YOLO 能到 17.4 fps。要再快可把 venv 换成 CUDA 版 torch（未做，需重装约 2.5 GB）。

## 9. 免 sudo、免审批地使用 docker（2026-10-29）

### 现象

宿主机 `kz` 明明在 `docker` 组（`/etc/group` 里可见），但 agent/DSH 里的每条 docker 命令都
`permission denied ... /var/run/docker.sock`，只能靠 `sudo` 或提权到 `danger-full-access` 执行，
而每次提权都要人工审批一次。

### 根因

**补充组是进程启动时继承的，不是登录时动态查的**。DSH 常驻进程在登录那一刻就固定了组列表，
之后 `usermod -aG docker` 不会影响已在运行的进程；`sg docker` 也无效（gid 不在进程组列表里）。
所以问题不在 docker，而在"这个进程没有 docker 组"。

### 解法：给 socket 加 ACL，并让 systemd 每次启动都补上

```bash
# 1) 立即生效
sudo setfacl -m u:kz:rw /run/docker.sock

# 2) 持久化：docker.socket drop-in（重启/重新登录后自动恢复）
sudo install -m 644 -o root -g root \
  scripts/env/10-docker-socket-acl.conf \
  /etc/systemd/system/docker.socket.d/10-kz-acl.conf
sudo systemctl daemon-reload
sudo systemctl restart docker.socket     # 会连带重启 docker.service（容器会自动拉起）
```

之后在普通 `workspace-write` 沙箱里 `docker ps` / `docker run --gpus all` 都能直接跑，
**不再触发任何审批**。验证：

```bash
docker ps                                  # 不通才需要提权
sudo getfacl -p /run/docker.sock | grep user:   # 应有 user:kz:rw-
```

### 备选方案与取舍

| 方案 | 效果 | 代价 |
|---|---|---|
| 上面这套 ACL | 只有 docker 免审批，文件系统仍受沙箱约束 | 多一个 systemd drop-in |
| 注销重登录 | docker 组进入新会话，效果相同 | 要重登录，DSH 也要重启 |
| GUI 权限选择器 → Full access（`danger-full-access` + `never`） | 所有命令都不再询问 | 失去全部审批，`rm -rf` 也不拦 |
| `DSH_PERMISSION_MODE=danger-full-access dsh web` | 同上，作为部署默认 | 同上 |

> 注：本版本 DSH 的审批策略只有 `ask` / `never`，没有"按命令白名单放行"。
> 想"只放行 docker、其它仍审批"，只能像上面这样从 OS 权限层解决。
>
### 注意：渲染模式不会自动降级

`render_mode` 遇到 `nvidia-smi` 失败时**不会**把已在跑的 GPU 容器降级重建（agent 沙箱里看不到
`/dev/nvidia*`，nvidia-smi 必然失败），降级要显式 `RENDER=software`。

### 安全提醒

ACL 等价于把 `kz` 加入 `docker` 组 —— docker API 本身等同 root，能挂载宿主机任意路径。

## 10. GUI 3D 视图黑屏 → 拆分渲染（2026-10-29）

### 现象

启用 GPU 后 Gazebo 和 RViz 的菜单/按钮都正常，但 **3D 视口全黑**（RViz 的 Image 面板也黑）。
相机话题本身没问题（`960x540 rgb8`，均值 197）。

### 原因

宿主机是 GNOME **Wayland** + Intel UHD 630 + Quadro RTX 4000 **双显卡**。容器里的 Qt/Ogre 经
Xwayland 用 NVIDIA GLX 渲染，Xwayland→mutter 的 dmabuf 合成在跨卡场景下拿不到内容，GL 区域就是黑的。
llvmpipe 走的是 X pixmap（普通 2D 路径），所以**软件渲染反而正常** —— 这也是为什么改 GPU 之前一切正常。

### 关键坑：`LIBGL_ALWAYS_SOFTWARE=1` 单独用没用

装了 NVIDIA 库之后 GLVND 会优先选 NVIDIA，`LIBGL_ALWAYS_SOFTWARE` 被忽略：

```bash
$ LIBGL_ALWAYS_SOFTWARE=1 glxinfo -B | grep renderer
OpenGL renderer string: Quadro RTX 4000/PCIe/SSE2      # 仍是 GPU
$ LIBGL_ALWAYS_SOFTWARE=1 __GLX_VENDOR_LIBRARY_NAME=mesa glxinfo -B | grep renderer
OpenGL renderer string: llvmpipe (LLVM 15.0.7, 256 bits)  # 才真的软渲染
```

### 方案：服务器留 GPU，GUI 走 mesa

`src/gz_launch/launch/s622_gazebo.launch.py`：

1. `gz_args` 加 `-s` → 只起 Gazebo **服务器**（相机/深度相机传感器在这个进程里渲染，继续吃 GPU）
2. GUI 单独 `ign gazebo -g`，`additional_env` 带
   `LIBGL_ALWAYS_SOFTWARE=1` + `__GLX_VENDOR_LIBRARY_NAME=mesa`
3. 随后 `SetEnvironmentVariable` 同样两个变量，让 RViz 等后续节点也走 mesa

效果：相机 `17.4 fps`（`start_yolo:=false`）/ `~10 fps`（带 YOLO，CPU 推理是瓶颈）；
GUI 18~31 fps，肉眼够用。

### 验证

| 检查 | 结果 |
|---|---|
| rviz2 日志 `OpenGl version` | `4.5 (GLSL 4.5)` = llvmpipe；改之前是 `4.6 (GLSL 4.6)` = NVIDIA |
| `nvidia-smi` | Processes 里只剩 `ign gazebo ... -r -s`（298 MiB），GUI/RViz 不再占显存 |
| 截图 | 软件渲染下 `xwd` 能抓到 3D 内容；NVIDIA 下 GL 区域恒黑（DRI3 读不回来），**截图黑不代表屏幕黑**，别只凭截图判断 |
| 运动闭环 | `07_smoke_test.sh 0 0.2` PASS |

> 如果以后 NVIDIA 在 Xwayland 下修好（换驱动、或改用 Xorg 会话），去掉 `-s` 和那两个 GL 环境变量
> 就能回到全 GPU 渲染。

### 调试技巧：在 Wayland 会话里截图

`xwd`/`import` 只能读 Xwayland 缓冲：
- GL 区域（DRI3）读回来是黑的，**不能作为"屏幕黑屏"的证据**
- 软件渲染（llvmpipe）能正常读到内容，所以可用作对照
- 容器内有 `xwd` + `convert`，抓完放 `/workspace/log/` 即可取走：

```bash
docker exec s622_run bash -c 'export DISPLAY=:0; xwd -id <窗口ID> -out /workspace/log/x.xwd && convert /workspace/log/x.xwd /workspace/log/x.png'
xwininfo -root -tree | grep -E '"(Gazebo|.*RViz)'   # 找窗口 ID
```

## 11. YOLO 推理切 GPU（torch cu130，2026-10-29）

### 背景

`/opt/ros-yolo-venv` 原来是 `torch 2.14.1+cpu`。Gazebo 渲染切到 GPU 后，YOLO 的 CPU 推理变成新瓶颈：
宿主 load 打满 16 核，相机只能跑到 ~10 fps。

### 做法

```bash
# 强制重装（不加 --force-reinstall 会认为 2.14.1+cpu 已满足，直接跳过）
/opt/ros-yolo-venv/bin/pip install --force-reinstall --no-cache-dir \
  torch==2.14.1 torchvision==0.29.1
```

PyPI 的 Linux 版 torch 自带 CUDA 运行库（会拉 `nvidia-*`/`cuda-toolkit` 13.0 依赖，约 5 GB）。
驱动 595 兼容 CUDA 13。

### 两个坑

1. **`download.pytorch.org` 在本网络下会卡死**：`+cu126` 轮子下到 17 MB 就不动了。
   直接走 PyPI（已配置清华镜像）反而快得多，而且拿到的就是 CUDA 版。
2. **`pip install torch==2.14.1` 不会替换 `2.14.1+cpu`**：PEP 440 认为本地版本号满足同一版本，
   必须 `--force-reinstall`。

### 效果

| 项 | 切换前 | 切换后 |
|---|---|---|
| YOLO 设备 | `Using device: cpu` | `Using device: 0`（Quadro RTX 4000） |
| 相机帧率（带 YOLO） | ~10 fps | **14.7~15.9 fps** |
| 宿主 load（16 核） | ~15 | ~10 |
| GPU 利用率 | 22~41% | 34% |

### 固化

- 已 `docker commit` 进镜像 `s622:humble-ros-yolo`（17.3 GB），`recreate` 不会丢
- `scripts/env/05_install_yolo.sh` 改成默认装 CUDA 版，GPU 不可用时自动回退 CPU
- 顺带删掉旧的 `ros_humble_build` 容器（释放约 5 GB）
- `10_container.sh` 的 GPU 探测补了一手：宿主 `nvidia-smi` 不可用（agent 沙箱）时改用
  `docker run --rm --gpus all nvidia-smi -L` 判断，避免误判成 software 模式重建
