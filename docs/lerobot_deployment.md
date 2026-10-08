# LeRobot 开发环境部署文档 (RTX 4060 Ti, 直接部署)

## 1. 系统信息

| 项目        | 详情                               |
| ----------- | ---------------------------------- |
| OS          | Ubuntu 22.04.3 LTS (Jammy)         |
| 内核        | 5.4.0-155-generic                  |
| GPU         | NVIDIA GeForce RTX 4060 Ti (16 GB) |
| NVIDIA 驱动 | 535.86.10                          |
| CUDA 版本   | 12.2 (驱动) / 12.1 (toolkit)       |
| 内存        | 125 GiB                            |
| 磁盘        | overlay 30G (已用 15G, 可用 16G)   |
| Conda       | 26.5.3 (安装于 /opt/miniconda3)    |

## 2. 核心安装版本

| 包            | 版本         | 说明                                 |
| ------------- | ------------ | ------------------------------------ |
| Python        | 3.12.13      | conda-forge                          |
| PyTorch       | 2.10.0+cu128 | pip 自动升级 (LeRobot 依赖要求 ≥2.7) |
| torchvision   | 0.25.0+cu128 |                                      |
| LeRobot       | 0.5.1        | PyPI                                 |
| MuJoCo        | 3.8.1        | PyPI (随 gym-aloha 自动安装)         |
| dm-control    | 1.0.41       | PyPI (随 gym-aloha 自动安装)         |
| gym-aloha     | 0.1.4        | PyPI                                 |
| gym-pusht     | 0.1.6        | PyPI                                 |
| transformers  | 5.3.0        | PyPI (随 lerobot[pi] 自动安装)       |
| FFmpeg        | 7.1.1        | conda-forge                          |
| numpy         | 2.2.6        | PyPI                                 |
| scipy         | 1.18.0       | PyPI                                 |
| opencv-python | 5.0.0.93     | PyPI                                 |

## 3. 部署步骤（重现用）

### 3.1 系统依赖
```bash
apt-get update
apt-get install -y libegl1 libegl1-mesa libegl-mesa0 libglfw3 \
  libosmesa6 libglib2.0-0 libgl1-mesa-glx \
  cmake build-essential python3-dev pkg-config \
  libavformat-dev libavcodec-dev libavdevice-dev libavutil-dev \
  libswscale-dev libswresample-dev libavfilter-dev
```

### 3.2 安装 Miniconda
```bash
curl -fsSL -o /tmp/miniconda_installer.sh \
  https://mirrors.tuna.tsinghua.edu.cn/anaconda/miniconda/Miniconda3-latest-Linux-x86_64.sh
bash /tmp/miniconda_installer.sh -b -p /opt/miniconda3
```

### 3.3 配置镜像
Conda (`~/.condarc`): 清华镜像
```yaml
auto_activate_base: false
show_channel_urls: true
channels:
  - https://mirrors.tuna.tsinghua.edu.cn/anaconda/cloud/conda-forge
  - https://mirrors.tuna.tsinghua.edu.cn/anaconda/pkgs/main
  - https://mirrors.tuna.tsinghua.edu.cn/anaconda/pkgs/r
  - defaults
```

Pip (`~/.pip/pip.conf`): 阿里云镜像
```ini
[global]
index-url = https://mirrors.aliyun.com/pypi/simple/
trusted-host = mirrors.aliyun.com
timeout = 180
retries = 10
disable-pip-version-check = true
```
> **注意**: LeRobot 和 PyTorch 安装时必须用 `-i https://pypi.org/simple/` 直连，清华/阿里镜像对部分包返回 403 或极慢。

### 3.4 创建环境
```bash
# 步骤 1: 创建基础环境
conda create -n lerobot python=3.12 pip -y
conda activate lerobot
conda install -c conda-forge ffmpeg=7.1.1 -y

# 步骤 2: 安装 LeRobot
#    pip 会自动解决 PyTorch 依赖并安装兼容版本 (本环境最终为 2.10.0+cu128)
pip install --no-cache-dir lerobot[aloha,pusht,pi]==0.5.1 -i https://pypi.org/simple/
```


### 3.5 验证

```bash
# 环境验证 (必须用 env var 或脚本内 setdefault 在 import 前设置 EGL)
MUJOCO_GL=egl PYOPENGL_PLATFORM=egl python /root/lerobot_smoke_test.py

# 训练流程验证
python /root/lerobot_training_smoke_test.py
```

## 4. 快速激活环境
```bash
source /opt/miniconda3/etc/profile.d/conda.sh
conda activate lerobot
```

## 5. 关键文件路径

| 文件           | 路径                                 |
| -------------- | ------------------------------------ |
| Conda 环境     | /opt/miniconda3/envs/lerobot         |
| Conda 安装     | /opt/miniconda3                      |
| 环境冒烟测试   | /root/lerobot_smoke_test.py          |
| 训练流程测试   | /root/lerobot_training_smoke_test.py |
| pip freeze     | /root/lerobot_requirements.txt       |
| 部署文档       | /root/lerobot_deployment.md          |
| Conda 镜像配置 | /root/.condarc                       |
| Pip 镜像配置   | /root/.pip/pip.conf                  |

## 6. 故障排查

### EGL 渲染失败 "GLFW / DISPLAY / X11" 错误

```
错误: The DISPLAY environment variable is missing
     or GLFW library is not initialized
根因: MUJOCO_GL 环境变量必须在 import mujoco 之前设置
```

- ✅ 脚本内设置：`os.environ.setdefault("MUJOCO_GL", "egl")` 放在所有 import **之前**
- ✅ 命令行传入：`MUJOCO_GL=egl PYOPENGL_PLATFORM=egl python script.py`

### 磁盘空间不足 "No space left on device"

```bash
# 清理 pip 缓存 (可释放数 GB)
rm -rf /root/.cache/pip /tmp/pip-*
# 安装时禁用缓存
pip install --no-cache-dir <package>
```

### 清华镜像 HTTP 403 / 阿里镜像超时

临时切换到 PyPI 直连：`pip install <pkg> -i https://pypi.org/simple/`

### 安装时 "pip check" 报 broken requirements

通常是 pip 升级 PyTorch 时部分替换残留。运行：
```bash
pip uninstall torch torchvision -y && pip install lerobot[aloha,pusht,pi]==0.5.1 -i https://pypi.org/simple/
```

## 7. 与旧环境 (ACT/MuJoCo Docker) 的区别

| 方面     | 旧环境             | 新环境              |
| -------- | ------------------ | ------------------- |
| 部署方式 | Docker 容器        | 直接部署 (Conda)    |
| GPU      | RTX 4090 D (24 GB) | RTX 4060 Ti (16 GB) |
| 框架     | ACT                | LeRobot             |
| Python   | 3.10               | 3.12                |
| PyTorch  | 2.4.0+cu121        | 2.10.0+cu128        |
| MuJoCo   | 2.3.7              | 3.8.1               |
| 备份方式 | Docker push ACR    | 平台网站备份镜像    |

## 8. 注意事项

1. **磁盘仅 30G**：pip 安装必须使用 `--no-cache-dir`，训练数据集考虑外部存储
2. **驱动兼容**：NVIDIA 535 驱动 + PyTorch 2.10 cu128 可正常运行 (前向兼容)
3. **EGL 渲染**：必须在 mujoco import 之前设置 `MUJOCO_GL=egl`
4. **4060 Ti 16G VRAM**：训练大模型时注意 batch size 和模型大小
5. **PyTorch 版本**：pip 安装 LeRobot 时会自动升级 PyTorch，不需要手动指定版本
