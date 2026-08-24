# ACT/MuJoCo 开发环境部署文档

## 1. 系统信息

| 项目        | 详情                              |
| ----------- | --------------------------------- |
| 服务器      | `I29ed08025d00701752`             |
| OS          | Ubuntu 22.04.3 LTS (Jammy)        |
| 内核        | 5.4.0-107-generic                 |
| GPU         | NVIDIA GeForce RTX 4090 D (24 GB) |
| NVIDIA 驱动 | 575.57.08                         |
| CUDA 版本   | 12.9 (驱动) / 12.1 (toolkit)      |
| 内存        | 125 GiB                           |
| Conda       | 26.5.3 (安装于 /opt/miniconda3)   |
| Docker      | 29.1.3 (vfs 存储驱动, 无特权模式) |

## 2. 项目目录结构

```
/root/s622_embodied/
├── README_ENVIRONMENT.md          # 本文件
├── docker/
│   ├── Dockerfile                 # 镜像构建定义
│   ├── requirements.lock.txt      # pip 精确依赖
│   ├── smoke_test.py              # 容器内冒烟测试
│   ├── entrypoint.sh              # 容器入口脚本
│   ├── .dockerignore              # 构建排除规则
│   └── README.md                  # Docker 构建说明
├── scripts/
│   ├── smoke_test.py              # Conda 环境冒烟测试
│   ├── push_image.sh              # ACR 推送脚本
│   └── pull_and_run.sh            # ACR 拉取并运行脚本
├── locks/
│   ├── act_reference_pip_freeze.txt  # pip freeze 输出
│   └── act_reference_conda.yml       # conda env export
├── wheelhouse/                    # pip wheel 缓存
└── setup_logs/
    ├── system_check.log           # 系统检查日志
    ├── conda_smoke_test.log       # Conda 冒烟测试日志
    ├── conda_install.log          # Conda 安装日志
    ├── docker_build.log           # Docker 构建日志
    └── docker_smoke_test.log      # Docker 冒烟测试日志
```

## 3. 实际安装版本

### 核心依赖

| 包                     | 版本         | 来源                    |
| ---------------------- | ------------ | ----------------------- |
| Python                 | 3.10.20      | conda-forge             |
| torch                  | 2.4.0+cu121  | PyTorch CUDA 12.1 wheel |
| torchvision            | 0.19.0+cu121 | PyTorch CUDA 12.1 wheel |
| mujoco                 | 2.3.7        | PyPI (manylinux)        |
| dm-control             | 1.0.9        | PyPI                    |
| numpy                  | 1.24.4       | PyPI                    |
| scipy                  | 1.10.1       | PyPI                    |
| h5py                   | 3.8.0        | PyPI                    |
| opencv-python-headless | 4.7.0.72     | PyPI                    |
| einops                 | 0.8.2        | PyPI                    |
| wandb                  | 0.28.1       | PyPI                    |
| ipython                | 8.39.0       | PyPI                    |
| packaging              | 26.2         | conda-forge             |

### NVIDIA CUDA 组件 (随 PyTorch 安装)

nvidia-cublas-cu12 12.1.3.1, nvidia-cuda-cupti-cu12 12.1.105,
nvidia-cuda-nvrtc-cu12 12.1.105, nvidia-cuda-runtime-cu12 12.1.105,
nvidia-cudnn-cu12 9.1.0.70, nvidia-cufft-cu12 11.0.2.54,
nvidia-curand-cu12 10.3.2.106, nvidia-cusolver-cu12 11.4.5.107,
nvidia-cusparse-cu12 12.1.0.106, nvidia-nccl-cu12 2.20.5,
nvidia-nvtx-cu12 12.1.105, triton 3.0.0

## 4. Conda 环境创建过程

### 4.1 安装 Miniconda

```bash
curl -fsSL -o /tmp/miniconda_installer.sh \
  https://mirrors.tuna.tsinghua.edu.cn/anaconda/miniconda/Miniconda3-latest-Linux-x86_64.sh
bash /tmp/miniconda_installer.sh -b -p /opt/miniconda3
```

### 4.2 配置国内镜像

**Conda** (`~/.condarc`):
```yaml
auto_activate_base: false
show_channel_urls: true
channels:
  - https://mirrors.tuna.tsinghua.edu.cn/anaconda/cloud/conda-forge
  - https://mirrors.tuna.tsinghua.edu.cn/anaconda/pkgs/main
  - https://mirrors.tuna.tsinghua.edu.cn/anaconda/pkgs/r
  - defaults
```

**pip** (`~/.pip/pip.conf`):
```ini
[global]
index-url = https://mirrors.aliyun.com/pypi/simple/
trusted-host = mirrors.aliyun.com
timeout = 180
retries = 10
disable-pip-version-check = true
```

> **注意**: PyTorch CUDA wheel 必须通过 `--index-url https://download.pytorch.org/whl/cu121` 安装，
> 不能使用阿里云镜像替代。如需加速，可使用 `--extra-index-url https://pypi.tuna.tsinghua.edu.cn/simple/`。

### 4.3 创建环境

```bash
conda create -n act_reference python=3.10 pip -y

# 升级基础工具
python -m pip install --upgrade pip setuptools wheel

# 安装兼容版本 numpy/scipy
python -m pip install numpy==1.24.4 scipy==1.10.1 \
  -i https://pypi.tuna.tsinghua.edu.cn/simple/

# 安装 PyTorch (CUDA 12.1)
python -m pip install torch==2.4.0 torchvision==0.19.0 \
  --index-url https://download.pytorch.org/whl/cu121 \
  --extra-index-url https://pypi.tuna.tsinghua.edu.cn/simple/

# 安装 MuJoCo/ACT 依赖
python -m pip install \
  mujoco==2.3.7 dm-control==1.0.9 \
  opencv-python-headless==4.7.0.72 h5py==3.8.0 \
  einops packaging wandb ipython \
  -i https://pypi.tuna.tsinghua.edu.cn/simple/
```

### 4.4 安装 EGL 系统库

```bash
apt-get install -y libegl1 libegl1-mesa libegl-mesa0 libglfw3 libosmesa6 libglib2.0-0
```

## 5. 冒烟测试结果

### Conda 环境测试

```bash
MUJOCO_GL=egl PYOPENGL_PLATFORM=egl \
  /opt/miniconda3/envs/act_reference/bin/python \
  scripts/smoke_test.py
```

输出: **ALL_SMOKE_TESTS_PASSED** (28/28 通过)

测试覆盖:
- [PASS] Python 3.10 ✓
- [PASS] torch 2.4.0+cu121, CUDA available ✓
- [PASS] GPU: NVIDIA GeForce RTX 4090 D ✓
- [PASS] GPU tensor matmul ✓
- [PASS] mujoco 2.3.7 ✓
- [PASS] dm-control 1.0.9, reset + step ✓
- [PASS] MuJoCo EGL 离屏渲染 (128,128,3) ✓
- [PASS] pip check 无依赖错误 ✓

## 6. Docker 构建

### 6.1 前置条件

- Docker Engine (已安装: 29.1.3)
- NVIDIA Container Toolkit (已安装: 1.19.1)
- 基础镜像 `pytorch/pytorch:2.4.0-cuda12.1-cudnn9-runtime` 可在本地或 ACR 访问

### 6.2 构建命令

```bash
# 使用 Docker Hub 基础镜像 (需要外网)
DOCKER_BUILDKIT=1 docker build \
  -t act-reference:torch2.4-cu121-mj2.3.7-v1 \
  /root/s622_embodied/docker

# 使用私有 ACR 基础镜像 (国内推荐)
DOCKER_BUILDKIT=1 docker build \
  --build-arg BASE_IMAGE=${ACR_REGISTRY}/${ACR_NAMESPACE}/pytorch:2.4.0-cuda12.1-cudnn9-runtime \
  -t act-reference:torch2.4-cu121-mj2.3.7-v1 \
  /root/s622_embodied/docker
```

### 6.3 基础镜像同步

如果当前服务器无法访问 Docker Hub，需要从有权限的机器同步基础镜像:

```bash
# 1. 在有外网的机器上拉取并推送
docker pull pytorch/pytorch:2.4.0-cuda12.1-cudnn9-runtime
docker tag pytorch/pytorch:2.4.0-cuda12.1-cudnn9-runtime \
  ${ACR_REGISTRY}/${ACR_NAMESPACE}/pytorch:2.4.0-cuda12.1-cudnn9-runtime
docker push ${ACR_REGISTRY}/${ACR_NAMESPACE}/pytorch:2.4.0-cuda12.1-cudnn9-runtime

# 2. 在本机使用 ACR 基础镜像构建
```

## 7. Docker GPU 测试

```bash
docker run --rm \
  --gpus all \
  --shm-size=8g \
  -e MUJOCO_GL=egl \
  -e PYOPENGL_PLATFORM=egl \
  act-reference:torch2.4-cu121-mj2.3.7-v1 \
  python /opt/smoke_test.py
```

预期: `ALL_SMOKE_TESTS_PASSED`

## 8. 镜像 Tag 规范

| Tag                                  | 含义             |
| ------------------------------------ | ---------------- |
| `torch2.4-cu121-mj2.3.7-v1`          | 稳定版本 tag     |
| `torch2.4-cu121-mj2.3.7-v1-YYYYMMDD` | 带日期的版本 tag |
| 未来: `torchX.X-cuXXX-mjX.X.X-vN`    | 升级后新版本     |

## 9. ACR 推送和拉取

### 9.1 推送

```bash
export ACR_REGISTRY="<your-registry>.cn-hangzhou.aliyuncs.com"
export ACR_NAMESPACE="<your-namespace>"
export ACR_USERNAME="<your-username>"
export ACR_PASSWORD="<your-password>"

bash scripts/push_image.sh
```

### 9.2 拉取并运行

```bash
bash scripts/pull_and_run.sh
```

或者手动:
```bash
docker pull ${ACR_REGISTRY}/${ACR_NAMESPACE}/act-reference:torch2.4-cu121-mj2.3.7-v1

docker run --rm -it --gpus all --shm-size=8g \
  -e MUJOCO_GL=egl -e PYOPENGL_PLATFORM=egl \
  ${ACR_REGISTRY}/${ACR_NAMESPACE}/act-reference:torch2.4-cu121-mj2.3.7-v1 \
  bash
```

## 10. 项目目录挂载方式

推荐运行命令:

```bash
docker run --rm -it \
  --gpus all \
  --shm-size=8g \
  -e MUJOCO_GL=egl \
  -e PYOPENGL_PLATFORM=egl \
  -v /root/s622_embodied:/workspace/act \
  -v /data:/data \
  -v /checkpoints:/checkpoints \
  -w /workspace/act \
  <镜像地址> \
  bash
```

## 11. 数据集和 Checkpoint 挂载

```bash
# 数据集目录
-v /path/to/datasets:/data

# Checkpoint 目录
-v /path/to/checkpoints:/checkpoints

# Wandb 缓存 (可选)
-v /path/to/wandb:/root/.cache/wandb
```

## 12. 常见错误排查

### EGL 渲染失败

```
错误: libEGL.so.1: cannot open shared object file
解决: apt-get install -y libegl1 libegl1-mesa
```

```
错误: GLEW initialization failed
解决: apt-get install -y libglfw3
检查: MUJOCO_GL=egl PYOPENGL_PLATFORM=egl
```

### CUDA 不可用

```
torch.cuda.is_available() = False
检查: nvidia-smi 是否正常
检查: 容器是否使用 --gpus all
检查: NVIDIA Container Toolkit 是否正确安装
```

### Docker Hub 拉取失败

```
Error: context deadline exceeded (Docker Hub timeout)
解决: 
  1. 配置 Docker registry-mirrors
  2. 从私有 ACR 拉取基础镜像
  3. 使用 --build-arg BASE_IMAGE= 指定 ACR 地址
```

### pip 安装慢/失败

```
解决: 检查 ~/.pip/pip.conf 镜像配置
PyTorch: 必须使用 --index-url https://download.pytorch.org/whl/cu121
其他包: 使用 --extra-index-url https://pypi.tuna.tsinghua.edu.cn/simple/
```

### scipy 1.10.1 下载超时

```
阿里云镜像可能对此包响应慢，切换到清华镜像:
pip install scipy==1.10.1 -i https://pypi.tuna.tsinghua.edu.cn/simple/
```

## 13. 在新服务器上部署

```bash
# 1. 安装 Docker 和 NVIDIA Container Toolkit
# 2. 登录 ACR
docker login ${ACR_REGISTRY} -u ${ACR_USERNAME}

# 3. 拉取镜像
docker pull ${ACR_REGISTRY}/${ACR_NAMESPACE}/act-reference:torch2.4-cu121-mj2.3.7-v1

# 4. 测试
docker run --rm --gpus all --shm-size=8g \
  -e MUJOCO_GL=egl -e PYOPENGL_PLATFORM=egl \
  ${ACR_REGISTRY}/${ACR_NAMESPACE}/act-reference:torch2.4-cu121-mj2.3.7-v1 \
  python /opt/smoke_test.py

# 5. 进入开发环境
bash scripts/pull_and_run.sh
```

## 14. 依赖升级与 v2 Tag

```bash
# 1. 更新 requirements.lock.txt
# 2. 修改 Dockerfile 中的版本号
# 3. 构建新镜像
DOCKER_BUILDKIT=1 docker build \
  -t act-reference:torch2.4-cu121-mj2.3.7-v2 \
  docker/

# 4. 测试
docker run --rm --gpus all ... act-reference:torch2.4-cu121-mj2.3.7-v2 \
  python /opt/smoke_test.py

# 5. 推送
docker tag act-reference:torch2.4-cu121-mj2.3.7-v2 \
  ${ACR_REGISTRY}/${ACR_NAMESPACE}/act-reference:torch2.4-cu121-mj2.3.7-v2
docker push ${ACR_REGISTRY}/${ACR_NAMESPACE}/act-reference:torch2.4-cu121-mj2.3.7-v2
```

## 15. 回滚到 v1

```bash
docker pull ${ACR_REGISTRY}/${ACR_NAMESPACE}/act-reference:torch2.4-cu121-mj2.3.7-v1
docker tag ${ACR_REGISTRY}/${ACR_NAMESPACE}/act-reference:torch2.4-cu121-mj2.3.7-v1 \
  act-reference:torch2.4-cu121-mj2.3.7-v1
```

## 16. 关键文件路径汇总

| 文件                | 路径                                                   |
| ------------------- | ------------------------------------------------------ |
| Conda 环境          | /opt/miniconda3/envs/act_reference                     |
| Conda 冒烟测试      | /root/s622_embodied/scripts/smoke_test.py              |
| Conda 测试日志      | /root/s622_embodied/setup_logs/conda_smoke_test.log    |
| 系统检查日志        | /root/s622_embodied/setup_logs/system_check.log        |
| pip freeze          | /root/s622_embodied/locks/act_reference_pip_freeze.txt |
| conda env YAML      | /root/s622_embodied/locks/act_reference_conda.yml      |
| Dockerfile          | /root/s622_embodied/docker/Dockerfile                  |
| Docker requirements | /root/s622_embodied/docker/requirements.lock.txt       |
| Docker 冒烟测试     | /root/s622_embodied/docker/smoke_test.py               |
| Docker entrypoint   | /root/s622_embodied/docker/entrypoint.sh               |
| 推送脚本            | /root/s622_embodied/scripts/push_image.sh              |
| 拉取运行脚本        | /root/s622_embodied/scripts/pull_and_run.sh            |
| 部署文档            | /root/s622_embodied/README_ENVIRONMENT.md              |

## 17. Docker 网络说明

当前服务器运行在容器化环境中（overlay 文件系统），Docker Hub (`registry-1.docker.io`)
不可直接访问。已配置 Docker daemon 使用以下国内镜像加速器:

- `https://docker.m.daocloud.io`
- `https://dockerproxy.com`
- `https://registry.cn-hangzhou.aliyuncs.com`

但由于容器缺少特权模式（`--privileged`），Docker 以 vfs 存储驱动运行。

**Docker 构建建议**: 在具有完整 Docker 支持和外网访问的宿主机上进行，
或先将基础镜像同步到私有 ACR。