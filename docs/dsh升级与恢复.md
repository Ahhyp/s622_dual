# DSH 升级 0.2.1 与恢复手册

> 生成时间：2026-10-17 22:32
> 环境：WSL（DESKTOP-I7Q0R33），node v24.19.0，npm 全局安装

## 一、当前状态（升级前快照）

| 项目 | 值 |
|------|-----|
| 已安装版本 | `@deepseek-ai/dsh@0.2.0-rc.2`（npm `latest` 标签） |
| 安装位置 | `/home/yep/.nvm/versions/node/v24.19.0/lib/node_modules/@deepseek-ai/dsh` |
| 可执行文件 | `/home/yep/.nvm/versions/node/v24.19.0/bin/dsh` |
| 数据目录 | `/home/yep/.dsh`（1.1G） |
| npm 可用版本 | `0.2.1-rc.2`（`next`）、`0.2.1-alpha.5`（`alpha`）；**无正式版 0.2.1** |
| 回滚锚点 | `0.2.0-rc.2`（registry 上仍然可用，可重装） |

`~/.dsh` 体积构成：`profiles` 830M（含插件 node_modules）、`sessions` 184M（5 个会话，144 个条目）、`storages` 26M、`attachments` 7.1M、`cache` 4.9M（备份时已排除，可重建）。

## 二、备份

备份位置：**D 盘** `/mnt/d/dsh-backups/`

| 文件 | 说明 |
|------|------|
| `dsh-backup-20261017-223122.tar.gz` | 460M / 38783 个条目，完整 `.dsh`（排除 `cache`） |
| `dsh-backup-20261017-223122.tar.gz.sha256` | 校验值 `150c653791ed16846087d2c3b7c7c6d8ef65bdf8921b12dc18eb98e97d20d1b8` |
| `dsh-backup-20261017-223122.tar.gz.manifest.txt` | 备份时的 `dsh --version`、`node`/`npm` 版本、npm 全局包列表、`profiles/web/package.json` |
| `dsh-backup-20261015-154511.tar.gz` | 上一次备份（401M，10-15） |

归档内路径前缀是 `.dsh/`，所以解包必须配 `-C /home/yep`。

校验归档完整性：

```bash
cd /mnt/d/dsh-backups
sha256sum -c dsh-backup-20261017-223122.tar.gz.sha256
```

> 注意：备份是在 dsh 进程运行中做的（会话是追加写）。当前活动会话的尾部可能被截断，可用 `session_repair` 修。若要字节级一致快照，先停服务再重跑一次。

## 三、升级步骤

关键点：**这个 GUI（`127.0.0.1:3080`）就是待升级进程在服务**，`npm i -g` 会重建 `node_modules`，运行中进程按需加载后续模块会崩。先停再升。

```bash
# 1. 停掉所有 dsh 实例（3080 的 web，以及 4581-4584 的四个实例）
kill 557789 359952 360210 360623 358795

# 2. 升级到 0.2.1 的最新候选版
npm i -g @deepseek-ai/dsh@0.2.1-rc.2
dsh --version          # 期望 0.2.1-rc.2

# 3. 重启
dsh web
```

第 3 步可能出现的插件兼容提示：`~/.dsh/profiles/web/package.json` 里装了 6 个树外插件（`dsh-remote`、`dshmarket`、`billion-context`、`dsh-context`、`dsh-better-sidebar`、`dsh-session-surgeon@link:/home/yep/dsh-session-surgeon`），它们声明了 DSH peer 版本范围，0.2.1-rc 可能要求显式确认或版本豁免。按提示确认，或先跑：

```bash
dsh plugin --profile web update
```

## 四、恢复（回滚）

### 4.1 完整回滚（版本 + 数据）

```bash
# 1. 停服务
pkill -f "dsh --profile web" ; pkill -f "dsh web"

# 2. 版本回滚
npm i -g @deepseek-ai/dsh@0.2.0-rc.2
dsh --version          # 期望 0.2.0-rc.2

# 3. 现场先挪走（同一文件系统内 rename，瞬间完成，别 rm）
cd /home/yep
mv .dsh ".dsh.broken-$(date +%Y%m%d-%H%M%S)"

# 4. 解包恢复（.dsh 的权限 700、.credentials.yaml 的 600 都随归档保留）
tar -xzf /mnt/d/dsh-backups/dsh-backup-20261017-223122.tar.gz -C /home/yep

# 5. 核对
ls -la /home/yep/.dsh
dsh web
```

### 4.2 只恢复数据（版本不动）

把 4.1 的第 2 步跳过即可。

### 4.3 只恢复某一部分

```bash
# 只看归档里有什么
tar -tzf /mnt/d/dsh-backups/dsh-backup-20261017-223122.tar.gz | less

# 只恢复会话
tar -xzf /mnt/d/dsh-backups/dsh-backup-20261017-223122.tar.gz -C /home/yep .dsh/sessions

# 只恢复插件组合（cordis.yml / cordis.patch.yml / package.json + 锁文件）
tar -xzf /mnt/d/dsh-backups/dsh-backup-20261017-223122.tar.gz -C /home/yep \
  .dsh/profiles/web/cordis.yml \
  .dsh/profiles/web/cordis.patch.yml \
  .dsh/profiles/web/package.json \
  .dsh/profiles/web/pnpm-lock.yaml

# 只恢复凭据
tar -xzf /mnt/d/dsh-backups/dsh-backup-20261017-223122.tar.gz -C /home/yep .dsh/.credentials.yaml
```

### 4.4 插件依赖被新版本改坏时

只恢复 `profiles/web/package.json` + `pnpm-lock.yaml` 后：

```bash
dsh plugin --profile web install
```

### 4.5 收尾

确认新版本一切正常、会话与插件都在之后，再删旧现场：

```bash
rm -rf /home/yep/.dsh.broken-*
```

## 五、以后重做一次备份

```bash
mkdir -p /mnt/d/dsh-backups
TS=$(date +%Y%m%d-%H%M%S)
OUT=/mnt/d/dsh-backups/dsh-backup-$TS.tar.gz
tar -czf "$OUT" --exclude='.dsh/cache' -C /home/yep .dsh
sha256sum "$OUT" > "$OUT.sha256"
ls -lh "$OUT"
```

恢复始终是：`tar -xzf <归档> -C /home/yep`。

## 六、插件升级记录（2026-10-18）

升级到 `@deepseek-ai/dsh@0.2.1-rc.2` 后，profile 里 6 个树外插件的对齐结果：

| 插件 | 升级前 | 升级后 | 原因 |
|------|--------|--------|------|
| `dsh-better-sidebar` | 0.26.0 | **0.28.0** | 0.26.0 依赖 `webRuntime` 服务，0.2.1-rc.2 已移除该服务 → 服务端半边永远 pending（`/sidebar/api` 不挂载）；0.28.0 改用 `connection` 服务，peer 为 `^0.2.1-rc.1` |
| `dsh-context` | 0.70.0 | **0.73.0** | 常规跟随升级，peer `dsh-session >=0.1.7-rc.2` 满足 |
| `dshmarket` | 1.66.21 | **1.66.35** | 常规跟随升级，peer 含 `^0.2.1-rc.1` |
| `dsh-remote` | 0.8.46 | 0.8.44 → **0.8.46** | 批量 `pnpm add` 时被重解析降级，已单独补回 |
| `billion-context` | 0.1.198 | 0.1.199 | 非本次改动（区间内被更新，`package.json` 已写 0.1.199） |
| `dsh-session-surgeon` | 0.2.6（link） | 0.2.6（link） | 版本不变，只改了 peer 范围，见下 |

### 6.1 session-surgeon 的兼容闸门

`/home/yep/dsh-session-surgeon/package.json` 的 peer 声明陈旧（停留在 0.1.x 时代）：

```diff
- "@deepseek-ai/dsh-tools": ">=0.1.0-rc.6 <0.2.0"
+ "@deepseek-ai/dsh-tools": ">=0.1.0-rc.6 <0.3.0"
```

依据（实测，不是猜）：`SESSION_FORMAT_VERSION` 仍是 4；`dsh-tools@0.2.1-rc.2` 仍导出 `defineTool`；用它的 CLI 对 0.2.1 产出的 44 个会话 `scan` 全 `header-ok`、`failedFrames: 0`。改完 `dsh --profile web --dump-config` 不再报 incompatible。

> 这个改动尚未提交到该仓库的 git（`git diff` 只 1 行）；`devDependencies` 里仍钉 `0.1.7-rc.2`，需要跑测试时再一并升。

### 6.2 本次备份与回滚

```bash
# 升级前存档（package.json / pnpm-lock.yaml / versions-before.txt）
ls /home/yep/dsh-backups/plugins-20261017-231704/
```

回滚插件组合：

```bash
cp /home/yep/dsh-backups/plugins-20261017-231704/{package.json,pnpm-lock.yaml} \
   /home/yep/.dsh/profiles/web/
cd /home/yep && dsh plugin --profile web install
# 若还需要回退 dsh 本体，见第四节
```

### 6.3 验证方法（不动正在服务的 3080 实例）

```bash
# 1) 组合配置无兼容告警
dsh --profile web --dump-config > /tmp/web-dump.yml   # 期望 rc=0，stderr 无 incompatible

# 2) 用一个临时端口试启动，读启动告警后让它超时退出
timeout 35 dsh web --no-open --host 127.0.0.1 --port 4599
# 期望：只打印 URL，没有 "skipping profile bundle" / "did not activate" / "pending (waiting for service: webRuntime)"

# 3) pnpm 的 "missing peer" 告警是噪音：
#    @deepseek-ai/* 由运行时的安装树提供，profile 开启 autoInstallPeers: false；
#    better-locale 只在注释里出现（可选 opt-in），不参与加载。
```

### 6.4 以后升级插件的常用命令

```bash
dsh plugin --profile web update                      # 只在 package.json 的 semver 范围内升
dsh plugin --profile web add <pkg>@<version>         # 指定版本（会同步 bundles 列表）
dsh plugin --profile web version-exemptions          # 看已授予的精确版本豁免
dsh plugin --profile web allow-version <pkg>@<ver> --dsh-version <exact> --accept-risk
```
