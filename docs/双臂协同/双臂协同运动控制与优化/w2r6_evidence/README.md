# W2-R6 分配归属证据归档

本目录是 C2 收口的**可复算**证据集。所有数字都能用 `reconcile.py` 从这些聚合表重算并校验。

## 文件

| 文件 | 来源 | 内容 |
|---|---|---|
| `gtest_l1_typed.txt` | gtest L1 负载（638 次 `solve()`）在类型化探针下的账本 | 入口 / 分类×入口 / 归属 / 全栈符号 |
| `driver_prod_static.txt` | `l1_driver 638 … static`（生产库，固定目标） | 同上 + 阶段表 |
| `driver_ctrl_moving.txt` | `l1_driver_ctrl 638 … moving`（未打补丁对照，运动目标） | 同上 |
| `driver_bprime_moving.txt` | `l1_driver_bprime 638 … moving`（B′ 原型） | 同上 |
| `countonly_static.txt` / `countonly_moving.txt` | 仅计数探针（无栈、无 `dladdr`） | 独立交叉核验 |

`#raw` 逐条记录段已剥离以控制仓库体积（每文件约 2.7 万–3.3 万行）。

## 复算

```
cd docs/双臂协同/双臂协同运动控制与优化/w2r6_evidence
python3 reconcile.py     # 退出码 0 = 全部恒等式成立
```

## 重生成命令（逐条原始记录）

```
# 类型化账本（生产库 / 对照 / B′）
LD_PRELOAD=/tmp/w2r5/liballoctyped.so MW_LEDGER=<out.txt> \
LD_LIBRARY_PATH=install/lib ./l1_driver 638 x.txt moving dq.csv

# 仅计数探针
MW_COUNT=<out.txt> LD_PRELOAD=/tmp/w2r5/libcountonly.so \
LD_LIBRARY_PATH=install/lib ./l1_driver 638 x.txt moving dq.csv
```

## 已知限制

1. `gtest_l1_typed.txt` 由**未输出阶段表**的探针版本产生 ⇒ 该文件无 `# by phase` 段，阶段恒等式不适用（`reconcile.py` 已显式区分）。
2. 只覆盖**堆**分配；栈对象不在统计内。
3. 生产构建下 `operator new` 与底层 `malloc` 会重复计入各自入口，故"入口"维度不可跨实现直接相比，**归属维度**才是可比口径。
