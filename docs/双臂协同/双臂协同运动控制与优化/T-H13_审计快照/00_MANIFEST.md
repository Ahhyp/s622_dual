# 00 MANIFEST —— T-H13 只读审计快照

生成时间：2026-10-26 23:23（本机时间，Asia/Shanghai）/ 生成者：实施方（只读整理，**不含任何新实施**）
对应代码状态：`HEAD = bb36fe2`（`master`），未提交；生产库 `strings install/lib/libdual_arm_qp_controller.so | grep -c test_fail_readstate` = 0。

本目录**只做抄录与引用**：没有修改任何源码、测试、冻结设计、证据文件或 Gate 文档；所有 `.md` 抽取均用 `sed -n` 原样截取（保留原行号信息），CSV 为逐字节复制。

## 目录内容

| 文件 | 内容 | 来源 |
|---|---|---|
| `01_冻结设计原文摘录.md` | 冻结设计 §2 相位表、§3.1–3.3 时钟与数据结构、§4.2 三态、**§5.2 判据 C1–C6 原文**、§6.1–6.2 OFF 恢复步骤 —— 逐字 | `docs/双臂协同/…/C4.3_S4_W3-R1a_T-H13_RT相位计时点设计.md`（215 行，冻结） |
| `02_源码diff.patch` | 四个源码文件的完整 `git diff`（含 `--stat`） | `git diff -- <4 files>` |
| `03_埋点与出口审计.md` | `update()` 10 个出口 × 相位标记覆盖矩阵、顺序检查、与设计 §2 边界对照、5 条审计发现（F1–F5） | 本机 `grep -n` + 人工逐行核对 |
| `04_csv/{b,b2,c,c2,c_ee}_timing.csv` | 五份原始计时 CSV（B/B2/C/C2 + E8 早退探针），13 列含表头；数据行 674/663/663/663/388 | `/tmp/th13/<label>/timing.csv` 逐字节复制 |
| `04_round_json/*.json` | 每轮的 `timing_dump` 服务**原文**、最终 `state/q_cmd/t_traj`、state 人口 | `/tmp/th13/<label>/round.json` |
| `05_scripts/*` | 统计与驱动脚本：`analyze.py`、`analyze_ee.py`、`run_round.py`、`run_rounds.sh`、`run_ee.sh`、A/B/C 参数 `params_{a,b,c}.yaml` | `/tmp/th13/` |
| `06_证据文档摘录_§5_§6_§12-§14.md` | 证据文档 §5 统计、§6 C1–C6 判定、§12 偏差、§13 强度分级、§14 复核索引 —— 逐字 | `docs/双臂协同/…/C4.3_S4_W3-R1a_T-H13_实施证据.md`（518 行） |
| `07_Gate文档_§1-§3.md` | Gate 文档全文（含 §1 变更清单、§2 矩阵、§3 回填） | `docs/双臂协同/…/C4.3_S4_CR-W3-R1A-001_Gate范围变更.md`（59 行） |

## 独立重算路径（不需要 ROS 环境）

```bash
cd <repo>/docs/双臂协同/双臂协同运动控制与优化/T-H13_审计快照
# 1) 逐样本相位和 ≤ cycle_us（C3）与 P6==0 分组（C4）：任何 awk/python 均可
python3 - <<'PY'
import csv
for lab in ["b","b2","c","c2","c_ee"]:
    rows=list(csv.DictReader(open(f"04_csv/{lab}_timing.csv")))
    bad=[r for r in rows if sum(float(r[k]) for k in
        ["p1_read_us","p2_dispatch_us","p3_safety_us","p4_task_solve_us","p5_write_us","p6_publish_us"])
        > float(r["cycle_us"])]
    p6z=[r for r in rows if float(r["p6_publish_us"])==0.0]
    print(lab, "rows",len(rows), "C3违规",len(bad), "P6==0",len(p6z),
          "其中branch",sorted({r["branch"] for r in p6z}))
PY
# 2) C2 配对：窗口 = 最长 branch==1 连续块的前 638 行，取 cycle_us 的 p99
python3 05_scripts/analyze.py     # 输出 JSON，含 analysis 的所有字段
```

## 快照内的已知限制

1. **改前/改后 P0/P7 原始数据不存在**（旧 5 列 CSV 已被本轮 ctest 覆盖为 13 列）——不伪造、不重建，登记为缺口。
2. `04_csv/*` 是**本轮实测**数据；A 轮（`timing_enable=false`）按设计不产生 CSV，无文件可抄。
3. `05_scripts/*` 内路径指向 `/tmp/th13/<label>/`（原运行目录），如需在其他目录重算请改 `BASE` 常量或复制 CSV 到对应位置。
4. 快照文件已设为只读（`chmod 444/555`），如需改动请复制到别处。

## 校验和（sha256）

```
1014af1d73b8bd8c00c1bda396175371f72acbba226a836ec73b06dbfab90c3e  ./01_冻结设计原文摘录.md
164df76247abd78716789ac074f8a5320aa11ed088319b8c04491af08d23ba7b  ./02_源码diff.patch
07e3e529cf746723fd5e595396b0d6c836b912d465e298e167c5b03132cde773  ./03_埋点与出口审计.md
102109895d02e3b5c8f1d5aaef75f0e7ead03d3740bcb23d08d3b6ff50fd1882  ./04_csv/b2_timing.csv
2ffb12b9fe01e3b8fe42a0570625af775a955ae405a09e9703f6a3d49ef73752  ./04_csv/b_timing.csv
411e816268999b5691159817dc841c55a4ab395a0fa96c909ec96fb604e5611f  ./04_csv/c2_timing.csv
fe2d7598136d1a7643e2dc201275d77015db308e2d7adeb03e1142f4e115bd22  ./04_csv/c_ee_timing.csv
895e76862aa877593dc90ad9e5df160b313cb8fcf7f2ed4c282d0eca911feb32  ./04_csv/c_timing.csv
9963e49ce953a763faf1948a619434c54a46840bda6e0ce994b56ad1f3c76b28  ./04_round_json/a_round.json
09d4e1be5408c4f3aa7436b860548b15afc7b3e4c37b2c8d14cda429796c6a1d  ./04_round_json/b2_round.json
80358c5c12fa5a5ee0903f36ac64f1e108c138766cbb066fa36281ee9d109e3e  ./04_round_json/b_round.json
030c75f2f4784644544b314f1cf1c00b858f30485c84b16d2228f98a77ac6c26  ./04_round_json/c2_round.json
e161648838b96bf417780abedf896bcaf606a518e043a2ce5476ede859203886  ./04_round_json/c_ee_round.json
534ca29ea129743b0dac1a71b323d42a0d24b88015cec163c1bfdc4bc052d2b3  ./04_round_json/c_round.json
610de8aa1c1bb13d712cf70de8f62c684f85ba0aa17da1eef95f350df9007f72  ./05_scripts/analyze.py
0cc9a5bfd6c56b09804cbdb5695b938c84dda11932bc42510bbb1d7f8973886e  ./05_scripts/analyze_ee.py
2765ce25370a9247d3a83e3ceebc172519d173406b320de4bc1a40684291bf52  ./05_scripts/params_a.yaml
74ac5d873fe3b3c0332cfb380a9b62487582f5247be9ec917251a2ea32325376  ./05_scripts/params_b.yaml
3875e922147336270ff8816bfedd43f328577e1ee708c9804cf4a6eac738e590  ./05_scripts/params_c.yaml
bc30539d54bc693f7c90bc9a14d69e1c813fb54a63115180f0a30341fc0278be  ./05_scripts/run_ee.sh
096070dff9b8aa09d752b4c5d07c2177c0a9db8f442a9e43e1238a6d90ffa12a  ./05_scripts/run_round.py
3e8bb74e84942c7b5b632f456904ea881913c3a233511855b927552d3cc889ca  ./05_scripts/run_rounds.sh
402c5a6a0d39ee473c946564b111c2847ae51d6910d8105db350653e8ca4e4bc  ./06_证据文档摘录_§5_§6_§12-§14.md
463cc7acac0b0409bf02002aa30038293384f6694fe8c18005cbfcbcf9693554  ./07_Gate文档_§1-§3.md
```
