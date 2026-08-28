# 执行记录：真机 ServoJ 链路排查（2026-08-28）

> 关联计划：`docs/2026-08-25_ServoJ动态跟踪与同步性能测试/README.md`
> 工具：`servoj_bare_test`（裸 SDK A/B 测试，无 ROS/sleep）+ `servo_stats`（插件内统计）
> 机械臂：192.168.58.3（保持当前姿态，零运动，安全）

---

## 1. 背景

真机 ros2_control 统计（`servo_stats` 插件内）发现：ServoJ P50≈2.6ms、周期性 ~1030ms 阻塞、
500Hz 周期 100% 超限。用裸 SDK 程序（无 ROS/MoveIt）做 A/B 定位来源。

## 2. A/B 测试设计（servoj_bare_test）

| mode | 内容 |
|------|------|
| 0 | 只循环 ServoJ（发当前姿态） |
| 1 | GetActualJointPosDegree(flag=0) + ServoJ（模拟 read+write） |
| 2 | GetActualJointPosDegree(flag=1) + ServoJ |
| 3 | ServoMoveStart + ServoJ + ServoMoveEnd |

记录：ServoJ/GetActual 各自耗时、>10ms 的 (cycle, ms, rc)、CloseRPC 前后标记、
`no_close=1` 跳过 CloseRPC（诊断）。

## 3. 结论（已确认）

### 3.1 ServoJ 基础耗时 ~2.6-3.4ms（P50≈2.6ms）
- 稳定在 2.4~3.4ms，GetActual 存在与否无影响
- **500Hz（2ms 周期）物理上不可达**：单次同步 ServoJ 就超过周期预算
- 实际可稳定运行约 300Hz（3.3ms）

### 3.2 每 100 次 ServoJ 一次 ~1030ms 阻塞（rc=0，最终成功）
- **规律极强**：mode 0 中 cycle=99, 199, 299, ..., 4999 全部 ~1010-1036ms
- **与 GetActual 无关**（mode 0 纯 ServoJ 也有）；GetActual 存在时次数反而少（27 vs 50）
- 阻塞时 ServoJ **返回 rc=0**（上位机最终处理，但延迟 1 秒）——不是 error/timeout
- 判定：**SDK/上位机内部周期同步机制**（每 N 次 ServoJ 后强制同步/缓存刷新），
  与 ROS/ros2_control 无关（裸程序复现）

### 3.3 ServoMoveStart 显著减少阻塞（重要修复线索）
| 运行 | >100ms 次数 |
|---|---|
| mode 0（无 Start，5000 次） | 50（每 100 次，规律） |
| mode 1（GetActual，5000 次） | 27（前 800 次每 100，后分散） |
| **mode 3（ServoMoveStart，3000 次）** | **8**（应 ~30，减少 ~75%） |

→ **ServoMoveStart 启动伺服会话后，SDK 不再每 100 次强制同步**。
待验证：插件 `on_activate` 加 `ServoMoveStart()`（**只加 Start，不加 Mode/RobotEnable**
——后者有"启动即切自动模式"副作用）后，真机阻塞是否消失。

### 3.4 GetActual 自身 ~0.002ms（可忽略，非阻塞源）
- P50=0.002ms、P95=0.003ms、max=0.144ms——read() 开销极小

### 3.5 CloseRPC() 触发 stack smashing（SDK cleanup bug，不影响运行）
- 所有模式：`loop finished → before CloseRPC → stack smashing`（统计先打印，数据无损）
- `no_close`（跳过 CloseRPC + `_Exit`）**完整退出不崩** → 崩溃点确认在 CloseRPC/SDK cleanup
- ServoMoveEnd 不能修复（mode 3 仍崩）
- 与之前记录的"关闭时 stack smashing（SDK RobotStateRoutineThread）"一致
- 影响：仅退出时崩溃，运行期无影响；正式插件不可用 `_Exit` 绕过，接受为已知 SDK bug

## 4. 工程影响

1. **500Hz controller_manager + 每周期同步 ServoJ 的架构不成立**：ServoJ 2.6ms 基础 +
   每 ~260ms 卡 1 秒。需降 update_rate（≤300Hz）或改用异步下发
2. **机械臂周期性停顿**：每 100 次 ServoJ（~260ms）机械臂停 1 秒——之前 20mm 微动
   成功是未撞上阻塞或阻塞后最终到位
3. **ServoMoveStart 是主要修复候选**：待真机验证（见 3.3）
4. **CloseRPC 崩溃**：记录为已知 SDK bug，不影响运行

## 5. 文件

- `src/fairino_hardware/src/servoj_bare_test.cpp`：裸 A/B 测试（ros2 run fairino_hardware servoj_bare_test <ip> [n] [mode] [gap_ms] [no_close]）
- `src/fairino_hardware/{include/}servo_stats.{hpp,cpp}`：插件内 ServoJ/周期统计（每 500 周期打印）
- `src/fairino_hardware/src/fairino_hardware_interface.cpp`：write() 加统计（第一轮保留日志，正式采数移出）

## 6. 待办

- [ ] 插件 `on_activate` 加 `ServoMoveStart()`（只加 Start）→ 真机验证阻塞消失
- [ ] 大样本 mode 3（n=5000）确认阻塞率
- [ ] update_rate 降到 ≤300Hz 的可行性测试（或确认 500Hz + ServoMoveStart 是否可行）
- [ ] CloseRPC 崩溃：确认是否可接受（运行无影响）或研究绕过

---

## 7. 补充：ServoMoveStart 插件实测（2026-08-28 下午）——无效且有害，已回退

**尝试**：基于裸测 mode 3 减阻塞 ~75%，在 `on_activate` 启用 `ServoMoveStart()`（仅 Start）。

**实测结果（真机 20mm 微动）**：
1. `ServoMoveStart success`（调用成功）
2. **1s 阻塞不消失**：`>3.0ms=20-21%`（与之前 16-32% 相当）、`max≈1034ms` 照旧
3. **运动结束后 ServoJ 连续报 14**：`Goal reached, success!` 后 write() 继续发 ServoJ（保持位）
   → 上位机认为伺服会话状态变化 → `ServoJ指令下发错误,错误码:14` 连续刷

**结论**：
- 裸测 mode 3 的减阻塞**仅在"机械臂不动"（ServoJ 发当前姿态）场景成立**，不可迁移到真实运动
- ServoMoveStart 在插件环境（真实运动）**无效且有害**（引入运动后 14）——robotarm 注释掉它是正确的
- **1s 阻塞 = SDK/上位机固有**（每 100 次 ServoJ 一次 ~1030ms），暂无法软件绕过；
  实际影响评估：20mm 微动成功（短动作未撞上或撞上后完成），真机 BT 全流程验证时观察

**处置**：已回退（恢复裸 ServoJ，diff 仅注释），重新编译。
