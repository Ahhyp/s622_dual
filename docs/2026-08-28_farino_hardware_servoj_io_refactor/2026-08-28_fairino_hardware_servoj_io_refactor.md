# 2026-08-28 fairino_hardware ServoJ I/O 分层重构设计文档（v2）

> 状态：**设计冻结，Phase 0 已取消（2026-08-28 决定），直接进入 Phase 1 实施**
> 关联：docs/2026-08-25_ServoJ动态跟踪与同步性能测试/、真机 ServoJ 14 排查（8/28）
> 参照：robotarm 最新源码（`/home/yep/fairino_robotarm`，jasonlee0617，2026-08-28 main 分支）

---

## 1. 背景与已测事实

### 1.1 问题链

8/28 真机排查确认的现象链：

```
1s 阻塞（每 ~100 次 ServoJ 一次 ~1030ms，rc=0）
   → 运动时阻塞后第一条指令 Δq = JTC 推进 1s 的位移
   → 上位机按 cmdT 计算等效速度 v_cmd = Δq/cmdT
   → 超过轴限速 → 上位机报"轴3 关节空间内指令速度超限"
   → 报警触发后 ServoJ 全部返回 ERR_EXECUTION_FAILED(14)（报警开关行为）
   → 清报警后恢复（Mode(0) rc=0、保持位 100 次无 14）
```

### 1.2 已测事实（实验确认，非推测）

| # | 事实 | 来源 |
|---|---|---|
| F1 | 裸 ServoJ P50 ≈ 2.6ms，P95 ≈ 3.3ms | 8/28 裸测 |
| F2 | 存在约 1.0~1.04s 的同步阻塞，阻塞期间 ServoJ rc=0 | 8/28 裸测长跑（断电重启后恒定） |
| F3 | 保持位 + 阻塞 → rc=0（不超限，因 Δq≈0）；运动 + 阻塞 → rc=14（超限） | 8/28 裸测 mode 0/7 |
| F4 | 超限报警触发后，所有写入（Mode/ServoJ）返回 14，清报警后恢复 | 8/28 裸测 + 用户确认 |
| F5 | 运动级 ServoMoveStart（mode 3：Start→ServoJ×3000→End）后 **stall 仍存在**（>500ms 8/3000 次，max≈1034ms），但发生率降低 | 8/28 裸测 mode 3 |
| F6 | cmdT 0.0026/0.003（500Hz 同步）无效；cmdT 0.004 + 实际 2ms 周期被拒（14） | 8/28 / 8/27 记录 |
| F7 | CloseRPC 触发 stack smashing（裸测），正式 shutdown stack trace 指向 RobotStateRoutineThread robot.cpp:135 | 8/28 裸测 no_close 对比 |
| F8 | 本机 V385 SDK 头文件有 `GetMotionQueueLength()`，**无** `servoJCmdNum`/`lastServoTarget`/`MotionQueueClear()` | 本次核实 robot.h |
| F9 | robotarm 最新版（GitHub main）已实现分层：write() 只更新缓存 + 独立 io_loop 线程（周期=cmdT）+ cmdT 默认 0.008 + 会话级 ServoMoveStart/End | 本次拉取源码核实 |

### 1.3 关键外部情报

- **FAIRINO frcobot_ros2 Issue #32**（open issue，用户实测）：`on_activate()` 不先 `ServoMoveStart()` 就 `write()` 调 ServoJ，真机出现严重 blocking；补上后恢复。→ **注意：是用户报告、open issue，非厂商确认的通用根因**。
- FAIRINO 官方文档对 cmdT 表述不一致：SDK 注释建议 0.001~0.0016s；官方示例用 0.008s；支持页面称命令间隔约 1~16ms。

---

## 2. 当前架构（改造前）

```
JTC / controller_manager (update_rate=500)
   ↓ write()  ← 同步调 ServoJ
ServoJ(..., cmdT=0.0016)   ← SDK 同步阻塞调用，1s stall 直接卡死主循环
   ↓
FAIRINO 控制器
read() ← GetActualJointPosDegree（同步，RT 循环内）
```

**问题**：
1. SDK 同步调用（含 1s stall）直接阻塞 ros2_control 主循环（read → JTC → write 全锁死）
2. 无 stall 检测 / 无速度保护：stall 后追 latest_command → 轴3 超限 → 14
3. 无会话级 ServoMoveStart（Issue #32 的 blocking 诱因之一）
4. rc≠0 只打日志，继续下发 → 14,14,14… 刷屏

---

## 3. 目标架构

```
JTC / controller_manager 500Hz
   ↓ write()  只锁内拷贝 _jnt_position_command → _latest_command（不调 SDK）
   ↓
独立 io_loop 线程（start-to-start 周期 = cmdT）
   ├─ 取 _latest_command（锁内）
   ├─ command-step velocity guard：per-joint |Δq|/cmdT ≤ 0.8·v_limit[i]
   │     └─ 任一超限 或 上次 stall → 本次候选 = 上次成功位置（不发最新）
   ├─ send-interval health check：dt_send = now - last_send_ns
   │     ├─ ~cmdT → 正常
   │     ├─ 明显漂移 → warning
   │     └─ > stall 阈值 → stream_broken → fault
   ├─ ServoJ(candidate, cmdT)  ← 计时
   │     ├─ rc≠0 → fault
   │     └─ duration > stall_fault_ms → stream_broken → fault
   ├─ 成功 → last_sent = candidate
   ├─ 读反馈 GetActual → _latest_state（stale > 100ms → fault）
   └─ sleep_until(next_tick)
fault 后：read()/write() 返回 ERROR → JTC 停止 → 保持/安全停止 → 读 actual → 须重新规划
手爪 DO：独立低频路径（状态变化才调用，单独计时，不进 ServoJ 关键循环）
```

**设计原则**：

1. **SDK 阻塞不传播到 ros2_control 主循环**——所有 FRRobot 调用集中在 io_loop 单线程
2. **stall 后可检测、可阻止**——watchdog 无法打断同步 ServoJ，但能检测已发生的 stall 并阻止下一条危险指令（不追赶 latest_command）
3. **速度保护按关节**——v_cmd = |Δq|/cmdT 与每轴安全限速（0.8×限速）比较，重点保护 J3（实测报警轴）
4. **fault 后不自动恢复追赶**——必须重新规划
5. **ServoMoveStart 必须加，但不依赖它根治 stall**——F5 已证明它只降低发生率

---

## 4. 线程 / 锁模型

```
┌─────────────────────────────┐
│ ros2_control RT 主线程       │  read()/write()：锁内拷贝 6 double，微秒级
│  (controller_manager 500Hz) │  fault 后返回 ERROR
└──────────────┬──────────────┘
               │ _io_mutex
┌──────────────▼──────────────┐
│ io_loop 线程（1 个）          │  唯一 FRRobot 运动类调用者：
│  周期 = cmdT (sleep_until)   │  ServoJ / GetActual / StopMotion / ServoMoveEnd
└─────────────────────────────┘
┌─────────────────────────────┐
│ gripper worker（低频，可选）   │  SetDO 仅状态变化时调用，单独计时
└─────────────────────────────┘
```

- 共享变量：`_latest_command`（write→io_loop）、`_latest_state`（io_loop→read）、`_last_sent`（io_loop 私有 + fault 时 read）、`_faulted/_io_running/_shutdown_requested`（atomic）
- 锁：单一 `_io_mutex` 保护 `_latest_command/_latest_state/_finger_position_command`；io_loop 内部自己的状态（`_last_sent` 等）不进锁
- **SDK 并发规则**：FRRobot 对象只允许 io_loop 线程访问；read/write/latch_fault 不得直接调 SDK（latch_fault 只设原子标志）

---

## 5. 生命周期（on_activate / on_deactivate）

### on_activate

```
RPC 连接（现有）
→ 读初始 actual 位置（失败则 ERROR）
→ 同步 command / _latest_command / _last_sent / _latest_state = actual
→ finger 初始 SetDO（已知状态）
→ ServoMoveStart()（会话级；失败 → CloseRPC + ERROR，不允许 active）
→ 初始化 atomics / 计数器
→ 启动 io_loop 线程
→ 日志：ServoJ I/O started: cmdT=...ms
```

### on_deactivate（清理顺序修正版）

```
on_deactivate:
   _shutdown_requested = true   （不直接碰 _io_running）
   → io_loop 在下个周期看到请求：
        io_loop: StopMotion() → ServoMoveEnd() → 退出循环
   → on_deactivate: join(io_loop)     ← 线程彻底退出后
   → CloseRPC()（KNOWN SDK ISSUE，见 §11）
   → release
```

要点：运行期所有 FRRobot 运动类调用集中在 io_loop；CloseRPC 保证发生在 I/O 线程彻底退出之后。

---

## 6. fault state machine

```
状态: NORMAL → FAULTED（latch，不自动恢复）

触发条件（任一）:
  1. ServoJ rc != 0（第一次即 fault，不再持续下发）
  2. ServoJ 调用耗时 > servo_stall_fault_ms（默认 20ms）→ stream_broken
  3. send-interval dt_send > stall 阈值 → stream_broken
  4. feedback stale > feedback_stale_fault_ms（默认 100ms）

latch_fault(reason):
  只做: _faulted=true（atomic）、记录 _fault_reason、_io_running=false
  （不调 SDK；StopMotion 由 io_loop 自己执行后退出）

FAULTED 后:
  read()/write() 返回 ERROR → ros2_control 层 JTC 停止
  io_loop 退出前执行 StopMotion()（保持/安全停止）
  恢复路径: 仅限外部重新激活（重新 on_activate 读 actual 再规划），不自动追赶
```

---

## 7. stall recovery policy（A+B：防跳变 + fault/abort）

```
ServoJ 返回，发现 duration > 20ms（或 dt_send 异常）:
  ① 绝对不发送 latest_command（防轴超限）      ← A 的防跳变
  ② latch fault + abort 当前 trajectory        ← B 的 fault
  ③ 保持/安全停止（io_loop 内 StopMotion）
  ④ 读 actual state 重新同步
  ⑤ 必须重新规划后才能重新开始（不自动恢复追赶）
```

**为什么不选纯 B（持续发 last_sent 等 JTC）**：JTC 不等人，持续 hold 期间 latest_command 与真实位置差距越来越大，最终恢复追赶必然大跳变。最安全 = 停止当前轨迹、重新规划。

---

## 8. 参数表（全部 hardware param 可配）

| 参数 | Phase 1 默认 | Phase 2 | 说明 |
|---|---|---|---|
| `servoj_cmd_t` | **0.008**（125Hz） | 0.004（250Hz） | 发送周期/声明周期，start-to-start |
| `servo_v_limit_0..5` | 真机限速 × 0.8 | 同左 | per-joint 等效速度上限（J3 重点保护） |
| `servo_stall_warn_ms` | 10 | 同左 | 慢调用警告 |
| `servo_stall_fault_ms` | 20 | 同左 | stall → stream_broken → fault |
| `feedback_stale_fault_ms` | 100 | 同左 | 反馈过期 → fault |
| update_rate（real_controllers.yaml） | **500** | 500 | 与 ServoJ 频率解耦 |

**速度判据（关键，修正版）**：

- **是否允许发送的唯一安全判据**：`v_cmd,i = |q_candidate,i - q_last_sent,i| / cmdT ≤ v_safe,i`（v_safe = 0.8 × v_limit）
- **实际发送间隔 dt_send 独立作为 stream-health 指标**（不做超速判据）：
  - ~cmdT → 正常
  - 明显漂移 → warning
  - > stall 阈值 → stream_broken / fault
- 注意：stall 1s 后 v_actual=Δq/1s 反而很小"看似安全"，**不能**用作超速判据——下一条 ServoJ 上位机仍按 cmdT 理解位置跳变。

---

## 9. Phase 计划

### Phase 0：裸测队列观测 —— **已取消（2026-08-28 用户决定）**

满队列假设不做验证。保留的核实结论（2026-08-28）：

- `GetMotionQueueLength(int *len)` 在 V385 SDK **有真实实现**（.so 导出符号 `_ZN7FRRobot20GetMotionQueueLengthEPi`，非空壳），真机调用行为未实测
- `servoJCmdNum` / `lastServoTarget` / `MotionQueueClear()` V385 无（需升级 SDK）
- 不升级 SDK（避免实验变量改变；SDK 与固件版本配套发布）
- 若日后想验证：在 bare test 中 cycle 95~105、195~205 及每次 >100ms stall 前后记录 queue 长度；**先观察，不自动 Clear**

### Phase 1：稳定性验证（cmdT=0.008 / 125Hz）

- 实现 §3 架构 + §5 生命周期 + §6/§7 fault/stall 策略
- update_rate 恢复 500
- 验收：§10 核心安全 + 基本功能

### Phase 2：性能（cmdT=0.004 / 250Hz）

- 稳定后切换，对比伺服跟踪与 P50/P95/P99/max

---

## 10. 验收指标

### 核心安全验收线（一票否决）

1. 任何 ServoJ stall 不得演变成关节速度超限（无"轴3 超限"报警）
2. stall 后不得追赶 latest_command（无大跳变指令）
3. ServoJ rc≠0 后不得继续持续下发（第一次即 fault，read/write 返回 ERROR）

### 基本功能验收线

4. 正常状态下 20mm demo 可**连续重复完成**
5. controller_manager 不因 ServoJ SDK 调用阻塞（write 周期稳定）
6. JTC/MoveIt 能正确收到硬件 fault（不是"上层成功、底层一直 14"）
7. 125Hz 稳定后再验证 250Hz

### 性能观测指标（记录，不设硬门槛）

- ServoJ 调用 P50/P95/P99/max
- stall 次数 / 频率
- send-interval 分布
- 每轴 max Δq
- 每轴 max 等效 command velocity

---

## 11. KNOWN SDK ISSUES（挂账，不阻塞本次）

| 问题 | 证据 | 处理 |
|---|---|---|
| `CloseRPC()` / `RobotStateRoutineThread` stack corruption | 裸测 stack smashing；shutdown stack trace → robot.cpp:135 | 单独跟踪：SDK/固件版本匹配、升级 SDK、厂商确认；本次只保证 CloseRPC 在 io_loop 彻底退出后调用 |
| Issue #32（无 ServoMoveStart → write 阻塞） | open issue 用户报告 | 本次采用会话级 ServoMoveStart 规避；不视为已确认根因 |

---

## 12. 已测事实 vs 假设（重要，回看时勿混淆）

### 已测事实（实验确认）

- bare ServoJ P50 ≈ 2.6ms、P95 ≈ 3.3ms（F1）
- 存在 ~1.0~1.04s 同步阻塞，阻塞中 rc=0，断电重启后恒定（F2）
- 保持位+阻塞 rc=0；运动+阻塞 rc=14（F3）
- 超限报警 → 全 14 → 清报警恢复（F4）
- 会话级 ServoMoveStart 后 stall 仍存在（8/3000 次 >500ms，max 1034ms）（F5）
- cmdT 0.0026/0.003 无效；0.004+2ms 周期被拒（F6）
- CloseRPC stack smashing（F7）
- V385 SDK 有 GetMotionQueueLength，无 servoJCmdNum/lastServoTarget/MotionQueueClear（F8）
- robotarm 最新版已分层（write 分离 + io_loop + cmdT 0.008 + 会话级 Start/End）（F9）

### 尚未证实的假设

- 1s stall 是否由 motion queue 导致（**已决定不验证**，Phase 0 取消；GetMotionQueueLength 有 .so 实现但真机行为未实测）
- 1s stall 是否直接导致轴3 超限（已有强相关性，因果未隔离验证）
- 降到 125/250Hz 是否能消除/降低 stall
- ServoMoveStart 在正式 hardware 路径中是否显著降低 stall 率
- 会话级 ServoMoveStart 在我们 58.2/58.3 固件上是否有副作用

---

## 13. 回滚

- 设计文档：本文件（docs/2026-08-28_fairino_hardware_servoj_io_refactor.md）
- 代码回滚点：`git checkout d978482 -- src/fairino_hardware`（8/28 排查收尾提交，回滚+cmdT 参数化版）
- 配置回滚：`real_controllers.yaml` update_rate 恢复 500（Phase 1 起）

---

## 14. 文档版本

| 版本 | 日期 | 说明 |
|---|---|---|
| v1 | 2026-08-28 | DeepSeek 初版方案（分层 + ServoMoveStart 会话级 + watchdog + last_sent） |
| v2 | 2026-08-28 | 用户评审修订：v_actual 改为 send-interval 健康指标；deactivate 清理顺序修正；V385 条件启用队列 API；验收拆分核心安全/基本功能/性能；"已测事实 vs 假设"章节 |
| v2.1 | 2026-08-28 | 用户决定取消 Phase 0（满队列假设不验证）；文档移入 docs/2026-08-28_farino_hardware_servoj_io_refactor/；核实 GetMotionQueueLength 有 .so 实现 |
