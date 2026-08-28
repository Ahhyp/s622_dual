// servoj_bare_test.cpp — 裸 ServoJ A/B 测试（无 ROS、无 sleep）
//
// 目的（docs/2026-08-25_ServoJ动态跟踪与同步性能测试）：
//   定位真机 ServoJ 链的 1 秒阻塞与 stack smashing 来源。
//   mode 0/1 对比已证明：1 秒阻塞来自 GetActual 与 ServoJ 的 SDK 通道竞争
//   （mode 0 无 1s 尖峰，mode 1 每 ~500 次出现 1s 阻塞）。
//   stack smashing 疑似 CloseRPC()/SDK cleanup 路径（循环完成后才崩）。
//
// mode:
//   0 = 只 ServoJ（发当前姿态，机械臂不动）
//   1 = GetActualJointPosDegree(flag=0) + ServoJ   （模拟 read()+write()）
//   2 = GetActualJointPosDegree(flag=1) + ServoJ   （flag 0/1 是否不同路径）
//   3 = ServoMoveStart + ServoJ + ServoMoveEnd     （servo 生命周期 A/B）
//   4 = Mode(0)+RobotEnable(1) 后 ServoJ           （诊断：断电重启后机械臂可能回手动模式，
//       ServoJ 报 14；切自动模式+使能后验证是否消除）
//   5 = 只 Mode(0) 后 ServoJ                       （区分是 Mode 还是 RobotEnable 的功劳）
//   6 = 只 RobotEnable(1) 后 ServoJ
//   7 = 模拟"运动后保持"：ServoJ 发 j1 偏移 2° ×100 次（运动），再发回原位 ×100 次
//       （运动结束后的保持位）——复现插件"运动后 ServoJ 14"于裸环境
// 额外: 记录 >10ms 的 (cycle, ms, rc)、GetActual 耗时、CloseRPC 前后标记。
//   no_close=1 时跳过 CloseRPC + _Exit（诊断，绕过 SDK cleanup 崩溃）
//
// 用法:
//   ros2 run fairino_hardware servoj_bare_test <ip> [n] [mode] [gap_ms] [no_close]
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <numeric>
#include <thread>
#include <vector>

#include "libfairino/include/robot.h"

struct SlowEvent
{
  int idx;        // 1-based cycle
  double ms;
  int rc;
};

static double pct(std::vector<double> v, double p)
{
  if (v.empty()) return 0.0;
  std::sort(v.begin(), v.end());
  return v[static_cast<std::size_t>(p * static_cast<double>(v.size() - 1))];
}

int main(int argc, char **argv)
{
  const char *ip = (argc > 1) ? argv[1] : "192.168.58.2";
  const int n = (argc > 2) ? std::atoi(argv[2]) : 5000;
  const int mode = (argc > 3) ? std::atoi(argv[3]) : 0;  // 0/1/2
  const int gap_ms = (argc > 4) ? std::atoi(argv[4]) : 0;
  const bool no_close = (argc > 5) && (std::atoi(argv[5]) == 1);  // 1=跳过 CloseRPC
  // 2026-08-28 诊断：cmdT 参数（默认 0.0016）。怀疑上位机检查 ServoJ 到达周期 ≤ cmdT，
  // 实际周期 ~3ms > 1.6ms → 连续 ServoJ 第二次起被拒（14）。试 0.003 匹配实际周期。
  const double servoj_cmd_t = (argc > 6) ? std::atof(argv[6]) : 0.0016;
  printf("servoj_cmd_t=%.4f\n", servoj_cmd_t);
  fflush(stdout);

  FRRobot robot;
  const int rc = robot.RPC(ip);
  printf("RPC(%s) rc=%d\n", ip, rc);
  if (rc != 0) {
    printf("RPC failed\n");
    return 1;
  }
  // 对齐 fairino_hardware_interface on_activate：RPC 后必须等待连接建立
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  JointPos q{};
  const int rcq = robot.GetActualJointPosDegree(0, &q);
  printf("GetActual rc=%d q=[%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]\n",
         rcq, q.jPos[0], q.jPos[1], q.jPos[2], q.jPos[3], q.jPos[4], q.jPos[5]);

  std::vector<double> dur(n), read_ms(n, 0.0);
  std::vector<SlowEvent> slow_events;  // ServoJ >10ms（含 rc）
  long nz_rc = 0, slow_5 = 0, slow_10 = 0, slow_100 = 0, slow_500 = 0;
  long read_nz = 0;

  if (mode == 3) {
    const int rs = robot.ServoMoveStart();
    printf("ServoMoveStart rc=%d\n", rs);
    fflush(stdout);
  }
  if (mode == 4) {
    const int re = robot.RobotEnable(1);
    const int rm = robot.Mode(0);
    printf("RobotEnable(1) rc=%d, Mode(0) rc=%d (诊断：切自动模式+使能)\n", re, rm);
    fflush(stdout);
  }
  if (mode == 5) {
    const int rm = robot.Mode(0);
    printf("Mode(0) only rc=%d (诊断：只切自动模式)\n", rm);
    fflush(stdout);
  }
  if (mode == 6) {
    const int re = robot.RobotEnable(1);
    printf("RobotEnable(1) only rc=%d (诊断：只使能)\n", re);
    fflush(stdout);
  }

  // mode 7：前半段发 j1 偏移 2°（运动），后半段发回原位（运动结束后的保持位）
  JointPos q_move = q;
  if (mode == 7) {
    q_move.jPos[0] += 2.0;
    printf("mode 7: 前 %d 次发 j1 偏移 +2°，后 %d 次发回原位（保持位）\n", n / 2, n - n / 2);
    fflush(stdout);
  }

  // ============================================================
  // mode 8：保持位扫频（2026-08-28 v2.2 实验）
  //   目的：钉死"每 ~100 次 ServoJ 卡 1s"的机制（调用计数 / 时间驱动 / 吞吐压力）
  //   条件：固定保持位（Δq=0，排除运动因素），严格 start-to-start 发送
  //   用法：servoj_bare_test <ip> <n> 8 0 <no_close> <cmdT>
  //     cmdT: 0.020→50Hz  0.010→100Hz  0.008→125Hz  0.004→250Hz
  //   记录：每次 >20ms stall 的 (cycle index, 绝对时间戳, duration, GetMotionQueueLength)
  //   判读：
  //     所有频率都是 99,199,299...       → 每 100 调用计数/队列机制
  //     cycle 不固定但间隔固定秒数        → 时间驱动 housekeeping
  //     250Hz 严重、50/100Hz 消失         → 吞吐/通信压力
  //     queue 涨到某值后 stall 再下降      → 队列假设坐实
  // ============================================================
  if (mode == 8) {
    printf("mode 8: 保持位扫频 freq=%.1f Hz (cmdT=%.4f s), n=%d, 严格 start-to-start\n",
           1.0 / servoj_cmd_t, servoj_cmd_t, n);
    fflush(stdout);
    const auto period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(servoj_cmd_t));
    auto next_tick = std::chrono::steady_clock::now();
    int stall_cnt = 0;
    std::vector<int> stall_cycles;
    for (int i = 0; i < n; ++i) {
      next_tick += period;
      const auto t0 = std::chrono::steady_clock::now();
      ExaxisPos ext{0, 0, 0, 0};
      const int r = robot.ServoJ(&q, &ext, 0, 0, servoj_cmd_t, 0, 0);
      const auto t1 = std::chrono::steady_clock::now();
      const double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
      dur[i] = ms;
      if (r != 0) nz_rc++;
      if (ms > 5.0) slow_5++;
      if (ms > 100.0) slow_100++;
      if (ms > 500.0) slow_500++;
      if (ms > 20.0) {
        int qlen = -1;
        robot.GetMotionQueueLength(&qlen); // V385 有 .so 实现；失败返回非 0，qlen 保持 -1
        const auto ts = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        printf("  [STALL] cycle=%d ts_ms=%lld dur=%.1f rc=%d queue=%d\n",
               i + 1, (long long)ts, ms, r, qlen);
        stall_cnt++;
        stall_cycles.push_back(i + 1);
        fflush(stdout);
      }
      if ((i + 1) % 500 == 0) {
        printf("  progress %d/%d (ServoJ %.3f ms)\n", i + 1, n, ms);
        fflush(stdout);
      }
      std::this_thread::sleep_until(next_tick); // 严格 start-to-start（stall 后 next_tick 落后会立即补发）
    }
    printf("=== mode 8 保持位扫频: freq=%.1f Hz n=%d ===\n", 1.0 / servoj_cmd_t, n);
    printf("stall(>20ms)=%d  slow>5ms=%ld >100ms=%ld >500ms=%ld  nonzero_rc=%ld\n",
           stall_cnt, slow_5, slow_100, slow_500, nz_rc);
    printf("stall cycles:");
    for (int c : stall_cycles) printf(" %d", c);
    printf("\n");
    fflush(stdout);
    if (no_close) {
      printf("skipping CloseRPC (--no-close diagnostic)\n");
      fflush(stdout);
      std::_Exit(0);
    }
    printf("before CloseRPC\n");
    fflush(stdout);
    const int close_rc = robot.CloseRPC();
    printf("after CloseRPC rc=%d\n", close_rc);
    fflush(stdout);
    return 0;
  }

  // 诊断环形缓冲：记录最近 20 个周期的 (idx, dur_ms, j1 指令, rc)，
  // ServoJ 返回 14 时打印——钉住"14 前发生了什么"（1s 阻塞后？正常周期突然 14？）
  struct CycleInfo { int idx; double dur; double cmd_j1; int rc; };
  std::vector<CycleInfo> ring;
  ring.reserve(20);

  for (int i = 0; i < n; ++i) {
    JointPos *target = &q;
    if (mode == 7) target = (i < n / 2) ? &q_move : &q;
    if (mode == 1 || mode == 2) {
      const auto tr0 = std::chrono::steady_clock::now();
      const int rr = robot.GetActualJointPosDegree(mode == 2 ? 1 : 0, &q);
      const auto tr1 = std::chrono::steady_clock::now();
      read_ms[i] = std::chrono::duration<double, std::milli>(tr1 - tr0).count();
      if (rr != 0) read_nz++;
    }
    ExaxisPos ext{0, 0, 0, 0};
    const auto t0 = std::chrono::steady_clock::now();
    const int r = robot.ServoJ(target, &ext, 0, 0, servoj_cmd_t, 0, 0);
    const auto t1 = std::chrono::steady_clock::now();

    dur[i] = std::chrono::duration<double, std::milli>(t1 - t0).count();
    if (r != 0) nz_rc++;
    if (dur[i] > 5.0) slow_5++;
    if (dur[i] > 10.0) { slow_10++; slow_events.push_back({i + 1, dur[i], r}); }
    if (dur[i] > 100.0) slow_100++;
    if (dur[i] > 500.0) slow_500++;

    // 更新环形缓冲
    ring.push_back({i + 1, dur[i], target->jPos[0], r});
    if (ring.size() > 20) ring.erase(ring.begin());
    if (r != 0) {
      printf("  [DIAG] rc=%d at cycle=%d, 前 %zu 个周期:\n", r, i + 1, ring.size());
      for (const auto &c : ring) {
        printf("    cycle=%d dur=%.1f ms cmd_j1=%.3f rc=%d\n", c.idx, c.dur, c.cmd_j1, c.rc);
      }
      fflush(stdout);
    }

    if (mode == 7 && i == n / 2 - 1) {
      printf("  [mode7] 运动段结束 rc_last=%d（后半段开始发保持位）\n", r);
      fflush(stdout);
    }
    if (gap_ms > 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(gap_ms));
    }
    if ((i + 1) % 500 == 0) {
      printf("  progress %d/%d (ServoJ %.3f ms, read %.3f ms)\n",
             i + 1, n, dur[i], read_ms[i]);
      fflush(stdout);
    }
  }

  if (mode == 3) {
    printf("before ServoMoveEnd\n");
    fflush(stdout);
    const int re = robot.ServoMoveEnd();
    printf("ServoMoveEnd rc=%d\n", re);
    fflush(stdout);
  }

  printf("loop finished\n");
  fflush(stdout);

  // ---- 统计在 CloseRPC 之前打印（CloseRPC 已知 stack smashing，不影响取数）----
  const double mean = std::accumulate(dur.begin(), dur.end(), 0.0) / static_cast<double>(n);
  const double mx = *std::max_element(dur.begin(), dur.end());
  printf("=== mode=%d n=%d gap=%d ===\n", mode, n, gap_ms);
  printf("ServoJ call: mean=%.3f P50=%.3f P95=%.3f P99=%.3f max=%.3f ms\n",
         mean, pct(dur, 0.5), pct(dur, 0.95), pct(dur, 0.99), mx);
  printf("slow: >5ms=%ld >10ms=%ld >100ms=%ld >500ms=%ld  nonzero_rc=%ld\n",
         slow_5, slow_10, slow_100, slow_500, nz_rc);
  if (mode != 0) {
    std::vector<double> rd;
    for (double x : read_ms) if (x > 0.0) rd.push_back(x);
    if (!rd.empty()) {
      printf("GetActual: mean=%.3f P50=%.3f P95=%.3f P99=%.3f max=%.3f ms  nonzero_rc=%ld\n",
             std::accumulate(rd.begin(), rd.end(), 0.0) / rd.size(),
             pct(rd, 0.5), pct(rd, 0.95), pct(rd, 0.99),
             *std::max_element(rd.begin(), rd.end()), read_nz);
    }
  }
  if (!slow_events.empty()) {
    printf("slow events >10ms:\n");
    for (const auto &e : slow_events) {
      printf("  cycle=%d ServoJ=%.3f ms rc=%d\n", e.idx, e.ms, e.rc);
    }
  }
  fflush(stdout);

  // ---- CloseRPC（已知 stack smashing）----
  if (no_close) {
    printf("skipping CloseRPC (--no-close diagnostic)\n");
    fflush(stdout);
    std::_Exit(0);  // 绕过 FRRobot 析构/SDK cleanup
  }
  printf("before CloseRPC\n");
  fflush(stdout);
  const int close_rc = robot.CloseRPC();
  printf("after CloseRPC rc=%d\n", close_rc);
  fflush(stdout);
  return 0;
}
