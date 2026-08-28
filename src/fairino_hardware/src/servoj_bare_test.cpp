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
//   3 = ServoMoveStart + ServoJ + ServoMoveEnd     （servo 生命周期 A/B：验证
//       ServoMoveEnd 是否是 CloseRPC stack smashing 的修复）
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

  for (int i = 0; i < n; ++i) {
    if (mode == 1 || mode == 2) {
      const auto tr0 = std::chrono::steady_clock::now();
      const int rr = robot.GetActualJointPosDegree(mode == 2 ? 1 : 0, &q);
      const auto tr1 = std::chrono::steady_clock::now();
      read_ms[i] = std::chrono::duration<double, std::milli>(tr1 - tr0).count();
      if (rr != 0) read_nz++;
    }
    ExaxisPos ext{0, 0, 0, 0};
    const auto t0 = std::chrono::steady_clock::now();
    const int r = robot.ServoJ(&q, &ext, 0, 0, 0.0016, 0, 0);
    const auto t1 = std::chrono::steady_clock::now();

    dur[i] = std::chrono::duration<double, std::milli>(t1 - t0).count();
    if (r != 0) nz_rc++;
    if (dur[i] > 5.0) slow_5++;
    if (dur[i] > 10.0) { slow_10++; slow_events.push_back({i + 1, dur[i], r}); }
    if (dur[i] > 100.0) slow_100++;
    if (dur[i] > 500.0) slow_500++;

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
