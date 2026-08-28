#pragma once

// servo_stats.hpp — ServoJ 调用耗时 + 控制周期统计（2026-08-28）
//
// 用途（docs/2026-08-25_ServoJ动态跟踪与同步性能测试 第 19/20 节）：
//   真机控制链每周期（update_rate=500Hz → 2ms）调一次 ServoJ 下发位置指令。
//   统计 ServoJ 单次调用阻塞耗时（SDK call duration）与控制周期抖动
//   （相邻 write() 起点间隔），回答"真机控制链到底能多稳"。
//
// 设计（GPT review 2026-08-28 修订）：
//   - 采样窗口 = 报告间隔（maybe_report 触发后 clear），无 ring-buffer，
//     避免 capacity 与 miss 统计窗口不一致
//   - mean 是真实均值（非 P50）；补样本标准差 σ
//   - deadline 用三档 overrun（>2.0 / >2.5 / >3.0 ms）而非单一阈值
//   - 纯 C++（不依赖 rclcpp），数据由调用方打印；正式采数时日志应移出
//     write() 实时路径（打印本身会制造周期尖峰）

#include <chrono>
#include <cstdint>
#include <vector>

namespace fairino_hardware
{

class ServoStats
{
public:
  struct Report
  {
    // ServoJ SDK call duration
    std::int64_t n_calls = 0;
    double call_mean_ms = 0.0;
    double call_p50_ms = 0.0;
    double call_p95_ms = 0.0;
    double call_p99_ms = 0.0;
    double call_max_ms = 0.0;
    // write cycle period
    std::int64_t n_periods = 0;
    double period_mean_ms = 0.0;
    double period_stddev_ms = 0.0;
    double period_p50_ms = 0.0;
    double period_p95_ms = 0.0;
    double period_p99_ms = 0.0;
    double period_max_ms = 0.0;
    double over_2ms_pct = 0.0;    // 周期 > 2.0 ms 占比（500Hz 名义周期）
    double over_2p5ms_pct = 0.0;  // 周期 > 2.5 ms 占比
    double over_3ms_pct = 0.0;    // 周期 > 3.0 ms 占比（严重超期）
  };

  ServoStats() = default;

  /// write() 开头调用：记录周期起点，累计周期样本（全模式统一计周期）
  void begin_cycle();

  /// ServoJ 返回后调用：记录单次调用阻塞耗时（毫秒）
  void record_call(double call_ms);

  /// 每 interval 次 begin_cycle() 调用返回 true，并填窗口统计报告（随后清空窗口）
  bool maybe_report(std::int64_t interval, Report &out);

private:
  static double mean(const std::vector<double> &v);
  static double stddev(const std::vector<double> &v, double m);
  static double percentile(std::vector<double> v, double p);

  std::vector<double> calls_;    // ServoJ 耗时样本（ms）
  std::vector<double> periods_;  // 周期样本（ms）
  std::chrono::steady_clock::time_point last_;
  bool have_last_ = false;
  std::int64_t count_ = 0;  // begin_cycle 累计次数
};

}  // namespace fairino_hardware
