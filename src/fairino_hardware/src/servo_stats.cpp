// servo_stats.cpp — ServoStats 实现
#include "fairino_hardware/servo_stats.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>

namespace fairino_hardware
{

void ServoStats::begin_cycle()
{
  const auto now = std::chrono::steady_clock::now();
  if (have_last_) {
    periods_.push_back(
      std::chrono::duration<double, std::milli>(now - last_).count());
  }
  last_ = now;
  have_last_ = true;
  count_++;
}

void ServoStats::record_call(double call_ms)
{
  calls_.push_back(call_ms);
}

bool ServoStats::maybe_report(std::int64_t interval, Report &out)
{
  if (interval <= 0 || count_ % interval != 0) {
    return false;
  }
  out.n_calls = static_cast<std::int64_t>(calls_.size());
  out.n_periods = static_cast<std::int64_t>(periods_.size());

  if (!calls_.empty()) {
    const auto v = std::vector<double>(calls_.begin(), calls_.end());
    out.call_mean_ms = mean(v);
    out.call_p50_ms = percentile(v, 0.50);
    out.call_p95_ms = percentile(v, 0.95);
    out.call_p99_ms = percentile(v, 0.99);
    out.call_max_ms = *std::max_element(v.begin(), v.end());
  }
  if (!periods_.empty()) {
    const auto v = std::vector<double>(periods_.begin(), periods_.end());
    out.period_mean_ms = mean(v);
    out.period_stddev_ms = stddev(v, out.period_mean_ms);
    out.period_p50_ms = percentile(v, 0.50);
    out.period_p95_ms = percentile(v, 0.95);
    out.period_p99_ms = percentile(v, 0.99);
    out.period_max_ms = *std::max_element(v.begin(), v.end());
    const auto n = static_cast<double>(v.size());
    out.over_2ms_pct = 100.0 * std::count_if(v.begin(), v.end(),
        [](double x) { return x > 2.0; }) / n;
    out.over_2p5ms_pct = 100.0 * std::count_if(v.begin(), v.end(),
        [](double x) { return x > 2.5; }) / n;
    out.over_3ms_pct = 100.0 * std::count_if(v.begin(), v.end(),
        [](double x) { return x > 3.0; }) / n;
  }

  // 清空窗口（无 ring-buffer：窗口 = 报告间隔，内存 4KB 级可忽略）
  calls_.clear();
  periods_.clear();
  return true;
}

double ServoStats::mean(const std::vector<double> &v)
{
  if (v.empty()) return 0.0;
  return std::accumulate(v.begin(), v.end(), 0.0) / static_cast<double>(v.size());
}

double ServoStats::stddev(const std::vector<double> &v, double m)
{
  if (v.size() < 2) return 0.0;
  double sum = 0.0;
  for (double x : v) {
    const double d = x - m;
    sum += d * d;
  }
  return std::sqrt(sum / static_cast<double>(v.size() - 1));  // 样本标准差
}

double ServoStats::percentile(std::vector<double> v, double p)
{
  if (v.empty()) return 0.0;
  std::sort(v.begin(), v.end());
  const std::size_t idx = static_cast<std::size_t>(p * static_cast<double>(v.size() - 1));
  return v[idx];
}

}  // namespace fairino_hardware
