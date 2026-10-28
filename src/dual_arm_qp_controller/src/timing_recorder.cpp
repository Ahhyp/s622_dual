#include "dual_arm_qp_controller/timing_recorder.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <numeric>
#include <sstream>

namespace dual_arm_qp_controller {
namespace {

double percentile(const std::vector<float>& sorted, double p) {
  if (sorted.empty()) return 0.0;
  const double idx = p * static_cast<double>(sorted.size() - 1);
  const auto lo = static_cast<size_t>(std::floor(idx));
  const auto hi = static_cast<size_t>(std::ceil(idx));
  const double frac = idx - static_cast<double>(lo);
  return static_cast<double>(sorted[lo]) * (1.0 - frac) +
         static_cast<double>(sorted[hi]) * frac;
}

}  // namespace

void TimingRecorder::configure(int capacity, double period_us, int phase_mask) {
  capacity_ = std::max(0, capacity);
  period_us_ = period_us;
  phase_mask_ = static_cast<unsigned>(std::max(0, phase_mask)) & ((1U << kPhases) - 1U);
  wall_ns_.assign(static_cast<size_t>(capacity_), 0);
  cycle_us_.assign(static_cast<size_t>(capacity_), 0.0F);
  solve_us_.assign(static_cast<size_t>(capacity_), 0.0F);
  interval_us_.assign(static_cast<size_t>(capacity_), 0.0F);
  for (int p = 0; p < kPhases; ++p) phase_us_[p].assign(static_cast<size_t>(capacity_), 0.0F);
  branch_.assign(static_cast<size_t>(capacity_), 2);
  exit_code_.assign(static_cast<size_t>(capacity_), 0);
  reset();
}

void TimingRecorder::reset() {
  count_.store(0, std::memory_order_release);
  dropped_ = 0;
  have_prev_ = false;
  t_begin_ns_ = 0;
  t_mark_ns_ = 0;
  t_prev_begin_ns_ = 0;
  for (int p = 0; p < kPhases; ++p) phase_acc_[p] = 0.0F;
  cur_branch_ = 2;
  cur_exit_code_ = 0;
  if (capacity_ > 0) {
    rss_reset_kb_ = nowRssKb();
    fd_reset_ = nowFdCount();
  }
}

void TimingRecorder::rebaseResources() {
  if (capacity_ <= 0) return;
  rss_reset_kb_ = nowRssKb();
  fd_reset_ = nowFdCount();
}

void TimingRecorder::end(double solve_us) noexcept {
  if (capacity_ <= 0) return;
  const uint64_t t_end_ns = static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now().time_since_epoch())
          .count());
  const int n = count_.load(std::memory_order_relaxed);
  double interval_us = 0.0;
  if (have_prev_ && t_begin_ns_ > t_prev_begin_ns_) {
    interval_us = static_cast<double>(t_begin_ns_ - t_prev_begin_ns_) * 1e-3;
  }
  t_prev_begin_ns_ = t_begin_ns_;
  have_prev_ = true;

  if (n >= capacity_) {
    // Overflow: keep dropping, but never let a phase accumulator leak into the
    // next cycle -- end() is the only place that resets it.
    for (int p = 0; p < kPhases; ++p) phase_acc_[p] = 0.0F;
    ++dropped_;
    return;
  }
  wall_ns_[static_cast<size_t>(n)] = t_begin_ns_;
  cycle_us_[static_cast<size_t>(n)] = static_cast<float>(
      static_cast<double>(t_end_ns > t_begin_ns_ ? t_end_ns - t_begin_ns_ : 0) * 1e-3);
  solve_us_[static_cast<size_t>(n)] = static_cast<float>(solve_us);
  interval_us_[static_cast<size_t>(n)] = static_cast<float>(interval_us);
  branch_[static_cast<size_t>(n)] = cur_branch_;
  exit_code_[static_cast<size_t>(n)] = cur_exit_code_;
  for (int p = 0; p < kPhases; ++p) {
    phase_us_[p][static_cast<size_t>(n)] = phase_acc_[p];
    phase_acc_[p] = 0.0F;
  }
  count_.store(n + 1, std::memory_order_release);
}

TimingRecorder::Summary TimingRecorder::summarize() const {
  Summary s;
  s.enabled = enabled();
  s.capacity = capacity_;
  s.period_us = period_us_;
  s.dropped = dropped_;
  s.samples = count_.load(std::memory_order_acquire);
  if (capacity_ > 0) {
    s.rss_reset_kb = rss_reset_kb_;
    s.rss_now_kb = nowRssKb();
    s.rss_growth_kb = s.rss_now_kb - s.rss_reset_kb;
    s.fd_reset = fd_reset_;
    s.fd_now = nowFdCount();
    s.fd_growth = s.fd_now - s.fd_reset;
  }
  if (s.samples <= 0) return s;

  std::vector<float> cyc(cycle_us_.begin(), cycle_us_.begin() + s.samples);
  std::vector<float> slv(solve_us_.begin(), solve_us_.begin() + s.samples);
  std::vector<float> itv(interval_us_.begin(), interval_us_.begin() + s.samples);
  std::sort(cyc.begin(), cyc.end());
  std::sort(slv.begin(), slv.end());

  const double sum = std::accumulate(cyc.begin(), cyc.end(), 0.0);
  s.cycle_mean_us = sum / static_cast<double>(cyc.size());
  s.cycle_min_us = static_cast<double>(cyc.front());
  s.cycle_max_us = static_cast<double>(cyc.back());
  s.cycle_p50_us = percentile(cyc, 0.50);
  s.cycle_p95_us = percentile(cyc, 0.95);
  s.cycle_p99_us = percentile(cyc, 0.99);
  s.cycle_p999_us = percentile(cyc, 0.999);

  const double ssum = std::accumulate(slv.begin(), slv.end(), 0.0);
  s.solve_mean_us = ssum / static_cast<double>(slv.size());
  s.solve_max_us = static_cast<double>(slv.back());
  s.solve_p99_us = percentile(slv, 0.99);
  s.solve_p999_us = percentile(slv, 0.999);

  s.interval_max_us = static_cast<double>(*std::max_element(itv.begin(), itv.end()));
  if (period_us_ > 0.0) {
    for (int i = 1; i < s.samples; ++i) {  // i=0 has no predecessor
      if (static_cast<double>(itv[static_cast<size_t>(i)]) > period_us_) ++s.overruns;
    }
    s.overrun_ratio =
        s.samples > 1 ? static_cast<double>(s.overruns) / static_cast<double>(s.samples - 1) : 0.0;
  }

  // C4.3 S4 W3-R1a: exit-branch census.  A sample recorded before the branch is
  // known keeps the default 2 (early exit), so the three counts always sum to
  // s.samples.
  s.phase_mask = static_cast<int>(phase_mask_);
  for (int i = 0; i < s.samples; ++i) {
    const int b = branch_[static_cast<size_t>(i)];
    if (b == 0) {
      ++s.branch_idle;
    } else if (b == 1) {
      ++s.branch_transport;
    } else {
      ++s.branch_early_exit;
    }
  }

  // Per-phase split.  A phase that never ran (early return above it) keeps its
  // zero, so `recorded` counts the samples in which the phase produced a
  // measurable interval; a phase that ran in less than the float/clock
  // granularity is indistinguishable from an unreached one, which makes
  // `recorded` a lower bound.  Mean/percentiles are over the recorded samples
  // only, so an early exit does not drag a phase's p99 towards zero.
  for (int p = 0; p < kPhases; ++p) {
    std::vector<float> pv;
    pv.reserve(static_cast<size_t>(s.samples));
    for (int i = 0; i < s.samples; ++i) {
      const float v = phase_us_[p][static_cast<size_t>(i)];
      if (v > 0.0F) pv.push_back(v);
    }
    PhaseSummary& ps = s.phase[p];
    ps.recorded = static_cast<int>(pv.size());
    if (pv.empty()) continue;
    std::sort(pv.begin(), pv.end());
    const double psum = std::accumulate(pv.begin(), pv.end(), 0.0);
    ps.mean_us = psum / static_cast<double>(pv.size());
    ps.p50_us = percentile(pv, 0.50);
    ps.p99_us = percentile(pv, 0.99);
    ps.max_us = static_cast<double>(pv.back());
  }
  return s;
}

bool TimingRecorder::dumpCsv(const std::string& path, std::string* err) const {
  if (path.empty()) {
    if (err) *err = "timing_csv_path is empty";
    return false;
  }
  std::FILE* f = std::fopen(path.c_str(), "w");
  if (f == nullptr) {
    if (err) *err = "cannot open " + path;
    return false;
  }
  const int n = count_.load(std::memory_order_acquire);
  // Columns 0..4 are unchanged (kWallNs=1 ... interval=4): the W1/S4-T2 and
  // S4-T17 readers are index based.  C4.3 S4 W3-R1a appends branch/exit_code
  // and the six phase columns.
  std::fprintf(f,
               "k,wall_ns,cycle_us,solve_us,interval_us,branch,exit_code,"
               "p1_read_us,p2_dispatch_us,p3_safety_us,p4_task_solve_us,p5_write_us,"
               "p6_publish_us\n");
  for (int i = 0; i < n; ++i) {
    const size_t k = static_cast<size_t>(i);
    std::fprintf(f, "%d,%llu,%.6f,%.6f,%.6f,%d,%d", i,
                 static_cast<unsigned long long>(wall_ns_[k]),
                 static_cast<double>(cycle_us_[k]), static_cast<double>(solve_us_[k]),
                 static_cast<double>(interval_us_[k]), branch_[k], exit_code_[k]);
    for (int p = 0; p < kPhases; ++p) {
      std::fprintf(f, ",%.6f", static_cast<double>(phase_us_[p][k]));
    }
    std::fputc('\n', f);
  }
  const bool ok = std::fclose(f) == 0;
  if (!ok && err) *err = "fclose failed for " + path;
  return ok;
}

std::string TimingRecorder::format(const Summary& s) {
  std::ostringstream o;
  o.setf(std::ios::fixed);
  o.precision(3);
  if (!s.enabled) {
    o << "timing disabled (timing_enable=false)";
    return o.str();
  }
  o << "samples=" << s.samples << " cap=" << s.capacity << " dropped=" << s.dropped
    << " period_us=" << s.period_us << " cycle_us{mean=" << s.cycle_mean_us
    << ",min=" << s.cycle_min_us << ",p50=" << s.cycle_p50_us << ",p95=" << s.cycle_p95_us
    << ",p99=" << s.cycle_p99_us << ",p999=" << s.cycle_p999_us << ",max=" << s.cycle_max_us
    << "} solve_us{mean=" << s.solve_mean_us << ",p99=" << s.solve_p99_us
    << ",p999=" << s.solve_p999_us << ",max=" << s.solve_max_us
    << "} interval_max_us=" << s.interval_max_us << " overruns=" << s.overruns
    << " overrun_ratio=" << s.overrun_ratio << " rss_kb{" << s.rss_reset_kb << "->"
    << s.rss_now_kb << ",growth=" << s.rss_growth_kb << "} fd{" << s.fd_reset << "->"
    << s.fd_now << ",growth=" << s.fd_growth << "}";
  // C4.3 S4 W3-R1a: appended after the W1 fields so the existing one-line
  // format keeps its prefix.
  o << " phase_mask=" << s.phase_mask << " branch{idle=" << s.branch_idle
    << ",transport=" << s.branch_transport << ",early_exit=" << s.branch_early_exit << "}";
  for (int p = 0; p < kPhases; ++p) {
    const PhaseSummary& ps = s.phase[p];
    o << " p" << (p + 1) << "_us{rec=" << ps.recorded << ",mean=" << ps.mean_us
      << ",p50=" << ps.p50_us << ",p99=" << ps.p99_us << ",max=" << ps.max_us << "}";
  }
  return o.str();
}

double TimingRecorder::nowRssKb() {
  std::FILE* f = std::fopen("/proc/self/statm", "r");
  if (f == nullptr) return 0.0;
  long size = 0;
  long resident = 0;
  const int got = std::fscanf(f, "%ld %ld", &size, &resident);
  std::fclose(f);
  if (got != 2) return 0.0;
  const long page_kb = 4;  // 4 KiB pages; only ratios matter for the growth test
  return static_cast<double>(resident * page_kb);
}

long TimingRecorder::nowFdCount() {
  std::error_code ec;
  long n = 0;
  for (auto it = std::filesystem::directory_iterator("/proc/self/fd", ec);
       !ec && it != std::filesystem::directory_iterator(); ++it) {
    ++n;
  }
  return n;
}

}  // namespace dual_arm_qp_controller
