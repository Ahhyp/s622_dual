#pragma once

// §7 S4-T2 (realtime budget) / S4-T17 (long-run realtime): per-cycle timing
// recorder for the transport controller.
//
// Design constraints, deliberately chosen so that measuring does not perturb
// what is being measured:
//   * one allocation, at configure()/reset() -- never in record();
//   * no syscall in the per-cycle path: RSS / fd counts are sampled at
//     reset() and at summarize(), not per cycle, so P99 is not polluted by
//     /proc reads (a stride-based sampler at 1% duty would land exactly on the
//     P99 boundary, which would defeat the purpose);
//   * the control thread and the dumping thread share the buffer through a
//     release-store counter, so a dump only ever sees whole samples.
//
// Disabled by default (`timing_enable=false`): a production controller pays
// nothing and behaves exactly as it did in W1.

#include <atomic>
#include <cstdint>
#include <string>
#include <vector>

namespace dual_arm_qp_controller {

class TimingRecorder {
 public:
  struct Summary {
    int samples = 0;
    int capacity = 0;
    int dropped = 0;
    bool enabled = false;

    double period_us = 0.0;

    double cycle_mean_us = 0.0;
    double cycle_min_us = 0.0;
    double cycle_max_us = 0.0;
    double cycle_p50_us = 0.0;
    double cycle_p95_us = 0.0;
    double cycle_p99_us = 0.0;
    double cycle_p999_us = 0.0;

    double solve_mean_us = 0.0;
    double solve_max_us = 0.0;
    double solve_p99_us = 0.0;
    double solve_p999_us = 0.0;

    double interval_max_us = 0.0;
    int overruns = 0;          ///< interval_us > period_us
    double overrun_ratio = 0.0;

    double rss_reset_kb = 0.0;
    double rss_now_kb = 0.0;
    double rss_growth_kb = 0.0;
    long fd_reset = 0;
    long fd_now = 0;
    long fd_growth = 0;
  };

  /// Non-RT.  `capacity` is the number of cycles to keep; `period_us` is the
  /// control period used to count deadline overruns (1e6 * DualArmQpParams::dt).
  void configure(int capacity, double period_us);

  /// Non-RT.  Clears samples and re-samples the RSS / fd baselines.
  void reset();

  /// Non-RT.  Re-samples the RSS / fd baselines without clearing samples.
  /// Called when a transport starts: S4-T17 is about drift *during the run*, so
  /// the baseline must be taken after the process has warmed up (ROS graph
  /// allocation happens in the first second and would otherwise be charged to
  /// the run and blow the 1 MB budget for the wrong reason).
  void rebaseResources();

  bool enabled() const { return capacity_ > 0; }
  int capacity() const { return capacity_; }

  /// RT path.  Call at the very start of update().
  void begin(uint64_t now_ns) noexcept { t_begin_ns_ = now_ns; }

  /// RT path.  Call on every exit path from update().  No allocation.
  void end(double solve_us) noexcept;

  /// Non-RT.  Sorts copies of the samples; never called from the control loop.
  Summary summarize() const;

  /// Non-RT.  Writes `k,wall_ns,cycle_us,solve_us,interval_us` for every
  /// recorded cycle.  Returns false with `*err` set on I/O failure.
  bool dumpCsv(const std::string& path, std::string* err) const;

  /// "samples=6375 cycle_p99_us=... overruns=..." -- one line, for the service
  /// response and for the acceptance record.
  static std::string format(const Summary& s);

 private:
  static double nowRssKb();
  static long nowFdCount();

  int capacity_ = 0;
  double period_us_ = 0.0;

  std::vector<uint64_t> wall_ns_;
  std::vector<float> cycle_us_;
  std::vector<float> solve_us_;
  std::vector<float> interval_us_;

  std::atomic<int> count_{0};
  int dropped_ = 0;
  uint64_t t_begin_ns_ = 0;
  uint64_t t_prev_begin_ns_ = 0;
  bool have_prev_ = false;

  double rss_reset_kb_ = 0.0;
  long fd_reset_ = 0;
};

}  // namespace dual_arm_qp_controller
