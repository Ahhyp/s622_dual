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
//
// C4.3 S4 W3-R1a / T-H13: the recorder also carries the six *phases* of one
// update() call (P1 readState .. P6 publishStatus; P0/P7 are begin()/end()
// themselves).  Phase marks are gated by the runtime parameter
// `timing_phases` (0 = phases off, 0x3F = all six): with the mask at 0 no
// clock is read and the recorded columns stay 0, so the control path is
// unchanged.  See docs/双臂协同/双臂协同运动控制与优化/
// C4.3_S4_W3-R1a_T-H13_RT相位计时点设计.md (frozen).

#include <atomic>
#include <cstdint>
#include <string>
#include <vector>

namespace dual_arm_qp_controller {

class TimingRecorder {
 public:
  /// P1..P6.  P0/P7 are begin()/end() and are not stored as phases.
  static constexpr int kPhases = 6;

  /// Per-phase aggregate over the samples in which the phase produced a
  /// measurable interval.  `recorded` counts those samples: a cycle that
  /// returns before a phase never reaches it and keeps a zero there, so
  /// `recorded` is a lower bound (a phase that ran in under the clock/float
  /// granularity also reads as zero).  mean/p50/p99/max are over the recorded
  /// samples only, so an early exit cannot drag a phase's p99 towards zero.
  struct PhaseSummary {
    int recorded = 0;
    double mean_us = 0.0;
    double p50_us = 0.0;
    double p99_us = 0.0;
    double max_us = 0.0;
  };

  struct Summary {
    int samples = 0;
    int capacity = 0;
    int dropped = 0;
    bool enabled = false;
    int phase_mask = 0;

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

    /// C4.3 S4 W3-R1a: exit-branch census (0=idle, 1=transport, 2=early_exit)
    /// and the per-phase split of the cycles recorded since configure().
    int branch_idle = 0;
    int branch_transport = 0;
    int branch_early_exit = 0;
    PhaseSummary phase[kPhases];

    double rss_reset_kb = 0.0;
    double rss_now_kb = 0.0;
    double rss_growth_kb = 0.0;
    long fd_reset = 0;
    long fd_now = 0;
    long fd_growth = 0;
  };

  /// Non-RT.  `capacity` is the number of cycles to keep; `period_us` is the
  /// control period used to count deadline overruns (1e6 * DualArmQpParams::dt).
  /// `phase_mask` selects which of P1..P6 are marked (bit i = phase i+1).
  void configure(int capacity, double period_us, int phase_mask);

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

  /// True if phase `phase` (0-based: P1..P6) is switched on.  Reading this
  /// before calling nowNs() is what keeps `timing_phases=0` free: no clock is
  /// read and no counter is touched.
  bool phaseEnabled(int phase) const noexcept {
    return capacity_ > 0 && (phase_mask_ & (1u << static_cast<unsigned>(phase))) != 0;
  }

  /// RT path.  Call at the very start of update().  Starts P1: the next mark()
  /// measures from this instant.
  void begin(uint64_t now_ns) noexcept {
    t_begin_ns_ = now_ns;
    t_mark_ns_ = now_ns;
  }

  /// RT path.  Closes phase `phase` at `now_ns`: the elapsed time since the
  /// previous mark (or begin()) is accumulated into that phase.  Callers ask
  /// phaseEnabled() first, so a masked-off phase costs nothing.
  void mark(int phase, uint64_t now_ns) noexcept {
    if (!phaseEnabled(phase)) return;
    const uint64_t dt_ns = now_ns > t_mark_ns_ ? now_ns - t_mark_ns_ : 0;
    t_mark_ns_ = now_ns;
    phase_acc_[phase] += static_cast<float>(static_cast<double>(dt_ns) * 1e-3);
  }

  /// RT path.  Labels the cycle that end() is about to close.  Defaults are
  /// set per cycle by the caller (2/0 = early exit until a branch is known).
  void setOutcome(int branch, int exit_code) noexcept {
    cur_branch_ = branch;
    cur_exit_code_ = exit_code;
  }

  /// RT path.  Call on every exit path from update().  No allocation.
  void end(double solve_us) noexcept;

  /// Non-RT.  Sorts copies of the samples; never called from the control loop.
  Summary summarize() const;

  /// Non-RT.  Writes one row per recorded cycle:
  /// `k,wall_ns,cycle_us,solve_us,interval_us,branch,exit_code,
  ///  p1_read_us,p2_dispatch_us,p3_safety_us,p4_task_solve_us,p5_write_us,
  ///  p6_publish_us`.  The first five columns keep their old positions so the
  /// previously accepted measurements stay index-compatible.
  /// Returns false with `*err` set on I/O failure.
  bool dumpCsv(const std::string& path, std::string* err) const;

  /// "samples=6375 cycle_p99_us=... overruns=..." -- one line, for the service
  /// response and for the acceptance record.
  static std::string format(const Summary& s);

 private:
  static double nowRssKb();
  static long nowFdCount();

  int capacity_ = 0;
  double period_us_ = 0.0;
  unsigned phase_mask_ = 0;

  std::vector<uint64_t> wall_ns_;
  std::vector<float> cycle_us_;
  std::vector<float> solve_us_;
  std::vector<float> interval_us_;
  // C4.3 S4 W3-R1a: one column group per phase, plus the exit-branch label.
  std::vector<float> phase_us_[kPhases];
  std::vector<int> branch_;
  std::vector<int> exit_code_;

  std::atomic<int> count_{0};
  int dropped_ = 0;
  uint64_t t_begin_ns_ = 0;
  uint64_t t_mark_ns_ = 0;
  uint64_t t_prev_begin_ns_ = 0;
  bool have_prev_ = false;
  float phase_acc_[kPhases] = {0, 0, 0, 0, 0, 0};
  int cur_branch_ = 2;   ///< 0=idle, 1=transport, 2=early_exit (default)
  int cur_exit_code_ = 0;

  double rss_reset_kb_ = 0.0;
  long fd_reset_ = 0;
};

}  // namespace dual_arm_qp_controller
