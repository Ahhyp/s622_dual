// include/dual_arm_qp_controller/period_monitor.hpp
// C2.R2: real control-period semantics.
//
// The QP is designed around dt_nominal (8 ms), but the controller must not
// pretend the scheduler is perfect.  This monitor:
//   - measures the ACTUAL period handed to update();
//   - flags deadline violations (period > nominal * fault_factor);
//   - keeps a bounded, allocation-free window for mean / P95 / P99 / max.
// A deadline violation is fed into the D7 interlock (both arms hold).
//
// ROS-free so it can be unit tested directly.

#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>

namespace dual_arm_qp_controller {

class PeriodMonitor {
public:
    static constexpr int kWindow = 4096;

    void configure(double nominal_s, double fault_factor = 2.0) {
        nominal_s_ = (nominal_s > 0.0 && std::isfinite(nominal_s)) ? nominal_s : 0.008;
        fault_factor_ = (fault_factor > 1.0 && std::isfinite(fault_factor)) ? fault_factor : 2.0;
        reset();
    }

    void reset() {
        window_.fill(0.0);
        idx_ = 0;
        filled_ = 0;
        sum_ = 0.0;
        max_ = 0.0;
        count_ = 0;
        faults_ = 0;
        last_ = nominal_s_;
    }

    /// Feed the actual period [s].  @return true if it is a deadline violation.
    bool update(double period_s) {
        if (!std::isfinite(period_s) || period_s <= 0.0) period_s = nominal_s_;
        last_ = period_s;
        if (window_[idx_] > 0.0) sum_ -= window_[idx_];  // overwrite oldest
        window_[idx_] = period_s;
        sum_ += period_s;
        idx_ = (idx_ + 1) % kWindow;
        if (filled_ < kWindow) ++filled_;
        max_ = std::max(max_, period_s);
        ++count_;
        const bool fault = period_s > nominal_s_ * fault_factor_;
        if (fault) ++faults_;
        return fault;
    }

    double lastMs() const { return last_ * 1e3; }
    double nominalMs() const { return nominal_s_ * 1e3; }
    double faultMs() const { return nominal_s_ * fault_factor_ * 1e3; }
    double meanMs() const { return filled_ > 0 ? (sum_ / filled_) * 1e3 : 0.0; }
    double maxMs() const { return max_ * 1e3; }
    long long count() const { return count_; }
    long long faults() const { return faults_; }

    /// Percentile [0,1] over the current window (sorts a stack copy).
    double percentileMs(double p) const {
        if (filled_ == 0) return 0.0;
        std::array<double, kWindow> tmp{};
        for (int i = 0; i < filled_; ++i) tmp[i] = window_[i];
        std::sort(tmp.begin(), tmp.begin() + filled_);
        const double clamped = std::min(1.0, std::max(0.0, p));
        const int idx = static_cast<int>(clamped * static_cast<double>(filled_ - 1));
        return tmp[idx] * 1e3;
    }

private:
    double nominal_s_ = 0.008;
    double fault_factor_ = 2.0;
    std::array<double, kWindow> window_{};
    int idx_ = 0;
    int filled_ = 0;
    double sum_ = 0.0;
    double max_ = 0.0;
    long long count_ = 0;
    long long faults_ = 0;
    double last_ = 0.008;
};

}  // namespace dual_arm_qp_controller
