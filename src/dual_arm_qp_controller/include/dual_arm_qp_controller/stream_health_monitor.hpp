// include/dual_arm_qp_controller/stream_health_monitor.hpp
// D7: dual-arm command/feedback stream health interlock.
//
// Decision D7 (see docs/双臂协同/双臂协同运动控制与优化/架构决策单_D1-D7与Step0.md):
//   in coordinated mode, if EITHER arm's command/feedback stream becomes
//   unhealthy, BOTH arms must hold.  A healthy arm must never keep moving
//   while the other is stalled, because that instantly invalidates the
//   relative-pose constraint in the physical world.
//
// This class is deliberately ROS-free so it can be unit tested directly.

#pragma once

#include <Eigen/Dense>
#include <cmath>
#include <string>

namespace dual_arm_qp_controller {

using Vec12 = Eigen::Matrix<double, 12, 1>;

struct StreamHealthParams {
    /// Per-arm max |q_state - q_cmd| before the arm is considered unhealthy
    /// (rad).  <= 0 disables the check.
    double tracking_error_limit = 0.0;
    /// Consecutive QP solve failures that count as "global unhealthy".
    int solve_failure_limit = 5;
    /// Simulation fault injection: -1 = off, 0 = left, 1 = right.
    int stall_inject_arm = -1;
    /// [s] after activation when the injected stall starts (<0 disables).
    double stall_start_s = -1.0;
    /// [s] duration of the injected stall.
    double stall_duration_s = 0.0;
};

/// Result of one health evaluation.
struct StreamHealth {
    bool healthy = true;
    int unhealthy_arm = -1;  // 0 left, 1 right, -1 none
    std::string reason;
};

class StreamHealthMonitor {
public:
    void configure(const StreamHealthParams& params) { params_ = params; }

    void reset() {
        solve_fail_streak_ = 0;
        last_ = StreamHealth{};
    }

    /// @param elapsed_s  seconds since activation
    /// @param q          measured joint positions (12)
    /// @param q_cmd      last commanded joint positions (12)
    /// @param solve_fail_streak  consecutive QP solve failures
    StreamHealth evaluate(double elapsed_s, const Vec12& q, const Vec12& q_cmd,
                          int solve_fail_streak, bool deadline_fault = false) {
        solve_fail_streak_ = solve_fail_streak;
        StreamHealth out;

        // 1) non-finite state or command -> that arm is unhealthy
        for (int arm = 0; arm < 2; ++arm) {
            for (int i = 0; i < 6; ++i) {
                const int k = arm * 6 + i;
                if (!std::isfinite(q[k]) || !std::isfinite(q_cmd[k])) {
                    out.healthy = false;
                    out.unhealthy_arm = arm;
                    out.reason = "non-finite state/command";
                    last_ = out;
                    return out;
                }
            }
        }

        // 1b) control-period deadline overrun (C2.R2): global hold
        if (deadline_fault) {
            out.healthy = false;
            out.unhealthy_arm = -1;
            out.reason = "control period overrun";
            last_ = out;
            return out;
        }

        // 2) injected single-arm stall (simulation only)
        if (params_.stall_inject_arm >= 0 && params_.stall_start_s >= 0.0 &&
            elapsed_s >= params_.stall_start_s &&
            elapsed_s < params_.stall_start_s + params_.stall_duration_s) {
            out.healthy = false;
            out.unhealthy_arm = params_.stall_inject_arm;
            out.reason = "injected stream stall";
            last_ = out;
            return out;
        }

        // 3) per-arm tracking error (proxy for over-load / stream desync)
        if (params_.tracking_error_limit > 0.0) {
            for (int arm = 0; arm < 2; ++arm) {
                double max_err = 0.0;
                for (int i = 0; i < 6; ++i) {
                    max_err = std::max(max_err, std::abs(q[arm * 6 + i] - q_cmd[arm * 6 + i]));
                }
                if (max_err > params_.tracking_error_limit) {
                    out.healthy = false;
                    out.unhealthy_arm = arm;
                    out.reason = "tracking error " + std::to_string(max_err);
                    last_ = out;
                    return out;
                }
            }
        }

        // 4) persistent solver failure is a global (both-arm) hold
        if (params_.solve_failure_limit > 0 && solve_fail_streak >= params_.solve_failure_limit) {
            out.healthy = false;
            out.unhealthy_arm = -1;
            out.reason = "QP solve failures";
            last_ = out;
            return out;
        }

        last_ = out;
        return out;
    }

    const StreamHealth& last() const { return last_; }

private:
    StreamHealthParams params_;
    int solve_fail_streak_ = 0;
    StreamHealth last_;
};

}  // namespace dual_arm_qp_controller
