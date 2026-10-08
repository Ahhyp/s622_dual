// test/test_closed_loop_c23.cpp
// C2.3 acceptance: closed-loop "relative pose hold" with a non-ideal actuator.
//
// The mock plant (GenericSystem) makes state == command, so it cannot show
// closed-loop behaviour.  Here we simulate a first-order actuator
//     q_{k+1} = q_k + (dt/tau) * (q_cmd_k - q_k) + disturbance_k
// and drive it with the SAME DualArmQp used by the ROS controller.
//
// It provides the C2.3 numbers the plan asks for:
//   relative-pose RMS / max, recovery time after a push, steady-state error.

#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <cmath>
#include <cstdio>
#include <random>

#include "dual_arm_kinematics/dual_arm_kinematics.hpp"
#include "dual_arm_qp_core/dual_arm_qp.hpp"

namespace {

using dual_arm::DualArmKinematics;
using dual_arm::Iso3;
using dual_arm::JointConfig;
using dual_arm::Q12;
using dual_arm_qp::DualArmQp;
using dual_arm_qp::DualArmQpParams;

const double kLower[6] = {-3.0543, -4.6251, -2.8274, -4.6251, -3.0543, -3.0543};
const double kUpper[6] = {3.0543, 1.4835, 2.8274, 1.4835, 3.0543, 3.0543};

JointConfig randomQ(std::mt19937& rng, double shrink) {
    JointConfig q;
    std::uniform_real_distribution<double> unit(0.0, 1.0);
    for (int i = 0; i < 6; ++i) {
        const double mid = 0.5 * (kLower[i] + kUpper[i]);
        const double half = 0.5 * (kUpper[i] - kLower[i]) * shrink;
        q[i] = mid + (2.0 * unit(rng) - 1.0) * half;
    }
    return q;
}

struct RelErr {
    double pos_mm = 0.0;
    double rot_deg = 0.0;
    double norm() const { return std::hypot(pos_mm, rot_deg); }
};

RelErr relativeError(const DualArmKinematics& kin, const Q12& q, const Iso3& target) {
    const dual_arm::Vec6 e = kin.relativeError(q, target);
    RelErr out;
    out.pos_mm = e.head<3>().norm() * 1000.0;
    out.rot_deg = e.tail<3>().norm() * 180.0 / M_PI;
    return out;
}

/// Simulates the plant + QP for `steps`, returns the error after each step.
std::vector<RelErr> runClosedLoop(const DualArmKinematics& kin, DualArmQpParams params,
                                  const Iso3& target, Q12 q, double tau,
                                  const std::function<void(int, double, Q12*)>& disturbance,
                                  int steps, double* solve_ms = nullptr) {
    DualArmQp qp(kin, params);
    Q12 q_cmd = q;
    const double dt = params.dt;
    std::vector<RelErr> errs;
    errs.reserve(steps);
    double total_ms = 0.0;
    for (int k = 0; k < steps; ++k) {
        const auto res = qp.solve(q, target);
        total_ms += res.solve_time_ms;
        if (res.converged) {
            q_cmd += res.dq;
            for (int i = 0; i < 12; ++i) {
                q_cmd[i] = std::min(params.q_max[i], std::max(params.q_min[i], q_cmd[i]));
            }
        }
        // first-order actuator
        q += (dt / tau) * (q_cmd - q);
        // external disturbance (state velocity bias)
        if (disturbance) disturbance(k, dt, &q);
        // hard clamp (joint limits are physical)
        for (int i = 0; i < 12; ++i) {
            q[i] = std::min(params.q_max[i], std::max(params.q_min[i], q[i]));
        }
        errs.push_back(relativeError(kin, q, target));
    }
    if (solve_ms) *solve_ms = total_ms / steps;
    return errs;
}

}  // namespace

TEST(C23ClosedLoop, PushAndDisturbanceRecovery) {
    DualArmKinematics kin;
    DualArmQpParams params;
    params.dt = 0.008;       // 125 Hz
    params.kp_rel = 10.0;    // 1/s
    params.w_rel = 1.0;
    params.w_slack = 1e3;
    params.w_center = 1e-2;
    params.w_reg = 1e-6;

    std::mt19937 rng(20261008u);
    const Q12 q0 = DualArmKinematics::stack(randomQ(rng, 0.6), randomQ(rng, 0.6));
    const Iso3 target = kin.leftTright(q0);

    const double tau = 0.05;          // 50 ms actuator time constant
    const int steps = 625;            // 5 s at 125 Hz
    const int k_push = 75;            // t = 0.6 s
    const int k_dist_start = 250;     // t = 2.0 s
    const int k_dist_end = 300;       // t = 2.4 s
    const double dist_rate = 0.25;    // rad/s bias on right j2/j3 during the window

    Q12 push_delta = Q12::Zero();
    push_delta[7] = 0.03;   // right j2
    push_delta[8] = -0.02;  // right j3

    const auto errs = runClosedLoop(
        kin, params, target, q0, tau,
        [&](int k, double dt, Q12* q) {
            if (k == k_push) *q += push_delta;  // instantaneous push
            if (k >= k_dist_start && k < k_dist_end) {
                (*q)[7] += dist_rate * dt;      // sustained disturbance
                (*q)[8] -= dist_rate * dt;
            }
        },
        steps);

    double push_peak = 0.0, push_rot_peak = 0.0;
    for (int k = k_push; k < k_dist_start; ++k) {
        push_peak = std::max(push_peak, errs[k].pos_mm);
        push_rot_peak = std::max(push_rot_peak, errs[k].rot_deg);
    }
    double dist_peak = 0.0, dist_rot_peak = 0.0;
    for (int k = k_dist_start; k < k_dist_end; ++k) {
        dist_peak = std::max(dist_peak, errs[k].pos_mm);
        dist_rot_peak = std::max(dist_rot_peak, errs[k].rot_deg);
    }
    // recovery time after the sustained disturbance ends: first k >= k_dist_end
    // with error below 1 mm / 0.05 deg, kept for the rest of the run
    int recover_k = -1;
    for (int k = k_dist_end; k < steps; ++k) {
        bool ok = true;
        for (int j = k; j < steps; ++j) {
            if (errs[j].pos_mm > 1.0 || errs[j].rot_deg > 0.05) { ok = false; break; }
        }
        if (ok) { recover_k = k; break; }
    }
    // steady state over the last 0.5 s
    double ss_pos = 0.0, ss_rot = 0.0;
    int n_ss = 0;
    for (int k = steps - 63; k < steps; ++k) {
        ss_pos += errs[k].pos_mm;
        ss_rot += errs[k].rot_deg;
        ++n_ss;
    }
    ss_pos /= n_ss;
    ss_rot /= n_ss;

    std::printf("[C2.3] push peak = %.3f mm / %.4f deg | disturbance peak = %.3f mm / %.4f deg\n",
                push_peak, push_rot_peak, dist_peak, dist_rot_peak);
    std::printf("[C2.3] recovery after disturbance = %.3f s | steady state = %.4f mm / %.5f deg\n",
                recover_k < 0 ? -1.0 : (recover_k - k_dist_end) * params.dt, ss_pos, ss_rot);
    // print a coarse trace
    std::printf("[C2.3] trace (mm):");
    for (int k = 50; k < steps; k += 50) std::printf(" %.2f", errs[k].pos_mm);
    std::printf("\n");

    EXPECT_GT(push_peak, 1.0);            // the push really perturbs the relative pose
    EXPECT_LT(errs.back().pos_mm, 1.0);   // and it is driven back
    EXPECT_LT(errs.back().rot_deg, 0.05);
    EXPECT_GE(recover_k, 0);
    EXPECT_LT(ss_pos, 1.0);
}

TEST(C23ClosedLoop, MeasurementNoiseRMS) {
    DualArmKinematics kin;
    DualArmQpParams params;
    params.dt = 0.008;
    params.kp_rel = 10.0;
    params.w_slack = 1e3;

    std::mt19937 rng(7u);
    const Q12 q0 = DualArmKinematics::stack(randomQ(rng, 0.5), randomQ(rng, 0.5));
    const Iso3 target = kin.leftTright(q0);

    DualArmQp qp(kin, params);
    Q12 q = q0, q_cmd = q0, q_true = q0;
    std::normal_distribution<double> noise(0.0, 1e-4);  // 0.1 mrad encoder noise
    const double dt = params.dt, tau = 0.05;
    const int steps = 625;
    double sum_sq_pos = 0.0, sum_sq_rot = 0.0, max_pos = 0.0;
    for (int k = 0; k < steps; ++k) {
        for (int i = 0; i < 12; ++i) q[i] = q_true[i] + noise(rng);
        const auto res = qp.solve(q, target);
        if (res.converged) q_cmd += res.dq;
        q_true += (dt / tau) * (q_cmd - q_true);
        if (k > 125) {  // skip the initial transient
            const RelErr e = relativeError(kin, q_true, target);
            sum_sq_pos += e.pos_mm * e.pos_mm;
            sum_sq_rot += e.rot_deg * e.rot_deg;
            max_pos = std::max(max_pos, e.pos_mm);
        }
    }
    const int n = steps - 125;
    std::printf("[C2.3] noise 0.1 mrad: pos RMS = %.4f mm (max %.4f), rot RMS = %.5f deg\n",
                std::sqrt(sum_sq_pos / n), max_pos, std::sqrt(sum_sq_rot / n));
    EXPECT_LT(std::sqrt(sum_sq_pos / n), 1.0);
}
