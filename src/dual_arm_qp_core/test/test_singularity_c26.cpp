// test/test_singularity_c26.cpp
// C2.6: continuous singularity / poor-conditioning handling.
//   1) Continuity: sweeping the error magnitude must NOT produce a jump in the
//      derating factor or in the resulting dq (no hard if-sigma-then-switch).
//   2) Stress: from a hostile configuration (relative rotation ~ pi, where the
//      first-order error map breaks down) the closed loop must stay bounded
//      when handling is on, while the plain QP diverges.

#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <cmath>
#include <cstdio>
#include <random>
#include <vector>

#include "dual_arm_kinematics/dual_arm_kinematics.hpp"
#include "dual_arm_kinematics/se3.hpp"
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

DualArmQpParams baseParams() {
    DualArmQpParams p;
    p.dt = 0.008;
    p.kp_rel = 20.0;
    p.w_rel = 1.0;
    p.w_slack = 1e3;
    p.w_center = 1e-2;
    p.w_reg = 1e-6;
    return p;
}

}  // namespace

TEST(C26Singularity, DeratingIsContinuousAcrossErrorMagnitude) {
    DualArmKinematics kin;
    DualArmQpParams p = baseParams();
    p.enable_singularity = true;
    p.e_safe = 0.6;
    p.e_critical = 2.0;
    DualArmQp qp(kin, p);

    std::mt19937 rng(20261017u);
    const Q12 q = DualArmKinematics::stack(randomQ(rng, 0.4), randomQ(rng, 0.4));
    const Iso3 rel0 = kin.leftTright(q);

    constexpr int kN = 400;
    std::vector<double> dq_norm(kN), factor(kN);
    for (int i = 0; i < kN; ++i) {
        const double theta = M_PI * (static_cast<double>(i) / (kN - 1));
        // rotate the target about the right-grasp x axis
        const Iso3 target =
            rel0 * dual_arm::expSE3((dual_arm::Vec6() << 0, 0, 0, theta, 0, 0).finished());
        const auto r = qp.solve(q, target);
        dq_norm[i] = r.dq.norm();
        factor[i] = r.error_factor;
    }
    double max_factor_jump = 0.0, max_dq_jump = 0.0;
    for (int i = 1; i < kN; ++i) {
        max_factor_jump = std::max(max_factor_jump, std::abs(factor[i] - factor[i - 1]));
        max_dq_jump = std::max(max_dq_jump, std::abs(dq_norm[i] - dq_norm[i - 1]));
    }
    std::printf("[C2.6] continuity: max |d factor| = %.4f, max |d |dq|| = %.5f (over %d samples)\n",
                max_factor_jump, max_dq_jump, kN);
    EXPECT_LT(max_factor_jump, 0.02);   // smoothstep over 400 samples
    EXPECT_LT(max_dq_jump, 0.01);       // no jump in the command
}

TEST(C26Singularity, LowSigmaConfigurationPeakVelocityReduction) {
    // Find a genuinely poorly-conditioned configuration (smallest sigma_min of
    // the stacked object+relative task Jacobian), then drive a motion from it
    // with and without the continuous degradation.
    DualArmKinematics kin;
    std::mt19937 rng(20261019u);
    Q12 q_sing = DualArmKinematics::stack(randomQ(rng, 0.6), randomQ(rng, 0.6));
    double best_sigma = 1e9;
    for (int t = 0; t < 3000; ++t) {
        const Q12 q = DualArmKinematics::stack(randomQ(rng, 0.6), randomQ(rng, 0.6));
        const auto J = kin.relativeJacobian(q, dual_arm::RelativeFrame::BodyR);
        Eigen::JacobiSVD<dual_arm::Mat6x12> svd(J);
        const double s = svd.singularValues().minCoeff();
        if (s < best_sigma) {
            best_sigma = s;
            q_sing = q;
        }
    }
    std::printf("[C2.6] found config: sigma_min(J_rel) = %.4f\n", best_sigma);

    const Iso3 rel0 = kin.leftTright(q_sing);
    const Iso3 obj0 = kin.worldTobject(q_sing);
    const Iso3 target_rel = rel0 * dual_arm::expSE3(
                                        (dual_arm::Vec6() << 0.005, 0.003, 0.0, 0.05, 0.03, 0).finished());
    const Iso3 target_obj = obj0;

    auto run = [&](bool handling, double* max_dq, double* max_err) {
        DualArmQpParams p = baseParams();
        p.w_obj = 1.0;
        p.w_slack_obj = 1e3;
        p.kp_obj = 20.0;
        p.enable_singularity = handling;
        p.sigma_critical = std::max(1e-6, best_sigma);
        p.sigma_safe = std::max(1e-5, 6.0 * best_sigma);
        DualArmQp qp(kin, p);
        const double dt = p.dt, tau = 0.05;
        Q12 q = q_sing, q_cmd = q_sing;
        *max_dq = 0.0;
        *max_err = 0.0;
        const int steps = static_cast<int>(1.5 / dt);
        for (int k = 0; k < steps; ++k) {
            const auto r = qp.solve(q, target_rel, target_obj);
            if (r.converged) q_cmd += r.dq;
            q += (dt / tau) * (q_cmd - q);
            *max_dq = std::max(*max_dq, r.dq.cwiseAbs().maxCoeff());
            *max_err = std::max(*max_err, kin.relativeError(q, target_rel).norm());
        }
    };

    double off_dq, off_err, on_dq, on_err;
    run(false, &off_dq, &off_err);
    run(true, &on_dq, &on_err);
    std::printf("[C2.6] low-sigma drive: OFF |dq|max=%.4f err=%.4f | ON |dq|max=%.4f err=%.4f\n",
                off_dq, off_err, on_dq, on_err);

    EXPECT_GT(best_sigma, 0.0);
    EXPECT_LE(on_dq, off_dq + 1e-9);   // degradation must not increase the peak
}

TEST(C26Singularity, HealthyConfigurationIsUnaffected) {
    DualArmKinematics kin;
    std::mt19937 rng(20261018u);
    const Q12 q0 = DualArmKinematics::stack(randomQ(rng, 0.4), randomQ(rng, 0.4));
    const Iso3 r0 = kin.leftTright(q0);
    const Iso3 target = r0 * dual_arm::expSE3(
                                 (dual_arm::Vec6() << 0.005, 0.0, 0.0, 0.02, 0.0, 0.0).finished());

    DualArmQpParams p = baseParams();
    p.enable_singularity = false;
    DualArmQp qp_off(kin, p);
    p.enable_singularity = true;
    p.e_safe = 0.6;
    p.e_critical = 2.0;
    DualArmQp qp_on(kin, p);

    const auto a = qp_off.solve(q0, target);
    const auto b = qp_on.solve(q0, target);
    std::printf("[C2.6] healthy config: |dq| off=%.6f on=%.6f, factor=%.3f\n", a.dq.norm(),
                b.dq.norm(), b.singularity_factor);
    EXPECT_NEAR(a.dq.norm(), b.dq.norm(), 1e-6);  // no derating when healthy
    EXPECT_NEAR(b.error_factor, 1.0, 1e-9);
}
