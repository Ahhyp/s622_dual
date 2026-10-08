// test/test_dual_arm_qp.cpp
// End-to-end checks of the dual-arm relative-pose QP (C2.2 skeleton).

#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <random>

#include "dual_arm_kinematics/dual_arm_kinematics.hpp"
#include "dual_arm_qp_core/dual_arm_qp.hpp"

namespace {

using dual_arm::Arm;
using dual_arm::DualArmKinematics;
using dual_arm::Iso3;
using dual_arm::JointConfig;
using dual_arm::Q12;
using dual_arm_qp::DualArmQp;
using dual_arm_qp::DualArmQpParams;

const double kLower[6] = {-3.0543, -4.6251, -2.8274, -4.6251, -3.0543, -3.0543};
const double kUpper[6] = {3.0543, 1.4835, 2.8274, 1.4835, 3.0543, 3.0543};

JointConfig randomQ(std::mt19937& rng, double shrink = 0.8) {
    JointConfig q;
    std::uniform_real_distribution<double> unit(0.0, 1.0);
    for (int i = 0; i < 6; ++i) {
        const double mid = 0.5 * (kLower[i] + kUpper[i]);
        const double half = 0.5 * (kUpper[i] - kLower[i]) * shrink;
        q[i] = mid + (2.0 * unit(rng) - 1.0) * half;
    }
    return q;
}

Q12 randomQ12(std::mt19937& rng) {
    return Q12(DualArmKinematics::stack(randomQ(rng), randomQ(rng)));
}

Iso3 translated(const Iso3& T, const Eigen::Vector3d& d) {
    Iso3 out = T;
    out.translation() += d;
    return out;
}

}  // namespace

TEST(DualArmQp, RegulationConverges) {
    DualArmKinematics kin;
    DualArmQpParams params;
    DualArmQp qp(kin, params);

    std::mt19937 rng(4242u);
    Q12 q = randomQ12(rng);
    const Iso3 target = kin.leftTright(q);

    // Perturb so the initial relative error is non-zero but reachable.
    std::normal_distribution<double> gauss(0.0, 0.02);
    for (int i = 0; i < 12; ++i) q[i] += gauss(rng);

    double first_err = kin.relativeError(q, target).norm();
    bool all_converged = true;
    for (int k = 0; k < 400; ++k) {
        const auto r = qp.solve(q, target);
        all_converged = all_converged && r.converged;
        q += r.dq;
        // hard constraints must hold every step
        for (int i = 0; i < 12; ++i) {
            ASSERT_GE(q[i], params.q_min[i] - 1e-9);
            ASSERT_LE(q[i], params.q_max[i] + 1e-9);
        }
    }
    const double final_err = kin.relativeError(q, target).norm();
    std::printf("[QP] regulation: err %.3e -> %.3e (all_converged=%d)\n", first_err, final_err,
                static_cast<int>(all_converged));
    EXPECT_TRUE(all_converged);
    EXPECT_LT(final_err, 1e-5);
    EXPECT_LT(final_err, first_err);
}

TEST(DualArmQp, HardConstraintsRespected) {
    DualArmKinematics kin;
    DualArmQpParams params;
    params.v_max.setConstant(0.5);  // tight: dq_max = 0.004 rad/step
    DualArmQp qp(kin, params);

    std::mt19937 rng(7u);
    const Q12 q = randomQ12(rng);
    const Iso3 target = translated(kin.leftTright(q), Eigen::Vector3d(0.30, 0.0, 0.0));

    const auto r = qp.solve(q, target);
    ASSERT_TRUE(r.converged);
    const double dq_max = 0.5 * params.dt;
    for (int i = 0; i < 12; ++i) {
        EXPECT_LE(std::abs(r.dq[i]), dq_max + 1e-9) << "joint " << i;
    }
    const Q12 q_next = q + r.dq;
    for (int i = 0; i < 12; ++i) {
        EXPECT_GE(q_next[i], params.q_min[i] - 1e-9);
        EXPECT_LE(q_next[i], params.q_max[i] + 1e-9);
    }
    std::printf("[QP] hard constraints: |dq|max=%.4f (limit %.4f), err=%.3e, slack=%.3e\n",
                r.dq.cwiseAbs().maxCoeff(), dq_max, r.relative_error_norm, r.slack.norm());
}

TEST(DualArmQp, SlackActivatesForUnreachableTarget) {
    DualArmKinematics kin;
    DualArmQpParams params;
    params.w_slack = 1.0;  // soft enough that the slack visibly absorbs error
    DualArmQp qp(kin, params);

    std::mt19937 rng(11u);
    const Q12 q = randomQ12(rng);
    const Iso3 reachable = kin.leftTright(q);

    const auto r_ok = qp.solve(q, reachable);
    const auto r_far = qp.solve(q, translated(reachable, Eigen::Vector3d(1.0, 0.0, 0.0)));

    ASSERT_TRUE(r_ok.converged);
    ASSERT_TRUE(r_far.converged);
    std::printf("[QP] slack: reachable=%.3e, unreachable=%.3e\n", r_ok.slack.norm(),
                r_far.slack.norm());
    EXPECT_LT(r_ok.slack.norm(), 1e-3);
    EXPECT_GT(r_far.slack.norm(), 1e-2);
    EXPECT_GT(r_far.slack.norm(), 10.0 * r_ok.slack.norm());
}

TEST(DualArmQp, GeneralInequalityPreservedWhenClampIsSkipped) {
    // C2.R3: with a general inequality row present, the element-wise box clamp
    // must NOT be applied (it would break C*dq <= d).  This row has d < 0, so
    // the origin is infeasible and an origin-scaling projection would break it
    // too -- the solver result must be returned as-is.
    DualArmKinematics kin;
    DualArmQpParams params;
    params.C_ineq = Eigen::MatrixXd::Zero(1, 24);   // 24 vars since C2.4b
    params.C_ineq(0, 0) = 1.0;                      // dq_0 <= -0.01
    params.d_ineq = Eigen::VectorXd::Constant(1, -0.01);
    DualArmQp qp(kin, params);

    std::mt19937 rng(3u);
    const Q12 q = randomQ12(rng);
    const Iso3 target = kin.leftTright(q);
    const auto r = qp.solve(q, target);

    ASSERT_TRUE(r.converged);
    EXPECT_LE(r.dq[0], -0.01 + 1e-6) << "general inequality row violated by post-solve handling";
    // and the box is still respected (by the solver itself)
    EXPECT_GE(r.dq[0], -params.v_max[0] * params.dt - 1e-6);
    EXPECT_LE(r.dq[0], params.v_max[0] * params.dt + 1e-6);
    std::printf("[QP] general ineq: dq0=%.6f (C dq <= -0.01), converged=%d\n", r.dq[0],
                static_cast<int>(r.converged));
}

TEST(DualArmQp, SolverTimingReport) {
    DualArmKinematics kin;
    DualArmQpParams params;
    DualArmQp qp(kin, params);

    std::mt19937 rng(99u);
    Q12 q = randomQ12(rng);
    const Iso3 target = kin.leftTright(q);
    std::normal_distribution<double> gauss(0.0, 0.01);
    for (int i = 0; i < 12; ++i) q[i] += gauss(rng);

    constexpr int kN = 500;
    double total_ms = 0.0;
    int max_iters = 0;
    for (int k = 0; k < kN; ++k) {
        const auto r = qp.solve(q, target);
        total_ms += r.solve_time_ms;
        max_iters = std::max(max_iters, r.iterations);
        q += r.dq;
    }
    std::printf("[QP] timing: %.3f ms/solve avg, max iters=%d (backend=%s)\n",
                total_ms / kN, max_iters, qp.solve(q, target).solver_name.c_str());
    EXPECT_LT(total_ms / kN, 8.0);  // must fit the 125 Hz budget
}
