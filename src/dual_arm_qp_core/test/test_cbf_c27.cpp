// test/test_cbf_c27.cpp  (C2.7b)
// Discrete CBF on the arm-arm distance, injected through DualArmQp::C_ineq.
//
//   h(q) = d(q) - d_safe >= 0
//   hdot + alpha h >= 0,   hdot = grad_d . qdot = grad_d . dq / dt
//   =>  -grad_d . dq  <=  alpha * h * dt          (the row we add)

#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <cmath>
#include <cstdio>
#include <random>

#include "dual_arm_collision/dual_arm_collision.hpp"
#include "dual_arm_kinematics/dual_arm_kinematics.hpp"
#include "dual_arm_qp_core/dual_arm_qp.hpp"

namespace {

using dual_arm::DualArmKinematics;
using dual_arm::Iso3;
using dual_arm::JointConfig;
using dual_arm::Q12;
using dual_arm_collision::DualArmCollisionModel;
using dual_arm_qp::DualArmQp;
using dual_arm_qp::DualArmQpParams;

struct RunResult {
    double min_distance = 1e9;
    double final_distance = 0.0;
    double rel_err = 0.0;
    int below_safe_steps = 0;
    int saturated_steps = 0;
    int failures = 0;
};

}  // namespace

TEST(C27Cbf, CbfKeepsMinimumArmDistance) {
    DualArmKinematics kin;
    DualArmCollisionModel model(kin, 0.06);

    // Start from extended arms where the initial distance is comfortable.
    JointConfig stretched;
    stretched << 0.0, -1.5708, 0.0, 0.0, 0.0, 0.0;
    const Q12 q0 = DualArmKinematics::stack(stretched, stretched);
    const Iso3 rel0 = kin.leftTright(q0);
    const double d0 = model.distance(q0).distance;

    // Pull the two grasps towards each other: shrink the relative translation.
    Iso3 target = rel0;
    target.translation() *= 0.25;

    const double dt = 0.008;
    const double d_safe = 0.10;
    const double alpha_cbf = 3.0;

    auto run = [&](bool cbf, RunResult* out) {
        DualArmQpParams p;
        p.dt = dt;
        p.kp_rel = 20.0;
        p.w_rel = 1.0;
        p.w_obj = 0.0;
        p.w_slack = 1e3;
        p.w_center = 1e-2;
        p.w_reg = 1e-6;
        DualArmQp qp(kin, p);
        Q12 q = q0, q_cmd = q0;
        const double tau = 0.05;
        const int steps = static_cast<int>(4.0 / dt);
        for (int k = 0; k < steps; ++k) {
            if (cbf) {
                const auto info = model.distance(q);
                const dual_arm_collision::Vec12 g = model.distanceGradient(q);
                DualArmQpParams pc = p;
                pc.C_ineq = Eigen::MatrixXd::Zero(1, 24);
                for (int i = 0; i < 12; ++i) pc.C_ineq(0, i) = -g[i];
                const double h = info.distance - d_safe;
                pc.d_ineq = Eigen::VectorXd::Constant(1, alpha_cbf * h * dt);
                qp.setParams(pc);
            }
            const auto r = qp.solve(q, target);
            if (!r.converged) ++out->failures; else q_cmd += r.dq;
            q += (dt / tau) * (q_cmd - q);

            const double d = model.distance(q).distance;
            out->min_distance = std::min(out->min_distance, d);
            out->final_distance = d;
            if (d < d_safe) ++out->below_safe_steps;
            if (r.dq.cwiseAbs().maxCoeff() > 0.999 * 0.02) ++out->saturated_steps;
        }
        out->rel_err = kin.relativeError(q, target).norm();
    };

    RunResult off, on;
    run(false, &off);
    run(true, &on);

    std::printf("[C2.7b] start distance = %.4f m, d_safe = %.3f m\n", d0, d_safe);
    std::printf("[C2.7b] CBF OFF: minDist=%.4f finalDist=%.4f belowSafe=%d/%d relErr=%.4f\n",
                off.min_distance, off.final_distance, off.below_safe_steps, 500, off.rel_err);
    std::printf("[C2.7b] CBF ON : minDist=%.4f finalDist=%.4f belowSafe=%d/%d relErr=%.4f\n",
                on.min_distance, on.final_distance, on.below_safe_steps, 500, on.rel_err);

    EXPECT_EQ(off.failures, 0);
    EXPECT_EQ(on.failures, 0);
    EXPECT_GT(d0, d_safe);
    // the unconstrained task really does violate the safe distance
    EXPECT_LT(off.min_distance, d_safe);
    // CBF keeps the arms apart
    EXPECT_GT(on.min_distance, off.min_distance);
    EXPECT_GT(on.min_distance, 0.95 * d_safe);
}
