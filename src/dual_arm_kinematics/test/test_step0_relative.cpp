// test/test_step0_relative.cpp
// Step 0 / S0.3, S0.4, S0.5:
//   - relative Jacobian (body / left / world-spatial) validated by finite
//     differences of the relative pose
//   - invariance under a common rigid motion of both bases
//   - "same rigid twist on both arms -> zero relative twist" (adjoint check)
//   - error-rate Jacobian consistency
//   - rough timing

#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <chrono>
#include <cstdio>
#include <random>

#include "dual_arm_kinematics/dual_arm_kinematics.hpp"

namespace {

constexpr double kH = 1e-6;
const double kLower[6] = {-3.0543, -4.6251, -2.8274, -4.6251, -3.0543, -3.0543};
const double kUpper[6] = {3.0543, 1.4835, 2.8274, 1.4835, 3.0543, 3.0543};

dual_arm::JointConfig randomQ(std::mt19937& rng) {
    dual_arm::JointConfig q;
    std::uniform_real_distribution<double> unit(0.0, 1.0);
    for (int i = 0; i < 6; ++i) {
        q[i] = kLower[i] + unit(rng) * (kUpper[i] - kLower[i]);
    }
    return q;
}

dual_arm::Q12 randomQ12(std::mt19937& rng) {
    return dual_arm::DualArmKinematics::stack(randomQ(rng), randomQ(rng));
}

dual_arm::Q12 randomDirection(std::mt19937& rng) {
    dual_arm::Q12 d;
    std::normal_distribution<double> gauss(0.0, 1.0);
    for (int i = 0; i < 12; ++i) d[i] = gauss(rng);
    return d / d.norm();
}

dual_arm::Vec6 randomTwist(std::mt19937& rng, double scale) {
    dual_arm::Vec6 xi;
    std::normal_distribution<double> gauss(0.0, 1.0);
    for (int i = 0; i < 6; ++i) xi[i] = gauss(rng);
    return xi / xi.norm() * scale;
}

}  // namespace

TEST(Step0Relative, BodyRelativeJacobianMatchesFiniteDifference) {
    dual_arm::DualArmKinematics kin;
    std::mt19937 rng(20261001u);
    double max_err = 0.0;
    for (int n = 0; n < 100; ++n) {
        const dual_arm::Q12 q = randomQ12(rng);
        const dual_arm::Q12 dq = randomDirection(rng);
        const dual_arm::Iso3 T0 = kin.leftTright(q);

        auto bodyTwist = [&](const dual_arm::Q12& qq) -> dual_arm::Vec6 {
            return dual_arm::logSE3(T0.inverse() * kin.leftTright(qq));
        };
        const dual_arm::Vec6 num = (bodyTwist(q + kH * dq) - bodyTwist(q - kH * dq)) / (2.0 * kH);
        const dual_arm::Vec6 ana =
            kin.relativeJacobian(q, dual_arm::RelativeFrame::BodyR) * dq;
        max_err = std::max(max_err, (num - ana).cwiseAbs().maxCoeff());
    }
    std::printf("[S0.3] J_rel(body) vs FD: max abs diff = %.3e\n", max_err);
    EXPECT_LT(max_err, 1e-5);
}

TEST(Step0Relative, LeftAndWorldFramesMatchFiniteDifference) {
    dual_arm::DualArmKinematics kin;
    std::mt19937 rng(20261002u);
    double max_left = 0.0;
    double max_world = 0.0;
    for (int n = 0; n < 100; ++n) {
        const dual_arm::Q12 q = randomQ12(rng);
        const dual_arm::Q12 dq = randomDirection(rng);
        const dual_arm::Iso3 T0 = kin.leftTright(q);
        dual_arm::JointConfig ql, qr;
        dual_arm::DualArmKinematics::split(q, &ql, &qr);

        auto bodyTwist = [&](const dual_arm::Q12& qq) -> dual_arm::Vec6 {
            return dual_arm::logSE3(T0.inverse() * kin.leftTright(qq));
        };
        const dual_arm::Vec6 num_body =
            (bodyTwist(q + kH * dq) - bodyTwist(q - kH * dq)) / (2.0 * kH);

        const dual_arm::Vec6 num_left = dual_arm::adjoint(T0) * num_body;
        const dual_arm::Vec6 ana_left =
            kin.relativeJacobian(q, dual_arm::RelativeFrame::LeftL) * dq;
        max_left = std::max(max_left, (num_left - ana_left).cwiseAbs().maxCoeff());

        const dual_arm::Vec6 num_world =
            dual_arm::adjoint(kin.worldTgrasp(dual_arm::Arm::Right, qr)) * num_body;
        const dual_arm::Vec6 ana_world =
            kin.relativeJacobian(q, dual_arm::RelativeFrame::WorldSpatial) * dq;
        max_world = std::max(max_world, (num_world - ana_world).cwiseAbs().maxCoeff());
    }
    std::printf("[S0.3] J_rel(left) vs FD = %.3e, J_rel(world) vs FD = %.3e\n",
                max_left, max_world);
    EXPECT_LT(max_left, 1e-5);
    EXPECT_LT(max_world, 1e-5);
}

TEST(Step0Relative, CommonRigidMotionInvariance) {
    dual_arm::DualArmKinematics kin;
    std::mt19937 rng(20261003u);
    double max_pose = 0.0;
    double max_jac = 0.0;
    for (int n = 0; n < 50; ++n) {
        const dual_arm::Q12 q = randomQ12(rng);
        const dual_arm::Iso3 G = dual_arm::expSE3(randomTwist(rng, 0.5));
        dual_arm::DualArmKinematics moved(G * kin.worldTbase(dual_arm::Arm::Left),
                                          G * kin.worldTbase(dual_arm::Arm::Right));

        const dual_arm::Iso3 T1 = kin.leftTright(q);
        const dual_arm::Iso3 T2 = moved.leftTright(q);
        max_pose = std::max(max_pose, (T1.matrix() - T2.matrix()).cwiseAbs().maxCoeff());

        const dual_arm::Mat6x12 J1 =
            kin.relativeJacobian(q, dual_arm::RelativeFrame::BodyR);
        const dual_arm::Mat6x12 J2 =
            moved.relativeJacobian(q, dual_arm::RelativeFrame::BodyR);
        max_jac = std::max(max_jac, (J1 - J2).cwiseAbs().maxCoeff());
    }
    std::printf("[S0.4] common-motion invariance: pose diff = %.3e, J diff = %.3e\n",
                max_pose, max_jac);
    EXPECT_LT(max_pose, 1e-10);
    EXPECT_LT(max_jac, 1e-10);
}

TEST(Step0Relative, SameBodyTwistCancelsInRelativeJacobian) {
    // Build dq = [dqL; dqR] such that both grasp frames get the *same* body
    // twist. The relative twist (and therefore J_rel * dq) must vanish.
    dual_arm::DualArmKinematics kin;
    std::mt19937 rng(20261004u);
    double max_res = 0.0;
    for (int n = 0; n < 50; ++n) {
        const dual_arm::JointConfig ql = randomQ(rng);
        const dual_arm::JointConfig qr = randomQ(rng);
        const dual_arm::Q12 q = dual_arm::DualArmKinematics::stack(ql, qr);

        const dual_arm::Mat6 JbL = kin.jacobianBody(dual_arm::Arm::Left, ql);
        const dual_arm::Mat6 JbR = kin.jacobianBody(dual_arm::Arm::Right, qr);
        const dual_arm::Iso3 T_rel = kin.leftTright(ql, qr);

        dual_arm::JointConfig dql;
        std::normal_distribution<double> gauss(0.0, 1.0);
        for (int i = 0; i < 6; ++i) dql[i] = gauss(rng);
        const dual_arm::Vec6 desired =
            dual_arm::adjoint(T_rel.inverse()) * (JbL * dql);
        const dual_arm::JointConfig dqr = JbR.fullPivLu().solve(desired);

        dual_arm::Q12 dq;
        dq.head<6>() = dql;
        dq.tail<6>() = dqr;
        const dual_arm::Vec6 res =
            kin.relativeJacobian(q, dual_arm::RelativeFrame::BodyR) * dq;
        max_res = std::max(max_res, res.norm());
    }
    std::printf("[S0.4] same-body-twist residual = %.3e\n", max_res);
    EXPECT_LT(max_res, 1e-10);
}

TEST(Step0Relative, ErrorJacobianMatchesFiniteDifference) {
    dual_arm::DualArmKinematics kin;
    std::mt19937 rng(20261005u);
    double max_err = 0.0;
    double max_err_offset = 0.0;
    for (int n = 0; n < 50; ++n) {
        const dual_arm::Q12 q0 = randomQ12(rng);
        const dual_arm::Q12 dq = randomDirection(rng);

        // target == current => e = 0 (tight check of the Jacobian)
        const dual_arm::Iso3 target = kin.leftTright(q0);
        auto errAt = [&](const dual_arm::Q12& qq) {
            return kin.relativeError(qq, target);
        };
        const dual_arm::Vec6 num =
            (errAt(q0 + kH * dq) - errAt(q0 - kH * dq)) / (2.0 * kH);
        const dual_arm::Vec6 ana = kin.relativeErrorJacobian(q0, target) * dq;
        max_err = std::max(max_err, (num - ana).cwiseAbs().maxCoeff());

        // small non-zero error: validates the Ad(T_err^-1) factor; the
        // remaining J_r^{SE3} ~= I approximation is O(|e|).
        const dual_arm::Iso3 target_off = kin.leftTright(q0) * dual_arm::expSE3(randomTwist(rng, 1e-4));
        auto errAt2 = [&](const dual_arm::Q12& qq) {
            return kin.relativeError(qq, target_off);
        };
        const dual_arm::Vec6 num2 =
            (errAt2(q0 + kH * dq) - errAt2(q0 - kH * dq)) / (2.0 * kH);
        const dual_arm::Vec6 ana2 = kin.relativeErrorJacobian(q0, target_off) * dq;
        max_err_offset = std::max(max_err_offset, (num2 - ana2).cwiseAbs().maxCoeff());
    }
    std::printf("[S0.4] error-rate Jacobian: e=0 diff = %.3e, small-offset diff = %.3e\n",
                max_err, max_err_offset);
    EXPECT_LT(max_err, 1e-5);
    EXPECT_LT(max_err_offset, 1e-3);
}

TEST(Step0Performance, RelativeTimingReport) {
    dual_arm::DualArmKinematics kin;
    std::mt19937 rng(2u);
    const dual_arm::Q12 q = randomQ12(rng);
    constexpr int kN = 10000;

    auto t0 = std::chrono::steady_clock::now();
    double sink = 0.0;
    for (int i = 0; i < kN; ++i) {
        sink += kin.relativeJacobian(q, dual_arm::RelativeFrame::BodyR).norm();
    }
    auto t1 = std::chrono::steady_clock::now();
    for (int i = 0; i < kN; ++i) {
        sink += kin.leftTright(q).matrix().norm();
    }
    auto t2 = std::chrono::steady_clock::now();
    const double us_j = std::chrono::duration<double, std::micro>(t1 - t0).count() / kN;
    const double us_rel = std::chrono::duration<double, std::micro>(t2 - t1).count() / kN;
    std::printf("[S0.5] J_rel(6x12) = %.2f us/call, left_T_right = %.2f us/call (sink=%.3f)\n",
                us_j, us_rel, sink);
    EXPECT_LT(us_j, 1000.0);
    EXPECT_LT(us_rel, 1000.0);
}
