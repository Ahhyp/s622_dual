// test/test_object_frame.cpp
// C2.4a: coordination / object frame (SE(3) midpoint) and its Jacobian.
//
// The object frame is the SE(3) midpoint of the two grasp frames:
//   p_O = (p_L + p_R) / 2
//   R_O = R_L * exp( 0.5 * log(R_L^T R_R) )
// Its exact Jacobian (world axes, object-origin referenced) is
//   v_O = 0.5 (v_L + v_R)
//   w_O = (I - G) w_L + G w_R,
//   G   = R_L * J_l(xi/2) * 0.5 * J_l(xi)^-1 * R_L^T,  xi = log(R_L^T R_R).

#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <cmath>
#include <cstdio>
#include <random>

#include "dual_arm_kinematics/dual_arm_kinematics.hpp"
#include "dual_arm_kinematics/se3.hpp"

namespace {

using dual_arm::Arm;
using dual_arm::DualArmKinematics;
using dual_arm::Iso3;
using dual_arm::JointConfig;
using dual_arm::Q12;

constexpr double kH = 1e-6;
const double kLower[6] = {-3.0543, -4.6251, -2.8274, -4.6251, -3.0543, -3.0543};
const double kUpper[6] = {3.0543, 1.4835, 2.8274, 1.4835, 3.0543, 3.0543};

JointConfig randomQ(std::mt19937& rng) {
    JointConfig q;
    std::uniform_real_distribution<double> unit(0.0, 1.0);
    for (int i = 0; i < 6; ++i) q[i] = kLower[i] + unit(rng) * (kUpper[i] - kLower[i]);
    return q;
}

Q12 randomQ12(std::mt19937& rng) {
    return DualArmKinematics::stack(randomQ(rng), randomQ(rng));
}

Q12 randomDirection(std::mt19937& rng) {
    Q12 d;
    std::normal_distribution<double> gauss(0.0, 1.0);
    for (int i = 0; i < 12; ++i) d[i] = gauss(rng);
    return d / d.norm();
}

}  // namespace

TEST(ObjectFrame, PoseIsTheSE3Midpoint) {
    DualArmKinematics kin;
    std::mt19937 rng(20261009u);
    double max_pos = 0.0, max_rot = 0.0;
    for (int n = 0; n < 200; ++n) {
        const Q12 q = randomQ12(rng);
        JointConfig ql, qr;
        DualArmKinematics::split(q, &ql, &qr);
        const Iso3 WL = kin.worldTgrasp(Arm::Left, ql);
        const Iso3 WR = kin.worldTgrasp(Arm::Right, qr);
        const Iso3 WO = kin.worldTobject(q);

        max_pos = std::max(max_pos,
                           (WO.translation() - 0.5 * (WL.translation() + WR.translation())).norm());
        // R_O must be the geodesic midpoint: log(R_L^T R_O) = 0.5 log(R_L^T R_R)
        const Eigen::Vector3d lhs = dual_arm::logSO3(WL.linear().transpose() * WO.linear());
        const Eigen::Vector3d rhs =
            0.5 * dual_arm::logSO3(WL.linear().transpose() * WR.linear());
        max_rot = std::max(max_rot, (lhs - rhs).norm());
    }
    std::printf("[C2.4a] midpoint pose: pos err = %.3e m, rot-midpoint err = %.3e rad\n", max_pos,
                max_rot);
    EXPECT_LT(max_pos, 1e-12);
    EXPECT_LT(max_rot, 1e-9);
}

TEST(ObjectFrame, ObjectJacobianBodyMatchesFiniteDifference) {
    DualArmKinematics kin;
    std::mt19937 rng(20261010u);
    double max_err = 0.0;
    for (int n = 0; n < 100; ++n) {
        const Q12 q = randomQ12(rng);
        const Q12 dq = randomDirection(rng);
        const Iso3 T0 = kin.worldTobject(q);
        auto bodyTwist = [&](const Q12& qq) -> dual_arm::Vec6 {
            return dual_arm::logSE3(T0.inverse() * kin.worldTobject(qq));
        };
        const dual_arm::Vec6 num = (bodyTwist(q + kH * dq) - bodyTwist(q - kH * dq)) / (2.0 * kH);
        const dual_arm::Vec6 ana = kin.objectJacobianBody(q) * dq;
        max_err = std::max(max_err, (num - ana).cwiseAbs().maxCoeff());
    }
    std::printf("[C2.4a] J_object(body) vs FD: max abs diff = %.3e\n", max_err);
    EXPECT_LT(max_err, 1e-5);
}

TEST(ObjectFrame, ObjectJacobianWorldLinearAndAngularMatchFiniteDifference) {
    DualArmKinematics kin;
    std::mt19937 rng(20261011u);
    double max_lin = 0.0, max_ang = 0.0;
    for (int n = 0; n < 100; ++n) {
        const Q12 q = randomQ12(rng);
        const Q12 dq = randomDirection(rng);
        const Iso3 T0 = kin.worldTobject(q);
        const Iso3 Tp = kin.worldTobject(q + kH * dq);
        const Iso3 Tm = kin.worldTobject(q - kH * dq);

        const Eigen::Vector3d v_fd = (Tp.translation() - Tm.translation()) / (2.0 * kH);
        const Eigen::Matrix3d dR = (Tp.linear() - Tm.linear()) / (2.0 * kH);
        const Eigen::Matrix3d W = dR * T0.linear().transpose();
        const Eigen::Vector3d w_fd(W(2, 1), W(0, 2), W(1, 0));

        const Eigen::Matrix<double, 6, 12> J = kin.objectJacobianWorld(q);
        max_lin = std::max(max_lin, (J.topRows<3>() * dq - v_fd).cwiseAbs().maxCoeff());
        max_ang = std::max(max_ang, (J.bottomRows<3>() * dq - w_fd).cwiseAbs().maxCoeff());
    }
    std::printf("[C2.4a] J_object(world) vs FD: linear %.3e, angular %.3e\n", max_lin, max_ang);
    EXPECT_LT(max_lin, 1e-5);
    EXPECT_LT(max_ang, 1e-5);
}

TEST(ObjectFrame, ErrorJacobianMatchesFiniteDifference) {
    DualArmKinematics kin;
    std::mt19937 rng(20261012u);
    double max_err = 0.0;
    for (int n = 0; n < 50; ++n) {
        const Q12 q0 = randomQ12(rng);
        const Q12 dq = randomDirection(rng);
        const Iso3 target = kin.worldTobject(q0);  // e = 0
        auto errAt = [&](const Q12& qq) { return kin.objectError(qq, target); };
        const dual_arm::Vec6 num = (errAt(q0 + kH * dq) - errAt(q0 - kH * dq)) / (2.0 * kH);
        const dual_arm::Vec6 ana = kin.objectErrorJacobian(q0, target) * dq;
        max_err = std::max(max_err, (num - ana).cwiseAbs().maxCoeff());
    }
    std::printf("[C2.4a] object error-rate Jacobian vs FD: max abs diff = %.3e\n", max_err);
    EXPECT_LT(max_err, 1e-5);
}

TEST(ObjectFrame, ExactAngularVsNaiveAverage) {
    // Quantify why the exact G was derived instead of using 0.5(w_L + w_R).
    DualArmKinematics kin;
    std::mt19937 rng(20261013u);
    double max_diff = 0.0, max_rel_rot = 0.0;
    for (int n = 0; n < 200; ++n) {
        const Q12 q = randomQ12(rng);
        JointConfig ql, qr;
        DualArmKinematics::split(q, &ql, &qr);
        const Eigen::Matrix<double, 6, 12> exact = kin.objectJacobianWorld(q);
        const Eigen::Matrix<double, 6, 6> JL = kin.jacobianWorldPoint(Arm::Left, ql);
        const Eigen::Matrix<double, 6, 6> JR = kin.jacobianWorldPoint(Arm::Right, qr);
        Eigen::Matrix<double, 6, 12> naive = Eigen::Matrix<double, 6, 12>::Zero();
        naive.topRows<3>().leftCols<6>() = 0.5 * JL.topRows<3>();
        naive.topRows<3>().rightCols<6>() = 0.5 * JR.topRows<3>();
        naive.bottomRows<3>().leftCols<6>() = 0.5 * JL.bottomRows<3>();
        naive.bottomRows<3>().rightCols<6>() = 0.5 * JR.bottomRows<3>();
        max_diff = std::max(max_diff, (exact - naive).cwiseAbs().maxCoeff());
        const Eigen::Vector3d xi = dual_arm::logSO3(
            kin.worldTgrasp(Arm::Left, ql).linear().transpose() *
            kin.worldTgrasp(Arm::Right, qr).linear());
        max_rel_rot = std::max(max_rel_rot, xi.norm());
    }
    std::printf("[C2.4a] exact vs naive-average angular Jacobian: max diff = %.3e "
                "(max |xi| = %.3f rad)\n",
                max_diff, max_rel_rot);
    EXPECT_GT(max_diff, 1e-6);  // the naive average really is different
}
