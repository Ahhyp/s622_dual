// C4.3 S3 -- W3 tests: anchored object task (S3-T2 / S3-T3 / S3-T8).
//
// These are the tests that make the S2 baseline's frozen numbers *self-checking*:
//   S3-T2  T_hat_{O,L}(q_grasp) reproduces world_T_object_star
//   S3-T3  the two independent anchors agree at the frozen grasp
//   S3-T8  the closed-chain identity of §2.5 (an independent judge, not a
//          re-derivation: it compares two quantities computed by different paths)
#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <array>
#include <cmath>
#include <cstdio>
#include <random>
#include <vector>

#include "dual_arm_kinematics/dual_arm_kinematics.hpp"
#include "dual_arm_kinematics/object_frame.hpp"
#include "dual_arm_kinematics/se3.hpp"
#include "dual_arm_transport_mock/object_task.hpp"

namespace {

using dual_arm::Arm;
using dual_arm::DualArmKinematics;
using dual_arm::Iso3;
using dual_arm::JointConfig;
using dual_arm::Mat6x12;
using dual_arm::Q12;
using dual_arm::Vec6;
using dual_arm_transport_mock::AnchoredObjectTask;
using dual_arm_transport_mock::AnchoredObjectTaskBuilder;
using dual_arm_transport_mock::anchorGap;

constexpr std::array<double, 12> kGraspQ = {
    2.795036909814,  -1.706950857914, 1.279963524342,  -1.143808993223,
    -1.570796326795, -0.346555743776, 0.346555743776,  -1.434641795676,
    -1.279963524342, -1.997783660367, 1.570796326795,  -2.795036909814};

Q12 graspQ() {
    Q12 q;
    for (int i = 0; i < 12; ++i) q[i] = kGraspQ[static_cast<std::size_t>(i)];
    return q;
}

/// S2 frozen grasp transforms G_L / G_R (transport_baseline.yaml):
///   R = [[0,1,0],[1,0,0],[0,0,-1]],  p = (+/-0.0497, 0, +0.025)
Iso3 frozenGrasp(double x_sign) {
    Iso3 G = Iso3::Identity();
    Eigen::Matrix3d R;
    R << 0, 1, 0, 1, 0, 0, 0, 0, -1;
    G.linear() = R;
    G.translation() = Eigen::Vector3d(x_sign * 0.0497, 0.0, 0.025);
    return G;
}

/// S2 frozen object pose: (I3, (0, 0, 0.275)).
Iso3 objectStar() {
    Iso3 T = Iso3::Identity();
    T.translation() = Eigen::Vector3d(0.0, 0.0, 0.275);
    return T;
}

AnchoredObjectTaskBuilder makeBuilder(const DualArmKinematics& kin) {
    return AnchoredObjectTaskBuilder(kin, frozenGrasp(+1.0), frozenGrasp(-1.0));
}

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

// ---------------------------------------------------------------------------
// S3-T2
// ---------------------------------------------------------------------------
TEST(ObjectTaskC43S3, T2_GraspTransformsReproduceTheFrozenObjectPose) {
    const DualArmKinematics kin;
    const AnchoredObjectTaskBuilder b = makeBuilder(kin);
    const Q12 q = graspQ();
    const Iso3 T_WO_star = objectStar();

    const Iso3 est = b.leftEstimate(q);
    const double pos = (est.translation() - T_WO_star.translation()).norm();
    const double rot =
        dual_arm::logSO3(T_WO_star.linear().transpose() * est.linear()).norm();

    EXPECT_LE(pos, 1e-12) << "G_L does not reproduce the frozen object position; "
                          << "||dp|| = " << pos << " m";
    EXPECT_LE(rot, 1e-12) << "G_L does not reproduce the frozen object rotation; "
                          << "||dR|| = " << rot << " rad";

    // The same must hold for the right arm, independently.
    const Iso3 est_r = b.rightEstimate(q);
    EXPECT_LE((est_r.translation() - T_WO_star.translation()).norm(), 1e-12);
    EXPECT_LE(dual_arm::logSO3(T_WO_star.linear().transpose() * est_r.linear()).norm(), 1e-12);
}

// ---------------------------------------------------------------------------
// S3-T3
// ---------------------------------------------------------------------------
TEST(ObjectTaskC43S3, T3_BothAnchorsAgreeAtTheFrozenGrasp) {
    const DualArmKinematics kin;
    const AnchoredObjectTaskBuilder b = makeBuilder(kin);
    const Q12 q = graspQ();

    const AnchoredObjectTask task = b.build(q, objectStar());
    EXPECT_LE(task.anchor_gap.pos_m, 1e-12);
    EXPECT_LE(task.anchor_gap.rot_rad, 1e-12);
    EXPECT_TRUE(task.anchor_ok);
    // e_O must be ~0 at the frozen grasp: the task starts exactly on target.
    EXPECT_LT(task.e.norm(), 1e-12);
    // J must be well conditioned on the left block (the right block is exactly 0
    // by construction: T_hat_{O,L} does not depend on q_right).
    EXPECT_TRUE(task.J.rightCols<6>().isZero(0.0)) << "right block must be EXACTLY zero";
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(task.J.leftCols<6>().eval());
    EXPECT_GT(svd.singularValues().minCoeff(), 1e-3);
}

// ---------------------------------------------------------------------------
// S3-T5e -- the anchored row against an independent oracle
// ---------------------------------------------------------------------------
TEST(ObjectTaskC43S3, T5e_AnchoredJacobianMatchesIndependentOracle) {
    const DualArmKinematics kin;
    const AnchoredObjectTaskBuilder builder = makeBuilder(kin);
    const Q12 q = graspQ();
    // A displaced desired pose: at the frozen grasp e_O is ~1e-13 and the row would
    // be tested at a point where every column is nearly zero.
    const Iso3 T_d = objectStar() * dual_arm::expSE3(Vec6(0.020, -0.010, 0.015, 0.0, 0.0, 0.05));
    const AnchoredObjectTask t = builder.build(q, T_d, Vec6::Zero());
    const Mat6x12 J = t.J;

    // (a) central difference, h = 1e-4: the FD error is O(h^2) plus O(eps/h) round-off,
    //     so 1e-6 relative is a real assertion about the analytic row.
    const double h = 1e-4;
    double worst_abs = 0.0;
    double worst_rel = 0.0;
    for (int j = 0; j < 6; ++j) {
        Q12 qp = q;
        Q12 qm = q;
        qp[j] += h;
        qm[j] -= h;
        const Iso3 T_hat_p = builder.leftEstimate(qp);
        const Iso3 T_hat_m = builder.leftEstimate(qm);
        const Vec6 fd = (dual_arm::decoupledPoseError(T_hat_p, T_d) -
                         dual_arm::decoupledPoseError(T_hat_m, T_d)) /
                        (2.0 * h);
        const double err = (fd - J.col(j)).norm();
        worst_abs = std::max(worst_abs, err);
        worst_rel = std::max(worst_rel, err / std::max(1e-3, fd.norm()));
    }
    std::printf("\n[ S3-T5e ] FD vs analytic: worst abs %.3e, worst rel %.3e\n", worst_abs,
                worst_rel);
    EXPECT_LE(worst_abs, 1e-6);
    EXPECT_LE(worst_rel, 1e-6);

    // (b) forward consistency for a SMALL step (the regime the QP actually uses):
    //     e_O(q + dq) - e_O(q) == J_O dq, to first order.
    Q12 dq = Q12::Zero();
    for (int i = 0; i < 6; ++i) dq[i] = 1e-5 * (i + 1);
    const Vec6 predicted = J.leftCols<6>() * dq.head<6>();
    const Vec6 actual = dual_arm::decoupledPoseError(builder.leftEstimate(q + dq), T_d) - t.e;
    std::printf("[ S3-T5e ] forward consistency: |predicted - actual| = %.3e (|predicted| %.3e)\n",
                (predicted - actual).norm(), predicted.norm());
    EXPECT_LE((predicted - actual).norm(), 1e-5);

    // (c) the right block is EXACTLY zero: T_hat_{O,L} is a function of q_left only,
    //     so this is an exact derivative, not a small quantity being rounded away.
    EXPECT_TRUE(J.rightCols<6>().isZero(0.0));
}

// ---------------------------------------------------------------------------
// S3-T8
// ---------------------------------------------------------------------------
TEST(ObjectTaskC43S3, T8_AnchorDisagreementEqualsTheClosureResidual) {
    const DualArmKinematics kin;
    const AnchoredObjectTaskBuilder b = makeBuilder(kin);
    const Iso3 G_L = frozenGrasp(+1.0);
    const Iso3 G_R = frozenGrasp(-1.0);
    const Iso3 K = G_L.inverse() * G_R;

    std::mt19937 rng(20241017u);
    int checked_matrix = 0;
    int checked_log = 0;
    int near_pi = 0;
    double worst_matrix = 0.0;
    double worst_log = 0.0;

    for (int trial = 0; trial < 100; ++trial) {
        const Q12 q = DualArmKinematics::stack(randomQ(rng, 0.9), randomQ(rng, 0.9));
        JointConfig ql, qr;
        DualArmKinematics::split(q, &ql, &qr);
        const Iso3 T_WL = kin.worldTgrasp(Arm::Left, ql);
        const Iso3 T_WR = kin.worldTgrasp(Arm::Right, qr);
        const Iso3 C = T_WL.inverse() * T_WR;

        const AnchoredObjectTask task = b.build(q, objectStar());
        ASSERT_TRUE(task.T_hat_left.matrix().allFinite());

        // ---- the identity, in the matrix domain (exact algebra) ------------
        // T_hat_L^-1 T_hat_R  ==  G_L (C K^-1) G_L^-1
        const Iso3 lhs = task.T_hat_left.inverse() * task.T_hat_right;
        const Iso3 rhs = G_L * (C * K.inverse()) * G_L.inverse();
        const double em = (lhs.matrix() - rhs.matrix()).cwiseAbs().maxCoeff();
        worst_matrix = std::max(worst_matrix, em);
        EXPECT_LE(em, 1e-12) << "trial " << trial << ": matrix identity broken by " << em;
        ++checked_matrix;

        // ---- the identity, in the log domain --------------------------------
        // logSE3(T_hat_L^-1 T_hat_R) == Ad_{G_L} logSE3(C K^-1).
        // Only asserted away from the pi branch: logSO3 itself loses ~5e-4 rad at
        // pi (F-S1-1), so a 1e-12 claim there would be testing the branch, not S3.
        const double margin = dual_arm::midpointMargin(T_WL, T_WR);
        const Vec6 el = dual_arm::logSE3(lhs) - dual_arm::adjoint(G_L) * dual_arm::logSE3(C * K.inverse());
        if (margin > 0.1) {
            worst_log = std::max(worst_log, el.cwiseAbs().maxCoeff());
            EXPECT_LE(el.cwiseAbs().maxCoeff(), 1e-12)
                << "trial " << trial << ": log identity broken by " << el.cwiseAbs().maxCoeff();
            ++checked_log;
        } else {
            ++near_pi;
            // Near pi we only require the matrix identity (asserted above) plus a
            // bounded log discrepancy -- never a wrong sign or a wrong magnitude.
            EXPECT_LT(el.norm(), 1.0) << "trial " << trial << ": log identity blew up";
        }
    }

    EXPECT_EQ(checked_matrix, 100);
    EXPECT_GT(checked_log, 50) << "too few trials away from the pi branch to be meaningful";

    // ---- T8b: the same identity driven INTO the pi branch --------------------
    // Random joint samples never land near margin ~ 0, so the pi branch has to be
    // constructed directly: keep T_WL, and sweep the relative rotation of T_WR
    // over [0.9 pi, 0.999 pi].  The matrix identity must still hold to 1e-12 --
    // that is the whole point of using the anchored form (no rotation averaging,
    // so no pi singularity).  The log form is only bounded, because logSO3 itself
    // is inaccurate there (F-S1-1).
    double worst_pi_matrix = 0.0;
    double worst_pi_log = 0.0;
    int pi_cases = 0;
    for (int k = 0; k < 40; ++k) {
        const double frac = 0.9 + 0.099 * (static_cast<double>(k) / 39.0);
        Eigen::AngleAxisd aa(frac * M_PI,
                             Eigen::Vector3d(0.3, -0.7, 0.5).normalized());
        Iso3 T_WL = Iso3::Identity();
        T_WL.linear() = Eigen::AngleAxisd(0.4, Eigen::Vector3d(1, 0, 0).normalized()).toRotationMatrix();
        T_WL.translation() = Eigen::Vector3d(0.3, -0.1, 0.4);
        Iso3 X = Iso3::Identity();
        X.linear() = aa.toRotationMatrix();
        X.translation() = Eigen::Vector3d(0.01, 0.02, -0.03);
        const Iso3 T_WR = T_WL * X;

        const Iso3 hat_L = dual_arm::anchoredObjectPose(T_WL, G_L);
        const Iso3 hat_R = dual_arm::anchoredObjectPose(T_WR, G_R);
        const Iso3 C = T_WL.inverse() * T_WR;
        const Iso3 lhs = hat_L.inverse() * hat_R;
        const Iso3 rhs = G_L * (C * K.inverse()) * G_L.inverse();
        const double em = (lhs.matrix() - rhs.matrix()).cwiseAbs().maxCoeff();
        worst_pi_matrix = std::max(worst_pi_matrix, em);
        EXPECT_LE(em, 1e-12) << "pi branch, frac=" << frac << ": matrix identity broken by " << em;

        const Vec6 el =
            dual_arm::logSE3(lhs) - dual_arm::adjoint(G_L) * dual_arm::logSE3(C * K.inverse());
        worst_pi_log = std::max(worst_pi_log, el.cwiseAbs().maxCoeff());
        EXPECT_LT(el.cwiseAbs().maxCoeff(), 1.0)
            << "pi branch, frac=" << frac << ": log identity blew up to " << el.cwiseAbs().maxCoeff();
        ++pi_cases;
    }
    EXPECT_EQ(pi_cases, 40);

    std::printf("[T8a] random-q: matrix worst=%.3e (%d trials), log worst=%.3e (%d trials), "
                "near-pi skipped=%d\n",
                worst_matrix, checked_matrix, worst_log, checked_log, near_pi);
    std::printf("[T8b] near-pi:   matrix worst=%.3e (%d cases), log worst=%.3e\n",
                worst_pi_matrix, pi_cases, worst_pi_log);
    // The near-pi log discrepancy must be visibly worse than the well-conditioned
    // one -- otherwise T8b is not actually entering the pi branch.
    EXPECT_GT(worst_pi_log, worst_log)
        << "T8b did not reach the pi branch (log discrepancy " << worst_pi_log
        << " vs " << worst_log << ")";
}

}  // namespace
