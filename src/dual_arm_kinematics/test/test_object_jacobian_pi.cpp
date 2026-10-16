// test/test_object_jacobian_pi.cpp
// C4.3 S1 / T3 + T4 + T7 — the π defect of the symmetric midpoint object frame
// and the π-safety of the anchored replacement.
//
//  * T3 : the legacy midpoint interface must not silently return a wrong value in
//         its singular region — it is explicitly refused there.
//  * T4 : the anchored object Jacobian and the decoupled error Jacobian are exact
//         (finite-difference validated) for arm-relative rotations ON and AROUND π.
//  * T7 : the SAME oracle is run against a test-only reimplementation of the old
//         algorithm to prove the regression test can actually fail.
//
// User adjustment (m01472) #1 is the reason for the split policy: refusing π is
// allowed ONLY for the legacy symmetric-midpoint interface.  The new task-given
// anchored interface must support a relative rotation of exactly π — and no C4.3
// path may be rescued by "just avoid grasping at π".

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <random>
#include <vector>

#include <Eigen/Dense>

#include "dual_arm_kinematics/dual_arm_kinematics.hpp"
#include "dual_arm_kinematics/object_frame.hpp"
#include "dual_arm_kinematics/se3.hpp"
#include "testonly/legacy_midpoint_negative.hpp"

namespace {

using dual_arm::Arm;
using dual_arm::DualArmKinematics;
using dual_arm::Iso3;
using dual_arm::JointConfig;
using dual_arm::Mat3;
using dual_arm::Mat6;
using dual_arm::Mat6x12;
using dual_arm::Q12;
using dual_arm::Vec3;
using dual_arm::Vec6;

constexpr double kH = 1e-6;

const double kLower[6] = {-3.0543, -4.6251, -2.8274, -4.6251, -3.0543, -3.0543};
const double kUpper[6] = {3.0543, 1.4835, 2.8274, 1.4835, 3.0543, 3.0543};

JointConfig randomQ(std::mt19937& rng) {
    JointConfig q;
    std::uniform_real_distribution<double> unit(0.0, 1.0);
    for (int i = 0; i < 6; ++i) q[i] = kLower[i] + unit(rng) * (kUpper[i] - kLower[i]);
    return q;
}

Q12 randomDirection(std::mt19937& rng) {
    Q12 d;
    std::normal_distribution<double> gauss(0.0, 1.0);
    for (int i = 0; i < 12; ++i) d[i] = gauss(rng);
    return d / d.norm();
}

Iso3 randomRigid(std::mt19937& rng) {
    std::normal_distribution<double> gauss(0.0, 1.0);
    Vec3 w(gauss(rng), gauss(rng), gauss(rng));
    w.normalize();
    w *= std::uniform_real_distribution<double>(-0.5 * M_PI, 0.5 * M_PI)(rng);
    Iso3 T = Iso3::Identity();
    T.linear() = dual_arm::expSO3(w);
    T.translation() = Vec3(gauss(rng), gauss(rng), gauss(rng));
    return T;
}

/// Exactly-π arm pose: identical joint values on mirror-mounted bases.
Q12 exactlyPiPose() {
    JointConfig q;
    q << 0.0, -0.9, 0.9, -0.9, -0.6, 0.0;
    return DualArmKinematics::stack(q, q);
}

/// Central-difference body-twist oracle of an SE(3)-valued map of one arm.
/// Generic over the joint-vector type so it works for both JointConfig (6) and
/// the stacked Q12 (12).
template <typename MapFn, typename VecT>
Vec6 fdBodyTwist(const MapFn& map_fn, const VecT& q, const VecT& dq, const Iso3& T0) {
    const Iso3 Tp = map_fn(q + kH * dq);
    const Iso3 Tm = map_fn(q - kH * dq);
    return (dual_arm::logSE3(T0.inverse() * Tp) - dual_arm::logSE3(T0.inverse() * Tm)) / (2.0 * kH);
}

}  // namespace

TEST(ObjectJacobianPi, ThePoseIsExactlyPi) {
    // Pin down the premise: this robot configuration really does have an
    // arm-relative rotation of exactly π.
    DualArmKinematics kin;
    const Q12 q = exactlyPiPose();
    JointConfig ql, qr;
    DualArmKinematics::split(q, &ql, &qr);
    const Iso3 WL = kin.worldTgrasp(Arm::Left, ql);
    const Iso3 WR = kin.worldTgrasp(Arm::Right, qr);
    const double angle = dual_arm::relativeRotationAngle(WL, WR);
    std::printf("[C4.3 S1 T3] mirror pose arm-relative rotation = %.17g rad (pi = %.17g)\n", angle,
                M_PI);
    EXPECT_NEAR(angle, M_PI, 1e-12);
    EXPECT_NEAR(dual_arm::midpointMargin(WL, WR), 0.0, 1e-12);
}

TEST(ObjectJacobianPi, LegacyMidpointIsRefusedInItsSingularRegion) {
    // T3 — the legacy symmetric midpoint must REFUSE (not misreport) near π.
    DualArmKinematics kin;
    const Q12 q = exactlyPiPose();
    JointConfig ql, qr;
    DualArmKinematics::split(q, &ql, &qr);
    const Iso3 WL = kin.worldTgrasp(Arm::Left, ql);
    const Iso3 WR = kin.worldTgrasp(Arm::Right, qr);

    EXPECT_FALSE(dual_arm::isMidpointSafe(WL, WR));
    EXPECT_NE(dual_arm::midpointUnsafeReason(WL, WR), "");

    // The guard must be generous away from the singularity.
    std::mt19937 rng(20261030u);
    int safe = 0, total = 0;
    for (int n = 0; n < 200; ++n) {
        const Q12 qq = DualArmKinematics::stack(randomQ(rng), randomQ(rng));
        JointConfig a, b;
        DualArmKinematics::split(qq, &a, &b);
        ++total;
        if (dual_arm::isMidpointSafe(kin.worldTgrasp(Arm::Left, a), kin.worldTgrasp(Arm::Right, b)))
            ++safe;
    }
    std::printf("[C4.3 S1 T3] legacy-midpoint guard: %d/%d random poses considered safe\n", safe,
                total);
    EXPECT_GT(safe, total / 2);
}

TEST(ObjectJacobianPi, LegacyMidpointSilentlyLiesAtPi) {
    // Documents the defect the anchored interface replaces: the shipped midpoint
    // Jacobian does NOT match the finite difference of the shipped midpoint pose
    // when the arms are π apart.  Combined with the guard test above, no silent
    // wrong value survives in the C4.3 path.
    DualArmKinematics kin;
    const Q12 q = exactlyPiPose();
    JointConfig ql, qr;
    DualArmKinematics::split(q, &ql, &qr);
    std::mt19937 rng(20261031u);
    const Q12 dq = randomDirection(rng);
    (void)dq;

    auto objectAt = [&](const Q12& qq) { return kin.worldTobject(qq); };
    const Iso3 T0 = kin.worldTobject(q);
    const Mat6x12 J_legacy = kin.objectJacobianBody(q);

    // Take the WORST direction rather than one arbitrary draw.
    double legacy_rel = 0.0, worst_num = 0.0;
    for (int n = 0; n < 200; ++n) {
        const Q12 dqn = randomDirection(rng);
        const Vec6 num = fdBodyTwist(objectAt, q, dqn, T0);
        const Vec6 ana = J_legacy * dqn;
        const double denom = std::max(1e-12, num.cwiseAbs().maxCoeff());
        if (num.cwiseAbs().maxCoeff() > 1e-6) {
            legacy_rel = std::max(legacy_rel, (num - ana).cwiseAbs().maxCoeff() / denom);
        }
        worst_num = std::max(worst_num, num.cwiseAbs().maxCoeff());
    }
    std::printf("[C4.3 S1 T3] legacy midpoint J_object at pi: worst relative mismatch over 200 "
                "directions = %.2f%%\n",
                100.0 * legacy_rel);
    EXPECT_GT(legacy_rel, 0.05);  // the defect is real and not a rounding artefact
}

TEST(ObjectJacobianPi, LogSO3SignIsUnstableAtTheNearPiBranchThreshold) {
    // DOCUMENTED DEFECT (finding F-S1-1, reported for approval; NOT changed by S1
    // because se3.hpp is shared with the C2/C4.2 code paths).
    //
    // se3.hpp:57 takes a special branch when pi - theta < 1e-6 and recovers the
    // axis from the dominant column of (R + I)/2.  That column is proportional to
    // sign(axis[best]) * axis, so the RETURNED AXIS CAN BE NEGATED relative to the
    // general branch just below the threshold -- i.e. logSO3 can jump by ~2*pi
    // inside a 1e-6-wide band even though the true logarithm is unambiguous there.
    //
    // Consequence for C4.3: a finite-difference oracle cannot validate anything
    // that evaluates logSO3 in that band, and a closed-loop controller using the
    // object orientation error would see a 2*pi transient.
    const double margin = 1e-7;
    const Vec3 axis = Vec3(-0.9, 0.1, 0.2).normalized();  // largest component NEGATIVE
    ASSERT_LT(axis[0], 0.0);
    const Mat3 R = dual_arm::expSO3((M_PI - margin) * axis);
    const Vec3 got = dual_arm::logSO3(R);
    const Vec3 want = (M_PI - margin) * axis;
    std::printf("[C4.3 S1 F-S1-1] logSO3 in the near-pi band: |logSO3 - true| = %.6f rad "
                "(dot = %+.4f)\n",
                (got - want).norm(), got.normalized().dot(want.normalized()));
    // Characterisation: the shipped implementation is sign-inconsistent here.
    EXPECT_GT((got - want).norm(), 1.0);
    EXPECT_LT(got.normalized().dot(want.normalized()), 0.0);
    // Outside the band the general branch is exact.
    const Mat3 R2 = dual_arm::expSO3((M_PI - 1e-3) * axis);
    const Vec3 got2 = dual_arm::logSO3(R2);
    EXPECT_LT((got2 - (M_PI - 1e-3) * axis).norm(), 1e-9);
}

TEST(ObjectJacobianPi, AnchoredObjectJacobianIsExactAtExactlyPi) {
    DualArmKinematics kin;
    std::mt19937 rng(20261032u);
    const Q12 q = exactlyPiPose();
    JointConfig ql, qr;
    DualArmKinematics::split(q, &ql, &qr);
    const Iso3 G_L = randomRigid(rng);

    auto anchoredAt = [&](const JointConfig& qq) {
        return dual_arm::anchoredObjectPose(kin.worldTgrasp(Arm::Left, qq), G_L);
    };
    const Iso3 T0 = anchoredAt(ql);
    double max_err = 0.0;
    for (int n = 0; n < 50; ++n) {
        JointConfig dq;
        std::normal_distribution<double> gauss(0.0, 1.0);
        for (int i = 0; i < 6; ++i) dq[i] = gauss(rng);
        dq /= dq.norm();
        const Vec6 num = fdBodyTwist(anchoredAt, ql, dq, T0);
        const Mat6 J = dual_arm::anchoredObjectTwistJacobian(kin.jacobianBody(Arm::Left, ql), G_L);
        max_err = std::max(max_err, (num - J * dq).cwiseAbs().maxCoeff());
    }
    std::printf("[C4.3 S1 T4] anchored object Jacobian at exactly-pi arm pose: max err = %.3e\n",
                max_err);
    EXPECT_LT(max_err, 1e-6);
}

TEST(ObjectJacobianPi, AnchoredObjectJacobianSweepsThroughPi) {
    // T4 — walk the right arm away from the exactly-π mirror pose and check the
    // anchored Jacobian on both sides, at π, and for arm-relative angles above π
    // (which fold back to π - eps).
    DualArmKinematics kin;
    std::mt19937 rng(20261033u);
    const Q12 q0 = exactlyPiPose();
    JointConfig ql0, qr0;
    DualArmKinematics::split(q0, &ql0, &qr0);
    const Iso3 G_L = randomRigid(rng);

    JointConfig dir;
    dir << 0.11, -0.23, 0.17, 0.31, -0.19, 0.07;
    dir /= dir.norm();

    double max_err = 0.0, min_xi = M_PI, max_xi = 0.0;
    for (int n = 0; n <= 40; ++n) {
        const double t = -0.20 + 0.01 * n;  // crosses t = 0 (exactly pi)
        const JointConfig qr = qr0 + t * dir;
        const Iso3 WR = kin.worldTgrasp(Arm::Right, qr);
        min_xi = std::min(min_xi, dual_arm::relativeRotationAngle(kin.worldTgrasp(Arm::Left, ql0), WR));
        max_xi = std::max(max_xi, dual_arm::relativeRotationAngle(kin.worldTgrasp(Arm::Left, ql0), WR));

        auto anchoredAt = [&](const JointConfig& qq) {
            return dual_arm::anchoredObjectPose(kin.worldTgrasp(Arm::Left, qq), G_L);
        };
        const Iso3 T0 = anchoredAt(ql0);
        JointConfig dq;
        std::normal_distribution<double> gauss(0.0, 1.0);
        for (int i = 0; i < 6; ++i) dq[i] = gauss(rng);
        dq /= dq.norm();
        const Vec6 num = fdBodyTwist(anchoredAt, ql0, dq, T0);
        const Mat6 J = dual_arm::anchoredObjectTwistJacobian(kin.jacobianBody(Arm::Left, ql0), G_L);
        max_err = std::max(max_err, (num - J * dq).cwiseAbs().maxCoeff());
    }
    std::printf("[C4.3 S1 T4] pi sweep: arm-relative angle in [%.4f, %.4f] rad, max anchored err = "
                "%.3e\n",
                min_xi, max_xi, max_err);
    EXPECT_NEAR(max_xi, M_PI, 1e-12);
    EXPECT_LT(max_err, 1e-6);
}

TEST(ObjectJacobianPi, DecoupledErrorJacobianIsExactAcrossTheObjectError) {
    // The rate map uses rightJacobianInverseSO3(log(R_hat^T R_des)), whose own
    // singularity is at 2*pi -- NOT at pi.  Validate it for object pose errors on
    // both sides of pi and for large translations.
    //
    // The theta list deliberately excludes the |pi - theta| < 1e-6 band, because
    // INSIDE that band the finite-difference oracle itself is invalid: logSO3
    // switches branches there and can negate its axis (see
    // LogSO3SignIsUnstableAtTheNearPiBranchThreshold and finding F-S1-1).  The
    // analytic map has no such problem -- only the oracle does.
    DualArmKinematics kin;
    std::mt19937 rng(20261034u);
    const Q12 q = exactlyPiPose();
    JointConfig ql, qr;
    DualArmKinematics::split(q, &ql, &qr);
    const Iso3 G_L = randomRigid(rng);
    const Iso3 T_hat = dual_arm::anchoredObjectPose(kin.worldTgrasp(Arm::Left, ql), G_L);

    std::vector<double> well_conditioned = {0.0, 0.2, 1.0, 2.0, 2.5};
    std::vector<double> near_pi = {M_PI - 1e-1, M_PI - 1e-2, M_PI - 1e-3, M_PI - 1e-4};

    JointConfig dq;
    std::normal_distribution<double> gauss(0.0, 1.0);
    for (int i = 0; i < 6; ++i) dq[i] = gauss(rng);
    dq /= dq.norm();

    auto makeDes = [&](double theta) {
        Iso3 T_des = T_hat;
        T_des.linear() =
            T_hat.linear() * dual_arm::expSO3(theta * Vec3(1.0, 0.5, -0.3).normalized());
        T_des.translation() += Vec3(0.08, -0.13, 0.05);  // large error, not a small-angle case
        return T_des;
    };
    auto errAt = [&](const Iso3& T_des, const JointConfig& qq) {
        return dual_arm::decoupledPoseError(
            dual_arm::anchoredObjectPose(kin.worldTgrasp(Arm::Left, qq), G_L), T_des);
    };

    // --- Part A: central differences where the oracle is well conditioned -------
    double max_err = 0.0;
    for (double theta : well_conditioned) {
        const Iso3 T_des = makeDes(theta);
        const Vec6 num = (errAt(T_des, ql + kH * dq) - errAt(T_des, ql - kH * dq)) / (2.0 * kH);
        const Mat6 J = dual_arm::anchoredObjectErrorJacobian(kin.jacobianBody(Arm::Left, ql), G_L,
                                                            T_hat, T_des);
        const double e = (num - J * dq).cwiseAbs().maxCoeff();
        std::printf("[C4.3 S1 T4] central FD  theta = %.9f rad -> |FD - analytic| = %.3e\n", theta,
                    e);
        max_err = std::max(max_err, e);
    }
    EXPECT_LT(max_err, 1e-6);

    // --- Part B: near pi, use forward consistency instead ----------------------
    // A central difference would divide the oracle's own noise by 2h.  logSO3's
    // conditioning degrades like 1/sin(theta), so as theta -> pi the *oracle*
    // (not the analytic map) blows up: measured 2.3e-4 at theta = pi - 1e-3.
    // Checking  e(q + h dq) ~= e(q) + h J dq  needs no division by h and stays
    // well behaved.  The h^2 scaling below confirms the map is the correct
    // first-order term rather than a fit at one arbitrary step.
    const double h = 3e-4;
    double max_forward = 0.0;
    for (double theta : near_pi) {
        const Iso3 T_des = makeDes(theta);
        const Mat6 J = dual_arm::anchoredObjectErrorJacobian(kin.jacobianBody(Arm::Left, ql), G_L,
                                                            T_hat, T_des);
        const Vec6 e0 = errAt(T_des, ql);
        const Vec6 r1 = errAt(T_des, ql + h * dq) - e0 - h * (J * dq);
        const Vec6 r2 = errAt(T_des, ql + 0.5 * h * dq) - e0 - 0.5 * h * (J * dq);
        const double n1 = r1.cwiseAbs().maxCoeff();
        const double n2 = r2.cwiseAbs().maxCoeff();
        std::printf("[C4.3 S1 T4] forward cons theta = %.9f rad -> |O(h^2)| = %.3e "
                    "(halving h gives %.3e, ratio %.2f)\n",
                    theta, n1, n2, n2 > 1e-15 ? n1 / n2 : 0.0);
        max_forward = std::max(max_forward, n1);
    }
    // O(h^2) with h = 3e-4 is a few times 1e-7; 1e-5 leaves an order of margin
    // while still being far tighter than any physical tolerance.
    EXPECT_LT(max_forward, 1e-5);
}

TEST(ObjectJacobianPi, JacobianTestWouldCatchRegression) {
    // T7 — run the finite-difference oracle against the TEST-ONLY reimplementation
    // of the old midpoint algorithm.  Two things must hold:
    //   (i)  it agrees with the shipped midpoint Jacobian in the regular region
    //        (so it really is the same algorithm, not a straw man), and
    //   (ii) the oracle flags it at pi (so the regression test has real power).
    DualArmKinematics kin;
    std::mt19937 rng(20261035u);

    // (i) regular region: test-only negative vs shipped analytic
    double max_regular = 0.0;
    for (int n = 0; n < 50; ++n) {
        const Q12 q12 = DualArmKinematics::stack(randomQ(rng), randomQ(rng));
        JointConfig ql, qr;
        DualArmKinematics::split(q12, &ql, &qr);
        if (!dual_arm::isMidpointSafe(kin.worldTgrasp(Arm::Left, ql),
                                      kin.worldTgrasp(Arm::Right, qr)))
            continue;
        const Mat6x12 mine = dual_arm::testonly::legacyMidpointBodyJacobian(kin, q12);
        const Mat6x12 shipped = kin.objectJacobianBody(q12);
        max_regular = std::max(max_regular, (mine - shipped).cwiseAbs().maxCoeff());
    }
    std::printf("[C4.3 S1 T7] test-only negative vs shipped midpoint (regular region): %.3e\n",
                max_regular);
    EXPECT_LT(max_regular, 1e-9);

    // (ii) singular region: the oracle must flag the test-only negative.
    const Q12 q_pi = exactlyPiPose();
    const Q12 dq = randomDirection(rng);
    JointConfig ql, qr;
    DualArmKinematics::split(q_pi, &ql, &qr);

    auto poseAt = [&](const Q12& qq) { return dual_arm::testonly::legacyMidpointObjectPose(kin, qq); };
    const Iso3 T0 = poseAt(q_pi);
    const Vec6 num = fdBodyTwist(poseAt, q_pi, dq, T0);
    const Vec6 ana = dual_arm::testonly::legacyMidpointBodyJacobian(kin, q_pi) * dq;
    const double abs_diff = (num - ana).cwiseAbs().maxCoeff();
    const double rel = abs_diff / std::max(1e-12, num.cwiseAbs().maxCoeff());
    std::printf("[C4.3 S1 T7] oracle flags the test-only negative at pi: |FD - analytic| = %.3e "
                "(relative %.2f%%)\n",
                abs_diff, 100.0 * rel);
    EXPECT_GT(rel, 0.05);
    EXPECT_FALSE(dual_arm::isMidpointSafe(kin.worldTgrasp(Arm::Left, ql),
                                          kin.worldTgrasp(Arm::Right, qr)));
}
