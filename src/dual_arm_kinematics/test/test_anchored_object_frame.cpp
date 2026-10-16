// test/test_anchored_object_frame.cpp
// C4.3 S1 / T5' + T8 + T9 — the anchored object interface's contract against the
// legacy symmetric midpoint, and the meaning of "left anchored".
//
//  * T5' : if the grasp geometry G_L / G_R is derived from an object frame, the
//          anchored estimators reproduce that frame EXACTLY — at every
//          configuration, including an arm-relative rotation of exactly π.
//          This is the guarantee that adopting the new interface does not move
//          the object frame under the C2/C4.2 code paths.
//  * T8  : "left anchored" describes where the OBJECT ESTIMATE comes from, it
//          does NOT mean "freeze the left arm".  T_hat_{O,L} is a function of
//          q_L alone; the right arm re-enters through the closure residual.
//          (User adjustment m01472 #2 demands this be explicit.)
//  * T9  : the anchored object task is fully actuable — rank 6.
//
// NOTE ON SCOPE: the plan's original T5 ("thin adapter equivalence, dq matching
// the 4-argument core call to 1e-12") cannot exist in S1 because S1 builds no
// adapter (report §A14.7).  It is deferred to S3 and replaced here by T5', which
// is the part of that equivalence that lives in the math layer.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <random>

#include <Eigen/Dense>

#include "dual_arm_kinematics/closure_residual.hpp"
#include "dual_arm_kinematics/dual_arm_kinematics.hpp"
#include "dual_arm_kinematics/object_frame.hpp"
#include "dual_arm_kinematics/se3.hpp"

namespace {

using dual_arm::Arm;
using dual_arm::DualArmKinematics;
using dual_arm::Iso3;
using dual_arm::JointConfig;
using dual_arm::Mat3;
using dual_arm::Mat6;
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

Q12 randomQ12(std::mt19937& rng) { return DualArmKinematics::stack(randomQ(rng), randomQ(rng)); }

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

/// Derive G_L / G_R from an object frame, i.e. the inverse of the anchored map.
void graspGeometryFromObjectFrame(const Iso3& world_T_object, const Iso3& world_T_left,
                                  const Iso3& world_T_right, Iso3* object_T_left,
                                  Iso3* object_T_right) {
    *object_T_left = world_T_object.inverse() * world_T_left;
    *object_T_right = world_T_object.inverse() * world_T_right;
}

}  // namespace

TEST(AnchoredObjectFrame, AnchoredEstimatorReproducesTheLegacyMidpointExactly) {
    // T5' — the object frame must not move when the same geometry is re-expressed
    // in the anchored parameterisation.  Checked at generic configurations AND at
    // an arm-relative rotation of exactly π.
    DualArmKinematics kin;
    std::mt19937 rng(20261101u);
    double worst_generic = 0.0, worst_left_right_star = 0.0;

    for (int n = 0; n < 200; ++n) {
        const Q12 q = randomQ12(rng);
        JointConfig ql, qr;
        DualArmKinematics::split(q, &ql, &qr);
        const Iso3 WL = kin.worldTgrasp(Arm::Left, ql);
        const Iso3 WR = kin.worldTgrasp(Arm::Right, qr);
        const Iso3 T_WO = kin.worldTobject(q);

        Iso3 G_L, G_R;
        graspGeometryFromObjectFrame(T_WO, WL, WR, &G_L, &G_R);

        worst_generic = std::max(worst_generic,
                                 (dual_arm::anchoredObjectPose(WL, G_L).matrix() - T_WO.matrix())
                                     .cwiseAbs()
                                     .maxCoeff());
        worst_generic = std::max(worst_generic,
                                 (dual_arm::anchoredObjectPose(WR, G_R).matrix() - T_WO.matrix())
                                     .cwiseAbs()
                                     .maxCoeff());
        // leftTrightStar() == G_L^-1 G_R == WL^-1 WR  (the frozen relative grasp)
        const Iso3 star = G_L.inverse() * G_R;
        worst_left_right_star =
            std::max(worst_left_right_star,
                     (star.matrix() - (WL.inverse() * WR).matrix()).cwiseAbs().maxCoeff());
    }
    std::printf("[C4.3 S1 T5'] anchored reproduces the midpoint frame: max err = %.3e "
                "(200 generic poses); frozen grasp G_L^-1 G_R err = %.3e\n",
                worst_generic, worst_left_right_star);
    EXPECT_LT(worst_generic, 1e-12);
    EXPECT_LT(worst_left_right_star, 1e-12);

    // Same statement AT pi, where the legacy midpoint is itself ill-conditioned.
    // The estimator equivalence is pure algebra and must not degrade.
    const Q12 qpi = exactlyPiPose();
    JointConfig ql, qr;
    DualArmKinematics::split(qpi, &ql, &qr);
    const Iso3 WL = kin.worldTgrasp(Arm::Left, ql);
    const Iso3 WR = kin.worldTgrasp(Arm::Right, qr);
    EXPECT_NEAR(dual_arm::midpointMargin(WL, WR), 0.0, 1e-12);  // premise: this IS pi
    EXPECT_FALSE(dual_arm::isMidpointSafe(WL, WR));             // premise: legacy is unsafe here
    const Iso3 T_WO_pi = kin.worldTobject(qpi);
    Iso3 G_L, G_R;
    graspGeometryFromObjectFrame(T_WO_pi, WL, WR, &G_L, &G_R);
    const double err_pi =
        std::max((dual_arm::anchoredObjectPose(WL, G_L).matrix() - T_WO_pi.matrix()).cwiseAbs().maxCoeff(),
                 (dual_arm::anchoredObjectPose(WR, G_R).matrix() - T_WO_pi.matrix()).cwiseAbs().maxCoeff());
    std::printf("[C4.3 S1 T5'] at exactly pi: anchored reproduces the midpoint frame to %.3e\n",
                err_pi);
    EXPECT_LT(err_pi, 1e-12);
}

TEST(AnchoredObjectFrame, TheAnchoredEstimateIgnoresTheOtherArm) {
    // T8a — T_hat_{O,L} is a function of q_L only.  Stated as a test so that a
    // future "improvement" that silently couples the two arms cannot slip in.
    DualArmKinematics kin;
    std::mt19937 rng(20261102u);
    const Q12 q = randomQ12(rng);
    JointConfig ql, qr;
    DualArmKinematics::split(q, &ql, &qr);
    const Iso3 G_L = randomRigid(rng);
    const Iso3 G_R = randomRigid(rng);

    auto leftEstimate = [&](const Q12& qq) {
        JointConfig a, b;
        DualArmKinematics::split(qq, &a, &b);
        return dual_arm::anchoredObjectPose(kin.worldTgrasp(Arm::Left, a), G_L);
    };
    auto rightEstimate = [&](const Q12& qq) {
        JointConfig a, b;
        DualArmKinematics::split(qq, &a, &b);
        return dual_arm::anchoredObjectPose(kin.worldTgrasp(Arm::Right, b), G_R);
    };

    double worst_L_wrt_R = 0.0, worst_R_wrt_L = 0.0, self_L = 0.0, self_R = 0.0;
    for (int n = 0; n < 50; ++n) {
        JointConfig dl, dr;
        DualArmKinematics::split(randomDirection(rng), &dl, &dr);
        Q12 move_right = Q12::Zero(), move_left = Q12::Zero();
        move_right.tail<6>() = dr;
        move_left.head<6>() = dl;

        // The LEFT estimate must be completely blind to q_R ...
        worst_L_wrt_R = std::max(
            worst_L_wrt_R,
            (leftEstimate(q + kH * move_right).matrix() - leftEstimate(q - kH * move_right).matrix())
                .cwiseAbs()
                .maxCoeff());
        // ... and fully sensitive to q_L.
        self_L = std::max(self_L,
                          (leftEstimate(q + kH * move_left).matrix() -
                           leftEstimate(q - kH * move_left).matrix())
                              .cwiseAbs()
                              .maxCoeff());
        // Mirror image for the right anchor.
        worst_R_wrt_L = std::max(
            worst_R_wrt_L,
            (rightEstimate(q + kH * move_left).matrix() - rightEstimate(q - kH * move_left).matrix())
                .cwiseAbs()
                .maxCoeff());
        self_R = std::max(self_R,
                          (rightEstimate(q + kH * move_right).matrix() -
                           rightEstimate(q - kH * move_right).matrix())
                              .cwiseAbs()
                              .maxCoeff());
    }
    std::printf("[C4.3 S1 T8a] d(T_hat_O,L)/d q_R = %.3e (must be 0) ; d/dq_L = %.3e\n"
                "                d(T_hat_O,R)/d q_L = %.3e (must be 0) ; d/dq_R = %.3e\n",
                worst_L_wrt_R, self_L, worst_R_wrt_L, self_R);
    EXPECT_EQ(worst_L_wrt_R, 0.0);
    EXPECT_EQ(worst_R_wrt_L, 0.0);
    EXPECT_GT(self_L, 1e-6);
    EXPECT_GT(self_R, 1e-6);
}

TEST(AnchoredObjectFrame, TheClosureResidualRespondsToBothArms) {
    // T8b — this is where "left anchored" stops meaning "left arm frozen".
    // The object-pose task is a one-arm task; the TWO-arm coupling is the
    // relative-pose constraint, and its residual must react to either arm.
    DualArmKinematics kin;
    std::mt19937 rng(20261103u);
    const Q12 q = randomQ12(rng);
    JointConfig ql, qr;
    DualArmKinematics::split(q, &ql, &qr);

    const Q12 d = randomDirection(rng);
    JointConfig dl, dr;
    DualArmKinematics::split(d, &dl, &dr);

    // A zero residual needs grasp geometry that is actually consistent with the
    // robot configuration, so derive G from this configuration first.
    Iso3 G_Lr, G_Rr;
    graspGeometryFromObjectFrame(kin.worldTobject(q), kin.worldTgrasp(Arm::Left, ql),
                                 kin.worldTgrasp(Arm::Right, qr), &G_Lr, &G_Rr);
    auto est = [&](const JointConfig& a, const JointConfig& b) {
        return dual_arm::closureResidual(
            dual_arm::anchoredObjectPose(kin.worldTgrasp(Arm::Left, a), G_Lr),
            dual_arm::anchoredObjectPose(kin.worldTgrasp(Arm::Right, b), G_Rr));
    };
    const double zero = est(ql, qr).xi().cwiseAbs().maxCoeff();
    const double only_left = (est(ql + kH * dl, qr).xi() - est(ql, qr).xi()).cwiseAbs().maxCoeff();
    const double only_right = (est(ql, qr + kH * dr).xi() - est(ql, qr).xi()).cwiseAbs().maxCoeff();
    std::printf("[C4.3 S1 T8b] closure residual: rigid grasp %.3e ; d/dq_L %.3e ; d/dq_R %.3e\n",
                zero, only_left, only_right);
    EXPECT_LT(zero, 1e-12);       // the two independent estimates agree for a rigid grasp
    EXPECT_GT(only_left, 1e-9);   // the left arm enters the two-arm coupling
    EXPECT_GT(only_right, 1e-9);  // ... and so does the right arm
}

TEST(AnchoredObjectFrame, TheAnchoredObjectTaskHasFullRankSix) {
    // T9 — a left-anchored object task must be able to command all six object
    // degrees of freedom from the left arm alone.  If this ever drops below 6 the
    // "anchored" formulation would be silently underactuated.
    DualArmKinematics kin;
    std::mt19937 rng(20261104u);
    int n_full = 0, n_total = 0;
    double worst_sigma = 1e9;
    for (int n = 0; n < 100; ++n) {
        const Q12 q = randomQ12(rng);
        JointConfig ql, qr;
        DualArmKinematics::split(q, &ql, &qr);
        const Iso3 G_L = randomRigid(rng);
        const Iso3 T_hat = dual_arm::anchoredObjectPose(kin.worldTgrasp(Arm::Left, ql), G_L);
        const Iso3 T_des = T_hat * randomRigid(rng);

        const Mat6 J = dual_arm::anchoredObjectErrorJacobian(kin.jacobianBody(Arm::Left, ql), G_L,
                                                            T_hat, T_des);
        Eigen::JacobiSVD<Mat6> svd(J);
        const double smin = svd.singularValues().minCoeff();
        worst_sigma = std::min(worst_sigma, smin);
        if (smin > 1e-9) ++n_full;
        ++n_total;
    }
    std::printf("[C4.3 S1 T9] anchored object error Jacobian: %d/%d poses rank 6, min sigma_min = "
                "%.3e\n",
                n_full, n_total, worst_sigma);
    EXPECT_EQ(n_full, n_total);
}
