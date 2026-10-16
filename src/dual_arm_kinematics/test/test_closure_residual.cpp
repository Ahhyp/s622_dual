// test/test_closure_residual.cpp
// C4.3 S1 / T2 + T6 — decomposition of the SE(3) closure residual under a rigid
// grasp, its invariance properties, controlled-perturbation response, and the
// finite-difference validation of the closure error-rate Jacobian.
//
// Requirement source: 正式执行规划 §3.2 (independent per-arm estimate, translation
// and rotation reported separately, no "look at the displayed object" cheat) and
// §3.4 (zero residual / common transform invariance / controlled perturbation).

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
using dual_arm::ClosureResidual;
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

Q12 randomQ12(std::mt19937& rng) { return DualArmKinematics::stack(randomQ(rng), randomQ(rng)); }

Q12 randomDirection(std::mt19937& rng) {
    Q12 d;
    std::normal_distribution<double> gauss(0.0, 1.0);
    for (int i = 0; i < 12; ++i) d[i] = gauss(rng);
    return d / d.norm();
}

Iso3 randomRigid(std::mt19937& rng) {
    std::normal_distribution<double> gauss(0.0, 1.0);
    Vec3 w;
    for (int i = 0; i < 3; ++i) w[i] = gauss(rng);
    w.normalize();
    w *= std::uniform_real_distribution<double>(-0.5 * M_PI, 0.5 * M_PI)(rng);
    Iso3 T = Iso3::Identity();
    T.linear() = dual_arm::expSO3(w);
    T.translation() = Vec3(gauss(rng), gauss(rng), gauss(rng));
    return T;
}

}  // namespace

TEST(ClosureResidual, IsZeroForARigidGrasp) {
    DualArmKinematics kin;
    std::mt19937 rng(20261020u);
    // A frozen grasp: both anchored estimates come from the SAME object pose.
    const Iso3 object_T_left = randomRigid(rng);
    const Iso3 object_T_right = randomRigid(rng);
    double max_t = 0.0, max_r = 0.0;
    for (int n = 0; n < 100; ++n) {
        const Iso3 world_T_object = randomRigid(rng);
        // T_WL* = T_WO* G_L  =>  anchored estimate recovers T_WO* exactly.
        const Iso3 left =
            dual_arm::anchoredObjectPose(world_T_object * object_T_left, object_T_left);
        const Iso3 right =
            dual_arm::anchoredObjectPose(world_T_object * object_T_right, object_T_right);
        const ClosureResidual e = dual_arm::closureResidual(left, right);
        max_t = std::max(max_t, e.translation_m);
        max_r = std::max(max_r, e.rotation_rad);
    }
    std::printf("[C4.3 S1 T2] rigid grasp closure: max |t| = %.3e m, max |phi| = %.3e rad\n", max_t,
                max_r);
    EXPECT_LT(max_t, 1e-12);
    EXPECT_LT(max_r, 1e-12);
}

TEST(ClosureResidual, TranslationOnlyPerturbationIsExact) {
    std::mt19937 rng(20261021u);
    const Iso3 T_L = randomRigid(rng);
    double max_err = 0.0;
    for (int n = 0; n < 50; ++n) {
        const Vec3 delta = Vec3(0.01 * n, -0.002 * n, 0.003 * n);
        const Iso3 T_R = T_L * Iso3(Eigen::Translation3d(delta));
        const ClosureResidual e = dual_arm::closureResidual(T_L, T_R);
        max_err = std::max(max_err, (e.translation - delta).cwiseAbs().maxCoeff());
        max_err = std::max(max_err, e.rotation_rad);
        max_err = std::max(max_err, std::abs(e.translation_m - delta.norm()));
        max_err = std::max(max_err, std::abs(e.position_gap_m - delta.norm()));
    }
    std::printf("[C4.3 S1 T2] translation-only residual vs exact: max err = %.3e\n", max_err);
    EXPECT_LT(max_err, 1e-12);
}

TEST(ClosureResidual, RotationOnlyPerturbationIsExact) {
    std::mt19937 rng(20261022u);
    const Iso3 T_L = randomRigid(rng);
    double max_err = 0.0;
    for (int n = 1; n < 20; ++n) {
        const double theta = n * 0.15;  // up to 2.85 rad, approaching pi
        const Vec3 axis = Vec3(1.0, 2.0, -0.5).normalized();
        const Iso3 T_R = T_L * Iso3(Eigen::AngleAxisd(theta, axis));
        const ClosureResidual e = dual_arm::closureResidual(T_L, T_R);
        max_err = std::max(max_err, (e.rotation - theta * axis).cwiseAbs().maxCoeff());
        max_err = std::max(max_err, e.translation_m);
        max_err = std::max(max_err, std::abs(e.rotation_rad - theta));
    }
    std::printf("[C4.3 S1 T2] rotation-only residual vs exact: max err = %.3e\n", max_err);
    EXPECT_LT(max_err, 1e-12);
}

TEST(ClosureResidual, ReportsTranslationAndRotationSeparately) {
    const Iso3 T_L = Iso3::Identity();
    // Pure translation: rotation must be untouched.
    const Iso3 T_trans = T_L * Iso3(Eigen::Translation3d(0.05, 0.0, 0.0));
    const ClosureResidual a = dual_arm::closureResidual(T_L, T_trans);
    EXPECT_GT(a.translation_m, 0.0499);
    EXPECT_LT(a.rotation_rad, 1e-15);
    // Pure rotation: translation must be untouched even though the point moves.
    const Iso3 T_rot = T_L * Iso3(Eigen::AngleAxisd(0.7, Vec3::UnitZ()));
    const ClosureResidual b = dual_arm::closureResidual(T_L, T_rot);
    EXPECT_LT(b.translation_m, 1e-15);
    EXPECT_GT(b.rotation_rad, 0.6999);
    // Both together: neither may absorb the other.
    const Iso3 T_both = T_L * Iso3(Eigen::Translation3d(0.05, 0.0, 0.0)) *
                        Iso3(Eigen::AngleAxisd(0.7, Vec3::UnitZ()));
    const ClosureResidual c = dual_arm::closureResidual(T_L, T_both);
    EXPECT_NEAR(c.translation_m, 0.05, 1e-12);
    EXPECT_NEAR(c.rotation_rad, 0.7, 1e-12);
}

TEST(ClosureResidual, IsInvariantUnderACommonRigidTransform) {
    std::mt19937 rng(20261023u);
    double max_t = 0.0, max_r = 0.0;
    for (int n = 0; n < 100; ++n) {
        const Iso3 T_L = randomRigid(rng);
        const Iso3 T_R = randomRigid(rng);
        const Iso3 common = randomRigid(rng);
        const ClosureResidual base = dual_arm::closureResidual(T_L, T_R);
        const ClosureResidual moved = dual_arm::closureResidual(common * T_L, common * T_R);
        max_t = std::max(max_t, (base.translation - moved.translation).cwiseAbs().maxCoeff());
        max_r = std::max(max_r, (base.rotation - moved.rotation).cwiseAbs().maxCoeff());
    }
    std::printf("[C4.3 S1 T2] common-transform invariance: %.3e m / %.3e rad\n", max_t, max_r);
    EXPECT_LT(max_t, 1e-12);
    EXPECT_LT(max_r, 1e-12);
}

TEST(ClosureResidual, IsAntisymmetricBetweenTheTwoAnchors) {
    // Anchoring on the RIGHT instead of the LEFT must give the same physical
    // mismatch re-expressed in the right frame:  t' = -R_c^T t, phi' = -R_c^T phi.
    std::mt19937 rng(20261024u);
    double max_err = 0.0;
    for (int n = 0; n < 100; ++n) {
        const Iso3 T_L = randomRigid(rng);
        const Iso3 T_R = randomRigid(rng);
        const Mat3 R_c = T_L.linear().transpose() * T_R.linear();
        const ClosureResidual a = dual_arm::closureResidual(T_L, T_R);
        const ClosureResidual b = dual_arm::closureResidual(T_R, T_L);
        max_err = std::max(max_err, (b.translation + R_c.transpose() * a.translation).cwiseAbs().maxCoeff());
        max_err = std::max(max_err, (b.rotation + R_c.transpose() * a.rotation).cwiseAbs().maxCoeff());
        // The frame-independent gap is symmetric.
        max_err = std::max(max_err, std::abs(a.position_gap_m - b.position_gap_m));
    }
    std::printf("[C4.3 S1 T2] anchor antisymmetry: max err = %.3e\n", max_err);
    EXPECT_LT(max_err, 1e-12);
}

TEST(ClosureResidual, MatchesTheLogSE3Definition) {
    // CONVENTION NOTE (deliberate): the C4.3 residual reports the *raw frame
    // difference* of the object origin,  t = R_L^T (p_R - p_L),  whose units are
    // metres and which is what a real relative-position measurement gives.  This
    // differs from the linear part of logSE3(T_L^-1 T_R), which is
    // J_l(phi)^-1 t.  The relation between them is asserted below so the choice
    // is pinned rather than accidental.
    std::mt19937 rng(20261025u);
    double max_rot = 0.0, max_pos = 0.0, max_raw = 0.0;
    for (int n = 0; n < 200; ++n) {
        const Iso3 T_L = randomRigid(rng);
        const Iso3 T_R = randomRigid(rng);
        const Vec6 xi = dual_arm::logSE3(T_L.inverse() * T_R);
        const ClosureResidual e = dual_arm::closureResidual(T_L, T_R);
        max_rot = std::max(max_rot, (e.rotation - xi.tail<3>()).cwiseAbs().maxCoeff());
        // position_gap is the frame-independent modulus of the same vector
        max_pos = std::max(max_pos, std::abs(e.position_gap_m - e.translation.norm()));
        // t == J_l(phi) * (linear part of the SE(3) log)
        const Vec3 p_from_log = dual_arm::leftJacobianSO3(xi.tail<3>()) * xi.head<3>();
        max_raw = std::max(max_raw, (e.translation - p_from_log).cwiseAbs().maxCoeff());
    }
    std::printf("[C4.3 S1 T2] residual vs logSE3: rot %.3e, |t| %.3e, decoupled-vs-Jl  %.3e\n",
                max_rot, max_pos, max_raw);
    EXPECT_LT(max_rot, 1e-12);
    EXPECT_LT(max_pos, 1e-15);
    EXPECT_LT(max_raw, 1e-12);
}

TEST(ClosureResidual, FlagsDirectionAmbiguityAtExactlyPi) {
    const Iso3 T_L = Iso3::Identity();
    const Iso3 T_R = Iso3(Eigen::AngleAxisd(M_PI, Vec3::UnitZ()));
    const ClosureResidual e = dual_arm::closureResidual(T_L, T_R);
    EXPECT_NEAR(e.rotation_rad, M_PI, 1e-12);
    EXPECT_TRUE(e.rotation_direction_ambiguous);
    // Just off pi the direction becomes identifiable again.
    const Iso3 T_R2 = Iso3(Eigen::AngleAxisd(M_PI - 1e-3, Vec3::UnitZ()));
    const ClosureResidual e2 = dual_arm::closureResidual(T_L, T_R2);
    EXPECT_FALSE(e2.rotation_direction_ambiguous);
}

TEST(ClosureResidual, ErrorJacobianMatchesFiniteDifference) {
    // T6 — the closure error-rate Jacobian is the only map C4.3 will use to drive
    // the relative sub-task, so it must match a finite-difference oracle.
    DualArmKinematics kin;
    std::mt19937 rng(20261026u);
    const Iso3 object_T_left = randomRigid(rng);
    const Iso3 object_T_right = randomRigid(rng);
    double max_err = 0.0;
    int n_checked = 0;
    for (int n = 0; n < 100; ++n) {
        const Q12 q = randomQ12(rng);
        const Q12 dq = randomDirection(rng);
        JointConfig ql, qr;
        DualArmKinematics::split(q, &ql, &qr);

        auto leftAt = [&](const Q12& qq) {
            JointConfig a, b;
            DualArmKinematics::split(qq, &a, &b);
            return dual_arm::anchoredObjectPose(kin.worldTgrasp(Arm::Left, a), object_T_left);
        };
        auto rightAt = [&](const Q12& qq) {
            JointConfig a, b;
            DualArmKinematics::split(qq, &a, &b);
            return dual_arm::anchoredObjectPose(kin.worldTgrasp(Arm::Right, b), object_T_right);
        };

        // A larger step than kH: the residual is assembled with logSE3, whose
        // theta extraction goes through acos(0.5(tr R - 1)) and therefore loses
        // precision as the relative rotation shrinks toward 0.  Dividing that
        // noise by 2h is what limits this oracle, so h is kept moderate here.
        const double h = 1e-5;
        const Vec6 num =
            (dual_arm::closureResidual(leftAt(q + h * dq), rightAt(q + h * dq)).xi() -
             dual_arm::closureResidual(leftAt(q - h * dq), rightAt(q - h * dq)).xi()) /
            (2.0 * h);

        const Mat6 J_L = dual_arm::anchoredObjectTwistJacobian(kin.jacobianBody(Arm::Left, ql),
                                                              object_T_left);
        const Mat6 J_R = dual_arm::anchoredObjectTwistJacobian(kin.jacobianBody(Arm::Right, qr),
                                                               object_T_right);
        const Mat6x12 J_c =
            dual_arm::closureErrorJacobian(leftAt(q), rightAt(q), J_L, J_R);
        const Vec6 ana = J_c * dq;

        max_err = std::max(max_err, (num - ana).cwiseAbs().maxCoeff());
        ++n_checked;
    }
    std::printf("[C4.3 S1 T6] closure error-rate Jacobian vs FD (%d poses): max abs diff = %.3e\n",
                n_checked, max_err);
    EXPECT_LT(max_err, 1e-6);
}

TEST(ClosureResidual, ErrorJacobianIsExactAtArmRelativePi) {
    // The whole point of the anchored formulation: no singularity at pi.
    DualArmKinematics kin;
    JointConfig q;
    q << 0.0, -0.9, 0.9, -0.9, -0.6, 0.0;
    const Q12 q12 = DualArmKinematics::stack(q, q);
    JointConfig ql, qr;
    DualArmKinematics::split(q12, &ql, &qr);
    const Iso3 WL = kin.worldTgrasp(Arm::Left, ql);
    const Iso3 WR = kin.worldTgrasp(Arm::Right, qr);
    ASSERT_NEAR(dual_arm::relativeRotationAngle(WL, WR), M_PI, 1e-12);

    std::mt19937 rng(20261027u);
    const Iso3 object_T_left = randomRigid(rng);
    const Iso3 object_T_right = randomRigid(rng);
    const Q12 dq = randomDirection(rng);

    auto leftAt = [&](const Q12& qq) {
        JointConfig a, b;
        DualArmKinematics::split(qq, &a, &b);
        return dual_arm::anchoredObjectPose(kin.worldTgrasp(Arm::Left, a), object_T_left);
    };
    auto rightAt = [&](const Q12& qq) {
        JointConfig a, b;
        DualArmKinematics::split(qq, &a, &b);
        return dual_arm::anchoredObjectPose(kin.worldTgrasp(Arm::Right, b), object_T_right);
    };
    const Vec6 num = (dual_arm::closureResidual(leftAt(q12 + kH * dq), rightAt(q12 + kH * dq)).xi() -
                      dual_arm::closureResidual(leftAt(q12 - kH * dq), rightAt(q12 - kH * dq)).xi()) /
                     (2.0 * kH);
    const Mat6 J_L =
        dual_arm::anchoredObjectTwistJacobian(kin.jacobianBody(Arm::Left, ql), object_T_left);
    const Mat6 J_R =
        dual_arm::anchoredObjectTwistJacobian(kin.jacobianBody(Arm::Right, qr), object_T_right);
    const Vec6 ana = dual_arm::closureErrorJacobian(leftAt(q12), rightAt(q12), J_L, J_R) * dq;
    std::printf("[C4.3 S1 T6] closure Jacobian at exactly-pi arm pose: max abs diff = %.3e\n",
                (num - ana).cwiseAbs().maxCoeff());
    EXPECT_LT((num - ana).cwiseAbs().maxCoeff(), 1e-6);
}

TEST(ClosureResidual, BothAnchorsAgreeExactlyAtArmRelativePi) {
    // The pi defect comes from AVERAGING two grasp rotations ~pi apart.  Comparing
    // two independent object estimates is immune: at exactly pi they still agree
    // to machine precision.
    DualArmKinematics kin;
    JointConfig q;
    q << 0.3, -1.1, 0.7, -0.8, -0.4, 0.2;
    const Q12 q12 = DualArmKinematics::stack(q, q);
    JointConfig ql, qr;
    DualArmKinematics::split(q12, &ql, &qr);
    const Iso3 WL = kin.worldTgrasp(Arm::Left, ql);
    const Iso3 WR = kin.worldTgrasp(Arm::Right, qr);
    ASSERT_NEAR(dual_arm::relativeRotationAngle(WL, WR), M_PI, 1e-12);

    // A rigid grasp: pick G_L, derive G_R from the true relative pose.
    const Iso3 G_L = Iso3(Eigen::Translation3d(0.03, 0.0, 0.0));
    const Iso3 T_rel = WL.inverse() * WR;
    const Iso3 G_R = G_L * T_rel;
    const ClosureResidual e =
        dual_arm::closureResidual(dual_arm::anchoredObjectPose(WL, G_L),
                                  dual_arm::anchoredObjectPose(WR, G_R));
    std::printf("[C4.3 S1 T6] anchors at exactly-pi arm pose: |t| = %.3e m, |phi| = %.3e rad\n",
                e.translation_m, e.rotation_rad);
    EXPECT_LT(e.translation_m, 1e-12);
    EXPECT_LT(e.rotation_rad, 1e-12);
}
