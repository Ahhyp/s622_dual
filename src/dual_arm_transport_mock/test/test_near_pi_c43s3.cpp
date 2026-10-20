// C4.3 S3 -- W9: the near-pi contrast (S3-T10 / S3-T11).
//
// The claim S3 rests on is narrow and testable:
//
//   the anchored object row never evaluates anything at the ARM-RELATIVE rotation,
//   so its conditioning does not depend on that angle -- including at pi.
//
// The legacy midpoint row does: it builds an object frame half-way between the two
// grippers, and at pi that half-way frame is undefined (the geodesic midpoint of a
// pi rotation has no unique sign).  T10 proves the anchored row is finite, correct
// and INVARIANT; T11 proves the legacy row is wrong there, which is what makes T10
// a real assertion rather than a tautology about a formula that never sees pi.
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <array>
#include <string>

#include "dual_arm_kinematics/dual_arm_kinematics.hpp"
#include "dual_arm_kinematics/object_frame.hpp"
#include "dual_arm_transport_mock/object_task.hpp"
#include "dual_arm_transport_mock/version.hpp"
#include "legacy_cases.hpp"

namespace {

using dual_arm::Arm;
using dual_arm::Iso3;
using dual_arm::Mat3;
using dual_arm::Mat6;
using dual_arm::Mat6x12;
using dual_arm::Q12;
using dual_arm::Vec3;
using dual_arm::Vec6;

constexpr double kPi = 3.14159265358979323846;

Iso3 objectStar() {
    Iso3 T = Iso3::Identity();
    T.translation() = Vec3(0.0, 0.0, 0.275);
    return T;
}

Iso3 frozenGraspLeft() {
    Iso3 T = Iso3::Identity();
    T.linear() << 0.0, 1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, -1.0;
    T.translation() = Vec3(0.0497, 0.0, 0.025);
    return T;
}

dual_arm::JointConfig armJoints(const Q12& q, bool left) {
    dual_arm::JointConfig qa;
    for (int i = 0; i < 6; ++i) qa[i] = q[left ? i : 6 + i];
    return qa;
}

void setArmJoints(Q12& q, bool left, const dual_arm::JointConfig& qa) {
    for (int i = 0; i < 6; ++i) q[left ? i : 6 + i] = qa[i];
}

/// Damped-least-squares IK for ONE arm.  Needed because the near-pi family has to
/// be constructed in the pose domain (a chosen relative rotation), not found by
/// hoping a joint sweep happens to pass through pi.
dual_arm::JointConfig solveRightArm(const dual_arm::DualArmKinematics& kin, const Iso3& target,
                                    dual_arm::JointConfig q) {
    for (int iter = 0; iter < 200; ++iter) {
        const Iso3 cur = kin.worldTgrasp(Arm::Right, q);
        const Vec6 err = dual_arm::logSE3(cur.inverse() * target);
        if (err.norm() < 1e-13) break;
        const Mat6 J = kin.jacobianBody(Arm::Right, q);
        const double lambda2 = 1e-8;
        const Eigen::Matrix<double, 6, 6> JJt = J * J.transpose() +
                                                lambda2 * Eigen::Matrix<double, 6, 6>::Identity();
        const Vec6 dq = J.transpose() * JJt.ldlt().solve(err);
        const double step = dq.cwiseAbs().maxCoeff();
        const double scale = step > 0.2 ? 0.2 / step : 1.0;
        q += scale * dq;
        if (scale * dq.norm() < 1e-15) break;
    }
    return q;
}

/// A configuration whose arm-relative rotation is exactly `kPi - delta`.
struct NearPi {
    Q12 q = Q12::Zero();
    double achieved_angle = 0.0;
};

NearPi buildNearPi(const dual_arm::DualArmKinematics& kin, double delta) {
    const Q12 grasp = legacy_golden::graspQ();
    const dual_arm::JointConfig ql = armJoints(grasp, true);
    // Right arm starts from the left arm's joints: a valid, reachable seed.
    dual_arm::JointConfig qr = ql;
    const Iso3 T_WL = kin.worldTgrasp(Arm::Left, ql);
    Iso3 X = Iso3::Identity();
    X.linear() = dual_arm::expSO3(Vec3(0.0, 0.0, kPi - delta));
    qr = solveRightArm(kin, T_WL * X, qr);
    NearPi out;
    setArmJoints(out.q, true, ql);
    setArmJoints(out.q, false, qr);
    const Iso3 rel = kin.leftTright(out.q);
    out.achieved_angle = std::acos(std::max(-1.0, std::min(1.0, (rel.linear().trace() - 1.0) / 2.0)));
    return out;
}

}  // namespace

// ------------------------------------------------------------------- S3-T10
TEST(NearPiC43S3, T10_AnchoredRowIsFiniteExactAndInvariantAtPi) {
    const dual_arm::DualArmKinematics kin;
    const Iso3 T_d = objectStar();
    const Iso3 G_L = frozenGraspLeft();

    std::printf("\n[ S3-T10 ] arm-relative rotation swept toward pi\n");
    std::printf("  %-10s %-14s %-14s %-14s %-14s %-14s\n", "delta", "angle", "det(J^T J)",
                "sigma_min", "|e_O|", "J_O finite");

    Mat6 J_ref = Mat6::Zero();
    Vec6 e_ref = Vec6::Zero();
    double angle_ref = 0.0;
    bool first = true;

    for (double delta : {0.0, 1e-6, 1e-4, 1e-2}) {
        const NearPi np = buildNearPi(kin, delta);
        // The construction is exact: what is being tested is the row, not the IK.
        // 1e-7 rad: the construction is exact to the damped-least-squares floor
        // (~2e-8 rad at the pi end); what is under test is the row, not the IK.
        EXPECT_NEAR(np.achieved_angle, kPi - delta, 1e-7)
            << "delta=" << delta << ": IK did not reach the intended relative rotation";

        const Iso3 T_WL = kin.worldTgrasp(Arm::Left, armJoints(np.q, true));
        const Iso3 T_hat = dual_arm::anchoredObjectPose(T_WL, G_L);
        const Mat6 J_L = kin.jacobianBody(Arm::Left, armJoints(np.q, true));
        const Mat6 J_O = dual_arm::anchoredObjectErrorJacobian(J_L, G_L, T_hat, T_d);
        const Vec6 e_O = dual_arm::decoupledPoseError(T_hat, T_d);

        const bool finite = J_O.allFinite() && e_O.allFinite();
        EXPECT_TRUE(finite) << "delta=" << delta;
        const Eigen::JacobiSVD<Mat6> svd(J_O);
        std::printf("  %-10.1e %-14.10f %-14.6e %-14.6e %-14.6e %s\n", delta, np.achieved_angle,
                    svd.singularValues()(0) * svd.singularValues()(0),
                    svd.singularValues()(5), e_O.norm(), finite ? "yes" : "NO");

        if (first) {
            J_ref = J_O;
            e_ref = e_O;
            angle_ref = np.achieved_angle;
            first = false;
            continue;
        }
        // INVARIANCE.  For a fixed left arm, the anchored row is a function of
        // (q_left, G_L, T_d) only -- the right arm does not appear in it.  So the row
        // at pi and the row 1e-2 away from pi are the same matrix, to the last bit.
        EXPECT_EQ(J_O.cwiseAbs().maxCoeff(), J_ref.cwiseAbs().maxCoeff())
            << "delta=" << delta << " changed the anchored Jacobian";
        EXPECT_TRUE(J_O == J_ref) << "delta=" << delta << " changed the anchored Jacobian";
        EXPECT_TRUE(e_O == e_ref) << "delta=" << delta << " changed the anchored error";
        EXPECT_GT(angle_ref, np.achieved_angle) << "the sweep must move away from pi";
    }

    // Exactness is established once, at the worst point (delta = 0): the row must
    // still be the true derivative, not merely finite.  Checked against a central
    // finite difference of the analytic error, restricted to the left arm (the only
    // block the anchored row fills).
    {
        const NearPi np = buildNearPi(kin, 0.0);
        const dual_arm::JointConfig ql = armJoints(np.q, true);
        const Iso3 T_WL = kin.worldTgrasp(Arm::Left, ql);
        const Iso3 T_hat = dual_arm::anchoredObjectPose(T_WL, G_L);
        const Mat6 J_L = kin.jacobianBody(Arm::Left, ql);
        const Mat6 J_O = dual_arm::anchoredObjectErrorJacobian(J_L, G_L, T_hat, T_d);
        const double h = 1e-6;
        double worst = 0.0;
        for (int j = 0; j < 6; ++j) {
            dual_arm::JointConfig qp = ql;
            dual_arm::JointConfig qm = ql;
            qp[j] += h;
            qm[j] -= h;
            const Iso3 T_hat_p = dual_arm::anchoredObjectPose(kin.worldTgrasp(Arm::Left, qp), G_L);
            const Iso3 T_hat_m = dual_arm::anchoredObjectPose(kin.worldTgrasp(Arm::Left, qm), G_L);
            const Vec6 fd = (dual_arm::decoupledPoseError(T_hat_p, T_d) -
                             dual_arm::decoupledPoseError(T_hat_m, T_d)) /
                            (2.0 * h);
            worst = std::max(worst, (fd - J_O.col(j)).norm() / std::max(1.0, fd.norm()));
        }
        std::printf("  [oracle] worst FD column mismatch at pi: %.3e\n", worst);
        EXPECT_LE(worst, 1e-6) << "the anchored row is finite at pi but NOT the true derivative";
    }
}

// ------------------------------------------------------------------- S3-T11
TEST(NearPiC43S3, T11_LegacyObjectRowIsSilentlyWrongAtPi) {
    // Measured, not assumed.  The first version of this test asserted that the legacy
    // row JUMPS between delta=0 and delta=1e-6; it does not -- it is continuous to
    // 3e-16.  What actually happens is worse: the row is a smooth function that is
    // simply WRONG near pi, by 31 % at pi and 0 % two hundredths of a radian away.
    // So the contrast T10 needs is "exact everywhere" vs "wrong near pi", and this
    // test measures the error curve instead of guessing its shape.
    const dual_arm::DualArmKinematics kin;
    const Iso3 G_L = frozenGraspLeft();
    const Iso3 T_d = objectStar();

    const Vec3 dq_dir = Vec3(1.0, -0.7, 0.4).normalized();
    Q12 dq = Q12::Zero();
    for (int i = 0; i < 6; ++i) dq[i] = dq_dir[i % 3];

    // The legacy row claims  V^b_O = J_obj * dq.  Differentiate the object pose that
    // the legacy path itself defines (worldTobject) and compare: both sides are body
    // twists of the same object frame, so no convention bookkeeping is needed.
    auto legacy_twist_error = [&](const Q12& q) {
        const double h = 1e-6;
        const Vec6 v_fd =
            dual_arm::logSE3(kin.worldTobject(q - h * dq).inverse() * kin.worldTobject(q + h * dq)) /
            (2.0 * h);
        const Vec6 v_row = kin.objectJacobianBody(q) * dq;
        return (v_row - v_fd).norm() / v_fd.norm();
    };
    auto anchored_row = [&](const Q12& q) {
        const Iso3 T_hat =
            dual_arm::anchoredObjectPose(kin.worldTgrasp(Arm::Left, armJoints(q, true)), G_L);
        return dual_arm::anchoredObjectErrorJacobian(
            kin.jacobianBody(Arm::Left, armJoints(q, true)), G_L, T_hat, T_d);
    };

    std::printf("\n[ S3-T11 ] relative error of the legacy object row, and the anchored\n");
    std::printf("           row's invariance, over the same sweep\n");
    std::printf("  %-10s %-14s %-18s %-14s\n", "delta", "legacy err", "anchored row moved?",
                "legacy sigma_min");

    std::array<double, 5> deltas{1e-2, 1e-3, 1e-5, 1e-6, 0.0};
    std::array<double, 5> errs{};
    Mat6 anchored_first = Mat6::Zero();
    bool first = true;
    for (std::size_t i = 0; i < deltas.size(); ++i) {
        const NearPi np = buildNearPi(kin, deltas[i]);
        errs[i] = legacy_twist_error(np.q);
        const Mat6 row = anchored_row(np.q);
        if (first) {
            anchored_first = row;
            first = false;
        }
        const unsigned long long differ = row == anchored_first ? 0ULL : 1ULL;
        const double sigma =
            Eigen::JacobiSVD<Mat6x12>(kin.objectJacobianBody(np.q)).singularValues()(5);
        std::printf("  %-10.1e %-14.4e %-18s %-14.4e\n", deltas[i], errs[i], differ ? "YES" : "no",
                    sigma);
        EXPECT_TRUE(kin.objectJacobianBody(np.q).allFinite())
            << "a NaN would be a loud failure; the hazard is a finite wrong answer";
        EXPECT_EQ(differ, 0ULL) << "delta=" << deltas[i]
                                << ": the anchored row moved with the relative rotation";
    }

    const double err_pi = errs.back();
    const double err_far = errs.front();
    std::printf("  ==> legacy row error: %.4f %% at pi, %.4f %% at pi-1e-2; "
                "anchored row: invariant (bit-identical)\n", err_pi * 100.0, err_far * 100.0);

    EXPECT_LT(err_far, 1e-3) << "the legacy row must be right away from pi, or this contrast is "
                                 "not about pi at all";
    EXPECT_GT(err_pi, 0.10) << "the legacy row was supposed to be wrong at pi";
    // The shape of the curve is NOT monotone, and that is the real finding: the error
    // peaks at ~1.0 in the last microradian before pi (the legacy rate map is unusable
    // in the whole neighbourhood) and lands at 0.31 exactly AT pi, where the canonical
    // log of a pi rotation happens to make the formula "work".  So the hazard is an
    // entire neighbourhood of pi, not a single singular point -- and an assertion of
    // monotonicity would have hidden it.
    std::size_t worst = 0;
    for (std::size_t i = 1; i < errs.size(); ++i) {
        if (errs[i] > errs[worst]) worst = i;
    }
    EXPECT_GT(errs[worst], 0.90) << "the legacy row must be unusable near pi, not merely biased";
    EXPECT_LT(deltas[worst], 1e-4) << "the peak error must be within 1e-4 rad of pi";
    EXPECT_GT(errs[worst] / std::max(err_far, 1e-12), 1e3)
        << "the defect must be a pi effect, not a constant bias";
}

// ------------------------------------------------------------------- S3-T17
// The point of this test is the *gap between two verdicts*: the legacy midpoint
// guard says "the grasp is fine" while the legacy object FRAME is off by exactly
// half the grasp width and a half turn.  A passing guard therefore does not
// certify the object convention -- which is the whole reason S3 exists.
TEST(NearPiC43S3, T17_MidpointGuardPassesWhileItsObjectFrameIsWrong) {
    const dual_arm::DualArmKinematics kin;
    const Q12 q = legacy_golden::graspQ();
    const Iso3 G_L = frozenGraspLeft();
    Iso3 G_R = G_L;  // the mirror grasp: same rotation, x negated
    G_R.translation() = Vec3(-G_L.translation().x(), G_L.translation().y(), G_L.translation().z());
    const Iso3 T_WL = kin.worldTgrasp(Arm::Left, armJoints(q, true));
    const Iso3 T_WR = kin.worldTgrasp(Arm::Right, armJoints(q, false));

    const bool safe = dual_arm::isMidpointSafe(T_WL, T_WR);
    const std::string reason = dual_arm::midpointUnsafeReason(T_WL, T_WR);
    const Iso3 legacy = kin.worldTobject(q);
    const Iso3 star = objectStar();
    const double dp = (legacy.translation() - star.translation()).norm();
    const double dR = dual_arm::logSO3(star.linear().transpose() * legacy.linear()).norm();

    const Iso3 left = dual_arm::anchoredObjectPose(T_WL, G_L);
    const Iso3 right = dual_arm::anchoredObjectPose(T_WR, G_R);
    const double left_gap = dual_arm::decoupledPoseError(left, star).norm();
    const double right_gap = dual_arm::decoupledPoseError(right, star).norm();

    std::printf("\n[ S3-T17 ] midpoint guard: safe=%d reason=\"%s\"\n", safe ? 1 : 0,
                reason.c_str());
    std::printf("           legacy object frame vs T_WO_star: |dp|=%.6f m, |dR|=%.6f rad\n", dp,
                dR);
    std::printf("           anchored estimates vs T_WO_star: left=%.3e right=%.3e\n", left_gap,
                right_gap);

    EXPECT_TRUE(safe) << "the legacy guard is supposed to pass here -- that is the finding";
    EXPECT_TRUE(reason.empty());
    EXPECT_NEAR(dp, 0.025, 1e-9);
    EXPECT_NEAR(dR, kPi, 1e-9);
    EXPECT_LE(left_gap, 1e-10) << "the anchored estimate must BE the frozen object pose";
    EXPECT_LE(right_gap, 1e-10);
}
