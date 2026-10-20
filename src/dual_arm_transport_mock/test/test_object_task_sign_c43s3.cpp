// C4.3 S3 -- W2b tests: the object task's SIGN, SCALE and USEFULNESS.
//
// User m03513 revision point 2: proving "w_obj == 0 is bit identical" (S3-T5b)
// says nothing about the enabled direction.  These tests enable the object task
// with a NON-ZERO anchored error and a non-zero feed-forward and assert the
// closed-loop consequences.  They are the final judge of
// C4.3_S3_实施方案.md §2.3.1(c)(d):
//   row.J  = J_O = d(e_O)/dq            (motion Jacobian, NOT negated)
//   row.e  = -e_O                       (one minus sign: "how much is left")
//   row.body_twist_ff = +v_d            (the same minus sign, applied to ff)
//
// S3-T5f -- direction of the proportional term (d(e_O) = -alpha*e_O)
// S3-T5g -- direction of the feed-forward term (+v_d)
// S3-T5h -- the object task is toothless unless w_slack_obj >= 1e3
#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <array>
#include <cmath>
#include <string>

#include "dual_arm_kinematics/dual_arm_kinematics.hpp"
#include "dual_arm_kinematics/object_frame.hpp"
#include "dual_arm_kinematics/se3.hpp"
#include "dual_arm_qp_core/dual_arm_qp.hpp"

namespace {

using dual_arm::Arm;
using dual_arm::DualArmKinematics;
using dual_arm::Iso3;
using dual_arm::JointConfig;
using dual_arm::Mat6;
using dual_arm::Mat6x12;
using dual_arm::Q12;
using dual_arm::Vec6;
using dual_arm_qp::DualArmQp;
using dual_arm_qp::DualArmQpParams;
using dual_arm_qp::DualArmQpResult;
using dual_arm_qp::ObjectTaskRow;
using dual_arm_qp::Vec12;

constexpr std::array<double, 12> kGraspQ = {
    2.795036909814,  -1.706950857914, 1.279963524342,  -1.143808993223,
    -1.570796326795, -0.346555743776, 0.346555743776,  -1.434641795676,
    -1.279963524342, -1.997783660367, 1.570796326795,  -2.795036909814};

Q12 graspQ() {
    Q12 q;
    for (int i = 0; i < 12; ++i) q[i] = kGraspQ[static_cast<std::size_t>(i)];
    return q;
}

/// S2 frozen grasp transform G_L = object_T_left_tcp (transport_baseline.yaml):
///   R = [[0,1,0],[1,0,0],[0,0,-1]],  p = (+0.0497, 0, +0.025)
Iso3 graspTransformLeft() {
    Iso3 G = Iso3::Identity();
    Eigen::Matrix3d R;
    R << 0, 1, 0, 1, 0, 0, 0, 0, -1;
    G.linear() = R;
    G.translation() = Eigen::Vector3d(0.0497, 0.0, 0.025);
    return G;
}

/// S2 frozen object pose: world_T_object_star = (I3, (0, 0, 0.275)).
Iso3 objectPoseStar() {
    Iso3 T = Iso3::Identity();
    T.translation() = Eigen::Vector3d(0.0, 0.0, 0.275);
    return T;
}

/// Everything the object task needs at one configuration, built ONLY from the
/// S1 public API (no legacy object-frame calls anywhere).
struct Anchored {
    Vec6 e_O;      ///< decoupledPoseError(T_hat, T_d)  = current - desired
    Mat6x12 J_O;   ///< d(e_O)/dq  (left block; right block zero: single anchor)
};

Anchored anchoredAt(const DualArmKinematics& kin, const Q12& q, const Iso3& T_desired) {
    JointConfig ql, qr;
    DualArmKinematics::split(q, &ql, &qr);
    (void)qr;
    const Iso3 G_L = graspTransformLeft();
    const Iso3 T_hat = dual_arm::anchoredObjectPose(kin.worldTgrasp(Arm::Left, ql), G_L);
    Anchored a;
    a.e_O = dual_arm::decoupledPoseError(T_hat, T_desired);
    a.J_O.setZero();
    a.J_O.leftCols<6>() =
        dual_arm::anchoredObjectErrorJacobian(kin.jacobianBody(Arm::Left, ql), G_L, T_hat,
                                              T_desired);
    return a;
}

/// The convention under test: row.e = -e_O, row.body_twist_ff = +v_d.
ObjectTaskRow makeRow(const Anchored& a, const Vec6& v_d, bool flip_e_sign) {
    ObjectTaskRow row;
    row.J = a.J_O;
    row.e = (flip_e_sign ? 1.0 : -1.0) * a.e_O;
    row.body_twist_ff = v_d;
    row.valid = true;
    return row;
}

DualArmQpParams makeParams(double w_obj, double w_slack_obj) {
    DualArmQpParams p;
    dual_arm_qp::defaultS622Limits(&p.q_min, &p.q_max, &p.v_max);
    p.w_obj = w_obj;
    p.w_slack_obj = w_slack_obj;
    return p;
}

/// A desired pose with a clearly non-zero object error (about 2 cm / 3 deg).
Iso3 displacedDesiredPose() {
    Vec6 xi;
    xi << 0.020, -0.010, 0.015, 0.0, 0.0, 0.05;
    return objectPoseStar() * dual_arm::expSE3(xi);
}

/// Held fixed: the relative-pose target is the current relative pose, so the
/// relative task contributes a restoring term without biasing the object test.
Iso3 holdRelative(const DualArmKinematics& kin, const Q12& q) { return kin.leftTright(q); }

// ---------------------------------------------------------------------------
// S3-T5f -- proportional sign
// ---------------------------------------------------------------------------
TEST(ObjectTaskSignC43S3, T5f_AnchoredObjectTaskSignIsClosedLoopCorrect) {
    const DualArmKinematics kin;
    const Q12 q = graspQ();
    const Iso3 T_d = displacedDesiredPose();
    const Anchored a = anchoredAt(kin, q, T_d);
    ASSERT_GT(a.e_O.norm(), 1e-3) << "test needs a non-zero object error";

    DualArmQp qp(kin, makeParams(1.0, 1e3));
    const DualArmQpResult r = qp.solve(q, holdRelative(kin, q), makeRow(a, Vec6::Zero(), false));
    ASSERT_TRUE(r.converged);

    const Vec6 de = a.J_O * r.dq;  // predicted change of e_O over one cycle
    // d(e_O) = -alpha*e_O  <=>  e_O . de < 0  and  |e_O + de| < |e_O|
    EXPECT_LT(a.e_O.dot(de), 0.0)
        << "row.e = -e_O must REDUCE the error; got d(e_O) = " << de.transpose()
        << " for e_O = " << a.e_O.transpose();
    EXPECT_LT((a.e_O + de).norm(), a.e_O.norm());

    // The beginner's mistake: feed +e_O.  This MUST be detectable by this very
    // assertion, so it can never silently return to the code (the draft of
    // §2.3.1(c) argued exactly that, and it was wrong).
    const DualArmQpResult r_wrong =
        DualArmQp(kin, makeParams(1.0, 1e3)).solve(q, holdRelative(kin, q),
                                                   makeRow(a, Vec6::Zero(), true));
    ASSERT_TRUE(r_wrong.converged);
    const Vec6 de_wrong = a.J_O * r_wrong.dq;
    EXPECT_GT(a.e_O.dot(de_wrong), 0.0)
        << "row.e = +e_O did NOT diverge -- the test is too weak to judge the sign";

    // 50-cycle closed loop: ||e_O|| must be non-increasing at every step.
    Q12 qk = q;
    double prev = a.e_O.norm();
    for (int k = 0; k < 50; ++k) {
        const Anchored ak = anchoredAt(kin, qk, T_d);
        const double nk = ak.e_O.norm();
        EXPECT_LE(nk, prev + 1e-12) << "step " << k << ": ||e_O|| grew from " << prev << " to "
                                    << nk;
        const DualArmQpResult rk =
            qp.solve(qk, holdRelative(kin, qk), makeRow(ak, Vec6::Zero(), false));
        ASSERT_TRUE(rk.converged) << "step " << k;
        qk += rk.dq;
        prev = nk;
    }
    EXPECT_LT(anchoredAt(kin, qk, T_d).e_O.norm(), 0.05 * a.e_O.norm())
        << "50 cycles should have removed most of the error";
}

// ---------------------------------------------------------------------------
// S3-T5g -- feed-forward sign
// ---------------------------------------------------------------------------
TEST(ObjectTaskSignC43S3, T5g_FeedforwardSignMatchesTheDesiredObjectTwist) {
    const DualArmKinematics kin;
    const Q12 q = graspQ();
    // Pure feed-forward: e_O is deliberately zeroed through the row, so the only
    // object-task input is v_d.  (J_O is still the real anchored Jacobian.)
    const Anchored a = anchoredAt(kin, q, objectPoseStar());

    Vec6 v_d;
    v_d << 0.010, -0.004, 0.015, 0.0, 0.0, 0.02;

    auto solveWith = [&](const Vec6& ff) {
        DualArmQp qp(kin, makeParams(1.0, 1e3));
        ObjectTaskRow row;
        row.J = a.J_O;
        row.e = Vec6::Zero();  // no proportional demand
        row.body_twist_ff = ff;
        row.valid = true;
        return qp.solve(q, holdRelative(kin, q), row);
    };

    const DualArmQpResult r_pos = solveWith(v_d);
    const DualArmQpResult r_neg = solveWith(-v_d);
    ASSERT_TRUE(r_pos.converged);
    ASSERT_TRUE(r_neg.converged);
    const Vec6 d_pos = a.J_O * r_pos.dq;
    const Vec6 d_neg = a.J_O * r_neg.dq;

    // (1) the along-v_d component flips sign
    EXPECT_GT(d_pos.dot(v_d), 0.0) << "ff = +v_d must move the object along +v_d";
    EXPECT_LT(d_neg.dot(v_d), 0.0) << "ff = -v_d must move the object along -v_d";

    // (2) linear in ||v_d||: doubling v_d doubles the produced twist component
    const DualArmQpResult r_dbl = solveWith(2.0 * v_d);
    ASSERT_TRUE(r_dbl.converged);
    const double c1 = (a.J_O * r_dbl.dq).dot(v_d) / v_d.squaredNorm();
    const double c0 = d_pos.dot(v_d) / v_d.squaredNorm();
    ASSERT_GT(std::abs(c0), 1e-9);
    EXPECT_NEAR(c1 / c0, 2.0, 2e-9) << "feed-forward is not linear in v_d";

    // (3) the WRONG convention (ff = -v_d paired with row.e = -e_O) is caught here
    const DualArmQpResult r_bad = solveWith(-v_d);
    EXPECT_LE((a.J_O * r_bad.dq).dot(v_d), 0.0)
        << "this assertion must distinguish +v_d from -v_d (it does: (1)) -- "
        << "if it ever stops doing so, T5g is vacuous";
}

// ---------------------------------------------------------------------------
// S3-T5h -- the object task is toothless without a large slack penalty
// ---------------------------------------------------------------------------
TEST(ObjectTaskSignC43S3, T5h_ObjectTaskIsToothlessWithoutSlackPenalty) {
    const DualArmKinematics kin;
    const Q12 q = graspQ();
    const Iso3 T_d = displacedDesiredPose();
    const Anchored a = anchoredAt(kin, q, T_d);
    ASSERT_GT(a.e_O.norm(), 1e-3);

    // Pure object task: the arm is not holding anything, so s_obj can absorb the
    // whole demand unless w_slack_obj is large.  This is exactly the trap the
    // C2.4b default (1e-6) sets up, and it is why w_obj > 0 requires >= 1e3.
    auto solveWith = [&](double w_obj, double w_slack_obj) {
        DualArmQp qp(kin, makeParams(w_obj, w_slack_obj));
        return qp.solve(q, holdRelative(kin, q), makeRow(a, Vec6::Zero(), false));
    };

    const DualArmQpResult r_off = solveWith(0.0, 1e-6);
    const DualArmQpResult r_toothless = solveWith(1.0, 1e-6);
    const DualArmQpResult r_real = solveWith(1.0, 1e3);
    ASSERT_TRUE(r_off.converged);
    ASSERT_TRUE(r_toothless.converged);
    ASSERT_TRUE(r_real.converged);

    const double gap_toothless = (r_toothless.dq - r_off.dq).cwiseAbs().maxCoeff();
    const double gap_real = (r_real.dq - r_off.dq).cwiseAbs().maxCoeff();

    // The leakage is NOT exactly zero: the effective object-task weight is
    // w_obj*w_slack_obj/(w_obj+w_slack_obj) ~ 1e-6, so a demand of O(1e-1)
    // leaks ~1e-7 into dq (measured 1.76e-7 at q_grasp).  The decisive,
    // scale-free statement is therefore "at least three orders of magnitude
    // weaker than the properly penalised case".
    EXPECT_GT(gap_real, 1e-3)
        << "w_obj=1 with w_slack_obj=1e3 changed dq by only " << gap_real
        << " -- the object task would be toothless, so T5f/T5g would be meaningless";
    EXPECT_LT(gap_toothless, 1e-5)
        << "w_obj=1 with w_slack_obj=1e-6 still moved dq by " << gap_toothless
        << " (absolute) -- s_obj should absorb essentially everything";
    EXPECT_LT(gap_toothless, 1e-3 * gap_real)
        << "w_slack_obj=1e-6 is only " << (gap_real / gap_toothless)
        << "x weaker than w_slack_obj=1e3; it must be >= 1e3x weaker "
        << "(toothless=" << gap_toothless << ", real=" << gap_real << ")";
}

}  // namespace
