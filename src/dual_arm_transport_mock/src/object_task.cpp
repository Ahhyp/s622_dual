// C4.3 S3 -- W3: anchored object task implementation.
//
// Every expression here is a direct transcription of S1
// (dual_arm_kinematics/object_frame.hpp) and of C4.3_S3_实施方案.md §2.3.  There
// is no tuning constant, no approximation and nothing to fit: S3's object task
// is fully determined by (q, G_L, G_R, T_desired, v_ff).
#include "dual_arm_transport_mock/object_task.hpp"

#include <cmath>

#include "dual_arm_kinematics/object_frame.hpp"

namespace dual_arm_transport_mock {

namespace {

using dual_arm::Arm;
using dual_arm::Iso3;
using dual_arm::JointConfig;
using dual_arm::Mat6;
using dual_arm::Q12;
using dual_arm::Vec6;

/// A rigid transform is usable iff it is finite and its rotation block is
/// orthonormal with det +1 (1e-9 is far below any real kinematic error and far
/// above the round-off of composing four transforms).
bool isUsable(const Iso3& T) {
    if (!T.matrix().allFinite()) return false;
    const Eigen::Matrix3d RtR = T.linear().transpose() * T.linear();
    if ((RtR - Eigen::Matrix3d::Identity()).cwiseAbs().maxCoeff() > 1e-9) return false;
    return std::abs(T.linear().determinant() - 1.0) <= 1e-9;
}

}  // namespace

AnchorGap anchorGap(const Iso3& T_a, const Iso3& T_b) {
    AnchorGap g;
    g.pos_m = (T_a.translation() - T_b.translation()).norm();
    g.rot_rad = dual_arm::logSO3(T_a.linear().transpose() * T_b.linear()).norm();
    return g;
}

AnchoredObjectTaskBuilder::AnchoredObjectTaskBuilder(const dual_arm::DualArmKinematics& kin,
                                                     const Iso3& object_T_left_tcp,
                                                     const Iso3& object_T_right_tcp)
    : kin_(kin),
      left_grasp_(object_T_left_tcp),
      right_grasp_(object_T_right_tcp),
      left_T_right_grasp_(object_T_left_tcp.inverse() * object_T_right_tcp) {}

Iso3 AnchoredObjectTaskBuilder::leftEstimate(const Q12& q) const {
    JointConfig ql, qr;
    dual_arm::DualArmKinematics::split(q, &ql, &qr);
    (void)qr;
    return dual_arm::anchoredObjectPose(kin_.worldTgrasp(Arm::Left, ql), left_grasp_);
}

Iso3 AnchoredObjectTaskBuilder::rightEstimate(const Q12& q) const {
    JointConfig ql, qr;
    dual_arm::DualArmKinematics::split(q, &ql, &qr);
    (void)ql;
    return dual_arm::anchoredObjectPose(kin_.worldTgrasp(Arm::Right, qr), right_grasp_);
}

AnchoredObjectTask AnchoredObjectTaskBuilder::build(const Q12& q,
                                                    const Iso3& world_T_object_desired,
                                                    const Vec6& object_body_twist_ff) const {
    AnchoredObjectTask out;
    out.v_ff = object_body_twist_ff;

    out.inputs_ok = isUsable(world_T_object_desired) && q.allFinite() &&
                    object_body_twist_ff.allFinite();
    if (!out.inputs_ok) {
        // Leave J zero and e zero: the caller turns inputs_ok == false into an
        // explicit failure code instead of a silently wrong task.
        out.anchor_ok = false;
        return out;
    }

    JointConfig ql, qr;
    dual_arm::DualArmKinematics::split(q, &ql, &qr);

    const Iso3 T_WL = kin_.worldTgrasp(Arm::Left, ql);
    const Iso3 T_WR = kin_.worldTgrasp(Arm::Right, qr);

    out.T_hat_left = dual_arm::anchoredObjectPose(T_WL, left_grasp_);
    out.T_hat_right = dual_arm::anchoredObjectPose(T_WR, right_grasp_);
    out.anchor_gap = anchorGap(out.T_hat_left, out.T_hat_right);
    out.anchor_ok = out.anchor_gap.pos_m <= kAnchorPosGapTolM &&
                    out.anchor_gap.rot_rad <= kAnchorRotGapTolRad;

    // Legacy midpoint frame distance -- diagnostics only (§2.6 reason A).
    out.legacy_margin_rad = dual_arm::midpointMargin(T_WL, T_WR);

    // e_O = current - desired, decoupled [m; rad]  (§2.3.1(c)).
    out.e = dual_arm::decoupledPoseError(out.T_hat_left, world_T_object_desired);

    // J = d(e_O)/dq.  The right block is exactly zero: T_hat_{O,L} is built from
    // q_left alone, so e_O does not depend on q_right at all.  The right arm is
    // constrained by the relative-pose task; §2.5's identity (S3-T8) is what
    // audits that this division of labour really keeps the grasp closed.
    const Mat6 J_left =
        dual_arm::anchoredObjectErrorJacobian(kin_.jacobianBody(Arm::Left, ql), left_grasp_,
                                              out.T_hat_left, world_T_object_desired);
    out.J.setZero();
    out.J.leftCols<6>() = J_left;

    return out;
}

}  // namespace dual_arm_transport_mock
