// C4.3 S4 W0: the anchored object task (moved from dual_arm_transport_mock, numerics unchanged).
//
// WHY THIS EXISTS (C4.3_S3_实施方案.md §2.6): the legacy object task (the
// DualArmKinematics object-error / object-Jacobian entry points) lives in the
// *symmetric midpoint* object frame, which differs from the frozen S2 object frame by a
// constant 0.025 m / pi offset on the frozen configuration (measured, not
// argued).  Feeding it into the QP would re-interpret every frozen number in a
// different convention.  This file builds the object task from the S1 anchored
// formulation instead:
//
//     T_hat_{O,L}(q) = world_T_left_tcp(q) * G_L^-1        (anchored estimate)
//     e_O            = decoupledPoseError(T_hat, T_desired) (m / rad, decoupled)
//     J_O            = anchoredObjectErrorJacobian(...)      (exact rate map)
//
// The QP row derived from here is (see §2.3.1(c)(d), enforced by S3-T5f/T5g):
//     row.J  = J_O          (d(e_O)/dq, NOT negated)
//     row.e  = -e_O         (one minus sign: "how much is left")
//     row.body_twist_ff = +v_d
//
// Deliberately does NOT include the QP header: this package must stay unit
// testable without pulling in the solver.
#ifndef DUAL_ARM_OBJECT_TASK__ANCHORED_OBJECT_TASK_HPP_
#define DUAL_ARM_OBJECT_TASK__ANCHORED_OBJECT_TASK_HPP_

#include <string>

#include "dual_arm_kinematics/dual_arm_kinematics.hpp"
#include "dual_arm_kinematics/se3.hpp"

namespace dual_arm_object_task {

/// Tolerance on the disagreement between the two independent anchored estimates.
/// Zero when the closed chain holds exactly; grows only if the grasp really slips.
inline constexpr double kAnchorPosGapTolM = 1.0e-9;
inline constexpr double kAnchorRotGapTolRad = 1.0e-9;

/// Disagreement between two object-pose estimates.
struct AnchorGap {
    double pos_m = 0.0;    ///< || p_a - p_b ||
    double rot_rad = 0.0;  ///< || logSO3(R_a^T R_b) ||
};

/// Geometric (translation + rotation) disagreement.  Purely kinematic: no
/// Jacobian, no linearisation, no QP.  Used both as a diagnostic and as the
/// closed-chain judge of §2.5.
AnchorGap anchorGap(const dual_arm::Iso3& T_a, const dual_arm::Iso3& T_b);

/// The object task for ONE control cycle, in the SAME convention as
/// dual_arm_qp::ObjectTaskRow (see the SIGN CONTRACT in dual_arm_qp.hpp).
struct AnchoredObjectTask {
    /// d(e_O)/dq for the whole 12-vector q.  EXACTLY zero in the right block:
    /// T_hat_{O,L} depends on q_left only, so this is not an approximation.
    dual_arm::Mat6x12 J = dual_arm::Mat6x12::Zero();
    /// decoupledPoseError(T_hat_{O,L}, T_desired)  [m; rad]  = current - desired.
    /// NOTE: this is >e_O<, not "row.e".  The QP row takes its negation.
    dual_arm::Vec6 e = dual_arm::Vec6::Zero();
    /// Desired object body twist (feed-forward), passed through untouched.
    /// The QP adds dt*v_ff to the demanded object motion.
    dual_arm::Vec6 v_ff = dual_arm::Vec6::Zero();

    /// Independent anchored estimates (left drives J/e, right is the audit).
    dual_arm::Iso3 T_hat_left = dual_arm::Iso3::Identity();
    dual_arm::Iso3 T_hat_right = dual_arm::Iso3::Identity();

    /// pi - ||logSO3(R_L^T R_R)||: how far the legacy symmetric midpoint is from
    /// being a meaningful object frame (0 on the frozen grasp => it is pi away).
    double legacy_margin_rad = 0.0;

    /// True when q and world_T_object_desired were usable at all.  THIS is what
    /// `ObjectTaskRow::valid` must be set from: the row exists because the inputs
    /// are well-formed, not because the two anchors happen to agree right now.
    bool inputs_ok = false;

    /// True when the two anchors agree within kAnchor*GapTol (i.e. the closed
    /// chain holds).  Diagnostic only: the QP is never gated on it -- in a moving
    /// transport the gap is exactly the relative-task violation, so gating on
    /// 1e-9 would reject the row as soon as the arms track anything at all
    /// (found by S3-T6: anchor_ok went false at cycle 39 while |e_O| was 7e-6 m).
    bool anchor_ok = false;

    /// The gap that produced anchor_ok (kept so the cycle log does not recompute it).
    AnchorGap anchor_gap;
};

class AnchoredObjectTaskBuilder {
public:
    /// `object_T_left_tcp` / `object_T_right_tcp` are the frozen grasp transforms
    /// G_L / G_R from the S2 baseline.  They are constant for a run.
    AnchoredObjectTaskBuilder(const dual_arm::DualArmKinematics& kin,
                              const dual_arm::Iso3& object_T_left_tcp,
                              const dual_arm::Iso3& object_T_right_tcp);

    /// Per control cycle.  No heap allocation: everything is fixed-size Eigen.
    /// No exceptions: a non-finite / non-rigid input is reported via anchor_ok
    /// and a zero Jacobian rather than thrown (the loop maps it to a failure code).
    AnchoredObjectTask build(const dual_arm::Q12& q,
                             const dual_arm::Iso3& world_T_object_desired,
                             const dual_arm::Vec6& object_body_twist_ff = dual_arm::Vec6::Zero()) const;

    dual_arm::Iso3 leftEstimate(const dual_arm::Q12& q) const;
    dual_arm::Iso3 rightEstimate(const dual_arm::Q12& q) const;

    /// T_hat_{O,L}(q_grasp) must equal the frozen world_T_object_star (S3-T2).
    const dual_arm::Iso3& leftGrasp() const { return left_grasp_; }
    const dual_arm::Iso3& rightGrasp() const { return right_grasp_; }

    /// exp of the frozen relative grasp:  G_L^-1 G_R, constant for a run.
    const dual_arm::Iso3& leftTrightGrasp() const { return left_T_right_grasp_; }

private:
    const dual_arm::DualArmKinematics& kin_;
    dual_arm::Iso3 left_grasp_;       ///< G_L = object_T_left_tcp
    dual_arm::Iso3 right_grasp_;      ///< G_R = object_T_right_tcp
    dual_arm::Iso3 left_T_right_grasp_;  ///< G_L^-1 G_R (frozen relative grasp)
};

}  // namespace dual_arm_object_task

#endif  // DUAL_ARM_OBJECT_TASK__ANCHORED_OBJECT_TASK_HPP_
