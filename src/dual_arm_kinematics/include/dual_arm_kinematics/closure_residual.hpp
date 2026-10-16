// include/dual_arm_kinematics/closure_residual.hpp
// C4.3 S1 / D3 — SE(3) closure residual between the two ANCHORED object estimates.
//
// ANTI-CHEAT RULE (正式执行规划 §3.2)
// -----------------------------------
// The residual must be computed from BOTH arms independently, never from the
// displayed/attached object pose:
//
//     e_closure = log( (T_hat_{O,L})^-1 * T_hat_{O,R} )      T_hat from each arm
//
// Translation and rotation are reported SEPARATELY (metres / radians) — never as
// one mixed scalar — and the raw frame-independent position gap ||p_R - p_L|| is
// reported alongside so a reader can tell a rigid-grasp violation from a
// frame-convention mistake.
//
// WHY THIS IS pi-SAFE
// -------------------
// The pi defect in DualArmKinematics::worldTobject() comes from *averaging* two
// grasp rotations that are ~pi apart.  Here the two quantities being compared are
// two independent estimates of the SAME object pose, so they agree to machine
// precision whenever the grasp is rigid — the arm-relative rotation being pi has
// no effect at all.  Verified: at the mirror configuration (||log(R_L^T R_R)|| is
// exactly pi) the residual between the two anchors is 0.
//
// RESIDUAL CONVENTION
// -------------------
// Decoupled, translation first:
//     t = R_L^T ( p_R - p_L )          [m]   position mismatch in the left object frame
//     phi = logSO3( R_L^T R_R )        [rad] orientation mismatch, same frame
// so `e = [t; phi]`, and with V^b = [v; w] body twists of the two anchored poses:
//     tdot   = -v_L + skew(t) w_L + R_c v_R
//     phidot = JlSO3(phi)^-1 ( R_c w_R - w_L ) ,   R_c = R_L^T R_R
// (R_c w_R is the right arm's angular velocity re-expressed in the left object
// frame, so the bracket is the relative angular velocity in that frame; the
// spatial-form left Jacobian inverse is the correct rate map for phi = log(R_c).)
// This is the exact rate map implemented by closureErrorJacobian().
#pragma once

#include <cmath>

#include <Eigen/Dense>

#include "dual_arm_kinematics/dual_arm_kinematics.hpp"
#include "dual_arm_kinematics/se3.hpp"

namespace dual_arm {

/// Decomposed closure residual between two independently anchored object poses.
struct ClosureResidual {
    Vec3 translation = Vec3::Zero();  ///< R_L^T (p_R - p_L)      [m]
    Vec3 rotation = Vec3::Zero();     ///< logSO3(R_L^T R_R)      [rad]
    double translation_m = 0.0;       ///< ||translation||        [m]
    double rotation_rad = 0.0;        ///< ||rotation||           [rad]
    double position_gap_m = 0.0;      ///< ||p_R - p_L||  (frame independent) [m]
    /// True when ||rotation|| sits at pi, where the rotation-vector DIRECTION is
    /// intrinsically ambiguous.  The magnitude is still meaningful; callers that
    /// need the direction must treat this as "not identifiable".
    bool rotation_direction_ambiguous = false;

    /// Combined 6-vector [translation; rotation] for logging / least squares.
    Vec6 xi() const {
        Vec6 e;
        e.head<3>() = translation;
        e.tail<3>() = rotation;
        return e;
    }
};

inline ClosureResidual closureResidual(const Iso3& object_L, const Iso3& object_R) {
    ClosureResidual r;
    const Mat3 R_c = object_L.linear().transpose() * object_R.linear();
    r.translation = object_L.linear().transpose() * (object_R.translation() - object_L.translation());
    r.rotation = logSO3(R_c);
    r.translation_m = r.translation.norm();
    r.rotation_rad = r.rotation.norm();
    r.position_gap_m = (object_R.translation() - object_L.translation()).norm();
    r.rotation_direction_ambiguous = (M_PI - r.rotation_rad) < 1e-9;
    return r;
}

/// EXACT rate map (6x12) such that  d/dt closureResidual(...) = J * [qdot_L; qdot_R].
///
/// `object_twist_jacobian_L/R` are the 6x6 anchored object twist Jacobians from
/// object_frame.hpp (anchoredObjectTwistJacobian), i.e. the maps
/// qdot_arm -> body twist of that arm's anchored object pose.
inline Mat6x12 closureErrorJacobian(const Iso3& object_L, const Iso3& object_R,
                                    const Mat6& object_twist_jacobian_L,
                                    const Mat6& object_twist_jacobian_R) {
    const Mat3 R_c = object_L.linear().transpose() * object_R.linear();
    const Vec3 t = object_L.linear().transpose() * (object_R.translation() - object_L.translation());
    const Mat3 Jl_inv = leftJacobianInverseSO3(logSO3(R_c));

    // 6x6 map from [v_L; w_L] to [tdot; phidot]
    Mat6 A = Mat6::Zero();
    A.topLeftCorner<3, 3>() = -Mat3::Identity();
    A.topRightCorner<3, 3>() = skew(t);
    A.bottomRightCorner<3, 3>() = -Jl_inv;

    // 6x6 map from [v_R; w_R] to [tdot; phidot]
    Mat6 B = Mat6::Zero();
    B.topLeftCorner<3, 3>() = R_c;
    B.bottomRightCorner<3, 3>() = Jl_inv * R_c;

    Mat6x12 J = Mat6x12::Zero();
    J.leftCols<6>() = A * object_twist_jacobian_L;
    J.rightCols<6>() = B * object_twist_jacobian_R;
    return J;
}

}  // namespace dual_arm
