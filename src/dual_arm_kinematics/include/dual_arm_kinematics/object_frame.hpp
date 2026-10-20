// include/dual_arm_kinematics/object_frame.hpp
// C4.3 S1 / D4 — task-given object frame, per-arm ANCHORED object pose, and the
// exact anchored object Jacobians.  This is the pi-safe replacement for the
// symmetric geodesic midpoint used by DualArmKinematics::worldTobject().
//
// WHY THIS EXISTS  (C4.3 S0 report §A14.1)
// ----------------------------------------
// worldTobject() defines R_O = R_L * expSO3( 0.5 * logSO3( R_L^T R_R ) ).  Two
// arms holding one rigid object from opposite sides have R_L^T R_R ~= pi, and at
// exactly pi that expression has TWO equally valid values (R_L*exp(+xi/2) and
// R_L*exp(-xi/2), which differ by pi).  Measured with a finite-difference oracle
// the resulting object Jacobian is wrong by ~460% there, and the error grows
// continuously as ||xi|| -> pi.  See test_object_jacobian_pi.cpp for the
// reproduction and test/legacy_midpoint_negative.hpp for the test-only negative
// implementation that proves the regression test can actually fail.
//
// THE REPLACEMENT
// ---------------
// Anchor the object on ONE arm and carry the grasp transform as a constant:
//
//     T_hat_{O,L}(q_L) = T_WL(q_L) * (G_L)^-1          (G_L = ^O T_L, frozen)
//
// There is no rotation log, no average and no midpoint anywhere in this map, so
// it is smooth and exact for ANY relative arm rotation, pi included.
//
// Its body twist is exactly
//
//     V^b_{T_hat} = Ad(G_L) * V^b_{WL},
//
// because  d/dt (T_WL * C) = T_WL * [V^b_WL]^ * C = T_hat * [Ad(C^-1) V^b_WL]^
// with C = (G_L)^-1.  With V^b_WL = J_body(q_L) * qdot_L this gives the EXACT
// velocity-level anchored Jacobian
//
//     J^twist_{O,L}(q_L) = Ad(G_L) * J_body(q_L).        (6x6, exact everywhere)
//
// POSE ERROR AND ITS EXACT RATE MAP
// ---------------------------------
// The pose error is DECOUPLED — translation and rotation are reported in their
// own units (m and rad), as required by 正式执行规划 §3.2:
//
//     e = [ R_d^T * ( p_hat - p_d ) ; logSO3( R_d^T * R_hat ) ]
//
// Its exact rate map w.r.t. V^b_{T_hat} = [v_hat; w_hat] is
//
//     edot = A(phi, R_hat, R_d) * V^b_{T_hat},
//     A    = [[ R_d^T R_hat ,        0        ],
//             [      0      , JrSO3(phi)^-1 ]].
//
// Derivation: p_hat = R_hat v_hat (body twist convention) so
// d/dt (R_d^T (p_hat - p_d)) = R_d^T R_hat v_hat; and with R_c = R_d^T R_hat,
// R_c^T dR_c/dt = [w_hat - R_c^T (R_d^T R_hat... )] reduces to
// phi_dot = JrSO3(phi)^-1 * w_hat.
//
// Only the SO(3) right-Jacobian inverse is needed, and that one is already
// implemented and verified in se3.hpp — no SE(3) Jacobian is involved, which is
// why this formulation stays exact at the arm-relative pi singularity.
//
// Twist convention: [v; w], linear first, body (moving-frame) axes.
#pragma once

#include <cmath>
#include <string>

#include <Eigen/Dense>

#include "dual_arm_kinematics/se3.hpp"

namespace dual_arm {

/// Minimum allowed margin `pi - ||logSO3(R_L^T R_R)||` for the LEGACY symmetric
/// midpoint interface to be considered trustworthy.
///
/// Chosen from the measured truncation behaviour (C4.3 S0 §A14.1 evidence 3/4):
/// the relative finite-difference error of objectJacobian* exceeds 1e-4 once
/// `pi - ||xi||` drops below ~1.5e-3 rad (~0.086 deg).  This guard applies ONLY
/// to the legacy interface; the anchored interface below is exact at margin 0
/// and must never be rejected because of it.
constexpr double kMidpointSafeMinMarginRad = 1.5e-3;

/// `||logSO3(R_L^T R_R)||` in [0, pi] — the geodesic arm-relative rotation angle.
inline double relativeRotationAngle(const Iso3& world_T_left_tcp,
                                    const Iso3& world_T_right_tcp) {
    return logSO3(world_T_left_tcp.linear().transpose() * world_T_right_tcp.linear()).norm();
}

/// Margin to the legacy midpoint singularity: `pi - relativeRotationAngle`.
/// Negative values (only possible through rounding) mean "effectively at pi".
inline double midpointMargin(const Iso3& world_T_left_tcp, const Iso3& world_T_right_tcp) {
    return M_PI - relativeRotationAngle(world_T_left_tcp, world_T_right_tcp);
}

/// Whether the LEGACY symmetric-midpoint interface (worldTobject /
/// objectJacobianWorld / objectJacobianBody) is trustworthy for this arm pose.
///
/// This is a *guard for the old interface*, not a restriction on C4.3: the
/// anchored interface below is exact at the singularity and must be used instead.
/// Callers of the legacy interface should treat `false` as "refuse / report
/// failure", never as "use the value anyway" — a silent wrong value is exactly
/// what C4.3 S0 flagged.
inline bool isMidpointSafe(const Iso3& world_T_left_tcp, const Iso3& world_T_right_tcp) {
    return midpointMargin(world_T_left_tcp, world_T_right_tcp) > kMidpointSafeMinMarginRad;
}

/// Anchored object pose estimate:  T_hat = world_T_tcp * (object_T_tcp)^-1.
///
/// `object_T_tcp` is the constant frozen grasp transform G (object frame -> TCP),
/// so this returns T_WO when the arm is exactly at the grasp configuration and
/// tracks the object rigidly afterwards.  Exact for any arm-relative rotation.
inline Iso3 anchoredObjectPose(const Iso3& world_T_tcp, const Iso3& object_T_tcp) {
    return world_T_tcp * object_T_tcp.inverse();
}

/// EXACT velocity-level anchored object Jacobian (6x6):
/// maps qdot of one arm to the body twist of the anchored object pose.
/// `jacobian_body` is that arm's DualArmKinematics::jacobianBody().
inline Mat6 anchoredObjectTwistJacobian(const Mat6& jacobian_body, const Iso3& object_T_tcp) {
    return adjoint(object_T_tcp) * jacobian_body;
}

/// Decoupled pose error, translation first:  [ R_d^T (p_hat - p_d) ; logSO3(R_d^T R_hat) ].
/// Units: metres for the first three entries, radians for the last three.
inline Vec6 decoupledPoseError(const Iso3& T_hat, const Iso3& T_desired) {
    Vec6 e;
    e.head<3>() = T_desired.linear().transpose() * (T_hat.translation() - T_desired.translation());
    e.tail<3>() = logSO3(T_desired.linear().transpose() * T_hat.linear());
    return e;
}

/// EXACT rate map A such that  d/dt decoupledPoseError(T_hat, T_desired) = A * V^b_{T_hat}.
inline Mat6 decoupledErrorRateMap(const Iso3& T_hat, const Iso3& T_desired) {
    Mat6 A = Mat6::Zero();
    const Mat3 R_hat_in_des = T_desired.linear().transpose() * T_hat.linear();
    A.topLeftCorner<3, 3>() = R_hat_in_des;
    A.bottomRightCorner<3, 3>() = rightJacobianInverseSO3(logSO3(R_hat_in_des));
    return A;
}

/// Anchored object error-rate Jacobian (6x6):
///   d/dt decoupledPoseError(T_hat_{O,arm}, T_desired) = J * qdot_arm.
///
/// Composed as  A(e) * Ad(G) * J_body(q).  No small-error approximation is taken
/// anywhere, and it is exact for ANY relative arm rotation, including pi -- the
/// arm-relative rotation never enters this expression.
///
/// BOUNDARY (measured, not assumed -- see C4.3 S3-T21).  The object-attitude error
/// does enter, through logSO3(R_d^T R_hat).  A(e) itself stays finite and
/// well-conditioned right up to pi (cond ~1.57, sigma_min ~1.0), but the ERROR
/// VECTOR it differentiates stops being differentiable there: as the object
/// attitude error approaches pi the finite-difference agreement degrades to
///   pi - |e_rot| = 1e-2 -> 1.3e-05 rel,  1e-3 -> 4.0e-03,  1e-4 -> 0.52,  1e-5 -> 1.14,
/// i.e. the analytic row is no longer the derivative of the returned error.  So the
/// honest statement is "exact at any arm-relative rotation, and exact in the OBJECT
/// error only away from |e_rot| ~ pi"; a caller must refuse inside that band rather
/// than trust the row.  MockTransportLoop does exactly that
/// (obj_rot_error_refuse_rad).
inline Mat6 anchoredObjectErrorJacobian(const Mat6& jacobian_body, const Iso3& object_T_tcp,
                                        const Iso3& T_hat, const Iso3& T_desired) {
    return decoupledErrorRateMap(T_hat, T_desired) * anchoredObjectTwistJacobian(jacobian_body,
                                                                                 object_T_tcp);
}

/// Human-readable reason why the legacy midpoint interface must not be used for
/// this pose, or an empty string when it is safe.
inline std::string midpointUnsafeReason(const Iso3& world_T_left_tcp,
                                        const Iso3& world_T_right_tcp) {
    const double margin = midpointMargin(world_T_left_tcp, world_T_right_tcp);
    if (margin > kMidpointSafeMinMarginRad) return std::string();
    return "legacy symmetric midpoint is ill-conditioned: pi - ||log(R_L^T R_R)|| = " +
           std::to_string(margin) + " <= " + std::to_string(kMidpointSafeMinMarginRad);
}

}  // namespace dual_arm
