// test/testonly/legacy_midpoint_negative.hpp
// C4.3 S1 / T7 — TEST-ONLY negative implementation of the ORIGINAL symmetric
// geodesic midpoint object frame.
//
// WHY THIS FILE EXISTS
// --------------------
// To prove that the π regression test has real detection power we need an
// implementation that is KNOWN to be defective.  We must not manufacture one by
// editing production code (approved instruction: "不要为了验证测试有效性而修改
// 生产代码"), so the defective algorithm is re-implemented here, straight from its
// mathematical definition, completely independently of
// DualArmKinematics::worldTobject().
//
// The regression oracle used against it is the SAME finite-difference oracle that
// the production anchored interface is tested with.  If the oracle flags this
// independent negative, it would also flag a future regression of the same
// algorithm — which is exactly the property T7 must demonstrate.
//
// This header must only ever be included by tests.  It is kept under test/ so it
// is never installed.
#pragma once

#include "dual_arm_kinematics/dual_arm_kinematics.hpp"
#include "dual_arm_kinematics/se3.hpp"

namespace dual_arm {
namespace testonly {

/// The pre-C4.3 object frame, re-derived from its definition:
///   p_O = (p_L + p_R) / 2
///   R_O = R_L * exp( 0.5 * log( R_L^T R_R ) )
inline Iso3 legacyMidpointObjectPose(const DualArmKinematics& kin, const Q12& q12) {
    JointConfig q_left, q_right;
    DualArmKinematics::split(q12, &q_left, &q_right);
    const Iso3 world_T_left = kin.worldTgrasp(Arm::Left, q_left);
    const Iso3 world_T_right = kin.worldTgrasp(Arm::Right, q_right);

    Iso3 out = Iso3::Identity();
    out.translation() = 0.5 * (world_T_left.translation() + world_T_right.translation());
    const Vec3 xi = logSO3(world_T_left.linear().transpose() * world_T_right.linear());
    out.linear() = world_T_left.linear() * expSO3(0.5 * xi);
    return out;
}

/// A representative *exactly-π* arm pose: identical joint values on both arms.
/// The S622 bases are yaw-π apart, so R_L^T R_R = R_g^T Rz(π) R_g — a rotation by
/// exactly π about R_g^T z.
inline Q12 exactlyPiPose(const DualArmKinematics& kin) {
    (void)kin;
    JointConfig q;
    // A non-degenerate, mid-range arm posture (not a singularity).
    q << 0.0, -0.9, 0.9, -0.9, -0.6, 0.0;
    return DualArmKinematics::stack(q, q);
}

/// Test-only analytic Jacobian of legacyMidpointObjectPose, in the object frame.
/// Re-derives the standard midpoint identity
///   v_O = 0.5 (v_L + v_R),  w_O = (I - G) w_L + G w_R
/// without calling the production objectJacobianBody().
inline Mat6x12 legacyMidpointBodyJacobian(const DualArmKinematics& kin, const Q12& q12) {
    JointConfig q_left, q_right;
    DualArmKinematics::split(q12, &q_left, &q_right);
    const Iso3 world_T_left = kin.worldTgrasp(Arm::Left, q_left);
    const Iso3 world_T_right = kin.worldTgrasp(Arm::Right, q_right);
    const Iso3 world_T_object = legacyMidpointObjectPose(kin, q12);

    // World axes with the reference point at each GRASP ORIGIN (this is the
    // convention the old midpoint identity was derived in), then shift to the
    // object origin by the midpoint linear average.
    const Mat6 J_L = kin.jacobianWorldPoint(Arm::Left, q_left);
    const Mat6 J_R = kin.jacobianWorldPoint(Arm::Right, q_right);

    const Mat3 R_L = world_T_left.linear();
    const Mat3 R_R = world_T_right.linear();
    const Vec3 xi = logSO3(R_L.transpose() * R_R);
    const Mat3 G = R_L * leftJacobianSO3(0.5 * xi) * (0.5 * leftJacobianInverseSO3(xi)) * R_L.transpose();

    Mat6x12 J_world = Mat6x12::Zero();
    // Columns 0..5 are the LEFT arm joints, 6..11 the RIGHT arm joints.
    J_world.topRows<3>().leftCols<6>() = 0.5 * J_L.topRows<3>();
    J_world.topRows<3>().rightCols<6>() = 0.5 * J_R.topRows<3>();
    J_world.bottomRows<3>().leftCols<6>() = (Mat3::Identity() - G) * J_L.bottomRows<3>();
    J_world.bottomRows<3>().rightCols<6>() = G * J_R.bottomRows<3>();

    // world axes -> object axes
    const Mat3 R_O = world_T_object.linear();
    Iso3 world_T_object_inv = Iso3::Identity();
    world_T_object_inv.linear() = R_O.transpose();
    return adjoint(world_T_object_inv) * J_world;
}

}  // namespace testonly
}  // namespace dual_arm
