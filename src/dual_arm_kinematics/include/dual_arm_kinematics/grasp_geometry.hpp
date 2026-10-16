// include/dual_arm_kinematics/grasp_geometry.hpp
// C4.3 S1 / D1 — parametric grasp geometry for dual-arm fixed-grasp transport.
//
// SCOPE
// -----
// This header only *describes* a grasp; it does not plan, solve IK or check
// collisions (those are S2/S3).  Everything here is deliberately independent of
// the M4 handover task: no real M4/M5 object dimension is hard-coded, and the
// object frame comes from the TASK (T_WO*), never from an average of the two
// grasp frames.
//
// FRAME / SYMBOL CONVENTION  (see C4.3_S1_坐标系契约.md)
// -----------------------------------------------------
//   W : world frame of the dual-arm model
//   O : object frame, given by the task as  T_WO,d(t)
//   L : left  TCP (left_grasp_frame)
//   R : right TCP (right_grasp_frame)
//
//   G_L = (T_WO*)^-1 * T_WL*     <-- object frame -> left  TCP, frozen at grasp time
//   G_R = (T_WO*)^-1 * T_WR*     <-- object frame -> right TCP
//
//   With a rigid grasp, T_WL = T_WO * G_L  and  T_WR = T_WO * G_R, hence the
//   relative pose is the constant
//       T_rel* = (T_WL)^-1 T_WR = (G_L)^-1 G_R.
//
// All twists in this package are [v; w] (linear first), body convention.
#pragma once

#include <string>
#include <vector>

#include <Eigen/Dense>

#include "dual_arm_kinematics/se3.hpp"

namespace dual_arm {

/// Parametric rectangular-box object + the frozen grasp configuration.
///
/// Deliberately a *description*, not a model file: the numbers are supplied by
/// the experiment and recorded per trial, as required by 正式执行规划 §4.1.
struct GraspGeometry {
    // ---- object body (box: size_xyz, mass, inertia about the object origin) ----
    Vec3 size_xyz = Vec3::Ones();
    double mass = 1.0;
    Mat3 inertia = Mat3::Identity();

    // ---- task-given grasp configuration, frozen at grasp time ----
    Iso3 world_T_object_star = Iso3::Identity();  ///< T_WO*  (task-given, NOT a midpoint)
    Iso3 object_T_left_tcp = Iso3::Identity();    ///< G_L
    Iso3 object_T_right_tcp = Iso3::Identity();   ///< G_R

    // ---- frame identifiers (informational + validated to be non-empty/distinct) ----
    std::string world_frame = "world";
    std::string object_frame = "object";
    std::string left_tcp_frame = "left_grasp_frame";
    std::string right_tcp_frame = "right_grasp_frame";

    /// Frozen relative grasp transform  T_rel* = (G_L)^-1 G_R.
    Iso3 leftTrightStar() const { return object_T_left_tcp.inverse() * object_T_right_tcp; }

    /// Empty string => valid.  Otherwise a human-readable reason.
    ///
    /// Validation NEVER repairs the input: an out-of-range or non-rigid value is
    /// reported so the caller can refuse the trial.  (正式执行规划 §4.1 /
    /// S0 §A7 T1: "非法输入必须被拒绝，不允许 clamp".)
    std::string validate() const;
};

/// True if `R` is orthonormal within `tol` and has det = +1.
inline bool isRotationMatrix(const Mat3& R, double tol = 1e-9) {
    if (!R.allFinite()) return false;
    if ((R.transpose() * R - Mat3::Identity()).cwiseAbs().maxCoeff() > tol) return false;
    return std::abs(R.determinant() - 1.0) <= 1e-6;
}

/// True if the 3x3 block of `T` is a valid rotation and the translation is finite.
inline bool isRigidTransform(const Iso3& T) {
    return isRotationMatrix(T.linear()) && T.translation().allFinite();
}

inline std::string GraspGeometry::validate() const {
    // ---- frame identifiers -------------------------------------------------
    if (world_frame.empty()) return "world_frame is empty";
    if (object_frame.empty()) return "object_frame is empty";
    if (left_tcp_frame.empty()) return "left_tcp_frame is empty";
    if (right_tcp_frame.empty()) return "right_tcp_frame is empty";
    if (object_frame == left_tcp_frame || object_frame == right_tcp_frame)
        return "object_frame collides with a TCP frame id";
    if (left_tcp_frame == right_tcp_frame)
        return "left_tcp_frame and right_tcp_frame are the same frame id";

    // ---- object body -------------------------------------------------------
    if (!size_xyz.allFinite()) return "size_xyz is not finite";
    if (size_xyz.minCoeff() <= 0.0) return "size_xyz must be strictly positive";
    if (!std::isfinite(mass) || mass <= 0.0) return "mass must be strictly positive";
    if (!inertia.allFinite()) return "inertia is not finite";
    if ((inertia - inertia.transpose()).cwiseAbs().maxCoeff() > 1e-12)
        return "inertia must be symmetric";
    Eigen::LLT<Mat3> llt(inertia);
    if (llt.info() != Eigen::Success) return "inertia must be positive definite";
    if ((llt.matrixL().toDenseMatrix().diagonal().array() <= 0.0).any())
        return "inertia must be positive definite";

    // ---- rigid transforms --------------------------------------------------
    if (!isRigidTransform(world_T_object_star))
        return "world_T_object_star is not a rigid transform (rotation or finite check)";
    if (!isRigidTransform(object_T_left_tcp))
        return "object_T_left_tcp is not a rigid transform";
    if (!isRigidTransform(object_T_right_tcp))
        return "object_T_right_tcp is not a rigid transform";

    // ---- degenerate grasp --------------------------------------------------
    // G_L == G_R means both TCPs sit at the same point of the object: the pair
    // then has no relative-motion authority and T_rel* is the identity, which
    // makes the C4.3 relative subtask vacuous.
    const double dG = (object_T_left_tcp.matrix() - object_T_right_tcp.matrix()).norm();
    if (dG <= 1e-9)
        return "object_T_left_tcp and object_T_right_tcp are identical (degenerate grasp)";

    return std::string();
}

}  // namespace dual_arm
