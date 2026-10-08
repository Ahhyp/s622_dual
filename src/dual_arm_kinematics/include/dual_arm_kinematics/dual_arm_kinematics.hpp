// include/dual_arm_kinematics/dual_arm_kinematics.hpp
// Step 0 deliverable: unified dual-arm kinematics built on the existing
// fairino_planning::DHKinematics (DH + analytical geometric Jacobian).
//
// Frame conventions
// -----------------
//  - "base"  : each arm's own base_link frame (left_base_link / right_base_link)
//  - "grasp" : left_grasp_frame / right_grasp_frame
//  - "world" : the world frame of the dual-arm URDF
//
//  DHKinematics::fkine / jacobian are expressed in each arm's *base* frame,
//  with the geometric Jacobian referenced to the grasp point:
//      d/dt [p_grasp]  = J_base * qdot    (axes = base, reference point = grasp)
//      d/dt [w]        = ...
//  This matches KDL's ChainJntToJacSolver (chain base frame), which is used as
//  an independent oracle in the tests.
//
//  Because the two bases have different orientations in world (left yaw = pi),
//  the relative Jacobian is assembled with SE(3) adjoints, never by naively
//  stacking [-J_left, J_right].

#pragma once

#include <array>

#include "dual_arm_kinematics/se3.hpp"
#include "fairino_planning_core/dh_kinematics.h"

namespace dual_arm {

using fairino_planning::DHKinematics;
using fairino_planning::DHParams;
using fairino_planning::Jacobian6d;
using fairino_planning::JointConfig;
using fairino_planning::ToolModel;
using fairino_planning::ToolParams;

using Q12 = Eigen::Matrix<double, 12, 1>;
using Mat6x12 = Eigen::Matrix<double, 6, 12>;

enum class Arm : int { Left = 0, Right = 1 };

/// Frame in which the relative twist / relative Jacobian is expressed.
///  - BodyR      : twist of  left_T_right  in the *right grasp* frame (body)
///  - LeftL      : same twist re-expressed in the left grasp frame
///  - WorldSpatial: same twist re-expressed as a spatial twist in world
enum class RelativeFrame : int { BodyR = 0, LeftL = 1, WorldSpatial = 2 };

class DualArmKinematics {
public:
    /// Default configuration: S622 DH params, gripper tool (+0.1168 m),
    /// URDF ground-truth base poses (left @ (0.35,0,0) yaw=pi, right @ (-0.35,0,0)).
    DualArmKinematics();

    DualArmKinematics(const Iso3& world_T_left_base, const Iso3& world_T_right_base);

    DualArmKinematics(const Iso3& world_T_left_base, const Iso3& world_T_right_base,
                      const DHParams& dh, const ToolParams& tool);

    static Iso3 defaultWorldTleftBase();
    static Iso3 defaultWorldTrightBase();

    const Iso3& worldTbase(Arm a) const { return base_[index(a)]; }
    const DHKinematics& dh(Arm a) const { return dh_[index(a)]; }

    // ---------------- single arm ----------------
    /// base_link -> grasp_frame (DH, gripper tool model)
    Iso3 baseTgrasp(Arm a, const JointConfig& q) const;

    /// world -> grasp_frame
    Iso3 worldTgrasp(Arm a, const JointConfig& q) const;

    /// Geometric Jacobian: axes = base, reference point = grasp origin,
    /// twist = [linear; angular]. Same convention as KDL's chain Jacobian.
    Jacobian6d jacobianBase(Arm a, const JointConfig& q) const;

    /// Body Jacobian: axes = grasp frame, reference point = grasp origin.
    Mat6 jacobianBody(Arm a, const JointConfig& q) const;

    /// Spatial Jacobian: axes = world, reference point = world origin.
    Mat6 spatialJacobianWorld(Arm a, const JointConfig& q) const;

    /// World axes, reference point = grasp origin ([v; w] in world axes).
    Mat6 jacobianWorldPoint(Arm a, const JointConfig& q) const;

    /// Intermediate DH frame i (0 = base, 1..6 = after joint i).  C2.5 uses
    /// this for the arm-arm clearance proxy (elbow / wrist origins).
    Iso3 baseTframe(Arm a, const JointConfig& q, int frame_index) const;
    Iso3 worldTframe(Arm a, const JointConfig& q, int frame_index) const;

    // ---------------- coordination / object frame (C2.4) ----------------
    /// SE(3) midpoint of the two grasp frames, expressed in world:
    ///   p_O = (p_L + p_R) / 2
    ///   R_O = R_L * exp( 0.5 * log(R_L^T R_R) )
    Iso3 worldTobject(const Q12& q12) const;

    /// Object Jacobian, axes = world, reference point = object origin.
    /// Exact:  v_O = 0.5 (v_L + v_R),
    ///         w_O = (I - G) w_L + G w_R,
    /// with G = R_L * J_l(xi/2) * 0.5 * J_l(xi)^-1 * R_L^T, xi = log(R_L^T R_R).
    Mat6x12 objectJacobianWorld(const Q12& q12) const;

    /// Object Jacobian with axes = object frame (reference point = object origin).
    Mat6x12 objectJacobianBody(const Q12& q12) const;

    /// Pose error twist  e = log( (world_T_object)^-1 * target ),  object frame.
    Vec6 objectError(const Q12& q12, const Iso3& world_T_object_target) const;

    /// First-order (small-error) error-rate Jacobian for objectError().
    Mat6x12 objectErrorJacobian(const Q12& q12, const Iso3& world_T_object_target) const;

    // ---------------- 12-DOF helpers ----------------
    static Q12 stack(const JointConfig& q_left, const JointConfig& q_right);
    static void split(const Q12& q12, JointConfig* q_left, JointConfig* q_right);

    // ---------------- relative pose / Jacobian ----------------
    /// left_T_right = (world_T_left_grasp)^-1 * (world_T_right_grasp)
    Iso3 leftTright(const Q12& q12) const;
    Iso3 leftTright(const JointConfig& q_left, const JointConfig& q_right) const;

    /// Relative Jacobian J_rel (6x12) such that  xi_rel = J_rel * qdot12,
    /// with xi_rel expressed in the requested frame.
    Mat6x12 relativeJacobian(const Q12& q12,
                             RelativeFrame frame = RelativeFrame::BodyR) const;

    /// Pose error twist  e = log( (left_T_right)^-1 * target ),  in right-grasp frame.
    Vec6 relativeError(const Q12& q12, const Iso3& left_T_right_target) const;

    /// First-order (small-error) map  edot ~= J_e * qdot12  consistent with
    /// relativeError().  Exact to first order; J_r^{SE3}(e) is taken as identity.
    Mat6x12 relativeErrorJacobian(const Q12& q12,
                                  const Iso3& left_T_right_target) const;

private:
    static int index(Arm a) { return static_cast<int>(a); }
    static Iso3 toIso(const Eigen::Matrix4d& T);

    std::array<Iso3, 2> base_;
    std::array<DHKinematics, 2> dh_;
};

}  // namespace dual_arm
