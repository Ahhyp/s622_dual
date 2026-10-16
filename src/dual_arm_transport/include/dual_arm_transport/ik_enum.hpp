// include/dual_arm_transport/ik_enum.hpp
// C4.3 S2 / W2 -- full IK branch enumeration for ONE frozen dual-end grasp.
//
// The grasp fixes BOTH TCP poses completely (T_WL = T_WO* G_L, T_WR = T_WO* G_R),
// so "IK multi-solution combination screening" (user m02331 执行要求) means the
// CARTESIAN PRODUCT of the two arms' surviving analytic branches -- never "take
// the first solution per arm".
//
// The IK itself is REUSED from `dual_arm_pose_optimization::IkLayer` (user m02331
// D6: duplicating an IK algorithm is forbidden).  The verification below is
// deliberately INDEPENDENT of it: every returned branch is re-checked with this
// package's own `dual_arm::DualArmKinematics::worldTgrasp` FK, so a branch that
// only satisfies fairino's internal FK check is still caught.
#pragma once

#include <string>
#include <vector>

#include <Eigen/Dense>

#include "dual_arm_kinematics/dual_arm_kinematics.hpp"
#include "dual_arm_transport/grasp_config.hpp"

namespace dual_arm_transport {

/// One arm's raw branch enumeration for its TCP target.
struct BranchSet {
  std::string arm;  ///< "left" | "right"
  bool ok = false;  ///< the solver produced at least one solution

  dual_arm::Iso3 target_world = dual_arm::Iso3::Identity();
  dual_arm::Iso3 target_base = dual_arm::Iso3::Identity();

  int total_branches = 0;
  int survive_q1 = 0;
  int survive_q5 = 0;
  int survive_q23 = 0;
  int survive_fk_verify = 0;
  int survive_unique = 0;
  int survive_joint_limits = 0;

  std::string failure_stage{"none"};
  std::string failure_code{"none"};

  std::vector<dual_arm::JointConfig> solutions;

  // ---- independent re-verification (NOT the solver's own FK check) ----
  /// max |p_FK(q) - p_target| over all returned branches [m]
  std::vector<double> fk_pos_err;
  /// max rotation-angle error of R_FK(q) vs R_target over all branches [rad]
  std::vector<double> fk_rot_err;

  std::string to_string() const;
};

/// Compute the two TCP targets of a frozen grasp and enumerate both arms.
/// `throwing` on a malformed grasp is deliberate: a bad grasp must stop the run.
struct DualGraspTargets {
  dual_arm::Iso3 world_T_object = dual_arm::Iso3::Identity();
  dual_arm::Iso3 world_T_left_tcp = dual_arm::Iso3::Identity();
  dual_arm::Iso3 world_T_right_tcp = dual_arm::Iso3::Identity();
};

DualGraspTargets graspTargets(const GraspConfig& cfg);

/// A bare (left, right) TCP pose pair in one frame -- used by the path checks,
/// which must reason about the TCP poses themselves rather than the object pose.
struct Iso3Pair {
  dual_arm::Iso3 left = dual_arm::Iso3::Identity();
  dual_arm::Iso3 right = dual_arm::Iso3::Identity();
};

inline Iso3Pair tcpPairOf(const DualGraspTargets& t) {
  return Iso3Pair{t.world_T_left_tcp, t.world_T_right_tcp};
}

/// Enumerate the surviving analytic branches of `arm` for `target_world`.
/// `kin` supplies the base transform and the independent FK.
BranchSet enumerateBranches(const GraspConfig& cfg, const dual_arm::DualArmKinematics& kin,
                            const dual_arm::Iso3& target_world, const std::string& arm,
                            double fk_pos_tol = 1e-6, double fk_rot_tol = 1e-6);

/// Prismatic finger strokes [m] that put the pads at the sample faces plus a
/// per-side pad clearance.  Returned as (finger1, finger2) because the URDF
/// gives the two joints opposite sign conventions.
std::pair<double, double> fingerStrokes(const GraspConfig& cfg, double pad_clearance_m);

}  // namespace dual_arm_transport
