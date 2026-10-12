// include/dual_arm_pose_optimization/m4_baseline.hpp
// C4.2 / S2: the M4 handover model and the OFFICIAL Baseline B rebuild.
//
// Why the rebuild is mandatory (C4.2_执行规划.md §2.1.1)
// -----------------------------------------------------
// M4's handover is a PROCEDURE, not a taught joint configuration.  The yaml only
// stores two "base points":
//     bb.left_hold_joints   -- the verified left-arm HOLD configuration
//     bb.right_receive_base -- the right arm's *IK seed / branch base*
//                              (j4..j6 fixed, j1..j3 solved along the infeed)
// Using `right_receive_base` verbatim as the baseline is measurably wrong: it
// sits 9.0 mm away from the real RECEIVE target derived from the rod pose.
// This header therefore implements
//
//     M4 target pose -> FairinoIK::solve() -> IKSelector -> q_R^{M4}
//     q_B = [ q_L^{M4} ; q_R^{M4} ]  -> FK -> T_L^B, T_R^B, T_O^B, T_rel^B
//
// Every number below is quoted from the repository; nothing is invented.
// The four open questions Q1..Q4 from the plan are recorded on the parameters
// they affect, and `BaselinePose::provisional` is true until they are resolved
// (plan §2.1.2: "基于当前配置的临时重建基线").

#pragma once

#include <string>
#include <vector>

#include <Eigen/Dense>

#include "dual_arm_kinematics/dual_arm_kinematics.hpp"
#include "dual_arm_pose_optimization/collision_backend.hpp"
#include "dual_arm_pose_optimization/ik_layer.hpp"
#include "dual_arm_pose_optimization/scene_loader.hpp"

namespace dual_arm_pose_optimization {

/// Parameters of the M4 long-rod handover, with their provenance.
struct M4Params {
  // ---- object: bb.object_size_x/y/z  (m4_handover.yaml) -------------------
  // Q3 (open): whether 0.03 x 0.15 x 0.03 is the real rod or only the collision
  // box.  The comment in the yaml calls it "碰撞盒（短边）".
  Eigen::Vector3d object_size{0.03, 0.15, 0.03};

  // ---- ^G_L T_O : bb.left_object_pos_* / left_object_quat_* ---------------
  // Q1 (open): the same file also carries `left_object_pos_v` /
  // `left_object_quat_v` with materially different values (pos differs ~1 mm,
  // the quaternion differs in sign and components).  A1 says: use the scalar
  // (`*_pos_*`) values, and record the discrepancy.
  Eigen::Vector3d object_in_left_grasp_pos{-0.0149, 0.0003, 0.0149};
  Eigen::Quaterniond object_in_left_grasp_quat{0.0506, 0.7025, 0.7081, -0.0503};  // w,x,y,z

  // ---- M4 RECEIVE geometry -----------------------------------------------
  // bb.receive_offset, frame "world": target = rod_center + offset (world axes)
  // Q2 (open): the yaml comment says the measured seating is z = +15.0 mm while
  // the stored value is 0.0030 m (and 0.0009 + 0.014 = 0.0149).  A1: use the
  // stored value, flag the comment/value mismatch.
  Eigen::Vector3d receive_offset_world{-0.0086, -0.054, 0.0030};
  // Q4 (open): the yaml also carries an object-frame variant in a comment.
  // `receive_offset_frame` is "world" — that is what is implemented here.
  Eigen::Quaterniond receive_quat{0.7314, 0.0087, 0.6819, 0.0060};  // w,x,y,z

  // ---- arm configurations -------------------------------------------------
  dual_arm::JointConfig left_hold_joints =
      (dual_arm::JointConfig() << 0.3353, -1.5739, -0.9095, -2.2290, 1.5708, -1.2355)
          .finished();
  dual_arm::JointConfig right_receive_base =
      (dual_arm::JointConfig() << 0.2856, -1.2598, -1.7115, -3.3161, -1.2915, -1.5708)
          .finished();

  // Links allowed to touch the object (bb.object_touch_links).
  std::vector<std::string> object_touch_links{"left_finger1", "left_finger2",
                                              "left_grasp_frame", "right_finger1",
                                              "right_finger2", "right_grasp_frame"};
  std::string object_id = "handover_bar";

  static M4Params repositoryDefaults() { return M4Params{}; }

  /// ^G_L T_O as an isometry.
  dual_arm::Iso3 objectInLeftGrasp() const;
};

/// The rebuilt (or annex) baseline pose, fully derived by FK from q_L / q_R.
struct BaselinePose {
  std::string id;                 ///< "M4_baseline_B" / "baseline_A_historical"
  bool official = false;          ///< true only for M4 Baseline B
  bool provisional = true;        ///< true until Q1..Q4 are confirmed (plan §2.1.2)

  dual_arm::JointConfig q_left = dual_arm::JointConfig::Zero();
  dual_arm::JointConfig q_right = dual_arm::JointConfig::Zero();

  dual_arm::Iso3 world_T_left = dual_arm::Iso3::Identity();
  dual_arm::Iso3 world_T_right = dual_arm::Iso3::Identity();
  dual_arm::Iso3 world_T_object = dual_arm::Iso3::Identity();
  dual_arm::Iso3 left_T_right = dual_arm::Iso3::Identity();

  // ---- rebuild diagnostics (empty for the historical annex) --------------
  bool rebuilt_via_ik = false;
  Eigen::Matrix4d target_world = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d target_right_base = Eigen::Matrix4d::Identity();
  /// ||FK(right_receive_base) - target||: the 9 mm that forces the rebuild.
  double pre_rebuild_pos_err = 0.0;
  double pre_rebuild_rot_err = 0.0;
  /// ||FK(q_R^{M4}) - target||: must be at IK tolerance.
  double post_rebuild_pos_err = 0.0;
  double post_rebuild_rot_err = 0.0;
  IkOutcome right_ik;

  /// q12 in C2 order (left then right).
  dual_arm::Q12 q12() const;
  std::vector<double> q12Vector() const;
};

/// Compute the M4 RECEIVE target (world frame) from the left arm's HOLD pose.
///   T_O = FK_left(q_L) * (^G_L T_O)
///   p_target = T_O.translation() + receive_offset_world   (world axes)
///   R_target = receive_quat
Eigen::Matrix4d m4ReceiveTargetWorld(const dual_arm::Iso3& world_T_left,
                                     const M4Params& params);

/// Rebuild the official M4 Baseline B: solve the right arm to the M4 target,
/// select a branch with `right_receive_base` as the seed, and re-derive every
/// pose by FK.  Throws SceneLoadError if IK yields no selectable branch.
BaselinePose rebuildBaselineB(const std::shared_ptr<DualArmScene>& scene,
                              const M4Params& params = M4Params::repositoryDefaults());

/// Baseline A (`dual_handover_ready`) -- HISTORICAL / STATIC REFERENCE ONLY.
/// Kept for the annex; never used as a denominator for gains.
BaselinePose baselineAHistorical();

/// Add the rod to the planning scene and register the object<->gripper ACM
/// exceptions.  The exemptions are INSTALLED INTO THE SCENE (PlanningScene's own
/// ACM is what `isStateColliding` consults) and the resulting matrix is returned
/// so tests can assert the entries.
/// Side effect: mutates `scene`'s world and ACM.
collision_detection::AllowedCollisionMatrix addRodToScene(
    const std::shared_ptr<DualArmScene>& scene, const BaselinePose& pose,
    const M4Params& params);

}  // namespace dual_arm_pose_optimization
