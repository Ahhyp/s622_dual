// include/dual_arm_transport/pair_screen.hpp
// C4.3 S2 / W3 + W4 -- cartesian-product screening of one frozen dual-end grasp.
//
// The two arms are checked by TWO INDEPENDENT ANCHORS (user m02331 D2 / S2-T8):
// the object pose is estimated once from the left grasp and once from the right
// grasp, using S1's `anchoredObjectPose()`.  A pair is accepted only when both
// anchors agree with the task pose.  This is deliberately NOT the legacy
// midpoint (`DualArmKinematics::worldTobject`), whose worst-case mismatch S1
// measured at 195.28%.
//
// Every rejection is attributed to exactly one code so the acceptance record can
// report a failure histogram instead of "no solution found".
#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <moveit/robot_state/robot_state.h>

#include "dual_arm_kinematics/dual_arm_kinematics.hpp"
#include "dual_arm_pose_optimization/collision_backend.hpp"
#include "dual_arm_pose_optimization/scene_loader.hpp"
#include "dual_arm_transport/grasp_config.hpp"
#include "dual_arm_transport/ik_enum.hpp"

namespace dual_arm_transport {

/// One (left branch, right branch) combination and everything measured about it.
struct PairCandidate {
  int left_index = -1;
  int right_index = -1;
  dual_arm::JointConfig q_left = dual_arm::JointConfig::Zero();
  dual_arm::JointConfig q_right = dual_arm::JointConfig::Zero();
  dual_arm::Q12 q12 = dual_arm::Q12::Zero();

  // ---- independent anchors (S2-T8) ----
  dual_arm::Iso3 object_from_left = dual_arm::Iso3::Identity();
  dual_arm::Iso3 object_from_right = dual_arm::Iso3::Identity();
  double anchor_pos_gap_m = 0.0;
  double anchor_rot_gap_rad = 0.0;
  /// Legacy midpoint estimate, kept ONLY so the record can show the difference.
  dual_arm::Iso3 object_midpoint = dual_arm::Iso3::Identity();
  double midpoint_vs_task_pos_m = 0.0;
  double midpoint_vs_task_rot_rad = 0.0;

  // ---- closure (S2-T10) ----
  double closure_pos_m = 0.0;
  double closure_rot_rad = 0.0;
  bool closure_dir_ambiguous = false;

  // ---- conditioning ----
  double sigma_min = 0.0;

  // ---- collision (S2-T11) ----
  bool collides = false;
  std::string collision_kind;  ///< "none" | "self" | "arm_arm" | "arm_proximal" | "world"
  std::string collision_detail;
  double min_clearance_m = 0.0;

  // ---- feasibility verdict (S2-T12/T13) ----
  bool feasible = false;
  std::string reject_code;
  std::string reject_detail;
};

struct ScreenOptions {
  /// Per-side pad clearance used to close the fingers on the sample [m].
  double pad_clearance_m = 1e-4;
  /// Required free distance between the robot and anything not intentionally
  /// touched [m].  Below this a pair is rejected as a clearance failure.
  double min_clearance_m = 0.002;
  /// Independent-anchor agreement tolerance [m] / [rad].
  double anchor_pos_tol_m = 1e-6;
  double anchor_rot_tol_rad = 1e-6;
  /// Closure residual acceptance [m] / [rad].
  double closure_pos_tol_m = 1e-6;
  double closure_rot_tol_rad = 1e-6;
  /// Minimum singular value of the 6x12 object Jacobian.
  double min_sigma = 1e-4;
  /// Keep every candidate in the report (default) or only feasible ones.
  bool keep_all = true;
};

struct ScreenReport {
  int n_pairs = 0;
  int n_feasible = 0;
  std::map<std::string, int> reject_histogram;
  std::vector<PairCandidate> candidates;
  std::string to_string(std::size_t max_rows = 40) const;
};

/// Owns the scene, injects the sample + support, and screens pairs.
class PairScreener {
public:
  PairScreener(std::shared_ptr<dual_arm_pose_optimization::DualArmScene> scene,
               const GraspConfig& cfg, const ScreenOptions& opt = ScreenOptions{});

  /// Insert/refresh the sample at `world_T_object` and the candidate support.
  void applyWorld(const dual_arm::Iso3& world_T_object);

  /// Remove the sample and support again (used by the failure-attribution tests).
  void clearObjects();

  /// Cartesian product of the two branch sets with full attribution.
  ScreenReport screen(const BranchSet& left, const BranchSet& right);

  /// The robot state that actually grasps: 12 arm joints + both prismatic
  /// finger pairs closed to `grip_thickness + 2 * pad_clearance`.
  moveit::core::RobotState graspState(const dual_arm::Q12& q12) const;

  /// Object pose estimated from one arm's grasp transform (independent anchor).
  dual_arm::Iso3 anchoredObject(const dual_arm::JointConfig& q, const std::string& arm) const;

  const ScreenOptions& options() const { return opt_; }
  const dual_arm_pose_optimization::MoveItCollisionBackend& backend() const { return *backend_; }
  std::shared_ptr<dual_arm_pose_optimization::DualArmScene> scene() const { return scene_; }

private:
  std::shared_ptr<dual_arm_pose_optimization::DualArmScene> scene_;
  std::unique_ptr<dual_arm_pose_optimization::MoveItCollisionBackend> backend_;
  GraspConfig cfg_;
  ScreenOptions opt_;
  dual_arm::DualArmKinematics kin_;
  dual_arm::Iso3 world_T_object_ = dual_arm::Iso3::Identity();
  bool world_applied_ = false;
};

}  // namespace dual_arm_transport
