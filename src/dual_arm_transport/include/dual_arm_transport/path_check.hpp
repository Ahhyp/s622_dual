// include/dual_arm_transport/path_check.hpp
// C4.3 S2 / W5 + W6 -- pre-grasp reachability and approach-path validation.
//
// Two DIFFERENT things are validated and they must not be conflated (user
// m02331 D7):
//   (a) the JOINT-SPACE motion from q_standby to the pre-grasp configuration --
//       a joint interpolation is acceptable here;
//   (b) the final APPROACH from the pre-grasp pose onto the grasp pose -- this
//       one is a constrained Cartesian motion and is checked against the
//       straight-line assumption, not merely for collisions.
//
// Nothing in this file plans or validates the post-grasp transport motion; that
// is explicitly out of S2 scope (m02331 D7).
#pragma once

#include <memory>
#include <string>

#include "dual_arm_pose_optimization/collision_backend.hpp"
#include "dual_arm_pose_optimization/ik_layer.hpp"
#include "dual_arm_pose_optimization/scene_loader.hpp"
#include "dual_arm_transport/grasp_config.hpp"
#include "dual_arm_transport/ik_enum.hpp"

namespace dual_arm_transport {

struct PathCheckOptions {
  int samples = 32;
  /// Required free distance at every sample [m].
  double min_clearance_m = 0.002;
  /// Max allowed Cartesian departure of a TCP from the commanded straight line
  /// during the approach [m].  ALWAYS measured and reported; it is only a hard
  /// failure when `require_straight_approach` is set (user m02331 D7: a
  /// joint-space pre-grasp motion is acceptable, and what the final approach
  /// needs is its own collision validation -- not a straightness guarantee that
  /// a joint interpolation cannot provide).
  double approach_line_tol_m = 1e-3;
  /// Turn the straight-line deviation above into a rejection.  Off by default.
  bool require_straight_approach = false;
  /// Max allowed rotation of a TCP from the commanded approach direction [rad].
  double approach_angle_tol_rad = 1e-2;
};

struct SegmentResult {
  bool ok = false;
  /// "ok" | "collision" | "insufficient_clearance" | "joint_limit"
  /// | "approach_not_straight" | "endpoint_invalid"
  std::string code;
  std::string detail;
  std::size_t n_samples = 0;
  int first_bad_sample = -1;
  double min_clearance_m = 0.0;
  /// Worst straight-line departure / angle error seen on an approach segment.
  double max_line_dev_m = 0.0;
  double max_angle_dev_rad = 0.0;
  dual_arm::Q12 q_bad = dual_arm::Q12::Zero();
  std::string to_string() const;
};

struct ReachabilityResult {
  bool ok = false;
  std::string code;
  std::string detail;
  /// Approach targets that were actually achievable; when the approach fails the
  /// pair is reported as unreachable rather than silently relaxed.
  dual_arm::JointConfig q_left_pregrasp = dual_arm::JointConfig::Zero();
  dual_arm::JointConfig q_right_pregrasp = dual_arm::JointConfig::Zero();
  SegmentResult standby_segment;
  SegmentResult approach_segment;
};

class PathChecker {
public:
  PathChecker(std::shared_ptr<dual_arm_pose_optimization::DualArmScene> scene,
              const GraspConfig& cfg, const PathCheckOptions& opt = PathCheckOptions{});

  /// True when every joint of `q12` is inside the model's limits.
  bool withinJointLimits(const dual_arm::Q12& q12, std::string* detail) const;

  /// Joint-space interpolation a -> b, densely sampled.
  SegmentResult jointSegment(const dual_arm::Q12& a, const dual_arm::Q12& b) const;

  /// Joint interpolation from `q_back` to `q_grasp`, additionally asserting that
  /// each TCP tracks the commanded straight-line approach within tolerance.
  SegmentResult approachSegment(const dual_arm::Q12& q_back, const dual_arm::Q12& q_grasp,
                                const Iso3Pair& task_world) const;

  /// Full W5+W6 check for one candidate.
  ReachabilityResult check(const dual_arm::Q12& q_standby, const dual_arm::Q12& q_grasp,
                           const Iso3Pair& task_world) const;

  /// Back the grasp TCP off along the tool z axis by `d_approach_m`.
  Iso3Pair pregraspTargets(const Iso3Pair& grasp_world) const;

  const PathCheckOptions& options() const { return opt_; }

private:
  moveit::core::RobotState stateOf(const dual_arm::Q12& q12) const;
  std::shared_ptr<dual_arm_pose_optimization::DualArmScene> scene_;
  std::unique_ptr<dual_arm_pose_optimization::MoveItCollisionBackend> backend_;
  dual_arm_pose_optimization::IkLayer ik_layer_;
  GraspConfig cfg_;
  PathCheckOptions opt_;
};

}  // namespace dual_arm_transport
