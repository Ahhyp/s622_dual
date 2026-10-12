// include/dual_arm_pose_optimization/collision_backend.hpp
// C4.2 / S1: collision & distance queries with EXPLICIT, TESTED scopes.
//
// The plan calls out three traps that make collision results silently wrong
// (C4.2_执行规划.md §2.5).  This header encodes the fix for each:
//
//  trap 2  distanceSelf(state) without an ACM always returns 0, because 34
//          adjacent link pairs in the SRDF are "designed to touch".
//          -> every query here takes an ACM; there is no ACM-less overload.
//
//  trap 3  the global minimum distance is NOT the arm-arm clearance -- it is
//          usually dominated by an INTRA-arm near pair.  So we expose two
//          distinct scopes and never conflate them:
//             whole_robot : min over every link pair (SRDF ACM)
//             arm_arm     : min over cross-arm pairs only (same-arm disabled)
//          plus a third, independent reading from the C2.7 capsule proxy.
//
//  trap 4  a floor whose top face is at z=0 touches base_link, which makes
//          every state "in collision" for distance queries.
//          -> the floor carries an explicit base_link exemption in the
//             robot-world ACM, and the exemption itself is unit-tested.

#pragma once

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <moveit/collision_detection/collision_matrix.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/robot_state.h>

#include "dual_arm_collision/dual_arm_collision.hpp"
#include "dual_arm_kinematics/dual_arm_kinematics.hpp"
#include "dual_arm_pose_optimization/scene_loader.hpp"

namespace dual_arm_pose_optimization {

/// Environment objects added to the planning scene.
struct EnvironmentOptions {
  bool add_floor = true;
  std::string floor_id = "c42_floor";
  /// Top surface of the floor box [m].  The box extends DOWNWARD from here.
  double floor_top_z = 0.0;
  double floor_size_xy = 4.0;
  double floor_thickness = 0.2;
  /// Links allowed to touch the floor (the arms are bolted to it).
  std::vector<std::string> floor_exempt_links{"left_base_link", "right_base_link"};
};

/// One pose evaluated in three distance scopes.
struct DistanceReading {
  double whole_robot = 0.0;      ///< [m] min over ALL link pairs (SRDF ACM)
  double arm_arm = 0.0;          ///< [m] min over CROSS-ARM pairs only
  double capsule_arm_arm = -1.0; ///< [m] C2.7 capsule proxy; <0 when not evaluated
  bool in_collision = false;     ///< dual_arm group self-collision (SRDF ACM)

  /// Closest cross-arm pair (empty when nothing was measured).
  std::string arm_arm_link_a;
  std::string arm_arm_link_b;
  Eigen::Vector3d arm_arm_point_a = Eigen::Vector3d::Zero();
  Eigen::Vector3d arm_arm_point_b = Eigen::Vector3d::Zero();

  /// Closest pair for the whole-robot scope (incl. intra-arm pairs).
  std::string whole_link_a;
  std::string whole_link_b;

  std::string to_string() const;
};

class MoveItCollisionBackend {
public:
  MoveItCollisionBackend(const std::shared_ptr<DualArmScene>& scene,
                         const EnvironmentOptions& env = EnvironmentOptions{});

  // ---- predicates -------------------------------------------------------
  /// Self-collision of the "dual_arm" group, SRDF ACM applied.
  bool isStateColliding(const moveit::core::RobotState& state) const;
  /// Self-collision against the whole robot (all groups).
  bool isStateCollidingWholeRobot(const moveit::core::RobotState& state) const;

  /// Discretised joint-space segment check: N samples of the straight line
  /// between the two states, each tested with isStateColliding().
  /// NOTE: this is a *segment* check, not a motion planner.  It answers
  /// "is the straight joint-space interpolation collision free", which is the
  /// same semantics MoveItCollisionChecker::isMotionValid uses (C2.7 era).
  bool isSegmentValid(const moveit::core::RobotState& a, const moveit::core::RobotState& b,
                      int samples = 32) const;

  // ---- distances --------------------------------------------------------
  /// @param q12 optional C2 joint vector; when given, the C2.7 capsule
  ///        cross-arm distance is filled into `capsule_arm_arm` as an
  ///        independent cross-check of the mesh-based number.
  DistanceReading distances(const moveit::core::RobotState& state,
                            const dual_arm::Q12* q12 = nullptr) const;

  /// Distance between the robot and the world (floor etc.), robot-world ACM
  /// applied.  Returns <= 0 when an unexempted contact exists.
  double robotWorldDistance(const moveit::core::RobotState& state) const;

  /// Same query with an explicitly supplied ACM.  Used by the tests/probe to
  /// show that the base_link<->floor exemption is what changes the result.
  double robotWorldDistanceWith(const moveit::core::RobotState& state,
                                const collision_detection::AllowedCollisionMatrix& acm) const;

  // ---- matrices (exposed so tests can assert their content) -------------
  /// Exactly the SRDF ACM from the planning scene.
  const collision_detection::AllowedCollisionMatrix& srdfAcm() const { return srdf_acm_; }
  /// SRDF ACM + every same-arm link pair disabled -> cross-arm pairs only.
  const collision_detection::AllowedCollisionMatrix& crossArmAcm() const {
    return cross_arm_acm_;
  }
  /// SRDF ACM + floor exempted for `floor_exempt_links` (and, when the floor
  /// is absent, identical to the SRDF ACM).
  const collision_detection::AllowedCollisionMatrix& robotWorldAcm() const {
    return robot_world_acm_;
  }
  /// The same matrix WITHOUT the floor exemption -- used by the tests to prove
  /// the exemption is what makes the difference.
  const collision_detection::AllowedCollisionMatrix& robotWorldAcmNoFloorExemption() const {
    return robot_world_acm_no_exemption_;
  }

  bool floorAdded() const { return floor_added_; }
  const std::string& floorId() const { return env_.floor_id; }
  const EnvironmentOptions& environment() const { return env_; }
  const std::shared_ptr<DualArmScene>& scene() const { return scene_; }

  /// Number of link pairs that the cross-arm ACM leaves enabled (diagnostic).
  std::size_t crossArmPairCount() const { return cross_arm_pairs_; }

private:
  collision_detection::AllowedCollisionMatrix buildCrossArmAcm() const;

  std::shared_ptr<DualArmScene> scene_;
  EnvironmentOptions env_;
  bool floor_added_ = false;
  mutable std::size_t cross_arm_pairs_ = 0;

  collision_detection::AllowedCollisionMatrix srdf_acm_;
  collision_detection::AllowedCollisionMatrix cross_arm_acm_;
  collision_detection::AllowedCollisionMatrix robot_world_acm_;
  collision_detection::AllowedCollisionMatrix robot_world_acm_no_exemption_;

  // capsule cross-check (C2.7), lazily constructed
  mutable std::unique_ptr<dual_arm::DualArmKinematics> kin_;
  mutable std::unique_ptr<dual_arm_collision::DualArmCollisionModel> capsule_;
};

}  // namespace dual_arm_pose_optimization
