// C4.3 D5-3 / R4 — software-only collision guard (three checks).
//
// Reuse rule: the arm-arm check is NOT re-derived here.  checkArmArm() calls
// dual_arm_collision::DualArmCollisionModel::distance(q) (capsule-capsule
// distance between the two arms) and only compares that measurement against a
// configured threshold.  This file contains no trigonometry, no capsule math
// and no second distance implementation.
//
// SCOPE NOTES (must stay honest)
//  * Check 2 (checkObjectPose) is an ENVIRONMENT PROXY: proxy geometry, not a
//    mesh/FCL scene; not a claim of environment collision completeness.
//  * Check 3 (checkSweep) is a SAMPLED sweep of the arm-arm check along a
//    straight joint-space segment: it is not a continuous swept-volume proof
//    and it sees only the arm-arm pair.
//  * Pure logic: no ROS, no clock, no I/O.  The guard is one software layer,
//    never the only protection of the machine.
//
// clearance_m convention (identical for all three checks)
//   clearance_m = (measured quantity) - (required threshold or radius)
//     >= 0.0  -> ok == true,  the value is the margin left over [m]
//     <  0.0  -> ok == false, |clearance_m| is the violation depth [m]
//   A verdict that cannot be evaluated at all (guard not configured, model
//   reports an invalid distance) is FAIL-CLOSED: ok == false and
//   clearance_m == -inf.  A passing verdict always has an EMPTY code.
#ifndef DUAL_ARM_TRANSPORT_EXECUTOR__COLLISION_GUARD_HPP_
#define DUAL_ARM_TRANSPORT_EXECUTOR__COLLISION_GUARD_HPP_

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "dual_arm_collision/dual_arm_collision.hpp"
#include "dual_arm_kinematics/dual_arm_kinematics.hpp"

namespace dual_arm_transport_executor
{

struct CollisionVerdict
{
  bool ok = true;
  std::string code;        // failure code; EMPTY when ok
  double clearance_m = std::numeric_limits<double>::infinity();
};

class CollisionGuard
{
public:
  /// Capsule radius handed to DualArmCollisionModel (matches its own default,
  /// the value used by the dual_arm_collision tests).
  static constexpr double kDefaultLinkRadius = 0.06;
  static constexpr int kMinSamples = 2;
  static constexpr int kMaxSamples = 64;

  CollisionGuard();
  ~CollisionGuard();

  /// Builds the underlying DualArmCollisionModel and resets the guard state.
  /// `arm_arm_min_gap_m` is the required surface-to-surface margin between the
  /// two arms; 0.0 therefore means "pure penetration test".  Configuring again
  /// clears every exclusion sphere added so far.
  void configure(double arm_arm_min_gap_m, bool object_checks_enabled);

  /// Check 1 — arm-arm clearance, delegated verbatim to
  /// DualArmCollisionModel::distance(q).  Code "collision_arm_arm".
  CollisionVerdict checkArmArm(const dual_arm::Q12 & q) const;

  /// Check 2 — object pose origin vs the exclusion spheres (environment proxy:
  /// proxy geometry, not a mesh/FCL scene; not a claim of environment collision
  /// completeness).  A sphere is violated when the object origin lies inside
  /// it.  Code "collision_object".
  ///
  /// When object_checks_enabled == false this check is DISABLED and returns
  /// {true, "", +inf} unconditionally — an explicit, documented bypass, not a
  /// measurement.
  CollisionVerdict checkObjectPose(const dual_arm::Iso3 & object_pose) const;

  /// Check 3 — linearly interpolated joint-space segment, `samples` points
  /// including both endpoints, `samples` clamped to [kMinSamples, kMaxSamples];
  /// checkArmArm() is run at every sample.  Code "collision_sweep".
  /// A passing verdict keeps the per-sample loop heap-allocation free (the only
  /// strings built are on the failure path); clearance_m is the tightest margin
  /// over all samples, not just the first violating one.
  CollisionVerdict checkSweep(const dual_arm::Q12 & q_from, const dual_arm::Q12 & q_to,
                              int samples) const;

  /// Add one exclusion sphere (centre in the same frame as the object pose
  /// origin, hard radius).  The radius is used exactly as given: radius == 0
  /// degrades to a point test and a NEGATIVE radius loosens the test by that
  /// amount (it can never be violated for a finite distance), so callers are
  /// expected to pass radius > 0.  No silent clamping is applied.
  void addExclusionSphere(const Eigen::Vector3d & c, double radius);

private:
  struct Sphere
  {
    Eigen::Vector3d center = Eigen::Vector3d::Zero();
    double radius = 0.0;
  };

  /// Declared BEFORE model_: the model holds a const reference to it.
  dual_arm::DualArmKinematics kin_;
  std::unique_ptr<dual_arm_collision::DualArmCollisionModel> model_;
  double arm_arm_min_gap_m_ = 0.0;
  bool object_checks_enabled_ = false;
  std::vector<Sphere> spheres_;
};

}  // namespace dual_arm_transport_executor

#endif  // DUAL_ARM_TRANSPORT_EXECUTOR__COLLISION_GUARD_HPP_
