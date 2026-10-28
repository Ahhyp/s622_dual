// C4.3 D5-3 / R4 — software-only collision guard implementation.
//
// The only collision mathematics touched here is
// dual_arm_collision::DualArmCollisionModel::distance(q); everything else is a
// threshold comparison or a straight geometric query on the exclusion spheres.

#include "dual_arm_transport_executor/collision_guard.hpp"

#include <algorithm>
#include <cmath>

namespace dual_arm_transport_executor
{

namespace
{

constexpr double kInf = std::numeric_limits<double>::infinity();

CollisionVerdict failClosed(const char * code)
{
  CollisionVerdict v;
  v.ok = false;
  v.code = code;
  v.clearance_m = -kInf;
  return v;
}

CollisionVerdict measured(double clearance_m, const char * code)
{
  CollisionVerdict v;
  v.ok = clearance_m >= 0.0;
  v.clearance_m = clearance_m;
  if (!v.ok) {
    v.code = code;
  }
  return v;
}

}  // namespace

CollisionGuard::CollisionGuard() = default;
CollisionGuard::~CollisionGuard() = default;

void CollisionGuard::configure(double arm_arm_min_gap_m, bool object_checks_enabled)
{
  arm_arm_min_gap_m_ = arm_arm_min_gap_m;
  object_checks_enabled_ = object_checks_enabled;
  spheres_.clear();
  model_ = std::make_unique<dual_arm_collision::DualArmCollisionModel>(
    kin_, CollisionGuard::kDefaultLinkRadius);
}

CollisionVerdict CollisionGuard::checkArmArm(const dual_arm::Q12 & q) const
{
  if (!model_) {
    return failClosed("collision_arm_arm");  // never configured: fail closed
  }
  const dual_arm_collision::DistanceInfo info = model_->distance(q);
  if (!info.valid) {
    return failClosed("collision_arm_arm");  // no measurement: fail closed
  }
  return measured(info.distance - arm_arm_min_gap_m_, "collision_arm_arm");
}

CollisionVerdict CollisionGuard::checkObjectPose(const dual_arm::Iso3 & object_pose) const
{
  if (!object_checks_enabled_) {
    CollisionVerdict v;  // documented bypass: no geometry is evaluated
    v.ok = true;
    v.clearance_m = kInf;
    return v;
  }
  if (spheres_.empty()) {
    CollisionVerdict v;  // no proxy geometry configured: nothing to violate
    v.ok = true;
    v.clearance_m = kInf;
    return v;
  }
  const Eigen::Vector3d p = object_pose.translation();
  double worst = kInf;
  for (const Sphere & s : spheres_) {
    worst = std::min(worst, (s.center - p).norm() - s.radius);
  }
  return measured(worst, "collision_object");
}

CollisionVerdict CollisionGuard::checkSweep(const dual_arm::Q12 & q_from,
                                            const dual_arm::Q12 & q_to, int samples) const
{
  const int n = std::min(kMaxSamples, std::max(kMinSamples, samples));
  const dual_arm::Q12 delta = q_to - q_from;
  double worst = kInf;
  for (int i = 0; i < n; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(n - 1);
    const dual_arm::Q12 q = q_from + t * delta;
    // checkArmArm() returns an empty code on success, so the per-sample loop
    // only builds a colliding verdict when there is actually a collision.
    const CollisionVerdict v = checkArmArm(q);
    worst = std::min(worst, v.clearance_m);
  }
  return measured(worst, "collision_sweep");
}

void CollisionGuard::addExclusionSphere(const Eigen::Vector3d & c, double radius)
{
  Sphere s;
  s.center = c;
  s.radius = radius;
  spheres_.push_back(s);
}

}  // namespace dual_arm_transport_executor
