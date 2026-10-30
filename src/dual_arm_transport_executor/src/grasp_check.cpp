#include "dual_arm_transport_executor/grasp_check.hpp"

#include <algorithm>
#include <cmath>

namespace dual_arm_transport_executor
{

GraspCheckResult checkGraspRelative(const Eigen::Matrix4d & measured,
                                    const Eigen::Matrix4d & expected,
                                    const GraspTolerances & tol)
{
  GraspCheckResult r;
  const Eigen::Vector3d dp =
    measured.block<3, 1>(0, 3) - expected.block<3, 1>(0, 3);
  r.pos_m = dp.norm();
  const Eigen::Matrix3d dR =
    measured.block<3, 3>(0, 0).transpose() * expected.block<3, 3>(0, 0);
  const double c = std::max(-1.0, std::min(1.0, 0.5 * (dR.trace() - 1.0)));
  r.rot_rad = std::acos(c);
  r.ok = std::isfinite(r.pos_m) && std::isfinite(r.rot_rad) &&
    r.pos_m <= tol.pos_m && r.rot_rad <= tol.rot_rad;
  return r;
}

}  // namespace dual_arm_transport_executor
