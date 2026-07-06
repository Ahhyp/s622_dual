#pragma once

#include <vector>
#include "fairino_planning_core/types.h"
#include "fairino_planning_core/dh_kinematics.h"

namespace fairino_planning
{

class FairinoIK
{
public:
  FairinoIK(const std::array<double, DOF>& d,
            const std::array<double, DOF>& a,
            const std::array<double, DOF>& alpha);

  std::vector<JointArray> solve(const Pose& target) const;

private:
  std::array<double, DOF> d_, a_, alpha_;
  DHKinematics fk_;

  static Eigen::Matrix3d R03(double q1, double q2, double q3);

  static std::vector<JointArray> uniqueSolutions(
      const std::vector<JointArray>& sols, double tol = 1e-5);
};

}  // namespace fairino_planning
