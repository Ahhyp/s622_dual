#pragma once

#include <Eigen/Core>
#include <cmath>

namespace dual_arm_collision {

// A distance query supplies a surface distance, its joint gradient, and a
// world-frame normal from obstacle to robot. This helper does no geometry or
// prediction. The caller must align all inputs to the same simulation time.
struct MovingObstacleCbfRow {
  Eigen::Matrix<double, 1, 24> coefficients = Eigen::Matrix<double, 1, 24>::Zero();
  double upper = 0.0;
  bool valid = false;
};

inline MovingObstacleCbfRow movingObstacleCbfRow(
    double distance, const Eigen::Matrix<double, 12, 1>& gradient,
    const Eigen::Vector3d& normal, const Eigen::Vector3d& obstacle_point_velocity,
    double sample_age, double max_age, double clearance, double alpha, double dt) {
  MovingObstacleCbfRow row;
  if (!std::isfinite(distance) || distance <= 0 || !gradient.allFinite() ||
      !normal.allFinite() || !obstacle_point_velocity.allFinite() ||
      !std::isfinite(sample_age) || !std::isfinite(max_age) || max_age <= 0 ||
      sample_age < 0 || sample_age > max_age ||
      !std::isfinite(clearance) || clearance <= 0 ||
      !std::isfinite(alpha) || alpha <= 0 || !std::isfinite(dt) || dt <= 0 ||
      alpha * dt > 1 || std::abs(normal.norm() - 1.0) > 1e-6) return row;
  row.coefficients.head<12>() = -gradient.transpose();
  row.upper = dt * (alpha * (distance - clearance) - normal.dot(obstacle_point_velocity));
  row.valid = std::isfinite(row.upper);
  return row;  // valid=false means STOP/reject, never "no obstacle".
}

}  // namespace dual_arm_collision
