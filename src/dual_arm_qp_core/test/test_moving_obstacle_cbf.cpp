#include <gtest/gtest.h>
#include <cmath>
#include <cstdio>
#include <limits>
#include "dual_arm_collision/moving_obstacle_cbf.hpp"
#include "dual_arm_qp_core/dual_arm_qp.hpp"

using dual_arm_collision::movingObstacleCbfRow;

TEST(MovingObstacleCbf, VelocitySignAndStaticReduction) {
  dual_arm::Q12 gradient = dual_arm::Q12::Zero(); gradient[0] = 2.;
  const auto row = [&](double speed) {
    return movingObstacleCbfRow(.15, gradient, Eigen::Vector3d::UnitX(),
        speed * Eigen::Vector3d::UnitX(), 0, .05, .10, 3, .008);
  };
  auto still = row(0), approaching = row(.02), departing = row(-.02);
  ASSERT_TRUE(still.valid); ASSERT_TRUE(approaching.valid); ASSERT_TRUE(departing.valid);
  EXPECT_NEAR(still.upper, 3 * .05 * .008, 1e-15);
  EXPECT_LT(approaching.upper, still.upper);
  EXPECT_GT(departing.upper, still.upper);
  EXPECT_DOUBLE_EQ(still.coefficients[0], -2.);
  EXPECT_DOUBLE_EQ(still.coefficients.tail<12>().norm(), 0.);
}

TEST(MovingObstacleCbf, RejectsInvalidInputs) {
  const dual_arm::Q12 gradient = dual_arm::Q12::Ones();
  auto valid = [&](double age, double distance, double alpha, Eigen::Vector3d n) {
    return movingObstacleCbfRow(distance, gradient, n, Eigen::Vector3d::Zero(),
        age, .05, .1, alpha, .008).valid;
  };
  EXPECT_TRUE(valid(.05, .1, 3, Eigen::Vector3d::UnitX()));
  EXPECT_FALSE(valid(.051, .1, 3, Eigen::Vector3d::UnitX()));
  EXPECT_FALSE(valid(-.001, .1, 3, Eigen::Vector3d::UnitX()));
  EXPECT_FALSE(valid(0, -.01, 3, Eigen::Vector3d::UnitX()));
  EXPECT_FALSE(valid(0, .1, 126, Eigen::Vector3d::UnitX()));
  EXPECT_FALSE(valid(0, .1, 3, Eigen::Vector3d::Zero()));
  EXPECT_FALSE(valid(0, std::numeric_limits<double>::quiet_NaN(), 3, Eigen::Vector3d::UnitX()));
  auto bad_gradient = gradient;
  bad_gradient[0] = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(movingObstacleCbfRow(.1, bad_gradient, Eigen::Vector3d::UnitX(),
      Eigen::Vector3d::Zero(), 0, .05, .1, 3, .008).valid);
  EXPECT_FALSE(movingObstacleCbfRow(.1, gradient, Eigen::Vector3d::UnitX(),
      Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN()),
      0, .05, .1, 3, .008).valid);
}

TEST(MovingObstacleCbf, InfeasibleConstraintIsNotReportedAsSuccess) {
  dual_arm::DualArmKinematics kin;
  const auto row = movingObstacleCbfRow(.05, dual_arm::Q12::Zero(),
      Eigen::Vector3d::UnitX(), Eigen::Vector3d(.1, 0, 0), 0, .05, .1, 3, .008);
  ASSERT_TRUE(row.valid); ASSERT_LT(row.upper, 0);
  dual_arm_qp::DualArmQpParams params;
  params.C_ineq = row.coefficients;
  params.d_ineq = Eigen::VectorXd::Constant(1, row.upper);
  dual_arm_qp::DualArmQp qp(kin, params);
  const dual_arm::Q12 q = dual_arm::Q12::Zero();
  // No available joint direction can satisfy this row. Do not apply its dq.
  EXPECT_FALSE(qp.solve(q, kin.leftTright(q)).converged);
}

TEST(MovingObstacleCbf, MovingSphereChangesCoordinatedQpMotion) {
  dual_arm::DualArmKinematics kin;
  dual_arm::JointConfig arm;
  arm << 0., -1.5708, 0., 0., 0., 0.;
  const dual_arm::Q12 initial = dual_arm::DualArmKinematics::stack(arm, arm);
  const auto target = kin.leftTright(initial);
  const Eigen::Vector3d p0 = kin.worldTgrasp(dual_arm::Arm::Left, arm).translation();
  constexpr double dt = .008, clearance = .08, radius = .05;
  struct Result { double minimum = 1e9, relative_position = 0, relative_angle = 0, motion = 0, residual = 0; int failures = 0; };
  auto run = [&](bool enabled) {
    dual_arm_qp::DualArmQpParams params;
    params.dt = dt; params.w_obj = 0; params.w_center = .01;
    params.dq_center.setZero();
    dual_arm_qp::DualArmQp qp(kin, params);
    dual_arm::Q12 q = initial;
    Result result;
    for (int k = 0; k < 500; ++k) {
      dual_arm::JointConfig left, right;
      dual_arm::DualArmKinematics::split(q, &left, &right);
      const Eigen::Vector3d p = kin.worldTgrasp(dual_arm::Arm::Left, left).translation();
      const Eigen::Vector3d center = p0 + Eigen::Vector3d(.15 - .01 * k * dt, 0, 0);
      const Eigen::Vector3d normal = (p - center).normalized();
      const double distance = (p - center).norm() - radius;
      dual_arm::Q12 gradient = dual_arm::Q12::Zero();
      gradient.head<6>() = kin.jacobianWorldPoint(dual_arm::Arm::Left, left).topRows<3>().transpose() * normal;
      const auto row = movingObstacleCbfRow(distance, gradient, normal,
          Eigen::Vector3d(-.01, 0, 0), 0, .05, clearance, 3, dt);
      EXPECT_TRUE(row.valid);
      if (enabled) {
        params.C_ineq = row.coefficients;
        params.d_ineq = Eigen::VectorXd::Constant(1, row.upper);
        qp.setParams(params);
      }
      const auto solution = qp.solve(q, target);
      if (!solution.converged) { ++result.failures; break; }
      if (enabled) result.residual = std::max(result.residual,
          (row.coefficients.head<12>() * solution.dq)[0] - row.upper);
      q += solution.dq;  // Ideal joint following, not Gazebo or delayed servos.
      dual_arm::DualArmKinematics::split(q, &left, &right);
      const Eigen::Vector3d next = kin.worldTgrasp(dual_arm::Arm::Left, left).translation();
      const Eigen::Vector3d next_center = center + Eigen::Vector3d(-.01 * dt, 0, 0);
      result.minimum = std::min(result.minimum, (next - next_center).norm() - radius);
      const auto error = kin.relativeError(q, target);
      result.relative_position = std::max(result.relative_position, error.head<3>().norm());
      result.relative_angle = std::max(result.relative_angle, error.tail<3>().norm());
    }
    result.motion = (q - initial).norm();
    return result;
  };
  const auto off = run(false), on = run(true);
  std::printf("Moving sphere / ideal joints: OFF min=%.9f motion=%.9f failures=%d; ON min=%.9f motion=%.9f relative_position_m=%.9g relative_angle_rad=%.9g row_residual=%.9g failures=%d\n",
      off.minimum, off.motion, off.failures, on.minimum, on.motion, on.relative_position, on.relative_angle, on.residual, on.failures);
  EXPECT_EQ(off.failures, 0); EXPECT_EQ(on.failures, 0);
  EXPECT_LT(off.minimum, clearance - .01);
  EXPECT_GE(on.minimum, clearance - 1e-4);
  EXPECT_GT(on.motion, .001);
  EXPECT_LT(on.relative_position, 1e-3);
  EXPECT_LT(on.relative_angle, 1e-3);
  EXPECT_LT(on.residual, 1e-5);
}
