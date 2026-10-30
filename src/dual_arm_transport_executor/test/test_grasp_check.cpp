// C4.3 S4 S4-T16 -- grasp-relative check tests (pure arithmetic).
#include <gtest/gtest.h>

#include <cmath>

#include "dual_arm_transport_executor/grasp_check.hpp"

namespace dae = dual_arm_transport_executor;

namespace
{
Eigen::Matrix4d translated(double dx, double dy, double dz)
{
  Eigen::Matrix4d m = Eigen::Matrix4d::Identity();
  m(0, 3) = dx;
  m(1, 3) = dy;
  m(2, 3) = dz;
  return m;
}

Eigen::Matrix4d rotated(double angle_rad)
{
  Eigen::Matrix4d m = Eigen::Matrix4d::Identity();
  m.block<3, 3>(0, 0) =
    Eigen::AngleAxisd(angle_rad, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  return m;
}
}  // namespace

TEST(GraspCheck, ExactPosePasses)
{
  const dae::GraspTolerances tol;
  const auto r = dae::checkGraspRelative(Eigen::Matrix4d::Identity(),
                                         Eigen::Matrix4d::Identity(), tol);
  EXPECT_TRUE(r.ok);
  EXPECT_DOUBLE_EQ(r.pos_m, 0.0);
  EXPECT_DOUBLE_EQ(r.rot_rad, 0.0);
}

TEST(GraspCheck, ThreeMillimetresIsTheBoundary)
{
  const dae::GraspTolerances tol;  // 3 mm / 1 deg
  const auto inside = dae::checkGraspRelative(
    translated(0.0, 0.0, 0.0029), Eigen::Matrix4d::Identity(), tol);
  const auto outside = dae::checkGraspRelative(
    translated(0.0, 0.0, 0.0031), Eigen::Matrix4d::Identity(), tol);
  EXPECT_TRUE(inside.ok);
  EXPECT_FALSE(outside.ok);
}

TEST(GraspCheck, FiveMillimetreLiftIsRejected)
{
  // S4-T16: the sample is raised 5 mm before GRASP; the relative pose must fail.
  const dae::GraspTolerances tol;
  const auto r = dae::checkGraspRelative(
    translated(0.0, 0.0, 0.005), Eigen::Matrix4d::Identity(), tol);
  EXPECT_FALSE(r.ok);
  EXPECT_NEAR(r.pos_m, 0.005, 1e-12);
}

TEST(GraspCheck, OneDegreeIsTheRotationBoundary)
{
  const dae::GraspTolerances tol;
  const auto inside = dae::checkGraspRelative(
    rotated(0.9 * M_PI / 180.0), Eigen::Matrix4d::Identity(), tol);
  const auto outside = dae::checkGraspRelative(
    rotated(1.1 * M_PI / 180.0), Eigen::Matrix4d::Identity(), tol);
  EXPECT_TRUE(inside.ok);
  EXPECT_FALSE(outside.ok);
}

TEST(GraspCheck, RotationIsFrameIndependentOfTranslation)
{
  const dae::GraspTolerances tol;
  Eigen::Matrix4d measured = translated(0.001, 0.0, 0.0) * rotated(0.5 * M_PI / 180.0);
  const auto r = dae::checkGraspRelative(measured, Eigen::Matrix4d::Identity(), tol);
  EXPECT_TRUE(r.ok);
}
