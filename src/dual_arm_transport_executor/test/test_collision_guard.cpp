// C4.3 D5-3 / R4 — collision guard tests.
//
// All joint configurations below are measured against the shared
// DualArmCollisionModel (link radius 0.06 m, S622 DH), not guessed:
//   "stretched" (both arms j2 = -1.5708)  -> arm-arm distance 0.579996 m
//   both arms j2 = -2.0                   -> 0.147207 m (marginal)
//   both arms j2 = -2.4 / -2.6            -> -0.120000 m (penetration)
// The stretched configuration is the one used by dual_arm_collision's own
// test_collision.cpp (ClosestConfigurationIsDistinguishable).
#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <string>

#include "dual_arm_transport_executor/collision_guard.hpp"

namespace due = dual_arm_transport_executor;

namespace
{

using dual_arm::Iso3;
using dual_arm::JointConfig;
using dual_arm::Q12;

constexpr double kGap = 0.10;  // configured arm-arm minimum gap [m]

Q12 bothArms(double j2)
{
  JointConfig q;
  q << 0.0, j2, 0.0, 0.0, 0.0, 0.0;
  return dual_arm::DualArmKinematics::stack(q, q);
}

Q12 stretched()
{
  return bothArms(-1.5708);
}

Iso3 poseAt(double x, double y, double z)
{
  Iso3 p = Iso3::Identity();
  p.translation() = Eigen::Vector3d(x, y, z);
  return p;
}

}  // namespace

TEST(CollisionGuard, ArmArmPass)
{
  due::CollisionGuard g;
  g.configure(kGap, true);
  const due::CollisionVerdict v = g.checkArmArm(stretched());
  EXPECT_TRUE(v.ok);
  EXPECT_TRUE(v.code.empty());
  EXPECT_NEAR(v.clearance_m, 0.579996 - kGap, 1e-6);
}

TEST(CollisionGuard, ArmArmViolation)
{
  due::CollisionGuard g;
  g.configure(kGap, true);
  const due::CollisionVerdict v = g.checkArmArm(bothArms(-2.4));
  EXPECT_FALSE(v.ok);
  EXPECT_EQ(v.code, "collision_arm_arm");
  EXPECT_NEAR(v.clearance_m, -0.120000 - kGap, 1e-6);
  EXPECT_LT(v.clearance_m, 0.0);
}

TEST(CollisionGuard, ArmArmVerdictTracksConfiguredGap)
{
  const Q12 q = bothArms(-2.0);  // measured 0.147207 m
  due::CollisionGuard pass;
  pass.configure(0.10, true);
  EXPECT_TRUE(pass.checkArmArm(q).ok);
  EXPECT_NEAR(pass.checkArmArm(q).clearance_m, 0.147207 - 0.10, 1e-6);

  due::CollisionGuard fail;
  fail.configure(0.20, true);
  const due::CollisionVerdict v = fail.checkArmArm(q);
  EXPECT_FALSE(v.ok);
  EXPECT_EQ(v.code, "collision_arm_arm");
  EXPECT_NEAR(v.clearance_m, 0.147207 - 0.20, 1e-6);
}

TEST(CollisionGuard, UnconfiguredGuardFailsClosed)
{
  due::CollisionGuard g;
  const due::CollisionVerdict v = g.checkArmArm(stretched());
  EXPECT_FALSE(v.ok);
  EXPECT_EQ(v.code, "collision_arm_arm");
  EXPECT_TRUE(std::isinf(v.clearance_m));
  EXPECT_LT(v.clearance_m, 0.0);
}

TEST(CollisionGuard, ObjectSpherePass)
{
  due::CollisionGuard g;
  g.configure(kGap, true);
  g.addExclusionSphere(Eigen::Vector3d(1.0, 0.0, 0.0), 0.20);
  const due::CollisionVerdict v = g.checkObjectPose(poseAt(0.0, 0.0, 0.0));
  EXPECT_TRUE(v.ok);
  EXPECT_TRUE(v.code.empty());
  EXPECT_NEAR(v.clearance_m, 0.80, 1e-9);
}

TEST(CollisionGuard, ObjectSphereViolation)
{
  due::CollisionGuard g;
  g.configure(kGap, true);
  g.addExclusionSphere(Eigen::Vector3d(1.0, 0.0, 0.0), 0.20);
  const due::CollisionVerdict v = g.checkObjectPose(poseAt(0.9, 0.0, 0.0));
  EXPECT_FALSE(v.ok);
  EXPECT_EQ(v.code, "collision_object");
  EXPECT_NEAR(v.clearance_m, -0.10, 1e-9);
}

TEST(CollisionGuard, SweepPass)
{
  due::CollisionGuard g;
  g.configure(kGap, true);
  // Endpoints 0.579996 m and 0.147207 m; the segment stays above the gap.
  const due::CollisionVerdict v = g.checkSweep(stretched(), bothArms(-2.0), 16);
  EXPECT_TRUE(v.ok);
  EXPECT_TRUE(v.code.empty());
  EXPECT_NEAR(v.clearance_m, 0.147207 - kGap, 2e-3);
}

TEST(CollisionGuard, SweepViolation)
{
  due::CollisionGuard g;
  g.configure(kGap, true);
  const due::CollisionVerdict v = g.checkSweep(stretched(), bothArms(-2.6), 16);
  EXPECT_FALSE(v.ok);
  EXPECT_EQ(v.code, "collision_sweep");
  EXPECT_LT(v.clearance_m, 0.0);
}

TEST(CollisionGuard, SweepSamplesClampedToAtLeastTwo)
{
  due::CollisionGuard g;
  g.configure(kGap, true);
  for (const int samples : {0, 1, -3}) {
    const due::CollisionVerdict v = g.checkSweep(stretched(), bothArms(-2.6), samples);
    EXPECT_FALSE(v.ok) << "samples=" << samples;
    EXPECT_EQ(v.code, "collision_sweep") << "samples=" << samples;
    EXPECT_LT(v.clearance_m, 0.0) << "samples=" << samples;
  }
  // Clamped to 2 either way; both endpoints are still evaluated.
  EXPECT_FALSE(g.checkSweep(bothArms(-2.6), stretched(), 0).ok);
}

TEST(CollisionGuard, SweepSamplesClampedToAtMost64)
{
  due::CollisionGuard g;
  g.configure(kGap, true);
  const due::CollisionVerdict v = g.checkSweep(stretched(), bothArms(-2.0), 1000);
  EXPECT_TRUE(v.ok);
  EXPECT_TRUE(v.code.empty());
  EXPECT_NEAR(v.clearance_m, 0.147207 - kGap, 2e-3);
}

TEST(CollisionGuard, ObjectChecksDisabled)
{
  due::CollisionGuard g;
  g.configure(kGap, false);
  g.addExclusionSphere(Eigen::Vector3d::Zero(), 1.0);  // origin is deep inside
  const due::CollisionVerdict v = g.checkObjectPose(poseAt(0.0, 0.0, 0.0));
  EXPECT_TRUE(v.ok);
  EXPECT_TRUE(v.code.empty());
  EXPECT_TRUE(std::isinf(v.clearance_m));
  EXPECT_GT(v.clearance_m, 0.0);
}

TEST(CollisionGuard, AddExclusionSphereAccumulatesAndTightens)
{
  due::CollisionGuard g;
  g.configure(kGap, true);
  const Iso3 origin = poseAt(0.0, 0.0, 0.0);

  g.addExclusionSphere(Eigen::Vector3d(1.0, 0.0, 0.0), 0.20);
  EXPECT_NEAR(g.checkObjectPose(origin).clearance_m, 0.80, 1e-9);

  g.addExclusionSphere(Eigen::Vector3d(0.0, 0.3, 0.0), 0.10);
  EXPECT_NEAR(g.checkObjectPose(origin).clearance_m, 0.20, 1e-9);

  g.addExclusionSphere(Eigen::Vector3d(0.0, 0.0, 0.05), 0.10);
  const due::CollisionVerdict v = g.checkObjectPose(origin);
  EXPECT_FALSE(v.ok);
  EXPECT_EQ(v.code, "collision_object");
  EXPECT_NEAR(v.clearance_m, -0.05, 1e-9);

  // Reconfiguring is a clean slate: the spheres above are gone, so the same
  // pose that just failed is no longer evaluated against any geometry.
  g.configure(kGap, true);
  const due::CollisionVerdict cleared = g.checkObjectPose(origin);
  EXPECT_TRUE(cleared.ok);
  EXPECT_TRUE(cleared.code.empty());
  EXPECT_TRUE(std::isinf(cleared.clearance_m));
}
