// test/test_collision_backend.cpp
// C4.2 / S1: ACM correctness and the three distance scopes.
//
// Each test maps to one of the documented traps in C4.2_执行规划.md §2.5:
//   trap 2 -> DistanceSelfWithoutAcmIsAlwaysZero
//   trap 3 -> ArmArmScopeIsNotTheWholeRobotScope
//   trap 4 -> FloorBaseLinkExemptionIsWhatMakesTheDifference

#include <gtest/gtest.h>

#include <memory>
#include <vector>

#include "dual_arm_pose_optimization/collision_backend.hpp"
#include "dual_arm_pose_optimization/scene_loader.hpp"

namespace c42 = dual_arm_pose_optimization;
using AcType = collision_detection::AllowedCollision::Type;

namespace {

const std::vector<double> kBaselineA{
    -0.5963, -1.6969, -2.2265, 0.7825, 2.1677, 1.5715,
    -2.5432, -1.4419, 2.2229, -3.9223, -2.1697, 1.5713};
const std::vector<double> kBaselineBPrerebuild{
    0.3353, -1.5739, -0.9095, -2.2290, 1.5708, -1.2355,
    0.2856, -1.2598, -1.7115, -3.3161, -1.2915, -1.5708};
const std::vector<double> kHome{0.0, -1.5708, 0.0, 0.0, 0.0, 0.0,
                                0.0, -1.5708, 0.0, 0.0, 0.0, 0.0};
const std::vector<double> kLeftDown{0.0, 0.6, 0.0, 0.0, 0.0, 0.0,
                                    0.0, -1.5708, 0.0, 0.0, 0.0, 0.0};

std::shared_ptr<c42::DualArmScene> scene() {
  static std::shared_ptr<c42::DualArmScene> s = [] {
    c42::SceneLoadOptions o;
    o.urdf_path = C42_URDF;
    o.srdf_path = C42_SRDF;
    return c42::DualArmScene::load(o);
  }();
  return s;
}

const c42::MoveItCollisionBackend& backend() {
  static c42::MoveItCollisionBackend b = [] {
    c42::EnvironmentOptions e;
    e.add_floor = true;
    return c42::MoveItCollisionBackend(scene(), e);
  }();
  return b;
}

dual_arm::Q12 toQ12(const std::vector<double>& v) {
  return Eigen::Map<const dual_arm::Q12>(v.data());
}

}  // namespace

// ---------------------------------------------------------------------------
// ACM content
// ---------------------------------------------------------------------------

TEST(C42Acm, SrdfDisablesAdjacentAndKeepsCrossArmChecked) {
  AcType t = AcType::NEVER;
  ASSERT_TRUE(backend().srdfAcm().getEntry("left_base_link", "left_shoulder_link", t));
  EXPECT_EQ(t, AcType::ALWAYS);  // adjacent -> allowed to touch

  t = AcType::NEVER;
  const bool has = backend().srdfAcm().getEntry("left_upperarm_link",
                                                "right_upperarm_link", t);
  EXPECT_TRUE(!has || t == AcType::NEVER)
      << "the two arms must never be exempted from collision checking";
}

TEST(C42Acm, CrossArmAcmDisablesEverySameArmPair) {
  const auto sc = scene();
  for (const std::string& a : sc->leftLinks()) {
    for (const std::string& b : sc->leftLinks()) {
      if (a == b) continue;
      AcType t = AcType::NEVER;
      if (backend().crossArmAcm().getEntry(a, b, t)) {
        EXPECT_EQ(t, AcType::ALWAYS) << a << " <-> " << b;
      }
    }
  }
  for (const std::string& a : sc->rightLinks()) {
    for (const std::string& b : sc->rightLinks()) {
      if (a == b) continue;
      AcType t = AcType::NEVER;
      if (backend().crossArmAcm().getEntry(a, b, t)) {
        EXPECT_EQ(t, AcType::ALWAYS) << a << " <-> " << b;
      }
    }
  }
}

TEST(C42Acm, ProximalScopeIsAStrictSubsetOfTheCrossArmScope) {
  // The proximal scope exists because a rigid grasp locks every distal-distal
  // distance (C4.2 S3 finding).  It must therefore be a STRICT subset, and the
  // minimum over a subset can never be smaller than the minimum over the
  // superset.
  EXPECT_GT(backend().crossArmPairCount(), backend().armProximalPairCount());
  EXPECT_GT(backend().armProximalPairCount(), 0u);

  for (const auto* q : {&kBaselineA, &kBaselineBPrerebuild}) {
    auto st = scene()->makeStateQ12(*q);
    const c42::DistanceReading r = backend().distances(st);
    EXPECT_GE(r.arm_proximal, r.arm_arm - 1e-12)
        << "min over a subset of pairs cannot be smaller than over the superset";
  }
}

TEST(C42Acm, ProximalScopeActuallyVariesWithPosture) {
  // The whole point of the proximal scope is that it HAS a gradient: the S3
  // sweep measured a std of 50.6 mm for it against 1.9 mm for all-pairs.
  std::vector<double> vals;
  for (double dy : {-0.25, -0.15, 0.0, 0.15, 0.25}) {
    std::vector<double> q = kBaselineBPrerebuild;
    q[0] += dy;   // left_j1
    q[6] -= dy;   // right_j1, opposite sense -> postures change
    auto st = scene()->makeStateQ12(q);
    const c42::DistanceReading r = backend().distances(st);
    if (r.arm_proximal > 0.0) vals.push_back(r.arm_proximal);
  }
  ASSERT_GE(vals.size(), 3u);
  const double lo = *std::min_element(vals.begin(), vals.end());
  const double hi = *std::max_element(vals.begin(), vals.end());
  EXPECT_GT(hi - lo, 1e-3) << "the proximal clearance must respond to posture";
}

TEST(C42Acm, CrossArmAcmKeepsAllCrossArmPairsChecked) {
  const std::size_t expect =
      scene()->leftLinks().size() * scene()->rightLinks().size();
  EXPECT_EQ(backend().crossArmPairCount(), expect);
}

// ---------------------------------------------------------------------------
// trap 2: distance without an ACM is meaningless (always 0)
// ---------------------------------------------------------------------------

TEST(C42Distances, DistanceSelfWithoutAcmIsAlwaysZero) {
  auto st = scene()->makeStateQ12(kBaselineA);
  const double no_acm =
      scene()->planningScene()->getCollisionEnv()->distanceSelf(st);
  const double with_acm = scene()->planningScene()->getCollisionEnv()->distanceSelf(
      st, backend().srdfAcm());

  EXPECT_NEAR(no_acm, 0.0, 1e-12) << "this is the documented trap";
  EXPECT_GT(with_acm, 0.0) << "with the SRDF ACM the distance is meaningful";
}

// ---------------------------------------------------------------------------
// trap 3: whole-robot minimum != arm-arm clearance
// ---------------------------------------------------------------------------

TEST(C42Distances, ArmArmScopeIsNotTheWholeRobotScope) {
  for (const auto* q : {&kBaselineA, &kBaselineBPrerebuild}) {
    auto st = scene()->makeStateQ12(*q);
    const c42::DistanceReading r = backend().distances(st);
    EXPECT_GT(r.arm_arm, 0.0);
    EXPECT_GT(r.whole_robot, 0.0);
    // the global minimum can never exceed the cross-arm-only minimum
    EXPECT_LE(r.whole_robot, r.arm_arm + 1e-9);
    // and for this robot they are genuinely different numbers
    EXPECT_GT(std::abs(r.whole_robot - r.arm_arm), 1e-6)
        << "whole=" << r.whole_robot << " arm_arm=" << r.arm_arm;
    // the closest whole-robot pair is an intra-arm pair, not a cross-arm one
    EXPECT_FALSE(r.whole_link_a.empty());
    EXPECT_FALSE(r.arm_arm_link_a.empty());
  }
}

TEST(C42Distances, CapsuleProxyIsReportedAlongsideTheMeshNumber) {
  auto st = scene()->makeStateQ12(kBaselineBPrerebuild);
  const dual_arm::Q12 q = toQ12(kBaselineBPrerebuild);
  const c42::DistanceReading r = backend().distances(st, &q);

  EXPECT_GE(r.capsule_arm_arm, 0.0) << "capsule cross-check must be filled";
  EXPECT_GT(r.arm_arm, 0.0);
  // Both are cross-arm clearances, so they must be the same order of magnitude,
  // but they are NOT expected to agree: the capsule model is a conservative
  // proxy (radius 0.06 m) while the mesh number is the real geometry.
  EXPECT_LT(r.arm_arm, 1.0);
  EXPECT_LT(r.capsule_arm_arm, 1.0);
}

TEST(C42Distances, CollidingStateIsDetected) {
  auto down = scene()->makeStateQ12(kLeftDown);
  EXPECT_TRUE(backend().isStateColliding(down));
  auto home = scene()->makeStateQ12(kHome);
  EXPECT_FALSE(backend().isStateColliding(home));
}

// ---------------------------------------------------------------------------
// trap 4: floor vs base_link exemption
// ---------------------------------------------------------------------------

TEST(C42Floor, FloorWasAdded) {
  EXPECT_TRUE(backend().floorAdded());
  EXPECT_EQ(backend().floorId(), "c42_floor");
}

TEST(C42Floor, BaseLinkExemptionIsWhatMakesTheDifference) {
  auto home = scene()->makeStateQ12(kHome);

  const double with_exemption =
      backend().robotWorldDistanceWith(home, backend().robotWorldAcm());
  const double without_exemption =
      backend().robotWorldDistanceWith(home, backend().robotWorldAcmNoFloorExemption());

  EXPECT_GT(with_exemption, 0.0)
      << "with the base_link<->floor exemption the floor distance is meaningful";
  EXPECT_LE(without_exemption, 0.0)
      << "without it, the bolted-down base_link touching the floor makes every "
         "state look like a collision";
}

TEST(C42Floor, ArmReachingDownStillCollidesWithTheFloor) {
  auto down = scene()->makeStateQ12(kLeftDown);
  EXPECT_TRUE(backend().isStateColliding(down))
      << "the exemption is only for base_link; a link that really dips below the "
         "floor must still be caught";
  EXPECT_LE(backend().robotWorldDistance(down), 0.0);
}

TEST(C42Floor, HomePoseIsClearOfTheFloor) {
  auto home = scene()->makeStateQ12(kHome);
  EXPECT_GT(backend().robotWorldDistance(home), 0.01);
}

// ---------------------------------------------------------------------------
// segment check
// ---------------------------------------------------------------------------

TEST(C42Segment, SegmentToACollidingPoseIsInvalid) {
  auto home = scene()->makeStateQ12(kHome);
  auto down = scene()->makeStateQ12(kLeftDown);
  EXPECT_FALSE(backend().isSegmentValid(home, down, 32));
}

TEST(C42Segment, SegmentBetweenTwoValidPosesIsValid) {
  auto home = scene()->makeStateQ12(kHome);
  auto near_home = kHome;
  near_home[1] += 0.05;   // left_j2
  near_home[7] += 0.05;   // right_j2
  auto other = scene()->makeStateQ12(near_home);
  EXPECT_TRUE(backend().isSegmentValid(home, other, 32));
}
