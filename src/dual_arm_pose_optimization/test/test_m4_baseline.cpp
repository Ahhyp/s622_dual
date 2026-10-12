// test/test_m4_baseline.cpp
// C4.2 / S2: the official Baseline B must be REBUILT via IK, never taken from
// `right_receive_base` verbatim (plan §2.1.1, risk R5b).

#include <gtest/gtest.h>

#include <memory>

#include "dual_arm_pose_optimization/collision_backend.hpp"
#include "dual_arm_pose_optimization/m4_baseline.hpp"

namespace c42 = dual_arm_pose_optimization;
using dual_arm::JointConfig;

namespace {

std::shared_ptr<c42::DualArmScene> scene() {
  static std::shared_ptr<c42::DualArmScene> s = [] {
    c42::SceneLoadOptions o;
    o.urdf_path = C42_URDF;
    o.srdf_path = C42_SRDF;
    return c42::DualArmScene::load(o);
  }();
  return s;
}

const c42::M4Params& params() {
  static const c42::M4Params p = c42::M4Params::repositoryDefaults();
  return p;
}

const c42::BaselinePose& baselineB() {
  static const c42::BaselinePose b = c42::rebuildBaselineB(scene(), params());
  return b;
}

}  // namespace

TEST(C42BaselineB, IsMarkedOfficialButProvisional) {
  const auto& b = baselineB();
  EXPECT_TRUE(b.official);
  EXPECT_TRUE(b.provisional) << "Q1..Q4 are unresolved -> provisional_rebuilt";
  EXPECT_TRUE(b.rebuilt_via_ik);
  EXPECT_EQ(b.id, "M4_baseline_B");
}

TEST(C42BaselineB, RightArmIsSolvedNotCopiedFromTheSeed) {
  const auto& b = baselineB();
  const double gap = (b.q_right - params().right_receive_base).norm();
  EXPECT_GT(gap, 1e-6)
      << "using right_receive_base verbatim is the documented failure mode (R5b)";
}

TEST(C42BaselineB, ResidualIsAtIkToleranceAndFixesTheMillimetreError) {
  const auto& b = baselineB();
  EXPECT_LT(b.post_rebuild_pos_err, 1e-4) << "IK residual must be sub-0.1 mm";
  EXPECT_LT(b.post_rebuild_rot_err, 1e-4);
  EXPECT_GT(b.pre_rebuild_pos_err, 0.003)
      << "the un-rebuilt base point is expected to be a few mm off";
  EXPECT_LT(b.pre_rebuild_pos_err, 0.020);
  EXPECT_LT(b.post_rebuild_pos_err * 10.0, b.pre_rebuild_pos_err)
      << "the rebuild must reduce the error by at least 10x";
}

TEST(C42BaselineB, SolverEnumeratedBranchesAndTheSelectorPassedOne) {
  const auto& b = baselineB();
  EXPECT_GT(b.right_ik.total_branches, 1);
  EXPECT_FALSE(b.right_ik.solutions.empty());
  EXPECT_TRUE(b.right_ik.ok);
  EXPECT_GE(b.right_ik.n_candidates, 1u);
  EXPECT_GE(b.right_ik.n_hard_pass, 1u);
  EXPECT_TRUE(b.right_ik.selected.has_value());
}

TEST(C42BaselineB, PosesAreConsistentWithFk) {
  const auto& b = baselineB();
  dual_arm::DualArmKinematics kin;
  const auto wl = kin.worldTgrasp(dual_arm::Arm::Left, b.q_left);
  const auto wr = kin.worldTgrasp(dual_arm::Arm::Right, b.q_right);
  EXPECT_LT((wl.translation() - b.world_T_left.translation()).norm(), 1e-12);
  EXPECT_LT((wr.translation() - b.world_T_right.translation()).norm(), 1e-12);

  // ^L T_R must be exactly the FK-consistent relative pose
  const auto rel = kin.leftTright(b.q12());
  EXPECT_LT((rel.translation() - b.left_T_right.translation()).norm(), 1e-12);

  // the rod must sit above the floor and be reachable-looking
  EXPECT_GT(b.world_T_object.translation().z(), 0.10);
  EXPECT_LT(b.world_T_object.translation().z(), 0.60);
}

TEST(C42BaselineB, TargetPoseMatchesTheDeclaredReceiveConvention) {
  const auto& b = baselineB();
  // p_target = rod_centre + receive_offset (world axes), R = receive_quat
  const Eigen::Vector3d expect =
      b.world_T_object.translation() + params().receive_offset_world;
  EXPECT_LT((b.target_world.block<3, 1>(0, 3) - expect).norm(), 1e-12);

  const Eigen::Matrix3d R = params().receive_quat.normalized().toRotationMatrix();
  EXPECT_LT((b.target_world.block<3, 3>(0, 0) - R).norm(), 1e-12);
}

TEST(C42BaselineB, IsCollisionFreeWithTheRodInTheScene) {
  const auto& b = baselineB();
  c42::addRodToScene(scene(), b, params());
  c42::EnvironmentOptions env;
  env.add_floor = true;
  c42::MoveItCollisionBackend backend(scene(), env);
  const auto st = scene()->makeStateQ12(b.q12Vector());
  const dual_arm::Q12 q12 = b.q12();
  const auto r = backend.distances(st, &q12);

  EXPECT_FALSE(r.in_collision) << "the M4 RECEIVE configuration must be self-collision free";
  EXPECT_GT(r.arm_arm, 0.0);
  EXPECT_GE(r.capsule_arm_arm, 0.0);
}

TEST(C42ObjectAcm, GripperLinksAreExemptFromTheObject) {
  const auto& b = baselineB();
  const auto acm = c42::addRodToScene(scene(), b, params());
  for (const std::string& link : params().object_touch_links) {
    collision_detection::AllowedCollision::Type t =
        collision_detection::AllowedCollision::NEVER;
    const bool has = acm.getEntry(link, params().object_id, t);
    EXPECT_TRUE(has && t == collision_detection::AllowedCollision::ALWAYS) << link;
  }
  // ...but a non-gripper link must NOT be exempt
  collision_detection::AllowedCollision::Type t = collision_detection::AllowedCollision::NEVER;
  const bool has = acm.getEntry("left_upperarm_link", params().object_id, t);
  EXPECT_TRUE(!has || t == collision_detection::AllowedCollision::NEVER);
}

TEST(C42BaselineA, HistoricalAnnexIsClearlyNotOfficial) {
  const c42::BaselinePose a = c42::baselineAHistorical();
  EXPECT_FALSE(a.official);
  EXPECT_FALSE(a.rebuilt_via_ik);
  EXPECT_EQ(a.id, "baseline_A_historical");
  // values reported in the S1 recon / plan annex
  EXPECT_NEAR(a.world_T_left.translation().x(), 0.0205, 2e-3);
  EXPECT_NEAR(a.world_T_left.translation().y(), 0.1996, 2e-3);
  EXPECT_NEAR(a.left_T_right.translation().norm(), 0.0406, 2e-3);
}
