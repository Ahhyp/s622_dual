// C4.3 S2 / W2 -- S2-T5: branch enumeration with INDEPENDENT forward-kinematic
// verification (the solver's own `fk_verify` stage is never trusted here).
#include <gtest/gtest.h>

#include <string>

#include "dual_arm_transport/grasp_config.hpp"
#include "dual_arm_transport/ik_enum.hpp"

namespace {
using namespace dual_arm_transport;
std::string cfgPath() { return std::string(C43_CONFIG_DIR) + "/object_and_grasps.yaml"; }
}  // namespace

TEST(IkEnum, TargetFramesSitOnTheTwoObjectEndFaces) {
  const GraspConfig cfg = loadGraspConfig(cfgPath());
  const DualGraspTargets t = graspTargets(cfg);
  const Iso3Pair p = tcpPairOf(t);

  // The TCP is INSET from the end face by half a pad: the pad bodies are 20.6 mm
  // wide (measured, see test_local_frame_and_gripper) and must not straddle the
  // end face, so the tool centre sits at 60 mm - 10.3 mm = 49.7 mm.
  const double inset = 0.060 - 0.0103;
  EXPECT_NEAR(p.left.translation().x(), +inset, 1e-9);
  EXPECT_NEAR(p.right.translation().x(), -inset, 1e-9);
  // Both tools point DOWN onto their end face (top-down grasp) ...
  EXPECT_NEAR((p.left.rotation() * dual_arm::Vec3::UnitZ()).z(), -1.0, 1e-12);
  EXPECT_NEAR((p.right.rotation() * dual_arm::Vec3::UnitZ()).z(), -1.0, 1e-12);
  // ... with the SAME orientation: this is not a handshake, it is two identical
  // parallel grippers descending onto the two ends of one rod.
  EXPECT_LT((p.left.rotation() - p.right.rotation()).norm(), 1e-15);
  // The separation between the two tool centres is therefore along the left
  // tool's own +y axis and equals twice the inset.
  const dual_arm::Iso3 lTr = p.left.inverse() * p.right;
  EXPECT_NEAR(lTr.translation().norm(), 2.0 * inset, 1e-9);
  EXPECT_NEAR(lTr.translation().x(), 0.0, 1e-9);
  EXPECT_NEAR(lTr.translation().y(), -2.0 * inset, 1e-9);
  EXPECT_NEAR(lTr.translation().z(), 0.0, 1e-9);
}

TEST(IkEnum, EightBranchesAreEnumeratedAndEachSurvivorIsVerifiedHere) {
  const GraspConfig cfg = loadGraspConfig(cfgPath());
  const DualGraspTargets t = graspTargets(cfg);
  dual_arm::DualArmKinematics kin;

  for (const auto& spec : {std::make_pair(std::string("left"), t.world_T_left_tcp),
                           std::make_pair(std::string("right"), t.world_T_right_tcp)}) {
    const BranchSet b = enumerateBranches(cfg, kin, spec.second, spec.first);
    ASSERT_TRUE(b.ok) << spec.first << ": " << b.failure_stage << "/" << b.failure_code;
    EXPECT_EQ(b.total_branches, 8);
    EXPECT_FALSE(b.solutions.empty());

    // These counters are STAGE-LOCAL: each reports how many of the 8 branches
    // satisfied THAT stage's test, not a cumulative funnel (measured: q5 can
    // exceed q1).  Only the bounds and the final count are meaningful here; the
    // real guarantee is the independent FK check below.
    for (int c : {b.survive_q1, b.survive_q5, b.survive_q23, b.survive_fk_verify,
                  b.survive_unique, b.survive_joint_limits}) {
      EXPECT_GE(c, 0);
      EXPECT_LE(c, b.total_branches);
    }
    EXPECT_EQ(static_cast<int>(b.solutions.size()), b.survive_joint_limits);

    // ★ Independent verification: our own FK must reproduce the commanded target
    // for every solution the solver handed back.
    const dual_arm::Arm arm = (spec.first == "left") ? dual_arm::Arm::Left : dual_arm::Arm::Right;
    for (const dual_arm::JointConfig& q : b.solutions) {
      const dual_arm::Iso3 fk = kin.worldTgrasp(arm, q);
      EXPECT_LT((fk.translation() - spec.second.translation()).norm(), 1e-6);
      EXPECT_LT(dual_arm::logSO3(fk.rotation().transpose() * spec.second.rotation()).norm(), 1e-6);
    }
    ASSERT_FALSE(b.fk_pos_err.empty());
    ASSERT_EQ(b.fk_pos_err.size(), b.fk_rot_err.size());
    for (std::size_t i = 0; i < b.fk_pos_err.size(); ++i) {
      EXPECT_LT(b.fk_pos_err[i], 1e-6);
      EXPECT_LT(b.fk_rot_err[i], 1e-6);
    }
  }
}

TEST(IkEnum, FingerStrokesAreSymmetricAndClearTheSample) {
  const GraspConfig cfg = loadGraspConfig(cfgPath());
  const std::pair<double, double> s = fingerStrokes(cfg, 1e-4);
  EXPECT_GT(s.first, 0.0);
  EXPECT_LT(s.second, 0.0);
  EXPECT_NEAR(s.first + s.second, 0.0, 1e-15);
  EXPECT_NEAR(s.first, cfg.grip_thickness_m / 2.0 + 1e-4, 1e-15);
  // Closing on the sample must not consume the whole stroke -- there has to be a
  // free approach with the pads wider than the sample.
  EXPECT_LT(2.0 * s.first, cfg.gripper_max_opening_m);
}
