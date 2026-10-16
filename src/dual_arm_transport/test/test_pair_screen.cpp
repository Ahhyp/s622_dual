// C4.3 S2 / W3+W4 -- S2-T6/T8/T10..T13: the cartesian product of the two arms'
// verified branches, screened for independent anchoring, closure consistency,
// dexterity and collision (gripper bodies included).
#include <gtest/gtest.h>

#include <map>
#include <memory>
#include <string>

#include "dual_arm_pose_optimization/scene_loader.hpp"
#include "dual_arm_transport/grasp_config.hpp"
#include "dual_arm_transport/ik_enum.hpp"
#include "dual_arm_transport/pair_screen.hpp"

namespace {
using namespace dual_arm_transport;

std::string cfgPath() { return std::string(C43_CONFIG_DIR) + "/object_and_grasps.yaml"; }

struct Fixture {
  std::shared_ptr<dual_arm_pose_optimization::DualArmScene> scene;
  GraspConfig cfg;
  DualGraspTargets targets;
  BranchSet left, right;
  ScreenReport rep;
};

Fixture runScreen() {
  Fixture f;
  dual_arm_pose_optimization::SceneLoadOptions o;
  o.urdf_path = C43_URDF;
  o.srdf_path = C43_SRDF;
  o.require_meshes = true;
  f.scene = dual_arm_pose_optimization::DualArmScene::load(o);
  f.cfg = loadGraspConfig(cfgPath());
  f.targets = graspTargets(f.cfg);

  dual_arm::DualArmKinematics kin;
  f.left = enumerateBranches(f.cfg, kin, f.targets.world_T_left_tcp, "left");
  f.right = enumerateBranches(f.cfg, kin, f.targets.world_T_right_tcp, "right");

  PairScreener scr(f.scene, f.cfg);
  scr.applyWorld(f.targets.world_T_object);
  f.rep = scr.screen(f.left, f.right);
  return f;
}
}  // namespace

TEST(PairScreen, TheCartesianProductIsFullyAccountedFor) {
  const Fixture f = runScreen();
  const int expected = static_cast<int>(f.left.solutions.size() * f.right.solutions.size());
  EXPECT_EQ(f.rep.n_pairs, expected);
  int rejected = 0;
  for (const auto& kv : f.rep.reject_histogram) rejected += kv.second;
  EXPECT_EQ(f.rep.n_feasible + rejected, f.rep.n_pairs);
  EXPECT_EQ(static_cast<int>(f.rep.candidates.size()), f.rep.n_pairs);
}

TEST(PairScreen, FeasiblePairsSatisfyEveryCriterion) {
  const Fixture f = runScreen();
  ASSERT_GE(f.rep.n_feasible, 1);
  for (const PairCandidate& c : f.rep.candidates) {
    if (!c.feasible) continue;
    EXPECT_FALSE(c.collides);
    EXPECT_GE(c.min_clearance_m, 2e-3);
    EXPECT_GE(c.sigma_min, 1e-4);
    EXPECT_LT(c.anchor_pos_gap_m, 1e-6);
    EXPECT_LT(c.anchor_rot_gap_rad, 1e-6);
    EXPECT_LT(c.closure_pos_m, 1e-6);
    EXPECT_LT(c.closure_rot_rad, 1e-6);
    EXPECT_FALSE(c.closure_dir_ambiguous);
    EXPECT_TRUE(c.reject_code.empty());
  }
}

// S2-T12: the independent anchors must agree with each other but NOT with the
// legacy midpoint -- that disagreement is the whole point of C4.3.
TEST(PairScreen, TheTwoAnchorsAgreeWhileTheLegacyMidpointDoesNot) {
  const Fixture f = runScreen();
  ASSERT_GE(f.rep.n_feasible, 1);
  for (const PairCandidate& c : f.rep.candidates) {
    if (!c.feasible) continue;
    EXPECT_LT(c.anchor_pos_gap_m, 1e-12);
    EXPECT_LT(c.anchor_rot_gap_rad, 1e-12);
    // Measured for this geometry: the midpoint is 25 mm / pi away.
    EXPECT_GT(c.midpoint_vs_task_pos_m, 1e-3);
    EXPECT_GT(c.midpoint_vs_task_rot_rad, 1.0);
  }
}

TEST(PairScreen, EveryRejectionCarriesAnAttributedReason) {
  const Fixture f = runScreen();
  ASSERT_GT(f.rep.n_pairs, 0);
  for (const PairCandidate& c : f.rep.candidates) {
    if (c.feasible) continue;
    EXPECT_FALSE(c.reject_code.empty()) << c.left_index << "," << c.right_index;
  }
  // The histogram must only contain codes that actually appear on a candidate.
  for (const auto& kv : f.rep.reject_histogram) {
    EXPECT_GT(kv.second, 0) << kv.first;
  }
}

TEST(PairScreen, RejectHistogramMatchesTheCandidates) {
  const Fixture f = runScreen();
  std::map<std::string, int> counted;
  for (const PairCandidate& c : f.rep.candidates)
    if (!c.feasible) counted[c.reject_code]++;
  EXPECT_EQ(counted, f.rep.reject_histogram);
}
