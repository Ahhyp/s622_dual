// C4.3 S2 / W5+W6 -- S2-T9/T18: joint-space standby reachability and an
// INDEPENDENT check of the final approach (user m02331 D7).  A joint
// interpolation cannot track a Cartesian straight line, so straightness is
// measured and reported but is NOT a rejection unless explicitly requested.
#include <gtest/gtest.h>

#include <memory>
#include <string>

#include "dual_arm_pose_optimization/scene_loader.hpp"
#include "dual_arm_transport/baseline.hpp"
#include "dual_arm_transport/grasp_config.hpp"
#include "dual_arm_transport/path_check.hpp"

namespace {
using namespace dual_arm_transport;

std::string cfgPath() { return std::string(C43_CONFIG_DIR) + "/object_and_grasps.yaml"; }
std::string baselinePath() { return std::string(C43_CONFIG_DIR) + "/transport_baseline.yaml"; }

std::shared_ptr<dual_arm_pose_optimization::DualArmScene> loadScene() {
  dual_arm_pose_optimization::SceneLoadOptions o;
  o.urdf_path = C43_URDF;
  o.srdf_path = C43_SRDF;
  o.require_meshes = true;
  return dual_arm_pose_optimization::DualArmScene::load(o);
}
}  // namespace

TEST(PathCheck, PregraspBacksOffAlongTheToolZ) {
  const auto scene = loadScene();
  const GraspConfig cfg = loadGraspConfig(cfgPath());
  PathChecker pc(scene, cfg);

  dual_arm::Iso3 g = dual_arm::Iso3::Identity();
  g.translate(dual_arm::Vec3(0.1, 0.2, 0.3));
  g.rotate(Eigen::AngleAxisd(0.4, dual_arm::Vec3::UnitY()));
  const Iso3Pair back = pc.pregraspTargets(Iso3Pair{g, g});

  for (const dual_arm::Iso3* p : {&back.left, &back.right}) {
    // The retreat is exactly d_approach along the tool's own -z axis: +z_tcp
    // points from the tool INTO the sample, so backing away is -z_tcp.
    const dual_arm::Iso3 delta = g.inverse() * (*p);
    EXPECT_NEAR(delta.translation().norm(), cfg.d_approach_m, 1e-12);
    EXPECT_NEAR(delta.translation().z(), -cfg.d_approach_m, 1e-12);
    EXPECT_NEAR(delta.translation().x(), 0.0, 1e-12);
    EXPECT_NEAR(delta.translation().y(), 0.0, 1e-12);
    // Orientation is untouched: this is a pure translation along the approach.
    EXPECT_LT(dual_arm::logSO3(delta.rotation()).norm(), 1e-12);
  }
}

TEST(PathCheck, JointLimitsRejectAnImpossibleConfiguration) {
  const auto scene = loadScene();
  const GraspConfig cfg = loadGraspConfig(cfgPath());
  PathChecker pc(scene, cfg);

  dual_arm::Q12 bad = dual_arm::Q12::Zero();
  bad(3) = 100.0;  // far outside any arm joint limit
  std::string detail;
  EXPECT_FALSE(pc.withinJointLimits(bad, &detail));
  EXPECT_FALSE(detail.empty());
}

// S2-T18: the final approach is validated on its own, and the frozen baseline
// passes that validation.
TEST(PathCheck, FrozenBaselineStandbyAndApproachBothValidate) {
  const auto scene = loadScene();
  const GraspConfig cfg = loadGraspConfig(cfgPath());
  const TransportBaseline b = readBaseline(baselinePath());

  PathChecker pc(scene, cfg);
  EXPECT_FALSE(pc.options().require_straight_approach);
  const Iso3Pair grasp{b.world_T_left_tcp, b.world_T_right_tcp};
  const ReachabilityResult rr = pc.check(b.q_standby, b.q_grasp, grasp);

  EXPECT_TRUE(rr.ok) << rr.code << ": " << rr.detail;
  EXPECT_EQ(rr.standby_segment.code, "ok") << rr.standby_segment.detail;
  EXPECT_EQ(rr.approach_segment.code, "ok") << rr.approach_segment.detail;
  EXPECT_GT(rr.standby_segment.n_samples, 0u);
  EXPECT_GT(rr.approach_segment.n_samples, 0u);
  EXPECT_GE(rr.standby_segment.min_clearance_m, 2e-3);
  EXPECT_GE(rr.approach_segment.min_clearance_m, 2e-3);
  // The two segments are distinct objects with their own measurements.
  EXPECT_NE(&rr.standby_segment, &rr.approach_segment);
}

TEST(PathCheck, StraightnessIsReportedEvenWhenItIsNotEnforced) {
  const auto scene = loadScene();
  const GraspConfig cfg = loadGraspConfig(cfgPath());
  const TransportBaseline b = readBaseline(baselinePath());
  const Iso3Pair grasp{b.world_T_left_tcp, b.world_T_right_tcp};

  PathChecker relaxed(scene, cfg);
  const ReachabilityResult a = relaxed.check(b.q_standby, b.q_grasp, grasp);
  // A joint interpolation genuinely departs from the straight line: the metric
  // is non-zero and yet the check still passes.
  EXPECT_GT(a.approach_segment.max_line_dev_m, 1e-4);
  EXPECT_EQ(a.approach_segment.code, "ok");

  // Turning the requirement on must change the verdict -- proving the metric is
  // actually wired into the decision, and that D7's relaxation is a deliberate
  // choice rather than a silently disabled check.
  PathCheckOptions strict_opt;
  strict_opt.require_straight_approach = true;
  PathChecker strict(scene, cfg, strict_opt);
  const ReachabilityResult s = strict.check(b.q_standby, b.q_grasp, grasp);
  EXPECT_FALSE(s.ok);
  EXPECT_EQ(s.approach_segment.code, "approach_not_straight");
  EXPECT_GT(s.approach_segment.max_line_dev_m, strict_opt.approach_line_tol_m);
}
