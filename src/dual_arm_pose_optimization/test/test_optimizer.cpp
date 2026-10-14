// test/test_optimizer.cpp
// C4.2 / S5: the refiner's contract -- normalisation is bounded, the layered
// penalty keeps infeasible poses strictly below feasible ones, and the V1
// rotation constraint is the Euclidean NORM (not a per-axis box).

#include <gtest/gtest.h>

#include <cmath>
#include <memory>

#include "dual_arm_pose_optimization/m4_baseline.hpp"
#include "dual_arm_pose_optimization/optimizer.hpp"

namespace c42 = dual_arm_pose_optimization;

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
const c42::M4Params& m4() {
  static const c42::M4Params p = c42::M4Params::repositoryDefaults();
  return p;
}
const c42::BaselinePose& baseB() {
  static const c42::BaselinePose b = c42::rebuildBaselineB(scene(), m4());
  return b;
}

}  // namespace

TEST(C42Optimizer, NormalisedObjectivesAreBoundedInUnitInterval) {
  c42::CandidateResult r;
  r.layer.s4_collision_free = true;
  r.smin_left = 1e6;     // absurdly large -> must clamp to 1
  r.smin_right = -5.0;   // negative -> must clamp to 0
  r.w_rel = 1e6;
  r.mesh_arm_arm = 1e6;
  r.joint_margin = 1e6;
  r.deviation = -1.0;    // -> closeness clamps to 1
  r.continuity = 1e6;
  r.arm_proximal_clearance = 1e6;

  const c42::ObjectiveSpec spec = c42::ObjectiveSpec::named("equal");
  const auto j = c42::normalisedObjectives(r, spec);
  for (int i = 0; i < 8; ++i) {
    EXPECT_GE(j[i], 0.0) << i;
    EXPECT_LE(j[i], 1.0) << i;
    EXPECT_TRUE(std::isfinite(j[i])) << i;
  }
  EXPECT_DOUBLE_EQ(j[1], 0.0);
  EXPECT_DOUBLE_EQ(j[5], 1.0);
}

TEST(C42Optimizer, InfeasiblePosesScoreStrictlyBelowFeasibleOnes) {
  const c42::ObjectiveSpec spec = c42::ObjectiveSpec::named("equal");

  c42::CandidateResult feasible;
  feasible.layer = {true, true, true, true, true, true};
  // deliberately poor but feasible metrics
  feasible.smin_left = 0.0; feasible.smin_right = 0.0; feasible.w_rel = 0.0;
  feasible.mesh_arm_arm = 0.0; feasible.joint_margin = 0.0;
  feasible.deviation = 1e6; feasible.continuity = 0.0;
  feasible.arm_proximal_clearance = 0.0;
  const double f_lo = c42::scoreResult(feasible, spec);
  EXPECT_GE(f_lo, 0.0);

  c42::CandidateResult bad = feasible;
  bad.layer.s4_collision_free = false;      // only the last layer missing
  const double b4 = c42::scoreResult(bad, spec);
  EXPECT_LT(b4, 0.0);
  EXPECT_LT(b4, f_lo);

  bad.layer.s3_grasp = false;
  const double b3 = c42::scoreResult(bad, spec);
  EXPECT_LT(b3, b4) << "deeper infeasibility must be penalised harder";

  bad.layer.s2_dual = false;
  EXPECT_LT(c42::scoreResult(bad, spec), b3);

  bad.layer.s0_point_reachable = false;
  EXPECT_LT(c42::scoreResult(bad, spec), c42::scoreResult(bad, spec) + 1e-12);
}

TEST(C42Optimizer, WeightSetsAreNormalisedToPositiveValues) {
  for (const char* name : {"equal", "precision_first", "safety_first"}) {
    const c42::ObjectiveSpec s = c42::ObjectiveSpec::named(name);
    double sum = 0.0;
    for (int i = 0; i < 8; ++i) {
      EXPECT_GT(s.weight[i], 0.0) << name << " " << i;
      EXPECT_GT(s.scale[i], 0.0) << name << " " << i;
      sum += s.weight[i];
    }
    EXPECT_GT(sum, 0.0);
  }
}

TEST(C42Optimizer, V1RespectsTheRotationVectorNormNotAPerAxisBox) {
  c42::FeasibilityPipeline pipeline(scene(), m4(), baseB());
  const Eigen::Matrix3d R = baseB().world_T_object.rotation();

  c42::RefineOptions o;
  o.allow_orientation = true;
  o.theta_max = 10.0 * M_PI / 180.0;
  o.max_evals = 120;

  const c42::ObjectiveSpec spec = c42::ObjectiveSpec::named("equal");
  c42::PoseRefiner ref(&pipeline, nullptr, spec, o);

  c42::PoseParams start;
  start.p_O = baseB().world_T_object.translation();
  const auto r = ref.refine(start, R);

  EXPECT_LE(r.best.rotvec.norm(), o.theta_max + 1e-9)
      << "the constraint is ||dtheta||_2 <= theta_max";
  // a per-axis box would allow sqrt(3)*10 deg; the norm must forbid it
  EXPECT_LT(r.best.rotvec.norm(), std::sqrt(3.0) * o.theta_max);
}
