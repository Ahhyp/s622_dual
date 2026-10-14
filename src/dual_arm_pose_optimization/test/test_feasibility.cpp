// test/test_feasibility.cpp
// C4.2 / S3: the layered ladder, and the structural identity that makes S3
// checkable rather than merely asserted.

#include <gtest/gtest.h>

#include <memory>
#include <vector>

#include "dual_arm_pose_optimization/feasibility.hpp"
#include "dual_arm_pose_optimization/m4_baseline.hpp"

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

c42::FeasibilityPipeline& pipeline() {
  static c42::FeasibilityPipeline p(scene(), m4(), baseB());
  return p;
}

/// The nominal M4 object pose, as a candidate.
c42::Candidate nominalCandidate() {
  c42::Candidate c;
  c.p_O = baseB().world_T_object.translation();
  c.R_O = baseB().world_T_object.rotation();
  c.pool = "test";
  return c;
}

}  // namespace

TEST(C42Feasibility, PipelineRejectsANonRebuiltBaseline) {
  c42::BaselinePose bad = c42::baselineAHistorical();
  EXPECT_THROW(c42::FeasibilityPipeline(scene(), m4(), bad), c42::SceneLoadError);
}

TEST(C42Feasibility, DesiredGraspsReproduceTheRebuiltBaselineAtTheNominalPose) {
  Eigen::Matrix4d TL, TR;
  const c42::Candidate c = nominalCandidate();
  pipeline().desiredGrasps(c, &TL, &TR);

  // At the nominal object pose the desired grasps must coincide with the
  // Baseline B grasps -- otherwise the whole parameterisation is off.
  EXPECT_LT((TL - baseB().world_T_left.matrix()).norm(), 1e-9);
  EXPECT_LT((TR - baseB().world_T_right.matrix()).norm(), 1e-9);

  // and T_R = T_L * T_rel^fixed, i.e. the relative grasp is fixed by construction
  const Eigen::Matrix4d rel = TL.inverse() * TR;
  EXPECT_LT((rel - baseB().left_T_right.matrix()).norm(), 1e-12);
}

TEST(C42Feasibility, RelativeGraspIsInvariantUnderPureTranslation) {
  // V0 moves p_O only; the rigid-grasp model says T_rel must not change.
  c42::Candidate a = nominalCandidate();
  c42::Candidate b = a;
  b.p_O += Eigen::Vector3d(0.01, -0.02, 0.005);

  Eigen::Matrix4d TLa, TRa, TLb, TRb;
  pipeline().desiredGrasps(a, &TLa, &TRa);
  pipeline().desiredGrasps(b, &TLb, &TRb);

  const Eigen::Matrix4d rel_a = TLa.inverse() * TRa;
  const Eigen::Matrix4d rel_b = TLb.inverse() * TRb;
  EXPECT_LT((rel_a - rel_b).norm(), 1e-12);
}

TEST(C42Feasibility, NominalCandidateReachesAllLayers) {
  const c42::CandidateResult r = pipeline().evaluate(nominalCandidate(), nullptr);
  EXPECT_TRUE(r.layer.s2_dual) << "both arms must be able to hold the nominal rod pose";
  EXPECT_TRUE(r.layer.s3_grasp);
  EXPECT_TRUE(r.layer.s4_collision_free);
  EXPECT_TRUE(r.has_chosen_pair);
  EXPECT_GT(r.mesh_arm_arm, 0.0);
  EXPECT_GE(r.capsule_arm_arm, 0.0);
  EXPECT_GT(r.pair_attempts, 0u);
}

TEST(C42Feasibility, S3ResidualCarriesOnlyTheIkError) {
  // This is the structural identity: under the rigid-grasp model H4 is
  // satisfied to IK tolerance for every candidate that reaches S3, so
  // |S3|/|S2| == 100% by construction.  Pinning it down as a TEST means the
  // claim is checkable rather than hand-waved.
  const c42::CandidateResult r = pipeline().evaluate(nominalCandidate(), nullptr);
  ASSERT_TRUE(r.layer.s3_grasp);
  EXPECT_LT(r.rel_grasp_pos_err, 1e-9);
  EXPECT_LT(r.rel_grasp_rot_err, 1e-9);
}

TEST(C42Feasibility, LayersAreNested) {
  // Nesting must hold for every candidate, whatever the outcome.
  std::vector<c42::Candidate> cands;
  const auto& p0 = nominalCandidate();
  cands.push_back(p0);

  const Eigen::Vector3d offsets[] = {
      {0, 0, 0}, {0.05, 0, 0}, {-0.05, 0, 0}, {0, 0.10, 0},
      {0, -0.10, 0}, {0, 0, 0.08}, {0, 0, -0.05}, {0.20, 0.0, 0.0},
      {0.0, 0.0, 0.30}, {0.5, 0.5, 0.3},
  };
  for (const auto& o : offsets) {
    c42::Candidate c = p0;
    c.p_O += o;
    cands.push_back(c);
  }

  for (const auto& c : cands) {
    const c42::CandidateResult r = pipeline().evaluate(c, nullptr);
    EXPECT_LE(r.layer.s2_dual, r.layer.s1_left);
    EXPECT_LE(r.layer.s2_dual, r.layer.s1_right);
    EXPECT_LE(r.layer.s3_grasp, r.layer.s2_dual);
    EXPECT_LE(r.layer.s4_collision_free, r.layer.s3_grasp);
    if (r.layer.s4_collision_free) {
      EXPECT_TRUE(r.has_chosen_pair);
      EXPECT_GT(r.mesh_arm_arm, 0.0) << "an S4 candidate must have a measured clearance";
    }
  }
}

TEST(C42Feasibility, FarAwayCandidateFailsThePoseLayers) {
  c42::Candidate c = nominalCandidate();
  c.p_O += Eigen::Vector3d(2.0, 2.0, 2.0);  // unreachable for a ~0.75 m arm
  const c42::CandidateResult r = pipeline().evaluate(c, nullptr);
  EXPECT_FALSE(r.layer.s2_dual);
  EXPECT_FALSE(r.layer.s3_grasp);
  EXPECT_FALSE(r.layer.s4_collision_free);
}

TEST(C42Feasibility, S0UsesTheC41MasksWhenProvided) {
  // Without an index S0 is assumed true; with an index it must actually filter.
  const c42::Candidate nominal = nominalCandidate();
  EXPECT_TRUE(pipeline().evaluate(nominal, nullptr).layer.s0_point_reachable);

  c42::Candidate absurd = nominal;
  absurd.p_O = Eigen::Vector3d(5.0, 5.0, 5.0);
  // A mask that contains nothing: build a tiny one lazily is expensive, so use
  // the real C4.1 index when available and just assert the direction of the
  // filter on an obviously-outside point.
  const char* bin = std::getenv("C42_C41_BIN");
  if (bin == nullptr) {
    GTEST_SKIP() << "set C42_C41_BIN to exercise the S0 mask";
  }
  c42::WorkspaceIndex::Options o;
  o.c41_bin = bin;
  o.max_handover_samples = 100;
  const c42::WorkspaceIndex idx = c42::WorkspaceIndex::load(o);
  EXPECT_FALSE(pipeline().evaluate(absurd, &idx).layer.s0_point_reachable);
}
