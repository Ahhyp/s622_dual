// C4.3 S2 / W1+W7 -- S2-T16: the support height and the approach distance are
// candidates that the screening DISPOSES OF; neither may arrive pre-approved
// (user m02331 D4/D7), and the artifact must record which state it is in.
#include <gtest/gtest.h>

#include <string>

#include "dual_arm_transport/baseline.hpp"
#include "dual_arm_transport/grasp_config.hpp"

namespace {
using namespace dual_arm_transport;
std::string cfgPath() { return std::string(C43_CONFIG_DIR) + "/object_and_grasps.yaml"; }
std::string baselinePath() { return std::string(C43_CONFIG_DIR) + "/transport_baseline.yaml"; }
}  // namespace

TEST(Support, TheConfigDeclaresItsHeightACandidate) {
  const GraspConfig cfg = loadGraspConfig(cfgPath());
  EXPECT_EQ(cfg.support.top_z_status, "candidate");
  EXPECT_FALSE(cfg.support.id.empty());
  EXPECT_GT(cfg.support.height_m, 0.0);
  EXPECT_GT(cfg.support.radius_m, 0.0);
}

// D4 demands that the support height be determined BY the screening and not
// assumed.  What the artifact can prove is the DERIVATION: the height is recomputed
// from the chosen placement (object centre z minus half the sample height) and the
// status flips candidate -> screened.  It cannot prove that the search MOVED the
// value, because in this run the very first candidate placement already screened
// feasible -- the frozen height coincides with the candidate, and this test says so
// plainly rather than manufacturing a difference that the run did not produce.
TEST(Support, TheHeightIsDerivedFromTheChosenPlacementNotCopiedFromTheCandidate) {
  const GraspConfig cfg = loadGraspConfig(cfgPath());
  const TransportBaseline b = readBaseline(baselinePath());

  // The input is explicitly a candidate ...
  EXPECT_EQ(cfg.support.top_z_status, "candidate");
  // ... and the output explicitly is not.
  EXPECT_EQ(b.support.top_z_status, "screened");

  // The frozen height is a FUNCTION of the frozen placement.
  EXPECT_NEAR(b.support.top_z_m, b.world_T_object.translation().z() - b.size_xyz.z() / 2.0, 1e-9);
  EXPECT_DOUBLE_EQ(b.support.height_m, b.support.top_z_m);
  // The sample really rests on the support: no float and no sinking.
  EXPECT_NEAR(b.world_T_object.translation().z() - b.size_xyz.z() / 2.0, b.support.top_z_m, 1e-9);

  // The first candidate placement won, so the screened height happens to equal the
  // candidate.  That is a coincidence of this run, NOT the source of the value, and
  // the acceptance record states it.  If a later run moves the placement, this
  // assertion is the one that will have to change.
  EXPECT_DOUBLE_EQ(b.support.top_z_m, 0.25);
  EXPECT_DOUBLE_EQ(cfg.support.top_z_m, b.support.top_z_m);
}

TEST(Support, AnInventedStatusIsRejected) {
  GraspConfig cfg = loadGraspConfig(cfgPath());
  cfg.support.top_z_status = "assumed";
  EXPECT_NE(validateGraspConfig(cfg), "");
}

TEST(Approach, TheRetreatDistanceIsStillACandidate) {
  const GraspConfig cfg = loadGraspConfig(cfgPath());
  const TransportBaseline b = readBaseline(baselinePath());
  EXPECT_DOUBLE_EQ(cfg.d_approach_m, 0.08);
  EXPECT_DOUBLE_EQ(b.d_approach_m, 0.08);
  EXPECT_GT(b.d_approach_m, 0.0);
}

TEST(Baseline, RoundTripsThroughYamlWithoutLosingAnything) {
  const TransportBaseline b = readBaseline(baselinePath());
  const TransportBaseline c = TransportBaseline::fromYaml(b.toYaml());

  EXPECT_EQ(c.schema, b.schema);
  EXPECT_EQ(c.stage, b.stage);
  EXPECT_EQ(c.object_frame, b.object_frame);
  EXPECT_NEAR((c.world_T_object.matrix() - b.world_T_object.matrix()).norm(), 0.0, 1e-12);
  EXPECT_LT((c.size_xyz - b.size_xyz).norm(), 1e-12);
  EXPECT_DOUBLE_EQ(c.mass, b.mass);

  EXPECT_EQ(c.support.id, b.support.id);
  EXPECT_EQ(c.support.shape, b.support.shape);
  EXPECT_DOUBLE_EQ(c.support.radius_m, b.support.radius_m);
  EXPECT_DOUBLE_EQ(c.support.height_m, b.support.height_m);
  EXPECT_DOUBLE_EQ(c.support.top_z_m, b.support.top_z_m);
  EXPECT_EQ(c.support.top_z_status, b.support.top_z_status);

  EXPECT_EQ(c.q_standby_id, b.q_standby_id);
  EXPECT_LT((c.q_standby - b.q_standby).norm(), 1e-9);
  EXPECT_LT((c.q_grasp - b.q_grasp).norm(), 1e-9);
  ASSERT_EQ(c.standby_candidates.size(), b.standby_candidates.size());

  EXPECT_EQ(c.left_index, b.left_index);
  EXPECT_EQ(c.right_index, b.right_index);
  EXPECT_EQ(c.left_tcp_frame, b.left_tcp_frame);
  EXPECT_EQ(c.right_tcp_frame, b.right_tcp_frame);
  EXPECT_LT((c.world_T_left_tcp.matrix() - b.world_T_left_tcp.matrix()).norm(), 1e-9);
  EXPECT_LT((c.world_T_right_tcp.matrix() - b.world_T_right_tcp.matrix()).norm(), 1e-9);
  EXPECT_LT((c.object_T_left_tcp.matrix() - b.object_T_left_tcp.matrix()).norm(), 1e-9);
  EXPECT_LT((c.object_T_right_tcp.matrix() - b.object_T_right_tcp.matrix()).norm(), 1e-9);

  EXPECT_NEAR(c.clearance_m, b.clearance_m, 1e-9);
  EXPECT_NEAR(c.sigma_min, b.sigma_min, 1e-9);
  EXPECT_NEAR(c.legacy_midpoint_err_pos_m, b.legacy_midpoint_err_pos_m, 1e-9);
  EXPECT_NEAR(c.legacy_midpoint_err_rot_rad, b.legacy_midpoint_err_rot_rad, 1e-9);

  EXPECT_EQ(c.standby_segment.code, b.standby_segment.code);
  EXPECT_EQ(c.approach_segment.code, b.approach_segment.code);
  EXPECT_NEAR(c.approach_segment.max_line_dev_m, b.approach_segment.max_line_dev_m, 1e-9);

  EXPECT_EQ(c.n_pairs, b.n_pairs);
  EXPECT_EQ(c.n_feasible, b.n_feasible);
  EXPECT_EQ(c.reject_histogram, b.reject_histogram);
  EXPECT_EQ(c.not_covered, b.not_covered);
}

TEST(Baseline, ReadBackRefusesToInventAMissingFile) {
  EXPECT_THROW(readBaseline("/tmp/c43_no_such_baseline_9f3a.yaml"), std::exception);
}
