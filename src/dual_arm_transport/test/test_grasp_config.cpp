// C4.3 S2 / W1 -- object + grasp configuration loading and validation.
// Covers S2-T1..T4 and the D1/D2/D4 rules the user required (m02331).
#include <gtest/gtest.h>

#include <fstream>
#include <string>

#include "dual_arm_transport/grasp_config.hpp"

namespace {
using namespace dual_arm_transport;

std::string cfgPath() { return std::string(C43_CONFIG_DIR) + "/object_and_grasps.yaml"; }

GraspConfig loadShipped() { return loadGraspConfig(cfgPath()); }
}  // namespace

TEST(GraspConfig, ShippedConfigLoadsAndValidates) {
  const GraspConfig cfg = loadShipped();
  EXPECT_EQ(validateGraspConfig(cfg), "") << validateGraspConfig(cfg);
  EXPECT_EQ(cfg.sample_geometry.variant, "box");
  EXPECT_GT(cfg.grasp.mass, 0.0);
  EXPECT_TRUE(cfg.grasp.size_xyz.minCoeff() > 0.0);
}

TEST(GraspConfig, GrippedExtentFitsTheGripperStrokeWithMargin) {
  const GraspConfig cfg = loadShipped();
  // D1: the 0.061 m opening is the STROKE, not a body envelope -- the gripped
  // extent must fit inside it with room for the pad clearance.
  EXPECT_DOUBLE_EQ(cfg.grip_thickness_m, cfg.grasp.size_xyz.y());
  EXPECT_LT(cfg.grip_thickness_m, cfg.gripper_max_opening_m);
  EXPECT_LT(cfg.gripper_max_opening_m - cfg.grip_thickness_m, 0.02);
}

TEST(GraspConfig, SupportHeightAndApproachAreStillCandidates) {
  const GraspConfig cfg = loadShipped();
  // D4 / D7: neither is allowed to arrive pre-approved.
  EXPECT_EQ(cfg.support.top_z_status, "candidate");
  EXPECT_DOUBLE_EQ(cfg.d_approach_m, 0.08);
}

TEST(GraspConfig, EndGraspTransformsMatchTheConfiguredSigns) {
  const GraspConfig cfg = loadShipped();
  const dual_arm::Iso3 gl = makeEndGraspTransform(cfg.grasp.size_xyz, +1, 0.0);
  const dual_arm::Iso3 gr = makeEndGraspTransform(cfg.grasp.size_xyz, -1, 0.0);
  EXPECT_LT((gl.matrix() - cfg.grasp.object_T_left_tcp.matrix()).norm(), 1e-12);
  EXPECT_LT((gr.matrix() - cfg.grasp.object_T_right_tcp.matrix()).norm(), 1e-12);
  // The tool comes DOWN onto each end face: -z_tcp == +z_obj.
  EXPECT_NEAR((gl.rotation() * dual_arm::Vec3::UnitZ()).z(), -1.0, 1e-12);
  EXPECT_NEAR((gr.rotation() * dual_arm::Vec3::UnitZ()).z(), -1.0, 1e-12);
}

TEST(GraspConfig, UnknownKeysAreRejected) {
  std::ifstream in(cfgPath());
  ASSERT_TRUE(in.good());
  std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
  text += "\nbogus_root_key: 3\n";
  const std::string path = "/tmp/c43_test_unknown_key.yaml";
  { std::ofstream out(path); out << text; }
  EXPECT_THROW(loadGraspConfig(path), GraspConfigError);
  std::remove(path.c_str());
}

TEST(GraspConfig, GripThicknessMustEqualTheGrippedExtent) {
  GraspConfig cfg = loadShipped();
  cfg.grip_thickness_m = 0.04;  // size_y is 0.05
  EXPECT_NE(validateGraspConfig(cfg), "");
}

TEST(GraspConfig, ParameterizedRigidSampleMustExplainItself) {
  GraspConfig cfg = loadShipped();
  cfg.sample_geometry.variant = "parameterized_rigid";
  cfg.sample_geometry.reason.clear();
  cfg.sample_geometry.diff_vs_box.clear();
  // D1 allows a non-box sample but forbids changing the shape silently.
  EXPECT_NE(validateGraspConfig(cfg), "");
  cfg.sample_geometry.reason = "independent grasp regions needed";
  cfg.sample_geometry.diff_vs_box = "corner relief 1 mm at both ends";
  EXPECT_EQ(validateGraspConfig(cfg), "");
}

TEST(GraspConfig, BoxInertiaIsSymmetricPositiveDefinite) {
  const dual_arm::Vec3 size(0.12, 0.05, 0.05);
  const dual_arm::Mat3 I = boxInertia(size, 0.35);
  EXPECT_LT((I - I.transpose()).norm(), 1e-15);
  Eigen::LLT<dual_arm::Mat3> llt(I);
  EXPECT_EQ(llt.info(), Eigen::Success);
}
