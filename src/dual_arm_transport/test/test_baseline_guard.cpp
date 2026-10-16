// C4.3 S2 / W7 -- S2-T7: the R1 regression defense.  The C4.3 baseline must not
// be the legacy midpoint, and it must not be the C4.2 M4 handover pose either.
#include <gtest/gtest.h>

#include <cmath>
#include <string>

#include "dual_arm_kinematics/dual_arm_kinematics.hpp"
#include "dual_arm_kinematics/se3.hpp"
#include "dual_arm_transport/baseline.hpp"
#include "dual_arm_transport/grasp_config.hpp"

namespace {
using namespace dual_arm_transport;
std::string cfgPath() { return std::string(C43_CONFIG_DIR) + "/object_and_grasps.yaml"; }
std::string baselinePath() { return std::string(C43_CONFIG_DIR) + "/transport_baseline.yaml"; }
}  // namespace

// The C4.2 M4 handover configuration, read back from C4.2_S2_验收记录.md, brings
// the two TCPs 55.36 mm apart with a 94.02 deg relative rotation.  A fixed
// dual-end grasp of this 0.12 m sample needs ~0.12 m and ~pi.  The two cannot be
// confused, which is what lets the screen tell a handover apart from a grasp.
TEST(BaselineGuard, C42M4HandoverIsNotAFixedDualEndGrasp) {
  const dual_arm::Q12 m4 = c42M4HandoverQ12();
  dual_arm::DualArmKinematics kin;
  const dual_arm::Iso3 lTr = kin.leftTright(m4);
  const double separation = lTr.translation().norm();
  const double relative = dual_arm::logSO3(lTr.rotation()).norm();

  EXPECT_NEAR(separation, 0.05536, 2e-3);
  EXPECT_NEAR(relative, 1.64100, 5e-3);  // 94.02 deg

  const GraspConfig cfg = loadGraspConfig(cfgPath());
  const double span = cfg.grasp.size_xyz.x();  // dual-end grasp TCP separation
  EXPECT_GT(std::abs(span - separation), 0.05);
  // A dual-end grasp needs a relative rotation of ~pi; the handover sits near a
  // right angle, so the two hypotheses are more than a radian apart.
  EXPECT_GT(std::abs(M_PI - relative), 1.0);
  // And the two configurations must not coincide.
  const dual_arm::Q12 b = readBaseline(baselinePath()).q_grasp;
  EXPECT_GT((b - m4).norm(), 1e-3);
}

TEST(BaselineGuard, FrozenBaselineIsAnchoredAndNotTheMidpoint) {
  const TransportBaseline b = readBaseline(baselinePath());
  EXPECT_EQ(b.schema, "c43_s2_transport_baseline/v1");
  EXPECT_EQ(b.stage, "C4.3-S2");
  EXPECT_NE(b.provenance, "");

  // The two independent anchors agree to machine precision ...
  EXPECT_LT(b.anchor_pos_gap_m, 1e-9);
  EXPECT_LT(b.anchor_rot_gap_rad, 1e-9);
  EXPECT_LT(b.closure_pos_m, 1e-9);
  EXPECT_LT(b.closure_rot_rad, 1e-9);
  // ... while the legacy midpoint is 25 mm and pi away.  This is the record that
  // the baseline is NOT a midpoint construction.
  EXPECT_NEAR(b.legacy_midpoint_err_pos_m, 0.025, 1e-6);
  EXPECT_NEAR(b.legacy_midpoint_err_rot_rad, M_PI, 1e-6);

  EXPECT_GE(b.clearance_m, 2e-3);
  EXPECT_GT(b.sigma_min, 1e-4);
  EXPECT_EQ(b.n_pairs, b.n_feasible + [&] {
    int r = 0;
    for (const auto& kv : b.reject_histogram) r += kv.second;
    return r;
  }());
  // The chosen pair really is one of the pairs the screen examined.
  EXPECT_GE(b.left_index, 0);
  EXPECT_GE(b.right_index, 0);
}

TEST(BaselineGuard, BaselineRecordsItsOwnScopeExclusions) {
  const TransportBaseline b = readBaseline(baselinePath());
  ASSERT_FALSE(b.not_covered.empty());
  bool says_no_transport = false;
  for (const std::string& s : b.not_covered)
    if (s.find("transport") != std::string::npos) says_no_transport = true;
  // D7: a post-grasp transport trajectory is out of S2 scope and the artifact
  // must say so rather than quietly implying that it was validated.
  EXPECT_TRUE(says_no_transport);
}

TEST(BaselineGuard, SupportHeightWasScreenedNotAssumed) {
  const TransportBaseline b = readBaseline(baselinePath());
  EXPECT_EQ(b.support.top_z_status, "screened");
  // The sample rests exactly on the screened support top.
  EXPECT_NEAR(b.world_T_object.translation().z() - b.size_xyz.z() / 2.0, b.support.top_z_m, 1e-9);
}
