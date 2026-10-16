// C4.3 S2 / W1 -- S2-T14 (grasp data is object-local) and S2-T15 (the dual-end
// grasp is interference free INCLUDING the gripper bodies, not just the fingers).
#include <gtest/gtest.h>

#include <memory>
#include <string>

#include "dual_arm_pose_optimization/scene_loader.hpp"
#include "dual_arm_transport/grasp_config.hpp"
#include "dual_arm_transport/ik_enum.hpp"
#include "dual_arm_transport/pair_screen.hpp"

namespace {
using namespace dual_arm_transport;

std::string cfgPath() { return std::string(C43_CONFIG_DIR) + "/object_and_grasps.yaml"; }

std::shared_ptr<dual_arm_pose_optimization::DualArmScene> loadScene() {
  dual_arm_pose_optimization::SceneLoadOptions o;
  o.urdf_path = C43_URDF;
  o.srdf_path = C43_SRDF;
  o.require_meshes = true;
  return dual_arm_pose_optimization::DualArmScene::load(o);
}
}  // namespace

// ---- S2-T14 -----------------------------------------------------------------

TEST(LocalFrame, GraspDataIsInvariantUnderAWorldTransform) {
  const GraspConfig a = loadGraspConfig(cfgPath());
  GraspConfig b = a;

  dual_arm::Iso3 T = dual_arm::Iso3::Identity();
  T.translate(dual_arm::Vec3(0.31, -0.22, 0.14));
  T.rotate(Eigen::AngleAxisd(1.1, dual_arm::Vec3(0.2, 0.3, 0.9).normalized()));
  b.grasp.world_T_object_star = T * a.grasp.world_T_object_star;

  // Relocating the object may not perturb one bit of the object-local grasp data.
  EXPECT_DOUBLE_EQ((a.grasp.object_T_left_tcp.matrix() - b.grasp.object_T_left_tcp.matrix()).norm(), 0.0);
  EXPECT_DOUBLE_EQ((a.grasp.object_T_right_tcp.matrix() - b.grasp.object_T_right_tcp.matrix()).norm(), 0.0);
  EXPECT_DOUBLE_EQ((a.left_region.approach_dir_local - b.left_region.approach_dir_local).norm(), 0.0);
  EXPECT_DOUBLE_EQ((a.right_region.approach_dir_local - b.right_region.approach_dir_local).norm(), 0.0);
  ASSERT_EQ(a.left_region.polygon_local.size(), b.left_region.polygon_local.size());
  for (std::size_t i = 0; i < a.left_region.polygon_local.size(); ++i) {
    EXPECT_DOUBLE_EQ((a.left_region.polygon_local[i] - b.left_region.polygon_local[i]).norm(), 0.0);
    EXPECT_DOUBLE_EQ((a.right_region.polygon_local[i] - b.right_region.polygon_local[i]).norm(), 0.0);
  }
  // ... while the still-valid config keeps validating: the world pose is the only
  // thing that moved.
  EXPECT_EQ(validateGraspConfig(b), "");
}

TEST(LocalFrame, WorldTargetsFollowTheObjectWhileLocalDataDoesNot) {
  const GraspConfig a = loadGraspConfig(cfgPath());
  GraspConfig b = a;
  dual_arm::Iso3 T = dual_arm::Iso3::Identity();
  T.translate(dual_arm::Vec3(0.0, 0.0, 0.05));
  b.grasp.world_T_object_star = T * a.grasp.world_T_object_star;

  const DualGraspTargets ta = graspTargets(a);
  const DualGraspTargets tb = graspTargets(b);
  EXPECT_NEAR((tb.world_T_left_tcp.translation() - ta.world_T_left_tcp.translation()).norm(),
              0.05, 1e-12);
  EXPECT_LT((tb.world_T_left_tcp.translation() - ta.world_T_left_tcp.translation()).norm()
                - 0.05, 0.0);
  // The relative pose between the two tools is a property of the grasp, not of
  // where the object sits.
  const dual_arm::Iso3 ra = ta.world_T_left_tcp.inverse() * ta.world_T_right_tcp;
  const dual_arm::Iso3 rb = tb.world_T_left_tcp.inverse() * tb.world_T_right_tcp;
  EXPECT_LT((ra.matrix() - rb.matrix()).norm(), 1e-12);
}

// ---- S2-T15 -----------------------------------------------------------------

TEST(Gripper, BothSidesMeasureTheSameGeometry) {
  const auto scene = loadScene();
  const GraspConfig cfg = loadGraspConfig(cfgPath());
  const GripperMeasurement l = measureGripper(*scene, "left");
  const GripperMeasurement r = measureGripper(*scene, "right");

  for (const GripperMeasurement* m : {&l, &r}) {
    EXPECT_EQ(m->tcp_frame, m->prefix + "_grasp_frame");
    EXPECT_EQ(m->wrist_link, m->prefix + "_wrist3_link");
    EXPECT_GE(m->n_finger_shapes, 2u);
    EXPECT_GE(m->n_wrist_shapes, 1u);
    // The stroke is real: closed pads nearly touch, open pads span the opening.
    EXPECT_LT(std::abs(m->closed_pad_gap_m), 1e-3);
    EXPECT_NEAR(m->open_pad_gap_m, cfg.gripper_max_opening_m, 1e-3);
    // grip_thickness must be strictly inside the stroke, else the pads bottom out.
    EXPECT_LT(cfg.grip_thickness_m, cfg.gripper_max_opening_m);
    // Measured pad window in the TCP frame is z in [-0.031, +0.0105]: the pads
    // start 31 mm behind the TCP (i.e. in front of the wrist, which is at
    // z ~ -0.164) and reach 10.5 mm past the TCP.  Positive z_tcp points INTO the
    // sample, so "far" beyond the TCP is the +z side -- not the -z side.
    EXPECT_LT(m->pad_z_near_m, 0.0);
    EXPECT_GT(m->pad_z_far_m, 0.0);
    EXPECT_NEAR(m->pad_z_far_m, 0.0105, 1e-3);
    EXPECT_NEAR(m->pad_z_near_m, -0.031, 1e-3);
    // The pad bodies must be narrower than the closed stroke, otherwise a gripped
    // sample cannot sit between them.
    EXPECT_LT(m->open_pad_gap_m, cfg.gripper_max_opening_m + 1e-9);
  }
  // Mirror symmetry: the two grippers are the same tool mounted twice.
  EXPECT_NEAR(l.open_pad_gap_m, r.open_pad_gap_m, 1e-9);
  EXPECT_NEAR(l.closed_pad_gap_m, r.closed_pad_gap_m, 1e-9);
  EXPECT_LT((l.fingers_min_tcp - r.fingers_min_tcp).norm(), 1e-6);
  EXPECT_LT((l.fingers_max_tcp - r.fingers_max_tcp).norm(), 1e-6);
}

TEST(Gripper, DualEndGraspIsInterferenceFreeIncludingTheGripperBody) {
  const auto scene = loadScene();
  const GraspConfig cfg = loadGraspConfig(cfgPath());

  const DualGraspTargets t = graspTargets(cfg);
  dual_arm::DualArmKinematics kin;
  const BranchSet left = enumerateBranches(cfg, kin, t.world_T_left_tcp, "left");
  const BranchSet right = enumerateBranches(cfg, kin, t.world_T_right_tcp, "right");
  ASSERT_TRUE(left.ok) << left.failure_stage << "/" << left.failure_code;
  ASSERT_TRUE(right.ok) << right.failure_stage << "/" << right.failure_code;

  PairScreener scr(scene, cfg);
  scr.applyWorld(t.world_T_object);
  const ScreenReport rep = scr.screen(left, right);

  ASSERT_GE(rep.n_feasible, 1) << "the configured dual-end grasp is not realisable\n" << rep.to_string();
  for (const PairCandidate& c : rep.candidates) {
    if (!c.feasible) continue;
    EXPECT_FALSE(c.collides);
    EXPECT_GE(c.min_clearance_m, 2e-3);
    EXPECT_LT(c.anchor_pos_gap_m, 1e-6);
    EXPECT_LT(c.anchor_rot_gap_rad, 1e-6);
  }
}
