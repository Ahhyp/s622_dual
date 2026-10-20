// C4.3 S3 -- W5 tests: the two-layer transport checker (S3-T9).
//
// S3-T9 is a NEGATIVE test on purpose: a checker that never fires is worse than
// no checker, so every assertion here injects a collision that is known to exist
// and demands that it be caught, with an attribution code that names the layer.
#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <string>
#include <vector>

#include "dual_arm_pose_optimization/scene_loader.hpp"
#include "dual_arm_transport/grasp_config.hpp"
#include "dual_arm_transport/ik_enum.hpp"
#include "dual_arm_transport_mock/transport_checker.hpp"

namespace {

using dual_arm::Iso3;
using dual_arm::Q12;
using dual_arm_pose_optimization::DualArmScene;
using dual_arm_pose_optimization::SceneLoadOptions;
using dual_arm_transport::GraspConfig;
using dual_arm_transport::loadGraspConfig;
using dual_arm_transport_mock::CheckOptions;
using dual_arm_transport_mock::StateCheck;
using dual_arm_transport_mock::TransportChecker;

constexpr std::array<double, 12> kGraspQ = {
    2.795036909814,  -1.706950857914, 1.279963524342,  -1.143808993223,
    -1.570796326795, -0.346555743776, 0.346555743776,  -1.434641795676,
    -1.279963524342, -1.997783660367, 1.570796326795,  -2.795036909814};

Q12 graspQ() {
    Q12 q;
    for (int i = 0; i < 12; ++i) q[i] = kGraspQ[static_cast<std::size_t>(i)];
    return q;
}

Iso3 objectStar() {
    Iso3 T = Iso3::Identity();
    T.translation() = Eigen::Vector3d(0.0, 0.0, 0.275);
    return T;
}

class CheckerC43S3 : public ::testing::Test {
protected:
    static void SetUpTestSuite() {
        SceneLoadOptions so;
        so.urdf_path = C43_URDF;
        so.srdf_path = C43_SRDF;
        so.require_meshes = true;
        scene = DualArmScene::load(so);
        cfg = loadGraspConfig(std::string(C43_S2_CONFIG_DIR) + "/object_and_grasps.yaml");
    }

    void SetUp() override {
        // A staged/throwing SetUpTestSuite leaves the statics at their defaults and
        // gtest records only an ad-hoc failure -- so a WRONG config path would make
        // every test silently run against the 1 m default box.  Fail loudly here.
        ASSERT_TRUE(scene != nullptr);
        ASSERT_GT(cfg.grip_thickness_m, 0.0) << "grasp config was not loaded (check C43_S2_CONFIG_DIR)";
        ASSERT_NEAR(cfg.grasp.size_xyz.x(), 0.12, 1e-12);
        CheckOptions opt;
        opt.min_clearance_m = 0.002;
        opt.sweep_samples = 16;
        checker = std::make_unique<TransportChecker>(scene, cfg, opt);
    }

    static std::shared_ptr<DualArmScene> scene;
    static GraspConfig cfg;
    std::unique_ptr<TransportChecker> checker;
};

std::shared_ptr<DualArmScene> CheckerC43S3::scene = nullptr;
GraspConfig CheckerC43S3::cfg;

Iso3 linkPose(const DualArmScene& scene, const Q12& q, const std::string& link) {
    moveit::core::RobotState st =
        scene.makeStateQ12(std::vector<double>(q.data(), q.data() + 12));
    st.update();
    const Eigen::Isometry3d T = st.getGlobalLinkTransform(link);
    Iso3 out = Iso3::Identity();
    out.linear() = T.rotation();
    out.translation() = T.translation();
    return out;
}

std::string describe(const StateCheck& sc) {
    char buf[512];
    std::snprintf(buf, sizeof(buf),
                  "ok=%d coll=%d code=%s whole=%.6f world=%.6f min=%.6f detail=%s (%s)", 
                  static_cast<int>(sc.ok), static_cast<int>(sc.in_collision), sc.code.c_str(),
                  sc.whole_robot, sc.robot_world, sc.min_clearance, sc.detail.c_str(),
                  sc.object_source.c_str());
    return std::string(buf);
}

// ---------------------------------------------------------------------------
// the nominal grasp must PASS -- otherwise every other assertion is vacuous
// ---------------------------------------------------------------------------
TEST_F(CheckerC43S3, T9a_NominalGraspIsClear) {
    checker->applyWorld(objectStar(), "desired");
    const StateCheck sc = checker->checkState(graspQ());
    std::printf("[T9a] %s\n", describe(sc).c_str());
    EXPECT_TRUE(sc.ok) << describe(sc);
    EXPECT_FALSE(sc.in_collision);
    EXPECT_EQ(sc.code, "ok");
    // S2 recorded 13.117 mm of clearance for exactly this configuration.  If the
    // checker reproduces it, the two implementations agree about the scene.
    EXPECT_NEAR(sc.min_clearance, 0.013117371, 5e-4) << describe(sc);
    EXPECT_EQ(sc.object_source, "desired");

    // a real self/arm-arm reading was produced, not a default-filled struct
    EXPECT_GT(sc.whole_robot, 0.0);
    EXPECT_GT(sc.robot_world, 0.0);
    EXPECT_FALSE(sc.link_a.empty());
}

// ---------------------------------------------------------------------------
// injected robot-world overlap with the wrist must be caught by layer 2
// ---------------------------------------------------------------------------
TEST_F(CheckerC43S3, T9b_SampleOverlappingTheWristIsCaught) {
    const Q12 q = graspQ();
    const Iso3 wrist = linkPose(*scene, q, "left_wrist3_link");
    checker->applyWorld(wrist, "desired");
    const StateCheck sc = checker->checkState(q);
    std::printf("[T9b] sample at left_wrist3 -> %s\n", describe(sc).c_str());
    EXPECT_FALSE(sc.ok) << describe(sc);
    EXPECT_TRUE(sc.in_collision) << describe(sc);
    EXPECT_EQ(sc.code, "collision");
    // layer 1 must stay clean: this overlap is invisible to a self-only query,
    // which is exactly the gap that S2's PairScreener had.
    EXPECT_GT(sc.whole_robot, 0.0);
    EXPECT_LE(sc.robot_world, 0.0);
}

// ---------------------------------------------------------------------------
// an injected swept collision must be caught, at an INTERIOR sample
//
// The injection is constructive, not hand-tuned: the far endpoint q1 is fixed, the
// path midpoint is evaluated with forward kinematics, and the sample is placed
// exactly there.  Both ENDPOINTS are then clear (they are far from the midpoint),
// so a checker that only looked at the ends would pass -- which is precisely the
// defect a swept check exists to catch.  The checker is told only q0, q1; it has to
// rediscover the contact for itself.
// ---------------------------------------------------------------------------
TEST_F(CheckerC43S3, T9c_SweepThroughTheSampleIsCaught) {
    const Q12 q0 = graspQ();
    Q12 q1;
    q1 << 0.0, -0.6, 0.4, -0.5, 0.0, 0.0, 0.0, -0.6, -0.4, -0.9, 0.0, 0.0;

    const int n = checker->options().sweep_samples;
    const int mid = n / 2;
    const double a_mid = static_cast<double>(mid) / static_cast<double>(n - 1);
    const Q12 qm = q0 + a_mid * (q1 - q0);

    checker->applyWorld(linkPose(*scene, qm, "left_wrist3_link"), "desired");

    const StateCheck a = checker->checkState(q0);
    const StateCheck b = checker->checkState(q1);
    const StateCheck m = checker->checkState(qm);
    std::printf("[T9c] endpoint q0 -> %s\n", describe(a).c_str());
    std::printf("[T9c] endpoint q1 -> %s\n", describe(b).c_str());
    std::printf("[T9c] midpoint sample %d -> %s\n", mid, describe(m).c_str());
    ASSERT_TRUE(a.ok) << "the start of the sweep must be clear: " << describe(a);
    ASSERT_TRUE(b.ok) << "the end of the sweep must be clear: " << describe(b);
    ASSERT_FALSE(m.ok) << "the injection is vacuous: the midpoint is not in contact";

    const auto swept = checker->checkSweep(q0, q1);
    std::printf("[T9c] sweep ok=%d code=%s first_bad=%d min_clearance=%.6f detail=%s\n",
                static_cast<int>(swept.ok), swept.code.c_str(), swept.first_bad_sample,
                swept.min_clearance_m, swept.detail.c_str());
    EXPECT_FALSE(swept.ok) << "a sweep through the sample must not pass";
    EXPECT_EQ(swept.code, "swept_collision");
    ASSERT_GT(swept.first_bad_sample, 0) << "the failure must be interior, not at an endpoint";
    EXPECT_LT(swept.first_bad_sample, n - 1);
    EXPECT_LE(std::abs(swept.first_bad_sample - mid), 1)
        << "the first offending sample must be at the constructed midpoint";
}

// ---------------------------------------------------------------------------
// the same sweep with nothing in the way must pass (negative control)
// ---------------------------------------------------------------------------
TEST_F(CheckerC43S3, T9d_TheSameSweepPassesWhenNothingIsInTheWay) {
    const Q12 q0 = graspQ();
    Q12 q1 = q0;
    q1 << 0.0, -0.6, 0.4, -0.5, 0.0, 0.0, 0.0, -0.6, -0.4, -0.9, 0.0, 0.0;

    Iso3 far = Iso3::Identity();
    far.translation() = Eigen::Vector3d(0.0, 0.0, 1.5);  // 1.5 m up, clear of everything
    checker->applyWorld(far, "desired");

    const auto swept = checker->checkSweep(q0, q1);
    std::printf("[T9d] sweep with sample far away: ok=%d code=%s min_clearance=%.6f\n",
                static_cast<int>(swept.ok), swept.code.c_str(), swept.min_clearance_m);
    EXPECT_TRUE(swept.ok) << swept.detail;
    EXPECT_EQ(swept.code, "ok");
    EXPECT_EQ(swept.first_bad_sample, -1);
}

}  // namespace
