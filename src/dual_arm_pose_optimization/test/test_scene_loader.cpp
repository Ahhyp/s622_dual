// test/test_scene_loader.cpp
// C4.2 / S1: the scene loader must either build a scene with REAL collision
// geometry, or fail loudly.  These tests cover both directions, including the
// negative cases that guard against the silent-failure mode.

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <string>

#include "dual_arm_pose_optimization/scene_loader.hpp"

namespace c42 = dual_arm_pose_optimization;
namespace fs = std::filesystem;

namespace {

c42::SceneLoadOptions fixtureOptions() {
  c42::SceneLoadOptions o;
  o.urdf_path = C42_URDF;
  o.srdf_path = C42_SRDF;
  return o;
}

std::shared_ptr<c42::DualArmScene> fixtureScene() {
  static std::shared_ptr<c42::DualArmScene> s = c42::DualArmScene::load(fixtureOptions());
  return s;
}

}  // namespace

TEST(C42SceneLoader, LoadsFixtureAndReportsProvenance) {
  const auto scene = fixtureScene();
  const c42::GeometryReport& g = scene->geometry();

  EXPECT_EQ(g.urdf_path, std::string(C42_URDF));
  EXPECT_EQ(g.srdf_path, std::string(C42_SRDF));
  EXPECT_GT(g.urdf_bytes, 1000u);
  EXPECT_GT(g.srdf_bytes, 1000u);
  EXPECT_FALSE(g.model_name.empty());
  EXPECT_FALSE(g.model_frame.empty());
  EXPECT_GT(g.urdf_links, 0u);
  EXPECT_GT(g.urdf_joints, 0u);
  EXPECT_GT(g.urdf_collision_elements, 0u);
  EXPECT_EQ(g.collision_detector_name, "FCL");
}

TEST(C42SceneLoader, EveryMeshResolvesAndIsNonEmpty) {
  const c42::GeometryReport& g = fixtureScene()->geometry();
  EXPECT_GT(g.urdf_mesh_refs, 0u);
  EXPECT_TRUE(g.failed_meshes.empty());
  EXPECT_EQ(g.meshes_resolved, g.urdf_mesh_refs);

  for (const std::string& p : g.resolved_mesh_paths) {
    ASSERT_TRUE(fs::exists(p)) << p;
    EXPECT_GT(fs::file_size(p), 0u) << p;
  }
}

TEST(C42SceneLoader, CollisionGeometryActuallyLoadedPerArm) {
  const c42::GeometryReport& g = fixtureScene()->geometry();

  // 9 meshes on the left arm (base/shoulder/upperarm/forearm/wrist1-3/finger1-2)
  // and 10 on the right: the same 9 plus the wrist camera BOX.
  //
  // Arm membership is decided by KINEMATIC ATTACHMENT, not by name prefix.
  // `wrist_camera_link` is rigidly attached to `right_grasp_frame` and therefore
  // belongs to the RIGHT arm even though its link name carries no `right_`
  // prefix.  A prefix rule used to file it under "other", which silently dropped
  // the (left arm x camera) pair from the cross-arm clearance scope -- a real
  // collision risk that was never measured.
  EXPECT_EQ(g.shapesOfArm("left"), 9u);
  EXPECT_EQ(g.shapesOfArm("right"), 10u);
  EXPECT_EQ(g.shapesOfArm("other"), 0u) << "every collision link must belong to an arm";
  EXPECT_EQ(g.total_collision_shapes, 19u);

  // The loaded shape count must cover every <collision> block the URDF declares.
  EXPECT_GE(g.total_collision_shapes, g.urdf_collision_elements);

  // Mesh geometry is really there (not just placeholders).
  EXPECT_GT(g.total_mesh_vertices, 1000u);
  EXPECT_GT(g.total_mesh_triangles, 1000u);

  EXPECT_EQ(g.shapes_per_type.count("mesh"), 1u);
  EXPECT_EQ(g.shapes_per_type.at("mesh"), 18u);
  EXPECT_EQ(g.shapes_per_type.at("box"), 1u);  // wrist_camera_link
}

TEST(C42SceneLoader, PerLinkDistributionIsReported) {
  const c42::GeometryReport& g = fixtureScene()->geometry();
  ASSERT_EQ(g.links.size(), 19u);

  // Every arm link carries exactly one mesh; the gripper fingers are included.
  const c42::LinkGeometry* upper = g.link("left_upperarm_link");
  ASSERT_NE(upper, nullptr);
  EXPECT_EQ(upper->arm, "left");
  EXPECT_EQ(upper->n_shapes, 1u);
  ASSERT_EQ(upper->shape_types.size(), 1u);
  EXPECT_EQ(upper->shape_types[0], "mesh");
  EXPECT_GT(upper->mesh_vertices, 0u);

  EXPECT_NE(g.link("right_finger2"), nullptr);

  // The camera is attached to the right grasp frame -> it is part of the RIGHT
  // arm, and must be reported as such (see the note in the test above).
  const c42::LinkGeometry* cam = g.link("wrist_camera_link");
  ASSERT_NE(cam, nullptr);
  EXPECT_EQ(cam->arm, "right");
  EXPECT_EQ(cam->shape_types.size(), 1u);
  EXPECT_EQ(cam->shape_types[0], "box");

  // Links with no collision geometry are listed explicitly.
  const auto& no_col = g.urdf_links_without_collision;
  EXPECT_NE(std::find(no_col.begin(), no_col.end(), "world"), no_col.end());
}

TEST(C42SceneLoader, ArmJointNamesAreInC2Order) {
  const auto scene = fixtureScene();
  const auto& j = scene->armJointNames();
  ASSERT_EQ(j.size(), 12u);
  EXPECT_EQ(j[0], "left_j1");
  EXPECT_EQ(j[5], "left_j6");
  EXPECT_EQ(j[6], "right_j1");
  EXPECT_EQ(j[11], "right_j6");

  std::vector<double> q12(12, 0.0);
  auto st = scene->makeStateQ12(q12);
  const double* v = st.getVariablePositions();
  EXPECT_NE(v, nullptr);
}

// ---------------------------------------------------------------------------
// Negative tests: a broken resource path must fail LOUDLY.
// ---------------------------------------------------------------------------

TEST(C42SceneLoader, MissingUrdfThrowsNamingThePath) {
  c42::SceneLoadOptions o = fixtureOptions();
  o.urdf_path = "/tmp/c42_definitely_missing.urdf";
  try {
    c42::DualArmScene::load(o);
    FAIL() << "expected SceneLoadError";
  } catch (const c42::SceneLoadError& e) {
    const std::string msg = e.what();
    EXPECT_NE(msg.find("/tmp/c42_definitely_missing.urdf"), std::string::npos) << msg;
  }
}

TEST(C42SceneLoader, MissingSrdfThrowsNamingThePath) {
  c42::SceneLoadOptions o = fixtureOptions();
  o.srdf_path = "/tmp/c42_definitely_missing.srdf";
  try {
    c42::DualArmScene::load(o);
    FAIL() << "expected SceneLoadError";
  } catch (const c42::SceneLoadError& e) {
    const std::string msg = e.what();
    EXPECT_NE(msg.find("/tmp/c42_definitely_missing.srdf"), std::string::npos) << msg;
  }
}

TEST(C42SceneLoader, BrokenMeshUriThrowsAndNamesTheUri) {
  std::string xml;
  ASSERT_TRUE(c42::readFileBytes(C42_URDF, &xml));

  const std::string needle = "package://s622_moveit_descriptions/meshes/";
  const std::string bogus_pkg = "c42_no_such_package_zzz";
  const std::string repl = "package://" + bogus_pkg + "/meshes/";
  // Replace every occurrence: the first one in the URDF is a <visual> mesh and
  // visuals are correctly ignored by the collision pre-flight.
  std::size_t pos = xml.find(needle);
  ASSERT_NE(pos, std::string::npos) << "fixture does not use the expected mesh URI";
  while (pos != std::string::npos) {
    xml.replace(pos, needle.size(), repl);
    pos = xml.find(needle, pos + repl.size());
  }

  fs::create_directories(C42_FIXTURE_DIR);
  const std::string bad = std::string(C42_FIXTURE_DIR) + "/broken_mesh.urdf";
  {
    std::ofstream out(bad);
    ASSERT_TRUE(out.good());
    out << xml;
  }

  c42::SceneLoadOptions o = fixtureOptions();
  o.urdf_path = bad;
  try {
    c42::DualArmScene::load(o);
    FAIL() << "expected SceneLoadError for a broken mesh URI";
  } catch (const c42::SceneLoadError& e) {
    const std::string msg = e.what();
    EXPECT_NE(msg.find(bogus_pkg), std::string::npos)
        << "the error must name the offending URI/package:\n" << msg;
    EXPECT_NE(msg.find("mesh"), std::string::npos) << msg;
  }
}

TEST(C42SceneLoader, RequireMeshesFalseSkipsThePreflightButStillLoads) {
  c42::SceneLoadOptions o = fixtureOptions();
  o.require_meshes = false;
  const auto scene = c42::DualArmScene::load(o);
  // With a healthy ament index this still loads real geometry.
  EXPECT_GT(scene->geometry().total_collision_shapes, 0u);
}

TEST(C42SceneLoader, ResolvePackageUriHelperBehaves) {
  std::string path, reason;
  EXPECT_TRUE(c42::resolvePackageUri(
      "package://s622_moveit_descriptions/meshes/base_link.STL", &path, &reason))
      << reason;
  EXPECT_TRUE(fs::exists(path));

  EXPECT_FALSE(c42::resolvePackageUri(
      "package://c42_no_such_package_zzz/meshes/base_link.STL", &path, &reason));
  EXPECT_NE(reason.find("c42_no_such_package_zzz"), std::string::npos) << reason;

  EXPECT_FALSE(c42::resolvePackageUri("package://s622_moveit_descriptions/nope.STL",
                                      &path, &reason));
  EXPECT_FALSE(c42::resolvePackageUri("package://malformed", &path, &reason));
}
