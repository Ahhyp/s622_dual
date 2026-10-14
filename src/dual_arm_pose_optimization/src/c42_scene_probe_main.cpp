// src/c42_scene_probe_main.cpp
// C4.2 / S1 acceptance probe.  Prints and JSON-dumps the evidence the S1
// acceptance list asks for:
//   1. collision geometry actually loaded + per-arm link distribution
//   2. URDF/SRDF and mesh resource load evidence
//   3. ACM behaviour tests (SRDF / cross-arm / world)
//   4. whole-robot distance vs cross-arm distance scope separation
//   5. floor <-> base_link exemption, and that it is what makes the difference
//   6. negative test: a broken mesh URI must fail loudly
//
// Exit code 0 == every self-check passed.

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "dual_arm_pose_optimization/collision_backend.hpp"
#include "dual_arm_pose_optimization/scene_loader.hpp"

namespace c42 = dual_arm_pose_optimization;
using dual_arm::Q12;

namespace {

struct Check {
  std::string name;
  bool pass = false;
  std::string detail;
};

std::string jsonEscape(const std::string& s) {
  std::string o;
  for (char ch : s) {
    switch (ch) {
      case '"': o += "\\\""; break;
      case '\\': o += "\\\\"; break;
      case '\n': o += "\\n"; break;
      case '\r': o += "\\r"; break;
      case '\t': o += "\\t"; break;
      default: o += ch;
    }
  }
  return o;
}

// Baseline poses (see C4.2_执行规划.md §2.1).
//  A : SRDF dual_handover_ready  -- HISTORICAL REFERENCE ONLY
//  B : M4 left_hold_joints + right_receive_base -- the *pre-rebuild* config,
//      used here only as a smoke pose, NOT as the official baseline.
const std::vector<double> kBaselineA{
    -0.5963, -1.6969, -2.2265, 0.7825, 2.1677, 1.5715,
    -2.5432, -1.4419, 2.2229, -3.9223, -2.1697, 1.5713};
const std::vector<double> kBaselineBPrerebuild{
    0.3353, -1.5739, -0.9095, -2.2290, 1.5708, -1.2355,
    0.2856, -1.2598, -1.7115, -3.3161, -1.2915, -1.5708};
const std::vector<double> kHome{0.0, -1.5708, 0.0, 0.0, 0.0, 0.0,
                                0.0, -1.5708, 0.0, 0.0, 0.0, 0.0};
const std::vector<double> kLeftDown{0.0, 0.6, 0.0, 0.0, 0.0, 0.0,
                                    0.0, -1.5708, 0.0, 0.0, 0.0, 0.0};

void add(std::vector<Check>& v, const std::string& n, bool ok, const std::string& d) {
  v.push_back({n, ok, d});
}

}  // namespace

int main(int argc, char** argv) {
  std::string urdf = "src/dual_arm_kinematics/test/fixtures/s622_dual_arm.urdf";
  std::string srdf = "src/s622_moveit_config/config/s622_dual_arm.srdf";
  std::string json_out;

  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    auto next = [&]() -> std::string {
      if (i + 1 >= argc) {
        std::cerr << "missing value for " << a << "\n";
        std::exit(2);
      }
      return argv[++i];
    };
    if (a == "--urdf") urdf = next();
    else if (a == "--srdf") srdf = next();
    else if (a == "--json") json_out = next();
    else if (a == "-h" || a == "--help") {
      std::cout << "usage: c42_scene_probe [--urdf P] [--srdf P] [--json OUT]\n";
      return 0;
    } else {
      std::cerr << "unknown argument: " << a << "\n";
      return 2;
    }
  }

  std::vector<Check> checks;

  // ----------------------------------------------------------------- load
  c42::SceneLoadOptions opts;
  opts.urdf_path = urdf;
  opts.srdf_path = srdf;

  std::shared_ptr<c42::DualArmScene> scene;
  try {
    scene = c42::DualArmScene::load(opts);
  } catch (const std::exception& e) {
    std::cerr << e.what() << "\n";
    return 1;
  }
  const c42::GeometryReport& g = scene->geometry();
  std::cout << g.to_string() << "\n";

  add(checks, "geometry.left_shapes > 0", g.shapesOfArm("left") > 0,
      "left=" + std::to_string(g.shapesOfArm("left")));
  add(checks, "geometry.right_shapes > 0", g.shapesOfArm("right") > 0,
      "right=" + std::to_string(g.shapesOfArm("right")));
  // The two arms are NOT symmetric: the right one carries the wrist camera
  // (wrist_camera_link is rigidly attached to right_grasp_frame).  Arm
  // membership is decided by KINEMATIC ATTACHMENT, so the camera counts as part
  // of the right arm even though its link name has no "right_" prefix.  Assert
  // the asymmetry is exactly the known one instead of pretending they match.
  {
    std::size_t extra = 0;
    bool extra_is_camera = false;
    for (const auto& lg : g.links) {
      if (lg.link_name == "wrist_camera_link" && lg.n_shapes > 0) {
        extra_is_camera = (lg.arm == "right");
        ++extra;
      }
    }
    const bool ok = extra_is_camera && extra == 1 &&
                    g.shapesOfArm("right") == g.shapesOfArm("left") + 1 &&
                    g.shapesOfArm("left") > 0;
    add(checks, "geometry.right_has_the_wrist_camera_only", ok,
        "left=" + std::to_string(g.shapesOfArm("left")) +
            " right=" + std::to_string(g.shapesOfArm("right")) +
            " (camera on right: " + (extra_is_camera ? "yes" : "NO") + ")");
  }
  add(checks, "geometry.no_orphan_collision_links", g.shapesOfArm("other") == 0,
      "other=" + std::to_string(g.shapesOfArm("other")) +
          " (every collision link must belong to an arm by attachment)");
  add(checks, "geometry.mesh_vertices > 0", g.total_mesh_vertices > 0,
      "verts=" + std::to_string(g.total_mesh_vertices) +
          " tris=" + std::to_string(g.total_mesh_triangles));
  add(checks, "geometry.all_urdf_collisions_loaded",
      g.total_collision_shapes >= g.urdf_collision_elements,
      "loaded=" + std::to_string(g.total_collision_shapes) +
          " urdf_declared=" + std::to_string(g.urdf_collision_elements));
  add(checks, "resources.all_meshes_resolved", g.failed_meshes.empty(),
      std::to_string(g.meshes_resolved) + "/" + std::to_string(g.urdf_mesh_refs) +
          " resolved");

  // ------------------------------------------------------------ backend
  c42::EnvironmentOptions env;
  env.add_floor = true;
  c42::MoveItCollisionBackend backend(scene, env);

  // ---- ACM behaviour ----
  {
    collision_detection::AllowedCollision::Type t =
        collision_detection::AllowedCollision::NEVER;
    const bool has = backend.srdfAcm().getEntry("left_base_link", "left_shoulder_link", t);
    add(checks, "acm.srdf_disables_adjacent_pair",
        has && t == collision_detection::AllowedCollision::ALWAYS,
        std::string("entry=") + (has ? "yes" : "no") + " allowed=" +
            ((t == collision_detection::AllowedCollision::ALWAYS) ? "yes" : "no"));

    collision_detection::AllowedCollision::Type t2 = collision_detection::AllowedCollision::NEVER;
    const bool has2 =
        backend.crossArmAcm().getEntry("left_upperarm_link", "right_upperarm_link", t2);
    add(checks, "acm.cross_arm_pair_still_checked",
        !has2 || t2 == collision_detection::AllowedCollision::NEVER,
        std::string("entry=") + (has2 ? "yes" : "no (default=checked)"));

    const std::size_t expect =
        scene->leftLinks().size() * scene->rightLinks().size();
    add(checks, "acm.cross_arm_pair_count",
        backend.crossArmPairCount() == expect,
        "checked=" + std::to_string(backend.crossArmPairCount()) +
            " expected=" + std::to_string(expect));
  }

  // ---- distance scopes on the two reference poses ----
  auto reading = [&](const char* tag, const std::vector<double>& q12) {
    auto st = scene->makeStateQ12(q12);
    const Q12 q = Eigen::Map<const Q12>(q12.data());
    const c42::DistanceReading r = backend.distances(st, &q);
    std::cout << "[distance] " << tag << "\n    " << r.to_string() << "\n";
    return r;
  };
  const c42::DistanceReading bA = reading("Baseline A (historical ref)", kBaselineA);
  const c42::DistanceReading bB = reading("Baseline B pre-rebuild (smoke only)",
                                        kBaselineBPrerebuild);

  add(checks, "distance.whole_le_arm_arm (A)", bA.whole_robot <= bA.arm_arm + 1e-9,
      "whole=" + std::to_string(bA.whole_robot) + " arm_arm=" + std::to_string(bA.arm_arm));
  add(checks, "distance.whole_le_arm_arm (B)", bB.whole_robot <= bB.arm_arm + 1e-9,
      "whole=" + std::to_string(bB.whole_robot) + " arm_arm=" + std::to_string(bB.arm_arm));
  add(checks, "distance.arm_arm_positive", bA.arm_arm > 0.0 && bB.arm_arm > 0.0,
      "A=" + std::to_string(bA.arm_arm) + " B=" + std::to_string(bB.arm_arm));
  add(checks, "distance.scopes_differ (proves trap 3)",
      std::abs(bA.whole_robot - bA.arm_arm) > 1e-6,
      "whole=" + std::to_string(bA.whole_robot) + " arm_arm=" + std::to_string(bA.arm_arm));

  // ---- floor behaviour ----
  {
    auto home = scene->makeStateQ12(kHome);
    auto down = scene->makeStateQ12(kLeftDown);

    const bool home_coll = backend.isStateColliding(home);
    const bool down_coll = backend.isStateColliding(down);
    add(checks, "floor.home_not_colliding", !home_coll, "");
    add(checks, "floor.left_arm_down_collides_with_floor", down_coll, "");

    // trap 4 evidence: the exemption is what makes the difference.
    const double hw_exempt = backend.robotWorldDistanceWith(home, backend.robotWorldAcm());
    const double hw_raw =
        backend.robotWorldDistanceWith(home, backend.robotWorldAcmNoFloorExemption());
    add(checks, "floor.base_link_exemption_matters", hw_exempt > 0.0 && hw_raw <= 0.0,
        "with_exemption=" + std::to_string(hw_exempt) +
            " without=" + std::to_string(hw_raw));

    // The exemption is ONLY for base_link: a link that really dips below the
    // floor must still register as a collision (distance <= 0).
    const double dw_exempt = backend.robotWorldDistanceWith(down, backend.robotWorldAcm());
    add(checks, "floor.arm_below_floor_still_detected", dw_exempt <= 0.0,
        "robotWorldDistance(arm down, exempted)=" + std::to_string(dw_exempt));
  }

  // ---- segment check ----
  {
    auto home = scene->makeStateQ12(kHome);
    auto down = scene->makeStateQ12(kLeftDown);
    const bool ok = backend.isSegmentValid(home, down, 32);
    add(checks, "segment.home_to_arm_down_invalid", !ok, "expected collision on the way");
  }

  // ---- negative test: broken mesh URI must throw ----
  {
    const std::string bad = "/tmp/c42_bad_mesh.urdf";
    std::string xml;
    if (c42::readFileBytes(urdf, &xml)) {
      // Replace EVERY occurrence: the first one in the file is a <visual> mesh,
      // and visuals are (correctly) not part of the collision pre-flight.
      const std::string needle = "package://s622_moveit_descriptions/meshes/";
      const std::string repl = "package://definitely_no_such_pkg_xyz/meshes/";
      std::size_t pos = xml.find(needle);
      std::size_t n_replaced = 0;
      while (pos != std::string::npos) {
        xml.replace(pos, needle.size(), repl);
        ++n_replaced;
        pos = xml.find(needle, pos + repl.size());
      }
      add(checks, "negative.fixture_had_mesh_uris", n_replaced > 0,
          "replaced " + std::to_string(n_replaced) + " occurrences");
      std::ofstream out(bad);
      out << xml;
      out.close();
      bool threw = false;
      std::string msg;
      try {
        c42::SceneLoadOptions bad_opts;
        bad_opts.urdf_path = bad;
        bad_opts.srdf_path = srdf;
        c42::DualArmScene::load(bad_opts);
      } catch (const c42::SceneLoadError& e) {
        threw = true;
        msg = e.what();
      }
      const bool names_it = msg.find("definitely_no_such_pkg_xyz") != std::string::npos;
      add(checks, "negative.broken_mesh_uri_throws", threw, msg.substr(0, 160));
      add(checks, "negative.error_names_the_uri", names_it, "");
    } else {
      add(checks, "negative.broken_mesh_uri_throws", false, "could not read source URDF");
    }
  }
  {
    bool threw = false;
    std::string msg;
    try {
      c42::SceneLoadOptions bad_opts;
      bad_opts.urdf_path = "/tmp/c42_does_not_exist.urdf";
      bad_opts.srdf_path = srdf;
      c42::DualArmScene::load(bad_opts);
    } catch (const c42::SceneLoadError& e) {
      threw = true;
      msg = e.what();
    }
    add(checks, "negative.missing_urdf_throws", threw, msg.substr(0, 120));
  }

  // ----------------------------------------------------------------- report
  std::size_t npass = 0;
  std::cout << "\n=== S1 self-checks ===\n";
  for (const auto& ck : checks) {
    std::cout << (ck.pass ? "  PASS  " : "  FAIL  ") << ck.name;
    if (!ck.detail.empty()) std::cout << "   [" << ck.detail << "]";
    std::cout << "\n";
    if (ck.pass) ++npass;
  }
  std::cout << "  ---> " << npass << "/" << checks.size() << " passed\n";

  if (!json_out.empty()) {
    std::ofstream js(json_out);
    js << "{\n";
    js << "  \"stage\": \"C4.2-S1\",\n";
    js << "  \"urdf\": \"" << jsonEscape(g.urdf_path) << "\",\n";
    js << "  \"srdf\": \"" << jsonEscape(g.srdf_path) << "\",\n";
    js << "  \"geometry\": {\n";
    js << "    \"total_collision_shapes\": " << g.total_collision_shapes << ",\n";
    js << "    \"urdf_collision_elements\": " << g.urdf_collision_elements << ",\n";
    js << "    \"shapes_per_arm\": {\"left\": " << g.shapesOfArm("left")
       << ", \"right\": " << g.shapesOfArm("right") << "},\n";
    js << "    \"links_with_collision\": [";
    for (std::size_t i = 0; i < g.links.size(); ++i) {
      js << (i ? ", " : "") << "\"" << jsonEscape(g.links[i].link_name) << "\"";
    }
    js << "],\n";
    js << "    \"mesh_vertices\": " << g.total_mesh_vertices << ",\n";
    js << "    \"mesh_triangles\": " << g.total_mesh_triangles << ",\n";
    js << "    \"meshes_resolved\": " << g.meshes_resolved << ",\n";
    js << "    \"meshes_failed\": " << g.failed_meshes.size() << ",\n";
    js << "    \"collision_detector\": \"" << jsonEscape(g.collision_detector_name)
       << "\",\n";
    js << "    \"srdf_disable_collisions\": " << g.srdf_disable_collisions << ",\n";
    js << "    \"cross_arm_pairs_checked\": " << backend.crossArmPairCount() << "\n";
    js << "  },\n";
    js << "  \"distance_scopes\": {\n";
    auto dump = [&](const char* k, const c42::DistanceReading& r, bool last) {
      js << "    \"" << k << "\": {\"whole_robot\": " << r.whole_robot
         << ", \"arm_arm\": " << r.arm_arm << ", \"capsule_arm_arm\": "
         << r.capsule_arm_arm << ", \"colliding\": " << (r.in_collision ? "true" : "false")
         << ", \"arm_arm_pair\": [\"" << jsonEscape(r.arm_arm_link_a) << "\", \""
         << jsonEscape(r.arm_arm_link_b) << "\"], \"whole_pair\": [\""
         << jsonEscape(r.whole_link_a) << "\", \"" << jsonEscape(r.whole_link_b)
         << "\"]}" << (last ? "" : ",") << "\n";
    };
    dump("baseline_A_historical", bA, false);
    dump("baseline_B_prerebuild_smoke", bB, true);
    js << "  },\n";
    js << "  \"checks\": [\n";
    for (std::size_t i = 0; i < checks.size(); ++i) {
      js << "    {\"name\": \"" << jsonEscape(checks[i].name) << "\", \"pass\": "
         << (checks[i].pass ? "true" : "false") << ", \"detail\": \""
         << jsonEscape(checks[i].detail) << "\"}" << (i + 1 == checks.size() ? "" : ",")
         << "\n";
    }
    js << "  ],\n";
    js << "  \"passed\": " << npass << ",\n  \"total\": " << checks.size() << "\n}\n";
    std::cout << "wrote " << json_out << "\n";
  }

  return npass == checks.size() ? 0 : 1;
}
