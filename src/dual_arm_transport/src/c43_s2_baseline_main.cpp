// src/c43_s2_baseline_main.cpp
// C4.3 S2 driver: run the screening chain and emit the independent C4.3
// `transport_baseline`.  Usage:
//
//   c43_s2_baseline --urdf <path> --srdf <path> [--config <yaml>] [--out <yaml>]
//
// Exit codes: 0 = a feasible baseline was produced; 2 = no feasible baseline
// (the chain ran but every candidate was rejected); 1 = usage/IO/parse failure.
#include <cstring>
#include <iostream>
#include <string>

#include "dual_arm_pose_optimization/scene_loader.hpp"
#include "dual_arm_transport/baseline.hpp"
#include "dual_arm_transport/grasp_config.hpp"

namespace {

void usage(const char* argv0) {
  std::cerr << "usage: " << argv0
            << " --urdf <path> --srdf <path> [--config <yaml>] [--out <yaml>]\n";
}

int fail(int code, const std::string& what) {
  std::cerr << "c43_s2_baseline: " << what << "\n";
  return code;
}

}  // namespace

int main(int argc, char** argv) {
  std::string urdf, srdf, config, out;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    const auto next = [&](std::string* dst) {
      if (i + 1 >= argc) return false;
      *dst = argv[++i];
      return true;
    };
    if (a == "--urdf") {
      if (!next(&urdf)) { usage(argv[0]); return 1; }
    } else if (a == "--srdf") {
      if (!next(&srdf)) { usage(argv[0]); return 1; }
    } else if (a == "--config") {
      if (!next(&config)) { usage(argv[0]); return 1; }
    } else if (a == "--out") {
      if (!next(&out)) { usage(argv[0]); return 1; }
    } else if (a == "-h" || a == "--help") {
      usage(argv[0]);
      return 0;
    } else {
      usage(argv[0]);
      return fail(1, "unknown argument: " + a);
    }
  }
  if (urdf.empty() || srdf.empty()) {
    usage(argv[0]);
    return fail(1, "--urdf and --srdf are required");
  }
  if (config.empty()) config = "install/share/dual_arm_transport/config/object_and_grasps.yaml";
  if (out.empty()) out = "install/share/dual_arm_transport/config/transport_baseline.yaml";

  dual_arm_transport::GraspConfig cfg;
  try {
    cfg = dual_arm_transport::loadGraspConfig(config);
  } catch (const std::exception& e) {
    return fail(1, std::string("grasp config: ") + e.what());
  }
  const std::string invalid = dual_arm_transport::validateGraspConfig(cfg);
  if (!invalid.empty()) return fail(1, "grasp config rejected: " + invalid);

  dual_arm_pose_optimization::SceneLoadOptions opt;
  opt.urdf_path = urdf;
  opt.srdf_path = srdf;
  opt.require_meshes = true;
  std::shared_ptr<dual_arm_pose_optimization::DualArmScene> scene;
  try {
    scene = dual_arm_pose_optimization::DualArmScene::load(opt);
  } catch (const std::exception& e) {
    return fail(1, std::string("scene: ") + e.what());
  }

  std::cout << "== C4.3 S2 screening ==\n";
  std::cout << "  urdf:   " << urdf << "\n";
  std::cout << "  srdf:   " << srdf << "\n";
  std::cout << "  config: " << config << "\n";

  dual_arm_transport::BaselineSolver solver(scene, cfg);
  solver.setProvenance(
      "C4.3 S2 c43_s2_baseline: independent C4.3 transport baseline produced by "
      "branch enumeration + cartesian-product screening + reachability check");

  dual_arm_transport::TransportBaseline b;
  try {
    b = solver.solve();
  } catch (const std::exception& e) {
    for (const std::string& line : solver.lastPlacementLog()) {
      std::cout << "  " << line << "\n";
    }
    std::cout << solver.lastScreen().to_string() << std::flush;
    return fail(2, std::string("no feasible baseline: ") + e.what());
  }

  std::cout << "\n" << solver.lastScreen().to_string() << "\n";
  std::cout << "standby: " << b.q_standby_id << "\n";
  std::cout << "  q12 = [" << b.q_standby.transpose() << "]\n";
  std::cout << "grasp pair (left,right) = (" << b.left_index << "," << b.right_index << ")\n";
  std::cout << "  clearance   = " << b.clearance_m << " m\n";
  std::cout << "  sigma_min   = " << b.sigma_min << "\n";
  std::cout << "  anchor gap  = " << b.anchor_pos_gap_m << " m / " << b.anchor_rot_gap_rad
            << " rad\n";
  std::cout << "  legacy midpoint error = " << b.legacy_midpoint_err_pos_m << " m / "
            << b.legacy_midpoint_err_rot_rad << " rad\n";
  std::cout << "object z = " << b.world_T_object.translation().z()
            << "  support top_z = " << b.support.top_z_m
            << " (" << b.support.top_z_status << ")\n";
  std::cout << "standby segment: " << b.standby_segment.to_string() << "\n";
  std::cout << "approach segment: " << b.approach_segment.to_string() << "\n";

  try {
    dual_arm_transport::writeBaseline(out, b);
  } catch (const std::exception& e) {
    return fail(1, std::string("write baseline: ") + e.what());
  }
  std::cout << "wrote " << out << "\n";
  return 0;
}
