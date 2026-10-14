// src/c42_baseline_probe_main.cpp
// C4.2 / S2 acceptance probe: analytic multi-solution IK + the official
// Baseline B rebuild (M4 target -> FairinoIK -> IKSelector -> q_R^M4).
//
// Prints and JSON-dumps:
//   * raw branch statistics and the per-candidate reject histogram
//   * the pre-rebuild error of `right_receive_base` (the 9 mm that forces the
//     rebuild) vs the post-rebuild error (must be at IK tolerance)
//   * q_B and every pose re-derived by FK
//   * collision readings of q_B with the rod in the scene
//   * Baseline A only as a clearly-labelled historical annex
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
#include "dual_arm_pose_optimization/m4_baseline.hpp"
#include "dual_arm_pose_optimization/scene_loader.hpp"

namespace c42 = dual_arm_pose_optimization;

namespace {

struct Check {
  std::string name;
  bool pass = false;
  std::string detail;
};
void add(std::vector<Check>& v, const std::string& n, bool ok, const std::string& d) {
  v.push_back({n, ok, d});
}

std::string jsonEscape(const std::string& s) {
  std::string o;
  for (char ch : s) {
    switch (ch) {
      case '"': o += "\\\""; break;
      case '\\': o += "\\\\"; break;
      case '\n': o += "\\n"; break;
      default: o += ch;
    }
  }
  return o;
}

std::string vecStr(const Eigen::Vector3d& v) {
  char buf[128];
  std::snprintf(buf, sizeof(buf), "[%.6f, %.6f, %.6f]", v.x(), v.y(), v.z());
  return buf;
}

std::string jointsStr(const dual_arm::JointConfig& q) {
  std::ostringstream os;
  os << "[";
  for (int i = 0; i < 6; ++i) os << (i ? ", " : "") << q(i);
  os << "]";
  return os.str();
}

void printBaseline(const c42::BaselinePose& b, const c42::M4Params& p) {
  std::cout << "\n[" << b.id << "]"
            << (b.official ? "  (OFFICIAL BASELINE)" : "  (historical / static reference only)")
            << (b.provisional ? "  [provisional_rebuilt]" : "") << "\n";
  std::cout << "  q_L            : " << jointsStr(b.q_left) << "\n";
  std::cout << "  q_R            : " << jointsStr(b.q_right) << "\n";
  std::cout << "  TCP L (world)  : " << vecStr(b.world_T_left.translation()) << "\n";
  std::cout << "  TCP R (world)  : " << vecStr(b.world_T_right.translation()) << "\n";
  std::cout << "  rod  (world)   : " << vecStr(b.world_T_object.translation()) << "\n";
  std::cout << "  |p_LR|         : " << b.left_T_right.translation().norm() << " m\n";
  std::cout << "  |rotvec(^L T_R)|: "
            << dual_arm::logSO3(b.left_T_right.rotation()).norm() * 180.0 / M_PI << " deg\n";
  if (b.rebuilt_via_ik) {
    std::cout << "  target (world) : " << vecStr(b.target_world.block<3, 1>(0, 3)) << "\n";
    std::cout << "  pre-rebuild err: pos=" << b.pre_rebuild_pos_err * 1000.0
              << " mm rot=" << b.pre_rebuild_rot_err * 180.0 / M_PI << " deg\n";
    std::cout << "  post-rebuild   : pos=" << b.post_rebuild_pos_err * 1000.0
              << " mm rot=" << b.post_rebuild_rot_err * 180.0 / M_PI << " deg\n";
    const auto& ik = b.right_ik;
    std::cout << "  IK branches    : total=" << ik.total_branches
              << " survive(q1/q5/q23/fk/uniq/limits)=" << ik.survive_q1 << "/"
              << ik.survive_q5 << "/" << ik.survive_q23 << "/" << ik.survive_fk_verify << "/"
              << ik.survive_unique << "/" << ik.survive_joint_limits
              << "  solutions=" << ik.solutions.size() << "\n";
    std::cout << "  selector       : candidates=" << ik.n_candidates
              << " hard_pass=" << ik.n_hard_pass << " cost=" << ik.selected_cost << "\n";
    std::cout << "  reject hist    :";
    for (const auto& kv : ik.reject_histogram) std::cout << " " << kv.first << "=" << kv.second;
    std::cout << "\n";
  }
  (void)p;
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
      std::cout << "usage: c42_baseline_probe [--urdf P] [--srdf P] [--json OUT]\n";
      return 0;
    } else {
      std::cerr << "unknown argument: " << a << "\n";
      return 2;
    }
  }

  std::vector<Check> checks;

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

  const c42::M4Params params = c42::M4Params::repositoryDefaults();

  // ---------------------------------------------------- Baseline B rebuild
  c42::BaselinePose bB;
  try {
    bB = c42::rebuildBaselineB(scene, params);
  } catch (const std::exception& e) {
    std::cerr << "Baseline B rebuild FAILED:\n" << e.what() << "\n";
    return 1;
  }
  const c42::BaselinePose bA = c42::baselineAHistorical();

  std::cout << "=== C4.2 S2 baseline report ===\n";
  printBaseline(bB, params);
  printBaseline(bA, params);

  // ------------------------------------------------------------- checks
  add(checks, "ik.solve_success", bB.right_ik.solve_success,
      "branches=" + std::to_string(bB.right_ik.total_branches));
  add(checks, "ik.solutions_nonempty", !bB.right_ik.solutions.empty(),
      "n=" + std::to_string(bB.right_ik.solutions.size()));
  add(checks, "ik.branch_selected", bB.right_ik.ok,
      "hard_pass=" + std::to_string(bB.right_ik.n_hard_pass));
  add(checks, "ik.multiple_branches_enumerated", bB.right_ik.total_branches > 1,
      "total=" + std::to_string(bB.right_ik.total_branches));

  add(checks, "rebuild.residual_within_ik_tol", bB.post_rebuild_pos_err < 1e-4,
      "pos_err=" + std::to_string(bB.post_rebuild_pos_err) + " m");
  add(checks, "rebuild.fixes_the_9mm_error",
      bB.post_rebuild_pos_err * 10.0 < bB.pre_rebuild_pos_err,
      "pre=" + std::to_string(bB.pre_rebuild_pos_err * 1000.0) +
          " mm  post=" + std::to_string(bB.post_rebuild_pos_err * 1000.0) + " mm");
  add(checks, "rebuild.pre_rebuild_error_is_millimetre_scale",
      bB.pre_rebuild_pos_err > 0.003 && bB.pre_rebuild_pos_err < 0.020,
      "pre=" + std::to_string(bB.pre_rebuild_pos_err * 1000.0) + " mm");

  // guard against silently using right_receive_base as the baseline (R5b)
  const double q_gap = (bB.q_right - params.right_receive_base).norm();
  add(checks, "guard.q_R_differs_from_right_receive_base", q_gap > 1e-6,
      "||dq||=" + std::to_string(q_gap) + " rad");

  // How far is the *seed* (`right_receive_base`) from the analytic branches?
  // If it is far from all of them then it is not an IK solution for the RECEIVE
  // target at all -- which is precisely why it cannot serve as the baseline.
  double seed_min_gap = 1e9;
  int seed_nearest_branch = -1;
  for (std::size_t i = 0; i < bB.right_ik.solutions.size(); ++i) {
    const double d = (bB.right_ik.solutions[i] - params.right_receive_base).norm();
    if (d < seed_min_gap) {
      seed_min_gap = d;
      seed_nearest_branch = static_cast<int>(i);
    }
  }
  std::cout << "\n[seed vs branches] right_receive_base is " << seed_min_gap
            << " rad (norm) from the nearest analytic solution (branch "
            << seed_nearest_branch << " of " << bB.right_ik.solutions.size() << ")\n";
  add(checks, "seed.is_not_a_solution_of_the_receive_target", seed_min_gap > 0.05,
      "min ||q_branch - right_receive_base|| = " + std::to_string(seed_min_gap) + " rad");

  add(checks, "pose.left_t_right_translation_positive",
      bB.left_T_right.translation().norm() > 0.0,
      "|p_LR|=" + std::to_string(bB.left_T_right.translation().norm()));
  const double rod_z = bB.world_T_object.translation().z();
  add(checks, "pose.rod_above_floor", rod_z > 0.05, "rod z=" + std::to_string(rod_z));

  // ------------------------------------------------- object + collisions
  const collision_detection::AllowedCollisionMatrix obj_acm =
      c42::addRodToScene(scene, bB, params);

  c42::EnvironmentOptions env;
  env.add_floor = true;
  c42::MoveItCollisionBackend backend(scene, env);
  const auto st = scene->makeStateQ12(bB.q12Vector());
  const dual_arm::Q12 q12 = bB.q12();
  const c42::DistanceReading rd = backend.distances(st, &q12);
  std::cout << "\n[distance with rod] " << rd.to_string() << "\n";

  // Diagnose a robot<->world (rod/floor) contact by naming the closest pair.
  {
    collision_detection::DistanceRequest req;
    req.enable_nearest_points = true;
    req.acm = &backend.robotWorldAcm();
    collision_detection::DistanceResult res;
    scene->planningScene()->getCollisionEnv()->distanceRobot(req, res, st);
    std::cout << "  closest robot<->world pair: " << res.minimum_distance.link_names[0]
              << " <-> " << res.minimum_distance.link_names[1]
              << "  d=" << res.minimum_distance.distance << " m\n";
  }

  std::cout << "\n[object ACM] entries for " << params.object_id << ":\n";
  for (const std::string& link : params.object_touch_links) {
    collision_detection::AllowedCollision::Type t =
        collision_detection::AllowedCollision::NEVER;
    const bool has = obj_acm.getEntry(link, params.object_id, t);
    std::cout << "    " << link << " -> "
              << (has && t == collision_detection::AllowedCollision::ALWAYS ? "ALLOWED"
                                                                             : "checked")
              << "\n";
  }

  add(checks, "collision.q_B_arm_arm_distance_positive", rd.arm_arm > 0.0,
      "arm_arm=" + std::to_string(rd.arm_arm) + " m");
  add(checks, "collision.capsule_crosscheck_present", rd.capsule_arm_arm >= 0.0,
      "capsule=" + std::to_string(rd.capsule_arm_arm) + " m");
  add(checks, "collision.q_B_not_self_colliding", !rd.in_collision,
      "colliding=" + std::string(rd.in_collision ? "true" : "false"));

  // ---------------------------------------------------------------- report
  std::size_t npass = 0;
  std::cout << "\n=== S2 self-checks ===\n";
  for (const auto& ck : checks) {
    std::cout << (ck.pass ? "  PASS  " : "  FAIL  ") << ck.name;
    if (!ck.detail.empty()) std::cout << "   [" << ck.detail << "]";
    std::cout << "\n";
    if (ck.pass) ++npass;
  }
  std::cout << "  ---> " << npass << "/" << checks.size() << " passed\n";

  if (!json_out.empty()) {
    std::ofstream js(json_out);
    auto dumpBaseline = [&](const char* key, const c42::BaselinePose& b, bool last) {
      js << "    \"" << key << "\": {\n";
      js << "      \"id\": \"" << jsonEscape(b.id) << "\",\n";
      js << "      \"official\": " << (b.official ? "true" : "false") << ",\n";
      js << "      \"baseline_status\": \""
         << (b.provisional ? "provisional_rebuilt" : "historical_reference") << "\",\n";
      js << "      \"source\": \"" << (b.rebuilt_via_ik ? "rebuilt_via_IK" : "srdf_group_state")
         << "\",\n";
      js << "      \"q_L\": " << jointsStr(b.q_left) << ",\n";
      js << "      \"q_R\": " << jointsStr(b.q_right) << ",\n";
      js << "      \"TCP_L\": " << vecStr(b.world_T_left.translation()) << ",\n";
      js << "      \"TCP_R\": " << vecStr(b.world_T_right.translation()) << ",\n";
      js << "      \"rod\": " << vecStr(b.world_T_object.translation()) << ",\n";
      js << "      \"rel_translation_norm\": " << b.left_T_right.translation().norm() << ",\n";
      js << "      \"rel_rotvec_deg\": "
         << dual_arm::logSO3(b.left_T_right.rotation()).norm() * 180.0 / M_PI << ",\n";
      js << "      \"pre_rebuild_pos_err_m\": " << b.pre_rebuild_pos_err << ",\n";
      js << "      \"post_rebuild_pos_err_m\": " << b.post_rebuild_pos_err << ",\n";
      js << "      \"post_rebuild_rot_err_rad\": " << b.post_rebuild_rot_err << ",\n";
      js << "      \"seed_min_branch_gap_rad\": " << seed_min_gap << ",\n";
      js << "      \"ik_branches\": " << b.right_ik.total_branches << ",\n";
      js << "      \"ik_solutions\": " << b.right_ik.solutions.size() << ",\n";
      js << "      \"ik_survive_joint_limits\": " << b.right_ik.survive_joint_limits << ",\n";
      js << "      \"selector_candidates\": " << b.right_ik.n_candidates << ",\n";
      js << "      \"selector_hard_pass\": " << b.right_ik.n_hard_pass << ",\n";
      js << "      \"reject_histogram\": {";
      for (std::size_t i = 0; i < b.right_ik.reject_histogram.size(); ++i) {
        js << (i ? ", " : "") << "\"" << jsonEscape(b.right_ik.reject_histogram[i].first)
           << "\": " << b.right_ik.reject_histogram[i].second;
      }
      js << "}\n    }" << (last ? "" : ",") << "\n";
    };

    js << "{\n  \"stage\": \"C4.2-S2\",\n";
    js << "  \"baselines\": {\n";
    dumpBaseline("M4_baseline_B", bB, false);
    dumpBaseline("baseline_A_historical", bA, true);
    js << "  },\n";
    js << "  \"distance_with_rod\": {\"whole_robot\": " << rd.whole_robot
       << ", \"arm_arm\": " << rd.arm_arm
       << ", \"capsule_arm_arm\": " << rd.capsule_arm_arm
       << ", \"colliding\": " << (rd.in_collision ? "true" : "false")
       << ", \"arm_arm_pair\": [\"" << jsonEscape(rd.arm_arm_link_a) << "\", \""
       << jsonEscape(rd.arm_arm_link_b) << "\"]},\n";
    js << "  \"checks\": [\n";
    for (std::size_t i = 0; i < checks.size(); ++i) {
      js << "    {\"name\": \"" << jsonEscape(checks[i].name) << "\", \"pass\": "
         << (checks[i].pass ? "true" : "false") << ", \"detail\": \""
         << jsonEscape(checks[i].detail) << "\"}" << (i + 1 == checks.size() ? "" : ",")
         << "\n";
    }
    js << "  ],\n  \"passed\": " << npass << ",\n  \"total\": " << checks.size() << "\n}\n";
    std::cout << "wrote " << json_out << "\n";
  }

  return npass == checks.size() ? 0 : 1;
}
