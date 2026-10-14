// src/c42_handover_target_main.cpp
// C4.2 -> simulation bridge (step 1 of 2).
//
// Turns a C4.2 handover POSE into an executable JOINT target:
//
//   p_O, rotvec  ->  T_O  ->  { T_L = T_O * (^G_L T_O)^-1 ,  T_R = T_L * T_rel^B }
//                ->  per-arm analytic IK + IKSelector
//                ->  solution-PAIR collision screening
//                ->  q12  +  a verification of what FK actually achieves
//
// The output JSON is consumed by analysis/send_handover.py, which ships the
// trajectory to the two arm JointTrajectoryControllers in Gazebo.
//
// Defaults are the C4.2 recommendation (P95 scales, V1): the pose that S6
// selected after fixing the objective-saturation defect.

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "dual_arm_pose_optimization/feasibility.hpp"
#include "dual_arm_pose_optimization/m4_baseline.hpp"

namespace c42 = dual_arm_pose_optimization;

namespace {

std::vector<double> parseDoubles(const std::string& s) {
  std::vector<double> v;
  std::string cur;
  for (char c : s + ",") {
    if (c == ',') {
      if (!cur.empty()) v.push_back(std::stod(cur));
      cur.clear();
    } else {
      cur += c;
    }
  }
  return v;
}

}  // namespace

int main(int argc, char** argv) {
  std::string urdf = "src/dual_arm_kinematics/test/fixtures/s622_dual_arm.urdf";
  std::string srdf = "src/s622_moveit_config/config/s622_dual_arm.srdf";
  std::string c41 = "src/dual_arm_workspace_analysis/analysis/data/workspace_dual_arm.bin";
  std::string json_out;
  // C4.2 recommendation: P95 scales, V1 (p_O + ||dtheta|| = 10 deg)
  std::string p_o_str = "0.079164,-0.228536,0.165815";
  std::string rotvec_str = "5.655735,-2.090612,-7.977594";  // degrees
  bool use_index = false;
  bool use_baseline_pose = false;
  std::string verify_q12;   ///< measure an ACHIEVED configuration instead

  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    auto next = [&]() -> std::string {
      if (i + 1 >= argc) { std::cerr << "missing value for " << a << "\n"; std::exit(2); }
      return argv[++i];
    };
    if (a == "--urdf") urdf = next();
    else if (a == "--srdf") srdf = next();
    else if (a == "--c41-bin") c41 = next();
    else if (a == "--p-o") p_o_str = next();
    else if (a == "--rotvec-deg") rotvec_str = next();
    else if (a == "--json") json_out = next();
    else if (a == "--use-index") use_index = true;
    else if (a == "--baseline-pose") use_baseline_pose = true;
    else if (a == "--verify-q12") verify_q12 = next();
    else if (a == "-h" || a == "--help") {
      std::cout << "usage: c42_handover_target [--p-o x,y,z] [--rotvec-deg rx,ry,rz]\n"
                   "                           [--json OUT] [--use-index] [--baseline-pose]\n"
                   "                           [--urdf P] [--srdf P] [--c41-bin P]\n";
      return 0;
    } else {
      std::cerr << "unknown argument: " << a << "\n";
      return 2;
    }
  }

  c42::SceneLoadOptions sopts;
  sopts.urdf_path = urdf;
  sopts.srdf_path = srdf;
  auto scene = c42::DualArmScene::load(sopts);
  const c42::M4Params m4 = c42::M4Params::repositoryDefaults();
  const c42::BaselinePose baseB = c42::rebuildBaselineB(scene, m4);
  c42::FeasibilityPipeline pipeline(scene, m4, baseB);

  std::unique_ptr<c42::WorkspaceIndex> index;
  if (use_index) {
    c42::WorkspaceIndex::Options o;
    o.c41_bin = c41;
    o.max_handover_samples = 100;
    index = std::make_unique<c42::WorkspaceIndex>(c42::WorkspaceIndex::load(o));
  }

  c42::Candidate c;
  if (use_baseline_pose) {
    c.p_O = baseB.world_T_object.translation();
    c.R_O = baseB.world_T_object.rotation();
  } else {
    const auto p = parseDoubles(p_o_str);
    const auto rv = parseDoubles(rotvec_str);
    if (p.size() != 3 || rv.size() != 3) {
      std::cerr << "--p-o and --rotvec-deg each need 3 values\n";
      return 2;
    }
    c.p_O = Eigen::Vector3d(p[0], p[1], p[2]);
    const Eigen::Vector3d w(rv[0] * M_PI / 180.0, rv[1] * M_PI / 180.0,
                            rv[2] * M_PI / 180.0);
    c.R_O = baseB.world_T_object.rotation() * dual_arm::expSO3(w);
  }
  c.pool = "handover_target";

  std::cout << std::fixed << std::setprecision(6);

  // ---- verification mode: measure a configuration the SIMULATOR achieved ----
  if (!verify_q12.empty()) {
    const auto v = parseDoubles(verify_q12);
    if (v.size() != 12) { std::cerr << "--verify-q12 needs 12 values\n"; return 2; }
    dual_arm::Q12 q;
    for (int i = 0; i < 12; ++i) q[i] = v[static_cast<std::size_t>(i)];
    const c42::CandidateResult m = c42::evaluateConfiguration(
        scene, pipeline.collision(), m4, baseB, q);

    const Eigen::Vector3d p_des(0.079164, -0.228536, 0.165815);
    const Eigen::Matrix3d R_des =
        baseB.world_T_object.rotation() *
        dual_arm::expSO3(Eigen::Vector3d(5.655735, -2.090612, -7.977594) * M_PI / 180.0);
    const Eigen::Vector3d dp = m.candidate.p_O - p_des;
    const double dr =
        dual_arm::logSO3(R_des.transpose() * m.candidate.R_O).norm() * 180.0 / M_PI;

    std::cout << "=== achieved configuration vs the C4.2 target ===\n";
    std::cout << "  object position error : " << dp.norm() * 1e3 << " mm  (d="
              << dp.transpose() * 1e3 << " mm)\n";
    std::cout << "  object rotation error : " << dr << " deg\n";
    std::cout << "\n  clearance     achieved      C4.2 predicted\n";
    std::cout << "  arm_arm       " << m.mesh_arm_arm << " m    0.033658 m\n";
    std::cout << "  arm_proximal  " << m.arm_proximal_clearance << " m    0.216672 m\n";
    std::cout << "\n  S4 collision-free at the achieved pose: "
              << (m.layer.s4_collision_free ? "yes" : "NO") << "\n";
    std::cout << "  joint margin (achieved): " << m.joint_margin << " rad\n";
    return m.layer.s4_collision_free ? 0 : 1;
  }

  std::cout << "=== C4.2 handover target -> joint target ===\n";
  std::cout << "p_O    = [" << c.p_O.transpose() << "]\n";
  const Eigen::Vector3d rv = dual_arm::logSO3(
      baseB.world_T_object.rotation().transpose() * c.R_O);
  std::cout << "dtheta = " << rv.norm() * 180.0 / M_PI << " deg\n";

  const c42::CandidateResult r = pipeline.evaluate(c, index.get());

  std::cout << "\n--- layer membership ---\n";
  std::cout << "  S0 point reachable : " << (r.layer.s0_point_reachable ? "yes" : "no") << "\n";
  std::cout << "  S1 left / right    : " << (r.layer.s1_left ? "yes" : "no") << " / "
            << (r.layer.s1_right ? "yes" : "no") << "\n";
  std::cout << "  S2 dual            : " << (r.layer.s2_dual ? "yes" : "no") << "\n";
  std::cout << "  S3 grasp           : " << (r.layer.s3_grasp ? "yes" : "no") << "\n";
  std::cout << "  S4 collision-free  : " << (r.layer.s4_collision_free ? "yes" : "no") << "\n";
  std::cout << "  pair attempts      : " << r.pair_attempts << "\n";

  if (!r.has_chosen_pair) {
    std::cerr << "\nNo executable solution pair: the target cannot be reached.\n";
    return 1;
  }

  dual_arm::JointConfig ql = r.q_left_chosen, qr = r.q_right_chosen;
  const dual_arm::Q12 q12 = dual_arm::DualArmKinematics::stack(ql, qr);

  // ---- verification: what does FK actually reach? ------------------------
  dual_arm::DualArmKinematics kin;
  const dual_arm::Iso3 wl = kin.worldTgrasp(dual_arm::Arm::Left, ql);
  const dual_arm::Iso3 wr = kin.worldTgrasp(dual_arm::Arm::Right, qr);
  const dual_arm::Iso3 T_O_achieved = wl * m4.objectInLeftGrasp();
  const Eigen::Vector3d dp = T_O_achieved.translation() - c.p_O;
  const double drot = dual_arm::logSO3(c.R_O.transpose() * T_O_achieved.rotation()).norm();
  const double rel_err = (wl.matrix().inverse() * wr.matrix() - baseB.left_T_right.matrix()).norm();

  std::cout << "\n--- FK verification of the joint target ---\n";
  std::cout << "  object position error : " << dp.norm() * 1e6 << " um\n";
  std::cout << "  object rotation error : " << drot * 180.0 / M_PI << " deg\n";
  std::cout << "  ^L T_R vs T_rel^B     : " << rel_err << " (matrix norm)\n";
  std::cout << "\n--- clearedances / metrics ---\n";
  std::cout << "  arm_arm (all pairs)   : " << r.mesh_arm_arm << " m\n";
  std::cout << "  arm_proximal          : " << r.arm_proximal_clearance << " m\n";
  std::cout << "  joint margin          : " << r.joint_margin << " rad\n";
  std::cout << "  sigma_min L / R       : " << r.smin_left << " / " << r.smin_right << "\n";

  std::cout << "\n--- q12 (C2 joint order: left_j1..6, right_j1..6) ---\n";
  for (int i = 0; i < 6; ++i) std::cout << "  left_j" << (i + 1) << "  " << ql[i] << "\n";
  for (int i = 0; i < 6; ++i) std::cout << "  right_j" << (i + 1) << " " << qr[i] << "\n";

  if (!json_out.empty()) {
    std::ofstream js(json_out);
    js << std::fixed << std::setprecision(9);
    js << "{\n  \"source\": \"C4.2 handover target (P95 scales, V1 recommendation)\",\n";
    js << "  \"p_O\": [" << c.p_O.x() << ", " << c.p_O.y() << ", " << c.p_O.z() << "],\n";
    js << "  \"dtheta_deg\": " << rv.norm() * 180.0 / M_PI << ",\n";
    js << "  \"layer\": {\"s0\": " << r.layer.s0_point_reachable
       << ", \"s1_left\": " << r.layer.s1_left << ", \"s1_right\": " << r.layer.s1_right
       << ", \"s2\": " << r.layer.s2_dual << ", \"s3\": " << r.layer.s3_grasp
       << ", \"s4\": " << r.layer.s4_collision_free << "},\n";
    js << "  \"q_left\": [";
    for (int i = 0; i < 6; ++i) js << (i ? ", " : "") << ql[i];
    js << "],\n  \"q_right\": [";
    for (int i = 0; i < 6; ++i) js << (i ? ", " : "") << qr[i];
    js << "],\n  \"q12\": [";
    for (int i = 0; i < 12; ++i) js << (i ? ", " : "") << q12[i];
    js << "],\n";
    js << "  \"verification\": {\"object_pos_err_m\": " << dp.norm()
       << ", \"object_rot_err_rad\": " << drot
       << ", \"rel_grasp_matrix_err\": " << rel_err << "},\n";
    js << "  \"metrics\": {\"arm_arm\": " << r.mesh_arm_arm
       << ", \"arm_proximal\": " << r.arm_proximal_clearance
       << ", \"joint_margin\": " << r.joint_margin
       << ", \"smin_left\": " << r.smin_left << ", \"smin_right\": " << r.smin_right << "},\n";
    js << "  \"joint_names\": [\"left_j1\",\"left_j2\",\"left_j3\",\"left_j4\",\"left_j5\",\"left_j6\","
          "\"right_j1\",\"right_j2\",\"right_j3\",\"right_j4\",\"right_j5\",\"right_j6\"]\n}\n";
    std::cout << "\nwrote " << json_out << "\n";
  }

  const bool ok = r.layer.s4_collision_free && dp.norm() < 1e-6 && drot < 1e-9;
  std::cout << "\n" << (ok ? "PASS" : "FAIL")
            << ": target is S4-feasible and FK reproduces it\n";
  return ok ? 0 : 1;
}
