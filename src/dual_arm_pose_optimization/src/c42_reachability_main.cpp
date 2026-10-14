// src/c42_reachability_main.cpp
// C4.2 / S3 acceptance sweep: the layered ladder S0 -> S4.
//
//   c42_reachability --c41-bin PATH [--p1 N] [--grid-step M] [--json OUT] [--csv OUT]
//
// Reports, per pool and combined:
//   |S1_L|/|S0|, |S1_R|/|S0|, |S2|/|S1_L and R|, |S3|/|S2|, |S4|/|S3|
// with every denominator printed alongside its numerator (plan §7).

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "dual_arm_pose_optimization/candidate_pool.hpp"
#include "dual_arm_pose_optimization/feasibility.hpp"
#include "dual_arm_pose_optimization/m4_baseline.hpp"
#include "dual_arm_pose_optimization/scene_loader.hpp"

namespace c42 = dual_arm_pose_optimization;
using Clock = std::chrono::steady_clock;

namespace {

struct Funnel {
  std::size_t total = 0;
  std::size_t s0 = 0;
  std::size_t s0_left = 0;   ///< p_L is a point the left arm has reached
  std::size_t s0_right = 0;  ///< p_R is a point the right arm has reached
  std::size_t s1_left = 0;
  std::size_t s1_right = 0;
  std::size_t s2 = 0;
  std::size_t s3 = 0;
  std::size_t s4 = 0;

  std::size_t left_branches = 0;
  std::size_t left_hard_pass = 0;
  std::size_t right_branches = 0;
  std::size_t right_hard_pass = 0;
  std::size_t pair_attempts = 0;

  double sum_mesh_arm_arm = 0.0;
  double min_mesh_arm_arm = 1e9;
  std::size_t n_clearance = 0;

  // rejection attribution for the arm that failed
  std::map<std::string, std::size_t> left_rejects;
  std::map<std::string, std::size_t> right_rejects;

  void add(const c42::CandidateResult& r) {
    ++total;
    if (r.layer.s0_point_reachable) ++s0;
    if (r.s0_left) ++s0_left;
    if (r.s0_right) ++s0_right;
    left_branches += r.left_ik.total_branches;
    right_branches += r.right_ik.total_branches;
    left_hard_pass += r.left_ik.n_hard_pass;
    right_hard_pass += r.right_ik.n_hard_pass;
    pair_attempts += r.pair_attempts;

    if (r.layer.s0_point_reachable) {
      if (r.layer.s1_left) ++s1_left; else attribute(left_rejects, r.left_ik);
      if (r.layer.s1_right) ++s1_right; else attribute(right_rejects, r.right_ik);
    }
    if (r.layer.s2_dual) ++s2;
    if (r.layer.s3_grasp) ++s3;
    if (r.layer.s4_collision_free) {
      ++s4;
      if (r.mesh_arm_arm > 0.0) {
        sum_mesh_arm_arm += r.mesh_arm_arm;
        min_mesh_arm_arm = std::min(min_mesh_arm_arm, r.mesh_arm_arm);
        ++n_clearance;
      }
    }
  }

  static void attribute(std::map<std::string, std::size_t>& m,
                        const c42::IkOutcome& ik) {
    if (!ik.solve_success) {
      m["solve_failed"] += 1;
      return;
    }
    if (ik.solutions.empty()) {
      m["no_analytic_solution"] += 1;
      return;
    }
    if (ik.n_hard_pass == 0) {
      // dominant reject reason among the scored candidates
      std::string worst = "hard_filter";
      std::size_t best = 0;
      for (const auto& kv : ik.reject_histogram) {
        if (kv.first == "accepted") continue;
        if (kv.second > best) {
          best = kv.second;
          worst = kv.first;
        }
      }
      m["all_rejected:" + worst] += 1;
      return;
    }
    m["other"] += 1;
  }

  double mean_clearance() const { return n_clearance ? sum_mesh_arm_arm / n_clearance : 0.0; }
};

std::string pct(std::size_t a, std::size_t b) {
  if (b == 0) return "n/a";
  char buf[64];
  std::snprintf(buf, sizeof(buf), "%.2f%%", 100.0 * static_cast<double>(a) /
                                                static_cast<double>(b));
  return buf;
}

void printFunnel(const std::string& name, const Funnel& f, std::ostream& os) {
  os << "\n--- funnel: " << name << " ---\n";
  os << "  evaluated candidates      : " << f.total << "\n";
  os << "  S0 point-reachable        : " << f.s0 << "  (" << pct(f.s0, f.total) << " of candidates)\n";
  os << "     S0 left half           : " << f.s0_left << "  (" << pct(f.s0_left, f.total) << ")\n";
  os << "     S0 right half          : " << f.s0_right << "  (" << pct(f.s0_right, f.total) << ")\n";
  os << "  S1 left  pose-reachable   : " << f.s1_left << "  (" << pct(f.s1_left, f.s0) << " of S0)\n";
  os << "  S1 right pose-reachable   : " << f.s1_right << "  (" << pct(f.s1_right, f.s0) << " of S0)\n";
  os << "  S2 dual pose-reachable    : " << f.s2 << "  (" << pct(f.s2, f.s0) << " of S0)\n";
  os << "  S3 grasp-consistent       : " << f.s3 << "  (" << pct(f.s3, f.s2) << " of S2)\n";
  os << "  S4 collision-free pair    : " << f.s4 << "  (" << pct(f.s4, f.s3) << " of S3)\n";
  os << "  --- diagnostics ---\n";
  os << "  analytic branches (L/R)   : " << f.left_branches << " / " << f.right_branches << "\n";
  os << "  hard-passing sols (L/R)   : " << f.left_hard_pass << " / " << f.right_hard_pass << "\n";
  os << "  solution-PAIR attempts    : " << f.pair_attempts << "\n";
  if (f.n_clearance) {
    os << "  mean cross-arm clearance  : " << f.mean_clearance()
       << " m   (min " << f.min_mesh_arm_arm << " m, n=" << f.n_clearance << ")\n";
  }
  os << "  left reject attribution   :";
  for (const auto& kv : f.left_rejects) os << " " << kv.first << "=" << kv.second;
  os << "\n  right reject attribution  :";
  for (const auto& kv : f.right_rejects) os << " " << kv.first << "=" << kv.second;
  os << "\n";
}

std::string jsonEscape(const std::string& s) {
  std::string o;
  for (char ch : s) {
    if (ch == '"' || ch == '\\') { o += '\\'; o += ch; }
    else if (ch == '\n') o += "\\n";
    else o += ch;
  }
  return o;
}

void dumpFunnelJson(std::ostream& js, const std::string& name, const Funnel& f, bool last) {
  js << "    \"" << name << "\": {\n";
  js << "      \"candidates\": " << f.total << ",\n";
  js << "      \"s0_point_reachable\": " << f.s0 << ",\n";
  js << "      \"s0_left\": " << f.s0_left << ",\n";
  js << "      \"s0_right\": " << f.s0_right << ",\n";
  js << "      \"s1_left\": " << f.s1_left << ",\n";
  js << "      \"s1_right\": " << f.s1_right << ",\n";
  js << "      \"s2_dual\": " << f.s2 << ",\n";
  js << "      \"s3_grasp\": " << f.s3 << ",\n";
  js << "      \"s4_collision_free\": " << f.s4 << ",\n";
  js << "      \"ratios\": {";
  auto ratio = [&](const char* k, std::size_t a, std::size_t b, bool last_r) {
    js << "\"" << k << "\": ";
    if (b == 0) js << "null";
    else js << (static_cast<double>(a) / static_cast<double>(b));
    js << (last_r ? "" : ", ");
  };
  ratio("S1L_over_S0", f.s1_left, f.s0, false);
  ratio("S1R_over_S0", f.s1_right, f.s0, false);
  ratio("S2_over_S0", f.s2, f.s0, false);
  ratio("S3_over_S2", f.s3, f.s2, false);
  ratio("S4_over_S3", f.s4, f.s3, false);
  ratio("S4_over_candidates", f.s4, f.total, true);
  js << "},\n";
  js << "      \"pair_attempts\": " << f.pair_attempts << ",\n";
  js << "      \"hard_pass_left\": " << f.left_hard_pass << ",\n";
  js << "      \"hard_pass_right\": " << f.right_hard_pass << ",\n";
  js << "      \"mean_cross_arm_clearance\": " << f.mean_clearance() << ",\n";
  js << "      \"min_cross_arm_clearance\": "
     << (f.n_clearance ? f.min_mesh_arm_arm : 0.0) << "\n";
  js << "    }" << (last ? "" : ",") << "\n";
}

}  // namespace

int main(int argc, char** argv) {
  std::string urdf = "src/dual_arm_kinematics/test/fixtures/s622_dual_arm.urdf";
  std::string srdf = "src/s622_moveit_config/config/s622_dual_arm.srdf";
  std::string c41 =
      "src/dual_arm_workspace_analysis/analysis/data/workspace_dual_arm.bin";
  std::string json_out, csv_out;
  std::size_t p1_max = 20000;
  double grid_step = 0.03;
  bool use_p1 = true, use_p3 = true;

  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    auto next = [&]() -> std::string {
      if (i + 1 >= argc) { std::cerr << "missing value for " << a << "\n"; std::exit(2); }
      return argv[++i];
    };
    if (a == "--urdf") urdf = next();
    else if (a == "--srdf") srdf = next();
    else if (a == "--c41-bin") c41 = next();
    else if (a == "--p1") p1_max = std::stoul(next());
    else if (a == "--grid-step") grid_step = std::stod(next());
    else if (a == "--no-p1") use_p1 = false;
    else if (a == "--no-p3") use_p3 = false;
    else if (a == "--json") json_out = next();
    else if (a == "--csv") csv_out = next();
    else if (a == "-h" || a == "--help") {
      std::cout << "usage: c42_reachability [--urdf P] [--srdf P] [--c41-bin P]\n"
                   "                       [--p1 N] [--grid-step M] [--no-p1] [--no-p3]\n"
                   "                       [--json OUT] [--csv OUT]\n";
      return 0;
    } else {
      std::cerr << "unknown argument: " << a << "\n";
      return 2;
    }
  }

  auto t0 = Clock::now();
  c42::SceneLoadOptions sopts;
  sopts.urdf_path = urdf;
  sopts.srdf_path = srdf;
  auto scene = c42::DualArmScene::load(sopts);

  const c42::M4Params m4 = c42::M4Params::repositoryDefaults();
  const c42::BaselinePose baseB = c42::rebuildBaselineB(scene, m4);

  c42::WorkspaceIndex::Options wopts;
  wopts.c41_bin = c41;
  wopts.max_handover_samples = p1_max;
  const c42::WorkspaceIndex index = c42::WorkspaceIndex::load(wopts);

  auto t1 = Clock::now();
  std::cout << std::fixed << std::setprecision(4);
  std::cout << "=== C4.2 / S3 layered reachability ===\n";
  std::cout << "[C4.1 index] records=" << index.totalRecords()
            << "  voxel=" << index.voxel() << " m\n";
  std::cout << "  left reachable voxels  : " << index.countLeft() << "\n";
  std::cout << "  right reachable voxels : " << index.countRight() << "\n";
  std::cout << "  CRW voxels (both)      : " << index.countDual() << "\n";
  std::cout << "  bbox min/max           : [" << index.bboxMin().transpose() << "] .. ["
            << index.bboxMax().transpose() << "]\n";
  std::cout << "  handover samples (P1)  : " << index.handoverSamples().size() << "\n";
  std::cout << "  load took              : "
            << std::chrono::duration<double>(t1 - t0).count() << " s\n";

  c42::FeasibilityPipeline pipeline(scene, m4, baseB);

  // ---- pools -------------------------------------------------------------
  // Pool character matters (see candidate_pool.hpp):
  //   P1a "left-hold reachable" : object centre DERIVED from the sample's own
  //        left-arm joints, p_O = FK(q_L) * (^G_L T_O).  The left arm can
  //        provably hold this rod pose, so S1_L is ~100% by construction and the
  //        funnel becomes a statement about the RIGHT arm.
  //   P1b "midpoint prior"     : the C4.1 sample's own object-frame midpoint,
  //        which is NOT tied to a particular q_L.
  //   P3  "grid"               : uniform over the CRW bbox, unfiltered.
  dual_arm::DualArmKinematics fk;
  const Eigen::Matrix4d G_L_T_O = m4.objectInLeftGrasp().matrix();
  std::vector<c42::Candidate> candidates;
  if (use_p1) {
    for (const auto& s : index.handoverSamples()) {
      const Eigen::Matrix4d TL =
          fk.worldTgrasp(dual_arm::Arm::Left, s.q_left).matrix();
      c42::Candidate a;
      a.p_O = (TL * G_L_T_O).block<3, 1>(0, 3);
      a.R_O = baseB.world_T_object.rotation();
      a.pool = "P1a_left_hold";
      candidates.push_back(a);

      c42::Candidate b;
      b.p_O = s.p_object;
      b.R_O = baseB.world_T_object.rotation();
      b.pool = "P1b_midpoint";
      candidates.push_back(b);
    }
  }
  if (use_p3) {
    for (const auto& p : index.gridObjectCentres(grid_step)) {
      c42::Candidate c;
      c.p_O = p;
      c.R_O = baseB.world_T_object.rotation();
      c.pool = "P3_grid";
      candidates.push_back(c);
    }
  }

  std::cout << "\nevaluating " << candidates.size() << " candidates ...\n" << std::flush;

  Funnel all, f1a, f1b, f3;
  std::ofstream csv;
  if (!csv_out.empty()) {
    csv.open(csv_out);
    csv << "pool,px,py,pz,s0,s1_left,s1_right,s2,s3,s4,pairs,mesh_arm_arm,pair_a,pair_b,"
           "smin_left,smin_right,w_rel,joint_margin,deviation,continuity,"
           "arm_proximal,prox_a,prox_b\n";
  }

  auto t2 = Clock::now();
  for (const auto& c : candidates) {
    const c42::CandidateResult r = pipeline.evaluate(c, &index);
    all.add(r);
    if (c.pool == "P1a_left_hold") f1a.add(r);
    else if (c.pool == "P1b_midpoint") f1b.add(r);
    else f3.add(r);
    if (csv) {
      csv << c.pool << "," << c.p_O.x() << "," << c.p_O.y() << "," << c.p_O.z() << ","
          << r.layer.s0_point_reachable << "," << r.layer.s1_left << ","
          << r.layer.s1_right << "," << r.layer.s2_dual << "," << r.layer.s3_grasp << ","
          << r.layer.s4_collision_free << "," << r.pair_attempts << ","
          << r.mesh_arm_arm << "," << r.mesh_pair_a << "," << r.mesh_pair_b << ","
          << r.smin_left << "," << r.smin_right << "," << r.w_rel << ","
          << r.joint_margin << "," << r.deviation << "," << r.continuity << ","
          << r.arm_proximal_clearance << "," << r.arm_proximal_pair_a << ","
          << r.arm_proximal_pair_b << "\n";
    }
  }
  const double sweep_s = std::chrono::duration<double>(Clock::now() - t2).count();

  printFunnel("ALL", all, std::cout);
  if (use_p1) {
    printFunnel("P1a_left_hold", f1a, std::cout);
    printFunnel("P1b_midpoint", f1b, std::cout);
  }
  if (use_p3) printFunnel("P3_grid", f3, std::cout);

  std::cout << "\n--- cost ---\n";
  std::cout << "  sweep                     : " << sweep_s << " s for " << all.total
            << " candidates\n";
  std::cout << "  per candidate             : "
            << (all.total ? 1000.0 * sweep_s / static_cast<double>(all.total) : 0.0)
            << " ms\n";

  // ---- S0..S4 nesting must hold by construction --------------------------
  const bool nested = all.s4 <= all.s3 && all.s3 <= all.s2 && all.s2 <= all.s0 &&
                      all.s2 <= all.s1_left && all.s2 <= all.s1_right;

  // P1a's left TCP is FK(q_L) of a configuration C4.1 actually sampled, so the
  // LEFT half of S0 must be ~100% for that pool.  Anything much below that means
  // either the voxel grid or the FK-derived pool is wrong.
  const bool p1a_left_ok =
      !use_p1 || f1a.total == 0 ||
      (static_cast<double>(f1a.s0_left) / static_cast<double>(f1a.total) > 0.95);

  // S3 is a structural identity under the rigid-grasp model; it must be exactly
  // 100% of S2 in EVERY pool, otherwise the target derivation is inconsistent.
  auto s3_full = [](const Funnel& f) { return f.s2 == 0 || f.s3 == f.s2; };
  const bool s3_structural = s3_full(all) && s3_full(f1a) && s3_full(f1b) && s3_full(f3);

  int npass = 0;
  const int ntotal = 3;
  std::cout << "\n=== S3 self-checks ===\n";
  std::cout << (nested ? "  PASS  " : "  FAIL  ")
            << "nesting S4 <= S3 <= S2 <= min(S1_L,S1_R) <= S0\n";
  npass += nested ? 1 : 0;
  std::cout << (p1a_left_ok ? "  PASS  " : "  FAIL  ")
            << "P1a S0-left ~ 100% (FK-derived pool + mask agree; got "
            << (f1a.total ? 100.0 * static_cast<double>(f1a.s0_left) /
                                static_cast<double>(f1a.total)
                          : 0.0)
            << "%)\n";
  npass += p1a_left_ok ? 1 : 0;
  std::cout << (s3_structural ? "  PASS  " : "  FAIL  ")
            << "S3 == S2 in every pool (structural identity under rigid grasp)\n";
  npass += s3_structural ? 1 : 0;
  std::cout << "  ---> " << npass << "/" << ntotal << " passed\n";
  const bool all_ok = (npass == ntotal);

  if (!json_out.empty()) {
    std::ofstream js(json_out);
    js << std::fixed << std::setprecision(6);
    js << "{\n  \"stage\": \"C4.2-S3\",\n";
    js << "  \"c41_index\": {\"records\": " << index.totalRecords()
       << ", \"voxel\": " << index.voxel() << ", \"left_voxels\": " << index.countLeft()
       << ", \"right_voxels\": " << index.countRight()
       << ", \"crw_voxels\": " << index.countDual()
       << ", \"handover_samples\": " << index.handoverSamples().size() << "},\n";
    js << "  \"options\": {\"p1_max\": " << p1_max << ", \"grid_step\": " << grid_step
       << ", \"use_p1\": " << (use_p1 ? "true" : "false")
       << ", \"use_p3\": " << (use_p3 ? "true" : "false") << "},\n";
    js << "  \"sweep_seconds\": " << sweep_s << ",\n";
    js << "  \"funnels\": {\n";
    dumpFunnelJson(js, "ALL", all, false);
    dumpFunnelJson(js, "P1a_left_hold", f1a, false);
    dumpFunnelJson(js, "P1b_midpoint", f1b, false);
    dumpFunnelJson(js, "P3_grid", f3, true);
    js << "  },\n";
    js << "  \"nesting_ok\": " << (nested ? "true" : "false") << ",\n";
    js << "  \"p1a_s0_left_ok\": " << (p1a_left_ok ? "true" : "false") << ",\n";
    js << "  \"s3_structural\": " << (s3_structural ? "true" : "false") << ",\n";
    js << "  \"checks_passed\": " << npass << ",\n";
    js << "  \"checks_total\": " << ntotal << ",\n";
    js << "  \"notes\": {\n";
    js << "    \"s3_is_structural\": true,\n";
    js << "    \"reason\": \"rigid-grasp model: T_R^des = T_L^des * T_rel^B, so the H4 "
          "relative-grasp residual carries only the IK residual\"\n";
    js << "  }\n}\n";
    std::cout << "wrote " << json_out << "\n";
  }
  return all_ok ? 0 : 1;
}
