// src/c42_optimize_pose_main.cpp
// C4.2 / S5 acceptance run:
//   * V0 refinement  (p_O only, R_O fixed at M4)
//   * V1 refinement  (p_O + ||dtheta||_2 <= theta_max about R_O^M4)
//   * theta_max sensitivity (5 / 10 / 15 deg, same norm definition)
//   * standby pose search, scored by JOINT-SPACE SEGMENT FEASIBILITY to the
//     optimised handover (explicitly NOT called "planning success")
// Outputs analysis/figs/c42_s5_optimization.json for the Python plotting step.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <random>
#include <vector>

#include "dual_arm_pose_optimization/m4_baseline.hpp"
#include "dual_arm_pose_optimization/optimizer.hpp"

namespace c42 = dual_arm_pose_optimization;
using Clock = std::chrono::steady_clock;

namespace {

std::string jsonEscape(const std::string& s) {
  std::string o;
  for (char ch : s) {
    if (ch == '"' || ch == '\\') { o += '\\'; o += ch; }
    else o += ch;
  }
  return o;
}

std::vector<std::string> split(const std::string& s, char d) {
  std::vector<std::string> out;
  std::string cur;
  for (char c : s) {
    if (c == d) { out.push_back(cur); cur.clear(); }
    else cur += c;
  }
  out.push_back(cur);
  return out;
}

std::vector<double> parseDoubles(const std::string& s) {
  std::vector<double> v;
  for (const auto& t : split(s, ',')) {
    if (!t.empty()) v.push_back(std::stod(t));
  }
  return v;
}

struct Seed {
  c42::Candidate candidate;
  Eigen::Matrix3d R_O;
  double score = 0.0;
};

void writePose(std::ostream& os, const Eigen::Vector3d& p, const Eigen::Vector3d& rv) {
  os << "[" << p.x() << ", " << p.y() << ", " << p.z() << "]";
  os << ", \"rotvec_deg\": [" << rv.x() * 180.0 / M_PI << ", " << rv.y() * 180.0 / M_PI
     << ", " << rv.z() * 180.0 / M_PI << "]";
}

void dumpResult(std::ostream& os, const c42::RefineResult& r, const c42::ObjectiveSpec& spec,
                const std::string& indent) {
  const auto j = c42::normalisedObjectives(r.result, spec);
  os << indent << "\"score\": " << r.score << ",\n";
  os << indent << "\"feasible\": " << (r.feasible ? "true" : "false") << ",\n";
  os << indent << "\"evals\": " << r.evals << ",\n";
  os << indent << "\"p_O\": ";
  writePose(os, r.best.p_O, r.best.rotvec);
  os << ",\n";
  os << indent << "\"theta_norm_deg\": " << r.best.rotvec.norm() * 180.0 / M_PI << ",\n";
  os << indent << "\"metrics\": {\"smin_left\": " << r.result.smin_left
     << ", \"smin_right\": " << r.result.smin_right << ", \"w_rel\": " << r.result.w_rel
     << ", \"arm_arm\": " << r.result.mesh_arm_arm
     << ", \"arm_proximal\": " << r.result.arm_proximal_clearance
     << ", \"joint_margin\": " << r.result.joint_margin
     << ", \"deviation\": " << r.result.deviation
     << ", \"continuity\": " << r.result.continuity << "},\n";
  os << indent << "\"normalised\": [";
  for (int i = 0; i < 8; ++i) os << (i ? ", " : "") << j[i];
  os << "]\n";
}

}  // namespace

int main(int argc, char** argv) {
  std::string urdf = "src/dual_arm_kinematics/test/fixtures/s622_dual_arm.urdf";
  std::string srdf = "src/s622_moveit_config/config/s622_dual_arm.srdf";
  std::string c41 =
      "src/dual_arm_workspace_analysis/analysis/data/workspace_dual_arm.bin";
  std::string csv = "src/dual_arm_pose_optimization/analysis/figs/c42_s3_candidates.csv";
  std::string json_out;
  std::string weight_set = "equal";
  std::string scales;
  int top_k = 12;
  int standby_n = 20000;
  int seg_samples = 32;
  int multi_seed = 0;          ///< 0 = off; N = run N randomised refinement starts
  unsigned multi_seed_seed = 20261014;

  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    auto next = [&]() -> std::string {
      if (i + 1 >= argc) { std::cerr << "missing value for " << a << "\n"; std::exit(2); }
      return argv[++i];
    };
    if (a == "--urdf") urdf = next();
    else if (a == "--srdf") srdf = next();
    else if (a == "--c41-bin") c41 = next();
    else if (a == "--csv") csv = next();
    else if (a == "--json") json_out = next();
    else if (a == "--weights") weight_set = next();
    else if (a == "--scales") scales = next();
    else if (a == "--top-k") top_k = std::stoi(next());
    else if (a == "--standby-n") standby_n = std::stoi(next());
    else if (a == "--seg-samples") seg_samples = std::stoi(next());
    else if (a == "--multi-seed") multi_seed = std::stoi(next());
    else if (a == "--multi-seed-seed") multi_seed_seed = std::stoul(next());
    else if (a == "-h" || a == "--help") {
      std::cout << "usage: c42_optimize_pose [--urdf P] [--srdf P] [--c41-bin P] [--csv P]\n"
                   "                        [--json OUT] [--weights equal|precision_first|safety_first]\n"
                   "                        [--scales a,b,c,d,e,f,g,h] [--top-k N]\n"
                   "                        [--standby-n N] [--seg-samples N]\n";
      return 0;
    } else {
      std::cerr << "unknown argument: " << a << "\n";
      return 2;
    }
  }

  c42::ObjectiveSpec spec = c42::ObjectiveSpec::named(weight_set);
  if (!scales.empty()) {
    const auto v = parseDoubles(scales);
    if (v.size() != 8) { std::cerr << "--scales needs 8 values\n"; return 2; }
    for (int i = 0; i < 8; ++i) spec.scale[i] = v[static_cast<std::size_t>(i)];
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
  wopts.max_handover_samples = 1000;  // masks only; P1 not needed here
  const c42::WorkspaceIndex index = c42::WorkspaceIndex::load(wopts);

  c42::FeasibilityPipeline pipeline(scene, m4, baseB);
  const Eigen::Matrix3d R_O_m4 = baseB.world_T_object.rotation();

  std::cout << std::fixed << std::setprecision(5);
  std::cout << "=== C4.2 / S5 pose optimisation ===\n";
  std::cout << "weight set: " << weight_set << "   top-k seeds: " << top_k << "\n";

  // ---------------------------------------------------- seeds from the S3 table
  std::vector<Seed> seeds;
  {
    std::ifstream in(csv);
    if (!in) { std::cerr << "cannot read " << csv << "\n"; return 1; }
    std::string line;
    std::getline(in, line);  // header
    const auto hdr = split(line, ',');
    auto col = [&](const std::string& n) {
      for (std::size_t i = 0; i < hdr.size(); ++i) if (hdr[i] == n) return i;
      return hdr.size();
    };
    const std::size_t cPx = col("px"), cPy = col("py"), cPz = col("pz"), cS4 = col("s4");
    const std::size_t cSL = col("smin_left"), cSR = col("smin_right"), cW = col("w_rel");
    const std::size_t cAA = col("mesh_arm_arm"), cJM = col("joint_margin");
    const std::size_t cDV = col("deviation"), cCT = col("continuity"), cAP = col("arm_proximal");
    while (std::getline(in, line)) {
      const auto f = split(line, ',');
      if (f.size() < hdr.size() || f[cS4] != "1") continue;
      c42::CandidateResult r;
      r.layer.s4_collision_free = true;
      r.candidate.p_O = Eigen::Vector3d(std::stod(f[cPx]), std::stod(f[cPy]), std::stod(f[cPz]));
      r.smin_left = std::stod(f[cSL]);
      r.smin_right = std::stod(f[cSR]);
      r.w_rel = std::stod(f[cW]);
      r.mesh_arm_arm = std::stod(f[cAA]);
      r.joint_margin = std::stod(f[cJM]);
      r.deviation = std::stod(f[cDV]);
      r.continuity = std::stod(f[cCT]);
      r.arm_proximal_clearance = std::stod(f[cAP]);
      Seed s;
      s.candidate = r.candidate;
      s.R_O = R_O_m4;
      s.score = c42::scoreResult(r, spec);
      seeds.push_back(s);
    }
  }
  std::sort(seeds.begin(), seeds.end(),
            [](const Seed& a, const Seed& b) { return a.score > b.score; });
  if (static_cast<int>(seeds.size()) > top_k) seeds.resize(static_cast<std::size_t>(top_k));
  std::cout << "seeds: " << seeds.size() << "  best seed score = "
            << (seeds.empty() ? 0.0 : seeds[0].score) << "\n";
  if (seeds.empty()) { std::cerr << "no S4 seeds found\n"; return 1; }

  // ---------------------------------------------------------------- V0 (3 DOF)
  c42::RefineOptions v0opts;
  v0opts.allow_orientation = false;
  v0opts.max_evals = 400;
  c42::PoseRefiner v0(&pipeline, &index, spec, v0opts);

  c42::RefineResult best_v0;
  best_v0.score = -1e9;
  for (const auto& s : seeds) {
    c42::PoseParams p;
    p.p_O = s.candidate.p_O;
    const auto r = v0.refine(p, R_O_m4);
    if (r.score > best_v0.score) best_v0 = r;
  }
  std::cout << "V0 best score = " << best_v0.score << "  p_O = ["
            << best_v0.best.p_O.transpose() << "]\n";

  // ------------------------------------------------- V1 (6 DOF, ||dtheta||<=)
  auto runV1 = [&](double theta_deg, int evals) {
    c42::RefineOptions o;
    o.allow_orientation = true;
    o.theta_max = theta_deg * M_PI / 180.0;
    o.max_evals = evals;
    c42::PoseRefiner ref(&pipeline, &index, spec, o);
    c42::RefineResult best;
    best.score = -1e9;
    // start from the V0 optimum and from the top seeds
    c42::PoseParams p0;
    p0.p_O = best_v0.best.p_O;
    auto r0 = ref.refine(p0, R_O_m4);
    best = r0;
    for (const auto& s : seeds) {
      c42::PoseParams p;
      p.p_O = s.candidate.p_O;
      const auto r = ref.refine(p, R_O_m4);
      if (r.score > best.score) best = r;
    }
    return best;
  };

  const c42::RefineResult best_v1 = runV1(10.0, 600);

  // ------------------------------------------------ Baseline B in the same metric set
  const c42::CandidateResult baseline_eval = c42::evaluateConfiguration(
      scene, pipeline.collision(), m4, baseB, baseB.q12());
  const double baseline_score = c42::scoreResult(baseline_eval, spec);
  std::cout << "Baseline B (same metric set): score " << baseline_score
            << "  smin_L " << baseline_eval.smin_left << "  smin_R "
            << baseline_eval.smin_right << "  arm_proximal "
            << baseline_eval.arm_proximal_clearance << "\n";

  // ------------------------------------------------ multi-seed experiment (S6)
  struct SeedRun { double score; bool feasible; Eigen::Vector3d p_O; };
  std::vector<SeedRun> multi;
  if (multi_seed > 0) {
    std::mt19937_64 rng(multi_seed_seed);
    std::normal_distribution<double> jitter(0.0, 0.02);
    std::uniform_int_distribution<std::size_t> pick(0, std::min<std::size_t>(seeds.size(), 200) - 1);
    c42::RefineOptions o;
    o.allow_orientation = true;
    o.theta_max = 10.0 * M_PI / 180.0;
    o.max_evals = 250;
    c42::PoseRefiner ref(&pipeline, &index, spec, o);
    for (int i = 0; i < multi_seed; ++i) {
      c42::PoseParams p;
      p.p_O = seeds[pick(rng)].candidate.p_O + Eigen::Vector3d(jitter(rng), jitter(rng), jitter(rng));
      const auto r = ref.refine(p, R_O_m4);
      multi.push_back({r.score, r.feasible, r.best.p_O});
    }
    std::sort(multi.begin(), multi.end(),
              [](const SeedRun& a, const SeedRun& b) { return a.score > b.score; });
    int feas = 0;
    for (const auto& m : multi) feas += m.feasible ? 1 : 0;
    std::cout << "multi-seed: " << multi.size() << " runs, " << feas
              << " feasible, best " << multi.front().score << " worst "
              << multi.back().score << "\n";
  }
  std::cout << "V1 best score = " << best_v1.score
            << "  ||dtheta|| = " << best_v1.best.rotvec.norm() * 180.0 / M_PI << " deg\n";

  // --------------------------------------------- theta_max sensitivity (norm)
  struct ThetaRow { double deg; double score; double used_deg; bool feasible; };
  std::vector<ThetaRow> theta_rows;
  for (double d : {5.0, 10.0, 15.0}) {
    const auto r = runV1(d, 500);
    theta_rows.push_back({d, r.score, r.best.rotvec.norm() * 180.0 / M_PI, r.feasible});
    std::cout << "  theta_max = " << d << " deg -> score " << r.score
              << "  (used " << r.best.rotvec.norm() * 180.0 / M_PI << " deg)\n";
  }

  // -------------------------------------------------------------- standby pose
  // Definition (A6): a safe preparation configuration for the next handover,
  // referenced to the OPTIMISED handover.  Primary criteria, in order:
  //   segment feasibility to the handover configuration, path length, joint
  //   motion, safety clearance; cartesian distance is auxiliary only.
  // NOTE: the metric is joint-space STRAIGHT-LINE segment feasibility, which is
  // deliberately NOT called "planning success" (see the S5 note in the record).
  const auto q12_best = [&]() {
    std::vector<double> v(12, 0.0);
    // rebuild the handover configuration for the V1 optimum
    c42::Candidate c;
    c.p_O = best_v1.best.p_O;
    c.R_O = R_O_m4 * dual_arm::expSO3(best_v1.best.rotvec);
    c.pool = "v1";
    const auto r = pipeline.evaluate(c, &index);
    for (int i = 0; i < 6; ++i) {
      v[static_cast<std::size_t>(i)] = r.q_left_chosen[i];
      v[static_cast<std::size_t>(i + 6)] = r.q_right_chosen[i];
    }
    return v;
  }();
  dual_arm::Q12 q_h;
  for (int i = 0; i < 12; ++i) q_h[i] = q12_best[static_cast<std::size_t>(i)];

  std::vector<dual_arm::Q12> pool = index.sampleQ12(static_cast<std::size_t>(standby_n));
  std::cout << "\nstandby pool: " << pool.size() << " configurations\n";

  struct StandbyRow {
    dual_arm::Q12 q;
    bool seg_ok = false;
    double path = 0.0, joint_motion = 0.0, clearance = 0.0, cart = 0.0;
    double score = 0.0;
  };
  std::vector<StandbyRow> rows;
  rows.reserve(pool.size());

  const auto* backend = pipeline.collision();
  dual_arm::DualArmKinematics fk;

  // pass 1: hard filters + raw metrics
  for (const auto& q : pool) {
    const std::vector<double> qv(q.data(), q.data() + 12);
    const auto st = scene->makeStateQ12(qv);
    if (backend->isStateColliding(st)) continue;  // rod is at the handover pose

    std::vector<double> mid(12, 0.0);
    for (int i = 0; i < 12; ++i) mid[static_cast<std::size_t>(i)] = q[i];
    const Eigen::Vector3d mid_s =
        0.5 * (fk.worldTgrasp(dual_arm::Arm::Left, Eigen::Map<const dual_arm::JointConfig>(q.data())).translation() +
               fk.worldTgrasp(dual_arm::Arm::Right,
                              Eigen::Map<const dual_arm::JointConfig>(q.data() + 6))
                   .translation());
    const double cart = (mid_s - best_v1.best.p_O).norm();
    if (cart < 0.20) continue;  // keep the standby OUT of the cooperative zone

    StandbyRow r;
    r.q = q;
    r.path = (q - q_h).norm();
    double mx = 0.0;
    for (int i = 0; i < 12; ++i) {
      mx = std::max(mx, std::abs(q[i] - q_h[i]));
    }
    r.joint_motion = mx;
    r.cart = cart;

    const auto q12 = q;  // same type
    const auto s2 = scene->makeStateQ12(qv);
    // segment feasibility uses the same backend as everything else
    // (start state, goal state) -- see isSegmentValid
    r.seg_ok = true;  // filled in pass 2 (needs the goal state)
    r.score = 0.0;
    rows.push_back(r);
    (void)s2;
    (void)q12;
  }

  // pass 2: segment feasibility + clearance (the expensive part)
  const auto handover_state = scene->makeStateQ12(q12_best);
  int tested = 0, feasible = 0;
  for (auto& r : rows) {
    const std::vector<double> qv(r.q.data(), r.q.data() + 12);
    auto st = scene->makeStateQ12(qv);
    r.seg_ok = backend->isSegmentValid(st, handover_state, seg_samples);
    const dual_arm::Q12 qq = r.q;
    const auto d = backend->distances(st, &qq);
    r.clearance = d.arm_proximal;
    ++tested;
    if (r.seg_ok) ++feasible;
  }
  std::cout << "standby candidates after hard filters: " << rows.size()
            << "   segment-feasible: " << feasible << " ("
            << (tested ? 100.0 * feasible / tested : 0.0) << "%)\n";

  // pass 3: normalise + score (path/motion/cart smaller is better)
  auto p95 = [&](auto getter) {
    std::vector<double> v;
    for (const auto& r : rows) v.push_back(getter(r));
    if (v.empty()) return 1.0;
    std::sort(v.begin(), v.end());
    return std::max(v[static_cast<std::size_t>(0.95 * (v.size() - 1))], 1e-9);
  };
  const double s_path = p95([](const StandbyRow& r) { return r.path; });
  const double s_motion = p95([](const StandbyRow& r) { return r.joint_motion; });
  const double s_cart = p95([](const StandbyRow& r) { return r.cart; });
  const double s_clr = 0.05;
  auto clamp01 = [](double v) { return std::max(0.0, std::min(1.0, v)); };
  for (auto& r : rows) {
    const double a = r.seg_ok ? 1.0 : 0.0;
    const double b = 1.0 - clamp01(r.path / s_path);
    const double c = 1.0 - clamp01(r.joint_motion / s_motion);
    const double d = clamp01(r.clearance / s_clr);
    const double e = 1.0 - clamp01(r.cart / s_cart);
    r.score = (1.5 * a + 1.0 * b + 1.0 * c + 1.0 * d + 0.5 * e) / 5.0;
  }
  std::sort(rows.begin(), rows.end(),
            [](const StandbyRow& a, const StandbyRow& b) { return a.score > b.score; });

  // segment-sample sensitivity: the metric must not depend on the resolution
  std::cout << "\nsegment-sample sensitivity (same best standby):\n";
  std::vector<std::pair<int, bool>> sens;
  if (!rows.empty()) {
    const std::vector<double> qv(rows[0].q.data(), rows[0].q.data() + 12);
    auto st = scene->makeStateQ12(qv);
    for (int n : {8, 16, 32, 64}) {
      const bool ok = backend->isSegmentValid(st, handover_state, n);
      sens.emplace_back(n, ok);
      std::cout << "  N=" << n << " -> " << (ok ? "feasible" : "blocked") << "\n";
    }
  }

  const double total_s = std::chrono::duration<double>(Clock::now() - t0).count();

  // -------------------------------------------------------------------- report
  std::cout << "\n=== S5 result ===\n";
  std::cout << "V0 : score " << best_v0.score << "  p_O = [" << best_v0.best.p_O.transpose()
            << "]\n";
  std::cout << "V1 : score " << best_v1.score << "  p_O = [" << best_v1.best.p_O.transpose()
            << "]  ||dtheta|| = " << best_v1.best.rotvec.norm() * 180.0 / M_PI << " deg\n";
  std::cout << "standby best: score " << (rows.empty() ? 0.0 : rows[0].score)
            << "  seg=" << (rows.empty() ? 0 : (rows[0].seg_ok ? 1 : 0))
            << "  path=" << (rows.empty() ? 0.0 : rows[0].path)
            << "  cart=" << (rows.empty() ? 0.0 : rows[0].cart) << " m\n";
  std::cout << "total time: " << total_s << " s\n";

  int npass = 0;
  const int ntot = 4;
  const bool v1_ge_v0 = best_v1.score >= best_v0.score - 1e-9;
  const bool v1_norm_ok = best_v1.best.rotvec.norm() <= 10.0 * M_PI / 180.0 + 1e-12;
  const bool both_feasible = best_v0.feasible && best_v1.feasible;
  const bool standby_ok = !rows.empty();
  std::cout << "\n=== S5 self-checks ===\n";
  std::cout << (v1_ge_v0 ? "  PASS  " : "  FAIL  ") << "V1 >= V0 (orientation freedom cannot hurt)\n";
  npass += v1_ge_v0;
  std::cout << (v1_norm_ok ? "  PASS  " : "  FAIL  ") << "V1 ||dtheta||_2 <= 10 deg\n";
  npass += v1_norm_ok;
  std::cout << (both_feasible ? "  PASS  " : "  FAIL  ") << "V0 and V1 optima are S4-feasible\n";
  npass += both_feasible;
  std::cout << (standby_ok ? "  PASS  " : "  FAIL  ") << "standby pool non-empty\n";
  npass += standby_ok;
  std::cout << "  ---> " << npass << "/" << ntot << " passed\n";

  if (!json_out.empty()) {
    std::ofstream js(json_out);
    js << std::fixed << std::setprecision(6);
    js << "{\n  \"stage\": \"C4.2-S5\",\n";
    js << "  \"weight_set\": \"" << jsonEscape(weight_set) << "\",\n";
    js << "  \"scales\": [";
    for (int i = 0; i < 8; ++i) js << (i ? ", " : "") << spec.scale[i];
    js << "],\n";
    js << "  \"baseline_B\": {\"p_O\": ";
    writePose(js, baseB.world_T_object.translation(), Eigen::Vector3d::Zero());
    js << "},\n";
    js << "  \"V0\": {\n";
    dumpResult(js, best_v0, spec, "    ");
    js << "  },\n";
    js << "  \"V1\": {\n";
    dumpResult(js, best_v1, spec, "    ");
    js << "  },\n";
    js << "  \"theta_sensitivity\": [";
    for (std::size_t i = 0; i < theta_rows.size(); ++i) {
      js << (i ? ", " : "")
         << "{\"theta_max_deg\": " << theta_rows[i].deg
         << ", \"score\": " << theta_rows[i].score
         << ", \"used_deg\": " << theta_rows[i].used_deg
         << ", \"feasible\": " << (theta_rows[i].feasible ? "true" : "false") << "}";
    }
    js << "],\n";
    js << "  \"segment_metric\": {\"name\": \"joint-space straight-line segment feasibility\",\n";
    js << "    \"note\": \"NOT a motion planner; reported as segment feasibility\",\n";
    js << "    \"samples\": " << seg_samples << ",\n";
    js << "    \"candidates\": " << tested << ",\n";
    js << "    \"feasible\": " << feasible << ",\n";
    js << "    \"rate\": " << (tested ? static_cast<double>(feasible) / tested : 0.0) << ",\n";
    js << "    \"sample_sensitivity\": [";
    for (std::size_t i = 0; i < sens.size(); ++i) {
      js << (i ? ", " : "") << "{\"N\": " << sens[i].first
         << ", \"feasible\": " << (sens[i].second ? "true" : "false") << "}";
    }
    js << "]},\n";
    js << "  \"standby\": {\n";
    js << "    \"pool_sampled\": " << pool.size() << ",\n";
    js << "    \"after_filters\": " << rows.size() << ",\n";
    js << "    \"scales\": {\"path\": " << s_path << ", \"joint_motion\": " << s_motion
       << ", \"cart\": " << s_cart << ", \"clearance\": " << s_clr << "},\n";
    js << "    \"best\": ";
    if (rows.empty()) {
      js << "null\n";
    } else {
      js << "{\"q12\": [";
      for (int i = 0; i < 12; ++i) js << (i ? ", " : "") << rows[0].q[i];
      js << "], \"segment_ok\": " << (rows[0].seg_ok ? "true" : "false")
         << ", \"path\": " << rows[0].path << ", \"joint_motion\": " << rows[0].joint_motion
         << ", \"clearance\": " << rows[0].clearance << ", \"cart\": " << rows[0].cart
         << ", \"score\": " << rows[0].score << "}\n";
    }
    js << "  },\n";
    js << "  \"baseline_eval_same_pipeline\": {\n";
    dumpResult(js, [&] {
      c42::RefineResult r;
      r.result = baseline_eval;
      r.score = baseline_score;
      r.feasible = baseline_eval.layer.s4_collision_free;
      r.best.p_O = baseline_eval.candidate.p_O;
      return r;
    }(), spec, "    ");
    js << "  },\n";
    if (!multi.empty()) {
      int feas = 0;
      for (const auto& m : multi) feas += m.feasible ? 1 : 0;
      js << "  \"multi_seed\": {\"runs\": " << multi.size() << ", \"feasible\": " << feas
         << ", \"seed\": " << multi_seed_seed << ", \"best\": " << multi.front().score
         << ", \"worst\": " << multi.back().score << ",\n";
      js << "    \"scores\": [";
      for (std::size_t i = 0; i < multi.size(); ++i) {
        js << (i ? ", " : "") << multi[i].score;
      }
      js << "],\n    \"p_O_best\": [" << multi.front().p_O.x() << ", " << multi.front().p_O.y()
         << ", " << multi.front().p_O.z() << "]\n  },\n";
    }
    js << "  \"checks_passed\": " << npass << ",\n  \"checks_total\": " << ntot << ",\n";
    js << "  \"total_seconds\": " << total_s << "\n}\n";
    std::cout << "wrote " << json_out << "\n";
  }
  return npass == ntot ? 0 : 1;
}
